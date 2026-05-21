#include "cccad/solver/sketch_solver_engine.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "cccad/solver/constraint_graph.h"
#include "cccad/solver/stage1_solver.h"

namespace cccad::solver {
namespace {

using cccad::solver::v1::CONSTRAINT_STATUS_ACTIVE;
using cccad::solver::v1::CONSTRAINT_STATUS_DELETED;
using cccad::solver::v1::CONSTRAINT_STATUS_SUPPRESSED;
using cccad::solver::v1::SOLVE_STATUS_FULLY_CONSTRAINED;
using cccad::solver::v1::SOLVE_STATUS_INCONSISTENT;
using cccad::solver::v1::SOLVE_STATUS_OVER_CONSTRAINED;
using cccad::solver::v1::SOLVE_STATUS_UNDER_CONSTRAINED;
using cccad::solver::v1::SOLVER_DIAGNOSTIC_LEVEL_ERROR;

bool IsActive(cccad::solver::v1::ConstraintStatus status) {
  return status == cccad::solver::v1::CONSTRAINT_STATUS_UNSPECIFIED ||
         status == CONSTRAINT_STATUS_ACTIVE;
}

bool IsFinite(double value) {
  return std::isfinite(value);
}

std::optional<std::size_t> ReadSizeLimit(const char* name) {
  const char* value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return std::nullopt;
  }

  char* end = nullptr;
  const uint64_t parsed = std::strtoull(value, &end, 10);
  if (end == value || *end != '\0') {
    return std::nullopt;
  }
  return static_cast<std::size_t>(parsed);
}

std::optional<int32_t> ReadIntLimit(const char* name) {
  const char* value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return std::nullopt;
  }

  char* end = nullptr;
  const int64_t parsed = std::strtoll(value, &end, 10);
  if (end == value || *end != '\0' || parsed < 0 ||
      parsed > std::numeric_limits<int32_t>::max()) {
    return std::nullopt;
  }
  return static_cast<int32_t>(parsed);
}

bool Contains(const std::unordered_set<std::string>& ids, const std::string& id) {
  return !id.empty() && ids.find(id) != ids.end();
}

template <typename T>
bool ContainsKey(const std::unordered_map<std::string, T>& values, const std::string& id) {
  return !id.empty() && values.find(id) != values.end();
}

void SortRepeatedStrings(google::protobuf::RepeatedPtrField<std::string>* values) {
  std::vector<std::string> sorted(values->begin(), values->end());
  std::sort(sorted.begin(), sorted.end());
  values->Clear();
  for (const std::string& value : sorted) {
    values->Add()->assign(value);
  }
}

}  // namespace

SketchSolverEngine::SketchSolverEngine(SolverLimits limits) : limits_(limits) {}

void SketchSolverEngine::Solve(const cccad::solver::v1::SolveRequest& request,
                               cccad::solver::v1::SolveResponse* response) const {
  response->Clear();
  ValidationResult validation = ValidateModel(request.model(), request.options());
  for (const auto& diagnostic : validation.diagnostics) {
    *response->add_diagnostics() = diagnostic;
  }
  if (!validation.ok) {
    response->set_status(SOLVE_STATUS_INCONSISTENT);
    return;
  }

  Stage1SolveResult solve_result = SolveStage1Model(
      request.model(), BuildStage1Model(request.model()), request.options(),
      limits_.default_max_iterations);
  WriteStage1Solution(request.model(), solve_result.model, response->mutable_solution());
  for (const auto& diagnostic : solve_result.residual_diagnostics) {
    *response->add_diagnostics() = diagnostic;
  }

  const int32_t degrees_of_freedom = solve_result.degrees_of_freedom;
  response->set_degrees_of_freedom(degrees_of_freedom);
  if (!solve_result.converged) {
    AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "numerical_failure",
                     "stage 1 solver did not converge to the requested tolerance", {}, {}, {},
                     &validation);
    *response->add_diagnostics() = validation.diagnostics.back();
    response->set_status(cccad::solver::v1::SOLVE_STATUS_NUMERICAL_FAILURE);
    return;
  }
  response->set_status(StatusForDegreesOfFreedom(degrees_of_freedom));
}

void SketchSolverEngine::Check(const cccad::solver::v1::CheckRequest& request,
                               cccad::solver::v1::CheckResponse* response) const {
  response->Clear();
  ValidationResult validation = ValidateModel(request.model(), request.options());
  for (const auto& diagnostic : validation.diagnostics) {
    *response->add_diagnostics() = diagnostic;
  }
  if (!validation.ok) {
    response->set_status(SOLVE_STATUS_INCONSISTENT);
    return;
  }

  const int32_t degrees_of_freedom =
      EstimateDegreesOfFreedom(request.model(), request.options());
  response->set_degrees_of_freedom(degrees_of_freedom);
  response->set_status(StatusForDegreesOfFreedom(degrees_of_freedom));
}

void SketchSolverEngine::ApplyIntent(
    const cccad::solver::v1::ApplyIntentRequest& request,
    cccad::solver::v1::ApplyIntentResponse* response) const {
  response->Clear();
  ValidationResult validation = ValidateModel(request.model(), request.options());
  ValidateIntent(request.intent(), request.model(), &validation);
  for (const auto& diagnostic : validation.diagnostics) {
    *response->add_diagnostics() = diagnostic;
  }
  if (!validation.ok) {
    response->set_status(SOLVE_STATUS_INCONSISTENT);
    return;
  }

  ConstraintGraph graph(request.model());
  const ConstraintComponentData affected_component = graph.ComponentForSeed(graph.SeedForIntent(request.intent()));

  Stage1Model stage_model = BuildStage1Model(request.model());
  switch (request.intent().kind_case()) {
    case cccad::solver::v1::UserIntent::kMovePoint: {
      auto point_it = stage_model.points.find(request.intent().move_point().point_id());
      if (point_it != stage_model.points.end()) {
        point_it->second.x = request.intent().move_point().target().x();
        point_it->second.y = request.intent().move_point().target().y();
        stage_model.intent_anchors.push_back(IntentAnchor{
            .point_id = request.intent().move_point().point_id(),
            .x = request.intent().move_point().target().x(),
            .y = request.intent().move_point().target().y(),
            .weight = request.intent().move_point().weight()});
      }
      break;
    }
    case cccad::solver::v1::UserIntent::kMoveEntity:
      // Stage 1 reports the affected graph component. Full rigid entity move is a later intent implementation.
      break;
    case cccad::solver::v1::UserIntent::kSetDimension:
      stage_model.dimension_value_overrides[request.intent().set_dimension().dimension_id()] =
          request.intent().set_dimension().value();
      break;
    case cccad::solver::v1::UserIntent::kAddConstraint:
      stage_model.extra_constraints.push_back(&request.intent().add_constraint().constraint());
      break;
    case cccad::solver::v1::UserIntent::KIND_NOT_SET:
      break;
  }

  Stage1SolveResult solve_result = SolveStage1Model(request.model(), std::move(stage_model),
                                                    request.options(),
                                                    limits_.default_max_iterations);
  WriteStage1Solution(request.model(), solve_result.model, response->mutable_solution());
  for (const auto& diagnostic : solve_result.residual_diagnostics) {
    *response->add_diagnostics() = diagnostic;
  }

  for (const auto& entity_id : affected_component.entity_ids) {
    response->add_affected_entity_ids(entity_id);
  }
  SortRepeatedStrings(response->mutable_affected_entity_ids());

  const int32_t degrees_of_freedom = solve_result.degrees_of_freedom;
  response->set_degrees_of_freedom(degrees_of_freedom);
  if (!solve_result.converged) {
    AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "numerical_failure",
                     "stage 1 solver did not converge to the requested tolerance", {}, {}, {},
                     &validation);
    *response->add_diagnostics() = validation.diagnostics.back();
    response->set_status(cccad::solver::v1::SOLVE_STATUS_NUMERICAL_FAILURE);
    return;
  }
  response->set_status(StatusForDegreesOfFreedom(degrees_of_freedom));
}

void SketchSolverEngine::Analyze(const cccad::solver::v1::AnalyzeRequest& request,
                                 cccad::solver::v1::AnalyzeResponse* response) const {
  response->Clear();
  ValidationResult validation = ValidateModel(request.model(), request.options());
  for (const auto& diagnostic : validation.diagnostics) {
    *response->add_diagnostics() = diagnostic;
  }
  if (!validation.ok) {
    response->set_status(SOLVE_STATUS_INCONSISTENT);
    return;
  }

  const int32_t degrees_of_freedom =
      EstimateDegreesOfFreedom(request.model(), request.options());
  response->set_degrees_of_freedom(degrees_of_freedom);
  response->set_status(StatusForDegreesOfFreedom(degrees_of_freedom));

  ConstraintGraph graph(request.model());
  for (const auto& component : graph.Components()) {
    ConstraintComponentData rank_component = component;
    rank_component.degrees_of_freedom =
        EstimateDegreesOfFreedom(request.model(), request.options(), component.entity_ids,
                                 component.constraint_ids, component.dimension_ids);
    rank_component.status = StatusForDegreesOfFreedom(rank_component.degrees_of_freedom);
    CopyComponentToProto(rank_component, response->add_components());
  }
}

SketchSolverEngine::ValidationResult SketchSolverEngine::ValidateModel(
    const cccad::solver::v1::SketchModel& model,
    const cccad::solver::v1::SolverOptions& options) const {
  ValidationResult result;

  if (static_cast<std::size_t>(model.entities_size()) > limits_.max_entities) {
    AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "entity_limit_exceeded",
                     "entity count exceeds configured limit", {}, {}, {}, &result);
  }
  if (static_cast<std::size_t>(model.constraints_size()) > limits_.max_constraints) {
    AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "constraint_limit_exceeded",
                     "constraint count exceeds configured limit", {}, {}, {}, &result);
  }
  if (static_cast<std::size_t>(model.dimensions_size()) > limits_.max_dimensions) {
    AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "dimension_limit_exceeded",
                     "dimension count exceeds configured limit", {}, {}, {}, &result);
  }
  if (static_cast<std::size_t>(model.parameters_size()) > limits_.max_parameters) {
    AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "parameter_limit_exceeded",
                     "parameter count exceeds configured limit", {}, {}, {}, &result);
  }
  if (options.max_iterations() < 0) {
    AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "invalid_solver_options",
                     "max_iterations must be non-negative", {}, {}, {}, &result);
  }
  if (options.tolerance() < 0.0 || !IsFinite(options.tolerance())) {
    AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "invalid_solver_options",
                     "tolerance must be finite and non-negative", {}, {}, {}, &result);
  }

  std::unordered_set<std::string> entity_ids;
  std::unordered_map<std::string, cccad::solver::v1::Entity::KindCase> entity_kinds;
  entity_ids.reserve(static_cast<std::size_t>(model.entities_size()));
  entity_kinds.reserve(static_cast<std::size_t>(model.entities_size()));
  for (const auto& entity : model.entities()) {
    if (entity.id().empty()) {
      AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "invalid_entity",
                       "entity id must not be empty", {}, {}, {}, &result);
    } else if (!entity_ids.insert(entity.id()).second) {
      AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "duplicate_entity_id",
                       "entity id must be unique", {entity.id()}, {}, {}, &result);
    } else {
      entity_kinds.emplace(entity.id(), entity.kind_case());
    }

    switch (entity.kind_case()) {
      case cccad::solver::v1::Entity::kPoint:
        if (!IsFinite(entity.point().x()) || !IsFinite(entity.point().y())) {
          AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "invalid_entity",
                           "point coordinates must be finite", {entity.id()}, {}, {}, &result);
        }
        break;
      case cccad::solver::v1::Entity::kCircle:
        if (!IsFinite(entity.circle().radius()) || entity.circle().radius() <= 0.0) {
          AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "invalid_entity",
                           "circle radius must be finite and positive", {entity.id()}, {}, {},
                           &result);
        }
        break;
      case cccad::solver::v1::Entity::kLine:
      case cccad::solver::v1::Entity::kArc:
        break;
      case cccad::solver::v1::Entity::KIND_NOT_SET:
        AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "unsupported_entity",
                         "entity kind must be set", {entity.id()}, {}, {}, &result);
        break;
    }
  }

  for (const auto& entity : model.entities()) {
    switch (entity.kind_case()) {
      case cccad::solver::v1::Entity::kLine:
        if (!ContainsKey(entity_kinds, entity.line().start_point_id()) ||
            !ContainsKey(entity_kinds, entity.line().end_point_id()) ||
            entity_kinds[entity.line().start_point_id()] != cccad::solver::v1::Entity::kPoint ||
            entity_kinds[entity.line().end_point_id()] != cccad::solver::v1::Entity::kPoint) {
          AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "invalid_entity_reference",
                           "line endpoints must reference existing point entities", {entity.id()},
                           {}, {}, &result);
        }
        break;
      case cccad::solver::v1::Entity::kCircle:
        if (!ContainsKey(entity_kinds, entity.circle().center_point_id()) ||
            entity_kinds[entity.circle().center_point_id()] != cccad::solver::v1::Entity::kPoint) {
          AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "invalid_entity_reference",
                           "circle center must reference an existing point entity", {entity.id()},
                           {}, {}, &result);
        }
        break;
      case cccad::solver::v1::Entity::kArc:
        if (!ContainsKey(entity_kinds, entity.arc().center_point_id()) ||
            !ContainsKey(entity_kinds, entity.arc().start_point_id()) ||
            !ContainsKey(entity_kinds, entity.arc().end_point_id()) ||
            entity_kinds[entity.arc().center_point_id()] != cccad::solver::v1::Entity::kPoint ||
            entity_kinds[entity.arc().start_point_id()] != cccad::solver::v1::Entity::kPoint ||
            entity_kinds[entity.arc().end_point_id()] != cccad::solver::v1::Entity::kPoint) {
          AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "invalid_entity_reference",
                           "arc points must reference existing point entities", {entity.id()}, {},
                           {}, &result);
        }
        break;
      case cccad::solver::v1::Entity::kPoint:
      case cccad::solver::v1::Entity::KIND_NOT_SET:
        break;
    }
  }

  std::unordered_set<std::string> constraint_ids;
  constraint_ids.reserve(static_cast<std::size_t>(model.constraints_size()));
  for (const auto& constraint : model.constraints()) {
    if (constraint.id().empty()) {
      AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "invalid_constraint",
                       "constraint id must not be empty", {}, {}, {}, &result);
    } else if (!constraint_ids.insert(constraint.id()).second) {
      AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "duplicate_constraint_id",
                       "constraint id must be unique", {}, {constraint.id()}, {}, &result);
    }
    if (constraint.status() == CONSTRAINT_STATUS_DELETED ||
        constraint.status() == CONSTRAINT_STATUS_SUPPRESSED) {
      continue;
    }
    if (constraint.kind_case() == cccad::solver::v1::Constraint::KIND_NOT_SET) {
      AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "unsupported_constraint",
                       "constraint kind must be set", {}, {constraint.id()}, {}, &result);
      continue;
    }

    std::vector<std::string> references;
    switch (constraint.kind_case()) {
      case cccad::solver::v1::Constraint::kCoincident:
        references = {constraint.coincident().point_a_id(), constraint.coincident().point_b_id()};
        break;
      case cccad::solver::v1::Constraint::kHorizontal:
        references = {constraint.horizontal().line_id()};
        break;
      case cccad::solver::v1::Constraint::kVertical:
        references = {constraint.vertical().line_id()};
        break;
      case cccad::solver::v1::Constraint::kParallel:
        references = {constraint.parallel().line_a_id(), constraint.parallel().line_b_id()};
        break;
      case cccad::solver::v1::Constraint::kPerpendicular:
        references = {constraint.perpendicular().line_a_id(),
                      constraint.perpendicular().line_b_id()};
        break;
      case cccad::solver::v1::Constraint::kTangent:
        references = {constraint.tangent().entity_a_id(), constraint.tangent().entity_b_id()};
        break;
      case cccad::solver::v1::Constraint::kEqual:
        references = {constraint.equal().entity_a_id(), constraint.equal().entity_b_id()};
        break;
      case cccad::solver::v1::Constraint::kFixed:
        references = {constraint.fixed().entity_id()};
        break;
      case cccad::solver::v1::Constraint::kMidpoint:
        references = {constraint.midpoint().midpoint_id(), constraint.midpoint().point_a_id(),
                      constraint.midpoint().point_b_id()};
        break;
      case cccad::solver::v1::Constraint::kConcentric:
        references = {constraint.concentric().circle_a_id(),
                      constraint.concentric().circle_b_id()};
        break;
      case cccad::solver::v1::Constraint::KIND_NOT_SET:
        break;
    }
    for (const auto& reference : references) {
      if (!Contains(entity_ids, reference)) {
        AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "invalid_constraint_reference",
                         "constraint references must point to existing entities", {reference},
                         {constraint.id()}, {}, &result);
      }
    }
  }

  std::unordered_set<std::string> dimension_ids;
  dimension_ids.reserve(static_cast<std::size_t>(model.dimensions_size()));
  for (const auto& dimension : model.dimensions()) {
    if (dimension.id().empty()) {
      AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "invalid_dimension",
                       "dimension id must not be empty", {}, {}, {}, &result);
    } else if (!dimension_ids.insert(dimension.id()).second) {
      AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "duplicate_dimension_id",
                       "dimension id must be unique", {}, {}, {dimension.id()}, &result);
    }
    if (!IsActive(dimension.status())) {
      continue;
    }
    if (dimension.kind_case() == cccad::solver::v1::Dimension::KIND_NOT_SET) {
      AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "unsupported_dimension",
                       "dimension kind must be set", {}, {}, {dimension.id()}, &result);
      continue;
    }

    std::vector<std::string> references;
    switch (dimension.kind_case()) {
      case cccad::solver::v1::Dimension::kDistance:
        references = {dimension.distance().ref_a_id(), dimension.distance().ref_b_id()};
        if (!IsFinite(dimension.distance().value()) || dimension.distance().value() < 0.0) {
          AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "invalid_dimension",
                           "distance dimension value must be finite and non-negative", {}, {},
                           {dimension.id()}, &result);
        }
        break;
      case cccad::solver::v1::Dimension::kRadius:
        references = {dimension.radius().entity_id()};
        if (!IsFinite(dimension.radius().value()) || dimension.radius().value() <= 0.0) {
          AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "invalid_dimension",
                           "radius dimension value must be finite and positive", {}, {},
                           {dimension.id()}, &result);
        }
        break;
      case cccad::solver::v1::Dimension::kDiameter:
        references = {dimension.diameter().entity_id()};
        if (!IsFinite(dimension.diameter().value()) || dimension.diameter().value() <= 0.0) {
          AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "invalid_dimension",
                           "diameter dimension value must be finite and positive", {}, {},
                           {dimension.id()}, &result);
        }
        break;
      case cccad::solver::v1::Dimension::kAngle:
        references = {dimension.angle().line_a_id(), dimension.angle().line_b_id()};
        if (!IsFinite(dimension.angle().value_rad())) {
          AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "invalid_dimension",
                           "angle dimension value must be finite", {}, {}, {dimension.id()},
                           &result);
        }
        break;
      case cccad::solver::v1::Dimension::KIND_NOT_SET:
        break;
    }
    for (const auto& reference : references) {
      if (!Contains(entity_ids, reference)) {
        AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "invalid_dimension_reference",
                         "dimension references must point to existing entities", {reference}, {},
                         {dimension.id()}, &result);
      }
    }
  }

  result.ok = result.diagnostics.empty();
  std::sort(result.diagnostics.begin(), result.diagnostics.end(),
            [](const auto& lhs, const auto& rhs) {
              if (lhs.code() != rhs.code()) {
                return lhs.code() < rhs.code();
              }
              return lhs.message() < rhs.message();
            });
  return result;
}

void SketchSolverEngine::ValidateIntent(const cccad::solver::v1::UserIntent& intent,
                                        const cccad::solver::v1::SketchModel& model,
                                        ValidationResult* result) const {
  std::unordered_set<std::string> entity_ids;
  std::unordered_set<std::string> dimension_ids;
  entity_ids.reserve(static_cast<std::size_t>(model.entities_size()));
  dimension_ids.reserve(static_cast<std::size_t>(model.dimensions_size()));
  for (const auto& entity : model.entities()) {
    entity_ids.insert(entity.id());
  }
  for (const auto& dimension : model.dimensions()) {
    dimension_ids.insert(dimension.id());
  }

  switch (intent.kind_case()) {
    case cccad::solver::v1::UserIntent::kMovePoint:
      if (!Contains(entity_ids, intent.move_point().point_id())) {
        AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "invalid_intent_reference",
                         "move point intent must reference an existing point",
                         {intent.move_point().point_id()}, {}, {}, result);
      }
      if (!IsFinite(intent.move_point().target().x()) ||
          !IsFinite(intent.move_point().target().y()) || !IsFinite(intent.move_point().weight())) {
        AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "invalid_intent",
                         "move point intent values must be finite", {}, {}, {}, result);
      }
      break;
    case cccad::solver::v1::UserIntent::kMoveEntity:
      if (!Contains(entity_ids, intent.move_entity().entity_id())) {
        AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "invalid_intent_reference",
                         "move entity intent must reference an existing entity",
                         {intent.move_entity().entity_id()}, {}, {}, result);
      }
      if (!IsFinite(intent.move_entity().delta().x()) ||
          !IsFinite(intent.move_entity().delta().y()) || !IsFinite(intent.move_entity().weight())) {
        AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "invalid_intent",
                         "move entity intent values must be finite", {}, {}, {}, result);
      }
      break;
    case cccad::solver::v1::UserIntent::kSetDimension:
      if (!Contains(dimension_ids, intent.set_dimension().dimension_id())) {
        AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "invalid_intent_reference",
                         "set dimension intent must reference an existing dimension", {}, {},
                         {intent.set_dimension().dimension_id()}, result);
      }
      if (!IsFinite(intent.set_dimension().value())) {
        AppendDiagnostic(SOLVER_DIAGNOSTIC_LEVEL_ERROR, "invalid_intent",
                         "set dimension value must be finite", {}, {}, {}, result);
      }
      break;
    case cccad::solver::v1::UserIntent::kAddConstraint:
    case cccad::solver::v1::UserIntent::KIND_NOT_SET:
      break;
  }

  result->ok = result->diagnostics.empty();
  std::sort(result->diagnostics.begin(), result->diagnostics.end(),
            [](const auto& lhs, const auto& rhs) {
              if (lhs.code() != rhs.code()) {
                return lhs.code() < rhs.code();
              }
              return lhs.message() < rhs.message();
            });
}

int32_t SketchSolverEngine::EstimateDegreesOfFreedom(
    const cccad::solver::v1::SketchModel& model,
    const cccad::solver::v1::SolverOptions& options,
    const std::vector<std::string>& entity_ids,
    const std::vector<std::string>& constraint_ids,
    const std::vector<std::string>& dimension_ids) const {
  std::vector<std::string> scoped_entity_ids = entity_ids;
  std::vector<std::string> scoped_constraint_ids = constraint_ids;
  std::vector<std::string> scoped_dimension_ids = dimension_ids;
  if (scoped_entity_ids.empty() && scoped_constraint_ids.empty() && scoped_dimension_ids.empty()) {
    for (const auto& entity : model.entities()) scoped_entity_ids.push_back(entity.id());
    for (const auto& constraint : model.constraints()) scoped_constraint_ids.push_back(constraint.id());
    for (const auto& dimension : model.dimensions()) scoped_dimension_ids.push_back(dimension.id());
  }
  return EstimateStage1DegreesOfFreedom(model, BuildStage1Model(model), options, scoped_entity_ids,
                                        scoped_constraint_ids, scoped_dimension_ids);
}

cccad::solver::v1::SolveStatus SketchSolverEngine::StatusForDegreesOfFreedom(
    int32_t degrees_of_freedom) const {
  return ConstraintGraph::StatusForDegreesOfFreedom(degrees_of_freedom);
}

void SketchSolverEngine::CopyModelToSolution(
    const cccad::solver::v1::SketchModel& model,
    cccad::solver::v1::SketchSolution* solution) const {
  solution->Clear();
  std::vector<const cccad::solver::v1::Entity*> entities;
  entities.reserve(static_cast<std::size_t>(model.entities_size()));
  for (const auto& entity : model.entities()) {
    entities.push_back(&entity);
  }
  std::sort(entities.begin(), entities.end(), [](const auto* lhs, const auto* rhs) {
    return lhs->id() < rhs->id();
  });

  for (const auto* entity : entities) {
    auto* solved = solution->add_entities();
    solved->set_id(entity->id());
    switch (entity->kind_case()) {
      case cccad::solver::v1::Entity::kPoint:
        solved->mutable_point()->set_x(entity->point().x());
        solved->mutable_point()->set_y(entity->point().y());
        break;
      case cccad::solver::v1::Entity::kCircle:
        solved->mutable_circle()->set_center_point_id(entity->circle().center_point_id());
        solved->mutable_circle()->set_radius(entity->circle().radius());
        break;
      case cccad::solver::v1::Entity::kArc:
        solved->mutable_arc()->set_center_point_id(entity->arc().center_point_id());
        solved->mutable_arc()->set_start_point_id(entity->arc().start_point_id());
        solved->mutable_arc()->set_end_point_id(entity->arc().end_point_id());
        solved->mutable_arc()->set_clockwise(entity->arc().clockwise());
        solved->mutable_arc()->set_branch(entity->arc().branch());
        break;
      case cccad::solver::v1::Entity::kLine:
        solved->mutable_line()->set_start_point_id(entity->line().start_point_id());
        solved->mutable_line()->set_end_point_id(entity->line().end_point_id());
        break;
      case cccad::solver::v1::Entity::KIND_NOT_SET:
        break;
    }
  }
}

void SketchSolverEngine::AppendDiagnostic(
    cccad::solver::v1::SolverDiagnosticLevel level, std::string_view code,
    std::string_view message, std::vector<std::string> entity_ids,
    std::vector<std::string> constraint_ids, std::vector<std::string> dimension_ids,
    ValidationResult* result) const {
  auto diagnostic = cccad::solver::v1::SolverDiagnostic{};
  diagnostic.set_level(level);
  diagnostic.set_code(std::string(code));
  diagnostic.set_message(std::string(message));
  std::sort(entity_ids.begin(), entity_ids.end());
  std::sort(constraint_ids.begin(), constraint_ids.end());
  std::sort(dimension_ids.begin(), dimension_ids.end());
  for (const auto& id : entity_ids) {
    diagnostic.add_entity_ids(id);
  }
  for (const auto& id : constraint_ids) {
    diagnostic.add_constraint_ids(id);
  }
  for (const auto& id : dimension_ids) {
    diagnostic.add_dimension_ids(id);
  }
  result->diagnostics.push_back(std::move(diagnostic));
}

SolverLimits LimitsFromEnvironment() {
  SolverLimits limits;
  if (auto value = ReadSizeLimit("SOLVER_MAX_ENTITIES")) {
    limits.max_entities = *value;
  }
  if (auto value = ReadSizeLimit("SOLVER_MAX_CONSTRAINTS")) {
    limits.max_constraints = *value;
  }
  if (auto value = ReadIntLimit("SOLVER_MAX_ITERATIONS")) {
    limits.default_max_iterations = *value;
  }
  return limits;
}

}  // namespace cccad::solver
