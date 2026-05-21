#include "cccad/solver/stage1_solver.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>

namespace cccad::solver {
namespace {

using cccad::solver::v1::CONSTRAINT_STATUS_ACTIVE;

constexpr double kDefaultTolerance = 1e-8;
constexpr double kDefaultStabilityWeight = 1e-6;
constexpr double kMinimumRadius = 1e-9;

bool IsActive(cccad::solver::v1::ConstraintStatus status) {
  return status == cccad::solver::v1::CONSTRAINT_STATUS_UNSPECIFIED ||
         status == CONSTRAINT_STATUS_ACTIVE;
}

bool IsFinite(double value) { return std::isfinite(value); }

double Square(double value) { return value * value; }

double Distance(double ax, double ay, double bx, double by) {
  return std::sqrt(Square(bx - ax) + Square(by - ay));
}

struct Stage1Variable {
  enum class Kind { kPointX, kPointY, kCircleRadius };
  Kind kind = Kind::kPointX;
  std::string id;
};

bool ScopeContains(const std::unordered_set<std::string>* ids, const std::string& id) {
  return ids == nullptr || ids->contains(id);
}

std::unordered_set<std::string> MakeScopeSet(const std::vector<std::string>& ids) {
  return std::unordered_set<std::string>(ids.begin(), ids.end());
}

std::vector<Stage1Variable> BuildVariables(const Stage1Model& model,
                                           const std::unordered_set<std::string>* entity_scope = nullptr) {
  std::vector<Stage1Variable> variables;
  std::vector<std::string> ids;

  ids.reserve(model.points.size());
  for (const auto& [id, point] : model.points) {
    ids.push_back(id);
  }
  std::sort(ids.begin(), ids.end());
  for (const auto& id : ids) {
    if (!ScopeContains(entity_scope, id)) {
      continue;
    }
    const auto& point = model.points.at(id);
    if (!point.lock_x) {
      variables.push_back({Stage1Variable::Kind::kPointX, id});
    }
    if (!point.lock_y) {
      variables.push_back({Stage1Variable::Kind::kPointY, id});
    }
  }

  ids.clear();
  ids.reserve(model.circles.size());
  for (const auto& [id, circle] : model.circles) {
    ids.push_back(id);
  }
  std::sort(ids.begin(), ids.end());
  for (const auto& id : ids) {
    if (!ScopeContains(entity_scope, id)) {
      continue;
    }
    const auto& circle = model.circles.at(id);
    if (!circle.lock_radius) {
      variables.push_back({Stage1Variable::Kind::kCircleRadius, id});
    }
  }

  return variables;
}

std::vector<double> ReadVariables(const Stage1Model& model, const std::vector<Stage1Variable>& variables) {
  std::vector<double> x;
  x.reserve(variables.size());
  for (const auto& variable : variables) {
    switch (variable.kind) {
      case Stage1Variable::Kind::kPointX:
        x.push_back(model.points.at(variable.id).x);
        break;
      case Stage1Variable::Kind::kPointY:
        x.push_back(model.points.at(variable.id).y);
        break;
      case Stage1Variable::Kind::kCircleRadius:
        x.push_back(model.circles.at(variable.id).radius);
        break;
    }
  }
  return x;
}

void WriteVariables(Stage1Model* model, const std::vector<Stage1Variable>& variables,
                    const std::vector<double>& x) {
  for (std::size_t index = 0; index < variables.size(); ++index) {
    const auto& variable = variables[index];
    switch (variable.kind) {
      case Stage1Variable::Kind::kPointX:
        (*model).points[variable.id].x = x[index];
        break;
      case Stage1Variable::Kind::kPointY:
        (*model).points[variable.id].y = x[index];
        break;
      case Stage1Variable::Kind::kCircleRadius:
        (*model).circles[variable.id].radius = std::max(x[index], kMinimumRadius);
        break;
    }
  }
}

void AddScaledResidual(std::vector<double>* residuals, double value, double weight = 1.0) {
  const double safe_weight = std::isfinite(weight) && weight > 0.0 ? weight : 1.0;
  residuals->push_back(std::sqrt(safe_weight) * value);
}

struct ResidualDiagnosticSource {
  std::string code;
  std::string message;
  std::vector<std::string> entity_ids;
  std::vector<std::string> constraint_ids;
  std::vector<std::string> dimension_ids;
};

struct ResidualEntry {
  double value = 0.0;
  ResidualDiagnosticSource diagnostic;
};

void SortStrings(std::vector<std::string>* values) {
  std::sort(values->begin(), values->end());
  values->erase(std::unique(values->begin(), values->end()), values->end());
}

bool RepeatedStringsEqual(const google::protobuf::RepeatedPtrField<std::string>& lhs,
                          const google::protobuf::RepeatedPtrField<std::string>& rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (int index = 0; index < lhs.size(); ++index) {
    if (lhs.Get(index) != rhs.Get(index)) {
      return false;
    }
  }
  return true;
}

void AddResidualEntry(std::vector<ResidualEntry>* residuals,
                      double value,
                      ResidualDiagnosticSource diagnostic,
                      double weight = 1.0) {
  const double safe_weight = std::isfinite(weight) && weight > 0.0 ? weight : 1.0;
  residuals->push_back({.value = std::sqrt(safe_weight) * value,
                        .diagnostic = std::move(diagnostic)});
}

void AddDiagnosticResiduals(const cccad::solver::v1::Constraint& constraint,
                            const Stage1Model& model,
                            std::vector<ResidualEntry>* residuals) {
  if (!IsActive(constraint.status())) {
    return;
  }

  switch (constraint.kind_case()) {
    case cccad::solver::v1::Constraint::kCoincident: {
      const auto point_a_it = model.points.find(constraint.coincident().point_a_id());
      const auto point_b_it = model.points.find(constraint.coincident().point_b_id());
      if (point_a_it == model.points.end() || point_b_it == model.points.end()) {
        return;
      }
      const ResidualDiagnosticSource diagnostic{
          .code = "constraint_residual",
          .message = "coincident constraint residual is above tolerance",
          .entity_ids = {constraint.coincident().point_a_id(), constraint.coincident().point_b_id()},
          .constraint_ids = {constraint.id()}};
      AddResidualEntry(residuals, point_a_it->second.x - point_b_it->second.x, diagnostic);
      AddResidualEntry(residuals, point_a_it->second.y - point_b_it->second.y, diagnostic);
      break;
    }
    case cccad::solver::v1::Constraint::kHorizontal: {
      const auto line_it = model.lines.find(constraint.horizontal().line_id());
      if (line_it == model.lines.end()) {
        return;
      }
      const auto start_it = model.points.find(line_it->second.start_point_id);
      const auto end_it = model.points.find(line_it->second.end_point_id);
      if (start_it == model.points.end() || end_it == model.points.end()) {
        return;
      }
      AddResidualEntry(residuals, end_it->second.y - start_it->second.y,
                       {.code = "constraint_residual",
                        .message = "horizontal constraint residual is above tolerance",
                        .entity_ids = {constraint.horizontal().line_id()},
                        .constraint_ids = {constraint.id()}});
      break;
    }
    case cccad::solver::v1::Constraint::kVertical: {
      const auto line_it = model.lines.find(constraint.vertical().line_id());
      if (line_it == model.lines.end()) {
        return;
      }
      const auto start_it = model.points.find(line_it->second.start_point_id);
      const auto end_it = model.points.find(line_it->second.end_point_id);
      if (start_it == model.points.end() || end_it == model.points.end()) {
        return;
      }
      AddResidualEntry(residuals, end_it->second.x - start_it->second.x,
                       {.code = "constraint_residual",
                        .message = "vertical constraint residual is above tolerance",
                        .entity_ids = {constraint.vertical().line_id()},
                        .constraint_ids = {constraint.id()}});
      break;
    }
    case cccad::solver::v1::Constraint::kFixed:
    case cccad::solver::v1::Constraint::kParallel:
    case cccad::solver::v1::Constraint::kPerpendicular:
    case cccad::solver::v1::Constraint::kTangent:
    case cccad::solver::v1::Constraint::kEqual:
    case cccad::solver::v1::Constraint::kMidpoint:
    case cccad::solver::v1::Constraint::kConcentric:
    case cccad::solver::v1::Constraint::KIND_NOT_SET:
      break;
  }
}

void AddDiagnosticDimensionResiduals(const cccad::solver::v1::Dimension& dimension,
                                     const Stage1Model& model,
                                     std::vector<ResidualEntry>* residuals) {
  if (!IsActive(dimension.status()) || !dimension.driving()) {
    return;
  }

  switch (dimension.kind_case()) {
    case cccad::solver::v1::Dimension::kDistance: {
      if (dimension.distance().ref_kind() != cccad::solver::v1::DISTANCE_REFERENCE_KIND_UNSPECIFIED &&
          dimension.distance().ref_kind() != cccad::solver::v1::DISTANCE_REFERENCE_KIND_POINT_POINT) {
        return;
      }
      const auto point_a_it = model.points.find(dimension.distance().ref_a_id());
      const auto point_b_it = model.points.find(dimension.distance().ref_b_id());
      if (point_a_it == model.points.end() || point_b_it == model.points.end()) {
        return;
      }
      double target = dimension.distance().value();
      if (const auto override_it = model.dimension_value_overrides.find(dimension.id());
          override_it != model.dimension_value_overrides.end()) {
        target = override_it->second;
      }
      AddResidualEntry(residuals,
                       Distance(point_a_it->second.x, point_a_it->second.y,
                                point_b_it->second.x, point_b_it->second.y) -
                           target,
                       {.code = "dimension_residual",
                        .message = "distance dimension residual is above tolerance",
                        .entity_ids = {dimension.distance().ref_a_id(), dimension.distance().ref_b_id()},
                        .dimension_ids = {dimension.id()}});
      break;
    }
    case cccad::solver::v1::Dimension::kRadius: {
      const auto circle_it = model.circles.find(dimension.radius().entity_id());
      if (circle_it == model.circles.end()) {
        return;
      }
      double target = dimension.radius().value();
      if (const auto override_it = model.dimension_value_overrides.find(dimension.id());
          override_it != model.dimension_value_overrides.end()) {
        target = override_it->second;
      }
      AddResidualEntry(residuals, circle_it->second.radius - target,
                       {.code = "dimension_residual",
                        .message = "radius dimension residual is above tolerance",
                        .entity_ids = {dimension.radius().entity_id()},
                        .dimension_ids = {dimension.id()}});
      break;
    }
    case cccad::solver::v1::Dimension::kDiameter: {
      const auto circle_it = model.circles.find(dimension.diameter().entity_id());
      if (circle_it == model.circles.end()) {
        return;
      }
      double target = dimension.diameter().value() * 0.5;
      if (const auto override_it = model.dimension_value_overrides.find(dimension.id());
          override_it != model.dimension_value_overrides.end()) {
        target = override_it->second * 0.5;
      }
      AddResidualEntry(residuals, circle_it->second.radius - target,
                       {.code = "dimension_residual",
                        .message = "diameter dimension residual is above tolerance",
                        .entity_ids = {dimension.diameter().entity_id()},
                        .dimension_ids = {dimension.id()}});
      break;
    }
    case cccad::solver::v1::Dimension::kAngle:
    case cccad::solver::v1::Dimension::KIND_NOT_SET:
      break;
  }
}

void AddConstraintResiduals(const cccad::solver::v1::Constraint& constraint,
                            const Stage1Model& model, std::vector<double>* residuals) {
  if (!IsActive(constraint.status())) {
    return;
  }

  switch (constraint.kind_case()) {
    case cccad::solver::v1::Constraint::kCoincident: {
      const auto point_a_it = model.points.find(constraint.coincident().point_a_id());
      const auto point_b_it = model.points.find(constraint.coincident().point_b_id());
      if (point_a_it == model.points.end() || point_b_it == model.points.end()) {
        return;
      }
      AddScaledResidual(residuals, point_a_it->second.x - point_b_it->second.x);
      AddScaledResidual(residuals, point_a_it->second.y - point_b_it->second.y);
      break;
    }
    case cccad::solver::v1::Constraint::kHorizontal: {
      const auto line_it = model.lines.find(constraint.horizontal().line_id());
      if (line_it == model.lines.end()) {
        return;
      }
      const auto start_it = model.points.find(line_it->second.start_point_id);
      const auto end_it = model.points.find(line_it->second.end_point_id);
      if (start_it == model.points.end() || end_it == model.points.end()) {
        return;
      }
      AddScaledResidual(residuals, end_it->second.y - start_it->second.y);
      break;
    }
    case cccad::solver::v1::Constraint::kVertical: {
      const auto line_it = model.lines.find(constraint.vertical().line_id());
      if (line_it == model.lines.end()) {
        return;
      }
      const auto start_it = model.points.find(line_it->second.start_point_id);
      const auto end_it = model.points.find(line_it->second.end_point_id);
      if (start_it == model.points.end() || end_it == model.points.end()) {
        return;
      }
      AddScaledResidual(residuals, end_it->second.x - start_it->second.x);
      break;
    }
    case cccad::solver::v1::Constraint::kFixed:
      break;
    case cccad::solver::v1::Constraint::kParallel:
    case cccad::solver::v1::Constraint::kPerpendicular:
    case cccad::solver::v1::Constraint::kTangent:
    case cccad::solver::v1::Constraint::kEqual:
    case cccad::solver::v1::Constraint::kMidpoint:
    case cccad::solver::v1::Constraint::kConcentric:
    case cccad::solver::v1::Constraint::KIND_NOT_SET:
      break;
  }
}

void AddDimensionResiduals(const cccad::solver::v1::Dimension& dimension,
                           const Stage1Model& model, std::vector<double>* residuals) {
  if (!IsActive(dimension.status()) || !dimension.driving()) {
    return;
  }

  switch (dimension.kind_case()) {
    case cccad::solver::v1::Dimension::kDistance: {
      if (dimension.distance().ref_kind() != cccad::solver::v1::DISTANCE_REFERENCE_KIND_UNSPECIFIED &&
          dimension.distance().ref_kind() != cccad::solver::v1::DISTANCE_REFERENCE_KIND_POINT_POINT) {
        return;
      }
      const auto point_a_it = model.points.find(dimension.distance().ref_a_id());
      const auto point_b_it = model.points.find(dimension.distance().ref_b_id());
      if (point_a_it == model.points.end() || point_b_it == model.points.end()) {
        return;
      }
      double target = dimension.distance().value();
      if (const auto override_it = model.dimension_value_overrides.find(dimension.id());
          override_it != model.dimension_value_overrides.end()) {
        target = override_it->second;
      }
      AddScaledResidual(residuals, Distance(point_a_it->second.x, point_a_it->second.y,
                                            point_b_it->second.x, point_b_it->second.y) -
                                        target);
      break;
    }
    case cccad::solver::v1::Dimension::kRadius: {
      const auto circle_it = model.circles.find(dimension.radius().entity_id());
      if (circle_it == model.circles.end()) {
        return;
      }
      double target = dimension.radius().value();
      if (const auto override_it = model.dimension_value_overrides.find(dimension.id());
          override_it != model.dimension_value_overrides.end()) {
        target = override_it->second;
      }
      AddScaledResidual(residuals, circle_it->second.radius - target);
      break;
    }
    case cccad::solver::v1::Dimension::kDiameter: {
      const auto circle_it = model.circles.find(dimension.diameter().entity_id());
      if (circle_it == model.circles.end()) {
        return;
      }
      double target = dimension.diameter().value() * 0.5;
      if (const auto override_it = model.dimension_value_overrides.find(dimension.id());
          override_it != model.dimension_value_overrides.end()) {
        target = override_it->second * 0.5;
      }
      AddScaledResidual(residuals, circle_it->second.radius - target);
      break;
    }
    case cccad::solver::v1::Dimension::kAngle:
    case cccad::solver::v1::Dimension::KIND_NOT_SET:
      break;
  }
}

std::vector<double> EvaluateResiduals(const cccad::solver::v1::SketchModel& proto_model,
                                      const Stage1Model& model,
                                      const cccad::solver::v1::SolverOptions& options,
                                      bool include_soft_residuals = true,
                                      const std::unordered_set<std::string>* constraint_scope = nullptr,
                                      const std::unordered_set<std::string>* dimension_scope = nullptr) {
  std::vector<double> residuals;

  for (const auto& constraint : proto_model.constraints()) {
    if (!ScopeContains(constraint_scope, constraint.id())) {
      continue;
    }
    AddConstraintResiduals(constraint, model, &residuals);
  }
  for (const auto* constraint : model.extra_constraints) {
    AddConstraintResiduals(*constraint, model, &residuals);
  }
  for (const auto& dimension : proto_model.dimensions()) {
    if (!ScopeContains(dimension_scope, dimension.id())) {
      continue;
    }
    AddDimensionResiduals(dimension, model, &residuals);
  }

  if (!include_soft_residuals) {
    return residuals;
  }

  const double stability_weight = options.stability_weight() > 0.0 && IsFinite(options.stability_weight())
                                      ? options.stability_weight()
                                      : kDefaultStabilityWeight;
  for (const auto& [id, point] : model.points) {
    if (!point.lock_x) {
      AddScaledResidual(&residuals, point.x - point.x0, stability_weight);
    }
    if (!point.lock_y) {
      AddScaledResidual(&residuals, point.y - point.y0, stability_weight);
    }
  }
  for (const auto& [id, circle] : model.circles) {
    if (!circle.lock_radius) {
      AddScaledResidual(&residuals, circle.radius - circle.radius0, stability_weight);
    }
  }

  const double default_intent_weight =
      options.soft_intent_weight() > 0.0 && IsFinite(options.soft_intent_weight())
          ? options.soft_intent_weight()
          : 100.0;
  for (const auto& anchor : model.intent_anchors) {
    const auto point_it = model.points.find(anchor.point_id);
    if (point_it == model.points.end()) {
      continue;
    }
    const double weight = anchor.weight > 0.0 && IsFinite(anchor.weight) ? anchor.weight : default_intent_weight;
    AddScaledResidual(&residuals, point_it->second.x - anchor.x, weight);
    AddScaledResidual(&residuals, point_it->second.y - anchor.y, weight);
  }

  return residuals;
}

std::vector<ResidualEntry> EvaluateDiagnosticResiduals(
    const cccad::solver::v1::SketchModel& proto_model,
    const Stage1Model& model,
    const cccad::solver::v1::SolverOptions& options,
    const std::unordered_set<std::string>* constraint_scope = nullptr,
    const std::unordered_set<std::string>* dimension_scope = nullptr) {
  (void)options;
  std::vector<ResidualEntry> residuals;
  for (const auto& constraint : proto_model.constraints()) {
    if (!ScopeContains(constraint_scope, constraint.id())) {
      continue;
    }
    AddDiagnosticResiduals(constraint, model, &residuals);
  }
  for (const auto* constraint : model.extra_constraints) {
    AddDiagnosticResiduals(*constraint, model, &residuals);
  }
  for (const auto& dimension : proto_model.dimensions()) {
    if (!ScopeContains(dimension_scope, dimension.id())) {
      continue;
    }
    AddDiagnosticDimensionResiduals(dimension, model, &residuals);
  }
  return residuals;
}

double SquaredNorm(const std::vector<double>& values) {
  double result = 0.0;
  for (const double value : values) {
    result += value * value;
  }
  return result;
}

bool SolveLinearSystem(std::vector<std::vector<double>> a, std::vector<double> b,
                       std::vector<double>* x) {
  const std::size_t n = b.size();
  x->assign(n, 0.0);
  if (n == 0) {
    return true;
  }

  for (std::size_t col = 0; col < n; ++col) {
    std::size_t pivot = col;
    double best = std::abs(a[col][col]);
    for (std::size_t row = col + 1; row < n; ++row) {
      const double candidate = std::abs(a[row][col]);
      if (candidate > best) {
        best = candidate;
        pivot = row;
      }
    }
    if (best < 1e-18 || !std::isfinite(best)) {
      return false;
    }
    if (pivot != col) {
      std::swap(a[pivot], a[col]);
      std::swap(b[pivot], b[col]);
    }

    const double diagonal = a[col][col];
    for (std::size_t j = col; j < n; ++j) {
      a[col][j] /= diagonal;
    }
    b[col] /= diagonal;

    for (std::size_t row = 0; row < n; ++row) {
      if (row == col) {
        continue;
      }
      const double factor = a[row][col];
      if (factor == 0.0) {
        continue;
      }
      for (std::size_t j = col; j < n; ++j) {
        a[row][j] -= factor * a[col][j];
      }
      b[row] -= factor * b[col];
    }
  }

  *x = std::move(b);
  return true;
}

std::vector<std::vector<double>> BuildJacobian(
    const cccad::solver::v1::SketchModel& proto_model,
    const Stage1Model& model,
    const cccad::solver::v1::SolverOptions& options,
    const std::vector<Stage1Variable>& variables,
    const std::vector<double>& x,
    const std::vector<double>& residuals,
    const std::unordered_set<std::string>* constraint_scope = nullptr,
    const std::unordered_set<std::string>* dimension_scope = nullptr) {
  const std::size_t rows = residuals.size();
  const std::size_t cols = variables.size();
  std::vector<std::vector<double>> jacobian(rows, std::vector<double>(cols, 0.0));

  for (std::size_t col = 0; col < cols; ++col) {
    std::vector<double> x_eps = x;
    const double h = 1e-6 * (1.0 + std::abs(x[col]));
    x_eps[col] += h;

    Stage1Model perturbed = model;
    WriteVariables(&perturbed, variables, x_eps);
    const std::vector<double> residuals_eps =
        EvaluateResiduals(proto_model, perturbed, options, false, constraint_scope,
                          dimension_scope);

    for (std::size_t row = 0; row < rows; ++row) {
      jacobian[row][col] = (residuals_eps[row] - residuals[row]) / h;
    }
  }

  return jacobian;
}

int32_t MatrixRank(std::vector<std::vector<double>> matrix, double tolerance) {
  if (matrix.empty() || matrix.front().empty()) {
    return 0;
  }
  const std::size_t rows = matrix.size();
  const std::size_t cols = matrix.front().size();
  std::size_t rank = 0;

  for (std::size_t col = 0; col < cols && rank < rows; ++col) {
    std::size_t pivot = rank;
    double best = std::abs(matrix[pivot][col]);
    for (std::size_t row = rank + 1; row < rows; ++row) {
      const double candidate = std::abs(matrix[row][col]);
      if (candidate > best) {
        best = candidate;
        pivot = row;
      }
    }
    if (best <= tolerance || !std::isfinite(best)) {
      continue;
    }

    if (pivot != rank) {
      std::swap(matrix[pivot], matrix[rank]);
    }

    const double diagonal = matrix[rank][col];
    for (std::size_t j = col; j < cols; ++j) {
      matrix[rank][j] /= diagonal;
    }

    for (std::size_t row = 0; row < rows; ++row) {
      if (row == rank) {
        continue;
      }
      const double factor = matrix[row][col];
      if (factor == 0.0) {
        continue;
      }
      for (std::size_t j = col; j < cols; ++j) {
        matrix[row][j] -= factor * matrix[rank][j];
      }
    }
    ++rank;
  }

  return static_cast<int32_t>(rank);
}

int32_t EstimateRankDegreesOfFreedom(
    const cccad::solver::v1::SketchModel& proto_model,
    const Stage1Model& model,
    const cccad::solver::v1::SolverOptions& options,
    const std::unordered_set<std::string>* entity_scope,
    const std::unordered_set<std::string>* constraint_scope,
    const std::unordered_set<std::string>* dimension_scope,
    int32_t* rank_out = nullptr) {
  Stage1Model scratch = model;
  const std::vector<Stage1Variable> variables = BuildVariables(scratch, entity_scope);
  if (variables.empty()) {
    if (rank_out != nullptr) {
      *rank_out = 0;
    }
    return 0;
  }

  const std::vector<double> x = ReadVariables(scratch, variables);
  WriteVariables(&scratch, variables, x);
  const std::vector<double> residuals =
      EvaluateResiduals(proto_model, scratch, options, false, constraint_scope,
                        dimension_scope);
  const double rank_tolerance =
      options.tolerance() > 0.0 && IsFinite(options.tolerance()) ? options.tolerance() * 10.0
                                                                 : 1e-7;
  const int32_t rank = residuals.empty()
                           ? 0
                           : MatrixRank(BuildJacobian(proto_model, scratch, options, variables, x,
                                                      residuals, constraint_scope,
                                                      dimension_scope),
                                        rank_tolerance);
  if (rank_out != nullptr) {
    *rank_out = rank;
  }
  return static_cast<int32_t>(variables.size()) - rank;
}

}  // namespace

Stage1Model BuildStage1Model(const cccad::solver::v1::SketchModel& model) {
  Stage1Model result;

  for (const auto& entity : model.entities()) {
    result.entity_kinds.emplace(entity.id(), entity.kind_case());
    switch (entity.kind_case()) {
      case cccad::solver::v1::Entity::kPoint: {
        Stage1Point point;
        point.id = entity.id();
        point.x = entity.point().x();
        point.y = entity.point().y();
        point.x0 = point.x;
        point.y0 = point.y;
        point.lock_x = entity.point().fixed();
        point.lock_y = entity.point().fixed();
        result.points.emplace(point.id, std::move(point));
        break;
      }
      case cccad::solver::v1::Entity::kLine: {
        Stage1Line line;
        line.id = entity.id();
        line.start_point_id = entity.line().start_point_id();
        line.end_point_id = entity.line().end_point_id();
        result.lines.emplace(line.id, std::move(line));
        break;
      }
      case cccad::solver::v1::Entity::kCircle: {
        Stage1Circle circle;
        circle.id = entity.id();
        circle.center_point_id = entity.circle().center_point_id();
        circle.radius = std::max(entity.circle().radius(), kMinimumRadius);
        circle.radius0 = circle.radius;
        result.circles.emplace(circle.id, std::move(circle));
        break;
      }
      case cccad::solver::v1::Entity::kArc:
      case cccad::solver::v1::Entity::KIND_NOT_SET:
        break;
    }
  }

  auto lock_point = [&](const std::string& point_id) {
    auto point_it = result.points.find(point_id);
    if (point_it != result.points.end()) {
      point_it->second.lock_x = true;
      point_it->second.lock_y = true;
    }
  };

  auto lock_entity = [&](const std::string& entity_id) {
    const auto kind_it = result.entity_kinds.find(entity_id);
    if (kind_it == result.entity_kinds.end()) {
      return;
    }
    switch (kind_it->second) {
      case cccad::solver::v1::Entity::kPoint:
        lock_point(entity_id);
        break;
      case cccad::solver::v1::Entity::kLine: {
        const auto line_it = result.lines.find(entity_id);
        if (line_it != result.lines.end()) {
          lock_point(line_it->second.start_point_id);
          lock_point(line_it->second.end_point_id);
        }
        break;
      }
      case cccad::solver::v1::Entity::kCircle: {
        const auto circle_it = result.circles.find(entity_id);
        if (circle_it != result.circles.end()) {
          lock_point(circle_it->second.center_point_id);
          circle_it->second.lock_radius = true;
        }
        break;
      }
      case cccad::solver::v1::Entity::kArc:
      case cccad::solver::v1::Entity::KIND_NOT_SET:
        break;
    }
  };

  for (const auto& constraint : model.constraints()) {
    if (IsActive(constraint.status()) && constraint.kind_case() == cccad::solver::v1::Constraint::kFixed) {
      lock_entity(constraint.fixed().entity_id());
    }
  }

  return result;
}

Stage1SolveResult SolveStage1Model(const cccad::solver::v1::SketchModel& proto_model,
                                   Stage1Model initial_model,
                                   const cccad::solver::v1::SolverOptions& options,
                                   int32_t default_max_iterations) {
  Stage1SolveResult result;
  result.model = std::move(initial_model);
  const int32_t max_iterations = options.max_iterations() > 0 ? options.max_iterations()
                                                              : default_max_iterations;
  const double tolerance = options.tolerance() > 0.0 && IsFinite(options.tolerance())
                               ? options.tolerance()
                               : kDefaultTolerance;
  auto finalize = [&]() {
    const std::vector<double> hard_residuals =
        EvaluateResiduals(proto_model, result.model, options, false);
    const double hard_residual_norm = std::sqrt(SquaredNorm(hard_residuals));
    result.converged = hard_residual_norm <= std::max(1e-5, tolerance * 10.0);
    result.degrees_of_freedom =
        EstimateRankDegreesOfFreedom(proto_model, result.model, options, nullptr, nullptr, nullptr,
                                     &result.jacobian_rank);
    result.residual_diagnostics =
        BuildStage1ResidualDiagnostics(proto_model, result.model, options);
  };

  const std::vector<Stage1Variable> variables = BuildVariables(result.model);
  if (variables.empty()) {
    const auto residuals = EvaluateResiduals(proto_model, result.model, options);
    result.residual_norm = std::sqrt(SquaredNorm(residuals));
    result.converged = result.residual_norm <= std::max(1e-7, tolerance * 10.0);
    finalize();
    return result;
  }

  std::vector<double> x = ReadVariables(result.model, variables);
  double lambda = 1e-3;

  for (int32_t iteration = 0; iteration < max_iterations; ++iteration) {
    WriteVariables(&result.model, variables, x);
    const std::vector<double> residuals = EvaluateResiduals(proto_model, result.model, options);
    const double current_cost = SquaredNorm(residuals);
    result.residual_norm = std::sqrt(current_cost);
    result.iterations = iteration + 1;
    if (result.residual_norm <= tolerance) {
      result.converged = true;
      finalize();
      return result;
    }

    const std::size_t m = residuals.size();
    const std::size_t n = variables.size();
    std::vector<std::vector<double>> jt_j(n, std::vector<double>(n, 0.0));
    std::vector<double> jt_r(n, 0.0);

    for (std::size_t col = 0; col < n; ++col) {
      std::vector<double> x_eps = x;
      const double h = 1e-6 * (1.0 + std::abs(x[col]));
      x_eps[col] += h;
      Stage1Model perturbed = result.model;
      WriteVariables(&perturbed, variables, x_eps);
      const std::vector<double> residuals_eps = EvaluateResiduals(proto_model, perturbed, options);

      std::vector<double> j_col(m, 0.0);
      for (std::size_t row = 0; row < m; ++row) {
        j_col[row] = (residuals_eps[row] - residuals[row]) / h;
        jt_r[col] += j_col[row] * residuals[row];
      }

      for (std::size_t other = 0; other <= col; ++other) {
        std::vector<double> x_eps_other = x;
        const double h_other = 1e-6 * (1.0 + std::abs(x[other]));
        x_eps_other[other] += h_other;
        Stage1Model perturbed_other = result.model;
        WriteVariables(&perturbed_other, variables, x_eps_other);
        const std::vector<double> residuals_eps_other =
            EvaluateResiduals(proto_model, perturbed_other, options);

        double value = 0.0;
        for (std::size_t row = 0; row < m; ++row) {
          const double j_other = (residuals_eps_other[row] - residuals[row]) / h_other;
          value += j_col[row] * j_other;
        }
        jt_j[col][other] = value;
        jt_j[other][col] = value;
      }
    }

    for (std::size_t i = 0; i < n; ++i) {
      jt_j[i][i] += lambda;
    }

    std::vector<double> rhs(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
      rhs[i] = -jt_r[i];
    }

    std::vector<double> delta;
    if (!SolveLinearSystem(jt_j, rhs, &delta)) {
      lambda *= 10.0;
      continue;
    }

    std::vector<double> candidate = x;
    double step_norm = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
      candidate[i] += delta[i];
      step_norm += delta[i] * delta[i];
    }

    Stage1Model candidate_model = result.model;
    WriteVariables(&candidate_model, variables, candidate);
    const double candidate_cost = SquaredNorm(EvaluateResiduals(proto_model, candidate_model, options));
    if (candidate_cost <= current_cost) {
      x = std::move(candidate);
      result.model = std::move(candidate_model);
      lambda = std::max(lambda * 0.3, 1e-12);
      if (std::sqrt(step_norm) <= tolerance * (1.0 + std::sqrt(SquaredNorm(x)))) {
        result.residual_norm = std::sqrt(candidate_cost);
        result.converged = result.residual_norm <= std::max(1e-7, tolerance * 10.0);
        finalize();
        return result;
      }
    } else {
      lambda = std::min(lambda * 10.0, 1e12);
    }
  }

  WriteVariables(&result.model, variables, x);
  result.residual_norm = std::sqrt(SquaredNorm(EvaluateResiduals(proto_model, result.model, options)));
  result.converged = result.residual_norm <= std::max(1e-7, tolerance * 10.0);
  finalize();
  return result;
}

int32_t EstimateStage1DegreesOfFreedom(
    const cccad::solver::v1::SketchModel& proto_model,
    const Stage1Model& model,
    const cccad::solver::v1::SolverOptions& options,
    const std::vector<std::string>& entity_ids,
    const std::vector<std::string>& constraint_ids,
    const std::vector<std::string>& dimension_ids) {
  const std::unordered_set<std::string> entity_scope = MakeScopeSet(entity_ids);
  const std::unordered_set<std::string> constraint_scope = MakeScopeSet(constraint_ids);
  const std::unordered_set<std::string> dimension_scope = MakeScopeSet(dimension_ids);
  return EstimateRankDegreesOfFreedom(proto_model, model, options,
                                      entity_ids.empty() ? nullptr : &entity_scope,
                                      constraint_ids.empty() ? nullptr : &constraint_scope,
                                      dimension_ids.empty() ? nullptr : &dimension_scope);
}

std::vector<cccad::solver::v1::SolverDiagnostic> BuildStage1ResidualDiagnostics(
    const cccad::solver::v1::SketchModel& proto_model,
    const Stage1Model& model,
    const cccad::solver::v1::SolverOptions& options) {
  const double tolerance =
      options.tolerance() > 0.0 && IsFinite(options.tolerance()) ? options.tolerance() : kDefaultTolerance;
  const double diagnostic_threshold = std::max(1e-5, tolerance * 10.0);
  std::vector<cccad::solver::v1::SolverDiagnostic> diagnostics;

  for (auto& residual : EvaluateDiagnosticResiduals(proto_model, model, options)) {
    if (std::abs(residual.value) <= diagnostic_threshold) {
      continue;
    }
    auto diagnostic = cccad::solver::v1::SolverDiagnostic{};
    diagnostic.set_level(cccad::solver::v1::SOLVER_DIAGNOSTIC_LEVEL_ERROR);
    diagnostic.set_code(residual.diagnostic.code);
    diagnostic.set_message(residual.diagnostic.message);
    SortStrings(&residual.diagnostic.entity_ids);
    SortStrings(&residual.diagnostic.constraint_ids);
    SortStrings(&residual.diagnostic.dimension_ids);
    for (const auto& id : residual.diagnostic.entity_ids) {
      diagnostic.add_entity_ids(id);
    }
    for (const auto& id : residual.diagnostic.constraint_ids) {
      diagnostic.add_constraint_ids(id);
    }
    for (const auto& id : residual.diagnostic.dimension_ids) {
      diagnostic.add_dimension_ids(id);
    }
    diagnostics.push_back(std::move(diagnostic));
  }

  std::sort(diagnostics.begin(), diagnostics.end(), [](const auto& lhs, const auto& rhs) {
    if (lhs.code() != rhs.code()) return lhs.code() < rhs.code();
    if (lhs.constraint_ids_size() != rhs.constraint_ids_size()) {
      return lhs.constraint_ids_size() < rhs.constraint_ids_size();
    }
    if (lhs.constraint_ids_size() > 0 && lhs.constraint_ids(0) != rhs.constraint_ids(0)) {
      return lhs.constraint_ids(0) < rhs.constraint_ids(0);
    }
    if (lhs.dimension_ids_size() > 0 && rhs.dimension_ids_size() > 0 &&
        lhs.dimension_ids(0) != rhs.dimension_ids(0)) {
      return lhs.dimension_ids(0) < rhs.dimension_ids(0);
    }
    return lhs.message() < rhs.message();
  });
  diagnostics.erase(std::unique(diagnostics.begin(), diagnostics.end(),
                                [](const auto& lhs, const auto& rhs) {
                                  return lhs.code() == rhs.code() &&
                                         lhs.message() == rhs.message() &&
                                         RepeatedStringsEqual(lhs.entity_ids(), rhs.entity_ids()) &&
                                         RepeatedStringsEqual(lhs.constraint_ids(), rhs.constraint_ids()) &&
                                         RepeatedStringsEqual(lhs.dimension_ids(), rhs.dimension_ids());
                                }),
                    diagnostics.end());
  return diagnostics;
}

void WriteStage1Solution(const cccad::solver::v1::SketchModel& proto_model,
                         const Stage1Model& model,
                         cccad::solver::v1::SketchSolution* solution) {
  solution->Clear();
  std::vector<const cccad::solver::v1::Entity*> entities;
  entities.reserve(static_cast<std::size_t>(proto_model.entities_size()));
  for (const auto& entity : proto_model.entities()) {
    entities.push_back(&entity);
  }
  std::sort(entities.begin(), entities.end(), [](const auto* lhs, const auto* rhs) {
    return lhs->id() < rhs->id();
  });

  for (const auto* entity : entities) {
    auto* solved = solution->add_entities();
    solved->set_id(entity->id());
    switch (entity->kind_case()) {
      case cccad::solver::v1::Entity::kPoint: {
        const auto point_it = model.points.find(entity->id());
        solved->mutable_point()->set_x(point_it != model.points.end() ? point_it->second.x : entity->point().x());
        solved->mutable_point()->set_y(point_it != model.points.end() ? point_it->second.y : entity->point().y());
        break;
      }
      case cccad::solver::v1::Entity::kLine:
        solved->mutable_line()->set_start_point_id(entity->line().start_point_id());
        solved->mutable_line()->set_end_point_id(entity->line().end_point_id());
        break;
      case cccad::solver::v1::Entity::kCircle: {
        const auto circle_it = model.circles.find(entity->id());
        solved->mutable_circle()->set_center_point_id(entity->circle().center_point_id());
        solved->mutable_circle()->set_radius(circle_it != model.circles.end()
                                                 ? circle_it->second.radius
                                                 : entity->circle().radius());
        break;
      }
      case cccad::solver::v1::Entity::kArc:
        solved->mutable_arc()->set_center_point_id(entity->arc().center_point_id());
        solved->mutable_arc()->set_start_point_id(entity->arc().start_point_id());
        solved->mutable_arc()->set_end_point_id(entity->arc().end_point_id());
        solved->mutable_arc()->set_clockwise(entity->arc().clockwise());
        solved->mutable_arc()->set_branch(entity->arc().branch());
        break;
      case cccad::solver::v1::Entity::KIND_NOT_SET:
        break;
    }
  }
}

}  // namespace cccad::solver
