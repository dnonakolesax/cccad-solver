#include <cmath>
#include <cstdlib>
#include <iostream>

#include "cccad/solver/sketch_solver_engine.h"

namespace {

using cccad::solver::SketchSolverEngine;
using cccad::solver::SolverLimits;
using cccad::solver::v1::CheckRequest;
using cccad::solver::v1::CheckResponse;
using cccad::solver::v1::AnalyzeRequest;
using cccad::solver::v1::AnalyzeResponse;
using cccad::solver::v1::ApplyIntentRequest;
using cccad::solver::v1::ApplyIntentResponse;
using cccad::solver::v1::SolveRequest;
using cccad::solver::v1::SolveResponse;
using cccad::solver::v1::SOLVE_STATUS_INCONSISTENT;
using cccad::solver::v1::SOLVE_STATUS_NUMERICAL_FAILURE;
using cccad::solver::v1::SOLVE_STATUS_UNDER_CONSTRAINED;
using cccad::solver::v1::SOLVE_STATUS_FULLY_CONSTRAINED;

void Require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    std::exit(1);
  }
}

void ValidPointIsUnderConstrained() {
  CheckRequest request;
  auto* point_entity = request.mutable_model()->add_entities();
  point_entity->set_id("p1");
  point_entity->mutable_point()->set_x(1.0);
  point_entity->mutable_point()->set_y(2.0);

  CheckResponse response;
  SketchSolverEngine{}.Check(request, &response);

  Require(response.status() == SOLVE_STATUS_UNDER_CONSTRAINED,
          "single free point should be under constrained");
  Require(response.degrees_of_freedom() == 2, "single free point should have two DOF");
  Require(response.diagnostics_size() == 0, "valid point should not produce diagnostics");
}

void DuplicateEntityIdsAreRejected() {
  CheckRequest request;
  auto* first = request.mutable_model()->add_entities();
  first->set_id("p1");
  first->mutable_point()->set_x(0.0);
  first->mutable_point()->set_y(0.0);
  auto* second = request.mutable_model()->add_entities();
  second->set_id("p1");
  second->mutable_point()->set_x(1.0);
  second->mutable_point()->set_y(1.0);

  CheckResponse response;
  SketchSolverEngine{}.Check(request, &response);

  Require(response.status() == SOLVE_STATUS_INCONSISTENT,
          "duplicate entity ids should be rejected");
  Require(response.diagnostics_size() == 1, "duplicate entity ids should produce one diagnostic");
  Require(response.diagnostics(0).code() == "duplicate_entity_id",
          "duplicate entity diagnostic should use a stable code");
}

void EntityLimitIsEnforced() {
  CheckRequest request;
  auto* point_entity = request.mutable_model()->add_entities();
  point_entity->set_id("p1");
  point_entity->mutable_point()->set_x(0.0);
  point_entity->mutable_point()->set_y(0.0);

  CheckResponse response;
  SketchSolverEngine{SolverLimits{.max_entities = 0}}.Check(request, &response);

  Require(response.status() == SOLVE_STATUS_INCONSISTENT, "entity limit should be enforced");
  Require(response.diagnostics_size() == 1, "entity limit should produce one diagnostic");
  Require(response.diagnostics(0).code() == "entity_limit_exceeded",
          "entity limit diagnostic should use a stable code");
}

void InvalidConstraintReferencesAreRejected() {
  CheckRequest request;
  auto* point_entity = request.mutable_model()->add_entities();
  point_entity->set_id("p1");
  point_entity->mutable_point()->set_x(0.0);
  point_entity->mutable_point()->set_y(0.0);

  auto* constraint = request.mutable_model()->add_constraints();
  constraint->set_id("c1");
  constraint->mutable_coincident()->set_point_a_id("p1");
  constraint->mutable_coincident()->set_point_b_id("missing");

  CheckResponse response;
  SketchSolverEngine{}.Check(request, &response);

  Require(response.status() == SOLVE_STATUS_INCONSISTENT,
          "bad constraint references should be rejected");
  Require(response.diagnostics_size() == 1,
          "bad constraint references should produce one diagnostic");
  Require(response.diagnostics(0).code() == "invalid_constraint_reference",
          "bad constraint reference diagnostic should use a stable code");
}

void HorizontalConstraintIsSolved() {
  SolveRequest request;
  auto* a = request.mutable_model()->add_entities();
  a->set_id("a");
  a->mutable_point()->set_x(0.0);
  a->mutable_point()->set_y(0.0);
  auto* b = request.mutable_model()->add_entities();
  b->set_id("b");
  b->mutable_point()->set_x(10.0);
  b->mutable_point()->set_y(5.0);

  auto* line = request.mutable_model()->add_entities();
  line->set_id("l1");
  line->mutable_line()->set_start_point_id("a");
  line->mutable_line()->set_end_point_id("b");

  auto* fixed = request.mutable_model()->add_constraints();
  fixed->set_id("fix_a");
  fixed->mutable_fixed()->set_entity_id("a");

  auto* horizontal = request.mutable_model()->add_constraints();
  horizontal->set_id("h1");
  horizontal->mutable_horizontal()->set_line_id("l1");

  SolveResponse response;
  SketchSolverEngine{}.Solve(request, &response);

  Require(response.status() == SOLVE_STATUS_UNDER_CONSTRAINED,
          "horizontal line with one fixed endpoint should still be under constrained");
  double solved_b_y = 1000.0;
  for (const auto& entity : response.solution().entities()) {
    if (entity.id() == "b") {
      solved_b_y = entity.point().y();
    }
  }
  Require(std::abs(solved_b_y) < 1e-5, "horizontal constraint should move b.y to a.y");
}

void PointPointDistanceDimensionIsSolved() {
  SolveRequest request;
  auto* a = request.mutable_model()->add_entities();
  a->set_id("a");
  a->mutable_point()->set_x(0.0);
  a->mutable_point()->set_y(0.0);
  auto* b = request.mutable_model()->add_entities();
  b->set_id("b");
  b->mutable_point()->set_x(3.0);
  b->mutable_point()->set_y(0.0);

  auto* fixed = request.mutable_model()->add_constraints();
  fixed->set_id("fix_a");
  fixed->mutable_fixed()->set_entity_id("a");

  auto* dimension = request.mutable_model()->add_dimensions();
  dimension->set_id("d1");
  dimension->set_driving(true);
  dimension->mutable_distance()->set_ref_a_id("a");
  dimension->mutable_distance()->set_ref_b_id("b");
  dimension->mutable_distance()->set_ref_kind(cccad::solver::v1::DISTANCE_REFERENCE_KIND_POINT_POINT);
  dimension->mutable_distance()->set_value(10.0);

  SolveResponse response;
  SketchSolverEngine{}.Solve(request, &response);

  double bx = 0.0;
  double by = 0.0;
  for (const auto& entity : response.solution().entities()) {
    if (entity.id() == "b") {
      bx = entity.point().x();
      by = entity.point().y();
    }
  }
  const double length = std::sqrt(bx * bx + by * by);
  Require(std::abs(length - 10.0) < 1e-5, "distance dimension should be solved");
}

void CircleRadiusDimensionIsSolved() {
  SolveRequest request;
  auto* center = request.mutable_model()->add_entities();
  center->set_id("c0");
  center->mutable_point()->set_x(0.0);
  center->mutable_point()->set_y(0.0);

  auto* circle = request.mutable_model()->add_entities();
  circle->set_id("circle1");
  circle->mutable_circle()->set_center_point_id("c0");
  circle->mutable_circle()->set_radius(2.0);

  auto* dimension = request.mutable_model()->add_dimensions();
  dimension->set_id("r1");
  dimension->set_driving(true);
  dimension->mutable_radius()->set_entity_id("circle1");
  dimension->mutable_radius()->set_value(8.0);

  SolveResponse response;
  SketchSolverEngine{}.Solve(request, &response);

  double radius = 0.0;
  for (const auto& entity : response.solution().entities()) {
    if (entity.id() == "circle1") {
      radius = entity.circle().radius();
    }
  }
  Require(std::abs(radius - 8.0) < 1e-5, "radius dimension should be solved");
}


void SolvedLineIsReturned() {
  SolveRequest request;
  auto* a = request.mutable_model()->add_entities();
  a->set_id("a");
  a->mutable_point()->set_x(0.0);
  a->mutable_point()->set_y(0.0);
  auto* b = request.mutable_model()->add_entities();
  b->set_id("b");
  b->mutable_point()->set_x(1.0);
  b->mutable_point()->set_y(0.0);
  auto* line = request.mutable_model()->add_entities();
  line->set_id("l1");
  line->mutable_line()->set_start_point_id("a");
  line->mutable_line()->set_end_point_id("b");

  SolveResponse response;
  SketchSolverEngine{}.Solve(request, &response);

  bool found_line = false;
  for (const auto& entity : response.solution().entities()) {
    if (entity.id() == "l1") {
      found_line = entity.has_line();
    }
  }
  Require(found_line, "solved solution should preserve line entities");
}

void AnalyzeReturnsRealComponents() {
  AnalyzeRequest request;
  auto* a = request.mutable_model()->add_entities();
  a->set_id("a");
  a->mutable_point()->set_x(0.0);
  a->mutable_point()->set_y(0.0);
  auto* b = request.mutable_model()->add_entities();
  b->set_id("b");
  b->mutable_point()->set_x(1.0);
  b->mutable_point()->set_y(0.0);
  auto* line = request.mutable_model()->add_entities();
  line->set_id("l1");
  line->mutable_line()->set_start_point_id("a");
  line->mutable_line()->set_end_point_id("b");
  auto* isolated = request.mutable_model()->add_entities();
  isolated->set_id("isolated");
  isolated->mutable_point()->set_x(5.0);
  isolated->mutable_point()->set_y(5.0);

  auto* horizontal = request.mutable_model()->add_constraints();
  horizontal->set_id("h1");
  horizontal->mutable_horizontal()->set_line_id("l1");

  AnalyzeResponse response;
  SketchSolverEngine{}.Analyze(request, &response);

  Require(response.components_size() == 2, "analyze should split disconnected sketch regions");
}

void ApplyIntentReportsAffectedComponentOnly() {
  ApplyIntentRequest request;
  auto* a = request.mutable_model()->add_entities();
  a->set_id("a");
  a->mutable_point()->set_x(0.0);
  a->mutable_point()->set_y(0.0);
  auto* b = request.mutable_model()->add_entities();
  b->set_id("b");
  b->mutable_point()->set_x(1.0);
  b->mutable_point()->set_y(0.0);
  auto* line = request.mutable_model()->add_entities();
  line->set_id("l1");
  line->mutable_line()->set_start_point_id("a");
  line->mutable_line()->set_end_point_id("b");
  auto* isolated = request.mutable_model()->add_entities();
  isolated->set_id("isolated");
  isolated->mutable_point()->set_x(10.0);
  isolated->mutable_point()->set_y(10.0);

  request.mutable_intent()->mutable_move_point()->set_point_id("a");
  request.mutable_intent()->mutable_move_point()->mutable_target()->set_x(2.0);
  request.mutable_intent()->mutable_move_point()->mutable_target()->set_y(0.0);
  request.mutable_intent()->mutable_move_point()->set_weight(100.0);

  ApplyIntentResponse response;
  SketchSolverEngine{}.ApplyIntent(request, &response);

  bool has_a = false;
  bool has_line = false;
  bool has_isolated = false;
  for (const auto& id : response.affected_entity_ids()) {
    if (id == "a") has_a = true;
    if (id == "l1") has_line = true;
    if (id == "isolated") has_isolated = true;
  }
  Require(has_a, "affected ids should include moved point");
  Require(has_line, "affected ids should include connected line");
  Require(!has_isolated, "affected ids should not include disconnected entities");
}

void RedundantConstraintsUseJacobianRankForDof() {
  CheckRequest request;
  auto* a = request.mutable_model()->add_entities();
  a->set_id("a");
  a->mutable_point()->set_x(0.0);
  a->mutable_point()->set_y(0.0);
  a->mutable_point()->set_fixed(true);
  auto* b = request.mutable_model()->add_entities();
  b->set_id("b");
  b->mutable_point()->set_x(1.0);
  b->mutable_point()->set_y(0.0);
  auto* line = request.mutable_model()->add_entities();
  line->set_id("l1");
  line->mutable_line()->set_start_point_id("a");
  line->mutable_line()->set_end_point_id("b");

  auto* h1 = request.mutable_model()->add_constraints();
  h1->set_id("h1");
  h1->mutable_horizontal()->set_line_id("l1");
  auto* h2 = request.mutable_model()->add_constraints();
  h2->set_id("h2");
  h2->mutable_horizontal()->set_line_id("l1");

  CheckResponse response;
  SketchSolverEngine{}.Check(request, &response);

  Require(response.status() == SOLVE_STATUS_UNDER_CONSTRAINED,
          "duplicate horizontal constraints should not reduce independent DOF twice");
  Require(response.degrees_of_freedom() == 1,
          "rank Jacobian DOF should report one remaining line endpoint DOF");
}

void UnsatisfiedDimensionReturnsResidualDiagnostic() {
  SolveRequest request;
  auto* a = request.mutable_model()->add_entities();
  a->set_id("a");
  a->mutable_point()->set_x(0.0);
  a->mutable_point()->set_y(0.0);
  a->mutable_point()->set_fixed(true);
  auto* b = request.mutable_model()->add_entities();
  b->set_id("b");
  b->mutable_point()->set_x(1.0);
  b->mutable_point()->set_y(0.0);
  b->mutable_point()->set_fixed(true);

  auto* dimension = request.mutable_model()->add_dimensions();
  dimension->set_id("d_bad");
  dimension->set_driving(true);
  dimension->mutable_distance()->set_ref_a_id("a");
  dimension->mutable_distance()->set_ref_b_id("b");
  dimension->mutable_distance()->set_ref_kind(cccad::solver::v1::DISTANCE_REFERENCE_KIND_POINT_POINT);
  dimension->mutable_distance()->set_value(10.0);

  SolveResponse response;
  SketchSolverEngine{}.Solve(request, &response);

  bool has_dimension_residual = false;
  for (const auto& diagnostic : response.diagnostics()) {
    if (diagnostic.code() == "dimension_residual" &&
        diagnostic.dimension_ids_size() == 1 &&
        diagnostic.dimension_ids(0) == "d_bad") {
      has_dimension_residual = true;
    }
  }

  Require(response.status() == SOLVE_STATUS_NUMERICAL_FAILURE,
          "unsatisfied fixed distance dimension should fail numerically");
  Require(has_dimension_residual,
          "unsatisfied fixed distance dimension should return per-residual diagnostic");
}


}  // namespace

int main() {
  ValidPointIsUnderConstrained();
  DuplicateEntityIdsAreRejected();
  EntityLimitIsEnforced();
  InvalidConstraintReferencesAreRejected();
  HorizontalConstraintIsSolved();
  PointPointDistanceDimensionIsSolved();
  CircleRadiusDimensionIsSolved();
  SolvedLineIsReturned();
  AnalyzeReturnsRealComponents();
  ApplyIntentReportsAffectedComponentOnly();
  RedundantConstraintsUseJacobianRankForDof();
  UnsatisfiedDimensionReturnsResidualDiagnostic();
  return 0;
}
