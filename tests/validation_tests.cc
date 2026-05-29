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

void PointLineDistanceDimensionIsSolved() {
  SolveRequest request;
  auto* a = request.mutable_model()->add_entities();
  a->set_id("a");
  a->mutable_point()->set_x(0.0);
  a->mutable_point()->set_y(0.0);
  a->mutable_point()->set_fixed(true);
  auto* b = request.mutable_model()->add_entities();
  b->set_id("b");
  b->mutable_point()->set_x(10.0);
  b->mutable_point()->set_y(0.0);
  b->mutable_point()->set_fixed(true);
  auto* p = request.mutable_model()->add_entities();
  p->set_id("p");
  p->mutable_point()->set_x(4.0);
  p->mutable_point()->set_y(2.0);
  auto* line = request.mutable_model()->add_entities();
  line->set_id("l1");
  line->mutable_line()->set_start_point_id("a");
  line->mutable_line()->set_end_point_id("b");

  auto* dimension = request.mutable_model()->add_dimensions();
  dimension->set_id("d_point_line");
  dimension->set_driving(true);
  dimension->mutable_distance()->set_ref_a_id("p");
  dimension->mutable_distance()->set_ref_b_id("l1");
  dimension->mutable_distance()->set_ref_kind(cccad::solver::v1::DISTANCE_REFERENCE_KIND_POINT_LINE);
  dimension->mutable_distance()->set_value(6.0);

  SolveResponse response;
  SketchSolverEngine{}.Solve(request, &response);

  double py = 0.0;
  for (const auto& entity : response.solution().entities()) {
    if (entity.id() == "p") {
      py = entity.point().y();
    }
  }
  Require(std::abs(std::abs(py) - 6.0) < 1e-5,
          "point-line distance dimension should be solved");
}

void LineLineDistanceDimensionIsSolved() {
  SolveRequest request;
  auto* a = request.mutable_model()->add_entities();
  a->set_id("a");
  a->mutable_point()->set_x(0.0);
  a->mutable_point()->set_y(0.0);
  a->mutable_point()->set_fixed(true);
  auto* b = request.mutable_model()->add_entities();
  b->set_id("b");
  b->mutable_point()->set_x(10.0);
  b->mutable_point()->set_y(0.0);
  b->mutable_point()->set_fixed(true);
  auto* c = request.mutable_model()->add_entities();
  c->set_id("c");
  c->mutable_point()->set_x(0.0);
  c->mutable_point()->set_y(5.0);
  auto* d = request.mutable_model()->add_entities();
  d->set_id("d");
  d->mutable_point()->set_x(10.0);
  d->mutable_point()->set_y(7.0);
  auto* l1 = request.mutable_model()->add_entities();
  l1->set_id("l1");
  l1->mutable_line()->set_start_point_id("a");
  l1->mutable_line()->set_end_point_id("b");
  auto* l2 = request.mutable_model()->add_entities();
  l2->set_id("l2");
  l2->mutable_line()->set_start_point_id("c");
  l2->mutable_line()->set_end_point_id("d");

  auto* dimension = request.mutable_model()->add_dimensions();
  dimension->set_id("d_line_line");
  dimension->set_driving(true);
  dimension->mutable_distance()->set_ref_a_id("l1");
  dimension->mutable_distance()->set_ref_b_id("l2");
  dimension->mutable_distance()->set_ref_kind(cccad::solver::v1::DISTANCE_REFERENCE_KIND_LINE_LINE);
  dimension->mutable_distance()->set_value(3.0);

  SolveResponse response;
  SketchSolverEngine{}.Solve(request, &response);

  double cy = 0.0;
  double dy = 0.0;
  for (const auto& entity : response.solution().entities()) {
    if (entity.id() == "c") cy = entity.point().y();
    if (entity.id() == "d") dy = entity.point().y();
  }
  Require(std::abs(cy - dy) < 1e-5,
          "line-line distance dimension should make lines parallel");
  Require(std::abs(std::abs(cy) - 3.0) < 1e-5,
          "line-line distance dimension should solve line offset");
}

void AngleDimensionIsSolved() {
  SolveRequest request;
  auto* a = request.mutable_model()->add_entities();
  a->set_id("a");
  a->mutable_point()->set_x(0.0);
  a->mutable_point()->set_y(0.0);
  a->mutable_point()->set_fixed(true);
  auto* b = request.mutable_model()->add_entities();
  b->set_id("b");
  b->mutable_point()->set_x(10.0);
  b->mutable_point()->set_y(0.0);
  b->mutable_point()->set_fixed(true);
  auto* c = request.mutable_model()->add_entities();
  c->set_id("c");
  c->mutable_point()->set_x(2.0);
  c->mutable_point()->set_y(2.0);
  c->mutable_point()->set_fixed(true);
  auto* d = request.mutable_model()->add_entities();
  d->set_id("d");
  d->mutable_point()->set_x(8.0);
  d->mutable_point()->set_y(4.0);
  auto* l1 = request.mutable_model()->add_entities();
  l1->set_id("l1");
  l1->mutable_line()->set_start_point_id("a");
  l1->mutable_line()->set_end_point_id("b");
  auto* l2 = request.mutable_model()->add_entities();
  l2->set_id("l2");
  l2->mutable_line()->set_start_point_id("c");
  l2->mutable_line()->set_end_point_id("d");

  auto* dimension = request.mutable_model()->add_dimensions();
  dimension->set_id("angle");
  dimension->set_driving(true);
  dimension->mutable_angle()->set_line_a_id("l1");
  dimension->mutable_angle()->set_line_b_id("l2");
  dimension->mutable_angle()->set_value_rad(3.14159265358979323846 / 2.0);

  SolveResponse response;
  SketchSolverEngine{}.Solve(request, &response);

  double dx = 1000.0;
  for (const auto& entity : response.solution().entities()) {
    if (entity.id() == "d") {
      dx = entity.point().x() - 2.0;
    }
  }
  Require(std::abs(dx) < 1e-5, "angle dimension should solve line angle");
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

void ArcRadiusDimensionIsSolved() {
  SolveRequest request;
  auto* center = request.mutable_model()->add_entities();
  center->set_id("center");
  center->mutable_point()->set_x(0.0);
  center->mutable_point()->set_y(0.0);
  center->mutable_point()->set_fixed(true);
  auto* start = request.mutable_model()->add_entities();
  start->set_id("start");
  start->mutable_point()->set_x(2.0);
  start->mutable_point()->set_y(0.0);
  auto* end = request.mutable_model()->add_entities();
  end->set_id("end");
  end->mutable_point()->set_x(0.0);
  end->mutable_point()->set_y(2.0);

  auto* arc = request.mutable_model()->add_entities();
  arc->set_id("arc");
  arc->mutable_arc()->set_center_point_id("center");
  arc->mutable_arc()->set_start_point_id("start");
  arc->mutable_arc()->set_end_point_id("end");

  auto* dimension = request.mutable_model()->add_dimensions();
  dimension->set_id("arc_radius");
  dimension->set_driving(true);
  dimension->mutable_radius()->set_entity_id("arc");
  dimension->mutable_radius()->set_value(6.0);

  SolveResponse response;
  SketchSolverEngine{}.Solve(request, &response);

  double sx = 0.0;
  double sy = 0.0;
  double ex = 0.0;
  double ey = 0.0;
  for (const auto& entity : response.solution().entities()) {
    if (entity.id() == "start") {
      sx = entity.point().x();
      sy = entity.point().y();
    } else if (entity.id() == "end") {
      ex = entity.point().x();
      ey = entity.point().y();
    }
  }
  Require(response.status() != SOLVE_STATUS_NUMERICAL_FAILURE,
          "arc radius dimension should converge");
  Require(std::abs(std::sqrt(sx * sx + sy * sy) - 6.0) < 1e-5,
          "arc radius dimension should solve start radius");
  Require(std::abs(std::sqrt(ex * ex + ey * ey) - 6.0) < 1e-5,
          "arc radius dimension should keep end radius consistent");
}

void ArcDiameterDimensionIsSolved() {
  SolveRequest request;
  auto* center = request.mutable_model()->add_entities();
  center->set_id("center");
  center->mutable_point()->set_x(0.0);
  center->mutable_point()->set_y(0.0);
  center->mutable_point()->set_fixed(true);
  auto* start = request.mutable_model()->add_entities();
  start->set_id("start");
  start->mutable_point()->set_x(2.0);
  start->mutable_point()->set_y(0.0);
  auto* end = request.mutable_model()->add_entities();
  end->set_id("end");
  end->mutable_point()->set_x(0.0);
  end->mutable_point()->set_y(2.0);

  auto* arc = request.mutable_model()->add_entities();
  arc->set_id("arc");
  arc->mutable_arc()->set_center_point_id("center");
  arc->mutable_arc()->set_start_point_id("start");
  arc->mutable_arc()->set_end_point_id("end");

  auto* dimension = request.mutable_model()->add_dimensions();
  dimension->set_id("arc_diameter");
  dimension->set_driving(true);
  dimension->mutable_diameter()->set_entity_id("arc");
  dimension->mutable_diameter()->set_value(10.0);

  SolveResponse response;
  SketchSolverEngine{}.Solve(request, &response);

  double sx = 0.0;
  double sy = 0.0;
  double ex = 0.0;
  double ey = 0.0;
  for (const auto& entity : response.solution().entities()) {
    if (entity.id() == "start") {
      sx = entity.point().x();
      sy = entity.point().y();
    } else if (entity.id() == "end") {
      ex = entity.point().x();
      ey = entity.point().y();
    }
  }
  Require(response.status() != SOLVE_STATUS_NUMERICAL_FAILURE,
          "arc diameter dimension should converge");
  Require(std::abs(std::sqrt(sx * sx + sy * sy) - 5.0) < 1e-5,
          "arc diameter dimension should solve start radius");
  Require(std::abs(std::sqrt(ex * ex + ey * ey) - 5.0) < 1e-5,
          "arc diameter dimension should keep end radius consistent");
}

void LineCircleTangentConstraintIsSolved() {
  SolveRequest request;
  auto* a = request.mutable_model()->add_entities();
  a->set_id("a");
  a->mutable_point()->set_x(0.0);
  a->mutable_point()->set_y(0.0);
  a->mutable_point()->set_fixed(true);
  auto* b = request.mutable_model()->add_entities();
  b->set_id("b");
  b->mutable_point()->set_x(10.0);
  b->mutable_point()->set_y(0.0);
  b->mutable_point()->set_fixed(true);
  auto* center = request.mutable_model()->add_entities();
  center->set_id("center");
  center->mutable_point()->set_x(4.0);
  center->mutable_point()->set_y(3.0);
  center->mutable_point()->set_fixed(true);

  auto* line = request.mutable_model()->add_entities();
  line->set_id("line");
  line->mutable_line()->set_start_point_id("a");
  line->mutable_line()->set_end_point_id("b");
  auto* circle = request.mutable_model()->add_entities();
  circle->set_id("circle");
  circle->mutable_circle()->set_center_point_id("center");
  circle->mutable_circle()->set_radius(1.0);

  auto* tangent = request.mutable_model()->add_constraints();
  tangent->set_id("line_circle_tangent");
  tangent->mutable_tangent()->set_entity_a_id("line");
  tangent->mutable_tangent()->set_entity_b_id("circle");

  SolveResponse response;
  SketchSolverEngine{}.Solve(request, &response);

  double radius = 0.0;
  for (const auto& entity : response.solution().entities()) {
    if (entity.id() == "circle") {
      radius = entity.circle().radius();
    }
  }
  Require(response.status() != SOLVE_STATUS_NUMERICAL_FAILURE,
          "line-circle tangent constraint should converge");
  Require(std::abs(radius - 3.0) < 1e-5,
          "line-circle tangent should solve radius to line distance");
}

void ExternalCircleCircleTangentConstraintIsSolved() {
  SolveRequest request;
  auto* c1 = request.mutable_model()->add_entities();
  c1->set_id("c1");
  c1->mutable_point()->set_x(0.0);
  c1->mutable_point()->set_y(0.0);
  c1->mutable_point()->set_fixed(true);
  auto* c2 = request.mutable_model()->add_entities();
  c2->set_id("c2");
  c2->mutable_point()->set_x(10.0);
  c2->mutable_point()->set_y(0.0);
  c2->mutable_point()->set_fixed(true);

  auto* circle1 = request.mutable_model()->add_entities();
  circle1->set_id("circle1");
  circle1->mutable_circle()->set_center_point_id("c1");
  circle1->mutable_circle()->set_radius(3.0);
  auto* circle2 = request.mutable_model()->add_entities();
  circle2->set_id("circle2");
  circle2->mutable_circle()->set_center_point_id("c2");
  circle2->mutable_circle()->set_radius(1.0);

  auto* fixed = request.mutable_model()->add_constraints();
  fixed->set_id("fixed_circle1");
  fixed->mutable_fixed()->set_entity_id("circle1");
  auto* tangent = request.mutable_model()->add_constraints();
  tangent->set_id("external_tangent");
  tangent->mutable_tangent()->set_entity_a_id("circle1");
  tangent->mutable_tangent()->set_entity_b_id("circle2");
  tangent->mutable_tangent()->set_branch(cccad::solver::v1::TANGENT_BRANCH_EXTERNAL);

  SolveResponse response;
  SketchSolverEngine{}.Solve(request, &response);

  double radius = 0.0;
  for (const auto& entity : response.solution().entities()) {
    if (entity.id() == "circle2") {
      radius = entity.circle().radius();
    }
  }
  Require(response.status() != SOLVE_STATUS_NUMERICAL_FAILURE,
          "external circle-circle tangent constraint should converge");
  Require(std::abs(radius - 7.0) < 1e-5,
          "external circle-circle tangent should solve radii sum to center distance");
}

void InternalCircleCircleTangentConstraintIsSolved() {
  SolveRequest request;
  auto* c1 = request.mutable_model()->add_entities();
  c1->set_id("c1");
  c1->mutable_point()->set_x(0.0);
  c1->mutable_point()->set_y(0.0);
  c1->mutable_point()->set_fixed(true);
  auto* c2 = request.mutable_model()->add_entities();
  c2->set_id("c2");
  c2->mutable_point()->set_x(10.0);
  c2->mutable_point()->set_y(0.0);
  c2->mutable_point()->set_fixed(true);

  auto* circle1 = request.mutable_model()->add_entities();
  circle1->set_id("circle1");
  circle1->mutable_circle()->set_center_point_id("c1");
  circle1->mutable_circle()->set_radius(14.0);
  auto* circle2 = request.mutable_model()->add_entities();
  circle2->set_id("circle2");
  circle2->mutable_circle()->set_center_point_id("c2");
  circle2->mutable_circle()->set_radius(1.0);

  auto* fixed = request.mutable_model()->add_constraints();
  fixed->set_id("fixed_circle1");
  fixed->mutable_fixed()->set_entity_id("circle1");
  auto* tangent = request.mutable_model()->add_constraints();
  tangent->set_id("internal_tangent");
  tangent->mutable_tangent()->set_entity_a_id("circle1");
  tangent->mutable_tangent()->set_entity_b_id("circle2");
  tangent->mutable_tangent()->set_branch(cccad::solver::v1::TANGENT_BRANCH_INTERNAL);

  SolveResponse response;
  SketchSolverEngine{}.Solve(request, &response);

  double radius = 0.0;
  for (const auto& entity : response.solution().entities()) {
    if (entity.id() == "circle2") {
      radius = entity.circle().radius();
    }
  }
  Require(response.status() != SOLVE_STATUS_NUMERICAL_FAILURE,
          "internal circle-circle tangent constraint should converge");
  Require(std::abs(radius - 4.0) < 1e-5,
          "internal circle-circle tangent should solve radii difference to center distance");
}

void ArcRadiusConsistencyIsSolved() {
  SolveRequest request;
  auto* center = request.mutable_model()->add_entities();
  center->set_id("center");
  center->mutable_point()->set_x(0.0);
  center->mutable_point()->set_y(0.0);
  center->mutable_point()->set_fixed(true);
  auto* start = request.mutable_model()->add_entities();
  start->set_id("start");
  start->mutable_point()->set_x(5.0);
  start->mutable_point()->set_y(0.0);
  start->mutable_point()->set_fixed(true);
  auto* end = request.mutable_model()->add_entities();
  end->set_id("end");
  end->mutable_point()->set_x(0.0);
  end->mutable_point()->set_y(2.0);

  auto* arc = request.mutable_model()->add_entities();
  arc->set_id("arc");
  arc->mutable_arc()->set_center_point_id("center");
  arc->mutable_arc()->set_start_point_id("start");
  arc->mutable_arc()->set_end_point_id("end");

  SolveResponse response;
  SketchSolverEngine{}.Solve(request, &response);

  double ex = 0.0;
  double ey = 0.0;
  for (const auto& entity : response.solution().entities()) {
    if (entity.id() == "end") {
      ex = entity.point().x();
      ey = entity.point().y();
    }
  }
  Require(std::abs(std::sqrt(ex * ex + ey * ey) - 5.0) < 1e-5,
          "arc radius consistency should solve end radius to start radius");
}

void ArcBranchIsPreserved() {
  SolveRequest request;
  auto* center = request.mutable_model()->add_entities();
  center->set_id("center");
  center->mutable_point()->set_x(0.0);
  center->mutable_point()->set_y(0.0);
  auto* start = request.mutable_model()->add_entities();
  start->set_id("start");
  start->mutable_point()->set_x(1.0);
  start->mutable_point()->set_y(0.0);
  auto* end = request.mutable_model()->add_entities();
  end->set_id("end");
  end->mutable_point()->set_x(0.0);
  end->mutable_point()->set_y(1.0);
  auto* arc = request.mutable_model()->add_entities();
  arc->set_id("arc");
  arc->mutable_arc()->set_center_point_id("center");
  arc->mutable_arc()->set_start_point_id("start");
  arc->mutable_arc()->set_end_point_id("end");
  arc->mutable_arc()->set_clockwise(true);
  arc->mutable_arc()->set_branch(cccad::solver::v1::ARC_BRANCH_MAJOR);

  SolveResponse response;
  SketchSolverEngine{}.Solve(request, &response);

  bool found_branch = false;
  for (const auto& entity : response.solution().entities()) {
    if (entity.id() == "arc") {
      found_branch = entity.arc().clockwise() &&
                     entity.arc().branch() == cccad::solver::v1::ARC_BRANCH_MAJOR;
    }
  }
  Require(found_branch, "arc branch and direction should be preserved in solution");
}

void LineArcTangentConstraintIsSolved() {
  SolveRequest request;
  auto* a = request.mutable_model()->add_entities();
  a->set_id("a");
  a->mutable_point()->set_x(0.0);
  a->mutable_point()->set_y(0.0);
  a->mutable_point()->set_fixed(true);
  auto* b = request.mutable_model()->add_entities();
  b->set_id("b");
  b->mutable_point()->set_x(10.0);
  b->mutable_point()->set_y(0.0);
  b->mutable_point()->set_fixed(true);
  auto* center = request.mutable_model()->add_entities();
  center->set_id("center");
  center->mutable_point()->set_x(4.0);
  center->mutable_point()->set_y(3.0);
  center->mutable_point()->set_fixed(true);
  auto* start = request.mutable_model()->add_entities();
  start->set_id("start");
  start->mutable_point()->set_x(5.0);
  start->mutable_point()->set_y(3.0);
  auto* end = request.mutable_model()->add_entities();
  end->set_id("end");
  end->mutable_point()->set_x(4.0);
  end->mutable_point()->set_y(4.0);

  auto* line = request.mutable_model()->add_entities();
  line->set_id("line");
  line->mutable_line()->set_start_point_id("a");
  line->mutable_line()->set_end_point_id("b");
  auto* arc = request.mutable_model()->add_entities();
  arc->set_id("arc");
  arc->mutable_arc()->set_center_point_id("center");
  arc->mutable_arc()->set_start_point_id("start");
  arc->mutable_arc()->set_end_point_id("end");

  auto* tangent = request.mutable_model()->add_constraints();
  tangent->set_id("line_arc_tangent");
  tangent->mutable_tangent()->set_entity_a_id("line");
  tangent->mutable_tangent()->set_entity_b_id("arc");

  SolveResponse response;
  SketchSolverEngine{}.Solve(request, &response);

  double sx = 0.0;
  double sy = 0.0;
  double ex = 0.0;
  double ey = 0.0;
  for (const auto& entity : response.solution().entities()) {
    if (entity.id() == "start") {
      sx = entity.point().x();
      sy = entity.point().y();
    } else if (entity.id() == "end") {
      ex = entity.point().x();
      ey = entity.point().y();
    }
  }
  const double start_radius = std::sqrt((sx - 4.0) * (sx - 4.0) + (sy - 3.0) * (sy - 3.0));
  const double end_radius = std::sqrt((ex - 4.0) * (ex - 4.0) + (ey - 3.0) * (ey - 3.0));
  Require(response.status() != SOLVE_STATUS_NUMERICAL_FAILURE,
          "line-arc tangent constraint should converge");
  Require(std::abs(start_radius - 3.0) < 1e-5,
          "line-arc tangent should solve arc radius to line distance");
  Require(std::abs(end_radius - start_radius) < 1e-5,
          "line-arc tangent should preserve arc radius consistency");
}

void ArcArcEqualRadiusConstraintIsSolved() {
  SolveRequest request;
  auto* c1 = request.mutable_model()->add_entities();
  c1->set_id("c1");
  c1->mutable_point()->set_x(0.0);
  c1->mutable_point()->set_y(0.0);
  c1->mutable_point()->set_fixed(true);
  auto* s1 = request.mutable_model()->add_entities();
  s1->set_id("s1");
  s1->mutable_point()->set_x(5.0);
  s1->mutable_point()->set_y(0.0);
  s1->mutable_point()->set_fixed(true);
  auto* e1 = request.mutable_model()->add_entities();
  e1->set_id("e1");
  e1->mutable_point()->set_x(0.0);
  e1->mutable_point()->set_y(5.0);
  e1->mutable_point()->set_fixed(true);
  auto* c2 = request.mutable_model()->add_entities();
  c2->set_id("c2");
  c2->mutable_point()->set_x(10.0);
  c2->mutable_point()->set_y(0.0);
  c2->mutable_point()->set_fixed(true);
  auto* s2 = request.mutable_model()->add_entities();
  s2->set_id("s2");
  s2->mutable_point()->set_x(12.0);
  s2->mutable_point()->set_y(0.0);
  auto* e2 = request.mutable_model()->add_entities();
  e2->set_id("e2");
  e2->mutable_point()->set_x(10.0);
  e2->mutable_point()->set_y(2.0);

  auto* arc1 = request.mutable_model()->add_entities();
  arc1->set_id("arc1");
  arc1->mutable_arc()->set_center_point_id("c1");
  arc1->mutable_arc()->set_start_point_id("s1");
  arc1->mutable_arc()->set_end_point_id("e1");
  auto* arc2 = request.mutable_model()->add_entities();
  arc2->set_id("arc2");
  arc2->mutable_arc()->set_center_point_id("c2");
  arc2->mutable_arc()->set_start_point_id("s2");
  arc2->mutable_arc()->set_end_point_id("e2");

  auto* equal = request.mutable_model()->add_constraints();
  equal->set_id("arc_arc_equal_radius");
  equal->mutable_equal()->set_entity_a_id("arc1");
  equal->mutable_equal()->set_entity_b_id("arc2");
  equal->mutable_equal()->set_kind(cccad::solver::v1::EQUAL_KIND_RADIUS);

  SolveResponse response;
  SketchSolverEngine{}.Solve(request, &response);

  double s2x = 0.0;
  double s2y = 0.0;
  double e2x = 0.0;
  double e2y = 0.0;
  for (const auto& entity : response.solution().entities()) {
    if (entity.id() == "s2") {
      s2x = entity.point().x();
      s2y = entity.point().y();
    } else if (entity.id() == "e2") {
      e2x = entity.point().x();
      e2y = entity.point().y();
    }
  }
  const double start_radius = std::sqrt((s2x - 10.0) * (s2x - 10.0) + s2y * s2y);
  const double end_radius = std::sqrt((e2x - 10.0) * (e2x - 10.0) + e2y * e2y);
  Require(std::abs(start_radius - 5.0) < 1e-5,
          "arc-arc equal radius should solve start radius");
  Require(std::abs(end_radius - 5.0) < 1e-5,
          "arc-arc equal radius should solve end radius through arc consistency");
}

void ParallelConstraintIsSolved() {
  SolveRequest request;
  auto* a = request.mutable_model()->add_entities();
  a->set_id("a");
  a->mutable_point()->set_x(0.0);
  a->mutable_point()->set_y(0.0);
  a->mutable_point()->set_fixed(true);
  auto* b = request.mutable_model()->add_entities();
  b->set_id("b");
  b->mutable_point()->set_x(10.0);
  b->mutable_point()->set_y(0.0);
  b->mutable_point()->set_fixed(true);
  auto* c = request.mutable_model()->add_entities();
  c->set_id("c");
  c->mutable_point()->set_x(0.0);
  c->mutable_point()->set_y(5.0);
  c->mutable_point()->set_fixed(true);
  auto* d = request.mutable_model()->add_entities();
  d->set_id("d");
  d->mutable_point()->set_x(10.0);
  d->mutable_point()->set_y(8.0);

  auto* l1 = request.mutable_model()->add_entities();
  l1->set_id("l1");
  l1->mutable_line()->set_start_point_id("a");
  l1->mutable_line()->set_end_point_id("b");
  auto* l2 = request.mutable_model()->add_entities();
  l2->set_id("l2");
  l2->mutable_line()->set_start_point_id("c");
  l2->mutable_line()->set_end_point_id("d");

  auto* constraint = request.mutable_model()->add_constraints();
  constraint->set_id("parallel");
  constraint->mutable_parallel()->set_line_a_id("l1");
  constraint->mutable_parallel()->set_line_b_id("l2");

  SolveResponse response;
  SketchSolverEngine{}.Solve(request, &response);

  double dy = 1000.0;
  for (const auto& entity : response.solution().entities()) {
    if (entity.id() == "d") {
      dy = entity.point().y() - 5.0;
    }
  }
  Require(response.status() != SOLVE_STATUS_NUMERICAL_FAILURE,
          "parallel constraint should converge");
  Require(std::abs(dy) < 1e-5, "parallel constraint should align line directions");
}

void PerpendicularConstraintIsSolved() {
  SolveRequest request;
  auto* a = request.mutable_model()->add_entities();
  a->set_id("a");
  a->mutable_point()->set_x(0.0);
  a->mutable_point()->set_y(0.0);
  a->mutable_point()->set_fixed(true);
  auto* b = request.mutable_model()->add_entities();
  b->set_id("b");
  b->mutable_point()->set_x(10.0);
  b->mutable_point()->set_y(0.0);
  b->mutable_point()->set_fixed(true);
  auto* c = request.mutable_model()->add_entities();
  c->set_id("c");
  c->mutable_point()->set_x(2.0);
  c->mutable_point()->set_y(2.0);
  c->mutable_point()->set_fixed(true);
  auto* d = request.mutable_model()->add_entities();
  d->set_id("d");
  d->mutable_point()->set_x(8.0);
  d->mutable_point()->set_y(8.0);

  auto* l1 = request.mutable_model()->add_entities();
  l1->set_id("l1");
  l1->mutable_line()->set_start_point_id("a");
  l1->mutable_line()->set_end_point_id("b");
  auto* l2 = request.mutable_model()->add_entities();
  l2->set_id("l2");
  l2->mutable_line()->set_start_point_id("c");
  l2->mutable_line()->set_end_point_id("d");

  auto* constraint = request.mutable_model()->add_constraints();
  constraint->set_id("perpendicular");
  constraint->mutable_perpendicular()->set_line_a_id("l1");
  constraint->mutable_perpendicular()->set_line_b_id("l2");

  SolveResponse response;
  SketchSolverEngine{}.Solve(request, &response);

  double solved_dx = 1000.0;
  for (const auto& entity : response.solution().entities()) {
    if (entity.id() == "d") {
      solved_dx = entity.point().x() - 2.0;
    }
  }
  Require(response.status() != SOLVE_STATUS_NUMERICAL_FAILURE,
          "perpendicular constraint should converge");
  Require(std::abs(solved_dx) < 1e-5,
          "perpendicular constraint should make second line vertical");
}

void MidpointConstraintIsSolved() {
  SolveRequest request;
  auto* a = request.mutable_model()->add_entities();
  a->set_id("a");
  a->mutable_point()->set_x(0.0);
  a->mutable_point()->set_y(0.0);
  a->mutable_point()->set_fixed(true);
  auto* b = request.mutable_model()->add_entities();
  b->set_id("b");
  b->mutable_point()->set_x(10.0);
  b->mutable_point()->set_y(4.0);
  b->mutable_point()->set_fixed(true);
  auto* m = request.mutable_model()->add_entities();
  m->set_id("m");
  m->mutable_point()->set_x(1.0);
  m->mutable_point()->set_y(1.0);

  auto* constraint = request.mutable_model()->add_constraints();
  constraint->set_id("midpoint");
  constraint->mutable_midpoint()->set_midpoint_id("m");
  constraint->mutable_midpoint()->set_point_a_id("a");
  constraint->mutable_midpoint()->set_point_b_id("b");

  SolveResponse response;
  SketchSolverEngine{}.Solve(request, &response);

  double mx = 0.0;
  double my = 0.0;
  for (const auto& entity : response.solution().entities()) {
    if (entity.id() == "m") {
      mx = entity.point().x();
      my = entity.point().y();
    }
  }
  Require(std::abs(mx - 5.0) < 1e-5, "midpoint constraint should solve midpoint x");
  Require(std::abs(my - 2.0) < 1e-5, "midpoint constraint should solve midpoint y");
}

void ConcentricConstraintIsSolved() {
  SolveRequest request;
  auto* a = request.mutable_model()->add_entities();
  a->set_id("a");
  a->mutable_point()->set_x(0.0);
  a->mutable_point()->set_y(0.0);
  a->mutable_point()->set_fixed(true);
  auto* b = request.mutable_model()->add_entities();
  b->set_id("b");
  b->mutable_point()->set_x(5.0);
  b->mutable_point()->set_y(7.0);

  auto* c1 = request.mutable_model()->add_entities();
  c1->set_id("c1");
  c1->mutable_circle()->set_center_point_id("a");
  c1->mutable_circle()->set_radius(3.0);
  auto* c2 = request.mutable_model()->add_entities();
  c2->set_id("c2");
  c2->mutable_circle()->set_center_point_id("b");
  c2->mutable_circle()->set_radius(2.0);

  auto* constraint = request.mutable_model()->add_constraints();
  constraint->set_id("concentric");
  constraint->mutable_concentric()->set_circle_a_id("c1");
  constraint->mutable_concentric()->set_circle_b_id("c2");

  SolveResponse response;
  SketchSolverEngine{}.Solve(request, &response);

  double bx = 1000.0;
  double by = 1000.0;
  for (const auto& entity : response.solution().entities()) {
    if (entity.id() == "b") {
      bx = entity.point().x();
      by = entity.point().y();
    }
  }
  Require(std::abs(bx) < 1e-5, "concentric constraint should align center x");
  Require(std::abs(by) < 1e-5, "concentric constraint should align center y");
}

void EqualLengthConstraintIsSolved() {
  SolveRequest request;
  auto* a = request.mutable_model()->add_entities();
  a->set_id("a");
  a->mutable_point()->set_x(0.0);
  a->mutable_point()->set_y(0.0);
  a->mutable_point()->set_fixed(true);
  auto* b = request.mutable_model()->add_entities();
  b->set_id("b");
  b->mutable_point()->set_x(10.0);
  b->mutable_point()->set_y(0.0);
  b->mutable_point()->set_fixed(true);
  auto* c = request.mutable_model()->add_entities();
  c->set_id("c");
  c->mutable_point()->set_x(0.0);
  c->mutable_point()->set_y(5.0);
  c->mutable_point()->set_fixed(true);
  auto* d = request.mutable_model()->add_entities();
  d->set_id("d");
  d->mutable_point()->set_x(3.0);
  d->mutable_point()->set_y(5.0);

  auto* l1 = request.mutable_model()->add_entities();
  l1->set_id("l1");
  l1->mutable_line()->set_start_point_id("a");
  l1->mutable_line()->set_end_point_id("b");
  auto* l2 = request.mutable_model()->add_entities();
  l2->set_id("l2");
  l2->mutable_line()->set_start_point_id("c");
  l2->mutable_line()->set_end_point_id("d");

  auto* constraint = request.mutable_model()->add_constraints();
  constraint->set_id("equal_length");
  constraint->mutable_equal()->set_entity_a_id("l1");
  constraint->mutable_equal()->set_entity_b_id("l2");
  constraint->mutable_equal()->set_kind(cccad::solver::v1::EQUAL_KIND_LENGTH);

  SolveResponse response;
  SketchSolverEngine{}.Solve(request, &response);

  double dx = 0.0;
  double dy = 0.0;
  for (const auto& entity : response.solution().entities()) {
    if (entity.id() == "d") {
      dx = entity.point().x();
      dy = entity.point().y() - 5.0;
    }
  }
  Require(std::abs(std::sqrt(dx * dx + dy * dy) - 10.0) < 1e-5,
          "equal length constraint should match line lengths");
}

void EqualRadiusConstraintIsSolved() {
  SolveRequest request;
  auto* a = request.mutable_model()->add_entities();
  a->set_id("a");
  a->mutable_point()->set_x(0.0);
  a->mutable_point()->set_y(0.0);
  a->mutable_point()->set_fixed(true);
  auto* b = request.mutable_model()->add_entities();
  b->set_id("b");
  b->mutable_point()->set_x(5.0);
  b->mutable_point()->set_y(0.0);
  b->mutable_point()->set_fixed(true);

  auto* c1 = request.mutable_model()->add_entities();
  c1->set_id("c1");
  c1->mutable_circle()->set_center_point_id("a");
  c1->mutable_circle()->set_radius(7.0);
  auto* c2 = request.mutable_model()->add_entities();
  c2->set_id("c2");
  c2->mutable_circle()->set_center_point_id("b");
  c2->mutable_circle()->set_radius(2.0);

  auto* fixed = request.mutable_model()->add_constraints();
  fixed->set_id("fixed_c1");
  fixed->mutable_fixed()->set_entity_id("c1");

  auto* constraint = request.mutable_model()->add_constraints();
  constraint->set_id("equal_radius");
  constraint->mutable_equal()->set_entity_a_id("c1");
  constraint->mutable_equal()->set_entity_b_id("c2");
  constraint->mutable_equal()->set_kind(cccad::solver::v1::EQUAL_KIND_RADIUS);

  SolveResponse response;
  SketchSolverEngine{}.Solve(request, &response);

  double radius = 0.0;
  for (const auto& entity : response.solution().entities()) {
    if (entity.id() == "c2") {
      radius = entity.circle().radius();
    }
  }
  Require(std::abs(radius - 7.0) < 1e-5,
          "equal radius constraint should match circle radii");
}

void RoundArcConstraintsAreSolved() {
  SolveRequest request;
  auto* circle_center = request.mutable_model()->add_entities();
  circle_center->set_id("circle_center");
  circle_center->mutable_point()->set_x(0.0);
  circle_center->mutable_point()->set_y(0.0);
  circle_center->mutable_point()->set_fixed(true);
  auto* arc_center = request.mutable_model()->add_entities();
  arc_center->set_id("arc_center");
  arc_center->mutable_point()->set_x(5.0);
  arc_center->mutable_point()->set_y(0.0);
  auto* arc_start = request.mutable_model()->add_entities();
  arc_start->set_id("arc_start");
  arc_start->mutable_point()->set_x(7.0);
  arc_start->mutable_point()->set_y(0.0);
  auto* arc_end = request.mutable_model()->add_entities();
  arc_end->set_id("arc_end");
  arc_end->mutable_point()->set_x(5.0);
  arc_end->mutable_point()->set_y(2.0);

  auto* circle = request.mutable_model()->add_entities();
  circle->set_id("circle");
  circle->mutable_circle()->set_center_point_id("circle_center");
  circle->mutable_circle()->set_radius(7.0);
  auto* arc = request.mutable_model()->add_entities();
  arc->set_id("arc");
  arc->mutable_arc()->set_center_point_id("arc_center");
  arc->mutable_arc()->set_start_point_id("arc_start");
  arc->mutable_arc()->set_end_point_id("arc_end");

  auto* fixed = request.mutable_model()->add_constraints();
  fixed->set_id("fixed_circle");
  fixed->mutable_fixed()->set_entity_id("circle");

  auto* concentric = request.mutable_model()->add_constraints();
  concentric->set_id("concentric_arc");
  concentric->mutable_concentric()->set_circle_a_id("circle");
  concentric->mutable_concentric()->set_circle_b_id("arc");

  auto* equal_radius = request.mutable_model()->add_constraints();
  equal_radius->set_id("equal_radius_arc");
  equal_radius->mutable_equal()->set_entity_a_id("circle");
  equal_radius->mutable_equal()->set_entity_b_id("arc");
  equal_radius->mutable_equal()->set_kind(cccad::solver::v1::EQUAL_KIND_RADIUS);

  SolveResponse response;
  SketchSolverEngine{}.Solve(request, &response);

  double cx = 1000.0;
  double cy = 1000.0;
  double sx = 0.0;
  double sy = 0.0;
  for (const auto& entity : response.solution().entities()) {
    if (entity.id() == "arc_center") {
      cx = entity.point().x();
      cy = entity.point().y();
    } else if (entity.id() == "arc_start") {
      sx = entity.point().x();
      sy = entity.point().y();
    }
  }
  Require(std::abs(cx) < 1e-5, "concentric should accept arc center x");
  Require(std::abs(cy) < 1e-5, "concentric should accept arc center y");
  Require(std::abs(std::sqrt((sx - cx) * (sx - cx) + (sy - cy) * (sy - cy)) - 7.0) < 1e-5,
          "equal radius should accept arc radius");
}

void EqualLengthRejectsNonLineReferences() {
  CheckRequest request;
  auto* p = request.mutable_model()->add_entities();
  p->set_id("p");
  p->mutable_point()->set_x(0.0);
  p->mutable_point()->set_y(0.0);
  auto* q = request.mutable_model()->add_entities();
  q->set_id("q");
  q->mutable_point()->set_x(1.0);
  q->mutable_point()->set_y(0.0);

  auto* constraint = request.mutable_model()->add_constraints();
  constraint->set_id("bad_equal");
  constraint->mutable_equal()->set_entity_a_id("p");
  constraint->mutable_equal()->set_entity_b_id("q");
  constraint->mutable_equal()->set_kind(cccad::solver::v1::EQUAL_KIND_LENGTH);

  CheckResponse response;
  SketchSolverEngine{}.Check(request, &response);

  Require(response.status() == SOLVE_STATUS_INCONSISTENT,
          "equal length should reject non-line references");
  Require(response.diagnostics_size() == 2,
          "equal length with two point references should produce two diagnostics");
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

void ClosedLineCircuitReturnsExtrudableProfile() {
  SolveRequest request;
  auto add_point = [&](const char* id, double x, double y) {
    auto* point = request.mutable_model()->add_entities();
    point->set_id(id);
    point->mutable_point()->set_x(x);
    point->mutable_point()->set_y(y);
  };
  auto add_line = [&](const char* id, const char* start_id, const char* end_id) {
    auto* line = request.mutable_model()->add_entities();
    line->set_id(id);
    line->mutable_line()->set_start_point_id(start_id);
    line->mutable_line()->set_end_point_id(end_id);
  };
  add_point("p1", 0.0, 0.0);
  add_point("p2", 40.0, 0.0);
  add_point("p3", 40.0, 30.0);
  add_point("p4", 0.0, 30.0);
  add_line("l1", "p1", "p2");
  add_line("l2", "p2", "p3");
  add_line("l3", "p3", "p4");
  add_line("l4", "p4", "p1");

  SolveResponse response;
  SketchSolverEngine{}.Solve(request, &response);

  Require(response.solution().profiles_size() == 1, "closed rectangle should return one profile");
  const auto& profile = response.solution().profiles(0);
  Require(profile.id() == "profile_1", "profile id should be deterministic");
  Require(profile.outer_loop().entity_ids_size() == 4, "rectangle profile should have four border entities");
  Require(profile.outer_loop().entity_ids(0) == "l1", "rectangle loop should start at l1");
  Require(profile.outer_loop().entity_ids(1) == "l2", "rectangle loop should include l2");
  Require(profile.outer_loop().entity_ids(2) == "l3", "rectangle loop should include l3");
  Require(profile.outer_loop().entity_ids(3) == "l4", "rectangle loop should include l4");
  Require(profile.inner_loops_size() == 0, "simple rectangle should not have inner loops");
  Require(std::abs(profile.area() - 1200.0) < 1e-8, "rectangle profile should report area");
  Require(profile.valid_for_extrude(), "closed rectangle should be valid for extrude");
}

void ReservedAxisLinesDoNotInvalidateSolve() {
  SolveRequest request;
  auto add_point = [&](const char* id, double x, double y) {
    auto* point = request.mutable_model()->add_entities();
    point->set_id(id);
    point->mutable_point()->set_x(x);
    point->mutable_point()->set_y(y);
  };
  auto add_line = [&](const char* id, const char* start_id, const char* end_id) {
    auto* line = request.mutable_model()->add_entities();
    line->set_id(id);
    line->mutable_line()->set_start_point_id(start_id);
    line->mutable_line()->set_end_point_id(end_id);
  };
  add_line("x-axis", "", "");
  add_line("y-axis", "", "");
  add_point("p1", 0.0, 0.0);
  add_point("p2", 1.0, 0.0);
  add_point("p3", 1.0, 1.0);
  add_point("p4", 0.0, 1.0);
  add_line("l1", "p1", "p2");
  add_line("l2", "p2", "p3");
  add_line("l3", "p3", "p4");
  add_line("l4", "p4", "p1");

  SolveResponse response;
  SketchSolverEngine{}.Solve(request, &response);

  Require(response.status() != SOLVE_STATUS_INCONSISTENT,
          "reserved axis helpers should not invalidate solve");
  Require(response.solution().profiles_size() == 1,
          "reserved axis helpers should not prevent profile extraction");
  for (const auto& entity : response.solution().entities()) {
    Require(entity.id() != "x-axis" && entity.id() != "y-axis",
            "reserved axis helpers should not be exported as solved sketch entities");
  }
}

void NestedClosedLineCircuitsReturnInnerLoop() {
  SolveRequest request;
  auto add_point = [&](const char* id, double x, double y) {
    auto* point = request.mutable_model()->add_entities();
    point->set_id(id);
    point->mutable_point()->set_x(x);
    point->mutable_point()->set_y(y);
  };
  auto add_line = [&](const char* id, const char* start_id, const char* end_id) {
    auto* line = request.mutable_model()->add_entities();
    line->set_id(id);
    line->mutable_line()->set_start_point_id(start_id);
    line->mutable_line()->set_end_point_id(end_id);
  };
  add_point("p1", 0.0, 0.0);
  add_point("p2", 10.0, 0.0);
  add_point("p3", 10.0, 10.0);
  add_point("p4", 0.0, 10.0);
  add_line("l1", "p1", "p2");
  add_line("l2", "p2", "p3");
  add_line("l3", "p3", "p4");
  add_line("l4", "p4", "p1");
  add_point("q1", 2.0, 2.0);
  add_point("q2", 4.0, 2.0);
  add_point("q3", 4.0, 4.0);
  add_point("q4", 2.0, 4.0);
  add_line("m1", "q1", "q2");
  add_line("m2", "q2", "q3");
  add_line("m3", "q3", "q4");
  add_line("m4", "q4", "q1");

  SolveResponse response;
  SketchSolverEngine{}.Solve(request, &response);

  Require(response.solution().profiles_size() == 1, "nested loops should return one profile");
  const auto& profile = response.solution().profiles(0);
  Require(profile.inner_loops_size() == 1, "nested loops should return one inner loop");
  Require(profile.inner_loops(0).entity_ids_size() == 4, "inner loop should list border entities");
  Require(profile.inner_loops(0).entity_ids(0) == "m1", "inner loop should be deterministic");
  Require(std::abs(profile.area() - 96.0) < 1e-8, "nested profile area should subtract hole");
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

void ApplyIntentSolvesOnlyAffectedComponent() {
  ApplyIntentRequest request;
  auto* moved = request.mutable_model()->add_entities();
  moved->set_id("moved");
  moved->mutable_point()->set_x(0.0);
  moved->mutable_point()->set_y(0.0);

  auto* anchor = request.mutable_model()->add_entities();
  anchor->set_id("anchor");
  anchor->mutable_point()->set_x(0.0);
  anchor->mutable_point()->set_y(10.0);
  anchor->mutable_point()->set_fixed(true);
  auto* unrelated = request.mutable_model()->add_entities();
  unrelated->set_id("unrelated");
  unrelated->mutable_point()->set_x(1.0);
  unrelated->mutable_point()->set_y(10.0);

  auto* dimension = request.mutable_model()->add_dimensions();
  dimension->set_id("unrelated_distance");
  dimension->set_driving(true);
  dimension->mutable_distance()->set_ref_a_id("anchor");
  dimension->mutable_distance()->set_ref_b_id("unrelated");
  dimension->mutable_distance()->set_ref_kind(cccad::solver::v1::DISTANCE_REFERENCE_KIND_POINT_POINT);
  dimension->mutable_distance()->set_value(10.0);

  request.mutable_intent()->mutable_move_point()->set_point_id("moved");
  request.mutable_intent()->mutable_move_point()->mutable_target()->set_x(2.0);
  request.mutable_intent()->mutable_move_point()->mutable_target()->set_y(3.0);
  request.mutable_intent()->mutable_move_point()->set_weight(100.0);

  ApplyIntentResponse response;
  SketchSolverEngine{}.ApplyIntent(request, &response);

  double moved_x = 0.0;
  double moved_y = 0.0;
  double unrelated_x = 0.0;
  for (const auto& entity : response.solution().entities()) {
    if (entity.id() == "moved") {
      moved_x = entity.point().x();
      moved_y = entity.point().y();
    } else if (entity.id() == "unrelated") {
      unrelated_x = entity.point().x();
    }
  }

  Require(response.status() != SOLVE_STATUS_NUMERICAL_FAILURE,
          "scoped apply intent should not fail on unrelated unsolved components");
  Require(std::abs(moved_x - 2.0) < 1e-5 && std::abs(moved_y - 3.0) < 1e-5,
          "scoped apply intent should solve the moved point");
  Require(std::abs(unrelated_x - 1.0) < 1e-5,
          "scoped apply intent should leave disconnected dimensions unsolved");
}

void AddConstraintIntentValidationsAreRejected() {
  auto add_base_entities = [](ApplyIntentRequest* request) {
    auto* a = request->mutable_model()->add_entities();
    a->set_id("a");
    a->mutable_point()->set_x(0.0);
    a->mutable_point()->set_y(0.0);

    auto* b = request->mutable_model()->add_entities();
    b->set_id("b");
    b->mutable_point()->set_x(1.0);
    b->mutable_point()->set_y(1.0);

    auto* line = request->mutable_model()->add_entities();
    line->set_id("l1");
    line->mutable_line()->set_start_point_id("a");
    line->mutable_line()->set_end_point_id("b");
  };

  {
    ApplyIntentRequest request;
    add_base_entities(&request);
    request.mutable_intent()->mutable_add_constraint()->mutable_constraint()
        ->mutable_horizontal()
        ->set_line_id("l1");

    ApplyIntentResponse response;
    SketchSolverEngine{}.ApplyIntent(request, &response);

    Require(response.status() == SOLVE_STATUS_INCONSISTENT,
            "add constraint intent with empty id should be rejected");
    Require(response.diagnostics_size() == 1,
            "add constraint intent with empty id should produce one diagnostic");
    Require(response.diagnostics(0).code() == "invalid_intent",
            "empty add constraint id diagnostic should use a stable code");
  }

  {
    ApplyIntentRequest request;
    add_base_entities(&request);
    auto* existing = request.mutable_model()->add_constraints();
    existing->set_id("c1");
    existing->mutable_fixed()->set_entity_id("a");

    auto* added = request.mutable_intent()->mutable_add_constraint()->mutable_constraint();
    added->set_id("c1");
    added->mutable_horizontal()->set_line_id("l1");

    ApplyIntentResponse response;
    SketchSolverEngine{}.ApplyIntent(request, &response);

    Require(response.status() == SOLVE_STATUS_INCONSISTENT,
            "add constraint intent with duplicate id should be rejected");
    Require(response.diagnostics_size() == 1,
            "add constraint intent with duplicate id should produce one diagnostic");
    Require(response.diagnostics(0).code() == "invalid_intent",
            "duplicate add constraint id diagnostic should use a stable code");
  }

  {
    ApplyIntentRequest request;
    add_base_entities(&request);
    auto* added = request.mutable_intent()->mutable_add_constraint()->mutable_constraint();
    added->set_id("c_new");
    added->mutable_horizontal()->set_line_id("missing");

    ApplyIntentResponse response;
    SketchSolverEngine{}.ApplyIntent(request, &response);

    Require(response.status() == SOLVE_STATUS_INCONSISTENT,
            "add constraint intent with missing reference should be rejected");
    Require(response.diagnostics_size() == 1,
            "add constraint intent with missing reference should produce one diagnostic");
    Require(response.diagnostics(0).code() == "invalid_intent_reference",
            "missing add constraint reference diagnostic should use a stable code");
  }

  {
    ApplyIntentRequest request;
    add_base_entities(&request);
    auto* added = request.mutable_intent()->mutable_add_constraint()->mutable_constraint();
    added->set_id("c_new");
    added->mutable_horizontal()->set_line_id("a");

    ApplyIntentResponse response;
    SketchSolverEngine{}.ApplyIntent(request, &response);

    Require(response.status() == SOLVE_STATUS_INCONSISTENT,
            "add constraint intent with wrong reference type should be rejected");
    Require(response.diagnostics_size() == 1,
            "add constraint intent with wrong reference type should produce one diagnostic");
    Require(response.diagnostics(0).code() == "invalid_intent_reference",
            "wrong add constraint reference type diagnostic should use a stable code");
  }

  {
    ApplyIntentRequest request;
    add_base_entities(&request);
    request.mutable_intent()->mutable_add_constraint()->mutable_constraint()->set_id("c_new");

    ApplyIntentResponse response;
    SketchSolverEngine{}.ApplyIntent(request, &response);

    Require(response.status() == SOLVE_STATUS_INCONSISTENT,
            "add constraint intent with unset kind should be rejected");
    Require(response.diagnostics_size() == 1,
            "add constraint intent with unset kind should produce one diagnostic");
    Require(response.diagnostics(0).code() == "unsupported_constraint",
            "unset add constraint kind diagnostic should use a stable code");
  }
}

void EditIntentsAreExplicitlyUnsupported() {
  auto add_corner_entities = [](ApplyIntentRequest* request) {
    auto* corner = request->mutable_model()->add_entities();
    corner->set_id("corner");
    corner->mutable_point()->set_x(0.0);
    corner->mutable_point()->set_y(0.0);

    auto* a = request->mutable_model()->add_entities();
    a->set_id("a");
    a->mutable_point()->set_x(10.0);
    a->mutable_point()->set_y(0.0);

    auto* b = request->mutable_model()->add_entities();
    b->set_id("b");
    b->mutable_point()->set_x(0.0);
    b->mutable_point()->set_y(10.0);

    auto* l1 = request->mutable_model()->add_entities();
    l1->set_id("l1");
    l1->mutable_line()->set_start_point_id("corner");
    l1->mutable_line()->set_end_point_id("a");

    auto* l2 = request->mutable_model()->add_entities();
    l2->set_id("l2");
    l2->mutable_line()->set_start_point_id("corner");
    l2->mutable_line()->set_end_point_id("b");
  };
  auto require_unsupported = [](const ApplyIntentRequest& request, const char* message) {
    ApplyIntentResponse response;
    SketchSolverEngine{}.ApplyIntent(request, &response);

    Require(response.status() == SOLVE_STATUS_INCONSISTENT, message);
    Require(response.diagnostics_size() == 1,
            "valid-looking unsupported edit intent should produce one diagnostic");
    Require(response.diagnostics(0).code() == "unsupported_intent",
            "unsupported edit intent diagnostic should use a stable code");
  };

  {
    ApplyIntentRequest request;
    add_corner_entities(&request);
    auto* fillet = request.mutable_intent()->mutable_apply_fillet();
    fillet->set_feature_id("fillet1");
    fillet->set_line1_id("l1");
    fillet->set_line2_id("l2");
    fillet->set_corner_point_id("corner");
    fillet->set_created_point1_id("fillet_p1");
    fillet->set_created_point2_id("fillet_p2");
    fillet->set_created_arc_id("fillet_arc");
    fillet->set_radius(1.0);

    require_unsupported(request, "fillet intent should be rejected until implemented");
  }

  {
    ApplyIntentRequest request;
    add_corner_entities(&request);
    auto* chamfer = request.mutable_intent()->mutable_apply_chamfer();
    chamfer->set_feature_id("chamfer1");
    chamfer->set_line1_id("l1");
    chamfer->set_line2_id("l2");
    chamfer->set_corner_point_id("corner");
    chamfer->set_created_point1_id("chamfer_p1");
    chamfer->set_created_point2_id("chamfer_p2");
    chamfer->set_created_line_id("chamfer_line");
    chamfer->set_distance1(1.0);
    chamfer->set_distance2(2.0);

    require_unsupported(request, "chamfer intent should be rejected until implemented");
  }

  {
    ApplyIntentRequest request;
    add_corner_entities(&request);
    auto* fillet = request.mutable_intent()->mutable_update_fillet();
    fillet->set_feature_id("fillet1");
    fillet->set_radius(2.0);

    require_unsupported(request, "update fillet intent should be rejected until implemented");
  }

  {
    ApplyIntentRequest request;
    add_corner_entities(&request);
    auto* chamfer = request.mutable_intent()->mutable_update_chamfer();
    chamfer->set_feature_id("chamfer1");
    chamfer->set_distance1(1.0);
    chamfer->set_distance2(2.0);

    require_unsupported(request, "update chamfer intent should be rejected until implemented");
  }

  {
    ApplyIntentRequest request;
    add_corner_entities(&request);
    auto* split = request.mutable_intent()->mutable_split_entity();
    split->set_entity_id("l1");
    split->mutable_pick_point()->set_x(5.0);
    split->mutable_pick_point()->set_y(0.0);
    split->set_created_point_id("split_p");
    split->add_created_entity_ids("split_l1");
    split->add_created_entity_ids("split_l2");

    require_unsupported(request, "split entity intent should be rejected until implemented");
  }

  {
    ApplyIntentRequest request;
    add_corner_entities(&request);
    auto* break_intent = request.mutable_intent()->mutable_break_entity_at_point();
    break_intent->set_entity_id("l1");
    break_intent->set_point_id("corner");
    break_intent->mutable_pick_point()->set_x(0.0);
    break_intent->mutable_pick_point()->set_y(0.0);
    break_intent->add_created_entity_ids("break_l1");
    break_intent->add_created_entity_ids("break_l2");

    require_unsupported(request,
                        "break entity at point intent should be rejected until implemented");
  }

  {
    ApplyIntentRequest request;
    add_corner_entities(&request);
    auto* trim = request.mutable_intent()->mutable_trim_entity();
    trim->set_entity_id("l1");
    trim->mutable_pick_point()->set_x(8.0);
    trim->mutable_pick_point()->set_y(0.0);
    trim->add_boundary_entity_ids("l2");

    require_unsupported(request, "trim entity intent should be rejected until implemented");
  }

  {
    ApplyIntentRequest request;
    add_corner_entities(&request);
    auto* extend = request.mutable_intent()->mutable_extend_entity();
    extend->set_entity_id("l1");
    extend->set_endpoint("end");
    extend->mutable_target()->set_x(12.0);
    extend->mutable_target()->set_y(0.0);
    extend->add_target_entity_ids("l2");

    require_unsupported(request, "extend entity intent should be rejected until implemented");
  }

  {
    ApplyIntentRequest request;
    add_corner_entities(&request);
    auto* mirror = request.mutable_intent()->mutable_mirror_entities();
    mirror->set_feature_id("mirror1");
    mirror->add_source_entity_ids("l1");
    mirror->set_mirror_line_id("l2");
    mirror->add_created_entity_ids("mirror_l1");

    require_unsupported(request, "mirror entities intent should be rejected until implemented");
  }

  {
    ApplyIntentRequest request;
    add_corner_entities(&request);
    auto* pattern = request.mutable_intent()->mutable_linear_pattern();
    pattern->set_feature_id("linear_pattern1");
    pattern->add_source_entity_ids("l1");
    pattern->mutable_direction()->set_x(1.0);
    pattern->mutable_direction()->set_y(0.0);
    pattern->set_spacing(2.0);
    pattern->set_count(3);
    pattern->add_created_entity_ids("linear_l1_copy1");
    pattern->add_created_entity_ids("linear_l1_copy2");

    require_unsupported(request, "linear pattern intent should be rejected until implemented");
  }

  {
    ApplyIntentRequest request;
    add_corner_entities(&request);
    auto* pattern = request.mutable_intent()->mutable_circular_pattern();
    pattern->set_feature_id("circular_pattern1");
    pattern->add_source_entity_ids("l1");
    pattern->set_center_point_id("corner");
    pattern->set_total_angle_rad(3.14159265358979323846);
    pattern->set_count(4);
    pattern->add_created_entity_ids("circular_l1_copy1");
    pattern->add_created_entity_ids("circular_l1_copy2");
    pattern->add_created_entity_ids("circular_l1_copy3");

    require_unsupported(request, "circular pattern intent should be rejected until implemented");
  }
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
  PointLineDistanceDimensionIsSolved();
  LineLineDistanceDimensionIsSolved();
  AngleDimensionIsSolved();
  CircleRadiusDimensionIsSolved();
  ArcRadiusDimensionIsSolved();
  ArcDiameterDimensionIsSolved();
  LineCircleTangentConstraintIsSolved();
  ExternalCircleCircleTangentConstraintIsSolved();
  InternalCircleCircleTangentConstraintIsSolved();
  ArcRadiusConsistencyIsSolved();
  ArcBranchIsPreserved();
  LineArcTangentConstraintIsSolved();
  ArcArcEqualRadiusConstraintIsSolved();
  ParallelConstraintIsSolved();
  PerpendicularConstraintIsSolved();
  MidpointConstraintIsSolved();
  ConcentricConstraintIsSolved();
  EqualLengthConstraintIsSolved();
  EqualRadiusConstraintIsSolved();
  RoundArcConstraintsAreSolved();
  EqualLengthRejectsNonLineReferences();
  SolvedLineIsReturned();
  ClosedLineCircuitReturnsExtrudableProfile();
  ReservedAxisLinesDoNotInvalidateSolve();
  NestedClosedLineCircuitsReturnInnerLoop();
  AnalyzeReturnsRealComponents();
  ApplyIntentReportsAffectedComponentOnly();
  ApplyIntentSolvesOnlyAffectedComponent();
  AddConstraintIntentValidationsAreRejected();
  EditIntentsAreExplicitlyUnsupported();
  RedundantConstraintsUseJacobianRankForDof();
  UnsatisfiedDimensionReturnsResidualDiagnostic();
  return 0;
}
