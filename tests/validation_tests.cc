#include <cstdlib>
#include <iostream>

#include "cccad/solver/sketch_solver_engine.h"

namespace {

using cccad::solver::SketchSolverEngine;
using cccad::solver::SolverLimits;
using cccad::solver::v1::CheckRequest;
using cccad::solver::v1::CheckResponse;
using cccad::solver::v1::SOLVE_STATUS_INCONSISTENT;
using cccad::solver::v1::SOLVE_STATUS_UNDER_CONSTRAINED;

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

}  // namespace

int main() {
  ValidPointIsUnderConstrained();
  DuplicateEntityIdsAreRejected();
  EntityLimitIsEnforced();
  InvalidConstraintReferencesAreRejected();
  return 0;
}
