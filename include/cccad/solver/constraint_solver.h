#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "sketch_solver.pb.h"

namespace cccad::solver {

struct IntentAnchor {
  std::string point_id;
  double x = 0.0;
  double y = 0.0;
  double weight = 1.0;
};

struct SolverPoint {
  std::string id;
  double x = 0.0;
  double y = 0.0;
  double x0 = 0.0;
  double y0 = 0.0;
  bool lock_x = false;
  bool lock_y = false;
};

struct SolverCircle {
  std::string id;
  std::string center_point_id;
  double radius = 1.0;
  double radius0 = 1.0;
  bool lock_radius = false;
};

struct SolverLine {
  std::string id;
  std::string start_point_id;
  std::string end_point_id;
};

struct SolverArc {
  std::string id;
  std::string center_point_id;
  std::string start_point_id;
  std::string end_point_id;
  bool clockwise = false;
  cccad::solver::v1::ArcBranch branch = cccad::solver::v1::ARC_BRANCH_UNSPECIFIED;
};

struct SolverModel {
  std::unordered_map<std::string, SolverPoint> points;
  std::unordered_map<std::string, SolverCircle> circles;
  std::unordered_map<std::string, SolverLine> lines;
  std::unordered_map<std::string, SolverArc> arcs;
  std::unordered_map<std::string, cccad::solver::v1::Entity::KindCase> entity_kinds;
  std::vector<const cccad::solver::v1::Constraint*> extra_constraints;
  std::unordered_map<std::string, double> dimension_value_overrides;
  std::vector<IntentAnchor> intent_anchors;
};

struct SolverResult {
  SolverModel model;
  bool converged = true;
  double residual_norm = 0.0;
  int32_t iterations = 0;
  int32_t jacobian_rank = 0;
  int32_t degrees_of_freedom = 0;
  std::vector<cccad::solver::v1::SolverDiagnostic> residual_diagnostics;
};

SolverModel BuildSolverModel(const cccad::solver::v1::SketchModel& model);
SolverResult SolveModel(const cccad::solver::v1::SketchModel& proto_model,
                                   SolverModel initial_model,
                                   const cccad::solver::v1::SolverOptions& options,
                                   int32_t default_max_iterations);
SolverResult SolveModelScoped(const cccad::solver::v1::SketchModel& proto_model,
                              SolverModel initial_model,
                              const cccad::solver::v1::SolverOptions& options,
                              int32_t default_max_iterations,
                              const std::vector<std::string>& entity_ids,
                              const std::vector<std::string>& constraint_ids,
                              const std::vector<std::string>& dimension_ids);
int32_t EstimateSolverDegreesOfFreedom(
    const cccad::solver::v1::SketchModel& proto_model,
    const SolverModel& model,
    const cccad::solver::v1::SolverOptions& options,
    const std::vector<std::string>& entity_ids = {},
    const std::vector<std::string>& constraint_ids = {},
    const std::vector<std::string>& dimension_ids = {});
std::vector<cccad::solver::v1::SolverDiagnostic> BuildResidualDiagnostics(
    const cccad::solver::v1::SketchModel& proto_model,
    const SolverModel& model,
    const cccad::solver::v1::SolverOptions& options);
void WriteSolverSolution(const cccad::solver::v1::SketchModel& proto_model,
                         const SolverModel& model,
                         cccad::solver::v1::SketchSolution* solution);

}  // namespace cccad::solver
