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

struct Stage1Point {
  std::string id;
  double x = 0.0;
  double y = 0.0;
  double x0 = 0.0;
  double y0 = 0.0;
  bool lock_x = false;
  bool lock_y = false;
};

struct Stage1Circle {
  std::string id;
  std::string center_point_id;
  double radius = 1.0;
  double radius0 = 1.0;
  bool lock_radius = false;
};

struct Stage1Line {
  std::string id;
  std::string start_point_id;
  std::string end_point_id;
};

struct Stage1Model {
  std::unordered_map<std::string, Stage1Point> points;
  std::unordered_map<std::string, Stage1Circle> circles;
  std::unordered_map<std::string, Stage1Line> lines;
  std::unordered_map<std::string, cccad::solver::v1::Entity::KindCase> entity_kinds;
  std::vector<const cccad::solver::v1::Constraint*> extra_constraints;
  std::unordered_map<std::string, double> dimension_value_overrides;
  std::vector<IntentAnchor> intent_anchors;
};

struct Stage1SolveResult {
  Stage1Model model;
  bool converged = true;
  double residual_norm = 0.0;
  int32_t iterations = 0;
  int32_t jacobian_rank = 0;
  int32_t degrees_of_freedom = 0;
  std::vector<cccad::solver::v1::SolverDiagnostic> residual_diagnostics;
};

Stage1Model BuildStage1Model(const cccad::solver::v1::SketchModel& model);
Stage1SolveResult SolveStage1Model(const cccad::solver::v1::SketchModel& proto_model,
                                   Stage1Model initial_model,
                                   const cccad::solver::v1::SolverOptions& options,
                                   int32_t default_max_iterations);
int32_t EstimateStage1DegreesOfFreedom(
    const cccad::solver::v1::SketchModel& proto_model,
    const Stage1Model& model,
    const cccad::solver::v1::SolverOptions& options,
    const std::vector<std::string>& entity_ids = {},
    const std::vector<std::string>& constraint_ids = {},
    const std::vector<std::string>& dimension_ids = {});
std::vector<cccad::solver::v1::SolverDiagnostic> BuildStage1ResidualDiagnostics(
    const cccad::solver::v1::SketchModel& proto_model,
    const Stage1Model& model,
    const cccad::solver::v1::SolverOptions& options);
void WriteStage1Solution(const cccad::solver::v1::SketchModel& proto_model,
                         const Stage1Model& model,
                         cccad::solver::v1::SketchSolution* solution);

}  // namespace cccad::solver
