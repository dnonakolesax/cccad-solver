#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

#include "sketch_solver.pb.h"

namespace cccad::solver {

struct SolverLimits {
  std::size_t max_entities = 10000;
  std::size_t max_constraints = 20000;
  std::size_t max_dimensions = 20000;
  std::size_t max_parameters = 20000;
  int32_t default_max_iterations = 100;
};

class SketchSolverEngine {
 public:
  explicit SketchSolverEngine(SolverLimits limits = {});

  void Solve(const cccad::solver::v1::SolveRequest& request,
             cccad::solver::v1::SolveResponse* response) const;
  void Check(const cccad::solver::v1::CheckRequest& request,
             cccad::solver::v1::CheckResponse* response) const;
  void ApplyIntent(const cccad::solver::v1::ApplyIntentRequest& request,
                   cccad::solver::v1::ApplyIntentResponse* response) const;
  void Analyze(const cccad::solver::v1::AnalyzeRequest& request,
               cccad::solver::v1::AnalyzeResponse* response) const;

 private:
  struct ValidationResult {
    bool ok = true;
    std::vector<cccad::solver::v1::SolverDiagnostic> diagnostics;
  };

  ValidationResult ValidateModel(const cccad::solver::v1::SketchModel& model,
                                 const cccad::solver::v1::SolverOptions& options) const;
  void ValidateIntent(const cccad::solver::v1::UserIntent& intent,
                      const cccad::solver::v1::SketchModel& model,
                      ValidationResult* result) const;
  int32_t EstimateDegreesOfFreedom(
      const cccad::solver::v1::SketchModel& model,
      const cccad::solver::v1::SolverOptions& options,
      const std::vector<std::string>& entity_ids = {},
      const std::vector<std::string>& constraint_ids = {},
      const std::vector<std::string>& dimension_ids = {}) const;
  cccad::solver::v1::SolveStatus StatusForDegreesOfFreedom(int32_t degrees_of_freedom) const;
  void CopyModelToSolution(const cccad::solver::v1::SketchModel& model,
                           cccad::solver::v1::SketchSolution* solution) const;
  void AppendDiagnostic(cccad::solver::v1::SolverDiagnosticLevel level,
                        std::string_view code,
                        std::string_view message,
                        std::vector<std::string> entity_ids,
                        std::vector<std::string> constraint_ids,
                        std::vector<std::string> dimension_ids,
                        ValidationResult* result) const;

  SolverLimits limits_;
};

SolverLimits LimitsFromEnvironment();

}  // namespace cccad::solver
