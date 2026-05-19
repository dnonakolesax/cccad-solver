#pragma once

#include "cccad/solver/sketch_solver_engine.h"
#include "sketch_solver.grpc.pb.h"

namespace cccad::solver {

class SketchSolverService final : public cccad::solver::v1::SketchSolver::Service {
 public:
  explicit SketchSolverService(SketchSolverEngine engine);

  grpc::Status Solve(grpc::ServerContext* context,
                     const cccad::solver::v1::SolveRequest* request,
                     cccad::solver::v1::SolveResponse* response) override;
  grpc::Status Check(grpc::ServerContext* context,
                     const cccad::solver::v1::CheckRequest* request,
                     cccad::solver::v1::CheckResponse* response) override;
  grpc::Status ApplyIntent(grpc::ServerContext* context,
                           const cccad::solver::v1::ApplyIntentRequest* request,
                           cccad::solver::v1::ApplyIntentResponse* response) override;
  grpc::Status Analyze(grpc::ServerContext* context,
                       const cccad::solver::v1::AnalyzeRequest* request,
                       cccad::solver::v1::AnalyzeResponse* response) override;

 private:
  SketchSolverEngine engine_;
};

}  // namespace cccad::solver
