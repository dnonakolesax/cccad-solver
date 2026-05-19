#include "cccad/solver/sketch_solver_service.h"

namespace cccad::solver {

SketchSolverService::SketchSolverService(SketchSolverEngine engine) : engine_(engine) {}

grpc::Status SketchSolverService::Solve(
    grpc::ServerContext* /*context*/, const cccad::solver::v1::SolveRequest* request,
    cccad::solver::v1::SolveResponse* response) {
  engine_.Solve(*request, response);
  return grpc::Status::OK;
}

grpc::Status SketchSolverService::Check(
    grpc::ServerContext* /*context*/, const cccad::solver::v1::CheckRequest* request,
    cccad::solver::v1::CheckResponse* response) {
  engine_.Check(*request, response);
  return grpc::Status::OK;
}

grpc::Status SketchSolverService::ApplyIntent(
    grpc::ServerContext* /*context*/, const cccad::solver::v1::ApplyIntentRequest* request,
    cccad::solver::v1::ApplyIntentResponse* response) {
  engine_.ApplyIntent(*request, response);
  return grpc::Status::OK;
}

grpc::Status SketchSolverService::Analyze(
    grpc::ServerContext* /*context*/, const cccad::solver::v1::AnalyzeRequest* request,
    cccad::solver::v1::AnalyzeResponse* response) {
  engine_.Analyze(*request, response);
  return grpc::Status::OK;
}

}  // namespace cccad::solver
