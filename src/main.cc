#include <grpcpp/grpcpp.h>

#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include "cccad/solver/sketch_solver_engine.h"
#include "cccad/solver/sketch_solver_service.h"

namespace {

std::string ServerAddressFromEnvironment() {
  const char* value = std::getenv("SOLVER_GRPC_ADDR");
  if (value == nullptr || *value == '\0') {
    return "0.0.0.0:50051";
  }
  return value;
}

int ReadMaxRequestBytes() {
  const char* value = std::getenv("SOLVER_MAX_REQUEST_BYTES");
  if (value == nullptr || *value == '\0') {
    return 16 * 1024 * 1024;
  }

  char* end = nullptr;
  const int64_t parsed = std::strtoll(value, &end, 10);
  if (end == value || *end != '\0' || parsed <= 0) {
    return 16 * 1024 * 1024;
  }
  return static_cast<int>(parsed);
}

}  // namespace

int main() {
  const std::string server_address = ServerAddressFromEnvironment();
  cccad::solver::SketchSolverService service{
      cccad::solver::SketchSolverEngine{cccad::solver::LimitsFromEnvironment()}};

  grpc::ServerBuilder builder;
  builder.SetMaxReceiveMessageSize(ReadMaxRequestBytes());
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);

  std::unique_ptr<grpc::Server> server = builder.BuildAndStart();
  if (server == nullptr) {
    std::cerr << "failed to start cccAD sketch solver on " << server_address << '\n';
    return 1;
  }

  std::cout << "cccad sketch solver listening on " << server_address << '\n';
  server->Wait();
  return 0;
}
