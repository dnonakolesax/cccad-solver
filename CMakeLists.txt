cmake_minimum_required(VERSION 3.24)

project(cccad_solver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Protobuf REQUIRED)
find_package(gRPC REQUIRED)

set(PROTO_FILE "${CMAKE_CURRENT_SOURCE_DIR}/docs/sketch_solver.proto")
set(GENERATED_PROTO_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(GENERATED_PROTO_SRCS
    "${GENERATED_PROTO_DIR}/sketch_solver.pb.cc"
    "${GENERATED_PROTO_DIR}/sketch_solver.grpc.pb.cc")
set(GENERATED_PROTO_HDRS
    "${GENERATED_PROTO_DIR}/sketch_solver.pb.h"
    "${GENERATED_PROTO_DIR}/sketch_solver.grpc.pb.h")

add_custom_command(
  OUTPUT ${GENERATED_PROTO_SRCS} ${GENERATED_PROTO_HDRS}
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${GENERATED_PROTO_DIR}"
  COMMAND protobuf::protoc
          "--proto_path=${CMAKE_CURRENT_SOURCE_DIR}/docs"
          "--cpp_out=${GENERATED_PROTO_DIR}"
          "--grpc_out=${GENERATED_PROTO_DIR}"
          "--plugin=protoc-gen-grpc=$<TARGET_FILE:gRPC::grpc_cpp_plugin>"
          "${PROTO_FILE}"
  DEPENDS "${PROTO_FILE}"
  VERBATIM)

add_library(cccad_solver_proto STATIC ${GENERATED_PROTO_SRCS} ${GENERATED_PROTO_HDRS})
target_include_directories(cccad_solver_proto PUBLIC "${GENERATED_PROTO_DIR}")
target_link_libraries(cccad_solver_proto PUBLIC protobuf::libprotobuf gRPC::grpc++)

add_library(cccad_solver_core STATIC
  src/constraint_graph.cc
  src/constraint_solver.cc
  src/sketch_solver_engine.cc
  src/sketch_solver_service.cc)
target_include_directories(cccad_solver_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(cccad_solver_core PUBLIC cccad_solver_proto)

add_executable(cccad_solver_server src/main.cc)
target_link_libraries(cccad_solver_server PRIVATE cccad_solver_core)

include(CTest)
if(BUILD_TESTING)
  add_executable(cccad_solver_validation_tests tests/validation_tests.cc)
  target_link_libraries(cccad_solver_validation_tests PRIVATE cccad_solver_core)
  add_test(NAME cccad_solver_validation_tests COMMAND cccad_solver_validation_tests)
endif()
