# cccad-solver

C++ gRPC sketch solver service for cccAD.

## Build

```bash
cmake -S . -B build -G Ninja
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

## Run

```bash
SOLVER_GRPC_ADDR=0.0.0.0:50051 ./build/cccad_solver_server
```

The public RPC contract is generated from `docs/sketch_solver.proto`; generated
protobuf files are emitted into the CMake build directory and are not edited by
hand.

Supported environment variables:

- `SOLVER_GRPC_ADDR`, default `0.0.0.0:50051`
- `SOLVER_MAX_ENTITIES`, default `10000`
- `SOLVER_MAX_CONSTRAINTS`, default `20000`
- `SOLVER_MAX_ITERATIONS`, default `100`
- `SOLVER_MAX_REQUEST_BYTES`, default `16777216`
