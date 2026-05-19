# AGENTS.md

## Service Overview

This repository contains the C++ sketch solver service for cccAD.

cccAD is an MVP cloud-based cooperative CAD system. At the current stage, the product focuses on 2D sketching, geometric constraints, collaborative editing, and a future path toward parametric features and finite element analysis.

The C++ solver is a stateless computational service. It receives complete sketch state and user intents from the Go backend over gRPC, solves or analyzes the constraint system, and returns deterministic results.

The service must not contain business logic related to users, permissions, documents, storage, authentication, or realtime delivery.

## Core Responsibility

The solver is responsible only for computational geometry and constraint solving.

It may implement:

- Sketch constraint solving
- Constraint validation
- Intent application
- Degrees-of-freedom analysis
- Overconstrained / underconstrained diagnostics
- Geometry normalization
- Numerical solving
- Constraint graph decomposition
- Warm-started recomputation
- Solver performance diagnostics

It must not implement:

- User authentication
- Keycloak integration
- Document permissions
- PostgreSQL access
- S3 access
- Vault access
- WebSocket delivery
- Realtime collaboration protocol
- Audit logging of business events
- Frontend-specific behavior

The only business-facing client of this service is the Go backend.

## Architecture Boundary

Expected architecture:

```text
Frontend: Vite + React + TypeScript
        |
        | HTTP / WebSocket
        v
Go backend
        |
        | gRPC
        v
C++ sketch-solver service
````

The Go backend owns:

* Keycloak JWT validation
* User/session identity
* Access control
* Document ownership
* Sketch persistence
* S3 object storage
* PostgreSQL metadata
* Realtime WebSocket broadcasts
* API orchestration

The C++ solver owns:

* Geometry
* Constraints
* Numerical solving
* Solver diagnostics
* Performance-critical computation

## Service Contract

The service contract is defined by protobuf.

The solver should expose gRPC methods such as:

* `Solve`
* `Check`
* `ApplyIntent`
* `Analyze`

The exact request and response schemas must follow the `.proto` files in the repository.

Do not introduce ad-hoc JSON APIs for solver operations.

Do not bypass protobuf-generated models for public RPC boundaries.

## Stateless Design

The solver must be stateless between requests.

Allowed:

* In-memory caches for process-local optimization
* Preallocated buffers
* Thread-local scratch memory
* Reusable arenas
* Metrics counters
* Static lookup tables

Not allowed:

* Storing canonical sketch state inside the solver
* Depending on previous requests for correctness
* Persisting documents
* Calling external storage
* Calling authentication services
* Calling backend APIs during solving

A request must contain everything required to compute the result.

## Determinism

The solver should be deterministic for the same input.

Avoid nondeterministic behavior caused by:

* Unordered iteration over hash maps in result construction
* Race-dependent floating-point accumulation
* Random solver initialization without fixed seed
* Parallel reduction without stable ordering
* Time-dependent logic in solving

When hash maps are used internally, convert results to a stable order before returning them.

All output entities, constraints, diagnostics, and errors should be sorted deterministically when practical.

## Performance Goals

This service is performance-critical.

Prefer:

* Contiguous arrays
* Compact IDs
* `uint32_t` handles instead of string IDs in hot paths
* Preallocated buffers
* Arena allocation
* `std::span`
* Value types
* Small fixed-size Eigen matrices
* Component-level parallelism
* Warm-started solving
* Sparse structures where appropriate

Avoid in hot paths:

* `std::string` IDs
* `std::shared_ptr` per entity
* Deep inheritance hierarchies
* Virtual calls per residual
* Heap allocation inside residual evaluation
* Exceptions as control flow
* Logging inside solver iterations
* JSON serialization
* Dynamic maps in tight loops
* Rebuilding unchanged graph components

## Internal Data Model

External IDs from protobuf may be strings.

Internally, the solver should remap them to compact numeric handles:

```text
external entity id -> EntityId
external constraint id -> ConstraintId
```

Recommended internal aliases:

```cpp
using EntityId = uint32_t;
using ConstraintId = uint32_t;
using ComponentId = uint32_t;
```

The hot computational model should use arrays and indices rather than object graphs.

Recommended structure:

```text
SketchModel
  entities[]
  constraints[]
  variables[]
  components[]
  id_remap
```

String IDs should only be used at import/export boundaries.

## Geometry Scope

The solver should support 2D sketch primitives, including:

* Point
* Line
* Polyline
* Arc
* Circle
* Rectangle

Supported constraints may include:

* Coincident
* Horizontal
* Vertical
* Parallel
* Perpendicular
* Tangent
* Equal
* Fixed
* Midpoint
* Concentric
* Distance
* Radius
* Diameter
* Angle

Constraint behavior must be explicit and test-covered.

Do not silently ignore unsupported constraints.

Return structured diagnostics instead.

## Numerical Solving

Use robust numerical methods.

Recommended stack:

* Eigen for linear algebra
* Ceres Solver for initial nonlinear least-squares implementation
* Custom residual/Jacobian code for performance-critical constraints when needed

Ceres is acceptable for MVP implementation, but long-term optimization may require custom solving paths for hot cases.

Every solver change should be benchmarked.

## Error Handling

The solver must return structured errors and diagnostics.

Prefer explicit status objects over process-level failure.

The service should distinguish:

* Invalid input
* Unsupported primitive
* Unsupported constraint
* Inconsistent constraint system
* Overconstrained system
* Underconstrained system
* Numerical convergence failure
* Timeout
* Internal solver error

Do not crash on malformed user-provided sketch data.

Do not use `assert` for user input validation.

Assertions may be used only for internal invariants.

## Security Model

The C++ solver is not a security boundary for users.

Authentication and authorization are handled by the Go backend.

However, the solver must still be robust against hostile input because users may indirectly control sketch data.

Required defensive behavior:

* Validate all protobuf inputs
* Enforce maximum entity counts
* Enforce maximum constraint counts
* Enforce maximum request size
* Enforce solver iteration/time limits
* Avoid unbounded recursion
* Avoid integer overflows in indexing
* Avoid undefined behavior
* Avoid trusting external IDs
* Avoid file system access during request handling

The service must not read secrets from Vault directly.

The service should not require database credentials.

## External Services

The solver should not call external business services.

Allowed external interfaces:

* gRPC server endpoint for solver requests
* Health check endpoint
* Metrics endpoint
* Optional OpenTelemetry exporter
* Optional logging output to stdout/stderr

Forbidden external dependencies:

* PostgreSQL
* S3
* Keycloak
* Vault
* Redis
* NATS
* Kafka
* WebSocket gateway
* Frontend APIs
* Backend HTTP APIs

Any exception to this rule must be justified in architecture documentation.

## Dependencies

Preferred libraries:

* gRPC C++
* Protocol Buffers
* Eigen
* Ceres Solver
* Abseil
* oneTBB
* mimalloc or jemalloc
* spdlog
* fmt
* GoogleTest
* Google Benchmark
* OpenTelemetry C++
* Perfetto, optional
* Google Highway, optional for SIMD hot paths

Do not introduce large dependencies without clear justification.

Do not add a new framework for simple functionality.

## Build System

Use CMake.

Recommended generator:

```bash
cmake -S . -B build -G Ninja
cmake --build build --parallel
```

Recommended release flags:

```text
-O3
-DNDEBUG
LTO enabled where possible
```

Avoid `-march=native` in portable container builds unless explicitly requested.

Use `-march=native` only for deployment-specific builds targeting known CPU architecture.

## Testing

All solver behavior must be testable without running the Go backend.

Required test categories:

* Unit tests for geometry primitives
* Unit tests for each constraint type
* Input validation tests
* Solver convergence tests
* Overconstrained sketch tests
* Underconstrained sketch tests
* Determinism tests
* Serialization/import/export tests
* Regression tests for known problematic sketches
* Performance benchmarks

Use GoogleTest for correctness tests.

Use Google Benchmark for performance tests.

## Benchmarking

Benchmarks should cover at least:

* Small sketch: 10-50 entities
* Medium sketch: 100-500 entities
* Large sketch: 1000+ entities
* Drag/update intent with warm start
* Full solve from cold state
* Constraint graph decomposition
* Residual evaluation
* Jacobian construction
* Import/export overhead

Do not optimize based on assumptions.

Profile before rewriting major code paths.

## Logging

Use structured logs.

Recommended library:

* spdlog

Logging rules:

* Log request-level events
* Log validation failures
* Log solver status
* Log convergence summary
* Do not log every solver iteration by default
* Do not log full sketch payloads in production
* Do not log secrets or tokens
* Do not log user-sensitive data

Debug-level detailed logs may be enabled for local development.

## Metrics

The service should expose metrics such as:

* Request count
* Request duration
* Solve duration
* Import duration
* Export duration
* Number of entities
* Number of constraints
* Number of components
* Iteration count
* Convergence status
* Failure count by reason
* Timeout count
* Memory usage if available

Metrics must not include raw sketch contents.

## Timeouts and Limits

Every solve request must have bounded resource usage.

Recommended limits:

* Maximum entity count
* Maximum constraint count
* Maximum solver iterations
* Maximum wall-clock solve duration
* Maximum request size
* Maximum diagnostic output size

When limits are exceeded, return a structured error instead of crashing or hanging.

## Concurrency

The service may process multiple requests concurrently.

Each request must have isolated mutable state.

Allowed:

* Thread pools
* oneTBB task arenas
* Thread-local scratch buffers
* Immutable shared tables

Avoid:

* Global mutable solver state
* Shared request buffers
* Data races
* Lock-heavy hot paths

Parallelism should be applied at component level first.

Do not parallelize tiny computations unless benchmarks prove a benefit.

## Memory Management

Prefer predictable memory behavior.

Use:

* Stack allocation for small fixed-size values
* Contiguous vectors
* Arena allocation for request-local temporary data
* Reused scratch buffers
* Custom allocators only when measured

Avoid:

* Per-residual heap allocations
* Shared ownership by default
* Long-lived accidental caches
* Large temporary copies
* Returning references to temporary data

## Floating-Point Rules

The solver uses floating-point math.

Be explicit about tolerances.

Do not compare doubles with exact equality except for special sentinel values.

Use named constants for tolerances:

```cpp
constexpr double kDistanceEpsilon = ...;
constexpr double kAngleEpsilon = ...;
constexpr double kResidualTolerance = ...;
```

Tolerance values must be documented and tested.

## Code Style

Use modern C++.

Preferred standard:

```text
C++23
```

General rules:

* Prefer value types
* Prefer `std::vector` for contiguous storage
* Prefer `std::span` for non-owning array views
* Prefer `std::unique_ptr` over `std::shared_ptr`
* Prefer explicit ownership
* Prefer `enum class`
* Prefer `constexpr` for constants
* Prefer small structs for geometry values
* Avoid macros unless necessary
* Avoid hidden global state

Public APIs should be small and explicit.

## Naming

Use clear domain names.

Examples:

```cpp
EntityId
ConstraintId
SketchModel
ConstraintGraph
SolveRequestContext
SolveResult
ResidualBlock
DegreeOfFreedomReport
```

Avoid vague names such as:

```cpp
Manager
Processor
Handler
Data
Object
Thing
```

## Generated Code

Protobuf and gRPC generated files should not be edited manually.

Generated files should either be:

* Built into the CMake output directory, or
* Generated as part of the build process

Do not commit generated files unless the repository policy explicitly requires it.

## Frontend Compatibility

The TypeScript frontend may have a lightweight local preview solver.

The C++ solver remains the authoritative solver.

Frontend preview results must not be trusted as final document state.

The backend should treat frontend operations as intents, not as authoritative geometry.

## Go Backend Compatibility

The Go backend is expected to:

* Validate user access
* Load the current sketch state
* Build protobuf requests
* Call the C++ solver
* Persist accepted results
* Broadcast results over realtime channels

The solver should return enough structured information for the backend to decide whether to accept, reject, or broadcast a result.

## Failure Behavior

The service should fail safely.

On invalid input:

* Return validation diagnostics

On inconsistent constraints:

* Return conflict diagnostics

On numerical failure:

* Return solver status and partial diagnostics if safe

On internal error:

* Return generic internal error
* Log technical details server-side

Do not return stack traces to clients.

Do not terminate the process for one bad request.

## Container Expectations

The service should be container-friendly.

Expected behavior:

* Run as non-root
* Listen on a configurable port
* Read configuration from environment variables or config file
* Log to stdout/stderr
* Expose health check
* Support graceful shutdown
* Avoid writing to the container filesystem during normal operation

Suggested default port:

```text
50051
```

## Configuration

Configuration should be minimal.

Possible environment variables:

```text
SOLVER_GRPC_ADDR=0.0.0.0:50051
SOLVER_MAX_ENTITIES=10000
SOLVER_MAX_CONSTRAINTS=20000
SOLVER_MAX_ITERATIONS=100
SOLVER_TIMEOUT_MS=200
SOLVER_LOG_LEVEL=info
SOLVER_METRICS_ADDR=0.0.0.0:9090
```

Do not require secrets for normal operation.

## Documentation

Document:

* Supported primitives
* Supported constraints
* Numerical tolerances
* Solver statuses
* Error codes
* Performance limits
* gRPC API usage
* Build instructions
* Benchmark instructions
* Known limitations

Every new constraint type must include a short mathematical description.

## Agent Instructions

When modifying this repository:

1. Respect the service boundary.
2. Do not add database, S3, Keycloak, Vault, or WebSocket logic.
3. Keep the solver stateless.
4. Keep hot paths allocation-light.
5. Prefer compact numeric IDs internally.
6. Validate all external input.
7. Preserve deterministic output.
8. Add tests for every behavior change.
9. Add benchmarks for performance-sensitive changes.
10. Do not introduce large dependencies casually.
11. Do not edit generated protobuf files manually.
12. Keep public APIs explicit and small.
13. Return structured diagnostics instead of crashing.
14. Do not optimize blindly; profile first.
15. Keep the Go backend as the only business-facing caller.

## Non-Goals

The C++ solver is not responsible for:

* User management
* Authentication
* Authorization
* Document storage
* Collaborative editing protocol
* Conflict resolution between users
* UI preview behavior
* Project/file management
* Billing
* Audit log persistence
* Long-term version history

Those responsibilities belong to other services.

## MVP Priority

For the MVP, prioritize:

1. Correct protobuf-based service boundary
2. Stable internal sketch model
3. Basic constraints support
4. Deterministic solving
5. Clear diagnostics
6. Input validation
7. Tests
8. Benchmarks
9. Simple containerized deployment

Avoid premature complexity.

The solver must be simple to call, easy to test, fast enough for interactive backend use, and designed so that more advanced CAD functionality can be added later.