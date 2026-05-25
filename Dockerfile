FROM ubuntu:24.04 AS build

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        libgrpc++-dev \
        libprotobuf-dev \
        ninja-build \
        protobuf-compiler \
        protobuf-compiler-grpc \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src

COPY CMakeLists.txt ./
COPY docs ./docs
COPY include ./include
COPY src ./src
COPY tests ./tests

RUN cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build --parallel \
    && ctest --test-dir build --output-on-failure

FROM ubuntu:24.04 AS runtime

ENV DEBIAN_FRONTEND=noninteractive
ENV SOLVER_GRPC_ADDR=0.0.0.0:50051

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        libgrpc++-dev \
        libprotobuf-dev \
    && rm -rf /var/lib/apt/lists/* \
    && useradd --create-home --shell /usr/sbin/nologin solver

WORKDIR /app

COPY --from=build /src/build/cccad_solver_server /app/cccad_solver_server

USER solver

EXPOSE 50051

ENTRYPOINT ["/app/cccad_solver_server"]
