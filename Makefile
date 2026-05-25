CMAKE ?= cmake
CTEST ?= ctest
CPPLINT ?= python3 -m cpplint
BUILD_DIR ?= build
CMAKE_GENERATOR ?= Ninja
SOLVER_GRPC_ADDR ?= 0.0.0.0:50051
CPPLINT_FLAGS ?= --linelength=120 --filter=-legal/copyright,-build/include_subdir

LINT_SOURCES := $(shell find include src tests -type f \( -name '*.cc' -o -name '*.h' \) | sort)

.PHONY: all configure build run test lint check clean help

all: build

configure:
	$(CMAKE) -S . -B $(BUILD_DIR) -G "$(CMAKE_GENERATOR)"

build: configure
	$(CMAKE) --build $(BUILD_DIR) --parallel

run: build
	SOLVER_GRPC_ADDR=$(SOLVER_GRPC_ADDR) $(BUILD_DIR)/cccad_solver_server

test: build
	$(CTEST) --test-dir $(BUILD_DIR) --output-on-failure

lint:
	$(CPPLINT) $(CPPLINT_FLAGS) $(LINT_SOURCES)

check: lint test

clean:
	$(CMAKE) -E rm -rf $(BUILD_DIR)

help:
	@echo "Targets:"
	@echo "  make configure  Configure CMake in $(BUILD_DIR)"
	@echo "  make build      Build the solver"
	@echo "  make run        Build and start the solver server"
	@echo "  make test       Build and run tests"
	@echo "  make lint       Run cpplint over include/src/tests"
	@echo "  make check      Run lint and tests"
	@echo "  make clean      Remove $(BUILD_DIR)"
