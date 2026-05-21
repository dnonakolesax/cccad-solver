CMAKE ?= cmake
CTEST ?= ctest
CPPLINT ?= python3 -m cpplint
BUILD_DIR ?= build
CMAKE_GENERATOR ?= Ninja
CPPLINT_FLAGS ?= --linelength=120 --filter=-legal/copyright,-build/include_subdir

LINT_SOURCES := $(shell find include src tests -type f \( -name '*.cc' -o -name '*.h' \) | sort)

.PHONY: all configure build test lint check clean help

all: build

configure:
	$(CMAKE) -S . -B $(BUILD_DIR) -G "$(CMAKE_GENERATOR)"

build: configure
	$(CMAKE) --build $(BUILD_DIR) --parallel

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
	@echo "  make test       Build and run tests"
	@echo "  make lint       Run cpplint over include/src/tests"
	@echo "  make check      Run lint and tests"
	@echo "  make clean      Remove $(BUILD_DIR)"
