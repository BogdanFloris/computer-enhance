# Architecture: native (arm64) or x86_64 via Rosetta 2
ARCH       ?= native
BUILD_TYPE ?= Debug

ifeq ($(ARCH),x86)
  CMAKE_EXTRA := -DCMAKE_OSX_ARCHITECTURES=x86_64
  BUILD_DIR   := build-x86
else
  CMAKE_EXTRA :=
  BUILD_DIR   := build
endif

.PHONY: configure build clean sim8086 haversine test

configure:
	cmake -S . -B $(BUILD_DIR) -G Ninja \
		-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
		$(CMAKE_EXTRA)

build: configure
	cmake --build $(BUILD_DIR)

sim8086: build
	./$(BUILD_DIR)/src/sim8086/sim8086 $(ARGS)

haversine: build
	./$(BUILD_DIR)/src/haversine/haversine $(ARGS)

test: build
	ctest --test-dir $(BUILD_DIR) --output-on-failure

clean:
	rm -rf $(BUILD_DIR)
