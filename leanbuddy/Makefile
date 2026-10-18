# LeanBuddy: native tests and tools, and the WebAssembly build of the core for the
# browser simulator.  The firmware builds with PlatformIO (see firmware/).

CC ?= cc
CFLAGS ?= -O2 -g -Wall -Wextra -Wshadow -std=gnu11
CORE := $(wildcard core/*.c)
HDRS := $(wildcard core/*.h)
BUILD := build
TESTS := test_math test_lean test_brain test_render

.PHONY: all test wasm screenshots gallery clean

all: test wasm

$(BUILD):
	mkdir -p $@

$(BUILD)/%: tests/%.c $(CORE) $(HDRS) tests/lbtest.h | $(BUILD)
	$(CC) $(CFLAGS) -Icore -Itests $(CORE) $< -lm -o $@

test: $(addprefix $(BUILD)/,$(TESTS))
	@set -e; for t in $(TESTS); do echo "== $$t"; $(BUILD)/$$t; done

$(BUILD)/render_frames: tools/render_frames.c $(CORE) $(HDRS) | $(BUILD)
	$(CC) $(CFLAGS) -Icore $(CORE) $< -lm -o $@

$(BUILD)/gallery: tools/gallery.c $(CORE) $(HDRS) | $(BUILD)
	$(CC) $(CFLAGS) -Icore $(CORE) $< -lm -o $@

# all expressions on one sheet: docs/expressions.png
gallery: $(BUILD)/gallery
	rm -rf $(BUILD)/gal && mkdir -p $(BUILD)/gal && $(BUILD)/gallery $(BUILD)/gal
	cd $(BUILD)/gal && python3 ../../tools/contact_sheet.py ../../docs/expressions.png \
		$$(for f in *.ppm; do n=$${f%.ppm}; echo "$$f=$${n#*_}"; done)

# Freestanding WebAssembly: no libc, no emscripten, just clang and wasm-ld.
WASM_CC ?= clang
WASM_FLAGS := --target=wasm32 -O2 -nostdlib -ffreestanding -fno-builtin-memset -std=c11 \
	-Wl,--no-entry -Wl,--export-dynamic -Wl,--strip-all -Wl,-z,stack-size=65536 -Wl,--initial-memory=1048576
wasm: sim/leanbuddy.wasm
sim/leanbuddy.wasm: sim/sim_api.c $(CORE) $(HDRS)
	$(WASM_CC) $(WASM_FLAGS) -Icore $(CORE) sim/sim_api.c -o $@

clean:
	rm -rf $(BUILD)
