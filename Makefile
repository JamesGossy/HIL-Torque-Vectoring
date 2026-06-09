export TMPDIR := /tmp

# This Makefile uses POSIX shell idioms (rm -rf, mkdir -p, the clang-format
# detection below), so pin the shell to sh rather than let Windows make default
# to cmd.exe. MSYS2/MinGW and every Unix provide /bin/sh.
SHELL := /bin/sh

# `all` is the default goal regardless of rule order (the generated-header rule
# below would otherwise become the first target, so a bare `make` would only
# regenerate track_data.h and stop).
.DEFAULT_GOAL := all

CC     = gcc
CFLAGS = -Wall -Wextra -std=c11 -O2 -D_POSIX_C_SOURCE=199309L

# Executable suffix: empty on Unix, .exe on Windows. Without this the run steps
# (`&& $(EVAL)` etc.) fail on a native Windows build, where gcc emits a .exe.
ifeq ($(OS),Windows_NT)
  EXE_EXT = .exe
else
  EXE_EXT =
endif

HIL_BUILD = HIL_Firmware/build
ECU_BUILD = ECU_Firmware/build
HIL_SIM   = $(HIL_BUILD)/hil_sim$(EXE_EXT)
ECU_OBJ   = $(ECU_BUILD)/torque_vectoring.o
TEST_TV   = $(HIL_BUILD)/test_tv$(EXE_EXT)
TEST_VM   = $(HIL_BUILD)/test_vehicle_model$(EXE_EXT)
TEST_PP   = $(HIL_BUILD)/test_path_planning$(EXE_EXT)
TEST_MC   = $(HIL_BUILD)/test_motion_control$(EXE_EXT)
TEST_LQR  = $(HIL_BUILD)/test_lqr$(EXE_EXT)
TEST_INT  = $(HIL_BUILD)/test_integration$(EXE_EXT)

HIL_FLAGS = $(CFLAGS) \
            -I HIL_Firmware/include \
            -I shared \
            -I ECU_Firmware/include

ECU_FLAGS = $(CFLAGS) \
            -I ECU_Firmware/include \
            -I shared

# Track cone data is generated from tracks/*.yaml (the source of truth) into
# HIL_Firmware/include/track_data.h by tools/gen_tracks.py, which runs before any
# compile. track_parser.c includes the generated header. Editing a YAML (or
# adding tracks/<name>.yaml) regenerates it on the next build.
PYTHON     ?= python
TRACK_YAML  = $(wildcard tracks/*.yaml)
TRACK_DATA  = HIL_Firmware/include/track_data.h

$(TRACK_DATA): $(TRACK_YAML) tools/gen_tracks.py
	$(PYTHON) tools/gen_tracks.py

# Sources clang-format owns: every hand-written C file. track_data.h is
# generated (gen_tracks.py controls its layout) so it is deliberately excluded.
#
# Find clang-format. Prefer one on PATH; otherwise fall back to the default
# Windows LLVM install dir, since the LLVM/winget installer does not always add
# it to the PATH of an already-open shell. Detection runs in the shell (GNU make
# cannot handle the space in "Program Files" via $(wildcard)); the chosen path
# may contain a space, so the recipes quote "$(CLANG_FORMAT)".
#
# This is a recursively-expanded (=) variable so the detection only runs when a
# format target actually uses it - `make all`/`eval`/`test` never evaluate it,
# and so never depend on a POSIX shell being present. Override with
# `make format CLANG_FORMAT=/path/to/clang-format`.
CLANG_FORMAT ?= $(shell command -v clang-format 2>/dev/null \
    || for p in "/c/Program Files/LLVM/bin/clang-format.exe" \
                "/c/Program Files (x86)/LLVM/bin/clang-format.exe"; do \
         [ -x "$$p" ] && { echo "$$p"; break; }; \
       done \
    || echo clang-format)
FORMAT_SRCS   = $(wildcard HIL_Firmware/src/*.c HIL_Firmware/include/*.h \
                           ECU_Firmware/src/*.c ECU_Firmware/include/*.h \
                           shared/*.h tests/*.c tools/*.c)
FORMAT_SRCS  := $(filter-out HIL_Firmware/include/track_data.h, $(FORMAT_SRCS))

# Steering is the model-based LQR law (HIL_Firmware/src/lqr_steer.c); the tuned
# gains are the in-source defaults (from the robustness-aware sweep, see
# tools/tool_smart_sweep_lqr.py), so no -D overrides are needed for a clean lap.
HIL_SRCS = HIL_Firmware/src/main.c \
           HIL_Firmware/src/vehicle_model.c \
           HIL_Firmware/src/track_parser.c \
           HIL_Firmware/src/path_planning.c \
           HIL_Firmware/src/motion_control.c \
           HIL_Firmware/src/lqr_steer.c \
           ECU_Firmware/src/torque_vectoring.c

HIL_OBJS = $(patsubst %.c, $(HIL_BUILD)/%.o, $(notdir $(HIL_SRCS)))

VM_SRCS = tests/test_vehicle_model.c \
          HIL_Firmware/src/vehicle_model.c

PP_SRCS = tests/test_path_planning.c \
          HIL_Firmware/src/path_planning.c

MC_SRCS = tests/test_motion_control.c \
          HIL_Firmware/src/motion_control.c \
          HIL_Firmware/src/vehicle_model.c \
          HIL_Firmware/src/path_planning.c \
          HIL_Firmware/src/lqr_steer.c

LQR_SRCS = tests/test_lqr.c \
           HIL_Firmware/src/lqr_steer.c

TV_SRCS = tests/test_tv.c \
          ECU_Firmware/src/torque_vectoring.c

# Integration test: the full driver -> ECU -> vehicle -> track loop, so it pulls
# in every module the sim wires together.
INT_SRCS = tests/test_integration.c \
           HIL_Firmware/src/motion_control.c \
           HIL_Firmware/src/vehicle_model.c \
           HIL_Firmware/src/track_parser.c \
           HIL_Firmware/src/path_planning.c \
           HIL_Firmware/src/lqr_steer.c \
           ECU_Firmware/src/torque_vectoring.c

EVAL     = $(HIL_BUILD)/eval_lap$(EXE_EXT)
EVAL_SRCS = tools/tool_eval_lap.c \
            HIL_Firmware/src/motion_control.c \
            HIL_Firmware/src/vehicle_model.c \
            HIL_Firmware/src/track_parser.c \
            HIL_Firmware/src/path_planning.c \
            HIL_Firmware/src/lqr_steer.c \
            ECU_Firmware/src/torque_vectoring.c

PERF      = $(HIL_BUILD)/perf_sim$(EXE_EXT)
PERF_SRCS = tools/tool_perf_sim.c \
            HIL_Firmware/src/motion_control.c \
            HIL_Firmware/src/vehicle_model.c \
            HIL_Firmware/src/track_parser.c \
            HIL_Firmware/src/path_planning.c \
            HIL_Firmware/src/lqr_steer.c \
            ECU_Firmware/src/torque_vectoring.c

.PHONY: all run eval test perf clean format format-check

all: $(TRACK_DATA) $(HIL_BUILD) $(ECU_BUILD) $(HIL_SIM) $(ECU_OBJ)

$(HIL_BUILD) $(ECU_BUILD):
	mkdir -p $@

# HIL sim objects (track_parser.o needs the generated track_data.h)
$(HIL_BUILD)/%.o: HIL_Firmware/src/%.c $(TRACK_DATA) | $(HIL_BUILD)
	$(CC) $(HIL_FLAGS) -c -o $@ $<

$(HIL_BUILD)/torque_vectoring.o: ECU_Firmware/src/torque_vectoring.c | $(HIL_BUILD)
	$(CC) $(HIL_FLAGS) -c -o $@ $<

$(HIL_SIM): $(HIL_OBJS)
	$(CC) $(HIL_FLAGS) -o $@ $^ -lm

# ECU boundary check — compile torque_vectoring.c with only shared/ and its own
# headers to verify the ECU has no accidental dependency on HIL_Firmware/.
$(ECU_OBJ): ECU_Firmware/src/torque_vectoring.c | $(ECU_BUILD)
	$(CC) $(ECU_FLAGS) -c -o $@ $<

test: $(TRACK_DATA) $(HIL_BUILD)
	$(CC) $(HIL_FLAGS) -o $(TEST_TV) $(TV_SRCS) -lm && $(TEST_TV)
	$(CC) $(HIL_FLAGS) -o $(TEST_VM) $(VM_SRCS) -lm && $(TEST_VM)
	$(CC) $(HIL_FLAGS) -o $(TEST_PP) $(PP_SRCS) -lm && $(TEST_PP)
	$(CC) $(HIL_FLAGS) -o $(TEST_MC) $(MC_SRCS) -lm && $(TEST_MC)
	$(CC) $(HIL_FLAGS) -o $(TEST_LQR) $(LQR_SRCS) -lm && $(TEST_LQR)
	$(CC) $(HIL_FLAGS) -o $(TEST_INT) $(INT_SRCS) -lm && $(TEST_INT)

run: all
	$(HIL_SIM)

# Headless lap evaluation: runs the full motion-control -> ECU -> vehicle loop
# as fast as possible and prints lap-tracking metrics (mean/worst cross-track
# error, cone contacts, lap time). Use it to catch driver regressions.
eval: $(TRACK_DATA) $(HIL_BUILD)
	$(CC) $(HIL_FLAGS) -o $(EVAL) $(EVAL_SRCS) -lm && $(EVAL)

# Compute-speed benchmark: runs the tick loop flat out for 1 wall-clock second
# and reports throughput. Pass a different budget as arg 1 (e.g. perf_sim 5).
perf: $(TRACK_DATA) $(HIL_BUILD)
	$(CC) $(HIL_FLAGS) -o $(PERF) $(PERF_SRCS) -lm && $(PERF)

clean:
	rm -rf $(HIL_BUILD) $(ECU_BUILD)

# Reformat all hand-written C in place to the .clang-format house style.
format:
	"$(CLANG_FORMAT)" -i $(FORMAT_SRCS)

# Check formatting without editing (non-zero exit if any file is off-style).
# Use in CI to keep the tree formatted.
format-check:
	"$(CLANG_FORMAT)" --dry-run --Werror $(FORMAT_SRCS)
