export TMPDIR := /tmp

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

HIL_FLAGS = $(CFLAGS) \
            -I HIL_Firmware/include \
            -I shared \
            -I ECU_Firmware/include

ECU_FLAGS = $(CFLAGS) \
            -I ECU_Firmware/include \
            -I shared

# Steering is the model-based LQR law (HIL_Firmware/src/lqr_steer.c); the tuned
# gains are the in-source defaults (from the robustness-aware sweep, see
# tools/tool_smart_sweep_lqr.py), so no -D overrides are needed for a clean ~26.5 s lap.
HIL_SRCS = HIL_Firmware/src/main.c \
           HIL_Firmware/src/vehicle_model.c \
           HIL_Firmware/src/track.c \
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

EVAL     = $(HIL_BUILD)/eval_lap$(EXE_EXT)
EVAL_SRCS = tools/tool_eval_lap.c \
            HIL_Firmware/src/motion_control.c \
            HIL_Firmware/src/vehicle_model.c \
            HIL_Firmware/src/track.c \
            HIL_Firmware/src/path_planning.c \
            HIL_Firmware/src/lqr_steer.c \
            ECU_Firmware/src/torque_vectoring.c

PERF      = $(HIL_BUILD)/perf_sim$(EXE_EXT)
PERF_SRCS = tools/tool_perf_sim.c \
            HIL_Firmware/src/motion_control.c \
            HIL_Firmware/src/vehicle_model.c \
            HIL_Firmware/src/track.c \
            HIL_Firmware/src/path_planning.c \
            HIL_Firmware/src/lqr_steer.c \
            ECU_Firmware/src/torque_vectoring.c

.PHONY: all run eval test perf clean

all: $(HIL_BUILD) $(ECU_BUILD) $(HIL_SIM) $(ECU_OBJ)

$(HIL_BUILD) $(ECU_BUILD):
	mkdir -p $@

# HIL sim objects
$(HIL_BUILD)/%.o: HIL_Firmware/src/%.c | $(HIL_BUILD)
	$(CC) $(HIL_FLAGS) -c -o $@ $<

$(HIL_BUILD)/torque_vectoring.o: ECU_Firmware/src/torque_vectoring.c | $(HIL_BUILD)
	$(CC) $(HIL_FLAGS) -c -o $@ $<

$(HIL_SIM): $(HIL_OBJS)
	$(CC) $(HIL_FLAGS) -o $@ $^ -lm

# ECU boundary check — compile torque_vectoring.c with only shared/ and its own
# headers to verify the ECU has no accidental dependency on HIL_Firmware/.
$(ECU_OBJ): ECU_Firmware/src/torque_vectoring.c | $(ECU_BUILD)
	$(CC) $(ECU_FLAGS) -c -o $@ $<

test: $(HIL_BUILD)
	$(CC) $(HIL_FLAGS) -o $(TEST_TV) $(TV_SRCS) -lm && $(TEST_TV)
	$(CC) $(HIL_FLAGS) -o $(TEST_VM) $(VM_SRCS) -lm && $(TEST_VM)
	$(CC) $(HIL_FLAGS) -o $(TEST_PP) $(PP_SRCS) -lm && $(TEST_PP)
	$(CC) $(HIL_FLAGS) -o $(TEST_MC) $(MC_SRCS) -lm && $(TEST_MC)
	$(CC) $(HIL_FLAGS) -o $(TEST_LQR) $(LQR_SRCS) -lm && $(TEST_LQR)

run: all
	$(HIL_SIM)

# Headless lap evaluation: runs the full motion-control -> ECU -> vehicle loop
# as fast as possible and prints lap-tracking metrics (mean/worst cross-track
# error, cone contacts, lap time). Use it to catch driver regressions.
eval: $(HIL_BUILD)
	$(CC) $(HIL_FLAGS) -o $(EVAL) $(EVAL_SRCS) -lm && $(EVAL)

# Compute-speed benchmark: runs the tick loop flat out for 1 wall-clock second
# and reports throughput. Pass a different budget as arg 1 (e.g. perf_sim 5).
perf: $(HIL_BUILD)
	$(CC) $(HIL_FLAGS) -o $(PERF) $(PERF_SRCS) -lm && $(PERF)

clean:
	rm -rf $(HIL_BUILD) $(ECU_BUILD)
