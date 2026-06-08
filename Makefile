export TMPDIR := /tmp

CC     = gcc
CFLAGS = -Wall -Wextra -std=c11 -O2 -D_POSIX_C_SOURCE=199309L

HIL_BUILD = HIL_Firmware/build
ECU_BUILD = ECU_Firmware/build
HIL_SIM   = $(HIL_BUILD)/hil_sim
ECU_OBJ   = $(ECU_BUILD)/torque_vectoring.o
TEST_TV   = $(HIL_BUILD)/test_tv
TEST_VM   = $(HIL_BUILD)/test_vehicle_model
TEST_PP   = $(HIL_BUILD)/test_path_planning
TEST_MC   = $(HIL_BUILD)/test_motion_control

HIL_FLAGS = $(CFLAGS) \
            -I HIL_Firmware/include \
            -I shared \
            -I ECU_Firmware/include

ECU_FLAGS = $(CFLAGS) \
            -I ECU_Firmware/include \
            -I shared

HIL_SRCS = HIL_Firmware/src/main.c \
           HIL_Firmware/src/vehicle_model.c \
           HIL_Firmware/src/track.c \
           HIL_Firmware/src/path_planning.c \
           HIL_Firmware/src/motion_control.c \
           ECU_Firmware/src/torque_vectoring.c

HIL_OBJS = $(patsubst %.c, $(HIL_BUILD)/%.o, $(notdir $(HIL_SRCS)))

VM_SRCS = tests/test_vehicle_model.c \
          HIL_Firmware/src/vehicle_model.c

PP_SRCS = tests/test_path_planning.c \
          HIL_Firmware/src/path_planning.c

MC_SRCS = tests/test_motion_control.c \
          HIL_Firmware/src/motion_control.c \
          HIL_Firmware/src/vehicle_model.c \
          HIL_Firmware/src/path_planning.c

TV_SRCS = tests/test_tv.c \
          ECU_Firmware/src/torque_vectoring.c

EVAL     = $(HIL_BUILD)/eval_lap
EVAL_SRCS = tests/tool_eval_lap.c \
            HIL_Firmware/src/motion_control.c \
            HIL_Firmware/src/vehicle_model.c \
            HIL_Firmware/src/track.c \
            HIL_Firmware/src/path_planning.c \
            ECU_Firmware/src/torque_vectoring.c

PERF      = $(HIL_BUILD)/perf_sim
PERF_SRCS = tests/tool_perf_sim.c \
            HIL_Firmware/src/motion_control.c \
            HIL_Firmware/src/vehicle_model.c \
            HIL_Firmware/src/track.c \
            HIL_Firmware/src/path_planning.c \
            ECU_Firmware/src/torque_vectoring.c

.PHONY: all run test eval perf clean

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

run: all
	$(HIL_SIM)

# Headless lap evaluation: runs the full motion-control -> ECU -> vehicle loop
# as fast as possible (no real-time sleep) and prints lap-tracking metrics —
# mean/worst cross-track error, sharp-corner behaviour, cone contacts, lap time.
# Used to catch regressions in how well the virtual driver follows the racing
# line (especially the tight FSG hairpins).
eval: $(HIL_BUILD)
	$(CC) $(HIL_FLAGS) -o $(EVAL) $(EVAL_SRCS) -lm && $(EVAL)

# Same as `eval`, but also appends the machine-readable RESULT line (tagged with
# timestamp + git SHA + dirty flag) to tests/eval_history.csv so you can track
# how mean/worst CTE and lap time move across tuning iterations. The CSV is
# gitignored — it is your local log. Header is written once on first run.
eval-log: $(HIL_BUILD)
	$(CC) $(HIL_FLAGS) -o $(EVAL) $(EVAL_SRCS) -lm
	@$(EVAL) | python3 tests/tool_log_eval.py - tests/eval_history.csv

# Compute-speed benchmark: runs the full tick loop with no real-time sleep for
# 1 wall-clock second and reports throughput (ticks/s, real-time factor, laps
# per wall-second).  Pass a different budget as the first argument, e.g.
# `HIL_Firmware/build/perf_sim 5` for a 5-second run.
perf: $(HIL_BUILD)
	$(CC) $(HIL_FLAGS) -o $(PERF) $(PERF_SRCS) -lm && $(PERF)

clean:
	rm -rf $(HIL_BUILD) $(ECU_BUILD)
