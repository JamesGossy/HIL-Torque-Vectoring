export TMPDIR := /tmp

CC     = gcc
CFLAGS = -Wall -Wextra -std=c11 -O2 -D_POSIX_C_SOURCE=199309L

HIL_BUILD = HIL_Firmware/build
ECU_BUILD = ECU_Firmware/build
HIL_SIM   = $(HIL_BUILD)/hil_sim
ECU_OBJ   = $(ECU_BUILD)/torque_vectoring.o
TEST_BIN  = $(HIL_BUILD)/test_tv

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

TEST_SRCS = tests/test_tv.c \
            ECU_Firmware/src/torque_vectoring.c

.PHONY: all run test clean

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
	$(CC) $(HIL_FLAGS) -o $(TEST_BIN) $(TEST_SRCS) -lm
	$(TEST_BIN)

run: all
	$(HIL_SIM)

clean:
	rm -rf $(HIL_BUILD) $(ECU_BUILD)
