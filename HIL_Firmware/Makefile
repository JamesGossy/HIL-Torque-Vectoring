CC     = gcc
CFLAGS = -Wall -Wextra -std=c11 -O2 \
         -I include \
         -I ../shared \
         -I ../ECU_Firmware/include

BUILD  = build
TARGET = $(BUILD)/hil_sim

SRCS = src/main.c \
       src/vehicle_model.c \
       src/track.c \
       src/autopilot.c \
       src/visualiser.c \
       ../ECU_Firmware/src/torque_vectoring.c

OBJS = $(patsubst %.c, $(BUILD)/%.o, $(notdir $(SRCS)))

.PHONY: all run clean

all: $(BUILD) $(TARGET)

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/%.o: src/%.c | $(BUILD)
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD)/torque_vectoring.o: ../ECU_Firmware/src/torque_vectoring.c | $(BUILD)
	$(CC) $(CFLAGS) -c -o $@ $<

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ -lm

run: all
	./$(TARGET)

clean:
	rm -rf $(BUILD)
