CC     = gcc
CFLAGS = -Wall -Wextra -std=c11 -O2 \
         -I include \
         -I ../shared

BUILD  = build
TARGET = $(BUILD)/torque_vectoring.o

.PHONY: all clean

# Compile torque_vectoring.c in isolation (no HIL headers) to prove the ECU
# boundary is clean — only shared/ and its own headers are needed.
all: $(BUILD)
	$(CC) $(CFLAGS) -c -o $(TARGET) src/torque_vectoring.c

$(BUILD):
	mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)
