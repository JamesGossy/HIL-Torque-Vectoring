.PHONY: all run test clean

all:
	$(MAKE) -C HIL_Firmware

run:
	$(MAKE) -C HIL_Firmware run

test:
	$(MAKE) -C HIL_Firmware test

clean:
	$(MAKE) -C HIL_Firmware clean
