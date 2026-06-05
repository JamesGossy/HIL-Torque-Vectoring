.PHONY: all run clean

all:
	$(MAKE) -C HIL_Firmware

run:
	$(MAKE) -C HIL_Firmware run

clean:
	$(MAKE) -C HIL_Firmware clean
