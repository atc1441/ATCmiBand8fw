#*******************************************************************************
#
# Simple Makefile to prepare binaries for AMOTA for Apollo4.
#
#*******************************************************************************

TOOL_CHAIN?=gcc

APOLLO4_BOARD = apollo4l_blue_kbr_eb
APOLLO4_SCRIPT = apollo4l_scripts
APOLLO4_KEYS = apollo4l_keys.py
UPDATEBIN_APOLLO4_BLUE =../../../boards/$(APOLLO4_BOARD)/examples/ble/ble_freertos_amota/$(TOOL_CHAIN)/bin/ble_freertos_amota.bin
APPBIN_APOLLO4_BLUE =../../../boards/$(APOLLO4_BOARD)/examples/ble/ble_freertos_amota/$(TOOL_CHAIN)/bin/ble_freertos_amota.bin

all: $(APPBIN_APOLLO4_BLUE) $(UPDATEBIN_APOLLO4_BLUE) $(UPDATEBIN_APOLLO4_BLUE_ETHERMIND) $(APPBIN_APOLLO4_BLUE_ETHERMIND)
	# Apollo4 Cordio
	cp $(APPBIN_APOLLO4_BLUE) starter_binary_apollo4_blue.bin
	cp -r ../../$(APOLLO4_SCRIPT)/oem_tools_pkg* oem_tools_pkg
	cp -r ../../$(APOLLO4_SCRIPT)/arm_utils* arm_utils
	cp ../../$(APOLLO4_SCRIPT)/am_defines.py am_defines.py
	cp ../../$(APOLLO4_SCRIPT)/$(APOLLO4_KEYS) $(APOLLO4_KEYS)
	cp ../../$(APOLLO4_SCRIPT)/create_cust_image_blob.py create_cust_image_blob.py
	cp ../../$(APOLLO4_SCRIPT)/key_table.py key_table.py
	cp ../../$(APOLLO4_SCRIPT)/sample/keys.ini keys.ini
	python create_cust_image_blob.py -c firmware.ini
	python ota_binary_converter.py --appbin temp_binary_apollo4_blue.bin -o update_binary_apollo4_blue
	@rm -rf temp_binary_apollo4_blue.bin
	@rm -rf key_table.py
	@rm -rf keys.ini
	@rm -rf am_defines.py
	@rm -rf create_cust_image_blob.py
	@rm -rf oem_tools_pkg
	@rm -rf arm_utils
	@rm -rf $(APOLLO4_KEYS)

$(APPBIN_APOLLO4_BLUE):
	$(MAKE) -C ../../../boards/$(APOLLO4_BOARD)/examples/ble/ble_freertos_amota/$(TOOL_CHAIN)/ $(MAKECMDGOALS)

clean:
	rm -rf *.bin
	rm -rf $(APPBIN_APOLLO4_BLUE)
	