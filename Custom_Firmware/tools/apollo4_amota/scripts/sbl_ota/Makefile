#*******************************************************************************
#
# Simple Makefile to prepare binaries for SBL OTA for Apollo4.
#
#*******************************************************************************

CURRENT_VER = sbl_ap4l_v0p3
ENCRPTED_SBL0 = ../../../$(APOLLO4_SCRIPT)/sbl_updates/$(CURRENT_VER)/encrypted_sbl0.bin
SBL_OTA = ../../../$(APOLLO4_SCRIPT)/sbl_updates/$(CURRENT_VER)/sbl_ota.bin

all: $(ENCRPTED_SBL0) $(SBL_OTA)
	# Apollo4 Cordio
	cp $(ENCRPTED_SBL0) encrypted_sbl0.bin
	cp $(SBL_OTA) sbl_ota.bin
	cat encrypted_sbl0.bin sbl_ota.bin > temp_sbl_apollo4.bin
	python3 ota_binary_converter.py --appbin temp_sbl_apollo4.bin -o update_sbl_apollo4
	@rm -rf temp_sbl_apollo4.bin encrypted_sbl0.bin sbl_ota.bin

clean:
	rm -rf *.bin
	