Minimum software requirements:
==============================
 - Python: Python3.5.2 
        - Ambiq recommends against using Python3.9


Brief info on various files in this directory:
==============================================
Scripts/configs:
================
- helper scripts: These are used by other scripts
        - am_defines.py, apollo4b_info0.py, apollo4b_keys.py, key_table.py
- create_info0.py => Should be used to generate INFO0
- create_cust_image_blob.py => Used to generate various OTA or wired images for Ambiq SBL
        - It requires keys.ini for keys information, and an operation specific ini file to generate the required image
- uart_wired_update.py => Used to interact with Ambiq SBL as part of Wired update
- sample: This folder contains various sample ini files. The required files should be copied to the parent directory and edited to be used with create_cust_image_blob.py
        - keys.ini => Determines location of key assets
        - wired.ini => Sample ini file to download a blob to a fixed location in the device
        - wired_ota.ini => Sample ini file to download a preconstructed OTA image to a temp place in device, and trigger OTA on reset
        - firmware.ini => Generate OTA image corresponding to a firmware update (Can be secure or non-secure)
        - oem_chain.ini => Generate OEM Cert chain update
- sbl_updates: This folder contains the SBL OTA images to do on field updates of Ambiq SBL
        - Depending on the SBL version, each OTA consists of two images: encrypted_sbl0.bin and sbl_ota.bin
        - These files should be copied to tools/apollo4b_scripts, to be used with supplied JLink scripts for SBL OTA
- oem_tools_package: This folder contains various key, asset and certificate generation utilities for provisioning and runtime usage

Jlink scripts:
==============
- jlink-prog-info0.txt => Sample script to program INFO0 (info0.bin generated using create_info0.py)
- jlink-update-firmware.txt => Sample script to initiate firmware OTA (OTA blob generated using create_cust_image_blob.py using firmware.ini)
- jlink-update-patch.txt => Sample script to apply Trim patches provided by Ambiq
- jlink-update-cchain.txt => Sample script to initiate OEM Cert Chain update (Blob generated using create_cust_image_blob.py using oem_chain.ini)
- jlink-prog-sbl*.txt => Sample script to initiate SBL OTA (See more details below)

SBL OTA Notes:
==============
Process of upgrading SBL depends on the location of current SBL.
By default SBL is installed at 0x8000

However, after the first upgrade SBL will be running from 0x10000
The location of SBL changes between 0x8000 and 0x10000 on each upgrade.

Upgrading SBL requires downloading two blobs - one with OTA metadat information (that is loaded in general MRAM area), and other is the signed/encrypted SBL image itself, which is directly loaded to the destination (i.e. to the designated next SBL location, be it 0x10000 or 0x8000)

So, if the current SBL is running at 0x8000, we need to download the update to 0x10000 and vice versa.

You can find out the location of current SBL by looking at the SBL SWO logs:

SecureBoot SBL_apollo4b_v1_test ver:0x1(0xa5b0) running with VTOR @ 0x8000
OR
SBL_ap4bv3.0 ver:0.3(0xa640) @ 0x8000

This indicates current SBL is running from 0x8000 (slot 0)
This in turn implies that the update needs to be loaded to slot 1

Programmatically, the location of the address where the update needs to be loaded can be determines by calling:
am_hal_security_get_info(). pSecInfo->sblStagingAddr identifies the address

Use the following script, depending on which slot you want to load the SBL to:

- To load the update to slot 1 (i.e. if current SBL is running at 0x8000) use jlink-prog-sbl1.txt
 => After successful upgrade, SBL should be running from 0x10000

- To load the update to slot 0 (i.e. if current SBL is running at 0x10000) use jlink-prog-sbl0.txt
 => After successful upgrade SBL should be running from 0x8000

These scripts use images encrypted_sbl0.bin and sbl_ota.bin, and expect them to be in same directory.
Please edit the scripts accordingly, if that’s not the case.
