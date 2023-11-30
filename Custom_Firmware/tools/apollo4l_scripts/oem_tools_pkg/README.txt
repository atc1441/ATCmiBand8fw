The provisioning tools are designed to run on a Linux host machine with the following requirements: 
- Linux – Kernel Version 4.15.0 – 107-generic (Ubuntu 16.04.2). 
- OpenSSL – 1.0.2g 
- Python – 3.5.2 

Please refer to "Apollo4 and Apollo4 Blue OEM Provisioning Process and Tools" User's Guide on how to use these tools.
This README is just to briefly describe the sample configs bundled with the SDK.

OEM Provisioning: 
=================
The OPT (OEM Provisioning Tool) is a Ambiq signed tool which is downloaded and executed on the chip during device production in the OEM manufacturing facility.
The tool is downloaded in SRAM at address 0x10030000, and the OEM encrypted assets (generated using provided tools), at the SRAM address 0x10037000.
After downloading these 2 blobs, the device needs to be reseti (POR). After a successful OEM provisioning, device is transitioned to secure LCS.

The OPT tool is available at the following location:
./oem_prov_tool/opt_image_pkg.bin

OEM Provisioning Data is generated using the provided tools.

AmbiqSuite SDK includes couple of sample provisioning configurations, and pregenerated provisioning blobs corresponding to the same as a reference.

- oem_asset_prov_utils/oem_asset_package/dmpu_prov_data_blob_nonsec.bin:
    - Corresponds to the OTP configuration as defined in: 
        oem_asset_prov_utils/oem_asset_package/am_config/am_dmpu_data_gen_nonsec.cfg
        oem_asset_prov_utils/oem_asset_package/am_config/oem_asset_gen_nonsec.cfg
    - Noteworthy configurations:
        - SecureBoot Mode off
        - Boot Override set for Pin 0 - Active Low
        - Wired Update enabled for UART 0
        - OTP DCU lock mask set to all 0's (None of the DCU's will be locked)
        - No Copy/Write Protections
- oem_asset_prov_utils/oem_asset_package/dmpu_prov_data_blob_sec.bin
    - Corresponds to the OTP configuration as defined in: 
        oem_asset_prov_utils/oem_asset_package/am_config/am_dmpu_data_gen_sec.cfg
        oem_asset_prov_utils/oem_asset_package/am_config/oem_asset_gen_sec.cfg
    - Noteworthy configurations:
        - SecureBoot Mode ON (Will only boot with valid cert chain)
        - Boot Override set for Pin 0 - Active Low
        - Wired Update enabled for UART 0
        - Security Policy set to enforce Authentication and Encryption for OTA/Wired updates
        - OTP DCU lock mask set to all 0's (None of the DCU's will be locked)
        - No Copy/Write Protections

Both the sample configurations use dummy keys as generated using provided tools:
- am_oem_key_gen_util/oemRSAKeys (For Asymm keys)
- am_oem_key_gen_util/oemAESKeys (For KCE and KCP)
- oem_asset_prov_utils/oem_asset_package/inputData/keyBank*.bin (For OTP keybank)

The keybank area is configured to be accessible by Bootloader, as well as enabled for runtime access using the master keys, as programmed in
OTP_CUSTOTP_READ_KEY* & OTP_CUSTOTP_PROG_KEY* in respective cfg files (oem_asset_gen*.cfg)

Secure Debug Certificates:
==========================
Secure Debug Certificates are generated in accordance with the procedure outlined in the document.

AmbiqSuite SDK includes sample configurations as a reference.
- Enable All Debug Configuration: Enables all the OEM controlled Debug settings.
        - Valid only for Secure LCS
        - This assumes cert_utils/cert_gen_utils/inputData/soc_id1.bin has be initialized with right values as corresponding to the chip
        - Configs:
                - cert_utils/cert_gen_utils/am_config/am_oem_dbg_enabler_cert_all.cfg
                - cert_utils/cert_gen_utils/am_config/am_oem_dbg_developer_cert_all.cfg
- RMA: Initiates OEM part of the RMA process
        - Valid only for Secure LCS
        - This assumes cert_utils/cert_gen_utils/inputData/soc_id1.bin has be initialized with right values as corresponding to the chip
        - Configs:
                - cert_utils/cert_gen_utils/am_config/am_oem_dbg_enabler_cert_rma.cfg
                - cert_utils/cert_gen_utils/am_config/am_oem_dbg_developer_cert_rma.cfg
