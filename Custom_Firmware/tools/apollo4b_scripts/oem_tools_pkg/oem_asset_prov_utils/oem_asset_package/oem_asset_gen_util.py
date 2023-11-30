import sys
import os


# Definitions for paths
if sys.platform != "win32":
    path_div = "//"
else:  # platform = win32
    path_div = "\\"

KEY_BANK_DATA_SIZE = 256
OEM_ASSET_SIZE = 2048

CURRENT_PATH = sys.path[0]
CURRENT_PATH_SCRIPTS = path_div + "common"
# this is the scripts local path, from where the program was called
sys.path.append(CURRENT_PATH + CURRENT_PATH_SCRIPTS)

OUTPUT_DIR_NAME = CURRENT_PATH + "/inputData/"

import configparser
from dmpu_util_helper import *
import sys


##################################################
# memcpy - Copy data fron one buffer to other.
# The Source and destination should be of bytes object
# Parameters are -  Destination,
#                   Source,
#                   destination offset in bytes
#                   size
###################################################
def copyBytes(dest, src, offset, size):
    for i in range(size):
        dest[i + offset] = src[i]


##################################################
# Parse given test configuration file and return
# test attributes as dictionary
#################################################
def parse_config_file(config, log_file):
    local_dict = {}
    section_name = "OEM_ASSET_GEN_CFG"
    if not config.has_section(section_name):
        log_sync(log_file, "section " + section_name + " wasn't found in cfg file\n")
        return None

    # Check for OTP_SBL_WPROT0-3: Flash write-protection bits
    if config.has_option(section_name, 'OTP_SBL_WPROT0'):
        otp_sbl_wprot0_str = config.get(section_name, 'OTP_SBL_WPROT0')
        local_dict['OTP_SBL_WPROT0'] = int(otp_sbl_wprot0_str, 16)
    else:
        print_and_log(log_file, "OTP_SBL_WPROT0: Flash Write-Protection Word 0 Not Provided \n")
        return None, None

    if config.has_option(section_name, 'OTP_SBL_WPROT1'):
        otp_sbl_wprot1_str = config.get(section_name, 'OTP_SBL_WPROT1')
        local_dict['OTP_SBL_WPROT1'] = int(otp_sbl_wprot1_str, 16)
    else:
        print_and_log(log_file, "OTP_SBL_WPROT1: Flash Write-Protection Word 1 Not Provided \n")
        return None, None

    if config.has_option(section_name, 'OTP_SBL_WPROT2'):
        otp_sbl_wprot2_str = config.get(section_name, 'OTP_SBL_WPROT2')
        local_dict['OTP_SBL_WPROT2'] = int(otp_sbl_wprot2_str, 16)
    else:
        print_and_log(log_file, "OTP_SBL_WPROT2: Flash Write-Protection Word 2 Not Provided \n")
        return None, None

    if config.has_option(section_name, 'OTP_SBL_WPROT3'):
        otp_sbl_wprot3_str = config.get(section_name, 'OTP_SBL_WPROT3')
        local_dict['OTP_SBL_WPROT3'] = int(otp_sbl_wprot3_str, 16)
    else:
        print_and_log(log_file, "OTP_SBL_WPROT3: Flash Write-Protection Word 3 Not Provided \n")
        return None, None

    # OTP_SBL_RPROT0-3: Flash copy/read-protection bits
    if config.has_option(section_name, 'OTP_SBL_RPROT0'):
        otp_sbl_rprot0_str = config.get(section_name, 'OTP_SBL_RPROT0')
        local_dict['OTP_SBL_RPROT0'] = int(otp_sbl_rprot0_str, 16)
    else:
        print_and_log(log_file, "OTP_SBL_RPROT0: Flash Copy/Read-Protection Word 0 Not Provided \n")
        return None, None

    if config.has_option(section_name, 'OTP_SBL_RPROT1'):
        otp_sbl_rprot1_str = config.get(section_name, 'OTP_SBL_RPROT1')
        local_dict['OTP_SBL_RPROT1'] = int(otp_sbl_rprot1_str, 16)
    else:
        print_and_log(log_file, "OTP_SBL_RPROT1: Flash Copy/Read-Protection Word 1 Not Provided \n")
        return None, None

    if config.has_option(section_name, 'OTP_SBL_RPROT2'):
        otp_sbl_rprot2_str = config.get(section_name, 'OTP_SBL_RPROT2')
        local_dict['OTP_SBL_RPROT2'] = int(otp_sbl_rprot2_str, 16)
    else:
        print_and_log(log_file, "OTP_SBL_RPROT2: Flash Copy/Read-Protection Word 2 Not Provided \n")
        return None, None

    if config.has_option(section_name, 'OTP_SBL_RPROT3'):
        otp_sbl_rprot3_str = config.get(section_name, 'OTP_SBL_RPROT3')
        local_dict['OTP_SBL_RPROT3'] = int(otp_sbl_rprot3_str, 16)
    else:
        print_and_log(log_file, "OTP_SBL_RPROT3: Flash Copy/Read-protection Word 3 Not Provided \n")
        return None, None

    # OTP_CUST_WPROT0-3: Flash write-protection bits
    if config.has_option(section_name, 'OTP_CUST_WPROT0'):
        otp_cust_wprot0_str = config.get(section_name, 'OTP_CUST_WPROT0')
        local_dict['OTP_CUST_WPROT0'] = int(otp_cust_wprot0_str, 16)
    else:
        print_and_log(log_file, "OTP_CUST_WPROT0: Flash Write-Protection Word 0 Not Provided \n")
        return None, None

    if config.has_option(section_name, 'OTP_CUST_WPROT1'):
        otp_cust_wprot1_str = config.get(section_name, 'OTP_CUST_WPROT1')
        local_dict['OTP_CUST_WPROT1'] = int(otp_cust_wprot1_str, 16)
    else:
        print_and_log(log_file, "OTP_CUST_WPROT1: Flash Write-Protection Word 1 Not Provided \n")
        return None, None

    if config.has_option(section_name, 'OTP_CUST_WPROT2'):
        otp_cust_wprot2_str = config.get(section_name, 'OTP_CUST_WPROT2')
        local_dict['OTP_CUST_WPROT2'] = int(otp_cust_wprot2_str, 16)
    else:
        print_and_log(log_file, "OTP_CUST_WPROT2: Flash Write-Protection Word 2 Not Provided \n")
        return None, None

    if config.has_option(section_name, 'OTP_CUST_WPROT3'):
        otp_cust_wprot3_str = config.get(section_name, 'OTP_CUST_WPROT3')
        local_dict['OTP_CUST_WPROT3'] = int(otp_cust_wprot3_str, 16)
    else:
        print_and_log(log_file, "OTP_CUST_WPROT3: Flash Write-Protection Word 3 Not Provided \n")
        return None, None

    # OTP_CUST_RPROT0-3: Flash copy/read-protection bits
    if config.has_option(section_name, 'OTP_CUST_RPROT0'):
        otp_cust_rprot0_str = config.get(section_name, 'OTP_CUST_RPROT0')
        local_dict['OTP_CUST_RPROT0'] = int(otp_cust_rprot0_str, 16)
    else:
        print_and_log(log_file, "OTP_CUST_RPROT0: Flash Copy/Read-Protection Word 0 Not Provided \n")
        return None, None

    if config.has_option(section_name, 'OTP_CUST_RPROT1'):
        otp_cust_rprot1_str = config.get(section_name, 'OTP_CUST_RPROT1')
        local_dict['OTP_CUST_RPROT1'] = int(otp_cust_rprot1_str, 16)
    else:
        print_and_log(log_file, "OTP_CUST_RPROT1: Flash Copy/Read-Protection Word 1 Not Provided \n")
        return None, None

    if config.has_option(section_name, 'OTP_CUST_RPROT2'):
        otp_cust_rprot2_str = config.get(section_name, 'OTP_CUST_RPROT2')
        local_dict['OTP_CUST_RPROT2'] = int(otp_cust_rprot2_str, 16)
    else:
        print_and_log(log_file, "OTP_CUST_RPROT2: Flash Copy/Read-Protection Word 2 Not Provided \n")
        return None, None

    if config.has_option(section_name, 'OTP_CUST_RPROT3'):
        otp_cust_rprot3_str = config.get(section_name, 'OTP_CUST_RPROT3')
        local_dict['OTP_CUST_RPROT3'] = int(otp_cust_rprot3_str, 16)
    else:
        print_and_log(log_file, "OTP_CUST_RPROT3: Flash Copy/Read-Protection Word 3 Not Provided \n")
        return None, None

    # OTP_CUST_KREVTRACK0-1: Customer Key Revocation Tracker Word0
    otp_cust_krevtrack0_str = '0x00000000'
    local_dict['OTP_CUST_KREVTRACK0'] = int(otp_cust_krevtrack0_str, 16)

    otp_cust_krevtrack1_str = '0x00000000'
    local_dict['OTP_CUST_KREVTRACK1'] = int(otp_cust_krevtrack1_str, 16)

    # OTP_DCU_DISABLEOVERRIDE
    if config.has_option(section_name, 'OTP_DCU_DISABLEOVERRIDE'):
        am_otp_dcu_disableoverride = config.get(section_name, 'OTP_DCU_DISABLEOVERRIDE')
        local_dict['OTP_DCU_DISABLEOVERRIDE'] = int(am_otp_dcu_disableoverride, 16)
    else:
        print_and_log(log_file, "OTP_DCU_DISABLEOVERRIDE: OTP DCU override setting not provided \n")
        return None, None

    # OTP_SEC_POL
    # OTP_SEC_POL.AUTH_ENF_ECC
    if config.has_option(section_name, 'OTP_SEC_POL.AUTH_ENF_ECC'):
        opt_sec_pol_auth_enf_ecc = int(config.get(section_name, 'OTP_SEC_POL.AUTH_ENF_ECC'),16)
        opt_sec_pol_auth_enf_ecc = opt_sec_pol_auth_enf_ecc << 29
    else:
        print_and_log(log_file, "OTP_SEC_POL.AUTH_ENF_ECC: Security Policy Not Provided \n")
        return None, None

    # OTP__SEC_POL.ENC_ENFORCE
    if config.has_option(section_name, 'OTP_SEC_POL.ENC_ENFORCE'):
        opt_sec_pol_enc_force = int(config.get(section_name, 'OTP_SEC_POL.ENC_ENFORCE'),16)
        opt_sec_pol_enc_force = opt_sec_pol_enc_force << 26
    else:
        print_and_log(log_file, "OTP_SEC_POL.ENC_ENFORCE: Security Policy Not Provided \n")
        return None, None

    # OTP_SEC_POL.AUTH_ENFORCE
    if config.has_option(section_name, 'OTP_SEC_POL.AUTH_ENFORCE'):
        opt_sec_pol_auth_enforce = int(config.get(section_name, 'OTP_SEC_POL.ENC_ENFORCE'),16)
        opt_sec_pol_auth_enforce = opt_sec_pol_auth_enforce << 23
    else:
        print_and_log(log_file, "OTP_SEC_POL.AUTH_ENFORCE: Security Policy Not Provided \n")
        return None, None

    # OTP_SEC_POL.WRAP_MODE
    if config.has_option(section_name, 'OTP_SEC_POL.WRAP_MODE'):
        opt_sec_pol_wrap_mode = int(config.get(section_name, 'OTP_SEC_POL.WRAP_MODE'),16)
        opt_sec_pol_wrap_mode = opt_sec_pol_wrap_mode << 19
    else:
        print_and_log(log_file, "OTP_SEC_POL.WRAP_MODE: Security Policy Not Provided \n")
        return None, None

    # OTP_SEC_POL.SBL_LOG_EN
    if config.has_option(section_name, 'OTP_SEC_POL.SBL_LOG_EN'):
        opt_sec_pol_sbl_log_en = int(config.get(section_name, 'OTP_SEC_POL.SBL_LOG_EN'),16)
        opt_sec_pol_sbl_log_en = opt_sec_pol_sbl_log_en << 18
    else:
        print_and_log(log_file, "OTP_SEC_POL.SBL_LOG_EN: Security Policy Not Provided \n")
        return None, None


    opt_sec_pol = opt_sec_pol_auth_enf_ecc | opt_sec_pol_enc_force | \
                  opt_sec_pol_auth_enforce | opt_sec_pol_wrap_mode | opt_sec_pol_sbl_log_en
    local_dict['OTP_SEC_POL'] = opt_sec_pol

    # OTP_BOOT_OVERRIDE
    # OTP_BOOT_OVERRIDE.ENABLE
    opt_boot_override = 0
    if config.has_option(section_name, 'OTP_BOOT_OVERRIDE.ENABLE'):
        opt_boot_override_enable = int(config.get(section_name, 'OTP_BOOT_OVERRIDE.ENABLE'), 16)
        opt_boot_override_enable = opt_boot_override_enable << 8
    else:
        print_and_log(log_file, "OTP_BOOT_OVERRIDE.ENABLE: Security Policy Not Provided \n")
        return None, None

    # OTP_BOOT_OVERRIDE.POL
    if config.has_option(section_name, 'OTP_BOOT_OVERRIDE.POL'):
        opt_boot_override_pol = int(config.get(section_name, 'OTP_BOOT_OVERRIDE.POL'), 16)
        opt_boot_override_pol = opt_boot_override_pol << 7
    else:
        print_and_log(log_file, "OTP_BOOT_OVERRIDE.POL: Security Policy Not Provided \n")
        return None, None

    # OTP_BOOT_OVERRIDE.GPIO
    if config.has_option(section_name, 'OTP_BOOT_OVERRIDE.GPIO'):
        opt_boot_override_gpio = int(config.get(section_name, 'OTP_BOOT_OVERRIDE.GPIO'), 16)
        opt_boot_override_gpio  = opt_boot_override_gpio
    else:
        print_and_log(log_file, "OTP_BOOT_OVERRIDE.GPIO: Security Policy Not Provided \n")
        return None, None

    opt_boot_override = opt_boot_override_enable | opt_boot_override_pol | opt_boot_override_gpio
    local_dict['OTP_BOOT_OVERRIDE'] = opt_boot_override

    # OTP_WIRED_CONFIG
    # OTP_WIRED_CONFIG.UART
    if config.has_option(section_name, 'OTP_WIRED_CONFIG.UART'):
        opt_wired_config_uart = int(config.get(section_name, 'OTP_WIRED_CONFIG.UART'), 16)
        opt_wired_config_uart = opt_wired_config_uart
    else:
        print_and_log(log_file, "OTP_WIRED_CONFIG.UART: Security Policy Not Provided \n")
        return None, None

    # OTP_WIRED_CONFIG.SPI
    if config.has_option(section_name, 'OTP_WIRED_CONFIG.SPI'):
        opt_wired_config_spi = int(config.get(section_name, 'OTP_WIRED_CONFIG.SPI'), 16)
        opt_wired_config_spi = opt_wired_config_spi << 1
    else:
        print_and_log(log_file, "OTP_WIRED_CONFIG.SPI: Security Policy Not Provided \n")
        return None, None

    # OTP_WIRED_CONFIG.I2C
    if config.has_option(section_name, 'OTP_WIRED_CONFIG.I2C'):
        opt_wired_config_i2c = int(config.get(section_name, 'OTP_WIRED_CONFIG.I2C'), 16)
        opt_wired_config_i2c = opt_wired_config_i2c << 2
    else:
        print_and_log(log_file, "OTP_WIRED_CONFIG.I2C: Security Policy Not Provided \n")
        return None, None

    # OTP_WIRED_CONFIG.SLAVEINTPIN
    if config.has_option(section_name, 'OTP_WIRED_CONFIG.SLAVEINTPIN'):
        opt_wired_config_slaveintpin = int(config.get(section_name, 'OTP_WIRED_CONFIG.SLAVEINTPIN'), 16)
        opt_wired_config_slaveintpin = opt_wired_config_slaveintpin << 3
    else:
        print_and_log(log_file, "OTP_WIRED_CONFIG.SLAVEINTPIN: Security Policy Not Provided \n")
        return None, None

    # OTP_WIRED_CONFIG.I2CADDR
    if config.has_option(section_name, 'OTP_WIRED_CONFIG.I2CADDR'):
        opt_wired_config_i2caddr = int(config.get(section_name, 'OTP_WIRED_CONFIG.I2CADDR'), 16)
        opt_wired_config_i2caddr = opt_wired_config_i2caddr << 9
    else:
        print_and_log(log_file, "OTP_WIRED_CONFIG.SLAVEINTPIN: Security Policy Not Provided \n")
        return None, None

    # OTP_WIRED_CONFIG.UARTMODULE
    if config.has_option(section_name, 'OTP_WIRED_CONFIG.UARTMODULE'):
        opt_wired_config_uartmodule = int(config.get(section_name, 'OTP_WIRED_CONFIG.UARTMODULE'), 16)
        opt_wired_config_uartmodule  = opt_wired_config_uartmodule << 16
    else:
        print_and_log(log_file, "OTP_WIRED_CONFIG.UARTMODULE: Security Policy Not Provided \n")
        return None, None

    opt_wired_config = opt_wired_config_uart | opt_wired_config_spi | opt_wired_config_i2c | \
                       opt_wired_config_slaveintpin | opt_wired_config_i2caddr | opt_wired_config_uartmodule
    local_dict['OTP_WIRED_CONFIG'] = opt_wired_config

    # OTP_CUSTOTP_READ_KEY0-3: 128-bit customer keybank read key Word0-3
    if config.has_option(section_name, 'OTP_CUSTOTP_READ_KEY0'):
        opt_custotp_read_key0_str = config.get(section_name, 'OTP_CUSTOTP_READ_KEY0')
        local_dict['OTP_CUSTOTP_READ_KEY0'] = int(opt_custotp_read_key0_str, 16)
    else:
        print_and_log(log_file, "OTP_CUSTOTP_READ_KEY0: 128-bit customer keybank read key Word0 Not Provided \n")
        return None, None

    if config.has_option(section_name, 'OTP_CUSTOTP_READ_KEY1'):
        opt_custotp_read_key1_str = config.get(section_name, 'OTP_CUSTOTP_READ_KEY1')
        local_dict['OTP_CUSTOTP_READ_KEY1'] = int(opt_custotp_read_key1_str, 16)
    else:
        print_and_log(log_file, "OTP_CUSTOTP_READ_KEY1: 128-bit customer keybank read key Word1 Not Provided \n")
        return None, None

    if config.has_option(section_name, 'OTP_CUSTOTP_READ_KEY2'):
        opt_custotp_read_key2_str = config.get(section_name, 'OTP_CUSTOTP_READ_KEY2')
        local_dict['OTP_CUSTOTP_READ_KEY2'] = int(opt_custotp_read_key2_str, 16)
    else:
        print_and_log(log_file, "OTP_CUSTOTP_READ_KEY2: 128-bit customer keybank read key Word2 Not Provided \n")
        return None, None

    if config.has_option(section_name, 'OTP_CUSTOTP_READ_KEY3'):
        opt_custotp_read_key3_str = config.get(section_name, 'OTP_CUSTOTP_READ_KEY3')
        local_dict['OTP_CUSTOTP_READ_KEY3'] = int(opt_custotp_read_key3_str, 16)
    else:
        print_and_log(log_file, "OTP_CUSTOTP_READ_KEY3: 128-bit customer keybank read key Word3 Not Provided \n")
        return None, None

    # OTP_CUSTOTP_PROG_KEY0-3 - 128-bit customer keybank PROG key Word0-3
    if config.has_option(section_name, 'OTP_CUSTOTP_PROG_KEY0'):
        opt_custotp_prog_key0_str = config.get(section_name, 'OTP_CUSTOTP_PROG_KEY0')
        local_dict['OTP_CUSTOTP_PROG_KEY0'] = int(opt_custotp_prog_key0_str, 16)
    else:
        print_and_log(log_file, "OTP_CUSTOTP_PROG_KEY0: 128-bit Customer Keybank PROG Key Word0 Not Provided \n")
        return None, None

    if config.has_option(section_name, 'OTP_CUSTOTP_PROG_KEY1'):
        opt_custotp_prog_key1_str = config.get(section_name, 'OTP_CUSTOTP_PROG_KEY1')
        local_dict['OTP_CUSTOTP_PROG_KEY1'] = int(opt_custotp_prog_key1_str, 16)
    else:
        print_and_log(log_file, "OTP_CUSTOTP_PROG_KEY1: 128-bit Customer Keybank PROG Key Word1 Not Provided \n")
        return None, None

    if config.has_option(section_name, 'OTP_CUSTOTP_PROG_KEY2'):
        opt_custotp_prog_key2_str = config.get(section_name, 'OTP_CUSTOTP_PROG_KEY2')
        local_dict['OTP_CUSTOTP_PROG_KEY2'] = int(opt_custotp_prog_key2_str, 16)
    else:
        print_and_log(log_file, "OTP_CUSTOTP_PROG_KEY2: 128-bit Customer Keybank PROG Key Word2 Not Provided \n")
        return None, None

    if config.has_option(section_name, 'OTP_CUSTOTP_PROG_KEY3'):
        opt_custotp_prog_key3_str = config.get(section_name, 'OTP_CUSTOTP_PROG_KEY3')
        local_dict['OTP_CUSTOTP_PROG_KEY3'] = int(opt_custotp_prog_key3_str, 16)
    else:
        print_and_log(log_file, "OTP_CUSTOTP_PROG_KEY3: 128-bit Customer Keybank PROG Key Word3 Not Provided \n")
        return None, None

    # OTP_CUSTOTP_PROGLOCK: Customer keybank program lock
    if config.has_option(section_name, 'OTP_CUSTOTP_PROGLOCK'):
        am_otp_prog_lock_str = config.get(section_name, 'OTP_CUSTOTP_PROGLOCK')
        local_dict['OTP_CUSTOTP_PROGLOCK'] = int(am_otp_prog_lock_str, 16)
    else:
        print_and_log(log_file, "OTP_CUSTOTP_PROGLOCK: Customer Keybank Program Lock Not Provided \n")
        return None, None

    # OTP_CUSTOTP_RDLOCK: Customer keybank read lock
    if config.has_option(section_name, 'OTP_CUSTOTP_RDLOCK'):
        am_otp_read_lock_str = config.get(section_name, 'OTP_CUSTOTP_RDLOCK')
        local_dict['OTP_CUSTOTP_RDLOCK'] = int(am_otp_read_lock_str, 16)
    else:
        print_and_log(log_file, "OTP_CUSTOTP_RDLOCK: Customer Keybank Read Lock Not Provided \n")
        return None, None

    # KeyBanks (0x1c00-0x1fcc)
    # Check for the Key Bank 0 keys bin file path
    if config.has_option(section_name, 'key-bank-0-File'):
        key_bank_0_str = config.get(section_name, 'key-bank-0-File')
        print(key_bank_0_str)

        # check if this is a valid path
        if os.path.isfile(key_bank_0_str):
            fileSize = os.path.getsize(key_bank_0_str)
            # The key size is 16 bytes (128 bit )
            if fileSize != KEY_BANK_DATA_SIZE:
                print_and_log(log_file, "option " + "Key Bank 0 - the key file has invalid size \n")
                return None, None
            local_dict['key_bank_0_File'] = key_bank_0_str
        else:
            print_and_log(log_file, "option " + key_bank_0_str + " file not found \n")
            return None, None
    else:
        print_and_log(log_file, "key-bank-0-File not provided \n")
        return None, None

    # Check for the Key Bank 1 keys bin file path
    if config.has_option(section_name, 'key-bank-1-File'):
        key_bank_1_str = config.get(section_name, 'key-bank-1-File')
        print(key_bank_1_str)
        # check if this is a valid path
        if os.path.isfile(key_bank_1_str):
            fileSize = os.path.getsize(key_bank_1_str)
            # The key size is 16 bytes (128 bit )
            if fileSize != KEY_BANK_DATA_SIZE:
                print_and_log(log_file, "option " + "Key Bank 1 - the key file has invalid size \n")
                return None, None
            local_dict['key_bank_1_File'] = key_bank_1_str
        else:
            print_and_log(log_file, "option " + key_bank_1_str + " file not found \n")
            return None, None
    else:
        print_and_log(log_file, "key-bank-1-File not provided \n")
        return None, None

        # Check for the Key Bank 2 keys bin file path
    if config.has_option(section_name, 'key-bank-2-File'):
        key_bank_2_str = config.get(section_name, 'key-bank-2-File')
        print(key_bank_2_str)
        # check if this is a valid path
        if os.path.isfile(key_bank_2_str):
            fileSize = os.path.getsize(key_bank_2_str)
            # The key size is 16 bytes (128 bit )
            if fileSize != KEY_BANK_DATA_SIZE:
                print_and_log(log_file, "option " + "Key Bank 2 - the key file has invalid size \n")
                return None, None
            local_dict['key_bank_2_File'] = key_bank_2_str
        else:
            print_and_log(log_file, "option " + key_bank_2_str + " file not found \n")
            return None, None
    else:
        print_and_log(log_file, "key-bank-2-File not provided \n")
        return None, None

    # Check for the Key Bank 3 keys bin file path
    if config.has_option(section_name, 'key-bank-3-File'):
        key_bank_3_str = config.get(section_name, 'key-bank-3-File')
        print(key_bank_3_str)
        # check if this is a valid path
        if os.path.isfile(key_bank_3_str):
            fileSize = os.path.getsize(key_bank_3_str)
            # The key size is 16 bytes (128 bit )
            if fileSize != KEY_BANK_DATA_SIZE:
                print_and_log(log_file, "option " + "Key Bank 3 - the key file has invalid size \n")
                return None, None
            local_dict['key_bank_3_File'] = key_bank_3_str
        else:
            print_and_log(log_file, "option " + key_bank_3_str + " file not found \n")
            return None, None
    else:
        print_and_log(log_file, "key-bank-3-File not provided \n")
        return None, None

    return local_dict


#########################################
# Parse script parameters
#########################################
def parse_shell_arguments():
    len_arg = len(sys.argv)
    if len_arg < 2:
        print_sync("len " + str(len_arg) + " invalid. Usage:" + sys.argv[0] + "<test configuration file>\n")
        for i in range(1, len_arg):
            print_sync("i " + str(i) + " arg " + sys.argv[i] + "\n")
        sys.exit(1)
    config_fname = sys.argv[1]
    if len_arg == 3:
        log_fname = sys.argv[2]
    else:
        log_fname = "oemAssetGen.log"
    return config_fname, log_fname


##################################
# close files and exit script
##################################
def exit_main_func(log_file, config_file, rc):
    log_file.close()
    config_file.close()
    sys.exit(rc)


###########################################
# Main
#########################################
def main():
    config_fname, log_fname = parse_shell_arguments()
    log_file = create_log_file(log_fname)
    print_and_log(log_file,
                  str(datetime.now()) + ": OEM Specific Asset Generator started (Logging to " + log_fname + ")\n")

    try:
        config_file = open(config_fname, 'r')
    except IOError as e:
        print_and_log(log_file, "Failed opening " + config_fname + " (" + e.strerror + ")\n")
        log_file.close()
        sys.exit(e.errno)

    config = configparser.ConfigParser()
    config.read(config_fname)
    data_dict = {}

    data_dict = parse_config_file(config, log_file)

    if (data_dict != None):

        oemAsset_bin = bytearray(OEM_ASSET_SIZE)
        fill_data = int('0x00000000', 16)

        # Get the key bank data
        keyBank0_size, keyBank0_data = GetDataFromBinFile(log_file, data_dict['key_bank_0_File'])
        keyBank1_size, keyBank1_data = GetDataFromBinFile(log_file, data_dict['key_bank_1_File'])
        keyBank2_size, keyBank2_data = GetDataFromBinFile(log_file, data_dict['key_bank_2_File'])
        keyBank3_size, keyBank3_data = GetDataFromBinFile(log_file, data_dict['key_bank_3_File'])

        # OTP_SBL_WPROT0-3: Flash write-protection bits (0x1800 - 0x180c)
        # 16 bytes
        copyBytes(oemAsset_bin, data_dict['OTP_SBL_WPROT0'].to_bytes(4, byteorder="little"), 0, 4)
        #print('OTP_SBL_WPROT0: %s' % hex(int('0x1800',16) + 0))

        copyBytes(oemAsset_bin, data_dict['OTP_SBL_WPROT1'].to_bytes(4, byteorder="little"), 4, 4)
        #print('OTP_SBL_WPROT1: %s' % hex(int('0x1800',16) + 4))

        copyBytes(oemAsset_bin, data_dict['OTP_SBL_WPROT2'].to_bytes(4, byteorder="little"), 8, 4)
        #print('OTP_SBL_WPROT2: %s' % hex(int('0x1800', 16) + 8))

        copyBytes(oemAsset_bin, data_dict['OTP_SBL_WPROT3'].to_bytes(4, byteorder="little"), 12, 4)
        #print('OTP_SBL_WPROT3: %s' % hex(int('0x1800', 16) + 12))

        # OTP_SBL_RPROT0-3: Flash copy/read-protection bits (0x1810-0x181c)
        # 16 bytes
        copyBytes(oemAsset_bin, data_dict['OTP_SBL_RPROT0'].to_bytes(4, byteorder="little"), 16, 4)
        #print('OTP_SBL_RPROT0: %s' % hex(int('0x1800', 16) + 16))

        copyBytes(oemAsset_bin, data_dict['OTP_SBL_RPROT1'].to_bytes(4, byteorder="little"), 20, 4)
        #print('OTP_SBL_RPROT1: %s' % hex(int('0x1800', 16) + 20))

        copyBytes(oemAsset_bin, data_dict['OTP_SBL_RPROT2'].to_bytes(4, byteorder="little"), 24, 4)
        #print('OTP_SBL_RPROT2: %s' % hex(int('0x1800', 16) + 24))

        copyBytes(oemAsset_bin, data_dict['OTP_SBL_RPROT3'].to_bytes(4, byteorder="little"), 28, 4)
        #print('OTP_SBL_RPROT3: %s' % hex(int('0x1800', 16) + 28))

        # OTP_CUST_WPROT0-3: Flash write-protection bits (0x1820-0x183c)
        # 16 bytes
        copyBytes(oemAsset_bin, data_dict['OTP_CUST_WPROT0'].to_bytes(4, byteorder="little"), 32, 4)
        #print('OTP_CUST_WPROT0: %s' % hex(int('0x1800', 16) + 32))

        copyBytes(oemAsset_bin, data_dict['OTP_CUST_WPROT1'].to_bytes(4, byteorder="little"), 36, 4)
        #print('OTP_CUST_WPROT1: %s' % hex(int('0x1800', 16) + 36))

        copyBytes(oemAsset_bin, data_dict['OTP_CUST_WPROT2'].to_bytes(4, byteorder="little"), 40, 4)
        #print('OOTP_CUST_WPROT2: %s' % hex(int('0x1800', 16) + 40))

        copyBytes(oemAsset_bin, data_dict['OTP_CUST_WPROT3'].to_bytes(4, byteorder="little"), 44, 4)
        #print('OOTP_CUST_WPROT3: %s' % hex(int('0x1800', 16) + 44))

        # OTP_CUST_RPROT0-3: Flash copy/read-protection bits (0x1830-0x183c)
        # 16 bytes
        copyBytes(oemAsset_bin, data_dict['OTP_CUST_RPROT0'].to_bytes(4, byteorder="little"), 48, 4)
        #print('OTP_CUST_RPROT0: %s' % hex(int('0x1800', 16) + 48))

        copyBytes(oemAsset_bin, data_dict['OTP_CUST_RPROT1'].to_bytes(4, byteorder="little"), 52, 4)
        #print('OTP_CUST_RPROT1: %s' % hex(int('0x1800', 16) + 52))

        copyBytes(oemAsset_bin, data_dict['OTP_CUST_RPROT2'].to_bytes(4, byteorder="little"), 56, 4)
        #print('OTP_CUST_RPROT2: %s' % hex(int('0x1800', 16) + 56))

        copyBytes(oemAsset_bin, data_dict['OTP_CUST_RPROT3'].to_bytes(4, byteorder="little"), 60, 4)
        #print('OTP_CUST_RPROT3: %s' % hex(int('0x1800', 16) + 60))

        # OTP_DCU_DISABLEOVERRIDE
        copyBytes(oemAsset_bin, data_dict['OTP_DCU_DISABLEOVERRIDE'].to_bytes(4, byteorder="little"), 64, 4)
        # print('OTP_DCU_DISABLEOVERRIDE: %s' % hex(int('0x1800', 16) + 64))

        # 20 bytes
        # 0x00001844: OTP_CUST_KREVTRACK0 - Customer Key Revocation Tracker Word0
        copyBytes(oemAsset_bin, fill_data.to_bytes(4, byteorder="little"), 68, 4)
        #print('Pad: %s' % hex(int('0x1800', 16) + 68))

        # 0x00001848: OTP_CUST_KREVTRACK1 - Customer Key Revocation Tracker
        copyBytes(oemAsset_bin, fill_data.to_bytes(4, byteorder="little"), 72, 4)
        #print('Pad: %s' % hex(int('0x1800', 16) + 72))

        # OTP_SEC_POL: Security policy (0x184c)
        copyBytes(oemAsset_bin, data_dict['OTP_SEC_POL'].to_bytes(4, byteorder="little"), 76, 4)
        #print('OTP_SEC_POL: %s' % hex(int('0x1800', 16) + 76))

        # OTP_BOOT_OVERRIDE: Boot override (0x1850)
        copyBytes(oemAsset_bin, data_dict['OTP_BOOT_OVERRIDE'].to_bytes(4, byteorder="little"), 80, 4)
        #print('OTP_BOOT_OVERRIDE: %s' % hex(int('0x1800', 16) + 80))

        # OTP_WIRED_CONFIG: Wired configuration (0x1854)
        copyBytes(oemAsset_bin, data_dict['OTP_WIRED_CONFIG'].to_bytes(4, byteorder="little"), 84, 4)
        #print('OTP_WIRED_CONFIG: %s' % hex(int('0x1800', 16) + 84))


        # 404 bytes
        # 0x00001858: OTP_RSVD616 - Reserved space: 9 words, word offsets 0x616-0x61E, byte offsets 0x1858 - 0x1878
        # 0x00001878: OTP_RSVD61E - Reserved space: 9 words, word offsets 0x616-0x61E, byte offsets 0x1858 - 0x1878
        # 0x00001880: OTP_RSVD620 - Reserved space: 92 words, word offsets 0x620-0x67B, byte offsets 0x1880 - 0x19EC.
        # 0x000019EC: OTP_RSVD67B - Reserved space: 92 words, word offsets 0x620-0x67B, byte offsets 0x1880 - 0x19EC.
        word_offset = 4
        for dword in range(102):
            copyBytes(oemAsset_bin, fill_data.to_bytes(4, byteorder="little"), 84 + word_offset, 4)
            #print('Reseverd: %s' % hex(int('0x1800', 16) + 84 + word_offset))
            word_offset = word_offset + 4


        # 0x000019F0: OTP_WRAP_KEY0-3 - Customer key word 0-3 (0x19f0 - 0x19fc)
        # 16 bytes
        word_offset = 4
        for dword in range(4):
            copyBytes(oemAsset_bin, fill_data.to_bytes(4, byteorder="little"), 492 + word_offset, 4)
            #print('Reseverd-OTP_WRAP_KEY0-3: %s' % hex(int('0x1800', 16) + 492 + word_offset))
            word_offset = word_offset + 4


        # 40 bytes
        # OTP_CUSTOTP_READ_KEY0-3: 128-bit customer keybank read key Word0-3 (0x1a00 - 0x1a0c)
        copyBytes(oemAsset_bin, data_dict['OTP_CUSTOTP_READ_KEY0'].to_bytes(4, byteorder="little"), 512, 4)
        #print('OTP_CUSTOTP_READ_KEY0: %s' % hex(int('0x1800', 16) + 512))

        copyBytes(oemAsset_bin, data_dict['OTP_CUSTOTP_READ_KEY1'].to_bytes(4, byteorder="little"), 516, 4)
        #print('OTP_CUSTOTP_READ_KEY1: %s' % hex(int('0x1800', 16) + 516))

        copyBytes(oemAsset_bin, data_dict['OTP_CUSTOTP_READ_KEY2'].to_bytes(4, byteorder="little"), 520, 4)
        #print('OTP_CUSTOTP_READ_KEY2: %s' % hex(int('0x1800', 16) + 520))

        copyBytes(oemAsset_bin, data_dict['OTP_CUSTOTP_READ_KEY3'].to_bytes(4, byteorder="little"), 524, 4)
        #print('OTP_CUSTOTP_READ_KEY3: %s' % hex(int('0x1800', 16) + 524))

        # OTP_CUSTOTP_PROG_KEY0-3: 128-bit customer keybank PROG key Word0-3 (0x1a10 - 0x1a1c)
        copyBytes(oemAsset_bin, data_dict['OTP_CUSTOTP_PROG_KEY0'].to_bytes(4, byteorder="little"), 528, 4)
        #print('OTP_CUSTOTP_PROG_KEY0: %s' % hex(int('0x1800', 16) + 528))

        copyBytes(oemAsset_bin, data_dict['OTP_CUSTOTP_PROG_KEY1'].to_bytes(4, byteorder="little"), 532, 4)
        #print('OTP_CUSTOTP_PROG_KEY1: %s' % hex(int('0x1800', 16) + 532))

        copyBytes(oemAsset_bin, data_dict['OTP_CUSTOTP_PROG_KEY2'].to_bytes(4, byteorder="little"), 536, 4)
        #print('OTP_CUSTOTP_PROG_KEY2: %s' % hex(int('0x1800', 16) + 536))

        copyBytes(oemAsset_bin, data_dict['OTP_CUSTOTP_PROG_KEY3'].to_bytes(4, byteorder="little"), 540, 4)
        #print('OTP_CUSTOTP_PROG_KEY3: %s' % hex(int('0x1800', 16) + 540))

        # OTP_CUSTOTP_PROGLOCK - Customer keybank program lock (0x1a20)
        copyBytes(oemAsset_bin, data_dict['OTP_CUSTOTP_PROGLOCK'].to_bytes(4, byteorder="little"), 544, 4)
        #print('OTP_CUSTOTP_PROGLOCK: %s' % hex(int('0x1800', 16) + 544))

        # OTP_CUSTOTP_RDLOCK - Customer keybank read lock (0x1a24)
        copyBytes(oemAsset_bin, data_dict['OTP_CUSTOTP_RDLOCK'].to_bytes(4, byteorder="little"), 548, 4)
        #print('OTP_CUSTOTP_RDLOCK: %s' % hex(int('0x1800', 16) + 548))


        # 472 bytes
        # 0x00001A28: OTP_RSVD68A - Reserved space: 137 words, word offsets 0x68A-0x6FF, byte offsets 0x1A28 - 0x1BFC.
        # 0x00001BFC: OTP_RSVD6FF - Reserved space: 137 words, word offsets 0x68A-0x6FF, byte offsets 0x1A28 - 0x1BFC.
        word_offset = 4
        for dword in range(118):
            copyBytes(oemAsset_bin, fill_data.to_bytes(4, byteorder="little"), 548 + word_offset, 4)
            #print('Reseverd: %s' % hex(int('0x1800', 16) + 548 + word_offset))
            word_offset = word_offset + 4

        # 1024 bytes
        copyBytes(oemAsset_bin, keyBank0_data, 1024, keyBank0_size)
        #print('keyBank0_data: %s' % hex(int('0x1800', 16) + 1024))

        copyBytes(oemAsset_bin, keyBank1_data, 1280, keyBank1_size)
        #print('keyBank1_data: %s' % hex(int('0x1800', 16) + 1280))

        copyBytes(oemAsset_bin, keyBank2_data, 1536, keyBank2_size)
        #print('keyBank2_data: %s' % hex(int('0x1800', 16) + 1536))

        copyBytes(oemAsset_bin, keyBank3_data, 1792, keyBank3_size)
        #print('keyBank3_data: %s' % hex(int('0x1800', 16) + 1792))

        file = open(OUTPUT_DIR_NAME + "oem_asset_test_data.bin", "wb")
        file.write(oemAsset_bin)
        file.close()

        print_and_log(log_file, "**** OEM Asset Generation completed successfully ****\n")
        exit_main_func(log_file, config_file, 0)

    else:
        print_and_log(log_file, "**** Invalid config file ****\n")
        exit_main_func(log_file, config_file, 1)

    FreeDLLGetHandle(DLLHandle)


#############################
if __name__ == "__main__":
    main()
