# *****************************************************************************
#
#    apollo4b_info0.py
#
#    Define INFO0 registers.
#
# *****************************************************************************

# *****************************************************************************
#
#    Copyright (c) 2023, Ambiq Micro, Inc.
#    All rights reserved.
#
#    Redistribution and use in source and binary forms, with or without
#    modification, are permitted provided that the following conditions are met:
#
#    1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
#    2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
#    3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from this
#    software without specific prior written permission.
#
#    Third party software included in this distribution is subject to the
#    additional license terms as defined in the /docs/licenses directory.
#
#    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#    POSSIBILITY OF SUCH DAMAGE.
#
#  This is part of revision release_sdk_4_4_1-7498c7b770 of the AmbiqSuite Development Package.
#
# *****************************************************************************

AM_REG_INFO0_BASEADDR = 0x42000000

AM_REG_INFO0_SIGNATURE0_O = 0x00000000
AM_REG_INFO0_SIGNATURE0_ADDR = 0x42000000
AM_REG_INFO0_SIGNATURE1_O = 0x00000004
AM_REG_INFO0_SIGNATURE1_ADDR = 0x42000004
AM_REG_INFO0_SIGNATURE2_O = 0x00000008
AM_REG_INFO0_SIGNATURE2_ADDR = 0x42000008
AM_REG_INFO0_SIGNATURE3_O = 0x0000000c
AM_REG_INFO0_SIGNATURE3_ADDR = 0x4200000c
AM_REG_INFO0_CUSTOMER_TRIM_O = 0x00000014
AM_REG_INFO0_CUSTOMER_TRIM_ADDR = 0x42000014
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG0_O = 0x00000028
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG0_ADDR = 0x42000028
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG1_O = 0x0000002c
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG1_ADDR = 0x4200002c
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG2_O = 0x00000030
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG2_ADDR = 0x42000030
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG3_O = 0x00000034
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG3_ADDR = 0x42000034
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG4_O = 0x00000038
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG4_ADDR = 0x42000038
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG5_O = 0x0000003c
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG5_ADDR = 0x4200003c
AM_REG_INFO0_SECURITY_VERSION_O = 0x00000040
AM_REG_INFO0_SECURITY_VERSION_ADDR = 0x42000040
AM_REG_INFO0_SECURITY_SRAM_RESV_O = 0x00000044
AM_REG_INFO0_SECURITY_SRAM_RESV_ADDR = 0x42000044
AM_REG_INFO0_SECURITY_RMAOVERRIDE_O = 0x00000048
AM_REG_INFO0_SECURITY_RMAOVERRIDE_ADDR = 0x42000048
AM_REG_INFO0_MAINPTR_ADDR = 0x42000050
AM_REG_INFO0_WIRED_TIMEOUT_O = 0x00000054
AM_REG_INFO0_WIRED_TIMEOUT_ADDR = 0x42000054
AM_REG_INFO0_SBR_SDCERT_ADDR_O = 0x00000058
AM_REG_INFO0_SBR_SDCERT_ADDR_ADDR = 0x42000058
AM_REG_INFO0_MAINPTR_O = 0x00000060
AM_REG_INFO0_MAINPTR_ADDR = 0x42000060
AM_REG_INFO0_CERTCHAINPTR_O = 0x00000064
AM_REG_INFO0_CERTCHAIN_ADDR = 0x42000064



AM_REG_INFO0_SIGNATURE0_SIG0_S = 0
AM_REG_INFO0_SIGNATURE0_SIG0_M = 0xFFFFFFFF

AM_REG_INFO0_SIGNATURE1_SIG1_S = 0
AM_REG_INFO0_SIGNATURE1_SIG1_M = 0xFFFFFFFF


AM_REG_INFO0_SIGNATURE2_SIG2_S = 0
AM_REG_INFO0_SIGNATURE2_SIG2_M = 0xFFFFFFFF


AM_REG_INFO0_SIGNATURE3_SIG3_S = 0
AM_REG_INFO0_SIGNATURE3_SIG3_M = 0xFFFFFFFF


AM_REG_INFO0_CUSTOMER_TRIM_SIMO_BUCK_enable_S = 0
AM_REG_INFO0_CUSTOMER_TRIM_SIMO_BUCK_enable_M = 0x00000001


AM_REG_INFO0_SECURITY_WIRED_IFC_CFG0_BAUDRATE_S = 8
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG0_BAUDRATE_M = 0x0FFFFF00
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG0_DATALEN_S = 6
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG0_DATALEN_M = 0x000000C0
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG0_2STOP_S = 5
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG0_2STOP_M = 0x00000020
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG0_EVEN_S = 4
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG0_EVEN_M = 0x00000010
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG0_PAR_S = 3
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG0_PAR_M = 0x00000008
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG0_CTS_S = 2
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG0_CTS_M = 0x00000004
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG0_RTS_S = 1
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG0_RTS_M = 0x00000002
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG0_UART_S = 0
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG0_UART_M = 0x00000001


AM_REG_INFO0_SECURITY_WIRED_IFC_CFG1_PIN3_S = 24
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG1_PIN3_M = 0xFF000000
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG1_PIN2_S = 16
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG1_PIN2_M = 0x00FF0000
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG1_PIN1_S = 8
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG1_PIN1_M = 0x0000FF00
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG1_PIN0_S = 0
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG1_PIN0_M = 0x000000FF


AM_REG_INFO0_SECURITY_WIRED_IFC_CFG2_PINCFG_S = 0
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG2_PINCFG_M = 0xFFFFFFFF


AM_REG_INFO0_SECURITY_WIRED_IFC_CFG3_PINCFG_S = 0
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG3_PINCFG_M = 0xFFFFFFFF


AM_REG_INFO0_SECURITY_WIRED_IFC_CFG4_PINCFG_S = 0
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG4_PINCFG_M = 0xFFFFFFFF


AM_REG_INFO0_SECURITY_WIRED_IFC_CFG5_PINCFG_S = 0
AM_REG_INFO0_SECURITY_WIRED_IFC_CFG5_PINCFG_M = 0xFFFFFFFF


AM_REG_INFO0_SECURITY_VERSION_VERSION_S = 0
AM_REG_INFO0_SECURITY_VERSION_VERSION_M = 0xFFFFFFFF


AM_REG_INFO0_SECURITY_SRAM_RESV_SRAM_RESV_S = 0
AM_REG_INFO0_SECURITY_SRAM_RESV_SRAM_RESV_M = 0x000FFFFF


AM_REG_INFO0_MAINPTR_ADDRESS_S = 0
AM_REG_INFO0_MAINPTR_ADDRESS_M = 0xFFFFFFFF


AM_REG_INFO0_WIRED_TIMEOUT_TIMEOUT_S = 0
AM_REG_INFO0_WIRED_TIMEOUT_TIMEOUT_M = 0x0000FFFF


AM_REG_INFO0_SBR_SDCERT_ADDR_ICV_ICV_S = 0
AM_REG_INFO0_SBR_SDCERT_ADDR_ICV_ICV_M = 0xFFFFFFFF


AM_REG_INFO0_SBR_SDCERT_ADDR_OEM_OEM_S = 0
AM_REG_INFO0_SBR_SDCERT_ADDR_OEM_OEM_M = 0xFFFFFFFF
