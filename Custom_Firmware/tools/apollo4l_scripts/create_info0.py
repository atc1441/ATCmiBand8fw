#!/usr/bin/env python3

# *****************************************************************************
#
#    create_info0.py
#
#    Utility to create INFO0 for Apollo4
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

import argparse
import sys
import array
import os
import binascii
import importlib

from am_defines import *

#******************************************************************************
#
# Generate the info0 blob as per command line parameters
#
#******************************************************************************
def process(valid, version, output, mainPtr, certChainPtr, custTrim, wiredTimeout, u0, u1, u2, u3, u4, u5, sresv, sdcert, rmaoverride):


    import apollo4l_info0 as info0
    am_print("Apollo4l INFO0")

    #generate mutable byte array for the header
    hdr_binarray = bytearray([0xFF]*INFO0_SIZE_BYTES);

    # initialize signature
    if (valid == 1):
        fill_word(hdr_binarray, info0.AM_REG_INFO0_SIGNATURE0_O, AM_SECBOOT_INFO0_SIGN_PROGRAMMED0)
        fill_word(hdr_binarray, info0.AM_REG_INFO0_SIGNATURE1_O, AM_SECBOOT_INFO0_SIGN_PROGRAMMED1)
        fill_word(hdr_binarray, info0.AM_REG_INFO0_SIGNATURE2_O, AM_SECBOOT_INFO0_SIGN_PROGRAMMED2)
        fill_word(hdr_binarray, info0.AM_REG_INFO0_SIGNATURE3_O, AM_SECBOOT_INFO0_SIGN_PROGRAMMED3)
    elif (valid == 0):
        fill_word(hdr_binarray, info0.AM_REG_INFO0_SIGNATURE0_O, AM_SECBOOT_INFO0_SIGN_UINIT0)
        fill_word(hdr_binarray, info0.AM_REG_INFO0_SIGNATURE1_O, AM_SECBOOT_INFO0_SIGN_UINIT1)
        fill_word(hdr_binarray, info0.AM_REG_INFO0_SIGNATURE2_O, AM_SECBOOT_INFO0_SIGN_UINIT2)
        fill_word(hdr_binarray, info0.AM_REG_INFO0_SIGNATURE3_O, AM_SECBOOT_INFO0_SIGN_UINIT3)
    else:
        fill_word(hdr_binarray, info0.AM_REG_INFO0_SIGNATURE0_O, FLASH_INVALID)
        fill_word(hdr_binarray, info0.AM_REG_INFO0_SIGNATURE1_O, FLASH_INVALID)
        fill_word(hdr_binarray, info0.AM_REG_INFO0_SIGNATURE2_O, FLASH_INVALID)
        fill_word(hdr_binarray, info0.AM_REG_INFO0_SIGNATURE3_O, FLASH_INVALID)

    am_print("info0 Signature...")
    am_print([hex(hdr_binarray[n]) for n in range(0, 16)])

    # Flash wipe is no longer supported
    if (valid == 1):

        # Customer trim
        custTrim &= ~0x1
        am_print("Customer Trim Programmed = ", hex(custTrim))
        fill_word(hdr_binarray, info0.AM_REG_INFO0_CUSTOMER_TRIM_O, custTrim)

        # Wired timeout config
        timeout = ((wiredTimeout & info0.AM_REG_INFO0_WIRED_TIMEOUT_TIMEOUT_M) << info0.AM_REG_INFO0_WIRED_TIMEOUT_TIMEOUT_S)
        am_print("WiredTimeout = ", hex(timeout))
        fill_word(hdr_binarray, info0.AM_REG_INFO0_WIRED_TIMEOUT_O, timeout)

        # Wired UART cfg
        am_print("UART Config ", hex(u0), hex(u1), hex(u2), hex(u3), hex(u4), hex(u5))
        fill_word(hdr_binarray, info0.AM_REG_INFO0_SECURITY_WIRED_IFC_CFG0_O, u0)
        fill_word(hdr_binarray, info0.AM_REG_INFO0_SECURITY_WIRED_IFC_CFG1_O, u1)
        fill_word(hdr_binarray, info0.AM_REG_INFO0_SECURITY_WIRED_IFC_CFG2_O, u2)
        fill_word(hdr_binarray, info0.AM_REG_INFO0_SECURITY_WIRED_IFC_CFG3_O, u3)
        fill_word(hdr_binarray, info0.AM_REG_INFO0_SECURITY_WIRED_IFC_CFG4_O, u4)
        fill_word(hdr_binarray, info0.AM_REG_INFO0_SECURITY_WIRED_IFC_CFG5_O, u5)

        # version
        am_print("Version = ", hex(version))
        fill_word(hdr_binarray, info0.AM_REG_INFO0_SECURITY_VERSION_O, version)

        # main ptr
        am_print("Main Ptr = ", hex(mainPtr))
        fill_word(hdr_binarray, info0.AM_REG_INFO0_MAINPTR_O, mainPtr)

        # certChain ptr
        am_print("Cert Chain Ptr = ", hex(certChainPtr))
        fill_word(hdr_binarray, info0.AM_REG_INFO0_CERTCHAINPTR_O, certChainPtr)

        # RMA Override info
        am_print("RMA Override info = ", hex(rmaoverride))
        fill_word(hdr_binarray, info0.AM_REG_INFO0_SECURITY_RMAOVERRIDE_O, rmaoverride)

        # SDCert ptr
        am_print("SDCert Ptr = ", hex(sdcert))
        fill_word(hdr_binarray, info0.AM_REG_INFO0_SBR_SDCERT_ADDR_O, sdcert)

        # SRAM Reservation
        sresv = ((sresv & info0.AM_REG_INFO0_SECURITY_SRAM_RESV_SRAM_RESV_M) << info0.AM_REG_INFO0_SECURITY_SRAM_RESV_SRAM_RESV_S)
        am_print("SRAM Reservation = ", hex(sresv))
        fill_word(hdr_binarray, info0.AM_REG_INFO0_SECURITY_SRAM_RESV_O, sresv)


    # now output the binary array in the proper order
    with open(output + '.bin', mode = 'wb') as out:
        am_print("Writing to file ", output + '.bin')
        out.write(hdr_binarray)

def parse_arguments():
    parser = argparse.ArgumentParser(description =
                     'Generate Apollo4l Info0 Blob')

    parser.add_argument('output',
                        help = 'Output filename (without the extension)')

    parser.add_argument('--valid', dest = 'valid', type=auto_int, default=1, choices = [0,1,2],
                        help = 'INFO0 Valid 0 = Uninitialized, 1 = Valid, 2 = Invalid (Default = 1)?')

    parser.add_argument('--version', dest = 'version', type=auto_int, default=0,
                        help = 'version (Default = 0)?')

    parser.add_argument('--main', dest = 'mainPtr', type=auto_int, default=hex(AM_SECBOOT_DEFAULT_NONSECURE_MAIN),
                        help = 'Main Firmware location (Default = ' + str(hex(AM_SECBOOT_DEFAULT_NONSECURE_MAIN)) + ')?')

    parser.add_argument('--cchain', dest = 'certChainPtr', type=auto_int, default=hex(0xFFFFFFFF),
                        help = 'Certificate Chain location (Default = 0xFFFFFFFF)?')

    parser.add_argument('--trim', dest = 'custTrim', type=auto_int, default=hex(0x0),
                        help = 'customer trim (bit 0 <old SIMOBUCKEN> has been deprecated and will be forced to be 0)?')

    parser.add_argument('--wTO', dest = 'wiredTimeout', type=auto_int, default=20000,
                        help = 'Wired interface timeout in millisec (default = 20000)')

    parser.add_argument('--u0', dest = 'u0', type=auto_int, default=hex(0xFFFFFFFF),
                        help = 'UART Config 0 (default = 0xFFFFFFFF)')

    parser.add_argument('--u1', dest = 'u1', type=auto_int, default=hex(0xFFFFFFFF),
                        help = 'UART Config 1 (default = 0xFFFFFFFF)')

    parser.add_argument('--u2', dest = 'u2', type=auto_int, default=hex(0xFFFFFFFF),
                        help = 'UART Config 2 (default = 0xFFFFFFFF)')

    parser.add_argument('--u3', dest = 'u3', type=auto_int, default=hex(0xFFFFFFFF),
                        help = 'UART Config 3 (default = 0xFFFFFFFF)')

    parser.add_argument('--u4', dest = 'u4', type=auto_int, default=hex(0xFFFFFFFF),
                        help = 'UART Config 4 (default = 0xFFFFFFFF)')

    parser.add_argument('--u5', dest = 'u5', type=auto_int, default=hex(0xFFFFFFFF),
                        help = 'UART Config 5 (default = 0xFFFFFFFF)')

    parser.add_argument('--sdcert', dest = 'sdcert', type=auto_int, default=hex(0x1ff400),
                        help = 'Secure Debug Cert Address (default = 0x1ff400)')

    parser.add_argument('--rma', dest = 'rmaoverride', type=auto_int, default=hex(0x2),
                        help = 'RMA Override Config 2 = Enabled, 5 = Disabled (default = 0x2)')

    parser.add_argument('--sresv', dest='sresv', type=auto_int, default = hex(0x0),
                        help='SRAM Reservation (Default 0x0)')

    parser.add_argument('--loglevel', dest='loglevel', type=auto_int, default=AM_PRINT_LEVEL_INFO,
                        choices = range(AM_PRINT_LEVEL_MIN, AM_PRINT_LEVEL_MAX+1),
                        help=helpPrintLevel)

    args = parser.parse_args()

    return args

#******************************************************************************
#
# Main function.
#
#******************************************************************************
def main():
    # Read the arguments.
    args = parse_arguments()
    am_set_print_level(args.loglevel)

    process(args.valid, args.version, args.output, args.mainPtr, args.certChainPtr, args.custTrim, args.wiredTimeout, args.u0, args.u1, args.u2, args.u3, args.u4, args.u5, args.sresv, args.sdcert, args.rmaoverride)

if __name__ == '__main__':
    main()
