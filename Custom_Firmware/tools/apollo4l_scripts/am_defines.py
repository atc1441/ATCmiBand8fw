#!/usr/bin/env python3

# *****************************************************************************
#
#    am_defines.py
#
#    Utility functions.
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

import sys
from Crypto.Cipher import AES
from Crypto.PublicKey import RSA
from Crypto.Signature import PKCS1_v1_5
from Crypto.Signature import pss
from Crypto.Hash import SHA256
import array
import hashlib
import hmac
import os
import binascii

MAX_DOWNLOAD_SIZE           = 0x48000               # 288K
AM_SECBOOT_DEFAULT_NONSECURE_MAIN   = 0x18000

# Encryption Algorithm

# String constants

# Authentication Algorithm

FLASH_INVALID               = 0xFFFFFFFF

# KeyWrap Mode

#******************************************************************************
#
# Magic Numbers
#
#******************************************************************************
AM_IMAGE_MAGIC_SBL            = 0xA3
AM_IMAGE_MAGIC_ICV_CHAIN      = 0xAC
AM_IMAGE_MAGIC_SECURE         = 0xC0
AM_IMAGE_MAGIC_OEM_CHAIN      = 0xCC
AM_IMAGE_MAGIC_NONSECURE      = 0xCB
AM_IMAGE_MAGIC_INFO0          = 0xCF
AM_IMAGE_MAGIC_CONTAINER      = 0xC1
AM_IMAGE_MAGIC_KEYREVOKE      = 0xCE
AM_IMAGE_MAGIC_DOWNLOAD       = 0xCD

#******************************************************************************
#
# Wired Message Types
#
#******************************************************************************
AM_SECBOOT_WIRED_MSGTYPE_HELLO          = 0
AM_SECBOOT_WIRED_MSGTYPE_STATUS         = 1
AM_SECBOOT_WIRED_MSGTYPE_OTADESC        = 2
AM_SECBOOT_WIRED_MSGTYPE_UPDATE         = 3
AM_SECBOOT_WIRED_MSGTYPE_ABORT          = 4
AM_SECBOOT_WIRED_MSGTYPE_RECOVER        = 5
AM_SECBOOT_WIRED_MSGTYPE_RESET          = 6
AM_SECBOOT_WIRED_MSGTYPE_ACK            = 7
AM_SECBOOT_WIRED_MSGTYPE_DATA           = 8


#******************************************************************************
#
# Wired Message ACK Status
#
#******************************************************************************
AM_SECBOOT_WIRED_ACK_STATUS_SUCCESS              = 0
AM_SECBOOT_WIRED_ACK_STATUS_FAILURE              = 1
AM_SECBOOT_WIRED_ACK_STATUS_INVALID_INFO0        = 2
AM_SECBOOT_WIRED_ACK_STATUS_CRC                  = 3
AM_SECBOOT_WIRED_ACK_STATUS_SEC                  = 4
AM_SECBOOT_WIRED_ACK_STATUS_MSG_TOO_BIG          = 5
AM_SECBOOT_WIRED_ACK_STATUS_UNKNOWN_MSGTYPE      = 6
AM_SECBOOT_WIRED_ACK_STATUS_INVALID_ADDR         = 7
AM_SECBOOT_WIRED_ACK_STATUS_INVALID_OPERATION    = 8
AM_SECBOOT_WIRED_ACK_STATUS_INVALID_PARAM        = 9
AM_SECBOOT_WIRED_ACK_STATUS_SEQ                  = 10
AM_SECBOOT_WIRED_ACK_STATUS_TOO_MUCH_DATA        = 11

#******************************************************************************
#
# Definitions related to Image Headers
#
#******************************************************************************
AM_MAX_UART_MSG_SIZE            = 8192  # 8K buffer in SBL
# Max Wired Update Image header size - this includes optional sign & encryption info
AM_WU_IMAGEHDR_SIZE             = (16 + 384 + 48 + 16)

#******************************************************************************
#
# INFOSPACE related definitions
#
#******************************************************************************
AM_SECBOOT_INFO0_SIGN_PROGRAMMED0   = 0x48EAAD88
AM_SECBOOT_INFO0_SIGN_PROGRAMMED1   = 0xC9705737
AM_SECBOOT_INFO0_SIGN_PROGRAMMED2   = 0x0A6B8458
AM_SECBOOT_INFO0_SIGN_PROGRAMMED3   = 0xE41A9D74

AM_SECBOOT_INFO0_SIGN_UINIT0        = 0x5B75A5FA
AM_SECBOOT_INFO0_SIGN_UINIT1        = 0x7B9C8674
AM_SECBOOT_INFO0_SIGN_UINIT2        = 0x869A96FE
AM_SECBOOT_INFO0_SIGN_UINIT3        = 0xAEC90860

INFO0_SIZE_BYTES                    = (2 * 1024)
INFO1_SIZE_BYTES                    = (6 * 1024)


#******************************************************************************
#
# CRC using ethernet poly, as used by Apollo4 hardware for validation
#
#******************************************************************************
def crc32(L):
    return (binascii.crc32(L) & 0xFFFFFFFF)

#******************************************************************************
#
# Pad the text to the block_size. bZeroPad determines how to handle text which
# is already multiple of block_size
#
#******************************************************************************
def pad_to_block_size(text, block_size, bZeroPad):
    text_length = len(text)
    amount_to_pad = block_size - (text_length % block_size)
    if (amount_to_pad == block_size):
        if (bZeroPad == 0):
            amount_to_pad = 0
    for i in range(0, amount_to_pad, 1):
        text += bytes(chr(amount_to_pad), 'ascii')
    return text


#******************************************************************************
#
# AES CBC encryption
#
#******************************************************************************
def encrypt_app_aes(cleartext, encKey, iv):
    key = array.array('B', encKey).tostring()
    ivVal = array.array('B', iv).tostring()
    plaintext = array.array('B', cleartext).tostring()

    encryption_suite = AES.new(key, AES.MODE_CBC, ivVal)
    cipher_text = encryption_suite.encrypt(plaintext)

    return cipher_text

#******************************************************************************
#
# AES 128 CBC encryption
#
#******************************************************************************
def encrypt_app_aes128(cleartext, encKey, iv):
    key = array.array('B', encKey).tostring()
    ivVal = array.array('B', iv).tostring()
    plaintext = array.array('B', cleartext).tostring()

    encryption_suite = AES.new(key, AES.MODE_CBC, ivVal)
    cipher_text = encryption_suite.encrypt(plaintext)

    return cipher_text

#******************************************************************************
#
# SHA256 HMAC
#
#******************************************************************************
def compute_hmac(key, data):
    sig = hmac.new(array.array('B', key).tostring(), array.array('B', data).tostring(), hashlib.sha256).digest()
    return sig

#******************************************************************************
#
# RSA PKCS1_v1_5 sign
#
#******************************************************************************
def compute_rsa_sign(prvKeyFile, data):
    key = open(prvKeyFile, "r").read()
    rsakey = RSA.importKey(key)
    signer = PKCS1_v1_5.new(rsakey)
    digest = SHA256.new()
    digest.update(bytes(data))
    sign = signer.sign(digest)
    return sign

#******************************************************************************
#
# RSA PKCS1_v1_5 sign verification
#
#******************************************************************************
def verify_rsa_sign(pubKeyFile, data, sign):
    key = open(pubKeyFile, "r").read()
    rsakey = RSA.importKey(key)
    #print(hex(rsakey.n))
    verifier = PKCS1_v1_5.new(rsakey)
    digest = SHA256.new()
    digest.update(bytes(data))
    return verifier.verify(digest, sign)

#******************************************************************************
#
# RSA PSS signing function.
#
#******************************************************************************
def compute_rsa_pss_sign(prvKeyFile, data):
    # Import the key, hash the data, create an RSA pss signature from the hash.
    with open(prvKeyFile, 'rb') as private_key_file:
        key = RSA.import_key(private_key_file.read())
        h = SHA256.new(data)
        signature = pss.new(key).sign(h)
        return signature, h

#******************************************************************************
#
# RSA PSS signature verification.
#
#******************************************************************************
def verify_rsa_pss_sign(pubKeyFile, data, sign):
    # Read the public key, hash the message, and use our key to make sure the
    # signature matches the hash.
    with open(pubKeyFile, 'rb') as public_key_file:
        key = RSA.import_key(public_key_file.read())
        h = SHA256.new(data)
        verifier = pss.new(key)

        try:
            verifier.verify(h, signature)
            return True
        except (ValueError, TypeError):
            return False

#******************************************************************************
#
# Fill one word in bytearray
#
#******************************************************************************
def fill_word(barray, offset, w):
    barray[offset + 0]  = (w >>  0) & 0x000000ff;
    barray[offset + 1]  = (w >>  8) & 0x000000ff;
    barray[offset + 2]  = (w >> 16) & 0x000000ff;
    barray[offset + 3]  = (w >> 24) & 0x000000ff;


#******************************************************************************
#
# Turn a 32-bit number into a series of bytes for transmission.
#
# This command will split a 32-bit integer into an array of bytes, ordered
# LSB-first for transmission over the UART.
#
#******************************************************************************
def int_to_bytes(n):
    A = [n & 0xFF,
         (n >> 8) & 0xFF,
         (n >> 16) & 0xFF,
         (n >> 24) & 0xFF]

    return A

#******************************************************************************
#
# Extract a word from a byte array
#
#******************************************************************************
def word_from_bytes(B, n):
    return (B[n] + (B[n + 1] << 8) + (B[n + 2] << 16) + (B[n + 3] << 24))


#******************************************************************************
#
# automatically figure out the integer format (base 10 or 16)
#
#******************************************************************************
def auto_int(x):
    return int(x, 0)

#******************************************************************************
#
# User controllable Prints control
#
#******************************************************************************
# Defined print levels
AM_PRINT_LEVEL_MIN     = 0
AM_PRINT_LEVEL_NONE    = AM_PRINT_LEVEL_MIN
AM_PRINT_LEVEL_ERROR   = 1
AM_PRINT_LEVEL_INFO    = 2
AM_PRINT_LEVEL_VERBOSE = 4
AM_PRINT_LEVEL_DEBUG   = 5
AM_PRINT_LEVEL_MAX     = AM_PRINT_LEVEL_DEBUG

# Global variable to control the prints
AM_PRINT_VERBOSITY = AM_PRINT_LEVEL_INFO

helpPrintLevel = 'Set Log Level (0: None), (1: Error), (2: INFO), (4: Verbose), (5: Debug) [Default = Info]'

def am_set_print_level(level):
    global AM_PRINT_VERBOSITY
    AM_PRINT_VERBOSITY = level

def am_print(*args, level=AM_PRINT_LEVEL_INFO, **kwargs):
    global AM_PRINT_VERBOSITY
    if (AM_PRINT_VERBOSITY >= level):
        print(*args, **kwargs)
