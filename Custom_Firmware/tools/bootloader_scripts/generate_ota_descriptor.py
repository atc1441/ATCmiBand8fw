#!/usr/bin/env python3
#******************************************************************************
#
# This scripts creates a single Blob, which can be programmed at OTA_POINTER
# The first 4 bytes contain address to OTA Descriptor, which immediately follows
#
#******************************************************************************


import argparse
import sys

magicnum = 0xDEADCAFE

#******************************************************************************
#
# Main function
#
#******************************************************************************
def main():

    # Read the binary file from the command line.
    with open(args.binfile, mode='rb') as binfile:
        application = binfile.read()

    if len(args.trailerfile) > 0:
        with open(args.trailerfile, mode='rb') as binfile:
            trailer= binfile.read()
    else:
        trailer = []

    applen = len(application)
    crc = crc32(application)

    ota_desc = []

    # OTA Descriptor immediately follows the OTA_POINTER
    ota_desc.extend(int_to_bytes(magicnum))
    ota_desc.extend(int_to_bytes(int(args.linkaddr, 0)))
    ota_desc.extend(int_to_bytes(applen))
    ota_desc.extend(int_to_bytes(crc))

    trailerlen = len(trailer)
    ota_desc.extend(int_to_bytes(trailerlen))
    ota_desc.extend(int_to_bytes(int(args.options, 0)))
    ota_desc.extend(int_to_bytes(int(args.flashaddr , 0)+ 40)) # pui32SecInfoPtr
    if (args.otaimageaddr != 0):
        # Image has been flashed seprately. Just code the address in descriptor
        ota_desc.extend(int_to_bytes(int(args.otaimageaddr, 0))) # pui32ImageAddr
    else:
        # Image is written immediately following the OTA descriptor
        # Need to determine if there is a padding needed in between to ensure proper alignment
        image_addr = int(args.flashaddr , 0) + 40 + trailerlen
        if (image_addr % int(args.alignment) != 0):
            pad_size = int(args.alignment) - (image_addr % int(args.alignment))
        else:
            pad_size = 0
        image_addr = image_addr + pad_size
        ota_desc.extend(int_to_bytes(image_addr)) # pui32ImageAddr
    ota_desc_crc = crc32(ota_desc)
    # OTA_POINTER contents
    new_image = []
    new_image.extend(int_to_bytes(int(args.flashaddr , 0)+ 4)) # OTA Descriptor address
    new_image.extend(ota_desc) # OTA Descriptor
    new_image.extend(int_to_bytes(ota_desc_crc))

    if trailerlen > 0:
        print('Adding Security Trailer')
        new_image.extend(trailer)
    if (pad_size != 0):
        print('Adding padding of {} bytes...'.format(pad_size), flush=True)
        pad_binarray = bytearray([0]*pad_size);
        new_image.extend(pad_binarray)
    if (args.otaimageaddr == 0):
        # Image
        new_image.extend(application)
    print('Saving OTA descriptor image {} bytes to {}...'.format(len(new_image), args.outfile), flush=True)
    with open(args.outfile, mode='wb') as imagefile:
        imagebytearray = bytearray(new_image)
        imagefile.write(imagebytearray)

    print('Done.')

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
# CRC function that matches the CRC used by the Apollo bootloader.
#
#******************************************************************************
poly32 = 0x1EDC6F41
def crc32(L):
    rem = 0
    for b in L:
        rem = rem ^ (b << 24)
        for i in range(8):
            if rem & 0x80000000:
                rem = ((rem << 1) ^ poly32)
            else:
                rem = (rem << 1)

            rem = rem & 0xFFFFFFFF
    return rem

#******************************************************************************
#
# Main program flow
#
#******************************************************************************
if __name__ == '__main__':

    parser = argparse.ArgumentParser(description =
                                     'Utility to generate OTA blob for multi boot test app')
    parser.add_argument('binfile',
                        help = 'Binary file to program into the target device')

    parser.add_argument('flashaddr', help = 'Flash address (hex) where blob will be flashed to')

    parser.add_argument('linkaddr', help = 'Link address (hex) of the program file')

    parser.add_argument('-a', dest = 'alignment', default=4,
                        help = 'Desired alignment for application in image')

    parser.add_argument('options', 
                        help = 'OTA options parameter')

    parser.add_argument('-s', dest = 'trailerfile', default='',
                        help = 'Binary file for (optional) security trailer')

    parser.add_argument('-i', dest='otaimageaddr', default=0,
                        help = '(optional) Flash address (hex) where the OTA image is flashed to - if not specified, the image is included in outfile immediately following the descriptor')


    parser.add_argument('-o', dest = 'outfile',
                        help = 'output filename for the ota descriptor')

    args = parser.parse_args()

    main()
