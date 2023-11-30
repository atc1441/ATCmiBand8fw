#!/usr/bin/env python3

import argparse
import serial
import sys

#******************************************************************************
#
# Main function
#
#******************************************************************************
def main():

    # Open a serial port, and load the image at the specified address.
    #
    # We will use a UART timeout value of 12 second. This should be long
    # enough that the slave will respond to us, but short enough that a human
    # operator should see errors quickly..
    # Max flashing time depends on the amount of SRAM available.
    # For very large images, the flashing happens page by page.
    # However if the image can fit in the free SRAM, it could take a long time
    # for the whole image to be flashed at the end.
    # The largest image which can be stored depends on the max SRAM.
    # Assuming worst case ~100 ms/page of flashing time, and allowing for the
    # image to be close to occupying full SRAM (256K) which is 128 pages.

    # Read the binary file from the command line.
    with open(args.binfile, mode='rb') as binfile:
        application = binfile.read()

    if len(args.trailerfile) > 0:
        with open(args.trailerfile, mode='rb') as binfile:
            trailer= binfile.read()
    else:
        trailer = []

    print('Loading {} bytes over serial port {}...'.format(len(application), args.port), flush=True)

    with serial.Serial(args.port, args.baud, timeout=12) as ser:
        load_image(application, trailer, int(args.address, 0), ser)

    print('Done.')

#******************************************************************************
#
# Load image
#
# Given a serial port and a binary file, loads the target device using the
# UART.
#
#******************************************************************************
def load_image(application, trailer, address, ser):

    # Gather the important binary metadata.
    applen = len(application)
    crc = crc32(application)

    new_image = []
    new_image.extend(int_to_bytes(address))
    new_image.extend(int_to_bytes(applen))
    new_image.extend(int_to_bytes(crc))

    trailerlen = len(trailer)
    if trailerlen > 0:
        print('Adding Security Trailer')
        new_image.extend(int_to_bytes(trailerlen))
        new_image.extend(trailer)

    #print([hex(n) for n in new_image])

    if (args.bauddetect):
        # Making sure autobaud gets set.
        ser.write([0x55])
        response = ser.read(1)

        # Make sure we got the number of bytes we asked for.
        if len(response) == 0:
            raise NoResponseError

    # Send a New Image command.
    response = send_bytewise_command(0x2, new_image, 4, ser)

    if response[0] != 0x2:
        raise NoAckError

    # Set the override pin.
    send_ackd_command(0x5, [args.ovr, args.level], ser)

    # Loop over the bytes in the image, and send them to the target.
    resp = 0
    for x in range(0, applen, 512):
        # Split the application into chunks of 512 bytes.
        # This is the max chunk size supported by the UART bootloader
        chunk = application[x:x+512]

        # Build a data packet with a "data command" a "length" and the actual
        # payload bytes, and send it to the target.
        resp = send_bytewise_command(0x3, int_to_bytes(len(chunk)) + list(chunk), 4, ser)

    # Check the CRC.
    if word_from_bytes(resp, 0) == 0x3:

        # If the CRC was good, optionally reset the target and let it run.
        if (args.reset):
            send_command(0x4, [], 0, ser)
    else:
        print('CRC was bad')

#******************************************************************************
#
# Send ACK'd command
#
# Sends a command, and waits for an ACK.
#
#******************************************************************************
def send_ackd_command(command, params, ser):
    response = send_command(command, params, 4, ser)

    if response[0] != 0x2:
        raise NoAckError

#******************************************************************************
#
# Send command
#
# Sends a command, and waits for the response.
#
#******************************************************************************
def send_command(command, params, response_len, ser):

    # Send the command first.
    ser.write(int_to_bytes(command))

    # Next, send the parameters.
    for param in params:
        ser.write(int_to_bytes(param))

    response = ''
    response = ser.read(response_len)

    # Make sure we got the number of bytes we asked for.
    if len(response) != response_len:
        print('No response for command 0x{:08X}'.format(command))
        raise NoResponseError

    return response

#******************************************************************************
#
# Send a command that uses an array of bytes as its parameters.
#
#******************************************************************************
def send_bytewise_command(command, params, response_len, ser):
    # Send the command first.
    ser.write(int_to_bytes(command))

    # Next, send the parameters.
    ser.write(params)

    response = ''
    response = ser.read(response_len)

    # Make sure we got the number of bytes we asked for.
    if len(response) != response_len:
        print('No response for command 0x{:08X}'.format(command))
        raise NoResponseError

    return response

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
# Errors
#
#******************************************************************************
class BootError(Exception):
    pass

class NoAckError(BootError):
    pass

class NoResponseError(BootError):
    pass

#******************************************************************************
#
# Main program flow
#
#******************************************************************************
if __name__ == '__main__':

    parser = argparse.ArgumentParser(description =
                                     'UART Boot Host for Apollo or Apollo2')
    parser.add_argument('binfile',
                        help = 'Binary file to program into the target device')

    parser.add_argument('address', help = 'Link address (hex)')

    parser.add_argument('port', help = 'Serial COMx Port')

    parser.add_argument('-b', dest='baud', default=115200, type=int,
                        help = 'Baud Rate (default is 115200)')

    parser.add_argument('-o', dest ='ovr', default = 18, type=int,
                        help = 'Override pin (default is 18)')

    parser.add_argument('-l', dest='level', default=0, type=int,
                        help = 'Override pin polarity (0 or 1) Default is active low')

    parser.add_argument('-s', dest = 'trailerfile', default='',
                        help = 'Binary file for (optional) security trailer')

    parser.add_argument('-r', dest = 'reset', default=1, type=int,
                        help = 'Should it send reset command after image download? (0/1) (default is 1)')

    parser.add_argument('-a', dest = 'bauddetect', default=1, type=int,
                        help = 'Should it send a preamble for slave to detect the baudrate? (0/1) (default is 1)')

    args = parser.parse_args()

    main()
