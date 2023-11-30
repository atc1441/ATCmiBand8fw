#!/usr/bin/env python3

import argparse


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
# Read in the binary files and output the merged binary
#
#******************************************************************************
#def process(boot_loader_filename, app_filename,  output):
def process(load_address, app_filename, secinfo_file_name, app_ver, bin_type, str_type, output, align):

    # Open the file, and read it into an array of integers.
    with open(app_filename, mode = 'rb') as f_app:
        app_binarray = f_app.read()
        f_app.close()

    sec_binarray = bytearray([])
    seclen = 0
    # Open the file, and read it into an array of integers.
    if len(secinfo_file_name) > 0:
        with open(secinfo_file_name, mode='rb') as f_sec:
            sec_binarray= f_sec.read()
            seclen = len(sec_binarray)

    app_length  = len(app_binarray)

    hdr_length  = 48;   #fixed header length
    print("hdr_length ",hdr_length);

    #generate mutable byte array for the boot loader
    hdr_binarray = bytearray([0]*hdr_length);

    # Insert encryption flag, always 0
    hdr_binarray[0]  = 0x00;
    hdr_binarray[1]  = 0x00;
    hdr_binarray[2]  = 0x00;
    hdr_binarray[3]  = 0x00;
    
    # Insert the application binary load address.
    loadaddress = int(load_address,16)
    print("load_address ",hex(loadaddress), "(",load_address,")")
    hdr_binarray[4]  = (loadaddress >>  0) & 0x000000ff;
    hdr_binarray[5]  = (loadaddress >>  8) & 0x000000ff;
    hdr_binarray[6]  = (loadaddress >> 16) & 0x000000ff;
    hdr_binarray[7]  = (loadaddress >> 24) & 0x000000ff;


    app_length  = len(app_binarray)
    print("app_size ",hex(app_length), "(",app_length,")")
    app_crc = crc32(app_binarray)
    print("app_crc =  ",hex(app_crc));
    print("Security Info Length", hex(seclen))

    pad_binarray = bytearray([])
    pad_size = 0
    if (seclen > 0):
        if (seclen % int(align) != 0):
            # Add Padding
            pad_size = int(align) - (seclen % int(align))
            pad_binarray = bytearray([0]*pad_size);

    # put the application binary size
    # Total blob length
    hdr_binarray[8]   = ((app_length + pad_size + seclen) >>  0) & 0x000000ff
    hdr_binarray[9]   = ((app_length + pad_size + seclen) >>  8) & 0x000000ff
    hdr_binarray[10]  = ((app_length + pad_size + seclen) >> 16) & 0x000000ff
    hdr_binarray[11]  = ((app_length + pad_size + seclen) >> 24) & 0x000000ff
	
    # compute the CRC for the blob
    crc = crc32(sec_binarray + pad_binarray + app_binarray)
    print("blob crc =  ",hex(crc));
    hdr_binarray[12]  = (crc >>  0) & 0x000000ff
    hdr_binarray[13]  = (crc >>  8) & 0x000000ff
    hdr_binarray[14]  = (crc >> 16) & 0x000000ff
    hdr_binarray[15]  = (crc >> 24) & 0x000000ff

    hdr_binarray[16]  = ((seclen + pad_size) >>  0) & 0x000000ff
    hdr_binarray[17]  = ((seclen + pad_size) >>  8) & 0x000000ff
    hdr_binarray[18]  = ((seclen + pad_size) >> 16) & 0x000000ff
    hdr_binarray[19]  = ((seclen + pad_size) >> 24) & 0x000000ff

	# word RFU
    hdr_binarray[20] = 0xff
    hdr_binarray[21] = 0xff
    hdr_binarray[22] = 0xff
    hdr_binarray[23] = 0xff

	# word RFU
    hdr_binarray[24] = 0xFF
    hdr_binarray[25] = 0xFF
    hdr_binarray[26] = 0xFF
    hdr_binarray[27] = 0xFF

	# word RFU
    hdr_binarray[28] = 0xFF
    hdr_binarray[29] = 0xFF
    hdr_binarray[30] = 0xFF
    hdr_binarray[31] = 0xFF
	
	# application software version here
    print("app_ver",int(app_ver,16), "(",app_ver,")")
    appver = int(app_ver,16)
    hdr_binarray[32]  = (appver >>  0) & 0x000000ff;
    hdr_binarray[33]  = (appver >>  8) & 0x000000ff;
    hdr_binarray[34]  = (appver >> 16) & 0x000000ff;
    hdr_binarray[35]  = (appver >> 24) & 0x000000ff;
	
	# binary type here
    print("bin_type",int(bin_type,16), "(",bin_type,")")
    bintype = int(bin_type,16)
    hdr_binarray[36]  = (bintype >>  0) & 0x000000ff;
    hdr_binarray[37]  = (bintype >>  8) & 0x000000ff;
    hdr_binarray[38]  = (bintype >> 16) & 0x000000ff;
    hdr_binarray[39]  = (bintype >> 24) & 0x000000ff;
	
    # storage type here, 0 = internal, 1 = external
    print("str_type",int(str_type,16), "(",str_type,")")
    strtype = int(str_type,16)
    if strtype != 0:
        strtype = 1
    hdr_binarray[40] = (strtype >>  0) & 0x000000ff;
    hdr_binarray[41] = (strtype >>  8) & 0x000000ff;
    hdr_binarray[42] = (strtype >> 16) & 0x000000ff;
    hdr_binarray[43] = (strtype >> 24) & 0x000000ff;
	
	# word RFU
    hdr_binarray[44] = 0xff
    hdr_binarray[45] = 0xff
    hdr_binarray[46] = 0xff
    hdr_binarray[47] = 0xff


    # now output all three binary arrays in the proper order
    with open(output + '.bin', mode = 'wb') as out:
        out.write(hdr_binarray)
        if (seclen > 0):
            print('Adding Security Info of {} bytes...'.format(seclen), flush=True)
            out.write(sec_binarray)
            if (pad_size != 0):
                # Add Padding
                print('Adding padding of {} bytes...'.format(pad_size), flush=True)
                out.write(pad_binarray)
        out.write(app_binarray)

def parse_arguments():
    parser = argparse.ArgumentParser(description =
                     'Combine two binary files in to a single download.')

    parser.add_argument('--load-address', dest='loadaddress', default='0x4000',
                        help='Load address of the application.')

    parser.add_argument('--appbin', dest='appbin', default='../keil/bin/freertos_fit_amota.bin',
                        help='Application binary file (app.bin)')
                        
    parser.add_argument('--secbin', dest = 'secbin', default='',
                        help = 'Binary file for (optional) security information')

    # add arg version
    parser.add_argument('--version', dest='app_ver', default='0x0',
                        help = 'Software version of the OTA image.')
    
    # add arg binary type
    parser.add_argument('--binary-type', dest='bin_type', default='0x0',
                        help = 'Binary type (0 = Firmware, 1 = Data) to be tranferred OTA')
                                                    
    # add arg binary type
    parser.add_argument('--storage-type', dest='str_type', default='0x0',
                        help = 'Storage type to for the image OTA.')

    parser.add_argument('-o', dest = 'output', default = 'binary_array',
                        help = 'Output filename (without the extension)')

    parser.add_argument('-a', dest = 'alignment', default=4,
                        help = 'Desired alignment for appbin in image blob')


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

    process(args.loadaddress, args.appbin, args.secbin, args.app_ver, args.bin_type, args.str_type, args.output, args.alignment)

if __name__ == '__main__':
    main()
