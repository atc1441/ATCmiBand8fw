# *****************************************************************************
#
#    key_table.py
#
#    @brief Utility functions.
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

import re
import configparser

comment_regex = re.compile(r'#.*')

def main():
    key_table = import_key_table('key_table.cfg')
    print(key_table)

def import_key_table(filename):
    config = configparser.ConfigParser()
    config.read(filename)

    key_table = dict()
    for keyname in config.sections():
        if keyname != 'Symmetric Keys':
            key = config[keyname]
            index = int(key["index"], 0)
            key_format = key["format"]
            key_file = key["filename"]
            pass_file = key.get("passfile", None)

            key_table[index] = KeyTableEntry(key_format, key_file, pass_file)

    return key_table

def import_key_table_cfg(filename):

    with open(filename) as key_config_file:

        # Read the key config file, and strip out the comments
        key_config_lines = (comment_regex.sub('', line).strip() for line in key_config_file)
        key_config_lines = filter(None, key_config_lines)

        # Split each line by whitespace.
        key_parameter_list = (x.split() for x in key_config_lines)

        # Each line should contain an index, format, and filename in that
        # order. Add each key to a dictionary based on its index.
        key_table = dict()
        for params in key_parameter_list:
            index = int(params[0], 0)
            key_format = params[1]
            key_file = params[2]
            pass_file = None
            if len(params) > 3:
                pass_file = params[3]

            key_table[index] = KeyTableEntry(key_format, key_file, pass_file)

        return key_table


class KeyTableEntry:
    def __init__(self, key_format, key_file, pass_file=None):
        self.format = key_format
        self.filename = key_file
        self.pass_file = pass_file

    def __str__(self):
        return 'KeyTableEntry(format={}, filename={})'.format(self.format, self.filename)

    def __repr__(self):
        return self.__str__()

class AESTableEntry:
    def __init__(self, key_format, key_file, pass_file=None):
        self.format = key_format
        self.filename = key_file

    def __str__(self):
        return 'KeyTableEntry(format={}, filename={})'.format(self.format, self.filename)

    def __repr__(self):
        return self.__str__()

if __name__ == '__main__':
    main()
