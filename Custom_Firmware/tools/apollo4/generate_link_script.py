#!/usr/bin/env python3

# *****************************************************************************
#
#    generate_link_script.py
#
#    @brief Generate link control files.
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
import yaml
import keil_link
import keil6_link
import gcc_link
import iar_link
import string
import os
import datetime


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('config_file')
    parser.add_argument('-k', dest='keil_link', default='keil/linker_script.sct')
    parser.add_argument('-s', dest='keil6_link', default='keil6/linker_script.sct')
    parser.add_argument('-g', dest='gcc_link', default='gcc/linker_script.ld')
    parser.add_argument('-i', dest='iar_link', default='iar/linker_script.icf')
    parser.add_argument('-m', dest='mem_header', default='src/am_memory_map.h')

    args = parser.parse_args()

    print('Generating linker scripts.')

    # Read the configuration file for memory options.
    memory_config = None
    with open(args.config_file) as config_file_obj:
        config = yaml.load(config_file_obj, Loader=yaml.FullLoader)
        memory_config = config['MemorySections']
        calculate_section_lengths(memory_config)

    if os.path.isdir('keil'):
        # Specify the newline parameter to output with Unix line endings.
        with open(args.keil_link, 'w', newline='\n') as keil_link_file:
            keil_link_file.write(keil_link.generate_link_script(memory_config))
            
    if os.path.isdir('keil6'):
        # Specify the newline parameter to output with Unix line endings.
        with open(args.keil6_link, 'w', newline='\n') as keil6_link_file:
            keil6_link_file.write(keil6_link.generate_link_script(memory_config))

    if os.path.isdir('gcc'):
        # Specify the newline parameter to output with Unix line endings.
        with open(args.gcc_link, 'w', newline='\n') as gcc_link_file:
            gcc_link_file.write(gcc_link.generate_link_script(memory_config))

    if os.path.isdir('iar'):
        # Specify the newline parameter to output with Unix line endings.
        with open(args.iar_link, 'w', newline='\n') as iar_link_file:
            iar_link_file.write(iar_link.generate_link_script(memory_config))

    # Specify the newline parameter to output with Unix line endings.
    with open(args.mem_header, 'w', newline='\n') as mem_header_file:
        mem_header_file.write(generate_memory_header(memory_config))


def calculate_section_lengths(config):
    for name, params in config.items():
        if name != "custom_sections":
            if 'length' in params:
                params['end'] = params['start'] + params['length'] - 1
            else:
                params['length'] = params['end'] - params['start'] + 1

    return


def generate_memory_header(config: dict):
    starts = []
    lengths = []
    for name, params in config.items():
        if name != "custom_sections":
            start = params['start']
            length = params['length']

            fmt = {
                'name': 'AM_MEM_' + name.upper(),
                'name_length' : 'AM_MEM_' + name.upper() + '_SIZE',
                'start': hex_format(start),
                'length': hex_format(length)
            }

            starts.append('#define {name:30} {start}'.format(**fmt))
            lengths.append('#define {name_length:30} {length}'.format(**fmt))

    return memory_header_template.substitute(block_starts='\n'.join(starts), block_lengths='\n'.join(lengths))

def hex_format(n):
    return '0x{:08X}'.format(n)


memory_header_string = '''\
//*****************************************************************************
//
// file am_memory_map.h
//
// brief Memory map include file.
//
// This file is generated by "generate_link_script.py".
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) {copyrightyear}, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of the AmbiqSuite Development Package.
//
//*****************************************************************************

#ifndef AM_MEMORY_MAP_H
#define AM_MEMORY_MAP_H

#ifdef __cplusplus
extern "C"
{{
#endif

//*****************************************************************************
//
// Memory block locations.
//
//*****************************************************************************
${{block_starts}}

//*****************************************************************************
//
// Memory block sizes (in bytes)
//
//*****************************************************************************
${{block_lengths}}

#ifdef __cplusplus
}}
#endif

#endif // AM_MEMORY_MAP_H
'''

currdatetime = datetime.datetime.now()
memory_header_template = string.Template(memory_header_string.format(copyrightyear=currdatetime.year))


if __name__ == '__main__':
    main()
