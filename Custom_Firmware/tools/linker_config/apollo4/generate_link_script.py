#!/usr/bin/env python3

import argparse
import yaml
import string
import os
import datetime

from . import keil_link
from . import keil6_link
from . import gcc_link
from . import iar_link


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('config_file')
    parser.add_argument('-k', dest='keil_link', default='keil/linker_script.sct')
    parser.add_argument('-s', dest='keil6_link', default='keil6/linker_script.sct')
    parser.add_argument('-g', dest='gcc_link', default='gcc/linker_script.ld')
    parser.add_argument('-i', dest='iar_link', default='iar/linker_script.icf')
    parser.add_argument('-m', dest='mem_header', default='src/am_memory_map.h')

    args = parser.parse_args()

    print('Generating linker scripts')

    # Read the configuration file for memory options.
    memory_config = None
    with open(args.config_file) as config_file_obj:
        config = yaml.load(config_file_obj, Loader=yaml.FullLoader)
        memory_config = config['MemorySections']

    if os.path.isfile(args.keil_link):
        # Specify the newline parameter to output with Unix line endings.
        with open(args.keil_link, 'w', newline='\n') as keil_link_file:
            keil_link_file.write(keil_link.generate_link_script(memory_config))

    if os.path.isfile(args.keil6_link):
        # Specify the newline parameter to output with Unix line endings.
        with open(args.keil6_link, 'w', newline='\n') as keil6_link_file:
            keil6_link_file.write(keil6_link.generate_link_script(memory_config))

    if os.path.isfile(args.gcc_link):
        # Specify the newline parameter to output with Unix line endings.
        with open(args.gcc_link, 'w', newline='\n') as gcc_link_file:
            gcc_link_file.write(gcc_link.generate_link_script(memory_config))

    if os.path.isfile(args.iar_link):
        # Specify the newline parameter to output with Unix line endings.
        with open(args.iar_link, 'w', newline='\n') as iar_link_file:
            iar_link_file.write(iar_link.generate_link_script(memory_config))

    # Specify the newline parameter to output with Unix line endings.
    with open(args.mem_header, 'w', newline='\n') as mem_header_file:
        mem_header_file.write(generate_memory_header(memory_config))


def generate_files(config_file, toolchains):

    memory_config = None
    with open(config_file) as config_file_obj:
        config = yaml.load(config_file_obj, Loader=yaml.FullLoader)
        memory_config = config['MemorySections']

    if 'keil' in toolchains:
        # Specify the newline parameter to output with Unix line endings.
        with open('keil/linker_script.sct', 'w', newline='\n') as keil_link_file:
            keil_link_file.write(keil_link.generate_link_script(memory_config))

    if 'keil6' in toolchains:
        # Specify the newline parameter to output with Unix line endings.
        with open('keil6/linker_script.sct', 'w', newline='\n') as keil6_link_file:
            keil6_link_file.write(keil6_link.generate_link_script(memory_config))

    if 'gcc' in toolchains:
        # Specify the newline parameter to output with Unix line endings.
        with open('gcc/linker_script.ld', 'w', newline='\n') as gcc_link_file:
            gcc_link_file.write(gcc_link.generate_link_script(memory_config))

    if 'iar' in toolchains:
        # Specify the newline parameter to output with Unix line endings.
        with open('iar/linker_script.icf', 'w', newline='\n') as iar_link_file:
            iar_link_file.write(iar_link.generate_link_script(memory_config))

    # Specify the newline parameter to output with Unix line endings.
    with open('src/am_memory_map.h', 'w', newline='\n') as mem_header_file:
        mem_header_file.write(generate_memory_header(memory_config))


def generate_memory_header(config):
    D = dict()
    D['mcu_mram_start'] = hex_format(config['MCU_MRAM']['start'])
    D['dsp0_mram_start'] = hex_format(config['DSP0_MRAM']['start'])
    D['dsp1_mram_start'] = hex_format(config['DSP1_MRAM']['start'])
    D['mcu_tcm_start'] = hex_format(config['MCU_TCM']['start'])
    D['dsp0_tcm_start'] = hex_format(config['DSP0_TCM']['start'])
    D['dsp1_tcm_start'] = hex_format(config['DSP1_TCM']['start'])
    D['mcu_sram_start'] = hex_format(config['MCU_SRAM']['start'])
    D['dsp0_sram_start'] = hex_format(config['DSP0_SRAM']['start'])
    D['dsp1_sram_start'] = hex_format(config['DSP1_SRAM']['start'])
    D['shared_sram_start'] = hex_format(config['SHARED_SRAM']['start'])

    D['mcu_mram_length'] = config['MCU_MRAM']['length']
    D['dsp0_mram_length'] = config['DSP0_MRAM']['length']
    D['dsp1_mram_length'] = config['DSP1_MRAM']['length']
    D['mcu_tcm_length'] = config['MCU_TCM']['length']
    D['dsp0_tcm_length'] = config['DSP0_TCM']['length']
    D['dsp1_tcm_length'] = config['DSP1_TCM']['length']
    D['mcu_sram_length'] = config['MCU_SRAM']['length']
    D['dsp0_sram_length'] = config['DSP0_SRAM']['length']
    D['dsp1_sram_length'] = config['DSP1_SRAM']['length']
    D['shared_sram_length'] = config['SHARED_SRAM']['length']

    return memory_header_template.substitute(**D)


def hex_format(n):
    return '0x{:08X}'.format(n)


memory_header_string = '''\
//*****************************************************************************
//
// file am_memory_map.h
//
// brief Memory map include file.
//
// This file is generated by linker_config "generate_link_script.py".
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) {copyrightyear}, Ambiq Micro
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
#define AM_MEM_MCU_MRAM                     ${{mcu_mram_start}}
#define AM_MEM_DSP0_MRAM                    ${{dsp0_mram_start}}
#define AM_MEM_DSP1_MRAM                    ${{dsp1_mram_start}}
#define AM_MEM_MCU_TCM                      ${{mcu_tcm_start}}
#define AM_MEM_DSP0_TCM                     ${{dsp0_tcm_start}}
#define AM_MEM_DSP1_TCM                     ${{dsp1_tcm_start}}
#define AM_MEM_MCU_SRAM                     ${{mcu_sram_start}}
#define AM_MEM_DSP0_SRAM                    ${{dsp0_sram_start}}
#define AM_MEM_DSP1_SRAM                    ${{dsp1_sram_start}}
#define AM_MEM_SHARED_SRAM                  ${{shared_sram_start}}

//*****************************************************************************
//
// Memory block sizes (in bytes)
//
//*****************************************************************************
#define AM_MEM_MCU_MRAM_LENGTH              ${{mcu_mram_length}}
#define AM_MEM_DSP0_MRAM_LENGTH             ${{dsp0_mram_length}}
#define AM_MEM_DSP1_MRAM_LENGTH             ${{dsp1_mram_length}}
#define AM_MEM_MCU_TCM_LENGTH               ${{mcu_tcm_length}}
#define AM_MEM_DSP0_TCM_LENGTH              ${{dsp0_tcm_length}}
#define AM_MEM_DSP1_TCM_LENGTH              ${{dsp1_tcm_length}}
#define AM_MEM_MCU_SRAM_LENGTH              ${{mcu_sram_length}}
#define AM_MEM_DSP0_SRAM_LENGTH             ${{dsp0_sram_length}}
#define AM_MEM_DSP1_SRAM_LENGTH             ${{dsp1_sram_length}}
#define AM_MEM_SHARED_SRAM_LENGTH           ${{shared_sram_length}}

#ifdef __cplusplus
}}
#endif

#endif // AM_MEMORY_MAP_H
'''

currdatetime = datetime.datetime.now()
memory_header_template = string.Template(memory_header_string.format(copyrightyear=currdatetime.year))


if __name__ == '__main__':
    main()
