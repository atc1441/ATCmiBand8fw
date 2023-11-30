# *****************************************************************************
#
#    iar_link.py
#
#    @brief Generate the IAR link control file.
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

from string import Template


def generate_link_script(config):
    D = dict()
    D['ro_base'] = format_hex(config['MCU_MRAM']['start'])
    D['ro_end'] = format_hex(config['MCU_MRAM']['start'] + config['MCU_MRAM']['length'])

    D['rw_base'] = format_hex(config['MCU_TCM']['start'])
    D['rw_end'] = format_hex(config['MCU_TCM']['start'] + config['MCU_TCM']['length'])

    D['shared_base'] = format_hex(config['SHARED_SRAM']['start'])
    D['shared_end'] = format_hex(config['SHARED_SRAM']['start'] + config['SHARED_SRAM']['length'])

    D['additional_sections'] = ''

    D['stack'] = '2048'

    return link_script_template.substitute(**D)


def format_hex(n):
    return '0x{:08X}'.format(n)


link_script_template = Template('''\
//*****************************************************************************
//
// linker_script.icf
//
// IAR linker Configuration File
//
//*****************************************************************************

//
// Define a memory section that covers the entire 4 GB addressable space of the
// processor. (32-bit can address up to 4GB)
//
define memory mem with size = 4G;

//
// Define regions for the various types of internal memory.
//
define region MCU_MRAM    = mem:[from ${ro_base} to ${ro_end}];
define region MCU_TCM     = mem:[from ${rw_base} to ${rw_end}];
define region SHARED_SRAM = mem:[from ${shared_base} to ${shared_end}];

//
// Define blocks for logical groups of data.
//
define block HEAP with alignment = 0x8, size = 0x00000000 { };
define block CSTACK with alignment = 0x8, size = ${stack} { };

define block FLASHBASE with fixed order
{
    readonly section .intvec,
    readonly section .patch
};

//
// Set section properties.
//
initialize by copy { readwrite };
initialize by copy { section SHARED_RW };
do not initialize { section .noinit };

//
// Place code sections in memory regions.
//
place at start of MCU_MRAM { block FLASHBASE };
place in MCU_MRAM { readonly };
place at start of MCU_TCM { block CSTACK, section .noinit };
place in MCU_TCM { block HEAP, readwrite };
place at start of SHARED_SRAM { section RESOURCE_TABLE };
place in SHARED_SRAM { section SHARED_RW };${additional_sections}
''')
