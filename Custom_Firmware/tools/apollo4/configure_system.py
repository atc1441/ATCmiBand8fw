# *****************************************************************************
#
#    configure_system.py
#
#    @brief Generate resource files for applications.
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
import datetime
from string import Template

resource_master_list = [
    'GPIO0', 'GPIO1', 'GPIO2', 'GPIO3', 'GPIO4', 'GPIO5', 'GPIO6', 'GPIO7',
    'GPIO8', 'GPIO9', 'GPIO10', 'GPIO11', 'GPIO12', 'GPIO13', 'GPIO14',
    'GPIO15', 'GPIO16', 'GPIO17', 'GPIO18', 'GPIO19', 'GPIO20', 'GPIO21',
    'GPIO22', 'GPIO23', 'GPIO24', 'GPIO25', 'GPIO26', 'GPIO27', 'GPIO28',
    'GPIO29', 'GPIO30', 'GPIO31', 'GPIO32', 'GPIO33', 'GPIO34', 'GPIO35',
    'GPIO36', 'GPIO37', 'GPIO38', 'GPIO39', 'GPIO40', 'GPIO41', 'GPIO42',
    'GPIO43', 'GPIO44', 'GPIO45', 'GPIO46', 'GPIO47', 'GPIO48', 'GPIO49',
    'GPIO50', 'GPIO51', 'GPIO52', 'GPIO53', 'GPIO54', 'GPIO55', 'GPIO56',
    'GPIO57', 'GPIO58', 'GPIO59', 'GPIO60', 'GPIO61', 'GPIO62', 'GPIO63',

    'TIMER0', 'TIMER1', 'TIMER2', 'TIMER3', 'TIMER4', 'TIMER5', 'TIMER6',
    'TIMER7', 'TIMER8', 'TIMER9', 'TIMER10', 'TIMER11', 'TIMER12', 'TIMER13',
    'TIMER14', 'TIMER15',

    'PDM',

    'IOM',

    'UART0', 'UART1',
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('filename')
    parser.add_argument('-o', dest='output')

    args = parser.parse_args()

    S = None
    with open(args.filename) as f:
        S = f.read()

    config = yaml.load(S, Loader=yaml.FullLoader)

    template_key = dict()

    mcu_words = get_resource_words(config, 'MCU')
    dsp0_words = get_resource_words(config, 'DSP0')
    dsp1_words = get_resource_words(config, 'DSP1')
    shared_words = get_shared_resources(config, ['MCU', 'DSP0', 'DSP1'])
    currdatetime = datetime.datetime.now()

    template_key['mcu_words'] = mcu_words
    template_key['dsp0_words'] = dsp0_words
    template_key['dsp1_words'] = dsp1_words
    template_key['shared_words'] = shared_words
    template_key['copyrightyear'] = currdatetime.year

    if (args.output):
        # Specify the newline parameter to output with Unix line endings.
        with open(args.output, 'w', newline='\n') as output_file:
            output_file.write(resource_file_template.substitute(**template_key))
    else:
        print(resource_file_template.substitute(**template_key))


def get_resource_bit(resource):
    """Find the word and bit that we need to modify for the named resource."""
    if resource not in resource_master_list:
        raise ResourceMissing(resource)

    n = resource_master_list.index(resource)
    return (n >> 5, 1 << (n & 0x1F))


def get_resource_words(config, core):
    resource_array = [0 for x in range(0, len(resource_master_list), 32)]

    if 'Resources' in config:
        if config['Resources'][core]:
            for r in config['Resources'][core]:
                word_index, resource_bit = get_resource_bit(r)
                resource_array[word_index] = resource_array[word_index] | resource_bit

    return ',\n    '.join('0x{:08X}'.format(x) for x in resource_array)


def get_shared_resources(config, cores):

    access_list = []
    if 'Resources' in config:
        for core in cores:
            if config['Resources'][core]:
                access_list.extend(config['Resources'][core])

    shared_resources = [0 for x in range(0, len(resource_master_list), 32)]

    for resource in resource_master_list:
        if access_list.count(resource) > 1:
            word_index, bit = get_resource_bit(resource)
            shared_resources[word_index] = shared_resources[word_index] | bit

    return ',\n    '.join('0x{:08X}'.format(x) for x in shared_resources)


resource_file_template = Template('''\
//*****************************************************************************
//
// file am_resources.c
//
// brief This generated file contains a list of resources and owners.
//
// This file is generated by "configure_system.py".
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) ${copyrightyear}, Ambiq Micro, Inc.
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

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"

//*****************************************************************************
//
// MCU Resources.
//
//*****************************************************************************
AM_SHARED_RW uint32_t ui32MCUClaimed[AM_HAL_ACCESS_STRUCT_SIZE];
AM_USED const uint32_t ui32MCUAllowed[AM_HAL_ACCESS_STRUCT_SIZE] =
{
    ${mcu_words}
};

//*****************************************************************************
//
// DSP0 Resources.
//
//*****************************************************************************
AM_SHARED_RW uint32_t ui32DSP0Claimed[AM_HAL_ACCESS_STRUCT_SIZE];
AM_USED const uint32_t ui32DSP0Allowed[AM_HAL_ACCESS_STRUCT_SIZE] =
{
    ${dsp0_words}
};

//*****************************************************************************
//
// DSP1 Resources.
//
//*****************************************************************************
AM_SHARED_RW uint32_t ui32DSP1Claimed[AM_HAL_ACCESS_STRUCT_SIZE];
AM_USED const uint32_t ui32DSP1Allowed[AM_HAL_ACCESS_STRUCT_SIZE] =
{
    ${dsp1_words}
};

//*****************************************************************************
//
// List of shared resources.
//
//*****************************************************************************
AM_USED const uint32_t ui32SharedAccess[AM_HAL_ACCESS_STRUCT_SIZE] =
{
    ${shared_words}
};

//*****************************************************************************
//
// The final resource structure to be passed on to the HAL
//
//*****************************************************************************
AM_USED const am_hal_access_t sGlobalAccess =
{
    ui32SharedAccess,
    ui32MCUAllowed,
    ui32DSP0Allowed,
    ui32DSP1Allowed,
    ui32MCUClaimed,
    ui32DSP0Claimed,
    ui32DSP1Claimed,
};

AM_RESOURCE_TABLE const am_hal_access_t *psGlobalAccessPtr = &sGlobalAccess;

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
''')


class SysConfigError(Exception):
    pass


class ResourceMissing(SysConfigError):
    def __init__(self, resource):
        self.resource = resource
        self.message = '"{}" is not a valid resource for Apollo4.'.format(resource)


if __name__ == '__main__':
    try:
        main()
    except SysConfigError as e:
        print('ERROR: {}'.format(e.message))
