# *****************************************************************************
#
#    apollo4_pinconfig.py
#
#    @brief Script for generating a BSP pin file.
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

import rsonlite
from string import Template
import re

#******************************************************************************
# Globals
#******************************************************************************
SrcVersion = 0


#******************************************************************************
#
# Function to convert RSON to a nested dictionary.
#
#******************************************************************************
def convert_rson(rson_string):

    def convert_object(rson_object):
        if isinstance(rson_object[0], tuple):
            D = dict()

            for t in rson_object:
                name = t[0]

                if name not in D.keys():
                    D[name] = convert_object(t[1])
                else:
                    if isinstance(D[name], list):
                        D[name].append(convert_object(t[1]))
                    else:
                        D[name] = [D[name]] + [convert_object(t[1])]
            return D
        else:
            return rson_object[0]

    return convert_object(rsonlite.loads(rson_string))


def is_number(N):
    if isinstance(N, int):
        return True
    else:
        try:
            int(N, 0)
            return True
        except ValueError:
            return False


def write_c_files(input_file, src_version, create_c):

    global SrcVersion

    SrcVersion = src_version

    input_object = None
    with open(input_file) as f:
        input_object = convert_rson(f.read())

    if create_c:
        write_c_file(input_object)
    else:
        write_h_file(input_object)


def write_c_file(input_object):

    global SrcVersion

    # Make a list of new dictionaries with all default settings, and then
    # update them with the settings we found in the actual src file.
    #
    # This construct allows us to specify our own default values from within
    # this python file.
    pins = input_object['pin']
    updated_pins = [dict() for pin in pins]

    default_values = dict()
    for option in option_list:
        default_values[option.name] = option.default_value

    for updated_pin, pin in zip(updated_pins, pins):
        updated_pin.update(default_values)
        updated_pin.update(pin)

    pin_structs = []
    for pin in updated_pins:
        pin_options = []
        for option in option_list:

            # Slewrate no longer exists after apollo4
            if SrcVersion >= 0x0005 and option.name == 'slewrate':
                continue

            value = ''
            if option.name == 'func_sel' and is_number(pin[option.name]) == False:
                value = '{}_{}_{}'.format(option.prefix, pin['pinnum'], pin[option.name])
            elif option.name == 'CEnum':
                if pin['CEnum'] == 'UNSET':
                    value = 0
                elif pin['CEnum'].startswith('AM_HAL_GPIO'):
                    value = pin['CEnum']
                elif re.match(r'[0-9]+', pin['CEnum']) == None:
                    value = 'AM_HAL_GPIO_NCE_{}'.format(pin['CEnum'])
                elif 'bIomMSPIn' in pin and pin['bIomMSPIn'] == 'IOM':
                    value = 'AM_HAL_GPIO_NCE_IOM{}CE{}'.format(pin['IOMnum'], pin['CEnum'])
                elif 'bIomMSPIn' in pin and pin['bIomMSPIn'] == 'MSPI':
                    value = 'AM_HAL_GPIO_NCE_MSPI{}CEN{}'.format(pin['MSPInum'], pin['CEnum'])
                else:
                    raise PinConfigError('Unknown CEnum: {}'.format(pin['CEnum']))
            elif option.name == 'drvstrength':
                if pin['drvstrength'].upper() in drvstrengthxlate:
                    # Translate the deprecated designator to the new version.
                    drvxlate = drvstrengthxlate[pin['drvstrength'].upper()]
                    value = '{}_{}'.format(option.prefix, drvxlate)
                else:
                    value = '{}_{}'.format(option.prefix, pin[option.name])
            elif is_number(pin[option.name]):
                value = pin[option.name]
            elif option.name == 'pullup' and pin[option.name]=='pulldown':
                value = 'AM_HAL_GPIO_PIN_PULLDOWN_50K'
            else:
                value = '{}_{}'.format(option.prefix, pin[option.name])

            # Let's don't upper-case the function names as there may be some
            # functions that purposely contain lower-case letters.
            if 0 and isinstance(value, str):
                value = value.upper()

            pin_mapping = {
                'c_name': '{:20}'.format(option.c_name),
                'value': value,
            }

            pin_options.append(pin_option_template.substitute(**pin_mapping))

        struct_mapping = {
            'name': pin['name'],
            'pinnum': pin['pinnum'],
            'desc': ' - ' + pin['desc'] if 'desc' in pin else '',
            'pin_options': '\n'.join(pin_options),
        }

        pin_structs.append(pin_struct_template.substitute(**struct_mapping))

    print(c_template.substitute(pin_structs='\n\n'.join(pin_structs)))


def write_h_file(input_object):

    # Make a list of new dictionaries with all default settings, and then
    # update them with the settings we found in the actual src file.
    #
    # This construct allows us to specify our own default values from within
    # this python file.
    pins = input_object['pin']
    updated_pins = [dict() for pin in pins]

    default_values = dict()
    for option in option_list:
        default_values[option.name] = option.default_value

    for updated_pin, pin in zip(updated_pins, pins):
        updated_pin.update(default_values)
        updated_pin.update(pin)

    pin_defines = []
    for pin in updated_pins:

        if pin['CEnum'] != 'UNSET' and re.match(r'.*[0-9]', pin['CEnum']):
            defname = 'AM_BSP_{}_CHNL'.format(pin['name'])
            cenum = pin['CEnum'][-1]
            extra_defines = '#define {:35} {}\n'.format(defname, cenum)
        else:
            extra_defines = ''

        mapping = {
            'name': pin['name'],
            'desc': pin['desc'],
            'define': '#define AM_BSP_GPIO_{:23} {}'.format(pin['name'], pin['pinnum']),
            'extra_defines': extra_defines
        }

        pin_defines.append(h_define_template.substitute(**mapping).strip())

    print(h_template.substitute(pin_defines='\n\n'.join(pin_defines)))


#******************************************************************************
#
# Templates for C files.
#
#******************************************************************************
c_template = Template('''\
//*****************************************************************************
//
//  am_bsp_pins.c
//! @file
//!
//! @brief BSP pin configuration definitions.
//!
//! @addtogroup BSP Board Support Package (BSP)
//! @addtogroup apollo3_evb_bsp BSP for the Apollo3 Engineering Board
//! @ingroup BSP
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2023, Ambiq Micro, Inc.
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
// This is part of revision release_sdk_4_4_1-7498c7b770 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include "am_bsp.h"

${pin_structs}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************\
''')

pin_struct_template = Template('''\
//*****************************************************************************
//
// ${name} (${pinnum})${desc}
//
//*****************************************************************************
am_hal_gpio_pincfg_t g_AM_BSP_GPIO_${name} =
{
${pin_options}
};\
''')

pin_option_template = Template('''\
    .GP.cfg_b.${c_name} = ${value},\
''')

#******************************************************************************
#
# Templates for H files.
#
#******************************************************************************
h_template = Template('''\
//*****************************************************************************
//
//  am_bsp_pins.h
//! @file
//!
//! @brief BSP pin configuration definitions.
//!
//! @addtogroup BSP Board Support Package (BSP)
//! @ingroup BSP
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2023, Ambiq Micro, Inc.
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
// This is part of revision release_sdk_4_4_1-7498c7b770 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#ifndef AM_BSP_PINS_H
#define AM_BSP_PINS_H

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"

#ifdef __cplusplus
extern "C"
{
#endif

${pin_defines}

#ifdef __cplusplus
}
#endif

#endif // AM_BSP_PINS_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************\
''')

h_define_template = Template('''\
//*****************************************************************************
//
// ${name} pin: ${desc}
//
//*****************************************************************************
${define}
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_${name};
${extra_defines}
''')


class GpioOption:
    def __init__(self, name, c_name, default_value, prefix):
        self.name = name
        self.c_name = c_name
        self.default_value = default_value
        self.prefix = prefix


# GPIO options
#
# name, C name, default, prefix
option_list = [
    GpioOption('func_sel',    'uFuncSel',       3,           'AM_HAL_PIN'),
    GpioOption('GPinput',     'eGPInput',       'NONE',      'AM_HAL_GPIO_PIN_INPUT'),
    GpioOption('GPRdZero',    'eGPRdZero',      'READPIN',   'AM_HAL_GPIO_PIN_RDZERO'),
    GpioOption('intdir',      'eIntDir',        'NONE',      'AM_HAL_GPIO_PIN_INTDIR'),
    GpioOption('GPOutcfg',    'eGPOutCfg',      'DISABLE',   'AM_HAL_GPIO_PIN_OUTCFG'),
    # GpioOption('drvstrength', 'eDriveStrength', '12MA',      'AM_HAL_GPIO_PIN_DRIVESTRENGTH'),
    GpioOption('drvstrength', 'eDriveStrength', '0P1X',      'AM_HAL_GPIO_PIN_DRIVESTRENGTH'),
    GpioOption('slewrate',    'uSlewRate',      0,           ''),
    GpioOption('pullup',      'ePullup',        'NONE',      'AM_HAL_GPIO_PIN_PULLUP'),
    GpioOption('CEnum',       'uNCE',           'UNSET',     ''),
    GpioOption('CEpol',       'eCEpol',         'ACTIVELOW', 'AM_HAL_GPIO_PIN_CEPOL'),
    GpioOption('rsvd_0',      'uRsvd_0',        0,           ''),
    GpioOption('powersw',     'ePowerSw',       'NONE',      'AM_HAL_GPIO_PIN_POWERSW'),
    GpioOption('forceinpen',  'eForceInputEn',  'NONE',      'AM_HAL_GPIO_PIN_FORCEEN'),
    GpioOption('forceouten',  'eForceOutputEn', 'NONE',      'AM_HAL_GPIO_PIN_FORCEEN'),
    GpioOption('rsvd_1',      'uRsvd_1',        0,           ''),
]


drvstrengthxlate = {
  "12MA": "0P1X",
  "16MA": "0P5X",
}

class PinConfigError(Exception):
    pass
