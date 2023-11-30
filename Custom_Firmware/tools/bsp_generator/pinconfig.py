#!/usr/bin/env python3

# *****************************************************************************
#
#    pinconfig.py
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

# *****************************************************************************
#    Imported modules
# *****************************************************************************
import argparse
import os.path
import rsonlite
import apollo4_pinconfig

# *****************************************************************************
#    Templates
# *****************************************************************************
filetemplateC = '''
//*****************************************************************************
//
//  {filename}
//! @file
//!
//! @brief BSP pin configuration definitions.
//!
//! @addtogroup BSP Board Support Package (BSP)
//! @ingroup BSP
//! @{{
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
{pin_defs}
//*****************************************************************************
//
// End Doxygen group.
//! @}}
//
//*****************************************************************************
'''.strip()


filetemplateH = '''
//*****************************************************************************
//
//  {filename}
//! @file
//!
//! @brief BSP pin configuration definitions.
//!
//! @addtogroup BSP Board Support Package (BSP)
//! @addtogroup apollo3_bsp BSP for the Apollo3 EVB.
//! @ingroup BSP
//! @{{
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

#ifndef {headerdef}
#define {headerdef}

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"

#ifdef __cplusplus
extern "C"
{{
#endif
{pin_defs}

#ifdef __cplusplus
}}
#endif

#endif // {headerdef}

//*****************************************************************************
//
// End Doxygen group.
//! @}}
//
//*****************************************************************************
'''.strip()


sectiontemplate = '''

//*****************************************************************************
//
//  {pin_descr}
//
//*****************************************************************************
'''.strip()


# *****************************************************************************
#
# Globals
#
# *****************************************************************************

# *****************************************************************************
#
# Command line support.
#
# *****************************************************************************
def read_arguments():
    parser = argparse.ArgumentParser()

    parser.add_argument('input', help='input src file name')
    parser.add_argument('CorH',  help='C to create C file, H to create H file',
                        choices=['C','H','c','h'])

    return parser.parse_args()

# *****************************************************************************
# parse_input()
# *****************************************************************************
def parse_input(filename):
    '''
    Simple wrapper to pull an rsonlite list from given file.
    '''
    with open(filename) as f:
        data = f.read()
    return rsonlite.loads(data)

# *****************************************************************************
# list_to_dict()
# *****************************************************************************
def list_to_dict(L):
    '''
    rsonlite loads data from the rson files as a very-nested list. This
    function converts that list to a set of nested dictionaries. At each level,
    the dictionary keys correspond to the "headings", and the dictionary values
    are a list of the items under that "heading". Values assigned by
    equals-sign statements are (unfortunately) also converted to (key, list)
    pairs, the same way that subheadings are.
    '''
    if len(L) == 1:
        return L[0]
    else:
        D = dict()
        for x in L:
            if x[0] in D:
                D[x[0]] = D[x[0]] + [list_to_dict(x[1])]
            else:
                D[x[0]] = [list_to_dict(x[1])]
        return D

# *****************************************************************************
# get_val()
# *****************************************************************************
def get_val(name, D):
    '''
    get_val is just a helper to make it more obvious when you are trying to get
    at an equals-sign-assigned value from the rson file dictionary.
    '''
    if name not in D:
        return "ERROR: {} VALUE MISSING".format(name)
    else:
        return D[name][0]

# *****************************************************************************
# get_version()
# *****************************************************************************
def get_version(filename):
    '''
    Given the filename of an 'src' file, this function will return a 'pin'
    object corresponding to the fields described in the src file.
    '''
    # Read in the contents of the src file, and use the rsonlite library to
    # parse them into a list.
    rson_data = parse_input(filename)

    # Convert the list to a dictionary.
    rson_dict = list_to_dict(rson_data)

    if 'pinsrc_ver' in rson_dict:
        # Convert rson list object to int
        ssrcver = rson_dict["pinsrc_ver"][0]
        if ssrcver[0:2].lower() == "0x":
            source_version = int(ssrcver, 16)
        else:
            source_version = int(ssrcver, 10)
    else:
        # If no src file version given, assume version for Apollo3
        source_version = 0x0003

    return source_version

# *****************************************************************************
# get_pinobj()
# *****************************************************************************
def get_pinobj(filename):
    '''
    Given the filename of an 'src' file, this function will return a 'pin'
    object corresponding to the fields described in the src file.
    '''

    # Set the location for the supplementary files (infoblock, base addresses, etc.)
    srcdir = os.path.dirname(filename)

    # Read in the contents of the src file, and use the rsonlite library to
    # parse them into a list.
    rson_data = parse_input(filename)

    # Convert the list to a dictionary.
    rson_dict = list_to_dict(rson_data)

    # Convert the dictionary to a block object, which will create field
    # objects as necessary. Return this object to the caller.
    return pinobj(rson_dict)


# *****************************************************************************
# pinfields
# *****************************************************************************
class pinfields:
    def __init__(self, pindict):

        self.name        = ''
        self.desc        = ''
        self.pinnum      = 0
        self.powersw     = ''
        self.pullup      = ''
        self.func_sel    = ''
        self.drvstrength = 0
        self.intdir      = 0
        self.GPOutcfg    = ''
        self.GPinput     = ''
        self.GPRdZero    = ''
        self.IOMnum      = ''
        self.CEnum       = ''
        self.CEpol       = ''
        self.bIomMSPIn   = ''

        intnotgiven      = 65535
        strnotgiven      = 'none_given'

        if 'name' in pindict:
            self.name = get_val('name', pindict)
        else:
            self.name = strnotgiven

        if 'desc' in pindict:
            self.desc = get_val('desc', pindict)
        else:
            self.desc = strnotgiven

        if 'pinnum' in pindict:
            self.pinnum = int(get_val('pinnum', pindict))
        else:
            self.pinnum = intnotgiven

        if 'powersw' in pindict:
            self.powersw = get_val('powersw', pindict)
        else:
            self.powersw = strnotgiven

        if 'pullup' in pindict:
            self.pullup = get_val('pullup', pindict)
        else:
            self.pullup = strnotgiven

        if 'func_sel' in pindict:
            self.func_sel = get_val('func_sel', pindict)
        else:
            self.func_sel = strnotgiven

        if 'drvstrength' in pindict:
            self.drvstrength = int(get_val('drvstrength', pindict))
        else:
            self.drvstrength = intnotgiven

        if 'intdir' in pindict:
            self.intdir = get_val('intdir', pindict)
        else:
            self.intdir = strnotgiven

        if 'GPOutcfg' in pindict:
            self.GPOutcfg = get_val('GPOutcfg', pindict)
        else:
            self.GPOutcfg = strnotgiven

        if 'GPinput' in pindict:
            self.GPinput = get_val('GPinput', pindict)
        else:
            self.GPinput = strnotgiven

        if 'GPRdZero' in pindict:
            self.GPRdZero = get_val('GPRdZero', pindict)
        else:
            self.GPRdZero = strnotgiven

        if 'IOMnum' in pindict:
            self.IOMnum = get_val('IOMnum', pindict)
        elif 'MSPInum' in pindict:
            self.IOMnum = get_val('MSPInum', pindict)
        else:
            self.IOMnum = strnotgiven

        if 'CEnum' in pindict:
            self.CEnum = get_val('CEnum', pindict)
        else:
            self.CEnum = strnotgiven

        if 'CEpol' in pindict:
            self.CEpol = get_val('CEpol', pindict)
        else:
            self.CEpol = strnotgiven

        if 'bIomMSPIn' in pindict:
            self.bIomMSPIn = get_val('bIomMSPIn', pindict)
        else:
            self.bIomMSPIn = strnotgiven

        #
        # Check for an invalid field in the src file. If so, report it.
        #
        for fld in pindict:
            if  fld != 'name'           and     \
                fld != 'desc'           and     \
                fld != 'pinnum'         and     \
                fld != 'powersw'        and     \
                fld != 'pullup'         and     \
                fld != 'func_sel'       and     \
                fld != 'drvstrength'    and     \
                fld != 'intdir'         and     \
                fld != 'GPOutcfg'       and     \
                fld != 'GPinput'        and     \
                fld != 'GPRdZero'       and     \
                fld != 'IOMnum'         and     \
                fld != 'MSPInum'        and     \
                fld != 'CEnum'          and     \
                fld != 'CEpol'          and     \
                fld != 'bIomMSPIn':
                    print("Invalid field: '%s.%s'" % (self.name, fld))


# *****************************************************************************
# pinobj
# *****************************************************************************
class pinobj:
    def __init__(self, pindict):
        self.name = 'name'
        self.srcver=0x0000
        self.pins = []

        if 'pinsrc_ver' in pindict:
            # Convert rson list object to int
            ssrcver = pindict["pinsrc_ver"][0]
            if ssrcver[0:2].lower() == "0x":
                self.srcver = int(ssrcver,16)
            else:
                self.srcver = int(ssrcver,10)
        else:
            # If no src file version given, assume version for Apollo3
            self.srcver = 0x0003

        # Run through all the pins given in the src file
        if 'pin' in pindict:
            #
            # Run through the 'pin' list.
            #
            for pin in pindict['pin']:
                self.pins.append(pinfields(pin))

# *****************************************************************************
# write_Cfiles()
# *****************************************************************************

def write_Cfiles(pinobj, bCreateC):
    #
    # Initializations
    #
    intnotgiven      = 65535
    strnotgiven      = 'none_given'
    strCfile         = ''
    strHfile         = ''

    for pin in pinobj.pins:
        sectiondesc = pin.name.strip() + ' pin'
        if pin.desc == strnotgiven:
            sectiondesc += '.'
        else:
            sectiondesc += ': ' + pin.desc.strip()
            if sectiondesc[-1] != '.':
                sectiondesc += '.'
        strCfile += '\n'
        strCfile += sectiontemplate.format(pin_descr=sectiondesc)
        strCfile += '\n'
        strHfile += '\n'
        strHfile += sectiontemplate.format(pin_descr=sectiondesc)
        strHfile += '\n'

        if pin.pinnum != intnotgiven:
            strHfile += '#define AM_BSP_GPIO_%-20s' % pin.name  +  '%d\n' % pin.pinnum
            strHfile += 'extern const am_hal_gpio_pincfg_t       g_AM_BSP_GPIO_%s;\n' % pin.name
            #strHfile += '\n'

        strCfile += 'const am_hal_gpio_pincfg_t g_AM_BSP_GPIO_%s =\n' % pin.name
        strCfile += '{\n'
        strCfile += '%-25s' % '    .uFuncSel' + '= %s,\n' % pin.func_sel

        if pin.powersw != strnotgiven:
            if (pin.powersw.lower() == "vdd")       or          \
               (pin.powersw.lower() == "vss")       or          \
               (pin.powersw.lower() == "none"):
                strCfile += '%-25s' % '    .ePowerSw' + '= AM_HAL_GPIO_PIN_POWERSW_%s,\n' % pin.powersw.upper()
            else:
                strCfile += '%-25s' % '    .ePowerSw' + '= %s,\n' % pin.powersw

        if pin.pullup != strnotgiven:
            if (pin.pullup.lower() == "none")       or          \
               (pin.pullup.lower() == "weak")       or          \
               (pin.pullup.lower() == "pulldown")   or          \
               (pin.pullup.lower() == "1_5k")       or          \
               (pin.pullup.lower() == "6k")         or          \
               (pin.pullup.lower() == "12k")        or          \
               (pin.pullup.lower() == "24k"):
                strCfile += '%-25s' % '    .ePullup' + '= AM_HAL_GPIO_PIN_PULLUP_%s,\n' % pin.pullup.upper()
            elif (pin.pullup == "1_5")              or          \
                 (pin.pullup == "6")                or          \
                 (pin.pullup == "12")               or          \
                 (pin.pullup == "24"):
                strCfile += '%-25s' % '    .ePullup' + '= AM_HAL_GPIO_PIN_PULLUP_%sK,\n' % pin.pullup
            else:
                strCfile += '%-25s' % '    .ePullup' + '= %s,\n' % pin.pullup

        if pin.drvstrength != intnotgiven:
            if (pin.drvstrength != 2)   and  (pin.drvstrength != 4)  and    \
               (pin.drvstrength != 8)   and  (pin.drvstrength != 12):
               pin.drvstrength = 2
            strCfile += '%-25s' % '    .eDriveStrength' + '= AM_HAL_GPIO_PIN_DRIVESTRENGTH_%dMA,\n' % pin.drvstrength

        if pin.GPOutcfg != strnotgiven:
            if (pin.GPOutcfg.lower() == "disable")    or        \
               (pin.GPOutcfg.lower() == "pushpull")   or        \
               (pin.GPOutcfg.lower() == "opendrain")  or        \
               (pin.GPOutcfg.lower() == "tristate"):
                strCfile += '%-25s' % '    .eGPOutcfg'  +  '= AM_HAL_GPIO_PIN_OUTCFG_%s,\n' % pin.GPOutcfg.upper()
            else:
                strCfile += '%-25s' % '    .eGPOutcfg' + '= %s,\n' % pin.GPOutcfg

        if pin.GPinput != strnotgiven:
            if (pin.GPinput.lower() == "true"):
                strCfile += '%-25s' % '    .eGPInput'  +  '= AM_HAL_GPIO_PIN_INPUT_ENABLE,\n'
            elif (pin.GPinput.lower() == "false"):
                strCfile += '%-25s' % '    .eGPInput'  +  '= AM_HAL_GPIO_PIN_INPUT_NONE,\n'
            else:
                strCfile += '%-25s' % '    .eGPInput' + '= %s,\n' % pin.GPinput

        if pin.GPRdZero != strnotgiven:
            if (pin.GPRdZero.lower() == "true")     or          \
               (pin.GPRdZero.lower() == "zero"):
                strCfile += '%-25s' % '    .eGPRdZero'  +  '= AM_HAL_GPIO_PIN_RDZERO_ZERO,\n'
            elif (pin.GPRdZero.lower() == "false")  or          \
                 (pin.GPRdZero.lower() == "readpin"):
                strCfile += '%-25s' % '    .eGPRdZero'  +  '= AM_HAL_GPIO_PIN_RDZERO_READPIN,\n'
            else:
                strCfile += '%-25s' % '    .eGPRdZero' + '= %s,\n' % pin.GPRdZero

        if pin.intdir != strnotgiven:
            if (pin.intdir.lower() == "none")       or          \
               (pin.intdir.lower() == "lo2hi")      or          \
               (pin.intdir.lower() == "hi2lo")      or          \
               (pin.intdir.lower() == "both"):
                strCfile += '%-25s' % '    .eIntDir' + '= AM_HAL_GPIO_PIN_INTDIR_%s,\n' % pin.intdir.upper()
            else:
                strCfile += '%-25s' % '    .eIntDir' + '= %s,\n' % pin.intdir

        if (pin.bIomMSPIn != strnotgiven):
            if pinobj.srcver >= 0x0003:
                if ((pin.bIomMSPIn[0:1].lower() != "m") and (pin.bIomMSPIn != "0")):
                    bIom = 1
                else:
                    bIom = 0
                strCfile += '%-25s' % '    .bIomMSPIn' + '= %d,\n' % bIom

        if pin.IOMnum != strnotgiven:
            strCfile += '%-25s' % '    .uIOMnum' + '= %s,\n' % str(pin.IOMnum)

        if pin.CEnum != strnotgiven:
            strCfile += '%-25s' % '    .uNCE' + '= %s,\n' % str(pin.CEnum)

            # Create the define
            strtmp = '#define AM_BSP_%s_CHNL' % (pin.name)
            strHfile += '%-40s' % strtmp  +  '%s\n' % str((pin.CEnum))

        if pin.CEpol != strnotgiven:
            if (pin.CEpol.lower() == "low")         or          \
               (pin.CEpol.lower() == "high"):
                strCfile += '%-25s' % '    .eCEpol' + '= AM_HAL_GPIO_PIN_CEPOL_ACTIVE%s,\n' % pin.CEpol.upper()
            elif (pin.CEpol.lower() == "activelow") or          \
                 (pin.CEpol.lower() == "activehigh"):
                strCfile += '%-25s' % '    .eCEpol' + '= AM_HAL_GPIO_PIN_CEPOL_%s,\n' % pin.CEpol.upper()
            else:
               strCfile += '%-25s' % '    .eCEpol' + '= %s,\n' % pin.CEpol


        # Eliminate the last comma from the last structure member
        strCfile = strCfile[:-2]

        # Terminate the structure
        strCfile += '\n};\n'

    if bCreateC:
        #
        # Develop and print the C file
        #
        S = filetemplateC.format(filename='am_bsp_pins.c', pin_defs=strCfile);
        print(S)
    else:
        #
        # Develop and print the H file
        #
        S = filetemplateH.format(filename='am_bsp_pins.h',
                                 pin_defs=strHfile,
                                 headerdef='AM_BSP_PINS_H')
        print(S)


# *****************************************************************************
#
# main
#
# *****************************************************************************
if __name__ == '__main__':

    #
    # Get arguments.
    #  First  arg is a string for the input .src file.
    #  Second arg must be a C or H.
    #
    args = read_arguments()
    if args.CorH.upper() == 'C':
        bCreateC = True
    else:
        bCreateC = False

    version = get_version(args.input)

    # Redirect the script based on the version number.
    if version == 0x0004 or version == 0x0005:
        apollo4_pinconfig.write_c_files(args.input, version, bCreateC)

    if version & 0xFF == 0x03:
        pinobj = get_pinobj(args.input)

        #
        # pinobj.pins is a list of the pins
        #
        write_Cfiles(pinobj, bCreateC)
