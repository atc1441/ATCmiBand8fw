from string import Template


def generate_link_script(config):
    D = dict()
    D['ro_base'] = format_hex(config['MCU_MRAM']['start'])
    D['ro_end'] = format_hex(config['MCU_MRAM']['start'] + config['MCU_MRAM']['length'])

    D['rw_base'] = format_hex(config['MCU_TCM']['start'])
    D['rw_end'] = format_hex(config['MCU_TCM']['start'] + config['MCU_TCM']['length'])

    D['ram_base'] = format_hex(config['MCU_SRAM']['start'])
    D['ram_end'] = format_hex(config['MCU_SRAM']['start'] + config['MCU_SRAM']['length'])

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
define region MCU_SRAM    = mem:[from ${ram_base} to ${ram_end}];
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
