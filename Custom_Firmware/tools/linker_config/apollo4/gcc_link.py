from string import Template


def generate_link_script(config):
    D = dict()
    D['ro_base'] = format_hex(config['MCU_MRAM']['start'])
    D['ro_length'] = config['MCU_MRAM']['length']

    D['rw_base'] = format_hex(config['MCU_TCM']['start'])
    D['rw_length'] = config['MCU_TCM']['length']

    D['ram_base'] = format_hex(config['MCU_SRAM']['start'])
    D['ram_length'] = config['MCU_SRAM']['length']

    D['shared_base'] = format_hex(config['SHARED_SRAM']['start'])
    D['shared_length'] = config['SHARED_SRAM']['length']

    D['additional_sections'] = ''

    return link_script_template.substitute(**D)


def format_hex(n):
    return '0x{:08X}'.format(n)


link_script_template = Template('''\
/******************************************************************************
 *
 * linker_script.ld - Linker script for applications using startup_gnu.c
 *
 *****************************************************************************/
ENTRY(Reset_Handler)

MEMORY
{
    MCU_MRAM     (rx)  : ORIGIN = ${ro_base}, LENGTH = ${ro_length}
    MCU_TCM      (rwx) : ORIGIN = ${rw_base}, LENGTH = ${rw_length}
    MCU_SRAM     (rx)  : ORIGIN = ${ram_base}, LENGTH = ${ram_length}
    SHARED_SRAM  (rwx) : ORIGIN = ${shared_base}, LENGTH = ${shared_length}
}

SECTIONS
{
    .text :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        KEEP(*(.patch))
        *(.text)
        *(.text*)
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
        _etext = .;
    } > MCU_MRAM

    /* User stack section initialized by startup code. */
    .stack (NOLOAD):
    {
        . = ALIGN(8);
        *(.stack)
        *(.stack*)
        . = ALIGN(8);
    } > MCU_TCM

    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > MCU_TCM AT>MCU_MRAM

    /* used by startup to initialize data */
    _init_data = LOADADDR(.data);

    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > MCU_TCM

    .mcu_sram :
    {
        . = ALIGN(4);
        *(.mcu_sram)
        . = ALIGN(4);
    } > MCU_SRAM AT>MCU_MRAM

    .shared :
    {
        . = ALIGN(4);
        KEEP(*(.resource_table))
        KEEP(*(.shared))
        . = ALIGN(4);
    } > SHARED_SRAM AT>MCU_MRAM
${additional_sections}    .ARM.attributes 0 : { *(.ARM.attributes) }
}\
''')
