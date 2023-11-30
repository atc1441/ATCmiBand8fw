from string import Template
import textwrap


def generate_link_script(config):

    mapping = dict()
    mapping['ro_base'] = format_number(config['MCU_MRAM']['start'])
    mapping['rw_base'] = format_number(config['MCU_TCM']['start'])
    mapping['sram_base'] = format_number(config['MCU_SRAM']['start'])
    mapping['shared_base'] = format_number(config['SHARED_SRAM']['start'])
    mapping['ro_size'] = format_number(config['MCU_MRAM']['length'])
    mapping['rw_size'] = format_number(config['MCU_TCM']['length'])
    mapping['sram_size'] = format_number(config['MCU_SRAM']['length'])
    mapping['shared_size'] = format_number(config['SHARED_SRAM']['length'])
    mapping['additional_sections'] = generate_sections(config)

    return link_script_template.substitute(**mapping)


def generate_sections(config):
    # If there aren't any custom sections in the config file, we don't need to
    # add anything to the linker scripts.
    if 'custom_sections' not in config:
        return ''
    elif not config['custom_sections']:
        return ''

    L = []
    for mem_section in config['custom_sections']:
        D = dict()
        D['name'] = mem_section['blockname']
        D['start'] = format_number(mem_section['start'])
        D['length'] = format_number(mem_section['length'])
        D['sections'] = '\n'.join('    * ({})'.format(x) for x in mem_section['sections'])

        S = extra_section_template.substitute(**D)
        L.append(textwrap.indent(S, 4 * ' '))

    return '\n' + '\n'.join(L)


def format_number(n):
    return '0x{:08X}'.format(n)


link_script_template = Template('''\
;******************************************************************************
;
; Scatter file for Keil linker configuration.
;
;******************************************************************************
LR_1 ${ro_base}
{
    MCU_MRAM ${ro_base} ${ro_size}
    {
        *.o (RESET, +First)
        * (+RO)
    }

    MCU_TCM ${rw_base} ${rw_size}
    {
        * (+RW, +ZI)
    }

    SHARED_SRAM ${shared_base} ${shared_size}
    {
        * (RESOURCE_TABLE, +First)
        * (SHARED_RW)
    }
${additional_sections}}\
''')

extra_section_template = Template('''\
${name} ${start} ${length}
{
${sections}
}
''')
