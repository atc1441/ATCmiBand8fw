import argparse
import apollo3p
import os
import apollo4

DEFAULT_TOOLCHAINS = ['iar', 'keil', 'gcc', 'keil6']

linker_generators = {
    'apollo3p': apollo3p.generate_files,
    'apollo4': apollo4.generate_files,
    #
    # Removed for SDK4.4.0
    #  
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('config_file')
    parser.add_argument('-i', dest='iar', action='store_true')
    parser.add_argument('-k', dest='keil', action='store_true')
    parser.add_argument('-s', dest='keil6', action='store_true')
    parser.add_argument('-g', dest='gcc', action='store_true')
    parser.add_argument('-p', dest='part')

    args = parser.parse_args()

    toolchains = []

    if args.iar:
        toolchains.append('iar')
    if args.keil:
        toolchains.append('keil')
    if args.keil6:
        toolchains.append('keil6')
    if args.gcc:
        toolchains.append('gcc')

    if toolchains == []:
        #
        # No command line arguments were specified.
        # Determine which toolchains need a linker control file based on whether
        #  a directory already exists for that toolchain.
        #
        for toolnm in DEFAULT_TOOLCHAINS:
            if os.path.isdir(toolnm):
                toolchains.append(toolnm)
        if toolchains == []:
            toolchains = DEFAULT_TOOLCHAINS

    if args.part in linker_generators:
        print('Building linker scripts for {} and toolchains {}.'.format(args.part, toolchains))
        linker_generators[args.part](args.config_file, toolchains)
    else:
        print("Can't generate linker scripts for {}. (No generator functions found)".format(args.part))


if __name__ == '__main__':
    main()
