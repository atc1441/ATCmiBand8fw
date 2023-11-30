README.txt

Overview:

This is an adaptation of the TrustZone CryptoCell-312 runtime library from
ARM. The original source code can be found at:

https://github.com/ARM-software/cryptocell-312-runtime

The mbedTLS and cc312 libraries work together to provide implementations of
common cryptographic functions supported by the ARM cryptocell hardware,
using a common API provided by mbedTLS.

The version of cc312 provided here has been modified in a few ways to help
facilitate integration with systems using Ambiq Apollo4 devices.
Specifically we have made the following changes:

    - Added configuration options in the cc312 build scripts to support Apollo4
    - Added mbedTLS configuration headers to accomodate Apollo4 ports of
      cc312 and mbedTLS for supported operating systems.
    - Added a new script to assist with building the cc312 and mbedTLS
      library in known configurations.
    - Several small tweaks to the HAL/PAL porting layers to allow for easier
      replacement of C stdlib functions.

Integration with existing code:

To access the cc312 and mbedTLS functionality from your project, you must add
the following libraries and include paths.

    Required libraries:
    - mbedtls/library/libmbedtls.a: mbedTLS functions
    - mbedtls/library/libmbedcrypto.a: Abstraction layer mapping mbedTLS to cc312
    - host/lib/libcc_312.a: Hardware-supported crypto functions.
    - host/lib/libpal_X.a (where X is a supported OS option): Hardware and
      platform abstration layers (HAL and PAL) mapping cc312 functions to
      hardware operations.

    Required include paths:
    - host/include (cc312 includes)
    - mbedtls/include (mbedTLS includes)
    - shared/include/pal (PAL includes)
    - shared/include/pal/X where "X" is a supported OS (Platform-specific includes)

Building libraries for your environment:

Several of the above libraries must be specically compiled to function within
a particular operating system or device. Both mbedTLS and cc312 provide build
scripts to help make this porting process easier. The standard distribution
provides build options for Linux, FreeRTOS, and baremetal build
configurations for a small number of supported devices. Ambiq has
specifically added configurations for Apollo4 for supported operating systems.

Ambiq has also added additional scripts to help ensure that all of the
appropriate build options are set correctly.

Here are the instructions for rebuilding cc312 + mbedTLS for each option
using GCC and a bash or bash-like environment.

    Bare metal:
        1. Clone the public cryptocell-312 repository (see address above) to a
           directory parallel to this one.

           For example (from the third_party directory):

           git clone https://github.com/ARM-software/cryptocell-312-runtime.git

        2. Take the ambiq-changes-r1p4.patch file from this directory, and apply
           it to the newly cloned crypto repository. This patch should be
           current as of version "update-cc110-bu-00000-r1p4".

           Example (from cryptocell folder): patch -p1 < ../crypto/ambiq-changes-r1p4.patch

        3. Run "source set_env_for_gcc.sh" (Ambiq script to set environment
           variables for the next step.)

        4. ./prepare_mbedtls.sh clone

        5. ./prepare_mbedtls.sh lib

        6. make -C host/src ARM_CPU=$ARM_CPU TEE_OS=$TEE_OS

Note: If you would like to rebuild the crypto examples in AmbiqSuite using the
new library built with this process, you will need to make sure the paths in
the example point to the crypto library and header files you created. You can
do this either by moving your new cryptocell directory to "third_party/crypto"
(deleting or archiving what was there before) or modifying the Makefiles and/or
project files in the example directory to replace all instances of
"third_party/crypto" with the path to your new directory.
