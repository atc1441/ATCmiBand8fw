//*****************************************************************************
//
//! @file am_ios_boot_handlers.c
//!
//! @brief Boot related functions for the IOS interface.
//!
//! This file contains the main state machine for handling boot commands via
//! the I2C interface.
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

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "am_multi_boot_private.h"

//*****************************************************************************
//
// SPI Slave Configuration
//
//*****************************************************************************
static am_hal_ios_config_t g_sIOSConfig =
{
    // Configure the IOS in SPI mode.
    .ui32InterfaceSelect = AM_HAL_IOS_USE_SPI,

    // Eliminate the "read-only" section, so an external host can use the
    // entire "direct write" section.
    .ui32ROBase = 0x78,

    // Set the FIFO base to the maximum value, making the "direct write"
    // section as big as possible.
    .ui32FIFOBase = 0x78,

    // We don't need any RAM space, so extend the FIFO all the way to the end
    // of the LRAM.
    .ui32RAMBase = 0x100,
};
static uint32_t g_iosIntPin;

static volatile bool g_bIosImageValid = false;

#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
void *g_IOSHandle;

#define     TEST_ACC_INT        1

//*****************************************************************************
//
// GPIO Configuration
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_AM_BOOT_GPIO_IOS_SCL =
{
    .uFuncSel            = AM_HAL_PIN_0_SLSCL,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA,
    .eGPInput            = AM_HAL_GPIO_PIN_INPUT_ENABLE
};

const am_hal_gpio_pincfg_t g_AM_BOOT_GPIO_IOS_SDA =
{
    .uFuncSel            = AM_HAL_PIN_1_SLSDAWIR3,
    .ePullup             = AM_HAL_GPIO_PIN_PULLUP_1_5K,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA,
    .eGPOutcfg           = AM_HAL_GPIO_PIN_OUTCFG_OPENDRAIN
};

const am_hal_gpio_pincfg_t g_AM_BOOT_GPIO_IOS_SCK =
{
    .uFuncSel            = AM_HAL_PIN_0_SLSCK,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA,
    .eGPInput            = AM_HAL_GPIO_PIN_INPUT_ENABLE
};

const am_hal_gpio_pincfg_t g_AM_BOOT_GPIO_IOS_CE =
{
    .uFuncSel            = AM_HAL_PIN_3_SLnCE,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA,
    .eGPInput            = AM_HAL_GPIO_PIN_INPUT_ENABLE,
    .uNCE                = 0,
    .eCEpol              = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW
};

const am_hal_gpio_pincfg_t g_AM_BOOT_GPIO_IOS_MISO =
{
    .uFuncSel            = AM_HAL_PIN_2_SLMISO
};

const am_hal_gpio_pincfg_t g_AM_BOOT_GPIO_IOS_MOSI =
{
    .uFuncSel            = AM_HAL_PIN_1_SLMOSI,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA,
    .eGPInput            = AM_HAL_GPIO_PIN_INPUT_ENABLE
};

const am_hal_gpio_pincfg_t g_AM_HAL_GPIO_INPUT_ENABLE =
{
    .uFuncSel             = AM_HAL_PIN_0_GPIO,
    .eGPInput             = AM_HAL_GPIO_PIN_INPUT_ENABLE,
};

const am_hal_gpio_pincfg_t g_AM_BOOT_GPIO_ENABLE =
{
    .uFuncSel            = 3,
    .eGPOutcfg           = AM_HAL_GPIO_PIN_OUTCFG_PUSHPULL
};

const am_hal_gpio_pincfg_t g_AM_BOOT_GPIO_DISABLE =
{
    .uFuncSel            = 3,
    .eGPOutcfg           = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
};

#endif

//*****************************************************************************
//
//! @brief Multiboot protocol handler for IOS implemented in IOS-ACC ISR
//!
//! This function should be invoked from respective IOS-ACC ISR
//!
//! @return none.
//
//*****************************************************************************
void
am_multiboot_ios_acc_isr_handler(void)
{
    uint32_t ui32Status;
    uint32_t *pui32Packet;

    //
    // Check to see what caused this interrupt, then clear the bit from the
    // interrupt register.
    //
#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
    am_hal_ios_control(g_IOSHandle, AM_HAL_IOS_REQ_ACC_INTGET, &ui32Status);
    am_hal_ios_control(g_IOSHandle, AM_HAL_IOS_REQ_ACC_INTCLR, &ui32Status);
#else
    ui32Status = am_hal_ios_access_int_status_get(false);
    am_hal_ios_access_int_clear(ui32Status);
#endif
    //
    // Set up a pointer for writing 32-bit aligned packets through the IO slave
    // interface.
    //
    pui32Packet = (uint32_t *) am_hal_ios_pui8LRAM;

    if ( ui32Status & AM_HAL_IOS_ACCESS_INT_03 )
    {
#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
        am_hal_gpio_state_write(g_iosIntPin, AM_HAL_GPIO_OUTPUT_SET);
#else
        am_hal_gpio_out_bit_set(g_iosIntPin);
#endif
        //
        // Figure out what to do next based on the packet header.
        //
        switch(pui32Packet[0])
        {
            case AM_BOOTLOADER_NEW_IMAGE:

                //
                // Parse the image packet, and store the result to the global
                // image structure.
                //
                g_bIosImageValid = image_start_packet_read(&g_sImage,
                                    (uint32_t *) am_hal_ios_pui8LRAM);

                //
                // Make sure the image packet had reasonable contents. If it
                // didn't, we need to let the host know.
                //
                if ( g_bIosImageValid )
                {
                    //
                    // Good image; Send back a "READY" packet.
                    //
                    pui32Packet[0] = AM_BOOTLOADER_READY;
#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
                    am_hal_gpio_state_write(g_iosIntPin, AM_HAL_GPIO_OUTPUT_CLEAR);
#else
                    am_hal_gpio_out_bit_clear(g_iosIntPin);
#endif
                }
                else
                {
                    //
                    // Bad image; Send back an error.
                    //
                    pui32Packet[0] = AM_BOOTLOADER_ERROR;
#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
                    am_hal_gpio_state_write(g_iosIntPin, AM_HAL_GPIO_OUTPUT_CLEAR);
#else
                    am_hal_gpio_out_bit_clear(g_iosIntPin);
#endif
                }

                break;

            case AM_BOOTLOADER_SET_OVERRIDE_CMD:
                //
                // Set the override GPIO settings based on the packet
                // information.
                //
                g_sImage.ui32OverrideGPIO = pui32Packet[1];
                g_sImage.ui32OverridePolarity = pui32Packet[2];

                //
                // Send back a "READY" packet.
                //
                pui32Packet[0] = AM_BOOTLOADER_READY;
#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
                am_hal_gpio_state_write(g_iosIntPin, AM_HAL_GPIO_OUTPUT_CLEAR);
#else
                am_hal_gpio_out_bit_clear(g_iosIntPin);
#endif
                break;

            case AM_BOOTLOADER_NEW_PACKET:
                //
                // Only take new packets if our image structure is valid.
                //
                if ( !g_bIosImageValid )
                {
                    pui32Packet[0] = AM_BOOTLOADER_ERROR;
#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
                    am_hal_gpio_state_write(g_iosIntPin, AM_HAL_GPIO_OUTPUT_CLEAR);
#else
                    am_hal_gpio_out_bit_clear(g_iosIntPin);
#endif
                    break;
                }

                //
                // Get the size information from the packet header, and set the src pointer
                // to the beginning of the actual data.
                //
                //
                // Parse the reset of the packet sitting in the IOS LRAM.
                //
                image_data_packet_read((uint8_t *)(am_hal_ios_pui8LRAM + 8),
                    *((uint32_t *) (am_hal_ios_pui8LRAM + 4)));

                //
                // If this packet completed the image...
                //
                if ( g_ui32BytesReceived == g_sImage.ui32NumBytes )
                {
#ifdef MULTIBOOT_SECURE
                    if ( (g_ui32CRC != g_sImage.ui32CRC) || multiboot_secure_verify(&g_sImage.ui32CRC) )
                    {
                        pui32Packet[0] = AM_BOOTLOADER_BAD_CRC;
                    }
#else
                    if ( g_ui32CRC != g_sImage.ui32CRC )
                    {
                        pui32Packet[0] = AM_BOOTLOADER_BAD_CRC;
                    }
#endif
                    else
                    {
                        // Protect (and optionally write if stored in SRAM)
                        // image in flash now as it has been validated now
                        program_image(g_sImage.bEncrypted);
                        // Validate the flash contents of a boot image to make
                        // sure it's safe to run
                        if ( am_bootloader_flash_check(&g_sImage) )
                        {
                            pui32Packet[0] = AM_BOOTLOADER_IMAGE_COMPLETE;
                        }
                        else
                        {
                            pui32Packet[0] = AM_BOOTLOADER_ERROR;
                        }
                    }
                }
                else
                {
                    //
                    // If this wasn't the end of the image, just send back a
                    // "READY" packet.
                    //
                    pui32Packet[0] = AM_BOOTLOADER_READY;
                }
                //
                // Assert the interrupt line so the host knows we have a
                // message.
                //
#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
                am_hal_gpio_state_write(g_iosIntPin, AM_HAL_GPIO_OUTPUT_CLEAR);
#else
                am_hal_gpio_out_bit_clear(g_iosIntPin);
#endif
                break;

            case AM_BOOTLOADER_RESET:
                if ( USE_FLAG_PAGE )
                {
                    //
                    // Write the flag page.
                    //
                    am_bootloader_flag_page_update(&g_sImage, (uint32_t *)FLAG_PAGE_LOCATION);
                }
#ifdef MULTIBOOT_SECURE
                wipe_sram();
#endif

            case AM_BOOTLOADER_RESTART:
                //
                // Perform a software reset.
                //
#if (defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P) || defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B))
                am_hal_reset_control(AM_HAL_RESET_CONTROL_SWPOI, 0);
#else
                am_hal_reset_poi();
#endif

                //
                // Wait for the reset to take effect.
                //
                while (1);

            case AM_BOOTLOADER_BL_VERSION_CMD:
                //
                // Respond with the version number.
                //
                pui32Packet[0] = AM_BOOTLOADER_BL_VERSION;
                pui32Packet[1] = AM_BOOTLOADER_VERSION_NUM;
#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
                am_hal_gpio_state_write(g_iosIntPin, AM_HAL_GPIO_OUTPUT_CLEAR);
#else
                am_hal_gpio_out_bit_clear(g_iosIntPin);
#endif
                break;

            case AM_BOOTLOADER_ACK_CMD:
            case AM_BOOTLOADER_NAK_CMD:
                break;

            default:
                // Error
                pui32Packet[0] = AM_BOOTLOADER_ERROR;
#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
                am_hal_gpio_state_write(g_iosIntPin, AM_HAL_GPIO_OUTPUT_CLEAR);
#else
                am_hal_gpio_out_bit_clear(g_iosIntPin);
#endif
                break;
        }
    }
}

//*****************************************************************************
//
//! @brief Configure the IOS for IOS boot
//!
//! @param interruptPin is the handshake pin to use for interrupting host.
//!
//! This function configures the IOS for bootloader
//!
//! @return none
//
//*****************************************************************************
void
am_multiboot_setup_ios_interface(uint32_t interruptPin)
{
#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
    uint32_t ui32Arg = 0;
    //
    // Check pin 0 to see if we should be using SPI or I2C
    //
    am_hal_gpio_pinconfig(0,  g_AM_HAL_GPIO_INPUT_ENABLE);
    am_hal_gpio_state_read(0, AM_HAL_GPIO_INPUT_READ, &ui32Arg);
    if ( ui32Arg )
    {
        //
        // If the clock line is high, we'll assume I2C.
        //
        am_hal_gpio_pinconfig(0,  g_AM_BOOT_GPIO_IOS_SCL);
        am_hal_gpio_pinconfig(1,  g_AM_BOOT_GPIO_IOS_SDA);

        g_sIOSConfig.ui32InterfaceSelect = (AM_HAL_IOS_USE_I2C |
                                            AM_HAL_IOS_I2C_ADDRESS(I2C_SLAVE_ADDR << 1));
    }
    else
    {
        //
        // If the clock line is low, we'll assume SPI.
        //
        am_hal_gpio_pinconfig(0, g_AM_BOOT_GPIO_IOS_SCK);
        am_hal_gpio_pinconfig(1, g_AM_BOOT_GPIO_IOS_MISO);
        am_hal_gpio_pinconfig(2, g_AM_BOOT_GPIO_IOS_MOSI);
        am_hal_gpio_pinconfig(3, g_AM_BOOT_GPIO_IOS_CE);

        g_sIOSConfig.ui32InterfaceSelect = AM_HAL_IOS_USE_SPI;
    }

    //
    // Configure the IOS interface and LRAM structure.
    //
    am_hal_ios_initialize(0, &g_IOSHandle);
    am_hal_ios_power_ctrl(g_IOSHandle, AM_HAL_SYSCTRL_WAKE, false);
    am_hal_ios_configure(g_IOSHandle, &g_sIOSConfig);
    g_iosIntPin = interruptPin;

    //
    // Make sure the interrupt pin is set up correctly
    //
    am_hal_gpio_state_write(g_iosIntPin, AM_HAL_GPIO_OUTPUT_SET);
    am_hal_gpio_pinconfig(g_iosIntPin, g_AM_BOOT_GPIO_ENABLE);

    //
    // Clear out any IOS register-access interrupts that may be active, and
    // enable interrupts for the registers we're interested in.
    //
#ifdef TEST_ACC_INT
    ui32Arg = AM_HAL_IOS_ACCESS_INT_ALL;
    am_hal_ios_control(g_IOSHandle, AM_HAL_IOS_REQ_ACC_INTCLR, &ui32Arg);
    ui32Arg = AM_HAL_IOS_ACCESS_INT_03;
    am_hal_ios_control(g_IOSHandle, AM_HAL_IOS_REQ_ACC_INTEN, &ui32Arg);
#endif
    am_hal_ios_interrupt_clear(g_IOSHandle, AM_HAL_IOS_INT_ALL);
    am_hal_ios_interrupt_enable(g_IOSHandle, AM_HAL_IOS_INT_FSIZE);

    //
    // Set the bit in the NVIC to accept access interrupts from the IO Slave.
    //
#if AM_CMSIS_REGS
        NVIC_EnableIRQ(IOSLAVEACC_IRQn);
//        NVIC_EnableIRQ(IOSLAVE_IRQn);
#else // AM_CMSIS_REGS
        am_hal_interrupt_enable(AM_HAL_INTERRUPT_IOSACC);
//        am_hal_interrupt_enable(AM_HAL_INTERRUPT_IOSLAVE);
#endif // AM_CMSIS_REGS

    //
    // Notify the host that we're ready to receive data.
    //
    *((uint32_t *) am_hal_ios_pui8LRAM) = AM_BOOTLOADER_READY;
    am_hal_gpio_state_write(g_iosIntPin, AM_HAL_GPIO_OUTPUT_CLEAR);
#else
    //
    // Check pin 0 to see if we should be using SPI or I2C
    //
    am_hal_gpio_pin_config(0, AM_HAL_GPIO_INPUT);
    if ( am_hal_gpio_input_bit_read(0) )
    {
        //
        // If the clock line is high, we'll assume I2C.
        //
        am_hal_gpio_pin_config(0, AM_HAL_PIN_0_SLSCL);
        am_hal_gpio_pin_config(1, AM_HAL_PIN_1_SLSDA);

        g_sIOSConfig.ui32InterfaceSelect = (AM_HAL_IOS_USE_I2C |
                                            AM_HAL_IOS_I2C_ADDRESS(I2C_SLAVE_ADDR << 1));
    }
    else
    {
        //
        // If the clock line is low, we'll assume SPI.
        //
        am_hal_gpio_pin_config(0, AM_HAL_PIN_0_SLSCK);
        am_hal_gpio_pin_config(1, AM_HAL_PIN_1_SLMISO);
        am_hal_gpio_pin_config(2, AM_HAL_PIN_2_SLMOSI);
        am_hal_gpio_pin_config(3, AM_HAL_PIN_3_SLnCE);

        g_sIOSConfig.ui32InterfaceSelect = AM_HAL_IOS_USE_SPI;
    }

    //
    // Configure the IOS interface and LRAM structure.
    //
    am_hal_ios_config(&g_sIOSConfig);
    g_iosIntPin = interruptPin;

    //
    // Make sure the interrupt pin is set up correctly
    //
    am_hal_gpio_out_bit_set(g_iosIntPin);
    am_hal_gpio_pin_config(g_iosIntPin, AM_HAL_PIN_OUTPUT);

    //
    // Clear out any IOS register-access interrupts that may be active, and
    // enable interrupts for the registers we're interested in.
    //
    am_hal_ios_access_int_clear(AM_HAL_IOS_ACCESS_INT_ALL);
    am_hal_ios_access_int_enable(AM_HAL_IOS_ACCESS_INT_03);

    am_hal_ios_int_clear(AM_HAL_IOS_INT_ALL);
    am_hal_ios_int_enable(AM_HAL_IOS_INT_FSIZE);

    //
    // Set the bit in the NVIC to accept access interrupts from the IO Slave.
    //
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_IOSACC);
//    am_hal_interrupt_enable(AM_HAL_INTERRUPT_IOSLAVE);

    //
    // Notify the host that we're ready to receive data.
    //
    *((uint32_t *) am_hal_ios_pui8LRAM) = AM_BOOTLOADER_READY;
    am_hal_gpio_out_bit_clear(g_iosIntPin);
#endif
}

//*****************************************************************************
//
//! @brief Cleanup the IOS
//!
//! This function un-configures the IOS
//!
//! @return none
//
//*****************************************************************************
void
am_multiboot_cleanup_ios_interface(void)
{
#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
    am_hal_gpio_pinconfig(0, g_AM_BOOT_GPIO_DISABLE);
    am_hal_gpio_pinconfig(1, g_AM_BOOT_GPIO_DISABLE);
    am_hal_gpio_pinconfig(2, g_AM_BOOT_GPIO_DISABLE);
    am_hal_gpio_pinconfig(3, g_AM_BOOT_GPIO_DISABLE);
#else
    am_hal_gpio_pin_config(0, AM_HAL_PIN_DISABLE);
    am_hal_gpio_pin_config(1, AM_HAL_PIN_DISABLE);
    am_hal_gpio_pin_config(2, AM_HAL_PIN_DISABLE);
    am_hal_gpio_pin_config(3, AM_HAL_PIN_DISABLE);
#endif
}

#if 0
//*****************************************************************************
//
// IO Slave Main ISR.
//
//*****************************************************************************
void
am_ioslave_ios_isr(void)
{
    uint32_t ui32Status;

#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
    //
    // Check to see what caused this interrupt, then clear the bit from the
    // interrupt register.
    //
    am_hal_ios_interrupt_status_get(g_IOSHandle, false, &ui32Status);
    am_hal_ios_interrupt_clear(g_IOSHandle, ui32Status);

    //
    // Service the I2C slave FIFO if necessary.
    //
    am_hal_ios_interrupt_service(g_IOSHandle, ui32Status);
#else
    //
    // Check to see what caused this interrupt, then clear the bit from the
    // interrupt register.
    //
    ui32Status = am_hal_ios_int_status_get(false);
    am_hal_ios_int_clear(ui32Status);

    //
    // Service the I2C slave FIFO if necessary.
    //
    am_hal_ios_fifo_service(ui32Status);
#endif
}

//*****************************************************************************
//
// IO Slave handshake
//
//*****************************************************************************
bool
am_ioslave_handshake(uint32_t ui32Timeout)
{
    uint32_t i;
    uint32_t *pui32Packet;
    bool bReturnVal = false;

    //
    // Set up a pointer for writing 32-bit aligned packets through the IO slave
    // interface.
    //
    pui32Packet = (uint32_t *) am_hal_ios_pui8LRAM;
#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
    am_hal_gpio_state_write(g_iosIntPin, AM_HAL_GPIO_OUTPUT_SET);
#else
    am_hal_gpio_out_bit_set(g_iosIntPin);
#endif

    for ( i = 0; i < ui32Timeout; i++ )
    {
        if ( pui32Packet[0] == 0x5A )
        {
            bReturnVal = true;

            //
            // Notify the host that we're ready to receive data.
            //
            *((uint32_t *) am_hal_ios_pui8LRAM) = AM_BOOTLOADER_READY;
#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
            am_hal_gpio_state_write(g_iosIntPin, AM_HAL_GPIO_OUTPUT_CLEAR);
#else
            am_hal_gpio_out_bit_clear(g_iosIntPin);
#endif
            break;
        }
        am_util_delay_us(1);
    }
    return bReturnVal;
}
#endif
