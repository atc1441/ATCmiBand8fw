//*****************************************************************************
//
//! @file am_bootloader.c
//!
//! @brief
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
#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"
#include "am_bootloader.h"

#ifdef BOOTLOADER_DEBUG
#include "am_util_stdio.h"
#include "am_util_delay.h"
// might need va_list someday, but not so far today.
#define DPRINTF(x) am_util_stdio_printf x
#else
#define DPRINTF(x)
#endif

#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
const am_hal_gpio_pincfg_t g_AM_HAL_GPIO_INPUT_ENABLE =
{
  .uFuncSel             = AM_HAL_PIN_0_GPIO,
  .eGPInput             = AM_HAL_GPIO_PIN_INPUT_ENABLE,
};

const am_hal_gpio_pincfg_t g_AM_HAL_GPIO_INPUT_DISABLE =
{
  .uFuncSel             = AM_HAL_PIN_0_GPIO,
  .eGPInput             = AM_HAL_GPIO_PIN_INPUT_NONE,
};
#endif

//*****************************************************************************
//
// Forward declarations.
//
//*****************************************************************************
#if defined(__GNUC__)  || defined(keil6)
void __attribute__((naked)) am_bootloader_encrypted_image_run(am_bootloader_image_t *psImage);
void __attribute__((naked)) am_bootloader_clear_image_run(am_bootloader_image_t *psImage);
#endif

#ifdef  keil
__asm void am_bootloader_encrypted_image_run(am_bootloader_image_t *psImage);
__asm void am_bootloader_clear_image_run(am_bootloader_image_t *psImage);
#endif

#ifdef iar
void am_bootloader_encrypted_image_run(am_bootloader_image_t *psImage);
void am_bootloader_clear_image_run(am_bootloader_image_t *psImage);
#endif

//*****************************************************************************
//
// CRC-32 table
// Polynomial = 0x1EDC6F41 (also listed as CRC-32C or CRC-32/4)
//
// This polynomial should catch all errors up to 4 bits for image sizes under
// about 255MB (which easily covers anything we can actually fit in flash), and
// it has a reasonably high probablility of catching bigger errors.
//
// See http://users.ece.cmu.edu/~koopman/crc for more information.
//
//*****************************************************************************
#define CRC32_POLYNOMIAL                    0x1EDC6F41
const static uint32_t g_pui32CRC32Table[256] =
{
    0x00000000, 0x1EDC6F41, 0x3DB8DE82, 0x2364B1C3,
    0x7B71BD04, 0x65ADD245, 0x46C96386, 0x58150CC7,
    0xF6E37A08, 0xE83F1549, 0xCB5BA48A, 0xD587CBCB,
    0x8D92C70C, 0x934EA84D, 0xB02A198E, 0xAEF676CF,
    0xF31A9B51, 0xEDC6F410, 0xCEA245D3, 0xD07E2A92,
    0x886B2655, 0x96B74914, 0xB5D3F8D7, 0xAB0F9796,
    0x05F9E159, 0x1B258E18, 0x38413FDB, 0x269D509A,
    0x7E885C5D, 0x6054331C, 0x433082DF, 0x5DECED9E,
    0xF8E959E3, 0xE63536A2, 0xC5518761, 0xDB8DE820,
    0x8398E4E7, 0x9D448BA6, 0xBE203A65, 0xA0FC5524,
    0x0E0A23EB, 0x10D64CAA, 0x33B2FD69, 0x2D6E9228,
    0x757B9EEF, 0x6BA7F1AE, 0x48C3406D, 0x561F2F2C,
    0x0BF3C2B2, 0x152FADF3, 0x364B1C30, 0x28977371,
    0x70827FB6, 0x6E5E10F7, 0x4D3AA134, 0x53E6CE75,
    0xFD10B8BA, 0xE3CCD7FB, 0xC0A86638, 0xDE740979,
    0x866105BE, 0x98BD6AFF, 0xBBD9DB3C, 0xA505B47D,
    0xEF0EDC87, 0xF1D2B3C6, 0xD2B60205, 0xCC6A6D44,
    0x947F6183, 0x8AA30EC2, 0xA9C7BF01, 0xB71BD040,
    0x19EDA68F, 0x0731C9CE, 0x2455780D, 0x3A89174C,
    0x629C1B8B, 0x7C4074CA, 0x5F24C509, 0x41F8AA48,
    0x1C1447D6, 0x02C82897, 0x21AC9954, 0x3F70F615,
    0x6765FAD2, 0x79B99593, 0x5ADD2450, 0x44014B11,
    0xEAF73DDE, 0xF42B529F, 0xD74FE35C, 0xC9938C1D,
    0x918680DA, 0x8F5AEF9B, 0xAC3E5E58, 0xB2E23119,
    0x17E78564, 0x093BEA25, 0x2A5F5BE6, 0x348334A7,
    0x6C963860, 0x724A5721, 0x512EE6E2, 0x4FF289A3,
    0xE104FF6C, 0xFFD8902D, 0xDCBC21EE, 0xC2604EAF,
    0x9A754268, 0x84A92D29, 0xA7CD9CEA, 0xB911F3AB,
    0xE4FD1E35, 0xFA217174, 0xD945C0B7, 0xC799AFF6,
    0x9F8CA331, 0x8150CC70, 0xA2347DB3, 0xBCE812F2,
    0x121E643D, 0x0CC20B7C, 0x2FA6BABF, 0x317AD5FE,
    0x696FD939, 0x77B3B678, 0x54D707BB, 0x4A0B68FA,
    0xC0C1D64F, 0xDE1DB90E, 0xFD7908CD, 0xE3A5678C,
    0xBBB06B4B, 0xA56C040A, 0x8608B5C9, 0x98D4DA88,
    0x3622AC47, 0x28FEC306, 0x0B9A72C5, 0x15461D84,
    0x4D531143, 0x538F7E02, 0x70EBCFC1, 0x6E37A080,
    0x33DB4D1E, 0x2D07225F, 0x0E63939C, 0x10BFFCDD,
    0x48AAF01A, 0x56769F5B, 0x75122E98, 0x6BCE41D9,
    0xC5383716, 0xDBE45857, 0xF880E994, 0xE65C86D5,
    0xBE498A12, 0xA095E553, 0x83F15490, 0x9D2D3BD1,
    0x38288FAC, 0x26F4E0ED, 0x0590512E, 0x1B4C3E6F,
    0x435932A8, 0x5D855DE9, 0x7EE1EC2A, 0x603D836B,
    0xCECBF5A4, 0xD0179AE5, 0xF3732B26, 0xEDAF4467,
    0xB5BA48A0, 0xAB6627E1, 0x88029622, 0x96DEF963,
    0xCB3214FD, 0xD5EE7BBC, 0xF68ACA7F, 0xE856A53E,
    0xB043A9F9, 0xAE9FC6B8, 0x8DFB777B, 0x9327183A,
    0x3DD16EF5, 0x230D01B4, 0x0069B077, 0x1EB5DF36,
    0x46A0D3F1, 0x587CBCB0, 0x7B180D73, 0x65C46232,
    0x2FCF0AC8, 0x31136589, 0x1277D44A, 0x0CABBB0B,
    0x54BEB7CC, 0x4A62D88D, 0x6906694E, 0x77DA060F,
    0xD92C70C0, 0xC7F01F81, 0xE494AE42, 0xFA48C103,
    0xA25DCDC4, 0xBC81A285, 0x9FE51346, 0x81397C07,
    0xDCD59199, 0xC209FED8, 0xE16D4F1B, 0xFFB1205A,
    0xA7A42C9D, 0xB97843DC, 0x9A1CF21F, 0x84C09D5E,
    0x2A36EB91, 0x34EA84D0, 0x178E3513, 0x09525A52,
    0x51475695, 0x4F9B39D4, 0x6CFF8817, 0x7223E756,
    0xD726532B, 0xC9FA3C6A, 0xEA9E8DA9, 0xF442E2E8,
    0xAC57EE2F, 0xB28B816E, 0x91EF30AD, 0x8F335FEC,
    0x21C52923, 0x3F194662, 0x1C7DF7A1, 0x02A198E0,
    0x5AB49427, 0x4468FB66, 0x670C4AA5, 0x79D025E4,
    0x243CC87A, 0x3AE0A73B, 0x198416F8, 0x075879B9,
    0x5F4D757E, 0x41911A3F, 0x62F5ABFC, 0x7C29C4BD,
    0xD2DFB272, 0xCC03DD33, 0xEF676CF0, 0xF1BB03B1,
    0xA9AE0F76, 0xB7726037, 0x9416D1F4, 0x8ACABEB5
};

//*****************************************************************************
//
//! @brief CRC-32 implementation for the boot loader.
//!
//! @param pvData - Pointer to the data to be checked.
//! @param ui32NumBytes - Number of bytes to check.
//!
//! This function performs a CRC-32 on the input data and returns the 32-bit
//! result. This version does not use a table, so it has a smaller code
//! footprint.
//!
//! @return 32-bit CRC value.
//
//*****************************************************************************
uint32_t
am_bootloader_crc32(const void *pvData, uint32_t ui32NumBytes)
{
    uint32_t ui32CRC, i, j;
    uint8_t *pui8Data;

    ui32CRC = 0;
    pui8Data = (uint8_t *) pvData;

    for ( i = 0; i < ui32NumBytes; i++ )
    {
        ui32CRC ^= pui8Data[i] << 24;

        for ( j = 0; j < 8; j++ )
        {
            ui32CRC = (ui32CRC & 0x80000000 ?
                       ((ui32CRC << 1) ^ CRC32_POLYNOMIAL):
                       (ui32CRC << 1));
        }
    }

    return ui32CRC;
}

//*****************************************************************************
//
//! @brief Faster CRC-32 implementation for the boot loader.
//!
//! @param pvData - Pointer to the data to be checked.
//! @param ui32NumBytes - Number of bytes to check.
//!
//! This function performs a CRC-32 on the input data and returns the 32-bit
//! result. This version uses a 256-entry lookup table to speed up the
//! computation of the result.
//!
//! @return 32-bit CRC value.
//
//*****************************************************************************
uint32_t
am_bootloader_fast_crc32(const void *pvData, uint32_t ui32NumBytes)
{
    uint32_t ui32CRC, ui32CRCIndex, i;
    uint8_t *pui8Data;

    ui32CRC = 0;
    pui8Data = (uint8_t *) pvData;

    for (i = 0; i < ui32NumBytes; i++ )
    {
        ui32CRCIndex = pui8Data[i] ^ (ui32CRC >> 24);
        ui32CRC = (ui32CRC << 8) ^ g_pui32CRC32Table[ui32CRCIndex];
    }

    return ui32CRC;
}

//*****************************************************************************
//
//! @brief CRC-32 implementation allowing multiple partial images.
//!
//! @param pvData - Pointer to the data to be checked.
//! @param ui32NumBytes - Number of bytes to check.
//! @param pui32CRC - Location to store the partial CRC32 result.
//!
//! This function performs a CRC-32 on the input data and returns the 32-bit
//! result. This version uses a 256-entry lookup table to speed up the
//! computation of the result. The result of the CRC32 is stored in the
//! location given by the caller. This allows the caller to keep a "running"
//! CRC for individual chunks of an image.
//!
//! @return 32-bit CRC value.
//
//*****************************************************************************
void
am_bootloader_partial_crc32(const void *pvData, uint32_t ui32NumBytes,
                            uint32_t *pui32CRC)
{
    uint32_t ui32CRCIndex, i;
    uint8_t *pui8Data;

    uint32_t ui32TempCRC = *pui32CRC;

    pui8Data = (uint8_t *) pvData;

    for ( i = 0; i < ui32NumBytes; i++ )
    {
        ui32CRCIndex = pui8Data[i] ^ (ui32TempCRC >> 24);
        ui32TempCRC = (ui32TempCRC << 8) ^ g_pui32CRC32Table[ui32CRCIndex];
    }

    *pui32CRC = ui32TempCRC;
}

//*****************************************************************************
//
//! @brief Check the flash contents of a boot image to make sure it's safe to run.
//!
//! @param psImage - Pointer to the image structure.
//!
//! This function is used to determine whether the flash contents of a boot image
//! is safe to run.
//!
//! @return true if the image is safe to run.
//
//*****************************************************************************
bool
am_bootloader_flash_check(am_bootloader_image_t *psImage)
{
    am_hal_mcuctrl_device_t sDevice;
    uint32_t ui32ResetVector, ui32StackPointer, ui32LinkAddress;

    ui32LinkAddress = (uint32_t) psImage->pui32LinkAddress;
    DPRINTF(("Entering %s 0x%08x\r\n", __func__, (uintptr_t)psImage));

    // Get chip specific info
#if AM_APOLLO3_MCUCTRL
    am_hal_mcuctrl_info_get(AM_HAL_MCUCTRL_INFO_DEVICEID, &sDevice);
#else // AM_APOLLO3_MCUCTRL
    am_hal_mcuctrl_device_info_get(&sDevice);
#endif // AM_APOLLO3_MCUCTRL

    //
    // Make sure the link address is in flash.
    //
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4L) || defined(AM_PART_APOLLO4P)
    if (((AM_HAL_MRAM_ADDR != 0) && (ui32LinkAddress < AM_HAL_MRAM_ADDR)) ||
        (ui32LinkAddress >= (AM_HAL_MRAM_ADDR + sDevice.ui32FlashSize)))
#else
    if (((AM_HAL_FLASH_ADDR != 0) && (ui32LinkAddress < AM_HAL_FLASH_ADDR)) ||
        (ui32LinkAddress >= (AM_HAL_FLASH_ADDR + sDevice.ui32FlashSize)))
#endif
    {
        DPRINTF(("Link address outside of flash. 0x%08x\r\n", ui32LinkAddress));
        return false;
    }

    //
    // Check to see if the image was encrypted. If it was, these tests won't
    // work. We'll need to just skip them.
    //
    if ( psImage->bEncrypted == false )
    {
        ui32StackPointer = psImage->pui32LinkAddress[0];
        ui32ResetVector = psImage->pui32LinkAddress[1];
    }
    else
    {
        ui32StackPointer = (uint32_t) psImage->pui32StackPointer;
        ui32ResetVector = (uint32_t) psImage->pui32ResetVector;
    }

#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4L) || defined(AM_PART_APOLLO4P)
    //
    // Make sure the stack is in SRAM.
    //
    if (((SRAM_BASEADDR != 0) && (ui32StackPointer < SRAM_BASEADDR))
        || (ui32StackPointer >= (SRAM_BASEADDR + sDevice.ui32DTCMSize)))
    {
        DPRINTF(("Stack not in SRAM 0x%08x\r\n", ui32StackPointer));
        return false;
    }
#else
    //
    // Make sure the stack is in SRAM.
    //
    if (((SRAM_BASEADDR != 0) && (ui32StackPointer < SRAM_BASEADDR))
        || (ui32StackPointer >= (SRAM_BASEADDR + sDevice.ui32SRAMSize)))
    {
        DPRINTF(("Stack not in SRAM 0x%08x\r\n", ui32StackPointer));
        return false;
    }
#endif
    
    //
    // Make sure the reset vector points somewhere in the image.
    //
    if (ui32ResetVector < ui32LinkAddress ||
        ui32ResetVector >= (ui32LinkAddress + psImage->ui32NumBytes))
    {
        DPRINTF(("Reset Vector not in image 0x%08x\r\n", ui32ResetVector));
        return false;
    }

    //
    // If the image isn't encrypted, run a CRC32.
    //
    if ( psImage->bEncrypted == false )
    {
        //
        // Run a CRC on the image to make sure it matches the stored checksum
        // value.
        //
        if ( am_bootloader_fast_crc32(psImage->pui32LinkAddress, psImage->ui32NumBytes) !=
             psImage->ui32CRC )
        {
            DPRINTF(("Bad CRC 0x%08x\r\n", psImage->ui32CRC));
            return false;
        }
    }

    //
    // If those tests pass, we're probably safe to run.
    //
    return true;
}

//*****************************************************************************
//
//! @brief Checks the override GPIO for manual override
//!
//! @param psImage - Pointer to the image structure.
//!
//! The image structure specifies a GPIO to manually override this test.
//! If the GPIO state matches the state specified in the structure, this function
//! returns false
//!
//! @return true if override is asserted
//
//*****************************************************************************
bool
am_hal_bootloader_override_check(am_bootloader_image_t *psImage)
{
    DPRINTF(("Entering %s 0x%08x\r\n", __func__, (uintptr_t)psImage));
    uint32_t    ui32OverridePin;
    //
    // Check the override GPIO
    //
    if ( psImage->ui32OverrideGPIO != 0xFFFFFFFF )
    {
#ifdef BOOTLOADER_DEBUG
        if ( psImage->ui32OverridePolarity & 0x2 )
        {
            am_hal_gpio_pin_config(17, AM_HAL_PIN_OUTPUT);
            am_hal_gpio_out_bit_set(17);
            am_util_delay_us(100);
            am_hal_gpio_out_bit_clear(17);
        }
#endif
        //
        // Temporarily configure the override pin as an input.
        //
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4L) || defined(AM_PART_APOLLO4P)
        am_hal_gpio_pinconfig(psImage->ui32OverrideGPIO, am_hal_gpio_pincfg_input);
#else
#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
        am_hal_gpio_pinconfig(psImage->ui32OverrideGPIO, g_AM_HAL_GPIO_INPUT_ENABLE);
#else
        am_hal_gpio_pin_config(psImage->ui32OverrideGPIO, AM_HAL_PIN_INPUT);
#endif
#endif
        //
        // If the override pin matches the specified polarity, force a failure.
        //
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4L) || defined(AM_PART_APOLLO4P)
        am_hal_gpio_state_read(psImage->ui32OverrideGPIO, AM_HAL_GPIO_INPUT_READ, &ui32OverridePin );
#else
#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
        am_hal_gpio_state_read(psImage->ui32OverrideGPIO, AM_HAL_GPIO_INPUT_READ, &ui32OverridePin );
#else
        ui32OverridePin = am_hal_gpio_input_bit_read(psImage->ui32OverrideGPIO);
#endif
#endif
        if ( ui32OverridePin == (psImage->ui32OverridePolarity & 0x1) )
        {
            DPRINTF(("Override Pin %d matches Polarity, force failure. %d, %d\r\n", psImage->ui32OverrideGPIO,  ui32OverridePin, psImage->ui32OverridePolarity));
            //
            // Make sure to disable the pin before continuing.
            //
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4L) || defined(AM_PART_APOLLO4P)
            am_hal_gpio_pinconfig(psImage->ui32OverrideGPIO, am_hal_gpio_pincfg_disabled);
#else
#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
            am_hal_gpio_pinconfig(psImage->ui32OverrideGPIO, g_AM_HAL_GPIO_INPUT_DISABLE);
#else
            am_hal_gpio_pin_config(psImage->ui32OverrideGPIO, AM_HAL_PIN_DISABLE);
#endif
#endif
            return true;
        }
#ifdef BOOTLOADER_DEBUG
        else
        {
            DPRINTF(("Override Pin %d mismatches Polarity, load image. %d, %d\r\n", psImage->ui32OverrideGPIO,  ui32OverridePin, psImage->ui32OverridePolarity));
        }
#endif

        //
        // If the test passed, we still need to make sure the GPIO is disabled,
        // as it might interfere with the program we are (presumably) about to
        // boot.
        //
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4L) || defined(AM_PART_APOLLO4P)
            am_hal_gpio_pinconfig(psImage->ui32OverrideGPIO, am_hal_gpio_pincfg_disabled);
#else
#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
            am_hal_gpio_pinconfig(psImage->ui32OverrideGPIO, g_AM_HAL_GPIO_INPUT_DISABLE);
#else
            am_hal_gpio_pin_config(psImage->ui32OverrideGPIO, AM_HAL_PIN_DISABLE);
#endif
#endif
    }

    return false;
}

//*****************************************************************************
//
//! @brief Checks a boot image to make sure it's safe to run.
//!
//! @param psImage - Pointer to the image structure.
//!
//! This function is used to determine whether a boot image is safe to run. It
//! verifies that the starting stack-pointer and reset vector entries for the
//! new image are reasonable. If these tests pass, it also runs a CRC-32 on the
//! image and checks the result against the expected CRC-32 value from the
//! image structure.
//!
//! The image structure can also specify a GPIO to manually override this test.
//! If the GPIO state matches the state specified in the structure, this test
//! will mark the image as "unsafe" regardless of the other conditions.
//!
//! @return true if the image is safe to run.
//
//*****************************************************************************
bool
am_bootloader_image_check(am_bootloader_image_t *psImage)
{
    if (am_hal_bootloader_override_check(psImage) == true)
    {
        return am_bootloader_flash_check(psImage);
    }
    else
    {
        return false;
    }

}

//*****************************************************************************
//
//! @brief Validates a information structure integrity.
//!
//! @param pInfo is a pointer to the information structure
//! @param size is the size of the structure
//!
//! This function will compute the CRC for the information structure and
//! validate it against the contents in the last 4 bytes of the same.
//! It assumes that the structure contains the CRC-32 of the information in the
//! last 4 bytes of the structure
//!
//! @return true if the check passes.
//
//*****************************************************************************
bool
am_bootloader_validate_structure(uint32_t *pInfo, uint32_t size)
{
    uint32_t ui32Crc = 0;
    // CRC value is the last 4 bytes of the structure
    uint32_t *pCrc = pInfo + size / 4 - 1;
    // Compute and validate CRC of the structure
    am_bootloader_partial_crc32(pInfo, size - 4, &ui32Crc);
    if (*pCrc != ui32Crc)
    {
        return false;
    }
    else
    {
        return true;
    }
}

//*****************************************************************************
//
//! @brief Updates the bootloader flag page with a new image pointer.
//!
//! @param psImage is a pointer to the image structure to use.
//! @param pui8FlagPage is a pointer to the flag page address.
//!
//! This function also computes and updates the checksum of the image info.
//! This function will overwrite the existing flag page entry with a new
//! reference to the image described by psImage.
//!
//! @return None.
//
//*****************************************************************************
int
am_bootloader_flag_page_update(am_bootloader_image_t *psImage,
                               uint32_t *pui32FlagPage)
{
    uint32_t ui32Critical;
    psImage->ui32Checksum = 0;
    DPRINTF(("Image to use: 0x%08x\r\n", (uintptr_t)psImage));
    DPRINTF(("Flag page address: 0x%08x\r\n", (uintptr_t)pui32FlagPage));

    // Compute CRC of the structure
    am_bootloader_partial_crc32(psImage, sizeof(*psImage) - 4, &psImage->ui32Checksum);
    //
    // Start a critical section.
    //
    ui32Critical = am_hal_interrupt_master_disable();
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4L) || defined(AM_PART_APOLLO4P)
    int rc = am_hal_mram_main_program(AM_HAL_MRAM_PROGRAM_KEY,
                              (uint32_t *) psImage,
                              pui32FlagPage,
                              sizeof(am_bootloader_image_t) / 4);
#else
    uint32_t ui32Block, ui32Page;

    //
    // Calculate the correct flag page number.
    //
    ui32Page = AM_HAL_FLASH_ADDR2PAGE((uintptr_t)pui32FlagPage);
    DPRINTF(("Flag page %d\r\n", ui32Page));
    ui32Block = AM_HAL_FLASH_ADDR2INST((uint32_t)pui32FlagPage);
    DPRINTF(("Flag page in block %d\r\n", ui32Block));

    //
    // Erase the page.
    //
    int rc = am_hal_flash_page_erase(AM_HAL_FLASH_PROGRAM_KEY, ui32Block, ui32Page);
    DPRINTF(("Flash Erased %d\r\n", rc));
    //
    // Write the psImage structure directly to the flag page.
    //
    rc = am_hal_flash_program_main(AM_HAL_FLASH_PROGRAM_KEY,
                              (uint32_t *) psImage,
                              pui32FlagPage,
                              sizeof(am_bootloader_image_t) / 4);
#endif
    //
    // Exit the critical section.
    //
    am_hal_interrupt_master_set(ui32Critical);
    DPRINTF(("Done. %d\r\n", rc));
    return rc;
}


//*****************************************************************************
//
//! @brief Check an index against a 32b mask value - used to implement a
//! monotonic counter.
//!
//! @param index is the index to be validated
//! @param pMask is the pointer to the mask.
//!
//! This function can be used to validate an index against valid values
//! The mask indicates the valid index values - up to 32 of them.
//! MSB corresponds to index 0, bit 30 corresponds to index 1 and so on,
//! LSB corresponds to index 31
//!
//! @return return false if the bit corresponding to index is set.
//
//*****************************************************************************
bool
am_bootloader_check_index(uint32_t index, uint32_t *pMask)
{
    uint32_t valid = *pMask;
    if (index > 31)
    {
        return true;
    }
    if (valid & (1 << (31 - index)))
    {
        return false;
    }
    return true;

}

//*****************************************************************************
//
//! @brief Erase a flash page
//!
//! @param ui32Addr is the starting address of flash page to be erased
//!
//! This function will erase the designated flash page
//! Any existing data on the page will be erased even if number of
//! bytes is less than page size
//!
//! @return none
//
//*****************************************************************************
void
am_bootloader_erase_flash_page(uint32_t ui32Addr)
{
#if !defined(AM_PART_APOLLO4) && !defined(AM_PART_APOLLO4B) && !defined(AM_PART_APOLLO4L) && !defined(AM_PART_APOLLO4P)
    uint32_t ui32Critical;
    uint32_t ui32CurrentPage, ui32CurrentBlock;
    //
    // Figure out what page and block we're working on.
    //
    ui32CurrentPage =  AM_HAL_FLASH_ADDR2PAGE(ui32Addr);
    ui32CurrentBlock = AM_HAL_FLASH_ADDR2INST(ui32Addr);
    //
    // Start a critical section.
    //
    ui32Critical = am_hal_interrupt_master_disable();
    am_hal_flash_page_erase(AM_HAL_FLASH_PROGRAM_KEY,
                                ui32CurrentBlock, ui32CurrentPage);
    //
    // Exit the critical section.
    //
    am_hal_interrupt_master_set(ui32Critical);
#endif
}

//*****************************************************************************
//
//! @brief Write to flash area within a flash page
//!
//! @param ui32WriteAddr is the starting address of flash to be programmed
//! @param pui32ReadAddr is the pointer to data to be programmed
//! @param numBytes is the number of bytes to be programmed
//!
//! This function will write to the designated flash area. This function does
//! not erase the page, and hence if it has not been erased earlier, it will
//! effectively AND the new data to existing flash content.
//!
//! All the write should be within a flash page.  It does not affect
//! rest of the flash page.
//!
//! @return none
//
//*****************************************************************************
void
am_bootloader_write_flash_within_page(uint32_t ui32WriteAddr,
                                      uint32_t *pui32ReadAddr,
                                      uint32_t ui32NumWords)
{
    uint32_t ui32Critical;

    //
    // Start a critical section.
    //
    ui32Critical = am_hal_interrupt_master_disable();
    //
    // Program the flash page with the data we just received.
    //
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B) ||  defined(AM_PART_APOLLO4L) || defined(AM_PART_APOLLO4P)
    am_hal_mram_main_program(AM_HAL_MRAM_PROGRAM_KEY, pui32ReadAddr,
                              (uint32_t *)ui32WriteAddr, ui32NumWords);
#else
    am_hal_flash_program_main(AM_HAL_FLASH_PROGRAM_KEY, pui32ReadAddr,
                              (uint32_t *)ui32WriteAddr, ui32NumWords);
#endif
    //
    // Exit the critical section.
    //
    am_hal_interrupt_master_set(ui32Critical);
}

//*****************************************************************************
//
//! @brief Reprogram a flash page
//!
//! @param ui32WriteAddr is the starting address of flash page to be programmed
//! @param pui32ReadAddr is the pointer to data to be programmed
//! @param numBytes is the number of bytes to be programmed
//!
//! This function will re-program the designated flash page by erasing & then
//! writing new data. Any existing data will be overwritten even if number of
//! bytes is less than page size
//!
//! @return none
//
//*****************************************************************************
void
am_bootloader_program_flash_page(uint32_t ui32WriteAddr,
                                 uint32_t *pui32ReadAddr, uint32_t numBytes)
{
    uint32_t ui32Critical;
    uint32_t ui32WordsInBuffer;

    am_bootloader_erase_flash_page(ui32WriteAddr);

    ui32WordsInBuffer = (numBytes + 3) / 4;
    //
    // Start a critical section.
    //
    ui32Critical = am_hal_interrupt_master_disable();
    //
    // Program the flash page with the data we just received.
    //
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4L) || defined(AM_PART_APOLLO4P)
    am_hal_mram_main_program(AM_HAL_MRAM_PROGRAM_KEY, pui32ReadAddr,
                              (uint32_t *)ui32WriteAddr, ui32WordsInBuffer);
#else
    am_hal_flash_program_main(AM_HAL_FLASH_PROGRAM_KEY, pui32ReadAddr,
                              (uint32_t *)ui32WriteAddr, ui32WordsInBuffer);
#endif
    //
    // Exit the critical section.
    //
    am_hal_interrupt_master_set(ui32Critical);
}


//*****************************************************************************
//
//! @brief Execute an image that has already been written to flash.
//!
//! @param psImage is a pointer to the image structure.
//!
//! This function can be used to "run" a program that has already been
//! downloaded and written to flash. It does this by reading the initial
//! stack-pointer and reset vector information from the image written in flash,
//! writing that information to the relevant registers, and immediately
//! branching to the new reset vector location.
//!
//! Note that this method does not include any type of reset. It is the callers
//! responsibility to ensure that the MCU is in a valid state for the
//! subsequent program to run. One way to guarantee this is to run this
//! function very early after a RESET event, before clocks or peripherals are
//! configured.
//!
//! @return The function does not return.
//
//*****************************************************************************
void
am_bootloader_image_run(am_bootloader_image_t *psImage)
{
    //
    // The underlying boot sequence is a little different depeding on whether
    // the image was delivered as an encrypted image or as a cleartext image.
    // We will call the correct assembly routine based on what the image
    // structure tells us.
    //
    if ( psImage->bEncrypted )
    {
        am_bootloader_encrypted_image_run(psImage);
    }
    else
    {
        am_bootloader_clear_image_run(psImage);
    }
}

//*****************************************************************************
//
//! @brief Execute an image that has already been written to flash.
//!
//! @param psImage is a pointer to the image structure.
//!
//! This function can be used to "run" a program that has already been
//! downloaded and written to flash. It does this by reading the initial
//! stack-pointer and reset vector information from the image written in flash,
//! writing that information to the relevant registers, and immediately
//! branching to the new reset vector location.
//!
//! Note that this method does not include any type of reset. It is the callers
//! responsibility to ensure that the MCU is in a valid state for the
//! subsequent program to run. One way to guarantee this is to run this
//! function very early after a RESET event, before clocks or peripherals are
//! configured.
//!
//! @return
//
//*****************************************************************************
#if (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION < 6000000)
__asm void
am_bootloader_encrypted_image_run(am_bootloader_image_t *psImage)
{
    //
    // Load the new stack pointer into R1 and the new reset vector into R2.
    //
    ldr     r1, [r0, #20]
    ldr     r2, [r0, #24]

    //
    // Load the link address of the boot image into R0.
    //
    ldr     r0, [r0, #0]

    //
    // Store the vector table pointer of the new image into VTOR.
    //
    movw    r3, #0xED08
    movt    r3, #0xE000
    str     r0, [r3, #0]

    //
    // Set the stack pointer for the new image.
    //
    mov     sp, r1

    //
    // Jump to the new reset vector.
    //
    bx      r2
}
#elif (defined (__ARMCC_VERSION)) && (__ARMCC_VERSION > 6000000)
void __attribute__((naked))
am_bootloader_encrypted_image_run(am_bootloader_image_t *psImage)
{
    //
    // Load the new stack pointer into R1 and the new reset vector into R2.
    //
    __asm("    ldr     r1, [r0, #20]");
    __asm("    ldr     r2, [r0, #24]");

    //
    // Load the link address of the boot image into R0.
    //
    __asm("    ldr     r0, [r0, #0]");

    //
    // Store the vector table pointer of the new image into VTOR.
    //
    __asm("    movw    r3, #0xED08");
    __asm("    movt    r3, #0xE000");
    __asm("    str     r0, [r3, #0]");

    //
    // Set the stack pointer for the new image.
    //
    __asm("    mov     sp, r1");

    //
    // Jump to the new reset vector.
    //
    __asm("    bx      r2");
}
#elif defined(__GNUC_STDC_INLINE__)
void __attribute__((naked))
am_bootloader_encrypted_image_run(am_bootloader_image_t *psImage)
{
    //
    // Load the new stack pointer into R1 and the new reset vector into R2.
    //
    __asm("    ldr     r1, [r0, #20]");
    __asm("    ldr     r2, [r0, #24]");

    //
    // Load the link address of the boot image into R0.
    //
    __asm("    ldr     r0, [r0, #0]");

    //
    // Store the vector table pointer of the new image into VTOR.
    //
    __asm("    movw    r3, #0xED08");
    __asm("    movt    r3, #0xE000");
    __asm("    str     r0, [r3, #0]");

    //
    // Set the stack pointer for the new image.
    //
    __asm("    mov     sp, r1");

    //
    // Jump to the new reset vector.
    //
    __asm("    bx      r2");
}
#elif defined(__IAR_SYSTEMS_ICC__)
__stackless void
am_bootloader_encrypted_image_run(am_bootloader_image_t *psImage)
{
    //
    // Load the new stack pointer into R1 and the new reset vector into R2.
    //
    __asm("    ldr     r1, [r0, #20]");
    __asm("    ldr     r2, [r0, #24]");

    //
    // Load the link address into R0.
    //
    __asm("    ldr     r0, [r0, #0]");

    //
    // Store the vector table pointer of the new image into VTOR.
    //
    __asm("    movw    r3, #0xED08");
    __asm("    movt    r3, #0xE000");
    __asm("    str     r0, [r3, #0]");

    //
    // Set the stack pointer for the new image.
    //
    __asm("    mov     sp, r1");

    //
    // Jump to the new reset vector.
    //
    __asm("    bx      r2");
}
#endif

//*****************************************************************************
//
//! @brief Execute an image that has already been written to flash.
//!
//! @param psImage is a pointer to the image structure.
//!
//! This function can be used to "run" a program that has already been
//! downloaded and written to flash. It does this by reading the initial
//! stack-pointer and reset vector information from the image written in flash,
//! writing that information to the relevant registers, and immediately
//! branching to the new reset vector location.
//!
//! Note that this method does not include any type of reset. It is the callers
//! responsibility to ensure that the MCU is in a valid state for the
//! subsequent program to run. One way to guarantee this is to run this
//! function very early after a RESET event, before clocks or peripherals are
//! configured.
//!
//! @return
//
//*****************************************************************************
#if defined(__GNUC__) || defined(keil6)
void __attribute__((naked))
am_bootloader_clear_image_run(am_bootloader_image_t *psImage)
{
    //
    // Load the link address of the boot image into R0.
    //
    __asm("    ldr     r0, [r0, #0]");

    //
    // Store the vector table pointer of the new image into VTOR.
    //
    __asm("    movw    r3, #0xED08");
    __asm("    movt    r3, #0xE000");
    __asm("    str     r0, [r3, #0]");

    //
    // Load the new stack pointer into R1 and the new reset vector into R2.
    //
    __asm("    ldr     r1, [r0, #0]");
    __asm("    ldr     r2, [r0, #4]");

    //
    // Set the stack pointer for the new image.
    //
    __asm("    mov     sp, r1");

    //
    // Jump to the new reset vector.
    //
    __asm("    bx      r2");
}
#elif defined(keil)
__asm void
am_bootloader_clear_image_run(am_bootloader_image_t *psImage)
{
    //
    // Load the link address of the boot image into R0.
    //
    ldr     r0, [r0, #0]

    //
    // Store the vector table pointer of the new image into VTOR.
    //
    movw    r3, #0xED08
    movt    r3, #0xE000
    str     r0, [r3, #0]

    //
    // Load the new stack pointer into R1 and the new reset vector into R2.
    //
    ldr     r1, [r0, #0]
    ldr     r2, [r0, #4]

    //
    // Set the stack pointer for the new image.
    //
    mov     sp, r1

    //
    // Jump to the new reset vector.
    //
    bx      r2
}
#elif defined(iar)
__stackless void
am_bootloader_clear_image_run(am_bootloader_image_t *psImage)
{
    //
    // Load the link address into R0.
    //
    __asm("    ldr     r0, [r0, #0]");

    //
    // Store the vector table pointer of the new image into VTOR.
    //
    __asm("    movw    r3, #0xED08");
    __asm("    movt    r3, #0xE000");
    __asm("    str     r0, [r3, #0]");

    //
    // Load the new stack pointer into R1 and the new reset vector into R2.
    //
    __asm("    ldr     r1, [r0, #0]");
    __asm("    ldr     r2, [r0, #4]");

    //
    // Set the stack pointer for the new image.
    //
    __asm("    mov     sp, r1");

    //
    // Jump to the new reset vector.
    //
    __asm("    bx      r2");
}
#endif

