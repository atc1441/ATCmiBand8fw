//*****************************************************************************
//
//! @file am_devices_led.h
//!
//! @brief Functions for controlling an array of LEDs
//!
//! @addtogroup LED SPI Device Control for programmable LEDs.
//! @ingroup devices
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
#ifndef AM_DEVICES_LED_H
#define AM_DEVICES_LED_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! @name LED polarity macros
//! @{
//
//*****************************************************************************
#define AM_DEVICES_LED_POL_POLARITY_M       0x1
#define AM_DEVICES_LED_ON_HIGH              0x1
#define AM_DEVICES_LED_ON_LOW               0x0

//! @}

//*****************************************************************************
//
//! @brief LED direct drive indicator macro
//! @details Or this in with the polarity value to use the GPIO DATA register instead of
//! the GPIO DATA ENABLE register to directly drive an LED buffer.
//
//*****************************************************************************
#define AM_DEVICES_LED_POL_DIRECT_DRIVE_M   0x2

//*****************************************************************************
//
//! LED OD driver macro
//! OR this in with the polarity value to use the Open Drain version of the
//! output driver
//
//*****************************************************************************
#define AM_DEVICES_LED_POL_OPEN_DRAIN       0x4

//*****************************************************************************
//
//! Structure for keeping track of LEDs
//
//*****************************************************************************
typedef struct
{
    uint32_t ui32GPIONumber;
    uint32_t ui32Polarity;
}
am_devices_led_t;

//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************

//*****************************************************************************
//
//! @brief Configures the necessary pins for an array of LEDs
//!
//! @param psLED   - Pointer to an LED structure.
//!
//! This function configures a GPIO to drive an LED in a low-power way.
//
//*****************************************************************************
extern void am_devices_led_init(am_devices_led_t *psLED);

//*****************************************************************************
//
//! @brief Configures the necessary pins for an array of LEDs
//!
//! @param psLEDs       - An array of LED structures.
//! @param ui32NumLEDs  - The total number of LEDs in the array.
//!
//! This function configures the GPIOs for an array of LEDs.
//
//*****************************************************************************
extern void am_devices_led_array_init(am_devices_led_t *psLEDs, uint32_t ui32NumLEDs);

//*****************************************************************************
//
//! @brief Disables an array of LEDs
//!
//! @param psLEDs       - An array of LED structures.
//! @param ui32NumLEDs  - The total number of LEDs in the array.
//!
//! This function disables the GPIOs for an array of LEDs.
//
//*****************************************************************************
extern void am_devices_led_array_disable(am_devices_led_t *psLEDs, uint32_t ui32NumLEDs);

//*****************************************************************************
//
//! @brief Turns on the requested LED.
//!
//! @param psLEDs       - An array of LED structures.
//! @param ui32LEDNum   - The LED number for the light to turn on.
//!
//! This function turns on a single LED.
//
//*****************************************************************************
extern void am_devices_led_on(am_devices_led_t *psLEDs, uint32_t ui32LEDNum);

//*****************************************************************************
//
//! @brief Turns off the requested LED.
//!
//! @param psLEDs       - An array of LED structures.
//! @param ui32LEDNum   - The LED number for the light to turn off.
//!
//! This function turns off a single LED.
//
//*****************************************************************************
extern void am_devices_led_off(am_devices_led_t *psLEDs, uint32_t ui32LEDNum);

//*****************************************************************************
//
//! @brief Toggles the requested LED.
//!
//! @param psLEDs       - An array of LED structures.
//! @param ui32LEDNum   - The LED number for the light to toggle.
//!
//! This function toggles a single LED.
//
//*****************************************************************************
extern void am_devices_led_toggle(am_devices_led_t *psLEDs, uint32_t ui32LEDNum);

//*****************************************************************************
//
//! @brief Gets the state of the requested LED.
//!
//! @param psLEDs     - An array of LED structures.
//! @param ui32LEDNum - The LED to check.
//!
//! This function checks the state of a single LED.
//!
//! @return true if the LED is on.
//
//*****************************************************************************
extern bool am_devices_led_get(am_devices_led_t *psLEDs, uint32_t ui32LEDNum);

//*****************************************************************************
//
//! @brief Display a binary value using LEDs.
//!
//! @param psLEDs      - An array of LED structures.
//! @param ui32NumLEDs - The number of LEDs in the array.
//! @param ui32Value   - The value to display on the LEDs.
//!
//! This function displays a value in binary across an array of LEDs.
//
//*****************************************************************************
extern void am_devices_led_array_out(am_devices_led_t *psLEDs, uint32_t ui32NumLEDs,
                                     uint32_t ui32Value);
#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_LED_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

