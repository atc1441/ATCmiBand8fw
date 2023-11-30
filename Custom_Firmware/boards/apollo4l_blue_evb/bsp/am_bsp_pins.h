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

//*****************************************************************************
//
// LED0 pin: The LED nearest the inside of the board (D3). Note: Active Low.
//
//*****************************************************************************
#define AM_BSP_GPIO_LED0                    12
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_LED0;

//*****************************************************************************
//
// LED1 pin: The middle LED (D4). Note: Active Low.
//
//*****************************************************************************
#define AM_BSP_GPIO_LED1                    15
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_LED1;

//*****************************************************************************
//
// LED2 pin: The LED nearest the outside of the board (D5). Note: Active Low.
//
//*****************************************************************************
#define AM_BSP_GPIO_LED2                    16
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_LED2;

//*****************************************************************************
//
// BUTTON0 pin: Labeled SW1 on the Apollo4l Blue EVB.
//
//*****************************************************************************
#define AM_BSP_GPIO_BUTTON0                 18
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_BUTTON0;

//*****************************************************************************
//
// BUTTON1 pin: Labeled SW2 on the Apollo4l Blue EVB.
//
//*****************************************************************************
#define AM_BSP_GPIO_BUTTON1                 19
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_BUTTON1;

//*****************************************************************************
//
// COM_UART_TX pin: This pin is UART2 transmit used by J-Link.
//
//*****************************************************************************
#define AM_BSP_GPIO_COM_UART_TX             13
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_COM_UART_TX;

//*****************************************************************************
//
// COM_UART_RX pin: This pin is UART2 receive used by J-Link.
//
//*****************************************************************************
#define AM_BSP_GPIO_COM_UART_RX             11
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_COM_UART_RX;

//*****************************************************************************
//
// UART1_TX pin: This pin is UART1 transmit. Note: ETM Trace CLK on GPIO0 cannot be used simultaneously
//
//*****************************************************************************
#define AM_BSP_GPIO_UART1_TX                0
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_UART1_TX;

//*****************************************************************************
//
// UART1_RX pin: This pin is UART1 receive.
//
//*****************************************************************************
#define AM_BSP_GPIO_UART1_RX                14
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_UART1_RX;

//*****************************************************************************
//
// UART1_CTS pin: This pin is the UART1 CTS.
//
//*****************************************************************************
#define AM_BSP_GPIO_UART1_CTS               51
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_UART1_CTS;

//*****************************************************************************
//
// UART1_RTS pin: This pin is the UART1 RTS.
//
//*****************************************************************************
#define AM_BSP_GPIO_UART1_RTS               49
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_UART1_RTS;

//*****************************************************************************
//
// UART2_TX pin: This pin is UART2 transmit.
//
//*****************************************************************************
#define AM_BSP_GPIO_UART2_TX                13
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_UART2_TX;

//*****************************************************************************
//
// UART2_RX pin: This pin is UART2 receive.
//
//*****************************************************************************
#define AM_BSP_GPIO_UART2_RX                11
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_UART2_RX;

//*****************************************************************************
//
// UART2_CTS pin: This pin is the UART2 CTS. Note: I2S0_WS cannot be used simultaneously
//
//*****************************************************************************
#define AM_BSP_GPIO_UART2_CTS               7
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_UART2_CTS;

//*****************************************************************************
//
// UART2_RTS pin: This pin is the UART2 RTS.
//
//*****************************************************************************
#define AM_BSP_GPIO_UART2_RTS               50
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_UART2_RTS;

//*****************************************************************************
//
// UART3_TX pin: This pin is UART3 transmit. Note: IOM6 clock cannot be used simultaneously.
//
//*****************************************************************************
#define AM_BSP_GPIO_UART3_TX                61
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_UART3_TX;

//*****************************************************************************
//
// UART3_RX pin: This pin is UART3 receive. Note, IOM6 MISO cannot be used simultaneously.
//
//*****************************************************************************
#define AM_BSP_GPIO_UART3_RX                63
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_UART3_RX;

//*****************************************************************************
//
// UART3_CTS pin: This pin is the UART3 CTS. Note: ADC0 cannot be used simultaneously.
//
//*****************************************************************************
#define AM_BSP_GPIO_UART3_CTS               19
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_UART3_CTS;

//*****************************************************************************
//
// UART3_RTS pin: This pin is the UART3 RTS.Note: ADC2 cannot be used simultaneously
//
//*****************************************************************************
#define AM_BSP_GPIO_UART3_RTS               17
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_UART3_RTS;

//*****************************************************************************
//
// IOM0_CS pin: I/O Master 0 chip select.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM0_CS                 72
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM0_CS;
#define AM_BSP_IOM0_CS_CHNL                 0

//*****************************************************************************
//
// IOM0_MISO pin: I/O Master 0 SPI MISO signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM0_MISO               7
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM0_MISO;

//*****************************************************************************
//
// IOM0_MOSI pin: I/O Master 0 SPI MOSI signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM0_MOSI               6
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM0_MOSI;

//*****************************************************************************
//
// IOM0_SCK pin: I/O Master 0 SPI SCK signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM0_SCK                5
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM0_SCK;

//*****************************************************************************
//
// IOM0_SCL pin: I/O Master 0 I2C clock signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM0_SCL                5
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM0_SCL;

//*****************************************************************************
//
// IOM0_SDA pin: I/O Master 0 I2C data signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM0_SDA                6
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM0_SDA;

//*****************************************************************************
//
// IOM1_CS pin: I/O Master 1 chip select.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM1_CS                 11
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM1_CS;
#define AM_BSP_IOM1_CS_CHNL                 0

//*****************************************************************************
//
// IOM1_MISO pin: I/O Master 1 SPI MISO signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM1_MISO               10
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM1_MISO;

//*****************************************************************************
//
// IOM1_MOSI pin: I/O Master 1 SPI MOSI signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM1_MOSI               9
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM1_MOSI;

//*****************************************************************************
//
// IOM1_SCK pin: I/O Master 1 SPI SCK signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM1_SCK                8
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM1_SCK;

//*****************************************************************************
//
// IOM1_SCL pin: I/O Master 1 I2C clock signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM1_SCL                8
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM1_SCL;

//*****************************************************************************
//
// IOM1_SDA pin: I/O Master 1 I2C data signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM1_SDA                9
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM1_SDA;

//*****************************************************************************
//
// IOM2_CS pin: I/O Master 2 chip select.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM2_CS                 37
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM2_CS;
#define AM_BSP_IOM2_CS_CHNL                 0

//*****************************************************************************
//
// IOM2_MISO pin: I/O Master 2 SPI MISO signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM2_MISO               27
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM2_MISO;

//*****************************************************************************
//
// IOM2_MOSI pin: I/O Master 2 SPI MOSI signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM2_MOSI               26
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM2_MOSI;

//*****************************************************************************
//
// IOM2_SCK pin: I/O Master 2 SPI SCK signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM2_SCK                25
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM2_SCK;

//*****************************************************************************
//
// IOM2_SCL pin: I/O Master 2 I2C clock signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM2_SCL                25
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM2_SCL;

//*****************************************************************************
//
// IOM2_SDA pin: I/O Master 2 I2C data signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM2_SDA                26
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM2_SDA;

//*****************************************************************************
//
// IOM3_CS pin: I/O Master 3 chip select.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM3_CS                 85
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM3_CS;
#define AM_BSP_IOM3_CS_CHNL                 0

//*****************************************************************************
//
// IOM3_MISO pin: I/O Master 3 SPI MISO signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM3_MISO               33
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM3_MISO;

//*****************************************************************************
//
// IOM3_MOSI pin: I/O Master 3 SPI MOSI signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM3_MOSI               32
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM3_MOSI;

//*****************************************************************************
//
// IOM3_SCK pin: I/O Master 3 SPI SCK signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM3_SCK                31
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM3_SCK;

//*****************************************************************************
//
// IOM3_SCL pin: I/O Master 3 I2C clock signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM3_SCL                31
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM3_SCL;

//*****************************************************************************
//
// IOM3_SDA pin: I/O Master 3 I2C data signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM3_SDA                32
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM3_SDA;

//*****************************************************************************
//
// IOM4_CS pin: I/O Master 4 chip select.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM4_CS                 54
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM4_CS;
#define AM_BSP_IOM4_CS_CHNL                 0

//*****************************************************************************
//
// IOM4_MISO pin: I/O Master 4 SPI MISO signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM4_MISO               36
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM4_MISO;

//*****************************************************************************
//
// IOM4_MOSI pin: I/O Master 4 SPI MOSI signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM4_MOSI               35
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM4_MOSI;

//*****************************************************************************
//
// IOM4_SCK pin: I/O Master 4 SPI SCK signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM4_SCK                34
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM4_SCK;

//*****************************************************************************
//
// IOM5_CS pin: I/O Master 5 chip select.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM5_CS                 73
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM5_CS;
#define AM_BSP_IOM5_CS_CHNL                 0

//*****************************************************************************
//
// IOM5_MISO pin: I/O Master 5 SPI MISO signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM5_MISO               49
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM5_MISO;

//*****************************************************************************
//
// IOM5_MOSI pin: I/O Master 5 SPI MOSI signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM5_MOSI               48
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM5_MOSI;

//*****************************************************************************
//
// IOM5_SCK pin: I/O Master 5 SPI SCK signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM5_SCK                47
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM5_SCK;

//*****************************************************************************
//
// IOM5_SCL pin: I/O Master 5 I2C clock signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM5_SCL                47
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM5_SCL;

//*****************************************************************************
//
// IOM5_SDA pin: I/O Master 5 I2C data signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM5_SDA                48
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM5_SDA;

//*****************************************************************************
//
// IOM6_CS pin: I/O Master 6 chip select.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM6_CS                 74
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM6_CS;
#define AM_BSP_IOM6_CS_CHNL                 0

//*****************************************************************************
//
// IOM6_MISO pin: I/O Master 6 SPI MISO signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM6_MISO               63
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM6_MISO;

//*****************************************************************************
//
// IOM6_MOSI pin: I/O Master 6 SPI MOSI signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM6_MOSI               62
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM6_MOSI;

//*****************************************************************************
//
// IOM6_SCK pin: I/O Master 6 SPI SCK signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM6_SCK                61
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM6_SCK;

//*****************************************************************************
//
// IOM6_SCL pin: I/O Master 6 I2C clock signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM6_SCL                61
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM6_SCL;

//*****************************************************************************
//
// IOM6_SDA pin: I/O Master 6 I2C data signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM6_SDA                62
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM6_SDA;

//*****************************************************************************
//
// IOM7_CS pin: I/O Master 7 chip select.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM7_CS                 88
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM7_CS;
#define AM_BSP_IOM7_CS_CHNL                 0

//*****************************************************************************
//
// IOM7_MISO pin: I/O Master 7 SPI MISO signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM7_MISO               24
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM7_MISO;

//*****************************************************************************
//
// IOM7_MOSI pin: I/O Master 7 SPI MOSI signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM7_MOSI               23
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM7_MOSI;

//*****************************************************************************
//
// IOM7_SCK pin: I/O Master 7 SPI SCK signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM7_SCK                22
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM7_SCK;

//*****************************************************************************
//
// IOM7_SCL pin: I/O Master 7 I2C clock signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM7_SCL                22
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM7_SCL;

//*****************************************************************************
//
// IOM7_SDA pin: I/O Master 7 I2C data signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOM7_SDA                23
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOM7_SDA;

//*****************************************************************************
//
// IOS_CE pin: I/O Slave chip select.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOS_CE                  3
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOS_CE;

//*****************************************************************************
//
// IOS_MISO pin: I/O Slave SPI MISO signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOS_MISO                2
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOS_MISO;

//*****************************************************************************
//
// IOS_MOSI pin: I/O Slave SPI MOSI signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOS_MOSI                1
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOS_MOSI;

//*****************************************************************************
//
// IOS_SCK pin: I/O Slave SPI SCK signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOS_SCK                 0
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOS_SCK;

//*****************************************************************************
//
// IOS_SCL pin: I/O Slave I2C clock signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOS_SCL                 0
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOS_SCL;

//*****************************************************************************
//
// IOS_SDA pin: I/O Slave I2C data signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_IOS_SDA                 1
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_IOS_SDA;

//*****************************************************************************
//
// MSPI0_CE0 pin: MSPI0 chip select for device 0
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_CE0               14
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI0_CE0;
#define AM_BSP_MSPI0_CE0_CHNL               0

//*****************************************************************************
//
// MSPI0_CE1 pin: MSPI0 chip select for device 1
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_CE1               7
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI0_CE1;
#define AM_BSP_MSPI0_CE1_CHNL               1

//*****************************************************************************
//
// MSPI0_D0 pin: MSPI0 data 0.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_D0                64
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI0_D0;

//*****************************************************************************
//
// MSPI0_D1 pin: MSPI0 data 1.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_D1                65
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI0_D1;

//*****************************************************************************
//
// MSPI0_D2 pin: MSPI0 data 2.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_D2                66
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI0_D2;

//*****************************************************************************
//
// MSPI0_D3 pin: MSPI0 data 3.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_D3                67
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI0_D3;

//*****************************************************************************
//
// MSPI0_D4 pin: MSPI0 data 4.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_D4                68
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI0_D4;

//*****************************************************************************
//
// MSPI0_D5 pin: MSPI0 data 5.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_D5                69
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI0_D5;

//*****************************************************************************
//
// MSPI0_D6 pin: MSPI0 data 6.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_D6                70
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI0_D6;

//*****************************************************************************
//
// MSPI0_D7 pin: MSPI0 data 7.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_D7                71
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI0_D7;

//*****************************************************************************
//
// MSPI0_SCK pin: MSPI0 clock.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_SCK               72
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI0_SCK;

//*****************************************************************************
//
// MSPI0_DQSDM pin: MSPI0 DQS DM.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI0_DQSDM             73
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI0_DQSDM;

//*****************************************************************************
//
// MSPI1_CE0 pin: MSPI1 chip select for device 0
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI1_CE0               86
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI1_CE0;
#define AM_BSP_MSPI1_CE0_CHNL               0

//*****************************************************************************
//
// MSPI1_CE1 pin: MSPI1 chip select for device 1
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI1_CE1               85
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI1_CE1;
#define AM_BSP_MSPI1_CE1_CHNL               1

//*****************************************************************************
//
// MSPI1_D0 pin: MSPI1 data 0.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI1_D0                37
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI1_D0;

//*****************************************************************************
//
// MSPI1_D1 pin: MSPI1 data 1.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI1_D1                38
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI1_D1;

//*****************************************************************************
//
// MSPI1_D2 pin: MSPI1 data 2.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI1_D2                39
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI1_D2;

//*****************************************************************************
//
// MSPI1_D3 pin: MSPI1 data 3.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI1_D3                40
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI1_D3;

//*****************************************************************************
//
// MSPI1_D4 pin: MSPI1 data 4.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI1_D4                41
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI1_D4;

//*****************************************************************************
//
// MSPI1_D5 pin: MSPI1 data 5.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI1_D5                42
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI1_D5;

//*****************************************************************************
//
// MSPI1_D6 pin: MSPI1 data 6.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI1_D6                43
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI1_D6;

//*****************************************************************************
//
// MSPI1_D7 pin: MSPI1 data 7.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI1_D7                44
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI1_D7;

//*****************************************************************************
//
// MSPI1_SCK pin: MSPI1 clock.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI1_SCK               45
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI1_SCK;

//*****************************************************************************
//
// MSPI1_DQSDM pin: MSPI1 DQS DM.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI1_DQSDM             104
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI1_DQSDM;

//*****************************************************************************
//
// MSPI2_CE0 pin: MSPI2 chip select for device 0
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_CE0               93
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI2_CE0;
#define AM_BSP_MSPI2_CE0_CHNL               0

//*****************************************************************************
//
// MSPI2_CE1 pin: MSPI2 chip select for device 1
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_CE1               91
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI2_CE1;
#define AM_BSP_MSPI2_CE1_CHNL               1

//*****************************************************************************
//
// MSPI2_D0 pin: MSPI2 data 0.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_D0                74
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI2_D0;

//*****************************************************************************
//
// MSPI2_D1 pin: MSPI2 data 1.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_D1                75
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI2_D1;

//*****************************************************************************
//
// MSPI2_D2 pin: MSPI2 data 2.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_D2                76
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI2_D2;

//*****************************************************************************
//
// MSPI2_D3 pin: MSPI2 data 3.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_D3                77
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI2_D3;

//*****************************************************************************
//
// MSPI2_D4 pin: MSPI2 data 4.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_D4                78
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI2_D4;

//*****************************************************************************
//
// MSPI2_D4_CLK pin: MSPI2 data 4 replace MSPI2_8(CLK).
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_D4_CLK            78
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI2_D4_CLK;

//*****************************************************************************
//
// MSPI2_D5 pin: MSPI2 data 5.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_D5                79
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI2_D5;

//*****************************************************************************
//
// MSPI2_D6 pin: MSPI2 data 6.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_D6                80
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI2_D6;

//*****************************************************************************
//
// MSPI2_D7 pin: MSPI2 data 7.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_D7                81
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI2_D7;

//*****************************************************************************
//
// MSPI2_SCK pin: MSPI2 clock.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_SCK               82
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI2_SCK;

//*****************************************************************************
//
// MSPI2_DQSDM pin: MSPI2 DQS DM.
//
//*****************************************************************************
#define AM_BSP_GPIO_MSPI2_DQSDM             83
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MSPI2_DQSDM;

//*****************************************************************************
//
// ITM_SWO pin: ITM Serial Wire Output.
//
//*****************************************************************************
#define AM_BSP_GPIO_ITM_SWO                 24
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_ITM_SWO;

//*****************************************************************************
//
// EMT_TRACE0 pin: ETM Serial Wire Debug Trace Out 0.
//
//*****************************************************************************
#define AM_BSP_GPIO_EMT_TRACE0              1
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_EMT_TRACE0;

//*****************************************************************************
//
// ETM_TRACE1 pin: ETM Serial Wire Debug Trace Out 1.
//
//*****************************************************************************
#define AM_BSP_GPIO_ETM_TRACE1              2
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_ETM_TRACE1;

//*****************************************************************************
//
// ETM_TRACE2 pin: ETM Serial Wire Debug Trace Out 2.
//
//*****************************************************************************
#define AM_BSP_GPIO_ETM_TRACE2              3
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_ETM_TRACE2;

//*****************************************************************************
//
// ETM_TRACE3 pin: ETM Serial Wire Debug Trace Out 3.
//
//*****************************************************************************
#define AM_BSP_GPIO_ETM_TRACE3              83
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_ETM_TRACE3;

//*****************************************************************************
//
// ETM_TRACECLK pin: ETM Serial Wire Debug Trace Clock.
//
//*****************************************************************************
#define AM_BSP_GPIO_ETM_TRACECLK            0
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_ETM_TRACECLK;

//*****************************************************************************
//
// I2S0_DATA pin: I2S0 Bidirectional Data.
//
//*****************************************************************************
#define AM_BSP_GPIO_I2S0_DATA               12
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_I2S0_DATA;

//*****************************************************************************
//
// I2S0_SDOUT pin: I2S0 Bidirectional Data.
//
//*****************************************************************************
#define AM_BSP_GPIO_I2S0_SDOUT              12
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_I2S0_SDOUT;

//*****************************************************************************
//
// I2S0_CLK pin: I2S0 Bit Clock.
//
//*****************************************************************************
#define AM_BSP_GPIO_I2S0_CLK                47
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_I2S0_CLK;

//*****************************************************************************
//
// I2S0_WS pin: I2S0 L/R Clock.
//
//*****************************************************************************
#define AM_BSP_GPIO_I2S0_WS                 49
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_I2S0_WS;

//*****************************************************************************
//
// I2S0_SDIN pin: I2S0 Data In.
//
//*****************************************************************************
#define AM_BSP_GPIO_I2S0_SDIN               14
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_I2S0_SDIN;

//*****************************************************************************
//
// PDM0_CLK pin: PDM 0 Clock Signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_PDM0_CLK                50
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_PDM0_CLK;

//*****************************************************************************
//
// PDM0_DATA pin: PDM 0 Data Signal.
//
//*****************************************************************************
#define AM_BSP_GPIO_PDM0_DATA               51
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_PDM0_DATA;

//*****************************************************************************
//
// SDIF_DAT0 pin: SD Data 0 pin.
//
//*****************************************************************************
#define AM_BSP_GPIO_SDIF_DAT0               84
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_SDIF_DAT0;

//*****************************************************************************
//
// SDIF_DAT1 pin: SD Data 1 pin.
//
//*****************************************************************************
#define AM_BSP_GPIO_SDIF_DAT1               85
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_SDIF_DAT1;

//*****************************************************************************
//
// SDIF_DAT2 pin: SD Data 2 pin.
//
//*****************************************************************************
#define AM_BSP_GPIO_SDIF_DAT2               86
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_SDIF_DAT2;

//*****************************************************************************
//
// SDIF_DAT3 pin: SD Data 3 pin.
//
//*****************************************************************************
#define AM_BSP_GPIO_SDIF_DAT3               87
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_SDIF_DAT3;

//*****************************************************************************
//
// SDIF_DAT4 pin: SD Data 4 pin.
//
//*****************************************************************************
#define AM_BSP_GPIO_SDIF_DAT4               79
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_SDIF_DAT4;

//*****************************************************************************
//
// SDIF_DAT5 pin: SD Data 5 pin.
//
//*****************************************************************************
#define AM_BSP_GPIO_SDIF_DAT5               80
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_SDIF_DAT5;

//*****************************************************************************
//
// SDIF_DAT6 pin: SD Data 6 pin.
//
//*****************************************************************************
#define AM_BSP_GPIO_SDIF_DAT6               81
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_SDIF_DAT6;

//*****************************************************************************
//
// SDIF_DAT7 pin: SD Data 7 pin.
//
//*****************************************************************************
#define AM_BSP_GPIO_SDIF_DAT7               82
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_SDIF_DAT7;

//*****************************************************************************
//
// SDIF_CMD pin: SD Command pin.
//
//*****************************************************************************
#define AM_BSP_GPIO_SDIF_CMD                83
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_SDIF_CMD;

//*****************************************************************************
//
// SDIF_CLKOUT pin: SD Clock to Card pin.
//
//*****************************************************************************
#define AM_BSP_GPIO_SDIF_CLKOUT             88
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_SDIF_CLKOUT;

//*****************************************************************************
//
// SWDIO pin: Cortex Serial Wire DIO.
//
//*****************************************************************************
#define AM_BSP_GPIO_SWDIO                   21
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_SWDIO;

//*****************************************************************************
//
// SWDCK pin: Cortex Serial Wire DCK.
//
//*****************************************************************************
#define AM_BSP_GPIO_SWDCK                   20
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_SWDCK;

//*****************************************************************************
//
// X16SPI_CE0 pin: X16SPI chip select
//
//*****************************************************************************
#define AM_BSP_GPIO_X16SPI_CE0              14
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_X16SPI_CE0;
#define AM_BSP_X16SPI_CE0_CHNL              0

//*****************************************************************************
//
// X16SPI_CE1 pin: X16SPI chip select
//
//*****************************************************************************
#define AM_BSP_GPIO_X16SPI_CE1              7
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_X16SPI_CE1;
#define AM_BSP_X16SPI_CE1_CHNL              1

//*****************************************************************************
//
// X16SPI_D0 pin: MSPI0 data 0.
//
//*****************************************************************************
#define AM_BSP_GPIO_X16SPI_D0               64
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_X16SPI_D0;

//*****************************************************************************
//
// X16SPI_D1 pin: X16SPI data 1.
//
//*****************************************************************************
#define AM_BSP_GPIO_X16SPI_D1               65
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_X16SPI_D1;

//*****************************************************************************
//
// X16SPI_D2 pin: X16SPI data 2.
//
//*****************************************************************************
#define AM_BSP_GPIO_X16SPI_D2               66
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_X16SPI_D2;

//*****************************************************************************
//
// X16SPI_D3 pin: X16SPI data 3.
//
//*****************************************************************************
#define AM_BSP_GPIO_X16SPI_D3               67
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_X16SPI_D3;

//*****************************************************************************
//
// X16SPI_D4 pin: X16SPI data 4.
//
//*****************************************************************************
#define AM_BSP_GPIO_X16SPI_D4               68
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_X16SPI_D4;

//*****************************************************************************
//
// X16SPI_D5 pin: X16SPI data 5.
//
//*****************************************************************************
#define AM_BSP_GPIO_X16SPI_D5               69
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_X16SPI_D5;

//*****************************************************************************
//
// X16SPI_D6 pin: X16SPI data 6.
//
//*****************************************************************************
#define AM_BSP_GPIO_X16SPI_D6               70
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_X16SPI_D6;

//*****************************************************************************
//
// X16SPI_D7 pin: X16SPI data 7.
//
//*****************************************************************************
#define AM_BSP_GPIO_X16SPI_D7               71
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_X16SPI_D7;

//*****************************************************************************
//
// X16SPI_SCK pin: X16SPI clock.
//
//*****************************************************************************
#define AM_BSP_GPIO_X16SPI_SCK              72
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_X16SPI_SCK;

//*****************************************************************************
//
// X16SPI_DQSDM pin: X16SPI DQS DM.
//
//*****************************************************************************
#define AM_BSP_GPIO_X16SPI_DQSDM            73
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_X16SPI_DQSDM;

//*****************************************************************************
//
// X16SPI_D8 pin: X16SPI data 8.
//
//*****************************************************************************
#define AM_BSP_GPIO_X16SPI_D8               37
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_X16SPI_D8;

//*****************************************************************************
//
// X16SPI_D9 pin: X16SPI data 9.
//
//*****************************************************************************
#define AM_BSP_GPIO_X16SPI_D9               38
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_X16SPI_D9;

//*****************************************************************************
//
// X16SPI_D10 pin: X16SPI data 10.
//
//*****************************************************************************
#define AM_BSP_GPIO_X16SPI_D10              39
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_X16SPI_D10;

//*****************************************************************************
//
// X16SPI_D11 pin: X16SPI data 11.
//
//*****************************************************************************
#define AM_BSP_GPIO_X16SPI_D11              40
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_X16SPI_D11;

//*****************************************************************************
//
// X16SPI_D12 pin: X16SPI data 12.
//
//*****************************************************************************
#define AM_BSP_GPIO_X16SPI_D12              41
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_X16SPI_D12;

//*****************************************************************************
//
// X16SPI_D13 pin: X16SPI data 13.
//
//*****************************************************************************
#define AM_BSP_GPIO_X16SPI_D13              42
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_X16SPI_D13;

//*****************************************************************************
//
// X16SPI_D14 pin: X16SPI data 14.
//
//*****************************************************************************
#define AM_BSP_GPIO_X16SPI_D14              43
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_X16SPI_D14;

//*****************************************************************************
//
// X16SPI_D15 pin: X16SPI data 15.
//
//*****************************************************************************
#define AM_BSP_GPIO_X16SPI_D15              44
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_X16SPI_D15;

//*****************************************************************************
//
// X16SPI_DQS1DM1 pin: X16SPI DQS1 DM1.
//
//*****************************************************************************
#define AM_BSP_GPIO_X16SPI_DQS1DM1          45
extern am_hal_gpio_pincfg_t g_AM_BSP_GPIO_X16SPI_DQS1DM1;

#ifdef __cplusplus
}
#endif

#endif // AM_BSP_PINS_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
