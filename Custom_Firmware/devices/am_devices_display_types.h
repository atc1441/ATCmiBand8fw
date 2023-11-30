//*****************************************************************************
//
//! @file am_devices_display_types.h
//!
//! @brief Display related types declarations that are common across all of the
//!        Ambiq Micro supported display devices.
//!
//! @addtogroup disp_type Display Devices Type Defines
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
#ifndef AM_DEVICES_DISPLAY_TYPES_H
#define AM_DEVICES_DISPLAY_TYPES_H

//
// NEMA_PLATFORM is a macro inherited from NemaGFX_SDK
// it is used here to distinguish graphics and non-graphics projects
//
#ifdef NEMA_PLATFORM

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"

//*****************************************************************************
//
//! @name Display controller IP type.
//! @brief The display controller IPs in Apollo SoC to drive the panel.
//!
//! @note DC stands for the dedicated Display Controller IP in Apollo4B/Apollo4P SoC,
//! MSPI stands for the Multi-SPI master interface IP in all Apollo series SoC.
//! More details please refer to the corresponding datasheet.
//! @{
//
//*****************************************************************************
#define  DISP_CTRL_IP_DC            0x01
#define  DISP_CTRL_IP_MSPI          0x02

#define DISP_CTRL_IP            DISP_CTRL_IP_MSPI

//*****************************************************************************
//
//! @brief Default MSPI instance number when MSPI is used as the controller
//
//*****************************************************************************
#if (DISP_CTRL_IP == DISP_CTRL_IP_MSPI)
#  if !defined (DISPLAY_MSPI_INST)
      //
      // use MSPI2 to drive the display by default, set to 1 if MSPI1 is used
      //
#    define DISPLAY_MSPI_INST       1
#  endif
#endif

//*****************************************************************************
//
//! @brief Default TE pin number
//! @} Display controller IP type.
//
//*****************************************************************************
#if !defined (DISPLAY_TE_PIN)
#  if (DISP_CTRL_IP == DISP_CTRL_IP_MSPI)
#    define DISPLAY_TE_PIN          43
#  else
     //
     // change to other TE pin according to your BSP usage
     // default is the one for DSI interface
     //
#    define DISPLAY_TE_PIN          AM_BSP_GPIO_DISP_DSI_TE
#  endif
#endif


#ifdef __cplusplus
extern "C"
{
#endif


//*****************************************************************************
//
//! @brief The display IC in the display panels which Apollo SoC talks to.
//!
//! @note These macros stand for different display panel IC types. For more details,
//! please refer to the IC datasheets.
//
//*****************************************************************************
typedef enum
{
    DISP_IC_RM69330                 = 0x01,
    DISP_IC_RM67162                 = 0x02
} am_devices_disp_ic_e;

//*****************************************************************************
//
//! @brief The interface type between Apollo SoC and the display panel.
//!
//! @note Apollo DC IP supports SPI4, DSPI, QSPI and MIPI-DSI together with DSI IP.
//! Apollo MSPI IP supports QSPI for now.
//
//*****************************************************************************
typedef enum
{
    DISP_IF_DSI                     = 0x00,
    DISP_IF_SPI4                    = 0x01,
    DISP_IF_DSPI                    = 0x02,
    DISP_IF_QSPI                    = 0x04,
    DISP_IF_JDI                     = 0x08,
    DISP_IF_DPI                     = 0x10,
    DISP_IF_DBI                     = 0x20
} am_devices_disp_if_e;

//*****************************************************************************
//
//! @brief The interface type between Apollo SoC and the display panel.
//!
//! @note Apollo DC IP supports SPI4, DSPI, QSPI and MIPI-DSI together with DSI IP.
//! Apollo MSPI IP supports QSPI for now.
//
//*****************************************************************************
typedef enum
{
    COLOR_FORMAT_RGB565,
    COLOR_FORMAT_RGB888
} am_devices_disp_color_e;

//*****************************************************************************
//
//! @brief The handling method by Apollo SoC of TE signal from the panel.
//!
//! @note Apollo could disable TE handling, or treat TE signal as an GPIO interrupt,
//! or handle the TE signal by its DC IP.
//
//*****************************************************************************
typedef enum
{
    DISP_TE_DISABLE,
    DISP_TE_GPIO,
    DISP_TE_DC
}am_devices_disp_te_type_e;

//*****************************************************************************
//
//! @brief Detailed result after display operations
//
//*****************************************************************************
typedef enum
{
    AM_DEVICES_DISPLAY_STATUS_SUCCESS,
    AM_DEVICES_DISPLAY_STATUS_ERROR,
    AM_DEVICES_DISPLAY_STATUS_INVALID_HANDLE,
    AM_DEVICES_DISPLAY_STATUS_IN_USE,
    AM_DEVICES_DISPLAY_STATUS_TIMEOUT,
    AM_DEVICES_DISPLAY_STATUS_OUT_OF_RANGE,
    AM_DEVICES_DISPLAY_STATUS_INVALID_ARG,
    AM_DEVICES_DISPLAY_STATUS_INVALID_OPERATION,
    AM_DEVICES_DISPLAY_STATUS_PANEL_ERR,
    AM_DEVICES_DISPLAY_STATUS_DPCTR_ERR,
    AM_DEVICES_DISPLAY_STATUS_TRY_AGAIN,
} am_devices_display_status_t;

//*****************************************************************************
//
//! @brief The display configurations at the BSP (board) level
//
//*****************************************************************************
typedef struct
{
    //! Display Panel IC
    am_devices_disp_ic_e    eIC;

    //! Display interface type
    am_devices_disp_if_e    eInterface;

    //! Display panel offset
    uint16_t                ui16Offset;

    //! Display flip or not
    bool                    bFlip;

    //! Display panel resolution
    uint16_t                ui16ResX;
    uint16_t                ui16ResY;

#if (DISP_CTRL_IP == DISP_CTRL_IP_MSPI)
    //
    //! MSPI as display controller IP
    //
    uint32_t                ui32Module;        // MSPI IP instance number
    am_hal_mspi_device_e    eDeviceConfig;     // MSPI interface mode & CE
    am_hal_mspi_clock_e     eClockFreq;        // MSPI clock frequency
    bool                    bClockonD4;        // MSPI clock on Data4
#endif

#if (DISP_CTRL_IP == DISP_CTRL_IP_DC)
    //
    //! DC as display controller IP
    //
    am_hal_dsi_freq_trim_e  eDsiFreq;          // DSI IF frequency
    am_hal_dsi_dbi_width_e  eDbiWidth;         // DSI IP source (DBI) width
    uint8_t ui8NumLanes;                       // DSI IF lane number
#endif

    uint8_t                 ui8DispMspiSelect; // DISP QSPI interface number

    //
    //! TE (Tearing-Effect) signal configurations
    //
    am_devices_disp_te_type_e   eTEType;           // TE type
    uint16_t                ui16TEpin;         // GPIO No (TE GPIO interrupt)
} am_devices_display_hw_config_t;




#ifdef __cplusplus
}
#endif

#endif // NEMA_PLATFORM

#endif // AM_DEVICES_DISPLAY_TYPES_H
//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
