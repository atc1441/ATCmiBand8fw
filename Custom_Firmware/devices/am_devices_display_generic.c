//*****************************************************************************
//
//! @file am_devices_display_generic.c
//!
//! @brief am_devices_display_generic included Display Controller drivers & MSPI
//! driver.this source contains lots of important macros.some of macros defined
//! in BSP will not explain in there.
//!
//! @addtogroup disp_generic Display Devices Generic Driver
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
#include "am_bsp.h"
#include "am_devices_display_generic.h"

#if (DISP_CTRL_IP == DISP_CTRL_IP_DC)
#include "am_devices_dc_xspi_raydium.c"
#include "am_devices_dc_dsi_raydium.c"
#endif //(DISP_CTRL_IP == DISP_CTRL_IP_DC)

#if (DISP_CTRL_IP == DISP_CTRL_IP_MSPI)
#include "am_devices_mspi_raydium.c"
#endif

#ifndef BAREMETAL
#  include "FreeRTOS.h"
#  include "semphr.h"
#endif

//*****************************************************************************
//
//! Macro definitions
//
//*****************************************************************************

//*****************************************************************************
//
//! Local functions.
//
//*****************************************************************************
static uint32_t am_devices_display_launch_transfer(void);
static uint32_t am_devices_display_prepare_transfer(void);

//*****************************************************************************
//
//! @brief the user configurable display settings
//!
//! how would the display users like customer applications, SDK
//! examples and testcases want to use the display
//
//*****************************************************************************
typedef struct
{
    // the display resolution
    uint16_t ui16ResX;
    uint16_t ui16ResY;

    // Address of vertex
    uint16_t ui16MinX;
    uint16_t ui16MinY;

    // the color mode of application frame-buffer
    am_devices_disp_color_e eColorMode;

} am_devices_display_user_setting_t;

//*****************************************************************************
//
//! @brief the transfer operation descriptor
//!
//! This structure hold the current transfer status and transfer parameter.
//
//*****************************************************************************
typedef struct
{

    // if frame transfer is pending for TE
    volatile bool bXferPending;

    // if frame trander in progress
    volatile bool bXferBusy;

    // frame transfer information
    uint16_t ui16XferResX;
    uint16_t ui16XferResY;
    uint32_t ui32XferAddress;

    // application callback when frame transfer completes
    am_devices_disp_handler_t fnXferDoneCb;
    void *pArgXferDone;

    // total stripe
    uint32_t total_stripe;

} am_devices_display_tranfer_t;

//*****************************************************************************
//
//! Local variables.
//
//*****************************************************************************
//
//! Display settings from user
//
static am_devices_display_user_setting_t sDispUserSetting = {0};

static am_devices_display_tranfer_t sDispTransfer =
{
    .bXferPending = false,
    .bXferBusy = false,
    .ui16XferResX = 0,
    .ui16XferResY = 0,
    .ui32XferAddress = 0,
    .fnXferDoneCb = NULL,
    .pArgXferDone = NULL,
};

#if (DISP_CTRL_IP == DISP_CTRL_IP_DC)
//
//! Default DC layer
//
static nemadc_layer_t sDefaultDCLayer =
{
    .startx        = 0,
    .sizex         = 0,
    .resx          = 0,
    .starty        = 0,
    .sizey         = 0,
    .resy          = 0,
    .stride        = -1,
#if defined (AM_PART_APOLLO4B)
    .format        = NEMADC_RGBA8888,
#else
    .format        = NEMADC_RGB24,
#endif
    .blendmode     = NEMADC_BL_SRC,
    .buscfg        = 0,
    .alpha         = 0xff,
    .flipx_en      = 0,
    .flipy_en      = 0,
};
#endif // DISP_CTRL_IP == DISP_CTRL_IP_DC

#if (DISP_CTRL_IP == DISP_CTRL_IP_MSPI)
static void                *g_MSPIDisplayHandle;
static void                 *g_DisplayHandle;

static uint32_t             ui32MspiDisplayQBuffer[(AM_HAL_MSPI_CQ_ENTRY_SIZE / 4) * 12];
//
//! Display MSPI configuration
//
static am_devices_mspi_rm69330_config_t QuadDisplayMSPICfg =
{
    .eDeviceConfig = AM_HAL_MSPI_FLASH_QUAD_CE0_1_4_4,
    .eClockFreq = AM_HAL_MSPI_CLK_48MHZ,
    .pNBTxnBuf = ui32MspiDisplayQBuffer,
    .ui32NBTxnBufLength = sizeof(ui32MspiDisplayQBuffer) / sizeof(uint32_t),
};

//*****************************************************************************
//
// DISPLAY MSPI ISR
//
//*****************************************************************************

//
//! MSPI interrupts.
//
static const IRQn_Type mspi_display_interrupts[] =
{
    MSPI0_IRQn,
    MSPI1_IRQn,
    MSPI2_IRQn,
};

//
//! Take over the interrupt handler for whichever MSPI we're using.
//
#define display_mspi_isr            am_mspi_isr1(DISPLAY_MSPI_INST)
#define am_mspi_isr1(n)             am_mspi_isr(n)
#define am_mspi_isr(n)              am_mspi ## n ## _isr

//*****************************************************************************
//
//! display_mspi_isr
//
//*****************************************************************************
void
display_mspi_isr(void)
{
    uint32_t      ui32Status;
    am_hal_mspi_interrupt_status_get(g_MSPIDisplayHandle, &ui32Status, false);
    am_hal_mspi_interrupt_clear(g_MSPIDisplayHandle, ui32Status);
    am_hal_mspi_interrupt_service(g_MSPIDisplayHandle, ui32Status);
}
#endif // DISP_CTRL_IP == DISP_CTRL_IP_MSPI

//*****************************************************************************
//
//! GPIO TE interrupts.
//
//*****************************************************************************
static const IRQn_Type te_interrupts[] =
{
    GPIO0_001F_IRQn,
    GPIO0_203F_IRQn,
    GPIO0_405F_IRQn,
    GPIO0_607F_IRQn
};

#define TE_GPIO_IDX GPIO_NUM2IDX(DISPLAY_TE_PIN)

#if TE_GPIO_IDX == 0
#define display_te_isr        am_gpio0_001f_isr
#elif TE_GPIO_IDX == 1
// #define display_te_isr        am_gpio0_203f_isr  // conflict with nema_event.c
#elif TE_GPIO_IDX == 2
#define display_te_isr        am_gpio0_405f_isr
#elif TE_GPIO_IDX == 3
#define display_te_isr        am_gpio0_607f_isr
#endif

//*****************************************************************************
//
//! TE & VSYNC interrupts sync.
//
//*****************************************************************************
#define XFER_MAX_WAIT_MS                100

#ifdef BAREMETAL
static bool bXferDone                   = false;
#else
static SemaphoreHandle_t semXferDone    = NULL;
#endif

//*****************************************************************************
//
//! @brief TE interrupt handle function
//!
//! Interrupt handler for the GPIO pins.
//
//*****************************************************************************
void
display_te_isr(void)
{
    uint32_t ui32Status;

    am_hal_gpio_interrupt_irq_status_get(te_interrupts[TE_GPIO_IDX], false, &ui32Status);
    am_hal_gpio_interrupt_irq_clear(te_interrupts[TE_GPIO_IDX], ui32Status);
    am_hal_gpio_interrupt_service(te_interrupts[TE_GPIO_IDX], ui32Status);
}

//*****************************************************************************
//
//! @brief TE interrupt handle function
//!
//! @param pvUnused            - invalid  parameter
//! @param ui32Unused          - invalid  parameter
//!
//! DC's TE interrupt handle function.
//
//*****************************************************************************
static void
am_devices_display_te_handler(void* pvUnused, uint32_t ui32Unused)
{
    //
    // Transfer the frame when TE interrupt arrives.
    //
    if (sDispTransfer.bXferPending)
    {
        am_devices_display_launch_transfer();
        sDispTransfer.bXferPending = false;
    }
}

//*****************************************************************************
//
//! @brief sets recommended scanline.
//!
//! @param TETimesPerFrame            - how many TE intervals transfer one frame
//!
//! This function sets recommended scanline,it's valid when TETimesPerFrame
//! equal to 1 or 2.
//!
//! @return Status.
//
//****************************************************************************
uint32_t
am_devices_display_set_scanline_recommended_parameter(uint8_t TETimesPerFrame)
{

#if (DISP_CTRL_IP == DISP_CTRL_IP_DC)
    uint16_t ui16ScanLine = 0;
    //
    // setting scanline.
    //
    if ( TETimesPerFrame == 2 )
    {
        //
        // setting scanline to start line plus 10 lines when frame transfer time is longer than TE signals interval.
        //
        ui16ScanLine = (sDispUserSetting.ui16MinY + 10) % g_sDispCfg.ui16ResY;
    }
    else if ( TETimesPerFrame == 1 )
    {
        //
        // setting scanline to start line minus 10 lines when frame transfer time is shorter than TE signals interval.
        //
        ui16ScanLine = (sDispUserSetting.ui16MinY + g_sDispCfg.ui16ResY - 10) % g_sDispCfg.ui16ResY;
    }
    else
    {
        return AM_DEVICES_DISPLAY_STATUS_INVALID_ARG;
    }

    if ( g_sDispCfg.eInterface == DISP_IF_DSI )
    {
        return am_devices_dc_dsi_set_scanline(ui16ScanLine, g_sDispCfg.ui16ResY);
    }
    else
    {
        return am_devices_dc_xspi_set_scanline(ui16ScanLine, g_sDispCfg.ui16ResY);
    }

#endif

#if (DISP_CTRL_IP == DISP_CTRL_IP_MSPI)
    return am_devices_mspi_rm69330_set_scanline_recommended_parameter(g_DisplayHandle, TETimesPerFrame);
#endif

}

//*****************************************************************************
//
// Initialize display subsystem.
//
//*****************************************************************************



am_devices_display_hw_config_t g_sDispCfg =
{
    //
    // Display driver IC type
    //
    .eIC = DISP_IC_RM69330,
    .eInterface = DISP_IF_DSI,
    //
    // Display te pin
    //
    .ui16TEpin = DISPLAY_TE_PIN,
    //
    // Display resolutions
    //
    .ui16ResX = 192,
    .ui16ResY = 490,
    .bFlip = false,

    .eInterface = DISP_IF_QSPI,
    .ui32Module = DISPLAY_MSPI_INST,
    .eDeviceConfig = AM_HAL_MSPI_FLASH_QUAD_CE0_1_4_4,
    .eClockFreq = AM_HAL_MSPI_CLK_48MHZ,
    .bClockonD4 = false,
    .ui8DispMspiSelect = 1,
    .eTEType = DISP_TE_GPIO,
    .ui16Offset = 0,
};


am_hal_gpio_pincfg_t g_AM_GPIO_TE_43 =
{
    .GP.cfg_b.uFuncSel             = AM_HAL_PIN_43_GPIO,
    .GP.cfg_b.eGPInput             = AM_HAL_GPIO_PIN_INPUT_ENABLE,
    .GP.cfg_b.eGPRdZero            = AM_HAL_GPIO_PIN_RDZERO_READPIN,
    .GP.cfg_b.eIntDir              = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
    .GP.cfg_b.eGPOutCfg            = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
    .GP.cfg_b.eDriveStrength       = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P1X,
    .GP.cfg_b.uSlewRate            = 0,
    .GP.cfg_b.ePullup              = AM_HAL_GPIO_PIN_PULLUP_100K,
    .GP.cfg_b.uNCE                 = 0,
    .GP.cfg_b.eCEpol               = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW,
    .GP.cfg_b.uRsvd_0              = 0,
    .GP.cfg_b.ePowerSw             = AM_HAL_GPIO_PIN_POWERSW_NONE,
    .GP.cfg_b.eForceInputEn        = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.eForceOutputEn       = AM_HAL_GPIO_PIN_FORCEEN_NONE,
    .GP.cfg_b.uRsvd_1              = 0,
};
void
am_bsp_disp_pins_enable1(void)
{

    if ( g_sDispCfg.eDeviceConfig == AM_HAL_MSPI_FLASH_QUAD_CE0_1_4_4 )
    {
        am_hal_gpio_pinconfig(86, g_AM_BSP_GPIO_MSPI2_CE0);
    }
    else if ( g_sDispCfg.eDeviceConfig == AM_HAL_MSPI_FLASH_QUAD_CE1_1_4_4 )
    {
        am_hal_gpio_pinconfig(52, g_AM_BSP_GPIO_MSPI2_CE1);
    }

    am_hal_gpio_pinconfig(37,  g_AM_BSP_GPIO_MSPI1_D0); 
    am_hal_gpio_pinconfig(38,  g_AM_BSP_GPIO_MSPI1_D1);
    am_hal_gpio_pinconfig(39,  g_AM_BSP_GPIO_MSPI1_D2);
    am_hal_gpio_pinconfig(40,  g_AM_BSP_GPIO_MSPI1_D3);
    am_hal_gpio_pinconfig(45, g_AM_BSP_GPIO_MSPI1_SCK);
        am_hal_gpio_pinconfig(43, g_AM_GPIO_TE_43);
}

int32_t
am_devices_display_init(uint16_t ui16ResX,
                        uint16_t ui16ResY,
                        am_devices_disp_color_e eColorMode,
                        bool bEnableTE)
{

    //
    // store the user setting
    //
    if (ui16ResX < g_sDispCfg.ui16ResX)
    {
        sDispUserSetting.ui16ResX = ui16ResX;
    }
    else
    {
        sDispUserSetting.ui16ResX = g_sDispCfg.ui16ResX;
    }

    sDispUserSetting.ui16MinX = ((g_sDispCfg.ui16ResX - sDispUserSetting.ui16ResX) >> 2) << 1;

    if (ui16ResY < g_sDispCfg.ui16ResY)
    {
        sDispUserSetting.ui16ResY = ui16ResY;
    }
    else
    {
        sDispUserSetting.ui16ResY = g_sDispCfg.ui16ResY;
    }

    sDispUserSetting.ui16MinY = ((g_sDispCfg.ui16ResY - sDispUserSetting.ui16ResY) >> 2) << 1;

    sDispUserSetting.eColorMode   = eColorMode;
    sDispTransfer.bXferPending = false;
    sDispTransfer.bXferBusy    = false;

    //
    // check if the user would like to use TE
    //
    if (!bEnableTE)
    {
        g_sDispCfg.eTEType = DISP_TE_DISABLE;
    }

    //
    // Initialize the display specific GPIO signals.
    //
    am_bsp_disp_pins_enable1();

    //
    // Initialize MCU side
    //
#if (DISP_CTRL_IP == DISP_CTRL_IP_DC)

    uint32_t ui32MipiCfg = MIPICFG_16RGB888_OPT0;
    am_devices_dc_xspi_raydium_config_t sDisplayPanelConfig;
    nemadc_initial_config_t sDCConfig;
    sDCConfig.ui16ResX = sDispUserSetting.ui16ResX;
    sDCConfig.ui16ResY = sDispUserSetting.ui16ResY;
    sDCConfig.bTEEnable = (g_sDispCfg.eTEType == DISP_TE_DC);
    sDisplayPanelConfig.ui16ResX = sDispUserSetting.ui16ResX;
    sDisplayPanelConfig.ui16ResY = sDispUserSetting.ui16ResY;
    sDisplayPanelConfig.ui16MinX = sDispUserSetting.ui16MinX + g_sDispCfg.ui16Offset;
    sDisplayPanelConfig.ui16MinY = sDispUserSetting.ui16MinY;
    sDisplayPanelConfig.bTEEnable = (g_sDispCfg.eTEType != DISP_TE_DISABLE);
    sDisplayPanelConfig.bFlip = g_sDispCfg.bFlip;

    if ( g_sDispCfg.eInterface == DISP_IF_DSI )
    {
        //
        // VDD18 control callback function
        //
        am_hal_dsi_register_external_vdd18_callback(am_bsp_external_vdd18_switch);
        //
        // Enable DSI power and configure DSI clock.
        //
        am_hal_dsi_init();
    }
    else
    {
        am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_DISPCLKSEL_HFRC96, NULL);
        am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_DCCLK_ENABLE, NULL);
    }

    am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_DISP);

    //
    // Initialize Nema|dc
    //
    int32_t i32Status = nemadc_init();

    if (i32Status != 0)
    {
        return AM_DEVICES_DISPLAY_STATUS_DPCTR_ERR;
    }

    if (g_sDispCfg.eInterface == DISP_IF_DSI)
    {
        uint8_t ui8LanesNum = g_sDispCfg.ui8NumLanes;
        uint8_t ui8DbiWidth = g_sDispCfg.eDbiWidth;
        uint32_t ui32FreqTrim = g_sDispCfg.eDsiFreq;
        pixel_format_t eFormat = FMT_RGB888;
        if (sDispUserSetting.eColorMode == COLOR_FORMAT_RGB565)
        {
            eFormat = FMT_RGB565;
            if (g_sDispCfg.eDsiFreq > AM_HAL_DSI_FREQ_TRIM_X13)
            {
                return AM_DEVICES_DISPLAY_STATUS_OUT_OF_RANGE;
            }
        }
        if (sDispUserSetting.eColorMode == COLOR_FORMAT_RGB888)
        {
            eFormat = FMT_RGB888;
            if (g_sDispCfg.eDsiFreq > AM_HAL_DSI_FREQ_TRIM_X20)
            {
                return AM_DEVICES_DISPLAY_STATUS_OUT_OF_RANGE;
            }
        }

        if (am_hal_dsi_para_config(ui8LanesNum, ui8DbiWidth, ui32FreqTrim) != 0)
        {
            return AM_DEVICES_DISPLAY_STATUS_ERROR;
        }
        switch (eFormat)
        {
            case FMT_RGB888:
                if (ui8DbiWidth == 16)
                {
                    ui32MipiCfg = MIPICFG_16RGB888_OPT0;
                }
                if (ui8DbiWidth == 8)
                {
                    ui32MipiCfg = MIPICFG_8RGB888_OPT0;
                }
                break;

            case FMT_RGB565:
                if (ui8DbiWidth == 16)
                {
                    ui32MipiCfg = MIPICFG_16RGB565_OPT0;
                }
                if (ui8DbiWidth == 8)
                {
                    ui32MipiCfg = MIPICFG_8RGB565_OPT0;
                }
                break;

            default:
                //
                // invalid color component index
                //
                return AM_DEVICES_DISPLAY_STATUS_INVALID_ARG;
        }
    }
#endif    // (DISP_CTRL_IP == DISP_CTRL_IP_DC)

    //
    //  Initialize display panel side
    //
#if (DISP_CTRL_IP == DISP_CTRL_IP_DC)
    switch (g_sDispCfg.eInterface)
    {
        case DISP_IF_SPI4:
            am_devices_dc_xspi_raydium_hardware_reset();
            sDCConfig.eInterface = DISP_INTERFACE_SPI4;
            if (sDispUserSetting.eColorMode == COLOR_FORMAT_RGB888)
            {
                sDCConfig.ui32PixelFormat = MIPICFG_1RGB888_OPT0;
            }
            else if (sDispUserSetting.eColorMode == COLOR_FORMAT_RGB565)
            {
                sDCConfig.ui32PixelFormat = MIPICFG_1RGB565_OPT0;
            }
            else
            {
            }
            sDisplayPanelConfig.ui32PixelFormat = sDCConfig.ui32PixelFormat;
            nemadc_configure(&sDCConfig);
            am_devices_dc_xspi_raydium_init(&sDisplayPanelConfig);
            break;
        case DISP_IF_DSPI:
            am_devices_dc_xspi_raydium_hardware_reset();
            sDCConfig.eInterface = DISP_INTERFACE_DSPI;
            if (sDispUserSetting.eColorMode == COLOR_FORMAT_RGB888)
            {
                sDCConfig.ui32PixelFormat = MIPICFG_2RGB888_OPT0;
            }
            else if (sDispUserSetting.eColorMode == COLOR_FORMAT_RGB565)
            {
                sDCConfig.ui32PixelFormat = MIPICFG_2RGB565_OPT0;
            }
            else
            {
            }
            sDisplayPanelConfig.ui32PixelFormat = sDCConfig.ui32PixelFormat;
            nemadc_configure(&sDCConfig);
            am_devices_dc_xspi_raydium_init(&sDisplayPanelConfig);
            break;
        case DISP_IF_QSPI:
            am_devices_dc_xspi_raydium_hardware_reset();
            sDCConfig.eInterface = DISP_INTERFACE_QSPI;
            if (sDispUserSetting.eColorMode == COLOR_FORMAT_RGB888)
            {
                sDCConfig.ui32PixelFormat = MIPICFG_4RGB888_OPT0;
            }
            else if (sDispUserSetting.eColorMode == COLOR_FORMAT_RGB565)
            {
                sDCConfig.ui32PixelFormat = MIPICFG_4RGB565_OPT0;
            }
            else
            {
            }
            sDisplayPanelConfig.ui32PixelFormat = sDCConfig.ui32PixelFormat;
            nemadc_configure(&sDCConfig);
            am_devices_dc_xspi_raydium_init(&sDisplayPanelConfig);
            break;
        case DISP_IF_DSI:
            am_devices_dc_dsi_raydium_hardware_reset();
            sDCConfig.eInterface = DISP_INTERFACE_DBIDSI;
            sDCConfig.ui32PixelFormat = ui32MipiCfg;
            sDisplayPanelConfig.ui32PixelFormat = ui32MipiCfg;
            nemadc_configure(&sDCConfig);
            am_devices_dc_dsi_raydium_init((am_devices_dc_dsi_raydium_config_t *) &sDisplayPanelConfig);
            break;
        default:
            //
            // return error.
            //
            return AM_DEVICES_DISPLAY_STATUS_INVALID_ARG;
    }

#endif //(DISP_CTRL_IP == DISP_CTRL_IP_DC)

#if (DISP_CTRL_IP == DISP_CTRL_IP_MSPI)

    uint32_t ui32Status = 0;
    uint8_t ui8Format = AM_DEVICES_MSPI_RM69330_COLOR_MODE_16BIT;

    if (sDispUserSetting.eColorMode == COLOR_FORMAT_RGB888)
    {
        ui8Format = AM_DEVICES_MSPI_RM69330_COLOR_MODE_24BIT;
    }
    else if (sDispUserSetting.eColorMode == COLOR_FORMAT_RGB565)
    {
        ui8Format = AM_DEVICES_MSPI_RM69330_COLOR_MODE_16BIT;
    }
    //
    // modified default row, column and format parameters.
    //
    am_devices_rm69330_set_parameters(sDispUserSetting.ui16MinX + g_sDispCfg.ui16Offset,
                                      ui16ResX,
                                      sDispUserSetting.ui16MinY,
                                      ui16ResY,
                                      ui8Format);
    //
    // Initialize the MSPI Display
    //
    QuadDisplayMSPICfg.eClockFreq = g_sDispCfg.eClockFreq;
    QuadDisplayMSPICfg.eDeviceConfig = g_sDispCfg.eDeviceConfig;

    ui32Status = am_devices_mspi_rm69330_init(g_sDispCfg.ui32Module,
                                              &QuadDisplayMSPICfg,
                                              &g_DisplayHandle,
                                              &g_MSPIDisplayHandle);
    if (AM_DEVICES_MSPI_RM69330_STATUS_SUCCESS != ui32Status)
    {
        return AM_DEVICES_DISPLAY_STATUS_PANEL_ERR;
    }
    NVIC_SetPriority(mspi_display_interrupts[g_sDispCfg.ui32Module], 0x4);
    NVIC_EnableIRQ(mspi_display_interrupts[g_sDispCfg.ui32Module]);

    am_devices_mspi_rm69330_display_on(g_DisplayHandle);

#endif //(DISP_CTRL_IP == DISP_CTRL_IP_MSPI)

#ifdef BAREMETAL
    bXferDone      = false;
#else
    semXferDone    = xSemaphoreCreateBinary();
#endif
    //
    // Setting default scanline
    //
    am_devices_display_set_scanline_recommended_parameter(1);
    //
    // Enable GPIO TE interrupt
    //
    if ( g_sDispCfg.eTEType == DISP_TE_GPIO )
    {
        uint32_t IntNum = g_sDispCfg.ui16TEpin;
        am_hal_gpio_mask_t gpio_mask = AM_HAL_GPIO_MASK_DECLARE_ZERO;
        gpio_mask.U.Msk[GPIO_NUM2IDX(IntNum)] = GPIO_NUM2MSK(IntNum);
        am_hal_gpio_interrupt_clear(AM_HAL_GPIO_INT_CHANNEL_0, &gpio_mask);
        am_hal_gpio_interrupt_register(AM_HAL_GPIO_INT_CHANNEL_0,
                                        IntNum,
                                       (am_hal_gpio_handler_t)am_devices_display_te_handler,
                                        NULL);
        am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_0, AM_HAL_GPIO_INT_CTRL_INDV_ENABLE, (void *)&IntNum);
        NVIC_SetPriority(te_interrupts[TE_GPIO_IDX], 0x4);
        NVIC_EnableIRQ(te_interrupts[TE_GPIO_IDX]);
    }

#if (DISP_CTRL_IP == DISP_CTRL_IP_DC)
    if (g_sDispCfg.eTEType == DISP_TE_DC)
    {
        nemadc_set_te_interrupt_callback(am_devices_display_te_handler);
    }
#endif // (DISP_CTRL_IP == DISP_CTRL_IP_DC)

    return AM_DEVICES_DISPLAY_STATUS_SUCCESS;
}


//*****************************************************************************
//
// Set display region.
//
//*****************************************************************************
uint32_t
am_devices_display_set_region(uint16_t ui16ResX,
                              uint16_t ui16ResY,
                              uint16_t ui16MinX,
                              uint16_t ui16MinY)
{
    uint32_t ui32Status = 0;

    //
    // panel offset
    //
    ui16MinX += g_sDispCfg.ui16Offset;

#if (DISP_CTRL_IP == DISP_CTRL_IP_DC)
    switch (g_sDispCfg.eInterface) // TODO
    {
        case DISP_IF_SPI4:
        case DISP_IF_DSPI:
        case DISP_IF_QSPI:
            am_devices_dc_xspi_raydium_set_region(ui16ResX,
                                                  ui16ResY,
                                                  ui16MinX,
                                                  ui16MinY);
            nemadc_timing(ui16ResX, 4, 10, 10,
                          ui16ResY, 10, 50, 10);
            break;
        case DISP_IF_DSI:
            am_devices_dc_dsi_raydium_set_region(ui16ResX,
                                                 ui16ResY,
                                                 ui16MinX,
                                                 ui16MinY);
            nemadc_timing(ui16ResX, 4, 10, 1,
                          ui16ResY, 1, 1, 1);
            break;
        default:
            return AM_DEVICES_DISPLAY_STATUS_INVALID_ARG;
    }
#endif //(DISP_CTRL_IP == DISP_CTRL_IP_DC)

#if (DISP_CTRL_IP == DISP_CTRL_IP_MSPI)
    ui32Status = am_devices_mspi_rm69330_set_transfer_window(g_DisplayHandle,
                                                             ui16MinX,
                                                             ui16ResX,
                                                             ui16MinY,
                                                             ui16ResY);
#endif //(DISP_CTRL_IP == DISP_CTRL_IP_MSPI)
    return ui32Status;
}

//*****************************************************************************
//
//! @brief MSPI transfer framebuffer completed Callbacks function
//!
//! @param pCallbackCtxt
//! @param transactionStatus
//!
//! MSPI transfer framebuffer completed Callbacks function
//
//*****************************************************************************
static void
display_transfer_complete(void *pCallbackCtxt, uint32_t transactionStatus)
{
    sDispTransfer.bXferBusy = false;

    if (sDispTransfer.fnXferDoneCb)
    {
        if ( sDispTransfer.total_stripe == 0 )
        {
            sDispTransfer.fnXferDoneCb(sDispTransfer.pArgXferDone);
        }
        else
        {
#if (DISP_CTRL_IP == DISP_CTRL_IP_DC)
            for ( uint32_t i = 0; i < sDispTransfer.total_stripe; i++ )
            {
                sDispTransfer.fnXferDoneCb((void*)i);
            }
#else
            sDispTransfer.fnXferDoneCb(pCallbackCtxt);
#endif
        }
    }

#ifdef BAREMETAL
    bXferDone = true;
#else
    xSemaphoreGiveFromISR(semXferDone, NULL);
#endif
}
//*****************************************************************************
//
//! @brief prepare for frame transfer
//!
//! @param ui16ResX     - X resolution of frame buffer/layer
//! @param ui16ResY     - Y resolution of frame buffer/layer
//! @param ui32Address  - Address of frame buffer
//! @param fnXferDoneCb - Transfer done callback
//! @param pArgXferDone - Transfer done argument
//! @param bContinue    - continue transfer (use COMMAND MEM_WRITE_CONTINUE)
//!
//! This function does the neccessary steps for frame transfer
//
//*****************************************************************************
static uint32_t
am_devices_display_prepare_transfer(void)
{
#if (DISP_CTRL_IP == DISP_CTRL_IP_DC)
    nemadc_transfer_frame_prepare(g_sDispCfg.eTEType != DISP_TE_DISABLE);

#if !defined(USE_NEMAGUI_LIB)
    //
    // Update the DC layer information
    //
    sDefaultDCLayer.sizex = sDefaultDCLayer.resx = sDispTransfer.ui16XferResX;
    sDefaultDCLayer.sizey = sDefaultDCLayer.resy = sDispTransfer.ui16XferResY;
    if (sDispUserSetting.eColorMode == COLOR_FORMAT_RGB565)
    {
        sDefaultDCLayer.format = NEMADC_RGB565;
        sDefaultDCLayer.stride = 2*sDispTransfer.ui16XferResX;
    }
    else
    {
#if defined (AM_PART_APOLLO4B)
        sDefaultDCLayer.format = NEMADC_RGBA8888;
        sDefaultDCLayer.stride = 4*sDispTransfer.ui16XferResX;
#else
        sDefaultDCLayer.format = NEMADC_RGB24;
        sDefaultDCLayer.stride = 3*sDispTransfer.ui16XferResX;
#endif
    }
    sDefaultDCLayer.baseaddr_virt = (void*)sDispTransfer.ui32XferAddress;
    sDefaultDCLayer.baseaddr_phys = (uintptr_t)sDispTransfer.ui32XferAddress;

    nemadc_set_layer(0, &sDefaultDCLayer);
#endif

    if (g_sDispCfg.eInterface == DISP_IF_DSI)
    {
        nemadc_timing(sDispTransfer.ui16XferResX, 4, 10, 1,
                      sDispTransfer.ui16XferResY, 1, 1, 1);
    }
    else
    {
        nemadc_timing(sDispTransfer.ui16XferResX, 4, 10, 10,
                      sDispTransfer.ui16XferResY, 10, 50, 10);
    }

    nemadc_set_vsync_interrupt_callback(display_transfer_complete, (void*)0);
#endif // DISP_CTRL_IP == DISP_CTRL_IP_DC

    return AM_DEVICES_DISPLAY_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Wait for transfer completion
//
//*****************************************************************************
uint32_t
am_devices_display_wait_transfer_done()
{
#ifdef BAREMETAL
    if (AM_HAL_STATUS_SUCCESS == am_hal_delay_us_status_change(XFER_MAX_WAIT_MS*1000,
                                                               (uint32_t)&bXferDone,
                                                               0xFF,
                                                               true))
    {
        bXferDone = false;
    }
    else
    {
        return AM_DEVICES_DISPLAY_STATUS_ERROR;
    }

#else
    if (pdFALSE == xSemaphoreTake(semXferDone, XFER_MAX_WAIT_MS * portTICK_PERIOD_MS))
    {
        return AM_DEVICES_DISPLAY_STATUS_ERROR;
    }
#endif

    return AM_DEVICES_DISPLAY_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief launch frame transfer
//!
//! This function is used to launch frame transfer.
//!
//! @return status.
//
//*****************************************************************************
static uint32_t
am_devices_display_launch_transfer()
{
    uint32_t ui32Status = AM_DEVICES_DISPLAY_STATUS_SUCCESS;

#if (DISP_CTRL_IP == DISP_CTRL_IP_DC)
    nemadc_transfer_frame_launch();
#endif //(DISP_CTRL_IP == DISP_CTRL_IP_DC)

#if (DISP_CTRL_IP == DISP_CTRL_IP_MSPI)
    //
    // If sDispTransfer.total_stripe is not set, we will send the frame buffer as a whole part, while
    // if the sDispTransfer.total_stripe is set, we will divided the frame buffer in several parts
    // with the same size.
    //
    uint32_t parts = (sDispTransfer.total_stripe == 0) ? 1 : sDispTransfer.total_stripe;
    uint32_t pixel_size = (sDispUserSetting.eColorMode == COLOR_FORMAT_RGB565) ? 2 : 3;
    uint32_t part_size = sDispTransfer.ui16XferResX *
                         (sDispTransfer.ui16XferResY / parts) *
                         pixel_size;

    for ( uint32_t i = 0; i < parts; i++ )
    {
        uint8_t* ptr = (uint8_t*)sDispTransfer.ui32XferAddress + i*part_size;

#ifdef USE_DEBUG_PIN_IN_GUI_TASK
        am_hal_gpio_output_set((i == 0)?DEBUG_PIN_4:DEBUG_PIN_5);
#endif
        //
        // reverse endian of half-word when color format is RGB565.
        //
        ui32Status = am_devices_mspi_rm69330_nonblocking_write_endian(
                        g_DisplayHandle,
                        ptr,
                        part_size,
                        0,
                        0,
                        (am_hal_mspi_callback_t)display_transfer_complete,
                        (void*)i,
                        (i != 0),
                        sDispUserSetting.eColorMode == COLOR_FORMAT_RGB565);

        if (ui32Status)
        {
            break;
        }
    }
#endif //(DISP_CTRL_IP == DISP_CTRL_IP_MSPI)

    return ui32Status;
}

//*****************************************************************************
//
//! @brief Common transfer operations
//!
//! This function contains the common operations of am_devices_display_transfer_frame
//! and am_devices_display_transfer_frame_by_stripe APIs.
//!
//! @return status.
//
//*****************************************************************************
static uint32_t
am_devices_display_transfer_frame_inter(void)
{
    //
    // prepare for the transfer
    //
    am_devices_display_prepare_transfer();


    if (g_sDispCfg.eTEType == DISP_TE_DISABLE)
    {
        //
        // If TE is not used, trigger the transfer here
        //
        return am_devices_display_launch_transfer();
    }
    else
    {
        //
        // If TE is used, transfer will be triggerd in TE ISR.
        //
        sDispTransfer.bXferPending    = true;
    }

    return AM_DEVICES_DISPLAY_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Send one frame via MSPI interface, or via DC with only one layer
//
//*****************************************************************************
uint32_t
am_devices_display_transfer_frame(uint16_t ui16ResX,
                                  uint16_t ui16ResY,
                                  uint32_t ui32Address,
                                  am_devices_disp_handler_t fnXferDoneCb,
                                  void* pArgXferDone)
{
    //
    // Check busy, this API may be used in a multithread envionment.
    //
    if ( sDispTransfer.bXferBusy )
    {
        return AM_DEVICES_DISPLAY_STATUS_TRY_AGAIN;
    }

    //
    // Record the transfer setting
    //
    sDispTransfer.bXferBusy       = true;
    sDispTransfer.bXferPending    = false;
    sDispTransfer.ui16XferResX    = ui16ResX;
    sDispTransfer.ui16XferResY    = ui16ResY;
    sDispTransfer.ui32XferAddress = ui32Address;
    sDispTransfer.fnXferDoneCb    = fnXferDoneCb;
    sDispTransfer.pArgXferDone    = pArgXferDone;
    sDispTransfer.total_stripe    = 0;

    return am_devices_display_transfer_frame_inter();
}


//*****************************************************************************
//
// transfer one frame stripe by stripe
//
//*****************************************************************************
uint32_t
am_devices_display_transfer_frame_by_stripe(uint16_t ui16ResX,
                                            uint16_t ui16ResY,
                                            uint32_t ui32Address,
                                            uint32_t total_stripe,
                                            am_devices_disp_handler_t fnStripeDoneCb)
{
    //
    // Check busy, this API may be used in a multithread envionment.
    //
    if ( sDispTransfer.bXferBusy )
    {
        return AM_DEVICES_DISPLAY_STATUS_TRY_AGAIN;
    }

    //
    // Record the transfer setting
    //
    sDispTransfer.bXferBusy       = true;
    sDispTransfer.bXferPending    = false;
    sDispTransfer.ui16XferResX    = ui16ResX;
    sDispTransfer.ui16XferResY    = ui16ResY;
    sDispTransfer.ui32XferAddress = ui32Address;
    sDispTransfer.fnXferDoneCb    = fnStripeDoneCb;
    sDispTransfer.pArgXferDone    = NULL;
    sDispTransfer.total_stripe    = total_stripe;

    return am_devices_display_transfer_frame_inter();
}

//*****************************************************************************
//
// DC and display panel de-initialization function.
//
//*****************************************************************************
int32_t
am_devices_display_generic_deinit(void)
{
    int32_t i32Status = 0;

#if (DISP_CTRL_IP == DISP_CTRL_IP_MSPI)
    am_devices_mspi_rm69330_term(g_DisplayHandle);
#endif

    return i32Status;
}
//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

