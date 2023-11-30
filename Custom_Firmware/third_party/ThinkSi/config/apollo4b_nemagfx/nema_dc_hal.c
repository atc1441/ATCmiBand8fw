// -----------------------------------------------------------------------------
// Copyright (c) 2019 Think Silicon S.A.
// Think Silicon S.A. Confidential Proprietary
// -----------------------------------------------------------------------------
//     All Rights reserved - Unpublished -rights reserved under
//         the Copyright laws of the European Union
//
//  This file includes the Confidential information of Think Silicon S.A.
//  The receiver of this Confidential Information shall not disclose
//  it to any third party and shall protect its confidentiality by
//  using the same degree of care, but not less than a reasonable
//  degree of care, as the receiver uses to protect receiver's own
//  Confidential Information. The entire notice must be reproduced on all
//  authorised copies and copies may only be made to the extent permitted
//  by a licensing agreement from Think Silicon S.A..
//
//  The software is provided 'as is', without warranty of any kind, express or
//  implied, including but not limited to the warranties of merchantability,
//  fitness for a particular purpose and noninfringement. In no event shall
//  Think Silicon S.A. be liable for any claim, damages or other liability, whether
//  in an action of contract, tort or otherwise, arising from, out of or in
//  connection with the software or the use or other dealings in the software.
//
//
//                    Think Silicon S.A.
//                    http://www.think-silicon.com
//                    Patras Science Park
//                    Rion Achaias 26504
//                    Greece
// -----------------------------------------------------------------------------

#include "stdint.h"
#include "nema_dc_regs.h"
#include "nema_dc_hal.h"
#include "nema_dc_mipi.h"
#include "nema_dc_intern.h"
#include "nema_dc.h"
#include "nema_dc_dsi.h"
#include "nema_ringbuffer.h"
#include "nema_sys_defs.h"
#include "am_mcu_apollo.h"
#include "am_util_delay.h"

#ifdef BAREMETAL

#  ifndef WAIT_IRQ_POLL
#    define WAIT_IRQ_POLL              1
#  endif

#  ifdef WAIT_IRQ_BINARY_SEMAPHORE
#    warning semaphore will not work under baremetal
#    undef WAIT_IRQ_BINARY_SEMAPHORE
#    define WAIT_IRQ_BINARY_SEMAPHORE  0
#  endif

#else  // BAREMETAL

#ifdef SYSTEM_VIEW
#  include "SEGGER_SYSVIEW_FreeRTOS.h"
#endif

#  include "FreeRTOS.h"
#  include "task.h"

#  ifndef WAIT_IRQ_POLL
#    define WAIT_IRQ_POLL               0
#  endif

#  ifndef WAIT_IRQ_BINARY_SEMAPHORE
#    define WAIT_IRQ_BINARY_SEMAPHORE   1
#  endif

#  if WAIT_IRQ_BINARY_SEMAPHORE
#    include "semphr.h"
     static SemaphoreHandle_t xSemaphore_vsync = NULL;
     static SemaphoreHandle_t xSemaphore_TE = NULL;
#  endif

#endif // BAREMETAL

#ifndef NEMADC_BASEADDR
#include "apollo4b.h"
#define NEMADC_BASEADDR       DC_BASE
#endif

// IRQ number
#ifndef NEMADC_IRQ
#define NEMADC_IRQ           ((IRQn_Type)29U)
#endif

// IRQ handler
//#define prvVsyncInterruptHandler        am_dc_isr

static uintptr_t nemadc_regs = 0;

volatile int irq_count = 0;

//
// declaration of display controller interrupt callback function.
//

nema_dc_interrupt_callback  nemadc_te_cb = NULL;
nema_dc_interrupt_callback  nemadc_vsync_cb = NULL;
static void* vsync_arg;
static bool bNeedLaunchInTe = false;

//*****************************************************************************
//
//! @brief DC's TE interrupt callback initialize function
//!
//! @param  fnTECallback                - DC TE interrupt callback function
//!
//! this function used to initialize display controller te interrupt
//! callback function.
//!
//! @return None.
//
//*****************************************************************************
void
nemadc_set_te_interrupt_callback(nema_dc_interrupt_callback fnTECallback)
{
    nemadc_te_cb = fnTECallback;
}

//*****************************************************************************
//
//! @brief DC's vsync interrupt callback initialize function
//!
//! @param  fnVsyncCallback - DC Vsync interrupt callback function
//! @param  arg             - DC Vsync interrupt callback argument
//!
//! this function used to initialize display controller vsync interrupt
//! callback function.
//!
//! @return None.
//
//*****************************************************************************
void
nemadc_set_vsync_interrupt_callback(nema_dc_interrupt_callback fnVsyncCallback, void* arg)
{
    nemadc_vsync_cb = fnVsyncCallback;
    vsync_arg = arg;
}

//*****************************************************************************
//
//! @brief wait NemaDC to become idle
//!
//! Wait NemaDC to become idle.
//!
//! @return None.
//
//*****************************************************************************
void
wait_dbi_idle(void)
{
    uint32_t ui32usMaxDelay = 100000;

    //
    // wait NemaDC to become idle
    //
    am_hal_delay_us_status_change(ui32usMaxDelay, (uint32_t)&DC->STATUS, DC_STATUS_dbi_busy, 0);
//
// TODO RTOS support
//
}

//*****************************************************************************
//
//! @brief Configure NemaDC.
//!
//! @param  psDCConfig     - NemaDC configuration structure.
//!
//! This function configures NemaDC display interface, output color format,
//! clock settings, TE setting and timing.
//!
//! @return None.
//
//*****************************************************************************
void
nemadc_configure(nemadc_initial_config_t *psDCConfig)
{

    uint32_t cfg = 0;
    uint32_t ui32Mode = 0;
    if (psDCConfig->eInterface == DISP_INTERFACE_DBIDSI)
    {
        //
        // Enable dc clock
        //
        nemadc_reg_write(NEMADC_REG_CLKCTRL_CG, NemaDC_clkctrl_cg_clk_en);

        //
        // Set clock divider. B2 and later versions of Apollo4 support setting DC primary clock divide ratio to 1.
        //
        if (APOLLO4_GE_B2)
        {
            nemadc_clkdiv(1, 1, 4, 0);
        }
        else
        {
            nemadc_clkdiv(2, 1, 4, 0);
        }
        //
        // Enable fast pixel generation slow transfer
        //
        if (APOLLO4_GE_B2)
        {
            nemadc_reg_write(NEMADC_REG_CLKCTRL_CG,
                            (NemaDC_clkctrl_cg_clk_swap |
                             NemaDC_clkctrl_cg_l0_bus_clk |
                             NemaDC_clkctrl_cg_clk_en));
        }
        else
        {
            nemadc_reg_write(NEMADC_REG_CLKCTRL_CG,
                            (NemaDC_clkctrl_cg_clk_swap |
                             NemaDC_clkctrl_cg_clk_en));
        }
        nemadc_clkctrl((uint32_t)TB_LCDPANEL_MIPI_DBIB );

        //
        // Program NemaDC MIPI interface
        //
        if (APOLLO4_GE_B2)
        {
            cfg = MIPICFG_DBI_EN | MIPICFG_RESX | MIPICFG_EXT_CTRL | MIPICFG_EN_STALL | MIPICFG_PIXCLK_OUT_EN | psDCConfig->ui32PixelFormat;
            if (!psDCConfig->bTEEnable)
            {
                cfg |= MIPICFG_DIS_TE;
            }
            nemadc_MIPI_CFG_out(cfg);
        }
        else
        {
            cfg = MIPICFG_DBI_EN | MIPICFG_RESX | MIPICFG_EXT_CTRL | MIPICFG_PIXCLK_OUT_EN | psDCConfig->ui32PixelFormat;
            if (!psDCConfig->bTEEnable)
            {
                cfg |= MIPICFG_DIS_TE;
            }
            nemadc_MIPI_CFG_out(cfg);
        }
        //
        // Program NemaDC to transfer a resx*resy region
        //
        nemadc_timing(psDCConfig->ui16ResX, 4, 10, 1,
                      psDCConfig->ui16ResY, 1, 1, 1);
    }

    if ((psDCConfig->eInterface == DISP_INTERFACE_QSPI) ||
        (psDCConfig->eInterface == DISP_INTERFACE_DSPI) ||
        (psDCConfig->eInterface == DISP_INTERFACE_SPI4))
    {
        // ui8SPIMode = (ui32Mode &(MIPICFG_QSPI | MIPICFG_DSPI)) >> 9U;
        if (psDCConfig->eInterface == DISP_INTERFACE_QSPI)
        {
            ui32Mode = MIPICFG_QSPI | MIPICFG_SPI4;
        }
        else if (psDCConfig->eInterface == DISP_INTERFACE_DSPI)
        {
            ui32Mode = MIPICFG_DSPI | MIPICFG_SPI4;
        }
        else if (psDCConfig->eInterface == DISP_INTERFACE_SPI4)
        {
            ui32Mode = MIPICFG_SPI4;
        }
        cfg = ui32Mode | MIPICFG_SPI_CSX_V | MIPICFG_DBI_EN | MIPICFG_RESX | psDCConfig->ui32PixelFormat;
        if(!psDCConfig->bTEEnable)
        {
            cfg |= MIPICFG_DIS_TE;
        }
        nemadc_MIPI_CFG_out(cfg);

        // Program NemaDC to transfer a resx*resy region
        nemadc_timing(psDCConfig->ui16ResX, 4, 10, 10,
                      psDCConfig->ui16ResY, 10, 50, 10);
    }
}

//*****************************************************************************
//
//! @brief Prepared operations before sending frame
//!
//! @param  bAutoLaunch    - true:launch transfer in DC TE interrupt implicitly.
//!
//! This function configures clock gating, sends MIPI_write_memory_start
//! command before sending frame. If DBIDSI interface is selected, this function
//! also configures HS/LP mode and data/command type of DSI.
//! Note: bLaunchInTE taks effect in the DC TE interrupt handler, which means
//! if GPIO TE is used or TE signal is ignored, setting this parameter to true or false
//! makes no difference, user still need to call nemadc_transfer_frame_launch manually.
//!
//! @return None.
//
//*****************************************************************************
void
nemadc_transfer_frame_prepare(bool bAutoLaunch)
{
    uint32_t ui32Cfg = 0;
    ui32Cfg = nemadc_reg_read(NEMADC_REG_DBIB_CFG);
    uint32_t ui32MemWrCmd = MIPI_write_memory_start;

    if (((ui32Cfg & MIPICFG_DBI_EN) != 0) &&
        ((ui32Cfg & (MIPICFG_SPI3 | MIPICFG_SPI4 | MIPICFG_DSPI | MIPICFG_QSPI | MIPICFG_DSPI_SPIX )) == 0))
    {
        uint32_t ui32Gpio = nemadc_reg_read(NEMADC_REG_GPIO);

        //
        // High Speed
        //
        nemadc_reg_write(NEMADC_REG_GPIO, ui32Gpio & (~0x1));
        //
        // disable clock gating
        //
        nemadc_reg_write(NEMADC_REG_CLKCTRL_CG, 0xFFFFFFF5U);

        wait_dbi_idle();
        //
        // Set data/commands command type
        //
        nemadc_dsi_ct((uint32_t)0, // Unused parameter
                      (uint32_t)0, // Unused parameter
                      NemaDC_dcs_datacmd);
        //
        // Set scan-line (DCS) command
        //
        nemadc_MIPI_out(MIPI_DBIB_CMD | MIPI_write_memory_continue | NemaDC_sline_cmd);

        uint32_t ui32DbiCfg = nemadc_reg_read(NEMADC_REG_DBIB_CFG);
        nemadc_MIPI_CFG_out(ui32DbiCfg | MIPICFG_SPI_HOLD);
        //
        // Send DCS write_memory_start command
        //
        nemadc_MIPI_out(MIPI_DBIB_CMD | ui32MemWrCmd);
        wait_dbi_idle();
    }

    if (((ui32Cfg & MIPICFG_QSPI) != 0) &&
        ((ui32Cfg & MIPICFG_SPI4) != 0) &&
        ((ui32Cfg & (MIPICFG_SPI3 | MIPICFG_DSPI | MIPICFG_DSPI_SPIX )) == 0))
    {
        //
        // disable clock gating
        //
        nemadc_reg_write(NEMADC_REG_CLKCTRL_CG, 0xAA000000U);
        // ui32DBICfg = nemadc_reg_read( NEMADC_REG_DBIB_CFG);
        nemadc_MIPI_CFG_out( ui32Cfg | MIPICFG_SPI_HOLD);
        nemadc_MIPI_out    ( MIPI_DBIB_CMD | MIPI_MASK_QSPI | CMD1_DATA4);
        nemadc_MIPI_out    ( MIPI_DBIB_CMD | MIPI_MASK_QSPI | MIPI_CMD24 |
                           (ui32MemWrCmd << CMD_OFFSET));
    }
    if (((ui32Cfg & MIPICFG_DSPI) != 0) &&
        ((ui32Cfg & MIPICFG_SPI4) != 0) &&
        ((ui32Cfg & (MIPICFG_SPI3 | MIPICFG_QSPI | MIPICFG_DSPI_SPIX )) == 0))
    {
        //
        // disable clock gating
        //
        nemadc_reg_write(NEMADC_REG_CLKCTRL_CG, 0xAA000000U);
        // ui32DBICfg = nemadc_reg_read( NEMADC_REG_DBIB_CFG);
        nemadc_MIPI_CFG_out( ui32Cfg & (~MIPICFG_DSPI) & (~MIPICFG_QSPI));
        // Start MIPI Panel Memory Write
        nemadc_MIPI_out(MIPI_DBIB_CMD | ui32MemWrCmd);
        nemadc_MIPI_CFG_out(((ui32Cfg & (~MIPICFG_SPI4)) | MIPICFG_SPI3)
                           | MIPICFG_SPIDC_DQSPI | MIPICFG_SPI_HOLD);
    }
    if (((ui32Cfg & MIPICFG_SPI4) != 0) &&
        ((ui32Cfg & (MIPICFG_SPI3 | MIPICFG_DSPI | MIPICFG_QSPI | MIPICFG_DSPI_SPIX )) == 0))
    {
        //
        // disable clock gating
        //
        nemadc_reg_write(NEMADC_REG_CLKCTRL_CG, 0xAA000000U);
        // Start MIPI Panel Memory Write
        nemadc_MIPI_out(MIPI_DBIB_CMD | ui32MemWrCmd);
    }

    //
    // turn DC TE interrupt
    //
    if ((ui32Cfg & MIPICFG_DIS_TE) == 0)
    {
        nemadc_reg_write(NEMADC_REG_INTERRUPT, 1 << 3);
    }

    bNeedLaunchInTe = bAutoLaunch;
}

//*****************************************************************************
//
//! @brief Launch frame transfer
//!
//! This function enables DC frame end interrupt and launches frame transfer.
//!
//! @return None.
//
//*****************************************************************************
void
nemadc_transfer_frame_launch()
{
    uint32_t ui32Cfg = 0;
    ui32Cfg = nemadc_reg_read(NEMADC_REG_DBIB_CFG);
    if (((ui32Cfg & MIPICFG_DBI_EN) != 0) &&
        ((ui32Cfg & (MIPICFG_SPI3 | MIPICFG_SPI4 | MIPICFG_DSPI | MIPICFG_QSPI | MIPICFG_DSPI_SPIX )) == 0))
    {
        //
        // Enable frame end interrupt
        //
        nemadc_reg_write(NEMADC_REG_INTERRUPT, 1 << 4);
        nemadc_set_mode(NEMADC_ONE_FRAME | NEMADC_OUTP_OFF);
    }
    if (((ui32Cfg & (MIPICFG_SPI3 | MIPICFG_SPI4 | MIPICFG_DSPI | MIPICFG_QSPI | MIPICFG_DSPI_SPIX)) != 0))
    {
        //
        // Enable frame end interrupt
        //
        nemadc_reg_write(NEMADC_REG_INTERRUPT, 1 << 4);
        //
        // Send One Frame
        //
        nemadc_set_mode(NEMADC_ONE_FRAME);
    }
}


//*****************************************************************************
//
//! @brief Required operations after completing DSI frame/command transfer
//!
//! @param  ui32Cmd        - DBIB configurations
//!
//! This function includes a workaroud for B2 and previous versions, sets
//! NEMADC_REG_DBIB_CFG to normal mode after completing frame/command transfer.
//! This function is callled by nemadc_transfer_frame_end and DSI command
//! sending functions.
//!
//! @return None.
//
//****************************************************************************
static inline void
finish_cmd_transfer(uint32_t ui32Cmd)
{
    uint32_t ui32Val6 = 0;
    //
    // B2 and previous versions need this work around for FSV-396 (ERR074).
    //
    if (!APOLLO4_GT_B2)
    {
        am_hal_delay_us(20);
        ui32Val6 = CLKGEN->DISPCLKCTRL_b.DISPCLKSEL;
        am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_DISPCLKSEL_OFF, NULL);
    }
    nemadc_MIPI_CFG_out(ui32Cmd);
    if (!APOLLO4_GT_B2)
    {
        CLKGEN->DISPCLKCTRL_b.DISPCLKSEL = ui32Val6;
        am_hal_delay_us(10);
    }
}

//*****************************************************************************
//
//! @brief Required operations after completing frame transfer
//!
//! This function sets NemaDC back to normal mode after completing frame
//! transfer. This function should be called in frame end interrupt.
//!
//! @return None.
//
//****************************************************************************
static void
nemadc_transfer_frame_end()
{
    uint32_t ui32Cfg = 0;
    ui32Cfg = nemadc_reg_read(NEMADC_REG_DBIB_CFG);

    if (((ui32Cfg & MIPICFG_DBI_EN) != 0) &&
        ((ui32Cfg & (MIPICFG_SPI3 | MIPICFG_SPI4 | MIPICFG_DSPI | MIPICFG_QSPI | MIPICFG_DSPI_SPIX )) == 0))
    {
        uint32_t ui32Gpio = nemadc_reg_read(NEMADC_REG_GPIO);

        finish_cmd_transfer(ui32Cfg & (~MIPICFG_SPI_HOLD));

        nemadc_reg_write(NEMADC_REG_CLKCTRL_CG, 0x5); // enable clock gating
        nemadc_reg_write(NEMADC_REG_GPIO, ui32Gpio | 0x1); // LP
    }

    if (((ui32Cfg & MIPICFG_QSPI) != 0) &&
        ((ui32Cfg & MIPICFG_SPI4) != 0) &&
        ((ui32Cfg & (MIPICFG_SPI3 | MIPICFG_DSPI | MIPICFG_DSPI_SPIX )) == 0))
    {
        nemadc_MIPI_CFG_out(ui32Cfg & (~MIPICFG_SPI_HOLD));
        nemadc_reg_write(NEMADC_REG_CLKCTRL_CG, 0);
    }
    if (((ui32Cfg & MIPICFG_DSPI) != 0) &&
        ((ui32Cfg & MIPICFG_SPI4) != 0) &&
        ((ui32Cfg & (MIPICFG_SPI3 | MIPICFG_QSPI | MIPICFG_DSPI_SPIX )) == 0))
    {
        nemadc_MIPI_CFG_out((ui32Cfg | MIPICFG_SPI4) & (~MIPICFG_SPI3)
                            & (~MIPICFG_SPIDC_DQSPI) & (~MIPICFG_SPI_HOLD));
        nemadc_reg_write(NEMADC_REG_CLKCTRL_CG, 0);
    }
    if (((ui32Cfg & MIPICFG_SPI4) != 0) &&
        ((ui32Cfg & (MIPICFG_SPI3 | MIPICFG_DSPI | MIPICFG_QSPI | MIPICFG_DSPI_SPIX )) == 0))
    {
        //
        // enable clock gating
        //
        nemadc_reg_write(NEMADC_REG_CLKCTRL_CG, 0);
    }

}

//*****************************************************************************
//
//! @brief  Send DSI DCS(Display Command Set) write commands
//!
//! @param  ui8Cmd        - command
//! @param  pui8Para      - pointer of parameters to be sent
//! @param  ui8ParaLen    - length of parameters to be sent
//! @param  bHS           - high speed mode or low power mode (escape mode)
//!
//! This function sends DSI DCS commands to raydium's DSI interface IC rm67162 or rm69330.
//!
//! @return None.
//
//****************************************************************************
static void
dsi_dcs_write(uint8_t ui8Cmd, uint8_t* pui8Para, uint8_t ui8ParaLen, bool bHS)
{
    uint32_t ui32Mode = 0;
    uint32_t ui32Gpio = nemadc_reg_read(NEMADC_REG_GPIO);
    //
    // B2 supports setting DC primary clock divide ratio to 1 when sending frame,
    // but it only supports setting DC primary clock divide ratio to 2 when sending commands.
    //
    if (APOLLO4_B2)
    {
        nemadc_clkdiv(2, 1, 4, 0);
    }

    if (bHS == true) // high speed mode
    {
        nemadc_reg_write(NEMADC_REG_GPIO, ui32Gpio & (~0x1));
    }
    else // low power mode
    {
        nemadc_reg_write(NEMADC_REG_GPIO, ui32Gpio | 0x1);
    }

    nemadc_dsi_ct((uint32_t)0, // Unused parameter
                  (uint32_t)0, // Unused parameter
                   NemaDC_dcs_datacmd);
    wait_dbi_idle();
    //
    // enable command/parameters packing
    //
    ui32Mode = nemadc_reg_read(NEMADC_REG_DBIB_CFG);
    nemadc_MIPI_CFG_out(ui32Mode | MIPICFG_SPI_HOLD);
    //
    // Download command & parameter to DBI i/f
    //
    nemadc_MIPI_out(NemaDC_DBI_cmd | ui8Cmd);
    for (uint8_t ui8Index = 0; ui8Index < ui8ParaLen; ui8Index++)
    {
        nemadc_MIPI_out(pui8Para[ui8Index]);
    }
    //
    // Send command-parameter packet.
    //
    finish_cmd_transfer(ui32Mode);
    wait_dbi_idle();
    //
    // For B2, change DC primary clock ratio back to 1 after sending commands.
    //
    if (APOLLO4_B2)
    {
        nemadc_clkdiv(1, 1, 4, 0);
    }
}


//*****************************************************************************
//
//! @brief  Send DSI generic write command.
//!
//! @param  pui8Para        - pointer of parameters to be sent
//! @param  ui8ParaLen      - length of parameters to be sent
//! @param  bHS             - high speed mode or low power mode (escape mode)
//!
//! This function sends DSI generic commands to raydium's display IC rm69330 or rm67162.
//!
//! @return None.
//
//****************************************************************************
static void
dsi_generic_write(uint8_t* pui8Para, uint8_t ui8ParaLen, bool bHS)
{
    uint32_t ui32Mode = 0;
    uint32_t ui32Gpio = nemadc_reg_read(NEMADC_REG_GPIO);
    nemadc_reg_write(NEMADC_REG_GPIO, ui32Gpio & (~0x6)); // set vc_no to 00.
    ui32Gpio = nemadc_reg_read(NEMADC_REG_GPIO);
    //
    // B2 supports setting DC primary clock divide ratio to 1 when sending frame,
    // but it only supports setting DC primary clock divide ratio to 2 when sending commands.
    //
    if (APOLLO4_B2)
    {
        nemadc_clkdiv(2, 1, 4, 0);
    }

    if (bHS == true) //high speed mode
    {
        nemadc_reg_write(NEMADC_REG_GPIO, ui32Gpio & (~0x1));
    }
    else // low power mode
    {
        nemadc_reg_write(NEMADC_REG_GPIO, ui32Gpio | 0x1);
    }

    nemadc_dsi_ct((uint32_t)0,  // Unused parameter
                  (uint32_t)0,  // Unused parameter
                   NemaDC_ge_datacmd);
    wait_dbi_idle();

    //
    // enable command/parameters packing
    //
    ui32Mode = nemadc_reg_read(NEMADC_REG_DBIB_CFG);
    nemadc_MIPI_CFG_out(ui32Mode | MIPICFG_SPI_HOLD);
    //
    // Download command & parameter to DBI i/f
    //
    if ( ui8ParaLen == 0 )
    {
        nemadc_MIPI_out(NemaDC_DBI_cmd | 0x00);
    }
    else
    {
        for (uint8_t ui8Index = 0; ui8Index < ui8ParaLen; ui8Index++)
        {
            if ( ui8ParaLen < 3 )
            {
                nemadc_MIPI_out(NemaDC_DBI_cmd | pui8Para[ui8Index]);
            }
            else
            {
                nemadc_MIPI_out(pui8Para[ui8Index]);
            }
        }
    }
    //
    // Send command-parameter packet.
    //
    finish_cmd_transfer(ui32Mode);
    wait_dbi_idle();
    //
    // For B2, change DC primary clock ratio back to 1 after sending commands.
    //
    if (APOLLO4_B2)
    {
        nemadc_clkdiv(1, 1, 4, 0);
    }
}

//*****************************************************************************
//
//! @brief Send SPI4/DSPI/QSPI/DSI command via Display Controller(DC)
//!
//! @param  ui8Command      - command
//! @param  p_ui8Para       - pointer of parameters to be sent
//! @param  ui8ParaLen      - length of parameters to be sent
//! @param  bDCS            - DCS command or generic command
//! @param  bHS             - high speed mode or low power mode (escape mode)
//!
//! This function sends commands to display drive IC.
//!
//! @return None.
//
//****************************************************************************
void
nemadc_mipi_cmd_write(uint8_t ui8Command,
                      uint8_t* p_ui8Para,
                      uint8_t ui8ParaLen,
                      bool bDCS,
                      bool bHS)
{
    uint32_t ui32Cfg = 0;
    ui32Cfg = nemadc_reg_read(NEMADC_REG_DBIB_CFG);

    if (((ui32Cfg & MIPICFG_DBI_EN) != 0) &&
        ((ui32Cfg & (MIPICFG_SPI3 | MIPICFG_SPI4 | MIPICFG_DSPI | MIPICFG_QSPI | MIPICFG_DSPI_SPIX )) == 0))
    {
        if (bDCS)
        {
            dsi_dcs_write(ui8Command, p_ui8Para, ui8ParaLen, bHS);
        }
        else
        {
            dsi_generic_write(p_ui8Para, ui8ParaLen, bHS);
        }
    }
    if (((ui32Cfg & MIPICFG_QSPI) != 0) &&
        ((ui32Cfg & MIPICFG_SPI4) != 0) &&
        ((ui32Cfg & (MIPICFG_SPI3 | MIPICFG_DSPI | MIPICFG_DSPI_SPIX )) == 0))
    {   //QSPI
        uint32_t dbi_cfg = nemadc_reg_read(NEMADC_REG_DBIB_CFG);
        nemadc_MIPI_CFG_out(dbi_cfg | MIPICFG_SPI_HOLD);
        nemadc_MIPI_out(MIPI_DBIB_CMD | MIPI_MASK_QSPI | CMD1_DATA1);
        nemadc_MIPI_out(MIPI_DBIB_CMD | MIPI_MASK_QSPI | MIPI_CMD24 | (ui8Command << CMD_OFFSET));
        for (uint8_t i = 0; i < ui8ParaLen; i++ )
        {
            nemadc_MIPI_out(MIPI_DBIB_CMD | MIPI_MASK_QSPI | p_ui8Para[i]);
        }
        nemadc_MIPI_CFG_out(dbi_cfg);
    }
    if (((ui32Cfg & MIPICFG_DSPI) != 0) &&
        ((ui32Cfg & MIPICFG_SPI4) != 0) &&
        ((ui32Cfg & (MIPICFG_SPI3 | MIPICFG_QSPI | MIPICFG_DSPI_SPIX )) == 0))
    {   //DSPI
        uint32_t dbi_cfg = nemadc_reg_read(NEMADC_REG_DBIB_CFG);
        nemadc_MIPI_CFG_out(dbi_cfg & (~MIPICFG_DSPI) & (~MIPICFG_QSPI));
        nemadc_MIPI_out(MIPI_DBIB_CMD | ui8Command);
        for (uint8_t i = 0; i < ui8ParaLen; i++ )
        {
            nemadc_MIPI_out(p_ui8Para[i]);
        }
        nemadc_MIPI_CFG_out(dbi_cfg);
    }
    if (((ui32Cfg & MIPICFG_SPI4) != 0) &&
        ((ui32Cfg & (MIPICFG_SPI3 | MIPICFG_DSPI | MIPICFG_QSPI | MIPICFG_DSPI_SPIX )) == 0))
    {   //SPI4
        nemadc_MIPI_out(MIPI_DBIB_CMD | ui8Command);
        for (uint8_t i = 0; i < ui8ParaLen; i++ )
        {
            nemadc_MIPI_out(p_ui8Para[i]);
        }
    }

}


//*****************************************************************************
//
//! @brief  Send DSI DCS(Display Command Set) read commands
//!
//! @param  ui8Cmd        - command
//! @param  ui8DataLen    - length of data to be read
//! @param  bHS           - high speed mode or low power mode (escape mode)
//!
//! This function sends DSI DCS commands to raydium's DSI interface IC rm67162
//! or rm69330 then to read data from display panel.
//!
//! @return data.
//
//****************************************************************************
static uint32_t
dsi_dcs_read(uint8_t ui8Cmd, uint8_t ui8DataLen, bool bHS)
{
    uint32_t dc_cmd_field = 0x0;
    uint32_t ui32Mode = 0;
    uint32_t mask = 0;
    uint32_t ui32Gpio = nemadc_reg_read(NEMADC_REG_GPIO);
    //
    // set return packet size (bytes)
    //
    am_hal_dsi_set_return_size(ui8DataLen, bHS);
    //
    // B2 supports setting DC primary clock divide ratio to 1 when sending frame,
    // but it only supports setting DC primary clock divide ratio to 2 when sending commands.
    //
    if (APOLLO4_B2)
    {
        nemadc_clkdiv(2, 1, 4, 0);
    }

    if (bHS == true) // high speed mode
    {
        nemadc_reg_write(NEMADC_REG_GPIO, ui32Gpio & (~0x1));
    }
    else // low power mode
    {
        nemadc_reg_write(NEMADC_REG_GPIO, ui32Gpio | 0x1);
    }

    switch ( ui8DataLen )
    {
        case 1 :
            dc_cmd_field = 0x0;
            mask = 0xff;
            break;
        case 2 :
            dc_cmd_field = NemaDC_rcmd16;
            mask = 0xffff;
            break;
        case 3 :
            dc_cmd_field = NemaDC_rcmd24;
            mask = 0xffffff;
            break;
        case 4 :
            dc_cmd_field = NemaDC_rcmd32;
            mask = 0xffffffff;
            break;
        default :
            mask = 0xff;
            break;
    }

    nemadc_dsi_ct((uint32_t)0,  // Unused parameter
                  (uint32_t)0,  // Unused parameter
                   NemaDC_dcs_datacmd);
    wait_dbi_idle();

    //
    // enable command/parameters packing
    //
    ui32Mode = nemadc_reg_read(NEMADC_REG_DBIB_CFG);
    nemadc_MIPI_CFG_out(ui32Mode | MIPICFG_EN_DVALID | MIPICFG_SPI_HOLD);
    nemadc_reg_write(NEMADC_REG_DBIB_CMD, dc_cmd_field | NemaDC_DBI_cmd | NemaDC_DBI_read | ui8Cmd);
    //
    // Send command-parameter packet.
    //
    finish_cmd_transfer(ui32Mode | MIPICFG_EN_DVALID);

    wait_dbi_idle();
    nemadc_MIPI_CFG_out(ui32Mode);
    //
    // For B2, change DC primary clock ratio back to 1 after sending commands.
    //
    if (APOLLO4_B2)
    {
        nemadc_clkdiv(1, 1, 4, 0);
    }

    am_hal_dsi_set_return_size(1, bHS);
    //
    // return read parameters
    //
    return nemadc_reg_read(NEMADC_REG_DBIB_RDAT)&mask;
}

//*****************************************************************************
//
//! @brief  Send DSI generic read command.
//!
//! @param  p_ui8Para           - pointer of parameters to be sent
//! @param  ui8ParaLen          - length of parameters to be sent
//! @param  ui8DataLen          - length of data to be read
//! @param  bHS                 - high speed mode or low power mode (escape mode)
//!
//! This function sends DSI generic command to raydium's DSI interface IC rm67162
//! or rm69330 then to read data from display panel.
//!
//! @return data.
//
//****************************************************************************
static uint32_t
dsi_generic_read(uint8_t *p_ui8Para, uint8_t ui8ParaLen, uint8_t ui8DataLen, bool bHS)
{
    uint32_t dc_cmd_field = 0x0;
    uint32_t mask = 0;
    uint32_t ui32Mode = 0;
    uint32_t ui32Gpio = nemadc_reg_read(NEMADC_REG_GPIO);
    //
    // set return packet size (bytes)
    //
    am_hal_dsi_set_return_size(ui8DataLen, bHS);

    nemadc_reg_write(NEMADC_REG_GPIO, ui32Gpio & (~0x6)); // set vc_no to 00.
    ui32Gpio = nemadc_reg_read(NEMADC_REG_GPIO);
    //
    // B2 supports setting DC primary clock divide ratio to 1 when sending frame,
    // but it only supports setting this ratio to 2 when sending commands.
    //
    if (APOLLO4_B2)
    {
        nemadc_clkdiv(2, 1, 4, 0);
    }

    if (bHS == true) // high speed mode
    {
        nemadc_reg_write(NEMADC_REG_GPIO, ui32Gpio & (~0x1));
    }
    else // low power mode
    {
        nemadc_reg_write(NEMADC_REG_GPIO, ui32Gpio | 0x1);
    }

    switch ( ui8DataLen )
    {
        case 1 :
            dc_cmd_field = 0x0;
            mask = 0xff;
            break;
        case 2 :
            dc_cmd_field = NemaDC_rcmd16;
            mask = 0xffff;
            break;
        case 3 :
            dc_cmd_field = NemaDC_rcmd24;
            mask = 0xffffff;
            break;
        case 4 :
            dc_cmd_field = NemaDC_rcmd32;
            mask = 0xffffffff;
            break;
        default :
            mask = 0xff;
            break;
    }

    nemadc_dsi_ct((uint32_t)0,  // Unused parameter
                  (uint32_t)0,  // Unused parameter
                   NemaDC_ge_datacmd);
    //
    // wait NemaDC idle
    //
    wait_dbi_idle();

    //
    // send command
    //
    ui32Mode = nemadc_reg_read(NEMADC_REG_DBIB_CFG);
    nemadc_MIPI_CFG_out(ui32Mode | MIPICFG_EN_DVALID | MIPICFG_SPI_HOLD);

    if (ui8ParaLen == 0) // Generic read, no parameter
    {
        nemadc_reg_write(NEMADC_REG_DBIB_CMD, dc_cmd_field | NemaDC_DBI_cmd | NemaDC_ext_ctrl | NemaDC_DBI_read | 0x00);
    }
    else
    {
        for (uint32_t i = 0; i < ui8ParaLen; i++)
        {
            nemadc_reg_write(NEMADC_REG_DBIB_CMD, dc_cmd_field | NemaDC_DBI_cmd | NemaDC_ext_ctrl | NemaDC_DBI_read | p_ui8Para[i]);
        }
    }
    //
    // Send command-parameter packet.
    //
    finish_cmd_transfer(ui32Mode | MIPICFG_EN_DVALID);
    //
    // wait NemaDC idle
    //
    wait_dbi_idle();
    nemadc_MIPI_CFG_out(ui32Mode);
    //
    // For B2, change DC primary clock ratio back to 1 after sending commands.
    //
    if (APOLLO4_B2)
    {
        nemadc_clkdiv(1, 1, 4, 0);
    }

    am_hal_dsi_set_return_size(1, bHS);
    //
    // return read parameters
    //
    return nemadc_reg_read(NEMADC_REG_DBIB_RDAT)&mask;
}

//*****************************************************************************
//
//! @brief Send SPI4/DSPI/QSPI/DSI read command via Display Controller(DC)
//!
//! @param  ui8Command          - command
//! @param  p_ui8Para           - pointer of parameters to be sent
//! @param  ui8ParaLen          - length of parameters to be sent
//! @param  p_ui32Data          - pointer of data to be read
//! @param  ui8DataLen          - length of data to be read (number of bytes)
//! @param  bDCS                - DCS command or generic command
//! @param  bHS                 - high speed mode or low power mode (escape mode)
//!
//! This function sends read commands to display drive IC.
//!
//! @return Status.
//
//****************************************************************************
int32_t
nemadc_mipi_cmd_read(uint8_t ui8Command,
              uint8_t* p_ui8Para,
              uint8_t ui8ParaLen,
              uint32_t* p_ui32Data,
              uint8_t ui8DataLen,
              bool bDCS,
              bool bHS)
{
    uint32_t ui32Cfg = 0;
    ui32Cfg = nemadc_reg_read(NEMADC_REG_DBIB_CFG);

    if (((ui32Cfg & MIPICFG_DBI_EN) != 0) &&
        ((ui32Cfg & (MIPICFG_SPI3 | MIPICFG_SPI4 | MIPICFG_DSPI | MIPICFG_QSPI | MIPICFG_DSPI_SPIX )) == 0))
    {
        if (bDCS)
        {
            *p_ui32Data = dsi_dcs_read(ui8Command, ui8DataLen, bHS);
        }
        else
        {
            *p_ui32Data = dsi_generic_read(p_ui8Para, ui8ParaLen, ui8DataLen, bHS);
        }
    }
    else
    {   //QSPI/DSPI/SPI4
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
    return AM_HAL_STATUS_SUCCESS;
}


//*****************************************************************************
//
//! @brief DC's Vsync interrupt handle function
//!
//! @param  pvUnused            - invalid  parameter
//!
//! this function used to clear Vsync interrupt status bit.and execute callback.
//!
//! @return None.
//
//*****************************************************************************
static void 
prvVsyncInterruptHandler( void *pvUnused )
{
    ++irq_count;

    /* Clear the interrupt */
    nemadc_reg_write(NEMADC_REG_INTERRUPT, nemadc_reg_read(NEMADC_REG_INTERRUPT) & (~(1U << 4)));

    //
    // DC workarounds after transfer is finsihed
    //
    nemadc_transfer_frame_end();

    //
    // vsync interrupt callback function
    //
    if (nemadc_vsync_cb != NULL)
    {
        nemadc_vsync_cb(vsync_arg, 0);
    }
    else
    {
#  if WAIT_IRQ_BINARY_SEMAPHORE
        xSemaphoreGiveFromISR(xSemaphore_vsync, NULL);
#  endif
    }
}
//*****************************************************************************
//
//! @brief DC's TE interrupt handle function
//!
//! @param  pvUnused            - invalid  parameter
//!
//! this function used to clear TE interrupt status bit.and execute callback.
//!
//! @return None.
//
//*****************************************************************************
static void 
prvTEInterruptHandler( void *pvUnused )
{
    if (bNeedLaunchInTe)
    {
        nemadc_transfer_frame_launch();
        bNeedLaunchInTe = false;
    }
    ++irq_count;

    /* Clear the interrupt */
    nemadc_reg_write(NEMADC_REG_INTERRUPT, nemadc_reg_read(NEMADC_REG_INTERRUPT) & (~(1U << 3)));

    //
    // TE interrupt callback function
    //
    if (nemadc_te_cb != NULL)
    {
        nemadc_te_cb(NULL, 0);
    }
    else
    {
#if WAIT_IRQ_BINARY_SEMAPHORE
        xSemaphoreGiveFromISR(xSemaphore_TE, NULL);
#endif
    }
}

//*****************************************************************************
//
//! @brief Display Controller interrupt service function.
//!
//! this function will be called automatically when DC's interrupt(s) arrived.
//!
//! @return None.
//
//*****************************************************************************
void 
am_disp_isr()
{
#ifdef SYSTEM_VIEW
        traceISR_ENTER();
#endif

    if ((nemadc_reg_read(NEMADC_REG_INTERRUPT) & (1 << 4)) != 0)
    {
        prvVsyncInterruptHandler(NULL);
    }
    if ((nemadc_reg_read(NEMADC_REG_INTERRUPT) & (1 << 3)) != 0)
    {
        prvTEInterruptHandler(NULL);
    }

#ifdef SYSTEM_VIEW
    traceISR_EXIT();
#endif
}

//*****************************************************************************
//
//! @brief Display Controller initialize function.
//!
//! @return None.
//
//*****************************************************************************
int32_t 
nemadc_sys_init(void) {
    // xil_printf( "nemadc_sys_init()\r\n" );
    nemadc_regs = (uintptr_t)NEMADC_BASEADDR;

    /* Clear the interrupt */
    nemadc_reg_write(NEMADC_REG_INTERRUPT, 0);

    /* Install Interrupt Handler */
    NVIC_SetPriority(NEMADC_IRQ, AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(NEMADC_IRQ);

#if WAIT_IRQ_BINARY_SEMAPHORE
    if ((nemadc_vsync_cb == NULL) && (xSemaphore_vsync == NULL))
    {
        xSemaphore_vsync = xSemaphoreCreateBinary();
    }
    if (xSemaphore_TE == NULL)
    {
        xSemaphore_TE = xSemaphoreCreateBinary();
    }
#endif

    return 0;
}

//*****************************************************************************
//
//! @brief Wait for Vsync interrupt.
//!
//! this function invokes to wait Vsync interrupt.
//!
//! @return None.
//
//*****************************************************************************
void 
nemadc_wait_vsync(void)
{
    /* Wait for the interrupt */
#if WAIT_IRQ_POLL == 1
    uint32_t ui32usMaxDelay = 1000000; // 1 sec
    uint32_t ui32Status;
    //irq_handler sets NEMADC_REG_INTERRUPT to 0. Poll until this happens
    ui32Status = am_hal_delay_us_status_change(ui32usMaxDelay, (uint32_t)&DC->INTERRUPT, 1UL << 4, 0);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return;
    }
    //am_util_delay_ms(10);
#endif // WAIT_IRQ_POLL

#if WAIT_IRQ_BINARY_SEMAPHORE == 1
    xSemaphoreTake( xSemaphore_vsync, portMAX_DELAY );
#endif
}

//*****************************************************************************
//
//! @brief Wait for TE interrupt.
//!
//! this function invokes to Wait TE interrupt.
//!
//! @return None.
//
//*****************************************************************************
void 
nemadc_wait_te(void)
{
    /* Wait for the interrupt */
#if WAIT_IRQ_POLL == 1
    uint32_t ui32usMaxDelay = 100000;
    uint32_t ui32Status;
    //irq_handler sets NEMADC_REG_INTERRUPT to 0. Poll until this happens
    ui32Status = am_hal_delay_us_status_change(ui32usMaxDelay, (uint32_t)&DC->INTERRUPT, 1UL << 3, 0);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return;
    }
#endif // WAIT_IRQ_POLL

#if WAIT_IRQ_BINARY_SEMAPHORE == 1
    xSemaphoreTake( xSemaphore_TE, portMAX_DELAY );
#endif // WAIT_IRQ_BINARY_SEMAPHORE
}

//*****************************************************************************
//
//! @brief Read NemaDC Hardware Register
//!
//! @param  reg            - Register address
//!
//! this function invokes to Read DC register.
//!
//! @return value          - Register Value
//
//*****************************************************************************
uint32_t  
nemadc_reg_read(uint32_t reg) {
    volatile uint32_t *ptr = (volatile uint32_t *)(nemadc_regs + reg);
    return *ptr;
}

//*****************************************************************************
//
//! @brief Write NemaDC Hardware Register
//!
//! @param  reg            - Register address
//! @param  value            - Register Value
//!
//! this function invokes to write DC register.
//!
//! @return None.
//
//*****************************************************************************
void 
nemadc_reg_write(uint32_t reg, uint32_t value) {
    volatile uint32_t *ptr = (volatile uint32_t *)(nemadc_regs + reg);
    *ptr = value;
}

#if 0
//*****************************************************************************
//
//! @brief nemadc_get_vsync
//!
//!
//! @return 32-bit am_hal_status_e status
//! @return AM_HAL_STATUS_IN_USE means NemaDC is still in processing
//! @return AM_HAL_STATUS_SUCCESS means NemaDC is complete all operation
//
//*****************************************************************************
am_hal_status_e
nemadc_get_vsync(void)
{
    if (nemadc_reg_read(NEMADC_REG_INTERRUPT) & 1<<4)
    {
        return AM_HAL_STATUS_IN_USE;
    }
    else
    {
        return AM_HAL_STATUS_SUCCESS;
    }
}

//*****************************************************************************
//
//! @brief nemadc_get_te
//!
//!
//! @return 32-bit am_hal_status_e status
//! @return AM_HAL_STATUS_IN_USE means TE is not arrived
//! @return AM_HAL_STATUS_SUCCESS means TE is arrived
//
//*****************************************************************************
am_hal_status_e
nemadc_get_te(void)
{
    if (nemadc_reg_read(NEMADC_REG_INTERRUPT) & 1<<3)
    {
        return AM_HAL_STATUS_IN_USE;
    }
    else
    {
        return AM_HAL_STATUS_SUCCESS;
    }
}
#endif

