//*****************************************************************************
//
//! @file am_hal_gpio.h
//!
//! @brief General Purpose Input Output Functionality
//!
//! @addtogroup gpio_4p GPIO - General Purpose Input Output
//! @ingroup apollo4p_hal
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
#ifndef AM_HAL_GPIO_H
#define AM_HAL_GPIO_H

#ifdef __cplusplus
extern "C"
{
#endif

//
//! Maximum number of GPIOs on this device.
//
#define AM_HAL_GPIO_MAX_PADS        AM_HAL_PIN_TOTAL_GPIOS
#define AM_HAL_GPIO_NUMWORDS        ((AM_HAL_GPIO_MAX_PADS + 31) / 32)

//*****************************************************************************
//
// Macros
//
//*****************************************************************************

//*****************************************************************************
//
//! @name Compute some relative offsets needed in the HAL.
//! GPIO_INTX_DELTA:   The number of bytes between successive interrupt registers
//!                    (i.e. INTEN0, INTEN1, INTEN2, ...).
//! GPIO_NXINT_DELTA:  The number of bytes between MCUN0INTxx and MCUN1INTxx regs.
//! @{
//
//*****************************************************************************
#define GPIO_INTX_DELTA         (offsetof(GPIO_Type, MCUN0INT1EN)   - offsetof(GPIO_Type, MCUN0INT0EN))
#define GPIO_NXINT_DELTA        (offsetof(GPIO_Type, MCUN1INT0EN)   - offsetof(GPIO_Type, MCUN0INT0EN))
//! @}

//*****************************************************************************
//
//! @name The interrupt register order is: EN, STAT, CLR, SET.
//! GPIO_INTCLR_DELTA:  The number of bytes between EN and CLR registers.
//! GPIO_INTSTAT_DELTA: The number of bytes between EN and STAT registers.
//! GPIO_INTSET_DELTA: The number of bytes between EN and SET registers.
//! @{
//
//*****************************************************************************
#define GPIO_INTSTAT_DELTA      (offsetof(GPIO_Type, MCUN1INT0STAT) - offsetof(GPIO_Type, MCUN0INT0EN))
#define GPIO_INTCLR_DELTA       (offsetof(GPIO_Type, MCUN1INT0CLR)  - offsetof(GPIO_Type, MCUN0INT0EN))
#define GPIO_INTSET_DELTA       (offsetof(GPIO_Type, MCUN1INT0SET)  - offsetof(GPIO_Type, MCUN0INT0EN))
//! @}

//*****************************************************************************
//
//! @name Other GPIO helper macros.
//! GPIO_IRQ2N()     Given an IRQ, determine N (0 or 1).
//! GPIO_IRQ2IDX()   Given a GPIO number, determine the interrupt reg index
//!                  relative to N (0-3).
//! GPIO_NUM2IDX()   Given a GPIO number, compute the 32-bit index
//!                  (e.g. 31=0, 32=1, 63=1, 64=2)
//! GPIO_NUM2MSK()   Given a GPIO number, determine the interrupt mask for
//!                  that bit.
//! GPIO_NUM_IRQS    The total number of GPIO IRQs.
//! @{
//
//*****************************************************************************
#define GPIO_IRQ2N(irq)     ( (irq - GPIO0_001F_IRQn) / (GPIO0_607F_IRQn - GPIO0_001F_IRQn + 1) )
#define GPIO_IRQ2IDX(irq)   ( (irq - GPIO0_001F_IRQn) % (GPIO0_607F_IRQn - GPIO0_001F_IRQn + 1) )
#define GPIO_NUM2IDX(num)   ( num / 32 )
#define GPIO_NUM2MSK(num)   ( 1 << (num & 0x1F) )

#define GPIO_NUM_IRQS       (GPIO1_607F_IRQn - GPIO0_001F_IRQn + 1)
//! @}

//*****************************************************************************
//
// Global definitions
//
//*****************************************************************************

//*****************************************************************************
//
//! GPIO Interrupt Channels
//! The GPIO module generates interrupts through two "channels". l
//
//*****************************************************************************
typedef enum
{
    AM_HAL_GPIO_INT_CHANNEL_0,
    AM_HAL_GPIO_INT_CHANNEL_1,
    AM_HAL_GPIO_INT_CHANNEL_BOTH
} am_hal_gpio_int_channel_e;

//*****************************************************************************
//
//! GPIO Interrupt Control definitions.
//! These include Enable, Disable, and Clear functions.
//
//*****************************************************************************
typedef enum
{
    //
    // GPIO Interrupt Enable controls
    //
    AM_HAL_GPIO_INT_CTRL_INDV_DISABLE,
    AM_HAL_GPIO_INT_CTRL_INDV_ENABLE,
    AM_HAL_GPIO_INT_CTRL_MASK_DISABLE,
    AM_HAL_GPIO_INT_CTRL_MASK_ENABLE,
    AM_HAL_GPIO_INT_CTRL_LAST = AM_HAL_GPIO_INT_CTRL_MASK_ENABLE
} am_hal_gpio_int_ctrl_e;

//*****************************************************************************
//
//! Pin configuration
//
//*****************************************************************************
typedef enum
{
    AM_HAL_GPIO_PIN_FUNCTION_DOES_NOT_EXIST = AM_HAL_STATUS_MODULE_SPECIFIC_START,
} am_hal_gpio_status_e;

//
//! Configuration structure for GPIOs.
//
typedef struct
{
    union
    {
        volatile uint32_t cfg;

        struct
        {
            uint32_t uFuncSel        : 4;   // [3:0]
            uint32_t eGPInput        : 1;   // [4:4]
            uint32_t eGPRdZero       : 1;   // [5:5]
            uint32_t eIntDir         : 2;   // [7:6]
            uint32_t eGPOutCfg       : 2;   // [9:8]
            uint32_t eDriveStrength  : 2;   // [11:10]
            uint32_t uSlewRate       : 1;   // [12:12]
            uint32_t ePullup         : 3;   // [15:13]
            uint32_t uNCE            : 6;   // [21:16]
            uint32_t eCEpol          : 1;   // [22:22]
            uint32_t uRsvd_0         : 2;   // [24:23]
            uint32_t ePowerSw        : 1;   // [25:25]  // Select pads only, otherwise reserved
            uint32_t eForceInputEn   : 1;   // [26:26]
            uint32_t eForceOutputEn  : 1;   // [27:27]
            uint32_t uRsvd_1         : 4;   // [31:28]
        } cfg_b;
    } GP;
} am_hal_gpio_pincfg_t;

//
//! Drive strengths.
// Designated as relative full-driver strength.
// All physical (non-virtual) pads support 0P1X and 0P5X.
// Only select pads support OP75X and 1P0X.
//
typedef enum
{
    AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P1X  = 0x0,  // 0.1x  output driver selected
    AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P5X  = 0x1,  // 0.5x  output driver selected
    AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P75X = 0x2,  // 0.75x output driver selected
    AM_HAL_GPIO_PIN_DRIVESTRENGTH_1P0X  = 0x3   // 1.0x  output driver selected
} am_hal_gpio_drivestrength_e;

//
// Deprecated. Please use the above enums instead.
//
#define AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA  AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P1X
#define AM_HAL_GPIO_PIN_DRIVESTRENGTH_16MA  AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P5X

//
//! Power switch configuration.
//
typedef enum
{
    AM_HAL_GPIO_PIN_POWERSW_NONE,
    AM_HAL_GPIO_PIN_POWERSW_VDD,
    AM_HAL_GPIO_PIN_POWERSW_VSS,
    AM_HAL_GPIO_PIN_POWERSW_INVALID,
} am_hal_gpio_powersw_e;

//
//! Pull-up values.
//
typedef enum
{
    AM_HAL_GPIO_PIN_PULLUP_NONE         = 0x00,
    AM_HAL_GPIO_PIN_PULLDOWN_50K,
    AM_HAL_GPIO_PIN_PULLUP_1_5K,
    AM_HAL_GPIO_PIN_PULLUP_6K,
    AM_HAL_GPIO_PIN_PULLUP_12K,
    AM_HAL_GPIO_PIN_PULLUP_24K,
    AM_HAL_GPIO_PIN_PULLUP_50K,
    AM_HAL_GPIO_PIN_PULLUP_100K,    // Weak pullup
} am_hal_gpio_pullup_e;

//
//! GPIO Input configuration.
//
typedef enum
{
    AM_HAL_GPIO_PIN_INPUT_AUTO          = 0x0,
    AM_HAL_GPIO_PIN_INPUT_NONE          = 0x0,
    AM_HAL_GPIO_PIN_INPUT_ENABLE        = 0x1
} am_hal_gpio_input_e;

//
//! Output configurations.
//
typedef enum
{
    AM_HAL_GPIO_PIN_OUTCFG_DISABLE     = 0x0,
    AM_HAL_GPIO_PIN_OUTCFG_PUSHPULL    = 0x1,
    AM_HAL_GPIO_PIN_OUTCFG_OPENDRAIN   = 0x2,
    AM_HAL_GPIO_PIN_OUTCFG_TRISTATE    = 0x3
} am_hal_gpio_outcfg_e;

//
//! GPIO "read zero" configuration.
//
typedef enum
{
    AM_HAL_GPIO_PIN_RDZERO_READPIN      = 0x0,
    AM_HAL_GPIO_PIN_RDZERO_ZERO         = 0x1
} am_hal_gpio_readen_e;

//
//! GPIO interrupt configuration.
//
typedef enum
{
    AM_HAL_GPIO_PIN_INTDIR_NONE         = 0x0,
    AM_HAL_GPIO_PIN_INTDIR_HI2LO        = 0x1,
    AM_HAL_GPIO_PIN_INTDIR_LO2HI        = 0x2,
    AM_HAL_GPIO_PIN_INTDIR_BOTH         = 0x3
} am_hal_gpio_intdir_e;

//
//! GPIO NCE module selection.
//
typedef enum
{
    AM_HAL_GPIO_NCE_IOM0CE0    =  GPIO_PINCFG0_NCESRC0_IOM0CE0,         // IOM 0 NCE 0 module
    AM_HAL_GPIO_NCE_IOM0CE1    =  GPIO_PINCFG0_NCESRC0_IOM0CE1,         // IOM 0 NCE 1 module
    AM_HAL_GPIO_NCE_IOM0CE2    =  GPIO_PINCFG0_NCESRC0_IOM0CE2,         // IOM 0 NCE 2 module
    AM_HAL_GPIO_NCE_IOM0CE3    =  GPIO_PINCFG0_NCESRC0_IOM0CE3,         // IOM 0 NCE 3 module
    AM_HAL_GPIO_NCE_IOM1CE0    =  GPIO_PINCFG0_NCESRC0_IOM1CE0,         // IOM 1 NCE 0 module
    AM_HAL_GPIO_NCE_IOM1CE1    =  GPIO_PINCFG0_NCESRC0_IOM1CE1,         // IOM 1 NCE 1 module
    AM_HAL_GPIO_NCE_IOM1CE2    =  GPIO_PINCFG0_NCESRC0_IOM1CE2,         // IOM 1 NCE 2 module
    AM_HAL_GPIO_NCE_IOM1CE3    =  GPIO_PINCFG0_NCESRC0_IOM1CE3,         // IOM 1 NCE 3 module
    AM_HAL_GPIO_NCE_IOM2CE0    =  GPIO_PINCFG0_NCESRC0_IOM2CE0,         // IOM 2 NCE 0 module
    AM_HAL_GPIO_NCE_IOM2CE1    =  GPIO_PINCFG0_NCESRC0_IOM2CE1,         // IOM 2 NCE 1 module
    AM_HAL_GPIO_NCE_IOM2CE2    =  GPIO_PINCFG0_NCESRC0_IOM2CE2,         // IOM 2 NCE 2 module
    AM_HAL_GPIO_NCE_IOM2CE3    =  GPIO_PINCFG0_NCESRC0_IOM2CE3,         // IOM 2 NCE 3 module
    AM_HAL_GPIO_NCE_IOM3CE0    =  GPIO_PINCFG0_NCESRC0_IOM3CE0,         // IOM 3 NCE 0 module
    AM_HAL_GPIO_NCE_IOM3CE1    =  GPIO_PINCFG0_NCESRC0_IOM3CE1,         // IOM 3 NCE 1 module
    AM_HAL_GPIO_NCE_IOM3CE2    =  GPIO_PINCFG0_NCESRC0_IOM3CE2,         // IOM 3 NCE 2 module
    AM_HAL_GPIO_NCE_IOM3CE3    =  GPIO_PINCFG0_NCESRC0_IOM3CE3,         // IOM 3 NCE 3 module
    AM_HAL_GPIO_NCE_IOM4CE0    =  GPIO_PINCFG0_NCESRC0_IOM4CE0,         // IOM 4 NCE 0 module
    AM_HAL_GPIO_NCE_IOM4CE1    =  GPIO_PINCFG0_NCESRC0_IOM4CE1,         // IOM 4 NCE 1 module
    AM_HAL_GPIO_NCE_IOM4CE2    =  GPIO_PINCFG0_NCESRC0_IOM4CE2,         // IOM 4 NCE 2 module
    AM_HAL_GPIO_NCE_IOM4CE3    =  GPIO_PINCFG0_NCESRC0_IOM4CE3,         // IOM 4 NCE 3 module
    AM_HAL_GPIO_NCE_IOM5CE0    =  GPIO_PINCFG0_NCESRC0_IOM5CE0,         // IOM 5 NCE 0 module
    AM_HAL_GPIO_NCE_IOM5CE1    =  GPIO_PINCFG0_NCESRC0_IOM5CE1,         // IOM 5 NCE 1 module
    AM_HAL_GPIO_NCE_IOM5CE2    =  GPIO_PINCFG0_NCESRC0_IOM5CE2,         // IOM 5 NCE 2 module
    AM_HAL_GPIO_NCE_IOM5CE3    =  GPIO_PINCFG0_NCESRC0_IOM5CE3,         // IOM 5 NCE 3 module
    AM_HAL_GPIO_NCE_IOM6CE0    =  GPIO_PINCFG0_NCESRC0_IOM6CE0,         // IOM 6 NCE 0 module
    AM_HAL_GPIO_NCE_IOM6CE1    =  GPIO_PINCFG0_NCESRC0_IOM6CE1,         // IOM 6 NCE 1 module
    AM_HAL_GPIO_NCE_IOM6CE2    =  GPIO_PINCFG0_NCESRC0_IOM6CE2,         // IOM 6 NCE 2 module
    AM_HAL_GPIO_NCE_IOM6CE3    =  GPIO_PINCFG0_NCESRC0_IOM6CE3,         // IOM 6 NCE 3 module
    AM_HAL_GPIO_NCE_IOM7CE0    =  GPIO_PINCFG0_NCESRC0_IOM7CE0,         // IOM 7 NCE 0 module
    AM_HAL_GPIO_NCE_IOM7CE1    =  GPIO_PINCFG0_NCESRC0_IOM7CE1,         // IOM 7 NCE 1 module
    AM_HAL_GPIO_NCE_IOM7CE2    =  GPIO_PINCFG0_NCESRC0_IOM7CE2,         // IOM 7 NCE 2 module
    AM_HAL_GPIO_NCE_IOM7CE3    =  GPIO_PINCFG0_NCESRC0_IOM7CE3,         // IOM 7 NCE 3 module
    AM_HAL_GPIO_NCE_MSPI0CEN0  =  GPIO_PINCFG0_NCESRC0_MSPI0CEN0,       // MSPI 0 NCE 0 module
    AM_HAL_GPIO_NCE_MSPI0CEN1  =  GPIO_PINCFG0_NCESRC0_MSPI0CEN1,       // MSPI 0 NCE 1 module
    AM_HAL_GPIO_NCE_MSPI1CEN0  =  GPIO_PINCFG0_NCESRC0_MSPI1CEN0,       // MSPI 1 NCE 0 module
    AM_HAL_GPIO_NCE_MSPI1CEN1  =  GPIO_PINCFG0_NCESRC0_MSPI1CEN1,       // MSPI 1 NCE 1 module
    AM_HAL_GPIO_NCE_MSPI2CEN0  =  GPIO_PINCFG0_NCESRC0_MSPI2CEN0,       // MSPI 2 NCE 0 module
    AM_HAL_GPIO_NCE_MSPI2CEN1  =  GPIO_PINCFG0_NCESRC0_MSPI2CEN1,       // MSPI 2 NCE 1 module
    AM_HAL_GPIO_NCE_DCDPIDE    =  GPIO_PINCFG0_NCESRC0_DC_DPI_DE,       // DC DPI DE module
    AM_HAL_GPIO_NCE_DCCSX      =  GPIO_PINCFG0_NCESRC0_DISP_CONT_CSX,   // DC CSX module
    AM_HAL_GPIO_NCE_DCSPICSN   =  GPIO_PINCFG0_NCESRC0_DC_SPI_CS_N,     // DC SPI CS_N module
    AM_HAL_GPIO_NCE_QSPICSN    =  GPIO_PINCFG0_NCESRC0_DC_QSPI_CS_N,    // DC QSPI CS_N module
} am_hal_gpio_nce_sel_e;

//
//! GPIO NCE polarity.
//
typedef enum
{
    AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW     = 0x0,
    AM_HAL_GPIO_PIN_CEPOL_ACTIVEHIGH    = 0x1
} am_hal_gpio_cepol_e;

//
//! Force output or input enable.
//
typedef enum
{
    AM_HAL_GPIO_PIN_FORCEEN_NONE       = 0x0,
    AM_HAL_GPIO_PIN_FORCEEN_FORCE      = 0x1
} am_hal_gpio_forceen_e;

//*****************************************************************************
//
//! Read types for am_hal_gpio_state_read().
//
//*****************************************************************************
typedef enum
{
    AM_HAL_GPIO_INPUT_READ,
    AM_HAL_GPIO_OUTPUT_READ,
    AM_HAL_GPIO_ENABLE_READ
} am_hal_gpio_read_type_e;

//*****************************************************************************
//
//! Write types for am_hal_gpio_state_write().
//!
//! It's important to note that types:
//! AM_HAL_GPIO_OUTPUT_TRISTATE_OUTPUT_EN and AM_HAL_GPIO_OUTPUT_TRISTATE_OUTPUT_DIS
//! operate on the output enable of the pin.
//! Therefore
//!     AM_HAL_GPIO_OUTPUT_TRISTATE_OUTPUT_EN enables the output,
//!     AM_HAL_GPIO_OUTPUT_TRISTATE_OUTPUT_DIS puts the pin into hi-impedance.
//!
//! Given this behavior, perhaps more appropriate names might have been:
//!     AM_HAL_GPIO_OUTPUT_TRISTATE_OUTPUTEN
//!     AM_HAL_GPIO_OUTPUT_TRISTATE_OUTPUTDIS
//
//*****************************************************************************
typedef enum
{
    AM_HAL_GPIO_OUTPUT_CLEAR,
    AM_HAL_GPIO_OUTPUT_SET,
    AM_HAL_GPIO_OUTPUT_TOGGLE,
    AM_HAL_GPIO_OUTPUT_TRISTATE_OUTPUT_DIS,    // Disable output, i.e. Hi-Z
    AM_HAL_GPIO_OUTPUT_TRISTATE_OUTPUT_EN,     // Enable output
    AM_HAL_GPIO_OUTPUT_TRISTATE_OUTPUT_TOG
} am_hal_gpio_write_type_e;

//*****************************************************************************
//
// These enums have been deprecated due to improper naming conventions.
//
//*****************************************************************************
#define AM_HAL_GPIO_OUTPUT_TRISTATE_DISABLE    AM_HAL_GPIO_OUTPUT_TRISTATE_OUTPUT_DIS
#define AM_HAL_GPIO_OUTPUT_TRISTATE_ENABLE     AM_HAL_GPIO_OUTPUT_TRISTATE_OUTPUT_EN
#define AM_HAL_GPIO_OUTPUT_TRISTATE_TOGGLE     AM_HAL_GPIO_OUTPUT_TRISTATE_OUTPUT_TOG

//
//! The handler type includes a void* parameter.
//
typedef void (*am_hal_gpio_handler_t)(void *pArg);

//*****************************************************************************
//
//!
//! @name Common configurations.
//! @{
//
//*****************************************************************************
#define AM_HAL_GPIO_PINCFG_DEFAULT                                            \
    {                                                                         \
        .GP.cfg_b.uFuncSel         = 3,                                       \
        .GP.cfg_b.eGPOutCfg        = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,          \
        .GP.cfg_b.eDriveStrength   = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P1X,      \
        .GP.cfg_b.ePullup          = AM_HAL_GPIO_PIN_PULLUP_NONE,             \
        .GP.cfg_b.eGPInput         = AM_HAL_GPIO_PIN_INPUT_NONE,              \
        .GP.cfg_b.eGPRdZero        = AM_HAL_GPIO_PIN_RDZERO_READPIN,          \
        .GP.cfg_b.eIntDir          = AM_HAL_GPIO_PIN_INTDIR_NONE,             \
        .GP.cfg_b.uSlewRate        = 0,                                       \
        .GP.cfg_b.uNCE             = 0,                                       \
        .GP.cfg_b.eCEpol           = 0,                                       \
        .GP.cfg_b.ePowerSw         = 0,                                       \
        .GP.cfg_b.eForceInputEn    = 0,                                       \
        .GP.cfg_b.eForceOutputEn   = 0,                                       \
        .GP.cfg_b.uRsvd_0          = 0,                                       \
        .GP.cfg_b.uRsvd_1          = 0,                                       \
    }

#define AM_HAL_GPIO_PINCFG_OUTPUT                                             \
    {                                                                         \
        .GP.cfg_b.uFuncSel         = 3,                                       \
        .GP.cfg_b.eGPOutCfg        = AM_HAL_GPIO_PIN_OUTCFG_PUSHPULL,         \
        .GP.cfg_b.eDriveStrength   = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P1X,      \
        .GP.cfg_b.ePullup          = AM_HAL_GPIO_PIN_PULLUP_NONE,             \
        .GP.cfg_b.eGPInput         = AM_HAL_GPIO_PIN_INPUT_NONE,              \
        .GP.cfg_b.eGPRdZero        = AM_HAL_GPIO_PIN_RDZERO_READPIN,          \
        .GP.cfg_b.eIntDir          = AM_HAL_GPIO_PIN_INTDIR_LO2HI,            \
        .GP.cfg_b.uSlewRate        = 0,                                       \
        .GP.cfg_b.uNCE             = 0,                                       \
        .GP.cfg_b.eCEpol           = 0,                                       \
        .GP.cfg_b.ePowerSw         = 0,                                       \
        .GP.cfg_b.eForceInputEn    = 0,                                       \
        .GP.cfg_b.eForceOutputEn   = 0,                                       \
        .GP.cfg_b.uRsvd_0          = 0,                                       \
        .GP.cfg_b.uRsvd_1          = 0,                                       \
    }

#define AM_HAL_GPIO_PINCFG_OUTPUT_WITH_READ                                   \
    {                                                                         \
        .GP.cfg_b.uFuncSel         = 3,                                       \
        .GP.cfg_b.eGPOutCfg        = AM_HAL_GPIO_PIN_OUTCFG_PUSHPULL,         \
        .GP.cfg_b.eDriveStrength   = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P1X,      \
        .GP.cfg_b.ePullup          = AM_HAL_GPIO_PIN_PULLUP_NONE,             \
        .GP.cfg_b.eGPInput         = AM_HAL_GPIO_PIN_INPUT_ENABLE,            \
        .GP.cfg_b.eGPRdZero        = AM_HAL_GPIO_PIN_RDZERO_READPIN,          \
        .GP.cfg_b.eIntDir          = AM_HAL_GPIO_PIN_INTDIR_LO2HI,            \
        .GP.cfg_b.uSlewRate        = 0,                                       \
        .GP.cfg_b.uNCE             = 0,                                       \
        .GP.cfg_b.eCEpol           = 0,                                       \
        .GP.cfg_b.ePowerSw         = 0,                                       \
        .GP.cfg_b.eForceInputEn    = 0,                                       \
        .GP.cfg_b.eForceOutputEn   = 0,                                       \
        .GP.cfg_b.uRsvd_0          = 0,                                       \
        .GP.cfg_b.uRsvd_1          = 0,                                       \
    }

#define AM_HAL_GPIO_PINCFG_INPUT                                              \
    {                                                                         \
        .GP.cfg_b.uFuncSel         = 3,                                       \
        .GP.cfg_b.eGPOutCfg        = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,          \
        .GP.cfg_b.eDriveStrength   = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P1X,      \
        .GP.cfg_b.ePullup          = AM_HAL_GPIO_PIN_PULLUP_NONE,             \
        .GP.cfg_b.eGPInput         = AM_HAL_GPIO_PIN_INPUT_ENABLE,            \
        .GP.cfg_b.eGPRdZero        = AM_HAL_GPIO_PIN_RDZERO_READPIN,          \
        .GP.cfg_b.eIntDir          = AM_HAL_GPIO_PIN_INTDIR_LO2HI,            \
        .GP.cfg_b.uSlewRate        = 0,                                       \
        .GP.cfg_b.uNCE             = 0,                                       \
        .GP.cfg_b.eCEpol           = 0,                                       \
        .GP.cfg_b.ePowerSw         = 0,                                       \
        .GP.cfg_b.eForceInputEn    = 0,                                       \
        .GP.cfg_b.eForceOutputEn   = 0,                                       \
        .GP.cfg_b.uRsvd_0          = 0,                                       \
        .GP.cfg_b.uRsvd_1          = 0,                                       \
    }

#define AM_HAL_GPIO_PINCFG_TRISTATE                                           \
    {                                                                         \
        .GP.cfg_b.uFuncSel         = 3,                                       \
        .GP.cfg_b.eGPOutCfg        = AM_HAL_GPIO_PIN_OUTCFG_TRISTATE,         \
        .GP.cfg_b.eDriveStrength   = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P1X,      \
        .GP.cfg_b.ePullup          = AM_HAL_GPIO_PIN_PULLUP_NONE,             \
        .GP.cfg_b.eGPInput         = AM_HAL_GPIO_PIN_INPUT_NONE,              \
        .GP.cfg_b.eGPRdZero        = AM_HAL_GPIO_PIN_RDZERO_READPIN,          \
        .GP.cfg_b.eIntDir          = AM_HAL_GPIO_PIN_INTDIR_LO2HI,            \
        .GP.cfg_b.uSlewRate        = 0,                                       \
        .GP.cfg_b.uNCE             = 0,                                       \
        .GP.cfg_b.eCEpol           = 0,                                       \
        .GP.cfg_b.ePowerSw         = 0,                                       \
        .GP.cfg_b.eForceInputEn    = 0,                                       \
        .GP.cfg_b.eForceOutputEn   = 0,                                       \
        .GP.cfg_b.uRsvd_0          = 0,                                       \
        .GP.cfg_b.uRsvd_1          = 0,                                       \
    }

#define AM_HAL_GPIO_PINCFG_OPENDRAIN                                          \
    {                                                                         \
        .GP.cfg_b.uFuncSel         = 3,                                       \
        .GP.cfg_b.eGPOutCfg        = AM_HAL_GPIO_PIN_OUTCFG_OPENDRAIN,        \
        .GP.cfg_b.eDriveStrength   = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P1X,      \
        .GP.cfg_b.ePullup          = AM_HAL_GPIO_PIN_PULLUP_NONE,             \
        .GP.cfg_b.eGPInput         = AM_HAL_GPIO_PIN_INPUT_NONE,              \
        .GP.cfg_b.eGPRdZero        = AM_HAL_GPIO_PIN_RDZERO_READPIN,          \
        .GP.cfg_b.eIntDir          = AM_HAL_GPIO_PIN_INTDIR_LO2HI,            \
        .GP.cfg_b.uSlewRate        = 0,                                       \
        .GP.cfg_b.uNCE             = 0,                                       \
        .GP.cfg_b.eCEpol           = 0,                                       \
        .GP.cfg_b.ePowerSw         = 0,                                       \
        .GP.cfg_b.eForceInputEn    = 0,                                       \
        .GP.cfg_b.eForceOutputEn   = 0,                                       \
        .GP.cfg_b.uRsvd_0          = 0,                                       \
        .GP.cfg_b.uRsvd_1          = 0,                                       \
    }

#define AM_HAL_GPIO_PINCFG_DISABLED                                           \
    {                                                                         \
        .GP.cfg_b.uFuncSel         = 3,                                       \
        .GP.cfg_b.eGPOutCfg        = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,          \
        .GP.cfg_b.eDriveStrength   = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P1X,      \
        .GP.cfg_b.ePullup          = AM_HAL_GPIO_PIN_PULLUP_NONE,             \
        .GP.cfg_b.eGPInput         = AM_HAL_GPIO_PIN_INPUT_NONE,              \
        .GP.cfg_b.eGPRdZero        = AM_HAL_GPIO_PIN_RDZERO_READPIN,          \
        .GP.cfg_b.eIntDir          = AM_HAL_GPIO_PIN_INTDIR_NONE,             \
        .GP.cfg_b.uSlewRate        = 0,                                       \
        .GP.cfg_b.uNCE             = 0,                                       \
        .GP.cfg_b.eCEpol           = 0,                                       \
        .GP.cfg_b.ePowerSw         = 0,                                       \
        .GP.cfg_b.eForceInputEn    = 0,                                       \
        .GP.cfg_b.eForceOutputEn   = 0,                                       \
        .GP.cfg_b.uRsvd_0          = 0,                                       \
        .GP.cfg_b.uRsvd_1          = 0,                                       \
    }

#define AM_HAL_GPIO_PINCFG_PULLEDUP_DISABLED                                  \
    {                                                                         \
        .GP.cfg_b.uFuncSel         = 3,                                       \
        .GP.cfg_b.eGPOutCfg        = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,          \
        .GP.cfg_b.eDriveStrength   = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA,      \
        .GP.cfg_b.ePullup          = AM_HAL_GPIO_PIN_PULLUP_100K,             \
        .GP.cfg_b.eGPInput         = AM_HAL_GPIO_PIN_INPUT_NONE,              \
        .GP.cfg_b.eGPRdZero        = AM_HAL_GPIO_PIN_RDZERO_READPIN,          \
        .GP.cfg_b.eIntDir          = AM_HAL_GPIO_PIN_INTDIR_LO2HI,            \
        .GP.cfg_b.uSlewRate        = 0,                                       \
        .GP.cfg_b.uNCE             = 0,                                       \
        .GP.cfg_b.eCEpol           = 0,                                       \
        .GP.cfg_b.ePowerSw         = 0,                                       \
        .GP.cfg_b.eForceInputEn    = 0,                                       \
        .GP.cfg_b.eForceOutputEn   = 0,                                       \
        .GP.cfg_b.uRsvd_0          = 0,                                       \
        .GP.cfg_b.uRsvd_1          = 0,                                       \
    }
//! @}

//*****************************************************************************
//
//! @name Structures where default configurations can be accessed.
//! @{
//
//*****************************************************************************
extern const am_hal_gpio_pincfg_t am_hal_gpio_pincfg_default;
extern const am_hal_gpio_pincfg_t am_hal_gpio_pincfg_disabled;
extern const am_hal_gpio_pincfg_t am_hal_gpio_pincfg_pulledup_disabled;
extern const am_hal_gpio_pincfg_t am_hal_gpio_pincfg_output;
extern const am_hal_gpio_pincfg_t am_hal_gpio_pincfg_output_with_read;
extern const am_hal_gpio_pincfg_t am_hal_gpio_pincfg_input;
extern const am_hal_gpio_pincfg_t am_hal_gpio_pincfg_tristate;
extern const am_hal_gpio_pincfg_t am_hal_gpio_pincfg_opendrain;
//! @}

//*****************************************************************************
//
//! Structure for defining bitmasks used in the interrupt functions.
//
//*****************************************************************************
typedef struct
{
    union
    {
        volatile uint32_t Msk[AM_HAL_GPIO_NUMWORDS];

        struct
        {
            volatile uint32_t b0 :   1;
            volatile uint32_t b1 :   1;
            volatile uint32_t b2 :   1;
            volatile uint32_t b3 :   1;
            volatile uint32_t b4 :   1;
            volatile uint32_t b5 :   1;
            volatile uint32_t b6 :   1;
            volatile uint32_t b7 :   1;
            volatile uint32_t b8 :   1;
            volatile uint32_t b9 :   1;
            volatile uint32_t b10 :  1;
            volatile uint32_t b11 :  1;
            volatile uint32_t b12 :  1;
            volatile uint32_t b13 :  1;
            volatile uint32_t b14 :  1;
            volatile uint32_t b15 :  1;
            volatile uint32_t b16 :  1;
            volatile uint32_t b17 :  1;
            volatile uint32_t b18 :  1;
            volatile uint32_t b19 :  1;
            volatile uint32_t b20 :  1;
            volatile uint32_t b21 :  1;
            volatile uint32_t b22 :  1;
            volatile uint32_t b23 :  1;
            volatile uint32_t b24 :  1;
            volatile uint32_t b25 :  1;
            volatile uint32_t b26 :  1;
            volatile uint32_t b27 :  1;
            volatile uint32_t b28 :  1;
            volatile uint32_t b29 :  1;
            volatile uint32_t b30 :  1;
            volatile uint32_t b31 :  1;
            volatile uint32_t b32 :  1;
            volatile uint32_t b33 :  1;
            volatile uint32_t b34 :  1;
            volatile uint32_t b35 :  1;
            volatile uint32_t b36 :  1;
            volatile uint32_t b37 :  1;
            volatile uint32_t b38 :  1;
            volatile uint32_t b39 :  1;
            volatile uint32_t b40 :  1;
            volatile uint32_t b41 :  1;
            volatile uint32_t b42 :  1;
            volatile uint32_t b43 :  1;
            volatile uint32_t b44 :  1;
            volatile uint32_t b45 :  1;
            volatile uint32_t b46 :  1;
            volatile uint32_t b47 :  1;
            volatile uint32_t b48 :  1;
            volatile uint32_t b49 :  1;
            volatile uint32_t b50 :  1;
            volatile uint32_t b51 :  1;
            volatile uint32_t b52 :  1;
            volatile uint32_t b53 :  1;
            volatile uint32_t b54 :  1;
            volatile uint32_t b55 :  1;
            volatile uint32_t b56 :  1;
            volatile uint32_t b57 :  1;
            volatile uint32_t b58 :  1;
            volatile uint32_t b59 :  1;
            volatile uint32_t b60 :  1;
            volatile uint32_t b61 :  1;
            volatile uint32_t b62 :  1;
            volatile uint32_t b63 :  1;
            volatile uint32_t b64 :  1;
            volatile uint32_t b65 :  1;
            volatile uint32_t b66 :  1;
            volatile uint32_t b67 :  1;
            volatile uint32_t b68 :  1;
            volatile uint32_t b69 :  1;
            volatile uint32_t b70 :  1;
            volatile uint32_t b71 :  1;
            volatile uint32_t b72 :  1;
            volatile uint32_t b73 :  1;
            volatile uint32_t b74 :  1;
            volatile uint32_t b75 :  1;
            volatile uint32_t b76 :  1;
            volatile uint32_t b77 :  1;
            volatile uint32_t b78 :  1;
            volatile uint32_t b79 :  1;
            volatile uint32_t b80 :  1;
            volatile uint32_t b81 :  1;
            volatile uint32_t b82 :  1;
            volatile uint32_t b83 :  1;
            volatile uint32_t b84 :  1;
            volatile uint32_t b85 :  1;
            volatile uint32_t b86 :  1;
            volatile uint32_t b87 :  1;
            volatile uint32_t b88 :  1;
            volatile uint32_t b89 :  1;
            volatile uint32_t b90 :  1;
            volatile uint32_t b91 :  1;
            volatile uint32_t b92 :  1;
            volatile uint32_t b93 :  1;
            volatile uint32_t b94 :  1;
            volatile uint32_t b95 :  1;
            volatile uint32_t b96 :  1;
            volatile uint32_t b97 :  1;
            volatile uint32_t b98 :  1;
            volatile uint32_t b99 :  1;
            volatile uint32_t b100 : 1;
            volatile uint32_t b101 : 1;
            volatile uint32_t b102 : 1;
            volatile uint32_t b103 : 1;
            volatile uint32_t b104 : 1;
            volatile uint32_t b105 : 1;
            volatile uint32_t b106 : 1;
            volatile uint32_t b107 : 1;
            volatile uint32_t b108 : 1;
            volatile uint32_t b109 : 1;
            volatile uint32_t b110 : 1;
            volatile uint32_t b111 : 1;
            volatile uint32_t b112 : 1;
            volatile uint32_t b113 : 1;
            volatile uint32_t b114 : 1;
            volatile uint32_t b115 : 1;
            volatile uint32_t b116 : 1;
            volatile uint32_t b117 : 1;
            volatile uint32_t b118 : 1;
            volatile uint32_t b119 : 1;
            volatile uint32_t b120 : 1;
            volatile uint32_t b121 : 1;
            volatile uint32_t b122 : 1;
            volatile uint32_t b123 : 1;
            volatile uint32_t b124 : 1;
            volatile uint32_t b125 : 1;
            volatile uint32_t b126 : 1;
            volatile uint32_t b127 : 1;
        } Msk_b;
    } U;
} am_hal_gpio_mask_t;

//
//! Use AM_HAL_GPIO_MASK_ZERO to initialize a am_hal_gpio_mask_t union/structure
//! to all zeros at declaration time. e.g.
//!  am_hal_gpio_mask_t gpionewmask = AM_HAL_GPIO_MASK_DECLARE_ZERO;
//
#define AM_HAL_GPIO_MASK_DECLARE_ZERO   \
{                                       \
    .U.Msk[0] = 0,                      \
    .U.Msk[1] = 0,                      \
    .U.Msk[2] = 0,                      \
    .U.Msk[3] = 0                       \
}

//
//! Initialize a mask to a given value (typically 0 or 0xFFFFFFFF).
//
#define AM_HAL_GPIO_MASK_INIT(msk, val)     \
{                                           \
    msk.U.Msk[0] = val;                     \
    msk.U.Msk[1] = val;                     \
    msk.U.Msk[2] = val;                     \
    msk.U.Msk[3] = val;                     \
}

//
//! Given a pointer to am_hal_gpio_mask_t, use this macro to initialize
//! all members to the given value (typically 0 or 0xFFFFFFFF).
//
#define AM_HAL_GPIO_PMASK_INIT(pmsk, val)   \
{                                           \
    pmsk->U.Msk[0] = val;                   \
    pmsk->U.Msk[1] = val;                   \
    pmsk->U.Msk[2] = val;                   \
    pmsk->U.Msk[3] = val;                   \
}

//*****************************************************************************
//
//! @name Helper macros
//! @{
//
//*****************************************************************************
#define AM_HAL_MASK32(n)        ((uint32_t)1 << ((n) & 0x1F))

#define AM_HAL_GPIO_RDn(pin)    ((volatile uint32_t *)&GPIO->RD0  + (((pin) >> 5) & 0x3))
#define AM_HAL_GPIO_WTn(pin)    ((volatile uint32_t *)&GPIO->WT0  + (((pin) >> 5) & 0x3))
#define AM_HAL_GPIO_WTCn(pin)   ((volatile uint32_t *)&GPIO->WTC0 + (((pin) >> 5) & 0x3))
#define AM_HAL_GPIO_WTSn(pin)   ((volatile uint32_t *)&GPIO->WTS0 + (((pin) >> 5) & 0x3))
#define AM_HAL_GPIO_ENn(pin)    ((volatile uint32_t *)&GPIO->EN0  + (((pin) >> 5) & 0x3))
#define AM_HAL_GPIO_ENCn(pin)   ((volatile uint32_t *)&GPIO->ENC0 + (((pin) >> 5) & 0x3))
#define AM_HAL_GPIO_ENSn(pin)   ((volatile uint32_t *)&GPIO->ENS0 + (((pin) >> 5) & 0x3))
//! @}

//*****************************************************************************
//
//! @brief Macros to read GPIO values in an optimized manner.
//!
//! @param n - The GPIO number to be read.
//!
//! In almost all cases, it is reasonable to use am_hal_gpio_state_read() to
//! read GPIO values with all of the inherent error checking, critical
//! sectioning, and general safety.
//!
//! However, occasionally there is a need to read a GPIO value in an optimized
//! manner.  These 3 macros will accomplish that.  Each macro will return a
//! value of 1 or 0.
//!
//! Note that the macros are named as lower-case counterparts to the
//! enumerations for the am_hal_gpio_state_read() function.  That is:
//!
//!     AM_HAL_GPIO_INPUT_READ  -> am_hal_gpio_input_read(n)
//!     AM_HAL_GPIO_OUTPUT_READ -> am_hal_gpio_output_read(n)
//!     AM_HAL_GPIO_ENABLE_READ -> am_hal_gpio_enable_read(n)
//!
//! @return Each macro will return a 1 or 0 per the value of the requested GPIO.
//!
//
//*****************************************************************************
#define am_hal_gpio_input_read(n)   ((*AM_HAL_GPIO_RDn((n)) >> ((n) % 32)) & 1)
#define am_hal_gpio_output_read(n)  ((*AM_HAL_GPIO_WTn((n)) >> ((n) % 32)) & 1)
#define am_hal_gpio_enable_read(n)  ((*AM_HAL_GPIO_ENn((n)) >> ((n) % 32)) & 1)

//*****************************************************************************
//
//! @brief Macros to write GPIO values in an optimized manner.
//!
//! @param n - The GPIO number to be written.
//!
//! In almost all cases, it is reasonable to use am_hal_gpio_state_write() to
//! write GPIO values with all of the inherent error checking, critical
//! sectioning, and general safety.
//!
//! However, occasionally there is a need to write a GPIO value in an optimized
//! manner.  These 3 macros will accomplish that.
//!
//! Note that the macros are named as lower-case counterparts to the
//! enumerations for the am_hal_gpio_state_read() function.  That is:
//!
//!    AM_HAL_GPIO_OUTPUT_CLEAR                -> am_hal_gpio_output_clear(n)
//!    AM_HAL_GPIO_OUTPUT_SET                  -> am_hal_gpio_output_set(n)
//!    AM_HAL_GPIO_OUTPUT_TOGGLE               -> am_hal_gpio_output_toggle(n)
//!    AM_HAL_GPIO_OUTPUT_TRISTATE_OUTPUT_DIS  -> am_hal_gpio_output_tristate_output_dis(n)
//!    AM_HAL_GPIO_OUTPUT_TRISTATE_OUTPUT_EN   -> am_hal_gpio_output_tristate_output_en(n)
//!    AM_HAL_GPIO_OUTPUT_TRISTATE_OUTPUT_TOG  -> am_hal_gpio_output_toggle(n).
//!
//! It's important to note that the macros:
//!     am_hal_gpio_output_tristate_output_en()
//!     am_hal_gpio_output_tristate_output_dis()
//! operate on the output enable of the pin. Therefore,
//!     am_hal_gpio_output_tristate_output_en() enables the output,
//!     am_hal_gpio_output_tristate_output_dis() puts the pin into hi-impedance.
//! Given this behavior, perhaps more appropriate names might have been:
//!     am_hal_gpio_output_tristate_outputen()
//!     am_hal_gpio_output_tristate_outputdis()
//!
//*****************************************************************************
#define am_hal_gpio_output_clear(n)             (*AM_HAL_GPIO_WTCn((n)) = AM_HAL_MASK32(n))
#define am_hal_gpio_output_set(n)               (*AM_HAL_GPIO_WTSn((n)) = AM_HAL_MASK32(n))
#define am_hal_gpio_output_toggle(n)                                            \
    if ( 1 )                                                                    \
    {                                                                           \
        AM_CRITICAL_BEGIN                                                       \
        (*AM_HAL_GPIO_WTn((n)) ^= AM_HAL_MASK32(n));                            \
        AM_CRITICAL_END                                                         \
    }

#define am_hal_gpio_output_tristate_output_dis(n)  (*AM_HAL_GPIO_ENCn((n)) = AM_HAL_MASK32(n))
#define am_hal_gpio_output_tristate_output_en(n)   (*AM_HAL_GPIO_ENSn((n)) = AM_HAL_MASK32(n))
#define am_hal_gpio_output_tristate_output_tog(n)                                   \
    if ( 1 )                                                                    \
    {                                                                           \
        AM_CRITICAL_BEGIN                                                       \
        (*AM_HAL_GPIO_ENn((n)) ^=  AM_HAL_MASK32(n));                           \
        AM_CRITICAL_END                                                         \
    }

//*****************************************************************************
//
// These macros have been deprecated due to improper naming conventions.
//
//*****************************************************************************
#define am_hal_gpio_output_tristate_disable(n)   am_hal_gpio_output_tristate_output_dis(n)
#define am_hal_gpio_output_tristate_enable(n)    am_hal_gpio_output_tristate_output_en(n)
#define am_hal_gpio_output_tristate_toggle(n)    am_hal_gpio_output_tristate_output_tog(n)

//*****************************************************************************
//
//*****************************************************************************
#define am_hal_gpio_intdir_toggle(n)                                            \
    if ( 1 )                                                                    \
    {                                                                           \
        volatile uint32_t *pui32Config = &GPIO->PINCFG0;                        \
        AM_CRITICAL_BEGIN                                                       \
        GPIO->PADKEY = GPIO_PADKEY_PADKEY_Key;                                  \
        pui32Config[n] ^= ((uint32_t)0x3 << GPIO_PINCFG0_IRPTEN0_Pos);          \
        AM_CRITICAL_END                                                         \
    }

//*****************************************************************************
//
// External functions.
//
//*****************************************************************************

//*****************************************************************************
//
//! @brief Return the current configuration of a pin.
//!
//! @param ui32GpioNum is the GPIO pin number to configure.
//! @param psGpioCfg - Ptr for the return value of the current configuration.
//!
//! This function returns the current configuration of a GPIO.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t
am_hal_gpio_pinconfig_get(uint32_t ui32GpioNum, am_hal_gpio_pincfg_t* psGpioCfg);

//*****************************************************************************
//
//! @brief Configure the function of a single pin.
//!
//! @param ui32GpioNum is the GPIO pin number to configure.
//! @param sGpioCfg Structure corresponding to the desired configuration.
//!
//! This function sets the configuration of a GPIO.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_gpio_pinconfig(uint32_t ui32GpioNum,
                                      const am_hal_gpio_pincfg_t sGpioCfg);

//*****************************************************************************
//
//! @brief Configure the function of a single pin.
//!
//! @param ui32GpioNum is the GPIO pin number to configure.
//! @param sGpioCfg Structure corresponding to the desired configuration.
//! @param eFunction Function Select type
//!
//! This function sets the configuration of a GPIO.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_gpio_pinconfig_override(uint32_t ui32GpioNum,
                                               am_hal_gpio_pincfg_t sGpioCfg,
                                               am_hal_pin_function_e eFunction);

//*****************************************************************************
//
//! @brief Read GPIO state values
//!
//! @param ui32GpioNum is the pin to read.
//! @param eReadType is the type of read to perform.
//! @param pui32ReadState returns the requested value.
//!
//! This function allows the caller to read any of the following values
//! associated with a GPIO:
//! - Input value
//! - Output value
//! - Output enable value
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_gpio_state_read(uint32_t ui32GpioNum,
                                       am_hal_gpio_read_type_e eReadType,
                                       uint32_t *pui32ReadState);

//*****************************************************************************
//
//! @brief Write GPIO state values
//!
//! @param ui32GpioNum is the pin to write to
//! @param eWriteType is the type of write to perform.
//!
//! This function allows the caller to write any of the following values
//! associated with a GPIO:
//! - Ouput drive value
//! - Output enable value
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_gpio_state_write(uint32_t ui32GpioNum,
                                        am_hal_gpio_write_type_e eWriteType);

//*****************************************************************************
//
//! @brief Enable one or more GPIO interrupts.
//!
//! @param eChannel:  Selects the GPIO channel to operate on, must be one of:
//!     AM_HAL_GPIO_INT_CHANNEL_0,
//!     AM_HAL_GPIO_INT_CHANNEL_1,
//!     AM_HAL_GPIO_INT_CHANNEL_BOTH
//!
//! @param eControl: Specify the control operation on an individual or multiple
//! GPIO interrupts. The control operation is one of:
//!     AM_HAL_GPIO_INT_CTRL_INDV_DISABLE
//!     AM_HAL_GPIO_INT_CTRL_INDV_ENABLE
//!     AM_HAL_GPIO_INT_CTRL_MASK_DISABLE
//!     AM_HAL_GPIO_INT_CTRL_MASK_ENABLE
//!
//! @param pGpioIntMaskOrNumber allows specifying either an individual GPIO or
//! multiple GPIOs via a mask.  Its actual usage is determined based on the
//! eControl parameter as follows:
//!     AM_HAL_GPIO_INT_CTRL_INDV_DISABLE or AM_HAL_GPIO_INT_CTRL_INDV_ENABLE:
//!       gGpioIntMaskOrNumber points to a uint32_t which specifies the single
//!       GPIO interrupt number to operate on.
//!     AM_HAL_GPIO_INT_CTRL_MASK_DISABLE or AM_HAL_GPIO_INT_CTRL_MASK_ENABLE:
//!       gGpioIntMaskOrNumber points to a am_hal_gpio_mask_t structure which
//!       specifies a mask of interrupts to operate on (similar to the way that
//!       previous Apollo devices were handled in the HAL).
//!
//! This function enables an individual interrupt or multiple interrupts for
//! the GPIO module.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_gpio_interrupt_control(am_hal_gpio_int_channel_e eChannel,
                                              am_hal_gpio_int_ctrl_e eControl,
                                              void *pGpioIntMaskOrNumber);

//*****************************************************************************
//
//! @brief Read the GPIO interrupt status.
//!
//! @param eChannel:  Selects the GPIO channel to operate on, must be one of:
//!     AM_HAL_GPIO_INT_CHANNEL_0,
//!     AM_HAL_GPIO_INT_CHANNEL_1,
//!     AM_HAL_GPIO_INT_CHANNEL_BOTH
//! @param bEnabledOnly determines whether disabled interrupts are included in
//! the status.
//! @param pGpioIntMask - Mask of GPIO interrupts to be cleared.
//!
//! This function reads the current interrupt status for the GPIO module. If the
//! \e bEnabledOnly parameter is set, then only interrupt bits associated with
//! active interrupts will be set.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_gpio_interrupt_status_get(am_hal_gpio_int_channel_e eChannel,
                                                 bool bEnabledOnly,
                                                 am_hal_gpio_mask_t *pGpioIntMask);

//*****************************************************************************
//
//! @brief Clear GPIO interrupts.
//!
//! @param eChannel - One of:
//!     AM_HAL_GPIO_INT_CHANNEL_0,
//!     AM_HAL_GPIO_INT_CHANNEL_1,
//!     AM_HAL_GPIO_INT_CHANNEL_BOTH
//! @param pGpioIntMask - Mask of GPIO interrupts to be cleared.
//! This mask is typically returned from am_hal_gpio_interrupt_status_get().
//!
//! @return Status.
//!         Fails if any invalid bits are set in pGpioIntMask.
//
//*****************************************************************************
extern uint32_t am_hal_gpio_interrupt_clear(am_hal_gpio_int_channel_e eChannel,
                                            am_hal_gpio_mask_t *pGpioIntMask);

//*****************************************************************************
//
//! @brief Read the GPIO interrupt status of a given GPIO IRQ.
//!
//! @param ui32GpioIrq is the desired GPIO IRQ number.  It is one of:
//!     GPIO0_001F_IRQn, GPIO0_203F_IRQn, GPIO0_405F_IRQn, GPIO0_607F_IRQn
//!     GPIO1_001F_IRQn, GPIO1_203F_IRQn, GPIO1_405F_IRQn, GPIO1_607F_IRQn
//! @param bEnabledOnly determines whether disabled interrupts are included in
//! the status.
//! @param pui32IntStatus returns the GPIO interrupt status.  The value that
//! is returned is the mod 32 value of the register containing the GPIO.
//!
//! This function reads the current interrupt status for the given GPIO IRQ.
//! The IRQ number covers up to 32 GPIOs.  If the bEnabledOnly parameter is set,
//! then only interrupt bits associated with active interrupts for that IRQ
//! will be returned.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_gpio_interrupt_irq_status_get(uint32_t ui32GpioIrq,
                                                     bool bEnabledOnly,
                                                     uint32_t *pui32IntStatus);

//*****************************************************************************
//
//! @brief Clear the interrupts(s) for specified GPIO IRQ.
//!
//! @param ui32GpioIrq - The GPIO IRQ group to clear.  It is one of:
//!     GPIO0_001F_IRQn, GPIO0_203F_IRQn, GPIO0_405F_IRQn, GPIO0_607F_IRQn
//!     GPIO1_001F_IRQn, GPIO1_203F_IRQn, GPIO1_405F_IRQn, GPIO1_607F_IRQn
//! @param ui32GpioIntMaskStatus - The bitmask for the interrupts to be cleared for the
//!     given GPIO IRQ. This value is usually obtained from a call to
//!     am_hal_gpio_interrupt_irq_status_get().
//!
//! @return Status.
//
//*****************************************************************************
extern uint32_t am_hal_gpio_interrupt_irq_clear(uint32_t ui32GpioIrq,
                                                uint32_t ui32GpioIntMaskStatus);

//*****************************************************************************
//
//! @brief Register an interrupt handler for a specific GPIO.
//!
//! @param eChannel:  Selects the GPIO channel to operate on, must be one of:
//!     AM_HAL_GPIO_INT_CHANNEL_0,
//!     AM_HAL_GPIO_INT_CHANNEL_1,
//!     AM_HAL_GPIO_INT_CHANNEL_BOTH
//! @param ui32GpioNum GPIO pin number to register an interrupt for.
//! @param pfnHandler is a function pointer for an interrupt handler for this
//! pin.
//! @param pArg
//!
//! This routine was designed to work with \e am_hal_gpio_interrupt_service().
//! Together, the two routines can be used to call pin-specific interrupt
//! handler. This function adds the \e pfnHandler argument to a table of
//! interrupt handlers so that \e am_hal_gpio_interrupt_service() can call it in
//! response to interrupts from the associated GPIO pin.

//! @note Usage of this function is entirely optional. It is always possible to
//! use if-statements (or a similar construct) to check the GPIO interrupt
//! status and call the correct routine from within the main GPIO interrupt
//! handler.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_gpio_interrupt_register(am_hal_gpio_int_channel_e eChannel,
                                               uint32_t ui32GpioNum,
                                               am_hal_gpio_handler_t pfnHandler,
                                               void *pArg);

//*****************************************************************************
//
//! @brief Relay interrupts from the main GPIO module to individual handlers.
//!
//! @param ui32GpioIrq - Interrupt to read
//! @param ui32GpioIntMaskStatus - recently read GPIO interrupt status.
//!
//! This routine was designed to work with \e am_hal_gpio_interrupt_register().
//! Together, the two routines can be used to call pin-specific interrupt
//! handler. Once an interrupt handler has been registered with \e
//! am_hal_gpio_interrupt_register(), this function will call it in response to
//! interrupt signals from the associated pin.

//! @note Usage of this function is entirely optional. It is always possible to
//! use if-statements (or a similar construct) to check the GPIO interrupt
//! status and call the correct routine from within the main GPIO interrupt
//! handler.
//!
//! @return Standard HAL status code.
//
//*****************************************************************************
extern uint32_t am_hal_gpio_interrupt_service(uint32_t ui32GpioIrq,
                                              uint32_t ui32GpioIntMaskStatus);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_GPIO_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
