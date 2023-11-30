//*****************************************************************************
//
//! @file am_devices_tma525.h
//!
//! @brief General I2C touch driver.
//!
//! @addtogroup tma525 TMA525 - I2C Touch Driver
//! @ingroup devices
//! @{
//
//**************************************************************************

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

#ifndef AM_DEVICES_TMA525_H
#define AM_DEVICES_TMA525_H

#ifdef __cplusplus
extern "C"
{
#endif

#define AM_DEVICES_TMA525_DISP_RESX                 (454-1)
#define AM_DEVICES_TMA525_DISP_RESY                 (454-1)

#define AM_DEVICES_TMA525_READ_DATA_BLOCK           (0x22)

#define AM_DEVICES_TMA525_I2C_WR                    0x0000
#define AM_DEVICES_TMA525_I2C_RD                    (1u << 0)

#define AM_DEVICES_TMA525_SWRESET_COUNT             10
#define AM_DEVICES_TMA525_SWRESET_DELAY_US          100

#define __DRV_ZTW523_H__

#define TOUCH_POINT_MODE                0
#define MAX_SUPPORTED_FINGER_NUM        1
#define TPD_RES_139_46_MAX_X            453
#define TPD_RES_139_46_MAX_Y            453
#define TPD_RES_119_42_MAX_X            389
#define TPD_RES_119_42_MAX_Y            389
#define I2C_BUFFER_SIZE                 2
#define TC_SECTOR_SZ                    8
#define PAGE_SIZE                       64

#define TP_POWERON_DELAY                10
#define CHIP_ON_DELAY                   10
#define CHIP_OFF_DELAY                  70
#define FIRMWARE_ON_DELAY               50
#define ENABLE                          1
#define DISABLE                         0

#define CHIP_ID_REG                     0xCC00
#define CHIP_ID_VALUE                   0xE628
#define TMA525_SLAVE_ADDR               0x24

// different technique in 1.39 and 1.19
// 1.39:Lens GFF
// 1.19:DEO oncell

#define HW_ID_BOE_LENS_139_46           0x0103
#define HW_ID_EDO_LENS_139_46           0x0203
#define HW_ID_EDO_EDO_119_42            0x2

// chip code
#define ZTW523_CHIP_CODE                0xE628

//! @name Register Map
//! @{
#define ZINITIX_SWRESET_CMD             0x0000
#define ZINITIX_WAKEUP_CMD              0x0001
#define ZINITIX_IDLE_CMD                0x0004
#define ZINITIX_SLEEP_CMD               0x0005
#define ZINITIX_CLEAR_INT_STATUS_CMD    0x0003
#define ZINITIX_CALIBRATE_CMD           0x0006
#define ZINITIX_SAVE_STATUS_CMD         0x0007
#define ZINITIX_SAVE_CALIBRATION_CMD    0x0008
#define ZINITIX_RECALL_FACTORY_CMD      0x000f
#define ZINITIX_VENDOR_CMD              0xC000
#define ZINITIX_INTN_CLEAR_CMD          0xC004
#define ZINITIX_NVM_INIT_CMD            0xC002
#define ZINITIX_PROGRAM_START_CMD       0xC001
#define ZINITIX_NVM_VPP                 0xC003
#define ZINITIX_NVM_WP                  0xC104
#define ZINITIX_POSTURE_REG             0x0126

#define ZINITIX_INIT_RETRY_CNT          3
#define TOUCH_CHECK_SHORT_MODE          14
#define TOUCH_SEC_MODE                  48
#define TOUCH_REF_MODE                  10
#define TOUCH_NORMAL_MODE               5
#define TOUCH_DELTA_MODE                3
#define TOUCH_DND_MODE                  6
#define TOUCH_PDND_MODE                 11
#define NORMAL_SHORT_VALUE              1000

#define ZINITIX_SENSITIVITY                     0x0020
#define ZINITIX_I2C_CHECKSUM_WCNT               0x016a
#define ZINITIX_I2C_CHECKSUM_RESULT             0x016c
#define ZINITIX_DEBUG_REG                       0x0115
#define ZINITIX_TOUCH_MODE                      0x0010
#define ZINITIX_CHIP_REVISION                   0x0011
#define ZINITIX_FIRMWARE_VERSION                0x0012
#define ZINITIX_MINOR_FW_VERSION                0x0121
#define ZINITIX_DATA_VERSION_REG                0x0013
#define ZINITIX_HW_ID                           0x0014
#define ZINITIX_SUPPORTED_FINGER_NUM            0x0015
#define ZINITIX_EEPROM_INFO                     0x0018
#define ZINITIX_INITIAL_TOUCH_MODE              0x0019
#define ZINITIX_TOTAL_NUMBER_OF_X               0x0060
#define ZINITIX_TOTAL_NUMBER_OF_Y               0x0061
#define ZINITIX_DELAY_RAW_FOR_HOST              0x007f
#define ZINITIX_BUTTON_SUPPORTED_NUM            0x00B0
#define ZINITIX_BUTTON_SENSITIVITY              0x00B2
#define ZINITIX_X_RESOLUTION                    0x00C0
#define ZINITIX_Y_RESOLUTION                    0x00C1
#define ZINITIX_POINT_STATUS_REG                0x0080
#define ZINITIX_ICON_STATUS_REG                 0x00AA
#define ZINITIX_AFE_FREQUENCY                   0x0100
#define ZINITIX_DND_N_COUNT                     0x0122
#define ZINITIX_DND_U_COUNT                     0x0135
#define ZINITIX_RAWDATA_REG                     0x0200
#define ZINITIX_EEPROM_INFO_REG                 0x0018
#define ZINITIX_INT_ENABLE_FLAG                 0x00f0
#define ZINITIX_PERIODICAL_INTERRUPT_INTERVAL   0x00f1
#define ZINITIX_CHECKSUM_RESULT                 0x012c
#define ZINITIX_INIT_FLASH                      0x01d0
#define ZINITIX_WRITE_FLASH                     0x01d1
#define ZINITIX_READ_FLASH                      0x01d2
#define ZINITIX_VENDOR_REG                      0xC000
#define ZINITIX_NVM_REG                         0xC002
#define ZINITIX_VENDOR_ID                       0x001C
#define ZINITIX_VENDOR_ID_VALUE                 0x5A49

#define BIT_PT_CNT_CHANGE               0
#define BIT_DOWN                        1
#define BIT_MOVE                        2
#define BIT_UP                          3
#define BIT_PALM                        4
#define BIT_PALM_REJECT                 5
#define BIT_WAKEUP                      6
#define RESERVED_1                      7
#define BIT_WEIGHT_CHANGE               8
#define BIT_PT_NO_CHANGE                9
#define BIT_REJECT                      10
#define BIT_PT_EXIST                    11
#define RESERVED_2                      12
#define BIT_MUST_ZERO                   13
#define BIT_DEBUG                       14
#define BIT_ICON_EVENT                  15

#define SUB_BIT_EXIST                   0
#define SUB_BIT_DOWN                    1
#define SUB_BIT_MOVE                    2
#define SUB_BIT_UP                      3
#define SUB_BIT_UPDATE                  4
#define SUB_BIT_WAIT                    5

//! @}


//! @name Test Mode (Monitoring Raw Data)
//! @{
#define SEC_DND_N_COUNT                 10
#define SEC_DND_U_COUNT                 2
#define SEC_DND_FREQUENCY               99

#define SEC_PDND_N_COUNT_139_46         27
#define SEC_PDND_U_COUNT_139_46         3
#define SEC_PDND_FREQUENCY_139_46       37

#define SEC_PDND_N_COUNT_119_42         41
#define SEC_PDND_U_COUNT_119_42         9
#define SEC_PDND_FREQUENCY_119_42       37
//! @}


//! preriod raw data interval
#define RAWDATA_DELAY_FOR_HOST      100

#define zinitix_bit_set(val, n)     ((val) &= ~(1 << (n)), (val) |= (1 << (n)))
#define zinitix_bit_clr(val, n)     ((val) &= ~(1 << (n)))
#define zinitix_bit_test(val, n)    ((val) & (1 << (n)))
#define zinitix_swap_v(a, b, t)     ((t) = (a), (a) = (b), (b) = (t))
#define zinitix_swap_16(s)          (((s & 0x00FF) << 8) | ((s & 0xFF00) >> 8))
#define zinitix_abs(a, b)           ((a >= b) ? (a - b) : (b - a))
#define zinitix_max(a, b)           (a > b ? a : b)

typedef struct
{
    uint32_t    ui32Module;
    uint32_t    ui32CS;
    uint32_t    ui32MaxTransSize;
    void        *pIomHandle;
    bool        bOccupied;
} am_devices_iom_tma525_t;
//
//!
//! @note do not need to modify the alignment
//
struct _ts_zinitix_coord
{
    uint16_t    x;
    uint16_t    y;
    uint8_t    width;
    uint8_t    sub_status;
};

struct _ts_zinitix_point_info
{
    uint16_t    status;
    #if TOUCH_POINT_MODE
    uint16_t event_flag;
    #else
    uint8_t    finger_cnt;
    uint8_t    time_stamp;
    #endif
    struct _ts_zinitix_coord    coord[MAX_SUPPORTED_FINGER_NUM];
};

struct ztw_touch_drivers
{
    //! struct touch_drivers driver;
    struct _ts_zinitix_point_info touch_info;
};
typedef struct ztw_touch_drivers *ztw_touch_drv_t;

#ifndef offset_of
    #define offset_of(T, x) ((size_t) &((T *)0)->x)
#endif
#ifndef container_of
    #define container_of(p, T, x) ((T *)((uint8_t *)(p) - offset_of(T, x)))
#endif

typedef int                             rt_bool_t;
typedef long                            rt_base_t;
typedef unsigned long                   rt_ubase_t;

typedef rt_base_t                       rt_err_t;
typedef rt_base_t                       rt_flag_t;
typedef rt_ubase_t                      rt_size_t;
typedef rt_ubase_t                      rt_dev_t;
typedef rt_base_t                       rt_off_t;

typedef signed   char                   rt_int8_t;
typedef signed   short                  rt_int16_t;
typedef signed   int                    rt_int32_t;
typedef unsigned char                   rt_uint8_t;
typedef unsigned short                  rt_uint16_t;
typedef unsigned int                    rt_uint32_t;


struct rt_i2c_msg
{
    uint16_t addr;
    uint16_t flags;
    uint16_t len;
    uint8_t  *buf;
};

struct touch_message
{
    rt_uint16_t x;
    rt_uint16_t y;
    rt_uint8_t event;
};

//*****************************************************************************
//
// Global type definitions.
//
//*****************************************************************************
typedef enum
{
    AM_DEVICES_TMA525_STATUS_SUCCESS,
    AM_DEVICES_TMA525_STATUS_ERROR
} am_devices_tma525_status_t;

typedef enum
{
    AM_DEVICES_TMA525_STATUS_NO_EVENT,
    AM_DEVICES_TMA525_STATUS_TOUCH_DOWN,
    AM_DEVICES_TMA525_STATUS_SIGNIFICANT_DISPLACEMENT,
    AM_DEVICES_TMA525_STATUS_LIFT_OFF
} am_devices_tma525_event_status_t;


//*****************************************************************************
//
// External function definitions.
//
//*****************************************************************************

//*****************************************************************************
//
//! @brief tma525_data_read the Device Driver and IOM for the TMA525 Touch
//! Sensor
//!
//! @param pui8RxBuffer - rx buffer.
//! @param RxNumBytes - read byte size.
//!
//! This function disables power to the IOM module on tma525
//!
//! @return status - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_devices_tma525_data_read(uint8_t *pui8RxBuffer, uint32_t RxNumBytes);

//*****************************************************************************
//
//! @brief tma525_get_point the Device Driver and IOM for the TMA525 Touch
//! Sensor
//!
//! @param x - coordinate X information.
//! @param y - coordinate Y information.
//! @param touch_released - touch status.
//!
//! This function disables power to the IOM module on tma525
//!
//! @return status - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_devices_tma525_get_point(uint16_t *x, uint16_t *y, bool *touch_released);

//*****************************************************************************
//
//! @brief Init the Device Driver and IOM for the TMA525 Touch Sensor
//!
//! @param ui32Module - IOM Module Number.
//! @param *psIOMSettings - Pointer to the IOM Settings.
//! @param **ppIomHandle - Pointer to the Handle for the IOM Instance.
//!
//! This function enables power to the peripheral and waits for a
//! confirmation from the hardware.
//!
//! @return status - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_devices_tma525_init(uint32_t ui32Module, am_hal_iom_config_t *psIOMSettings, void **ppIomHandle);

//*****************************************************************************
//
//! @brief deinit the Device Driver and IOM for the TMA525 Touch Sensor
//!
//! @param ui32Module - IOM Module Number.
//!
//! This function disables power to the IOM module on tma525
//!
//! @return status - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_devices_tma525_deinit(uint32_t ui32Module);


//*****************************************************************************
//
//! @brief Init the Device Driver and IOM for the TMA525 Touch Sensor
//!
//! @param ui32Module - IOM Module Number.
//! @param *psIOMSettings - Pointer to the IOM Settings.
//! @param **ppHandle - Pointer to the Handle for the IOM Device Instance.
//! @param **ppIomHandle - Pointer to the Handle for the IOM Instance.
//!
//! This function enables power to the peripheral and waits for a
//! confirmation from the hardware.
//!
//! @return status - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_devices_tma525_multidrop_iom_init(uint32_t ui32Module,
                                                     am_hal_iom_config_t *psIOMSettings,
                                                     void **ppHandle,
                                                     void **ppIomHandle);

//*****************************************************************************
//
//! @brief Init the Device Driver without IOM for the TMA525 Touch Sensor
//!
//! @param ui32Module - IOM Module Number.
//! @param *pDevConfig - Pointer to the IOM Settings.
//! @param **ppHandle - Pointer to the Handle for the IOM Device Instance.
//! @param **ppIomHandle - Pointer to the Handle for the IOM Instance.
//!
//! Initialize the TMA525 driver for Multidrop if another device has already
//!  initialized the IOM for DMA and multidrop
//!
//! @return status - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_devices_tma525_multidrop_no_iom_init(uint32_t ui32Module, am_hal_iom_config_t *pDevConfig, void **ppHandle, void **ppIomHandle);

//*****************************************************************************
//
//! @brief Deinit the Device Driver for the TMA525 Touch Sensor
//!
//! @param *pHandle - Pointer to the Handle for the IOM Device Instance.
//!
//! This function deinitializes & shuts down the IOM and multidrop TMA525 driver
//!
//! @return status - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_devices_tma525_multidrop_term(void *pHandle);

#ifdef __cplusplus
}
#endif

#endif

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

