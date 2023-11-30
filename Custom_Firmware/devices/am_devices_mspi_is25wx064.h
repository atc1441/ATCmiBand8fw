//*****************************************************************************
//
//! @file am_devices_mspi_is25wx064.h
//!
//! @brief ISSI Serial NOR SPI Flash driver.
//!
//! @addtogroup mspi_serial_flash IS25WX064 MSPI FLASH Driver
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


#ifndef AM_DEVICES_MSPI_IS25WX064_H
#define AM_DEVICES_MSPI_IS25WX064_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! @name Device specific definitions for the flash size information
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_IS25WX064_PAGE_SIZE       0x100         //256 bytes, minimum program unit
#define AM_DEVICES_MSPI_IS25WX064_SECTOR_SIZE     0x20000       //128K bytes
#define AM_DEVICES_MSPI_IS25WX064_SECTOR_SHIFT    17            //128K bytes per sector
#define AM_DEVICES_MSPI_IS25WX064_MAX_BLOCKS      256
#define AM_DEVICES_MSPI_IS25WX064_MAX_SECTORS     128           // Sectors within 4-byte address range.
//! @}

//*****************************************************************************
//
//! @name Global definitions for Serial SPI flash commands
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_IS25WX064_PAGE_PROGRAM                              0x02
#define AM_DEVICES_MSPI_IS25WX064_PAGE_PROGRAM_4B                           0x12
#define AM_DEVICES_MSPI_IS25WX064_READ                                      0x03
#define AM_DEVICES_MSPI_IS25WX064_WRITE_DISABLE                             0x04
#define AM_DEVICES_MSPI_IS25WX064_READ_STATUS                               0x05
#define AM_DEVICES_MSPI_IS25WX064_WRITE_ENABLE                              0x06
#define AM_DEVICES_MSPI_IS25WX064_FAST_READ                                 0x0B
#define AM_DEVICES_MSPI_IS25WX064_FAST_READ_4B                              0x0C
#define AM_DEVICES_MSPI_IS25WX064_READ_4B                                   0x13
#define AM_DEVICES_MSPI_IS25WX064_SUBSECTOR_ERASE                           0x20
#define AM_DEVICES_MSPI_IS25WX064_RESET_ENABLE                              0x66
#define AM_DEVICES_MSPI_IS25WX064_RESET_MEMORY                              0x99
#define AM_DEVICES_MSPI_IS25WX064_READ_ID                                   0x9F
#define AM_DEVICES_MSPI_IS25WX064_ENTER_4B                                  0xB7
#define AM_DEVICES_MSPI_IS25WX064_CHIP_ERASE                                0xC7
#define AM_DEVICES_MSPI_IS25WX064_SECTOR_ERASE                              0xD8
#define AM_DEVICES_MSPI_IS25WX064_EXIT_4B                                   0xE9
#define AM_DEVICES_MSPI_IS25WX064_READ_VOLATILE_CR                          0x85
#define AM_DEVICES_MSPI_IS25WX064_WRITE_VOLATILE_CR                         0x81
#define AM_DEVICES_MSPI_IS25WX064_READ_NONVOLATILE_CR                       0xB5
#define AM_DEVICES_MSPI_IS25WX064_WRITE_NONVOLATILE_CR                      0xB1
#define AM_DEVICES_MSPI_IS25WX064_ENTER_4BYTE_ADDRESS_MODE                  0xB7
#define AM_DEVICES_MSPI_IS25WX064_EXIT_4BYTE_ADDRESS_MODE                   0xE9
#define AM_DEVICES_MSPI_IS25WX064_READ_FLAG_STATUS_REGISTER                 0x70
//! @}

//*****************************************************************************
//
//! @name Global definitions for 1-8-8 SPI flash commands
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_IS25WX064_OCTAL_1_1_8_FAST_READ_4B                  0x7C
#define AM_DEVICES_MSPI_IS25WX064_OCTAL_1_8_8_FAST_READ_4B                  0xCC
#define AM_DEVICES_MSPI_IS25WX064_OCTAL_1_1_8_FAST_PGM_4B                   0x84
#define AM_DEVICES_MSPI_IS25WX064_OCTAL_1_8_8_FAST_PGM_4B                   0x8E
//! @}

//*****************************************************************************
//
//! @name  Global definitions for Octal DDR SPI flash commands
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_IS25WX064_OCTA_FAST_READ                            0x0B0BU  /*!< Octa IO Read DTR                                 */
#define AM_DEVICES_MSPI_IS25WX064_OCTA_READ_DTR_CMD                         0xFDFDU  /*!< Octa IO Read DTR                                 */
#define AM_DEVICES_MSPI_IS25WX064_OCTA_PAGE_PROG_CMD                        0x1212U  /*!< Octa Page Program                                */
#define AM_DEVICES_MSPI_IS25WX064_OCTA_SUBSECTOR_ERASE_4K_CMD               0x2121U  /*!< Octa SubSector Erase 4KB                         */
#define AM_DEVICES_MSPI_IS25WX064_OCTA_SECTOR_ERASE_128K_4BYTE              0xDCDCU  /*!< Octa Sector Erase 128KB 3                        */
#define AM_DEVICES_MSPI_IS25WX064_OCTA_CHIP_ERASE_CMD                       0x6060U  /*!< Octa Bulk Erase                                  */
#define AM_DEVICES_MSPI_IS25WX064_OCTA_SECTOR_ERASE_128K                    0xD8D8U  /*!< Octa IO Read DTR                                 */
//! @}

//*****************************************************************************
//
//! @name  Setting commands
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_IS25WX064_OCTA_WRITE_ENABLE_CMD                     0x0606U   /*!< Octa Write Enable                               */
#define AM_DEVICES_MSPI_IS25WX064_OCTA_WRITE_DISABLE_CMD                    0x0404U   /*!< Octa Write Disable                              */
#define AM_DEVICES_MSPI_IS25WX064_OCTA_PROG_ERASE_SUSPEND_CMD               0x7575U   /*!< Octa Program/Erase suspend                      */
#define AM_DEVICES_MSPI_IS25WX064_OCTA_PROG_ERASE_RESUME_CMD                0x7A7AU   /*!< Octa Program/Erase resume                       */
#define AM_DEVICES_MSPI_IS25WX064_OCTA_ENTER_DEEP_POWER_DOWN_CMD            0xB9B9U   /*!< Octa Enter deep power down                      */
//! @}

//*****************************************************************************
//
//! @name  Reset commands
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_IS25WX064_OCTA_NOP_CMD                              0x0000U   /*!< Octa No operation                               */ //NOP becasue 00h is invalid cmd for IS25LX512
#define AM_DEVICES_MSPI_IS25WX064_OCTA_RESET_ENABLE_CMD                     0x6666U   /*!< Octa Reset Enable                               */
#define AM_DEVICES_MSPI_IS25WX064_OCTA_RESET_MEMORY_CMD                     0x9999U   /*!< Octa Reset Memory                               */
//! @}

//*****************************************************************************
//
//! @name  Register Commands (OPI)
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_IS25WX064_OCTA_READ_ID_CMD                          0x9F9FU   /*!< Octa Read IDentification                        */
#define AM_DEVICES_MSPI_IS25WX064_OCTA_READ_SERIAL_FLASH_DISCO_PARAM_CMD    0x5A5AU   /*!< Octa Read Serial Flash Discoverable Parameter   */
#define AM_DEVICES_MSPI_IS25WX064_OCTA_READ_STATUS_REG_CMD                  0x0505U   /*!< Octa Read Status Register                       */
#define AM_DEVICES_MSPI_IS25WX064_OCTA_WRITE_STATUS_REG_CMD                 0x0101U   /*!< Octa Write Status Register                      */
#define AM_DEVICES_MSPI_IS25WX064_OCTA_READ_FLAG_STATUS_REG_CMD             0x7070U   /*!< Read Flag Status Register                       */
#define AM_DEVICES_MSPI_IS25WX064_OCTA_CLEAR_FLAG_STATUS_REG_CMD            0x5050U   /*!< Clear Flag Status Register                      */
#define AM_DEVICES_MSPI_IS25WX064_OCTA_READ_NV_CFG_REG_CMD                  0xB5B5U   /*!< Read non-volatile configuration Register        */
#define AM_DEVICES_MSPI_IS25WX064_OCTA_WRITE_NV_CFG_REG_CMD                 0xB1B1U   /*!< Write non-volatile configuration Register       */
#define AM_DEVICES_MSPI_IS25WX064_OCTA_READ_V_CFG_REG_CMD                   0x8585U   /*!< Read configuration Register                     */
#define AM_DEVICES_MSPI_IS25WX064_OCTA_WRITE_V_CFG_REG_CMD                  0x8181U   /*!< Write configuration Register                    */

#define AM_DEVICES_MSPI_IS25WX064_OCTA_READ_LOCK_REG_CMD                    0x2D2DU   /*!< Octa Read lock Register                         */
#define AM_DEVICES_MSPI_IS25WX064_OCTA_WRITE_LOCK_REG_CMD                   0x2C2CU   /*!< Octa Write lock Register                        */

#define AM_DEVICES_MSPI_IS25WX064_OCTA_READ_DPB_REG_CMD                     0xE8E8U   /*!< Octa Read DPB register                          */
#define AM_DEVICES_MSPI_IS25WX064_OCTA_WRITE_DPB_REG_CMD                    0xE5E5U   /*!< Octa Write DPB register                         */
#define AM_DEVICES_MSPI_IS25WX064_OCTA_READ_SPB_STATUS_CMD                  0xE2E2U   /*!< Octa Read SPB status                            */
#define AM_DEVICES_MSPI_IS25WX064_OCTA_WRITE_SPB_BIT_CMD                    0xE3E3U   /*!< Octa SPB bit program                            */
#define AM_DEVICES_MSPI_IS25WX064_OCTA_ERASE_ALL_SPB_CMD                    0xE4E4U   /*!< Octa Erase all SPB bit                          */
#define AM_DEVICES_MSPI_IS25WX064_OCTA_WRITE_PROTECT_MNGT_REG_CMD           0x6868U   /*!< Write Protect selection                               */
#define AM_DEVICES_MSPI_IS25WX064_OCTA_READ_PASSWORD_REGISTER_CMD           0x2727U   /*!< Octa Read Password                              */
#define AM_DEVICES_MSPI_IS25WX064_OCTA_WRITE_PASSWORD_REGISTER_CMD          0x2828U   /*!< Octa Write Password                             */
#define AM_DEVICES_MSPI_IS25WX064_OCTA_PASSWORD_UNLOCK_CMD                  0x2929U   /*!< Octa Unlock Password                            */


#define AM_DEVICES_MSPI_IS25WX064_3BYTE_ADDRESS                         0xFF
#define AM_DEVICES_MSPI_IS25WX064_4BYTE_ADDRESS                         0xFE
//! @}


//*****************************************************************************
//
//! @name  Status Register
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_IS25WX064_SR_WIP                               0x01U   /*!< Write in progress                                       */
#define AM_DEVICES_MSPI_IS25WX064_SR_WEL                               0x02U   /*!< Write enable latch                                      */
#define AM_DEVICES_MSPI_IS25WX064_SR_TB                                0x20U   /*!< Top / bottom  selected                                  */
#define AM_DEVICES_MSPI_IS25WX064_SR_PB                                0x5CU   /*!< Block protected against program and erase operations    */
//! @}

//*****************************************************************************
//
//! @name Non-volatile and Volatile Configuration Register
//! Address : 0x00 IO mode
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_IS25WX064_CR_REG_ADDR_00                       0x00U  /*!< Volatile CR register address 0x00            */
#define AM_DEVICES_MSPI_IS25WX064_CR_IO_MODE_EXTENDED_SPI              0xFFU        /*!< Extended SPI                                       */
#define AM_DEVICES_MSPI_IS25WX064_CR_IO_MODE_EXTENDED_SPI_WO_DQS       0xDFU        /*!< Extended SPI without DQS                           */
#define AM_DEVICES_MSPI_IS25WX064_CR_IO_MODE_OCTAL_DDR                 0xE7U        /*!< Octal DDR                                          */
#define AM_DEVICES_MSPI_IS25WX064_CR_IO_MODE_OCTAL_DDR_WO_DQS          0xC7U        /*!< Octal DDR without DQS                              */
//! @}

//*****************************************************************************
//
//! @name  Address : 0x01 Dummy cycle
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_IS25WX064_CR_REG_ADDR_01                       0x01U  /*!< VCR register address 0x01                    */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_DEF0                           0x00U        /*!< Default Dummy cycle                                */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_DEF1                           0x1FU        /*!< Default Dummy cycle                                */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_01_CYCLES                      0x01U        /*!< 1 Dummy cycles                                     */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_02_CYCLES                      0x02U        /*!< 2 Dummy cycles                                     */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_03_CYCLES                      0x03U        /*!< 3 Dummy cycles                                     */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_04_CYCLES                      0x04U        /*!< 4 Dummy cycles                                     */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_05_CYCLES                      0x05U        /*!< 5 Dummy cycles                                     */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_06_CYCLES                      0x06U        /*!< 6 Dummy cycles                                     */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_07_CYCLES                      0x07U        /*!< 7 Dummy cycles                                     */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_08_CYCLES                      0x08U        /*!< 8 Dummy cycles                                     */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_09_CYCLES                      0x09U        /*!< 9 Dummy cycles                                     */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_0A_CYCLES                      0x0AU        /*!< 10 Dummy cycles                                    */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_0B_CYCLES                      0x0BU        /*!< 11 Dummy cycles                                    */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_0C_CYCLES                      0x0CU        /*!< 12 Dummy cycles                                    */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_0D_CYCLES                      0x0DU        /*!< 13 Dummy cycles                                    */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_0E_CYCLES                      0x0EU        /*!< 14 Dummy cycles                                    */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_0F_CYCLES                      0x0FU        /*!< 15 Dummy cycles                                    */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_10_CYCLES                      0x10U        /*!< 16 Dummy cycles                                    */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_11_CYCLES                      0x11U        /*!< 17 Dummy cycles                                    */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_12_CYCLES                      0x12U        /*!< 18 Dummy cycles                                    */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_13_CYCLES                      0x13U        /*!< 19 Dummy cycles                                    */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_14_CYCLES                      0x14U        /*!< 20 Dummy cycles                                    */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_15_CYCLES                      0x15U        /*!< 21 Dummy cycles                                    */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_16_CYCLES                      0x16U        /*!< 22 Dummy cycles                                    */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_17_CYCLES                      0x17U        /*!< 23 Dummy cycles                                    */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_18_CYCLES                      0x18U        /*!< 24 Dummy cycles                                    */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_19_CYCLES                      0x19U        /*!< 25 Dummy cycles                                    */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_1A_CYCLES                      0x1AU        /*!< 26 Dummy cycles                                    */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_1B_CYCLES                      0x1BU        /*!< 27 Dummy cycles                                    */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_1C_CYCLES                      0x1CU        /*!< 28 Dummy cycles                                    */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_1D_CYCLES                      0x1DU        /*!< 29 Dummy cycles                                    */
#define AM_DEVICES_MSPI_IS25WX064_CR_DC_1E_CYCLES                      0x1EU        /*!< 30 Dummy cycles                                    */
//! @}

//*****************************************************************************
//
//! @name Address : 0x03 ODS
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_IS25WX064_CR_REG_ADDR_03                       0x03U  /*!< VCR register address 0x03                    */
#define AM_DEVICES_MSPI_IS25WX064_CR_ODS_50_OHM                        0xFFU        /*!< 50_OHM ODS                                         */
#define AM_DEVICES_MSPI_IS25WX064_CR_ODS_35_OHM                        0xFEU        /*!< 35_OHM ODS                                         */
#define AM_DEVICES_MSPI_IS25WX064_CR_ODS_25_OHM                        0xFDU        /*!< 25_OHM ODS                                         */
#define AM_DEVICES_MSPI_IS25WX064_CR_ODS_18_OHM                        0xFCU        /*!< 18_OHM ODS                                         */
//! @}

//*****************************************************************************
//
//! @name Address : 0x05 Address mode
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_IS25WX064_CR_REG_ADDR_05                       0x05U  /*!< VCR register address 0x05                    */
#define AM_DEVICES_MSPI_IS25WX064_CR_ADDR_MODE_3_BYTE                  0xFFU        /*!< 3-Byte address mode                                */
#define AM_DEVICES_MSPI_IS25WX064_CR_ADDR_MODE_4_BYTE                  0xFEU        /*!< 4-Byte address mode                                */
//! @}

//*****************************************************************************
//
//! @name Address : Address : 0x06 XIP configuration
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_IS25WX064_CR_REG_ADDR_06                       0x06U  /*!< VCR register address 0x06                    */
#define AM_DEVICES_MSPI_IS25WX064_CR_XIP_DISABLE                       0xFFU        /*!< XIP disable                                        */
#define AM_DEVICES_MSPI_IS25WX064_CR_XIP_ENABLE                        0xFEU        /*!< XIP enable                                         */
//! @}

//*****************************************************************************
//
//! @name Address : 0x07 Wrap configuration
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_IS25WX064_CR_REG_ADDR_07                       0x07U  /*!< VCR register address 0x07                    */
#define AM_DEVICES_MSPI_IS25WX064_CR_WRAP_CONTINUOUS                   0xFFU        /*!< Continuous                                         */
#define AM_DEVICES_MSPI_IS25WX064_CR_WRAP_64_BYTE                      0xFEU        /*!< 64 Byte wrap                                       */
#define AM_DEVICES_MSPI_IS25WX064_CR_WRAP_32_BYTE                      0xFDU        /*!< 32 Byte wrap                                       */
#define AM_DEVICES_MSPI_IS25WX064_CR_WRAP_16_BYTE                      0xFCU        /*!< 16 Byte wrap                                       */
//! @}

//*****************************************************************************
//
//! @name Flag Status Register
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_IS25WX064_FLAGSR_ADDRESSING                    0x01U        /*!< 3-byte / 4-byte addressing                          */
#define AM_DEVICES_MSPI_IS25WX064_FLAGSR_PROT_ERR                      0x02U        /*!< Protection Error Bit                                */
#define AM_DEVICES_MSPI_IS25WX064_FLAGSR_PSB                           0x04U        /*!< Program suspend bit                                 */
#define AM_DEVICES_MSPI_IS25WX064_FLAGSR_ESB                           0x40U        /*!< Erase suspend bit                                   */
#define AM_DEVICES_MSPI_IS25WX064_FLAGSR_P_FAIL                        0x10U        /*!< Program fail flag                                   */
#define AM_DEVICES_MSPI_IS25WX064_FLAGSR_E_FAIL                        0x20U        /*!< Erase fail flag                                     */
#define AM_DEVICES_MSPI_IS25WX064_FLAGSR_WIPB                          0x80U        /*!< WIP# (0 is BUSY)                              */
//! @}

//*****************************************************************************
//
//! @name Global definitions for the flash status register
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_IS25WX064_WEL       0x00000002        // Write enable latch
#define AM_DEVICES_MSPI_IS25WX064_WIP       0x00000001        // Write in progress
//! @}

//*****************************************************************************
//
//! @name Device specific identification.
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_IS25WX064_ID        0x009D5B17
#define AM_DEVICES_MSPI_IS25WX064_ID_MASK   0x00FFFFFF
//! @}

//*****************************************************************************
//
//! @name Global definitions for the flash status register
//! @{
//
//*****************************************************************************
#define AM_DEVICES_IS25WX064_RSTE           0x00000010        // Reset enable
#define AM_DEVICES_IS25WX064_WEL            0x00000002        // Write enable latch
#define AM_DEVICES_IS25WX064_WIP            0x00000001        // Operation in progress
//! @}

//*****************************************************************************
//
//! @name Global definitions for the MSPI instance to use.
//! @{
//
//*****************************************************************************
#define AM_DEVICES_MSPI_IS25WX064_MSPI_INSTANCE     0

#define AM_DEVICES_MSPI_IS25WX064_MAX_DEVICE_NUM    1
//! @}

//*****************************************************************************
//
// Global type definitions.
//
//*****************************************************************************

//
//! Device status
//
typedef enum
{
    AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS,
    AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR,
    AM_DEVICES_MSPI_IS25WX064_STATUS_BUSY,
    AM_DEVICES_MSPI_IS25WX064_STATUS_READY
} am_devices_mspi_is25wx064_status_t;

//
//! Device config struct
//
typedef struct
{
    am_hal_mspi_device_e eDeviceConfig;
    am_hal_mspi_clock_e eClockFreq;
    uint32_t *pNBTxnBuf;
    uint32_t ui32NBTxnBufLength;
    uint32_t ui32ScramblingStartAddr;
    uint32_t ui32ScramblingEndAddr;
} am_devices_mspi_is25wx064_config_t;

//
//! MSPI for timing config
//
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
typedef struct
{
    uint32_t ui32Turnaround;
    uint32_t ui32Rxneg;
    uint32_t ui32Rxdqsdelay;
} am_devices_mspi_is25wx064_timing_config_t;
#elif defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
typedef struct
{
    bool            bTxNeg;
    bool            bRxNeg;
    bool            bRxCap;
    uint8_t         ui8TxDQSDelay;
    uint8_t         ui8RxDQSDelay;
    uint8_t         ui8Turnaround;
} am_devices_mspi_is25wx064_timing_config_t;
#endif

//*****************************************************************************
//
// External function definitions.
//
//*****************************************************************************

//*****************************************************************************
//
//! @brief Initialize the mspi_flash driver.
//!
//! @param ui32Module - MSPI instance
//! @param psMSPISettings - MSPI device structure describing the target spi flash.
//! @param ppHandle - Device handle which needs to be return
//! @param ppMspiHandle - MSPI handle which needs to be return
//!
//! This function should be called before any other am_devices_spiflash
//! functions. It is used to set tell the other functions how to communicate
//! with the external spiflash hardware.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_is25wx064_init(uint32_t ui32Module,
                               am_devices_mspi_is25wx064_config_t *psMSPISettings,
                               void **ppHandle,
                               void **ppMspiHandle);

//*****************************************************************************
//
//! @brief DeInitialize the mspi_flash driver.
//!
//! @param pHandle - MSPI Device handle.
//!
//! @return status.
//
//*****************************************************************************
extern uint32_t am_devices_mspi_is25wx064_deinit(void *pHandle);

//*****************************************************************************
//
//! @brief Reads the ID of the external flash and returns the value.
//!
//! @param pHandle - MSPI Device handle.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_is25wx064_id(void *pHandle);

//*****************************************************************************
//! @brief Reset the external flash.
//!
//! @param pHandle - MSPI Device handle.
//! @param pDevCconfig - Pointer to the current device config
//!
//! This function resets the external flash
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_is25wx064_reset(void *pHandle,
                                        am_hal_mspi_dev_config_t *pDevCconfig);

//*****************************************************************************
//
//! @brief Reads the contents of the external flash into a buffer.
//!
//! @param pHandle - MSPI Device handle.
//! @param pui8RxBuffer - Buffer to store the received data from the flash
//! @param ui32ReadAddress - Address of desired data in external flash
//! @param ui32NumBytes - Number of bytes to read from external flash
//! @param bWaitForCompletion - Flag until operation complete
//!
//! This function reads the external flash at the provided address and stores
//! the received data into the provided buffer location. This function will
//! only store ui32NumBytes worth of data.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_is25wx064_read(void *pHandle,
                                               uint8_t *pui8RxBuffer,
                                               uint32_t ui32ReadAddress,
                                               uint32_t ui32NumBytes,
                                               bool bWaitForCompletion);

//*****************************************************************************
//
//! @brief Programs the given range of flash addresses.
//!
//! @param pHandle - Device handle of the external flash
//! @param pui8TxBuffer - Buffer to write the external flash data from
//! @param ui32WriteAddress - Address to write to in the external flash
//! @param ui32NumBytes - Number of bytes to write to the external flash
//! @param bWaitForCompletion - Flag until operation complete
//!
//! This function uses the data in the provided pui8TxBuffer and copies it to
//! the external flash at the address given by ui32WriteAddress. It will copy
//! exactly ui32NumBytes of data from the original pui8TxBuffer pointer. The
//! user is responsible for ensuring that they do not overflow the target flash
//! memory or underflow the pui8TxBuffer array
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_is25wx064_write(void *pHandle,
                                                uint8_t *pui8TxBuffer,
                                                uint32_t ui32WriteAddress,
                                                uint32_t ui32NumBytes,
                                                bool bWaitForCompletion);

//*****************************************************************************
//
//! @brief Erases the entire contents of the external flash
//!
//! @param pHandle - MSPI Device handle.
//!
//! This function uses the "Bulk Erase" instruction to erase the entire
//! contents of the external flash.
//
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_is25wx064_mass_erase(void *pHandle);

//*****************************************************************************
//
//! @brief Erases the contents of a single sector of flash
//!
//! @param pHandle - MSPI Device handle.
//! @param ui32SectorAddress - Address to erase in the external flash
//!
//! This function erases a single sector of the external flash as specified by
//! ui32EraseAddress. The entire sector where ui32EraseAddress will
//! be erased.
//
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_is25wx064_sector_erase(void *pHandle,
                                                    uint32_t ui32SectorAddress);

//*****************************************************************************
//
//! @brief Sets up the MSPI and external flash into XIP mode.
//!
//! @param pHandle - Device handle of the external flash
//!
//! This function sets the external flash device and the MSPI into XIP mode.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_is25wx064_enable_xip(void *pHandle);

//*****************************************************************************
//
//! @brief Removes the MSPI and external flash from XIP mode.
//!
//! @param pHandle - Device handle of the external flash
//!
//! This function removes the external device and the MSPI from XIP mode.
//
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_is25wx064_disable_xip(void *pHandle);

//*****************************************************************************
//
//! @brief Sets up the MSPI and external FLASH into scrambling mode.
//!
//! @param pHandle - Device handle of the external flash
//!
//! This function sets the external flash device and the MSPI into scrambling mode.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_is25wx064_enable_scrambling(void *pHandle);

//*****************************************************************************
//
//! @brief Disable the MSPI and external flash from scrambling mode.
//!
//! @param pHandle - Device handle of the external flash
//!
//! This function removes the external device and the MSPI from XIP mode.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_is25wx064_disable_scrambling(void *pHandle);

//*****************************************************************************
//
//! @brief Reads the contents of the external flash into a buffer.
//!
//! @param pHandle - MSPI Device handle.
//! @param pui8RxBuffer - Buffer to store the received data from the flash
//! @param ui32ReadAddress - Address of desired data in external flash
//! @param ui32NumBytes - Number of bytes to read from external flash
//! @param ui32PauseCondition - Pause the condition
//! @param ui32StatusSetClr - Set status set clear
//! @param pfnCallback - Inside use nonblocking call back function
//! @param pCallbackCtxt - Call back function parameters
//!
//! This function reads the external flash at the provided address and stores
//! the received data into the provided buffer location. This function will
//! only store ui32NumBytes worth of data.
//!
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_is25wx064_read_adv(void *pHandle,
                                           uint8_t *pui8RxBuffer,
                                           uint32_t ui32ReadAddress,
                                           uint32_t ui32NumBytes,
                                           uint32_t ui32PauseCondition,
                                           uint32_t ui32StatusSetClr,
                                           am_hal_mspi_callback_t pfnCallback,
                                           void *pCallbackCtxt);

//*****************************************************************************
//
//! @brief Reads the contents of the external flash into a buffer.
//!
//! @param pHandle - MSPI Device handle.
//! @param pui8RxBuffer - Buffer to store the received data from the flash
//! @param ui32ReadAddress - Address of desired data in external flash
//! @param ui32NumBytes - Number of bytes to read from external flash
//! @param bWaitForCompletion - Flag until operation complete
//!
//! This function reads the external flash at the provided address and stores
//! the received data into the provided buffer location. This function will
//! only store ui32NumBytes worth of data.
//
//! @return 32-bit status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_is25wx064_read_hiprio(void *pHandle,
                                                      uint8_t *pui8RxBuffer,
                                                      uint32_t ui32ReadAddress,
                                                      uint32_t ui32NumBytes,
                                                      bool bWaitForCompletion);

//*****************************************************************************
//
//! @brief Checks FLASH timing and determine a delay setting.
//!
//! @param module - MSPI Module Number.
//! @param pDevCfg - Pointer to the flash device config structure
//! @param psDevTimingCfg - Pointer to the ddr timing config structure
//!
//! This function scans through the delay settings of MSPI DDR mode and selects
//! the best parameter to use by tuning TURNAROUND/RXNEG/RXDQSDELAY0 values.
//! This function is only valid in DDR mode and ENABLEDQS0 = 0.
//!
//! @return 32-bit status, scan result in structure type
//
//*****************************************************************************
extern uint32_t am_devices_mspi_is25wx064_init_timing_check(uint32_t module,
                    am_devices_mspi_is25wx064_config_t *pDevCfg,
                    am_devices_mspi_is25wx064_timing_config_t *psDevTimingCfg);

//*****************************************************************************
//
//! @brief Apply given DDR timing settings to target MSPI instance.
//!
//! @param pHandle - Handle to the FLASH.
//! @param psDevTimingCfg - Pointer to the ddr timing config structure
//!
//! This function applies the ddr timing settings to the selected mspi instance.
//! This function must be called after MSPI instance is initialized into
//! ENABLEFINEDELAY0 = 1 mode.
//!
//! @return status
//
//*****************************************************************************
extern uint32_t am_devices_mspi_is25wx064_apply_ddr_timing(void *pHandle,
                am_devices_mspi_is25wx064_timing_config_t *psDevTimingCfg);



#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_MSPI_IS25WX064_H
//*****************************************************************************
// End Doxygen group.
//! @}
//
//*****************************************************************************
