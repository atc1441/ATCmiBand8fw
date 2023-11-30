//*****************************************************************************
//
//! @file am_hal_card.h
//!
//! @brief Functions for interfacing with the card host.
//!
//! @addtogroup card_4p Card Functionality for SD/MMC/eMMC/SDIO
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
#ifndef AM_HAL_CARD_H
#define AM_HAL_CARD_H

#ifdef __cplusplus
extern "C"
{
#endif

//
// SD/MMC(eMMC)/SDIO protocol HAL driver APIs
//

#define MMC_CMD_GO_IDLE_STATE           0
#define MMC_CMD_SEND_OP_COND            1
#define MMC_CMD_ALL_SEND_CID            2
#define MMC_CMD_SET_RELATIVE_ADDR       3
#define MMC_CMD_SET_DSR                 4
#define MMC_CMD_SLEEP_AWAKE             5
#define MMC_CMD_SWITCH                  6
#define MMC_CMD_SELECT_CARD             7
#define MMC_CMD_SEND_EXT_CSD            8
#define MMC_CMD_SEND_CSD                9
#define MMC_CMD_SEND_CID                10
#define MMC_CMD_STOP_TRANSMISSION       12
#define MMC_CMD_SEND_STATUS             13
#define MMC_CMD_READ_TUNING_BLOCK       14
#define MMC_CMD_SET_BLOCKLEN            16
#define MMC_CMD_READ_SINGLE_BLOCK       17
#define MMC_CMD_READ_MULTIPLE_BLOCK     18
#define MMC_CMD_SEND_TUNING_BLOCK       19
#define MMC_CMD_SEND_TUNING_BLOCK_HS200 21
#define MMC_CMD_SET_BLOCK_COUNT         23
#define MMC_CMD_WRITE_SINGLE_BLOCK      24
#define MMC_CMD_WRITE_MULTIPLE_BLOCK    25
#define MMC_CMD_ERASE_GROUP_START       35
#define MMC_CMD_ERASE_GROUP_END         36
#define MMC_CMD_ERASE                   38
#define MMC_CMD_APP_CMD                 55
#define MMC_CMD_CMD56                   56
#define MMC_CMD_SPI_READ_OCR            58
#define MMC_CMD_SPI_CRC_ON_OFF          59
#define MMC_CMD_RES_MAN                 62

#define MMC_RSP_PRESENT (1 << 0)
#define MMC_RSP_136     (1 << 1) /* 136 bit response */
#define MMC_RSP_CRC     (1 << 2) /* expect valid crc */
#define MMC_RSP_BUSY    (1 << 3) /* card may send busy */
#define MMC_RSP_OPCODE  (1 << 4) /* response contains opcode */

#define MMC_RSP_NONE (0)
#define MMC_RSP_R1   (MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE)
#define MMC_RSP_R1b  (MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE | MMC_RSP_BUSY)
#define MMC_RSP_R2   (MMC_RSP_PRESENT | MMC_RSP_136 | MMC_RSP_CRC)
#define MMC_RSP_R3   (MMC_RSP_PRESENT)
#define MMC_RSP_R4   (MMC_RSP_PRESENT)
#define MMC_RSP_R5   (MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE)
#define MMC_RSP_R6   (MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE)
#define MMC_RSP_R7   (MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE)

//
// defines for EXT_CSD register in MMC HIGH SPEED cards
//
// Valid POWER_OFF_NOTIFICATION values
#define MMC_EXT_CSD_NO_POWER_NOTIFICATION   (0x00)
#define MMC_EXT_CSD_POWERED_ON              (0x01)
#define MMC_EXT_CSD_POWER_OFF_SHORT         (0x02)
#define MMC_EXT_CSD_POWER_OFF_LONG          (0x03)
#define MMC_EXT_CSD_SLEEP_NOTIFICATION      (0x04)

//
// Supported Bus Width Types
//
#define SD_BUS_WIDTH1     0
#define SD_BUS_WIDTH4     1
#define SD_BUS_WIDTH8     2
#define SD_BUS_WIDTH4_DDR 5
#define SD_BUS_WIDTH8_DDR 6

#define MMC_EXT_MODE_COMMAND_SET (0 << 24)
#define MMC_EXT_MODE_SET_BIT     (1 << 24)
#define MMC_EXT_MODE_CLEAR_BITS  (2 << 24)
#define MMC_EXT_MODE_WRITE_BYTE (3 << 24)

// Added for MMC4.3
#define eMMC_BGA_TYPE 0x01

#define MMC_EXT_REGS_CMD_SET             (191 << 16)
#define MMC_EXT_REGS_POWER_CLASS         (187 << 16)
#define MMC_EXT_REGS_HIGH_SPEED          (185 << 16)
#define MMC_EXT_REGS_BUS_WIDTH           (183 << 16)
#define MMC_EXT_REGS_BOOT_CONFIG         (179 << 16)
#define MMC_EXT_REGS_BOOT_GONFIGPROT     (178 << 16)
#define MMC_EXT_REGS_BOOT_BUSWIDTH       (177 << 16)
#define MMC_EXT_REGS_ERASEGRP_DEFN       (175 << 16)
#define MMC_EXT_REGS_BOOT_WP             (173 << 16)
#define MMC_EXT_REGS_USR_WP              (171 << 16)
#define MMC_EXT_REGS_FW_CONFIG           (169 << 16)
#define MMC_EXT_REGS_SANITIZE_START      165
#define MMC_EXT_REGS_BKOPS_START         164
#define MMC_EXT_REGS_BKOPS_ENABLE        163
#define MMC_EXT_REGS_RST_FUNC            (162 << 16)
#define MMC_EXT_REGS_HPI_MGMT            161
#define MMC_EXT_REGS_PART_ATTRB          (156 << 16)
#define MMC_EXT_REGS_PART_COMPLETE       (155 << 16)
#define MMC_EXT_REGS_GP_SIZE_MULT_4_2    (154 << 16)
#define MMC_EXT_REGS_GP_SIZE_MULT_4_1    (153 << 16)
#define MMC_EXT_REGS_GP_SIZE_MULT_4_0    (152 << 16)
#define MMC_EXT_REGS_GP_SIZE_MULT_3_2    (151 << 16)
#define MMC_EXT_REGS_GP_SIZE_MULT_3_1    (150 << 16)
#define MMC_EXT_REGS_GP_SIZE_MULT_3_0    (149 << 16)
#define MMC_EXT_REGS_GP_SIZE_MULT_2_2    (148 << 16)
#define MMC_EXT_REGS_GP_SIZE_MULT_2_1    (147 << 16)
#define MMC_EXT_REGS_GP_SIZE_MULT_2_0    (146 << 16)
#define MMC_EXT_REGS_GP_SIZE_MULT_1_2    (145 << 16)
#define MMC_EXT_REGS_GP_SIZE_MULT_1_1    (144 << 16)
#define MMC_EXT_REGS_GP_SIZE_MULT_1_0    (143 << 16)
#define MMC_EXT_REGS_ENH_SIZE_MULT2      (142 << 16)
#define MMC_EXT_REGS_ENH_SIZE_MULT1      (141 << 16)
#define MMC_EXT_REGS_ENH_SIZE_MULT0      (140 << 16)
#define MMC_EXT_REGS_ENH_START_ADDR      (136 << 16)
#define MMC_EXT_REGS_EXCEPTION_EVTS_CTRL (56 << 16)
#define MMC_EXT_REGS_CONTEXT_CONF        37
#define MMC_EXT_REGS_PWR_OFF_NOTIFY      34
#define MMC_EXT_REGS_CACHE_CTRL          33
#define MMC_EXT_REGS_FLUSH_CACHE         32
#define MMC_EXT_REGS_HS_TIMING           185
#define MMC_EXT_REGS_PARTITON_CONFIG     179
#define MMC_EXT_REGS_RPMB_SIZE_MULT      168

#define MMC_EXP_PACKED_EVENT_EN  0x8
#define MMC_EXP_SYSPOOL_EVENT_EN 0x4
#define MMC_EXP_DYN_CAP_EVENT_EN 0x2
#define MMC_EXT_SEC_SANITIZE     (0x1 << 6)

#define MMC_EXT_SET_BUS_WIDTH1     (SD_BUS_WIDTH1 << 8)
#define MMC_EXT_SET_BUS_WIDTH4     (SD_BUS_WIDTH4 << 8)
#define MMC_EXT_SET_BUS_WIDTH8     (SD_BUS_WIDTH8 << 8)
#define MMC_EXT_SET_BUS_WIDTH4_DDR (SD_BUS_WIDTH4_DDR << 8)
#define MMC_EXT_SET_BUS_WIDTH8_DDR (SD_BUS_WIDTH8_DDR << 8)

#define MMC_EXT_SET_HIGH_SPEED (1 << 8)
#define MMC_EXT_SET_CMD_ATA    (1 << 4)

// Added for MMC4.3

// CARD TYPE FLAGS
#define HS_26MHZ           0x1
#define HS_52MHZ           0x2
#define HS_DDR_52MHZ_18_3V 0x4
#define HS_DDR_52MHZ_12V   0x4
#define HS200_200MHZ_18V   0x10
#define HS200_200MHZ_12V   0x20
#define HS200_TIMING       0x02

// BOOT ACK FLAGS
#define eMMC_BOOT_ACK_EN 0x01
#define eMMC_NO_BOOT_ACK 0x00

// BOOT PARTITION ENABLE FLAGS
#define eMMC_NO_BOOT_EN    0x00
#define eMMC_BOOT_PART1_EN 0x01
#define eMMC_BOOT_PART2_EN 0x02
#define eMMC_BOOT_USER_EN  0x07

// BOOT PARTITION ACCESS FLAGS
#define eMMC_NO_BOOT_ACCESS 0x00
#define eMMC_BOOT1_ACCESS   0x01
#define eMMC_BOOT2_ACCESS   0x02
#define eMMC_RPMB_ACCESS    0x03
#define eMMC_GPP1_ACCESS    0x04
#define eMMC_GPP2_ACCESS    0x05
#define eMMC_GPP3_ACCESS    0x06
#define eMMC_GPP4_ACCESS    0x07

// BOOT BUS WIDTH FLAGS
#define eMMC_RESET_BUSWDITH  0x00
#define eMMC_RETAIN_BUSWDITH 0x01

// BOOT BUS WIDTH MODES
#define SDR_BCKWARD_COMP_BOOT_MOD 0x00
#define SDR_HS_TIMING_BOOT_MODE   0x01
#define DDR_BOOT_MODE             0x02

// BOOT BUS WIDTHS
#define eMMC_BOOT_BUSWIDTH1 0x00
#define eMMC_BOOT_BUSWIDTH4 0x01
#define eMMC_BOOT_BUSWIDTH8 0x02

// Boot Config Protection Flags
#define PERM_BOOT_CONFIG_PROT_EN  0x01
#define PERM_BOOT_CONFIG_PROT_DIS 0x00
#define PWR_BOOT_CONFIG_PROT_EN   0x01
#define PWR_BOOT_CONFIG_PROT_DIS  0x00

// Boot Write Protection Flags
#define BOOT_PWR_WP_DIS  0x01
#define BOOT_PWR_WP_USE  0x00
#define BOOT_PERM_WP_DIS 0x01
#define BOOT_PERM_WP_USE 0x00
#define BOOT_PERM_WP_EN  0x01
#define BOOT_PERM_NO_WP  0x00
#define BOOT_PWR_WP_EN   0x01
#define BOOT_PWR_NO_WP   0x00

// User Write Protection Flags
#define PERM_PSWD_DIS           0x1
#define PERM_PSWD_EN            0x0
#define CD_PERM_WP_DIS          0x1
#define CD_PERM_WP_EN           0x0
#define US_PERM_WP_GRP_DIS      0x1
#define US_PERM_WP_GRP_EN       0x0
#define US_PWR_WP_GRP_DIS       0x1
#define US_PWR_WP_GRP_EN        0x0
#define US_PERM_WP_CMD28GRP_EN  0x1
#define US_PERM_WP_CMD28GRP_DIS 0x0
#define US_PWR_WP_CMD28GRP_EN   0x1
#define US_PWR_WP_CMD28GRP_DIS  0x0

// FW update Flags
#define FW_UPDATE_EN  0x0
#define FW_UPDATE_DIS 0x1

// Reset Function Enable
#define RST_N_TEMP_DIS 0x0
#define RST_N_PERM_EN  0x1
#define RST_N_PERM_DIS 0x2

// Partition Support Flags
#define ENH_ATTRIBUTE_EN  0x1
#define ENH_ATTRIBUTE_DIS 0x0
#define PARTITIONING_EN   0x1
#define PARTITIONING_DIS  0x0

// Partition Atrribute Features
#define SET_ENH_USR            0x1
#define SET_ENH_GPP1           0x2
#define SET_ENH_GPP2           0x4
#define SET_ENH_GPP3           0x8
#define SET_ENH_GPP4           0x10
#define SET_ALL_PART_ATTRB     0x1F
#define SET_ALL_GPP_PART_ATTRB 0x1E

// Partition Completion Flags
#define PARTITION_COMPLETE 0x1

// Ext CSD Read Flags
#define HS_BOOT_MODE_SUPPORT  0x1
#define DDR_BOOT_MODE_SUPPORT (1 << 1)
#define ALT_BOOT_MODE_SUPPORT (1 << 2)
#define SEC_ER_EN             0x1
#define SEC_GB_CL_EN          0x10

// eMMC44 RPMB Flags
#define AUTH_KEY_PGM_REQ 0x0001
#define WR_CNTR_VAL_REQ  0x0002
#define AUTH_DAT_WR_REQ  0x0003
#define AUTH_DAT_RD_REQ  0x0004
#define RESULT_RD_REQ    0x0005

#define AUTH_KEY_PGM_RESP 0x0100
#define WR_CNTR_VAL_RESP  0x0200
#define AUTH_DAT_WR_RESP  0x0300
#define AUTH_DAT_RD_RESP  0x0400

#define OPR_OK                    0x00
#define WRCNT_EXP_OPR_OK          (0x80)
#define GEN_FAILURE               0x01
#define WRCNT_EXP_GEN_FAILURE     (0x81)
#define AUTH_FAILURE              0x02
#define WRCNT_EXP_AUTH_FAILURE    (0x82)
#define COUNTER_FAILURE           0x03
#define WRCNT_EXP_COUNTER_FAILURE (0x83)
#define ADDR_FAILURE              0x04
#define WRCNT_EXP_ADDR_FAILURE    (0x84)
#define WRITE_FAILURE             0x05
#define WRCNT_EXP_WRITE_FAILURE   (0x85)
#define READ_FAILURE              0x06
#define WRCNT_EXP_READ_FAILURE    (0x86)
#define AUTH_KEY_NOT_PGMMED       0x07

#define RPMB_MAC_KEY_LENGTH        32
#define RPMB_NONCE_LENGTH          16
#define RPMB_DATA_LENGTH           256
#define RPMB_STUFF_LENGTH          196
#define RPMB_COUNTER_LENGTH        4
#define RPMB_ADDR_LENGTH           2
#define RPMB_BLKCNT_LENGTH         2
#define RPMB_RESULT_LENGTH         2
#define RPMB_RSPREQ_LENGTH         2
#define TOTAL_MAC_LEN_PER_FRAME    284
#define MACLEN_EXCL_DATA_PER_FRAME 28
#define RPMB_BLKCNT_MAX(x)         ((x)*128*1024/256)

// EMMC ERROR CODES
#define EMMC_INVALID_BOOT_ACK          -26
#define EMMC_INVALID_BOOT_PART         -27
#define EMMC_INVALID_BOOT_ACCESS       -28
#define EMMC_INVALID_BOOT_BUSWIDTH     -29
#define EMMC_INVALID_BOOT_BUSWIDTHFLAG -30
#define EMMC_INVALID_BOOT_BUSMODE      -31
#define EMMC_INVALID_BOOT_CONFIGPROT   -32
#define EMMC_INVALID_BOOT_WPENABLE     -33
#define EMMC_INVALID_USR_WPENABLE      -34
#define EMMC_INVALID_ERASEGRPDEFN      -35
#define EMMC_INVALID_GPP_NO            -36

// Added for eMMC4.51
#define EMMC_CMD38_ERASE        0x00000000
#define EMMC_CMD38_SECURE_ERASE 0x80000000
#define EMMC_CMD38_SECURE_TRIM1 0x80000001
#define EMMC_CMD38_SECURE_TRIM2 0x80008000
#define EMMC_CMD38_TRIM         0x00000001
#define EMMC_CMD38_DISCARD      0x00000003

#define DEFAULT_CMD6_TIMEOUT_MS  500

#define MMC_STATUS_MASK         (~0x0206BF7F)
#define MMC_STATUS_SWITCH_ERROR (1 << 7)
#define MMC_STATUS_RDY_FOR_DATA (1 << 8)
#define MMC_STATUS_CURR_STATE   (0xf << 9)
#define MMC_STATUS_ERROR        (1 << 19)

//
// SD/MMC/eMMC/SDIO protocl driver APIs
//

static inline uint32_t am_hal_unstuff_bits(uint32_t *resp, uint32_t start, uint32_t size)
{
    const int __size = size;
    const uint32_t __mask = (__size < 32 ? 1 << __size : 0) - 1;
    const int __off = 3 - ((start) / 32);
    const int __shft = (start)&31;
    uint32_t __res;

    __res = resp[__off] >> __shft;
    if (__size + __shft > 32)
    {
        __res |= resp[__off - 1] << ((32 - __shft) % 32);
    }
    return __res &__mask;
}

static inline uint32_t am_hal_unstuff_bytes(uint32_t *ext_csd, uint32_t start, uint32_t size)
{
    int i;
    const uint32_t __size = size;
    uint32_t __res = 0x0;
    const uint8_t *__p = (const uint8_t *)ext_csd;
    for (i = 0; i < __size; i++)
    {
        __res |= __p[start + i] << 8 * i;
    }
    return __res;
}


typedef enum
{
    AM_HAL_ERASE        =          0x00000000,  // Erase
    AM_HAL_DISCARD      =          0x00000003,  // Discard
    AM_HAL_SECURE_ERASE = (int32_t)0x80000000,  // Secure Erase
    AM_HAL_TRIM         =          0x00000001,  // Trim
    AM_HAL_SECURE_TRIM1 = (int32_t)0x80000001,  // Secure Trim Step 1
    AM_HAL_SECURE_TRIM2 = (int32_t)0x80008000,  // Secure Trim Step 2
} am_hal_card_erase_type_t;

//
// SDHC card types
//
typedef enum
{
    AM_HAL_CARD_TYPE_UNKNOWN,
    AM_HAL_CARD_TYPE_MMC,
    AM_HAL_CARD_TYPE_EMMC,
    AM_HAL_CARD_TYPE_SDSC,
    AM_HAL_CARD_TYPE_SDHC,
    AM_HAL_CARD_TYPE_SDIO,
    AM_HAL_CARD_TYPE_COMBO,
} am_hal_card_type_e;

typedef enum
{
    AM_HAL_CARD_STATE_NOT_PRESENT,
    AM_HAL_CARD_STATE_PRESENT,
    AM_HAL_CARD_STATE_IDLE,
    AM_HAL_CARD_STATE_READY,
    AM_HAL_CARD_STATE_IDENT,
    AM_HAL_CARD_STATE_STDY,
    AM_HAL_CARD_STATE_TRANS,
    AM_HAL_CARD_STATE_DATA,
    AM_HAL_CARD_STATE_RCV,
    AM_HAL_CARD_STATE_PRG,
    AM_HAL_CARD_STATE_DIS,
    AM_HAL_CARD_STATE_BTST,
    AM_HAL_CARD_STATE_SLP,
    AM_HAL_CARD_STATE_ERROR,
    AM_HAL_CARD_STATE_PWROFF,
    AM_HAL_CARD_STATE_PWRON,
} am_hal_card_state_e;

typedef struct
{
    am_hal_card_type_e  eType;        // Card type
    uint16_t ui16CmdClass;            // Card command classes
    uint8_t  ui8RCA;                  // Relative card address
    uint32_t ui32MaxBlks;             // Card capacity in blocks
    uint32_t ui32BlkSize;             // Block size in bytes
    uint32_t ui32LogiMaxBlks;         // Card logical capacity in blocks
    uint32_t ui32LogiBlkSize;         // Logical block size in bytes
} am_hal_card_info_t;

typedef enum
{
    AM_HAL_CARD_PWR_ON,
    AM_HAL_CARD_PWR_OFF,
} am_hal_card_pwr_e;

typedef uint32_t (*am_hal_card_pwr_ctrl_func)(am_hal_card_pwr_e eCardPwr);

typedef enum
{
    AM_HAL_CARD_PWR_CTRL_NONE,
    AM_HAL_CARD_PWR_CTRL_SDHC_OFF,
    AM_HAL_CARD_PWR_CTRL_SDHC_OFF_AND_CARD_SLEEP,
    AM_HAL_CARD_PWR_CTRL_SDHC_AND_CARD_OFF,
} am_hal_card_pwr_ctrl_policy_e;

//
// SDHC card
//
typedef struct
{
    am_hal_card_type_e  eType;
    am_hal_card_state_e eState;
    uint32_t bCidValid:1;
    uint32_t bCsdValid:1;
    uint32_t bExtCsdValid:1;
    uint32_t ui32OCR;
    uint8_t  ui8RCA;
    uint32_t ui32CID[4];
    uint32_t ui32CSD[4];
    uint32_t ui32ExtCSD[128];
    uint8_t  ui8SpecVer;
    uint8_t  ui8ExtCSDRev;
    uint8_t  ui8PowerOffNotification;
    bool     bHighCapcity;
    uint32_t ui32CacheSize;
    uint32_t ui32SleepNotificationTimeout;
    uint32_t ui32PowerOffNotificationLongTimeout;
    uint32_t ui32GenericCmd6Timeout;
    uint32_t ui32MaxBlks;
    uint32_t ui32Capacity;
    uint8_t  ui8SecureErase;
    uint8_t  ui8DeviceType;
    uint32_t ui32BlkNum;
    bool     bUseBlkEmulation;
    uint32_t ui32NativeBlkSize;
    uint32_t ui32BlkSize;
    uint32_t ui32RpmbSizeMult;
    uint16_t ui16CmdClass;
    am_hal_card_pwr_ctrl_func pCardPwrCtrlFunc;
    am_hal_card_pwr_ctrl_policy_e eCardPwrCtrlPolicy;
    am_hal_card_cfg_t cfg;
    am_hal_card_host_t *pHost;
}  am_hal_card_t;

//*****************************************************************************
//
//! @brief Power off the SDHC or eMMC CARD
//!
//! @param pCard        - pointer to the card instance.
//!
//! This function is used to power off the SDHC host controller or eMMC card according
//! to the power policy setting in pCard->eCardPwrCtrlPolicy
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_card_pwrctrl_sleep(am_hal_card_t *pCard);

//*****************************************************************************
//
//! @brief Power on the SDHC or eMMC CARD
//!
//! @param pCard        - pointer to the card instance.
//!
//! This function is used to power on the SDHC host controller or eMMC card according
//! to the power policy setting in pCard->eCardPwrCtrlPolicy
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
extern uint32_t am_hal_card_pwrctrl_wakeup(am_hal_card_t *pCard);

//*****************************************************************************
//
//! @brief Get the card instance function
//!
//! @param pHost        - pointer to the card host instance.
//!
//! @param pCard        - pointer to the card instance.
//!
//! This function finds a card and trying to initialize it. if
//! card's initialization succeeds, the card will be 'Trans' state and can do
//! block read, write and erase operations.
//! @return status      - generic or interface specific status..
//
//*****************************************************************************
extern uint32_t am_hal_card_host_find_card(am_hal_card_host_t *pHost, am_hal_card_t *pCard);

//*****************************************************************************
//
//! @brief Set the card operation configurations
//!
//! @param pCard        - pointer to the card instance.
//!
//! @param eType        - card type like eMMC, MMC, SDIO, .etc
//!
//! @param eBusWidth    - card bus width like 1, 4, 8 bit bus width
//!
//! @param ui32Clock    - card bus clock speed
//!
//! @param eIoVoltage   - card bus operation voltage
//!
//! @param eUHSMode     - card UHS mode
//!
//! This function sets the card's type, bus width, clock speed, bus operation voltage and UHS mode
//! but these settings will take effect immediately.
//!
//! @return status      - generic or interface specific status..
//
//*****************************************************************************
extern uint32_t am_hal_card_cfg_set(am_hal_card_t *pCard, am_hal_card_type_e eType,
    am_hal_host_bus_width_e eBusWidth, uint32_t ui32Clock, am_hal_host_bus_voltage_e eIoVoltage,
    am_hal_host_uhs_mode_e eUHSMode);

//*****************************************************************************
//
//! @brief Initialize the card instance function
//!
//! @param pCard              - pointer to the card instance.
//! @param pCardPwrCtrlFunc   - pointer to the User defined Card Pwr Ctrl Func
//! @param eCardPwrCtrlPolicy - Power Control Policy can be:
//!     AM_HAL_CARD_PWR_CTRL_NONE
//!     AM_HAL_CARD_PWR_CTRL_SDHC_OFF
//!     AM_HAL_CARD_PWR_CTRL_SDHC_OFF_AND_CARD_SLEEP
//!     AM_HAL_CARD_PWR_CTRL_SDHC_AND_CARD_OFF
//!
//! This function sends a sequence of commands to initalize the card and let the card
//! enter into the 'Trans' state.
//!
//! @return status      - generic or interface specific status..
//
//*****************************************************************************
extern uint32_t am_hal_card_init(am_hal_card_t *pCard, am_hal_card_pwr_ctrl_func pCardPwrCtrlFunc,
    am_hal_card_pwr_ctrl_policy_e eCardPwrCtrlPolicy);

//*****************************************************************************
//
//! @brief De-Initialize the card instance function
//!
//! @param pCard        - pointer to the card instance.
//!
//! This function resets the card and cleans the card related setting and information.
//!
//! @return status      - generic or interface specific status..
//
//*****************************************************************************
extern uint32_t am_hal_card_deinit(am_hal_card_t *pCard);

//*****************************************************************************
//
//! @brief synchronous block-oriented read function
//!
//! @param pCard        - pointer to the card instance.
//!
//! @param ui32Blk      - start block number
//!
//! @param ui32BlkCnt   - read block count
//!
//! @param pui8Buf      - read buffer
//!
//! This function reads the 'ui32BlkCnt' blocks starting from 'ui32Blk' block and
//! saves the data in the 'pui8Buf' read buffer. The caller will be blocked until all
//! data has been received or failed.
//!
//! @return status      - generic or interface specific status..
//
//*****************************************************************************
extern uint32_t am_hal_card_block_read_sync(am_hal_card_t *pCard, uint32_t ui32Blk, uint32_t ui32BlkCnt, uint8_t *pui8Buf);

//*****************************************************************************
//
//! @brief synchronous block-oriented write function
//!
//! @param pCard        - pointer to the card instance.
//!
//! @param ui32Blk      - start block number
//!
//! @param ui32BlkCnt   - write block count
//!
//! @param pui8Buf      - write buffer
//!
//! This function writes 'ui32BlkCnt' blocks in the 'pui8Buf' write buffer to the card blocks
//! starting from 'ui32Blk' block. The caller will be blocked until all data has been sent
//! out or failed.
//!
//! @return status      - generic or interface specific status..
//
//*****************************************************************************
extern uint32_t am_hal_card_block_write_sync(am_hal_card_t *pCard, uint32_t ui32Blk, uint32_t ui32BlkCnt, uint8_t *pui8Buf);

//*****************************************************************************
//
//! @brief asynchronous block-oriented read function
//!
//! @param pCard        - pointer to the card instance.
//!
//! @param ui32Blk      - start block number
//!
//! @param ui32BlkCnt   - read block count
//!
//! @param pui8Buf      - read buffer
//!
//! This function reads the 'ui32BlkCnt' blocks starting from 'ui32Blk' block and
//! saves the data in the 'pui8Buf' read buffer. The caller will not be blocked,
//! Data is ready in the buffer will be notified by the register callback function.
//!
//! @return status      - generic or interface specific status..
//
//*****************************************************************************
extern uint32_t am_hal_card_block_read_async(am_hal_card_t *pCard, uint32_t ui32Blk, uint32_t ui32BlkCnt, uint8_t *pui8Buf);

//*****************************************************************************
//
//! @brief asynchronous block-oriented write function
//!
//! @param pCard        - pointer to the card instance.
//!
//! @param ui32Blk      - start block number
//!
//! @param ui32BlkCnt   - write block count
//!
//! @param pui8Buf      - write buffer
//!
//! This function writes 'ui32BlkCnt' blocks in the 'pui8Buf' write buffer to the card blocks
//! starting from 'ui32Blk' block. The caller will not be blocked, Completion of data transfer
//! will be notified by the register callback function.
//!
//! @return status      - generic or interface specific status..
//
//*****************************************************************************
extern uint32_t am_hal_card_block_write_async(am_hal_card_t *pCard, uint32_t ui32Blk, uint32_t ui32BlkCnt, uint8_t *pui8Buf);

//*****************************************************************************
//
//! @brief block-oriented erase function
//!
//! @param pCard        - pointer to the card instance.
//!
//! @param ui32Blk      - start block number
//!
//! @param ui32BlkCnt   - write block count
//!
//! @param erasetype   - erase type
//!
//! @param ui32TimeoutMS - erase timeout value in millisecond
//!
//! This function erases 'ui32BlkCnt' blocks staring from 'ui32Blk' block. The caller will be blocked
//! until the erase is done or failed.
//!
//! @return status      - generic or interface specific status..
//
//*****************************************************************************
extern uint32_t am_hal_card_block_erase(am_hal_card_t *pCard, uint32_t ui32Blk, uint32_t ui32BlkCnt, am_hal_card_erase_type_t erasetype, uint32_t ui32TimeoutMS);

//*****************************************************************************
//
//! @brief Get the card status function
//!
//! @param pCard        - pointer to the card instance.
//!
//! @param pui32Status  - the card status
//!
//! This function gets the card status by sending CMD13.
//!
//! @return status      - generic or interface specific status..
//
//*****************************************************************************
extern uint32_t am_hal_card_status(am_hal_card_t *pCard, uint32_t *pui32Status);

//*****************************************************************************
//
//! @brief Get the card state function
//!
//! @param pCard        - pointer to the card instance.
//!
//! This function gets the card state by sending CMD13.
//!
//! @return status      - emmc card state.
//
//*****************************************************************************
extern am_hal_card_state_e am_hal_card_state(am_hal_card_t *pCard);

//*****************************************************************************
//
//! @brief register the card event call back function
//!
//! @param pCard        - pointer to the card instance.
//!
//! @param pfunCallback - function pointer to the call back function.
//!
//! This function registers a card event callback function for async block-oriented
//! read and write functions.
//!
//! @return status      - generic or interface specific status..
//
//*****************************************************************************
extern uint32_t am_hal_card_register_evt_callback(am_hal_card_t *pCard, am_hal_host_event_cb_t pfunCallback);

//*****************************************************************************
//
//! @brief card cid information parse function
//!
//! @param pCard        - pointer to the card instance.
//! @param ui16Offset  - bit start position in the CID data structure.
//! @param ui8Size     - bit length in the CID data structrure.
//!
//! This function parses the specified CID field in the CID data structure and returns
//! as 32bit length integer.
//!
//! @return status      - generic or interface specific status..
//
//*****************************************************************************
extern uint32_t am_hal_card_get_cid_field(am_hal_card_t *pCard, uint16_t ui16Offset, uint8_t ui8Size);

//*****************************************************************************
//
//! @brief card csd information parse function
//!
//! @param pCard        - pointer to the card instance.
//! @param ui16Offset  - bit start position in the CSD data structure.
//! @param ui8Size     - bit length in the CSD data structrure.
//!
//! This function parses the specified CSD field in the CSD data structure and returns
//! as 32bit length integer.
//!
//! @return status      - generic or interface specific status..
//
//*****************************************************************************
extern uint32_t am_hal_card_get_csd_field(am_hal_card_t *pCard, uint16_t ui16Offset, uint8_t ui8Size);

//*****************************************************************************
//
//! @brief card ext csd information parse function
//!
//! @param pCard        - pointer to the card instance.
//! @param ui16Offset  - byte start position in the EXT CSD data structure.
//! @param ui8Size     - byte length in the EXT CSD data structrure.
//!
//! This function parses the specified EXT CSD field in the EXT CSD data structure and returns
//! as 32bit length integer.
//!
//! @return status      - generic or interface specific status..
//
//*****************************************************************************
extern uint32_t am_hal_card_get_ext_csd_field(am_hal_card_t *pCard, uint16_t ui16Offset, uint8_t ui8Size);

//*****************************************************************************
//
//! @brief Get the card information function
//!
//! @param pCard        - pointer to the card instance.
//! @param pCardInfo    - pointer to am_hal_card_info_t structure that contains card info.
//!
//! This function gets the general eMMC card info into am_hal_card_info_t structure.
//!
//! @return status      - generic or interface specific status..
//
//*****************************************************************************
extern uint32_t am_hal_card_get_info(am_hal_card_t *pCard, am_hal_card_info_t *pCardInfo);

//*****************************************************************************
//
//! @brief Set the card mode function
//!
//! @param pCard        - pointer to the card instance.
//! @param ui32Mode     - the card mode
//! @param ui32Timeout  - mode switch timeout
//!
//! This function sets the card mode by sending CMD6.
//!
//! @return status      - generic or interface specific status..
//
//*****************************************************************************
extern uint32_t am_hal_card_mode_switch(am_hal_card_t *pCard, uint32_t ui32Mode, uint32_t ui32Timeout);

//*****************************************************************************
//
//! @brief Get the TX/RX delay setting by calibration
//!
//! @param eUHSMode     - card UHS mode
//! @param ui32Clock    - card bus clock speed
//! @param eBusWidth    - SDIO bus width.
//! @param ui8CalibBuf  - data buffer used to do the calibration
//! @param ui32StartBlk - eMMC start block used to do the calibration
//! @param ui32BlkCnt   - eMMC block number used to do the calibration
//! @param ui8TxRxDelays- an averaged the TX/RX delay values
//!
//! This function get the tx rx delay setting by finding all workable TX/RX delay
//! settings, then an average TX/RX values are returned.
//!
//! @return status      - generic or interface specific status..
//
//*****************************************************************************
extern uint32_t am_hal_card_emmc_calibrate(am_hal_host_uhs_mode_e eUHSMode,
                                           uint32_t ui32Clock,
                                           am_hal_host_bus_width_e eBusWidth,
                                           uint8_t *ui8CalibBuf,
                                           uint32_t ui32StartBlk,
                                           uint32_t ui32BlkCnt,
                                           uint8_t ui8TxRxDelays[2]);

//*****************************************************************************
//
//! @brief Read blocks of data from the card (GEN_CMD) asynchronously
//!
//! @param pCard    - pointer to the card instance.
//! @param ui32Arg  - command arguments
//! @param pui8Buf  - mode switch timeout
//!
//! @return status  - generic or interface specific status..
//
//*****************************************************************************
extern uint32_t am_hal_card_cmd56_read_async(am_hal_card_t *pCard, uint32_t ui32Arg, uint8_t *pui8Buf);

//*****************************************************************************
//
//! @brief Read blocks of data from the card (GEN_CMD) synchronously
//!
//! @param pCard    - pointer to the card instance.
//! @param ui32Arg  - command arguments
//! @param pui8Buf  - mode switch timeout
//!
//! @return status  - generic or interface specific status..
//
//*****************************************************************************
extern uint32_t am_hal_card_cmd56_read_sync(am_hal_card_t *pCard, uint32_t ui32Arg, uint8_t *pui8Buf);

//*****************************************************************************
//
//! @brief emmc rpmb read write function
//!
//! @param pCard        - pointer to the card instance.
//!
//! @param pui8Buf      - write/read buffer
//!
//! @param bRead        - eMMC rpmb write/read direction
//!
//! @param bRelWrite    - eMMC rpmb is reliable write type.
//!
//! This function only support for eMMC rpmb operation.
//!
//! @return status      - generic or interface specific status..
//
//*****************************************************************************
extern uint32_t
am_hal_card_block_rpmb_rw(am_hal_card_t *pCard, uint8_t *pui8Buf, bool bRead, bool bRelWrite);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_CARD_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

