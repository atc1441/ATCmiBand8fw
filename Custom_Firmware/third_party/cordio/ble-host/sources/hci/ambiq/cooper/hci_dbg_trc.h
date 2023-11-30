//*****************************************************************************
//
//! @file hci_dbg_trc.h
//!
//! @brief Abstract debug log from BTLE radio application interface.
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
#include "wsf_types.h"
#include "bda.h"

#ifndef HCI_DBG_TRC_H
#define HCI_DBG_TRC_H

#define LE_CHNL_MAP_LEN     0x05
#define RAND_NB_LEN         0x08
#define SESS_KEY_DIV_LEN    0x08
#define INIT_VECT_LEN       0x04
#define LE_FEATS_LEN        0x08

//By default enable the Cooper debug trace
#define ENABLE_BLE_CTRL_TRACE
#ifdef ENABLE_BLE_CTRL_TRACE
//By default only enable the Cooper custom debug trace(TRC_CUSTOM_BIT)
#define TRC_CUSTOM_ONLY     (1u)

#define HCI_TRACE_TYPE      0x6
#define TRC_CFG_WORD_LEN    0x2
#define MAX_TRACE_LEN       100
#define BLE_RX_DESC_SIZE    28
#define TRC_TIME_UNIT       3125

typedef struct
{
    uint16_t t_ms;
    uint8_t  t_sec;
    uint8_t  t_min;
    uint8_t  t_hur;
}time_type_t;

typedef enum
{
    TRACE_NULL                    = 0x00000000,
    /// Kernel message
    TRACE_KE_MSG_BIT              = 0x00000001,
    /// Kernel timer
    TRACE_KE_TMR_BIT              = 0x00000002,
    /// Kernel event
    TRACE_KE_EVT_BIT              = 0x00000004,
    /// Memory allocation and deallocation
    TRACE_MEM_BIT                 = 0x00000008,
    /// Sleep mode
    TRACE_SLEEP_BIT               = 0x00000010,
    /// Software Assert
    TRACE_SW_ASS_BIT              = 0x00000020,
    /// Programming of the exchange table, updating of the event counter and handling of "End of Event" interrupt
    TRACE_PROG_BIT                = 0x00000040,
    /// BLE Control structure
    TRACE_CS_BLE_BIT              = 0x00000080,
    /// Processing of RX descriptors at each LLD driver
    TRACE_RX_DESC_BIT             = 0x00000200,
    /// LLCP transmission, reception and acknowledgment
    TRACE_LLCP_BIT                = 0x00000400,
    /// L2CAP transmission, reception and acknowledgment
    TRACE_L2CAP_BIT               = 0x00001000,
    /// Scheduling request, cancellation, shift and remove
    TRACE_ARB_BIT                 = 0x00002000,
    /// LLC state transition
    TRACE_LLC_STATE_TRANS_BIT     = 0x00004000,
    /// LC state transition
    TRACE_LC_STATE_TRANS_BIT      = 0x00008000,
    /// HCI messages (in Full embedded mode)
    TRACE_HCI_BIT                 = 0x00010000,
    /// Custom trace
    TRC_CUSTOM_BIT                = 0x00080000,

}ble_trace_cfg;

// configuration for which trace to be output
#define TRACE_BITMAP  (TRC_CUSTOM_BIT)

// Debug trace packet type
typedef enum
{
    /// Trace message
    TRACE      = 0x01,
    /// Configuration message
    TRACE_CFG  = 0x02,
    /// Acknowledgment message
    TRACE_ACK  = 0x03
}dbg_trc_pkt_type;

typedef enum
{
    KE_MSG_SEND         = 0x00,
    KE_MSG_HANDLED      = 0x01,
    KE_TMR_SET          = 0x02,
    KE_TMR_CLR          = 0x03,
    KE_TMR_EXP          = 0x04,
    KE_EVT_SET          = 0x05,
    KE_EVT_HANDLED      = 0x06,
    MEM_ALLOC           = 0x07,
    MEM_FREE            = 0x08,
    SLEEP_ENTER         = 0x09,
    SLEEP_WAKEUP        = 0x0A,
    ASSERT_WARNING      = 0x0B,
    ASSERT_ERROR        = 0x0C,
    ET_PROG             = 0x0D,
    CONN_EVT_CNT        = 0x0E,
    FRM_CMP_BLE         = 0x10,
    CS_BLE              = 0x11,
    RX_DESC             = 0x13,
    LLCP_TX             = 0x14,
    LLCP_RX             = 0x15,
    LLCP_ACK            = 0x16,
    L2CAP_TX            = 0x1A,
    L2CAP_RX            = 0x1B,
    L2CAP_ACK           = 0x1C,
    SCH_ARB_REQ         = 0x1D,
    SCH_ARB_CANC        = 0x1E,
    SCH_ARB_REM         = 0x1F,
    SCH_ARB_START       = 0x20,
    SCH_ARB_SHIFT       = 0x21,
    LC_ST               = 0x22,
    LLC_ST              = 0x23,
    HCI_CMD             = 0x24,
    HCI_CMD_STAT_EVT    = 0x25,
    HCI_CMD_CMP_EVT     = 0x26,
    HCI_EVT             = 0x27,
    HCI_LE_EVT          = 0x28,
    ACL_RX              = 0x35,
    ACL_PROG_TX         = 0x36,
    ACL_ACK_TX          = 0x37,
    CUSTOM              = 0x38,
}trace_type;

typedef enum
{
    CUSTOM_ID_32K_VALUE = 0x00,
    CUSTOM_ID_RF_CH_IDX = 0x01,
    CUSTOM_ID_CONN_TIMEOUT_CAUSE= 0x02,
    //More custom information about disconnection
    //Disconnection debug log format: link_id(8bit):LLC_PROC_TYPE(8bit):LLC_PROC_ID(8bit):error_log(8bit)
    CUSTOM_ID_DISC_DETAIL = 0x09,
    //Suplement information about Sync & crc error count for disconnection analyze
    //CUSTOM_ID_CON_SYNC_CRC_ERR_CNT debug log format: sync_err_count(16bit) : crc_err_count(16bit)
    CUSTOM_ID_CON_SYNC_CRC_ERR_CNT = 0x0A,
}dbg_trc_custom_id;

#define CUSTOM_DISC_ERROR_LINK_ID_OFFSET          (24)
#define CUSTOM_DISC_ERROR_LLC_PROC_TYPE_OFFSET    (16)
#define CUSTOM_DISC_ERROR_LLC_PROC_ID_OFFSET      (8)

typedef enum
{
    DBG_CONN_NO_ERROR               = 0x00,
    DBG_TARGET_CLOCK_TIMEOUT        = 0x01,
    DBG_SCH_ARB_FLAG_NO_ASAP        = 0x02,
    DBG_SCH_ARB_FLAG_ASAP_LIMIT     = 0x03,
    DBG_SCH_ARB_CONFLICT_STARTED    = 0x04,
    DBG_SCH_ARB_FLAG_ASAP_LIMIT_2   = 0x05,
}conn_timeout_reason_t;

typedef enum
{
    LL_CONNECTION_UPDATE_IND_OPCODE = 0x00,
    LL_CHANNEL_MAP_IND_OPCODE       = 0x01,
    LL_TERMINATE_IND_OPCODE         = 0x02,
    LL_ENC_REQ_OPCODE               = 0x03,
    LL_ENC_RSP_OPCODE               = 0x04,
    LL_START_ENC_REQ_OPCODE         = 0x05,
    LL_START_ENC_RSP_OPCODE         = 0x06,
    LL_UNKNOWN_RSP_OPCODE           = 0x07,
    LL_FEATURE_REQ_OPCODE           = 0x08,
    LL_FEATURE_RSP_OPCODE           = 0x09,
    LL_PAUSE_ENC_REQ_OPCODE         = 0x0A,
    LL_PAUSE_ENC_RSP_OPCODE         = 0x0B,
    LL_VERSION_IND_OPCODE           = 0x0C,
    LL_REJECT_IND_OPCODE            = 0x0D,
    LL_SLAVE_FEATURE_REQ_OPCODE     = 0x0E,
    LL_CONNECTION_PARAM_REQ_OPCODE  = 0x0F,
    LL_CONNECTION_PARAM_RSP_OPCODE  = 0x10,
    LL_REJECT_EXT_IND_OPCODE        = 0x11,
    LL_PING_REQ_OPCODE              = 0x12,
    LL_PING_RSP_OPCODE              = 0x13,
    LL_LENGTH_REQ_OPCODE            = 0x14,
    LL_LENGTH_RSP_OPCODE            = 0x15,
    LL_PHY_REQ_OPCODE               = 0x16,
    LL_PHY_RSP_OPCODE               = 0x17,
    LL_PHY_UPDATE_IND_OPCODE        = 0x18,
    LL_MIN_USED_CHANNELS_IND_OPCODE = 0x19,
    LL_CTE_REQ_OPCODE               = 0x1A,
    LL_CTE_RSP_OPCODE               = 0x1B,
    LL_PER_SYNC_IND_OPCODE          = 0x1C,
    LL_CLK_ACC_REQ_OPCODE           = 0x1D,
    LL_CLK_ACC_RSP_OPCODE           = 0x1E,

    /// Opcode length
    LL_OPCODE_MAX_OPCODE,
    LL_OPCODE_DEBUG = 0xFF,
}llcp_op_code;


struct le_chnl_map
{
    ///5-byte channel map array
    uint8_t map[LE_CHNL_MAP_LEN];
};
struct rand_nb
{
    ///8-byte array for random number
    uint8_t     nb[RAND_NB_LEN];
};

struct sess_k_div_x
{
    ///8-byte array for diversifier value
    uint8_t skdiv[SESS_KEY_DIV_LEN];
};
///Initiator vector
/*@TRACE*/
struct init_vect
{
    ///4-byte array for vector
    uint8_t iv[INIT_VECT_LEN];
};

///Supported LE Features structure
/*@TRACE*/
struct le_features
{
    ///8-byte array for LE features
    uint8_t feats[LE_FEATS_LEN];
};

/// LL_CONNECTION_UPDATE_IND structure.
/*@TRACE
 * @NO_PAD
*/
struct  ll_connection_update_ind
{
    /// op_code
    uint8_t         op_code;
    /// window size (units of 1.25 ms)
    uint8_t         win_size;
    /// window offset (units of 1.25 ms)
    uint16_t        win_off;
    /// interval (units of 1.25 ms)
    uint16_t        interv;
    /// connection latency (unit of connection event)
    uint16_t        latency;
    /// link supervision timeout (unit of 10 ms)
    uint16_t        timeout;
    /// instant (unit of connection event)
    uint16_t        instant;
};

/// LL_CHANNEL_MAP_IND structure.
/*@TRACE
 * @NO_PAD
*/
struct  ll_channel_map_ind
{
    /// op_code
    uint8_t            op_code;
    /// channel mapping
    struct le_chnl_map ch_map;
    /// instant
    uint16_t           instant;
};

/// LL_TERMINATE_IND structure.
/*@TRACE
 * @NO_PAD
*/
struct  ll_terminate_ind
{
    /// op_code
    uint8_t         op_code;
    /// termination code
    uint8_t         err_code;
};

/// LL_ENC_REQ structure.
/*@TRACE
 * @NO_PAD
*/
struct  ll_enc_req
{
    /// op_code
    uint8_t               op_code;
    /// random value
    struct rand_nb        rand;
    /// ediv
    uint16_t              ediv;
    /// skdm
    struct sess_k_div_x   skdm;
    /// ivm
    struct init_vect      ivm;
};

/// LL_ENC_RSP structure.
/*@TRACE
 * @NO_PAD
*/
struct  ll_enc_rsp
{
    /// op_code
    uint8_t             op_code;
    /// skds
    struct sess_k_div_x   skds;
    /// ivs
    struct init_vect    ivs;
};

/// LL_START_ENC_REQ structure.
/*@TRACE
 * @NO_PAD
*/
struct  ll_start_enc_req
{
    /// op_code
    uint8_t             op_code;
};

/// LL_START_ENC_RSP structure.
/*@TRACE
 * @NO_PAD
*/
struct  ll_start_enc_rsp
{
    /// op_code
    uint8_t             op_code;
};

/// LL_UNKNOWN_RSP structure.
/*@TRACE
 * @NO_PAD
*/
struct  ll_unknown_rsp
{
    /// op_code
    uint8_t         op_code;
    /// unknown type
    uint8_t         unk_type;
};

/// LL_FEATURE_REQ structure.
/*@TRACE
 * @NO_PAD
*/
struct  ll_feature_req
{
    /// op_code
    uint8_t             op_code;
    /// le features
    struct le_features  feats;
};


/// LL_FEATURE_RSP structure.
/*@TRACE
 * @NO_PAD
*/
struct  ll_feature_rsp
{
    /// op_code
    uint8_t             op_code;
    /// le features
    struct le_features  feats;
};

/// LL_PAUSE_ENC_REQ structure.
/*@TRACE
 * @NO_PAD
*/
struct  ll_pause_enc_req
{
    /// op_code
    uint8_t             op_code;
};

/// LL_PAUSE_ENC_RSP structure.
/*@TRACE
 * @NO_PAD
*/
struct  ll_pause_enc_rsp
{
    /// op_code
    uint8_t             op_code;
};

/// LL_VERSION_IND structure
/*@TRACE
 * @NO_PAD
*/
struct ll_version_ind
{
    /// op_code
    uint8_t     op_code;
    /// version
    uint8_t     vers;
    /// company id
    uint16_t    compid;
    /// sub version
    uint16_t    subvers;
};

/// LL_REJECT_IND structure.
/*@TRACE
 * @NO_PAD
*/
struct  ll_reject_ind
{
    /// op_code
    uint8_t         op_code;
    /// reject reason
    uint8_t         err_code;
};

/// LL_SLAVE_FEATURE_REQ structure.
/*@TRACE
 * @NO_PAD
*/
struct  ll_slave_feature_req
{
    /// op_code
    uint8_t             op_code;
    /// le features
    struct le_features  feats;
};

/// LL_CONNECTION_PARAM_REQ structure.
/*@TRACE
 * @NO_PAD
*/
struct  ll_connection_param_req
{
    /// op_code
    uint8_t         op_code;
    /// minimum value of connInterval (units of 1.25 ms)
    uint16_t        interval_min;
    /// maximum value of connInterval (units of 1.25 ms)
    uint16_t        interval_max;
    /// connSlaveLatency value (unit of connection event)
    uint16_t        latency;
    /// connSupervisionTimeout value (unit of 10 ms)
    uint16_t        timeout;
    /// preferred periodicity (units of 1.25 ms)
    uint8_t         pref_period;
    /// ReferenceConnEventCount (unit of connection event)
    uint16_t        ref_con_event_count;
    /// Offset0 (units of 1.25 ms)
    uint16_t        offset0;
    /// Offset1 (units of 1.25 ms)
    uint16_t        offset1;
    /// Offset2 (units of 1.25 ms)
    uint16_t        offset2;
    /// Offset3 (units of 1.25 ms)
    uint16_t        offset3;
    /// Offset4 (units of 1.25 ms)
    uint16_t        offset4;
    /// Offset5 (units of 1.25 ms)
    uint16_t        offset5;
};

/// LL_CONNECTION_PARAM_RSP structure.
/*@TRACE
 * @NO_PAD
*/
struct  ll_connection_param_rsp
{
    /// op_code
    uint8_t         op_code;
    /// minimum value of connInterval (units of 1.25 ms)
    uint16_t        interval_min;
    /// maximum value of connInterval (units of 1.25 ms)
    uint16_t        interval_max;
    /// connSlaveLatency value (unit of connection event)
    uint16_t        latency;
    /// connSupervisionTimeout value (unit of 10 ms)
    uint16_t        timeout;
    /// preferred periodicity (units of 1.25 ms)
    uint8_t         pref_period;
    /// ReferenceConnEventCount (unit of connection event)
    uint16_t        ref_con_event_count;
    /// Offset0 (units of 1.25 ms)
    uint16_t        offset0;
    /// Offset1 (units of 1.25 ms)
    uint16_t        offset1;
    /// Offset2 (units of 1.25 ms)
    uint16_t        offset2;
    /// Offset3 (units of 1.25 ms)
    uint16_t        offset3;
    /// Offset4 (units of 1.25 ms)
    uint16_t        offset4;
    /// Offset5 (units of 1.25 ms)
    uint16_t        offset5;
};

/// LL_REJECT_EXT_IND structure.
/*@TRACE
 * @NO_PAD
*/
struct  ll_reject_ext_ind
{
    /// op_code
    uint8_t         op_code;
    /// rejected op_code
    uint8_t         rej_op_code;
    /// reject reason
    uint8_t         err_code;
};

/// LL_PING_REQ structure.
/*@TRACE
 * @NO_PAD
*/
struct  ll_ping_req
{
    /// op_code
    uint8_t         op_code;
};

/// LL_PING_RSP structure.
/*@TRACE
 * @NO_PAD
*/
struct  ll_ping_rsp
{
    /// op_code
    uint8_t         op_code;
};

/// LL_LENGTH_REQ structure.
/*@TRACE
 * @NO_PAD
*/
struct  ll_length_req
{
    /// op_code
    uint8_t     op_code;
    /// The max size in reception (unit of byte)
    uint16_t    max_rx_octets;
    /// The max time in reception (unit of microsecond)
    uint16_t    max_rx_time;
    /// The max size in transmission (unit of byte)
    uint16_t    max_tx_octets;
    /// The max time in transmission (unit of microsecond)
    uint16_t    max_tx_time;
};

/// LL_LENGTH_RSP structure.
/*@TRACE
 * @NO_PAD
*/
struct  ll_length_rsp
{
    /// op_code
    uint8_t     op_code;
    /// The max size in reception (unit of byte)
    uint16_t    max_rx_octets;
    /// The max time in reception (unit of microsecond)
    uint16_t    max_rx_time;
    /// The max size in transmission (unit of byte)
    uint16_t    max_tx_octets;
    /// The max time in transmission (unit of microsecond)
    uint16_t    max_tx_time;
};
/// LL_PHY_REQ structure.
/*@TRACE
 * @NO_PAD
*/
struct  ll_phy_req
{
    /// op_code
    uint8_t    op_code;
    /// Tx phy selection
    uint8_t    tx_phys;
    /// Rx phy selection
    uint8_t    rx_phys;
};

/// LL_PHY_RSP structure.
/*@TRACE
 * @NO_PAD
*/
struct  ll_phy_rsp
{
    /// op_code
    uint8_t    op_code;
    /// Tx phy selection
    uint8_t    tx_phys;
    /// Rx phy selection
    uint8_t    rx_phys;
};

/// LL_PHY_UPDATE_IND structure.
/*@TRACE
 * @NO_PAD
*/
struct  ll_phy_update_ind
{
    /// op_code
    uint8_t    op_code;
    /// master to slave phy selected
    uint8_t    m_to_s_phy;
    /// slave to master phy selected
    uint8_t    s_to_m_phy;
    /// Instant
    uint16_t   instant;
};

/// LL_MIN_USED_CHANNELS_IND structure.
/*@TRACE
 * @NO_PAD
*/
struct  ll_min_used_channels_ind
{
    /// op_code
    uint8_t    op_code;
    /// PHY(s) for which the slave has a minimum number of used channels requirement
    uint8_t    phys;
    /// minimum number of channels to be used on the specified PHY
    uint8_t    min_used_ch;
};

/**
 * CTE length and type bit field definition
 *
 * |     5 bits     |  1b |   2 bits    |
 * |  MinCTELenReq  | RFU | CTETypeReq  |
 *
 *   - MinCTELenReq: minimum CTE length (x 8us)
 *   - CTETypeReq: 0-AoA, 1-AoD_1us, 2-AoD_2us, 3-RFU
 */
enum cte_len_type
{
    CTE_TYPE_REQ_MASK       = 0xC0,
    CTE_TYPE_REQ_LSB        = 6,

    MIN_CTE_LEN_REQ_MASK    = 0x1F,
    MIN_CTE_LEN_REQ_LSB     = 0,
};

/// LL_CTE_REQ structure.
/*@TRACE
 * @NO_PAD
*/
struct ll_cte_req
{
    /// op_code
    uint8_t    op_code;
    /// CTE length and type (@see enum cte_len_type)
    uint8_t    cte_len_type;
};

/// LL_PER_SYNC_IND structure.
/*@TRACE
 * @NO_PAD
*/
struct ll_per_sync_ind
{
    /// op_code
    uint8_t        op_code;
    /// ID (provided by the Host)
    uint16_t       id;
    /// Sync Info (ref definition or spec)
    uint8_t        sync_info[18];
    /// Connection event counter
    uint16_t       con_evt_cnt;
    /// Last periodic advertising event counter
    uint16_t       last_pa_evt_cnt;
    /// Advertising Set ID, advertiser address type and sleep clock accuracy
    uint8_t        sid_atype_sca;
    /// Periodic advertising PHY (@see enum le_phy_mask)
    uint8_t        phy;
    /// Periodic advertiser address
    //struct bd_addr adva;
    bdAddr_t  adva;
    /// Sync Connection event counter
    uint16_t       sync_con_evt_cnt;
};

/// LL_CLK_ACC_REQ structure.
/*@TRACE
 * @NO_PAD
*/
struct ll_clk_acc_req
{
    /// op_code
    uint8_t    op_code;
    /// SCA (@see enum SCA)
    uint8_t    sca;
};

/// LL_CLK_ACC_RSP structure.
/*@TRACE
 * @NO_PAD
*/
struct ll_clk_acc_rsp
{
    /// op_code
    uint8_t    op_code;
    /// SCA (@see enum SCA)
    uint8_t    sca;
};

/// LLCP pdu format
/*@TRACE
 @trc_ref co_llcp_op_code
 */
union llcp_pdu
{
    /// op_code
    uint8_t  op_code;

    //@trc_union op_code == LL_CONNECTION_UPDATE_IND_OPCODE
    struct ll_connection_update_ind     con_update_ind;

    //@trc_union op_code == LL_CHANNEL_MAP_IND_OPCODE
    struct ll_channel_map_ind           channel_map_ind;

    //@trc_union op_code == LL_TERMINATE_IND_OPCODE
    struct ll_terminate_ind             terminate_ind;

    //@trc_union op_code == LL_ENC_REQ_OPCODE
    struct ll_enc_req                   enc_req;

    //@trc_union op_code == LL_ENC_RSP_OPCODE
    struct ll_enc_rsp                   enc_rsp;

    //@trc_union op_code == LL_START_ENC_REQ_OPCODE
    struct ll_start_enc_req             start_enc_req;

    //@trc_union op_code == LL_START_ENC_RSP_OPCODE
    struct ll_start_enc_rsp             start_enc_rsp;

    //@trc_union op_code == LL_UNKNOWN_RSP_OPCODE
    struct ll_unknown_rsp               unknown_rsp;

    //@trc_union op_code == LL_FEATURE_REQ_OPCODE
    struct ll_feature_req               feats_req;

    //@trc_union op_code == LL_FEATURE_RSP_OPCODE
    struct ll_feature_rsp               feats_rsp;

    //@trc_union op_code == LL_PAUSE_ENC_REQ_OPCODE
    struct ll_pause_enc_req             pause_enc_req;

    //@trc_union op_code == LL_PAUSE_ENC_RSP_OPCODE
    struct ll_pause_enc_rsp             pause_enc_rsp;

    //@trc_union op_code == LL_VERSION_IND_OPCODE
    struct ll_version_ind               vers_ind;

    //@trc_union op_code == LL_REJECT_IND_OPCODE
    struct ll_reject_ind                reject_ind;

    //@trc_union op_code == LL_SLAVE_FEATURE_REQ_OPCODE
    struct ll_slave_feature_req         slave_feature_req;

    //@trc_union op_code == LL_CONNECTION_PARAM_REQ_OPCODE
    struct ll_connection_param_req      con_param_req;

    //@trc_union op_code == LL_CONNECTION_PARAM_RSP_OPCODE
    struct ll_connection_param_rsp      con_param_rsp;

    //@trc_union op_code == LL_REJECT_EXT_IND_OPCODE
    struct ll_reject_ext_ind            reject_ind_ext;

    //@trc_union op_code == LL_PING_REQ_OPCODE
    struct ll_ping_req                  ping_req;

    //@trc_union op_code == LL_PING_RSP_OPCODE
    struct ll_ping_rsp                  ping_rsp;

    //@trc_union op_code == LL_LENGTH_REQ_OPCODE
    struct ll_length_req                length_req;

    //@trc_union op_code == LL_LENGTH_RSP_OPCODE
    struct ll_length_rsp                length_rsp;

    //@trc_union op_code == LL_PHY_REQ_OPCODE
    struct ll_phy_req                   phy_req;

    //@trc_union op_code == LL_PHY_RSP_OPCODE
    struct ll_phy_rsp                   phy_rsp;

    //@trc_union op_code == LL_PHY_UPDATE_IND_OPCODE
    struct ll_phy_update_ind            phy_upd_ind;

    //@trc_union op_code == LL_MIN_USED_CHANNELS_IND_OPCODE
    struct ll_min_used_channels_ind     min_used_channels_ind;

    //@trc_union op_code == LL_CLK_ACC_REQ_OPCODE
    struct ll_clk_acc_req               clk_acc_req;

    //@trc_union op_code == LL_CLK_ACC_RSP_OPCODE
    struct ll_clk_acc_rsp               clk_acc_rsp;

    //@trc_union op_code == LL_PER_SYNC_IND_OPCODE
    struct ll_per_sync_ind              per_sync_ind;
};

enum custom_trace_types
{
    UINT8_T     =   0x01,
    UINT16_T    =   0x02,
    UINT32_T    =   0x03,
    VOID        =   0x04,
};

void hci_process_trace_data(uint8_t *dbg_data, uint32_t len);
void HciVscSetTraceBitMap(ble_trace_cfg bit_map);
#endif

#endif
