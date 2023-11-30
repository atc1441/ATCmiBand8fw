//*****************************************************************************
//
//! @file hci_dbg_trc.c
//!
//! @brief Abstract debug log from BTLE radio.
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
#include "hci_dbg_trc.h"
#include "bstream.h"
#include <string.h>
#include <stdio.h>
#include "wsf_buf.h"
#include "am_util_debug.h"
#include "hci_api.h"
#include "am_util.h"

#ifdef ENABLE_BLE_CTRL_TRACE
#if !TRC_CUSTOM_ONLY
char *trace_type_name[] =
{
    "KE_MSG_SEND",      // 0x00
    "KE_MSG_HANDLED",
    "KE_TMR_SET",
    "KE_TMR_CLR",       // 0x03,
    "KE_TMR_EXP",       // 0x04,
    "KE_EVT_SET",       // 0x05,
    "KE_EVT_HANDLED",   // 0x06,
    "MEM_ALLOC",        // 0x07,
    "MEM_FREE",         // 0x08,
    "SLEEP_ENTER",      // 0x09,
    "SLEEP_WAKEUP",     // 0x0A,
    "ASSERT_WARNING",   // 0x0B,
    "ASSERT_ERROR",     // 0x0C,
    "ET_PROG",          // 0x0D,
    "CONN_EVT_CNT",     // 0x0E,
    NULL,
    "FRM_CMP_BLE",      // 0x10,
    "CS_BLE",           // 0x11,
    NULL,
    "RX_DESC",          // 0x13,
    "LLCP_TX",          // 0x14,
    "LLCP_RX",          // 0x15,
    "LLCP_ACK",         // 0x16,
    NULL,
    NULL,
    NULL,
    "L2CAP_TX",         // 0x1A,
    "L2CAP_RX",         // 0x1B,
    "L2CAP_ACK",        // 0x1C,
    "SCH_ARB_REQ",      // 0x1D,
    "SCH_ARB_CANC",     // 0x1E,
    "SCH_ARB_REM",      // 0x1F,
    "SCH_ARB_START",    // 0x20,
    "SCH_ARB_SHIFT",    // 0x21,
    "LC_ST",            // 0x22,
    "LLC_ST",           // 0x23,
    "HCI_CMD",          // 0x24,
    "HCI_CMD_STAT_EVT", // 0x25,
    "HCI_CMD_CMP_EVT",  // 0x26,
    "HCI_EVT",          // 0x27,
    "HCI_LE_EVT",       // 0x28,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    "ACL_RX",           // 0x35,
    "ACL_PROG_TX",      // 0x36,
    "ACL_ACK_TX",
    "CUSTOM",
};

char *llcp_opcode_name[] =
{
    "LL_CONNECTION_UPDATE_IND", // 0x00,
    "LL_CHANNEL_MAP_IND",       // 0x01,
    "LL_TERMINATE_IND",         // 0x02,
    "LL_ENC_REQ",               // 0x03,
    "LL_ENC_RSP",               // 0x04,
    "LL_START_ENC_REQ",         // 0x05,
    "LL_START_ENC_RSP",         // 0x06,
    "LL_UNKNOWN_RSP",           // 0x07,
    "LL_FEATURE_REQ",           // 0x08,
    "LL_FEATURE_RSP",           // 0x09,
    "LL_PAUSE_ENC_REQ",         // 0x0A,
    "LL_PAUSE_ENC_RSP",         // 0x0B,
    "LL_VERSION_IND",           // 0x0C,
    "LL_REJECT_IND",            // 0x0D,
    "LL_SLAVE_FEATURE_REQ",     // 0x0E,
    "LL_CONNECTION_PARAM_REQ",  // 0x0F,
    "LL_CONNECTION_PARAM_RSP",  // 0x10,
    "LL_REJECT_EXT_IND",        // 0x11,
    "LL_PING_REQ",              // 0x12,
    "LL_PING_RSP",              // 0x13,
    "LL_LENGTH_REQ",            // 0x14,
    "LL_LENGTH_RSP",            // 0x15,
    "LL_PHY_REQ",               // 0x16,
    "LL_PHY_RSP",               // 0x17,
    "LL_PHY_UPDATE_IND",        // 0x18,
    "LL_MIN_USED_CHANNELS_IND", // 0x19,
    "LL_CTE_REQ",               // 0x1A,
    "LL_CTE_RSP",               // 0x1B,
    "LL_PER_SYNC_IND",          // 0x1C,
    "LL_CLK_ACC_REQ",           // 0x1D,
    "LL_CLK_ACC_RSP",           // 0x1E,
};
#endif

char *conn_timeout_str[] = {
    NULL,
    "schedule request rejected",
    "schedule element not found",
    "schedule request rejected due to bandwidth full",
    "schedule element should not be programmed ASAP",
    "schedule reach time limited",
    "schedlue elment conflict started",
    "schedule reach cheduling time limit during conflict case",
    "receive error packet exceed supervision timeout",
    "window size exceed half connection interval"
};

time_type_t TimeStamp2Time(uint32_t ms)
{
    time_type_t time_ret;

    time_ret.t_ms  = ms%1000;
    time_ret.t_sec = ms/1000%60;
    time_ret.t_min = ms/1000/60%60;
    time_ret.t_hur = ms/1000/60/60;

    return time_ret;
}

/*************************************************************************************************/
/*!
 *  \fn     Bytes2Str
 *
 *  \brief  Convert length bytes to a string.
 *
 *  \param  pBytes    Pointer to bytes array
 *  \len              length of the bytes
 *  \return Pointer to string.
 * \ notes: user needs to free the memory
 */
/*************************************************************************************************/
#if !TRC_CUSTOM_ONLY
char *Bytes2Str(const uint8_t *pBytes, uint16_t len)
{
  static const char hex[] = "0123456789ABCDEF";
  char  *str = NULL;
  char  *pStr = NULL;

  str = WsfBufAlloc(len*2);
  if(str == NULL)
  {
    return NULL;
  }

  pStr = str;

  while (pStr < &str[len*2])
  {
    *pStr++ = hex[*pBytes >> 4];
    *pStr++ = hex[*pBytes & 0x0F];
    ++pBytes;
  }

  /* null terminate string */
  *pStr = 0;
  return str;
}

void llcp_packet_process(uint8_t *llcp_pkt, uint8_t len)
{
    uint8_t *data_ptr = llcp_pkt;
    uint8_t llcp_opcode = *data_ptr++;

    switch(llcp_opcode)
    {
        case LL_CONNECTION_UPDATE_IND_OPCODE:
        {
            struct ll_connection_update_ind conn_update;

            BSTREAM_TO_UINT8(conn_update.win_size, data_ptr);
            BSTREAM_TO_UINT16(conn_update.win_off, data_ptr);
            BSTREAM_TO_UINT16(conn_update.interv, data_ptr);
            BSTREAM_TO_UINT16(conn_update.latency, data_ptr);
            BSTREAM_TO_UINT16(conn_update.timeout, data_ptr);
            BSTREAM_TO_UINT16(conn_update.instant, data_ptr);

            am_util_debug_printf("<%s>, param: WinSize:%d, offset:%d, int:%d, lat:%d, timout:%d, inst:%d\n",llcp_opcode_name[llcp_opcode]
            ,conn_update.win_size, conn_update.win_off, conn_update.interv,conn_update.latency,conn_update.timeout,conn_update.instant);
        }
        break;

        case LL_CHANNEL_MAP_IND_OPCODE:
        {
            struct ll_channel_map_ind ch_map_ind;
            char *map_buf = NULL;

            map_buf = Bytes2Str(data_ptr, LE_CHNL_MAP_LEN);
            if(map_buf == NULL)
            {
                am_util_debug_printf("buffer alloc fail!\n");
                break;
            }


            BSTREAM_TO_UINT16(ch_map_ind.instant, data_ptr);

            am_util_debug_printf("<%s>, param: ChlMap:%s, instant:%d\n",llcp_opcode_name[llcp_opcode]
                ,map_buf,ch_map_ind.instant);

            WsfBufFree(map_buf);
        }
        break;

        case LL_TERMINATE_IND_OPCODE:
        {
            struct ll_terminate_ind ter_ind;
            BSTREAM_TO_UINT8(ter_ind.err_code, data_ptr);

            am_util_debug_printf("<%s>, param: ErrCode:%d\n",llcp_opcode_name[llcp_opcode], ter_ind.err_code);
        }
        break;

        case LL_ENC_REQ_OPCODE:
        {
            struct ll_enc_req enc_req;
            uint64_t enc_rand;
            uint64_t skm = 0;
            uint64_t ivm = 0;
            //memcpy(enc_req.rand, data_ptr, RAND_NB_LEN);
            //data_ptr += RAND_NB_LEN;
            BSTREAM_TO_UINT64(enc_rand, data_ptr);
            BSTREAM_TO_UINT16(enc_req.ediv, data_ptr);
            //memcpy(enc_req.skdm, data_ptr, SESS_KEY_DIV_LEN);
            //data_ptr += SESS_KEY_DIV_LEN;
            BSTREAM_TO_UINT64(skm, data_ptr);
            //memcpy(enc_req.ivm, data_ptr, INIT_VECT_LEN);
            BSTREAM_TO_UINT32(ivm, data_ptr);
            am_util_debug_printf("<%s>, param: rand:%llx, ediv:%x, skm:%llX, ivm:%X\n",llcp_opcode_name[llcp_opcode],
                enc_rand, enc_req.ediv, skm, ivm);
        }
        break;

        case LL_ENC_RSP_OPCODE:
        {
            //ll_enc_rsp enc_rsp;
            uint64_t skds;
            uint32_t ivs;

            BSTREAM_TO_UINT64(skds, data_ptr);
            BSTREAM_TO_UINT32(ivs, data_ptr);

            am_util_debug_printf("<%s>, param: skds:%llX, ivs:%X\n",llcp_opcode_name[llcp_opcode],
                skds, ivs);
        }
        break;

        case LL_START_ENC_REQ_OPCODE:
        case LL_START_ENC_RSP_OPCODE:
        case LL_PAUSE_ENC_REQ_OPCODE:
        case LL_PAUSE_ENC_RSP_OPCODE:
        case LL_PING_REQ_OPCODE:
        case LL_PING_RSP_OPCODE:
        case LL_CTE_RSP_OPCODE:
        {
            am_util_debug_printf("<%s>\n", llcp_opcode_name[llcp_opcode]);
        }
        break;

        case LL_UNKNOWN_RSP_OPCODE:
        {
            struct ll_unknown_rsp unkn_rsp;

            BSTREAM_TO_UINT8(unkn_rsp.unk_type, data_ptr);

            am_util_debug_printf("<%s>, param: UnknownType:%x\n",llcp_opcode_name[llcp_opcode],
                unkn_rsp.unk_type);
        }
        break;

        case LL_FEATURE_REQ_OPCODE:
        case LL_FEATURE_RSP_OPCODE:
        case LL_SLAVE_FEATURE_REQ_OPCODE:
        {
            uint64_t feature = 0;

            BSTREAM_TO_UINT64(feature, data_ptr);
            am_util_debug_printf("<%s>, param: Feature:%llx\n",llcp_opcode_name[llcp_opcode],
                feature);
        }
        break;

        case LL_VERSION_IND_OPCODE:
        {
            struct ll_version_ind ver;

            BSTREAM_TO_UINT8(ver.vers, data_ptr);
            BSTREAM_TO_UINT16(ver.compid, data_ptr);
            BSTREAM_TO_UINT16(ver.subvers, data_ptr);

            am_util_debug_printf("<%s>, param: Vers:%d, CmpnyId:%x, SubVers:%x\n",llcp_opcode_name[llcp_opcode],
                ver.vers,ver.compid,ver.subvers);
        }
        break;

        case LL_REJECT_IND_OPCODE:
        {
            struct ll_reject_ind err_code;

            BSTREAM_TO_UINT8(err_code.err_code, data_ptr);

            am_util_debug_printf("<%s>, param: ErrCode:%x\n",llcp_opcode_name[llcp_opcode],
                err_code.err_code);
        }
        break;

        case LL_CONNECTION_PARAM_REQ_OPCODE:
        case LL_CONNECTION_PARAM_RSP_OPCODE:
        {
            struct ll_connection_param_req conn_para_req;

            BSTREAM_TO_UINT16(conn_para_req.interval_max, data_ptr);
            BSTREAM_TO_UINT16(conn_para_req.interval_min, data_ptr);
            BSTREAM_TO_UINT16(conn_para_req.latency, data_ptr);
            BSTREAM_TO_UINT16(conn_para_req.timeout, data_ptr);
            BSTREAM_TO_UINT8(conn_para_req.pref_period, data_ptr);
            BSTREAM_TO_UINT16(conn_para_req.ref_con_event_count, data_ptr);

            am_util_debug_printf("<%s>, param: InterMax:%d,InterMin:%d,Lat:%d,Timout:%d,PrePrid:%d, RefCon event counter:%d\n",llcp_opcode_name[llcp_opcode],
             conn_para_req.interval_max,conn_para_req.interval_min,conn_para_req.latency,conn_para_req.timeout,conn_para_req.pref_period,conn_para_req.ref_con_event_count);
        }
        break;

        case LL_REJECT_EXT_IND_OPCODE:
        {
            struct ll_reject_ext_ind rej_ext;

            BSTREAM_TO_UINT8(rej_ext.rej_op_code, data_ptr);
            BSTREAM_TO_UINT8(rej_ext.err_code, data_ptr);

            am_util_debug_printf("<%s>, param: RejOpCode:%x, ErrCode:0x%x\n",llcp_opcode_name[llcp_opcode],
                rej_ext.rej_op_code, rej_ext.err_code);
        }
        break;

        case LL_LENGTH_REQ_OPCODE:
        case LL_LENGTH_RSP_OPCODE:
        {
            struct ll_length_req len_req;

            BSTREAM_TO_UINT16(len_req.max_rx_octets, data_ptr);
            BSTREAM_TO_UINT16(len_req.max_rx_time, data_ptr);
            BSTREAM_TO_UINT16(len_req.max_tx_octets, data_ptr);
            BSTREAM_TO_UINT16(len_req.max_tx_time, data_ptr);

            am_util_debug_printf("<%s>, param: RxOct:%d,RxTime:%d,TxOct:%d,TxTime:%d\n",llcp_opcode_name[llcp_opcode],
                len_req.max_rx_octets, len_req.max_rx_time,len_req.max_tx_octets,len_req.max_tx_time);
        }
        break;

        case LL_PHY_REQ_OPCODE:
        case LL_PHY_RSP_OPCODE:
        {
            struct ll_phy_req phy_req;

            BSTREAM_TO_UINT8(phy_req.tx_phys, data_ptr);
            BSTREAM_TO_UINT8(phy_req.rx_phys, data_ptr);

            am_util_debug_printf("<%s>, param: TxPhy:%d,RxPhy:%d\n",llcp_opcode_name[llcp_opcode],
                phy_req.tx_phys, phy_req.rx_phys);
        }
        break;

        case LL_PHY_UPDATE_IND_OPCODE:
        {
            struct ll_phy_update_ind phy_update;

            BSTREAM_TO_UINT8(phy_update.m_to_s_phy, data_ptr);
            BSTREAM_TO_UINT8(phy_update.s_to_m_phy, data_ptr);
            BSTREAM_TO_UINT16(phy_update.instant, data_ptr);

            am_util_debug_printf("<%s>, param: Mstr2SlvPhy:%d,Slv2MstrPhy:%d,instant:%d\n",llcp_opcode_name[llcp_opcode],
                phy_update.m_to_s_phy, phy_update.s_to_m_phy,phy_update.instant);
        }
        break;

        case LL_MIN_USED_CHANNELS_IND_OPCODE:
        {
            struct ll_min_used_channels_ind min_used_ch;

            BSTREAM_TO_UINT8(min_used_ch.phys, data_ptr);
            BSTREAM_TO_UINT8(min_used_ch.min_used_ch, data_ptr);

            am_util_debug_printf("<%s>, param: phys:%d,minUsedChan:%d\n",llcp_opcode_name[llcp_opcode],
                min_used_ch.phys, min_used_ch.min_used_ch);
        }
        break;

        case LL_CTE_REQ_OPCODE:
        {
            struct ll_cte_req cte_req;

            BSTREAM_TO_UINT8(cte_req.cte_len_type, data_ptr);
            am_util_debug_printf("<%s>, param: cteLenType:%d\n",llcp_opcode_name[llcp_opcode],
                cte_req.cte_len_type);
        }
        break;

        case LL_PER_SYNC_IND_OPCODE:
        {
            struct ll_per_sync_ind per_sync;

            BSTREAM_TO_UINT16(per_sync.id, data_ptr);

            data_ptr += 18; // skip 18bytes sync info

            BSTREAM_TO_UINT16(per_sync.con_evt_cnt, data_ptr);
            BSTREAM_TO_UINT16(per_sync.last_pa_evt_cnt, data_ptr);
            BSTREAM_TO_UINT8(per_sync.sid_atype_sca, data_ptr);
            BSTREAM_TO_UINT8(per_sync.phy, data_ptr);
            BSTREAM_TO_BDA(per_sync.adva, data_ptr);
            BSTREAM_TO_UINT16(per_sync.sync_con_evt_cnt, data_ptr);

            am_util_debug_printf("<%s>, param: ConEvtCnt:%d,lastPaEvtCnt:%d, phy:%d, adva:%s, SyncConEvtCnt:%d\n",llcp_opcode_name[llcp_opcode],
            per_sync.con_evt_cnt,per_sync.last_pa_evt_cnt,per_sync.phy,Bda2Str(per_sync.adva),per_sync.sync_con_evt_cnt);
        }
        break;

        case LL_CLK_ACC_REQ_OPCODE:
        case LL_CLK_ACC_RSP_OPCODE:
        {
            struct ll_clk_acc_rsp clk_acc;
            BSTREAM_TO_UINT8(clk_acc.sca, data_ptr);
            am_util_debug_printf("<%s>, param: sca:%x\n",llcp_opcode_name[llcp_opcode], clk_acc.sca);
        }
        break;

        default:
            am_util_debug_printf("unknown LLCP opcode\n");
        break;
    }


}

void bytes_to_stream(uint8_t *dest_str, uint8_t * src, uint16_t len)
{
    uint16_t i = 0;

    for(i=0; i<len; i++)
      ;//sprintf((char *)(dest_str+(i*3)), "%02x ", src[i]);
}
#endif
/*************************************************************************************************/
/*!
 *  \brief  process trace data and output it to SWO.
 *
 *  \param  dbg_data    debug trace data from controller
 *  \param  len         length of trace data
 *
 *  \return None.
 */
/*************************************************************************************************/
void hci_process_trace_data(uint8_t *dbg_data, uint32_t len)
{
    uint8_t *data_buf = NULL;
    uint8_t packet_type = 0;
    uint32_t bitmap = 0;

    data_buf = dbg_data;
    packet_type = *data_buf;
    data_buf++;

    switch(packet_type)
    {
        // format: 1packet type, 2cofig word, 4bitmap
        case TRACE_ACK:
            data_buf += TRC_CFG_WORD_LEN; // skip 2 bytes config word length
            BYTES_TO_UINT32(bitmap, data_buf);
            am_util_debug_printf("Hci config trace ack, bitmap:0x%08x\n", bitmap);
        break;

        // format:1 packet typye, 2 length,2 sequence number,4 timestamp,
        // 1 trace type, 2 message id,2 dest id,2 opcode, 2 length
        case TRACE:
        {
            uint32_t ts = 0;
            uint8_t trace_type = 0;
            uint32_t ms = 0;
            uint32_t sec = 0;
            uint8_t min = 0;
            uint8_t hour = 0;
            uint16_t ms_dis = 0;
#if !TRC_CUSTOM_ONLY
            uint16_t sn = 0;
            uint16_t opcode = 0;
            uint16_t msg_id = 0;
            uint16_t trace_len = 0;
            char *trace_data = NULL;
#endif
            time_type_t cur_time;
            uint16_t us = 0;

            data_buf += 2; // skip 2bytes length
            data_buf += 2; // skip sequence number

            BYTES_TO_UINT32(ts, data_buf);  // uint in 312.5us(0.3125ms)
            data_buf += 4;
            //ms = (ts >>5 )*10;
            us = (ts*TRC_TIME_UNIT)/10%1000;
            ms = (ts*TRC_TIME_UNIT)/10/1000;
            cur_time = TimeStamp2Time(ms);
            ms_dis = cur_time.t_ms;
            sec    = cur_time.t_sec;
            min    = cur_time.t_min;
            hour   = cur_time.t_hur;

            trace_type = *data_buf;
            data_buf ++;

            switch(trace_type)
            {
#if !TRC_CUSTOM_ONLY
                // format: 2 message id, 2 dest id, 2 Opcode, 2 Length
                case KE_MSG_SEND:
                {
                    BYTES_TO_UINT16(msg_id, data_buf);
                    data_buf += 2;

                    data_buf += 2; // skip destination identifier

                    BYTES_TO_UINT16(opcode, data_buf);
                    data_buf += 2;

                    BYTES_TO_UINT16(trace_len, data_buf);

                    if(trace_len > 0)
                    {
                        trace_data = Bytes2Str(data_buf, trace_len);
                        if(trace_data == NULL)
                        {
                            am_util_debug_printf("buffer alloc fail!\n");
                            break;
                        }
                        am_util_debug_printf("trace data : %s, len :%d\n", trace_data, trace_len);
                    }

                    am_util_debug_printf("[%02d:%02d:%02d:%03d.%03d] SN:%d, msg id:0x%x, opCode:0x%x, trace len:%d, data:%s\n",
                                        hour, min, sec, ms_dis, us, sn, msg_id, opcode, trace_len, trace_data);
                    WsfBufFree(trace_data);
                }
                break;

                // format: 2 message id, 2 dest id, 2 opcode, 1 status
                case KE_MSG_HANDLED:
                {
                    BYTES_TO_UINT16(msg_id, data_buf);
                    data_buf += 2;

                    data_buf += 2; // skip destination identifier

                    BYTES_TO_UINT16(opcode, data_buf);
                    data_buf += 2;

                    uint8_t status = *data_buf;

                    am_util_debug_printf("[%02d:%02d:%02d:%03d.%03d] SN:%d, msg id:0x%x, opCode:0x%x, status:%d\n",
                                        hour, min, sec, ms_dis, us, sn, msg_id, opcode, status);
                }
                break;
                // 4 taget time, 2 task id, 2 msg id
                case KE_TMR_SET:
                case KE_TMR_CLR:
                case KE_TMR_EXP:
                {

                }
                break;

                // 1 event type
                case KE_EVT_SET:
                case KE_EVT_HANDLED:
                {

                }
                break;

                //1   heap id, 4 blk address, 4 size
                case MEM_ALLOC:
                case MEM_FREE:
                {

                }
                break;

                // 1 sleep type
                case SLEEP_ENTER:
                case SLEEP_WAKEUP:
                {
                    am_util_debug_printf("%s \n", trace_type_name[trace_type]);
                }
                break;

                // no use
                case ASSERT_WARNING:
                case ASSERT_ERROR:
                {

                }
                break;

                case ET_PROG:
                {
                    uint8_t elt_idx;
                    uint16_t extab;
                    uint16_t csptr;

                    BSTREAM_TO_UINT8(elt_idx, data_buf);
                    BSTREAM_TO_UINT16(extab, data_buf);
                    BSTREAM_TO_UINT16(csptr, data_buf);

                    am_util_debug_printf("[%02d:%02d:%02d:%03d.%03d] <%s>, eltIdx:%d,extab:0x%x,csptr:0x%x\n",
                        hour, min, sec, ms_dis, us, trace_type_name[trace_type], elt_idx, extab, csptr);

                }
                break;
                case CONN_EVT_CNT:
                {
                    uint16_t conhdl;
                    uint16_t value;

                    BSTREAM_TO_UINT16(conhdl, data_buf);
                    BSTREAM_TO_UINT16(value, data_buf);

                    am_util_debug_printf("[%02d:%02d:%02d:%03d.%03d] <%s>, conhdl:0x%x, con evt counter: %d\n",
                        hour, min, sec, ms_dis, us, trace_type_name[trace_type], conhdl, value);

                }
                break;

                case FRM_CMP_BLE:
                {
                    uint32_t timestamp;
                    uint8_t frm_id;
                    uint32_t frm_cb;
                    time_type_t cur_time;

                    BSTREAM_TO_UINT32(timestamp, data_buf);
                    cur_time = TimeStamp2Time((timestamp>>5)*10);

                    BSTREAM_TO_UINT8(frm_id, data_buf);
                    BSTREAM_TO_UINT32(frm_cb, data_buf);

                    am_util_debug_printf("[%02d:%02d:%02d:%03d.%03d] <%s>, time:[%02d:%02d:%02d:%03d],frm_id:%d,frm_cb:0x%x\n",
                        hour, min, sec, ms_dis, us, trace_type_name[trace_type], cur_time.t_hur, cur_time.t_min,cur_time.t_sec,cur_time.t_ms,frm_id, frm_cb);
                }

                break;

                // trace control structrue, 112 bytes
                case CS_BLE:
                break;

                case RX_DESC:
                {
                    //uint8_t drv_type = 0;
                    uint8_t rx_desc[BLE_RX_DESC_SIZE*2+1] ={0};

                    //drv_type = *data_buf;
                    data_buf++; // skip drive type

                    data_buf++; // skip act id

                    memcpy(rx_desc, data_buf, BLE_RX_DESC_SIZE);
                    bytes_to_stream(rx_desc, data_buf, BLE_RX_DESC_SIZE);

                    am_util_debug_printf("[%02d:%02d:%02d:%03d.%03d] <%s>, rx desc: sts 0x%04x rxlen: 0x%02x\n",
                                        hour, min, sec, ms_dis, us, trace_type_name[trace_type], *(uint16_t*)(rx_desc + 2), *(uint8_t*)(rx_desc + 5));
                }
                break;

                case LLCP_TX:
                case LLCP_RX:
                case LLCP_ACK:
                {
                    uint16_t conn_handle = 0;
                    uint8_t len = 0;

                    BSTREAM_TO_UINT16(conn_handle, data_buf);
                    BSTREAM_TO_UINT8(len, data_buf);

                    am_util_debug_printf("[%02d:%02d:%02d:%03d.%03d] <%s>, conHdl:%d, ",
                                        hour, min, sec, ms_dis, us, trace_type_name[trace_type], conn_handle);

                    llcp_packet_process(data_buf, len);
                }
                break;

                case L2CAP_TX:
                case L2CAP_RX:
                {
                    uint16_t conn_handle = 0;
                    uint16_t len = 0;
                    uint8_t *l2cap_pdu_data = NULL;

                    BSTREAM_TO_UINT16(conn_handle, data_buf);
                    BSTREAM_TO_UINT16(len, data_buf);

                    l2cap_pdu_data = WsfBufAlloc(len*3+1);
                    if(l2cap_pdu_data != NULL)
                    {
                        bytes_to_stream(l2cap_pdu_data, data_buf, len);
                    }

                    am_util_debug_printf("[%02d:%02d:%02d:%03d.%03d] <%s>, conn_handle:%d,data:%s, len:%d\n",
                                        hour, min, sec, ms_dis, us, trace_type_name[trace_type], conn_handle,l2cap_pdu_data, len);

                    WsfBufFree(l2cap_pdu_data);
                }
                break;

                case L2CAP_ACK:
                {
                    uint16_t conn_handle = 0;
                    uint16_t num_ack_pkts = 0;

                    BSTREAM_TO_UINT16(conn_handle, data_buf);
                    BSTREAM_TO_UINT16(num_ack_pkts, data_buf);

                    am_util_debug_printf("[%02d:%02d:%02d:%03d.%03d] <%s>, conn_handle:%d, num_ack_pkts:%d\n",
                                        hour, min, sec, ms_dis, us, trace_type_name[trace_type], conn_handle, num_ack_pkts);
                }
                break;

                case SCH_ARB_REQ:
                {
                    uint32_t target_time;
                    //uint32_t sch_arb_elt;
                    uint8_t status;
                    uint32_t dur_mini;
                    uint16_t asap_settings;
                    /// Current priority
                    uint8_t current_prio;
                    time_type_t cur_tart_time;

                    BSTREAM_TO_UINT32(target_time, data_buf);  // in half-slot
                    cur_tart_time = TimeStamp2Time((target_time>>5)*10);
                    //BSTREAM_TO_UINT32(sch_arb_elt, data_buf);
                    data_buf += 4; // skip arb_elt
                    BSTREAM_TO_UINT8(status, data_buf);

                    // Scheduling Arbiter Element
                    data_buf += 4; // skip 4 byte header
                    data_buf += 8; // skip 8 byte time
                    data_buf += 4; // skip 4 byte asap limit

                    BSTREAM_TO_UINT32(dur_mini, data_buf);  // in half-us
                    BSTREAM_TO_UINT16(asap_settings, data_buf); //
                    BSTREAM_TO_UINT16(current_prio, data_buf); //

                    am_util_debug_printf("[%02d:%02d:%02d:%03d.%03d] <%s>, targetTime[%02d:%02d:%02d:%03d], status:%d,durMini:%dus, asapSet:0x%x,CurPrio:%d\n",
                        hour, min, sec, ms_dis, us, trace_type_name[trace_type], cur_tart_time.t_hur, cur_tart_time.t_min,cur_tart_time.t_sec,cur_tart_time.t_ms, status,(dur_mini/2),asap_settings,current_prio);
                }
                break;

                case SCH_ARB_SHIFT:
                {
                    //uint32_t sch_arb_elt;
                    uint8_t current_prio;
                    uint32_t duration_min;
                    uint32_t time_hs;
                    time_type_t cur_time;

                    //BSTREAM_TO_UINT32(sch_arb_elt, data_buf);
                    data_buf += 4; // skip sch_arb_elt
                    BSTREAM_TO_UINT8(current_prio, data_buf);
                    BSTREAM_TO_UINT32(duration_min, data_buf);
                    BSTREAM_TO_UINT32(time_hs, data_buf);
                    cur_time = TimeStamp2Time((time_hs >>5 )*10);

                    am_util_debug_printf("[%02d:%02d:%02d:%03d.%03d] <%s>, curPrio:%d,durMini:%dus,time [%02d:%02d:%02d:%03d]\n",
                        hour, min, sec, ms_dis, us, trace_type_name[trace_type], current_prio,(duration_min/2),cur_time.t_hur, cur_time.t_min,cur_time.t_sec,cur_time.t_ms);
                }
                break;

                case SCH_ARB_CANC:
                case SCH_ARB_REM:
                case SCH_ARB_START:
                {
                    //uint32_t arb_elt = 0;

                    //BSTREAM_TO_UINT32(arb_elt, data_buf);

                    am_util_debug_printf("[%02d:%02d:%02d:%03d.%03d] <%s>\n", hour, min, sec, ms_dis, us, trace_type_name[trace_type]);
                }
                break;

                case LLC_ST:
                {
                    uint16_t conn_handle;
                    uint8_t proc_type;
                    uint8_t prev_state;
                    uint8_t curr_state;

                    BSTREAM_TO_UINT16(conn_handle, data_buf);
                    BSTREAM_TO_UINT8(proc_type, data_buf);
                    BSTREAM_TO_UINT8(prev_state, data_buf);
                    BSTREAM_TO_UINT8(curr_state, data_buf);

                    am_util_debug_printf("[%02d:%02d:%02d:%03d.%03d] <%s>, conn_handle:%d, proc_type:%d, prev_state:%d, curr_state:%d\n",
                    hour, min, sec, ms_dis, us, trace_type_name[trace_type], conn_handle,proc_type,prev_state,curr_state);
                }
                break;

                case HCI_CMD:
                {
                    uint16_t opcode;
                    uint16_t len;
                    uint8_t *cmd_payload;

                    BSTREAM_TO_UINT16(opcode, data_buf);
                    BSTREAM_TO_UINT16(len, data_buf);

                    if(len > 0)
                    {
                        cmd_payload = WsfBufAlloc(len*3+1);
                        if(cmd_payload != NULL)
                        {
                            bytes_to_stream(cmd_payload, data_buf, len);
                        }
                    }

                    am_util_debug_printf("[%02d:%02d:%02d:%03d.%03d] <%s>, opcode:0x%x,len:%d\n",
                                        hour, min, sec, ms_dis, us, trace_type_name[trace_type], opcode, len);

                    if(len >0)
                    {
                        am_util_debug_printf("cmd data:%s", cmd_payload);
                        WsfBufFree(cmd_payload);
                    }
                }
                break;

                case HCI_CMD_STAT_EVT:
                case HCI_CMD_CMP_EVT:
                case HCI_EVT:
                case HCI_LE_EVT:
                {
                    uint16_t opcode;
                    uint16_t len;
                    uint8_t *evt_payload;

                    BSTREAM_TO_UINT16(opcode, data_buf);
                    BSTREAM_TO_UINT16(len, data_buf);

                    if(len > 0)
                    {
                        evt_payload = WsfBufAlloc(len*3+1);
                        if(evt_payload != NULL)
                        {
                            bytes_to_stream(evt_payload, data_buf, len);
                        }
                    }

                    am_util_debug_printf("[%02d:%02d:%02d:%03d.%03d] <%s>, opcode:0x%x,len:%d\n",
                                        hour, min, sec, ms_dis, us, trace_type_name[trace_type], opcode, len);

                    if(len >0)
                    {
                        am_util_debug_printf("event data:%s", evt_payload);
                        WsfBufFree(evt_payload);
                    }
                }
                break;

                case ACL_RX:
                {
                    uint16_t len;
                    uint8_t *acl_payload;

                    BSTREAM_TO_UINT16(len, data_buf);

                    if(len > 0)
                    {
                        acl_payload = WsfBufAlloc(len*3+1);
                        if(acl_payload != NULL)
                        {
                            bytes_to_stream(acl_payload, data_buf, len);
                        }
                    }

                    am_util_debug_printf("[%02d:%02d:%02d:%03d.%03d] <%s>, opcode:0x%x,len:%d\n",
                                        hour, min, sec, ms_dis, us, trace_type_name[trace_type], opcode, len);

                    if(len >0)
                    {
                        am_util_debug_printf("acl Rx data:%s", acl_payload);
                        WsfBufFree(acl_payload);
                    }
                }
                break;

                case ACL_PROG_TX:
                case ACL_ACK_TX:
                {
                    uint16_t len;
                    uint8_t *acl_payload;

                    BSTREAM_TO_UINT16(len, data_buf);

                    if(len > 0)
                    {
                        acl_payload = WsfBufAlloc(len*3+1);
                        if(acl_payload != NULL)
                        {
                            bytes_to_stream(acl_payload, data_buf, len);
                        }
                    }

                    am_util_debug_printf("[%02d:%02d:%02d:%03d.%03d] <%s>, opcode:0x%x,len:%d\n",
                                        hour, min, sec, ms_dis, us, trace_type_name[trace_type], opcode, len);

                    if(len >0)
                    {
                        am_util_debug_printf("acl Tx data:%s", acl_payload);
                        WsfBufFree(acl_payload);
                    }
                }
#endif
                case CUSTOM:
                {
                    uint8_t  custom_id;
                    uint8_t  custom_data_type;
                    uint32_t custom_data = 0;
                    uint8_t link_id = 0, llc_proc_type = 0, llc_proc_id = 0, error_log = 0;
                    uint16_t sync_err_count = 0, crc_err_count = 0;

                    BSTREAM_TO_UINT8(custom_id, data_buf);
                    BSTREAM_TO_UINT8(custom_data_type, data_buf);

                    if (UINT8_T == custom_data_type) {
                        BSTREAM_TO_UINT8(custom_data, data_buf);
                    }
                    else if (UINT16_T == custom_data_type) {
                        BSTREAM_TO_UINT16(custom_data, data_buf);
                    }
                    else {
                        BSTREAM_TO_UINT32(custom_data, data_buf);
                    }

                    am_util_debug_printf("[%02d:%02d:%02d:%03d.%03d] Custom data: id 0x%x, 0x%08x\n",
                                        hour, min, sec, ms_dis, us, custom_id, custom_data);
                    if (custom_id == CUSTOM_ID_32K_VALUE)
                    {
                        am_util_debug_printf("32K clock= %d Hz\n", custom_data);
                    }
                    else if (custom_id == CUSTOM_ID_CONN_TIMEOUT_CAUSE)
                    {
                        link_id = custom_data & 0xFF;

                        if (((custom_data >> 8) & 0xFF) != 0)
                        {
                            am_util_debug_printf("Link %d connection timeout: %s\r\n", link_id, conn_timeout_str[((custom_data >>8) & 0xFF)]);
                        }
                    }
                    else if (custom_id == CUSTOM_ID_DISC_DETAIL)
                    {
                        //Disconnection debug log format: link_id(8bit):LLC_PROC_TYPE(8bit):LLC_PROC_ID(8bit):error_log(8bit)
                        link_id = (custom_data >> CUSTOM_DISC_ERROR_LINK_ID_OFFSET) & 0xFF;
                        llc_proc_type = (custom_data >> CUSTOM_DISC_ERROR_LLC_PROC_TYPE_OFFSET) & 0xFF;
                        llc_proc_id = (custom_data >> CUSTOM_DISC_ERROR_LLC_PROC_ID_OFFSET) & 0xFF;
                        error_log = custom_data & 0xFF;
                        am_util_debug_printf("Link %d LLC_PROC_TYPE: %d, LLC_PROC_ID: %d, error_log: %d\r\n", link_id, llc_proc_type, llc_proc_id, error_log);
                    }
                    else if (custom_id == CUSTOM_ID_CON_SYNC_CRC_ERR_CNT)
                    {
                        //CUSTOM_ID_CON_SYNC_CRC_ERR_CNT debug log format: sync_err_count(16bit) : crc_err_count(16bit)
                        sync_err_count = (custom_data >> 16) & 0xFFFF;
                        crc_err_count = custom_data& 0xFFFF;
                        am_util_debug_printf("Continous sync error %d times, continous crc error %d times\r\n", sync_err_count, crc_err_count);
                    }
                }

                break;

                default:
                break;
            }
        }
        break;

        default:
            am_util_debug_printf("unknow trace packet type\n");
            break;
    }
}
#endif

