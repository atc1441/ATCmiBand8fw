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
//------------------------------------------------------------------------------
#ifndef NEMADC_DSI_H__
#define NEMADC_DSI_H__
//------------------------------------------------------------------------------
#include "nema_sys_defs.h"
//------------------------------------------------------------------------------

//-----------------------------------------------------

#define NemaDC_dt_vsync_start                        (0x01U)
#define NemaDC_dt_vsync_end                          (0x11U)
#define NemaDC_dt_hsync_start                        (0x21U)
#define NemaDC_dt_hsync_end                          (0x31U)
#define NemaDC_dt_cmpr_mode                          (0x07U)
#define NemaDC_dt_end_of_trans                       (0x08U)
#define NemaDC_dt_pic_param                          (0x0aU)
#define NemaDC_dt_cmpr_pix_stream                    (0x0bU)
#define NemaDC_dt_color_mode_off                     (0x02U)
#define NemaDC_dt_color_mode_on                      (0x12U)
#define NemaDC_dt_shut_down_peripheral               (0x22U)
#define NemaDC_dt_turn_on_peripheral                 (0x32U)
#define NemaDC_dt_generic_short_write_param_no       (0x03U)
#define NemaDC_dt_generic_short_write_param_n1       (0x13U)
#define NemaDC_dt_generic_short_write_param_n2       (0x23U)
#define NemaDC_dt_generic_read_param_no              (0x04U)
#define NemaDC_dt_generic_read_param_n1              (0x14U)
#define NemaDC_dt_execute_queue                      (0x16U)
#define NemaDC_dt_generic_read_param_n2              (0x24U)
#define NemaDC_dt_DCS_short_write_param_no           (0x05U)
#define NemaDC_dt_DCS_short_write_param_n1           (0x15U)
#define NemaDC_dt_DCS_read_param_no                  (0x06U)
#define NemaDC_dt_set_max_return_packet_size         (0x37U)
#define NemaDC_dt_blanking_packet                    (0x19U)
#define NemaDC_dt_generic_long_write                 (0x29U)
#define NemaDC_dt_DCS_long_write                     (0x39U)
#define NemaDC_dt_packed_pixel_stream_rgb565         (0x0eU)
#define NemaDC_dt_packed_pixel_stream_rgb666         (0x1eU)
#define NemaDC_dt_loosely_packed_pixel_stream_rgb666 (0x2eU)
#define NemaDC_dt_loosely_packed_pixel_stream_rgb888 (0x3eU)


    //----------------------------------------------
#define NemaDC_dcs_datacmd       (  0U  )
#define NemaDC_ge_data           (1U<<30)
#define NemaDC_ge_cmd            (1U<<31)
#define NemaDC_ge_datacmd        ((1U<<30U)|(1U<<31U))
    //----------------------------------------------
//-----------------------------------------------------

//------------------------------------------------------------------------------

/** \brief Send scanline command and start memory write
*
*/
void nemadc_dsi_start_frame_transfer(void);

/** \brief Send scanline command and start memory write (generic)
*
*/
void nemadc_dsi_start_frame_transfer_generic(void);

/** \brief DC DBI interface to DSI
 *
 * \param  data_type  Data (pixel) type
 * \param  cmd_type   Command type
 * \param  type       DSI command type
 */
void nemadc_dsi_ct(uint32_t data_type, uint32_t cmd_type, uint32_t type);


#endif // NEMADC_DSI_H__
