// -----------------------------------------------------------------------------
// Copyright (c) 2022 Think Silicon S.A.
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

#ifndef NEMA_RINGBUFFER_H__
#define NEMA_RINGBUFFER_H__

#include "nema_sys_defs.h"
#include "nema_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------
//@function nema_rb_submit_cmdlist
//@brief Enqueue a Command List to the Ring Buffer for execution
//@param nema_buffer_t *bo desc: Pointer to the buffer object of the Command List
//@param uint32_t size desc: Size of the populated Command List
//@return int desc: Return submission id
//--------------------------------------------------
/** \private */
int32_t nema_rb_submit_cmdlist(nema_buffer_t *bo, int size);

int32_t nema_rb_submit_cmdlist2(uintptr_t base_phys, int size);

void nema_rb_reset(void);

//--------------------------------------------------
//@function nema_rb_inline_cmd
//@brief Enqueue a Command to the Ring Buffer for execution
//@param uint32_t reg desc: Hardware Register to be written
//@param uint32_t data desc: Data to be written
//--------------------------------------------------
/** \private */
void nema_rb_inline_cmd(uint32_t reg, uint32_t data);

/** \private */
void nema_rb_force_flush(void);

/** \private */
void nema_rb_submit_cl_id(int cl_id);

#ifdef __cplusplus
}
#endif

#endif
