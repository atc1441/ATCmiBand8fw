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

#ifndef NEMA_SHADERSPECIFIC_H__
#define NEMA_SHADERSPECIFIC_H__

#include "nema_sys_defs.h"
#include "nema_hal.h"
#include "nema_programHW.h"
#include "nema_graphics.h"

#ifdef __cplusplus
extern "C" {
#endif

void nema_blit_yuv( uintptr_t src_addr_Y,
                    uintptr_t src_addr_U,
                    uintptr_t src_addr_V,
                    uint32_t src_xres,
                    uint32_t src_yres,
                    int src_stride,
                    int dst_x,
                    int dst_y);
void nema_blit_yuv10( uintptr_t src_addr_Y,
                    uintptr_t src_addr_U,
                    uintptr_t src_addr_V,
                    uint32_t src_xres,
                    uint32_t src_yres,
                    int src_stride,
                    int dst_x,
                    int dst_y);
void nema_blit_warp(uintptr_t warpBase, uint32_t warpW, uint32_t warpH, nema_tex_format_t warpMode,
                    int warpStride, int x, int y);
void nema_blit_blur(unsigned char matrix[3][3], int x, int y, int w, int h);
void nema_blit_edge(unsigned char matrix[3][3], int x, int y, int w, int h);

void nema_blit_mean(int x, int y, int w, int h);
void nema_blit_gauss(int x, int y, int w, int h);
void nema_blit_sharpen_gauss(int x, int y, int w, int h, float sharpen);
void nema_blit_sharpen_laplace(int x, int y, int w, int h, float sharpen);
void nema_blit_contrast_linear(int x, int y, int w, int h, uint8_t min, uint8_t max);
void nema_blit_color_correction(uint8_t matrix[3][3], int x, int y, int w, int h);
void nema_blit_rgb_to_ycbcr(int w, int h);
void nema_blit_median(int w, int h);
void nema_blit_hist_equalization(int w, int h, uint32_t histogram[256]);
void nema_blit_gamma(int w, int h,nema_buffer_t *bo);
void nema_blit_binary(int x, int y, int w, int h, float threshold);
void nema_blit_debayer(int w, int h);
void nema_blit_ycbcr_to_rgb(int w, int h);

void nema_blit_bayer_L8_to_RGB(int w, int h);
void nema_blit_3L8_to_RGB(int w, int h);
void nema_blit_RGB_to_3L8(int w, int h);

#ifdef __cplusplus
}
#endif

#endif
