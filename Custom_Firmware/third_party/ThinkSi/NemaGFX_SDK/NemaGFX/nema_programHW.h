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

#ifndef NEMA_PROGRAM_HW_H__
#define NEMA_PROGRAM_HW_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "nema_sys_defs.h"
#include "nema_matrix3x3.h"

/** \private */
typedef struct {
    uint32_t      base;     /**< Base address as seen by the GPU. */
    int32_t       w;        /**< Width. */
    int32_t       h;        /**< Height. */
    uint32_t      fstride;  /**< Format and Stride. */
    int32_t       valid;    /**< 1 if valid. */
} tex_t;

typedef struct nema_context_t_{
    unsigned char en_tscFB ;
    unsigned char en_ZCompr;
    unsigned char en_sw_depth;
    uint32_t surface_tile  ;
    uint32_t tri_cul       ;
    uint32_t color_grad    ;
    uint32_t draw_flags    ;
    uint32_t aa            ;
    uint32_t nema_error    ;
    int32_t  prev_clip_xy[2];
    uint32_t prev_clip_wh[2];
    int16_t  breakpoint    ;
    tex_t texs[8];
} nema_context_t;

extern TLS_VAR nema_context_t nema_context;

typedef union {
    float    f;
    uint32_t u;
    int32_t  i;
} nema_multi_union_t;

#define IS_NEMA_T      ( (nema_readHwConfig() & (0x80000U)) != 0U )

#define YX16TOREG32(y, x) ( (((unsigned)(y)) << 16) | (((unsigned)(x)) & 0xffffU))

// MatMult
//-----------------------------------------------------------------------------------------------------------------------
#define MMUL_QUAD_BEZ   (0x02000000U) //(1U<<25)
#define MMUL_BYPASS     (0x10000000U) //(1U<<28)
#define MMUL_DONTPLUS05 (0x20000000U) //(1U<<29)
#define MMUL_NONPERSP   (0x80000000U) //(1U<<31)

// Z-Function
//-----------------------------------------------------------------------------------------------------------------------
#define NEMA_Z_COMPARE_OP_NEVER         (0x0U)
#define NEMA_Z_COMPARE_OP_LESS          (0x1U)
#define NEMA_Z_COMPARE_OP_EQUAL         (0x2U)
#define NEMA_Z_COMPARE_OP_LESS_EQUAL    (0x3U)
#define NEMA_Z_COMPARE_OP_GREATER       (0x4U)
#define NEMA_Z_COMPARE_OP_NOT_EQUAL     (0x5U)
#define NEMA_Z_COMPARE_OP_GREATER_EQUAL (0x6U)
#define NEMA_Z_COMPARE_OP_ALWAYS        (0x7U)

// Z-Control
//-----------------------------------------------------------------------------------------------------------------------
#define Z_LATE       (0x8U )
#define Z_EARLY      (0x10U)
#define Z_BOTH       (0x18U)


#define BYPASS_COLOR         ( 1U)
#define BYPASS_CODEPTR_FG    ( 2U)
#define BYPASS_CODEPTR_BG    ( 3U)
#define BYPASS_CLIP_MinX     ( 4U)
#define BYPASS_CLIP_MinY     ( 5U)
#define BYPASS_CLIP_MaxX     ( 6U)
#define BYPASS_CLIP_MaxY     ( 7U)
#define BYPASS_PT0_X         ( 8U)
#define BYPASS_PT0_Y         ( 9U)
#define BYPASS_PT1_X         (10U)
#define BYPASS_PT1_Y         (11U)
#define BYPASS_DEPTH_START_L (12U)
#define BYPASS_DEPTH_START_H (13U)
#define BYPASS_DEPTH_DX_L    (14U)
#define BYPASS_DEPTH_DX_H    (15U)
#define BYPASS_DEPTH_DY_L    (16U)
#define BYPASS_DEPTH_DY_H    (17U)
#define BYPASS_E0S           (18U)
#define BYPASS_E0dx          (19U)
#define BYPASS_E0dy          (20U)
#define BYPASS_E1S           (21U)
#define BYPASS_E1dx          (22U)
#define BYPASS_E1dy          (23U)
#define BYPASS_E2S           (24U)
#define BYPASS_E2dx          (25U)
#define BYPASS_E2dy          (26U)
#define BYPASS_E3S           (27U)
#define BYPASS_E3dx          (28U)
#define BYPASS_E3dy          (29U)
#define BYPASS_NEG_AREA      (30U)
#define BYPASS_CMD           (31U)


#define NEMA_CONF_MASK_AXIM        (0x80000000U) //(0x1U  <<31)
#define NEMA_CONF_MASK_TEXFILTER   (0x40000000U) //(0x1U  <<30)
#define NEMA_CONF_MASK_TSC6        (0x20000000U) //(0x1U  <<29)
#define NEMA_CONF_MASK_ROP_BLENDER (0x10000000U) //(0x1U  <<28)
#define NEMA_CONF_MASK_ASYNC       (0x08000000U) //(0x1U  <<27)
#define NEMA_CONF_MASK_DIRTY       (0x04000000U) //(0x1U  <<26)
#define NEMA_CONF_MASK_TYPES       (0x03C00000U) //(0xfU  <<22)
#define NEMA_CONF_MASK_MMU         (0x00200000U) //(0x1U  <<21)
#define NEMA_CONF_MASK_ZCOMPR      (0x00100000U) //(0x1U  <<20)
#define NEMA_CONF_MASK_VRX         (0x00080000U) //(0x1U  <<19)
#define NEMA_CONF_MASK_ZBUF        (0x00040000U) //(0x1U  <<18)
#define NEMA_CONF_MASK_TSC         (0x00020000U) //(0x1U  <<17)
#define NEMA_CONF_MASK_CG          (0x00010000U) //(0x1U  <<16)
#define NEMA_CONF_MASK_CORES       (0x00000F00U) //(0xfU  <<8 )
#define NEMA_CONF_MASK_THREADS     (0x000000FFU) //(0xffU <<0)

#define NEMA_CONF_MASK_AA          (0x00000001U) //(0x1U <<0)
#define NEMA_CONF_MASK_DEC         (0x00000002U) //(0x1U <<1)
#define NEMA_CONF_MASK_10BIT       (0x00000004U) //(0x1U <<2)
#define NEMA_CONF_MASK_RADIAL      (0x00000100U) //(0x1U <<8)
#define NEMA_CONF_MASK_VG          (0x00000200U) //(0x1U <<9)

#define NEMA_RASTER_LINE_SIZE         (3U)
#define NEMA_RASTER_TRIANGLE_FX_SIZE  (7U)
#define NEMA_RASTER_QUAD_FX_SIZE      (9U)
#define NEMA_RASTER_RECT_SIZE         (3U)
#define NEMA_RASTER_PIXEL_SIZE        (3U)

// -----------------------------------------------------------------------------
// ------------------------------- FUNCTIONS -----------------------------------
// -----------------------------------------------------------------------------
/** \private */
uint32_t nema_readHwConfig(void);

/** \private */
uint32_t nema_readHwConfigH(void);

// -------------------------------- LOADCTRL -----------------------------------
/** \private */
void nema_setLoadCtrlReg(uint32_t val);

// ------------------------------- MATMULT -------------------------------------
/** \private */
void nema_matmul_bypass(int enable);

/** \brief Load GPU's Matrix Multiplier with a given 3x3 matrix
 *
 * \param m Matrix to be loaded
 *
 */
void nema_set_matrix(nema_matrix3x3_t m);

/** \brief Load GPU's Matrix Multiplier for scaling
 *
 * \param dst_x X coordinate of upper-left vertex of the destination
 * \param dst_y Y coordinate of upper-left vertex of the destination
 * \param dst_xres Width of destination rectangular area
 * \param dst_yres Height of destination rectangular area
 * \param src_x X coordinate of upper-left vertex of the source
 * \param src_y Y coordinate of upper-left vertex of the source
 * \param src_xres Width of source rectangular area
 * \param src_yres Height of source rectangular area
 *
 */
void nema_set_matrix_scale( float dst_x, float dst_y, float dst_xres, float dst_yres,
                            float src_x, float src_y, float src_xres, float src_yres);

/** \brief Load GPU's Matrix Multiplier for a simple Blit (affine translation)
 *
 * \param dst_x X coordinate of upper-left vertex of the destination
 * \param dst_y Y coordinate of upper-left vertex of the destination
 *
 */
void nema_set_matrix_translate(float dst_x, float dst_y);

// ------------------------------- BLENDER ------------------------------------

/** \brief Load a precompiled Shader to the GPU's internal memory
 *
 * \param cmd Pointer to the shader
 * \param count Number of commands
 * \param codeptr Internal Memory address to be written (default is 0)
 *
 */
void nema_load_frag_shader(const uint32_t *cmd, uint32_t count, uint32_t codeptr);

/** \brief Set the Internal Memory address of the fragment shader to be executed
 *
 * \param ptr Internal Memory address of the fragment shader
 *
 */
void nema_set_frag_ptr(uint32_t ptr);


/** \brief Load a precompiled Shader to the GPU's internal memory and set fragment pointer
 *
 * \param cmd Pointer to the shader
 * \param count Number of commands
 * \param codeptr Internal Memory address to be written (default is 0)
 * \param ptr Internal Memory address of the fragment shader
 *
 */
void nema_load_frag_shader_ptr(const uint32_t *cmd, uint32_t count, uint32_t codeptr, uint32_t ptr);

// ------------------------------- ROP BLENDER ------------------------------------
/** \brief Set ROP blending mode
 *
 * \param bl_mode Blending mode
 *
 */
void nema_set_rop_blend_mode(uint32_t bl_mode);

/** \brief Set ROP destination color key
 *
 * \param rgba Destination Color Key
 * \see nema_rgba()
 *
 */
void nema_set_rop_dst_color_key(uint32_t rgba);

/** \brief Set ROP constant color
 *
 * \param rgba Constant color
 * \see nema_rgba()
 *
 */
void nema_set_rop_const_color(uint32_t rgba);

// ------------------------------- Z-BUFFER ------------------------------------
/** \private */
void nema_set_depth_ctrl(uint32_t val);

// ------------------------------ INTERRUPT ------------------------------------
/** \private */
void nema_set_interrupt_ctrl(uint32_t val);
/** \private */
void nema_set_interrupt_ctrl_imm(uint32_t val);

// -------------------------------- UTILS --------------------------------------
/** \private */
void nema_set_clip_temp(int32_t x,
              int32_t y,
              uint32_t w,
              uint32_t h);

/** \private */
void nema_set_clip_pop(void);

void nema_enable_tiling(uint32_t enable);

/** \brief Set maximum and minimum values for depth buffer. Available ony for Nema|T
*
* \param min_depth Minimum value
* \param max_depth Maximum value
*
*/
void nema_set_depth_range(float min_depth, float max_depth);

//void nema_force_draw_cmd(uint32_t val);
void nema_set_matmul_ctrl(uint32_t val);

void nema_set_matrix_all(nema_matrix3x3_t m);

void nema_set_bypass_reg( uint32_t addr, uint32_t data );

// -------------------------- RASTERIZER (GRAD) --------------------------------
void nema_set_gradient_fx(int32_t r_init, int32_t g_init, int32_t b_init,
                          int32_t a_init, int32_t r_dx,   int32_t r_dy,
                          int32_t g_dx,   int32_t g_dy,   int32_t b_dx,
                          int32_t b_dy,   int32_t a_dx,   int32_t a_dy);

// -------------------------- RASTERIZER (DEPTH) -------------------------------
void nema_set_depth_imm(uint32_t startl, uint32_t starth,
                        uint32_t dxl   , uint32_t dxh,
                        uint32_t dyl   , uint32_t dyh);



/** \brief Convert a float value to Internal representation of half float and write it to a Constant Register
 *
 * \param reg Constant Register to be written
 * \param v Register's Index to  be written
 * \param value Value to be written
 *
 */
void nema_set_const_reg_half(uint32_t reg, uint32_t v, float value);

/** \brief Convert a float value to Internal representation of float and write it to a Constant Register
 *
 * \param reg Constant Register to be written
 * \param v Register's Index to  be written
 * \param value Value to be written
 *
 */
void nema_set_const_reg_single(uint32_t reg, uint32_t v, float value);


#ifdef __cplusplus
}
#endif

#endif
