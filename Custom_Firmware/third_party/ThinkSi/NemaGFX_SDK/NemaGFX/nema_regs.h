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

#ifndef NEMA_REGS_H__
#define NEMA_REGS_H__

#ifdef __cplusplus
extern "C" {
#endif

#define NEMA_HOLDCMD      (0xff000000U)

#ifndef NEMAP_HOLDCMD
#define NEMAP_HOLDCMD     (0xff000000U)
#endif

#define NEMAP_HOLDGPFLAG  ((uint32_t)1U << 27)

    // Texture Mapping and FB
    //-----------------------------
#define NEMA_TEX0_BASE        (0x000U)
#define NEMA_TEX0_BASE_H      (0x804U)
#define NEMA_TEX0_FSTRIDE     (0x004U)
#define NEMA_TEX0_RESXY       (0x008U)
    //-----------------------------
#define NEMA_TEX1_BASE        (0x010U)
#define NEMA_TEX1_BASE_H      (0x80cU)
#define NEMA_TEX1_FSTRIDE     (0x014U)
#define NEMA_TEX1_RESXY       (0x018U)
#define NEMA_TEX_COLOR        (0x01cU)
    //-----------------------------
#define NEMA_TEX2_BASE        (0x020U)
#define NEMA_TEX2_BASE_H      (0x814U)
#define NEMA_TEX2_FSTRIDE     (0x024U)
#define NEMA_TEX2_RESXY       (0x028U)
    //-----------------------------
#define NEMA_TEX3_BASE        (0x030U)
#define NEMA_TEX3_BASE_H      (0x81cU)
#define NEMA_TEX3_FSTRIDE     (0x034U)
#define NEMA_TEX3_RESXY       (0x038U)

    // Rasterizer
    //-----------------------------
#define NEMA_DRAW_CMD         (0x100U | NEMAP_HOLDCMD)
#define NEMA_DRAW_CMD_NOHOLD  (0x100U)
#define NEMA_DRAW_STARTXY     (0x104U)
#define NEMA_DRAW_ENDXY       (0x108U)
#define NEMA_CLIPMIN          (0x110U)
#define NEMA_CLIPMAX          (0x114U)
#define NEMA_MATMULT          (0x118U)
#define NEMA_CODEPTR          (0x11CU)
    //-----------------------------
#define NEMA_DRAW_PT0_X       (0x120U)
#define NEMA_DRAW_PT0_Y       (0x124U)
#define NEMA_DRAW_COLOR       (0x12cU)
     //-----------------------------
#define NEMA_DRAW_PT1_X       (0x130U)
#define NEMA_DRAW_PT1_Y       (0x134U)
     //-----------------------------
#define NEMA_DRAW_PT2_X       (0x140U)
#define NEMA_DRAW_PT2_Y       (0x144U)
     //-----------------------------
#define NEMA_DRAW_PT3_X       (0x150U)
#define NEMA_DRAW_PT3_Y       (0x154U)
    //-----------------------------
#define NEMA_RAST_BYPASS      (0x388U)
    //-----------------------------
//NemaP specific -->
    //-----------------------------
#define NEMA_BYPASS_ADDR      (0x138U)
#define NEMA_BYPASS_DATA      (0x13cU)
    //-----------------------------
#define NEMA_MM00             (0x160U)
#define NEMA_MM01             (0x164U)
#define NEMA_MM02             (0x168U)
#define NEMA_MM10             (0x16cU)
#define NEMA_MM11             (0x170U)
#define NEMA_MM12             (0x174U)
#define NEMA_MM20             (0x178U)
#define NEMA_MM21             (0x17cU)
#define NEMA_MM22             (0x180U)
    //-----------------------------
#define NEMA_DEPTH_START_L    (0x184U)
#define NEMA_DEPTH_START_H    (0x188U)
#define NEMA_DEPTH_DX_L       (0x18cU)
#define NEMA_DEPTH_DX_H       (0x190U)
#define NEMA_DEPTH_DY_L       (0x194U)
#define NEMA_DEPTH_DY_H       (0x198U)
    //-----------------------------
#define NEMA_RED_DX           (0x1a0U)
#define NEMA_RED_DY           (0x1a4U)
#define NEMA_GRE_DX           (0x1a8U)
#define NEMA_GRE_DY           (0x1acU)
#define NEMA_BLU_DX           (0x1b0U)
#define NEMA_BLU_DY           (0x1b4U)
#define NEMA_ALF_DX           (0x1b8U)
#define NEMA_ALF_DY           (0x1bcU)
#define NEMA_RED_INIT         (0x1c0U)
#define NEMA_GRE_INIT         (0x1c4U)
#define NEMA_BLU_INIT         (0x1c8U)
#define NEMA_ALF_INIT         (0x1ccU)
//<--
//NemaT specific -->
    //-----------------------------
#define NEMA_ZFUNC            (0x1e0U)
#define NEMA_DEPTH_MIN        (0x1e4U)
#define NEMA_DEPTH_MAX        (0x1e8U)
    //-----------------------------
//<--

    // Processor
    //-----------------------------
#define NEMA_ROPBLENDER_BLEND_MODE  (0x1d0U)
#define NEMA_ROPBLENDER_DST_CKEY    (0x1d4U)
#define NEMA_ROPBLENDER_CONST_COLOR (0x1d8U)
    //-----------------------------

    // Processor
    //-----------------------------
#define NEMA_IMEM_ADDR        (0x0c4U)
#define NEMA_IMEM_DATAH       (0x0c8U)
#define NEMA_IMEM_DATAL       (0x0ccU)
    //-----------------------------
#define NEMA_C0_REG           (0x200U)
#define NEMA_C1_REG           (0x204U)
#define NEMA_C2_REG           (0x208U)
#define NEMA_C3_REG           (0x20cU)
#define NEMA_CMAX_REG         (0x20cU)
#define NEMA_FRAG_CONADDR     (0x210U)
#define NEMA_FRAG_CONDATA     (0x214U)

    // Status & Control
    //-----------------------------
#define NEMA_CLID             (0x148U)
#define NEMA_LOADCTRL         (0x1f0U)
#define NEMA_CONFIG           (0x1f0U)
#define NEMA_CONFIGH          (0x1f4U)
#define NEMA_IDREG            (0x1ecU)
#define NEMA_CMDSTATUS        (0x0e8U)
#define NEMA_CMDRINGSTOP      (0x0ecU)
#define NEMA_CMDRINGSTOP_H    (0x844U)
#define NEMA_CMDADDR          (0x0f0U)
#define NEMA_CMDADDR_H        (0x84cU)
#define NEMA_CMDSIZE          (0x0f4U)
#define NEMA_INTERRUPT        (0x0f8U)
#define NEMA_IRQ_ID           (0xff0U)
#define NEMA_BURST_SIZE       (0x0d0U)

// bits [11:8]: GP_FLAGS_IRQ_MASK
// bits [ 7:4]: GP_FLAGS_MASK
// bits [ 3:0]: GP_FLAGS
#define NEMA_GP_FLAGS         (0xff4U)

#define NEMA_SYS_INTERRUPT    (0xff8U)
#define NEMA_STATUS           (0x0fcU)

#define NEMA_BREAKPOINT       (0x080U)
#define NEMA_BREAKPOINT_MASK  (0x08CU)
    //-----------------------------

    // Debug & Monitor
    //-----------------------------
#define NEMA_DBG_STATUS       (0x2f0U)
#define NEMA_DBG_ADDR         (0x2f4U)
#define NEMA_DBG_DATA         (0x2f8U)
#define NEMA_DBG_CTRL         (0x2fcU)
    //-----------------------------

    // Clockgating control
    //----------------------------
#define NEMA_CGCMD            (0x090U)
#define NEMA_CGCTRL           (0x094U)

    // Dirty Region
    //----------------------------
#define NEMA_DIRTYMIN         (0x098U)
#define NEMA_DIRTYMAX         (0x09cU)

#ifdef __cplusplus
}
#endif

#endif
