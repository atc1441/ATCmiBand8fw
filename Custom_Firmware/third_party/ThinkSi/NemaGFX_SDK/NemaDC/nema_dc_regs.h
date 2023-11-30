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
#ifndef NEMADC_REGS_H__
#define NEMADC_REGS_H__

#ifdef __cplusplus
extern "C" {
#endif

// Registers
//-----------------------------------------------------------------------------------------------------------------------
#define	NEMADC_REG_MODE                     (0x00U)
#define	NEMADC_REG_CLKCTRL                  (0x04U)
#define NEMADC_REG_PLAY                     (0x10U)
#define NEMADC_REG_CLKCTRL_CG              (0x1a8U)
#define	NEMADC_REG_BGCOLOR                  (0x08U)
#define	NEMADC_REG_RESXY                    (0x0cU)
#define	NEMADC_REG_FRONTPORCHXY             (0x14U)
#define	NEMADC_REG_BLANKINGXY               (0x18U)
#define	NEMADC_REG_BACKPORCHXY              (0x1cU)
#define	NEMADC_REG_CURSORXY                 (0x20U)
#define	NEMADC_REG_STARTXY                  (0x24U)
#define	NEMADC_REG_DBIB_CFG                 (0x28U)
#define	NEMADC_REG_GPIO                     (0x2cU)

#define	NEMADC_REG_LAYER0_MODE              (0x30U)
#define	NEMADC_REG_LAYER0_STARTXY           (0x34U)
#define	NEMADC_REG_LAYER0_SIZEXY            (0x38U)
#define	NEMADC_REG_LAYER0_BASEADDR          (0x3cU)
#define	NEMADC_REG_LAYER0_STRIDE            (0x40U)
#define	NEMADC_REG_LAYER0_RESXY             (0x44U)
#define	NEMADC_REG_LAYER0_SCALEX            (0x48U)
#define	NEMADC_REG_LAYER0_SCALEY            (0x4cU)
#define NEMADC_REG_LAYER0_COLORDEC_STARTXY (0x104U)

#define	NEMADC_REG_LAYER1_MODE              (0x50U)
#define	NEMADC_REG_LAYER1_STARTXY           (0x54U)
#define	NEMADC_REG_LAYER1_SIZEXY            (0x58U)
#define	NEMADC_REG_LAYER1_BASEADDR          (0x5cU)
#define	NEMADC_REG_LAYER1_STRIDE            (0x60U)
#define	NEMADC_REG_LAYER1_RESXY             (0x64U)
#define	NEMADC_REG_LAYER1_SCALEX            (0x68U)
#define	NEMADC_REG_LAYER1_SCALEY            (0x6cU)
#define NEMADC_REG_LAYER1_COLORDEC_STARTXY (0x108U)

#define	NEMADC_REG_LAYER2_MODE              (0x70U)
#define	NEMADC_REG_LAYER2_STARTXY           (0x74U)
#define	NEMADC_REG_LAYER2_SIZEXY            (0x78U)
#define	NEMADC_REG_LAYER2_BASEADDR          (0x7cU)
#define	NEMADC_REG_LAYER2_STRIDE            (0x80U)
#define	NEMADC_REG_LAYER2_RESXY             (0x84U)
#define	NEMADC_REG_LAYER2_SCALEX            (0x88U)
#define	NEMADC_REG_LAYER2_SCALEY            (0x8cU)
#define NEMADC_REG_LAYER2_COLORDEC_STARTXY (0x10cU)

#define	NEMADC_REG_LAYER3_MODE              (0x90U)
#define	NEMADC_REG_LAYER3_STARTXY           (0x94U)
#define	NEMADC_REG_LAYER3_SIZEXY            (0x98U)
#define	NEMADC_REG_LAYER3_BASEADDR          (0x9cU)
#define	NEMADC_REG_LAYER3_STRIDE            (0xa0U)
#define	NEMADC_REG_LAYER3_RESXY             (0xa4U)
#define	NEMADC_REG_LAYER3_SCALEX            (0xa8U)
#define	NEMADC_REG_LAYER3_SCALEY            (0xacU)
#define NEMADC_REG_LAYER3_COLORDEC_STARTXY (0x110U)

#define	NEMADC_REG_LAYER0_UBASE             (0xd0U)
#define	NEMADC_REG_LAYER0_VBASE             (0xd4U)
#define	NEMADC_REG_LAYER0_UVSTRIDE          (0xd8U)
#define	NEMADC_REG_LAYER1_UBASE             (0xdcU)
#define	NEMADC_REG_LAYER1_VBASE             (0xe0U)
#define	NEMADC_REG_LAYER1_UVSTRIDE          (0xe4U)
#define	NEMADC_REG_LAYER2_UBASE            (0x188U)
#define	NEMADC_REG_LAYER2_VBASE            (0x18cU)
#define	NEMADC_REG_LAYER2_UVSTRIDE         (0x190U)
#define	NEMADC_REG_LAYER3_UBASE            (0x194U)
#define	NEMADC_REG_LAYER3_VBASE            (0x198U)
#define	NEMADC_REG_LAYER3_UVSTRIDE         (0x19cU)

#define	NEMADC_REG_DBIB_CMD                 (0xe8U)
#define	NEMADC_REG_DBIB_RDAT                (0xecU)

#define	NEMADC_REG_CONFIG                   (0xf0U)
#define	NEMADC_REG_IDREG                    (0xf4U)
#define	NEMADC_REG_INTERRUPT                (0xf8U)
#define	NEMADC_REG_STATUS                   (0xfcU)
#define	NEMADC_REG_COLMOD                  (0x100U)
#define	NEMADC_REG_CRC                     (0x184U)
#define	NEMADC_REG_IP_VERSION              (0x180U)

#define NEMADC_REG_FORMAT_CTRL             (0x1a0U)
#define NEMADC_REG_FORMAT_CTRL2            (0x1a4U)
#define NEMADC_REG_FORMAT_CTRL3            (0x1acU)

#define	NEMADC_REG_PALETTE                 (0x400U)
#define	NEMADC_REG_CURSOR_IMAGE            (0x800U)
#define	NEMADC_REG_CURSOR_LUT              (0xA00U)

#define	NEMADC_REG_GAMMALUT_0             (0x1000U)
#define	NEMADC_REG_GAMMALUT_1             (0x1400U)
#define	NEMADC_REG_GAMMALUT_2             (0x1800U)
#define	NEMADC_REG_GAMMALUT_3             (0x1c00U)

// Layer parametric
#define NEMADC_REG_LAYER_MODE(i)             (0x030 + 0x20*(i))
#define NEMADC_REG_LAYER_STARTXY(i)          (0x034 + 0x20*(i))
#define NEMADC_REG_LAYER_SIZEXY(i)           (0x038 + 0x20*(i))
#define NEMADC_REG_LAYER_BASEADDR(i)         (0x03c + 0x20*(i))
#define NEMADC_REG_LAYER_STRIDE(i)           (0x040 + 0x20*(i))
#define NEMADC_REG_LAYER_RESXY(i)            (0x044 + 0x20*(i))
#define NEMADC_REG_LAYER_SCALEX(i)           (0x048 + 0x20*(i))
#define NEMADC_REG_LAYER_SCALEY(i)           (0x04c + 0x20*(i))
#define NEMADC_REG_GAMMALUT(i)               (0x1000+ 0x400*(i))
#define NEMADC_REG_LAYER_COLORDEC_STARTXY(i) (0x104 + 0x4*(i))



#ifdef __cplusplus
}
#endif

#endif
