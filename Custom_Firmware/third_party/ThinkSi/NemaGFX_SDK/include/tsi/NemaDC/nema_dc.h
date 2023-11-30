/*******************************************************************************
 * Copyright (c) 2022 Think Silicon S.A.
 *
   Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this header file and/or associated documentation files to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies of the
 * Materials, and to permit persons to whom the Materials are furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Materials.
 *
 * MODIFICATIONS TO THIS FILE MAY MEAN IT NO LONGER ACCURATELY REFLECTS
 * NEMAGFX API. THE UNMODIFIED, NORMATIVE VERSIONS OF THINK-SILICON NEMAGFX
 * SPECIFICATIONS AND HEADER INFORMATION ARE LOCATED AT:
 *   https://think-silicon.com/products/software/nemagfx-api
 *
 *  The software is provided 'as is', without warranty of any kind, express or
 *  implied, including but not limited to the warranties of merchantability,
 *  fitness for a particular purpose and noninfringement. In no event shall
 *  Think Silicon S.A. be liable for any claim, damages or other liability, whether
 *  in an action of contract, tort or otherwise, arising from, out of or in
 *  connection with the software or the use or other dealings in the software.
 ******************************************************************************/


#ifndef NEMA_DC_H__
#define NEMA_DC_H__

#include "nema_sys_defs.h"
#include "nema_dc_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// ThinkLCD CNFG register bits
#define NEMADC_CFG_LAYER_EXISTS(i)    (1U << (8 + (i)*4))
#define NEMADC_CFG_LAYER_BLENDER(i)   (1U << (8 + (i)*4 + 1))
#define NEMADC_CFG_LAYER_SCALER(i)    (1U << (8 + (i)*4 + 2))
#define NEMADC_CFG_LAYER_GAMMA(i)     (1U << (8 + (i)*4 + 3))
#define NEMADC_COLMOD_LAYER_FLIPX(i)  (1U << (23 + i))
#define NEMADC_COLMOD_FLIPY           (1U << 27)

// Layer Control
//--------------------------------------------------------------------------

#define NEMADC_LAYER_DISABLE     (   0U   )   /**< Disable Layer */
#define NEMADC_LAYER_ENABLE      (1U << 31)   /**< Enable Layer */
#define NEMADC_FORCE_A           (1U << 30)   /**< Force Alpha */
#define NEMADC_SCALE_NN          (1U << 29)   /**< Activate Bilinear Filter */
#define NEMADC_MODULATE_A        (1U << 28)   /**< Modulate Alpha */
#define NEMADC_LAYER_AHBLOCK     (1U << 27)   /**< Activate HLOCK signal on AHB DMAs */
#define NEMADC_LAYER_GAMMALUT_EN (1U << 26)   /**< Enable Gamma Look Up Table */

#define NEMADC_UNDERFLOW_MECH    (1U << 29)   /**< Underflow mechanism, when set to 1 it disables this mechanism */

//--------------------------------------------------------------------------

#define NEMADC_BF_ZERO           (0x0U) /**< Black */
#define NEMADC_BF_ONE            (0x1U) /**< White */
#define NEMADC_BF_SRCALPHA       (0x2U) /**< Alpha Source */
#define NEMADC_BF_GLBALPHA       (0x3U) /**< Alpha Global */
#define NEMADC_BF_SRCGBLALPHA    (0x4U) /**< Alpha Source And Alpha Global */
#define NEMADC_BF_INVSRCALPHA    (0x5U) /**< Inverted Source */
#define NEMADC_BF_INVGBLALPHA    (0x6U) /**< Inverted Global */
#define NEMADC_BF_INVSRCGBLALPHA (0x7U) /**< Inverted Source And Global */
#define NEMADC_BF_DSTALPHA       (0xaU) /**< Alpha Destination */
#define NEMADC_BF_INVDSTALPHA    (0xbU) /**< Inverted Destination */


                                /*  source factor            destination factor */
#define NEMADC_BL_SIMPLE         (NEMADC_BF_SRCALPHA     | (NEMADC_BF_INVSRCALPHA  <<4))   /**< Sa * Sa + Da * (1 - Sa) */
#define NEMADC_BL_CLEAR          (NEMADC_BF_ZERO         | (NEMADC_BF_ZERO         <<4))   /**< 0 */
#define NEMADC_BL_SRC            (NEMADC_BF_ONE          | (NEMADC_BF_ZERO         <<4))   /**< Sa */
#define NEMADC_BL_SRC_OVER       (NEMADC_BF_ONE          | (NEMADC_BF_INVSRCALPHA  <<4))   /**< Sa + Da * (1 - Sa) */
#define NEMADC_BL_DST_OVER       (NEMADC_BF_INVDSTALPHA  | (NEMADC_BF_ONE          <<4))   /**< Sa * (1 - Da) + Da */
#define NEMADC_BL_SRC_IN         (NEMADC_BF_DSTALPHA     | (NEMADC_BF_ZERO         <<4))   /**< Sa * Da */
#define NEMADC_BL_DST_IN         (NEMADC_BF_ZERO         | (NEMADC_BF_SRCALPHA     <<4))   /**< Da * Sa */
#define NEMADC_BL_SRC_OUT        (NEMADC_BF_INVDSTALPHA  | (NEMADC_BF_ZERO         <<4))   /**< Sa * (1 - Da) */
#define NEMADC_BL_DST_OUT        (NEMADC_BF_ZERO         | (NEMADC_BF_INVSRCALPHA  <<4))   /**< Da * (1 - Sa) */
#define NEMADC_BL_SRC_ATOP       (NEMADC_BF_DSTALPHA     | (NEMADC_BF_INVSRCALPHA  <<4))   /**< Sa * Da + Da * (1 - Sa) */
#define NEMADC_BL_DST_ATOP       (NEMADC_BF_INVDSTALPHA  | (NEMADC_BF_SRCALPHA     <<4))   /**< Sa * (1 - Da) + Da * Sa */
#define NEMADC_BL_ADD            (NEMADC_BF_ONE          | (NEMADC_BF_ONE          <<4))   /**< Sa + Da */
#define NEMADC_BL_XOR            (NEMADC_BF_INVDSTALPHA  | (NEMADC_BF_INVSRCALPHA  <<4))   /**< Sa * (1 - Da) + Da * (1 - Sa) */


//--------------------------------------------------------------------------
#define NEMADC_LUT8      (0x00U)  /**< LUT8 */
#define NEMADC_RGBA8888  (0x0dU)  /**< RGBA8888 */
#define NEMADC_BGRA8888  (0x06U)  /**< BGRA8888 */
#define NEMADC_ARGB8888  (0x0eU)  /**< ARGB8888 */
#define NEMADC_ABGR8888  (0x02U)  /**< ABGR8888 */
#define NEMADC_RGB24     (0x0bU)  /**< RGB24 */
#define NEMADC_BGR24     (0x03U)  /**< BGR24 */         //(Available if HW enabled - check HW manual) */
#define NEMADC_RGBA4444  (0x15U)  /**< RGBA4444 */
#define NEMADC_BGRA4444  (0x0aU)  /**< BGRA4444 */      //(Available if HW enabled - check HW manual) */
#define NEMADC_ARGB4444  (0x18U)  /**< ARGB4444 */
#define NEMADC_ABGR4444  (0x0cU)  /**< ABGR4444 */      //(Available if HW enabled - check HW manual) */
#define NEMADC_RGBA5551  (0x01U)  /**< RGBA5551 */
#define NEMADC_BGRA5551  (0x0fU)  /**< BGRA5551 */      //(Available if HW enabled - check HW manual) */
#define NEMADC_ARGB1555  (0x11U)  /**< ARGB1555 */      //(Available if HW enabled - check HW manual) */
#define NEMADC_ABGR1555  (0x1fU)  /**< ABGR1555 */      //(Available if HW enabled - check HW manual) */
#define NEMADC_RGB565    (0x05U)  /**< RGB565 */
#define NEMADC_BGR565    (0x16U)  /**< BGR565 */        //(Available if HW enabled - check HW manual) */
#define NEMADC_AL88      (0x17U)  /**< AL88 */          //(Available if HW enabled - check HW manual) */
#define NEMADC_AL44      (0x19U)  /**< AL44 */          //(Available if HW enabled - check HW manual) */
#define NEMADC_RGB332    (0x04U)  /**< RGB332 */
#define NEMADC_RGBA2222  (0x1aU)  /**< RGBA2222 */      //(Available if HW enabled - check HW manual) */
#define NEMADC_BGRA2222  (0x1bU)  /**< BGRA2222 */      //(Available if HW enabled - check HW manual) */
#define NEMADC_ARGB2222  (0x1cU)  /**< ARGB2222 */      //(Available if HW enabled - check HW manual) */
#define NEMADC_ABGR2222  (0x1dU)  /**< ABGR2222 */      //(Available if HW enabled - check HW manual) */
#define NEMADC_A8        (0x1eU)  /**< A8 */            //(Available if HW enabled - check HW manual) */
#define NEMADC_L8        (0x07U)  /**< L8 */
#define NEMADC_L1        (0x08U)  /**< L1 */            //(Available if HW enabled - check HW manual) */
#define NEMADC_L4        (0x09U)  /**< L4 */            //(Available if HW enabled - check HW manual) */
#define NEMADC_YUYV      (0x0aU)  /**< YUYV */          //(Available if HW enabled - check HW manual) */
#define NEMADC_YUY2      (0x0cU)  /**< YUY2 */          //(Available if HW enabled - check HW manual) */
#define NEMADC_V_YUV420  (0x10U)  /**< V_YUV420 */      //(Available if HW enabled - check HW manual) */
#define NEMADC_TLYUV420  (0x11U)  /**< TLYUV420 */      //(Available if HW enabled - check HW manual) */
#define NEMADC_TSC4      (0x12U)  /**< TSC4 */
#define NEMADC_TSC6      (0x13U)  /**< TSC6 */
#define NEMADC_TSC6A     (0x14U)  /**< TSC6A */

// Hardware Colour Formats Display Modes
//--------------------------------------------------------------------------

#define NEMADC_DISABLE     (   0U   )  /**< DISABLE */
#define NEMADC_ENABLE      (1U << 31)  /**< ENABLE */
#define NEMADC_CURSOR      (1U << 30)  /**< CURSOR */
#define NEMADC_NEG_V       (1U << 28)  /**< NEG_V */
#define NEMADC_NEG_H       (1U << 27)  /**< NEG_H */
#define NEMADC_NEG_DE      (1U << 26)  /**< NEG_DE */
#define NEMADC_DITHER      (1U << 24)  /**< DITHER 18-bit */
#define NEMADC_DITHER16    (2U << 24)  /**< DITHER 16-bit */
#define NEMADC_DITHER15    (3U << 24)  /**< DITHER 15-bit */
#define NEMADC_SINGLEV     (1U << 23)  /**< SINGLEV */
#define NEMADC_INVPIXCLK   (1U << 22)  /**< INVPIXCLK */
#define NEMADC_PALETTE     (1U << 20)  /**< PALETTE */
#define NEMADC_GAMMA       (1U << 20)  /**< GAMMA */
#define NEMADC_BLANK       (1U << 19)  /**< BLANK */
#define NEMADC_INTERLACE   (1U << 18)  /**< INTERLACE */
#define NEMADC_ONE_FRAME   (1U << 17)  /**< ONE_FRAME */
#define NEMADC_P_RGB3_18B  (1U << 12)  /**< P_RGB3 */
#define NEMADC_P_RGB3_18B1 (2U << 12)  /**< P_RGB3 */
#define NEMADC_P_RGB3_16B  (3U << 12)  /**< P_RGB3 */
#define NEMADC_P_RGB3_16B1 (4U << 12)  /**< P_RGB3 */
#define NEMADC_P_RGB3_16B2 (5U << 12)  /**< P_RGB3 */
#define NEMADC_CLKOUTDIV   (1U << 11)  /**< CLKOUTDIV */
#define NEMADC_LVDSPADS    (1U << 10)  /**< LVDSPADS */
#define NEMADC_YUVOUT      (1U <<  9)  /**< YUVOUT */
#define NEMADC_MIPI_OFF    (1U <<  4)  /**< MIPI_OFF */
#define NEMADC_OUTP_OFF    (1U <<  3)  /**< OUTP_OFF */
#define NEMADC_LVDS_OFF    (1U <<  2)  /**< LVDS_OFF */
#define NEMADC_SCANDOUBLE  (1U <<  1)  /**< SCANDOUBLE */
#define NEMADC_TESTMODE    (1U <<  0)  /**< TESTMODE */
#define NEMADC_P_RGB3      (0U <<  5)  /**< P_RGB3 */
#define NEMADC_S_RGBX4     (1U <<  5)  /**< S_RGBX4 */
#define NEMADC_S_RGB3      (2U <<  5)  /**< S_RGB3 */
#define NEMADC_S_12BIT     (3U <<  5)  /**< S_12BIT */
#define NEMADC_LVDS_ISP68  (4U <<  5)  /**< LVDS_ISP68 */
#define NEMADC_LVDS_ISP8   (5U <<  5)  /**< LVDS_ISP8 */
#define NEMADC_T_16BIT     (6U <<  5)  /**< T_16BIT */
#define NEMADC_BT656       (7U <<  5)  /**< BT656 */
#define NEMADC_JDIMIP      (8U <<  5)  /**< JDIMIP */

// Configuration Check
//--------------------------------------------------------------------------

#define NEMADC_CFG_PALETTE       (1U <<  0)  /**< Global Gamma enabled */
#define NEMADC_CFG_FIXED_CURSOR  (1U <<  1)  /**< Fixed Cursor enabled */
#define NEMADC_CFG_PROGR_CURSOR  (1U <<  2)  /**< Programmable Cursor enabled */
#define NEMADC_CFG_DITHERING     (1U <<  3)  /**< Dithering enabled */
#define NEMADC_CFG_FORMAT        (1U <<  4)  /**< Formatting enabled */
#define NEMADC_CFG_HiQ_YUV       (1U <<  5)  /**< High Quality YUV converted enabled */
#define NEMADC_CFG_DBIB          (1U <<  6)  /**< DBI Type-B interface enabled */
#define NEMADC_CFG_YUVOUT        (1U <<  7)  /**< RGB to YUV converted */
#define NEMADC_CFG_L0_ENABLED    (1U <<  8)  /**< Layer 0 enabled */
#define NEMADC_CFG_L0_BLENDER    (1U <<  9)  /**< Layer 0 has blender */
#define NEMADC_CFG_L0_SCALER     (1U << 10)  /**< Layer 0 has scaler */
#define NEMADC_CFG_L0_GAMMA      (1U << 11)  /**< Layer 0 has gamma LUT */
#define NEMADC_CFG_L1_ENABLED    (1U << 12)  /**< Layer 1 enabled */
#define NEMADC_CFG_L1_BLENDER    (1U << 13)  /**< Layer 1 has blender */
#define NEMADC_CFG_L1_SCALER     (1U << 14)  /**< Layer 1 has scaler */
#define NEMADC_CFG_L1_GAMMA      (1U << 15)  /**< Layer 1 has gamma LUT */
#define NEMADC_CFG_L2_ENABLED    (1U << 16)  /**< Layer 2 enabled */
#define NEMADC_CFG_L2_BLENDER    (1U << 17)  /**< Layer 2 has blender */
#define NEMADC_CFG_L2_SCALER     (1U << 18)  /**< Layer 2 has scaler */
#define NEMADC_CFG_L2_GAMMA      (1U << 19)  /**< Layer 2 has gamma LUT */
#define NEMADC_CFG_L3_ENABLED    (1U << 20)  /**< Layer 3 enabled */
#define NEMADC_CFG_L3_BLENDER    (1U << 21)  /**< Layer 3 has blender */
#define NEMADC_CFG_L3_SCALER     (1U << 22)  /**< Layer 3 has scaler */
#define NEMADC_CFG_L3_GAMMA      (1U << 23)  /**< Layer 3 has gamma LUT */
#define NEMADC_CFG_SPI           (1U << 24)
#define NEMADC_CFG_L0_YUVMEM     (1U << 28)
#define NEMADC_CFG_L1_YUVMEM     (1U << 29)
#define NEMADC_CFG_L2_YUVMEM     (1U << 30)
#define NEMADC_CFG_L3_YUVMEM     (1U << 31)

#define NEMADC_CFG_L0_TSCMEM NEMADC_CFG_L0_YUVMEM
#define NEMADC_CFG_L1_TSCMEM NEMADC_CFG_L1_YUVMEM
#define NEMADC_CFG_L2_TSCMEM NEMADC_CFG_L2_YUVMEM
#define NEMADC_CFG_L3_TSCMEM NEMADC_CFG_L3_YUVMEM

//--------------------------------------------------------------------------

//--------------------------------------------------------------------------

//Display/LCD Parameters
//--------------------------------------------------------------------------
typedef struct __nemadc_display_t {
    uint32_t resx ;  /**< Resolution X */
    uint32_t resy ;  /**< Resolution Y */
    uint32_t fpx  ;  /**< Front Porch X */
    uint32_t fpy  ;  /**< Front Porch Y */
    uint32_t bpx  ;  /**< Back Porch X */
    uint32_t bpy  ;  /**< Back Porch Y */
    uint32_t blx  ;  /**< Blanking X */
    uint32_t bly  ;  /**< Blanking Y */
} nemadc_display_t;

//Layer Parameters
//--------------------------------------------------------------------------
typedef struct __nemadc_layer_t {
    void            *baseaddr_virt ; /**< Virtual Address */
    uintptr_t        baseaddr_phys ; /**< Physical Address */
    uint32_t         resx          ; /**< Resolution X */
    uint32_t         resy          ; /**< Resolution Y */
    int32_t          stride        ; /**< Stride */
    int32_t          startx        ; /**< Start X */
    int32_t          starty        ; /**< Start Y */
    uint32_t         sizex         ; /**< Size X */
    uint32_t         sizey         ; /**< Size Y */
    uint8_t          alpha         ; /**< Alpha */
    uint8_t          blendmode     ; /**< Blending Mode */
    uint8_t          buscfg        ; /**< ?? */
    uint32_t         format        ; /**< Format */
    uint32_t         mode          ; /**< Mode */
    //-------------------
    uint32_t         u_base        ; /**< U Base */
    uint32_t         v_base        ; /**< Y Base */
    uint32_t         u_stride      ; /**< U Stride */
    uint32_t         v_stride      ; /**< V Stride */
    //-------------------
    uint8_t          flipx_en      ; /**< Flipx_en */
    uint8_t          flipy_en      ; /**< Flipy_en */
    //-------------------
} nemadc_layer_t;


#define NEMADC_EN_PIXCLK  (1U<<22)
#define NEMADC_EN_CFCLK   (1U<<23)
#define NEMADC_EN_L0BUS   (1U<<24)
#define NEMADC_EN_L0PIX   (1U<<25)
#define NEMADC_EN_L1BUS   (1U<<26)
#define NEMADC_EN_L1PIX   (1U<<27)
#define NEMADC_EN_L2BUS   (1U<<28)
#define NEMADC_EN_L2PIX   (1U<<29)
#define NEMADC_EN_L3BUS   (1U<<30)
#define NEMADC_EN_L3PIX   (1U<<31)
//--------------------------------------------------------------------------

#define DC_STATUS_rsrvd_0             (1U<<31)
#define DC_STATUS_rsrvd_1             (1U<<30)
#define DC_STATUS_rsrvd_2             (1U<<29)
#define DC_STATUS_rsrvd_3             (1U<<28)
#define DC_STATUS_rsrvd_4             (1U<<27)
#define DC_STATUS_rsrvd_5             (1U<<26)
#define DC_STATUS_rsrvd_6             (1U<<25)
#define DC_STATUS_rsrvd_7             (1U<<24)
#define DC_STATUS_rsrvd_8             (1U<<23)
#define DC_STATUS_rsrvd_9             (1U<<22)
#define DC_STATUS_rsrvd_10            (1U<<21)
#define DC_STATUS_rsrvd_11            (1U<<20)
#define DC_STATUS_rsrvd_12            (1U<<19)
#define DC_STATUS_gpi_stuck           (1U<<18) // GPI i/f got stack durring packet transaction
#define DC_STATUS_crc_ready           (1U<<17) // CRC Register read ready
#define DC_STATUS_dbi_rd_wr_on        (1U<<16) // DBI/SPI i/f read or write ongoing operation
#define DC_STATUS_dbi_cmd_ready       (1U<<15) // DBI/SPI i/f fifo full
#define DC_STATUS_dbi_cs              (1U<<14) // DBI/SPI i/f Chip Select status
#define DC_STATUS_frame_end           (1U<<13) // End of frame pulse
#define DC_STATUS_dbi_pending_trans   (1U<<12) // DBI/SPI i/f pending command/data transaction
#define DC_STATUS_dbi_pending_cmd     (1U<<11) // DBI/SPI i/f pending command
#define DC_STATUS_dbi_pending_data    (1U<<10) // DBI/SPI i/f pending pixel data
#define DC_STATUS_mmu_error           (1U<< 9) // -- not implemented --
#define DC_STATUS_te                  (1U<< 8) // tearing
#define DC_STATUS_sticky              (1U<< 7) // underflow flag
#define DC_STATUS_underflow           (1U<< 6) // underflow signal
#define DC_STATUS_LASTROW             (1U<< 5) // last scan-row (?)
#define DC_STATUS_DPI_Csync           (1U<< 4) // DPI C-sync
#define DC_STATUS_vsync_te            (1U<< 3) // Vsync or Tearing
#define DC_STATUS_hsync               (1U<< 2) // Hsync
#define DC_STATUS_framegen_busy       (1U<< 1) // Frame-generation in-progress
#define DC_STATUS_ACTIVE              (1U<< 0) // active

#define DC_STATUS_dbi_busy            (DC_STATUS_dbi_rd_wr_on       | \
                                       DC_STATUS_dbi_cs             | \
                                       DC_STATUS_frame_end          | \
                                       DC_STATUS_dbi_pending_trans  | \
                                       DC_STATUS_dbi_pending_cmd    | \
                                       DC_STATUS_dbi_pending_data   ) // DBI/SPI i/f busy


#define NemaDC_clkctrl_cg_l3_bus_clk (1U<<31)
#define NemaDC_clkctrl_cg_l3_pix_clk (1U<<30)
#define NemaDC_clkctrl_cg_l2_bus_clk (1U<<29)
#define NemaDC_clkctrl_cg_l2_pix_clk (1U<<28)
#define NemaDC_clkctrl_cg_l1_bus_clk (1U<<27)
#define NemaDC_clkctrl_cg_l1_pix_clk (1U<<26)
#define NemaDC_clkctrl_cg_l0_bus_clk (1U<<25)
#define NemaDC_clkctrl_cg_l0_pix_clk (1U<<24)
#define NemaDC_clkctrl_cg_regfil_clk (1U<<23)
#define NemaDC_clkctrl_cg_bypass_clk (1U<<22)
//----------------------------------------------
#define NemaDC_clkctrl_cg_rsrvd_21   (1U<<21)
#define NemaDC_clkctrl_cg_rsrvd_20   (1U<<20)
#define NemaDC_clkctrl_cg_rsrvd_19   (1U<<19)
#define NemaDC_clkctrl_cg_rsrvd_18   (1U<<18)
#define NemaDC_clkctrl_cg_rsrvd_17   (1U<<17)
#define NemaDC_clkctrl_cg_rsrvd_16   (1U<<16)
#define NemaDC_clkctrl_cg_rsrvd_15   (1U<<15)
#define NemaDC_clkctrl_cg_rsrvd_14   (1U<<14)
#define NemaDC_clkctrl_cg_rsrvd_13   (1U<<13)
#define NemaDC_clkctrl_cg_rsrvd_12   (1U<<12)
#define NemaDC_clkctrl_cg_rsrvd_11   (1U<<11)
#define NemaDC_clkctrl_cg_rsrvd_10   (1U<<10)
#define NemaDC_clkctrl_cg_rsrvd_9    (1U<< 9)
#define NemaDC_clkctrl_cg_rsrvd_8    (1U<< 8)
#define NemaDC_clkctrl_cg_rsrvd_7    (1U<< 7)
#define NemaDC_clkctrl_cg_rsrvd_6    (1U<< 6)
#define NemaDC_clkctrl_cg_rsrvd_5    (1U<< 5)
#define NemaDC_clkctrl_cg_rsrvd_4    (1U<< 4)
#define NemaDC_clkctrl_cg_rsrvd_3    (1U<< 3)
//----------------------------------------------
#define NemaDC_clkctrl_cg_clk_swap   (1U<< 2)
#define NemaDC_clkctrl_cg_clk_inv    (1U<< 1)
#define NemaDC_clkctrl_cg_clk_en     (1U<< 0)
//--------------------------------------------------------------------------

// Functions
//-----------------------------------------------------------------------------------------------------------------------
/** \brief Initialize NemaDC library
 *
 * \return -1 on error
 *
 */
int      nemadc_init       (void);

/** \brief Read Configuration Register
 *
 * \return Configuration Register Value
 *
 */
uint32_t nemadc_get_config(void);

/** \brief Read CRC Checksum Register
 *
 * \return CRC checksum value of last frame. For testing purposes
 *
 */
uint32_t nemadc_get_crc(void);

/** \brief Set NemaDC Background Color
 *
 * \param rgba Color as a 32-bit rgba value (0xRRGGBBXX - Red: color[31:24], Green: color[23:16], Blue: color[15:8])
 *
 */
void     nemadc_set_bgcolor(uint32_t rgba);

/** \brief Set Display timing parameters
 *
 * \param resx Resolution X
 * \param fpx Front Porch X
 * \param blx Blanking X
 * \param bpx Back Porch X
 * \param resy Resolution Y
 * \param fpy Front Porch Y
 * \param bly Blanking Y
 * \param bpy Back Porch Y
 *
 */
void     nemadc_timing(int resx, int fpx, int blx, int bpx,
                       int resy, int fpy, int bly, int bpy);

/** \brief Return stride size in bytes
 *
 * \param format Texture color format
 * \param width  Texture width
 * \return Stride in bytes
 *
 */
int      nemadc_stride_size(uint32_t format, int width);


/** \brief Set the built-in Clock Dividers and DMA Line Prefetch. (See Configuration Register 0x4)
 *
 * \param div Set Divider 1
 * \param div2 Set Divider 2
 * \param dma_prefetch Set number of lines for the dma to prefetch
 * \param phase Clock phase shift
 *
 */
void     nemadc_clkdiv(int div, int div2, int dma_prefetch, int phase);

/** \brief Control the clock gaters
 *
 * \param ctrl clock control
 */
void     nemadc_clkctrl   (uint32_t ctrl);

/** \brief Set operation mode
 *
 * \param mode Mode of operation (See Register 0)
 */
void     nemadc_set_mode  (int mode);

/** \brief Get status from Status Register
 *
 *
 */
uint32_t nemadc_get_status (void);

/** \brief Request a VSync Interrupt without blocking
 *
 *
 */
void     nemadc_request_vsync_non_blocking(void);

/** \brief Set the Layer Mode. This function can enable a layer and set attributes to it
 *
 * \param layer_no The layer number
 * \param layer Layer Attributes struct
 *
 */
void     nemadc_set_layer (int layer_no, nemadc_layer_t *layer);

/** \brief Set the physical address of a layer.
 *
 * \param layer_no The layer number
 * \param addr Layer Physical Address
 *
 */
void     nemadc_set_layer_addr(int layer_no, uintptr_t addr);

/** \brief Set an entry in the lut8 Palette Gamma table for a layer
 *
 * \param layer Layer number
 * \param index Color Index
 * \param Color 32-bit RGBA color value or gamma index
 *
 */
void     nemadc_set_layer_gamma_lut(int layer, int index, int colour);


/** \brief Get an entry in the lut8 Palette Gamma table for a layer
 *
 * \param layer Layer number
 * \param index Color Index
 * \return Palette index
 *
 **/
int      nemadc_get_layer_gamma_lut(int layer, int index);

/** \brief Sets an entry in the lut8 Palatte Gamma table
 *
 * \param uint32_t index Color Index
 * \param uint32_t colour 32-bit RGBA colour value or Gamma index
 *
 **/
void     nemadc_set_palette(uint32_t index, uint32_t colour);

/** \brief Reads an entry from the lut8 Palatte Gamma table
 *
 * \param index Color Index
 * \return Return Colour for given palette index
 *
 **/
int      nemadc_get_palette(uint32_t index);

/** \brief Disable layer
 *
 * \param layer_no Layer Number
 *
 */
void     nemadc_layer_disable(int layer_no);

/** \brief Enable layer
 *
 * \param layer_no Layer Number
 *
 */
void     nemadc_layer_enable(int layer_no);

/** \brief Enable or Disable fixed cursor
 *
 * \param enable 1 for enable or 0 for disable cursor
 *
 */
void     nemadc_cursor_enable(int enable);

/** \brief Set the location of the cursor
 *
 * \param x Cursor X coordinate
 * \param x Cursor Y coordinate
 *
 */
void     nemadc_cursor_xy(int x, int y);


/** \brief Set programmable cursor image (32x32 pixels)
 *
 * \param img Base address of the 32x32 Cursor Image
 *
 */
void     nemadc_set_cursor_img(unsigned char *img);


/** \brief Set a color for the Cursor LUT
 *
 * \param index Color index
 * \param color 32-bit RGBA value
 *
 */
void     nemadc_set_cursor_lut(uint32_t index, uint32_t color);


/** \brief Check whether NemaDC supports a specific characteristic
 *
 * \param flag Flag to query
 * \return True if the characteristic is supported
 *
 */
unsigned char nemadc_check_config(uint32_t flag);


/** \brief Read Color Mode Register
 *
 * \return Color mode register
 *
 */
uint32_t nemadc_get_col_mode(void);


/** \brief Get the number of layers available
 *
 * \return Number of layers
 *
 */
int      nemadc_get_layer_count(void);
//-----------------------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif

#endif
