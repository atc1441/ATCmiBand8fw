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

#ifndef NEMA_DC_INTERN_H__
#define NEMA_DC_INTERN_H__


#define NEMADC_MAGIC     0x87452365
//--------------------------------------------------------------------------

//--------------------------------------------------------------------------
//Color Mode Check
//--------------------------------------------------------------------------
#define NEMADC_DBIB_STALL         (1U << 31)
#define NEMADC_DBIB_STALL_CHK     (1U << 31)
#define NEMADC_OPENLDI_CHK        (1U << 30)
#define NEMADC_JDIMIP_CHK         (1U << 29)
#define NEMADC_ARGB4444_CHK       (1U << 22)
#define NEMADC_RGBA4444_CHK       (1U << 21)
#define NEMADC_GPI_CHK            (1U << 20)
#define NEMADC_EXTRCTRL_CHK       (1U << 19)
#define NEMADC_TSC6_CHK           (1U << 18)
#define NEMADC_TSC_CHK            (1U << 17)
#define NEMADC_LUT8_CHK           (1U << 16)
#define NEMADC_RGBA5551_CHK       (1U << 15)
#define NEMADC_ABGR8888_CHK       (1U << 14)
#define NEMADC_RGB332_CHK         (1U << 13)
#define NEMADC_RGB565_CHK         (1U << 12)
#define NEMADC_BGRA8888_CHK       (1U << 11)
#define NEMADC_L8_CHK             (1U << 10)
#define NEMADC_L1_CHK             (1U <<  9)
#define NEMADC_L4_CHK             (1U <<  8)
#define NEMADC_YUVV_CHK           (1U <<  7)
#define NEMADC_RGB24_CHK          (1U <<  6)
#define NEMADC_YUY2_CHK           (1U <<  5)
#define NEMADC_RGBA8888_CHK       (1U <<  4)
#define NEMADC_ARGB8888_CHK       (1U <<  3)
#define NEMADC_V_YUV420_CHK       (1U <<  2)
#define NEMADC_TLYUV420_CHK       (1U <<  1)
#define NEMADC_BLOCK4X4_CHK       (1U <<  0)


//--------------------------------------------------------------------------
//AXI control
//--------------------------------------------------------------------------
#define NEMADC_AXI_16BEAT   (0U)
#define NEMADC_AXI_2BEAT    (1U)
#define NEMADC_AXI_4BEAT    (2U)
#define NEMADC_AXI_8BEAT    (3U)
#define NEMADC_AXI_32BEAT   (5U)
#define NEMADC_AXI_64BEAT   (6U)
#define NEMADC_AXI_128BEAT  (7U)

#define NEMADC_AXI_FT_HF    (0U << 3)
#define NEMADC_AXI_FT_2B    (1U << 3)
#define NEMADC_AXI_FT_4B    (2U << 3)
#define NEMADC_AXI_FT_8B    (3U << 3)


#define NemaDC_snapshot              (255U)
    //------------------------------------
#define NemaDC_store_base_addr       (1U<<31)
#define NemaDC_DBI_cmd               (1U<<30)
#define NemaDC_wcmd16                (1U<<28)
#define NemaDC_wcmd24                (1U<<29)
#define NemaDC_wcmd32                ((1U<<29)|(1U<<28))
#define NemaDC_rcmd16                (1U<<28)
#define NemaDC_rcmd24                (1U<<29)
#define NemaDC_rcmd32                ((1U<<29)|(1U<<28))
#define NemaDC_mask_qspi             (1U<<27)
#define NemaDC_DBI_ge                (1U<<27)
#define NemaDC_DBI_read              (1U<<26)
#define NemaDC_ext_ctrl              (1U<<25)
#define NemaDC_sline_cmd             (1U<<24)
    //-------------------------------------
#define NemaDC_read_byte             (0U<<30)
#define NemaDC_read_2byte            (1U<<30)
#define NemaDC_read_3byte            (2U<<30)
#define NemaDC_read_4byte            (3U<<30)


#define SPI_WRITE                                  (2U)
#define SPI_READ                                   (3U)
//--
#define SPICMD                                 (1U<<5)
#define QSPICMD                                (0U<<5)
#define QSPIDATA                               (1U<<4)

#define CMD_OFFSET                                 (8U)
//--
#define CMD1_DATA1         (                 SPI_WRITE)
#define CMD1_DATA4         ( SPICMD|QSPIDATA|SPI_WRITE)
#define CMD4_DATA4         (QSPICMD|QSPIDATA|SPI_WRITE)
#define CMD1_RDAT1         (                 SPI_READ )

#endif
