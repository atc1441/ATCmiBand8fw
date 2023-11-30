//*****************************************************************************
//
//! @file am_hal_usbregs.h
//!
//! @brief CMSIS-style register definitions for the USB registers.
//!
//
//! The Ambiq USB implementation is 8-bit oriented, yet multiple USB registers
//! are packed into 32-bit registers. Also, many of the USB aregisters have
//! read or write side-effects. Therefore many of the registers must be byte
//! accessed as reading the entire 32-bits would trigger the side-effects of
//! registers that are not even of interest.
//!
//! Given all that, and that CMSIS generated registers are 32-bit oriented,
//! this CMSIS-like register structure is implemented.
//!
//! Thus we define the USB registers such that the compilers will access each
//! register with an appropriate sized access.  e.g. 8-bit registers will be
//! accessed as bytes and 16-bit registers as half-words, etc.
//!
//!
//! Usage of the USB registers is very similar to CMSIS access.
//! Some simple usage examples:
//!
//!  uint8_t ui32Faddr = 0x1234;
//!  USBHAL->FADDR = ui32Faddr;
//!
//!  if ( USBHAL->CSR0_b.OutPktRdy ) { do stuff };
//!
//!  if ( USBHAL->COUNT0_b.Count0 == 0x4 ) { do stuff };
//!
//!  USBHAL->INTRUSBen_b.SOF = 1;
//!
//! @addtogroup usb_regs_4b USBRREGS - USB Register Functionality
//! @ingroup apollo4b_hal
//! @{
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

#ifndef AM_HAL_USBREGS_H
#define AM_HAL_USBREGS_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! This anonymous union handling is taken right from the CMSIS generated file.
//
//*****************************************************************************
#if defined (__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined (__ICCARM__)
  #pragma language = extended
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Wc11-extensions"
  #pragma clang diagnostic ignored "-Wreserved-id-macro"
  #pragma clang diagnostic ignored "-Wgnu-anonymous-struct"
  #pragma clang diagnostic ignored "-Wnested-anon-types"
#elif defined (__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined (__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined (__TASKING__)
  #pragma warning 586
#elif defined (__CSMC__)
  /* anonymous unions are enabled by default */
#else
  #warning Not supported compiler type
#endif

//*****************************************************************************
//
//! Ambiq USB registers
//
//*****************************************************************************
typedef struct
{
    //! CFG0
    union
    {
        __IOM uint32_t CFG0;                         //32bit
        struct
        {
            struct
            {                                       // 0x0
                __IOM uint8_t FuncAddr               : 7;
                __IM  uint8_t Update                 : 1;
            } FADDR_b;
            struct
            {                                       // 0x1
                __IOM uint8_t EnableSuspendM         : 1;
                __IM  uint8_t SuspendMode            : 1;
                __IOM uint8_t Resume                 : 1;
                __IM  uint8_t Reset                  : 1;
                __IM  uint8_t HSMode                 : 1;
                __IOM uint8_t HSEnable               : 1;
                __IOM uint8_t VersionSpecific        : 1;
                __IOM uint8_t ISOUpdate              : 1;
            } POWER_b;
            union
            {
                __IOM uint16_t INTRIN;               // 0x2,3
                struct
                {
                    __IOM uint16_t EP0               : 1;
                    __IOM uint16_t EP1               : 1;
                    __IOM uint16_t EP2               : 1;
                    __IOM uint16_t EP3               : 1;
                    __IOM uint16_t EP4               : 1;
                    __IOM uint16_t EP5               : 1;
                    __IM  uint16_t RSVD              : 10;
                } INTRIN_b;
            };
        };
    };

    //! CFG1
    union
    {
        __IOM uint32_t CFG1;                         //32bit
        struct
        {
            union
            {
                __IOM uint16_t INTROUT;              // 0x4,5
                struct
                {
                    __IOM uint16_t EP0               : 1;
                    __IOM uint16_t EP1               : 1;
                    __IOM uint16_t EP2               : 1;
                    __IOM uint16_t EP3               : 1;
                    __IOM uint16_t EP4               : 1;
                    __IOM uint16_t EP5               : 1;
                    __IM  uint16_t RSVD              : 10;
                } INTROUT_b;
            };
            union
            {
                __IOM uint16_t INTRINE;              // 0x6,7
                struct
                {
                    __IOM uint16_t EP0               : 1;
                    __IOM uint16_t EP1               : 1;
                    __IOM uint16_t EP2               : 1;
                    __IOM uint16_t EP3               : 1;
                    __IOM uint16_t EP4               : 1;
                    __IOM uint16_t EP5               : 1;
                    __IM  uint16_t RSVD              : 10;
                } INTRINE_b;
            };
        };
    };

    //! CFG2
    union
    {
        __IOM uint32_t CFG2;                         //32bit
        struct
        {
            union
            {
                __IOM uint16_t INTROUTE;             // 0x8,9
                struct
                {
                    __IOM uint16_t EP0               : 1;
                    __IOM uint16_t EP1               : 1;
                    __IOM uint16_t EP2               : 1;
                    __IOM uint16_t EP3               : 1;
                    __IOM uint16_t EP4               : 1;
                    __IOM uint16_t EP5               : 1;
                    __IM  uint16_t RSVD              : 10;
                } INTROUTE_b;
            };
            union
            {
                __IOM uint8_t INTRUSB;               // 0xA
                struct
                {
                    __IM uint8_t Suspend             : 1;
                    __IM uint8_t Resume              : 1;
                    __IM uint8_t Reset               : 1;
                    __IM uint8_t SOF                 : 1;
                    __IM uint8_t RSVD                : 4;
                } INTRUSB_b;
            };
            union
            {
                __IOM uint8_t INTRUSBE;              // 0xB
                struct
                {
                    __IOM uint8_t Suspend            : 1;
                    __IOM uint8_t Resume             : 1;
                    __IOM uint8_t Reset              : 1;
                    __IOM uint8_t SOF                : 1;
                    __IM  uint8_t RSVD               : 4;
                } INTRUSBE_b;
            };
        };
    };

    //! CFG3
    union
    {
        __IOM uint32_t CFG3;                         //32bit
        struct
        {
            struct
            {                                       // 0xC,D
                __IOM uint16_t FrameNumber           : 11;
                __IM  uint16_t RSVD                  : 5;
            } FRAME_b;
            struct
            {                                       // 0xE
                __IOM uint8_t EndPoint               : 4;
                __IM  uint8_t RSVD                   : 4;
            } INDEX_b;
            struct
            {                                       // 0xF
                __IOM uint8_t TestSE0NAK             : 1;
                __IOM uint8_t TestJ                  : 1;
                __IOM uint8_t TestK                  : 1;
                __IOM uint8_t TestPacket             : 1;
                __IOM uint8_t ForceHS                : 1;
                __IOM uint8_t ForceFS                : 1;
                __IM  uint8_t RSVD                   : 2;
            } TESTMODE_b;
        };
    };

    //! IDX0
    union
    {
        __IOM uint32_t IDX0;                         //32bit
        struct
        {
            struct                                   // 0x10
            {
               __IOM uint16_t MaxPayload             : 11;
               __IOM uint16_t SplitMultiplier        : 5;
            } INMAXP_b;
            union                                    // 0x12
            {
                struct
                {
                   __IOM uint8_t OutPktRdy           : 1;
                   __IOM uint8_t InPktRdy            : 1;
                   __IOM uint8_t SentStall           : 1;
                   __IOM uint8_t DataEnd             : 1;
                   __IOM uint8_t SetupEnd            : 1;
                   __IOM uint8_t SendStall           : 1;
                   __IOM uint8_t ServicedOutPktRdy   : 1;
                   __IOM uint8_t ServicedSetupEnd    : 1;
                } CSR0_b;
                struct
                {
                   __IOM uint8_t InPktRdy            : 1;
                   __IOM uint8_t FIFONotEmpty        : 1;
                   __IOM uint8_t UnderRun            : 1;
                   __IOM uint8_t FlushFIFO           : 1;
                   __IOM uint8_t SendStall           : 1;
                   __IOM uint8_t SentStall           : 1;
                   __IOM uint8_t ClrDataTog          : 1;
                   __IOM uint8_t IncompTx            : 1;
                } INCSRL_b;
            };
            struct                                   // 0x13
            {
               __IOM uint8_t Unused                  : 1;
               __IOM uint8_t DPktBufDis              : 1;
               __IOM uint8_t DMAReqMode              : 1;
               __IOM uint8_t FrcDataTog              : 1;
               __IOM uint8_t DMAReqEnab              : 1;
               __IOM uint8_t Mode                    : 1;
               __IOM uint8_t ISO                     : 1;
               __IOM uint8_t AutoSet                 : 1;
            } INCSRU_b;
        };
    };

    //! IDX1
    union
    {
        __IOM uint32_t IDX1;                         //32bit
        struct
        {
            struct
            {                                       // 0x14-0x15
               __IOM uint16_t MaxPayload             : 11;
               __IOM uint16_t CombineMultiplier      : 5;
            } OUTMAXP_b;
            struct
            {                                       // 0x16
               __IOM uint8_t OutPktRdy               : 1;
               __IOM uint8_t FIFOFull                : 1;
               __IOM uint8_t OverRun                 : 1;
               __IOM uint8_t DataError               : 1;
               __IOM uint8_t FlushFIFO               : 1;
               __IOM uint8_t SendStall               : 1;
               __IOM uint8_t SentStall               : 1;
               __IOM uint8_t ClrDataTog              : 1;
            } OUTCSRL_b;
            struct
            {                                       // 0x17
               __IOM uint8_t IncompRx                : 1;
               __IOM uint8_t DPktBufDis              : 1;
               __IOM uint8_t Unused                  : 1;
               __IOM uint8_t DMAReqMode              : 1;
               __IOM uint8_t DisNyetPIDError         : 1;
               __IOM uint8_t DMAReqEnab              : 1;
               __IOM uint8_t ISO                     : 1;
               __IOM uint8_t AutoClear               : 1;
            } OUTCSRU_b;
        };
    };

    //! IDX2
    union
    {
        __IOM uint32_t IDX2;
        struct
        {
            union
            {
                struct
                {                                   // 0x18
                    __IOM uint8_t COUNT0             : 7;
                    __IM  uint8_t Pad                : 1;
                } COUNT0_b;
                struct
                {                                   // 0x18
                    __IOM uint16_t OUTCOUNT          : 13;
                    __IM  uint16_t Pad               : 3;
                } OUTCOUNT_b;
            };
            struct
            {                                       // 0x1a
                __IM  uint8_t Unused                 : 3;
                __IOM uint8_t DPB                    : 1;
                __IOM uint8_t InFIFOsz               : 4;
            } InFIFOsz_b;
            struct
            {                                       // 0x1b
                __IM  uint8_t Unused                 : 3;
                __IOM uint8_t DPB                    : 1;
                __IOM uint8_t OutFIFOsz              : 4;
            } OutFIFOsz_b;
        };
    };

    //! In FIFO Address
    struct
    {                                               // 0x1c
        __IM  uint16_t Unused                        : 2;
        __IM  uint16_t Reserved                      : 1;
        __IOM uint16_t InFIFOAddr                    : 13;
    } InFIFOAddr_b;

    //! Out FIFO Address
    struct
    {                                               //0x1e
        __IM  uint16_t Unused                        : 2;
        __IM  uint16_t Reserved                      : 1;
        __IOM uint16_t OutFIFOAddr                   : 13;
    } OutFIFOAddr_b;

    //! FIFOx
    __IOM uint32_t  FIFOx[6];                        // 0x20 - 0x34

    //! RESERVED
    __IM  uint32_t  RESERVED[13];

    //! HWVERS
    union
    {
        __IOM uint32_t HWVERS;                       // 0x6C
        struct
        {
            __IOM uint32_t yyy                       : 10;
            __IOM uint32_t xx                        : 5;
            __IOM uint32_t RC                        : 1;
        } HWVERS_b;
    };

    //! RESERVED1
    __IM  uint32_t RESERVED1[2];

    //! INFO
    union
    {
        __IOM uint32_t INFO;                         // 0x78
        struct
        {
            __IOM uint32_t InEndPoints               : 4;
            __IOM uint32_t OutEndPoints              : 4;
            __IOM uint32_t RamBits                   : 4;
            __IOM uint32_t DMAChains                 : 4;
            __IOM uint32_t RSTS                      : 1;
            __IM  uint32_t                           : 6;
            __IOM uint32_t EDMAOUT                   : 1;
            __IOM uint32_t EDMAIN                    : 1;
        } INFO_b;
    };

    //! RESERVED2
    __IM uint32_t RESERVED2;

    //! TIMEOUT1
    union
    {
        __IOM uint32_t TIMEOUT1;                     // 0x80
        struct
        {
            __IOM uint32_t CTUCH                     : 16;
        } TIMEOUT1_b;
    };

    //! TIMEOUT2
    union
    {
        __IOM uint32_t TIMEOUT2;                     //0x84
        struct
        {
            __IOM uint32_t CTHRSTN                   : 16;
        } TIMEOUT2_b;
    };

    //! RESERVED3
    __IM uint32_t RESERVED3[2014];

    //! CLKCTRL
    union
    {
        __IOM uint32_t CLKCTRL;                      //0x2000
        struct
        {
            __IOM uint32_t PHYREFCLKDIS              : 1;
            __IM  uint32_t                           : 7;
            __IOM uint32_t CTRLAPBCLKDIS             : 1;
            __IM  uint32_t                           : 7;
            __IOM uint32_t PHYAPBLCLKDIS             : 1;
        } CLKCTRL_b;
    };

    //! SRAMCTRL
    union
    {
        __IOM uint32_t SRAMCTRL;                     // 0x2004
        struct
        {
            __IOM uint32_t RET1N                     : 1;
            __IOM uint32_t EMA                       : 3;
            __IOM uint32_t EMAS                      : 1;
            __IOM uint32_t EMAW                      : 2;
            __IOM uint32_t RAWLM                     : 2;
            __IOM uint32_t RAWL                      : 1;
            __IOM uint32_t WABLM                     : 3;
            __IOM uint32_t WABL                      : 1;
            __IOM uint32_t STOV                      : 1;
        } SRAMCTRL_b;
    };

    //! RESERVED4
    __IM uint32_t RESERVED4[3];

    //! UTMISTICKYSTATUS
    union
    {
        __IOM uint32_t UTMISTICKYSTATUS;             //0x2014
        struct
        {
            __IOM uint32_t obsportstciky             : 2;
        } UTMISTICKYSTATUS_b;
    };

    //! OBSCLRSTAT
    union
    {
        __IOM uint32_t OBSCLRSTAT;
        struct
        {
            __IOM uint32_t CLRSTAT                   : 1;
        } OBSCLRSTAT_b;
    };

    //! DPDMPULLDOWN
    union
    {
        __IOM uint32_t DPDMPULLDOWN;
        struct
        {
            __IOM uint32_t DMPULLDOWN                : 1;
            __IOM uint32_t DPPULLDOWN                : 1;
        } DPDMPULLDOWN_b;
    };
} USBHAL_Type;

//*****************************************************************************
//
//! This anonymous union handling is taken right from the CMSIS generated file.
//
//*****************************************************************************
#if defined (__CC_ARM)
#pragma pop
#elif defined (__ICCARM__)
/* leave anonymous unions enabled */
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#pragma clang diagnostic pop
#elif defined (__GNUC__)
/* anonymous unions are enabled by default */
#elif defined (__TMS470__)
/* anonymous unions are enabled by default */
#elif defined (__TASKING__)
#pragma warning restore
#elif defined (__CSMC__)
/* anonymous unions are enabled by default */
#endif

//*****************************************************************************
//
//! USB Peripheral declaration
//
//*****************************************************************************
#define USBHAL      ((USBHAL_Type*)             ((uint8_t*)(USB_BASE)))

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_USBREGS_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

