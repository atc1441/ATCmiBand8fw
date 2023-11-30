//*****************************************************************************
//
//! @file hci_drv_apollo.h
//!
//! @brief Additional header information for the Apollo implementation of HCI.
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
#ifndef HCI_DRV_APOLLO_H
#define HCI_DRV_APOLLO_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// Errors
//
//*****************************************************************************
#define HCI_DRV_SPECIFIC_ERROR_START   0x09000000
typedef enum
{
    HCI_DRV_TRANSMIT_QUEUE_FULL = HCI_DRV_SPECIFIC_ERROR_START,
    HCI_DRV_TX_PACKET_TOO_LARGE,
    HCI_DRV_RX_PACKET_TOO_LARGE,
    HCI_DRV_BLE_STACK_UNABLE_TO_ACCEPT_PACKET,
    HCI_DRV_PACKET_TRANSMIT_FAILED,
    HCI_DRV_IRQ_STUCK_HIGH,
    HCI_DRV_TOO_MANY_PACKETS,
}
hci_drv_error_t;

typedef void (*hci_drv_error_handler_t)(uint32_t ui32Error);

//*****************************************************************************
//
// Function prototypes.
//
//*****************************************************************************
extern void HciDrvUartEnable(void);
extern void HciDrvUartDisable(void);
extern void HciDrvUartFlowOff(void);
extern void HciDrvUartFlowOn(void);
extern void HciDrvUartPause(void);
extern void HciDrvUartUnpause(void);
extern bool HciDrvUartSafeShutdown(void);

#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4L) || defined(AM_PART_APOLLO4P)
extern uint32_t HciDrvRadioBoot(bool bColdBoot);
#else
#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
extern uint32_t HciDrvRadioBoot(bool bColdBoot);
#else
extern void HciDrvRadioBoot(uint32_t ui32UartModule);
#endif
#endif

extern void HciDrvRadioShutdown(void);
extern void HciDrvUartISR(uint32_t ui32Status);
extern bool_t HciDataReadyISR(void);
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4L) || defined(AM_PART_APOLLO4P)
#else
extern void HciDrvIntService(void);
#endif
extern void HciDrvGPIOService(void);
extern void HciDrvHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg);
extern void HciDrvErrorHandlerSet(hci_drv_error_handler_t pfnErrorHandler);

#ifdef __cplusplus
};
#endif

#endif // HCI_DRV_APOLLO_H
