//*****************************************************************************
//
//! @file svc_throughput.h
//!
//! @brief AmbiqMicro throughput service definition
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
#ifndef SVC_THROUGHPUT_H
#define SVC_THROUGHPUT_H

//
// Put additional includes here if necessary.
//

#ifdef __cplusplus
extern "C"
{
#endif
//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************

/*! Base UUID:  00002760-08C2-11E1-9073-0E8AC72EXXXX */
#define ATT_UUID_AMBIQ_BASE             0x2E, 0xC7, 0x8A, 0x0E, 0x73, 0x90, \
                                            0xE1, 0x11, 0xC2, 0x08, 0x60, 0x27, 0x00, 0x00

/*! Macro for building Ambiq UUIDs */
#define ATT_UUID_AMBIQ_BUILD(part)      UINT16_TO_BYTES(part), ATT_UUID_AMBIQ_BASE

/*! Partial throughput service UUIDs */
#define ATT_UUID_THROUGHPUT_SERVICE_PART     0x5450

/*! Partial throughput rx characteristic UUIDs */
#define ATT_UUID_THROUGHPUT_RX_PART          0x5401

/*! Partial throughput tx characteristic UUIDs */
#define ATT_UUID_THROUGHPUT_TX_PART          0x5402
/*! Partial throughput connection update characteristic UUIDs */
#define ATT_UUID_THROUGHPUT_CON_UPT_PART     0x5403

/* Throughput services */
#define ATT_UUID_THROUGHPUT_SERVICE          ATT_UUID_AMBIQ_BUILD(ATT_UUID_THROUGHPUT_SERVICE_PART)

/* Throughput characteristics */
#define ATT_UUID_THROUGHPUT_RX               ATT_UUID_AMBIQ_BUILD(ATT_UUID_THROUGHPUT_RX_PART)
#define ATT_UUID_THROUGHPUT_TX               ATT_UUID_AMBIQ_BUILD(ATT_UUID_THROUGHPUT_TX_PART)
#define ATT_UUID_THROUGHPUT_CON_UPT          ATT_UUID_AMBIQ_BUILD(ATT_UUID_THROUGHPUT_CON_UPT_PART)
// throughput Service
#define THROUGHPUT_START_HDL               0x0840
#define THROUGHPUT_END_HDL                 (THROUGHPUT_MAX_HDL - 1)


/* Throughput Service Handles */
enum
{
  THROUGHPUT_SVC_HDL = THROUGHPUT_START_HDL,     /* Throughput service declaration */
  THROUGHPUT_RX_CH_HDL,                     /* Throughput write command characteristic */
  THROUGHPUT_RX_HDL,                        /* Throughput write command data */
  THROUGHPUT_TX_CH_HDL,                     /* Throughput notify characteristic */
  THROUGHPUT_TX_HDL,                        /* Throughput notify data */
  THROUGHPUT_TX_CH_CCC_HDL,                 /* Throughput notify client characteristic configuration */
  THROUGHPUT_CON_UPT_CH_HDL,                 /* Throughput connection update characteristic*/
  THROUGHPUT_CON_UPT_HDL,                    /* Throughput connection update data*/
  THROUGHPUT_CON_UPT_CCC_HDL,                /* Throughput connection update data notify client characteristic configuration */
  THROUGHPUT_MAX_HDL
};


//*****************************************************************************
//
// External variable definitions
//
//*****************************************************************************

//*****************************************************************************
//
// Function definitions.
//
//*****************************************************************************
void SvcThroughputAddGroup(void);
void SvcThroughputRemoveGroup(void);
void SvcThroughputCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback);

#ifdef __cplusplus
}
#endif

#endif // SVC_THROUGHPUT_H
