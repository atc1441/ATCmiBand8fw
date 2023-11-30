//*****************************************************************************
//
//! @file am_hal_secure_ota.h
//!
//! @brief Implementation for Secure OTA Functionality.
//!
//! @addtogroup secure_ota_4b Secure OTA Functionality
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

#ifndef AM_HAL_SECURE_OTA_H
#define AM_HAL_SECURE_OTA_H
// Ambiq Standard Image Format related definitions
// Magic Numbers
#define AM_IMAGE_MAGIC_SBL                0xA3
#define AM_IMAGE_MAGIC_SECURE             0xC0
#define AM_IMAGE_MAGIC_OEM_CHAIN          0xCC
#define AM_IMAGE_MAGIC_NONSECURE          0xCB
#define AM_IMAGE_MAGIC_INFO0              0xCF
#define AM_IMAGE_MAGIC_CONTAINER          0xC1
#define AM_IMAGE_MAGIC_KEYREVOKE          0xCE
#define AM_IMAGE_MAGIC_DOWNLOAD           0xCD

typedef struct
{
    union
    {
        uint32_t     ui32;
        struct
        {
            uint32_t    blobSize     : 22;
            uint32_t    rsvd         : 4;
            uint32_t    crcCheck     : 1;
            uint32_t    enc          : 1;
            uint32_t    authCheck    : 1;
            uint32_t    ccIncluded   : 1;
            uint32_t    ambiq        : 1;
            uint32_t    rsvd1        : 1;
        } s;
    } w0;
    uint32_t        crc;
    union
    {
        uint32_t    ui32;
        uint32_t    ui32Resv;
        struct
        {
            uint32_t    authKeyIdx   : 8;
            uint32_t    encKeyIdx    : 8;
            uint32_t    authAlgo     : 4;
            uint32_t    encAlgo      : 4;
            uint32_t    resvd        : 8;
        } s;
    } w2;
    union
    {
        uint32_t ui32;
    } w3;
} am_image_hdr_common_t;

typedef union
{
    uint32_t        ui32Words[12];
    uint8_t         ui8Bytes[48];
    struct
    {
        uint32_t        kek[4];
        uint32_t        iv[4];
        uint32_t        rsvd[4];
    } aesCbc128;
    struct
    {
        uint32_t        kek[4];
        uint32_t        iv[4];
        uint32_t        rsvd[4];
    } aesCtr128;
#if 0
    struct
    {
        uint32_t        kek[4];
        uint32_t        tag[4];
        uint32_t        nonce[3];
        uint32_t        rsvd[1];
    } aesCcm128;
#endif
} am_image_enc_info_t;

typedef union
{
    uint8_t        ui8Signature[384];
    uint32_t       ui32Signature[96];
} am_image_auth_info_t;

typedef union
{
    uint32_t   ui32;
    uint8_t    ui8[4];
    struct
    {
        uint32_t    magicNum    : 8;
        uint32_t    ccSize      : 11;
        uint32_t    rsvd        : 13;
    } s;
    struct
    {
        uint32_t    magicNum    : 8;
        uint32_t    ota         : 1;
        uint32_t    sblOta      : 1;
        uint32_t    options     : 6;
        uint32_t    rsvd        : 16;
    } wired;
} am_image_opt0_t;

typedef union
{
    uint32_t   ui32;
    struct
    {
        uint32_t    rsvd        : 2;
        uint32_t    loadAddrMsb : 30;
    } fw;
    struct
    {
        uint32_t    offset      : 12;
        uint32_t    size        : 12;
        uint32_t    rsvd        : 8;
    } info0;
    struct
    {
        uint32_t    rsvd        : 2;
        uint32_t    loadAddrMsb : 30;
    } wired;
} am_image_opt1_t;

typedef union
{
    uint32_t   ui32;
    uint32_t   ui32Key;
} am_image_opt2_t;

typedef union
{
    uint32_t   ui32;
} am_image_opt3_t;

typedef struct
{
   am_image_opt0_t  opt0;
   am_image_opt1_t  opt1;
   am_image_opt2_t  opt2;
   am_image_opt3_t  opt3;
} am_image_opt_info_t;


// Maximum number of OTAs
#define AM_HAL_SECURE_OTA_MAX_OTA           8

typedef struct
{
    uint32_t upgrade[AM_HAL_SECURE_OTA_MAX_OTA + 1];
} am_hal_otadesc_t;

// Reserved magic numbers allowed to be used by customer's own bootloader
//#define AM_IMAGE_MAGIC_CUST(x)   ((((x) & 0xF0) == 0xC0) && ((x) != 0xC0) && ((x) != 0xCC) && ((x) != 0xCB) && ((x) != 0xCF))
#define AM_IMAGE_MAGIC_CUST(x)   (((x) == AM_IMAGE_MAGIC_INFO0)         ||  \
                                  ((x) == AM_IMAGE_MAGIC_SECURE)        ||  \
                                  ((x) == AM_IMAGE_MAGIC_NONSECURE)     ||  \
                                  ((x) == AM_IMAGE_MAGIC_DOWNLOAD)      ||  \
                                  ((x) == AM_IMAGE_MAGIC_KEYREVOKE)     ||  \
                                  ((x) == AM_IMAGE_MAGIC_OEM_CHAIN)     ||  \
                                  ((x) == AM_IMAGE_MAGIC_CONTAINER))

// OTA Upgrade related definitions
#define AM_HAL_SECURE_OTA_OTA_LIST_END_MARKER           0xFFFFFFFF

// OTA Protocol between OTA application and SecureBoot
// OTAPOINTER will be initialized as follows:
// Most significant 30 bits will correspond to most significant 30 bits of OTA Descriptor
// Least Significant bit (bit 0) should be initialized to 1 to indicate a valid OTA Descriptor
// bit 1 should be initialized to 1 to indicate that the list contains an SBL OTA
// OTA Descriptor points to a list of entries, each corresponding to an OTA blob, list terminating in 0xFFFFFFFF
// Each list entry word comprises of following:
// Most significant 30 bits will correspond to most significant 30 bits of OTA blob pointer
// Least Significant 2 bits should be initialized to 1 to indicate a valid OTA Pending
// After Secboot processes an OTA, it clears the least significant bit (bit 0)
// bit 1 indicates the status of the OTA - 0 for Success, 1 for Failure

// Store the b1 to store the return status - pass/fail
// b0 is used to indicate pending/done
#define AM_HAL_OTA_VALID_MASK                0x3
// Use bit 27, 29, 30, 31 to store the return status, if failure
#define AM_HAL_OTA_STATUS_MASK               0xE8000000
#define AM_HAL_OTA_GET_BLOB_PTR(ptr)         (((uint32_t)(ptr) & ~(AM_HAL_OTA_VALID_MASK | AM_HAL_OTA_STATUS_MASK)))
#define AM_HAL_OTA_IS_VALID(ptr)             (((uint32_t)(ptr) & AM_HAL_OTA_VALID_MASK) == AM_HAL_OTA_VALID_MASK)
#define AM_HAL_OTA_IS_IN_PROGRESS(ptr)       (((uint32_t)(ptr) & AM_HAL_OTA_VALID_MASK) == AM_HAL_OTA_DONE_STATUS_IN_PROGRESS)

#define AM_HAL_OTA_DONE_STATUS_MASK            0x3
#define AM_HAL_OTA_DONE_STATUS_IN_PROGRESS     0x1
#define AM_HAL_OTA_DONE_STATUS_FAILURE         0x2
#define AM_HAL_OTA_DONE_STATUS_SUCCESS         0x0

// This batch is used as bitmask - more than one can be set - Used only for OTAPOINTER register status
#define AM_HAL_OTADESC_STATUS_MASK      0xE8000001
typedef enum
{
    AM_HAL_OTADESC_STATUS_SUCCESS           =          0x00000000,
    AM_HAL_OTADESC_STATUS_PENDING           =          0x00000001,
    AM_HAL_OTADESC_STATUS_INVALID_OTAPTR    = (int32_t)0x80000000,
    AM_HAL_OTADESC_STATUS_INVALID_OTADESC   =          0x40000000,
    AM_HAL_OTADESC_STATUS_MAX_OTA_EXCEED    =          0x20000000,
} am_hal_otadesc_status_e;

// OTA Status
typedef enum
{
    //
    // Note: Enums that use bit31 must be treated as signed decimal values in
    //       order to avoid compiler warnings. We'll cast those as int32_t.
    //
    AM_HAL_OTA_STATUS_SUCCESS               =          0x00000000,
    AM_HAL_OTA_STATUS_IN_PROGRESS           =          0x00000001,
    AM_HAL_OTA_STATUS_FAILURE               =          0x00000002,
    AM_HAL_OTA_STATUS_PENDING               =          0x00000003,
    // Remaining values are used as numbers - only bits 27, 29, 30, 31 can be used - Along with setting the FAILURE indication as value 2 in bits 0,1
    AM_HAL_OTA_STATUS_INVALID_OWNER         = (int32_t)0x80000002,
    AM_HAL_OTA_STATUS_AUTH_POLICY           =          0x40000002,
    AM_HAL_OTA_STATUS_ENC_POLICY            = (int32_t)0xC0000002,
    AM_HAL_OTA_STATUS_CRC                   =          0x20000002,
    AM_HAL_OTA_STATUS_AUTH                  = (int32_t)0xA0000002,
    AM_HAL_OTA_STATUS_ENC                   = (int32_t)0xE0000002,
    AM_HAL_OTA_STATUS_CC                    =          0x60000002,
    AM_HAL_OTA_STATUS_MAGIC                 = (int32_t)0x88000002,
    AM_HAL_OTA_STATUS_RECURSE               =          0x48000002,
    AM_HAL_OTA_STATUS_ERR                   = (int32_t)0xC8000002,
    AM_HAL_OTA_STATUS_INVALID_IMAGE         = (int32_t)0xE8000002,
    AM_HAL_OTA_STATUS_FAIL                  =          0x68000002,      // Only for Patch
    AM_HAL_OTA_STATUS_VALIDATION            =          0x08000002,
    AM_HAL_OTA_STATUS_INVALID_OPERATION     =          0x28000002,      // Only for Wired
    AM_HAL_OTA_STATUS_INTERRUPTED           = (int32_t)0xA8000002,
} am_hal_ota_status_e;

// Per Image OTA Status information
typedef struct
{
    uint32_t            *pImage;
    am_hal_ota_status_e status;
} am_hal_ota_status_t;

// 3 stage Certificate Chain
typedef struct
{
    void       *pRootCert;
    void       *pKeyCert;
    void       *pContentCert;
} am_hal_certchain_t;

#ifdef __cplusplus
extern "C"
{
#endif

// pOtaDesc should be start of OTA Descriptor
// It will also initialize the OTAPOINTER to point to this descriptor, with LSB indicating it as invalid
uint32_t am_hal_ota_init(uint32_t ui32ProgramKey, am_hal_otadesc_t *pOtaDesc);

// Add a new OTA to descriptor
// This will program the next available entry in OTA descriptor
// Will also set the valid/sbl flags in OTA pointer register
uint32_t am_hal_ota_add(uint32_t ui32ProgamKey, uint8_t imageMagic, uint32_t *pImage);

// Get OTA Status
// Can be called anytime (generally after coming back from reset to check the status of OTA
// Will be also used by sbl_main to identify list of OTA's left for it (would show up as PENDING)
//*****************************************************************************
//
//! @brief  Get Current OTA Descriptor state
//!
//! @param  maxOta Determines the size of the following buffer
//! @param  pStatus - Return Parameter - populated by this function indicating the OTA
//! status of various OTA's
//! @param  pOtaDescStatus - Return Parameter - populated by this function indicating the overall
//! OTA descriptor processin status
//!
//! This will retrieve the current OTA status of various images added to the OTA descr
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success
//
//*****************************************************************************
uint32_t am_hal_get_ota_status(uint32_t maxOta, am_hal_ota_status_t *pStatus, uint32_t *pOtaDescStatus);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_SECURE_OTA_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

