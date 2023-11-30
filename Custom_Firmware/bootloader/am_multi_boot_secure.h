//*****************************************************************************
//
//! @file am_multi_boot_secure.h
//!
//! @brief Secure Bootloader Definitions
//! This file declares the customizable secure boot function hooks
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

#ifndef AM_MULTI_BOOT_SECURE_H
#define AM_MULTI_BOOT_SECURE_H

#include "am_mcu_apollo.h"
#include "am_bootloader.h"

//*****************************************************************************
//
//! @brief This func verifies the security trailer & initializes security params
//!
//! @param length - Length of the security trailer
//! @param pData - Pointer to the security trailer
//! @param bStoreInSram - -Indicates if the image can be accumulated in SRAM as
//! a whole. If not set, multiboot would need to keep flashing the image
//! segments as they arrive overwriting the existing content, even before the
//! image could be verified.
//! @param psImage - Pointer to the image properties as operated upon by multiboot
//! @param pProtect - Used to pass information back to multiboot, if the flashed
//! image needs any protection features. This is a place holder for future.
//!
//! This func verifies the security trailer & initializes security engine
//! required for decryption. It could also be used to verify the validity of the
//! key used for encryption.
//!
//! @return 0 for success, non-zero for failure.
//
//*****************************************************************************
int
init_multiboot_secure(uint32_t length, uint32_t *pData,
                      bool bStoreInSram, am_bootloader_image_t *pImage,
                      uint32_t *pProtect);

//*****************************************************************************
//
//! @brief This func performs in-place decryption of the supplied block of data
//!
//! @param pData - Pointer to encrypted image data in SRAM
//! @param ui32NumBytes - Length of data
//!
//! 
//! This function should implement an in-place decryption of the data using the
//! selected security algorithm. The decryption engine should have been
//! initialized as part of init_multiboot_secure implementation.
//! This function should also compute the running CRC for the clear image.
//! This function could also be used to implement any other verification on the
//! image - based on prior knowledge of image structure.
//!
//! @return none.
//
//*****************************************************************************
void
multiboot_secure_decrypt(void *pData, uint32_t ui32NumBytes);

// Authenticate/Validate the image
// Return Clear CRC of the image
// for validation on subsequent boots
//*****************************************************************************
//
//! @brief This func performs final verification of the downloaded image
//!
//! @param ui32NumBytes - Pointer used to return the CRC of the clear image
//!
//! 
//! This function could implement additional verification or authentication of
//! the downloaded (and decrypted) image. On successful verification, and if the
//! image did not need copy-protection in flash, it returns the clear image in
//! flash, so that on subsequent boots the bootloader can check the image for
//! integrity.
//!
//! @return It returns 0 if the verification succeeds, non-zero for failure
//
//*****************************************************************************
int
multiboot_secure_verify(uint32_t *pui32ClearCRC);

#endif // AM_MULTI_BOOT_SECURE_H

