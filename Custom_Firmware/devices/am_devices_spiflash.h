//*****************************************************************************
//
//! @file am_devices_spiflash.h
//!
//! @brief Generic spiflash driver.
//!
//! @addtogroup spiflash SPI FLASH Driver
//! @ingroup devices
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

#ifndef AM_DEVICES_SPIFLASH_H
#define AM_DEVICES_SPIFLASH_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! @name Global definitions for flash commands
//! @{
//
//*****************************************************************************
#define AM_DEVICES_SPIFLASH_WREN        0x06        // Write enable
#define AM_DEVICES_SPIFLASH_WRDI        0x04        // Write disable
#define AM_DEVICES_SPIFLASH_RDID        0x9E        // Read Identification
#define AM_DEVICES_SPIFLASH_RDRSR       0x05        // Read status register
#define AM_DEVICES_SPIFLASH_WRSR        0x01        // Write status register
#define AM_DEVICES_SPIFLASH_READ        0x03        // Read data bytes
#define AM_DEVICES_SPIFLASH_PP          0x02        // Page program
#define AM_DEVICES_SPIFLASH_SE          0xD8        // Sector Erase
#define AM_DEVICES_SPIFLASH_BE          0xC7        // Bulk Erase
//! @}

//*****************************************************************************
//
//! @name Global definitions for the flash status register
//! @{
//
//*****************************************************************************
#define AM_DEVICES_SPIFLASH_WEL         0x02        // Write enable latch
#define AM_DEVICES_SPIFLASH_WIP         0x01        // Write in progress
//! @}

//*****************************************************************************
//
//! @name Global definitions for the flash size information
//! @{
//
//*****************************************************************************
#define AM_DEVICES_SPIFLASH_PAGE_SIZE       0x100    //256 bytes, minimum program unit
#define AM_DEVICES_SPIFLASH_SUBSECTOR_SIZE  0x1000   //4096 bytes
#define AM_DEVICES_SPIFLASH_SECTOR_SIZE     0x10000  //65536 bytes
//! @}

//*****************************************************************************
//
//! @name Function pointers for SPI write and read.
//! @{
//
//*****************************************************************************
typedef bool (*am_devices_spiflash_write_t)(uint32_t ui32Module,
                                            uint32_t ui32ChipSelect,
                                            uint32_t *pui32Data,
                                            uint32_t ui32NumBytes,
                                            uint32_t ui32Options);

typedef bool (*am_devices_spiflash_read_t)(uint32_t ui32Module,
                                           uint32_t ui32ChipSelect,
                                           uint32_t *pui32Data,
                                           uint32_t ui32NumBytes,
                                           uint32_t ui32Options);
//! @}

//*****************************************************************************
//
//! Device structure used for communication.
//
//*****************************************************************************
typedef struct
{
    //
    //! Module number to use for IOM access.
    //
    uint32_t ui32IOMModule;

    //
    //! Chip Select number to use for IOM access.
    //
    uint32_t ui32ChipSelect;
}
am_devices_spiflash_t;

//*****************************************************************************
//
// External function definitions.
//
//*****************************************************************************
//*****************************************************************************
//
//! @brief Initialize the spiflash driver.
//!
//! @param psIOMSettings - IOM device structure describing the target spiflash.
//!
//! This function should be called before any other am_devices_spiflash
//! functions. It is used to set tell the other functions how to communicate
//! with the external spiflash hardware.
//!
//! The \e pfnWriteFunc and \e pfnReadFunc variables may be used to provide
//! alternate implementations of SPI write and read functions respectively. If
//! they are left set to 0, the default functions am_hal_iom_spi_write() and
//! am_hal_iom_spi_read() will be used.
//
//*****************************************************************************
extern void am_devices_spiflash_init(am_devices_spiflash_t *psIOMSettings);

//*****************************************************************************
//
//! @brief Reads the current status of the external flash
//!
//! This function reads the status register of the external flash, and returns
//! the result as an 8-bit unsigned integer value. The processor will block
//! during the data transfer process, but will return as soon as the status
//! register had been read.
//!
//! Macro definitions for interpreting the contents of the status register are
//! included in the header file.
//!
//! @return 8-bit status register contents
//
//*****************************************************************************
extern uint8_t am_devices_spiflash_status(void);

//*****************************************************************************
//
//! @brief Reads the ID register for the external flash
//!
//! This function reads the ID register of the external flash, and returns the
//! result as a 32-bit unsigned integer value. The processor will block during
//! the data transfer process, but will return as soon as the ID register had
//! been read. The ID contents for this flash only contains 24 bits of data, so
//! the result will be stored in the lower 24 bits of the return value.
//!
//! @return 32-bit ID register contents
//
//*****************************************************************************
extern uint32_t am_devices_spiflash_id(void);

//*****************************************************************************
//
//! @brief Reads the contents of the external flash into a buffer.
//!
//! @param pui8RxBuffer - Buffer to store the received data from the flash
//! @param ui32ReadAddress - Address of desired data in external flash
//! @param ui32NumBytes - Number of bytes to read from external flash
//!
//! This function reads the external flash at the provided address and stores
//! the received data into the provided buffer location. This function will
//! only store ui32NumBytes worth of data.
//
//*****************************************************************************
extern void am_devices_spiflash_read(uint8_t *pui8RxBuffer,
                                     uint32_t ui32ReadAddress,
                                     uint32_t ui32NumBytes);

//*****************************************************************************
//
//! @brief Programs the given range of flash addresses.
//!
//! @param pui8TxBuffer - Buffer to write the external flash data from
//! @param ui32WriteAddress - Address to write to in the external flash
//! @param ui32NumBytes - Number of bytes to write to the external flash
//!
//! This function uses the data in the provided pui8TxBuffer and copies it to
//! the external flash at the address given by ui32WriteAddress. It will copy
//! exactly ui32NumBytes of data from the original pui8TxBuffer pointer. The
//! user is responsible for ensuring that they do not overflow the target flash
//! memory or underflow the pui8TxBuffer array
//
//*****************************************************************************
extern void am_devices_spiflash_write(uint8_t *pui8TxBuffer,
                                      uint32_t ui32WriteAddress,
                                      uint32_t ui32NumBytes);

//*****************************************************************************
//
//! @brief Erases the entire contents of the external flash
//!
//! This function uses the "Bulk Erase" instruction to erase the entire
//! contents of the external flash.
//
//*****************************************************************************
extern void am_devices_spiflash_mass_erase(void);

//*****************************************************************************
//
//! @brief Erases the contents of a single sector of flash
//!
//! @param ui32SectorAddress - Address to erase in the external flash
//!
//! This function erases a single sector of the external flash as specified by
//! ui32EraseAddress. The entire sector where ui32EraseAddress will
//! be erased.
//
//*****************************************************************************
extern void am_devices_spiflash_sector_erase(uint32_t ui32SectorAddress);

#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_SPIFLASH_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

