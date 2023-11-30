//*****************************************************************************
//
//! @file am_devices_spiflash.c
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

#include "am_mcu_apollo.h"
#include "am_devices_spiflash.h"

//*****************************************************************************
//
//! Global variables.
//
//*****************************************************************************
am_devices_spiflash_t *g_psIOMSettings;

//*****************************************************************************
//
// Initialize the spiflash driver.
//
//*****************************************************************************
void
am_devices_spiflash_init(am_devices_spiflash_t *psIOMSettings)
{
    //
    // Initialize the IOM settings for the spiflash.
    //
    g_psIOMSettings = psIOMSettings;
}

//*****************************************************************************
//
// Reads the current status of the external flash
//
//*****************************************************************************
uint8_t
am_devices_spiflash_status(void)
{
    am_hal_iom_buffer(1) psResponse;

    //
    // Send the command and read the response.
    //
    am_hal_iom_spi_read(g_psIOMSettings->ui32IOMModule,
                        g_psIOMSettings->ui32ChipSelect,
                        psResponse.words, 1,
                        AM_HAL_IOM_OFFSET(AM_DEVICES_SPIFLASH_RDRSR));

    //
    // Return the status read from the external flash.
    //
    return psResponse.bytes[0];
}

//*****************************************************************************
//
// Reads the ID register for the external flash
//
//*****************************************************************************
uint32_t
am_devices_spiflash_id(void)
{
    am_hal_iom_buffer(3) psResponse;

    //
    // Send a command to read the ID register in the external flash.
    //
    am_hal_iom_spi_read(g_psIOMSettings->ui32IOMModule,
                        g_psIOMSettings->ui32ChipSelect,
                        psResponse.words, 3,
                        AM_HAL_IOM_OFFSET(AM_DEVICES_SPIFLASH_RDID));

    //
    // Return the ID
    //
    return psResponse.words[0] & 0x00FFFFFF;
}

//*****************************************************************************
//
// Reads the contents of the external flash into a buffer.
//
//*****************************************************************************
void
am_devices_spiflash_read(uint8_t *pui8RxBuffer, uint32_t ui32ReadAddress,
                         uint32_t ui32NumBytes)
{
    uint32_t i, ui32BytesRemaining, ui32TransferSize, ui32CurrentReadAddress;
    uint8_t *pui8Dest;

    uint32_t pui32WriteBuffer[1];
    uint8_t *pui8WritePtr;

    uint32_t pui32ReadBuffer[16];
    uint8_t *pui8ReadPtr;

    pui8WritePtr = (uint8_t *)(&pui32WriteBuffer);
    pui8ReadPtr = (uint8_t *)(&pui32ReadBuffer);

    //
    // Set the total number of bytes,and the starting transfer destination.
    //
    ui32BytesRemaining = ui32NumBytes;
    pui8Dest = pui8RxBuffer;
    ui32CurrentReadAddress = ui32ReadAddress;

    while ( ui32BytesRemaining )
    {
        //
        // Set the transfer size to either 64, or the number of remaining
        // bytes, whichever is smaller.
        //
        ui32TransferSize = ui32BytesRemaining > 64 ? 64 : ui32BytesRemaining;

        pui8WritePtr[0] = AM_DEVICES_SPIFLASH_READ;
        pui8WritePtr[1] = (ui32CurrentReadAddress & 0x00FF0000) >> 16;
        pui8WritePtr[2] = (ui32CurrentReadAddress & 0x0000FF00) >> 8;
        pui8WritePtr[3] = ui32CurrentReadAddress & 0x000000FF;

        //
        // Send the read command.
        //
        am_hal_iom_spi_write(g_psIOMSettings->ui32IOMModule,
                             g_psIOMSettings->ui32ChipSelect,
                             pui32WriteBuffer, 4,
                             AM_HAL_IOM_CS_LOW | AM_HAL_IOM_RAW);

        am_hal_iom_spi_read(g_psIOMSettings->ui32IOMModule,
                            g_psIOMSettings->ui32ChipSelect, pui32ReadBuffer,
                            ui32TransferSize, AM_HAL_IOM_RAW);

        //
        // Copy the received bytes over to the RxBuffer
        //
        for ( i = 0; i < ui32TransferSize; i++ )
        {
            pui8Dest[i] = pui8ReadPtr[i];
        }

        //
        // Update the number of bytes remaining and the destination.
        //
        ui32BytesRemaining -= ui32TransferSize;
        pui8Dest += ui32TransferSize;
        ui32CurrentReadAddress += ui32TransferSize;
    }
}

//*****************************************************************************
//
// Programs the given range of flash addresses.
//
//*****************************************************************************
void
am_devices_spiflash_write(uint8_t *pui8TxBuffer, uint32_t ui32WriteAddress,
                          uint32_t ui32NumBytes)
{
    uint32_t i;
    uint32_t ui32DestAddress;
    uint32_t ui32BytesRemaining;
    uint32_t ui32TransferSize;
    uint8_t *pui8Source;

    am_hal_iom_buffer(1) psEnableCommand;
    am_hal_iom_buffer(64) psWriteCommand;

    //
    // Prepare the command for write-enable.
    //
    psEnableCommand.bytes[0] = AM_DEVICES_SPIFLASH_WREN;

    //
    // Set the total number of bytes, and the starting transfer destination.
    //
    ui32BytesRemaining = ui32NumBytes;
    pui8Source = pui8TxBuffer;
    ui32DestAddress = ui32WriteAddress;

    while ( ui32BytesRemaining )
    {
        //
        // Set up a write command to hit the beginning of the next "chunk" of
        // flash.
        //
        psWriteCommand.bytes[0] = AM_DEVICES_SPIFLASH_PP;
        psWriteCommand.bytes[1] = (ui32DestAddress & 0x00FF0000) >> 16;
        psWriteCommand.bytes[2] = (ui32DestAddress & 0x0000FF00) >> 8;
        psWriteCommand.bytes[3] = ui32DestAddress & 0x000000FF;

        //
        // Set the transfer size to either 32, or the number of remaining
        // bytes, whichever is smaller.
        //
        ui32TransferSize = ui32BytesRemaining > 32 ? 32 : ui32BytesRemaining;

        //
        // Fill the rest of the command buffer with the data that we actually
        // want to write to flash.
        //
        for ( i = 0; i < ui32TransferSize; i++ )
        {
            psWriteCommand.bytes[4 + i] = pui8Source[i];
        }

        //
        // Send the write-enable command to prepare the external flash for
        // program operations, and wait for the write-enable latch to be set in
        // the status register.
        //
        am_hal_iom_spi_write(g_psIOMSettings->ui32IOMModule,
                             g_psIOMSettings->ui32ChipSelect,
                             psEnableCommand.words,
                             1, AM_HAL_IOM_RAW);

        while ( !(am_devices_spiflash_status() & AM_DEVICES_SPIFLASH_WEL) );

        //
        // Send the write command.
        //
        am_hal_iom_spi_write(g_psIOMSettings->ui32IOMModule,
                             g_psIOMSettings->ui32ChipSelect,
                             psWriteCommand.words,
                             (ui32TransferSize + 4), AM_HAL_IOM_RAW);

        //
        // Wait for status to indicate that the write is no longer in progress.
        //
        while ( am_devices_spiflash_status() & AM_DEVICES_SPIFLASH_WIP );

        //
        // Update the number of bytes remaining, as well as the source and
        // destination pointers
        //
        ui32BytesRemaining -= ui32TransferSize;
        pui8Source += ui32TransferSize;
        ui32DestAddress += ui32TransferSize;
    }

    return;
}

//*****************************************************************************
//
// Erases the entire contents of the external flash
//
//*****************************************************************************
void
am_devices_spiflash_mass_erase(void)
{
    am_hal_iom_buffer(1) psCommand;

    //
    // Send the write-enable command to prepare the external flash for program
    // operations.
    //
    psCommand.bytes[0] = AM_DEVICES_SPIFLASH_WREN;
    am_hal_iom_spi_write(g_psIOMSettings->ui32IOMModule,
                         g_psIOMSettings->ui32ChipSelect,
                         psCommand.words, 1, AM_HAL_IOM_RAW);

    //
    // Wait for the write enable latch status bit.
    //
    while ( !(am_devices_spiflash_status() & AM_DEVICES_SPIFLASH_WEL) );

    //
    // Send the bulk erase command.
    //
    psCommand.bytes[0] = AM_DEVICES_SPIFLASH_BE;
    am_hal_iom_spi_write(g_psIOMSettings->ui32IOMModule,
                         g_psIOMSettings->ui32ChipSelect,
                         psCommand.words, 1, AM_HAL_IOM_RAW);

    //
    // Wait for status to indicate that the write is no longer in progress.
    //
    while ( am_devices_spiflash_status() & AM_DEVICES_SPIFLASH_WIP );

    return;
}

//*****************************************************************************
//
// Erases the contents of a single sector of flash
//
//*****************************************************************************
void
am_devices_spiflash_sector_erase(uint32_t ui32SectorAddress)
{
    am_hal_iom_buffer(4) psCommand;

    //
    // Send the write-enable command to prepare the external flash for program
    // operations.
    //
    psCommand.bytes[0] = AM_DEVICES_SPIFLASH_WREN;
    am_hal_iom_spi_write(g_psIOMSettings->ui32IOMModule,
                         g_psIOMSettings->ui32ChipSelect,
                         psCommand.words, 1, AM_HAL_IOM_RAW);

    //
    // Wait for the write enable latch status bit.
    //
    while ( !(am_devices_spiflash_status() & AM_DEVICES_SPIFLASH_WEL) );

    //
    // Prepare the sector erase command, followed by the three-byte external
    // flash address.
    //
    psCommand.bytes[0] = AM_DEVICES_SPIFLASH_SE;
    psCommand.bytes[1] = (ui32SectorAddress & 0x00FF0000) >> 16;
    psCommand.bytes[2] = (ui32SectorAddress & 0x0000FF00) >> 8;
    psCommand.bytes[3] = ui32SectorAddress & 0x000000FF;

    //
    // Send the command to erase the desired sector.
    //
    am_hal_iom_spi_write(g_psIOMSettings->ui32IOMModule,
                         g_psIOMSettings->ui32ChipSelect,
                         psCommand.words, 4, AM_HAL_IOM_RAW);

    //
    // Wait for status to indicate that the write is no longer in progress.
    //
    while ( am_devices_spiflash_status() & AM_DEVICES_SPIFLASH_WIP );

    return;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

