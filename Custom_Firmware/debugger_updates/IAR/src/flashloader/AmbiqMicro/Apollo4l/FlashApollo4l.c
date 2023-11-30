//*****************************************************************************
//
//! @file FlashApollo4p.c
//!
//! @brief The flash loader for IAR Embedded Workbench for Apollo4 Plus.
//!
//! This modules contains the functions necessary for programming applications
//! to the Apollo4 non-volatile memory.
//
//*****************************************************************************

//*****************************************************************************
//
// ${copyright}
//
// This is part of revision ${version} of the AmbiqSuite Development Package.
//
//*****************************************************************************

//*****************************************************************************
//
// Includes
//
//*****************************************************************************
#include "flash_loader.h"
#include "flash_loader_extra.h"


//*****************************************************************************
//
// From am_reg_base_addresses.h
//
//*****************************************************************************
//
// SRAM address space
//
#define SRAM_BASEADDR                           (0x10000000UL)

//
// Flash address space
//
#define MRAM_BASEADDR                           (0x00000000UL)


//*****************************************************************************
//
// Ambiq Micro includes
//
//*****************************************************************************
#include "am_hal_mram.h"
#include "am_hal_bootrom_helper.h"


//*****************************************************************************
//
// Local defines
//
//*****************************************************************************
#define NV_WIPE     0       // nv_program_main2() Program_nWipe parameter
#define NV_PROGRAM  1       //  "

//*****************************************************************************
// Select SBL or nbl
//*****************************************************************************
#define AM_DEV_SBL  1
#if AM_DEV_SBL  // SBL is the normal case
#define AM_STARTADDR    (AM_HAL_MRAM_ADDR + 0x00018000)
#define AM_NVSIZE       (AM_HAL_MRAM_TOTAL_SIZE - AM_STARTADDR)
#else           // NBL
#define AM_STARTADDR    0x00000000
#define AM_NVSIZE       AM_HAL_MRAM_TOTAL_SIZE
#endif


//*****************************************************************************
//
// Prototypes
//
//*****************************************************************************


//*****************************************************************************
//
// Globals
//
//*****************************************************************************

const am_hal_bootrom_helper_t g_am_hal_bootrom_helper =
{
         ((int  (*)(uint32_t, uint32_t))                                    0x0800004D),    // nv_mass_erase
         ((int  (*)(uint32_t, uint32_t, uint32_t))                          0x08000051),    // nv_page_erase
         ((int  (*)(uint32_t, uint32_t *, uint32_t *, uint32_t))            0x08000055),    // nv_program_main
         ((int  (*)(uint32_t, uint32_t *, uint32_t, uint32_t))              0x08000059),    // nv_program_info_area
         ((int  (*)(uint32_t, uint32_t, uint32_t, uint32_t, uint32_t))      0x0800006D),    // nv_program_main2
         ((uint32_t (*)(uint32_t *))                                        0x08000075),    // br_util_read_word
         ((void (*)( uint32_t *, uint32_t))                                 0x08000079),    // br_util_write_word
         ((int  (*)( uint32_t))                                             0x08000081),    // nv_info_erase
         ((int  (*)( uint32_t ))                                            0x08000099),    // nv_recovery
         ((void (*)(uint32_t ))                                             0x0800009D),    // br_util_delay_cycles
};



//*****************************************************************************
//
// FlashInit()
//
//*****************************************************************************
#if USE_ARGC_ARGV
uint32_t FlashInit(void *base_of_flash, uint32_t image_size,
                   uint32_t link_address, uint32_t flags,
                   int argc, char const *argv[])
#else
uint32_t FlashInit(void *base_of_flash, uint32_t image_size,
                   uint32_t link_address, uint32_t flags)
#endif
{
    uint32_t ui32Retval;

    if ( flags & FLAG_ERASE_ONLY )
    {
        ui32Retval = g_am_hal_bootrom_helper.nv_program_main2(
                        AM_HAL_MRAM_PROGRAM_KEY,
                        NV_WIPE,            // Fill
                        0xFFFFFFFF,         // Fill value
                        AM_STARTADDR / 4,   // Word offset
                        AM_NVSIZE / 4);     // Number of words

        return ui32Retval ? RESULT_ERROR : RESULT_ERASE_DONE;
    }

    return RESULT_OK;
} // FlashInit()


//*****************************************************************************
//
// FlashWrite()
//
//*****************************************************************************
uint32_t FlashWrite(void *block_start,
                    uint32_t offset_into_block,
                    uint32_t count,
                    char const *buffer)
{
    uint32_t ui32Retval;
    uint32_t ui32Addr = (uint32_t)block_start + offset_into_block;

    //
    // Both source and destination addresses must be word-aligned.
    // Note: We may want to also check for ui32Addr of 0xC000 or larger.
    //
    if ( ((uint32_t)buffer & 0x3) || (ui32Addr & 0x3) )
    {
        return RESULT_ERROR;
    }

    ui32Retval = g_am_hal_bootrom_helper.nv_program_main2(
                    AM_HAL_MRAM_PROGRAM_KEY,
                    NV_PROGRAM,             // Program MRAM
                    (uint32_t)buffer,       // Byte address to begin programming
                    ui32Addr / 4,           // Word offset
                    count / 4);             // Number of words

//  return ui32Retval ? RESULT_ERROR : RESULT_OK;
    return ui32Retval;

} // FlashWrite()


//*****************************************************************************
//
// FlashErase()
//
//*****************************************************************************
uint32_t FlashErase(void *block_start,
                    uint32_t block_size)
{
    uint32_t ui32Addr = (uint32_t)block_start;
    uint32_t ui32Retval;

    if ( ui32Addr & 0x3 )
    {
        return RESULT_ERROR;
    }

#if AM_DEV_SBL != 0
    //
    // Check that the starting address is outside the SBL region.
    //
    if ( (ui32Addr < AM_STARTADDR) || (ui32Addr & 0x3) )
    {
        return RESULT_ERROR;
    }
#endif

    ui32Retval = g_am_hal_bootrom_helper.nv_program_main2(
                    AM_HAL_MRAM_PROGRAM_KEY,
                    NV_WIPE,                // Fill
                    0xFFFFFFFF,             // Fill value
                    ui32Addr / 4,           // Word offset
                    block_size / 4);        // Number of words

//  return ui32Retval ? RESULT_ERROR : RESULT_OK;
    return ui32Retval;

} // FlashErase()
