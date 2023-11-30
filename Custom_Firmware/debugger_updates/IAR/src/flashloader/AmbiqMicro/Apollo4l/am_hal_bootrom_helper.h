//*****************************************************************************
//
//  am_hal_bootrom_helper.h
//! @file
//!
//! @brief definition of BootROM helper function table
//!
//! @ingroup apollo4hal
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// ${copyright}
//
// This is part of revision ${version} of the AmbiqSuite Development Package.
//
//*****************************************************************************
#ifndef AM_HAL_BOOTROM_HELPER_H
#define AM_HAL_BOOTROM_HELPER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>


//*****************************************************************************
//
//! Structure of pointers to helper functions invoking flash operations.
//
//! The functions we are pointing to here are in the Apollo 4
//! integrated BOOTROM.
//
//*****************************************************************************
typedef struct am_hal_bootrom_helper_struct
{
    //
    // Basics functions required by most toolchains.
    //
    int  (*nv_mass_erase)(uint32_t, uint32_t);
    int  (*nv_page_erase)(uint32_t, uint32_t, uint32_t);
    int  (*nv_program_main)(uint32_t, uint32_t *, uint32_t *, uint32_t);

    //
    // Infospace programming function.
    //
    int  (*nv_program_info_area)(uint32_t, uint32_t *, uint32_t, uint32_t);

    //
    // Helpful utilities.
    //
    int  (*nv_program_main2)(uint32_t, uint32_t, uint32_t, uint32_t, uint32_t);

    uint32_t (*bootrom_util_read_word)( uint32_t *);
    void (*bootrom_util_write_word)( uint32_t *, uint32_t);

    //
    // Infospace erase functions.
    //
    int  (*nv_info_erase)( uint32_t);

    //
    // Non-Volatile Recovery function.
    //
    int  (*nv_recovery)( uint32_t value);

    //
    // Cycle accurate delay function.
    //
    void (*bootrom_delay_cycles)(uint32_t ui32Cycles);

} am_hal_bootrom_helper_t;

extern const am_hal_bootrom_helper_t g_am_hal_bootrom_helper;

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_BOOTROM_HELPER_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
