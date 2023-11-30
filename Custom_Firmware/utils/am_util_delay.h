//*****************************************************************************
//
//! @file am_util_delay.h
//!
//! @brief A few useful delay functions.
//!
//! Functions for fixed delays.
//!
//! @addtogroup delay Delay Functionality
//! @ingroup utils
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
#ifndef AM_UTIL_DELAY_H
#define AM_UTIL_DELAY_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************

//*****************************************************************************
//
//! @brief Delays for a desired amount of loops.
//!
//! This function will delay for a number of cycle loops.
//!
//! @note - the number of cycles each loops takes to execute is approximately 3.
//! Therefore the actual number of cycles executed will be ~3x ui32Iterations.
//!
//! For example, a ui32Iterations value of 100 will delay for 300 cycles.
//!
//! @param ui32Iterations - Desired number of cycle loops to delay for.
//
//*****************************************************************************
extern void am_util_delay_cycles(uint32_t ui32Iterations);

//*****************************************************************************
//
//! @brief Delays for a desired amount of milliseconds.
//!
//! This function will delay for a number of milliseconds.
//!
//! @param ui32MilliSeconds - number of milliseconds to delay for.
//
//*****************************************************************************
extern void am_util_delay_ms(uint32_t ui32MilliSeconds);

//*****************************************************************************
//
//! @brief Delays for a desired amount of microseconds.
//!
//! This function will delay for a number of microseconds.
//!
//! @param ui32MicroSeconds - number of microseconds to delay for.
//
//*****************************************************************************
extern void am_util_delay_us(uint32_t ui32MicroSeconds);

#ifdef __cplusplus
}
#endif

#endif // AM_UTIL_DELAY_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

