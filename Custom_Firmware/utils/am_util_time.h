//*****************************************************************************
//
//! @file am_util_time.h
//!
//! @brief Functions useful for RTC, calendar, time, etc. computations.
//!
//! @addtogroup time Time - RTC Time Computations
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
#ifndef AM_UTIL_TIME_H
#define AM_UTIL_TIME_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! @brief Compute the day of the week given the month, day, and year.
//!
//! @param iYear  - The year of the desired date (e.g. 2016).
//! @param iMonth - The month of the desired date (1-12).
//! @param iDay   - The day of the month of the desired date (1-31).
//!
//! This function is general in nature, but is designed to be used with the RTC.
//!
//! @returns An index value indicating the day of the week.
//! 0-6 indicate  Sun, Mon, Tue, Wed, Thu, Fri, Sat, respectively.
//! 7   indicates that the given date is invalid (e.g. 2/29/2015).
//
//*****************************************************************************
extern int am_util_time_computeDayofWeek(int iYear, int iMonth, int iDay);

#ifdef __cplusplus
}
#endif

#endif // AM_UTIL_TIME_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

