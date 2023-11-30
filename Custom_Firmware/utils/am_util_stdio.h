//*****************************************************************************
//
//! @file am_util_stdio.h
//!
//! @brief A few printf-style functions for use with Ambiq products
//!
//! Functions for performing printf-style operations without dynamic memory
//! allocation.
//!
//! For further information about this module concerning its history, uses,
//! and limitations, please see the Ambiq Micro KB article "Q&A: What does
//! the AmbiqSuite SDK am_util_stdio_printf() function do?" at:
//!
//! https://support.ambiqmicro.com/hc/en-us/articles/360040441631
//!
//! @addtogroup stdio STDIO - Ambiq's Implementation
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
#ifndef AM_UTIL_STDIO_H
#define AM_UTIL_STDIO_H

/* get va_list from compiler. */
#include <stdarg.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif
//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************

// buffer size for printf
#ifndef AM_PRINTF_BUFSIZE
#define AM_PRINTF_BUFSIZE       1024   // Global printf buffer size
#endif

typedef void (*am_util_stdio_print_char_t)(char *pcStr);

//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************
//*****************************************************************************
//
//! @brief Sets the interface for printf calls.
//!
//! @param pfnCharPrint - Function pointer to be used to print to interface
//!
//! This function initializes the global print function which is used for
//! printf. This allows users to define their own printf interface and pass it
//! in as a am_util_stdio_print_char_t type.
//
//*****************************************************************************
extern void am_util_stdio_printf_init(am_util_stdio_print_char_t pfnCharPrint);

//*****************************************************************************
//
//! @brief Converts strings to 32-bit unsigned integers.
//!
//! @param *str - Pointer to the string to convert
//! @param **endptr - strtoul will set this to the char after the converted num
//! @param base - Base of the number as written in the input string.
//!
//! This function was implemented in a way similar to the strtoul function
//! found in the C standard library. It will attempt to extract a numerical
//! value from the input string, and return it to the caller. Invalid input
//! strings will return a value of zero.
//!
//! @return uint32_t representing the numerical value from the string.
//
//*****************************************************************************
extern uint32_t am_util_stdio_strtoul(const char *str, char **endptr, int base);

//*****************************************************************************
//
//! @brief Text mode translates linefeed (\n) characters to carriage return/
//! linefeed (CR/LF) combinations in printf() and sprintf() functions.
//!
//! @param bSetTextTranslationMode - true: Do LF to CR/LF translation.
//! false: Don't do the text mode translation.
//!
//! This function causes the printf() and sprintf() functions to translate
//! newline characters (\\n) into CR/LF (\\r\\n) combinations.
//!
//! @return Previous mode.
//
//*****************************************************************************
extern bool am_util_stdio_textmode_set(bool bSetTextTranslationMode);

//******************************************************************************
//
//! @brief Format data into string. (va_list implementation)
//!
//! @param *pcBuf - Pointer to the buffer to store the string
//! @param *pcFmt - Pointer to formatter string
//! @param *pArgs - Pointer to extended arguments
//!
//! A lite version of vsprintf().
//!      Currently handles the following specifiers:
//!      %c
//!      %s
//!      %[0][width]d (also %i)
//!      %[0][width]u
//!      %[0][width]x
//!      %[.precision]f
//!
//!     Note than any unrecognized or unhandled format specifier character is
//!     simply printed.  For example, "%%" will print a '%' character.
//!
//! @return uint32_t representing the number of characters printed.
//
//******************************************************************************
extern uint32_t am_util_stdio_vsprintf(char *pcBuf, const char *pcFmt, va_list pArgs);

//******************************************************************************
//
//! @brief Format data into string.
//!
//! @param *pcBuf - Pointer to the buffer to store the string
//! @param *pcFmt - Pointer to formater string
//!
//! A lite version of vsprintf().
//!      Currently handles the following specifiers:
//!      %c
//!      %s
//!      %[0][width]d (also %i)
//!      %[0][width]u
//!      %[0][width]x
//!
//!     Note than any unrecognized or unhandled format specifier character is
//!     simply printed.  For example, "%%" will print a '%' character.
//!
//! @return uint32_t representing the number of characters printed.
//
//******************************************************************************
extern uint32_t am_util_stdio_sprintf(char *pcBuf, const char *pcFmt, ...);

//*****************************************************************************
//
//! @brief A lite version of printf()
//!
//! @param *pcFmt - Pointer to formatter string
//!
//!  See am_util_stdio_sprintf() for more details.
//!
//! @return uint32_t representing the number of characters printed.
//
// *****************************************************************************
extern uint32_t am_util_stdio_printf(const char *pcFmt, ...);

//******************************************************************************
//
//! @brief Format data into string. (va_list implementation)
//!
//! @param *pcBuf - Pointer to the buffer to store the string
//! @param n - number of arguments
//! @param *pcFmt - Pointer to formatter string
//! @param *pArgs - Pointer to extended arguments
//!
//! A lite version of vsprintf().
//!      Currently handles the following specifiers:
//!      %c
//!      %s
//!      %[0][width]d (also %i)
//!      %[0][width]u
//!      %[0][width]x
//!      %[.precision]f
//!
//!     Note than any unrecognized or unhandled format specifier character is
//!     simply printed.  For example, "%%" will print a '%' character.
//!
//! @return uint32_t representing the number of characters printed.
//
//******************************************************************************
extern uint32_t am_util_stdio_vsnprintf(char *pcBuf, uint32_t n, const char *pcFmt,
                        va_list pArgs);

//******************************************************************************
//
//! @brief Format data into string.
//!
//! @param *pcBuf - Pointer to the buffer to store the string
//! @param n - number of arguments
//! @param *pcFmt - Pointer to formater string
//!
//! A lite version of vsprintf().
//!      Currently handles the following specifiers:
//!      %c
//!      %s
//!      %[0][width]d (also %i)
//!      %[0][width]u
//!      %[0][width]x
//!
//!     Note than any unrecognized or unhandled format specifier character is
//!     simply printed.  For example, "%%" will print a '%' character.
//!
//! @return uint32_t representing the number of characters printed.
//
//******************************************************************************
extern uint32_t am_util_stdio_snprintf(char *pcBuf, uint32_t n, const char *pcFmt, ...);

//******************************************************************************
//
//! @brief Format data into string. (va_list implementation)
//!
//! @param *pcFmt - Pointer to formatter string
//! @param *pArgs - Pointer to extended arguments
//!
//! A lite version of vsprintf().
//!      Currently handles the following specifiers:
//!      %c
//!      %s
//!      %[0][width]d (also %i)
//!      %[0][width]u
//!      %[0][width]x
//!      %[.precision]f
//!
//!     Note than any unrecognized or unhandled format specifier character is
//!     simply printed.  For example, "%%" will print a '%' character.
//!
//! @return uint32_t representing the number of characters printed.
//
//******************************************************************************
extern uint32_t am_util_stdio_vprintf(const char *pcFmt, va_list pArgs);

//*****************************************************************************
//
//! @brief Clear the terminal screen
//!
//! This function clears a standard terminal screen.
//
//*****************************************************************************
extern void am_util_stdio_terminal_clear(void);

#ifdef __cplusplus
}
#endif

#endif // AM_UTIL_STDIO_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

