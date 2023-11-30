//*****************************************************************************
//
//! @file am_util_string.h
//!
//! @brief A subset of the functions provided in the C standard string library.
//!
//! The functions here are reimplementation of some of the standard "string"
//! functions.
//!
//! @addtogroup string String - Ambiq subset of C String Library
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
#ifndef AM_UTIL_STRING_H
#define AM_UTIL_STRING_H

#ifdef __cplusplus
extern "C"
{
#endif


//*****************************************************************************
//
// Character attributes lookup table defines.
//
//*****************************************************************************
#define AM_CATTR_NONE       0x00
#define AM_CATTR_ALPHA      0x01
#define AM_CATTR_LOWER      0x02
#define AM_CATTR_UPPER      0x04
#define AM_CATTR_DIGIT      0x08
#define AM_CATTR_XDIGIT     0x10
#define AM_CATTR_WHSPACE    0x20
#define AM_CATTR_FILENM83   0x80

//
// Set MINIMIZE_CATTR_TABLE to 1 to configure for minimal CATTR table size,
//  (256 instead of 512 bytes) but at a cost of slightly larger code size.
//  However, setting this option also provides an additional level of checking
//  of the argument; if the argument is not a uint8_t, the functions are
//  guaranteed to return 0.
//
#define MINIMIZE_CATTR_TABLE    0


//*****************************************************************************
//
// Globals
//
//*****************************************************************************
extern const uint8_t am_cattr[];


//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************

//*****************************************************************************
//
//! @brief Compare two strings.
//!
//! @param str1 is the first string to compare.
//! @param str2 is the second string to compare
//!
//! This function steps through a pair of strings, character by character, to
//! determine if the strings contain the same characters. If the strings match,
//! this function will return a zero. If str1 is alphabetically earlier than
//! str2, the return value will be negative. Otherwise, the return value will
//! be positive.
//!
//! @return 0 for a perfect match, negative value if str1<str2, positive value
//!  if str1>str2.
//
//*****************************************************************************
extern int32_t am_util_string_strcmp(const char *str1, const char *str2);

//*****************************************************************************
//
//! @brief Compare two strings with case-insensitivity.
//!
//! @param str1 is the first string to compare.
//! @param str2 is the second string to compare
//!
//! This function compares each character in the 2 strings, converting all
//! alpha characters to lower-case to make the comparison.
//!
//! To illustrate a possible unexpected outcome due to comparing the strings
//! as lower case, consider the example strings "AMBIQ_MICRO" and "AMBIQMICRO".
//! For these strings, stricmp() will return a negative value (indicating the
//! first as before the second), whereas strcmp() will return a positive value.
//!
//! @return 0 for a case-insensitive match, negative value if str1<str2,
//!  positive value if str1>str2.
//
//*****************************************************************************
extern int32_t am_util_string_stricmp(const char *str1, const char *str2);

//*****************************************************************************
//
//! @brief Compare two strings with a specified count.
//!
//! @param str1 is the first string to compare.
//! @param str2 is the second string to compare
//! @param num is the maximum number of characters to compare.
//!
//! This function steps through a pair of strings, character by character, to
//! determine if the strings contain the same characters. If the strings match,
//! this function will return a zero. If str1 is alphabetically earlier than
//! str2, the return value will be negative. Otherwise, the return value will
//! be positive.
//!
//! @return 0 for a perfect match, negative value if str1<str2, positive value
//!  if str1>str2.
//
//*****************************************************************************
extern int32_t am_util_string_strncmp(const char *str1, const char *str2,
                                      uint32_t num);

//*****************************************************************************
//
//! @brief Compare two strings with a specified count and without regard to
//! letter case in the strings.
//!
//! @param str1 is the first string to compare.
//! @param str2 is the second string to compare
//! @param num is the maximum number of characters to compare.
//!
//! This function steps through a pair of strings, character by character, to
//! determine if the strings contain the same characters. If the strings match,
//! this function will return a zero. If str1 is alphabetically earlier than
//! str2, the return value will be negative. Otherwise, the return value will
//! be positive.
//!
//! @return 0 for a perfect match, negative value if str1<str2, positive value
//!  if str1>str2.
//
//*****************************************************************************
extern int32_t am_util_string_strnicmp(const char *str1, const char *str2,
                                      int num);

//*****************************************************************************
//
//! @brief Return the length of a string.
//!
//! @param pcStr pointer to the string.
//!
//! This function returns the length of the string at pcStr.
//!
//! @return length of the string pcStr.
//
//*****************************************************************************
extern uint32_t am_util_string_strlen(const char *pcStr);

//*****************************************************************************
//
//! @brief Copies a string.
//!
//! @param pcDst pointer to the destination string.
//! @param pcSrc pointer to the source string to be copied to pcDst.
//!
//! This function copies pcSrc to the location specified by pcDst.
//!
//! @return pcDst (the location of the destination string).
//
//*****************************************************************************
extern char *am_util_string_strcpy(char *pcDst, const char *pcSrc);

//*****************************************************************************
//
//! @brief Copies a specified number of characters of a string.
//!
//! @param pcDst pointer to the destination string.
//! @param pcSrc pointer to the source string to be copied to pcDst.
//! @param uNum  length of string
//!
//! This function copies uNum characters of pcSrc to the location specified
//!  by pcDst.
//! If uNum is less than the length of pcSrc, a NULL terminating character
//!  is not appended to the copied string. Thus the resultant string will be
//!  exactly uNum chars in length and not terminated.
//! If uNum is greater than the length of pcSrc, then pcDst is padded with
//!  NULL characters up to the uNum length.
//! Behavior is undefined if the addresses ranges overlap.
//!
//! @return pcDst (the location of the destination string).
//
//*****************************************************************************
extern char *am_util_string_strncpy(char *pcDst, const char *pcSrc, uint32_t uNum);

//*****************************************************************************
//
//! @brief Concatenate a string.
//!
//! @param pcDst pointer to the destination string.
//! @param pcSrc pointer to the source string to be copied to pcDst.
//!
//! This function concatenates the string at pcSrc to the existing string
//! at pcDst.
//!
//! Both strings, pcDst and pcSrc, must be NULL-terminated.
//! No overflow checking is performed.
//! pcDst and pcSrc shall not overlap.
//!
//! @return pcDst (the location of the destination string).
//
//*****************************************************************************
extern char *am_util_string_strcat(char *pcDst, const char *pcSrc);

//*****************************************************************************
//
// Character "is" macros and functions
//
// By default all of the "is" functions are implemented as macros.  To implement
//  as functions rather than macros, use a global compiler command line (-D)
//  option to define AM_UTIL_STRING_CTYPE_DISABLE_MACROS.
//
#ifdef AM_UTIL_STRING_CTYPE_DISABLE_MACROS
//*****************************************************************************
//
//! @brief Check if the integer param is alphanumeric
//!
//! Tests a given integer value in order to determine
//!  whether the integer satisfies the test condition.
//! This function is based on the C99 standard function.
//!
//! By default this function is implemented as a macro. To implement
//!  as a function, use a global compiler command line (-D)
//!  option to define AM_UTIL_STRING_CTYPE_DISABLE_MACROS.
//!
//! @param c - integer to test
//!
//! @return returns a nonzero value if the integer satisfies
//!  the test condition and 0 if it does not.
//
//*****************************************************************************
extern int am_util_string_isalnum(int c);

//*****************************************************************************
//
//! @brief Check if the integer param is alphabetic
//!
//! Tests a given integer value in order to determine
//!  whether the integer satisfies the test condition.
//! This function is based on the C99 standard function.
//!
//! By default this function is implemented as a macro. To implement
//!  as a function, use a global compiler command line (-D)
//!  option to define AM_UTIL_STRING_CTYPE_DISABLE_MACROS.
//!
//! @param c - integer to test
//!
//! @return returns a nonzero value if the integer satisfies
//!  the test condition and 0 if it does not.
//
//*****************************************************************************
extern int am_util_string_isalpha(int c);

//*****************************************************************************
//
//! @brief Check if the integer param is a digit
//!
//! Tests a given integer value in order to determine
//!  whether the integer satisfies the test condition.
//! This function is based on the C99 standard function.
//!
//! By default this function is implemented as a macro. To implement
//!  as a function, use a global compiler command line (-D)
//!  option to define AM_UTIL_STRING_CTYPE_DISABLE_MACROS.
//!
//! @param c - integer to test
//!
//! @return returns a nonzero value if the integer satisfies
//!  the test condition and 0 if it does not.
//
//*****************************************************************************
extern int am_util_string_isdigit(int c);

//*****************************************************************************
//
//! @brief Check if the integer param is lower case
//!
//! Tests a given integer value in order to determine
//!  whether the integer satisfies the test condition.
//! This function is based on the C99 standard function.
//!
//! By default this function is implemented as a macro. To implement
//!  as a function, use a global compiler command line (-D)
//!  option to define AM_UTIL_STRING_CTYPE_DISABLE_MACROS.
//!
//! @param c - integer to test
//!
//! @return returns a nonzero value if the integer satisfies
//!  the test condition and 0 if it does not.
//
//*****************************************************************************
extern int am_util_string_islower(int c);

//*****************************************************************************
//
//! @brief Check if the integer param is the space character
//!
//! Tests a given integer value in order to determine
//!  whether the integer satisfies the test condition.
//! This function is based on the C99 standard function.
//!
//! By default this function is implemented as a macro. To implement
//!  as a function, use a global compiler command line (-D)
//!  option to define AM_UTIL_STRING_CTYPE_DISABLE_MACROS.
//!
//! @param c - integer to test
//!
//! @return returns a nonzero value if the integer satisfies
//!  the test condition and 0 if it does not.
//
//*****************************************************************************
extern int am_util_string_isspace(int c);

//*****************************************************************************
//
//! @brief Check if the integer param is upper case
//!
//! Tests a given integer value in order to determine
//!  whether the integer satisfies the test condition.
//! This function is based on the C99 standard function.
//!
//! By default this function is implemented as a macro. To implement
//!  as a function, use a global compiler command line (-D)
//!  option to define AM_UTIL_STRING_CTYPE_DISABLE_MACROS.
//!
//! @param c - integer to test
//!
//! @return returns a nonzero value if the integer satisfies
//!  the test condition and 0 if it does not.
//
//*****************************************************************************
extern int am_util_string_isupper(int c);

//*****************************************************************************
//
//! @brief Check if the integer param is a hex digit
//!
//! Tests a given integer value in order to determine
//!  whether the integer satisfies the test condition.
//! This function is based on the C99 standard function.
//!
//! By default this function is implemented as a macro. To implement
//!  as a function, use a global compiler command line (-D)
//!  option to define AM_UTIL_STRING_CTYPE_DISABLE_MACROS.
//!
//! @param c - integer to test
//!
//! @return returns a nonzero value if the integer satisfies
//!  the test condition and 0 if it does not.
//
//*****************************************************************************
extern int am_util_string_isxdigit(int c);

//*****************************************************************************
//
//! @brief Converts the character to lower case
//!
//! Tests a given integer value in order to determine
//!  whether the integer satisfies the test condition.
//! This function is based on the C99 standard function.
//!
//! By default this function is implemented as a macro. To implement
//!  as a function, use a global compiler command line (-D)
//!  option to define AM_UTIL_STRING_CTYPE_DISABLE_MACROS.
//!
//! @param c - integer to test
//!
//! @return returns the input param as lower case
//
//*****************************************************************************
extern int am_util_string_tolower(int c);

//*****************************************************************************
//
//! @brief Converts the character to upper case
//!
//! Tests a given integer value in order to determine
//!  whether the integer satisfies the test condition.
//! This function is based on the C99 standard function.
//!
//! By default this function is implemented as a macro. To implement
//!  as a function, use a global compiler command line (-D)
//!  option to define AM_UTIL_STRING_CTYPE_DISABLE_MACROS.
//!
//! @param c - integer to test
//!
//! @return returns the input param as upper case
//
//*****************************************************************************
extern int am_util_string_toupper(int c);

//*****************************************************************************
//
//! @brief Check if the integer param is part of a filename char
//!
//! Tests a given integer value in order to determine
//!  whether the integer satisfies the test condition.
//! This function is based on the C99 standard function.
//!
//! By default this function is implemented as a macro. To implement
//!  as a function, use a global compiler command line (-D)
//!  option to define AM_UTIL_STRING_CTYPE_DISABLE_MACROS.
//!
//! @param c - integer to test
//!
//! @return returns a nonzero value if the integer satisfies
//!  the test condition and 0 if it does not.
//
//*****************************************************************************
extern int am_util_string_isfilenm83(int c);
#else
#if MINIMIZE_CATTR_TABLE
#define am_util_string_isalnum(c)       ((c & 0xffffff80) ? 0 : (am_cattr[c] & (AM_CATTR_ALPHA | AM_CATTR_DIGIT)) ? 1 : 0)
#define am_util_string_isalpha(c)       ((c & 0xffffff80) ? 0 : (am_cattr[c] & AM_CATTR_ALPHA) ? 1 : 0)
#define am_util_string_isdigit(c)       ((c & 0xffffff80) ? 0 : (am_cattr[c] & AM_CATTR_DIGIT) ? 1 : 0)
#define am_util_string_islower(c)       ((c & 0xffffff80) ? 0 : (am_cattr[c] & AM_CATTR_LOWER) ? 1 : 0)
#define am_util_string_isspace(c)       ((c & 0xffffff80) ? 0 : (am_cattr[c] & AM_CATTR_WHSPACE) ? 1 : 0)
#define am_util_string_isupper(c)       ((c & 0xffffff80) ? 0 : (am_cattr[c] & AM_CATTR_UPPER) ? 1 : 0)
#define am_util_string_isxdigit(c)      ((c & 0xffffff80) ? 0 : (am_cattr[c] & AM_CATTR_XDIGIT) ? 1 : 0)
#define am_util_string_tolower(c)       ((am_cattr[c & 0x7f] & AM_CATTR_UPPER) ? c | 0x20 : c)
#define am_util_string_toupper(c)       ((am_cattr[c & 0x7f] & AM_CATTR_LOWER) ? c & ~0x20 : c)
#else
#define am_util_string_isalnum(c)       (am_cattr[c & 0xff] & (AM_CATTR_ALPHA | AM_CATTR_DIGIT))
#define am_util_string_isalpha(c)       (am_cattr[c & 0xff] & AM_CATTR_ALPHA)
#define am_util_string_isdigit(c)       (am_cattr[c & 0xff] & AM_CATTR_DIGIT)
#define am_util_string_islower(c)       (am_cattr[c & 0xff] & AM_CATTR_LOWER)
#define am_util_string_isspace(c)       (am_cattr[c & 0xff] & AM_CATTR_WHSPACE)
#define am_util_string_isupper(c)       (am_cattr[c & 0xff] & AM_CATTR_UPPER)
#define am_util_string_isxdigit(c)      (am_cattr[c & 0xff] & AM_CATTR_XDIGIT)
#define am_util_string_tolower(c)       ((am_cattr[c & 0xff] & AM_CATTR_UPPER) ? c | 0x20 : c)
#define am_util_string_toupper(c)       ((am_cattr[c & 0xff] & AM_CATTR_LOWER) ? c & ~0x20 : c)
#endif // MINIMIZE_CATTR_TABLE

#define am_util_string_isfilenm83(c)    (am_cattr[c & 0xff] & AM_CATTR_FILENM83)
#endif // AM_UTIL_STRING_CTYPE_DISABLE_MACROS

#ifdef __cplusplus
}
#endif

#endif // AM_UTIL_STRING_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

