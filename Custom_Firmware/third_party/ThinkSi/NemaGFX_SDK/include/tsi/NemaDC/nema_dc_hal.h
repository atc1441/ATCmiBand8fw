/*******************************************************************************
 * Copyright (c) 2022 Think Silicon S.A.
 *
   Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this header file and/or associated documentation files to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies of the
 * Materials, and to permit persons to whom the Materials are furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Materials.
 *
 * MODIFICATIONS TO THIS FILE MAY MEAN IT NO LONGER ACCURATELY REFLECTS
 * NEMAGFX API. THE UNMODIFIED, NORMATIVE VERSIONS OF THINK-SILICON NEMAGFX
 * SPECIFICATIONS AND HEADER INFORMATION ARE LOCATED AT:
 *   https://think-silicon.com/products/software/nemagfx-api
 *
 *  The software is provided 'as is', without warranty of any kind, express or
 *  implied, including but not limited to the warranties of merchantability,
 *  fitness for a particular purpose and noninfringement. In no event shall
 *  Think Silicon S.A. be liable for any claim, damages or other liability, whether
 *  in an action of contract, tort or otherwise, arising from, out of or in
 *  connection with the software or the use or other dealings in the software.
 ******************************************************************************/


#ifndef NEMADC_HAL_H__
#define NEMADC_HAL_H__

#include "nema_sys_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/** \brief Initialize system. Implementor defined. Called in nemadc_init()
 *
 * \return 0 if no errors occurred
 * \see nema_init()
 *
 */
int32_t nemadc_sys_init(void);


/** \brief Wait for VSYNC
 *
 *
 */
void nemadc_wait_vsync(void);

/** \brief Read Hardware register
 *
 * \param reg Register to read
 * \return Value read from the register
 * \see nema_reg_write
 *
 */
uint32_t nemadc_reg_read(uint32_t reg);

/** \brief Write Hardware Register
 *
 * \param reg Register to write
 * \param value Value to be written
 * \see nema_reg_read()
 *
 */
void nemadc_reg_write(uint32_t reg, uint32_t value);

#ifdef __cplusplus
}
#endif

#endif
