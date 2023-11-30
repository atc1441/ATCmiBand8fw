/****************************************************************************
* The confidential and proprietary information contained in this file may    *
* only be used by a person authorised under and to the extent permitted      *
* by a subsisting licensing agreement from ARM Limited or its affiliates.    *
* 	(C) COPYRIGHT [2001-2017] ARM Limited or its affiliates.	     		 *
*	    ALL RIGHTS RESERVED						    						 *
* This entire notice must be reproduced on all copies of this file           *
* and copies of this file may only be made by a person if such person is     *
* permitted to do so under the terms of a subsisting license agreement       *
* from ARM Limited or its affiliates.					    				 *
*****************************************************************************/

#ifndef __DX_REG_BASE_HOST_H__
#define __DX_REG_BASE_HOST_H__

/* Identify platform: ARM MPS2 PLUS */
#define DX_PLAT_MPS2_PLUS 1

#define DX_BASE_CC 0x50088000
#define DX_BASE_CODE 0x1E000000

#define DX_BASE_ENV_REGS 0x400A8000
#define DX_BASE_ENV_NVM_LOW 0x400AA000
#define DX_BASE_ENV_NVM_HI  0x400AB000
#define DX_BASE_ENV_PERF_RAM 0x400A9000

#define DX_BASE_HOST_RGF 0x0UL
#define DX_BASE_CRY_KERNEL     0x0UL

#define DX_BASE_RNG 0x0000UL

#endif /*__DX_REG_BASE_HOST_H__*/
