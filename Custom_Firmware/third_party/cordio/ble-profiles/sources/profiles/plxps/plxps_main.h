/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Pulse Oximeter profile sensor internal interfaces.
 *
 *  Copyright (c) 2012-2018 Arm Ltd. All Rights Reserved.
 *
 *  Copyright (c) 2019 Packetcraft, Inc.
 *  
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *  
 *      http://www.apache.org/licenses/LICENSE-2.0
 *  
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
/*************************************************************************************************/
#ifndef PLXPS_MAIN_H
#define PLXPS_MAIN_H

#include "app_hw.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! \addtogroup PULSE_OXIMETER_PROFILE
 *  \{ */

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! \brief Minimum RACP write length */
#define PLXPS_RACP_MIN_WRITE_LEN       2

/*! \brief RACP response length */
#define PLXPS_RACP_RSP_LEN             4

/*! \brief Pulse Oximeter RACP number of stored records response length */
#define PLXPS_RACP_NUM_REC_RSP_LEN     4

/*! \brief RACP operand maximum length */
#define PLXPS_OPERAND_MAX              ((CH_RACP_GLS_FILTER_TIME_LEN * 2) + 1)

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/*! \brief Pulse Oximeter measurement record */
typedef struct
{
  plxpScm_t   spotCheck;              /*!< \brief Pulse Oximeter spot check measurement */
} plxpsRec_t;


/*************************************************************************************************/
/*!
 *  \brief  Initialize the pulse oximeter record database.
 *
 *  \return None.
 */
/*************************************************************************************************/
void plxpsDbInit(void);

/*************************************************************************************************/
/*!
 *  \brief  Get the next record that matches the given filter parameters that follows
 *          the given current record.
 *
 *  \param  oper        Operator.
 *  \param  pCurrRec    Pointer to current record.
 *  \param  pRec        Return pointer to next record, if found.
 *
 *  \return \ref CH_RACP_RSP_SUCCESS if a record is found, otherwise an error status is returned.
 */
/*************************************************************************************************/
uint8_t plxpsDbGetNextRecord(uint8_t oper, plxpsRec_t *pCurrRec,  plxpsRec_t **pRec);

/*************************************************************************************************/
/*!
 *  \brief  Delete records that match the given filter parameters.
 *
 *  \param  oper        Operator.
 *
 *  \return \ref CH_RACP_RSP_SUCCESS if records deleted, otherwise an error status is returned.
 */
/*************************************************************************************************/
uint8_t plxpsDbDeleteRecords(uint8_t oper);

/*************************************************************************************************/
/*!
 *  \brief  Get the number of records matching the filter parameters.
 *
 *  \param  oper        Operator.
 *  \param  pNumRec     Returns number of records which match filter parameters.

 *
 *  \return RACP status.
 */
/*************************************************************************************************/
uint8_t plxpsDbGetNumRecords(uint8_t oper, uint8_t *pNumRec);

/*************************************************************************************************/
/*!
*  \brief  Generate a new record.
*
*  \return None.
*/
/*************************************************************************************************/
void plxpsDbGenerateRecord(void);


/*! \} */    /* PULSE_OXIMETER_PROFILE */

#ifdef __cplusplus
};
#endif

#endif /* PLXPS_MAIN_H */
