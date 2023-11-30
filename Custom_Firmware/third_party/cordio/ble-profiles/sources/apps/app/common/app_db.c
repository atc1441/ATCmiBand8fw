/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Application framework device database example, using simple RAM-based storage.
 *
 *  Copyright (c) 2011-2019 Arm Ltd. All Rights Reserved.
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

#include <string.h>
#include "am_mcu_apollo.h"
#include "wsf_types.h"
#include "wsf_assert.h"
#include "util/bda.h"
#include "app_api.h"
#include "app_main.h"
#include "app_db.h"
#include "app_cfg.h"

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/*! Database record */
typedef struct
{
  /*! Common for all roles */
  bdAddr_t     peerAddr;                      /*! Peer address */
  uint8_t      addrType;                      /*! Peer address type */
  dmSecIrk_t   peerIrk;                       /*! Peer IRK */
  dmSecCsrk_t  peerCsrk;                      /*! Peer CSRK */
  uint8_t      keyValidMask;                  /*! Valid keys in this record */
  bool_t       inUse;                         /*! TRUE if record in use */
  bool_t       valid;                         /*! TRUE if record is valid */
  bool_t       peerAddedToRl;                 /*! TRUE if peer device's been added to resolving list */
  bool_t       peerRpao;                      /*! TRUE if RPA Only attribute's present on peer device */

  /*! For slave local device */
  dmSecLtk_t   localLtk;                      /*! Local LTK */
  uint8_t      localLtkSecLevel;              /*! Local LTK security level */
  bool_t       peerAddrRes;                   /*! TRUE if address resolution's supported on peer device (master) */

  /*! For master local device */
  dmSecLtk_t   peerLtk;                       /*! Peer LTK */
  uint8_t      peerLtkSecLevel;               /*! Peer LTK security level */

  /*! for ATT server local device */
  uint16_t     cccTbl[APP_DB_NUM_CCCD];       /*! Client characteristic configuration descriptors */
  uint32_t     peerSignCounter;               /*! Peer Sign Counter */
  uint8_t      changeAwareState;              /*! Peer client awareness to state change in database */
  uint8_t      csf[ATT_CSF_LEN];              /*! Peer client supported features record */

  /*! for ATT client */
  bool_t       cacheByHash;                   /*! TRUE if cached handles are maintained by comparing database hash */
  uint8_t      dbHash[ATT_DATABASE_HASH_LEN]; /*! Peer database hash */
  uint16_t     hdlList[APP_DB_HDL_LIST_LEN];  /*! Cached handle list */
  uint8_t      discStatus;                    /*! Service discovery and configuration status */
  bool_t       master_role;                   /*! True if local device is master for this record */
#ifdef AM_VOS_SDK
  uint32_t     dummyReserved[3];              // Need to review!! (16 bytes align for flash write.)
#endif // AM_VOS_SDK
} appDbRec_t;

/*! Database type */
typedef struct
{
  appDbRec_t  rec[APP_DB_NUM_RECS];               /*! Device database records */
  char        devName[ATT_DEFAULT_PAYLOAD_LEN];   /*! Device name */
  uint8_t     devNameLen;                         /*! Device name length */
  uint8_t     dbHash[ATT_DATABASE_HASH_LEN];      /*! Device GATT database hash */
} appDb_t;

#define NVM_REC_BYTE_LEN  (256)
#define NVM_REC_WORD_LEN  (NVM_REC_BYTE_LEN/4)
/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! Database */
static appDb_t appDb;

/*! When all records are allocated use this index to determine which to overwrite */
static appDbRec_t *pAppDbNewRec = appDb.rec;


//
// Copy Record list from NVM into the active record list, if any
//
//
#ifdef AM_VOS_SDK
#if defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4P) || defined(AM_PART_APOLLO4L)
appDbRec_t * pRecListNvmPointer = (appDbRec_t *)0x000FE000;
#endif
#else
appDbRec_t * pRecListNvmPointer = (appDbRec_t *)0x00070000; //temporarily put the data here
#endif // AM_VOS_SDK

void AppLoadResList(void)
{
    appDbRec_t* p_nvm_rec = pAppDbNewRec;

    for(uint8_t i=0; i<APP_DB_NUM_RECS; i++)
    {
        void *pPeerAddr = p_nvm_rec->peerAddr;

        if ( (*(uint32_t*)pPeerAddr != 0xFFFFFFFF)
            && (*(uint32_t*)pPeerAddr != 0x00000000) )
        {
            DmPrivAddDevToResList(p_nvm_rec->peerIrk.addrType, p_nvm_rec->peerAddr, p_nvm_rec->peerIrk.key,
                          DmSecGetLocalIrk(), TRUE, 0);
        }

        p_nvm_rec++;
    }
}
void AppCopyRecListInNvm(appDbRec_t *pRecord)
{
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4L) || defined(AM_PART_APOLLO4P)
    uint32_t* uint32RecListPointer =  (uint32_t *)pRecListNvmPointer;
#else
    appDbRec_t* pNvmRec =  pRecListNvmPointer;
#endif
    uint8_t i;
    for(i=0;i<APP_DB_NUM_RECS;i++)
    {
    #if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4L) || defined(AM_PART_APOLLO4P)
        appDbRec_t* pNvmRec = (appDbRec_t*)uint32RecListPointer;
    #endif

        void *pPeerAddr = pNvmRec->peerAddr;
        if ( (*(uint32_t*)pPeerAddr != 0xFFFFFFFF)  &&
             (*(uint32_t*)pPeerAddr != 0x00000000) )
        {
            //valid record in NVM
            memcpy(pRecord, pNvmRec, sizeof(appDbRec_t));

            //pRecord->inUse = FALSE;
            //pRecord->valid = TRUE;
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4L) || defined(AM_PART_APOLLO4P)
            uint32RecListPointer += NVM_REC_WORD_LEN;
#else
            pNvmRec++;
#endif
            pRecord++;
        }
        else
        {
            break;
        }
    }
}
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4L) || defined(AM_PART_APOLLO4P)

// sizeof(appDbRec_t) = 176 bytes of buffer
uint8_t ui8DbRamBuffer[APP_DB_NUM_RECS * NVM_REC_BYTE_LEN];
uint16_t ui16DbRamBufferSize = APP_DB_NUM_RECS * NVM_REC_BYTE_LEN;
#else
uint8_t ui8DbRamBuffer[APP_DB_NUM_RECS * sizeof(appDbRec_t)];
uint16_t ui16DbRamBufferSize = APP_DB_NUM_RECS * sizeof(appDbRec_t);
#endif
static void updateRecordInNVM(uint32_t* pDest, uint32_t* pSrc, uint32_t* pFlashAddr)
{
    uint16_t ui16Size = sizeof(appDbRec_t);

    // read the flash out
    memcpy(ui8DbRamBuffer, pFlashAddr, ui16DbRamBufferSize);
    // update the record
    memcpy((uint8_t *)pDest, (uint8_t *)pSrc, ui16Size);

    uint32_t ui32Critical = am_hal_interrupt_master_disable();
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4L) || defined(AM_PART_APOLLO4P)
    am_hal_mram_main_program(AM_HAL_MRAM_PROGRAM_KEY,
                              (uint32_t *)ui8DbRamBuffer,
                              (uint32_t *)pFlashAddr,
                              ui16DbRamBufferSize/4);
#else
    // erase the page
    uint32_t ui32CurrentPage =  AM_HAL_FLASH_ADDR2PAGE((uint32_t)pFlashAddr);
    uint32_t ui32CurrentBlock = AM_HAL_FLASH_ADDR2INST((uint32_t)pFlashAddr);

    am_hal_flash_page_erase(AM_HAL_FLASH_PROGRAM_KEY,
                                ui32CurrentBlock, ui32CurrentPage);
    // program the data back
    am_hal_flash_program_main(AM_HAL_FLASH_PROGRAM_KEY,
                              (uint32_t *)ui8DbRamBuffer,
                              (uint32_t *)pFlashAddr,
                              ui16DbRamBufferSize%4?((ui16DbRamBufferSize/4) + 1):(ui16DbRamBufferSize/4));
#endif
    am_hal_interrupt_master_set(ui32Critical);
}

int32_t AppDbUpdateNVM(appDbHdl_t hdl)
{
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4L) || defined(AM_PART_APOLLO4P)
    uint32_t* pNvmRec =  (uint32_t *)pRecListNvmPointer;
    uint32_t * pRamBufRec = (uint32_t *)ui8DbRamBuffer;
#else
    appDbRec_t* pNvmRec =  pRecListNvmPointer;
    appDbRec_t* pRamBufRec = (appDbRec_t*)ui8DbRamBuffer;
#endif

    int i;


    for(i=0;i<APP_DB_NUM_RECS;i++)
    {
        void *pPeerAddr = ((appDbRec_t* )pNvmRec)->peerAddr;
        if ( (*(uint32_t*)pPeerAddr != 0xFFFFFFFF)  &&
             (*(uint32_t*)pPeerAddr != 0x00000000) )
        {
            if(BdaCmp(((appDbRec_t*)hdl)->peerAddr, ((appDbRec_t* )pNvmRec)->peerAddr))
            {
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4L) || defined(AM_PART_APOLLO4P)
                updateRecordInNVM((uint32_t*)&pRamBufRec[i*NVM_REC_BYTE_LEN], (uint32_t*)((appDbRec_t*)hdl)->peerAddr, (uint32_t*)pRecListNvmPointer);
#else
                updateRecordInNVM((uint32_t*)&pRamBufRec[i], (uint32_t*)((appDbRec_t*)hdl)->peerAddr, (uint32_t*)pRecListNvmPointer);
#endif
                return true;
            }

            //skip
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4L) || defined(AM_PART_APOLLO4P)
            pNvmRec += NVM_REC_WORD_LEN;
#else
            pNvmRec++;
#endif
        }
        else
        {
            uint32_t ui32Critical = am_hal_interrupt_master_disable();
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4L) || defined(AM_PART_APOLLO4P)
            am_hal_mram_main_program(AM_HAL_MRAM_PROGRAM_KEY,
                                      (uint32_t *)((appDbRec_t*)hdl)->peerAddr,
                                      (uint32_t *)pNvmRec,
                                      ui16DbRamBufferSize/4);
#else
            am_hal_flash_program_main(AM_HAL_FLASH_PROGRAM_KEY,
                                      (uint32_t *)((appDbRec_t*)hdl)->peerAddr,
                                      (uint32_t *)pNvmRec,
                                      (sizeof(appDbRec_t)%4?((sizeof(appDbRec_t)/4) + 1):(sizeof(appDbRec_t)/4)));
#endif
            am_hal_interrupt_master_set(ui32Critical);

            return true;
        }
    }

    // if we get here, the record NVM are full
    // function spec: erase all the record and record the current one
    // erase the page
    uint32_t ui32Critical = am_hal_interrupt_master_disable();
#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4L) || defined(AM_PART_APOLLO4P)
    am_hal_mram_main_program(AM_HAL_MRAM_PROGRAM_KEY,
                              (uint32_t *)((appDbRec_t*)hdl)->peerAddr,
                              (uint32_t *)pRecListNvmPointer,
                               ui16DbRamBufferSize/4);
#else
    uint32_t ui32CurrentPage =  AM_HAL_FLASH_ADDR2PAGE((uint32_t)pRecListNvmPointer);
    uint32_t ui32CurrentBlock = AM_HAL_FLASH_ADDR2INST((uint32_t)pRecListNvmPointer);

    am_hal_flash_page_erase(AM_HAL_FLASH_PROGRAM_KEY,
                                ui32CurrentBlock, ui32CurrentPage);
    // program the data back
    am_hal_flash_program_main(AM_HAL_FLASH_PROGRAM_KEY,
                              (uint32_t *)((appDbRec_t*)hdl)->peerAddr,
                              (uint32_t *)pRecListNvmPointer,
                              (sizeof(appDbRec_t)%4?((sizeof(appDbRec_t)/4) + 1):(sizeof(appDbRec_t)/4)));
#endif
    am_hal_interrupt_master_set(ui32Critical);

    return false;

}
/*************************************************************************************************/
/*!
 *  \brief  Initialize the device database.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppDbInit(void)
{
#ifdef AM_BLE_USE_NVM
    AppCopyRecListInNvm(pAppDbNewRec);
#endif
  return;
}

/*************************************************************************************************/
/*!
 *  \brief  Create a new device database record.
 *
 *  \param  addrType  Address type.
 *  \param  pAddr     Peer device address.
 *
 *  \return Database record handle.
 */
/*************************************************************************************************/
appDbHdl_t AppDbNewRecord(uint8_t addrType, uint8_t *pAddr, bool_t master_role)
{
  appDbRec_t  *pRec = appDb.rec;
  uint8_t     i;

  /* find a free record */
  for (i = APP_DB_NUM_RECS; i > 0; i--, pRec++)
  {
    if (!pRec->inUse)
    {
      break;
    }
  }

  /* if all records were allocated */
  if (i == 0)
  {
    /* overwrite a record */
    pRec = pAppDbNewRec;

    /* get next record to overwrite */
    pAppDbNewRec++;
    if (pAppDbNewRec == &appDb.rec[APP_DB_NUM_RECS])
    {
      pAppDbNewRec = appDb.rec;
    }
  }

  /* initialize record */
  memset(pRec, 0, sizeof(appDbRec_t));
  pRec->inUse = TRUE;
  pRec->addrType = addrType;
  BdaCpy(pRec->peerAddr, pAddr);
  pRec->peerAddedToRl = FALSE;
  pRec->peerRpao = FALSE;
  pRec->master_role = master_role;

  return (appDbHdl_t) pRec;
}

/*************************************************************************************************/
/*!
*  \brief  Get the next database record for a given record. For the first record, the function
*          should be called with 'hdl' set to 'APP_DB_HDL_NONE'.
*
*  \param  hdl  Database record handle.
*
*  \return Next record handle found. APP_DB_HDL_NONE, otherwise.
*/
/*************************************************************************************************/
appDbHdl_t AppDbGetNextRecord(appDbHdl_t hdl)
{
  appDbRec_t  *pRec;

  /* if first record is requested */
  if (hdl == APP_DB_HDL_NONE)
  {
    pRec = appDb.rec;
  }
  /* if valid record passed in */
  else if (AppDbRecordInUse(hdl))
  {
    pRec = (appDbRec_t *)hdl;
    pRec++;
  }
  /* invalid record passed in */
  else
  {
    return APP_DB_HDL_NONE;
  }

  /* look for next valid record */
  while (pRec < &appDb.rec[APP_DB_NUM_RECS])
  {
    /* if record is in use */
    if (pRec->inUse && pRec->valid)
    {
      /* record found */
      return (appDbHdl_t)pRec;
    }

    /* look for next record */
    pRec++;
  }

  /* end of records */
  return APP_DB_HDL_NONE;
}

/*************************************************************************************************/
/*!
 *  \brief  Delete a new device database record.
 *
 *  \param  hdl       Database record handle.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppDbDeleteRecord(appDbHdl_t hdl)
{
  ((appDbRec_t *) hdl)->inUse = FALSE;
}

/*************************************************************************************************/
/*!
 *  \brief  Validate a new device database record.  This function is called when pairing is
 *          successful and the devices are bonded.
 *
 *  \param  hdl       Database record handle.
 *  \param  keyMask   Bitmask of keys to validate.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppDbValidateRecord(appDbHdl_t hdl, uint8_t keyMask)
{
  ((appDbRec_t *) hdl)->valid = TRUE;
  ((appDbRec_t *) hdl)->keyValidMask = keyMask;

#ifdef AM_BLE_USE_NVM
  AppDbUpdateNVM(hdl);
#endif
}

/*************************************************************************************************/
/*!
 *  \brief  Check if a record has been validated.  If it has not, delete it.  This function
 *          is typically called when the connection is closed.
 *
 *  \param  hdl       Database record handle.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppDbCheckValidRecord(appDbHdl_t hdl)
{
  if (((appDbRec_t *) hdl)->valid == FALSE)
  {
    AppDbDeleteRecord(hdl);
  }
}

/*************************************************************************************************/
/*!
*  \brief  Check if a database record is in use.

*  \param  hdl       Database record handle.
*
*  \return TURE if record in use. FALSE, otherwise.
*/
/*************************************************************************************************/
bool_t AppDbRecordInUse(appDbHdl_t hdl)
{
  appDbRec_t  *pRec = appDb.rec;
  uint8_t     i;

  /* see if record is in database record list */
  for (i = APP_DB_NUM_RECS; i > 0; i--, pRec++)
  {
    if (pRec->inUse && pRec->valid && (pRec == ((appDbRec_t *)hdl)))
    {
      return TRUE;
    }
  }

  return FALSE;
}

/*************************************************************************************************/
/*!
 *  \brief  Check if there is a stored bond with any device.
 *
 *  \param  hdl       Database record handle.
 *
 *  \return TRUE if a bonded device is found, FALSE otherwise.
 */
/*************************************************************************************************/
bool_t AppDbCheckBonded(void)
{
  appDbRec_t  *pRec = appDb.rec;
  uint8_t     i;

  /* find a record */
  for (i = APP_DB_NUM_RECS; i > 0; i--, pRec++)
  {
    if (pRec->inUse && !pRec->master_role)
    {
      return TRUE;
    }
  }

  return FALSE;
}

/*************************************************************************************************/
/*!
 *  \brief  Delete all database records.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppDbDeleteAllRecords(void)
{
  appDbRec_t  *pRec = appDb.rec;
  uint8_t     i;

  /* set in use to false for all records */
  for (i = APP_DB_NUM_RECS; i > 0; i--, pRec++)
  {
    pRec->inUse = FALSE;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Find a device database record by peer address.
 *
 *  \param  addrType  Address type.
 *  \param  pAddr     Peer device address.
 *
 *  \return Database record handle or APP_DB_HDL_NONE if not found.
 */
/*************************************************************************************************/
appDbHdl_t AppDbFindByAddr(uint8_t addrType, uint8_t *pAddr)
{
  appDbRec_t  *pRec = appDb.rec;
  uint8_t     peerAddrType = DmHostAddrType(addrType);
  uint8_t     i;

  /* find matching record */
  for (i = APP_DB_NUM_RECS; i > 0; i--, pRec++)
  {
    if (pRec->inUse && (pRec->addrType == peerAddrType) && BdaCmp(pRec->peerAddr, pAddr))
    {
      return (appDbHdl_t) pRec;
    }
  }

  return APP_DB_HDL_NONE;
}

/*************************************************************************************************/
/*!
 *  \brief  Find a device database record by data in an LTK request.
 *
 *  \param  encDiversifier  Encryption diversifier associated with key.
 *  \param  pRandNum        Pointer to random number associated with key.
 *
 *  \return Database record handle or APP_DB_HDL_NONE if not found.
 */
/*************************************************************************************************/
appDbHdl_t AppDbFindByLtkReq(uint16_t encDiversifier, uint8_t *pRandNum)
{
  appDbRec_t  *pRec = appDb.rec;
  uint8_t     i;

  /* find matching record */
  for (i = APP_DB_NUM_RECS; i > 0; i--, pRec++)
  {
    if (pRec->inUse && (pRec->localLtk.ediv == encDiversifier) &&
        (memcmp(pRec->localLtk.rand, pRandNum, SMP_RAND8_LEN) == 0))
    {
      return (appDbHdl_t) pRec;
    }
  }

  return APP_DB_HDL_NONE;
}

/*************************************************************************************************/
/*!
 *  \brief  Get a key from a device database record.
 *
 *  \param  hdl       Database record handle.
 *  \param  type      Type of key to get.
 *  \param  pSecLevel If the key is valid, the security level of the key.
 *
 *  \return Pointer to key if key is valid or NULL if not valid.
 */
/*************************************************************************************************/
dmSecKey_t *AppDbGetKey(appDbHdl_t hdl, uint8_t type, uint8_t *pSecLevel)
{
  dmSecKey_t *pKey = NULL;

  /* if key valid */
  if ((type & ((appDbRec_t *) hdl)->keyValidMask) != 0)
  {
    switch(type)
    {
      case DM_KEY_LOCAL_LTK:
        *pSecLevel = ((appDbRec_t *) hdl)->localLtkSecLevel;
        pKey = (dmSecKey_t *) &((appDbRec_t *) hdl)->localLtk;
        break;

      case DM_KEY_PEER_LTK:
        *pSecLevel = ((appDbRec_t *) hdl)->peerLtkSecLevel;
        pKey = (dmSecKey_t *) &((appDbRec_t *) hdl)->peerLtk;
        break;

      case DM_KEY_IRK:
        pKey = (dmSecKey_t *)&((appDbRec_t *)hdl)->peerIrk;
        break;

      case DM_KEY_CSRK:
        pKey = (dmSecKey_t *)&((appDbRec_t *)hdl)->peerCsrk;
        break;

      default:
        break;
    }
  }

  return pKey;
}

/*************************************************************************************************/
/*!
 *  \brief  Set a key in a device database record.
 *
 *  \param  hdl       Database record handle.
 *  \param  pKey      Key data.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppDbSetKey(appDbHdl_t hdl, dmSecKeyIndEvt_t *pKey)
{
  switch(pKey->type)
  {
    case DM_KEY_LOCAL_LTK:
      ((appDbRec_t *) hdl)->localLtkSecLevel = pKey->secLevel;
      ((appDbRec_t *) hdl)->localLtk = pKey->keyData.ltk;
      break;

    case DM_KEY_PEER_LTK:
      ((appDbRec_t *) hdl)->peerLtkSecLevel = pKey->secLevel;
      ((appDbRec_t *) hdl)->peerLtk = pKey->keyData.ltk;
      break;

    case DM_KEY_IRK:
      ((appDbRec_t *)hdl)->peerIrk = pKey->keyData.irk;

      /* make sure peer record is stored using its identity address */
      ((appDbRec_t *)hdl)->addrType = pKey->keyData.irk.addrType;
      BdaCpy(((appDbRec_t *)hdl)->peerAddr, pKey->keyData.irk.bdAddr);
      break;

    case DM_KEY_CSRK:
      ((appDbRec_t *)hdl)->peerCsrk = pKey->keyData.csrk;

      /* sign counter must be initialized to zero when CSRK is generated */
      ((appDbRec_t *)hdl)->peerSignCounter = 0;
      break;

    default:
      break;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Get the peer's database hash.
 *
 *  \param  hdl       Database record handle.
 *
 *  \return Pointer to database hash.
 */
/*************************************************************************************************/
uint8_t *AppDbGetPeerDbHash(appDbHdl_t hdl)
{
  return ((appDbRec_t *) hdl)->dbHash;
}

/*************************************************************************************************/
/*!
 *  \brief  Set a new peer database hash.
 *
 *  \param  hdl       Database record handle.
 *  \param  pDbHash   Pointer to new hash.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppDbSetPeerDbHash(appDbHdl_t hdl, uint8_t *pDbHash)
{
  WSF_ASSERT(pDbHash != NULL);

  memcpy(((appDbRec_t *) hdl)->dbHash, pDbHash, ATT_DATABASE_HASH_LEN);

#ifdef AM_BLE_USE_NVM
  AppDbUpdateNVM(hdl);
#endif
}

/*************************************************************************************************/
/*!
 *  \brief  Check if cached handles' validity is determined by reading the peer's database hash
 *
 *  \param  hdl       Database record handle.
 *
 *  \return \ref TRUE if peer's database hash must be read to verify handles have not changed.
 */
/*************************************************************************************************/
bool_t AppDbIsCacheCheckedByHash(appDbHdl_t hdl)
{
  return ((appDbRec_t *) hdl)->cacheByHash;
}

/*************************************************************************************************/
/*!
 *  \brief  Set if cached handles' validity is determined by reading the peer's database hash.
 *
 *  \param  hdl           Database record handle.
 *  \param  cacheByHash   \ref TRUE if peer's database must be read to verify cached handles.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppDbSetCacheByHash(appDbHdl_t hdl, bool_t cacheByHash)
{
  ((appDbRec_t *) hdl)->cacheByHash = cacheByHash;

#ifdef AM_BLE_USE_NVM
  AppDbUpdateNVM(hdl);
#endif
}

/*************************************************************************************************/
/*!
 *  \brief  Get the client characteristic configuration descriptor table.
 *
 *  \param  hdl       Database record handle.
 *
 *  \return Pointer to client characteristic configuration descriptor table.
 */
/*************************************************************************************************/
uint16_t *AppDbGetCccTbl(appDbHdl_t hdl)
{
  return ((appDbRec_t *) hdl)->cccTbl;
}

/*************************************************************************************************/
/*!
 *  \brief  Set a value in the client characteristic configuration table.
 *
 *  \param  hdl       Database record handle.
 *  \param  idx       Table index.
 *  \param  value     client characteristic configuration value.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppDbSetCccTblValue(appDbHdl_t hdl, uint16_t idx, uint16_t value)
{
  WSF_ASSERT(idx < APP_DB_NUM_CCCD);

  ((appDbRec_t *) hdl)->cccTbl[idx] = value;

#ifdef AM_BLE_USE_NVM
  uint8_t connId = AppConnIsOpen();

  if(AppCheckBonded(connId))
  {
    AppDbUpdateNVM(hdl);
  }
#endif
}

/*************************************************************************************************/
/*!
 *  \brief  Get the client supported features record.
 *
 *  \param  hdl                Database record handle.
 *  \param  pChangeAwareState  Pointer to peer client's change aware status to a change in the database.
 *  \param  pCsf               Pointer to csf value pointer.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppDbGetCsfRecord(appDbHdl_t hdl, uint8_t *pChangeAwareState, uint8_t **pCsf)
{
  *pChangeAwareState = ((appDbRec_t *)hdl)->changeAwareState;
  *pCsf = ((appDbRec_t *) hdl)->csf;
}

/*************************************************************************************************/
/*!
 *  \brief  Set a client supported features record.
 *
 *  \param  hdl               Database record handle.
 *  \param  changeAwareState  The state of awareness to a change, see ::attClientAwareStates.
 *  \param  pCsf              Pointer to the client supported features value.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppDbSetCsfRecord(appDbHdl_t hdl, uint8_t changeAwareState, uint8_t *pCsf)
{
  if ((pCsf != NULL) && (hdl != APP_DB_HDL_NONE))
  {
    ((appDbRec_t *) hdl)->changeAwareState = changeAwareState;
    memcpy(&((appDbRec_t *) hdl)->csf, pCsf, ATT_CSF_LEN);

  }
}

/*************************************************************************************************/
/*!
 *  \brief  Set client's state of awareness to a change in the database.
 *
 *  \param  hdl        Database record handle. If \ref hdl == \ref NULL, state is set for all
 *                     clients.
 *  \param  state      The state of awareness to a change, see ::attClientAwareStates.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppDbSetClientChangeAwareState(appDbHdl_t hdl, uint8_t state)
{
  if (hdl == APP_DB_HDL_NONE)
  {
    appDbRec_t  *pRec = appDb.rec;
    uint8_t     i;

    /* Set all clients status to change-unaware. */
    for (i = APP_DB_NUM_RECS; i > 0; i--, pRec++)
    {
      pRec->changeAwareState = state;
    }
  }
  else
  {
    ((appDbRec_t *) hdl)->changeAwareState = state;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Get device's GATT database hash.
 *
 *  \return Pointer to database hash.
 */
/*************************************************************************************************/
uint8_t *AppDbGetDbHash(void)
{
  return appDb.dbHash;
}

/*************************************************************************************************/
/*!
 *  \brief  Set device's  GATT database hash.
 *
 *  \param  pHash    GATT database hash to store.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppDbSetDbHash(uint8_t *pHash)
{
  if (pHash != NULL)
  {
    memcpy(appDb.dbHash, pHash, ATT_DATABASE_HASH_LEN);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Get the discovery status.
 *
 *  \param  hdl       Database record handle.
 *
 *  \return Discovery status.
 */
/*************************************************************************************************/
uint8_t AppDbGetDiscStatus(appDbHdl_t hdl)
{
  return ((appDbRec_t *) hdl)->discStatus;
}

/*************************************************************************************************/
/*!
 *  \brief  Set the discovery status.
 *
 *  \param  hdl       Database record handle.
 *  \param  state     Discovery status.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppDbSetDiscStatus(appDbHdl_t hdl, uint8_t status)
{
  ((appDbRec_t *) hdl)->discStatus = status;
}

/*************************************************************************************************/
/*!
 *  \brief  Get the cached handle list.
 *
 *  \param  hdl       Database record handle.
 *
 *  \return Pointer to handle list.
 */
/*************************************************************************************************/
uint16_t *AppDbGetHdlList(appDbHdl_t hdl)
{
  return ((appDbRec_t *) hdl)->hdlList;
}

/*************************************************************************************************/
/*!
 *  \brief  Set the cached handle list.
 *
 *  \param  hdl       Database record handle.
 *  \param  pHdlList  Pointer to handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppDbSetHdlList(appDbHdl_t hdl, uint16_t *pHdlList)
{
  memcpy(((appDbRec_t *) hdl)->hdlList, pHdlList, sizeof(((appDbRec_t *) hdl)->hdlList));

#ifdef AM_BLE_USE_NVM
  AppDbUpdateNVM(hdl);
#endif
}

/*************************************************************************************************/
/*!
 *  \brief  Get the device name.
 *
 *  \param  pLen      Returned device name length.
 *
 *  \return Pointer to UTF-8 string containing device name or NULL if not set.
 */
/*************************************************************************************************/
char *AppDbGetDevName(uint8_t *pLen)
{
  /* if first character of name is NULL assume it is uninitialized */
  if (appDb.devName[0] == 0)
  {
    *pLen = 0;
    return NULL;
  }
  else
  {
    *pLen = appDb.devNameLen;
    return appDb.devName;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Set the device name.
 *
 *  \param  len       Device name length.
 *  \param  pStr      UTF-8 string containing device name.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppDbSetDevName(uint8_t len, char *pStr)
{
  /* check for maximum device length */
  len = (len <= sizeof(appDb.devName)) ? len : sizeof(appDb.devName);

  memcpy(appDb.devName, pStr, len);
}

/*************************************************************************************************/
/*!
 *  \brief  Get address resolution attribute value read from a peer device.
 *
 *  \param  hdl        Database record handle.
 *
 *  \return TRUE if address resolution is supported in peer device. FALSE, otherwise.
 */
/*************************************************************************************************/
bool_t AppDbGetPeerAddrRes(appDbHdl_t hdl)
{
  return ((appDbRec_t *)hdl)->peerAddrRes;
}

/*************************************************************************************************/
/*!
 *  \brief  Set address resolution attribute value for a peer device.
 *
 *  \param  hdl        Database record handle.
 *  \param  addrRes    Address resolution attribue value.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppDbSetPeerAddrRes(appDbHdl_t hdl, uint8_t addrRes)
{
  ((appDbRec_t *)hdl)->peerAddrRes = addrRes;

#ifdef AM_BLE_USE_NVM
  AppDbUpdateNVM(hdl);
#endif
}

/*************************************************************************************************/
/*!
 *  \brief  Get sign counter for a peer device.
 *
 *  \param  hdl        Database record handle.
 *
 *  \return Sign counter for peer device.
 */
/*************************************************************************************************/
uint32_t AppDbGetPeerSignCounter(appDbHdl_t hdl)
{
  return ((appDbRec_t *)hdl)->peerSignCounter;
}

/*************************************************************************************************/
/*!
 *  \brief  Set sign counter for a peer device.
 *
 *  \param  hdl          Database record handle.
 *  \param  signCounter  Sign counter for peer device.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppDbSetPeerSignCounter(appDbHdl_t hdl, uint32_t signCounter)
{
  ((appDbRec_t *)hdl)->peerSignCounter = signCounter;
}

/*************************************************************************************************/
/*!
 *  \brief  Get the peer device added to resolving list flag value.
 *
 *  \param  hdl        Database record handle.
 *
 *  \return TRUE if peer device's been added to resolving list. FALSE, otherwise.
 */
/*************************************************************************************************/
bool_t AppDbGetPeerAddedToRl(appDbHdl_t hdl)
{
  return ((appDbRec_t *)hdl)->peerAddedToRl;
}

/*************************************************************************************************/
/*!
 *  \brief  Set the peer device added to resolving list flag to a given value.
 *
 *  \param  hdl           Database record handle.
 *  \param  peerAddedToRl Peer device added to resolving list flag value.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppDbSetPeerAddedToRl(appDbHdl_t hdl, bool_t peerAddedToRl)
{
  ((appDbRec_t *)hdl)->peerAddedToRl = peerAddedToRl;
}

/*************************************************************************************************/
/*!
 *  \brief  Get the resolvable private address only attribute flag for a given peer device.
 *
 *  \param  hdl        Database record handle.
 *
 *  \return TRUE if RPA Only attribute is present on peer device. FALSE, otherwise.
 */
/*************************************************************************************************/
bool_t AppDbGetPeerRpao(appDbHdl_t hdl)
{
  return ((appDbRec_t *)hdl)->peerRpao;
}

/*************************************************************************************************/
/*!
 *  \brief  Set the resolvable private address only attribute flag for a given peer device.
 *
 *  \param  hdl        Database record handle.
 *  \param  peerRpao   Resolvable private address only attribute flag value.
 *
 *  \return None.
 */
/*************************************************************************************************/
void AppDbSetPeerRpao(appDbHdl_t hdl, bool_t peerRpao)
{
  ((appDbRec_t *)hdl)->peerRpao = peerRpao;

#ifdef AM_BLE_USE_NVM
  AppDbUpdateNVM(hdl);
#endif
}
