/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Wireless Data Exchange Protocol Definitions.
 *
 *  Copyright (c) 2013-2018 Arm Ltd. All Rights Reserved.
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

#ifndef WDX_DEFS_H
#define WDX_DEFS_H

#ifdef __cplusplus
extern "C"
{
#endif

/*! \addtogroup WIRELESS_DATA_EXCHANGE_PROFILE
 *  \{ */

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! \brief Base UUID:  005fXXXX-2ff2-4ed5-b045-4C7463617865 */
#define WDX_UUID_PART1                0x65, 0x78, 0x61, 0x63, 0x74, 0x4c, 0x45, 0xb0, \
                                      0xd5, 0x4e, 0xf2, 0x2f
/*! \brief Base UUID Part 2 */
#define WDX_UUID_PART2                0x5f, 0x00

/*! \brief Macro for building UUIDs */
#define WDX_UUID_BUILD(part)          WDX_UUID_PART1, UINT16_TO_BYTES(part), WDX_UUID_PART2

/*! \brief WDX Service */
#define WDX_SVC_UUID                  0xFEF6

/*! \brief WDX Device Configuration Characteristic */
#define WDX_DC_UUID                   WDX_UUID_BUILD(0x0002)

/*! \brief WDX File Transfer Control Characteristic */
#define WDX_FTC_UUID                  WDX_UUID_BUILD(0x0003)

/*! \brief WDX File Transfer Data Characteristic */
#define WDX_FTD_UUID                  WDX_UUID_BUILD(0x0004)

/*! \brief WDX Authentication Characteristic */
#define WDX_AU_UUID                   WDX_UUID_BUILD(0x0005)


/**************************************************************************************************
  Constant Definitions
**************************************************************************************************/

/** \name WDXS File List Configuration
 *
 */
/**@{*/
#define WDX_FLIST_HANDLE               0             /*!< \brief File List handle */
#define WDX_FLIST_FORMAT_VER           1             /*!< \brief File List version */
#define WDX_FLIST_HDR_SIZE             7             /*!< \brief File List header length */
#define WDX_FLIST_RECORD_SIZE          40            /*!< \brief File List record length */
/*! \brief File list max length. */
#define WDX_FLIST_MAX_LEN              (WDX_FLIST_HDR_SIZE + (WDX_FLIST_RECORD_SIZE * (WSF_EFS_MAX_FILES-1)))
/**@}*/

/*! \brief Device configuration characteristic message header length */
#define WDX_DC_HDR_LEN                 2

/** \name Device Configuration Characteristic oOperations
 *
 */
/**@{*/
#define WDX_DC_OP_GET                  0x01         /*!< \brief Get a parameter value */
#define WDX_DC_OP_SET                  0x02         /*!< \brief Set a parameter value */
#define WDX_DC_OP_UPDATE               0x03         /*!< \brief Send an update of a parameter value */
/**@}*/

/** \name Device Control Characteristic Parameter IDs
 *
 */
/**@{*/
#define WDX_DC_ID_CONN_UPDATE_REQ      0x01         /*!< \brief Connection Parameter Update Request */
#define WDX_DC_ID_CONN_PARAM           0x02         /*!< \brief Current Connection Parameters */
#define WDX_DC_ID_DISCONNECT_REQ       0x03         /*!< \brief Disconnect Request */
#define WDX_DC_ID_CONN_SEC_LEVEL       0x04         /*!< \brief Connection Security Level */
#define WDX_DC_ID_SECURITY_REQ         0x05         /*!< \brief Security Request */
#define WDX_DC_ID_SERVICE_CHANGED      0x06         /*!< \brief Service Changed */
#define WDX_DC_ID_DELETE_BONDS         0x07         /*!< \brief Delete Bonds */
#define WDX_DC_ID_ATT_MTU              0x08         /*!< \brief Current ATT MTU */
#define WDX_DC_ID_PHY_UPDATE_REQ       0x09         /*!< \brief PHY update request */
#define WDX_DC_ID_PHY                  0x0A         /*!< \brief Current PHY */
#define WDX_DC_ID_BATTERY_LEVEL        0x20         /*!< \brief Battery level */
#define WDX_DC_ID_MODEL_NUMBER         0x21         /*!< \brief Device Model */
#define WDX_DC_ID_FIRMWARE_REV         0x22         /*!< \brief Device Firmware Revision */
#define WDX_DC_ID_ENTER_DIAGNOSTICS    0x23         /*!< \brief Enter Diagnostic Mode */
#define WDX_DC_ID_DIAGNOSTICS_COMPLETE 0x24         /*!< \brief Diagnostic Complete */
#define WDX_DC_ID_DISCONNECT_AND_RESET 0x25         /*!< \brief Disconnect and Reset */
/**@}*/

/** \name Device Control Parameter Lengths
 *
 */
/**@{*/
#define WDX_DC_LEN_DATA_FORMAT         1            /*!< \brief Data format */
#define WDX_DC_LEN_SEC_LEVEL           1            /*!< \brief Security Level */
#define WDX_DC_LEN_ATT_MTU             2            /*!< \brief ATT MTU */
#define WDX_DC_LEN_BATTERY_LEVEL       1            /*!< \brief Battery level */
#define WDX_DC_LEN_CONN_PARAM_REQ      8            /*!< \brief Connection parameter request */
#define WDX_DC_LEN_CONN_PARAM          7            /*!< \brief Current connection parameters */
#define WDX_DC_LEN_PHY_UPDATE_REQ      5            /*!< \brief PHY update request */
#define WDX_DC_LEN_PHY                 3            /*!< \brief Current PHY */
#define WDX_DC_LEN_DIAG_COMPLETE       0            /*!< \brief Diagnostic complete */
#define WDX_DC_LEN_DEVICE_MODEL        18           /*!< \brief Device Model */
#define WDX_DC_LEN_FIRMWARE_REV        16           /*!< \brief Firmware Revision */
/**@}*/

/** \name File Transfer Control Characteristic Message Header Length
 *
 */
/**@{*/
#define WDX_FTC_HDR_LEN                1           /*!< \brief Header length. */
#define WDX_FTC_HANDLE_LEN             2           /*!< \brief Handle length. */

/** \name File Transfer Control Characteristic Operations
 *
 */
/**@{*/
#define WDX_FTC_OP_NONE                0x00        /*!< \brief No operation */
#define WDX_FTC_OP_GET_REQ             0x01        /*!< \brief Get a file from the server */
#define WDX_FTC_OP_GET_RSP             0x02        /*!< \brief File get response */
#define WDX_FTC_OP_PUT_REQ             0x03        /*!< \brief Put a file to the server */
#define WDX_FTC_OP_PUT_RSP             0x04        /*!< \brief File put response */
#define WDX_FTC_OP_ERASE_REQ           0x05        /*!< \brief Erase a file on the server */
#define WDX_FTC_OP_ERASE_RSP           0x06        /*!< \brief File erase response */
#define WDX_FTC_OP_VERIFY_REQ          0x07        /*!< \brief Verify a file (e.g. check its CRC) */
#define WDX_FTC_OP_VERIFY_RSP          0x08        /*!< \brief File verify response */
#define WDX_FTC_OP_ABORT               0x09        /*!< \brief Abort a get, put, or list operation in progress */
#define WDX_FTC_OP_EOF                 0x0a        /*!< \brief End of file reached */
/**@}*/

/** \name File Transfer Control Permissions
 *
 */
/**@{*/
#define WDX_FTC_GET_PERMITTED          0x01        /*!< \brief File Get Permitted */
#define WDX_FTC_PUT_PERMITTED          0x02        /*!< \brief File Put Permitted */
#define WDX_FTC_ERASE_PERMITTED        0x04        /*!< \brief File Erase Permitted */
#define WDX_FTC_VERIFY_PERMITTED       0x08        /*!< \brief File Verify Permitted */
/**@}*/

/** \name File Transfer Control Characteristic Status
 *
 */
/**@{*/
#define WDX_FTC_ST_SUCCESS             0           /*!< \brief Success */
#define WDX_FTC_ST_INVALID_OP_FILE     1           /*!< \brief Invalid operation for this file */
#define WDX_FTC_ST_INVALID_HANDLE      2           /*!< \brief Invalid file handle */
#define WDX_FTC_ST_INVALID_OP_DATA     3           /*!< \brief Invalid operation data */
#define WDX_FTC_ST_IN_PROGRESS         4           /*!< \brief Operation in progress */
#define WDX_FTC_ST_VERIFICATION        5           /*!< \brief Verification failure */
/**@}*/

/** \name File Transfer Control Transport
 *
 */
/**@{*/
#define WDX_FTC_TRANSPORT_TYPE         0           /*!< \brief Transport Type */
#define WDX_FTC_TRANSPORT_ID           0x0030      /*!< \brief Transport ID */
/**@}*/

/*! \brief File transfer data characteristic message header length */
#define WDX_FTD_HDR_LEN                0

/*! \brief Authentication message header length */
#define WDX_AU_HDR_LEN                 1

/** \name Authentication Characteristic Operations
 *
 */
/**@{*/
#define WDX_AU_OP_START                0x01        /*!< \brief Authentication start */
#define WDX_AU_OP_CHALLENGE            0x02        /*!< \brief Authentication challenge */
#define WDX_AU_OP_REPLY                0x03        /*!< \brief Authentication reply */
/**@}*/

/** \name Proprietary ATT Error Codes
 *
 */
/**@{*/
#define WDX_APP_AUTH_REQUIRED          0x80        /*!< \brief Application authentication required */
#define WDX_AU_ST_INVALID_MESSAGE      0x81        /*!< \brief Authentication invalid message */
#define WDX_AU_ST_INVALID_STATE        0x82        /*!< \brief Authentication invalid state */
#define WDX_AU_ST_AUTH_FAILED          0x83        /*!< \brief Authentication failed */
/**@}*/

/** \name Authentication Characteristic Authentication Level
 *
 */
/**@{*/
#define WDX_AU_LVL_NONE                0x00        /*!< \brief None */
#define WDX_AU_LVL_USER                0x01        /*!< \brief User level */
#define WDX_AU_LVL_MAINT               0x02        /*!< \brief Maintenance level */
#define WDX_AU_LVL_DEBUG               0x03        /*!< \brief Debug level */
/**@}*/

/** \name Authenttication Characteristic Message Parameter Lengths
 *
 */
/**@{*/
#define WDX_AU_MSG_HDR_LEN             1           /*!< \brief Message header length */
#define WDX_AU_PARAM_LEN_START         2           /*!< \brief Authentication start */
#define WDX_AU_PARAM_LEN_CHALLENGE     16          /*!< \brief Authentication challenge */
#define WDX_AU_PARAM_LEN_REPLY         8           /*!< \brief Authentication reply */
/**@}*/

/** \name Authenttication Characteristic Random Number and Key Lengths
 *
 */
/**@{*/
#define WDX_AU_RAND_LEN                16          /*!< \brief Authentication Random challenge length (bytes)*/
#define WDX_AU_KEY_LEN                 16          /*!< \brief Authentication Key length (bytes) */
#define WDX_AU_HASH_LEN                8           /*!< \brief Authentication Hash length (bytes) */
/**@}*/

/** \name WDXS Media Types
 *
 */
/**@{*/
#define WDX_FLASH_MEDIA                0           /*!< \brief Flash media type. */
#define WDX_OTA_MEDIA                  1           /*!< \brief OTA media type. */
#define WDX_RAM_MEDIA                  2           /*!< \brief RAM media type. */
#define WDX_STREAM_MEDIA               3           /*!< \brief Stream media type. */
/**@}*/

/** \name WDXS File Transfer Control Command Message Lengths
 *
 */
/**@{*/
#define WDX_FTC_ABORT_LEN              3           /*!< \brief Abort message length. */
#define WDX_FTC_ERASE_LEN              3           /*!< \brief Erase message length. */
#define WDX_FTC_VERIFY_LEN             3           /*!< \brief Verify message length. */
#define WDX_FTC_PUT_LEN                16          /*!< \brief Put message length. */
#define WDX_FTC_GET_LEN                12          /*!< \brief Get message length. */
/**@}*/

/*! \} */    /* WIRELESS_DATA_EXCHANGE_PROFILE */

#ifdef __cplusplus
}
#endif

#endif /* WDX_DEFS_H */
