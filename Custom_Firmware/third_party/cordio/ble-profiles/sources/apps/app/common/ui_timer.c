/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  User Interface main module - WSF timer implementation.
 *
 *  Copyright (c) 2018-2019 Arm Ltd. All Rights Reserved.
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
#include "wsf_types.h"
#include "wsf_trace.h"
#include "wsf_timer.h"
#include "wsf_assert.h"
#include "ui_api.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Max number of active timers */
#define UI_TIMER_WSF_MAX_TIMERS           3

/*! Unused timer ID */
#define UI_TIMER_WSF_UNUSED               0

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/* UI Timers */
static wsfTimer_t uiTimerList[UI_TIMER_WSF_MAX_TIMERS];

/* UI timer task handler ID */
wsfHandlerId_t iuTimerWsfHandlerId;

/*************************************************************************************************/
/*!
 *  \brief      Add a WSF timer for a UI event.
 *
 *  \param      event     Event to pass to UiProcEvent on timer expiration.
 *
 *  \return     WSF timer or NULL if all timers are in use.
 */
/*************************************************************************************************/
static wsfTimer_t *uiTimerAddTimer(uint8_t event)
{
  wsfTimer_t *pTimer = uiTimerList;
  int8_t i;

  for (i = 0; i < UI_TIMER_WSF_MAX_TIMERS; i++, pTimer++)
  {
    if (pTimer->msg.event == UI_TIMER_WSF_UNUSED)
    {
      pTimer->msg.event = event;
      pTimer->handlerId = iuTimerWsfHandlerId;

      return pTimer;
    }
  }

  /* If ASSERT happens, increase UI_TIMER_WSF_MAX_TIMERS to needed number of active timers. */
  WSF_ASSERT(0);

  return NULL;
}

/*************************************************************************************************/
/*!
 *  \brief      Get the WSF timer for a UI event.
 *
 *  \param      event     Timer event.
 *  \param      addTimer  TRUE to add a timer if the event doesn't exist.
 *
 *  \return     WSF timer or NULL if timer for event does not exist.
 */
/*************************************************************************************************/
static wsfTimer_t *uiTimerGetTimerByEvent(uint8_t event, bool_t addTimer)
{
  wsfTimer_t *pTimer = uiTimerList;
  int8_t i;

  for (i = 0; i < UI_TIMER_WSF_MAX_TIMERS; i++, pTimer++)
  {
    if (pTimer->msg.event == event)
    {
      return pTimer;
    }
  }

  if (addTimer)
  {
    return uiTimerAddTimer(event);
  }

  return NULL;
}

/*************************************************************************************************/
/*!
 *  \brief  Application handler init function - for internal test.
 *
 *  \param  handlerID  WSF handler ID.
 *
 *  \return None.
 */
/*************************************************************************************************/
void uiTimerWsfHandlerInit(wsfHandlerId_t handlerId)
{
  iuTimerWsfHandlerId = handlerId;
}

/*************************************************************************************************/
/*!
 *  \brief  WSF event handler for application - For internal test.
 *
 *  \param  event   WSF event mask.
 *  \param  pMsg    WSF message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void uiTimerWsfHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
  uint8_t timerEvent = pMsg->event;
  wsfTimer_t *pTimer = uiTimerGetTimerByEvent(timerEvent, FALSE);

  if (pTimer)
  {
    /* Free the timer */
    pTimer->msg.event = UI_TIMER_WSF_UNUSED;

    /* Process the timer event */
    UiProcEvent(timerEvent);
  }
}

/*************************************************************************************************/
/*!
 *  \brief      Start a UI timer.
 *
 *  \param      event   Event to pass to UiProcEvent on timer expiration.
 *  \param      ms      Time in milliseconds until timer expiration.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void UiTimerStart(uint8_t event, uint32_t ms)
{
  wsfTimer_t *pTimer = uiTimerGetTimerByEvent(event, TRUE);

  if (pTimer)
  {
    WsfTimerStartMs(pTimer, ms);
  }
}

/*************************************************************************************************/
/*!
 *  \brief      Stop a UI timer.
 *
 *  \param      event   Event to pass to UiProcEvent on timer expiration.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void UiTimerStop(uint8_t event)
{
  wsfTimer_t *pTimer = uiTimerGetTimerByEvent(event, FALSE);

  if (pTimer)
  {
    /* Stop and free the timer */
    WsfTimerStop(pTimer);
    pTimer->msg.event = UI_TIMER_WSF_UNUSED;
  }
}

/*************************************************************************************************/
/*!
 *  \brief      Initialize the UI Timer subsystem.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void UiTimerInit(void)
{
  /* Clear timers */
  memset(uiTimerList, 0, sizeof(uiTimerList));

  /* Start timer handler */
  iuTimerWsfHandlerId = WsfOsSetNextHandler(uiTimerWsfHandler);
  uiTimerWsfHandlerInit(iuTimerWsfHandlerId);
}
