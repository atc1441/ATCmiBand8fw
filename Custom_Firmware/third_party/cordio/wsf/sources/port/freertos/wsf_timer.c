/*************************************************************************************************/
/*!
 *  \file   wsf_timer.c
 *
 *  \brief  Timer service.
 *
 *          $Date: 2016-12-28 16:12:14 -0600 (Wed, 28 Dec 2016) $
 *          $Revision: 10805 $
 *
 *  Copyright (c) 2009-2017 ARM Ltd., all rights reserved.
 *  ARM Ltd. confidential and proprietary.
 *
 *  IMPORTANT.  Your use of this file is governed by a Software License Agreement
 *  ("Agreement") that must be accepted in order to download or otherwise receive a
 *  copy of this file.  You may not use or copy this file for any purpose other than
 *  as described in the Agreement.  If you do not agree to all of the terms of the
 *  Agreement do not use this file and delete all copies in your possession or control;
 *  if you do not have a copy of the Agreement, you must contact ARM Ltd. prior
 *  to any use, copying or further distribution of this software.
 */
/*************************************************************************************************/

#include "wsf_types.h"
#include "wsf_queue.h"
#include "wsf_timer.h"
#include "wsf_assert.h"
#include "wsf_trace.h"
#include "FreeRTOS.h"
#include "timers.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/* convert seconds to timer ticks */
#define WSF_TIMER_SEC_TO_TICKS(sec)         ((1000 / WSF_MS_PER_TICK) * (sec))

/* convert milliseconds to timer ticks */
#define WSF_TIMER_MS_TO_TICKS(ms)           ((ms) / WSF_MS_PER_TICK)

#define CLK_TICKS_PER_WSF_TICKS             (WSF_MS_PER_TICK*configTICK_RATE_HZ/1000)

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

wsfQueue_t  wsfTimerTimerQueue;     /*!< Timer queue */
TimerHandle_t xWsfTimer;
static uint32_t g_ui32LastTime = 0;

/*************************************************************************************************/
/*!
 *  \fn     wsfTimerRemove
 *
 *  \brief  Remove a timer from queue.  Note this function does not lock task scheduling.
 *
 *  \param  pTimer  Pointer to timer.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void wsfTimerRemove(wsfTimer_t *pTimer)
{
  wsfTimer_t  *pElem;
  wsfTimer_t  *pPrev = NULL;

  pElem = (wsfTimer_t *) wsfTimerTimerQueue.pHead;

  /* find timer in queue */
  while (pElem != NULL)
  {
    if (pElem == pTimer)
    {
      break;
    }
    pPrev = pElem;
    pElem = pElem->pNext;
  }

  /* if timer found remove from queue */
  if (pElem != NULL)
  {
    WsfQueueRemove(&wsfTimerTimerQueue, pTimer, pPrev);

    pTimer->isStarted = FALSE;
  }
}

/*************************************************************************************************/
/*!
 *  \fn     wsfTimerInsert
 *
 *  \brief  Insert a timer into the queue sorted by the timer expiration.
 *
 *  \param  pTimer  Pointer to timer.
 *  \param  ticks   Timer ticks until expiration.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void wsfTimerInsert(wsfTimer_t *pTimer, wsfTimerTicks_t ticks)
{
  wsfTimer_t  *pElem;
  wsfTimer_t  *pPrev = NULL;

  /* task schedule lock */
  WsfTaskLock();

  /* if timer is already running stop it first */
  if (pTimer->isStarted)
  {
    wsfTimerRemove(pTimer);
  }

  pTimer->isStarted = TRUE;
  pTimer->ticks = ticks;

  pElem = (wsfTimer_t *) wsfTimerTimerQueue.pHead;

  /* find insertion point in queue */
  while (pElem != NULL)
  {
    if (pTimer->ticks < pElem->ticks)
    {
      break;
    }
    pPrev = pElem;
    pElem = pElem->pNext;
  }

  /* insert timer into queue */
  WsfQueueInsert(&wsfTimerTimerQueue, pTimer, pPrev);

  /* task schedule unlock */
  WsfTaskUnlock();
}

static void WsfTimer_handler(TimerHandle_t xTimer)
{
  WsfTaskSetReady(0, WSF_TIMER_EVENT);
}

/*************************************************************************************************/
/*!
 *  \fn     WsfTimerInit
 *
 *  \brief  Initialize the timer service.  This function should only be called once
 *          upon system initialization.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WsfTimerInit(void)
{
  WSF_QUEUE_INIT(&wsfTimerTimerQueue);

  if(xWsfTimer == NULL)
  {
    xWsfTimer = xTimerCreate("WSF Timer", pdMS_TO_TICKS(WSF_MS_PER_TICK),
          pdFALSE, NULL, WsfTimer_handler);
    configASSERT(xWsfTimer);
    g_ui32LastTime = xTaskGetTickCount();
  }
}

/*************************************************************************************************/
/*!
 *  \fn     WsfTimerStartSec
 *
 *  \brief  Start a timer in units of seconds.
 *
 *  \param  pTimer  Pointer to timer.
 *  \param  sec     Seconds until expiration.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WsfTimerStartSec(wsfTimer_t *pTimer, wsfTimerTicks_t sec)
{
  WSF_TRACE_INFO2("WsfTimerStartSec pTimer:0x%x ticks:%u", (uint32_t)pTimer, WSF_TIMER_SEC_TO_TICKS(sec));

  /* insert timer into queue */
  wsfTimerInsert(pTimer, WSF_TIMER_SEC_TO_TICKS(sec));
}

/*************************************************************************************************/
/*!
 *  \fn     WsfTimerStartMs
 *
 *  \brief  Start a timer in units of milliseconds.
 *
 *  \param  pTimer  Pointer to timer.
 *  \param  ms     Milliseconds until expiration.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WsfTimerStartMs(wsfTimer_t *pTimer, wsfTimerTicks_t ms)
{
  WSF_TRACE_INFO2("WsfTimerStartMs pTimer:0x%x ticks:%u", (uint32_t)pTimer, WSF_TIMER_MS_TO_TICKS(ms));

  /* insert timer into queue */
  wsfTimerInsert(pTimer, WSF_TIMER_MS_TO_TICKS(ms));
}

/*************************************************************************************************/
/*!
 *  \fn     WsfTimerStop
 *
 *  \brief  Stop a timer.
 *
 *  \param  pTimer  Pointer to timer.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WsfTimerStop(wsfTimer_t *pTimer)
{
  WSF_TRACE_INFO1("WsfTimerStop pTimer:0x%x", pTimer);

  /* task schedule lock */
  WsfTaskLock();

  wsfTimerRemove(pTimer);

  /* task schedule unlock */
  WsfTaskUnlock();
}

/*************************************************************************************************/
/*!
 *  \fn     WsfTimerUpdate
 *
 *  \brief  Update the timer service with the number of elapsed ticks.
 *
 *  \param  ticks  Number of ticks since last update.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WsfTimerUpdate(wsfTimerTicks_t ticks)
{
  wsfTimer_t  *pElem;

  /* task schedule lock */
  WsfTaskLock();

  pElem = (wsfTimer_t *) wsfTimerTimerQueue.pHead;

  /* iterate over timer queue */
  while (pElem != NULL)
  {
    /* decrement ticks while preventing underflow */
    if (pElem->ticks > ticks)
    {
      pElem->ticks -= ticks;
    }
    else
    {
      pElem->ticks = 0;

      /* timer expired; set task for this timer as ready */
      WsfTaskSetReady(pElem->handlerId, WSF_TIMER_EVENT);
    }

    pElem = pElem->pNext;
  }

  /* task schedule unlock */
  WsfTaskUnlock();
}

/*************************************************************************************************/
/*!
 *  \fn     WsfTimerNextExpiration
 *
 *  \brief  Return the number of ticks until the next timer expiration.  Note that this
 *          function can return zero even if a timer is running, indicating a timer
 *          has expired but has not yet been serviced.
 *
 *  \param  pTimerRunning   Returns TRUE if a timer is running, FALSE if no timers running.
 *
 *  \return The number of ticks until the next timer expiration.
 */
/*************************************************************************************************/
wsfTimerTicks_t WsfTimerNextExpiration(bool_t *pTimerRunning)
{
  wsfTimerTicks_t ticks;

  /* task schedule lock */
  WsfTaskLock();

  if (wsfTimerTimerQueue.pHead == NULL)
  {
    *pTimerRunning = FALSE;
    ticks = 0;
  }
  else
  {
    *pTimerRunning = TRUE;
    ticks = ((wsfTimer_t *) wsfTimerTimerQueue.pHead)->ticks;
  }

  /* task schedule unlock */
  WsfTaskUnlock();

  return ticks;
}

/*************************************************************************************************/
/*!
 *  \fn     WsfTimerServiceExpired
 *
 *  \brief  Service expired timers for the given task.
 *
 *  \param  taskId      Task ID.
 *
 *  \return Pointer to timer or NULL.
 */
/*************************************************************************************************/
wsfTimer_t *WsfTimerServiceExpired(wsfTaskId_t taskId)
{
  wsfTimer_t  *pElem;
  wsfTimer_t  *pPrev = NULL;

  /* Unused parameters */
  (void)taskId;

  /* task schedule lock */
  WsfTaskLock();

  /* find expired timers in queue */
  if (((pElem = (wsfTimer_t *) wsfTimerTimerQueue.pHead) != NULL) &&
      (pElem->ticks == 0))
  {
    /* remove timer from queue */
    WsfQueueRemove(&wsfTimerTimerQueue, pElem, pPrev);

    pElem->isStarted = FALSE;

    /* task schedule unlock */
    WsfTaskUnlock();

    WSF_TRACE_INFO1("Timer expired pTimer:0x%x", pElem);

    /* return timer */
    return pElem;
  }

  /* task schedule unlock */
  WsfTaskUnlock();

  return NULL;
}

//*****************************************************************************
//
// Calculate the elapsed time, and update the WSF software timers.
//
//*****************************************************************************
void WsfTimerUpdateTicks(void)
{
    uint32_t ui32CurrentTime, ui32ElapsedTime;
    bool_t bTimerRunning;
    wsfTimerTicks_t xNextExpiration;

    //
    // Read the continuous timer.
    //
    ui32CurrentTime = xTaskGetTickCount();

    //
    // Figure out how long it has been since the last time we've read the
    // continuous timer. We should be reading often enough that we'll never
    // have more than one overflow.
    //
    ui32ElapsedTime = ui32CurrentTime - g_ui32LastTime;

    //
    // Check to see if any WSF ticks need to happen.
    //
    if ( (ui32ElapsedTime / CLK_TICKS_PER_WSF_TICKS) > 0 )
    {
        //
        // Update the WSF timers and save the current time as our "last
        // update".
        //
        WsfTimerUpdate(ui32ElapsedTime / CLK_TICKS_PER_WSF_TICKS);

        g_ui32LastTime = ui32CurrentTime;
    }

    //
    // Check to see when the next timer expiration should happen.
    //
    xNextExpiration = WsfTimerNextExpiration(&bTimerRunning);

    //
    // If there's a pending WSF timer event, set an interrupt to wake us up in
    // time to service it.
    //
    if ( xNextExpiration )
    {
        configASSERT(pdPASS == xTimerChangePeriod( xWsfTimer,
                pdMS_TO_TICKS(xNextExpiration*CLK_TICKS_PER_WSF_TICKS), 100)) ;
    }
}

