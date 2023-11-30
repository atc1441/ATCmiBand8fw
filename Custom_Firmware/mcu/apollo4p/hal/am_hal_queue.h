//*****************************************************************************
//
//! @file am_hal_queue.h
//!
//! @brief Functions for implementing a queue system.
//!
//! @addtogroup queue4_4p Queue Implementation
//! @ingroup apollo4p_hal
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
#ifndef AM_HAL_QUEUE_H
#define AM_HAL_QUEUE_H

//*****************************************************************************
//
//! @brief A data structure that will operate as a queue.
//!
//! This data structure holds information necessary for operating a thread-safe
//! queue. When declaring a structure of type am_hal_queue_t, you will also need
//! to provide some working memory for the queue to use. For more information on
//! setting up and using the am_hal_queue_t structure, please see the
//! documentation for am_hal_queue_init().
//
//*****************************************************************************
typedef struct
{
    volatile uint32_t ui32WriteIndex;
    volatile uint32_t ui32ReadIndex;
    volatile uint32_t ui32Length;
    uint32_t ui32Capacity;
    uint32_t ui32ItemSize;
    uint8_t *pui8Data;
}
am_hal_queue_t;

//*****************************************************************************
//
// Function-like macros.
//
//*****************************************************************************

//
//! Returns true if the queue is empty.
//
#define am_hal_queue_empty(psQueue)                                           \
    ((psQueue)->ui32Length == 0)

//
//! Returns true if the queue is full.
//
#define am_hal_queue_full(psQueue)                                            \
    ((psQueue)->ui32Length == (psQueue)->ui32Capacity)

//
//! Returns the amount of space left in the queue (in bytes).
//
#define am_hal_queue_space_left(psQueue)                                      \
    ((psQueue)->ui32Capacity - (psQueue)->ui32Length)

//
//! Returns the number of configured items that will fit in the queue.
//
#define am_hal_queue_slots_left(psQueue)                                      \
    (((psQueue)->ui32Capacity - (psQueue)->ui32Length)                        \
     / (psQueue)->ui32ItemSize)

//
//! Returns the amount of data in the queue (in bytes).
//
#define am_hal_queue_data_left(psQueue)                                       \
    ((psQueue)->ui32Length)

//
//! Returns the number of configured items left in the queue.
//
#define am_hal_queue_items_left(psQueue)                                      \
    ((psQueue)->ui32Length / (psQueue)->ui32ItemSize)

//
//! Can be used as a pointer to the next item to be read from the queue.
//
#define am_hal_queue_peek(psQueue)                                            \
    ((void *) &((psQueue)->pui8Data[(psQueue)->ui32ReadIndex]))

//
//! Can be used as a pointer to the next available slot in the queue memory.
//
#define am_hal_queue_next_slot(psQueue)                                       \
    ((void *) &((psQueue)->pui8Data[(psQueue)->ui32WriteIndex]))

//*****************************************************************************
//
//! Use this to make sure you get the size parameters right.
//
//*****************************************************************************
#define am_hal_queue_from_array(queue, array)                                 \
    am_hal_queue_init((queue), (array), sizeof((array)[0]), sizeof(array))

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// External function definitions.
//
//*****************************************************************************
//*****************************************************************************
//
//! @brief Initializes a queue.
//!
//! @param psQueue - Pointer to a queue structure.
//! @param pvData - Pointer to a memory location to be used for data storage.
//! @param ui32ItemSize - Number of bytes per item in the queue.
//! @param ui32ArraySize - Number of bytes in the data array.
//!
//! This function initializes the members of a queue structure and attaches it
//! to an array of memory that it can use for storage. This function should be
//! called before the queue is used.
//!
//! In this example, we are creating a queue that can hold 1024 32-bit
//! integers. The integers themselves will be stored in the array named
//! pui32WorkingSpace, while information about the queue itself will be stored
//! in sDataQueue.
//!
//! @note The caller should not modify any of the members of am_hal_queue_t
//! structures. The queue API will handle these members in a thread-safe way.
//!
//! @note The queue will remember what size data is in it. Other queue API
//! functions will perform transfers in units of "items" where one "item" is
//! the number of bytes you specify in the \e ui32ItemSize argument upon
//! initialization.
//!
//! Example usage:
//!
//! @code
//!
//! //
//! // Declare a queue structure and an array of bytes we can use to store
//! // data.
//! //
//! am_hal_queue_t sDataQueue;
//! uint32_t pui32WorkingSpace[1024];
//!
//! //
//! // Attach the queue structure to the working memory.
//! //
//! am_hal_queue_init(&sDataQueue, pui8WorkingSpace, sizeof(uint32_t)
//!                   sizeof(pui32WorkingSpace));
//!
//! @endcode
//!
//! The am_hal_queue_from_array macro is a convenient shorthand for this
//! operation. The code below does the same thing as the code above.
//!
//! @code
//!
//! //
//! // Declare a queue structure and an array of bytes we can use to store
//! // data.
//! //
//! am_hal_queue_t sDataQueue;
//! uint32_t pui32WorkingSpace[1024];
//!
//! //
//! // Attach the queue structure to the working memory.
//! //
//! am_hal_queue_from_array(&sDataQueue, pui8WorkingSpace);
//!
//! @endcode
//
//*****************************************************************************
extern void am_hal_queue_init(am_hal_queue_t *psQueue, void *pvData, uint32_t ui32ItemSize, uint32_t ui32ArraySize);

//*****************************************************************************
//
//! @brief Adds an item to the Queue
//!
//! @param psQueue - Pointer to a queue structure.
//! @param pvSource - Pointer to the data to be added.
//! @param ui32NumItems - Number of items to be added.
//!
//! This function will copy the data pointed to by pvSource into the queue. The
//! \e ui32NumItems term specifies the number of items to be copied from \e
//! pvSource. The size of an "item" depends on how the queue was initialized.
//! Please see am_hal_queue_init() for more information on this.
//!
//! @return true if the add operation was successful, or false if the queue
//! didn't have enough space.
//
//*****************************************************************************
extern bool am_hal_queue_item_add(am_hal_queue_t *psQueue, const void *pvSource, uint32_t ui32NumItems);

//*****************************************************************************
//
//! @brief Removes an item from the Queue
//!
//! @param psQueue - Pointer to a queue structure.
//! @param pvDest - Pointer to the data to be added.
//! @param ui32NumItems - Number of items to be added.
//!
//! This function will copy the data from the queue into the memory pointed to
//! by pvDest. The \e ui32NumItems term specifies the number of items to be
//! copied from the queue. The size of an "item" depends on how the queue was
//! initialized.  Please see am_hal_queue_init() for more information on this.
//!
//! @return true if we were able to pull the requested number of items from the
//! queue, or false if the queue didn't have that many items to pull.
//
//*****************************************************************************
extern bool am_hal_queue_item_get(am_hal_queue_t *psQueue, void *pvDest, uint32_t ui32NumItems);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_QUEUE_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

