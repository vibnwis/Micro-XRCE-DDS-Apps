// # Circular Byte Buffer For Embedded Applications (Index Based)
// Author: Brian Khuu (July 2020) (briankhuu.com) (mofosyne@gmail.com)
// Reason: Malloc free, minimum overhead implementation of a circular buffer.
//   Static inlined handlers for speed and ease of usage in various projects.
//   Index based implementation diverges from harshkn's version, since it is
//   easier for me to grok. However may come at cost of speed and optimisation.
//   Also uses byte based rather than item based for easier understability
//   when used for simpler byte array circular buffers.
//   I have also written an pointer based circular buffer version.
// Based on harshkn's circular buffer: https://gist.github.com/harshkn/909546
// This Gist (Pointer): https://gist.github.com/mofosyne/d7a4a8d6a567133561c18aaddfd82e6f
// This Gist (Index): https://gist.github.com/mofosyne/82020d5c0e1e11af0eb9b05c73734956

#include <stdint.h> // uint8_t
#include <stddef.h> // size_t
#include <stdbool.h> // bool
#include "stm32f4xx_hal.h"
#include "circularByteBuffer.h"


/*******************************************************************************
 * Init/Reset
*******************************************************************************/

bool cb_lock = RESET;

bool circularByteBuffer_Init(circularByteBuffer_t *cb)
{
  // Init Struct
  cb->capacity = MAX_CB_SIZE;
  cb->count = 0;
  cb->head = 0;
  cb->tail = 0;
  cb_lock = RESET;
  return true; ///< Successful
}

bool circularByteBuffer_Reset(circularByteBuffer_t *cb)
{
  cb->count = 0;
  cb->head = 0;
  cb->tail = 0;
  cb_lock = RESET;
  return true; ///< Successful
}


/*******************************************************************************
 * Circular byte buffer Push/Pop
*******************************************************************************/

 bool circularByteBuffer_Enqueue(circularByteBuffer_t *cb, uint8_t b)
{
  // Full
  if (cb->count >= cb->capacity)
    return false; ///< Failed
  
   wait_cb_unlock();  	// check and wait cb_lock if being locked
   cb_lock = SET;     	//It is my turn. Set the lock

  // Push value
  cb->buffer[cb->tail] = b;

  // Increment tail
  cb->tail = (cb->tail + 1) % cb->capacity;
  cb->count = cb->count + 1;


  cb_lock = RESET;		//Release the lock

  return true; ///< Successful
}


 /*******************************************************************************
  * Circular byte buffer Push/Pop
 *******************************************************************************/

 bool circularByteBuffer_element_Enqueue(circularByteBuffer_t *cb, uint8_t *b)
 {
	 int i = 0;
	 for (i = 0; i < 4; i ++) {
		 circularByteBuffer_Enqueue(cb, *(b+i));
	 }
	 return true;
 }


bool circularByteBuffer_Dequeue(circularByteBuffer_t *cb, uint8_t *b)
{

  // Empty
  if (cb->count == 0)
    return false; ///< Failed
  

   wait_cb_unlock();  	// check and wait cb_lock if being locked
   cb_lock = SET;     	//It is my turn. Set the lock

  // Pop value
  *b = cb->buffer[cb->head];

  // Increment head
  cb->head = (cb->head + 1) % cb->capacity;
  cb->count = cb->count - 1;

  cb_lock = RESET;		//Release the lock

  return true; ///< Successful
}


/*******************************************************************************
 * Circular byte buffer utility functions
*******************************************************************************/

size_t circularByteBuffer_Capacity(circularByteBuffer_t *cb)
{
  return cb->capacity;
}

size_t circularByteBuffer_Count(circularByteBuffer_t *cb)
{
  return cb->count;
}

bool circularByteBuffer_IsFull(circularByteBuffer_t *cb)
{
  return (cb->count >= cb->capacity);
}

bool circularByteBuffer_IsEmpty(circularByteBuffer_t *cb)
{
  return (cb->count == 0);
}

void wait_cb_unlock(void)
{
	while (cb_lock == SET); //wait till lock released
}






