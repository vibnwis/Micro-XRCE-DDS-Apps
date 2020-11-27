#ifndef CIRCULARBYTEBUFFER_H_
#define CIRCULARBYTEBUFFER_H_

#ifdef __cplusplus
 extern "C" {
#endif

#define  MAX_CB_SIZE			1024
#define  CB_SIZE				MAX_CB_SIZE

typedef struct circularByteBuffer_t
{
  size_t capacity; ///< Maximum number of items in the buffer
  size_t count;    ///< Number of items in the buffer
  uint8_t buffer[CB_SIZE]; ///< Data Buffer
  size_t head;     ///< Head Index
  size_t tail;     ///< Tail Index
} circularByteBuffer_t;


bool circularByteBuffer_Init(circularByteBuffer_t *cb);

bool circularByteBuffer_Reset(circularByteBuffer_t *cb);

bool circularByteBuffer_Enqueue(circularByteBuffer_t *cb, uint8_t b);

bool circularByteBuffer_Dequeue(circularByteBuffer_t *cb, uint8_t *b);

size_t circularByteBuffer_Capacity(circularByteBuffer_t *cb);

size_t circularByteBuffer_Count(circularByteBuffer_t *cb);

bool circularByteBuffer_IsFull(circularByteBuffer_t *cb);

bool circularByteBuffer_IsEmpty(circularByteBuffer_t *cb);

void wait_cb_unlock(void);


#ifdef __cplusplus
}
#endif


#endif /* CIRCULARBYTEBUFFER_H_ */
