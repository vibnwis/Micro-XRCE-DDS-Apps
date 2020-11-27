#ifndef CAN_UTILS_H_
#define CAN_UTILS_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdio.h>
#include <stdint.h> // uint8_t
#include <stddef.h> // size_t
#include <stdbool.h> // bool
#include "circularByteBuffer.h"


#define     CAN1_FRAME_SIZE				12		// For simplicity, the CAN frame of std_ID(4 bytes) + DLC(4bytes) + data(4 bytes)

void circularByteBuffer_Int2Bytes(uint32_t num, uint8_t *a);

void circularByteBuffer_element_Enqueue(circularByteBuffer_t *cb, uint8_t *b);

void circularByteBuffer_Bytes2Int(uint8_t *a, uint32_t *num);

void CANreadElement(circularByteBuffer_t *cb, uint32_t *num);

void CANreadPayload(circularByteBuffer_t *cb, uint8_t *a, uint32_t len);

bool init_CAN_bus(void);

void CAN1_TX_pending_wait(void);

void CAN1_TX(void);

int32_t cb_read_CAN_frame(uint8_t *arr);

#ifdef __cplusplus
}
#endif

#endif  /* CAN_UTILS_H_ */
