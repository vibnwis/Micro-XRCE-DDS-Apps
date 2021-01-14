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

#define  	CAN2_MAX_ID_SIZE 			2
#define     CAN1_FRAME_SIZE				12		// For simplicity, the CAN frame of std_ID(4 bytes) + DLC(4bytes) + data(4 bytes)
#define  	CAN_MAX_DATA_SIZE 			8
#define     XRCE_CAN2_MAX_FRAME			255
#define	    XRCE_CAN2_MTU			    CAN_MAX_DATA_SIZE * XRCE_CAN2_MAX_FRAME

 /* TO DO */
// There is a possibility where other CAN devices may carry the same ID. Hence to differentiate
// a CAN frame come from the same message of a CAN device, an XRCE_CAN2_ID is created. It uses the upper
// four bits of the first byte of each frame. Besides ID, XRCE_CAN2_ID is combined to generate a unique frame.
// However, due to the number of bits, it can only represent up to 16 devices,

#define	    XRCE_CAN2_ID			    15


typedef struct CAN2_Messaging_t
{
	  size_t length; 	///< Maximum number of items in the buffer
	  size_t fr_num;    ///< Number of frame in the buffer
	  size_t id; 		///< Data Buffer
	  size_t index;     ///< Buffer Index
	  uint8_t buf[XRCE_CAN2_MTU];     ///< Tail Index
} CAN2_Messaging_t;

void circularByteBuffer_Int2Bytes(uint32_t num, uint8_t *a);

void circularByteBuffer_element_Enqueue(circularByteBuffer_t *cb, uint8_t *b);

void circularByteBuffer_Bytes2Int(uint8_t *a, uint32_t *num);

void CANreadElement(circularByteBuffer_t *cb, uint32_t *num);

void CANreadPayload(circularByteBuffer_t *cb, uint8_t *a, uint32_t len);

bool init_CAN_bus(void);

int CAN1_TX_MAILBOX_FREE_level(void);

void CAN1_TX_pending_wait(void);

int CAN1_TX(uint8_t *a, uint32_t len);

size_t TX_CAN_message(uint8_t * buf, size_t len);

void EnqueueFrame(int id, int dlc, uint8_t * arr);

int32_t cb_read_CAN_frame(uint8_t *arr, uint32_t *id, uint32_t *length);

uint8_t isIDRegistered (uint32_t id);

uint8_t registerID(uint32_t id, uint8_t *fr, uint8_t fr_num, uint8_t len);

bool resetIDMessage(uint32_t id);

uint8_t addFrame(uint8_t *fr, uint8_t len, uint8_t idx);

size_t retrieveFullMesg(uint8_t *msg, uint8_t idx);

size_t IsLastFrame(uint8_t idx, uint8_t *msg, uint8_t fr_num);

uint32_t processInFrame(uint32_t id, uint8_t *fr, uint8_t fr_num, uint32_t len);

bool resetIDXMessage(uint8_t idx);

void initCAN2Message(void);

#ifdef __cplusplus
}
#endif

#endif  /* CAN_UTILS_H_ */
