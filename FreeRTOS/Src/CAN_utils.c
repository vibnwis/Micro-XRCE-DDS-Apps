#include <stdint.h> // uint8_t
#include <string.h> //memecpy
#include <stddef.h> // size_t
#include <stdbool.h> // bool
#include "CAN_utils.h"
//#include "circularByteBuffer.h"
#include "stm32f4xx_hal.h"
/*
* Little-endian e.g., 0x87654321 0x87 = a [3],.. 0x21 = a [0]  
*/
//#define DISABLE_CAN1_TX_PENDING_WAIT

CAN2_Messaging_t can2_messages[CAN2_MAX_ID_SIZE];

static void Error_Handler(char * str);

extern CAN_TxHeaderTypeDef txHeader; //declare a specific header for message transmittions
extern CAN_RxHeaderTypeDef rxHeader; //declare header for message reception
extern uint32_t TxMailbox;
extern CAN_FilterTypeDef sFilterConfig; //declare CAN filter structure
extern circularByteBuffer_t cb_han;
extern CAN_HandleTypeDef hcan1;


bool init_CAN_bus (void) {

	/*
	 * CAN_TxHeaderTypeDef txHeader, CAN_RxHeaderTypeDef rxHeader are for future development
	 * For time being, simply reset the circular buffer of CAN-Bus
	 */
	initCAN2Message();
	return circularByteBuffer_Reset(&cb_han);

}


int32_t cb_read_CAN_frame(uint8_t *arr, uint32_t *id, uint32_t *length)
{

	uint32_t cb_element;
	int32_t len;
	int i;
	uint8_t arr_val [8];

	printf("\n\rcb_read_CAN_frame() entered \n\r");
	len = circularByteBuffer_Count(&cb_han);
	printf("\n\rcircularByteBuffer length %d  \n\r", (int)len);

	if (len > 0 ) //CAN1_FRAME_SIZE)
	{

		//read std_ID
		CANreadElement(&cb_han, &cb_element);
		printf("\n\rCAN ID, 0x%x \n\r", (unsigned int)cb_element);
		*id = cb_element;

		//read CAN DLC
		CANreadElement(&cb_han, &cb_element);
		printf("\n\rCAN DLC, 0x%x \n\r", (unsigned int)cb_element);
		*length = cb_element;

		//read CAN DATA
		CANreadPayload(&cb_han, arr_val,  *length);  //cb_element = CAN DLC
		printf("\n\rCAN payload: ");
		for (i=0; i< *length; i++) {
			printf("data[%d] = 0x%x ",i, (unsigned int)arr_val[i]);
		}
		printf("\n\r");
		memcpy(arr, arr_val, len);

	}

	return len;

}



void EnqueueFrame(int id, int dlc, uint8_t * arr)
{
	uint8_t arr_val[4];
	int i;
	/*
		 * Store the CAN frame in this structure only
		 * i.e., | rxHeader.StdId (4 bytes) | rxHeader.DLC (4 bytes) | data (4 bytes) |
		 *
		 * Note: the term "element" depicts data type of uint32_t
		 */

	/* convert rxHeader.StdId to 4 bytes */
    circularByteBuffer_Int2Bytes(id, arr_val);

	/* Store rxHeader.StdId, i.e, arr_val in the circular buffer */
	circularByteBuffer_element_Enqueue(&cb_han, arr_val);
	printf("\n\rEnqueue id = 0x%x ", id);
	for (i=0; i<4; i++)
	{
		printf("[%d]=0x%x ", i, arr_val[i]);
	}
	printf("\n\r");


	/* convert rxHeader.DCL to bytes */
	circularByteBuffer_Int2Bytes(dlc, arr_val);
	/* Store rxHeader.StdId, i.e, arr_val in the circular buffer */
	circularByteBuffer_element_Enqueue(&cb_han, arr_val);

	printf("\n\rEnqueue DCL = 0x%x ", dlc);
	for (i=0; i<4; i++)
	{
		printf("[%d]=0x%x ", i, arr_val[i]);
	}
	printf("\n\r");

	/* Store in the circular buffer byte by byte*/
	printf("\n\rEnqueue data");
	for (i=0; i< dlc; i++) {
		circularByteBuffer_Enqueue(&cb_han, arr[i]);
		printf("[%d]=0x%x ", i, arr[i]);
	}
	printf("\n\r");

}


void circularByteBuffer_Int2Bytes(uint32_t num, uint8_t *a)
{
 
  a[3] = (num>>24) & 0xFF;
 // printf("\n\r num 0x%x a[3] = %x\n\r", (unsigned int)num, (unsigned int)a[3]);

  a[2] = (num>>16) & 0xFF;
//  printf("\n\r num 0x%x a[2] = %x\n\r", (unsigned int)num, (unsigned int)a[2]);

  a[1] = (num>>8) & 0xFF;
//  printf("\n\r num 0x%x a[1] = %x\n\r", (unsigned int)num, (unsigned int)a[1]);

  a[0] = num & 0xFF;
//  printf("\n\r num 0x%x a[0] = %x\n\r", (unsigned int)num, (unsigned int)a[0]);

}

/*
* Little-endian 0x87654321 0x87 = a [3],.. 0x21 = a [0]
*/
void circularByteBuffer_Bytes2Int(uint8_t *a, uint32_t *num)
{

  *num = ((a[3]<<24) & 0xFF000000) |( (a[2]<<16) & 0x00FF0000 )|( (a[1]<<8) & 0x0000FF00 )|( a[0] & 0x000000FF);

}

void CANreadElement(circularByteBuffer_t *cb, uint32_t *num)
{

  uint8_t a[4]; 
  
  circularByteBuffer_Dequeue(cb, &a[0]);
  circularByteBuffer_Dequeue(cb, &a[1]);
  circularByteBuffer_Dequeue(cb, &a[2]);
  circularByteBuffer_Dequeue(cb, &a[3]);
  
  circularByteBuffer_Bytes2Int(a, num);

}

void CANreadPayload(circularByteBuffer_t *cb, uint8_t *a, uint32_t len)
{

  uint32_t i=0;

  for (i=0; i<len; i++) { 
    circularByteBuffer_Dequeue(cb, a+i);
  }
  
}

int CAN1_TX(uint8_t *a, uint32_t len)
{
	/*
	 * Frame payload length DLC
	 */
	if (len > 0 && len <=8) {
		txHeader.DLC=len;  			// set message size
	}
	else {
		printf("\n\r ERROR CAN1_TX() **************** \n\r");
		printf("\n\r ERROR CAN1_TX()- standard frame length exceeded %d \n\r", (int)len);
		printf("\n\r ERROR CAN1_TX() **************** \n\r");
		return 1;
	}
	/*
	 * Frame ID
	 */
	txHeader.StdId=0x123;

	/*
	 * Frame Flags
	 */
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.TransmitGlobalTime = DISABLE;

	/*
	 * Check for free line
	 */
	if ((CAN1_TX_MAILBOX_FREE_level()) != HAL_OK)
	{
		return 1;
	}


	uint8_t txData[8]; 	// example {0xAA,0x01,0x02,0x03,0x04,0x05,0x06,0x07};

	memcpy (txData, a, len);

	if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, (uint32_t*)&TxMailbox) != HAL_OK) {
		printf("\n\rHAL_CAN_AddTxMessage() Failed \n\r");
		Error_Handler("\n\rCAN1_TX\n\r");

	}
	 printf("\n\rHAL_CAN_AddTxMessage() Passed \n\r");

	 return 0;

//#ifndef DISABLE_CAN1_TX_PENDING_WAIT
//	CAN1_TX_pending_wait();
//#endif
}

size_t TX_CAN_message(uint8_t * buf, size_t len){
	size_t index = 0;
	size_t to_write, frame_size;
	uint8_t sendbuf[8];
	size_t f_num = 0, quotient  = 0, remainder = 0;
	size_t max_frame_data_size = CAN_MAX_DATA_SIZE-1;

	if (len <= 0)
		return 0;

	/* calculate number of smaller CAN frames */
	if (len <= max_frame_data_size)
		f_num = 1;
	else {
		quotient = len / max_frame_data_size;
		remainder = len % max_frame_data_size;

		if (!remainder)
			f_num = quotient;
		else
			f_num = quotient + 1;

	}

	while(len > 0){

		to_write = (len <= max_frame_data_size) ? len : max_frame_data_size;

		sendbuf[0] = --f_num; // sendbuf[0] stores the frame number

		memcpy(&sendbuf[1],&buf[index],to_write); // sendbuf[1..7] store data

		frame_size = to_write + 1; //however, CAN frame size remain to_write + 1

		len -= to_write;

		index += to_write;

		CAN1_TX(sendbuf, frame_size);

	}

	 return len;
}

int CAN1_TX_MAILBOX_FREE_level(void)
{
	int i=0;
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
	{
		i++;
		if(i>0xfffe)
			return 1;
	}
	HAL_Delay(500);
	return 0;
}

void CAN1_TX_pending_wait(void) {

	while (HAL_CAN_IsTxMessagePending(&hcan1, (long int)&TxMailbox) != HAL_OK) {
		printf("\n\rHAL_CAN_IsTxMessagePending() failed\n\r");
		HAL_Delay(10);
	}

}

void initCAN2Message(void){
	int i;

	for (i=0; i<CAN2_MAX_ID_SIZE; i++){
		memset(&can2_messages[i], 0x0, sizeof(struct CAN2_Messaging_t));
	}
}

uint8_t isIDRegistered (uint32_t id) {
	int i;
	uint8_t idx = 0xFF;

	for (i = 0; i < CAN2_MAX_ID_SIZE; i++) {
		if (id == can2_messages[i].id) {
			idx = i;
			break;
		}
	}
	return idx;
}

uint8_t registerID(uint32_t id, uint8_t *fr, uint8_t fr_num, uint8_t len) {
	uint8_t i;
	uint8_t idx = 0xFF;

	idx = isIDRegistered(id);
	if (idx == 0xFF) { //has not registered
		for (i = 0; i < CAN2_MAX_ID_SIZE; i++) {
			if (0x0 == can2_messages[i].id) {
				can2_messages[i].id = id;
				can2_messages[i].fr_num = fr_num;
				can2_messages[i].length = 0;
				can2_messages[i].index = fr_num;
				addFrame(fr, len, i);
				idx = i;
				break;
			}
		}
	}
	else {
		addFrame(fr, len, idx);
	}
	return idx;
}


bool resetIDMessage(uint32_t id) {
	bool rv = false;
	uint8_t idx = 0xFF;

	idx =isIDRegistered(id);
	if (idx != 0xFF) {
		memset(&can2_messages[idx], 0x0, sizeof(struct CAN2_Messaging_t));
		rv = true;
	}
	return rv;
}

bool resetIDXMessage(uint8_t idx) {
	bool rv = false;
	if (idx != 0xFF) {
		memset(&can2_messages[idx], 0x0, sizeof(struct CAN2_Messaging_t));
		rv = true;
	}
	return rv;
}

uint8_t addFrame(uint8_t *fr, uint8_t len, uint8_t idx) {

	can2_messages[idx].index--;
	memcpy(&can2_messages[idx].buf[can2_messages[idx].length], &fr[1] ,len-1);
	can2_messages[idx].length += len-1;

	return len-1;
}

size_t retrieveFullMesg(uint8_t *msg, uint8_t idx) {

	size_t rv = can2_messages[idx].length;
	memcpy(msg, &can2_messages[idx].buf[0], can2_messages[idx].length);
	memset(&can2_messages[idx], 0x0, sizeof(struct CAN2_Messaging_t));

	return rv;
}

size_t IsLastFrame(uint8_t idx, uint8_t *msg, uint8_t fr_num){
	size_t msg_len = 0;

	if (!fr_num){
		msg_len = can2_messages[idx].length;
		memcpy(msg, &can2_messages[idx].buf[0], can2_messages[idx].length);
		//memset(&can2_messages[idx], 0x0, sizeof(struct CAN2_Messaging_t));
	}

	return msg_len;
}


uint32_t processInFrame(uint32_t id, uint8_t *fr, uint8_t fr_num, uint32_t len) {

	uint8_t idx = 0xFF;
	uint32_t msg_len = 0;

	idx = registerID(id, fr, fr_num, len);  //fr bring data in

	msg_len = IsLastFrame(idx, fr, fr_num); //fr bring data out

	return msg_len;
}



static void Error_Handler(char * str)
{
	while (1) {
		printf("\n\r%s Failed \n\r", str);
		HAL_Delay(1000);
	}
}

