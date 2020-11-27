#include <stdint.h> // uint8_t
#include <stddef.h> // size_t
#include <stdbool.h> // bool
#include "CAN_utils.h"
//#include "circularByteBuffer.h"
#include "stm32f4xx_hal.h"
/*
* Little-endian e.g., 0x87654321 0x87 = a [3],.. 0x21 = a [0]  
*/
#define DISABLE_CAN1_TX_PENDING_WAIT

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
	return circularByteBuffer_Reset(&cb_han);

}


int32_t cb_read_CAN_frame(uint8_t *arr)
{

	uint32_t cb_element;
	int32_t len;
	uint8_t arr_val [4];

	printf("cb_read_CAN_frame() entered \n");
	if (circularByteBuffer_Count(&cb_han) >= CAN1_FRAME_SIZE)
	{

#if 1
		//read std_ID
		CANreadElement(&cb_han, &cb_element);
		printf("CAN std_ID, 0x%x \n", (unsigned int)cb_element);


		//read CAN DLC
		CANreadElement(&cb_han, &cb_element);
		printf("CAN DLC, 0x%x \n", (unsigned int)cb_element);
		len = cb_element;


		//read CAN DATA
		CANreadPayload(&cb_han, arr_val,  len);  //cb_element = CAN DLC
		printf("CAN DATA, 0x%x 0x%x 0x%x 0x%x \n", (unsigned int)arr_val[0] ,(unsigned int)arr_val[1] ,(unsigned int)arr_val[2] ,(unsigned int)arr_val[3]);
		arr[0] = arr_val[0];
		arr[1] = arr_val[1];
		arr[2] = arr_val[2];
		arr[3] = arr_val[3];

#else
		int i = 0;
		//read std_ID
		for (i=0; i<4; i++)
		{
			 circularByteBuffer_Dequeue(&cb_han, &arr_val[i]);
		}

		printf("CAN std_ID, 0x%x 0x%x 0x%x 0x%x \n", arr_val[3] ,arr_val[2] ,arr_val[1] ,arr_val[0]);

		//read DLC
		for (i=0; i<4; i++)
		{
			arr_val[i] = circularByteBuffer_Dequeue(&cb_han, &arr_val[i]);
		}

		printf("CAN DLC, 0x%x 0x%x 0x%x 0x%x \n", arr_val[3] ,arr_val[2] ,arr_val[1] ,arr_val[0]);

		//read data
		for (i=0; i<4; i++)
		{
			arr_val[i] = circularByteBuffer_Dequeue(&cb_han, &arr_val[i]);
		}

		printf("CAN DATA, 0x%x 0x%x 0x%x 0x%x \n", arr_val[3] ,arr_val[2] ,arr_val[1] ,arr_val[0]);
#endif
	}

	return len;

}


void circularByteBuffer_Int2Bytes(uint32_t num, uint8_t *a)
{
 
  a[3] = (num>>24) & 0xFF;
  printf(" num 0x%x a[0] = %x", (unsigned int)num, (unsigned int)a[3]);

  a[2] = (num>>16) & 0xFF;
  printf(" num 0x%x a[0] = %x", (unsigned int)num, (unsigned int)a[2]);

  a[1] = (num>>8) & 0xFF;
  printf(" num 0x%x a[0] = %x", (unsigned int)num, (unsigned int)a[1]);

  a[0] = num & 0xFF;
  printf(" num 0x%x a[0] = %x", (unsigned int)num, (unsigned int)a[0]);

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

void CAN1_TX(void)
{
	uint8_t txData[8] = {0xA5,0x01,0x02,0x03,0x04,0x05,0x06,0x07};

	if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, (uint32_t*)&TxMailbox) != HAL_OK) {
		//printf("HAL_CAN_AddTxMessage() Failed \n");
		Error_Handler("CAN1_TX");

	}
	 printf("HAL_CAN_AddTxMessage() Passed \n");

#ifndef DISABLE_CAN1_TX_PENDING_WAIT
	CAN1_TX_pending_wait();
#endif
}


void CAN1_TX_pending_wait(void) {

	while (HAL_CAN_IsTxMessagePending(&hcan1, (long int)&TxMailbox) != HAL_OK) {
		printf("HAL_CAN_IsTxMessagePending() failed\n");
		HAL_Delay(10);
	}

}

static void Error_Handler(char * str)
{
	while (1) {
		printf("%s Failed \n", str);
		HAL_Delay(1000);
	}
}

