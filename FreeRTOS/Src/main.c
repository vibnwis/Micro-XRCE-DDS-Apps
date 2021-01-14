/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h" 
#include "task.h" 
#include "app.h"
#include "common.h"
#include "CAN_utils.h"
#include "circularByteBuffer.h"
#include "retarget.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//#define		USART3_PRINTF
#define 	RTOS_MODE
//#define 	CAN1_LOOPBACK_MODE
//#define     CAN1_TX_ONLY
//#define     CAN1_RX_ONLY
#define 	CAN1_ENABLE

#define		DISABLE_CAN1_TX_PENDING_WAIT

void ISR_Error_Handler(void);
#ifdef CAN1_ENABLE
void CAN1_RX_Config(void);
void CAN1_TX_Config(void);

void ms_delay(uint32_t delay);
void CAN1_RX_polling(void);
//void CAN1_TX(void);
//void cb_read_CAN_frame(circularByteBuffer_t *cb);
#endif
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

bool can1_ready = RESET;
int count = 0;
uint8_t can1_buf[8] = {0xA1,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7,0xA8};

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* Definitions for initTask */
osThreadId_t initTaskHandle;
const osThreadAttr_t initTask_attributes = {
  .name = "initTask",
  .priority = (osPriority_t) osPriorityBelowNormal7,
  .stack_size = 1500
};
/* USER CODE BEGIN PV */
#ifdef CAN1_ENABLE
CAN_TxHeaderTypeDef txHeader; //declare a specific header for message transmittions
CAN_RxHeaderTypeDef rxHeader; //declare header for message reception
uint32_t TxMailbox, len;
uint8_t a,r, arr[12]; //declare byte to be transmitted //declare a receive byte

CAN_FilterTypeDef sFilterConfig; //declare CAN filter structure
circularByteBuffer_t cb_han;

#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_CAN1_Init(void);
void initTaskFunction(void *argument);

/* USER CODE BEGIN PFP */
extern struct netif gnetif; 
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  circularByteBuffer_Init(&cb_han);

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  //  printf("HAL_Init() passed\n");
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
//  printf("SystemClock_Config() passed\n");
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_LWIP_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();


  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  RetargetInit(&huart3);  // Assign uart3 for printf() to external ttyUSB* terminal
#ifdef CAN1_ENABLE
  printf("MX_CAN1_Init() passed\n");

  CAN1_TX_Config();
  printf("CAN1_TX_Config() passed\n");

  CAN1_RX_Config();
  printf("CAN1_RX_Config() passed\n");

#endif

#if !defined (CAN1_TX_ONLY) && !defined (CAN1_RX_ONLY)


  /* USER CODE END 2 */


  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of initTask */
  initTaskHandle = osThreadNew(initTaskFunction, NULL, &initTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#endif

  while (1)
  {

#if defined (CAN1_LOOPBACK_MODE)
	  CAN1_TX(can1_buf, 8);
	  printf("loop %d\n\r", ++count);  //test the SWV, but no luck

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_Delay(1000);

	  //CAN1_RX_polling();

	  //Read back from the circular buffer
	  len = cb_read_CAN_frame(arr);
#endif
	  HAL_Delay(50);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

	/* PCLK2 = 168MHz / 4 = 42MHz
	* Prescaler = 12;
	* TimeSeg1 = CAN_BS1_11TQ;
	* TimeSeg2 = CAN_BS2_2TQ;
	* to create 250Kb/s CAN bitrate
	*/
  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 12;
#if defined (CAN1_LOOPBACK_MODE)
  hcan1.Init.Mode = CAN_MODE_LOOPBACK;
#else
  hcan1.Init.Mode = CAN_MODE_NORMAL;
#endif
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  can1_ready = SET;
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  /* Output a message on Hyperterminal using printf function */
    printf("\n\r UART3 init passed\n\r");
  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */
  printf("\n\r UART6 init passed\n\r");
  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PG6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
#ifdef CAN1_ENABLE

void CAN1_TX_Config(void)
{
	txHeader.DLC=8;  			// give message size of 8 byte FOR NOW
	txHeader.IDE=CAN_ID_STD; 	//set identifier to standard
	txHeader.RTR=CAN_RTR_DATA; 	//set data type to remote transmission request?
	txHeader.StdId=0x123; 		//define a standard identifier, used for message identification by filters (switch this for the other microcontroller)


	txHeader.ExtId = 0x00;
	txHeader.TransmitGlobalTime = DISABLE;
	printf("\n\rCAN1_TX_Config() Passed \n\r");

}

void CAN1_RX_polling(void)
{
	uint8_t rxData[8];

//	while (HAL_CAN_GetRxFifoFillLevel(&hcan1,CAN_RX_FIFO0) != HAL_OK);
//	printf("HAL_CAN_GetRxFifoFillLevel() Passed \n");

	if (HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
		printf("\n\rHAL_CAN_GetRxMessage() Failed \n\r");
			Error_Handler();

	}

	 printf("\n\rHAL_CAN_GetRxMessage() Passed %s\n\r", rxData);

	if ((rxHeader.IDE == CAN_ID_STD) && (rxHeader.StdId == 0x321) && (rxHeader.DLC == 4))
	{
		printf("\n\rRdDataLength %d , RxData 0x%x\n\r", (int)(rxHeader.DLC>> 16), (unsigned int)rxData[0]);
	}


}



void CAN1_RX_Config(void)
{
		//filter one (stack light blink)
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0; //set fifo assignment
	sFilterConfig.FilterIdHigh=0x0;      //245<<5; //the ID that the filter looks for (switch this for the other microcontroller)
	sFilterConfig.FilterIdLow=0x0;
	sFilterConfig.FilterMaskIdHigh=0x0;
	sFilterConfig.FilterMaskIdLow=0x0;
	sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterActivation=ENABLE;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) //configure CAN filter
	{
		printf("\n\rHAL_CAN_ConfigFilter() failed \n\r");
		 Error_Handler();
	}
	printf("\n\rHAL_CAN_ConfigFilter() passed \n\r");

	HAL_CAN_Start(&hcan1); //start CAN
	printf("HAL_CAN_Start() passed \n\r");

	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //enable interrupts
	printf("\n\rHAL_CAN_ActivateNotification() passed \n\r");
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rxHeader;
	uint8_t rcvd_msg[8];
	uint8_t arr_val [4];
	int i = 0;

	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	printf("\n\rHAL_CAN_RxFifo0MsgPendingCallback() entered \n\r");
	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader,rcvd_msg) != HAL_OK)
	{
		printf("\n\rHAL_CAN_RxFifo Callback - HAL_CAN_GetRxMessage() Failed\n\r");
		ISR_Error_Handler();
	}

	if ((rxHeader.IDE == CAN_ID_STD)) // && (rxHeader.StdId == 0x321))
	{
		//printf("\n\rID = 0x%x RdDataLength %d , RxData 0x%x\n\r", (int)(rxHeader.DLC), (unsigned int)rcvd_msg[0]);
		printf("\n\r ID = 0x%x RdDataLength %d \n\r", (int)rxHeader.StdId, (int)(rxHeader.DLC));
		/* print received data */
		for (i=0; i< rxHeader.DLC; i++)
		{
			printf("Data[%d]=0x%x ", i, (int)rcvd_msg[i]);
		}

		printf("\n\r");
	}

	/*
	 * Store the CAN frame in this structure only
	 * i.e., | rxHeader.StdId (4 bytes) | rxHeader.DLC (4 bytes) | data (4 bytes) |
	 *
	 * Note: the term "element" depicts data type of uint32_t
	 */

	EnqueueFrame(rxHeader.StdId, rxHeader.DLC, rcvd_msg);

#if 0
	/* convert rxHeader.StdId to bytes */
	circularByteBuffer_Int2Bytes(rxHeader.StdId, arr_val);
	printf("\n\rEnqueue ID %x \n\r", (int)(rxHeader.DLC));
	/* Store rxHeader.StdId, i.e, arr_val in the circular buffer */
	circularByteBuffer_element_Enqueue(&cb_han, arr_val);


	/* convert rxHeader.DCL to bytes */
	circularByteBuffer_Int2Bytes(rxHeader.DLC, arr_val);
	printf("\n\rEnqueue RdDataLength %x \n\r", (int)(rxHeader.DLC));
	/* Store rxHeader.DCL in arr_val into the circular buffer */
	circularByteBuffer_element_Enqueue(&cb_han, arr_val);

	/*
	 *  It is for not fixed at 4 bytes for sending, hence receiving 4 bytes
	 */
	/* Store in the circular buffer */
	for (i=0; i< rxHeader.DLC; i++) {
		circularByteBuffer_Enqueue(&cb_han, (uint8_t)rcvd_msg[i]);
	}

#endif
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

}

/*
 * This is routine being use by the transport layer reading the frame contents. However, it can be used here for
 * confirming the operationg the routine. Test it in the main() while after receiving a frame from the RxFifo0 CALLBACK routine
 *
 */



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	printf("HAL_GPIO_EXTI_Callback() entered \n\r");
	if(GPIO_Pin == WK_Button_Pin)   //WK_Button_GPIO_Port WK_Button_Pin
	{
		printf("\n\rWK_Button_Pin pressed entered \n\r");

		//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		//CAN1_TX();
	}

}

#endif
/* USER CODE END 4 */

/* USER CODE BEGIN Header_initTaskFunction */
/**
  * @brief  Function implementing the initTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_initTaskFunction */
void initTaskFunction(void *argument)
{
  /* USER CODE BEGIN 5 */
	int count = 0;
	uint32_t id, length = 0;
#if defined (CAN1_ENABLE)

	printf("\n\rCAN1-bus %d - 1=Ready, 0=Off-line \r\n", can1_ready);

	// Launch app thread when IP configured

	osThreadAttr_t attributes;
    memset(&attributes, 0x0, sizeof(osThreadAttr_t));
	attributes.name = "microxrceddsapp";
	attributes.stack_size = 4*3000;
    attributes.priority = (osPriority_t) osPriorityNormal1;
	osThreadNew(appMain, NULL, &attributes);

	TaskHandle_t xHandle;
	xHandle = xTaskGetHandle("microxrceddsapp");

	while (1) {
		printf("\n\r Count %d", count++);

		if (eTaskGetState(xHandle) != eSuspended){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			osDelay(100);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			osDelay(100);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			osDelay(150);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
				       osDelay(500);
			}
		else{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			osDelay(1000);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			osDelay(1000);
		}

#if defined (CAN1_LOOPBACK_MODE)
		  //HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		 int32_t len;
		 uint8_t arr[12];
		 CAN1_TX(can1_buf, 8);
		  printf("\n\rloop %d\n\r", ++count);  //test the SWV, but no luck

		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	    /* USER CODE END WHILE */


	    /* USER CODE BEGIN 3 */
		  HAL_Delay(500);

		  //CAN1_RX_polling();

		  //Read back from the circular buffer
		  len = cb_read_CAN_frame(arr);
#endif
		  //Read back from the circular buffer
		  // len = cb_read_CAN_frame(arr);
		   len = cb_read_CAN_frame(arr, &id, &length);
	}


#else
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
	uint8_t availableNetwork = 0;

	printf("\n\rEthernet Initialization\r\n");

	//Waiting for an IP
	printf("Waiting for IP\r\n");
	int retries = 0;

	while(gnetif.ip_addr.addr == 0 && retries < 30){
		osDelay(500);
		retries++;
	};

	availableNetwork = (gnetif.ip_addr.addr != 0) ? 1 : 0;
	if (availableNetwork != 0){
		printf("\n\rIP: %s\r\n",ip4addr_ntoa(&gnetif.ip_addr));
	}
	else{
		printf("\n\rImpossible to retrieve an IP\n\r");
	}

  // Launch app thread when IP configured

  osThreadAttr_t attributes;
  memset(&attributes, 0x0, sizeof(osThreadAttr_t));
  attributes.name = "microxrceddsapp";
  attributes.stack_size = 4*3000;
  attributes.priority = (osPriority_t) osPriorityNormal1;
  osThreadNew(appMain, NULL, &attributes);

  TaskHandle_t xHandle;
  xHandle = xTaskGetHandle("microxrceddsapp");

  while (1){
    availableNetwork = (gnetif.ip_addr.addr != 0) ? 1 : 0;
    if (eTaskGetState(xHandle) != eSuspended && availableNetwork != 0){
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
      osDelay(100);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
      osDelay(100);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
      osDelay(150);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
      osDelay(500);
    }else{
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
      osDelay(1000);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
      osDelay(1000);
    }
  }
  
#endif

  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	while (1) {
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_Delay (30);
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_Delay (100);
	  }
}

void ISR_Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while (1) {
	  /*            @arg GPIO_PIN_RESET: to clear the port pin
	  *            @arg GPIO_PIN_SET: to set the port pin
	  */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
