/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../lan9252/lan9252.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
PROCBUFFER_OUT 	BufferOut;
PROCBUFFER_IN 	BufferIn;

spiCTX ethercat_slave;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// joint 1-3 in can1
FDCAN_TxHeaderTypeDef joint_1;
FDCAN_TxHeaderTypeDef joint_2;
FDCAN_TxHeaderTypeDef joint_3;
// joint 4-6 in can2
FDCAN_TxHeaderTypeDef joint_4;
FDCAN_TxHeaderTypeDef joint_5;
FDCAN_TxHeaderTypeDef joint_6;

FDCAN_TxHeaderTypeDef joint_encoder;

uint8_t tx_msg_buffer[8];

uint8_t joint_1_data[8];
uint8_t joint_2_data[8];
uint8_t joint_3_data[8];
uint8_t joint_4_data[8];
uint8_t joint_5_data[8];
uint8_t joint_6_data[8];
uint8_t joint_encoder_data[8];

uint64_t joint_r_data[6];

FDCAN_RxHeaderTypeDef rx_header;
uint8_t rx_data[8];

// for ethercat var
uint64_t hs_ = 0;

union Byte8
{
	uint64_t udata;
	uint8_t buffer[8];
};

union Byte8 byte_8;
union Byte8 byte_8_reply;
uint64_t reply_hs[6];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim4, 0); 
	while (__HAL_TIM_GET_COUNTER(&htim4) < us) ;
}

void unpack_reply(FDCAN_RxHeaderTypeDef *pRxHeader, uint8_t *data)
{
	//TODO 
	/*if (pRxHeader->DataLength == FDCAN_DLC_BYTES_6)
	{
		int id = data[0] & 0xF;
		if (id > 0 && id < 7)
		{
			for (size_t i = 0; i < 6; i++)
			{
				byte_8_reply.buffer[i] = data[i];
			}
			
			joint_r_data[id - 1] = byte_8_reply.udata;
			reply_hs[id - 1] = hs_;
		}
	}
	else if (pRxHeader->DataLength == FDCAN_DLC_BYTES_5)
	{
		int id = data[4];
		if (id > 0 && id < 7)
		{
			for (size_t i = 0; i < 5; i++)
			{
				byte_8_reply.buffer[i] = data[i];
			}
			
			for (size_t i = 5; i < 8; i++)
			{
				byte_8_reply.buffer[i] = 0;
			}
			
			
			joint_r_data[id - 1] = byte_8_reply.udata;
			reply_hs[id - 1] = hs_;
		}
	}*/
	
	if (pRxHeader->DataLength == FDCAN_DLC_BYTES_8)
	{
		int id = pRxHeader->Identifier;
		if (id > 0 && id < 7)
		{
			for (size_t i = 0; i < 8; i++)
			{
				byte_8_reply.buffer[i] = data[i];
			}
			
			joint_r_data[id - 1] = byte_8_reply.udata;
			reply_hs[id - 1] = hs_;
		}
	}
	else if (pRxHeader->DataLength == FDCAN_DLC_BYTES_6)
	{
		int id = data[0] & 0xF;
		if (id > 0 && id < 7)
		{
			for (size_t i = 0; i < 6; i++)
			{
				byte_8_reply.buffer[i] = data[i];
			}
			
			joint_r_data[id - 1] = byte_8_reply.udata;
			reply_hs[id - 1] = hs_;
		}
	}
	
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void motor_enable(FDCAN_TxHeaderTypeDef* joint_tx, uint8_t* data_buffer)
{
	joint_tx->DataLength = FDCAN_DLC_BYTES_8;
	data_buffer[0] = 0xff;
	data_buffer[1] = 0xff;
	data_buffer[2] = 0xff;
	data_buffer[3] = 0xff;
	data_buffer[4] = 0xff;
	data_buffer[5] = 0xff;
	data_buffer[6] = 0xff;
	data_buffer[7] = 0xfc;
}

void motor_zero(FDCAN_TxHeaderTypeDef* joint_tx, uint8_t* data_buffer)
{
	joint_tx->DataLength = FDCAN_DLC_BYTES_8;
	data_buffer[0] = 0xff;
	data_buffer[1] = 0xff;
	data_buffer[2] = 0xff;
	data_buffer[3] = 0xff;
	data_buffer[4] = 0xff;
	data_buffer[5] = 0xff;
	data_buffer[6] = 0xff;
	data_buffer[7] = 0xfe;
}

void motor_disable(FDCAN_TxHeaderTypeDef* joint_tx, uint8_t* data_buffer)
{
	joint_tx->DataLength = FDCAN_DLC_BYTES_8;
	data_buffer[0] = 0xff;
	data_buffer[1] = 0xff;
	data_buffer[2] = 0xff;
	data_buffer[3] = 0xff;
	data_buffer[4] = 0xff;
	data_buffer[5] = 0xff;
	data_buffer[6] = 0xff;
	data_buffer[7] = 0xfd;
}

void motor_encoder_val(FDCAN_TxHeaderTypeDef* joint_tx, uint8_t* data_buffer, uint8_t joint_no)
{
	joint_tx->DataLength = FDCAN_DLC_BYTES_8;
	data_buffer[0] = joint_no;
	data_buffer[1] = 0xff;
	data_buffer[2] = 0xff;
	data_buffer[3] = 0xff;
	data_buffer[4] = 0xff;
	data_buffer[5] = 0xff;
	data_buffer[6] = 0xff;
	data_buffer[7] = 0xf8;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	
	//1. init tx msg
	joint_1.Identifier = 0x1;
	joint_1.IdType = FDCAN_STANDARD_ID;
	joint_1.TxFrameType = FDCAN_DATA_FRAME;
	joint_1.DataLength = FDCAN_DLC_BYTES_8;
	joint_1.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	joint_1.BitRateSwitch = FDCAN_BRS_OFF;
	joint_1.FDFormat = FDCAN_CLASSIC_CAN;
	joint_1.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	joint_1.MessageMarker = 0;
	
	joint_2.Identifier = 0x2;
	joint_2.IdType = FDCAN_STANDARD_ID;
	joint_2.TxFrameType = FDCAN_DATA_FRAME;
	joint_2.DataLength = FDCAN_DLC_BYTES_8;
	joint_2.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	joint_2.BitRateSwitch = FDCAN_BRS_OFF;
	joint_2.FDFormat = FDCAN_CLASSIC_CAN;
	joint_2.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	joint_2.MessageMarker = 0;
	
	joint_3.Identifier = 0x3;
	joint_3.IdType = FDCAN_STANDARD_ID;
	joint_3.TxFrameType = FDCAN_DATA_FRAME;
	joint_3.DataLength = FDCAN_DLC_BYTES_8;
	joint_3.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	joint_3.BitRateSwitch = FDCAN_BRS_OFF;
	joint_3.FDFormat = FDCAN_CLASSIC_CAN;
	joint_3.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	joint_3.MessageMarker = 0;
	
	joint_4.Identifier = 0x4;
	joint_4.IdType = FDCAN_STANDARD_ID;
	joint_4.TxFrameType = FDCAN_DATA_FRAME;
	joint_4.DataLength = FDCAN_DLC_BYTES_8;
	joint_4.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	joint_4.BitRateSwitch = FDCAN_BRS_OFF;
	joint_4.FDFormat = FDCAN_CLASSIC_CAN;
	joint_4.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	joint_4.MessageMarker = 0;
	
	joint_5.Identifier = 0x5;
	joint_5.IdType = FDCAN_STANDARD_ID;
	joint_5.TxFrameType = FDCAN_DATA_FRAME;
	joint_5.DataLength = FDCAN_DLC_BYTES_8;
	joint_5.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	joint_5.BitRateSwitch = FDCAN_BRS_OFF;
	joint_5.FDFormat = FDCAN_CLASSIC_CAN;
	joint_5.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	joint_5.MessageMarker = 0;
	
	joint_6.Identifier = 0x6;
	joint_6.IdType = FDCAN_STANDARD_ID;
	joint_6.TxFrameType = FDCAN_DATA_FRAME;
	joint_6.DataLength = FDCAN_DLC_BYTES_8;
	joint_6.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	joint_6.BitRateSwitch = FDCAN_BRS_OFF;
	joint_6.FDFormat = FDCAN_CLASSIC_CAN;
	joint_6.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	joint_6.MessageMarker = 0;
	
	joint_encoder.Identifier = 0x7FF;
	joint_encoder.IdType = FDCAN_STANDARD_ID;
	joint_encoder.TxFrameType = FDCAN_DATA_FRAME;
	joint_encoder.DataLength = FDCAN_DLC_BYTES_8;
	joint_encoder.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	joint_encoder.BitRateSwitch = FDCAN_BRS_OFF;
	joint_encoder.FDFormat = FDCAN_CLASSIC_CAN;
	joint_encoder.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	joint_encoder.MessageMarker = 0;
	
	HAL_FDCAN_Start(&hfdcan1);
	HAL_FDCAN_Start(&hfdcan2);
	
	HAL_Delay(10);
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start(&htim4);
	
	HAL_Delay(10);

	// 1. init lan9252
	ethercat_slave.spi = &hspi1;
	ethercat_slave.uart = &huart2;
	ethercat_slave.bIn = &BufferIn;
	ethercat_slave.bOut = &BufferOut;
	HAL_Delay(10);
	
	init9252(&ethercat_slave);
	
	HAL_Delay(100);
	
	HAL_TIM_Base_Start_IT(&htim5);
	
	HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
	  {
		  unpack_reply(&rx_header, rx_data);
	  }
	  if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
	  {
		  unpack_reply(&rx_header, rx_data);
	  }
	  delay_us(10);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 8;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 10;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 8;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 10;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 12;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 12;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = ENABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 8;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 10;
  hfdcan2.Init.NominalTimeSeg2 = 4;
  hfdcan2.Init.DataPrescaler = 8;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 10;
  hfdcan2.Init.DataTimeSeg2 = 4;
  hfdcan2.Init.MessageRAMOffset = 1024;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.RxFifo0ElmtsNbr = 12;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 12;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 239;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 239;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 239;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CSS_GPIO_Port, CSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CSS_Pin */
  GPIO_InitStruct.Pin = CSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CSS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


uint32_t can1_error_counter = 0;
uint32_t can2_error_counter = 0;
uint32_t can1_last_error_code = 0;
uint32_t can2_last_error_code = 0;

uint16_t control_word;

int is_enable = 0;
int motor_init_state = 0;

void send_to_all_slave()
{
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &joint_1, joint_1_data) != HAL_OK)
	{
		can1_error_counter += 1;
	}
		
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &joint_2, joint_2_data) != HAL_OK)
	{
		can1_error_counter += 1;
	}
		
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &joint_3, joint_3_data) != HAL_OK)
	{
		can1_error_counter += 1;
	}
		
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &joint_4, joint_4_data) != HAL_OK)
	{
		can2_error_counter += 1;
	}
		
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &joint_5, joint_5_data) != HAL_OK)
	{
		can2_error_counter += 1;
	}
		
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &joint_6, joint_6_data) != HAL_OK)
	{
		can2_error_counter += 1;
	}
}

void send_to_joint(int joint_no)
{
	if (joint_no == 1 || joint_no == 2 || joint_no == 3)
	{
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &joint_encoder, joint_encoder_data) != HAL_OK)
		{
			can1_error_counter += 1;
		}
	}
	else if (joint_no == 4 || joint_no == 5 || joint_no == 6)
	{
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &joint_encoder, joint_encoder_data) != HAL_OK)
		{
			can1_error_counter += 1;
		}
	}
}

void pack_motor_data()
{
	byte_8.udata = BufferOut.Cust.motor_1;
	for (size_t i = 0; i < 8; i++)
	{
		joint_1_data[i] = byte_8.buffer[i];
	}
	
	byte_8.udata = BufferOut.Cust.motor_2;
	for (size_t i = 0; i < 8; i++)
	{
		joint_2_data[i] = byte_8.buffer[i];
	}
	
	byte_8.udata = BufferOut.Cust.motor_3;
	for (size_t i = 0; i < 8; i++)
	{
		joint_3_data[i] = byte_8.buffer[i];
	}
	
	byte_8.udata = BufferOut.Cust.motor_4;
	for (size_t i = 0; i < 8; i++)
	{
		joint_4_data[i] = byte_8.buffer[i];
	}
	
	byte_8.udata = BufferOut.Cust.motor_5;
	for (size_t i = 0; i < 8; i++)
	{
		joint_5_data[i] = byte_8.buffer[i];
	}
	
	byte_8.udata = BufferOut.Cust.motor_6;
	for (size_t i = 0; i < 8; i++)
	{
		joint_6_data[i] = byte_8.buffer[i];
	}
}

void control()
{
	/*int is_init = 0;
	
	
	if (control_word == 3 && is_enable == 0)
	{
		// send enable cmd
		motor_encoder_val(&joint_encoder, joint_encoder_data, 1);
		send_to_joint(1);
		motor_encoder_val(&joint_encoder, joint_encoder_data, 2);
		send_to_joint(2);
		motor_encoder_val(&joint_encoder, joint_encoder_data, 3);
		send_to_joint(3);
		motor_encoder_val(&joint_encoder, joint_encoder_data, 4);
		send_to_joint(4);
		motor_encoder_val(&joint_encoder, joint_encoder_data, 5);
		send_to_joint(5);
		motor_encoder_val(&joint_encoder, joint_encoder_data, 6);
		send_to_joint(6);
	}
	
	if (control_word == 2 && is_enable == 1)
	{
		//safe torque off
		//TODO:
		motor_disable(&joint_1, joint_1_data);
		motor_disable(&joint_2, joint_2_data);
		motor_disable(&joint_3, joint_3_data);
		motor_disable(&joint_4, joint_4_data);
		motor_disable(&joint_5, joint_5_data);
		motor_disable(&joint_6, joint_6_data);
		
		send_to_all_slave();
		
	}
	if (control_word == 1 && is_enable == 0)
	{
		// send enable cmd
		motor_enable(&joint_1, joint_1_data);
		motor_enable(&joint_2, joint_2_data);
		motor_enable(&joint_3, joint_3_data);
		motor_enable(&joint_4, joint_4_data);
		motor_enable(&joint_5, joint_5_data);
		motor_enable(&joint_6, joint_6_data);
		
		send_to_all_slave();
		
		is_enable = 1;
		motor_init_state = 1;
		is_init = 1;
	}
	if (is_init)
	{
		return;
	}
	
	if (motor_init_state == 1)
	{
		motor_init_state = 0;
	}*/
	
	if (control_word == 1)
	{
		pack_motor_data();
		send_to_all_slave();
	}
}

uint16_t get_motor_status()
{
	int motor_msg[6];
	
	uint16_t motor_status_ = 0;
	
	for (size_t i = 0; i < 6; i++)
	{
		if ((reply_hs[i] + 10) < hs_)
		{
			motor_msg[i] = 0;
		}
		else
		{
			motor_msg[i] = 1;
			motor_status_ |= 1 << i;
		}
	}
	return motor_status_;
}

void pack_hs_loss_data(int slave_no, uint64_t data)
{
	uint64_t bit_offset = ((slave_no - 1) * 8);
	uint64_t tmp_one = 1;
	BufferIn.Cust.hs_record = BufferIn.Cust.hs_record & (~(tmp_one << (bit_offset)));
	BufferIn.Cust.hs_record = BufferIn.Cust.hs_record & (~(tmp_one << (bit_offset + 1)));
	BufferIn.Cust.hs_record = BufferIn.Cust.hs_record & (~(tmp_one << (bit_offset + 2)));
	BufferIn.Cust.hs_record = BufferIn.Cust.hs_record & (~(tmp_one << (bit_offset + 3)));
	BufferIn.Cust.hs_record = BufferIn.Cust.hs_record & (~(tmp_one << (bit_offset + 4)));
	BufferIn.Cust.hs_record = BufferIn.Cust.hs_record & (~(tmp_one << (bit_offset + 5)));
	BufferIn.Cust.hs_record = BufferIn.Cust.hs_record & (~(tmp_one << (bit_offset + 6)));
	BufferIn.Cust.hs_record = BufferIn.Cust.hs_record & (~(tmp_one << (bit_offset + 7)));
	BufferIn.Cust.hs_record = (BufferIn.Cust.hs_record | (data << bit_offset));
}

void pack_hs_data()
{
	for (size_t i = 0; i < 6; i++)
	{
		uint64_t tmp = hs_ - reply_hs[i];
		
		if (tmp > 250)
		{
			tmp = 250;
		}
		
		pack_hs_loss_data(i, tmp);
	
	}
}

void pack_ethercat_data()
{
	BufferIn.Cust.hs = hs_;
	
	BufferIn.Cust.motor_1 = joint_r_data[0];
	BufferIn.Cust.motor_2 = joint_r_data[1];
	BufferIn.Cust.motor_3 = joint_r_data[2];
	BufferIn.Cust.motor_4 = joint_r_data[3];
	BufferIn.Cust.motor_5 = joint_r_data[4];
	BufferIn.Cust.motor_6 = joint_r_data[5];
	
	BufferIn.Cust.can1_error_log = can1_error_counter;
	BufferIn.Cust.can2_error_log = can2_error_counter;
	
	BufferIn.Cust.rec_error_can1 = (uint16_t)can1_last_error_code;
	BufferIn.Cust.rec_error_can2 = (uint16_t)can2_last_error_code;
	
	// pack hs data
	pack_hs_data();
	
	//error protect
	//BufferIn.Cust.motor_status = get_motor_status();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM5)
	{
		
		pack_ethercat_data();
		main_task(&ethercat_slave);
		
		uint64_t tmp_hs_ = BufferOut.Cust.hs;
		
		// 1. control
		if (tmp_hs_ > hs_ || tmp_hs_ == 1)
		{
			control_word = BufferOut.Cust.control_word;
			
			if (tmp_hs_ == 1)
			{
				is_enable = 0;
			}
			
			control();
			
			hs_ = tmp_hs_;
			
		}
		
		
		// error check
		can1_last_error_code = READ_REG(hfdcan1.Instance->PSR);	
		can1_last_error_code = can1_last_error_code & 0x0007;
		
		can2_last_error_code = READ_REG(hfdcan2.Instance->PSR);	
		can2_last_error_code = can2_last_error_code & 0x0007;
		
		FDCAN_ErrorCountersTypeDef ErrorCounters;
		uint8_t error_counter1;
		uint8_t error_counter2;
		uint8_t error_counter3;
		uint8_t error_counter4;
		HAL_FDCAN_GetErrorCounters(&hfdcan1, &ErrorCounters);
		error_counter1 = (uint8_t)ErrorCounters.RxErrorCnt;
		error_counter2 = (uint8_t)ErrorCounters.TxErrorCnt; 
		
		can1_error_counter += error_counter1;
		can1_error_counter += error_counter2;
		
		HAL_FDCAN_GetErrorCounters(&hfdcan2, &ErrorCounters);
		error_counter3 = (uint8_t)ErrorCounters.RxErrorCnt;
		error_counter4 = (uint8_t)ErrorCounters.TxErrorCnt;
		
		can2_error_counter += error_counter3;
		can2_error_counter += error_counter4;
	}
}

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
