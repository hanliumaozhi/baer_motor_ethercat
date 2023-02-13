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

UART_HandleTypeDef huart8;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

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
union Byte8 contact_info;
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
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART8_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

void delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim4, 0); 
	while (__HAL_TIM_GET_COUNTER(&htim4) < us) ;
}

void unpack_reply(FDCAN_RxHeaderTypeDef *pRxHeader, uint8_t *data)
{
	//TODO 
	if (pRxHeader->DataLength == FDCAN_DLC_BYTES_8)
	{
		int id = data[0] & 0xF;
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

// contact sensor data

static uint8_t contact_sensor_1_buffer[64];
static uint8_t contact_sensor_1_raw_data[64];
static uint8_t contact_sensor_1_data[32];

int cs1_raw_counter = 0;
int cs1_buffer_index = 0;
int cs1_index_remain = 0;

volatile int cs1_data_available = 0;

#define CS_DATA_LEN 6

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_UART8_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
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
	
	HAL_UART_Receive_DMA(&huart1, contact_sensor_1_buffer, CS_DATA_LEN);

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
  hfdcan1.Init.AutoRetransmission = DISABLE;
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
  hfdcan1.Init.RxFifo0ElmtsNbr = 6;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 6;
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
  hfdcan2.Init.AutoRetransmission = DISABLE;
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
  hfdcan2.Init.RxFifo0ElmtsNbr = 6;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 6;
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
  * @brief UART8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart8.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart8, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart8, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */

  /* USER CODE END UART8_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();

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
	int is_init = 0;
	
	
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
		motor_zero(&joint_1, joint_1_data);
		motor_zero(&joint_2, joint_2_data);
		motor_zero(&joint_3, joint_3_data);
		motor_zero(&joint_4, joint_4_data);
		motor_zero(&joint_5, joint_5_data);
		motor_zero(&joint_6, joint_6_data);
		send_to_all_slave();
		motor_init_state = 0;
	}
	
	if (control_word == 1 && is_enable)
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
	
	//error protect
	BufferIn.Cust.motor_status = get_motor_status();
	
	// contact sensor data
	if (cs1_data_available == 1)
	{
		// for cs1
		contact_info.buffer[0] = contact_sensor_1_data[1];
		contact_info.buffer[1] = contact_sensor_1_data[2];
		contact_info.buffer[2] = contact_sensor_1_data[3];
		contact_info.buffer[3] = contact_sensor_1_data[4];
		cs1_data_available = 0;
	}
	
	BufferIn.Cust.test_word_byte_8 = contact_info.udata;
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


void usart_process_data(const void* data, size_t len) {
	const uint8_t* d = (uint8_t*)(data);
	for (int i = 0; i != len; ++i) {
		contact_sensor_1_raw_data[cs1_raw_counter++] = d[i];
	}
	if (cs1_raw_counter >= CS_DATA_LEN) {
		for (cs1_buffer_index = 0; cs1_buffer_index < (cs1_raw_counter - 1); ++cs1_buffer_index) {
			if (contact_sensor_1_raw_data[cs1_buffer_index] == 0xFE) {
				break;
			}
		}
		if (cs1_buffer_index != (cs1_raw_counter - 1)) {
			if ((cs1_raw_counter - cs1_buffer_index) >= CS_DATA_LEN) {
				if ((cs1_raw_counter - cs1_buffer_index) >= CS_DATA_LEN) {
					memcpy(contact_sensor_1_data, &contact_sensor_1_raw_data[cs1_buffer_index], CS_DATA_LEN);
					cs1_index_remain = (cs1_raw_counter - cs1_buffer_index - CS_DATA_LEN);
					for (int i = 0; i != cs1_index_remain; ++i) {
						contact_sensor_1_raw_data[i] = contact_sensor_1_raw_data[(cs1_buffer_index + CS_DATA_LEN + i)];
					}
					cs1_raw_counter = cs1_index_remain;
					cs1_data_available = 1;
				}
			}
		}
			
	}
		
	if (cs1_raw_counter >= 64) {
		cs1_raw_counter = 64;
	}
}


void usart_rx_check(void) {
	static  size_t old_pos;
	size_t pos;
	
	pos = CS_DATA_LEN - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
	if (pos != old_pos) {
		if (pos > old_pos) {
			usart_process_data(&contact_sensor_1_buffer[old_pos], pos - old_pos);
		}
		else {
			/* We are in "overflow" mode */
		   /* First process data to the end of buffer */
			usart_process_data(&contact_sensor_1_buffer[old_pos], CS_DATA_LEN - old_pos);
			/* Check and continue with beginning of buffer */
			if (pos > 0) {
				usart_process_data(&contact_sensor_1_buffer[0], pos);
			}
		}
	}
	
	old_pos = pos;
	if (old_pos == CS_DATA_LEN) {
		old_pos = 0;
	}
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		usart_rx_check();
	}
	/*else if (huart->Instance == UART4)
	{
		sbus_usart_rx_check();
	}*/
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
	if (huart->Instance == USART1)
	{
		usart_rx_check();
		HAL_UART_Receive_DMA(&huart1, contact_sensor_1_buffer, CS_DATA_LEN); 
	}
	/*else if (huart->Instance == UART4)
	{
		sbus_usart_rx_check();
		HAL_UART_Receive_DMA(&huart4, sbus_data_buffer, SBUS_DATA_LEN); 
	}*/
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
