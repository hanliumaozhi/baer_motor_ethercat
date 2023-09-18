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


FDCAN_TxHeaderTypeDef slave_1;
FDCAN_TxHeaderTypeDef slave_2;
FDCAN_TxHeaderTypeDef slave_3;
FDCAN_TxHeaderTypeDef slave_4;
FDCAN_TxHeaderTypeDef slave_5;
FDCAN_TxHeaderTypeDef slave_6;
FDCAN_TxHeaderTypeDef slave_7;
FDCAN_TxHeaderTypeDef slave_8;
FDCAN_TxHeaderTypeDef slave_9;
FDCAN_TxHeaderTypeDef slave_10;


uint8_t tx_msg_buffer[8];

uint8_t slave_1_data[8];
uint8_t slave_2_data[8];
uint8_t slave_3_data[8];
uint8_t slave_4_data[8];
uint8_t slave_5_data[8];
uint8_t slave_6_data[8];
uint8_t slave_7_data[8];
uint8_t slave_8_data[8];
uint8_t slave_9_data[8];
uint8_t slave_10_data[8];
uint8_t joint_encoder_data[8];

uint64_t joint_r_data[10];

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
uint64_t reply_hs[10];


// configure which can port
int slave_routing[10];

//out: ethercat master -> ethercat slave
uint64_t can_msg_length_bin_out = 0;
int can_msg_length_out[10];

//in: ethercat slave -> ethercat master
uint64_t can_msg_length_bin_in = 0;
int can_msg_length_in[10];
uint8_t can_id_list[10];

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

int get_data_len(uint32_t data_len_code)
{
	if (data_len_code == FDCAN_DLC_BYTES_0)
	{
		return 0;
	}
	else if (data_len_code == FDCAN_DLC_BYTES_1)
	{
		return 1;
	}
	else if (data_len_code == FDCAN_DLC_BYTES_2)
	{
		return 2;
	}
	else if (data_len_code == FDCAN_DLC_BYTES_3)
	{
		return 3;
	}
	else if (data_len_code == FDCAN_DLC_BYTES_4)
	{
		return 4;
	}
	else if (data_len_code == FDCAN_DLC_BYTES_5)
	{
		return 5;
	}
	else if (data_len_code == FDCAN_DLC_BYTES_6)
	{
		return 6;
	}
	else if (data_len_code == FDCAN_DLC_BYTES_7)
	{
		return 7;
	}
	else if (data_len_code == FDCAN_DLC_BYTES_8)
	{
		return 8;
	}
}

uint32_t get_data_len_code(uint8_t data_len)
{
	if (data_len == 0) return FDCAN_DLC_BYTES_0;
	if (data_len == 1) return FDCAN_DLC_BYTES_1;
	if (data_len == 2) return FDCAN_DLC_BYTES_2;
	if (data_len == 3) return FDCAN_DLC_BYTES_3;
	if (data_len == 4) return FDCAN_DLC_BYTES_4;
	if (data_len == 5) return FDCAN_DLC_BYTES_5;
	if (data_len == 6) return FDCAN_DLC_BYTES_6;
	if (data_len == 7) return FDCAN_DLC_BYTES_7;
	if (data_len == 8) return FDCAN_DLC_BYTES_8;
	return 0;
}

void can_slave_routing_unpack()
{
	for (int i = 0; i < 10; ++i) {
		slave_routing[i] = 0;
	}
	
	for (int i = 0; i < 10; ++i) {
		uint64_t can1 = BufferOut.Cust.can1_id;
		if (((can1 >> i) & 1) == 1) {
			slave_routing[i] = 1;
		}
	}
	for (int i = 0; i < 10; ++i) {
		uint64_t can2 = BufferOut.Cust.can2_id;
		if (((can2 >> i) & 1) == 1) {
			slave_routing[i] = 2;
		}
	}
}

//master -> slave
void can_msg_length_unpack()
{
	uint64_t can_msg_length_bin = BufferOut.Cust.can_length;
	for (int i = 0; i < 10; ++i) {
		int length_tmp = (int)((can_msg_length_bin >> ((i) * 4)) & 15);
		can_msg_length_out[i] = length_tmp;
	}
}

//master -> slave
void can_id_unpack()
{
	uint64_t can_id_list_1 = BufferOut.Cust.data_1;
	uint64_t tmp_ff = 0xff;
	for (int i = 0; i < 8; ++i) {
		uint8_t id_tmp = (uint8_t)((can_id_list_1 >> ((i) * 8)) & tmp_ff);
		can_id_list[i] = id_tmp;
	}
	
	uint64_t can_id_list_2 = BufferOut.Cust.data_2;
	
	uint8_t id_tmp_1 = (uint8_t)((can_id_list_2 >> ((0) * 8)) & tmp_ff);
	can_id_list[8] = id_tmp_1;
	
	uint8_t id_tmp_2 = (uint8_t)((can_id_list_2 >> ((1) * 8)) & tmp_ff);
	can_id_list[9] = id_tmp_2;
}

// slave -> master
void can_msg_length_pack(int slave_no, uint64_t length)
{
	// 
	uint64_t bit_offset = ((slave_no - 1) * 4);
	uint64_t tmp_one = 1;
	BufferIn.Cust.can_length = BufferIn.Cust.can_length & (~(tmp_one << (bit_offset)));
	BufferIn.Cust.can_length = BufferIn.Cust.can_length & (~(tmp_one << (bit_offset + 1)));
	BufferIn.Cust.can_length = BufferIn.Cust.can_length & (~(tmp_one << (bit_offset + 2)));
	BufferIn.Cust.can_length = BufferIn.Cust.can_length & (~(tmp_one << (bit_offset + 3)));
	BufferIn.Cust.can_length = (BufferIn.Cust.can_length | (length << bit_offset));
}

void reply_pack_1(int slave_no, uint32_t hs_reply)
{
	// 
	uint32_t bit_offset = ((slave_no - 1) * 8);
	uint32_t tmp_one = 1;
	BufferIn.Cust.rec_loss_time1 = BufferIn.Cust.rec_loss_time1 & (~(tmp_one << (bit_offset)));
	BufferIn.Cust.rec_loss_time1 = BufferIn.Cust.rec_loss_time1 & (~(tmp_one << (bit_offset + 1)));
	BufferIn.Cust.rec_loss_time1 = BufferIn.Cust.rec_loss_time1 & (~(tmp_one << (bit_offset + 2)));
	BufferIn.Cust.rec_loss_time1 = BufferIn.Cust.rec_loss_time1 & (~(tmp_one << (bit_offset + 3)));
	BufferIn.Cust.rec_loss_time1 = BufferIn.Cust.rec_loss_time1 & (~(tmp_one << (bit_offset + 4)));
	BufferIn.Cust.rec_loss_time1 = BufferIn.Cust.rec_loss_time1 & (~(tmp_one << (bit_offset + 5)));
	BufferIn.Cust.rec_loss_time1 = BufferIn.Cust.rec_loss_time1 & (~(tmp_one << (bit_offset + 6)));
	BufferIn.Cust.rec_loss_time1 = BufferIn.Cust.rec_loss_time1 & (~(tmp_one << (bit_offset + 7)));
	BufferIn.Cust.rec_loss_time1 = (BufferIn.Cust.rec_loss_time1 | (hs_reply << bit_offset));
}

void reply_pack_2(int slave_no, uint32_t hs_reply)
{
	// 
	uint32_t bit_offset = ((slave_no - 1) * 8);
	uint32_t tmp_one = 1;
	BufferIn.Cust.rec_loss_time2 = BufferIn.Cust.rec_loss_time2 & (~(tmp_one << (bit_offset)));
	BufferIn.Cust.rec_loss_time2 = BufferIn.Cust.rec_loss_time2 & (~(tmp_one << (bit_offset + 1)));
	BufferIn.Cust.rec_loss_time2 = BufferIn.Cust.rec_loss_time2 & (~(tmp_one << (bit_offset + 2)));
	BufferIn.Cust.rec_loss_time2 = BufferIn.Cust.rec_loss_time2 & (~(tmp_one << (bit_offset + 3)));
	BufferIn.Cust.rec_loss_time2 = BufferIn.Cust.rec_loss_time2 & (~(tmp_one << (bit_offset + 4)));
	BufferIn.Cust.rec_loss_time2 = BufferIn.Cust.rec_loss_time2 & (~(tmp_one << (bit_offset + 5)));
	BufferIn.Cust.rec_loss_time2 = BufferIn.Cust.rec_loss_time2 & (~(tmp_one << (bit_offset + 6)));
	BufferIn.Cust.rec_loss_time2 = BufferIn.Cust.rec_loss_time2 & (~(tmp_one << (bit_offset + 7)));
	BufferIn.Cust.rec_loss_time2 = (BufferIn.Cust.rec_loss_time2 | (hs_reply << bit_offset));
}

void reply_pack_3(int slave_no, uint32_t hs_reply)
{
	// 
	uint32_t bit_offset = ((slave_no - 1) * 8);
	uint32_t tmp_one = 1;
	BufferIn.Cust.rec_loss_time3 = BufferIn.Cust.rec_loss_time3 & (~(tmp_one << (bit_offset)));
	BufferIn.Cust.rec_loss_time3 = BufferIn.Cust.rec_loss_time3 & (~(tmp_one << (bit_offset + 1)));
	BufferIn.Cust.rec_loss_time3 = BufferIn.Cust.rec_loss_time3 & (~(tmp_one << (bit_offset + 2)));
	BufferIn.Cust.rec_loss_time3 = BufferIn.Cust.rec_loss_time3 & (~(tmp_one << (bit_offset + 3)));
	BufferIn.Cust.rec_loss_time3 = BufferIn.Cust.rec_loss_time3 & (~(tmp_one << (bit_offset + 4)));
	BufferIn.Cust.rec_loss_time3 = BufferIn.Cust.rec_loss_time3 & (~(tmp_one << (bit_offset + 5)));
	BufferIn.Cust.rec_loss_time3 = BufferIn.Cust.rec_loss_time3 & (~(tmp_one << (bit_offset + 6)));
	BufferIn.Cust.rec_loss_time3 = BufferIn.Cust.rec_loss_time3 & (~(tmp_one << (bit_offset + 7)));
	BufferIn.Cust.rec_loss_time3 = (BufferIn.Cust.rec_loss_time3 | (hs_reply << bit_offset));
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
	
	uint32_t id = pRxHeader->Identifier;
	if (id > 0 && id < 11)
	{
		int buffer_len = get_data_len(pRxHeader->DataLength);
		
		for (int i = 0; i < buffer_len; i++)
		{
			byte_8_reply.buffer[i] = data[i];
		}
		can_msg_length_pack(id, buffer_len);
		
		//can_msg_length_in[id - 1] = buffer_len;
		joint_r_data[id - 1] = byte_8_reply.udata;
		reply_hs[id - 1] = hs_;
	}
	
}
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
	slave_1.Identifier = 0x1;
	slave_1.IdType = FDCAN_STANDARD_ID;
	slave_1.TxFrameType = FDCAN_DATA_FRAME;
	slave_1.DataLength = FDCAN_DLC_BYTES_8;
	slave_1.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	slave_1.BitRateSwitch = FDCAN_BRS_OFF;
	slave_1.FDFormat = FDCAN_CLASSIC_CAN;
	slave_1.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	slave_1.MessageMarker = 0;
	
	slave_2.Identifier = 0x2;
	slave_2.IdType = FDCAN_STANDARD_ID;
	slave_2.TxFrameType = FDCAN_DATA_FRAME;
	slave_2.DataLength = FDCAN_DLC_BYTES_8;
	slave_2.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	slave_2.BitRateSwitch = FDCAN_BRS_OFF;
	slave_2.FDFormat = FDCAN_CLASSIC_CAN;
	slave_2.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	slave_2.MessageMarker = 0;
	
	slave_3.Identifier = 0x3;
	slave_3.IdType = FDCAN_STANDARD_ID;
	slave_3.TxFrameType = FDCAN_DATA_FRAME;
	slave_3.DataLength = FDCAN_DLC_BYTES_8;
	slave_3.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	slave_3.BitRateSwitch = FDCAN_BRS_OFF;
	slave_3.FDFormat = FDCAN_CLASSIC_CAN;
	slave_3.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	slave_3.MessageMarker = 0;
	
	slave_4.Identifier = 0x4;
	slave_4.IdType = FDCAN_STANDARD_ID;
	slave_4.TxFrameType = FDCAN_DATA_FRAME;
	slave_4.DataLength = FDCAN_DLC_BYTES_8;
	slave_4.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	slave_4.BitRateSwitch = FDCAN_BRS_OFF;
	slave_4.FDFormat = FDCAN_CLASSIC_CAN;
	slave_4.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	slave_4.MessageMarker = 0;
	
	slave_5.Identifier = 0x5;
	slave_5.IdType = FDCAN_STANDARD_ID;
	slave_5.TxFrameType = FDCAN_DATA_FRAME;
	slave_5.DataLength = FDCAN_DLC_BYTES_8;
	slave_5.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	slave_5.BitRateSwitch = FDCAN_BRS_OFF;
	slave_5.FDFormat = FDCAN_CLASSIC_CAN;
	slave_5.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	slave_5.MessageMarker = 0;
	
	slave_6.Identifier = 0x6;
	slave_6.IdType = FDCAN_STANDARD_ID;
	slave_6.TxFrameType = FDCAN_DATA_FRAME;
	slave_6.DataLength = FDCAN_DLC_BYTES_8;
	slave_6.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	slave_6.BitRateSwitch = FDCAN_BRS_OFF;
	slave_6.FDFormat = FDCAN_CLASSIC_CAN;
	slave_6.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	slave_6.MessageMarker = 0;
	
	slave_7.Identifier = 0x7;
	slave_7.IdType = FDCAN_STANDARD_ID;
	slave_7.TxFrameType = FDCAN_DATA_FRAME;
	slave_7.DataLength = FDCAN_DLC_BYTES_8;
	slave_7.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	slave_7.BitRateSwitch = FDCAN_BRS_OFF;
	slave_7.FDFormat = FDCAN_CLASSIC_CAN;
	slave_7.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	slave_7.MessageMarker = 0;
	
	slave_8.Identifier = 0x8;
	slave_8.IdType = FDCAN_STANDARD_ID;
	slave_8.TxFrameType = FDCAN_DATA_FRAME;
	slave_8.DataLength = FDCAN_DLC_BYTES_8;
	slave_8.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	slave_8.BitRateSwitch = FDCAN_BRS_OFF;
	slave_8.FDFormat = FDCAN_CLASSIC_CAN;
	slave_8.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	slave_8.MessageMarker = 0;
	
	slave_9.Identifier = 0x9;
	slave_9.IdType = FDCAN_STANDARD_ID;
	slave_9.TxFrameType = FDCAN_DATA_FRAME;
	slave_9.DataLength = FDCAN_DLC_BYTES_8;
	slave_9.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	slave_9.BitRateSwitch = FDCAN_BRS_OFF;
	slave_9.FDFormat = FDCAN_CLASSIC_CAN;
	slave_9.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	slave_9.MessageMarker = 0;
	
	slave_10.Identifier = 0xa;
	slave_10.IdType = FDCAN_STANDARD_ID;
	slave_10.TxFrameType = FDCAN_DATA_FRAME;
	slave_10.DataLength = FDCAN_DLC_BYTES_8;
	slave_10.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	slave_10.BitRateSwitch = FDCAN_BRS_OFF;
	slave_10.FDFormat = FDCAN_CLASSIC_CAN;
	slave_10.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	slave_10.MessageMarker = 0;
	
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
  hfdcan1.Init.RxFifo0ElmtsNbr = 20;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 20;
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
  hfdcan2.Init.RxFifo0ElmtsNbr = 22;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 20;
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
	
	if (slave_routing[0] == 1)
	{
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &slave_1, slave_1_data) != HAL_OK)
		{
			can1_error_counter += 1;
		}
	}
	else if (slave_routing[0] == 2)
	{
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &slave_1, slave_1_data) != HAL_OK)
		{
			can2_error_counter += 1;
		}
	}
	else
	{
		// pass
	}
	
	if (slave_routing[1] == 1)
	{
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &slave_2, slave_2_data) != HAL_OK)
		{
			can1_error_counter += 1;
		}
	}
	else if (slave_routing[1] == 2)
	{
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &slave_2, slave_2_data) != HAL_OK)
		{
			can2_error_counter += 1;
		}
	}
	else
	{
		// pass
	}
	
	if (slave_routing[2] == 1)
	{
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &slave_3, slave_3_data) != HAL_OK)
		{
			can1_error_counter += 1;
		}
	}
	else if (slave_routing[2] == 2)
	{
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &slave_3, slave_3_data) != HAL_OK)
		{
			can2_error_counter += 1;
		}
	}
	else
	{
		// pass
	}
	
	if (slave_routing[3] == 1)
	{
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &slave_4, slave_4_data) != HAL_OK)
		{
			can1_error_counter += 1;
		}
	}
	else if (slave_routing[3] == 2)
	{
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &slave_4, slave_4_data) != HAL_OK)
		{
			can2_error_counter += 1;
		}
	}
	else
	{
		// pass
	}
	
	if (slave_routing[4] == 1)
	{
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &slave_5, slave_5_data) != HAL_OK)
		{
			can1_error_counter += 1;
		}
	}
	else if (slave_routing[4] == 2)
	{
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &slave_5, slave_5_data) != HAL_OK)
		{
			can2_error_counter += 1;
		}
	}
	else
	{
		// pass
	}
	
	if (slave_routing[5] == 1)
	{
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &slave_6, slave_6_data) != HAL_OK)
		{
			can1_error_counter += 1;
		}
	}
	else if (slave_routing[5] == 2)
	{
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &slave_6, slave_6_data) != HAL_OK)
		{
			can2_error_counter += 1;
		}
	}
	else
	{
		// pass
	}
	
	if (slave_routing[6] == 1)
	{
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &slave_7, slave_7_data) != HAL_OK)
		{
			can1_error_counter += 1;
		}
	}
	else if (slave_routing[6] == 2)
	{
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &slave_7, slave_7_data) != HAL_OK)
		{
			can2_error_counter += 1;
		}
	}
	else
	{
		// pass
	}
	
	uint64_t tmp_one = 0x01;
	if ((BufferOut.Cust.data_3 & tmp_one) == tmp_one)
	{
		if (slave_routing[7] == 1)
		{
			if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &slave_8, slave_8_data) != HAL_OK)
			{
				can1_error_counter += 1;
			}
		}
		else if (slave_routing[7] == 2)
		{
			if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &slave_8, slave_8_data) != HAL_OK)
			{
				can2_error_counter += 1;
			}
		}
		else
		{
			// pass
		}
	}
	
	uint64_t tmp_two = 0x02;
	if ((BufferOut.Cust.data_3 & tmp_two) == tmp_two)
	{
		if (slave_routing[8] == 1)
		{
			if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &slave_9, slave_9_data) != HAL_OK)
			{
				can1_error_counter += 1;
			}
		}
		else if (slave_routing[8] == 2)
		{
			if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &slave_9, slave_9_data) != HAL_OK)
			{
				can2_error_counter += 1;
			}
		}
		else
		{
			// pass
		}
	}
	
	uint64_t tmp_four = 0x04;
	if ((BufferOut.Cust.data_3 & tmp_four) == tmp_four)
	{
		if (slave_routing[9] == 1)
		{
			if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &slave_10, slave_10_data) != HAL_OK)
			{
				can1_error_counter += 1;
			}
		}
		else if (slave_routing[9] == 2)
		{
			if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &slave_10, slave_10_data) != HAL_OK)
			{
				can2_error_counter += 1;
			}
		}
		else
		{
			// pass
		}
	}
}

void send_to_joint(int joint_no)
{
	/*if (joint_no == 1 || joint_no == 2 || joint_no == 3)
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
	}*/
}

void pack_motor_data()
{
	can_msg_length_unpack();
	can_id_unpack();
	
	byte_8.udata = BufferOut.Cust.node_1;
	for (size_t i = 0; i < 8; i++)
	{
		slave_1_data[i] = byte_8.buffer[i];
	}
	
	byte_8.udata = BufferOut.Cust.node_2;
	for (size_t i = 0; i < 8; i++)
	{
		slave_2_data[i] = byte_8.buffer[i];
	}
	
	byte_8.udata = BufferOut.Cust.node_3;
	for (size_t i = 0; i < 8; i++)
	{
		slave_3_data[i] = byte_8.buffer[i];
	}
	
	byte_8.udata = BufferOut.Cust.node_4;
	for (size_t i = 0; i < 8; i++)
	{
		slave_4_data[i] = byte_8.buffer[i];
	}
	
	byte_8.udata = BufferOut.Cust.node_5;
	for (size_t i = 0; i < 8; i++)
	{
		slave_5_data[i] = byte_8.buffer[i];
	}
	
	byte_8.udata = BufferOut.Cust.node_6;
	for (size_t i = 0; i < 8; i++)
	{
		slave_6_data[i] = byte_8.buffer[i];
	}
	
	byte_8.udata = BufferOut.Cust.node_7;
	for (size_t i = 0; i < 8; i++)
	{
		slave_7_data[i] = byte_8.buffer[i];
	}
	
	byte_8.udata = BufferOut.Cust.node_8;
	for (size_t i = 0; i < 8; i++)
	{
		slave_8_data[i] = byte_8.buffer[i];
	}
	
	byte_8.udata = BufferOut.Cust.node_9;
	for (size_t i = 0; i < 8; i++)
	{
		slave_9_data[i] = byte_8.buffer[i];
	}
	
	byte_8.udata = BufferOut.Cust.node_10;
	for (size_t i = 0; i < 8; i++)
	{
		slave_10_data[i] = byte_8.buffer[i];
	}
	
	// setting msg data length
	slave_1.DataLength = get_data_len_code(can_msg_length_out[0]);
	slave_2.DataLength = get_data_len_code(can_msg_length_out[1]);
	slave_3.DataLength = get_data_len_code(can_msg_length_out[2]);
	slave_4.DataLength = get_data_len_code(can_msg_length_out[3]);
	slave_5.DataLength = get_data_len_code(can_msg_length_out[4]);
	slave_6.DataLength = get_data_len_code(can_msg_length_out[5]);
	slave_7.DataLength = get_data_len_code(can_msg_length_out[6]);
	slave_8.DataLength = get_data_len_code(can_msg_length_out[7]);
	slave_9.DataLength = get_data_len_code(can_msg_length_out[8]);
	slave_10.DataLength = get_data_len_code(can_msg_length_out[9]);
	
	slave_1.Identifier = can_id_list[0];
	slave_2.Identifier = can_id_list[1];
	slave_3.Identifier = can_id_list[2];
	slave_4.Identifier = can_id_list[3];
	slave_5.Identifier = can_id_list[4];
	slave_6.Identifier = can_id_list[5];
	slave_7.Identifier = can_id_list[6];
	slave_8.Identifier = can_id_list[7];
	slave_9.Identifier = can_id_list[8];
	slave_10.Identifier = can_id_list[9];
}

void control()
{
	//pass
	pack_motor_data();
	
	send_to_all_slave();
}

uint16_t get_motor_status()
{
	//pass
}

void pack_ethercat_data()
{
	BufferIn.Cust.hs = hs_;
	
	BufferIn.Cust.node_1 = joint_r_data[0];
	BufferIn.Cust.node_2 = joint_r_data[1];
	BufferIn.Cust.node_3 = joint_r_data[2];
	BufferIn.Cust.node_4 = joint_r_data[3];
	BufferIn.Cust.node_5 = joint_r_data[4];
	BufferIn.Cust.node_6 = joint_r_data[5];
	BufferIn.Cust.node_7 = joint_r_data[6];
	BufferIn.Cust.node_8 = joint_r_data[7];
	BufferIn.Cust.node_9 = joint_r_data[8];
	BufferIn.Cust.node_10 = joint_r_data[9];
	
	
	/*BufferIn.Cust.data1 = slave_routing[0];
	BufferIn.Cust.node_1 = BufferOut.Cust.node_1;
	BufferIn.Cust.data2 = can_id_list[0];
	BufferIn.Cust.node_2 = BufferOut.Cust.can1_id;
	BufferIn.Cust.node_3 = BufferOut.Cust.data_1;
	BufferIn.Cust.node_4 = BufferOut.Cust.hs;*/
	for (size_t i = 0; i < 4; i++)
	{
		uint32_t hs_tmp = hs_ - reply_hs[i];	
		reply_pack_1(i, hs_tmp);
	}
	
	for (size_t i = 4; i < 8; i++)
	{
		uint32_t hs_tmp = hs_ - reply_hs[i];	
		reply_pack_1((i-4), hs_tmp);
	}
	
	for (size_t i = 8; i < 10; i++)
	{
		uint32_t hs_tmp = hs_ - reply_hs[i];	
		reply_pack_1((i-8), hs_tmp);
	}
	
	BufferIn.Cust.rec_error_can1 = (uint16_t)can1_last_error_code;
	BufferIn.Cust.rec_error_can2 = (uint16_t)can2_last_error_code;
	
	
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM5)
	{
		
		pack_ethercat_data();
		main_task(&ethercat_slave);
		
		uint64_t tmp_hs_ = BufferOut.Cust.hs;
		
		// 1. control
		if (tmp_hs_ > hs_ || tmp_hs_ < 5)
		{
			control_word = BufferOut.Cust.control_word;
			
			if (tmp_hs_ < 5)
			{
				is_enable = 0;
				// read routing configure
				can_slave_routing_unpack();
			}
			
			if (control_word == 1)
			{
				control();
			}
			
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
