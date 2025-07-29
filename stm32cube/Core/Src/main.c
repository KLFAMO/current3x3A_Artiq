/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "interface.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FLASH_PARAM_START_ADDR  ((uint32_t)0x081E0000)  // bank 2, sektor 7
#define FLASH_WORD_SIZE        (32)  // Flash word = 256-bit = 32 bytes
#define BUFFER_SIZE 100
#define MAX(a,b) (((a)>(b))?(a):(b))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi4;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for interfaceTask */
osThreadId_t interfaceTaskHandle;
const osThreadAttr_t interfaceTask_attributes = {
  .name = "interfaceTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
char spi_buf[30];
int state;
uint16_t d_in;

/* create an array for DAC's values:

		row 0 for DAC1: X1, Y1, Z1, T1
		row 1 for DAC2: X2, Y2, Z2, T2
		row 2 for DAC3: X3, Y3, Z3, T3
    row 3 for DAC4: X4, Y4, Z4, T4
		row 4 for transitions between states

		X,Y,Z are voltage value in volt and T is time in ms

*/
double DAC[5][4] = {
		{0.0, 0.0, 0.0, 1},
		{0.0, 0.0, 0.0, 1},
		{0.0, 0.0, 0.0, 1},
    {0.0, 0.0, 0.0, 1},
		{0.0, 0.0, 0.0, 1} // <- this row is used for transitions between states
};
const double v_ref = 3.0;
const int max_dec = 65536;
int last_r = 4;



void Flash_Write_Params(uint32_t address, parameters *data) {
  HAL_FLASH_Unlock();  // Odblokowanie pamięci flash

  FLASH_EraseInitTypeDef eraseInitStruct;
  uint32_t sectorError;

  // Kasowanie sektora przed zapisem
  eraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
  eraseInitStruct.Banks        = FLASH_BANK_2;  // **Bank 2**
  eraseInitStruct.Sector       = FLASH_SECTOR_7;  // **Sektor 7**
  eraseInitStruct.NbSectors    = 1;
  eraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  if (HAL_FLASHEx_Erase(&eraseInitStruct, &sectorError) != HAL_OK) {
      HAL_FLASH_Lock();
      return;  // Błąd kasowania
  }
  
  uint64_t *data_ptr = (uint64_t*)data;
  uint64_t flash_word[4];
  for (uint32_t i = 0; i < sizeof(parameters) / 8; i += 4) {
      flash_word[0] = (i < sizeof(parameters) / 8) ? data_ptr[i] : 0xFFFFFFFFFFFFFFFF;
      flash_word[1] = (i + 1 < sizeof(parameters) / 8) ? data_ptr[i + 1] : 0xFFFFFFFFFFFFFFFF;
      flash_word[2] = (i + 2 < sizeof(parameters) / 8) ? data_ptr[i + 2] : 0xFFFFFFFFFFFFFFFF;
      flash_word[3] = (i + 3 < sizeof(parameters) / 8) ? data_ptr[i + 3] : 0xFFFFFFFFFFFFFFFF;

      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address + i * 8, (uint64_t)flash_word) != HAL_OK) {
          HAL_FLASH_Lock();
          return;  // Błąd zapisu
      }
  }

  HAL_FLASH_Lock();  // Zablokowanie pamięci flash
}

void Flash_Read_Params(uint32_t address, parameters *data) {
  memcpy(data, (void*)address, sizeof(parameters));  // Odczytaj całą strukturę
}

uint32_t Flash_Read_Version(uint32_t address) {
  return *(volatile double*)address;  // Odczytaj pierwsze 4 bajty
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART4_Init(void);
static void MX_ADC2_Init(void);
static void MX_SPI4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
void StartDefaultTask(void *argument);
void StartInterfaceTask(void *argument);

/* USER CODE BEGIN PFP */
void SendSpiMesToDac(uint32_t);
void SetDAC(uint8_t channel, uint16_t value);
void SendToDAC(int r);
int ExtractMessageOld(char* msg, char* out);
void ExtractMessage(char* rxBuffer, char* txBuffer);
void arrayToString(double DAC[4][4], char *result);
void update_array();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern parameters par;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_UART4_Init();
  MX_ADC2_Init();
  MX_SPI4_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  initInterface();

  // read par from flash
  if (par.version != Flash_Read_Version(FLASH_PARAM_START_ADDR)){
    Flash_Write_Params(FLASH_PARAM_START_ADDR, &par);
  }
  else{
    Flash_Read_Params(FLASH_PARAM_START_ADDR, &par);
  }

  // DAC - Reset select.
	// If RSTSEL is low, input coding is binary;
	// if high = 2's complement
	HAL_GPIO_WritePin(RSTSEL_GPIO_Port, RSTSEL_Pin, GPIO_PIN_RESET);

	// if SET = work, if RESET = set all outputs to 0
	// (this pin was solderd to 3.3V on board in 3x3A_v2 version because of bug
	HAL_GPIO_WritePin(DACRST_GPIO_Port, DACRST_Pin, GPIO_PIN_SET);

	SetDAC(0, 0);
	SetDAC(1, 0);
	SetDAC(2, 0);
	SetDAC(3, 0);

	// DIR SET means: positive and DIR RESET means: negative
	HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DIR3_GPIO_Port, DIR3_Pin, GPIO_PIN_SET);
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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of interfaceTask */
  interfaceTaskHandle = osThreadNew(StartInterfaceTask, NULL, &interfaceTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 18;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 6144;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 18;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 6144;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  hadc2.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi2.Init.DataSize = SPI_DATASIZE_24BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_SLAVE;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

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
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart6, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart6, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DAT4_OUT_Pin|DIR3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, DIR1_Pin|DIR2_Pin|RSTSEL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DACRST_Pin|ENABLE_Pin|LDAC_Pin|DAT7_OUT_Pin
                          |DAT6_OUT_Pin|DAT5_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DAT4_OUT_Pin DIR3_Pin */
  GPIO_InitStruct.Pin = DAT4_OUT_Pin|DIR3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DAT4_Pin DAT5_Pin DAT6_Pin DAT7_Pin
                           ES1_Pin */
  GPIO_InitStruct.Pin = DAT4_Pin|DAT5_Pin|DAT6_Pin|DAT7_Pin
                          |ES1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ES3_Pin ES2_Pin */
  GPIO_InitStruct.Pin = ES3_Pin|ES2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR1_Pin DIR2_Pin RSTSEL_Pin */
  GPIO_InitStruct.Pin = DIR1_Pin|DIR2_Pin|RSTSEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : DACRST_Pin ENABLE_Pin LDAC_Pin DAT7_OUT_Pin
                           DAT6_OUT_Pin DAT5_OUT_Pin */
  GPIO_InitStruct.Pin = DACRST_Pin|ENABLE_Pin|LDAC_Pin|DAT7_OUT_Pin
                          |DAT6_OUT_Pin|DAT5_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TTL0_Pin TTL1_Pin TTL2_Pin TTL3_Pin
                           TTL4_Pin TTL5_Pin SD_DET1_Pin TTL6_Pin */
  GPIO_InitStruct.Pin = TTL0_Pin|TTL1_Pin|TTL2_Pin|TTL3_Pin
                          |TTL4_Pin|TTL5_Pin|SD_DET1_Pin|TTL6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDIO1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/*
	 * In Artiq version, there are 7 input TTLs.
	 * TTL1, TTL2 are used to set state (TODO: add TTL3-6)
	 * TTL0 is used to trigger state change
	 */

  if(GPIO_Pin==TTL0_Pin)
  {
	  par.state.val=3;
	  if(HAL_GPIO_ReadPin(TTL1_GPIO_Port, TTL1_Pin) == GPIO_PIN_RESET &&
		 HAL_GPIO_ReadPin(TTL2_GPIO_Port, TTL2_Pin) == GPIO_PIN_RESET
	  ){
		  // state 1 row 0 in the DAC's array
		  //if(last_r != 0){
			  SendToDAC(0);
			  par.state.val=0;
		  //}
	  }else if(HAL_GPIO_ReadPin(TTL1_GPIO_Port, TTL1_Pin) == GPIO_PIN_SET &&
			  HAL_GPIO_ReadPin(TTL2_GPIO_Port, TTL2_Pin) == GPIO_PIN_RESET
	  ){
		  // state 2 row 1 in the DAC's array
		  if(last_r != 1){
			  SendToDAC(1);
		  }
		  par.state.val=1;
	  }else if(HAL_GPIO_ReadPin(TTL1_GPIO_Port, TTL1_Pin) == GPIO_PIN_RESET &&
				 HAL_GPIO_ReadPin(TTL2_GPIO_Port, TTL2_Pin) == GPIO_PIN_SET
				 ){
		  // state 3 row 2 in the DAC's array
		  if(last_r != 2){
			  SendToDAC(2);
		  }
		  par.state.val=2;
	  }
    }else if(HAL_GPIO_ReadPin(TTL1_GPIO_Port, TTL1_Pin) == GPIO_PIN_SET &&
				 HAL_GPIO_ReadPin(TTL2_GPIO_Port, TTL2_Pin) == GPIO_PIN_SET
				 ){
		  // state 4 row 2 in the DAC's array
		  if(last_r != 3){
			  SendToDAC(3);
		  }
		  par.state.val=3;
	  }
  }
}


void arrayToString(double DAC[5][4], char *result) {
    char buffer[50];
    result[0] = '\0';

    for (int i = 0; i < 5; i++) {
        strcat(result, "{ ");
        for (int j = 0; j < 4; j++) {
            sprintf(buffer, "%.2f", DAC[i][j]);
            strcat(result, buffer);
            if (j < 3) strcat(result, ", ");
        }
        strcat(result, " }");
        if (i < 3) strcat(result, ",\n");
    }
}

void SendToDAC(int r)  // original Mehrdad's function
/*
 * r - state number (0-3) (depends on TTLs state) - row number of array DAC
 */
{
//	uint32_t t0, t1, t;
	/* transition time must be min 1ms */
	if(DAC[r][3] < 1){
		DAC[r][3] = 1;
	}

	int n = round(DAC[r][3]*2);	// 58 to apply values to DAC
	/* n is a number of steps in entire transition (assuming 58 steps per 1 ms) */

	double dif1, dif2, dif3;

	/* difx is single step voltage change for channel x */
	dif1 = fabs(DAC[r][0] - DAC[last_r][0])/n;
	dif2 = fabs(DAC[r][1] - DAC[last_r][1])/n;
	dif3 = fabs(DAC[r][2] - DAC[last_r][2])/n;

	// x? this part need for sending correct value in first loop
	DAC[4][0] = DAC[last_r][0];
	DAC[4][1] = DAC[last_r][1];
	DAC[4][2] = DAC[last_r][2];

	/* this case will never heppen?
	if(r == 3){
		n = 1;
	} */

//	t0 = HAL_GetTick();
	for(int i = 1; i <= n; i++){

		if(DAC[r][0] > DAC[last_r][0]){
			DAC[4][0] += dif1;
		}else{
			DAC[4][0] -= dif1;
		}
		if(DAC[r][1] > DAC[last_r][1]){
			DAC[4][1] += dif2;
		}else{
			DAC[4][1] -= dif2;
		}
		if(DAC[r][2] > DAC[last_r][2]){
			DAC[4][2] += dif3;
		}else{
			DAC[4][2] -= dif3;
		}

		for(int j = 0; j < 3; j++){
			  state = 1;
			  switch(j){
				  case 0:
					  if(DAC[4][j] >= 0){
						  HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_SET);
					  }else{
						  HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_RESET);
					  }
					  break;
				  case 1:
					  if(DAC[4][j] >= 0){
						  HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_SET);
					  }else{
						  HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_RESET);
					  }
					  break;
				  case 2:
					  if(DAC[4][j] >= 0){
						  HAL_GPIO_WritePin(DIR3_GPIO_Port, DIR3_Pin, GPIO_PIN_SET);
					  }else{
						  HAL_GPIO_WritePin(DIR3_GPIO_Port, DIR3_Pin, GPIO_PIN_RESET);
					  }
					  break;
			  }

			  if(fabs(DAC[4][j]) > v_ref){
				  d_in = 0xffff;
			  }else{
				  d_in = abs(round((DAC[4][j]/v_ref) * max_dec));
			  }

			  SetDAC(j, d_in);
		}
	}

//	t1 = HAL_GetTick();
//	t = t1 - t0;
	last_r = r;
}

void update_array(){
  DAC[0][0] = par.s0.v1.val;
  DAC[0][2] = par.s0.v3.val;
  DAC[0][1] = par.s0.v2.val;
  DAC[0][3] = par.s0.t.val;
  DAC[1][0] = par.s1.v1.val;
  DAC[1][1] = par.s1.v2.val;
  DAC[1][2] = par.s1.v3.val;
  DAC[1][3] = par.s1.t.val;
  DAC[2][0] = par.s2.v1.val;
  DAC[2][1] = par.s2.v2.val;
  DAC[2][2] = par.s2.v3.val;
  DAC[2][3] = par.s2.t.val;
  DAC[3][0] = par.s3.v1.val;
  DAC[3][1] = par.s3.v2.val;
  DAC[3][2] = par.s3.v3.val;
  DAC[3][3] = par.s3.t.val;
}

void ExtractMessage(char* rxBuffer, char* txBuffer)
{
    cmd_string_interpret(rxBuffer, txBuffer);
    txBuffer[BUFFER_SIZE - 1] = '\0';
    update_array();
}

void SendSpiMesToDac(uint32_t message){
	/*
	 * send prepared message to DAC
	 */
	HAL_StatusTypeDef status;
	uint8_t dataToSend[3] = {
			(message >> 0) & 0xFF,
	        (message >> 8) & 0xFF,
	        (message >> 16)& 0xFF
	};
	status = HAL_SPI_Transmit(&hspi2, dataToSend, 1, 100);
	if (status != HAL_OK) {
	}
}

void SetDAC(uint8_t channel, uint16_t value){
	/*set DAC, channel (0,1,2,3), value - in bits*/
	uint32_t message = 0x00000000;
	message = message | (value & 0xFFFF);
	message = message | (((uint32_t)channel & 0b11) << 17);
	SendSpiMesToDac(message);

	// LDAC load DACs, rising edge triggered loads all DAC register
	HAL_GPIO_WritePin(LDAC_GPIO_Port, LDAC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LDAC_GPIO_Port, LDAC_Pin, GPIO_PIN_RESET);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartInterfaceTask */
/**
* @brief Function implementing the interfaceTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartInterfaceTask */
void StartInterfaceTask(void *argument)
{
  /* USER CODE BEGIN StartInterfaceTask */
  uint8_t rxChar;
  uint8_t rxBuffer[BUFFER_SIZE];
  uint8_t txBuffer[BUFFER_SIZE];
//  uint8_t tmpBuffer[BUFFER_SIZE];
  uint16_t index = 0;
  uint8_t helloMsg[] = "\n";
  HAL_StatusTypeDef status;

  HAL_UART_Transmit(&huart4, helloMsg, strlen(helloMsg), HAL_MAX_DELAY);

  /* Infinite loop */
  for(;;)
  {
	  /* Receive a single character */
	  status = HAL_UART_Receive(&huart4, &rxChar, 1, HAL_MAX_DELAY);
	  if (status == HAL_OK)
	  {
		  if ((rxChar == '\r' || rxChar == '\n') && index>0){
			  rxBuffer[index++]='\n';
			  rxBuffer[index++]='\0';
			  ExtractMessage((char*) rxBuffer, (char*) txBuffer);
//			  strcpy(tmpBuffer, helloMsg);
//			  strcat(tmpBuffer, txBuffer);
//			  strcpy(txBuffer, tmpBuffer);

			  HAL_UART_Transmit(&huart4, txBuffer, strlen(txBuffer), HAL_MAX_DELAY);
			  HAL_UART_Transmit(&huart4, helloMsg, strlen(helloMsg), HAL_MAX_DELAY);
			  index=0;
		  }
		  else{
			  rxBuffer[index++] = rxChar;
			  if (index >= BUFFER_SIZE) index = 0;
		  }
	  }

    if (par.save.val == 1){
      par.save.val = 0;
      Flash_Write_Params(FLASH_PARAM_START_ADDR, &par);
    }
    if (par.load.val == 1){
      par.load.val = 0;
      Flash_Read_Params(FLASH_PARAM_START_ADDR, &par);
    }
  }
  /* USER CODE END StartInterfaceTask */
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
