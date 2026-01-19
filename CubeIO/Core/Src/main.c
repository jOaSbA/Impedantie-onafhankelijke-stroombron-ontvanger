/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

osThreadId UartRegelingHandle;
osThreadId StroomregelingHandle;
osThreadId ModeSwitchTaskHandle;
osMessageQId uartDataQueueHandle;
osMessageQId stroomDataQueueHandle;
osMessageQId modeQueueHandle;
osMutexId CRCMutexHandle;
osMutexId ADCMutexHandle;
osMutexId UARTMutexHandle;
/* USER CODE BEGIN PV */

uint8_t adres = 1;
uint8_t RXData[4];
bool stopRegelingFlag = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
void StartUartRegeling(void const * argument);
void StartStroomregeling(void const * argument);
void StartModeSwitchTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//check of werkt
void calculateMessage(uint16_t data, uint8_t adress, uint8_t* TXData)
{
    // Vul het frame
    TXData[0] = adress;
    TXData[1] = (data >> 8) & 0x0F;   // bovenste 4 bits van 12-bit data
    TXData[2] = data & 0xFF;          // onderste 8 bits

    osMutexWait(CRCMutexHandle, osWaitForever);  // Mutex CRC aan

    //3 bytes array naar een uint32_t
    uint32_t crc_input =	((uint32_t)TXData[0] << 24) |
    						((uint32_t)TXData[1] << 16) |
							((uint32_t)TXData[2] << 8);

    TXData[3] = (uint8_t)HAL_CRC_Calculate(&hcrc, &crc_input, 1);
    __HAL_CRC_DR_RESET(&hcrc);	// Reset CRC
    osMutexRelease(CRCMutexHandle);     // Geef mutex terug
}


bool checkCRCMessage(uint8_t *RXData)
{
	//Mutex claimen
	osMutexWait(CRCMutexHandle, osWaitForever);

    uint32_t crc_input =	((uint32_t)RXData[0] << 24) |
    						((uint32_t)RXData[1] << 16) |
							((uint32_t)RXData[2] << 8);

    uint8_t crc = (uint8_t)HAL_CRC_Calculate(&hcrc, &crc_input, 1);
    //Reset CRC
    __HAL_CRC_DR_RESET(&hcrc);

    //Mutex loslaten
    osMutexRelease(CRCMutexHandle);

    return (crc == RXData[3]);
}

bool isAdresCorrect(uint8_t *RXData, uint8_t adres){
	return (adres == RXData[0]);
}

//Called bij een uart interrupt klaar
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if (huart->Instance != USART1){
    	return;
    }

    //Combineert de 4 byte array en stuurt naar task
    uint32_t RXData32 = ((uint32_t)RXData[0] << 24) |
                		((uint32_t)RXData[1] << 16) |
						((uint32_t)RXData[2] << 8)  |
						((uint32_t)RXData[3]);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(uartDataQueueHandle, &RXData32, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    //klaar voor volgend bericht
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); //Receive mode
    //HAL_Delay(10);
	HAL_UART_Receive_DMA(&huart1, RXData, 4);
}

int16_t ADCtoStroom(uint16_t adc_value)
{
	const uint16_t ADC4m  = 891;
	const uint16_t ADC20m = 3883;

    // lineair, ook onder 4m en boven 20m
    return ((int16_t)(adc_value - ADC4m) * 4096) / (ADC20m - ADC4m);
}


void sendDAC(uint16_t DACData){
	//Stuur DAC aan op basis van 12 bit waarde
	//Zonder MUTEX, want maar 1 plek gebruikt het
	if(DACData > 0x0FFF){
		DACData = 0x0FFF;
	}

    uint8_t buffer[3];
    buffer[0] = 0x40;            		// Command byte
    buffer[1] = (DACData >> 4) & 0xFF; 	// Upper 8 bits
    buffer[2] = (DACData & 0x0F) << 4; 	// Lower 4 bits

    //I2C DAC aansturing
    HAL_I2C_Master_Transmit(&hi2c1, (0x61 << 1), buffer, 3, HAL_MAX_DELAY);
}

uint16_t ADCChannelSamples(uint32_t channel, uint8_t ADCSamples)
{
    uint32_t sum = 0;
    ADC_ChannelConfTypeDef sConfig = {0};

    // Mutex claimen
    osMutexWait(ADCMutexHandle, osWaitForever);

    // Channel instellen
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // Averaging
    for (uint8_t i = 0; i < ADCSamples; i++) {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        sum += HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
    }

    // Mutex loslaten
    osMutexRelease(ADCMutexHandle);

    return (uint16_t)(sum / ADCSamples);
}

static inline float clampf(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); //Receive mode
  HAL_Delay(100);
  HAL_UART_Receive_DMA(&huart1, RXData, 4);

  sendDAC((uint16_t)0);

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of CRCMutex */
  osMutexDef(CRCMutex);
  CRCMutexHandle = osMutexCreate(osMutex(CRCMutex));

  /* definition and creation of ADCMutex */
  osMutexDef(ADCMutex);
  ADCMutexHandle = osMutexCreate(osMutex(ADCMutex));

  /* definition and creation of UARTMutex */
  osMutexDef(UARTMutex);
  UARTMutexHandle = osMutexCreate(osMutex(UARTMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of uartDataQueue */
  osMessageQDef(uartDataQueue, 4, uint32_t);
  uartDataQueueHandle = osMessageCreate(osMessageQ(uartDataQueue), NULL);

  /* definition and creation of stroomDataQueue */
  osMessageQDef(stroomDataQueue, 4, uint16_t);
  stroomDataQueueHandle = osMessageCreate(osMessageQ(stroomDataQueue), NULL);

  /* definition and creation of modeQueue */
  osMessageQDef(modeQueue, 1, bool);
  modeQueueHandle = osMessageCreate(osMessageQ(modeQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of UartRegeling */
  osThreadDef(UartRegeling, StartUartRegeling, osPriorityNormal, 0, 128);
  UartRegelingHandle = osThreadCreate(osThread(UartRegeling), NULL);

  /* definition and creation of Stroomregeling */
  osThreadDef(Stroomregeling, StartStroomregeling, osPriorityLow, 0, 128);
  StroomregelingHandle = osThreadCreate(osThread(Stroomregeling), NULL);

  /* definition and creation of ModeSwitchTask */
  osThreadDef(ModeSwitchTask, StartModeSwitchTask, osPriorityIdle, 0, 128);
  ModeSwitchTaskHandle = osThreadCreate(osThread(ModeSwitchTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.GeneratingPolynomial = 7;
  hcrc.Init.CRCLength = CRC_POLYLENGTH_8B;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00503D58;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
  huart1.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9|DE__RE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB9 DE__RE_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_9|DE__RE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartUartRegeling */
/**
* @brief Function implementing the UartRegeling thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartRegeling */
void StartUartRegeling(void const * argument)
{
  /* USER CODE BEGIN 5 */

	uint32_t receivedRXData32;

  /* Infinite loop */
  for(;;)
  {
	  if(xQueueReceive(uartDataQueueHandle, &receivedRXData32, pdMS_TO_TICKS(50)) == pdTRUE){
		  //Stuurt zelfde data naar controller (adres 0)
		  uint8_t receivedRXData[4];
		  receivedRXData[0] = (receivedRXData32 >> 24) & 0xFF;
		  receivedRXData[1] = (receivedRXData32 >> 16) & 0xFF;
		  receivedRXData[2] = (receivedRXData32 >> 8)  & 0xFF;
		  receivedRXData[3] =  receivedRXData32        & 0xFF;

		  //Check of het adres en de CRC klopt
		  if(isAdresCorrect(receivedRXData, adres) && checkCRCMessage(receivedRXData)){
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET); //Error led uit
			  uint16_t receiveStroomData = 	((uint16_t)receivedRXData[1] << 8) | receivedRXData[2];
			  uint8_t ack[4];
			  calculateMessage(receiveStroomData, 0, ack); //0 is de controller

			  //Stuur de ack
			  osMutexWait(UARTMutexHandle, osWaitForever);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); // Send mode
			  HAL_UART_Transmit(&huart1, ack, 4, HAL_MAX_DELAY);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); //Receive mode
			  osMutexRelease(UARTMutexHandle);

			  //Via queue naar andere task sturen die de stroom regelt receiveStroomData
			  xQueueGenericSend(stroomDataQueueHandle, &receiveStroomData, portMAX_DELAY, queueSEND_TO_BACK);
		  } else{
			  //Adres of CRC klopte niet
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET); //Error led aan
		  }
	  }
	  osDelay(10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartStroomregeling */
/**
* @brief Function implementing the Stroomregeling thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStroomregeling */
void StartStroomregeling(void const * argument)
{
  /* USER CODE BEGIN StartStroomregeling */
  /* Infinite loop */

	bool modeRegeling = 0;

	uint16_t stroomData = 0;	//startwaarde stroom
	float kp = 0.2f; 			//P regeling factor
	float ki = 0.004f; 			//I regeling factor
	float integral = 0.0f;		//integraal waarde
	float maxintegraal = 2000.0f;
	float DACData = 0; 			//DAC begin waarde

	//0.01 mA afwijking toestaan voor stabiele waarde
	#define dodeband (0.01f * 4096.0f / 16.0f)

  for(;;)
  {
	  if (xQueueReceive(modeQueueHandle, &modeRegeling, pdMS_TO_TICKS(50)) == pdTRUE){
		  //modeRegeling is veranderd, dus reset alle waardes
		  integral = 0;
		  DACData = 0;
		  stopRegelingFlag = false;
	  }

	  if(modeRegeling){
		  //Interne regeling

		  //Ontvang de nieuwe stroomdata
		  if (xQueueReceive(stroomDataQueueHandle, &stroomData, pdMS_TO_TICKS(50)) == pdTRUE){
			  stopRegelingFlag = false;
			  osDelay(10);
		  }

		  if(stopRegelingFlag == false){
		  //ADC uitlezen
		  uint16_t adc_value = ADCChannelSamples(ADC_CHANNEL_6, 16); //6 is de stroomsense channel

		  //converteren naar stroomwaarde, 4-20mA is de range
		  int16_t huidigeStroomData = ADCtoStroom(adc_value);

		  //PI regeling
		  float error = (float)stroomData - (float)huidigeStroomData; // verschil

		  if(fabs(error) <= dodeband){
		  	stopRegelingFlag = true;
		  	continue;
		  }

		  integral += error; //integraal update
		  integral = clampf(integral, -maxintegraal, maxintegraal);
		  DACData += kp * error + ki * integral; // pas DAC waarde aan (PI)

		  // Limiet
		  if (DACData > 4095.0f) {
		      DACData = 4095.0f;
		      if (error > 0){
		    	  integral -= error; //anti windum
		      }
		  }
		  if (DACData < 0.0f) {
		      DACData = 0.0f;
		      integral -= error;
		  }

		  // DAC aansturen op basis van de regeling
		  sendDAC((uint16_t)DACData);
		  }
	  } else{
		  //Externe regeling

		  //Ontvang de nieuwe stroomdata
		  if (xQueueReceive(stroomDataQueueHandle, &stroomData, pdMS_TO_TICKS(50)) == pdTRUE){
			  osDelay(10);

			  //ADC uitlezen
			  uint16_t adc_value = ADCChannelSamples(ADC_CHANNEL_6, 16); //6 is de stroomsense channel

			  //converteren naar stroomwaarde, 4-20mA is de range
			  int16_t huidigeStroomData = ADCtoStroom(adc_value);

			  //huidigeStroomData terugsturen
			  uint8_t TXData[4];
			  calculateMessage((uint16_t)huidigeStroomData, 0, TXData); //0 is de controller

			  // Stuur TXData via RS485
			  osMutexWait(UARTMutexHandle, osWaitForever);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); // Send mode
			  HAL_UART_Transmit(&huart1, TXData, 4, HAL_MAX_DELAY);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // Receive mode
			  osMutexRelease(UARTMutexHandle);
		  }
	  }
    osDelay(50);
  }
  /* USER CODE END StartStroomregeling */
}

/* USER CODE BEGIN Header_StartModeSwitchTask */
/**
* @brief Function implementing the ModeSwitchTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartModeSwitchTask */
void StartModeSwitchTask(void const * argument)
{
  /* USER CODE BEGIN StartModeSwitchTask */
  /* Infinite loop */

	//1 is intern, 0 is extern
	bool regelingMode = 1;

  for(;;)
  {
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9); //Running led

	 uint16_t adc_value = ADCChannelSamples(ADC_CHANNEL_9, 16); //9 is de externe V channel

	if((adc_value >= 620) && (regelingMode == 1)){ //rond een halve volt over de ADC, rond 1,5V voor extV
		//Intern naar extern
		regelingMode = 0;
        xQueueGenericSend(modeQueueHandle, &regelingMode, portMAX_DELAY, queueSEND_TO_BACK);
	}
	//4 mA voor 51 ohm is adc 82, onder de 70 zal er dus geen regeling zijn
	if((adc_value <= 70) && (regelingMode == 0)){
		//Extern naar intern
		regelingMode = 1;
        xQueueGenericSend(modeQueueHandle, &regelingMode, portMAX_DELAY, queueSEND_TO_BACK);
	}

    osDelay(1000);
  }
  /* USER CODE END StartModeSwitchTask */
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
  if (htim->Instance == TIM1)
  {
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
