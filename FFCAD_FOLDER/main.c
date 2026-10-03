/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - FF-CAD_GEN2 09/18/2026 - Joseph Haas, KE0FF
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  *
  * FF-CAD GEN2 ACC-Version Code (c) 2026 by Joseph Haas, KE0FF
  *
  * Uses TMR5 to set a 10KHz interrupt where ADC samples are gathered, triggered,
  * 	and buffered to produce a high-quality delay line using the STM32 SRAM.
  * 	The SRAM array uses a head/tail buffer structure with the head and tail
  * 	differential corresponding to the desired delay (in number of samples).
  *
  * For the ACC mode (SPARE_0 = GND), the delay is fixed at 76.8 ms and the
  * 	inhibit input is ignored.  SPARE_0 is only tested at reset, so a power-cycle
  * 	is required to update this setting.  If SPARE_0 = open (high), the DLY[3:0]
  * 	and DLY_INH_1, and DLY250_0 signals are processed which allows variable delay
  * 	and the selection of 256ms of max delay or 128ms of max delay.
  *
  * DLY[3:0] specifies the delay setting.  The delay in ms is found with the
  * 	following equation:
  *
  * 	delay = ((DLY[3:0] * MAX_DELAY)/16)/10, where MAX_DELAY is either 2560 or 1280 samples
  *
  * DLY_INH_1 active sets the delay to 0.  This allows the delay to be "turned off"
  * 	using a single logic level signal.  The inhibit affects the "playback"
  * 	of the audio stream, not the record, so there is no loss of audio when the
  * 	inhibit is activated or deactivated.
  *
  * All of the signals except SPARE_0 may be changed at any time without needing
  * 	a power-cycle reset.
  *
  * Resource allocations:
  * DLY3:			PC0 - delay bit 3 (8)
  * DLY2:			PC0 - delay bit 2 (4)
  * DLY1:			PC0 - delay bit 1 (2)
  * DLY0:			PC0 - delay bit 0 (1)
  * ADC_IN0:		PA0 - analog, audio input, 4Vpp max, 2.5V = center reference
  * PTTIN_1:		PA1 (future legacy FF-CAD compatibility support)
  * MUTE1_0:		PA2 (future legacy FF-CAD compatibility support)
  * DAC_OUT1:		PA5 - analog, audio output, 4Vpp max, 2.5V = 0reference
  * DLY250_0:		PA6 - selects 256 or 128 ms max delay
  * SPARE_0:		PA7 - selects ACC mode
  * TST_O:			PB13 - test LED (I'm alive/heartbeat) output
  * TP5:			PC10 - reserved solder strap
  * TP4:			PC11 - reserved solder strap
  * DLY_INH_1:		PB4 - delay inhibit input
  * PTTO_1:			PB5  (future legacy FF-CAD compatibility support)
  * USART1_TX:		PB6	- reserved (version reset banner message)
  * USART1_RX:		PB7	- reserved
  *
  * Unused GPIO is assigned as inputs with pullups enabled.
  *
  ******************************************************************************
  * REV NOTES
  * Rev 0.3 10/01/26 jmh
  * - Added test bench using build directive and re-tasking TEST output to be a
  * 	10ms burst of 1KHz square wave at 2 sec intervals to use as a precision test
  * 	stimulus.  Build and load on Nucleo.
  * - Debug random delay issues.  max_delay is now file local and is used as max_buffer
  * 	to set rollover point in the capture array.
  * - Re-worked delay calculation to correctly handle rollover.  Depends on max_delay
  * 	= max_buffer mod.
  * - Skewed DLY input by +1.  "0" = one delay step, 0xf = 16 delay steps. Zero delay
  * 	requires DLY_INH to be activated.
  * - Reworked init code to initialize ACC delay properly.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "stm32f4xx_hal_uart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;

#define	ACC_MODE	1				// ACC mode build define (set to 0 for FF-800 mode)
#define PTR_MAX		2560			// approx 0.25sec max delay (approx 0.125 sec for fast mode)
#define PTR_MAX_ACC	768				// approx 75 ms fixed delay for ACC mode
#define	MAX_BIN		16				// 16 delay settings (0-15)
#define	DLY_MASK	0x0f
#define	ADC_MID		0x8000			// mid-point of ADC/DAC range
volatile uint16_t	fifo[PTR_MAX];
volatile uint16_t	dac_hold;
volatile uint16_t	iptr;
volatile uint16_t	optr;
volatile uint8_t	hbflag;
volatile uint32_t	max_delay;

#ifdef	TESTSET
volatile uint8_t	tst_div;
volatile uint32_t	pulse_div;
#define	DIV_1K		5
#define	PULSE_PER	20000
#define	PULSE_DUR	100
#endif

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
uint8_t brev4(uint8_t dlyin);
int puts0(const char *pstr);
void putchar0(const char c);
char nybasc(uint8_t nyb);

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

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
  uint8_t	i;
  uint8_t	dly_last;
  uint8_t	inh_last;
  uint32_t	ii;

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
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_NVIC_DisableIRQ(TIM5_IRQn);														// hold TIMER ISR whil fiddling with the ISR registers/pointers
  for(ii=0; ii<PTR_MAX; ii++){															// init ADC buffer to mid-point (aero reference)
	  fifo[ii] = ADC_MID;
  }
  i = 0;																				// init mode reg
  dly_last = ((GPIOC->IDR) ^ 0xff) & DLY_MASK;											// force IPL update for delay setting and inhibit
  inh_last = ~(uint8_t)HAL_GPIO_ReadPin(DLY_INH_1_GPIO_Port, DLY_INH_1_Pin);
  if(!HAL_GPIO_ReadPin(SPARE_0_GPIO_Port, SPARE_0_Pin)){								// process ACC delay setting
	  i = 1;
	  max_delay = PTR_MAX_ACC;
	  iptr = 0;																			// init the fixed, ACC< I/O delay
	  optr = 1;
  }else{
	  i = 2;
	  if(HAL_GPIO_ReadPin(DLY250_0_GPIO_Port, DLY250_0_Pin)){							// process max delay setting for non-ACC mode
		  max_delay = PTR_MAX/2;
	  }else{
		  max_delay = PTR_MAX;
	  }
	  iptr = 0;																			// init the delay to 0 as a fail-safe
	  optr = iptr;
  }
  HAL_NVIC_EnableIRQ(TIM5_IRQn);														// ISR is clear to navigate...

#ifdef	TESTSET
  tst_div = 1;
  pulse_div = PULSE_PER;
#endif

  // IPL Banners...

#ifdef TESTSET
  puts0("\nFF-CAD GEN-II TEST SET VT.01 by Joseph Haas, KE0FF\n(c) 9/30/2026, All Rights Reserved"); // send version/copyright notice to UART
#else
  puts0("\nFF-CAD GEN-II V0.03 by Joseph Haas, KE0FF\n(c) 10/01/2026, All Rights Reserved"); // send version/copyright notice to UART
#endif

  switch(i){ 																			// send mode setting to UART
  case 0:
  default:
	  puts0("!! Mode ERROR !!");
	  break;

  case 1:
	  puts0("ACC Mode");
	  break;

  case 2:
	  puts0("FF-800 Mode");
	  break;
  }
  switch(max_delay){ 																	// send max buffer size to UART
  default:
	  puts0("!! SIZE ERROR !!");
	  break;

  case PTR_MAX_ACC:
	  puts0("76.8 ms");
	  break;

  case PTR_MAX/2:
  	  puts0("128 ms");
  	  break;

  case PTR_MAX:
	  puts0("256 ms");
	  break;
  }
  hbflag = 0;	// allow one toggle of the HB LED ... differentiates main code running from ISRs running

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(max_delay != PTR_MAX_ACC){									// delay setting and inhibit are only allowed if !ACC_MODE
		  ///////////// DLY SETTING PROCESSING //////////////////
		  i = (GPIOC->IDR) & DLY_MASK;								// get current delay setting
		  if(i != dly_last){										// if there is a change to delay, update tap index
			  dly_last = i;											// update last delay reg
			  ii = (uint32_t)brev4(i);								// brev delay and put in U32
			  putchar0(nybasc((uint8_t)ii));
			  putchar0('\n');
			  ii = (((ii + 1) * max_delay) / MAX_BIN) - 1;						// calculate delay index delta
			  // hold interrupts while updating ISR registers
			  HAL_NVIC_DisableIRQ(TIM5_IRQn);
			  if(ii){												// non-zero delays get processed here
				  ii = iptr - ii;
				  if(ii >= max_delay) ii += max_delay;
				  optr = ii;
			  }else{
				  optr = iptr;										// zero delay means the in and out pointers are the same
			  }
			  // enable interrupts
			  HAL_NVIC_EnableIRQ(TIM5_IRQn);
		  }
		  ///////////// DLY INH PROCESSING //////////////////
		  i = (uint8_t)HAL_GPIO_ReadPin(DLY_INH_1_GPIO_Port, DLY_INH_1_Pin);
		  if(i != inh_last){
			  inh_last = i;
			  if(i){
				  // hold interrupts to adjust optr
				  HAL_NVIC_DisableIRQ(TIM5_IRQn);
				  optr = iptr;										// zero delay
				  HAL_NVIC_EnableIRQ(TIM5_IRQn);
				  puts0("inh"); 															// send mode setting to UART
			  }else{
				  dly_last = ((GPIOC->IDR) ^ 0xff) & DLY_MASK;		// force delay update on next pass through main wait(1)
				  puts0("dly on"); 															// send mode setting to UART
			  }
		  }
	  }
#ifndef	TESTSET
	  if(!hbflag){
		  HAL_GPIO_TogglePin(TST_O_GPIO_Port, TST_O_Pin);			// process isr heartbeat ... this ensures that the TMR5 and the main loop are both running
		  hbflag = 1;												// re-arm ISR flag
	  }
#endif

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


/**
  * @brief bit-reverse input nybble into output nybble
  * @retval bit reversed low nybble of input value
  */
uint8_t brev4(uint8_t dlyin)
{
  uint8_t	i;		//temps
  uint8_t	j;
  uint8_t	q = 0;

  for(i=0x01,j=0x08; j; i<<=1,j>>=1){
	  if(i&dlyin) q|=j;
  }
  return q;
}


/**
  * @brief local, null-term'd, puts fn
  * @retval int 0 (per general puts() spec)
  */

int puts0(const char *pstr){
	while(*pstr){
		putchar0(*pstr++);
	}
	putchar0('\n');
	return 0;
}

/**
  * @brief local, putchar fn
  * @retval none
  */

void putchar0(const char c){

	if(c == '\n'){
		// wait for UART TXE == 1 //
		while((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) ? SET : RESET) == RESET);
		// send data to uart DR //
		huart1.Instance->DR = '\r';
	}
	// wait for UART TXE == 1 //
	while((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) ? SET : RESET) == RESET);
	// send data to uart DR //
	huart1.Instance->DR = c;
	return;
}

/**
  * @brief local, nybasc fn
  * @retval ascii hex char
  */

char nybasc(uint8_t nyb){
	char	c;

	c = (nyb & 0x0f) + '0';
	if(c > '9'){
		c += 'A' - '9' - 1;
	}
	return c;
}

//////////////// END OF MAIN() APP & Fns /////////////////////////

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  HAL_ADC_Start(&hadc1);
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T5_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */
  HAL_DAC_Start (&hdac, DAC_CHANNEL_1);
  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */
  __HAL_RCC_TIM5_CLK_ENABLE();

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1600;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);

  /* USER CODE END TIM5_Init 2 */

}

/*TIM5 handler
 *
 *
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PeriodElapsedHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_CaptureHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_TriggerHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim);
*/

void TIM5_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim5);  // Pass control to HAL interrupt handler
}

/* TIM5 callback
 *
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	volatile static uint32_t i;

	if (htim->Instance == TIM5)
	{
		i = HAL_ADC_GetValue(&hadc1);									// get ADC
		HAL_ADC_Start(&hadc1);											// start a new ADC sample
		fifo[iptr] = i;													// place ADC into buffer (input)
		i = fifo[optr];													// get DAC value from buffer (output)
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, i);		// set DAC
		if(++iptr >= max_delay){											// update input pointer (& process rollover)
			iptr = 0;
			hbflag = 0;													// clear the process heartbeat flag
		}
		if(++optr >= max_delay) optr = 0;									// update output pointer (& process rollover)
	}

#ifdef	TESTSET
	if(--pulse_div < PULSE_DUR){
		if(--tst_div == 0){
			  tst_div = DIV_1K;											// tst pulse
			  HAL_GPIO_TogglePin(TST_O_GPIO_Port, TST_O_Pin);			// process test pulse
		}
	}
	if(!pulse_div){
		pulse_div = PULSE_PER;
	}
#endif
}

/* This function is called by the HAL library when the TIM interrupt occurs */
/*void TIM5_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim5);
}*/

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TST_O_Pin|PTTO_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 DLY3_Pin DLY2_Pin DLY1_Pin
                           DLY0_Pin PC4 PC5 PC6
                           PC7 PC8 PC9 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|DLY3_Pin|DLY2_Pin|DLY1_Pin
                          |DLY0_Pin|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PH1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PTTIN_1_Pin PA11 PA12 */
  GPIO_InitStruct.Pin = PTTIN_1_Pin|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MUTE1_0_Pin PA3 DLY250_0_Pin SPARE_0_Pin */
  GPIO_InitStruct.Pin = MUTE1_0_Pin|GPIO_PIN_3|DLY250_0_Pin|SPARE_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB14 PB15 PB11 DLY_INH_1_Pin
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_11|DLY_INH_1_Pin
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TST_O_Pin PTTO_1_Pin */
  GPIO_InitStruct.Pin = TST_O_Pin|PTTO_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TP5_Pin TP4_Pin */
  GPIO_InitStruct.Pin = TP5_Pin|TP4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
