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
#include "string.h"
#include "stdio.h"
#include "thermistor.h"
#include "math.h"
#include "stdlib.h"
#include "driver_queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIN_TEMP    					-300
#define MAX_TEMP     					1000
#define MaxFrameIndex 				255


unsigned char MY_SLAVE_ID;
volatile unsigned char ResponseFrameSize;

volatile unsigned char data_in[MaxFrameIndex+1];
volatile unsigned char data_out[MaxFrameIndex+1];
volatile uint8_t DataPos = 0;
volatile unsigned int TotalCharsReceived;

static uint16_t Size_buff;
static int temp[MaxFrameIndex+1];
static uint8_t count = 0;
static unsigned int counter = 0;
static uint8_t checkavr = 0;
static unsigned char info[88] = "#ACT01-DS18B20/FW:STM8S_StdPeriph_Driver-Version:2.3.0/ID:28-72-E0-83-0B-00-00-A3/80\r\n";

static uint32_t timout0 = 0;
static uint32_t timout1 = 0; 
static uint32_t timout2 = 0; 
static uint32_t timout3 = 0; 
static uint8_t RS_OK;

uint8_t buffer;
uint8_t flagtest = 0;
uint8_t error_check = 0;

int bleng;
unsigned char buff1[128];

static unsigned char nsec[256];
char str[30];
char CS;
Queue queue;
uint8_t bufx[UART_RXSIZE_QUEUE];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
void MBSendData(unsigned char count, unsigned char data[]);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static char NMEA_checkSum(char *nmea_data, int length)
{
  int crc = 0;
  int i;
  for (i = 0; i < length; i++)
  {
    crc ^= nmea_data[i];
  }
  return (char)crc;
}
void MBSendData(unsigned char count, unsigned char data[])	//Send final data over UART
{
	for (unsigned char c=0; c<count;c++)
	{
		while( !( USART2->ISR & (1<<7u) ) ) {};	//wait till transmit buffer is empty
		USART2->TDR = data[c];
	}
}
void temp_cal(uint16_t beta_value)
{
	uint16_t ADC_VAL;
	ADC_Start(11);
	ADC_WaitForConv();
	ADC_VAL = ADC_GetVal(); 
	voltage = (float)(ADC_VAL*1.0) * VCC / 4095.0;
    
  thermistorResistor = RESISTOR2_VALUE * voltage /(VOLTAGE_PULLUP - voltage); // Voltage divider equation
  temperatureK = 1 / (
                        log( thermistorResistor / RT_AT_ROOM_TEMP ) / beta_value +
                        1 / ROOM_TEMP );
     
  temperatureC = (temperatureK - 273.15 + 1)*10; // Temp in °C*10
	temp[count++] = temperatureC;
	if((temperatureC < MIN_TEMP)||(temperatureC > MAX_TEMP)||(ADC_VAL == 4095))
	{
		counter = 0; 
	}
	else
	{
		counter++;
	}
}
void Data_process()
{
	uint8_t index = 0;
	int readbyte;
	while(queue.qout != queue.qin)
	{
		readbyte = Queue_read(&queue);
		data_in[DataPos++] = readbyte;
		if(readbyte == '#')
		{
			if((strncmp((char*)data_in, "ping",4) == 0)&&(strlen((char*)data_in) == 5))
			{
				printf("#OK,0B\r\n");
				RS_OK = 1;
				flagtest = 1;
				Size_buff = 0;
				if(Size_buff == 0)
				{
					checkavr = 0;
				}
			}
			
			else if((strncmp((char*)data_in, "ping",4) == 0)&&(strlen((char*)data_in) > 5))
			{
				RS_OK = 1;
				index = strlen((char*)data_in);
				memcpy(&nsec, (unsigned char*)&data_in[4], index - 5);
				for(int i = 0 ; i<index - 5; i++)
				{
					if((nsec[i] != '0')&&(nsec[i] != '1')&&(nsec[i] != '2')&&(nsec[i] != '3')&&(nsec[i] != '4')&&(nsec[i] != '5')&&(nsec[i] != '6')&&(nsec[i] != '7')&&(nsec[i] != '8')&&(nsec[i] != '9'))
					{
						error_check++;
					}
				}

				if(error_check > 0)
				{
					memset((unsigned char*)nsec, 0, index - 5);
					flagtest = 1;
					error_check = 0;
				}		
				Size_buff = atoi((const char*)nsec);
				memset((unsigned char*)nsec, 0, index - 5);
				if(Size_buff != 0)
				{
					checkavr = 1;
					count = 1;
				}
				else 
				{
					checkavr = 0;
				}
				if(checkavr == 1)
				{
					for(int j = 1; j < Size_buff; j++)
					{
						temp[Size_buff] += temp[j];
					}
					temp[0] =  (int)(temp[Size_buff] / (Size_buff));	
					flagtest = 2;
				}
			}

			
		 if ((strncmp((char*)data_in, "ping",4) != 0))
			{
				RS_OK = 0;
				printf("#OK,0B\r\n");
				NVIC_SystemReset();
			}		
			memset((unsigned char*)data_in, 0, MaxFrameIndex);	
			DataPos = 0;
		}
	}
	if(counter > 1000)
	{
			switch (flagtest)
			{
				case 1:
				{
					if(HAL_GetTick() - timout1 > 999)
					{
						sprintf(str, "#Temperature,%d,",(int)temperatureC);
						CS = NMEA_checkSum(str, (int) (strlen(str)));
						printf("#Temperature,%d,%X\r\n", (int)temperatureC, CS);
						timout1 = HAL_GetTick();
					}
				}
				break;
				case 2:
				{
					if(HAL_GetTick() - timout2 > (999*Size_buff))
					{
						sprintf(str, "#Temperature,%d,",temp[0]);
						CS = NMEA_checkSum(str, (int) (strlen(str)));
						printf("#Temperature,%d,%X\r\n", temp[0],CS);
						timout2 = HAL_GetTick();
					}
				}
				break;
			}
		}
		else if(counter == 0)
		{	
			if(HAL_GetTick() - timout3 > 999)
				{
					sprintf(str, "#Temperature,32767,");
					CS = NMEA_checkSum(str, (int) (strlen(str)));
					printf("#Temperature,32767,%X\r\n",CS);
					timout3 = HAL_GetTick();
				}
		}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	RS_OK = 0;
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
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
	HAL_Delay(2000);
	printf("#ACT01-DS18B20/FW:STM8S_StdPeriph_Driver-Version:2.3.0/ID:28-72-E0-83-0B-00-00-A3/80\r\n");
	Queue_Init(&queue,bufx,UART_RXSIZE_QUEUE);
	LL_ADC_Enable(ADC1);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
//		IWDG->KR = 0xAAAA;
		HAL_IWDG_Refresh(&hiwdg);		
		
		temp_cal(3950);
		Data_process();
		
		if(RS_OK == 0)
		{
			if(HAL_GetTick() - timout0 > 19999)
			{
				MBSendData(88, info);
				timout0 = HAL_GetTick();
			}		
		}	
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  sConfig.Channel = ADC_CHANNEL_11;
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
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Window = 625;
  hiwdg.Init.Reload = 625;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  /* USER CODE BEGIN USART2_Init 2 */
	 USART2->CR1 |=(1<<5U);
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
__STATIC_INLINE uint32_t USART_IsActiveFlag_TXE(USART_TypeDef *USARTx)
{
  return ((READ_BIT(USARTx->ISR, USART_ISR_TXE_TXFNF) == (USART_ISR_TXE_TXFNF)) ? 1UL : 0UL);
}
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
	while(!USART_IsActiveFlag_TXE(USART2)){;}
	USART2->TDR = ch;
	return ch;
}
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
