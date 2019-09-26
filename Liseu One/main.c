/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cansat_includes.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t spi_sendrecv(uint8_t byte);

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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_FATFS_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */


//  uint8_t c;
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
//  spi_sendrecv(0x9f); //#define R_JEDEC_ID	0x9f
//  c = spi_sendrecv(0x00);
//  spi_sendrecv(0x00);
//  spi_sendrecv(0x00);
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
//


  // uint16_t flashRead(uint32_t addr, uint8_t *buf, uint16_t n)
  // void flashWritePage(uint32_t addr_start,uint8_t *buf)

  uint32_t addr_start = 0;
  uint8_t buf[256]; // byte
  for(uint8_t i=0; i<256; i++)
  {
	  buf[i]=i;
  }
  flashWritePage(addr_start, buf);









	// Barometer variables
	//MS5611 baroData;
	MS5611_result baroResult;
	MS5611 ms5611;
	ms5611.initialized = 0;


	baroResult = MS5611_init(&hi2c1, &ms5611, MS5611_STANDARD);
	if( MS5611_OK != baroResult )
	{
		log_data(LOG_TYPE_WARRNING, LOG_DEVICE_MS5611, "MS5611 init error!");
	}
	else   // Calibrate zero level for 'MS5611_calculateAltitude':
	{
		HAL_Delay(200);
		MS5611_requestTemperature();
		HAL_Delay(100);
		MS5611_getTemperature( &ms5611 );

		MS5611_requestPressure();
		HAL_Delay(100);
		MS5611_getPressure( &ms5611 );

		MS5611_setLevelZero( &ms5611 );
		log_data(LOG_TYPE_INFO, LOG_DEVICE_MS5611, "MS5611 init successed");
	}














 uint16_t properMPUName = 0x71;
 uint8_t mpuInitRes = MPU9250Init(properMPUName);
 int16_t acc[3];
 readAccelData(&acc);


 typedef struct {
   float processNoise;
   float measurementNoise;
   float value;
   float error;
   float gain;
 } kalman_t;

 kalman_t z_axis;
 z_axis.processNoise = 0.01;
 z_axis.measurementNoise = 0.25;
 z_axis.error = 1;
 z_axis.value = 0;

 kalman_t pressure;
 pressure.processNoise = 0.05;
 pressure.measurementNoise = 0.4;
 pressure.error = 1;
 pressure.value = 0;




	uint16_t log_length;
	char log_buffer[32] = {0};

	int32_t measurement;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		readAccelData(&acc);

		measurement = acc[2];
	  // Prediction update
	  z_axis.error = z_axis.error + z_axis.processNoise;
	  // Measurement update
	  z_axis.gain = z_axis.error / (z_axis.error + z_axis.measurementNoise);
	  z_axis.value = z_axis.value + z_axis.gain * (measurement - z_axis.value);
	  z_axis.error = (1 - z_axis.gain) * z_axis.error;




//	    log_length = sprintf(log_buffer, "[%lu] %d %d %d\n", HAL_GetTick(),acc[0], acc[1], acc[2]);
//		HAL_UART_Transmit(&huart2, log_buffer, log_length, HAL_MAX_DELAY);

		//log_length = sprintf(log_buffer, "%d %d\n", measurement, (int)z_axis.value);
		//HAL_UART_Transmit(&huart2, log_buffer, log_length, HAL_MAX_DELAY);

	  	MS5611_getPressure( &ms5611 );
		MS5611_requestPressure();

		measurement = ms5611.pressure;
		// Prediction update
		pressure.error = pressure.error + pressure.processNoise;
		// Measurement update
		pressure.gain = pressure.error / (pressure.error + pressure.measurementNoise);
		pressure.value = pressure.value + pressure.gain * (measurement - pressure.value);
		pressure.error = (1 - pressure.gain) * pressure.error;

		log_length = sprintf(log_buffer, "%d %d\n", measurement, (int)pressure.value);
		//log_length = sprintf(log_buffer, "%d\n", ms5611.pressure);
		HAL_UART_Transmit(&huart2, log_buffer, log_length, HAL_MAX_DELAY);


		HAL_Delay(100);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint8_t spi_sendrecv(uint8_t byte)
{
 uint8_t answer;

 HAL_SPI_TransmitReceive(&hspi1, &byte, &answer, 1, HAL_MAX_DELAY);

 return answer;
}



bool flashBusy()
{
  uint8_t r1;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  transfer(0x05); //#define R_SR1	0x05	//read status reg 1
  r1 = transfer(0xff);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  if(r1 & 0x01) //#define SR1_BUSY_MASK	0x01
    return true;
  return false;
}

uint16_t flashRead(uint32_t addr, uint8_t *buf, uint16_t n)
{
  if(flashBusy())
    return 0;

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  transfer(0x03); //#define READ		0x03
  transfer(addr>>16);
  transfer(addr>>8);
  transfer(addr);
  for(uint16_t i=0;i<n;i++)
  {
    buf[i] = transfer(0x00);
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);


  return n;
}



void flashWritePage(uint32_t addr_start,uint8_t *buf)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  transfer(0x02); //#define PAGE_PGM	0x02	//page program
  transfer(addr_start>>16);
  transfer(addr_start>>8);
  transfer(0x00);
  uint8_t i=0;
  do {
    transfer(buf[i]);
    i++;
  }while(i!=0);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

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
