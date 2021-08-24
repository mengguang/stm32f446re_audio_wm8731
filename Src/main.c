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
  * Copyright (c) 2019 STMicroelectronics International N.V. 
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
#include "i2s.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "wm8731.h"
#include "stdbool.h"
#include "ff.h"
#include "adpcm.h"
#include <stdio.h>
#include "string.h"
#include "stm32_adafruit_sd.h"
#include "sd_diskio.h"
#include "misc_utils.h"
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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len) {
	(void) file; /* Not used, avoid warning */
//	SEGGER_RTT_Write(0, ptr, len);
	HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 1000);
	return len;
}

gpio_t sd_cs = {SD_CS_GPIO_Port,SD_CS_Pin};
gpio_t i2s_csb = {CSB_GPIO_Port,CSB_Pin};

void cs_pins_init() {
	digitalWrite(sd_cs,HIGH);
	digitalWrite(i2s_csb,LOW);
}

FATFS SD_FatFs;

bool mass_storage_mount() {
	FATFS_UnLinkDriver(USERPath);
	uint8_t retUSER = FATFS_LinkDriver(&SD_Driver, USERPath);
	if (retUSER == 0) {
		/* Initialize the SD mounted on adafruit 1.8" TFT shield */
		if (BSP_SD_Init() != MSD_OK) {
			printf("BSP_SD_Init error.\n");
			return false;
		}

		/* Check the mounted device */
		if (f_mount(&SD_FatFs, (TCHAR const*) "/", 0) != FR_OK) {
			printf("f_mount error.\n");
			return false;
		}
	}
	return true;
}

__IO uint8_t I2S_TX_CPLT = 0;

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
 /* Prevent unused argument(s) compilation warning */
 UNUSED(hi2s);
 I2S_TX_CPLT = 1;
}


//__IO uint8_t I2S_TX_HALF_CPLT = 0;

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
 /* Prevent unused argument(s) compilation warning */
 UNUSED(hi2s);
 I2S_TX_CPLT = 1;
}

uint8_t block[2048];
uint16_t samples[2][4082];

/**
 * Support play ADPCM WAV file.
 * Audio: IMA ADPCM 48000Hz stereo 385kbps
 * [A: adpcm_ima_wav, 48000 Hz, 2 channels, s16, 385 kb/s]
 * Example:
 *
 * General
 * Format                         : Wave
 * File size                      : 11.5 MiB
 * Duration                       : 4 min 10 s
 * Overall bit rate mode          : Constant
 * Overall bit rate               : 384 kb/s
 *
 * Audio
 * Format                         : ADPCM
 * Codec ID                       : 11
 * Codec ID/Hint                  : Intel
 * Duration                       : 4 min 10 s
 * Bit rate mode                  : Constant
 * Bit rate                       : 384 kb/s
 * Channel(s)                     : 2 channels
 * Sampling rate                  : 48.0 kHz
 * Bit depth                      : 4 bits
 * Stream size                    : 11.5 MiB (100%)
 *
 */

void ADPCM_Play_File(FIL *file) {
	FRESULT result;
	unsigned int nread = 0;
	f_rewind(file);
	IMA_ADPCM_HEADER header;
	result = f_read(file,&header,sizeof(header),&nread);
	if(result != FR_OK) {
		printf("f_read error: %d\n",result);
		while(1);
	}
	ADPCM_WaveParsing(&header);

	int8_t samples_index = 0;

	memset(samples,0,4082*2*2);
	HAL_I2S_Transmit_DMA(&hi2s2, samples[0], 4082*2);

	while(!f_eof(file)) {
		result = f_read(file,block,sizeof(block),&nread);
		if(result != FR_OK) {
			printf("f_read error: %d\n",result);
			while(1);
		}

		while(I2S_TX_CPLT == 0);
		I2S_TX_CPLT = 0;

		int32_t left_predsample = *((int16_t *)(block+0));
		int16_t left_index = block[2];

		int32_t right_predsample = *((int16_t *)(block+4));
		int16_t right_index = block[6];

		//memset(samples[samples_index],0,4082*2);

		int samples_i = 0;
		samples[samples_index][samples_i] = left_predsample;
		samples_i++;
		samples[samples_index][samples_i] = right_predsample;
		samples_i ++;

		uint8_t code;

		ADPCM_Block_Init(left_index,left_predsample);
		for(int i=8;i<2048;) {
			//left channel
			code = (block[i] & 0x0F);
			samples[samples_index][samples_i] = ADPCM_Decode(code);
			samples_i += 2;
			code = (block[i] >> 4);
			samples[samples_index][samples_i] = ADPCM_Decode(code);
			samples_i += 2;

			i++;
			code = (block[i] & 0x0F);
			samples[samples_index][samples_i] = ADPCM_Decode(code);
			samples_i += 2;
			code = (block[i] >> 4);
			samples[samples_index][samples_i] = ADPCM_Decode(code);
			samples_i += 2;

			i++;
			code = (block[i] & 0x0F);
			samples[samples_index][samples_i] = ADPCM_Decode(code);
			samples_i += 2;
			code = (block[i] >> 4);
			samples[samples_index][samples_i] = ADPCM_Decode(code);
			samples_i += 2;

			i++;
			code = (block[i] & 0x0F);
			samples[samples_index][samples_i] = ADPCM_Decode(code);
			samples_i += 2;
			code = (block[i] >> 4);
			samples[samples_index][samples_i] = ADPCM_Decode(code);
			samples_i += 2;

			i+=5;
		}

		samples_i = 3;
		ADPCM_Block_Init(right_index,right_predsample);
		for(int i=12;i<2048;) {
			//right channel
			code = (block[i] & 0x0F);
			samples[samples_index][samples_i] = ADPCM_Decode(code);
			samples_i += 2;
			code = (block[i] >> 4);
			samples[samples_index][samples_i] = ADPCM_Decode(code);
			samples_i += 2;

			i++;
			code = (block[i] & 0x0F);
			samples[samples_index][samples_i] = ADPCM_Decode(code);
			samples_i += 2;
			code = (block[i] >> 4);
			samples[samples_index][samples_i] = ADPCM_Decode(code);
			samples_i += 2;

			i++;
			code = (block[i] & 0x0F);
			samples[samples_index][samples_i] = ADPCM_Decode(code);
			samples_i += 2;
			code = (block[i] >> 4);
			samples[samples_index][samples_i] = ADPCM_Decode(code);
			samples_i += 2;

			i++;
			code = (block[i] & 0x0F);
			samples[samples_index][samples_i] = ADPCM_Decode(code);
			samples_i += 2;
			code = (block[i] >> 4);
			samples[samples_index][samples_i] = ADPCM_Decode(code);
			samples_i += 2;

			i+=5;
		}
		samples_index = 1 - samples_index;
	}
	HAL_I2S_DMAStop(&hi2s2);
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
  MX_USART2_UART_Init();
  MX_I2S2_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  cs_pins_init();
  wm8731_dac_init();

  if(!mass_storage_mount()) {
	  Error_Handler();
  }

  FIL file;
  FRESULT result = f_open(&file,"adpcm.wav",FA_READ);
  if(result != FR_OK) {
	  printf("f_open error: %d\n",result);
	  while(1);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  printf("Begin to play music!\n");
	  //Play music in SDIO SD Card.
	  f_rewind(&file);
	  ADPCM_Play_File(&file);
	  delay(1000);

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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
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
