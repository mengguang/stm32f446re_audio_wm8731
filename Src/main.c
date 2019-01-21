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
#include "sdio.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SEGGER_RTT.h"
#include "wm8731.h"
#include "stdbool.h"
#include "ff.h"
#include "adpcm.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Audio file size and start offset address are defined here since the audio wave file is
   stored in Flash memory as a constant table of 16-bit data
 */
#define AUDIO_FILE_SIZE               411444        /* Size of audio file */
#define AUDIO_START_OFFSET_ADDRESS    44            /* Offset relative to audio file header size */
#define AUDIO_FILE_ADDRESS            0x08010000    /* Audio file address */
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

// Attention Please!
// You must enable pull up on SDIO interface.

int _write(int file, char *ptr, int len) {
	(void) file; /* Not used, avoid warning */
	SEGGER_RTT_Write(0, ptr, len);
	return len;
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
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(100);
  MX_I2C1_Init();
  wm8731_dac_init();

  //while(1);

  unsigned int nread = 0;

  FIL file;
  FRESULT result = f_open(&file,"adpcm.wav",FA_READ);
  if(result != FR_OK) {
	  printf("f_open error: %d\n",result);
	  while(1);
  }



  ADPCM_Play_File(&file);

  while(1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
	  printf("Begin to play music!\n");

	  //Play music in SDIO SD Card.
	  f_rewind(&file);
	  WAVE_FormatTypeDef header;

	  result = f_read(&file,&header,sizeof(WAVE_FormatTypeDef),&nread);
	  if(result != FR_OK) {
		  printf("f_read error: %d\n",result);
		  Error_Handler();
	  }
	  wm8731_display_wav_info(&header);

	  uint8_t read_buffer[2][16384];
	  //I2S_TX_CPLT = 1;
	  uint8_t read_buffer_index = 0;

	  result = f_read(&file,read_buffer[read_buffer_index],16384 * 2,&nread);
	  if(result != FR_OK) {
		  printf("f_read error: %d\n",result);
		  Error_Handler();
	  }
	  // circular buffer
	  HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t *)(read_buffer[read_buffer_index]), 16384);

	  while(!f_eof(&file)) {
		  while(I2S_TX_CPLT == 0);
		  I2S_TX_CPLT = 0;

		  printf("start read: %lu\n",f_tell(&file));
		  result = f_read(&file,read_buffer[read_buffer_index],16384,&nread);
		  if(result != FR_OK) {
			  printf("f_read error: %d\n",result);
			  Error_Handler();
		  }
		  printf("nread: %d\n",nread);
		  read_buffer_index = 1 - read_buffer_index;
	  }
	  HAL_I2S_DMAStop(&hi2s2);


	  //Play music from internal flash.
      /*
	  WAVE_FormatTypeDef *waveformat = (WAVE_FormatTypeDef*)AUDIO_FILE_ADDRESS;
      wm8731_display_wav_info(waveformat);
	  uint32_t begin = AUDIO_FILE_ADDRESS + AUDIO_START_OFFSET_ADDRESS;
	  uint32_t current = 0;
	  uint32_t total = AUDIO_FILE_SIZE - AUDIO_START_OFFSET_ADDRESS;
	  uint32_t block_size = 16384;
	  while(current < total) {
		  if(total - current < block_size) {
			  block_size = total - current;
		  }
		  printf("current: %lu\n",current);
		  HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t *)(begin + current), (block_size / 2));
		  while(I2S_TX_CPLT == 0);
		  I2S_TX_CPLT = 0;
		  current += block_size;
	  }
	  */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB1|RCC_PERIPHCLK_SDIO
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 50;
  PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLI2SP_DIV2;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 8;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
  PeriphClkInitStruct.PLLI2SDivQ = 1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
  PeriphClkInitStruct.SdioClockSelection = RCC_SDIOCLKSOURCE_CLK48;
  PeriphClkInitStruct.I2sApb1ClockSelection = RCC_I2SAPB1CLKSOURCE_PLLI2S;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
