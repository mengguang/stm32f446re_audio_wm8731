/**
  ******************************************************************************
  * @file ADPCM/inc/adpcm.h 
  * @author  MCD Application Team
  * @version  V2.0.0
  * @date  04/27/2009
  * @brief  Header file for adpcm.c
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADPCM_H
#define __ADPCM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdbool.h"


/* Exported types ------------------------------------------------------------*/
// VS1053 Datasheet 10.8.4
typedef struct {
	uint8_t		ChunkID[4];
	uint32_t 	ChunkSize;
	uint8_t 	Format[4];
	uint8_t 	SubChunk1ID[4];
	uint32_t	SubChunk1Size;
	uint16_t	AudioFormat;
	uint16_t	NumberOfChannels;
	uint32_t	SampleRate;
	uint32_t	ByteRate;
	uint16_t	BlockAlign;
	uint16_t	BitsPerSample;
	uint16_t	ByteExtraData;
	uint16_t	SamplesPerBlock;
	uint8_t		SubChunk2ID[4];
	uint32_t	SubChunk2Size;
	uint32_t	NumOfSamples;
	uint8_t		SubChunk3ID[4];
	uint32_t	DataSize;
} IMA_ADPCM_HEADER;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
uint8_t ADPCM_Encode(int32_t sample);
int16_t ADPCM_Decode(uint8_t code);
void ADPCM_Block_Init(int16_t _index, int32_t _predsample);

bool ADPCM_WaveParsing(IMA_ADPCM_HEADER *header);

#endif /* __ADPCM_H*/
/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
