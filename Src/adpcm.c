/**
 ******************************************************************************
 * @file ADPCM/src/adpcm.c
 * @author  MCD Application Team
 * @version  V2.0.0
 * @date  04/27/2009
 * @brief  This is ADPCM decoder/encoder souce file
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


/* Includes ------------------------------------------------------------------*/
#include "adpcm.h"
#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "stdio.h"

/* Private define ------------------------------------------------------------*/
/* Quantizer step size lookup table */
const uint16_t StepSizeTable[89]={7,8,9,10,11,12,13,14,16,17,
		19,21,23,25,28,31,34,37,41,45,
		50,55,60,66,73,80,88,97,107,118,
		130,143,157,173,190,209,230,253,279,307,
		337,371,408,449,494,544,598,658,724,796,
		876,963,1060,1166,1282,1411,1552,1707,1878,2066,
		2272,2499,2749,3024,3327,3660,4026,4428,4871,5358,
		5894,6484,7132,7845,8630,9493,10442,11487,12635,13899,
		15289,16818,18500,20350,22385,24623,27086,29794,32767};
/* Table of index changes */
const int8_t IndexTable[16]={0xff,0xff,0xff,0xff,2,4,6,8,0xff,0xff,0xff,0xff,2,4,6,8};

/* Audio Parsing Constants */
#define  CHUNK_ID             0x52494646  /* correspond to the letters 'RIFF' */
#define  FILE_FORMAT          0x57415645  /* correspond to the letters 'WAVE' */
#define  FORMAT_ID            0x666D7420  /* correspond to the letters 'fmt ' */
#define  DATA_ID              0x64617461  /* correspond to the letters 'data' */
#define  FACT_ID              0x66616374  /* correspond to the letters 'fact' */
/* Wave file format */
#define  WAVE_FORMAT_PCM        0x01
#define  WAVE_FORMAT_ADPCM      0x11

#define  FORMAT_CHUNK_SIZE    0x10
#define  CHANNEL_MONO         0x01
#define  CHANNEL_STEREO       0x02
/* Audio Frequencies */
#define  SAMPLE_RATE_8000     8000
#define  SAMPLE_RATE_16000    16000
#define  SAMPLE_RATE_22050    22050
#define  SAMPLE_RATE_44100    44100
#define  SAMPLE_RATE_48000    48000
/* Data format */
#define  BITS_PER_SAMPLE_8    8
#define  BITS_PER_SAMPLE_16   16

#define DUMMY_DATA            0x1111



/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


bool ADPCM_WaveParsing(IMA_ADPCM_HEADER *header)
{
	char str_buffer[16];
	memset(str_buffer,0,sizeof(str_buffer));
	memcpy(str_buffer,header->ChunkID,sizeof(header->ChunkID));
	printf("ChunkID: %s\n",str_buffer);
	printf("ChunkSize: %lu\n",header->ChunkSize);
	memset(str_buffer,0,sizeof(str_buffer));
	memcpy(str_buffer,header->Format,sizeof(header->Format));
	printf("ChunkID: %s\n",str_buffer);
	printf("AudioFormat: %u\n",header->AudioFormat);
	printf("Number of Channels: %u\n",header->NumberOfChannels);
	printf("Sample Rate: %lu\n",header->SampleRate);
	printf("Block Align: %u\n",header->BlockAlign);
	printf("Bits per sample: %u\n",header->BitsPerSample);
	printf("Samples per block: %u\n",header->SamplesPerBlock);
	printf("Number of samples: %lu\n",header->NumOfSamples);
	printf("Data size: %lu\n",header->DataSize);
	return true;
}

bool WAV_PCM_WaveParsing(WAV_PCM_HEADER *header)
{
	char str_buffer[16];
	memset(str_buffer,0,sizeof(str_buffer));
	memcpy(str_buffer,header->ChunkID,sizeof(header->ChunkID));
	printf("ChunkID: %s\n",str_buffer);
	printf("ChunkSize: %lu\n",header->ChunkSize);
	memset(str_buffer,0,sizeof(str_buffer));
	memcpy(str_buffer,header->Format,sizeof(header->Format));
	printf("ChunkID: %s\n",str_buffer);
	printf("AudioFormat: %u\n",header->AudioFormat);
	printf("Number of Channels: %u\n",header->NumberOfChannels);
	printf("Sample Rate: %lu\n",header->SampleRate);
	printf("Block Align: %u\n",header->BlockAlign);
	printf("Bits per sample: %u\n",header->BitsPerSample);
	printf("Data size: %lu\n",header->DataSize);
	return true;
}


int16_t adpcm_index = 0;
int32_t predsample = 0;

void ADPCM_Block_Init(int16_t _index, int32_t _predsample) {
	adpcm_index = _index;
	predsample = _predsample;
}

/**
 * @brief  ADPCM_Encode.
 * @param sample: a 16-bit PCM sample
 * @retval : a 4-bit ADPCM sample
 */
uint8_t ADPCM_Encode(int32_t sample)
{
	//  static int16_t  index = 0;
	//  static int32_t predsample = 0;
	uint8_t code=0;
	uint16_t tmpstep=0;
	int32_t diff=0;
	int32_t diffq=0;
	uint16_t step=0;

	step = StepSizeTable[adpcm_index];

	/* 2. compute diff and record sign and absolut value */
	diff = sample-predsample;
	if (diff < 0)
	{
		code=8;
		diff = -diff;
	}

	/* 3. quantize the diff into ADPCM code */
	/* 4. inverse quantize the code into a predicted diff */
	tmpstep = step;
	diffq = (step >> 3);

	if (diff >= tmpstep)
	{
		code |= 0x04;
		diff -= tmpstep;
		diffq += step;
	}

	tmpstep = tmpstep >> 1;

	if (diff >= tmpstep)
	{
		code |= 0x02;
		diff -= tmpstep;
		diffq+=(step >> 1);
	}

	tmpstep = tmpstep >> 1;

	if (diff >= tmpstep)
	{
		code |=0x01;
		diffq+=(step >> 2);
	}

	/* 5. fixed predictor to get new predicted sample*/
	if (code & 8)
	{
		predsample -= diffq;
	}
	else
	{
		predsample += diffq;
	}

	/* check for overflow*/
	if (predsample > 32767)
	{
		predsample = 32767;
	}
	else if (predsample < -32768)
	{
		predsample = -32768;
	}

	/* 6. find new stepsize index */
	adpcm_index += IndexTable[code];
	/* check for overflow*/
	if (adpcm_index <0)
	{
		adpcm_index = 0;
	}
	else if (adpcm_index > 88)
	{
		adpcm_index = 88;
	}

	/* 8. return new ADPCM code*/
	return (code & 0x0f);
}



/**
 * @brief  ADPCM_Decode.
 * @param code: a byte containing a 4-bit ADPCM sample.
 * @retval : 16-bit ADPCM sample
 */
int16_t ADPCM_Decode(uint8_t code)
{
	//  static int16_t  index = 0;
	//  static int32_t predsample = 0;
	uint16_t step=0;
	int32_t diffq=0;

	step = StepSizeTable[adpcm_index];

	/* 2. inverse code into diff */
	diffq = step>> 3;
	if (code&4)
	{
		diffq += step;
	}

	if (code&2)
	{
		diffq += step>>1;
	}

	if (code&1)
	{
		diffq += step>>2;
	}

	/* 3. add diff to predicted sample*/
	if (code&8)
	{
		predsample -= diffq;
	}
	else
	{
		predsample += diffq;
	}

	/* check for overflow*/
	if (predsample > 32767)
	{
		predsample = 32767;
	}
	else if (predsample < -32768)
	{
		predsample = -32768;
	}

	/* 4. find new quantizer step size */
	adpcm_index += IndexTable [code];
	/* check for overflow*/
	if (adpcm_index < 0)
	{
		adpcm_index = 0;
	}
	if (adpcm_index > 88)
	{
		adpcm_index = 88;
	}

	/* 5. save predict sample and index for next iteration */
	/* done! static variables */

	/* 6. return new speech sample*/
	return ((int16_t)predsample);
}

/**
 * @}
 */


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
