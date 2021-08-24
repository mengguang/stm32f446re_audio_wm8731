/*
 * wm8731.c
 *
 *  Created on: 2019��1��21��
 *      Author: mengguang
 */


#include "wm8731.h"
#include <stdio.h>

void wm8731_display_wav_info(WAVE_FormatTypeDef * format)
{
  printf("Sampling frequency : %lu Hz\n", format->SampleRate);
  if (format->NbrChannels == 2)
  {
    printf("Format : %d bits stereo\n", format->BitPerSample);
  }
  else if (format->NbrChannels == 1)
  {
    printf("Format : %d bits mono\n", format->BitPerSample);
  }
}


//------------------------------------------------------------------------------
/// Write data to WM8731 Register.
/// \param regAddr       register address to read.
/// \param data    data to write
//------------------------------------------------------------------------------
bool wm8731_reg_write(uint8_t regAddr, uint16_t data)
{
    uint8_t tmpData[2];
    uint16_t tmp;
    tmp = ((regAddr & 0x7f) << 9) | (data & 0x1ff);

    tmpData[0] = (tmp & 0xff00) >> 8;
    tmpData[1] = tmp & 0xff;
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(&hi2c1, WM8731_SLAVE_ADDRESS << 1, tmpData, 2, 1000);
    if(st != HAL_OK) {
    	printf("WM8731_Write HAL_I2C_Master_Transmit error: %d\n",st);
    	return false;
    }
    return true;
}

//------------------------------------------------------------------------------
/// Init WM8731 to DAC mode.
//------------------------------------------------------------------------------
bool wm8731_dac_init()
{
    // reset
    wm8731_reg_write(WM8731_REG_RESET, 0);

    // analogue audio path control
    wm8731_reg_write(WM8731_REG_ANALOGUE_PATH_CTRL, 0x12);

    // digital audio path control
    wm8731_reg_write(WM8731_REG_DIGITAL_PATH_CTRL, 0x00);

    // power down control
    wm8731_reg_write(WM8731_REG_PWDOWN_CTRL, 0x7);

    // digital auduo interface format
    wm8731_reg_write(WM8731_REG_DA_INTERFACE_FORMAT,0b00010010);

    // sampling control
    //wm8731_reg_write(WM8731_REG_SAMPLECTRL,0b00000000);

    // volume
    wm8731_reg_write(WM8731_REG_LEFT_HPOUT, 0b01100001);
    wm8731_reg_write(WM8731_REG_RIGHT_HPOUT,0b01100001);

    // Active control
    wm8731_reg_write(WM8731_REG_ACTIVE_CTRL, 0x01);

    return true;
}

