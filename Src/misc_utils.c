/*
 * misc_utils.c
 *
 *  Created on: Feb 26, 2019
 *      Author: mengguang
 */


#include "misc_utils.h"
#include "ctype.h"

//int _write(int file, char *ptr, int len) {
//	(void) file; /* Not used, avoid warning */
//	SEGGER_RTT_Write(0, ptr, len);
//	return len;
//}

void digitalWrite(gpio_t gpio, uint8_t val) {
	if(val == HIGH) {
		HAL_GPIO_WritePin(gpio.port, gpio.pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(gpio.port, gpio.pin, GPIO_PIN_RESET);
	}
}

void digitalToggle(gpio_t gpio) {
	if(digitalRead(gpio) == HIGH) {
		digitalWrite(gpio,LOW);
	} else {
		digitalWrite(gpio,HIGH);
	}
}

uint8_t digitalRead(gpio_t gpio) {
	GPIO_PinState ps;
	ps = HAL_GPIO_ReadPin(gpio.port, gpio.pin);
	if(ps == GPIO_PIN_SET) {
		return HIGH;
	} else {
		return LOW;
	}
}

void delay(uint32_t ms) {
	HAL_Delay(ms);
}

void mcu_reset() {
	printf("Reset MCU.\n");
	delay(1000);
	NVIC_SystemReset();
}

void hex_dump(const char *label, const uint8_t *data, uint32_t length) {
	printf(label);
	printf("0x");
	for (int i = 0; i < length; i++) {
		printf("%02x", data[i]);
	}
	printf("\n");
}

uint32_t millis() {
	return HAL_GetTick();
}

static uint32_t _start_time = 0;

void time_start() {
	_start_time = millis();
}

void time_stop() {
	printf("time used: %lu ms.\n",millis() - _start_time);
}

static int char_to_int(char input) {
	if (input >= '0' && input <= '9')
		return input - '0';
	if (input >= 'A' && input <= 'F')
		return input - 'A' + 10;
	if (input >= 'a' && input <= 'f')
		return input - 'a' + 10;
	return 0;
}

// This function assumes src to be a zero terminated sanitized string with
// an even number of [0-9a-f] characters, and target to be sufficiently large
void hex_load(const char* src, uint8_t * target) {
    //skip 0x header
    if((src[0] == '0') && (tolower(src[1]) == 'x')) {
        src += 2;
    }
    while (*src && src[1]) {
        *(target++) = char_to_int(*src) * 16 + char_to_int(src[1]);
        src += 2;
    }
}



