/*
 * sfu_leds.h
 *
 *  Created on: Dec 8, 2020
 *      Author: eskoric
 */

#ifndef APPLICATION_SBSFU_APP_SFU_LEDS_H_
#define APPLICATION_SBSFU_APP_SFU_LEDS_H_

#include "main.h"
#include "sfu_def.h"

enum ledMode {
	BLINKING_SYNCHRONIZED_LED,
	BLINKING_ALTERNATE_LED,
};

void SFU_LEDs_TIM_Callback();
SFU_ErrorStatus SFU_LEDs_Init(enum ledMode mode);
SFU_ErrorStatus SFU_LEDs_DeInit();

#endif /* APPLICATION_SBSFU_APP_SFU_LEDS_H_ */
