/*
 * sfu_leds.c
 *
 *  Created on: Dec 8, 2020
 *      Author: eskoric
 */
#include "sfu_leds.h"
#include "sfu_low_level_security.h"
#include "sfu_low_level.h"

static SFU_BoolTypeDef m_b_initialized = SFU_FALSE;

#define BLINK_FREQ_HZ				1
#define BLINK_DUTY_CYCLE_PERCENT	50
#define NB_MS_IN_SEC				1000

void SFU_LEDs_TIM_Callback() {
  SFU_LL_TIM_Led_Stop();
  SFU_LL_PWM_Blue_Blinking_Led_Start();
}

SFU_ErrorStatus SFU_LEDs_Init(enum ledMode mode)
{
	if(m_b_initialized) {
		return SFU_SUCCESS;
	}

	SFU_ErrorStatus ret = SFU_ERROR;
	m_b_initialized = SFU_TRUE;

	ret = SFU_LL_TIM_Led_Init(BLINK_FREQ_HZ * NB_MS_IN_SEC / 2);

	if(SFU_SUCCESS == ret) {
		ret = SFU_LL_Leds_Init(BLINK_FREQ_HZ, BLINK_DUTY_CYCLE_PERCENT);
	}

	if(SFU_SUCCESS == ret) {
		ret = SFU_LL_PWM_Green_Blinking_Led_Start();
	}

	if(SFU_SUCCESS == ret) {
		if(mode == BLINKING_SYNCHRONIZED_LED) {
			ret = SFU_LL_PWM_Blue_Blinking_Led_Start();
		} else if(mode == BLINKING_ALTERNATE_LED) {
			ret = SFU_LL_TIM_Led_Start();
		}
	}

	return ret;
}

SFU_ErrorStatus SFU_LEDs_DeInit()
{
  if(m_b_initialized) {
    m_b_initialized = SFU_FALSE;

    SFU_LL_TIM_Led_Stop();
    SFU_LL_PWM_Green_Blinking_Led_Stop();
    SFU_LL_PWM_Blue_Blinking_Led_Stop();
    SFU_LL_TIM_Led_DeInit();
    SFU_LL_Leds_DeInit();
  }

  return SFU_SUCCESS;
}

