/*
 * helpfer_functions.c
 *
 *  Created on: 10.09.2019
 *      Author: Kurat
 */


#include <stdint.h>

#include "helper_functions.h"

#include "stm32f1xx_ll_cortex.h"

HF_Status HF_LL_WaitForFlag(
		uint32_t (*get_flag_status)(),
		uint32_t timeout) {
	while (!get_flag_status()) {
		if (LL_SYSTICK_IsActiveCounterFlag() && !(timeout-- > 0))
			return HF_Status_Timeout;
	}

	return HF_Status_Ok;
}
HF_Status HF_LL_WaitForFlagClear(
		uint32_t (*get_flag_status)(),
		uint32_t timeout) {
	while (get_flag_status()) {
		if (LL_SYSTICK_IsActiveCounterFlag() && !(timeout-- > 0))
			return HF_Status_Timeout;
	}

	return HF_Status_Ok;
}
