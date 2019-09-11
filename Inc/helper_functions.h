/*
 * helper_functions.h
 *
 *  Created on: 10.09.2019
 *      Author: Kurat
 */

#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

// HF stands for helper functions

typedef enum {
	HF_Status_Ok, HF_Status_Timeout, HF_Status_Error, HF_Status_Invalid
} HF_Status;

HF_Status HF_LL_WaitForFlag(uint32_t (*get_flag_status)(), uint32_t timeout);

HF_Status HF_LL_WaitForFlagClear(uint32_t (*get_flag_status)(),	uint32_t timeout);


#endif /* HELPER_FUNCTIONS_H_ */
