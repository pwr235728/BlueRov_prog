/*
 * rs485.c
 *
 *  Created on: 13.02.2019
 *      Author: Kurat
 */

#include "rs485.h"
#include "main.h"

void RS_Transmit(UART_HandleTypeDef* huart, uint8_t* buffer, uint32_t length)
{
	LL_GPIO_SetOutputPin(RS_DIR_GPIO_Port, RS_DIR_Pin);

	HAL_UART_Transmit(huart, buffer, length, 1000);

	// trzbea chwile odczekac przed zmaina na odbieranie bo inaczej bledy  sa
	//LL_mDelay(10);

	LL_GPIO_ResetOutputPin(RS_DIR_GPIO_Port, RS_DIR_Pin);
}


HAL_StatusTypeDef RS_Receive(UART_HandleTypeDef* huart, uint8_t* rx_data, uint32_t length, uint32_t timeout)
{
	return HAL_UART_Receive(huart,rx_data, length, timeout);

}

void RS_TxCpltcallback(void)
{

}

void RS_RxCpltCallback(void)
{

}
