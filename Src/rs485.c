/*
 * rs485.c
 *
 *  Created on: 13.02.2019
 *      Author: Kurat
 */

#include "rs485.h"
#include "main.h"
#include "helper_functions.h"
#include "frame_parser.h"

uint32_t _get_usart_txe_flag()
{
	return LL_USART_IsActiveFlag_TXE(USART1);
}

uint32_t _get_usart_tc_flag()
{
	return LL_USART_IsActiveFlag_TC(USART1);
}


extern frame_t frame;
extern uint8_t new_data_available;

void rs485_init()
{
	LL_GPIO_ResetOutputPin(DIR_GPIO_Port, DIR_Pin);
	LL_USART_Enable(USART1);
	LL_USART_EnableDirectionRx(USART1);
	LL_USART_EnableIT_RXNE(USART1);
	Parser_InitFrame(&frame);
}

void rs485_transmit(uint8_t *data, uint32_t length)
{
	LL_GPIO_SetOutputPin(DIR_GPIO_Port, DIR_Pin);

	// uart tx

	LL_USART_EnableDirectionTx(USART1);

	while(length--)
	{
		if(HF_LL_WaitForFlag(_get_usart_txe_flag,10) != HF_Status_Ok){
			return;
		}

		LL_USART_TransmitData8(USART1, *data++);
	}

	if(HF_LL_WaitForFlag(_get_usart_tc_flag,10) != HF_Status_Ok){
		return;
	}

	LL_USART_DisableDirectionTx(USART1);

	LL_GPIO_ResetOutputPin(DIR_GPIO_Port, DIR_Pin);
}

void rs485_rx_callback(uint8_t data)
{
	if(new_data_available != 0)
		return;

	if(Parser_ParseByte(data, &frame) == PARSER_COMPLETE)
	{
		new_data_available = 1;
	}
}
