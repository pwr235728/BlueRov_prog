/*
 * rs485.h
 *
 *  Created on: 13.02.2019
 *      Author: Kurat
 */

#ifndef RS485_H_
#define RS485_H_

#include <stdint.h>
#include "main.h"

void RS_Transmit(UART_HandleTypeDef* huart, uint8_t* buffer, uint32_t length);
HAL_StatusTypeDef RS_Receive(UART_HandleTypeDef* huart, uint8_t* rx_data, uint32_t length, uint32_t timeout);

void RS_TxCpltcallback(void);

void RS_RxCpltCallback(void);

#endif /* RS485_H_ */
