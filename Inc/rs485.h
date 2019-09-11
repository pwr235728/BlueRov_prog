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

void rs485_init(void);

void rs485_transmit(uint8_t *data, uint32_t length);
void rs485_rx_callback(uint8_t data);
#endif /* RS485_H_ */
