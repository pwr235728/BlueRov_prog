/*
 * frame_parser.h
 *
 *  Created on: 28.05.2019
 *      Author: proxima
 */

#ifndef FRAME_PARSER_H_
#define FRAME_PARSER_H_

#include <stdint.h>



/*

 thruster -> int16_t in range <-32'000; 32'000>

 high byte first
 [h_byte1][l_byte1][h_byte2][l_byte2][....]

 */

typedef struct
{
	float thrusters[5];
}controll_data_t;

typedef struct
{
	uint8_t v_txt[8]; // example: 15.321V\0
}voltage_data_t;

typedef struct
{
	uint8_t id;
	uint8_t length;
	uint8_t data[256];
}frame_t;


typedef enum
{
	PARSER_ERROR = 0,
	PARSER_COMPLETE
}ParserStatus;

typedef enum
{
    STATE_WAIT,
    STATE_GOT_MARKER,
    STATE_GOT_ID,
    STATE_GOT_LEN,
    STATE_GOT_HEADER,
    STATE_GOT_DATA
}ParserState;

uint8_t frame_GetHeaderSum(frame_t *frame);
uint8_t frame_GetDataSum(frame_t *frame);

ParserStatus Parser_ReadFrame(uint8_t data[], uint32_t data_len, frame_t *frame_out);

ParserStatus Parser_ParseControllData(frame_t *frame, controll_data_t *out_data);
ParserStatus Parser_ParseVoltageRequest(frame_t *frame);
ParserStatus Parser_ParseVoltageData(frame_t *frame, voltage_data_t *out_data);

frame_t Parser_CreateControlFrame(controll_data_t *ctrlData);
frame_t Parser_CreateVoltageFrame(voltage_data_t *voltData);

void Parser_CreateTxBuffer(frame_t *frame, uint8_t out_buff[], uint32_t max_len);
void Parser_CreateVoltageRequest(uint8_t out_buff[], uint32_t max_len);

#endif /* FRAME_PARSER_H_ */
