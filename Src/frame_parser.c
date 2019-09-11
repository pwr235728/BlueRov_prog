/*
 * frame_parser.c
 *
 *  Created on: 28.05.2019
 *      Author: proxima
 */

#include "frame_parser.h"

static const float THRUSTER_RANGE = 32000.0f;

static const uint8_t START_CHAR = '>';
static const uint8_t ID_CONTROLL_DATA = 'c';
static const uint8_t ID_VOLTAGE_REQUEST_FRAME = 'r';
static const uint8_t ID_VOLTAGE_FRAME = 'v';

uint8_t frame_GetHeaderSum(frame_t *frame) {
	uint8_t sum = 0;
	sum += frame->id;
	sum += frame->length;

	return sum;
}
uint8_t frame_GetDataSum(frame_t *frame) {
	uint8_t sum = 0;

	for (int i = 0; i < frame->length; i++) {
		sum += frame->data[i];
	}

	return sum;
}

void Parser_InitFrame(frame_t* frame) {
	frame->data_index = 0;
	frame->length = 0;
	frame->state = STATE_WAIT;
}

ParserStatus Parser_ParseByte(uint8_t byte, frame_t* frame) {
	switch (frame->state) {
	case STATE_WAIT: {
		if (byte == START_CHAR) {
			frame->state = STATE_GOT_MARKER;
		} else {
			Parser_InitFrame(frame);
			return PARSER_ERROR;
		}
	}break;

	case STATE_GOT_MARKER: {
		frame->id = byte;
		frame->state = STATE_GOT_ID;
	}break;

	case STATE_GOT_ID: {
		frame->length = byte;
		frame->state = STATE_GOT_LEN;
	}break;

	case STATE_GOT_LEN: {
		if (byte == frame_GetHeaderSum(frame)) {
			frame->state = (
					frame->length == 0 ? STATE_GOT_DATA : STATE_GOT_HEADER);
		} else {
			Parser_InitFrame(frame);
			return PARSER_ERROR;
		}
	}break;

	case STATE_GOT_HEADER: {
		frame->data[frame->data_index++] = byte;
		if (frame->data_index == frame->length) {
			frame->state = STATE_GOT_DATA;
		} else {
			// continue reading data
		}
	}break;

	case STATE_GOT_DATA: {
		frame->state = STATE_WAIT;

		if (byte == frame_GetDataSum(frame)) {
			return PARSER_COMPLETE;
		} else {
			Parser_InitFrame(frame);
			return PARSER_ERROR;
		}
	}break;

	}

	return PARSER_INCOMPLETE;
}

ParserStatus Parser_ReadFrame(uint8_t data[], uint32_t data_len,
		frame_t *frame_out) {
	ParserState state = STATE_WAIT;
	uint16_t data_index = 0;

	for (int i = 0; i < data_len; i++) {
		uint8_t byte = data[i];

		switch (state) {
		case STATE_WAIT: {
			if (byte == START_CHAR) {
				state = STATE_GOT_MARKER;
			} else {
				// ERROR - continue
			}
		}
			break;

		case STATE_GOT_MARKER: {
			frame_out->id = byte;
			state = STATE_GOT_ID;
		}
			break;

		case STATE_GOT_ID: {
			frame_out->length = byte;
			state = STATE_GOT_LEN;
		}
			break;

		case STATE_GOT_LEN: {
			if (byte == frame_GetHeaderSum(frame_out)) {
				state = (
						frame_out->length == 0 ?
								STATE_GOT_DATA : STATE_GOT_HEADER);
			} else {
				// some bananas happened xd
				state = STATE_WAIT;
			}
		}
			break;

		case STATE_GOT_HEADER: {
			frame_out->data[data_index++] = byte;
			if (data_index == frame_out->length) {
				state = STATE_GOT_DATA;
			} else {
				// continue reading data
			}
		}
			break;

		case STATE_GOT_DATA: {
			state = STATE_WAIT;

			if (byte == frame_GetDataSum(frame_out)) {
				// complete
				return PARSER_COMPLETE;
			} else {
				// coś się wyjebało xd przy transmiji
				return PARSER_ERROR;
			}
		}
			break;

		}
	}

	// wyjabao sie :<
	return PARSER_ERROR;
}

ParserStatus Parser_ParseControllData(frame_t *frame, controll_data_t *out_data) {
	if (frame->id != ID_CONTROLL_DATA)
		return PARSER_ERROR;

	for (int i = 0; i < 5; i++) {
		int16_t value = (int16_t) ((uint16_t) (frame->data[i * 2] << 8)
				| frame->data[i * 2 + 1]);
		float thruster = value;
		thruster = thruster / THRUSTER_RANGE;

		out_data->thrusters[i] = thruster;
	}
	return PARSER_COMPLETE;
}

ParserStatus Parser_ParseVoltageRequest(frame_t *frame) {
	return (frame->id == ID_VOLTAGE_REQUEST_FRAME ?
			PARSER_COMPLETE : PARSER_ERROR);
}
ParserStatus Parser_ParseVoltageData(frame_t *frame, voltage_data_t *out_data) {
	if (frame->id != ID_VOLTAGE_FRAME)
		return PARSER_ERROR;

	// banana
	if (frame->length != 8)
		return PARSER_ERROR;

	for (int i = 0; i < frame->length; i++) {
		out_data->v_txt[i] = frame->data[i];
	}

	return PARSER_COMPLETE;
}

volatile frame_t Parser_CreateControlFrame(controll_data_t *ctrlData) {
	volatile frame_t frame;
	frame.id = ID_CONTROLL_DATA;
	frame.length = 10;

	for (int i = 0; i < 5; i++) {
		float value_f = ctrlData->thrusters[i];
		int16_t value_int16 = (int16_t) (value_f * THRUSTER_RANGE);

		uint8_t h_byte = (value_int16 >> 8) & 0x00FF;
		uint8_t l_byte = value_int16 & 0x00FF;

		frame.data[i * 2] = h_byte;
		frame.data[i * 2 + 1] = l_byte;
	}

	return frame;
}

frame_t Parser_CreateVoltageFrame(voltage_data_t *voltData) {
	volatile frame_t frame;
	frame.id = ID_VOLTAGE_FRAME;
	frame.length = 8;

	for (int i = 0; i < 8; i++) {
		frame.data[i] = voltData->v_txt[i];
	}

	return frame;
}

void Parser_CreateTxBuffer(frame_t *frame, uint8_t out_buff[], uint32_t max_len,
		uint32_t* len_out) {

	*len_out = 5 + frame->length;

	uint8_t h_sum = frame_GetHeaderSum(frame);
	uint8_t d_sum = frame_GetDataSum(frame);

	out_buff[0] = START_CHAR;
	out_buff[1] = frame->id;
	out_buff[2] = frame->length;
	out_buff[3] = h_sum;

	for (int i = 0; i < frame->length; i++) {
		out_buff[4 + i] = frame->data[i];
	}

	out_buff[4 + frame->length] = d_sum;
}

void Parser_CreateVoltageRequest(uint8_t out_buff[], uint32_t max_len) {
	frame_t frame;
	frame.id = ID_VOLTAGE_REQUEST_FRAME;
	frame.length = 0;
	uint32_t tmp;
	Parser_CreateTxBuffer(&frame, out_buff, max_len, &tmp);
}
