/*
 * hil_sim_interface.c
 *
 * receives telemetry from the simulation engine via UART.
 * parses and extracts telemetry so that target controller can access it
 * using getter functions.
 *
 *  Created on: Aug. 5, 2023
 *      Author: jasper
 */

#include "hil_sim_interface.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"

// variables
extern UART_HandleTypeDef huart3;
hil_data_packet_t current_data;

uint8_t rx_dma_buffer[DMA_BUFFER_LENGTH]; // DMA interface with sim engine
uint8_t tx_buffer[DMA_BUFFER_LENGTH];


void hil_if_start_rx_dma(void) {
	HAL_UART_Receive_DMA(&HIL_IF_UART, rx_dma_buffer, RX_TELEMETRY_LENGTH);
}

void hil_if_init(void) {
	current_data.timestamp_ms = 0;
	current_data.accX_ms2 = 0;
	current_data.accZ_ms2 = 0;
	current_data.altitude_m = 0;

	// start wait for data from sim engine
	hil_if_start_rx_dma();
}

void hil_if_parse_telemetry() {
	// check header and trailer
	uint32_t *buffer = (uint32_t *)(rx_dma_buffer);
	if (buffer[0] != RX_TELEMETRY_HEADER || buffer[RX_TELEMETRY_LENGTH / sizeof(buffer[0]) - 1] != RX_TELEMETRY_TRAILER) {
		// ignore this message since it is not in the right format, clear the buffer
		memset(rx_dma_buffer, 0, RX_TELEMETRY_LENGTH);
	}
	else {
		// message is good, update current_data
		memcpy(&current_data, rx_dma_buffer, RX_TELEMETRY_LENGTH);
		memset(rx_dma_buffer, 0, RX_TELEMETRY_LENGTH);
	}
	hil_if_start_rx_dma();
}

// sends the current hardware target flight state to the sim engine
void hil_if_send_flight_state(flight_state_t *fs) {
	memset(tx_buffer, 0, DMA_BUFFER_LENGTH);
	if (*fs == LAUNCH_COMMAND) {
		HAL_UART_Transmit(&HIL_IF_UART, LAUNCHED_RESPONSE, CONTROL_RESPONSE_LENGTH, HIL_UART_TIMEOUT);
	}
	else {
		sprintf((char *)tx_buffer, CONTROL_RESPONSE_TEMPLATE, *fs);
		HAL_UART_Transmit(&HIL_IF_UART, tx_buffer, CONTROL_RESPONSE_LENGTH, HIL_UART_TIMEOUT);
	}
}

// -----------------------------------------------------------------------
// getters for each field of the struct of data provided by the sim engine
// can customize these getters to match the format of data received by the
// target hardware platform, e.g. combine all acceleration values in array
// -----------------------------------------------------------------------

void hil_if_poll_simulator(void) {
	// note: these UART calls block until complete
	// request data
	HAL_UART_Transmit(&HIL_IF_UART, POLL_MESSAGE, POLL_MESSAGE_LENGTH, HIL_UART_TIMEOUT);
	// wait until receiving telemetry
	HAL_UART_Receive(&HIL_IF_UART, rx_dma_buffer, RX_TELEMETRY_LENGTH, HIL_UART_TIMEOUT);
	// parse
	hil_if_parse_telemetry();
}

// grab entire packet if desired
void hil_if_get_current_data(hil_data_packet_t *data) {
	memcpy(data, &current_data, sizeof(hil_data_packet_t));
}

uint32_t hil_if_get_timestamp_ms() {
	return current_data.timestamp_ms;
}

float hil_if_get_accX_ms2() {
	return current_data.accX_ms2;
}

float hil_if_get_accZ_ms2() {
	return current_data.accZ_ms2;
}

float hil_if_get_altitude_m() {
	return current_data.altitude_m;
}

