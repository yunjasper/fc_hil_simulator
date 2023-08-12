/*
 * hil_sim_interface.h
 *
 *  Created on: Aug. 5, 2023
 *      Author: jasper
 */

#ifndef INC_HIL_SIM_INTERFACE_H_
#define INC_HIL_SIM_INTERFACE_H_

#include "stdint.h"

// configuration for communication with sim engine
#define HIL_IF_UART					huart3
#define HIL_UART_TIMEOUT			10  // ms

#define DMA_BUFFER_LENGTH 			24	// bytes
#define RX_TELEMETRY_LENGTH			24	// bytes: [header][Time][AccX][AccZ][Altitude][trailer], all fields 4 bytes

#define CONTROL_RESPONSE_TEMPLATE	((char *)"C,%d,E\r\n") // use for sprintf
#define CONTROL_RESPONSE_LENGTH		7

#define LAUNCHED_RESPONSE			((uint8_t *)"launch\n")

#define POLL_MESSAGE				((uint8_t *)"pollda\n")
#define POLL_MESSAGE_LENGTH			7

#define RX_TELEMETRY_HEADER			((uint32_t) 0xFFFF0000)
#define RX_TELEMETRY_TRAILER		((uint32_t) 0x0000FFFF)


// variables


// types
typedef enum {
	PAD = 0,
	BOOST = 1,
	COAST = 2,
	DROGUE_DESCENT = 3,
	MAIN_DESCENT = 4,
	LANDED = 5,
	LAUNCH_COMMAND = 8,
	FS_ERROR = 9
} flight_state_t;


// struct which matches format of telemetry packet from sim engine to hw platform
typedef struct {
	uint32_t header; // 4 bytes so accesses are aligned
	uint32_t timestamp_ms;
	float accX_ms2;
	float accZ_ms2;
	float altitude_m;
	uint32_t trailer;
} hil_data_packet_t;


// functions
void hil_if_init(void);
void hil_if_parse_telemetry();
void hil_if_send_flight_state(flight_state_t *fs);
void hil_if_poll_simulator(void);

// getters to access data
void hil_if_get_current_data(hil_data_packet_t *data);
uint32_t hil_if_get_timestamp_ms();
float hil_if_get_accX_ms2();
float hil_if_get_accZ_ms2();
float hil_if_get_altitude_m();

#endif /* INC_HIL_SIM_INTERFACE_H_ */
