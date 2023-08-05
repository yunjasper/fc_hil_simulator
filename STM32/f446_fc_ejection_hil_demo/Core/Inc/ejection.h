/*
 * ejection.h
 *
 *
 * ejection algorithm header file.
 *
 *  Created on: Aug 4, 2023
 *      Author: jasper
 */

#ifndef INC_EJECTION_H_
#define INC_EJECTION_H_

#include "stdint.h"
#include "hil_sim_interface.h"

#define EJ_ALGO_UPDATE_RATE_HZ		1

// ejection algo defines

// -- 2020 FC parameters -- //
#define		DROGUE_DELAY			500			// ms (Time that drogue is HIGH)
#define		MAIN_DELAY				500			// ms (Time that main is HIGH)

// Configurations
#define		NUM_MEAS_REG			50			// Number of historic measurements for linear regression
#define		ALT_MEAS_AVGING			500
#define		NUM_DESCENDING_SAMPLES	10			// Number of descending slope values for apogee detection to pass
// -- end 2020 parameters -- //

// -- 2021 megaloop parameters -- //
#define LAUNCH_ALT_CHANGE_THRESHOLD		75		// ft, change in altitude needed to change to "launched" state
#define LAUNCH_NUM_DESCENDING_SAMPLES	20		// number of samples needed to set as launched
#define APOGEE_NUM_DESCENDING_SAMPLES	30
#define MAIN_NUM_DESCENDING_SAMPLES		20
#define LANDING_NUM_DESCENDING_SAMPLES	150		// number of samples needed to set as landing
#define MAIN_DEPLOY_ALTITUDE			1500	// ft
#define LANDING_ALT_CHANGE_THRESHOLD	3		// ft

#define LOCAL_PRESSURE_HPA		1028	// hPa


// public variables
extern volatile uint8_t is_launch_command_received;

// function prototypes
void ej_init(void);
void ej_update_flight_state(flight_state_t *fs);
void ej_get_flight_state(flight_state_t *fs);
void ej_runAltitudeMeasurements(uint32_t currTick, float currAlt);

#endif /* INC_EJECTION_H_ */
