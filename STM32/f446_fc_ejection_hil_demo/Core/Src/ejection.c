/*
 * ejection.c
 *
 *  Created on: Aug 4, 2023
 *      Author: jaspe
 */


#include "ejection.h"

#include "stm32f4xx_hal.h" // for GetTick()

// private defines
#define NUM_MEAS_REG 	50	// samples for regression


// private variables
flight_state_t ej_fs;
float alt_previous[NUM_MEAS_REG];
float time_previous[NUM_MEAS_REG];
float timalt_previous[NUM_MEAS_REG];
float timsqr_previous[NUM_MEAS_REG];
uint8_t num_descending_samples = 0;
uint8_t currElem = 0;

float alt_current;
float alt_ground;
float alt_apogee;
float alt_prev;
float T;

uint32_t prevTick;


volatile uint8_t is_launch_command_received;

// function prototypes
void check_ejection_state(void);
float runAltitudeMeasurements(uint32_t currTick, float currAlt);

void ej_init(void) {
	ej_fs = PAD;
	for (int i = 0; i < NUM_MEAS_REG; i++) {
		alt_previous[i] = 0;
		time_previous[i] = 0;
		timalt_previous[i] = 0;
		timsqr_previous[i] = 0;
	}
}

void ej_get_flight_state(flight_state_t *fs) {
	*fs = ej_fs;
}

void ej_update_flight_state(struct ej_data *data, flight_state_t *fs) {
	// main ejection algorithm
	// can be used as an interface to another file
	runAltitudeMeasurements(HAL_GetTick(), data->altitude);
	check_ejection_state();
	*fs = ej_fs;
}


// Private functions
void storeAltitude(float new_altitude, float cTime);

// Public function implementation
float runAltitudeMeasurements(uint32_t currTick, float currAlt) {
  T = (float)(currTick - prevTick);

  prevTick = currTick;
  float alt_meas = currAlt - alt_ground; // Measures AGL altitude in feet
  float alt_filtered = alt_meas; // disable filtering for now
  storeAltitude(alt_filtered, currTick);
  alt_current = currAlt;
  return alt_filtered;
}

// -- Private function implementation --

void storeAltitude(float new_altitude, float cTime) {

	alt_previous[currElem] = new_altitude;
	time_previous[currElem] = cTime;
	timalt_previous[currElem] = cTime * new_altitude;
	timsqr_previous[currElem] = cTime * cTime;

	// circular buffer
	if (currElem == (NUM_MEAS_REG - 1)) {
		currElem = 0;
	}
	else {
		currElem += 1;
	}
}

float LSLinRegression() {
	float xsum = 0, ysum = 0, xy = 0, xsqr = 0;

	for (uint8_t i = 0; i < NUM_MEAS_REG; i++) {
		xsum += time_previous[i];
	    ysum += alt_previous[i];
	    xy += timalt_previous[i];
	    xsqr += timsqr_previous[i];
	}

	return (float)(NUM_MEAS_REG*xy - (xsum*ysum))/(NUM_MEAS_REG*xsqr - (xsum*xsum));
}

// logic to change states of flight -- from megaloop 2022
void check_ejection_state(void) {
	switch (ej_fs) {
	case PAD:

		if (is_launch_command_received == 0) { // changing atmospheric pressure may trigger false launch detection
			alt_ground = alt_current;
			num_descending_samples = 0; // if we go from launch received to disarmed, reset the number of samples
		}
		else {
			// check current state
			if (alt_current - alt_ground > LAUNCH_ALT_CHANGE_THRESHOLD) { // launched
				num_descending_samples += 1;

				if (num_descending_samples > LAUNCH_NUM_DESCENDING_SAMPLES) {
					ej_fs = BOOST;
					num_descending_samples = 0; // for next state, reset to zero count
//					// generate software interrupt to change TIM3 update rate
//					__HAL_GPIO_EXTI_GENERATE_SWIT(EXTI_SWIER_SWIER4);
				}
			}
			else {
				num_descending_samples = 0;
			}
		}
		break;

	case BOOST: // pre-apogee, waiting for ejection and drogue deployment

		if (LSLinRegression() < 0) {
			num_descending_samples += 1;

			if (num_descending_samples > APOGEE_NUM_DESCENDING_SAMPLES) {
//				// *** EJECTION AND DROGUE DEPLOYMENT *** //
//				 // can't hurt right? in case arming failed on the pad
//				HAL_GPIO_WritePin(Rcov_Gate_Drogue_GPIO_Port, Rcov_Gate_Drogue_Pin, SET);
//				HAL_GPIO_WritePin(Rcov_Gate_Main_GPIO_Port, Rcov_Gate_Main_Pin, RESET);
//
//				HAL_GPIO_WritePin(Rcov_Arm_GPIO_Port, Rcov_Arm_Pin, SET);
//				HAL_Delay(DROGUE_DELAY);
//				HAL_GPIO_WritePin(Rcov_Arm_GPIO_Port, Rcov_Arm_Pin, RESET);
//				HAL_GPIO_WritePin(Rcov_Gate_Drogue_GPIO_Port, Rcov_Gate_Drogue_Pin, RESET);
//				// *** ------------------------------ *** //

				ej_fs = DROGUE_DESCENT; // passed apogee
				num_descending_samples = 0;

				// turn off propulsion stuff to save power
//				HAL_GPIO_WritePin(Prop_Pyro_Arming_GPIO_Port, Prop_Pyro_Arming_Pin, RESET);
//				HAL_GPIO_WritePin(Prop_Gate_1_GPIO_Port, Prop_Gate_1_Pin, RESET);
//				HAL_GPIO_WritePin(Prop_Gate_2_GPIO_Port, Prop_Gate_2_Pin, RESET);
//				HAL_GPIO_WritePin(Vent_Valve_EN_GPIO_Port, Vent_Valve_EN_Pin, RESET); // open dump valve
//				HAL_GPIO_WritePin(PM_12V_EN_GPIO_Port, PM_12V_EN_Pin, RESET);

				alt_apogee = alt_current - alt_ground;
				if (alt_apogee < MAIN_DEPLOY_ALTITUDE) {
//					HAL_Delay(1500); // wait for drogue to open fully to tension the tender descenders if apogee < 1500 ft
				}

//				// generate software interrupt to change TIM3 update rate
//				__HAL_GPIO_EXTI_GENERATE_SWIT(EXTI_SWIER_SWIER4);
			}
		}
		else {
			num_descending_samples = 0;
		}

		break;

	case DROGUE_DESCENT: // post-apogee, waiting for main parachute deployment

		// check current state
		if (alt_current - alt_ground < MAIN_DEPLOY_ALTITUDE) {
			num_descending_samples++;

			if (num_descending_samples > MAIN_NUM_DESCENDING_SAMPLES) {
//				// *** DEPLOYING MAIN PARACHUTE *** //
//				HAL_GPIO_WritePin(Rcov_Gate_Drogue_GPIO_Port, Rcov_Gate_Drogue_Pin, RESET);
//				HAL_GPIO_WritePin(Rcov_Gate_Main_GPIO_Port, Rcov_Gate_Main_Pin, SET);
//
//				HAL_GPIO_WritePin(Rcov_Arm_GPIO_Port, Rcov_Arm_Pin, SET);
//				HAL_Delay(DROGUE_DELAY);
//				HAL_GPIO_WritePin(Rcov_Arm_GPIO_Port, Rcov_Arm_Pin, RESET);
//				HAL_GPIO_WritePin(Rcov_Gate_Main_GPIO_Port, Rcov_Gate_Main_Pin, RESET);
//				// *** ------------------------ *** //

				ej_fs = MAIN_DESCENT;
				alt_prev = alt_current; // in next stage we need to know the previous altitude
				num_descending_samples = 0;

//				// generate software interrupt to change TIM3 update rate
//				__HAL_GPIO_EXTI_GENERATE_SWIT(EXTI_SWIER_SWIER4);
			}
		} else {
			num_descending_samples = 0;
		}

		break;

	case MAIN_DESCENT:
		// post main deploy, want to transmit data fast to maximize possibility of getting good GPS coordinates
		; // empty statement to avoid 'a label can only be part of a statement and a declaration is not a statement' compiler error

		// check current state
		float alt_diff = alt_current - alt_prev;
		if (alt_diff < 0) {
			alt_diff *= -1; // absolute value
		}

		if (alt_diff < LANDING_ALT_CHANGE_THRESHOLD) {
			num_descending_samples++;

			if (num_descending_samples > LANDING_NUM_DESCENDING_SAMPLES) {
				ej_fs = LANDED;
				num_descending_samples = 0;

//				// generate software interrupt to change TIM3 update rate
//				__HAL_GPIO_EXTI_GENERATE_SWIT(EXTI_SWIER_SWIER4);
			}
		} else {
			num_descending_samples = 0;
		}

		alt_prev = alt_current;
		break;

	case LANDED: // landed
		// nothing to do
		break;

	default:
		break;
	}
}
