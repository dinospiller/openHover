/*
	Copyright 2017-2020 Dino Spiller (dinospiller@gmail.com)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * self_balancing.h
 *
 *  Created on: 05/apr/2017
 *      Author: mac_daino
 */

#ifndef CONTROLLER_BOARD_SELF_BALANCING_H_
#define CONTROLLER_BOARD_SELF_BALANCING_H_


#include "digital_filter.h"
#include "reference_builder.h"
#include "filter_and_control_params.h"

typedef struct{
	// motor and vehicle params
	uint8_t loop_period_ms;  	// [ms] loop period
	uint8_t n_poles;			// []total poles of the motor
	float 	kt; 				// [Nm/A] torque per Ampere
	float 	r_wheel;			// [m]wheel radius
	float 	rho;				// [] gear reduction ratio
	float 	d_interwheel;		// [m]distance between the 2 wheels

	// LQR controller params
	float gainsLQR[4];

	// aux vars, for filters and PID controller
	FILT_second_order dot_Xb_filt,dot_delta_filt,ddot_Xb_filt; 		// generic velocity filters
	FILT_first_order set_delta_filt, set_Xb_filt, set_dot_Xb_filt; 	// generic integrators
	FILT_first_order steering_PID;									// PI controller
	FILT_second_order accelero_filter;									// for noise removal

	float compl_filt_alpha;
	float compl_filt_beta;

	float gainsLQRsteering[2];

	float imu_accx_offset;
	float imu_accz_offset;
	float imu_gyro_offset;

	// backlash compensation
	float disturbance_torque_ampl;
	float disturbance_torque_omega;
	FILT_second_order gyro_filter;

	// filters for output torques
	FILT_second_order torque_bal_filt;
	FILT_second_order torque_steer_filt;

}SELF_BALANCING_PARAMS;

void self_balancing_init(void);
const SELF_BALANCING_PARAMS* self_balancing_get_params(void);
void self_balancing_set_params(SELF_BALANCING_PARAMS *new_params);
REF_SEQUENCE* self_balancing_get_sequence_dot_Xb(void);
REF_SEQUENCE* self_balancing_get_sequence_dot_delta(void);
bool self_balancing_start_simulation_sequence(void);
void self_balancing_enable_send_angle_bluetooth(bool enable);

#endif /* CONTROLLER_BOARD_SELF_BALANCING_H_ */
