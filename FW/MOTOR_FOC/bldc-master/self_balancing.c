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
 * self_balancing.c
 *
 *  Created on: 05/apr/2017
 *      Author: mac_daino
 */
#include <math.h>
#include <stdio.h>
#include "ch.h"
#include "hal.h"
#include "hw.h"
#include "conf_general.h"
#include "mc_interface.h"
#include "mcpwm.h"
#include "joystick.h"
#include "comm_can.h"
#include "buffer.h"
#include "IMU/MPU9250.h"
#include "IMU/mat_math.h"
#include "IMU/kalman.h"
#include "filter_and_control_params.h"
#include "timeout.h"
#include "self_balancing.h"

#define EMERGENCY_BUTT_PORT		HW_ICU_GPIO
#define EMERGENCY_BUTT_BIT		HW_ICU_PIN

#define CH_EVT_TMR_LOOP_TRIGGER 64
#define CH_EVT_MOT_L_TACHO_RECV 4
#define CH_EVT_TMR_LOOP_TIMEOUT 16

// other loop parameters
#define LOOP_TIMEOUT_MS			1
#define LOOP_TIMEOUT MS2ST(LOOP_TIMEOUT_MS)
#define LOOP_TIMER_FREQ  		10000

// if some sensor failed reading, how many retries are admitted?
#define MAX_LOOP_RETRIES 	2

#define PI				(float)3.14159					// [] greek pi
//#define TACHO_TO_ANGLE 	(float)2*PI/(N_POLES*N_HALL)	// transforming the hall quantum to angle (rads)

#define CTL_STEER_YAW_GAIN		(float)0.3162
#define CTL_STEER_DOTYAW_GAIN	(float)1.0887

#define CTL_MAX_TILT_ANGLE_DEGREES		15
#define CTL_MAX_TILT_ANGLE_RADS		(float)CTL_MAX_TILT_ANGLE_DEGREES*PI/180

#define JOY_FWD_TORQUE_GAIN		(float)5
#define JOY_STEER_GAIN			(float)-5

#define G_ACC	(float) 9.80665


static THD_WORKING_AREA(ctl_loop_thread_wa, 2048);
static thread_t *ctl_loop_thread;

// IMU specific vars (TODO remove from here)
int16_t ax, ay, az;
int16_t gx, gy, gz;

// definitions for left and right motor
typedef enum{
	MOT_R=CAN_MASTER_BOARD_ADDR,
	MOT_L=CAN_SLAVE_BOARD_ADDR,
	MOT_NUM
}mot_t;

typedef struct {
	int32_t tacho_L;
	int32_t tacho_R;

	// state vars
	float Xb, delta, theta_P;
	float dot_Xb, dot_delta, dot_theta_P;

	float pitchAngleComplementary;

	// setpoint vars
	float set_delta,set_dot_delta,set_Xb,set_dot_Xb,set_ddot_Xb;

	// linear acceleration
	float ddot_Xb;

	// torques to actuate
	float torque_bal,torque_steer;

	bool simulation_active;
	bool send_angle_via_bluetooth;
	volatile uint32_t loop_counter;
}STATE_VARS;

STATE_VARS state_vars;
SELF_BALANCING_PARAMS self_balancing_params;
REF_SEQUENCE ref_seq_dot_Xb;
REF_SEQUENCE ref_seq_dot_Delta;

// kalman filter structure
kalman_str_t* k_sys;

// IMU settings
uint8_t accel_full_scale;
uint8_t gyro_full_scale;
uint8_t imu_LPF_settings;
float accelero_coeff;
float gyro_coeff;

/*
 * ----------------- FUNCTIONS ----------------------
 */
void ResetStateVars(void);
void self_balancing_enable_send_angle_bluetooth(bool enable){
	state_vars.send_angle_via_bluetooth=enable;
}

void SendAngleFusionOverBluetooth(int32_t timestamp,
									float* accX,
									float* accZ,
									float* accelRead,
									float* accelReadFilt,
									float* gyroRead,
									float* complementaryFused,
									float* kalmanFused,
									float* kalmanInnovation);
void init_PWM_loop_trigger(uint8_t loop_period_ms);


REF_SEQUENCE* self_balancing_get_sequence_dot_Xb(void){
	return &ref_seq_dot_Xb;
}

REF_SEQUENCE* self_balancing_get_sequence_dot_delta(void){
	return &ref_seq_dot_Delta;
}

float TachoToAngle(void){
	return (float)(2*PI/(float)(self_balancing_params.n_poles*N_HALL));	// transforming the hall quantum to angle (rads)
}

const SELF_BALANCING_PARAMS* self_balancing_get_params(void){
	return &self_balancing_params;
}

void self_balancing_set_params(SELF_BALANCING_PARAMS *new_params){
	bool period_changed = (new_params->loop_period_ms != self_balancing_params.loop_period_ms);
	self_balancing_params = *new_params;

	if (period_changed){
		init_PWM_loop_trigger(self_balancing_params.loop_period_ms);
	}
}

// set torque to individual motor. NOTE: the motors are mounted in opposite direction, so it is necessary to flip MOT_R sign!
void SetTorqueLeftRight(float torque,mot_t mot_id){
	if(mot_id==MOT_L){
		comm_can_set_current(MOT_L,torque/KT);// the correct
	}else if(mot_id==MOT_R){
		mc_interface_set_current(-torque/KT);// the correct
		timeout_reset(); // very important: it is always necessary after a motor command!
	}
}

void UpdateTorquesToVehicle(void){
	SetTorqueLeftRight((state_vars.torque_bal+state_vars.torque_steer)/2,MOT_L);
	SetTorqueLeftRight((state_vars.torque_bal-state_vars.torque_steer)/2,MOT_R);
}

// set common-mode torque.
void SetBalanceTorque(float torque){
	state_vars.torque_bal = torque;
}

// set yaw (differential) torque
void SetYawTorque(float torque){
	state_vars.torque_steer = torque;
}

// NOTE: the motors are mounted in opposite direction, so it is necessary to flip MOT_R sign!
void slave_tacho_received(int32_t tacho) {
	state_vars.tacho_L = tacho;
	chEvtSignal(ctl_loop_thread,CH_EVT_MOT_L_TACHO_RECV);
}

void UpdateStateVars(int32_t alpha_tacho, int32_t beta_tacho, float theta_P){

	state_vars.Xb = /*TachoToAngle()*theta_P +*/ (alpha_tacho*self_balancing_params.r_wheel*self_balancing_params.rho*TachoToAngle())/2 + (beta_tacho*self_balancing_params.r_wheel*self_balancing_params.rho*TachoToAngle())/2;
	state_vars.delta = /*TachoToAngle()*theta_P +*/ (alpha_tacho*self_balancing_params.r_wheel*self_balancing_params.rho*TachoToAngle())/self_balancing_params.d_interwheel - (beta_tacho*R_WHEEL*RHO*TachoToAngle())/self_balancing_params.d_interwheel;
	state_vars.theta_P=theta_P;

	// calculate velocities by HPF of positions
	state_vars.dot_Xb=filter_second_order_process_sample(&self_balancing_params.dot_Xb_filt,state_vars.Xb);
	state_vars.dot_delta=filter_second_order_process_sample(&self_balancing_params.dot_delta_filt,state_vars.delta);

	// calculate acceleration
	state_vars.ddot_Xb=filter_second_order_process_sample(&self_balancing_params.ddot_Xb_filt,state_vars.dot_Xb);
}

float accel_norm;
float sin_pitch;
float cos_pitch;
// input: ax, ay, float cos_pitchaz, gx, gy, gz
// output: thetaP, dot_thetaP
void IMUSensorFusion(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, float *thetaP, float *dot_thetaP){
	float pitchAngAcc;
	// avoid warnings
	ay++;
	gx++;
	gz++;

	float accX,accZ,gyroY;
	float pitchAngAccLin;

	// Accelerations: the output is a raw number, compared with full-scale, and the unit is "g".
	// Convert the measure in  m/s^2
	accX = (float)ax * accelero_coeff;
	accZ = (float)az * accelero_coeff;

	// angular velocities: in rad/s
	gyroY = (float)gy * gyro_coeff;

	// subtract the (already measured) offsets
	accX = accX -self_balancing_params.imu_accx_offset;
	accZ = accZ -self_balancing_params.imu_accz_offset;
	gyroY= gyroY+self_balancing_params.imu_gyro_offset;

	// filter the gyro with the NOTCH filter, that removes the "disturbance torque"
	gyroY = filter_second_order_process_sample(&self_balancing_params.gyro_filter,gyroY);

	// variation: from ax, subtract the horizontal acceleration
	//accX = accX-(state_vars.ddot_Xb)*cosf(state_vars.theta_P);
	//accZ = accZ+(state_vars.ddot_Xb)*sinf(state_vars.theta_P);

	accX = accX-state_vars.ddot_Xb;//simplified

	//float accel_norm = sqrt((Axyz[0]*Axyz[0])+(Axyz[1]*Axyz[1])+(Axyz[2]*Axyz[2]));
//	accel_norm = sqrt((accX*accX)+(accZ*accZ));// probably more correct
//	sin_pitch = accX/accel_norm;
//	cos_pitch = sqrt(1.0-(sin_pitch*sin_pitch));
//
//	pitchAngAcc = atan2f(sin_pitch,cos_pitch);
	// avoid "divide-by-zero"
	if(accZ!=0.0)
		pitchAngAccLin = (accX/accZ);
	else
		pitchAngAccLin = (accX/0.0001);

	pitchAngAcc=pitchAngAccLin; // un-comment if you want to use the linearized

	// low-pass filter the angle obtained by the noisy accelerometer
	float pitchAccFiltered = filter_second_order_process_sample(&self_balancing_params.accelero_filter,pitchAngAcc);

	// sensor fusion: in order to reduce the noise in the angle, introduced by accelerometers, use
	// the angle speed integral:
	*dot_thetaP = -gyroY;

	// the set_Xb_filt filter is a generic integrator, so we use its coefficients
	//state_vars.pitchAngleComplementary =  -self_balancing_params.set_Xb_filt.D1*state_vars.pitchAngleComplementary + self_balancing_params.set_Xb_filt.N1*(*dot_thetaP);

	// decide what angle to use
	pitchAngAcc=pitchAccFiltered;

	//state_vars.pitchAngleComplementary = 	self_balancing_params.compl_filt_alpha*state_vars.pitchAngleComplementary +
	//							self_balancing_params.compl_filt_beta*pitchAngAcc;

	state_vars.pitchAngleComplementary = self_balancing_params.compl_filt_alpha*(state_vars.pitchAngleComplementary+*dot_thetaP*self_balancing_params.loop_period_ms/1000)+
										 self_balancing_params.compl_filt_beta*pitchAngAcc;

	kalman_Process_sample(k_sys,pitchAngAcc,*dot_thetaP);

	float pitchAngleKalman = k_sys->angle;

	if(state_vars.send_angle_via_bluetooth){
		SendAngleFusionOverBluetooth(state_vars.loop_counter,
									&accX,
									&accZ,
									&pitchAngAccLin,
									&pitchAccFiltered,
									dot_thetaP,
									&state_vars.pitchAngleComplementary,
									&pitchAngleKalman,
									&k_sys->err);
	}

	*thetaP = state_vars.pitchAngleComplementary;
	//*thetaP = pitchAngleKalman;
}

//void IMUSensorFusionOLD(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, float *thetaP, float *dot_thetaP){
//	static float pitchAngleComplementary = 0;
//
//	// by now, the calibration values are "cabled" here:
//	float Axyz[3];
//	float Gxyz[3];
//
//	// Accelerations: the output is a raw number, compared with full-scale, and the unit is "g".
//	// Convert the measure in  m/s^2
//
//
//	// accelerations: 1=1*g
//	Axyz[0] = (float) (ax-ax_offset) * 2/ 32768;//  LSB/g
////	Axyz[1] = (float) ay * 2/ 32768;
//	Axyz[2] = (float) (az-az_offset) * 2/ 32768;
//
//
//	// angular velocities: in degrees/s
//	//Gxyz[0] = (float) gx * 250 / 32768;
//	Gxyz[1] = (float) (gy-gy_offset) * 250 / 32768;
//	//Gxyz[2] = (float) gz * 250 / 32768;
//
//
//	//float accel_norm = sqrt((Axyz[0]*Axyz[0])+(Axyz[1]*Axyz[1])+(Axyz[2]*Axyz[2]));
////	float accel_norm = sqrt((Axyz[0]*Axyz[0])+(Axyz[2]*Axyz[2]));// probably more correct
////	float sin_roll = -Axyz[0]/accel_norm;
////	float cos_roll = sqrt(1.0-(sin_roll*sin_roll));
//
//	// variation: from ax, subtract the horizontal acceleration (measured in "g")
//	//Axyz[0] -= state_vars.ddot_Xb/G_ACC;
//
//	// alternative: linearized model supposing |theta| < 20° so theta approx.= ax/az
//	//float pitchAngLin = -(Axyz[0]/Axyz[2])*180.0/PI; // OK: fine enough, but use radians!
//	float pitchAngLin = (Axyz[0]/Axyz[2]);
//
//	// low-pass filter the angle obtained by the noisy accelerometer
//	float pitchAccFiltered = filter_second_order_process_sample(&self_balancing_params.accelero_LPF,pitchAngLin);
//
//	// sensor fusion: in order to reduce the noise in the angle, introduced by accelerometers, use
//	// the angle speed integral:
//	*dot_thetaP = -Gxyz[1]*PI/180;
//
//	// the set_Xb_filt filter is a generic integrator, so we use its coefficients
//	pitchAngleComplementary =  -self_balancing_params.set_Xb_filt.D1*pitchAngleComplementary + self_balancing_params.set_Xb_filt.N1*(*dot_thetaP);
//	//pitch_gyro_prev = -Gxyz[1]; // this is in degrees!
//	//pitch_gyro_prev = *dot_thetaP;
//
//	//try with the "filtered" accelero
//	float alpha=0.995;//0.997;
//	float beta=1-alpha;
//	pitchAngleComplementary = alpha*pitchAngleComplementary + beta*pitchAccFiltered;
//	//pitchAngleComplementary = alpha*pitchAngleComplementary + beta*pitchAngLin;
//
//	//kalman_Process_sample(k_sys,pitchAngLin,*dot_thetaP);
//	kalman_Process_sample(k_sys,pitchAccFiltered,*dot_thetaP);
//
//	float pitchAngleKalman = k_sys->angle;
//
//	if(state_vars.send_angle_via_bluetooth){
//		SendAngleFusionOverBluetooth(state_vars.loop_counter,
//									&Axyz[0],
//									&Axyz[2],
//									&pitchAngLin,
//									&pitchAccFiltered,
//									dot_thetaP,
//									&pitchAngleComplementary,
//									&pitchAngleKalman,
//									&k_sys->err);
//	}
//
//	//*thetaP = pitchAngleComplementary;
//	*thetaP = pitchAngleKalman;
//}
//  Schema:
// |Joy_FWD|*gain = set_ddot_Xb ->INTEGRATE-> set_dot_Xb ->INTEGRATE-> set_Xb
//
void ControlLoopBalance(void){
	if(state_vars.simulation_active){
		state_vars.set_dot_Xb = reference_get_curr_value(&ref_seq_dot_Xb,((float)state_vars.loop_counter*self_balancing_params.loop_period_ms/1000.0));
	}else{
		// the joystick (forward direction) sets the acceleration reference
		//state_vars.set_ddot_Xb = JOY_FWD_TORQUE_GAIN*JoystickGetForward();

		// find reference speed angle by integrating reference acceleration
		//state_vars.set_dot_Xb = filter_first_order_process_sample(&self_balancing_params.set_dot_Xb_filt,set_ddot_Xb);
		state_vars.set_dot_Xb = JOY_FWD_TORQUE_GAIN*JoystickGetForward();

		// by now, set all references to 0
		//state_vars.set_Xb = 0;
		//state_vars.set_dot_Xb = 0;
	}

	// find reference speed angle by integrating reference acceleration
	state_vars.set_Xb = filter_first_order_process_sample(&self_balancing_params.set_Xb_filt,state_vars.set_dot_Xb);

	// apply the gains to retrieve the balance torque:
	float bal_torque=self_balancing_params.gainsLQR[0]*(state_vars.set_Xb-state_vars.Xb)+
					self_balancing_params.gainsLQR[1]*(-state_vars.theta_P)+ // the setpoints of both thetaP and dot_thetaP will always be 0
					self_balancing_params.gainsLQR[2]*(state_vars.set_dot_Xb-state_vars.dot_Xb)+
					self_balancing_params.gainsLQR[3]*(-state_vars.dot_theta_P);// the setpoints of both thetaP and dot_thetaP will always be 0

	// add a "disturbing torque", to compensate for the coulomb friction that generates backlash
	float sin_arg = (state_vars.loop_counter*(float)self_balancing_params.loop_period_ms)/1000*self_balancing_params.disturbance_torque_omega;
	float disturbing_torque = self_balancing_params.disturbance_torque_ampl * sinf(sin_arg);

	bal_torque=bal_torque+disturbing_torque;

	// the output filter: can be either a LPF or NOTCH
	bal_torque=filter_second_order_process_sample(&self_balancing_params.torque_bal_filt,bal_torque);

	// the total torque to apply to balance subsystem
	SetBalanceTorque(bal_torque);
}

void ControlLoopSteeringPID(void){
	if(state_vars.simulation_active){
		state_vars.set_dot_delta = reference_get_curr_value(&ref_seq_dot_Delta,((float)state_vars.loop_counter*self_balancing_params.loop_period_ms/1000.0));
	}else{
		// the steering speed reference comes from joystick
		state_vars.set_dot_delta = JOY_STEER_GAIN*JoystickGetSteering();
	}

	float dot_delta_error=(state_vars.set_dot_delta-state_vars.dot_delta);
	// the PI controller is (at the end) a first-order filter, whose coefficients were calculated with
	// MATLAB
	float yaw_torque=filter_first_order_process_sample(&self_balancing_params.steering_PID,dot_delta_error);


	// filter the output
	yaw_torque=filter_second_order_process_sample(&self_balancing_params.torque_steer_filt,yaw_torque);

	// apply the gains to retrieve the steering torque:
	SetYawTorque(yaw_torque);
}

void ControlLoopSteeringLQR(void){

	if(state_vars.simulation_active){
		state_vars.set_dot_delta = reference_get_curr_value(&ref_seq_dot_Delta,((float)state_vars.loop_counter*self_balancing_params.loop_period_ms/1000.0));
	}else{
		// the steering speed reference comes from joystick
		state_vars.set_dot_delta = JOY_STEER_GAIN*JoystickGetSteering();
	}

	// find reference yaw angle by integrating angle speed
	state_vars.set_delta = filter_first_order_process_sample(&self_balancing_params.set_delta_filt,state_vars.set_dot_delta);

	//self_balancing_params.set_dot_delta= 0;
	float yaw_torque=(self_balancing_params.gainsLQRsteering[0]*(state_vars.set_delta-state_vars.delta)+
			self_balancing_params.gainsLQRsteering[1]*(state_vars.set_dot_delta-state_vars.dot_delta));

	// filter the output
	yaw_torque=filter_second_order_process_sample(&self_balancing_params.torque_steer_filt,yaw_torque);

	// apply the gains to retrieve the steering torque:
	SetYawTorque(yaw_torque);
}

void EmergencyBrake(void){
	SetBalanceTorque(0);
	SetYawTorque(0);
	UpdateTorquesToVehicle();
}

void SendStateOverBluetooth(int32_t timestamp){
	uint8_t buffer[100];
	int32_t ind=0;
	// first of all, send the timestamp
	buffer_append_int32(buffer,timestamp,&ind);
	// send state vars, using the 32-bit float representation
	buffer_append_int32(buffer,*(int32_t*)&state_vars.Xb,&ind);
	buffer_append_int32(buffer,*(int32_t*)&state_vars.theta_P,&ind);
	buffer_append_int32(buffer,*(int32_t*)&state_vars.dot_Xb,&ind);
	buffer_append_int32(buffer,*(int32_t*)&state_vars.dot_theta_P,&ind);
	buffer_append_int32(buffer,*(int32_t*)&state_vars.delta,&ind);
	buffer_append_int32(buffer,*(int32_t*)&state_vars.dot_delta,&ind);
	// now send the torques (currents) imposed
	buffer_append_int32(buffer,*(int32_t*)&state_vars.torque_bal,&ind);
	buffer_append_int32(buffer,*(int32_t*)&state_vars.torque_steer,&ind);
	// send also the "tacho" values
	buffer_append_int32(buffer,state_vars.tacho_L,&ind);
	buffer_append_int32(buffer,state_vars.tacho_R,&ind);

	// send references, using the 32-bit float representation
	buffer_append_int32(buffer,*(int32_t*)&state_vars.set_Xb,&ind);
	buffer_append_int32(buffer,*(int32_t*)&state_vars.set_dot_Xb,&ind);
	buffer_append_int32(buffer,*(int32_t*)&state_vars.set_dot_delta,&ind);

	comm_can_tunnel_bluetooth_buffer(buffer,ind);
}

void SendAngleFusionOverBluetooth(int32_t timestamp,
									float* accX,
									float* accZ,
									float* accelRead,
									float* accReadFilt,
									float* gyroRead,
									float* complementaryFused,
									float* kalmanFused,
									float* kalmanInnovation){
	uint8_t buffer[100];
	int32_t ind=0;
	// first of all, send the timestamp
	buffer_append_int32(buffer,timestamp,&ind);
	// send state vars, using the 32-bit float representation
	buffer_append_int32(buffer,*(int32_t*) accX,&ind);
	buffer_append_int32(buffer,*(int32_t*) accZ,&ind);
	buffer_append_int32(buffer,*(int32_t*) accelRead,&ind);
	buffer_append_int32(buffer,*(int32_t*) accReadFilt,&ind);
	buffer_append_int32(buffer,*(int32_t*) gyroRead,&ind);
	buffer_append_int32(buffer,*(int32_t*) complementaryFused,&ind);
	buffer_append_int32(buffer,*(int32_t*) kalmanFused,&ind);
	buffer_append_int32(buffer,*(int32_t*) kalmanInnovation,&ind);

	comm_can_tunnel_bluetooth_buffer(buffer,ind);
}

bool self_balancing_start_simulation_sequence(void){
	if(state_vars.simulation_active) return false;

	state_vars.loop_counter=0;
	state_vars.simulation_active=true;
	return state_vars.simulation_active;
}

static THD_FUNCTION(ControlLoopMain, arg){
	(void)arg;
	unsigned char buf[20];
	time_measurement_t tim_meas;
	chTMObjectInit(&tim_meas);
	ctl_loop_thread = chThdGetSelfX();
	bool imu_ok=false;
	eventmask_t evt_received;
	uint8_t retries_cnt=MAX_LOOP_RETRIES;
	static uint8_t emgcy_cnt=0;

	// reset the tachometers of both left and right wheels
	mc_interface_get_tachometer_value(true); // local motor's tacho

	for(;;){
		chEvtWaitAny((eventmask_t) CH_EVT_TMR_LOOP_TRIGGER);// wait the event of the timer routine

		//if(state_vars.loop_counter<=100000){// for safety:  before stopping all
		if(true){// for safety:  before stopping all
			state_vars.loop_counter++;

			// the local motor position
			// NOTE: the motors are mounted in opposite direction, so it is necessary to flip MOT_R sign!
			state_vars.tacho_R = -(mc_interface_get_tachometer_value(false));

			//chTMStartMeasurementX(&tim_meas);
			// read the IMU (has embedded timeout)
			imu_ok=getMotion6(&ax, &ay, &az, &gx, &gy, &gz);


			//read the remote motor position
			comm_can_get_tacho(MOT_L);

			// wait the slave motor position to arrive (with a timeout, signaled as 0 value return)
			evt_received = chEvtWaitAllTimeout(CH_EVT_MOT_L_TACHO_RECV,LOOP_TIMEOUT);

			//chTMStopMeasurementX(&tim_meas);

			//float worst_time_ms= (float)tim_meas.worst*1000/SYSTEM_CORE_CLOCK;

			if(imu_ok && (evt_received!=0)){// if both motors received and even IMU received:
				retries_cnt=MAX_LOOP_RETRIES;// all OK: reset retries counter and exec routines only if no timeout
				IMUSensorFusion(ax, ay, az, gx, gy, gz,
							&state_vars.theta_P,
							&state_vars.dot_theta_P);

				UpdateStateVars(state_vars.tacho_L,
						state_vars.tacho_R,
						state_vars.theta_P);

				static uint8_t eq_cnt=0;
				// attention: apply the controls only if the tilt angle is within the limits!!!
				if((state_vars.theta_P<CTL_MAX_TILT_ANGLE_RADS)&&(state_vars.theta_P>(-CTL_MAX_TILT_ANGLE_RADS))){
					//ControlLoopSteeringPID();
					ControlLoopSteeringLQR();
					ControlLoopBalance();
					eq_cnt=0;
					// debug purpose
					//SetBalanceTorque(JoystickGetForward()*JOY_FWD_TORQUE_GAIN);
				}else{
					if((eq_cnt)<10){
						EmergencyBrake();
						eq_cnt++;
					}
				}

				if(state_vars.simulation_active){
					SendStateOverBluetooth(state_vars.loop_counter);
					// check if simulation has ended
					if((((float)state_vars.loop_counter*self_balancing_params.loop_period_ms/1000.0)>=ref_seq_dot_Delta.refpoints[ref_seq_dot_Delta.last_valid].time)&&
							(((float)state_vars.loop_counter*self_balancing_params.loop_period_ms/1000.0)>=ref_seq_dot_Xb.refpoints[ref_seq_dot_Xb.last_valid].time)){
						state_vars.simulation_active=false;
					}
				}

				// IMPORTANT safety guard: monitoring red button pressure
				if(palReadPad(EMERGENCY_BUTT_PORT,EMERGENCY_BUTT_BIT)){ // if red button is pressed
					if(emgcy_cnt<10){
						EmergencyBrake();
						// reset the tachometers of both left and right wheels
						mc_interface_get_tachometer_value(true); // local motor's tacho
						comm_can_reset_tacho(MOT_L);			 // remote motor's tacho
						ResetStateVars();
						emgcy_cnt++;
					}

				}else{
					emgcy_cnt=0;
					UpdateTorquesToVehicle();
				}

			}else{ // if there is a communication failure
				sprintf((char*)buf,"IMU:%d,CAN:%d\n",imu_ok,(int)evt_received);
				comm_can_tunnel_bluetooth_buffer(buf,12);
				// a timeout has occurred: decrease retries counter
				if(retries_cnt!=0){
					retries_cnt--;
				}else{ // all retries expired
					comm_can_tunnel_bluetooth_buffer((unsigned char*)"RETRIES EXPIRED!\n",17);
					EmergencyBrake();
					break;
				}
			}

		}else{ // timeout elapsed
			EmergencyBrake();
		}

	}
}

void ResetStateVars(void){
	state_vars.Xb			=0;
	state_vars.dot_Xb		=0;
	state_vars.delta		=0;
	state_vars.dot_delta	=0;
	state_vars.theta_P		=0;
	state_vars.dot_theta_P	=0;
	state_vars.torque_bal	=0;
	state_vars.torque_steer	=0;
	state_vars.pitchAngleComplementary=0;
	state_vars.set_delta    =0;
	state_vars.set_dot_delta=0;
	state_vars.set_Xb		=0;
	state_vars.set_dot_Xb	=0;
	state_vars.set_ddot_Xb	=0;

	// velocity calculators
	filter_reset_second_order_filter(&self_balancing_params.dot_Xb_filt);
	filter_reset_second_order_filter(&self_balancing_params.dot_delta_filt);
	// acceleration calculator
	filter_reset_second_order_filter(&self_balancing_params.ddot_Xb_filt);
	// integrators
	filter_reset_first_order_filter(&self_balancing_params.set_delta_filt);
	filter_reset_first_order_filter(&self_balancing_params.set_Xb_filt);
	filter_reset_first_order_filter(&self_balancing_params.set_dot_Xb_filt);
	// PID
	filter_reset_first_order_filter(&self_balancing_params.steering_PID);
	// accelerometer
	filter_reset_second_order_filter(&self_balancing_params.accelero_filter);
	//
	filter_reset_second_order_filter(&self_balancing_params.gyro_filter);
	// output filters
	filter_reset_second_order_filter(&self_balancing_params.torque_bal_filt);
	filter_reset_second_order_filter(&self_balancing_params.torque_steer_filt);
}

void InitStateVars(void){
	state_vars.simulation_active = false;
	state_vars.send_angle_via_bluetooth = false;
	state_vars.loop_counter	=0;

	ResetStateVars();
}

// PWM peripheral: at timer reload provides the "tick" for the control loop
static void pwmpcb(PWMDriver *pwmp) {
// PWM reset (period) routine
  (void)pwmp;
  chEvtSignal(ctl_loop_thread,CH_EVT_TMR_LOOP_TRIGGER);
}
// PWM peripheral: if it reaches the "compare" value, signal the TIMEOUT
static void pwmc1cb(PWMDriver *pwmp) {
// PWM channel1 routine
  (void)pwmp;
  chEvtSignal(ctl_loop_thread,CH_EVT_TMR_LOOP_TIMEOUT);
}

static PWMConfig pwmcfg = {
  LOOP_TIMER_FREQ,								/* Hz PWM clock frequency.   */
  (LOOP_TIMER_FREQ*LOOP_PERIOD_MS)/1000,        /* Loop period       */
  pwmpcb,
  {
   {PWM_OUTPUT_ACTIVE_HIGH, pwmc1cb},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL}
  },
  0,
  0
};


void init_PWM_loop_trigger(uint8_t loop_period_ms){
	pwmStop(&PWMD5);
	// initialize PWM: the timer that triggers the loop function.
	pwmStart(&PWMD5, &pwmcfg);
	pwmChangePeriod(&PWMD5,(LOOP_TIMER_FREQ*loop_period_ms)/1000);
	pwmEnablePeriodicNotification(&PWMD5);
	// Starts the PWM channel 0 using 11% duty cycle.
	pwmEnableChannel(&PWMD5, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD5, 1100));
	pwmEnableChannelNotification(&PWMD5, 0);
}

void self_balancing_init(void){
	// init variables
	InitStateVars();

	// initialize safety pin
	// setup "servo" pin as input pullup: it is used as "enable" for the control algorithm
	palSetPadMode(EMERGENCY_BUTT_PORT, EMERGENCY_BUTT_BIT, PAL_MODE_INPUT_PULLUP);

	// initialize auxiliary modules
#if (defined (PRINT_DEBUG))|(defined (PRINT_DEBUG_CTL_BALANCE))|(defined (PRINT_DEBUG_CTL_STEER))|(defined (PRINT_DEBUG_IMU))|(defined (PRINT_DEBUG_JOYSTICK))|(defined (PRINT_DEBUG_STATE_VARS))|(defined (PRINT_DEBUG_TORQUES))|(defined (PRINT_DEBUG_UARTMOT))
	comm_usb_serial_init(); // USB print debug
#endif
	accel_full_scale = MPU9250_ACCEL_FS_2;	// [g]
	gyro_full_scale = MPU9250_GYRO_FS_250;	// [deg/s]
	imu_LPF_settings = MPU9250_DLPF_BW_5; // [Hz]

	switch(accel_full_scale){
		case MPU9250_ACCEL_FS_2:
			accelero_coeff=(float) 2 /32768*G_ACC;
		break;
		case MPU9250_ACCEL_FS_4:
			accelero_coeff=(float) 8 /32768*G_ACC;
		break;
		case MPU9250_ACCEL_FS_8:
			accelero_coeff=(float) 4 /32768*G_ACC;
		break;
		case MPU9250_ACCEL_FS_16:
			accelero_coeff=(float) 16 /32768*G_ACC;
		break;
	}

	switch(gyro_full_scale){
		case MPU9250_GYRO_FS_250:
			gyro_coeff=(float) 250/32768*PI/180;
		break;
		case MPU9250_GYRO_FS_500:
			gyro_coeff=(float) 500/32768*PI/180;
		break;
		case MPU9250_GYRO_FS_1000:
			gyro_coeff=(float) 1000/32768*PI/180;
		break;
		case MPU9250_GYRO_FS_2000:
			gyro_coeff=(float) 2000/32768*PI/180;
		break;
	}

	MPU9250_init(accel_full_scale,gyro_full_scale,imu_LPF_settings);// IMU

	JoystickInit();			// Joystick

	conf_general_read_self_balancing_params(&self_balancing_params);
	//conf_general_get_default_self_balancing_params(&self_balancing_params);

	// INITIALIZE KALMAN FILTER
	k_sys = kalman_Init(2);
	k_sys->Q_angle = 0.00000001;//0.001;
	k_sys->Q_bias = 0.0001;//0.001;
	k_sys->R_measure = 20;//100;
	k_sys->dt = (MATRIX_DATATYPE)(self_balancing_params.loop_period_ms)/1000;

	// Give to CAN controller a function to call when tacho values are received.
	comm_can_set_callback_tacho_recv(slave_tacho_received);

	// create the thread of control loop. Attention: this function holds the event catchers, thus it
	// is necessary to start it before the event signals
	chThdCreateStatic(ctl_loop_thread_wa, sizeof(ctl_loop_thread_wa), HIGHPRIO , ControlLoopMain, NULL);

	init_PWM_loop_trigger(self_balancing_params.loop_period_ms);
}
