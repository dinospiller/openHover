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
 * kalman.c
 *
 *  Created on: 05/nov/2014
 *      Author: mac_daino
 *      this file contains main routines that performs kalman algorythm
 *      Note: system is supposed to be SISO, because of that: y is a single value, u is
 * 			a single value, S is an n*1 vector, and even B. C is a row vector
 */

#include "stdint.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "kalman.h"

//------------------------------ VARIABLES ------------------------------------


//---------------------------- END VARIABLES ----------------------------------
//
///**
// * @brief: update stage of the kalman filter
// * @note: system is supposed to be SISO, because of that: y is a single value, u is
// * 			a single value, S is an n*1 vector, and even B. C is a row vector
// * @return: 0 if everything is correct. Nonzero otherwise.
// */
//int kalman_Update(kalman_str_t* sys){
//	int ret=0;
//
///*
// 	L=P*C'*inv(C*P*C'+R);
//	e=y-C*x-D*u;
//	I=eye(size(L*C));
//
//	P=(I-L*C)*P*(I-L*C)'+L*R*L';
//	x=x+L*e;
//*/
//	return ret;
//}
///**
// * @brief: update stage of the kalman filter
// * @note: system is supposed to be SISO, because of that: y is a single value, u is
// *			a single value, S is an n*1 vector, and even B. C is a row vector
// * @params: P = previous-step variance of estimated state (will be modified)
// * @params: x = previous-step predicted state
// * @params: x_next = predicted value of state (is an output)
// * @params: C = C matrix
// * @params: R = R matrix
// * @params: y = current y(t) sample
// * @params: F = F matrix
// * @params: D = D matrix
// * @params: u = current u(t) sample
// * @params: B = B matrix
// * @params: Q = state covariance matrix
// * @params: S = state and output covariance matrix
// * @params: sysdim = dimension of the system
// * @return: 0 if everything is correct. Nonzero otherwise.
// */
//int kalman_Predict(float* P,float* x, float R,float y, float* F, float D, float u, float* S,float* Q, float* B){
//	int ret=0;
//	return ret;
//}



/**
 * @brief: initializes the kalman filter, resetting all local variables, and the state.
 * 			It is also possible to re-initialize the filter, with different covariance matrices
 * @params: sysdim = dimension of the system
 */
kalman_str_t* kalman_Init(uint8_t sysdim){
	kalman_str_t* kalman_struct = malloc(sizeof(kalman_str_t));

	kalman_struct->P = matrix_create(sysdim,sysdim);
	kalman_struct->K = matrix_create(sysdim,1);
	kalman_struct->angle = 0.0;
	kalman_struct->bias = 0.0;
	kalman_struct->rate = 0.0;
	kalman_struct->dt = 0.05;

	kalman_struct->P->matr[0][0] = 0.0;
	kalman_struct->P->matr[0][1] = 0.0;
	kalman_struct->P->matr[1][0] = 0.0;
	kalman_struct->P->matr[1][1] = 0.0;
	return kalman_struct;
}

void kalman_Destroy(kalman_str_t* sys){
	if(sys!=NULL){
		matrix_destroy(sys->P);
		matrix_destroy(sys->K);
		free(sys);
	}
}

/**
 * @brief: runs the kalman filter iteration. ATTENTION: optimized (not complete) routine for Kalman filtering, dedicated only to filter IMU signals
 * @param: the kalman structure where to act
 * @param: the new accelerometer value (measure of the output of the system)
 * @param: the gyroscope reading
 */
void kalman_Process_sample(kalman_str_t* sys,MATRIX_DATATYPE newAngle, MATRIX_DATATYPE newRate){

	// Discrete Kalman filter time update equations - Time Update ("Predict")
	// Update xhat - Project the state ahead
	/* Step 1 */
	sys->rate = newRate - sys->bias;
	sys->angle += sys->dt * sys->rate;
	// Update estimation error covariance - Project the error covariance ahead
	/* Step 2 */
	sys->P->matr[0][0] += sys->dt * (sys->dt*sys->P->matr[1][1] - sys->P->matr[0][1] - sys->P->matr[1][0] + sys->Q_angle);
	sys->P->matr[0][1] -= sys->dt * sys->P->matr[1][1];
	sys->P->matr[1][0] -= sys->dt * sys->P->matr[1][1];
	sys->P->matr[1][1] += sys->Q_bias * sys->dt;
	// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
	// Calculate Kalman gain - Compute the Kalman gain
	/* Step 4 */
	sys->S = sys->P->matr[0][0] + sys->R_measure;
	/* Step 5 */
	sys->K->matr[0][0] = sys->P->matr[0][0] / sys->S;
	sys->K->matr[1][0] = sys->P->matr[1][0] / sys->S;
	// Calculate angle and bias - Update estimate with measurement zk (newAngle)
	/* Step 3 */
	sys->err = newAngle - sys->angle;
	/* Step 6 */
	sys->angle += sys->K->matr[0][0] * sys->err;
	sys->bias += sys->K->matr[1][0] * sys->err;
	// Calculate estimation error covariance - Update the error covariance
	/* Step 7 */
	sys->P->matr[0][0] -= sys->K->matr[0][0] * sys->P->matr[0][0];
	sys->P->matr[0][1] -= sys->K->matr[0][0] * sys->P->matr[0][1];
	sys->P->matr[1][0] -= sys->K->matr[1][0] * sys->P->matr[0][0];
	sys->P->matr[1][1] -= sys->K->matr[1][0] * sys->P->matr[0][1];
}

