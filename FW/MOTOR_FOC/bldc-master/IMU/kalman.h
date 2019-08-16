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
 * kalman.h
 *
 *  Created on: 05/nov/2014
 *      Author: mac_daino
 */

#ifndef KALMAN_H_
#define KALMAN_H_

#include "mat_math.h"

typedef struct{

	MATRIX_DATATYPE bias;	// cumulative bias of the gyroscope sensor (drift)
	MATRIX_DATATYPE rate;	// angle speed measured by gyroscope sensor
	MATRIX_DATATYPE angle;	// angle measured by accelerometer
	MATRIX_DATATYPE dt;		// time between 2 successive measures [s]
	MATRIX_DATATYPE err;	// residual error: important for the tuning of the filter
	MATRIX_DATATYPE Q_angle;// variance of angle measure
	MATRIX_DATATYPE Q_bias;	// variance of gyroscope drift
	MATRIX_DATATYPE R_measure;	// variance of accelerometer measure

	matrix_t* P;
	MATRIX_DATATYPE S;
	matrix_t* K;
}kalman_str_t;

kalman_str_t* kalman_Init(uint8_t sysdim);
void kalman_Destroy(kalman_str_t* sys);
void kalman_Process_sample(kalman_str_t* sys,MATRIX_DATATYPE newAngle, MATRIX_DATATYPE newRate);
#endif /* KALMAN_H_ */
