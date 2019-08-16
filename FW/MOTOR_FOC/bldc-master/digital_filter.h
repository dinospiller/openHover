/*
	Copyright 2012-2014 Benjamin Vedder	benjamin@vedder.se

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
 * digital_filter.h
 *
 *  Created on: 24 nov 2012
 *      Author: benjamin
 */

#ifndef DIGITAL_FILTER_H_
#define DIGITAL_FILTER_H_

#include <stdint.h>


/** First order digital filter. It is represented in the form:
 * Y(z)     N0 + N1*z^(-1)
 * ---- =  ----------------
 * U(z)      1 + D1*z^(-1)
 * Thus, the 3 coefficients must be specified
 */
typedef struct{
	float prev_input; // u(z^(-1))
	float prev_output;// y(z^(-1))
	float N0;
	float N1;
	float D1;
}FILT_first_order;

/** Second order digital filter. It is represented in the form:
* Y(z)     N0 + N1*z^(-1) + N2*z^(-2)
* ---- =  ----------------------------
* U(z)      1 + D1*z^(-1) + D2*z^(-2)
* Thus, the 5 coefficients must be specified
*/
typedef struct{
	float prev_input; // u(z^(-1))
	float preprev_input; // u(z^(-2))
	float prev_output;// y(z^(-1))
	float preprev_output;// y(z^(-2))
	float N0;
	float N1;
	float N2;
	float D1;
	float D2;
}FILT_second_order;

// Functions
void filter_fft(int dir, int m, float *real, float *imag);
void filter_dft(int dir, int len, float *real, float *imag);
void filter_fftshift(float *data, int len);
void filter_hamming(float *data, int len);
void filter_zeroPad(float *data, float *result, int dataLen, int resultLen);
void filter_create_fir_lowpass(float *filter_vector, float f_break, int bits, int use_hamming);
float filter_run_fir_iteration(float *vector, float *filter, int bits, uint32_t offset);
void filter_add_sample(float *buffer, float sample, int bits, uint32_t *offset);
float filter_first_order_process_sample(FILT_first_order *filt, float new_sample);
float filter_second_order_process_sample(FILT_second_order *filt, float new_sample);
void filter_init_first_order_filter(FILT_first_order *filt,float N0, float N1, float D1);
void filter_init_second_order_filter(FILT_second_order *filt,float N0, float N1, float N2, float D1, float D2);
void filter_reset_first_order_filter(FILT_first_order *filt);
void filter_reset_second_order_filter(FILT_second_order *filt);
#endif /* DIGITAL_FILTER_H_ */
