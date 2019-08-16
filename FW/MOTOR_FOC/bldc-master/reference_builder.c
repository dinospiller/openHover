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
 * reference_builder.c
 *
 *  Created on: 10/set/2017
 *      Author: mac_daino
 */

#include "ch.h"
#include "reference_builder.h"

/*
 * Delete all setpoints of a reference sequence
 *
 * @param the sequence to be reset
 */
void reference_reset_sequence(REF_SEQUENCE* seq){
	uint8_t i;
	for(i=0;i<REF_MAX_REFERENCE_POINTS;i++){
		seq->refpoints[i].time=0;
		seq->refpoints[i].value=0;
	}
	seq->last_valid=-1;
}

/*
 * Add a setpoint to a reference sequence
 *
 * @param the sequence
 * @param the time of a setpoint
 * @param the value o set at the corresponding time
 * @return false if sequence is already full or if time requested is less than that of the last point
 */
bool reference_add_setpoint(REF_SEQUENCE* seq, float time_of_vlaue, float value){
	if(seq->last_valid==REF_MAX_REFERENCE_POINTS) return false;
	if(time_of_vlaue!=0 && time_of_vlaue <= seq->refpoints[seq->last_valid].time) return false;

	// if none of the previous, everything OK
	seq->refpoints[++seq->last_valid] = (REF_POINT) {time_of_vlaue,value};
	return true;
}

/*
 * This updates the current value and the other parameters, according to the time passed as a parameter,
 * by interpolating the reference points. Any reference point whose time value is less than its previous
 * is considered as non-valid, like all the others succeding
 *
 * @param the sequence in which to read points
 *
 * @param the absolute time in the sequence.
 *
 * @return the value corresponding to the time specified. If the time is greater than the time of
 * 			the last reference point, it will be returned the last value
 */
float reference_get_curr_value(REF_SEQUENCE* seq, float time){
	if(time==0){
		if(seq->refpoints[0].time == 0){
			return seq->refpoints[0].value;
		}else{
			return 0;
		}
	}else{
		// if time is greater than last valid setpoint
		if(time>=seq->refpoints[seq->last_valid].time)
			return seq->refpoints[seq->last_valid].value;

		uint8_t i=0;
		while(i<REF_MAX_REFERENCE_POINTS){ // find the imediately-next point
			if(seq->refpoints[i].time > time) break;
			i++;
		}
		// interpolation
		if(i==0){
			float ang_coeff = seq->refpoints[0].value/seq->refpoints[0].time;
			return time*ang_coeff;
		}else{
			float ang_coeff = (seq->refpoints[i].value-seq->refpoints[i-1].value)/
					(seq->refpoints[i].time-seq->refpoints[i-1].time);
			return (seq->refpoints[i-1].value)+(time-seq->refpoints[i-1].time)*ang_coeff;
		}
	}
}


