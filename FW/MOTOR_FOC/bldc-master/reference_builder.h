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
 * reference_builder.h
 *
 *  Created on: 09/set/2017
 *      Author: mac_daino
 */

#ifndef REFERENCE_BUILDER_H_
#define REFERENCE_BUILDER_H_

#define REF_MAX_REFERENCE_POINTS	10

typedef struct{
	float time;
	float value;
}REF_POINT;

typedef struct{
	int8_t last_valid; // index of last valid setpoint
	REF_POINT refpoints[REF_MAX_REFERENCE_POINTS];
}REF_SEQUENCE;

void reference_reset_sequence(REF_SEQUENCE* seq);
bool reference_add_setpoint(REF_SEQUENCE* seq, float time, float value);
float reference_get_curr_value(REF_SEQUENCE* seq,float time);

#endif /* REFERENCE_BUILDER_H_ */
