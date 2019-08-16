/*
 * joystick.h
 *
 *  Created on: 14/giu/2017
 *      Author: mac_daino
 */

#ifndef CONTROLLER_BOARD_JOYSTICK_H_
#define CONTROLLER_BOARD_JOYSTICK_H_

#include <stdint.h>

void JoystickInit(void);
float JoystickGetForward(void);
float JoystickGetSteering(void);

#endif /* CONTROLLER_BOARD_JOYSTICK_H_ */
