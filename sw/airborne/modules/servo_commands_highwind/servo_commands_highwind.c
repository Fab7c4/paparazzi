/*
 * Copyright (C) Fabian Girrbach
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/servo_commands_highwind/servo_commands_highwind.c"
 * @author Fabian Girrbach
 * Module for setting servos from data
 */

#include "modules/servo_commands_highwind/servo_commands_highwind.h"

#include "subsystems/datalink/datalink.h"
#include "generated/airframe.h"
#include "subsystems/actuators.h"

// One level of macro stack to allow redefinition of the default servo
#define _ServoManualCommand(_n, _v) ActuatorSet(_n, _v)
#define ServoManualCommand(_n, _v) _ServoManualCommand(_n, _v)

typedef union{ //message id 72
	uint8_t raw[14];
	struct Output_message {
			int16_t throttle;
			int16_t elevator;
			int16_t rudder;
			int16_t aileron_right;
			int16_t aileron_left;
			int16_t flap_right;
			int16_t flap_left;
		} message;
} InputData_t;

InputData_t input_actuators;

void servo_commands_highwind_init(void) {
}

void servo_commands_highwind_periodic(void)
{
    ServoManualCommand(SERVO_COMMAND_THROTTLE_SERVO,input_actuators.message.throttle);
	ServoManualCommand(SERVO_COMMAND_ELEVATOR_SERVO,input_actuators.message.elevator);
	ServoManualCommand(SERVO_COMMAND_RUDDER_SERVO, input_actuators.message.rudder);
	ServoManualCommand(SERVO_COMMAND_AILERON_RIGHT_SERVO,input_actuators.message.aileron_right);
	ServoManualCommand(SERVO_COMMAND_AILERON_LEFT_SERVO,input_actuators.message.aileron_left);
	ServoManualCommand(SERVO_COMMAND_FLAP_LEFT_SERVO,input_actuators.message.flap_right);
	ServoManualCommand(SERVO_COMMAND_FLAP_RIGHT_SERVO,input_actuators.message.flap_left);
}


void servo_commands_highwind_callback(void)
{
	for(uint16_t i=0;i<sizeof(InputData_t);i++){
        input_actuators.raw[i]=dl_buffer[2+i];
	}
}
