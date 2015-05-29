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
 * @file "modules/servo_commands_highwind/servo_commands_highwind.h"
 * @author Fabian Girrbach
 * Module for setting servos from data
 */

#ifndef SERVO_COMMANDS_HIGHWIND_H
#define SERVO_COMMANDS_HIGHWIND_H

#ifndef SERVO_COMMAND_THROTTLE_SERVO
#define SERVO_COMMAND_THROTTLE_SERVO THROTTLE
#endif

#ifndef SERVO_COMMAND_ELEVATOR_SERVO
#define SERVO_COMMAND_ELEVATOR_SERVO ELEVATOR
#endif

#ifndef SERVO_COMMAND_RUDDER_SERVO
#define SERVO_COMMAND_RUDDER_SERVO RUDDER
#endif

#ifndef SERVO_COMMAND_AILERON_RIGHT_SERVO
#define SERVO_COMMAND_AILERON_RIGHT_SERVO AILERON_RIGHT
#endif

#ifndef SERVO_COMMAND_AILERON_LEFT_SERVO
#define SERVO_COMMAND_AILERON_LEFT_SERVO AILERON_LEFT
#endif

#ifndef SERVO_COMMAND_FLAP_RIGHT_SERVO
#define SERVO_COMMAND_FLAP_RIGHT_SERVO FLAP_RIGHT
#endif

#ifndef SERVO_COMMAND_FLAP_LEFT_SERVO
#define SERVO_COMMAND_FLAP_LEFT_SERVO FLAP_LEFT
#endif

extern void servo_commands_highwind_init(void);
extern void servo_commands_highwind_periodic(void);
extern void servo_commands_highwind_callback(void);

#endif
