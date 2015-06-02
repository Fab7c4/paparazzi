/*
 * Copyright (C) Maximilian Ernestus
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
 * @file "modules/lineangle_uart_input_highwind/lineangle_uart_input_highwind.h"
 * @author Maximilian Ernestus
 * Reads lineangle readings via UART as sent by an arduino that does the actual communication with the lineangle sensor.
 */

#ifndef LINEANGLE_UART_INPUT_HIGHWIND_H
#define LINEANGLE_UART_INPUT_HIGHWIND_H
#include "modules/sensor_data_spi/sensor_data_spi.h"


extern void lineangle_uart_input_highwind_init(void);
extern void lineangle_uart_input_highwind_periodic(void);

void readLineanglePackage(sensor_data_lineangle_t* out);

#endif
