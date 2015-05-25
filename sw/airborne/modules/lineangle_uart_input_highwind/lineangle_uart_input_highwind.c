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
 * @file "modules/lineangle_uart_input_highwind/lineangle_uart_input_highwind.c"
 * @author Maximilian Ernestus
 * Reads lineangle readings via UART as sent by an arduino that does the actual communication with the lineangle sensor.
 */

#include "modules/lineangle_uart_input_highwind/lineangle_uart_input_highwind.h"

#include "mcu_periph/uart.h"

#define LA_STARTBYTE 'X'
#define LA_NUM_STARTBYTES 3


void lineangle_uart_input_highwind_init() {
	uart_periph_init(&LA_UART);
	uart_periph_set_mode(&LA_UART, false, true, false);
	//TODO: Set parity bits to match arduino
	//uart_periph_set_bits_stop_parity(&HS_LOG_UART, 8, 1, 0);
}

uint8_t numStartbytesRead = 0;

void lineangle_uart_input_highwind_periodic() {

	while(uart_char_available(&LA_UART) > 0 && numStartbytesRead != LA_NUM_STARTBYTES)
		if(uart_getch(&LA_UART) == LA_STARTBYTE)
			++numStartbytesRead;
		else
			numStartbytesRead = 0;

	if(numStartbytesRead == LA_NUM_STARTBYTES)
		if(uart_char_available(&LA_UART) >= 55)
			readLineanglePackage();

}

void readLineanglePackage() {

}


