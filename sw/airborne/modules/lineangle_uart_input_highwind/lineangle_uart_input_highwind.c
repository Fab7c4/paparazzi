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

#define LA_STARTBYTE 42
#define LA_NUM_STARTBYTES 3


void lineangle_uart_input_highwind_init(void) {
	uart_periph_init(&LA_UART);
	uart_periph_set_mode(&LA_UART, false, true, false);
	uart_periph_set_bits_stop_parity(&LA_UART, UBITS_8, USTOP_1, UPARITY_NO);
}

uint8_t numStartbytesRead = 0;
sensor_data_lineangle_t latestLineangleReading1;
sensor_data_lineangle_t latestLineangleReading2;

void lineangle_uart_input_highwind_periodic(void) {


	while(uart_char_available(&LA_UART) > 0 && numStartbytesRead != LA_NUM_STARTBYTES) {
		uint8_t inchar = uart_getch(&LA_UART);
		if(inchar == LA_STARTBYTE)
			++numStartbytesRead;
		else
			numStartbytesRead = 0;
	}

	if(numStartbytesRead == LA_NUM_STARTBYTES) {
		readLineanglePackage(&latestLineangleReading1);
		int i = 0;
	}
}

void readLineanglePackage(sensor_data_lineangle_t* out) {
	if(uart_char_available(&LA_UART) >= sizeof(latestLineangleReading1)) {
		char* buf = (char*) out;
		for(size_t i = 0; i < sizeof(latestLineangleReading1); ++i) {
			buf[i] = uart_getch(&LA_UART);
		}
		numStartbytesRead = 0;
	}
}
