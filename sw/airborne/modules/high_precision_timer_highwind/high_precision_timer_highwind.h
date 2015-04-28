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
 * @file "modules/high_precision_timer_highwind/high_precision_timer_highwind.h"
 * @author Maximilian Ernestus
 * 
 */

#ifndef HIGH_PRECISION_TIMER_HIGHWIND_H
#define HIGH_PRECISION_TIMER_HIGHWIND_H

extern unsigned long millis;

extern void high_precision_timer_highwind_init(void);
extern void high_precision_timer_highwind_periodic(void);
extern void high_precision_timer_highwind_set_time(unsigned long time);
extern unsigned long high_precision_timer_highwind_get_us(void);
#endif

