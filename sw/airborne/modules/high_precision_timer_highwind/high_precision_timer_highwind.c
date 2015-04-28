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
 * @file "modules/high_precision_timer_highwind/high_precision_timer_highwind.c"
 * @author Maximilian Ernestus
 * 
 */
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include "modules/high_precision_timer_highwind/high_precision_timer_highwind.h"
#include "mcu_arch.h"
#include "mcu_periph/sys_time.h"



void high_precision_timer_highwind_init(void) {
    rcc_periph_clock_enable(RCC_TIM7);
    timer_reset(TIM7);
    //based on https://github.com/libopencm3/libopencm3-examples/blob/master/examples/stm32/l1/stm32l-discovery/button-irq-printf/main.c
    //Increase timer at 50 khz
    timer_set_prescaler(TIM7, timer_get_frequency(TIM7)/50000 - 1);
    //We devide the second into 50k ticks.
    //That means each tick is 20 us long.
    timer_set_period(TIM7, 50000);
    timer_enable_counter(TIM7);
}

unsigned long high_precision_timer_highwind_get_us(void) {
    return timer_get_counter(TIM7) * 20;
}

void high_precision_timer_highwind_periodic(void) {

}

void high_precision_timer_highwind_set_time(unsigned long time) {
	timer_set_counter(TIM7, time);
}


