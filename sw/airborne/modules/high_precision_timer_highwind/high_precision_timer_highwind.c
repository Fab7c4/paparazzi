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


#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>


#include "modules/high_precision_timer_highwind/high_precision_timer_highwind.h"
#include "mcu_arch.h"
#include "mcu_periph/sys_time.h"


#define HIGH_PRECISION_TIMER_PRESCALE 9


#define GPS_HEARTBEAT_USER_PORT GPIOA
#define GPS_HEARTBEAT_USER_PIN GPIO0
#define GPS_HEARTBEAT_USER_EXTI EXTI0
#define GPS_HEARTBEAT_USER_isr exti0_isr
#define GPS_HEARTBEAT_USER_NVIC NVIC_EXTI0_IRQ

extern uint32_t high_precision_timer_ticks_per_sec = 0;

void high_precision_timer_highwind_init(void) {
    rcc_periph_clock_enable(RCC_TIM7);

    //setup TIM7
    timer_reset(TIM7);
    timer_set_prescaler(TIM7, HIGH_PRECISION_TIMER_PRESCALE-1);
    timer_set_period(TIM7, 0xFFFF);

    //enable the interrupt on TIM7
	#ifdef STM32F1
	  nvic_set_priority(NVIC_TIM7_IRQ, 2);
	  nvic_enable_irq(NVIC_TIM7_IRQ);
	#elif defined STM32F4
	  //the define says DAC IRQ, but it is also the global TIM6 IRQ
	  nvic_set_priority(NVIC_TIM7_DAC_IRQ, 2);
	  nvic_enable_irq(NVIC_TIM7_DAC_IRQ);
	#endif
    timer_enable_irq(TIM7, TIM_DIER_UIE);
    timer_clear_flag(TIM7, TIM_SR_UIF);

    //finally enable the whole timer TIM7
    timer_enable_counter(TIM7);

    //Store the ticks per second so they can be sent out together with the ticks
    high_precision_timer_ticks_per_sec = timer_get_frequency(TIM7)/HIGH_PRECISION_TIMER_PRESCALE;

	//setup the interrupt for the 1 second pulse coming from the gps (Does not compile yet)
//	nvic_enable_irq(GPS_HEARTBEAT_USER_NVIC);
//	gpio_mode_setup(GPS_HEARTBEAT_USER_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPS_HEARTBEAT_USER_PIN);
//	/* Configure the EXTI subsystem. */
//	exti_select_source(GPS_HEARTBEAT_USER_EXTI, GPS_HEARTBEAT_USER_PORT);
//	exti_set_trigger(GPS_HEARTBEAT_USER_EXTI, EXTI_TRIGGER_RISING);
//	exti_enable_request(GPS_HEARTBEAT_USER_EXTI);
}

uint32_t overflowCounter = 0;

#ifdef STM32F1
void tim7_isr(void) {
#elif defined STM32F4
void tim7_dac_isr(void) {
#endif
	timer_clear_flag(TIM7, TIM_SR_UIF);
	++overflowCounter;
}

uint32_t high_precision_timer_highwind_get_tics(void) {
	return overflowCounter * 0xFFFF + timer_get_counter(TIM7);
}

void high_precision_timer_highwind_periodic(void) {
}

void high_precision_timer_highwind_reset() {
	timer_set_counter(TIM7, 0);
	overflowCounter = 0;
}

void high_precision_timer_highwind_gps_pulse_interupt() {
	high_precision_timer_highwind_reset();
}


