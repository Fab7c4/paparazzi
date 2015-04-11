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
 * @file "modules/imu_highwind/imu_highwind.c"
 * @author Fabian Girrbach
 * Offer interface for storing IMU data in an array
 */

#include "modules/imu_highwind/imu_highwind.h"



PRINT_CONFIG_VAR(IMU_HIGHWIND_ARRAY_SIZE)
PRINT_CONFIG_VAR(USE_IMU_HIGHWIND)



Imu_highwind_t* imu_highwind_ptr;
Imu_highwind_t imu_highwind_array[IMU_HIGHWIND_ARRAY_SIZE];
uint16_t current_imu_highwind_array_idx;
uint32_t imu_highwind_sequence_number;


void imu_highwind_init() {
    current_imu_highwind_array_idx = 0;
    imu_highwind_sequence_number = 0;
    imu_highwind_ptr = &imu_highwind_array[0];
}

void imu_highwind_reset() {
    memset(imu_highwind_array, 0, sizeof(imu_highwind_array));
    current_imu_highwind_array_idx = 0;
    imu_highwind_ptr = &imu_highwind_array[0];
}

void imu_highwind_update() {
    current_imu_highwind_array_idx++;
    if (current_imu_highwind_array_idx > IMU_HIGHWIND_ARRAY_SIZE-1)
    {
        current_imu_highwind_array_idx = 0;
    }
    imu_highwind_ptr = &imu_highwind_array[current_imu_highwind_array_idx];
}
