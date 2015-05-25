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
 * @file "modules/imu_highwind/imu_highwind.h"
 * @author Fabian Girrbach
 * Offer interface for storing IMU data in an array
 */

#ifndef IMU_HIGHWIND_H
#define IMU_HIGHWIND_H

#include "math/pprz_algebra_int.h"


#ifndef IMU_HIGHWIND_ARRAY_SIZE
#define IMU_HIGHWIND_ARRAY_SIZE 20
#endif

#define IMU_TYPE_PLANE  1
#define IMU_TYPE_ARM  2

typedef struct {
    uint8_t type;
    uint32_t ticks;
    uint32_t incremented;
    uint32_t sequence_number;
    struct Int32Rates gyro;             ///< gyroscope measurements in rad/s in BFP with #INT32_RATE_FRAC
    struct Int32Vect3 accel;            ///< accelerometer measurements in m/s^2 in BFP with #INT32_ACCEL_FRAC
    struct Int32Vect3 mag;              ///< magnetometer measurements scaled to 1 in BFP with #INT32_MAG_FRAC
} Imu_highwind_t;

extern Imu_highwind_t* imu_highwind_ptr;
extern Imu_highwind_t imu_highwind_array[];
extern uint16_t current_imu_highwind_array_idx;
extern uint32_t imu_highwind_sequence_number;


extern void imu_highwind_init(void);

extern void imu_highwind_reset(void);

extern void imu_highwind_update(void);

#endif
