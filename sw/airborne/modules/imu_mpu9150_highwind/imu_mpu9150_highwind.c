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
 * @file "modules/imu_mpu9150_highwind/imu_mpu9150_highwind.c"
 * @author Fabian Girrbach
 * IMU MPU9150
 */

#include "modules/imu_mpu9150_highwind/imu_mpu9150_highwind.h"
#include "modules/imu_highwind/imu_highwind.h"
#include "modules/high_precision_timer_highwind/high_precision_timer_highwind.h"
#include "mcu_periph/i2c.h"


#include "math/pprz_algebra_int.h"


// void imu_mpu9150_highwind_init() {}
// void imu_mpu9150_highwind_periodic() {}
// void imu_mpu9150_highwind_event {}

// Default I2C address
#ifndef IMU_MPU9250_ADDR
#define IMU_MPU9250_ADDR MPU9250_ADDR
#endif
#ifndef IMU_MPU9250_I2C_DEV
#define IMU_MPU9250_I2C_DEV i2c2
#endif
PRINT_CONFIG_VAR(IMU_MPU9250_I2C_DEV)

struct Mpu9250_I2c mpu9150;

uint32_t sequenceNumber;

void imu_mpu9150_highwind_init(void)
{
  mpu9250_i2c_init(&mpu9150, &IMU_MPU9250_I2C_DEV, IMU_MPU9250_ADDR);
}

void imu_mpu9150_highwind_periodic(void)
{
  mpu9250_i2c_periodic(&mpu9150);
}

void imu_mpu9150_highwind_event(void)
{
  mpu9250_i2c_event(&mpu9150);
  uint32_t ticks_event = high_precision_timer_highwind_get_tics();
  uint32_t incremented = high_precision_timer_highwind_get_seconds();
  imu_highwind_ptr->ticks = ticks_event;
  imu_highwind_ptr->type = IMU_TYPE_ARM ;
  imu_highwind_ptr->incremented = incremented;
  imu_highwind_ptr->sequence_number = sequenceNumber;
  RATES_COPY(imu_highwind_ptr->gyro,mpu9150.data_rates.rates);
  VECT3_COPY(imu_highwind_ptr->accel,mpu9150.data_accel.vect);
  VECT3_COPY(imu_highwind_ptr->mag,mpu9150.akm.data.vect);
  sequenceNumber++;

  imu_highwind_update();
}
