/*
 * Copyright (C) Fabian Girrbach
 *
 * This file is part of paparazzi

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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#ifndef SENSOR_DATA_SPI_H
#define SENSOR_DATA_SPI_H

#include "std.h"
#include "modules/imu_highwind/imu_highwind.h"

extern void sensor_data_spi_init(void);

extern void sensor_data_spi_periodic(void);


#define PACKED __attribute__((__packed__))


typedef struct PACKED{
    uint32_t sequence_number;         // 1
    uint32_t ticks;         // 1
    int32_t acc_x;      // 5
    int32_t acc_y;
    int32_t acc_z;
    int32_t gyro_p;     // 2
    int32_t gyro_q;
    int32_t gyro_r;
    int32_t mag_x;      // 8
    int32_t mag_y;
    int32_t mag_z;
}sensor_data_imu_t ;


typedef struct PACKED{
    uint32_t sequence_number;         // 1
    uint32_t ticks;         // 1
    uint16_t raw;
    uint16_t offset;
    float scaled;        // 11
}sensor_data_airspeed_t ;

typedef struct PACKED{
    uint32_t sequence_number;         // 1
    uint32_t ticks;         // 1
    sensor_data_imu_t imu[IMU_HIGHWIND_ARRAY_SIZE];
    sensor_data_airspeed_t airspeed;
    uint8_t checksum1;     // 14
    uint8_t checksum2;     // 15
}sensor_data_t ;


#endif
