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
#define NUMBER_OF_IMU_DATA_PACKETS 10

///********************************************************************
/// GENERAL SUBMESSAGE DEFINITIONS
///********************************************************************


/// Footer definition
typedef struct PACKED{
    uint32_t crc32;                  // CRC32
} sensor_data_footer_t;

/// Header definition
typedef struct PACKED{
    uint32_t sequence_number;           // Incremented sequence number
    uint32_t ticks;                     // Number of ticks for timing
} sensor_data_header_t;

/// Datatype for VECTOR
typedef struct PACKED{
    int32_t x;      // 5
    int32_t y;
    int32_t z;
} sensor_data_xyz_t;

/// Datatype for RATES
typedef struct PACKED{
    int32_t p;
    int32_t q;
    int32_t r;
} sensor_data_pqr_t;


//********************************************************************
/// SENSOR MESSAGE DEFINITIONS
///********************************************************************


typedef struct PACKED{
    sensor_data_header_t header;
    sensor_data_xyz_t accel;
    sensor_data_pqr_t gyro;
    sensor_data_xyz_t mag;
}sensor_data_imu_t ;


typedef struct PACKED{
    sensor_data_header_t header;
    uint16_t raw;
    uint16_t offset;
    uint32_t scaled;
}sensor_data_airspeed_t ;


//********************************************************************
/// COMPLETE MESSAGE DEFINITION
///********************************************************************

typedef struct PACKED{
    sensor_data_header_t header;
    sensor_data_imu_t imu[NUMBER_OF_IMU_DATA_PACKETS];
    sensor_data_airspeed_t airspeed;
    sensor_data_footer_t footer;
}sensor_data_t ;

typedef struct PACKED{
    sensor_data_header_t header;
    sensor_data_footer_t footer;
}test_t ;


#endif
