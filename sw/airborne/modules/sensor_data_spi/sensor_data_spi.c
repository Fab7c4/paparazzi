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

#include "modules/sensor_data_spi/sensor_data_spi.h"
#include "modules/sensors/airspeed_ets.h"

#include "subsystems/imu.h"
#include "mcu_periph/spi.h"
#include "led.h"

#ifndef SENSOR_DATA_SPI_LINK_DEVICE
#define SENSOR_DATA_SPI_LINK_DEVICE spi1
#endif

sensor_data_t sensor_data;
struct spi_transaction sensors_spi_link_transaction;

static volatile bool_t sensors_spi_link_ready = TRUE;

static void sensors_spi_link_trans_cb( struct spi_transaction *trans );

static void spi_link_sensors_init(void);
static void calculate_checksum(uint8_t* buffer, uint16_t length);


uint8_t input[sizeof(sensor_data_t)];


void sensor_data_spi_init(void)
{
    spi_link_sensors_init();
    sensor_data.sequence_number = 0;
    //    sensor_data.test_buffer[0] =255;
    //    for (uint32_t i = 1 ; i < sizeof(sensor_data.test_buffer) ; i++ )
    //    {
    //        sensor_data.test_buffer[i] =(uint8_t) i;
    //    }
}


static void spi_link_sensors_init(void) {

    sensors_spi_link_transaction.cpol          = SPICpolIdleHigh;
    sensors_spi_link_transaction.cpha          = SPICphaEdge2;
    sensors_spi_link_transaction.dss           = SPIDss8bit;
    sensors_spi_link_transaction.bitorder      = SPIMSBFirst;
    sensors_spi_link_transaction.output_length = sizeof(sensor_data);
    sensors_spi_link_transaction.output_buf    = (uint8_t*) &sensor_data;
    sensors_spi_link_transaction.input_length  = sizeof(sensor_data);
    sensors_spi_link_transaction.input_buf     = (uint8_t*) input;
    sensors_spi_link_transaction.after_cb      = sensors_spi_link_trans_cb;
}

void sensor_data_spi_periodic(void)
{
    if (sensors_spi_link_ready) {
        LED_TOGGLE(5);
        sensors_spi_link_ready = FALSE;
        sensor_data.gyro_p     = imu.gyro.p;
        sensor_data.gyro_q     = imu.gyro.q;
        sensor_data.gyro_r     = imu.gyro.r;
        sensor_data.acc_x      = imu.accel.x;
        sensor_data.acc_y      = imu.accel.y;
        sensor_data.acc_z      = imu.accel.z;
        sensor_data.mag_x      = imu.mag.x;
        sensor_data.mag_y      = imu.mag.y;
        sensor_data.mag_z      = imu.mag.z;
        sensor_data.airspeed_raw = airspeed_ets_raw;
        sensor_data.airspeed_offset = airspeed_ets_offset;
        sensor_data.airspeed_scaled = airspeed_ets;
        calculate_checksum((uint8_t*) &sensor_data, sizeof(sensor_data_t)-2);
        spi_slave_register(&SENSOR_DATA_SPI_LINK_DEVICE, &sensors_spi_link_transaction);
    }
    sensor_data.sequence_number++;
}

static void calculate_checksum(uint8_t* buffer, uint16_t length)
{
     sensor_data.checksum1 = 0;
     sensor_data.checksum2 = 0;

    for (uint32_t idx = 0; idx < length; idx++)
    {
        sensor_data.checksum1 += buffer[idx];
        sensor_data.checksum2;
    }
}




static void sensors_spi_link_trans_cb( struct spi_transaction *trans __attribute__ ((unused)) ) {
    sensors_spi_link_ready = TRUE;
}
