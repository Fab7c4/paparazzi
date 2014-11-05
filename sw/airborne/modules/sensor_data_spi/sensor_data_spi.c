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

#ifndef SENSOR_DATA_SPI_LINK_DEVICE
#define SENSOR_DATA_SPI_LINK_DEVICE spi1
#endif

sensor_data_t sensor_data;
struct spi_transaction sensors_spi_link_transaction;

static volatile bool_t sensors_spi_link_ready = TRUE;

static void sensors_spi_link_trans_cb( struct spi_transaction *trans );

void spi_link_init(void);


void sensor_data_spi_init(void)
{
    spi_link_init();
    sensor_data.id = 0;
}

void sensor_data_spi_periodic(void)
{
    ImuScaleAccel(imu);
    ImuScaleGyro(imu);
    ImuScaleMag(imu);
    if (sensors_spi_link_ready)
    {
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

      spi_submit(&(SENSOR_DATA_SPI_LINK_DEVICE), &sensors_spi_link_transaction);
    }

    sensor_data.id++;

}

void spi_link_init(void) {

  sensors_spi_link_transaction.select        = SPISelectUnselect;
  sensors_spi_link_transaction.cpol          = SPICpolIdleHigh;
  sensors_spi_link_transaction.cpha          = SPICphaEdge2;
  sensors_spi_link_transaction.dss           = SPIDss8bit;
  sensors_spi_link_transaction.bitorder      = SPIMSBFirst;
  sensors_spi_link_transaction.cdiv          = SPIDiv64;
  sensors_spi_link_transaction.slave_idx     = SENSOR_DATA_SPI_LINK_SLAVE_NUMBER;
  sensors_spi_link_transaction.output_length = sizeof(sensor_data);
  sensors_spi_link_transaction.output_buf    = (uint8_t*) &sensor_data;
  sensors_spi_link_transaction.input_length  = 0;
  sensors_spi_link_transaction.input_buf     = NULL;
  sensors_spi_link_transaction.after_cb      = sensors_spi_link_trans_cb;
}

static void sensors_spi_link_trans_cb( struct spi_transaction *trans __attribute__ ((unused)) ) {
  sensors_spi_link_ready = TRUE;
}
