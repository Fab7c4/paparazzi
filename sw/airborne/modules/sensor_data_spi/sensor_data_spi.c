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
#include "mcu_periph/sys_time.h"
#include "led.h"
#include "mcu_periph/gpio.h"

#ifndef SENSOR_DATA_SPI_LINK_DEVICE
#define SENSOR_DATA_SPI_LINK_DEVICE spi1
#endif

sensor_data_t sensor_data;
struct spi_transaction sensors_spi_link_transaction;

static volatile bool_t sensors_spi_link_ready = TRUE;

/// Declaration of methods
static void sensors_spi_link_trans_cb( struct spi_transaction *trans );
static void spi_link_sensors_init(void);
static void getIMUData(void);
static void getAirspeedData(void);
static void calculate_checksum(uint8_t* buffer, uint16_t length);


uint8_t input[sizeof(sensor_data_t)];


#define USE_DEBUG_DATA

void sensor_data_spi_init(void)
{
    spi_link_sensors_init();
    sensor_data.header.sequence_number = 0;
    //    sensor_data.test_buffer[0] =255;
    //    for (uint32_t i = 1 ; i < sizeof(sensor_data.test_buffer) ; i++ )
    //    {
    //        sensor_data.test_buffer[i] =(uint8_t) i;
    //    }

    gpio_setup_output(GPIO_BANK_UART5_TX, GPIO_UART5_TX);

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
        gpio_set(GPIO_BANK_UART5_TX, GPIO_UART5_TX);
        LED_TOGGLE(5);
        sensors_spi_link_ready = FALSE;
        getIMUData();
        getAirspeedData();
// #ifndef USE_DEBUG_DATA
//         sensor_data.gyro_p     = imu.gyro.p;
//         sensor_data.gyro_q     = imu.gyro.q;
//         sensor_data.gyro_r     = imu.gyro.r;
//         sensor_data.acc_x      = imu.accel.x;
//         sensor_data.acc_y      = imu.accel.y;
//         sensor_data.acc_z      = imu.accel.z;
//         sensor_data.mag_x      = imu.mag.x;
//         sensor_data.mag_y      = imu.mag.y;
//         sensor_data.mag_z      = imu.mag.z;
//         sensor_data.airspeed_raw = airspeed_ets_raw;
//         sensor_data.airspeed_offset = airspeed_ets_offset;
//         sensor_data.airspeed_scaled = airspeed_ets;
// #else
//         sensor_data.acc_x      = 1;
//         sensor_data.acc_y      = 2;
//         sensor_data.acc_z      = 3;
//         sensor_data.gyro_p     = 4;
//         sensor_data.gyro_q     = 5;
//         sensor_data.gyro_r     = 6;
//         sensor_data.mag_x      = 7;
//         sensor_data.mag_y      = 8;
//         sensor_data.mag_z      = 9;
//         sensor_data.airspeed_raw = 10;
//         sensor_data.airspeed_offset = 11;
//         sensor_data.airspeed_scaled = 12.12f;
// #endif
        calculate_checksum((uint8_t*) &sensor_data, sizeof(sensor_data_t)-2);
        spi_slave_register(&SENSOR_DATA_SPI_LINK_DEVICE, &sensors_spi_link_transaction);
    }
    sensor_data.header.sequence_number++;
    sensor_data.header.ticks = sys_time.nb_tick;
}



static void getIMUData()
{
    for (uint16_t i = 0; i < IMU_HIGHWIND_ARRAY_SIZE; i++)
    {
        sensor_data.imu[i].header.sequence_number = imu_highwind_array[i].sequence_number;
        sensor_data.imu[i].header.ticks = imu_highwind_array[i].ticks;
        sensor_data.imu[i].accel.x = imu_highwind_array[i].accel.x;
        sensor_data.imu[i].accel.y = imu_highwind_array[i].accel.y;
        sensor_data.imu[i].accel.z = imu_highwind_array[i].accel.z;
        sensor_data.imu[i].gyro.p = imu_highwind_array[i].gyro.p;
        sensor_data.imu[i].gyro.q = imu_highwind_array[i].gyro.q;
        sensor_data.imu[i].gyro.r = imu_highwind_array[i].gyro.r;
        sensor_data.imu[i].mag.x = imu_highwind_array[i].mag.x;
        sensor_data.imu[i].mag.y = imu_highwind_array[i].mag.y;
        sensor_data.imu[i].mag.z = imu_highwind_array[i].mag.z;
    }
    imu_highwind_reset();
}

static void getAirspeedData()
{
    sensor_data.airspeed.header.ticks = 0;
    sensor_data.airspeed.header.sequence_number = 1;
    // sensor_data.airspeed.raw = airspeed_ets_raw;
    // sensor_data.airspeed.offset = airspeed_ets_offset;
    // sensor_data.airspeed.scaled = airspeed_ets;
    sensor_data.airspeed.raw = 2;
    sensor_data.airspeed.offset = 3;
    sensor_data.airspeed.scaled = 4;

}

static void calculate_checksum(uint8_t* buffer, uint16_t length)
{
     sensor_data.footer.checksum1 = 0;
     sensor_data.footer.checksum2 = 0;

    for (uint32_t idx = 0; idx < length; idx++)
    {
        sensor_data.footer.checksum1 += buffer[idx];
        sensor_data.footer.checksum2 += sensor_data.footer.checksum1;
    }
}




static void sensors_spi_link_trans_cb( struct spi_transaction *trans __attribute__ ((unused)) ) {
    sensors_spi_link_ready = TRUE;
    gpio_clear(GPIO_BANK_UART5_TX, GPIO_UART5_TX);
}
