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
#include "modules/high_precision_timer_highwind/high_precision_timer_highwind.h"
#include "modules/sensors/airspeed_ets.h"
#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/crc.h"
#include "subsystems/imu.h"
#include "mcu_periph/spi.h"
#include "mcu_periph/sys_time.h"
#include "led.h"
#include "mcu_periph/gpio.h"

#ifndef SENSOR_DATA_SPI_LINK_DEVICE
#define SENSOR_DATA_SPI_LINK_DEVICE spi1
#endif

//#define USE_DEBUG_DATA

#ifndef USE_DEBUG_DATA
  sensor_data_t sensor_data;
  #else
  test_t sensor_data;
  #endif
struct spi_transaction sensors_spi_link_transaction;

static volatile bool_t sensors_spi_link_ready = TRUE;

uint8_t test[12];

/// Declaration of methods
static void sensors_spi_link_trans_cb( struct spi_transaction *trans );
static void spi_link_sensors_init(void);
static void getIMUData(void);
static void getAirspeedData(void);
static void calculate_checksum(uint8_t* buffer, uint16_t length);


uint8_t input[sizeof(sensor_data)];



void sensor_data_spi_init(void)
{
    spi_link_sensors_init();
    sensor_data.header.sequence_number = 0;

    gpio_setup_output(GPIO_BANK_UART5_TX, GPIO_UART5_TX);

    rcc_periph_clock_enable(RCC_CRC);

}


static void spi_link_sensors_init(void) {


  for (int i = 0 ; i < 12; i++)
  {
    test[i] = i;
  }

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
        sensor_data.header.sequence_number = 0x12345678;
        sensor_data.header.ticks = high_precision_timer_highwind_get_tics() ;
        sensor_data.header.incremented = high_precision_timer_highwind_get_seconds() ;
        sensors_spi_link_ready = FALSE;
        gpio_set(GPIO_BANK_UART5_TX, GPIO_UART5_TX);
        LED_TOGGLE(5);
        gpio_clear(GPIO_BANK_UART5_TX, GPIO_UART5_TX);
        getIMUData();
        getAirspeedData();
        calculate_checksum((uint8_t*) &sensor_data, sizeof(sensor_data)-4);
        spi_slave_register(&SENSOR_DATA_SPI_LINK_DEVICE, &sensors_spi_link_transaction);
    }

}



static void getIMUData()
{
    for (uint16_t i = 0; i < NUMBER_OF_IMU_DATA_PACKETS; i++)
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
    // int32_t testval = 1;
    // for (uint16_t i = 0; i < NUMBER_OF_IMU_DATA_PACKETS; i++)
    // {
    //     sensor_data.imu[i].header.sequence_number = testval++;
    //     sensor_data.imu[i].header.ticks = testval++;
    //     sensor_data.imu[i].accel.x = testval++;
    //     sensor_data.imu[i].accel.y = testval++;
    //     sensor_data.imu[i].accel.z = testval++;
    //     sensor_data.imu[i].gyro.p = testval++;
    //     sensor_data.imu[i].gyro.q = testval++;
    //     sensor_data.imu[i].gyro.r = testval++;
    //     sensor_data.imu[i].mag.x = testval++;
    //     sensor_data.imu[i].mag.y = testval++;
    //     sensor_data.imu[i].mag.z = testval++;
    // }

    imu_highwind_reset();
}

static void getAirspeedData()
{
  #ifndef USE_DEBUG_DATA
    sensor_data.airspeed.header.ticks = 1;
    sensor_data.airspeed.header.sequence_number = 2;
    // sensor_data.airspeed.raw = airspeed_ets_raw;
    // sensor_data.airspeed.offset = airspeed_ets_offset;
    // sensor_data.airspeed.scaled = airspeed_ets;
    sensor_data.airspeed.raw = 3;
    sensor_data.airspeed.offset = 4;
    sensor_data.airspeed.scaled = 5;
    #endif

}

static void calculate_checksum(uint8_t* buffer, uint16_t length)
{
    //OLD PAPARAZZI CHECKSUM USED FOR TELEMETRY
    //  sensor_data.footer.checksum1 = 0;
    //  sensor_data.footer.checksum2 = 0;
    //
    // for (uint32_t idx = 0; idx < length; idx++)
    // {
    //     sensor_data.footer.checksum1 += buffer[idx];
    //     sensor_data.footer.checksum2 += sensor_data.footer.checksum1;
    // }
      uint32_t i;

      /* reset crc */
      CRC_CR = CRC_CR_RESET;

        /* calc in 8bit chunks */
        for (i = 0; i < (length & ~3); i += 4) {
          CRC_DR = (*(uint8_t *)(buffer + i)) |
                   (*(uint8_t *)(buffer + i + 1)) << 8 |
                   (*(uint8_t *)(buffer + i + 2)) << 16 |
                   (*(uint8_t *)(buffer + i + 3)) << 24;
        }


      /* remaining bytes */
      switch (length % 4) {
        case 1:
          CRC_DR = *(uint8_t *)(buffer + i);
          LED_TOGGLE(3);
          break;
        case 2:
          CRC_DR = (*(uint8_t *)(buffer + i)) |
                   (*(uint8_t *)(buffer + i + 1)) << 8;
                   LED_TOGGLE(4);
          break;
        case 3:
          CRC_DR = (*(uint8_t *)(buffer + i)) |
                   (*(uint8_t *)(buffer + i + 1)) << 8 |
                   (*(uint8_t *)(buffer + i + 2)) << 16;
                   LED_TOGGLE(5);
          break;
        default:
          break;
      }
      sensor_data.footer.crc32 = CRC_DR;
}




static void sensors_spi_link_trans_cb( struct spi_transaction *trans __attribute__ ((unused)) ) {
    sensors_spi_link_ready = TRUE;
    gpio_clear(GPIO_BANK_UART5_TX, GPIO_UART5_TX);
}
