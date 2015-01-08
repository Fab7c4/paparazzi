/*
 * Copyright (C) 2010 Antoine Drouin <poinix@gmail.com>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef GPS_PIKSI_HIGHWIND_H
#define GPS_PIKSI_HIGHWIND_H

#include "mcu_periph/uart.h"

#include "subsystems/gps/gps_piksi_messages.h"

//SwiftNav Binary Protocol
#define PIKSI_SBP_PREAMBLE 0x55
#define PIKSI_MESSAGE_ID_STARTUP 0xFF00  //4 System start-up message
#define PIKSI_MESSAGE_ID_HEARTBEAT 0xFFFF //4 System heartbeat message
#define PIKSI_MESSAGE_ID_GPS_TIME  0x0100 //11 GPS Time
#define PIKSI_MESSAGE_ID_DOPS 0x0206 //14 Dilution of Precision
#define PIKSI_MESSAGE_ID_POS_ECEF 0x0200 //32 Position in ECEF
#define PIKSI_MESSAGE_ID_POS_LLH 0x0201 //34 Geodetic Position
#define PIKSI_MESSAGE_ID_BASELINE_ECEF 0x0202 //20 Baseline in ECEF
#define PIKSI_MESSAGE_ID_BASELINE_NED 0x0203 //22 Baseline in NED
#define PIKSI_MESSAGE_ID_VEL_ECEF 0x0204 //20 Velocity in ECEF
#define PIKSI_MESSAGE_ID_VEL_NED  0x0205 //22 Velocity in NED 0XA8

/* last error type */
enum GpsPiksiError {
  GPS_PIKSI_ERR_NONE = 0,
  GPS_PIKSI_ERR_OVERRUN = -1,
  GPS_PIKSI_ERR_MSG_TOO_LONG = -2,
  GPS_PIKSI_ERR_CHECKSUM = -3,
  GPS_PIKSI_ERR_OUT_OF_SYNC = -4,
  GPS_PIKSI_ERR_UNEXPECTED = -5
};

#define GPS_PIKSI_MAX_MSG_SIZE 64
#define GPS_PIKSI_BUFFER_SIZE 256
struct GpsPiksi {
  sbp_header_t msg_header;
  uint8_t msg_buf[GPS_PIKSI_BUFFER_SIZE];
  bool_t  msg_available;

  uint8_t status;
  uint16_t msg_idx;
  uint16_t crc;
  uint8_t error_cnt;
  enum GpsPiksiError error_last;
};


struct GpsPiksiData {
    sbp_startup_t startup;
    sbp_baseline_ecef_t baseline_ecef;
    sbp_baseline_ned_t baseline_ned;
    sbp_dops_t dops;
    sbp_gps_time_t gps_time;
    sbp_heartbeat_t heartbeat;
    sbp_pos_ecef_t pos_ecef;
    sbp_pos_llh_t pos_llh;
    sbp_vel_ecef_t vel_ecef;
    sbp_vel_ned_t vel_ned;
};

extern struct GpsPiksi gps_piksi;
extern struct GpsPiksiData gps_piksi_data;


/*
 * This part is used by the autopilot to read data from a uart
 */
#define __GpsLink(dev, _x) dev##_x
#define _GpsLink(dev, _x)  __GpsLink(dev, _x)
#define GpsLink(_x) _GpsLink(GPS_LINK, _x)

#define GpsBuffer() GpsLink(ChAvailable())

#define GpsEvent(_sol_available_callback) {                     \
    if (GpsBuffer()) {                                          \
      ReadGpsBuffer();                                          \
    }                                                           \
    if (gps_piksi.msg_available) {                            \
      gps_piksi_read_message();                               \
      if (gps_piksi.msg_header.msg_id == PIKSI_MESSAGE_ID_POS_ECEF) {	\
        if (gps.fix == GPS_FIX_3D) {                            \
          gps.last_fix_ticks = sys_time.nb_sec_rem;             \
          gps.last_fix_time = sys_time.nb_sec;                  \
        }                                                       \
        _sol_available_callback();                              \
      }                                                         \
      gps_piksi.msg_available = FALSE;                        \
    }                                                           \
  }

#define ReadGpsBuffer() {						\
    while (GpsLink(ChAvailable())&&!gps_piksi.msg_available)	\
      gps_piksi_parse(GpsLink(Getch()));				\
  }


extern void gps_piksi_read_message(void);
extern void gps_piksi_parse(uint8_t c);

#endif /* GPS_PIKSI_H */
