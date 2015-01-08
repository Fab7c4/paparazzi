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
 *
 *
 * CODE is copied from https://github.com/swift-nav/libswiftnav
 * Copyright (C) 2015 SwiftNAV
 */

#ifndef GPS_PIKSI_MESSAGES_H
#define GPS_PIKSI_MESSAGES_H

/** Signed 8-bit integer. */
typedef int8_t s8;
/** Signed 16-bit integer. */
typedef int16_t s16;
/** Signed 32-bit integer. */
typedef int32_t s32;
/** Signed 64-bit integer. */
typedef int64_t s64;
/** Unsigned 8-bit integer. */
typedef uint8_t u8;
/** Unsigned 16-bit integer. */
typedef uint16_t u16;
/** Unsigned 32-bit integer. */
typedef uint32_t u32;
/** Unsigned 64-bit integer. */
typedef uint64_t u64;


/** Header of SBP protocol
 * Every message starts with this header after receiving the Startbyte 0x55
 */
typedef struct __attribute__((packed)) {
  u16 msg_id;       /**< ID of message send by PIKSI */
  u16 sender_id;    /**< Last two byte of serial number of PIKSI */
  u8 msg_length;    /**< Length of actual message */
} sbp_header_t;




/** System start-up message
 * The system start-up message is sent once on system start-up. It is
 * intended to be used to notify the host or other attached devices that
 * the system has started and is now ready to respond to commands or
 * configuration requests.
 */
#define SBP_STARTUP 0xFF00

typedef struct __attribute__((packed)) {
  u32 reserved; /**< Reserved */
} sbp_startup_t;


/** System heartbeat message
 * The heartbeat message is sent periodically to inform the host or
 * other attached devices that the system is running. It is intended to
 * be used to monitor for system malfunctions and also contains
 * status flags that indicate to the host the status of the system and
 * if it is operating correctly.
 *
 * The system error flag is used to indicate that an error has occurred in
 * the system. To determine the source of the error the remaining error
 * flags should be inspected.
 */
#define SBP_HEARTBEAT 0xFFFF

typedef struct __attribute__((packed)) {
  u32 flags; /**< Status flags */
} sbp_heartbeat_t;


/** GPS Time
 * GPS Time.
 */
#define SBP_GPS_TIME 0x0100

typedef struct __attribute__((packed)) {
  u16 wn;    /**< GPS week number [weeks] */
  u32 tow;   /**< GPS Time of Week rounded to the nearest ms [ms] */
  s32 ns;    /**< Nanosecond remainder of rounded tow [ns] */
  u8  flags; /**< Status flags (reserved) */
} sbp_gps_time_t;


/** Dilution of Precision
 * Dilution of Precision.
 */
#define SBP_DOPS 0x0206

typedef struct __attribute__((packed)) {
  u32 tow;  /**< GPS Time of Week [ms] */
  u16 gdop; /**< Geometric Dilution of Precision [0.01] */
  u16 pdop; /**< Position Dilution of Precision [0.01] */
  u16 tdop; /**< Time Dilution of Precision [0.01] */
  u16 hdop; /**< Horizontal Dilution of Precision [0.01] */
  u16 vdop; /**< Vertical Dilution of Precision [0.01] */
} sbp_dops_t;


/** Position in ECEF
 * Position solution in absolute Earth Centered Earth Fixed (ECEF) coordinates.
 */
#define SBP_POS_ECEF 0x0200

typedef struct __attribute__((packed)) {
  u32    tow;      /**< GPS Time of Week [ms] */
  double x;        /**< ECEF X coordinate [m] */
  double y;        /**< ECEF Y coordinate [m] */
  double z;        /**< ECEF Z coordinate [m] */
  u16    accuracy; /**< Position accuracy estimate [mm] */
  u8     n_sats;   /**< Number of satellites used in solution */
  u8     flags;    /**< Status flags */
} sbp_pos_ecef_t;


/** Geodetic Position
 * Geodetic position solution.
 */
#define SBP_POS_LLH 0x0201

typedef struct __attribute__((packed)) {
  u32    tow;        /**< GPS Time of Week [ms] */
  double lat;        /**< Latitude [deg] */
  double lon;        /**< Longitude [deg] */
  double height;     /**< Height [m] */
  u16    h_accuracy; /**< Horizontal position accuracy estimate [mm] */
  u16    v_accuracy; /**< Vertical position accuracy estimate [mm] */
  u8     n_sats;     /**< Number of satellites used in solution */
  u8     flags;      /**< Status flags */
} sbp_pos_llh_t;


/** Baseline in ECEF
 * Baseline in Earth Centered Earth Fixed (ECEF) coordinates.
 */
#define SBP_BASELINE_ECEF 0x0202

typedef struct __attribute__((packed)) {
  u32 tow;      /**< GPS Time of Week [ms] */
  s32 x;        /**< Baseline ECEF X coordinate [mm] */
  s32 y;        /**< Baseline ECEF Y coordinate [mm] */
  s32 z;        /**< Baseline ECEF Z coordinate [mm] */
  u16 accuracy; /**< Position accuracy estimate [mm] */
  u8  n_sats;   /**< Number of satellites used in solution */
  u8  flags;    /**< Status flags */
} sbp_baseline_ecef_t;


/** Baseline in NED
 * Baseline in local North East Down (NED) coordinates.
 */
#define SBP_BASELINE_NED 0x0203

typedef struct __attribute__((packed)) {
  u32 tow;        /**< GPS Time of Week [ms] */
  s32 n;          /**< Baseline North coordinate [mm] */
  s32 e;          /**< Baseline East coordinate [mm] */
  s32 d;          /**< Baseline Down coordinate [mm] */
  u16 h_accuracy; /**< Horizontal position accuracy estimate [mm] */
  u16 v_accuracy; /**< Vertical position accuracy estimate [mm] */
  u8  n_sats;     /**< Number of satellites used in solution */
  u8  flags;      /**< Status flags */
} sbp_baseline_ned_t;


/** Velocity in ECEF
 * Velocity in Earth Centered Earth Fixed (ECEF) coordinates.
 */
#define SBP_VEL_ECEF 0x0204

typedef struct __attribute__((packed)) {
  u32 tow;      /**< GPS Time of Week [ms] */
  s32 x;        /**< Velocity ECEF X coordinate [mm/s] */
  s32 y;        /**< Velocity ECEF Y coordinate [mm/s] */
  s32 z;        /**< Velocity ECEF Z coordinate [mm/s] */
  u16 accuracy; /**< Velocity accuracy estimate [mm/s] */
  u8  n_sats;   /**< Number of satellites used in solution */
  u8  flags;    /**< Status flags (reserved) */
} sbp_vel_ecef_t;


/** Velocity in NED
 * Velocity in local North East Down (NED) coordinates.
 */
#define SBP_VEL_NED 0x0205

typedef struct __attribute__((packed)) {
  u32 tow;        /**< GPS Time of Week [ms] */
  s32 n;          /**< Velocity North coordinate [mm/s] */
  s32 e;          /**< Velocity East coordinate [mm/s] */
  s32 d;          /**< Velocity Down coordinate [mm/s] */
  u16 h_accuracy; /**< Horizontal velocity accuracy estimate [mm/s] */
  u16 v_accuracy; /**< Vertical velocity accuracy estimate [mm/s] */
  u8  n_sats;     /**< Number of satellites used in solution */
  u8  flags;      /**< Status flags (reserved) */
} sbp_vel_ned_t;
#endif // GPS_PIKSI_MESSAGES_H
