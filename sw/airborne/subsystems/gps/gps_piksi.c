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

#include "subsystems/gps.h"
#include "led.h"
#include "subsystems/gps/gps_piksi.h"

#if GPS_USE_LATLONG
/* currently needed to get nav_utm_zone0 */
#include "subsystems/navigation/common_nav.h"
#include "math/pprz_geodetic_float.h"
#endif

struct GpsPiksi gps_piksi;
struct GpsPiksiData gps_piksi_data;
uint8_t* header_buf = (uint8_t*)&gps_piksi.msg_header;
uint8_t* crc_buf = (uint8_t*)&gps_piksi.crc;



/* parser status */
#define FIND_STARTBYTE 1
#define GET_HEADER 2
#define GET_MSG 3
#define GET_CRC 4




/* CRC16 implementation acording to CCITT standards */
static const uint16_t crc16tab[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};


static inline uint16_t bswap16(uint16_t a) {
    return (a<<8)|(a>>8);
}


// distance in cm (10km dist max any direction)
#define MAX_DISTANCE  1000000

static int distance_too_great(struct EcefCoor_i *ecef_ref, struct EcefCoor_i *ecef_pos);
static uint16_t crc16_ccitt(const uint8_t *buf, uint32_t len, uint16_t crc);

/** Initialize variables of PIKSI module.
 *
 * Reset error counters and start reading data
 */
void gps_impl_init(void) {

    LED_ON(3);
    gps_piksi.error_cnt = 0;
    gps_piksi.error_last = GPS_PIKSI_ERR_NONE;
    gps_piksi.status = FIND_STARTBYTE;

}

/** Parses received data in to actual message
 *
 * After message was received, the message is read
 * and the global gps variable is set accordingly to the
 * data of the message.
 *
 */
void gps_piksi_read_message(void) {

    LED_TOGGLE(3);
    switch (gps_piksi.msg_header.msg_id)
    {
    case PIKSI_MESSAGE_ID_GPS_TIME:
    {
        memcpy(&gps_piksi_data.gps_time, gps_piksi.msg_buf, sizeof(sbp_gps_time_t));
        gps_time_sync.t0_ticks      = sys_time.nb_tick;
        gps_time_sync.t0_tow        = gps_piksi_data.gps_time.tow;
        gps_time_sync.t0_tow_frac = gps_piksi_data.gps_time.ns;
        gps.week = (int16_t) gps_piksi_data.gps_time.wn;
        gps.tow = gps_piksi_data.gps_time.tow;
        break;
    }
    case PIKSI_MESSAGE_ID_DOPS:
    {
        memcpy(&gps_piksi_data.dops, gps_piksi.msg_buf, sizeof(sbp_dops_t));
        gps.pdop = gps_piksi_data.dops.pdop;
        break;
    }
    case PIKSI_MESSAGE_ID_BASELINE_NED:
    {
        memcpy(&gps_piksi_data.baseline_ned, gps_piksi.msg_buf, sizeof(sbp_baseline_ned_t));
        break;
    }
    case PIKSI_MESSAGE_ID_BASELINE_ECEF:
    {
        memcpy(&gps_piksi_data.baseline_ecef, gps_piksi.msg_buf, sizeof(sbp_baseline_ecef_t));
        break;
    }
    case PIKSI_MESSAGE_ID_POS_ECEF:
    {
        memcpy(&gps_piksi_data.pos_ecef, gps_piksi.msg_buf, sizeof(sbp_pos_ecef_t));
        gps.ecef_pos.x = (int32_t)(gps_piksi_data.pos_ecef.x* 1e2);
        gps.ecef_pos.y = (int32_t)(gps_piksi_data.pos_ecef.y* 1e2);
        gps.ecef_pos.z = (int32_t)(gps_piksi_data.pos_ecef.z* 1e2);
        gps.pacc = (uint32_t)(gps_piksi_data.pos_ecef.accuracy*1e-1);
        break;
    }
    case PIKSI_MESSAGE_ID_POS_LLH:
    {
        memcpy(&gps_piksi_data.pos_llh, gps_piksi.msg_buf, sizeof(sbp_pos_llh_t));
        gps.lla_pos.lat = (int32_t)(RadOfDeg(gps_piksi_data.pos_llh.lat)*1e7);
        gps.lla_pos.lon = (int32_t)(RadOfDeg(gps_piksi_data.pos_llh.lon)*1e7);
        gps.lla_pos.alt = (int32_t)(RadOfDeg(gps_piksi_data.pos_llh.height)*1e3);
        gps.pacc = (uint32_t)(gps_piksi_data.pos_llh.v_accuracy*1e-1);
        break;
    }
    case PIKSI_MESSAGE_ID_HEARTBEAT:
    {
        memcpy(&gps_piksi_data.heartbeat, gps_piksi.msg_buf, sizeof(sbp_heartbeat_t));
        break;
    }
    case PIKSI_MESSAGE_ID_STARTUP:
    {
        memcpy(&gps_piksi_data.startup, gps_piksi.msg_buf, sizeof(sbp_startup_t));
        break;
    }
    case PIKSI_MESSAGE_ID_VEL_ECEF:
    {
        memcpy(&gps_piksi_data.vel_ecef, gps_piksi.msg_buf, sizeof(sbp_vel_ecef_t));
        gps.ecef_vel.x = (int32_t)(gps_piksi_data.vel_ecef.x*1e2);
        gps.ecef_vel.y = (int32_t)(gps_piksi_data.vel_ecef.y*1e2);
        gps.ecef_vel.z = (int32_t)(gps_piksi_data.vel_ecef.z*1e2);
        gps.sacc = (uint32_t)(gps_piksi_data.vel_ecef.accuracy*1e-1);
        break;
    }
    case PIKSI_MESSAGE_ID_VEL_NED:
    {
        memcpy(&gps_piksi_data.vel_ned, gps_piksi.msg_buf, sizeof(sbp_vel_ned_t));
        gps.ned_vel.x = (int32_t)(gps_piksi_data.vel_ned.n*1e-1);
        gps.ned_vel.y = (int32_t)(gps_piksi_data.vel_ned.e*1e-1);
        gps.ned_vel.z = (int32_t)(gps_piksi_data.vel_ned.d*1e-1);
        gps.sacc = (uint32_t)(gps_piksi_data.vel_ned.h_accuracy*1e-1);
        break;
    }
    default:
        break;
    }



#if GPS_USE_LATLONG
    /* Computes from (lat, long) in the referenced UTM zone */
    struct LlaCoor_f lla_f;
    lla_f.lat = ((float) gps.lla_pos.lat) / 1e7;
    lla_f.lon = ((float) gps.lla_pos.lon) / 1e7;
    struct UtmCoor_f utm_f;
    utm_f.zone = nav_utm_zone0;
    /* convert to utm */
    utm_of_lla_f(&utm_f, &lla_f);
    /* copy results of utm conversion */
    gps.utm_pos.east = utm_f.east*100;
    gps.utm_pos.north = utm_f.north*100;
    gps.utm_pos.alt = gps.lla_pos.alt;
    gps.utm_pos.zone = nav_utm_zone0;
#endif
}

/** Receive characters from the UART port and parse them to SBP protocol
 *
 * The SBP protocol contains a 6-byte long message header.
 * After the header the actual message is transmitted.
 * At the end an CRC-value is transmitted which is compared
 * with the received data.
 *
 * CRC-values are equal MSG_AVAILABLE flag is set.
 */
void gps_piksi_parse(uint8_t c) {

    //Check if buffer is not full yet
    if (gps_piksi.msg_idx > GPS_PIKSI_BUFFER_SIZE-1)
    {
        gps_piksi.error_last = GPS_PIKSI_ERR_OVERRUN;
        goto error;
    }

    //Switch depending on status of receiving the message
    switch (gps_piksi.status) {
    case FIND_STARTBYTE:
        if (c == PIKSI_SBP_PREAMBLE) {
            gps_piksi.msg_idx = 0;
            gps_piksi.status = GET_HEADER;
        }
        break;
    case GET_HEADER:
        header_buf[gps_piksi.msg_idx] = c;
        gps_piksi.msg_idx++;
        if (gps_piksi.msg_idx == sizeof(sbp_header_t)) {
            if (gps_piksi.msg_header.msg_length > GPS_PIKSI_MAX_MSG_SIZE) {
                gps_piksi.error_last = GPS_PIKSI_ERR_MSG_TOO_LONG;
                goto error;
            }
            gps_piksi.status = GET_MSG;
            gps_piksi.msg_idx = 0;
        }
        break;
    case GET_MSG:
        gps_piksi.msg_buf[gps_piksi.msg_idx] = c;
        gps_piksi.msg_idx++;
        if (gps_piksi.msg_idx == gps_piksi.msg_header.msg_length)
        {
            gps_piksi.status = GET_CRC;
            gps_piksi.msg_idx = 0;
        }
        break;
    case GET_CRC:
        crc_buf[gps_piksi.msg_idx] = c;
        gps_piksi.msg_idx++;
        if (gps_piksi.msg_idx == sizeof(uint16_t)) {
            uint16_t crc = crc16_ccitt((uint8_t*)&(gps_piksi.msg_header.msg_id), sizeof(uint16_t), 0);
            crc = crc16_ccitt((uint8_t*)&(gps_piksi.msg_header.sender_id), sizeof(uint16_t), crc);
            crc = crc16_ccitt((uint8_t*)&(gps_piksi.msg_header.msg_length), sizeof(uint8_t), crc);
            crc = crc16_ccitt(gps_piksi.msg_buf, gps_piksi.msg_header.msg_length, crc);

            //If CRC values are equal set flag
            if (gps_piksi.crc == crc)
            {
                LED_TOGGLE(5);
                gps_piksi.msg_available = TRUE;
            }
            else
            {
                LED_TOGGLE(4);
                gps_piksi.error_last = GPS_PIKSI_ERR_CHECKSUM;
                goto error;
            }
            gps_piksi.status = FIND_STARTBYTE;
            gps_piksi.msg_idx = 0;
        }
        break;
    default:
        gps_piksi.status = FIND_STARTBYTE;
        gps_piksi.msg_idx = 0;
        break;
    }
    return;
error:
    gps_piksi.error_cnt++;
    gps_piksi.msg_idx = 0;
    gps_piksi.status =FIND_STARTBYTE;
    return;
}




/** Calculate CCITT 16-bit Cyclical Redundancy Check (CRC16).
 *
 * This implementation uses parameters used by XMODEM i.e. polynomial is:
 * \f[
 *   x^{16} + x^{12} + x^5 + 1
 * \f]
 * Mask 0x11021, not reversed, not XOR'd
 * (there are several slight variants on the CCITT CRC-16).
 *
 * \param buf Array of data to calculate CRC for
 * \param len Length of data array
 * \param crc Initial CRC value
 *
 * \return CRC16 value
 */
static uint16_t crc16_ccitt(const uint8_t *buf, uint32_t len, uint16_t crc)
{
    for (uint32_t i = 0; i < len; i++)
        crc = (crc << 8) ^ crc16tab[((crc >> 8) ^ *buf++) & 0x00FF];
    return crc;
}


//NOT USED: SKYTRAQ implementation -> to check for what it is necessary
//static int distance_too_great(struct EcefCoor_i *ecef_ref, struct EcefCoor_i *ecef_pos) {
//    int32_t xdiff = abs(ecef_ref->x - ecef_pos->x);
//    if (xdiff > MAX_DISTANCE) {
//        return TRUE;
//    }
//    int32_t ydiff = abs(ecef_ref->y - ecef_pos->y);
//    if (ydiff > MAX_DISTANCE) {
//        return TRUE;
//    }
//    int32_t zdiff = abs(ecef_ref->z - ecef_pos->z);
//    if (zdiff > MAX_DISTANCE) {
//        return TRUE;
//    }

//    return FALSE;
//}
