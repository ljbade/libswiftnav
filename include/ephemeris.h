/*
 * Copyright (C) 2010 Swift Navigation Inc.
 * Contact: Henry Hallam <henry@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_EPHEMERIS_H
#define LIBSWIFTNAV_EPHEMERIS_H

#include "gpstime.h"
#include "common.h"

typedef struct {
  double tgd;
  double crs, crc, cuc, cus, cic, cis;
  double dn, m0, ecc, sqrta, omega0, omegadot, w, inc, inc_dot;
  double af0, af1, af2;
  double ura, aodo;
  double fit_interval, age;
  gps_time_t toe, toc, start_time;
  u16 iodc_1;
  u8 iode_2, iode_3;
  u8 valid, stale, upload;
  u8 healthy, sv_health;
  u8 ca_or_p_on_l2;
  u8 ura_index;
  u8 l2_p_data_flag;
  u8 fit_interval_flag;
} ephemeris_t;

int calc_sat_pos(double pos[3], double vel[3],
                 double *clock_err, double *clock_rate_err,
                 ephemeris_t *ephemeris,
                 gps_time_t tot);

double predict_range(double rx_pos[3],
                     gps_time_t tot,
                     ephemeris_t *ephemeris);

void update_eph_age(ephemeris_t *ephemeris,
                    gps_time_t tot);

#endif /* LIBSWIFTNAV_EPHEMERIS_H */

