/*
 * Copyright (C) 2012 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <math.h>

#include <libswiftnav/troposphere.h>
#include <libswiftnav/common.h>
#include <libswiftnav/constants.h>

/** \defgroup troposphere Tropospheric models
 * Implemenations of tropospheric delay correction models.
 * \{ */

/** Average barometric pressure lookup table [mbar] */
static const double p_avg_lut[5] =
{
  1013.25,
  1017.25,
  1015.75,
  1011.75,
  1013.00
};

/** Average temperature lookup table [K] */
static const double t_avg_lut[5] =
{
  299.65,
  294.15,
  283.15,
  272.15,
  263.65
};

/** Average water vapour pressure lookup table [mbar] */
static const double e_avg_lut[5] =
{
  26.31,
  21.79,
  11.66,
  6.78,
  4.11
};

/** Average temperature lapse rate lookup table [K/m] */
static const double b_avg_lut[5] =
{
  6.30e-3,
  6.05e-3,
  5.58e-3,
  5.39e-3,
  4.53e-3
};

/** Average water vapour pressure height factor lookup table */
static const double l_avg_lut[5] =
{
  2.77,
  3.15,
  2.57,
  1.81,
  1.55
};

/** Average relative humidity lookup table */
static const double rh_avg_lut[5] =
{
  75.0,
  80.0,
  76.0,
  77.5,
  82.5
};

/** Amplitude barometric pressure lookup table [mbar] */
static const double p_amp_lut[5] =
{
   0.00,
  -3.75,
  -2.25,
  -1.75,
  -0.50
};

/** Amplitude temperature lookup table [K] */
static const double t_amp_lut[5] =
{
  0.00,
  7.00,
  11.00,
  15.00,
  14.50
};

/** Amplitude water vapour pressure lookup table [mbar] */
static const double e_amp_lut[5] =
{
  0.00,
  8.85,
  7.24,
  5.36,
  3.39
};

/** Amplitude temperature lapse rate lookup table [K/m] */
static const double b_amp_lut[5] =
{
  0.00,
  0.25e-3,
  0.32e-3,
  0.81e-3,
  0.62e-3
};

/** Amplitude water vapour pressure height factor lookup table */
static const double l_amp_lut[5] =
{
  0.00,
  0.33,
  0.46,
  0.74,
  0.30
};

/** Amplitude relative humidity lookup table */
static const double rh_amp_lut[5] =
{
  0.0,
  0.0,
  -1.0,
  -2.5,
  2.5
};

/** Compute tropo values by interpolation from look up table */
static double lookup_param(double lat, const double *lut)
{
  /* Handle southern hemisphere where latitude is negative */
  lat = fabs(lat);

  /* Below/above +/- 15 degrees latitude */
  if (lat <= 15.0) {
    return lut[0];
  /* Below/above +/- 75 degrees latitude */
  } else if (lat >= 75.0) {
    return lut[4];
  } else {
    /* Otherwise interpolate the value */
    u8 i = (lat - 15.0) / 15.0;
    double lat_i = i * 15.0 + 15.0;
    return lut[i] + (lut[i + 1] - lut[i]) / 15.0 * (lat - lat_i);
  }
}

static double calc_param(double lat, double doy, const double *avg_lut, const double *amp_lut)
{
  /* Compute average surface tropo values by interpolation */
  double avg = lookup_param(lat, avg_lut);

  /* Compute variation of average surface tropo values */
  double amp = lookup_param(lat, amp_lut);

// TODO check rad deg
// TODO check frac vs int doy
// TODO fortran has southern hemi adjustment here?

  /* Deal with southern hemisphere and yearly variation */
  if (lat < 0) {
    doy += 182.625;
  }

  const double doy_2_rad = 2.0 * M_PI / 365.25;
  return avg - amp * cos((doy - 28.0) * doy_2_rad);
}

/** Calculate tropospheric delay using Klobuchar model.
 *
 * References:
 *   -#
 *
 * \param doy Day of the year at which to calculate tropospheric delay [day]
 * \param lat Latitude of the receiver [rad]
 * \param lat Height of the receiver [m]
 * \param el Elevation of the satellite [rad]
 *
 * \return Tropospheric delay distance [m]
 */
double calc_troposphere(double doy, double lat, double h, double el,
                        double *t_out, double *p_out, double *e_out,
                        double *t_m_out)
{// TODO is height in MSL or WGS84 (seems orthometric height = MSL)
  // TODO lat to deg
  lat *= R2D;
  el *= R2D;
  // TODO calc day of year -> need UTC / date conversions
  // h = orthometric height in m

  // TODO take gps time struct and get day of year

  /* Compute surface tropo values */
  double p_0 = calc_param(lat, doy, p_avg_lut, p_amp_lut);
  double t_0 = calc_param(lat, doy, t_avg_lut, t_amp_lut);
  //double e_0 = calc_param(lat, doy, e_avg_lut, e_amp_lut);
  double b = calc_param(lat, doy, b_avg_lut, b_amp_lut) / 1000.0;
  double l = calc_param(lat, doy, l_avg_lut, l_amp_lut);
  double rh = calc_param(lat, doy, rh_avg_lut, rh_amp_lut);

  /* Compute the saturation vapour pressure */
  double e_s = 0.01 * exp(1.2378847e-5 * t_0 * t_0 - 1.9121316e-2 * t_0 + 33.93711047 - 6.3431645e3 / t_0);

  /* Compute the enhancement factor */
  double x = t_0 - 273.15;
  double f_w = 1.00062 + 3.14e-6 * p_0 + 5.6e-7 * x * x;

  /* Compute the water vapour pressure */
  double e_0 = rh / 100.0 * e_s * f_w;

  /* Compute the gas constant for dry air */
  const double r = 8314.34;
  const double m_d = 28.9644;
  const double r_d = r / m_d;

  /* Compute the refractivity constants */
  const double k_1 = 77.60;
  const double k_2 = 64.79;
  const double m_w = 18.0152;
  const double k_2_prim = k_2 - k_1 * (m_w * m_d);
  const double k_3 = 377600.0;
  const double c_1 = 2.2768e-3;

  /* Compute power value for pressure & water vapour */
  double e_p = 9.80665 / 287.054 / b;

  /* Scale surface values to required height */
  double t = t_0 - b * h;
  double p = p_0 * powf(t / t_0, e_p);
  double dl = l + 1.0;
  double e = e_0 * powf(t / t_0, e_p * dl); // TODO e is wrong

  /* Compute the acceleration at the mass center
     of a vertical column of the atmosphere */
  // TODO GEOLAT? what is this for?
  const double excen2 = 6.6943799901413e3; // ?
  double geo_lat = atan((1.0 - excen2) * tan(lat * D2R));
  double d_g_ref = 1.0 - 2.66e-3 * cos(2.0 * geo_lat) - 2.8e-7 * h;
  double g_m = 9.784 * d_g_ref;
  double den = g_m * dl;

  /* Compute mean temperature of the water vapor */
  double t_m = t * (1.0 - b * r_d / den);

  /* Compute zenith hydrostatic delay */
  double zhd = c_1 / d_g_ref * p;

  /* Compute zenith wet delay */
  double zwd = 10e-6 * (k_2_prim + k_3 / t_m) * r_d * e / den;

  double mhf = 1.0 + el; // TODO
  double mwf = 1.0;

  /* Compute total tropospheric delay */
  double td = mhf * zhd + mwf * zwd;

  *t_out = t;
  *p_out = p;
  *e_out = e;
  *t_m_out = t_m;
  return td;
}

/** \} */
