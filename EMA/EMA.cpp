/*!
 *  @file EMA.cpp
 *
 *  @mainpage Exponential Moving Average (EMA) Library
 *
 *  @section intro_sec Introduction
 *
 * 	Library class for Exponential Moving Averages (EMA).
 *  EMAs give more weighting to the most recent samples.
 *
 *  @section dependencies Dependencies
 *
 *  None
 *
 *  @section author Author
 *
 *  John Jordan
 *
 * 	@section license License
 *
 * 	MIT (see license.txt)
 *
 * 	@section  HISTORY
 *
 *  20220512  John Jordan - Added accessors for number of periods.
 *  20190828  John Jordan - Original.
 */

#include "EMA.h"

/**************************************************************************/
/*!
    @brief  Instantiates a new MPU6050 class
    @param  n_periods
            number of periods used in EMA calculations
    @param  samples_to_avg
            number of samples to be averaged to set initial EMA value
 */
/**************************************************************************/
EMA::EMA (int n_periods, int samples_to_avg) {
  setPeriods(n_periods);
  samples_to_average = samples_to_avg;
}

/**************************************************************************/
/*!
    @brief  Update the EMA (or average) value with a new sample
    @param  sample
            newest value to be averaged
    @returns New EMA
*/
/**************************************************************************/
float EMA::update (float sample) {
  if (averaging_done) {
    ema_value = k * sample + k2 * ema_value;
  } else {
    // recover sum, add new value ; save new average
    ema_value  = (ema_value * averaged_samples) + sample;
    ema_value /= ++averaged_samples;
    averaging_done = (averaged_samples >= samples_to_average);
  }
  return ema_value;
}

/**************************************************************************/
/*!
    @brief  Returns the current EMA value
    @returns the EMA value
*/
/**************************************************************************/
float EMA::value (void) { return ema_value; }

/**************************************************************************/
/*!
    @brief  Override the current EMA value or averaging results.
            Terminates averaging.
    @param  new_value
            the new EMA value
    @returns the previous EMA value
*/
/**************************************************************************/
float EMA::value (float new_value) {
  float old_value = ema_value;
  ema_value = new_value;
  averaging_done = true;
  return old_value;
}

/**************************************************************************/
/*!
    @brief  Test to see if still averaging samples
    @returns True if in EMA mode,
             False if still averaging for initial value.
*/
/**************************************************************************/
bool EMA::in_ema_mode (void) { return averaging_done; }

/**************************************************************************/
/*!
    @brief  Get the number of periods used in EMA calculation
    @returns the number of periods used in EMA calculation
*/
/**************************************************************************/
int EMA::getPeriods(void) { return periods; }

/**************************************************************************/
/*!
    @brief  Sets the number of periods to be used for EMA calculations
    @param  n_periods
            the number of periods used in future EMA calculations
    @returns the previous number of periods used
*/
/**************************************************************************/
int EMA::setPeriods (int n_periods) {
  int old_p = periods;
  periods = n_periods;
  k  = 2.0/(periods + 1);
  k2 = 1.0 - k;
  return old_p;
}
