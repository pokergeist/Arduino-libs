/*!
 *  @file EMA.h
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
 *  20220516  John Jordan - Periods and samples are now unsigned ints.
 *  20220512  John Jordan - Added accessors for number of periods.
 *  20190828  John Jordan - Original.
 */

#ifndef EMA_H
#define EMA_H

#define U_INT unsigned int    // use native unsigned int

class EMA {

 public:

  // c'tor - ema periods and number of samples to average to initialize ema
  EMA (U_INT n_periods, U_INT samples_to_avg);

  // update the ema (or average) value with a new sample
  float update (float sample);

  // get the current ema value
  float value (void);

  // override the current ema value or averaging results,
  // returns the old value to the caller
  // terminates averaging
  float value (float new_value);

  // test to see if still averaging samples
  bool in_ema_mode (void);

  // get number of periods
  U_INT getPeriods(void);

  // set number of periods; return old value
  U_INT setPeriods (U_INT n_periods);

 protected:

  U_INT  samples_to_average;   // samples to average
  U_INT  averaged_samples = 0; // how many so far
  bool averaging_done = false;  // done with averaging by count or override
  float k, k2, ema_value;       // constants and our progressive ema value
  U_INT  periods;              // number of periods EMA is calculated over
}; // class EMA

#endif /* _H */
