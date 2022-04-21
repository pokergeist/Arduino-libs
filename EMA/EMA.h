/****************************************************************************
 * Library class for Exponential Moving Averages (EMA).
 *
 * EMAs give more weighting to the most recent samples.
 *
 * 20190828  John Jordan - Original.
 ****************************************************************************/
 
#ifndef EMA_H
#define EMA_H

class EMA {

 public:

  // c'tor - ema periods and number of samples to average to initialize ema
  EMA (int periods, int samples_to_avg) {
    k  = 2.0/(periods + 1);
    k2 = 1.0 - k;
    samples_to_average = samples_to_avg;
  }

  // update the ema (or average) value with a new sample
  float update (float sample) {
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

  // get the current ema value
  float value (void) { return ema_value; }

  // override the current ema value or averaging results,
  // returns the old value to the caller
  // terminates averaging
  float value (float new_value) {
    float old_value = ema_value;
    ema_value = new_value;
    averaging_done = true;
    return old_value;
  }

  // test to see if still averaging samples
  bool in_ema_mode (void) { return averaging_done; }

 protected:
 
  int  samples_to_average;      // samples to average
  int  averaged_samples = 0;    // how many so far
  bool averaging_done = false;  // done with averaging by count or override
  float k, k2, ema_value;       // constants and our progressive ema value
};

#endif
