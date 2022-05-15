# Arduino EMA Library

Calculate Exponential Moving Averages.

### Overview

This library calculates exponential moving averages (EMA) where the value is biased toward the most recent sample as opposed to simple moving averages (SMA). The more periods used by the EMA, the less influence the last sample has on the EMA value. EMAs are commonly used in investing but could be used for example to display a channel bitrate where the value is smoothed, but converges more closely to the most recent sample. See Investopedia for more info.

The constructor accepts two values, the number of periods to use in forming constants for its calculations. The second is the number of samples to average to set the initial value of the EMA. This value depends on your data and how eager you are to get into EMA mode. You can also initialize the EMA value at any time and terminate the averaging phase.

### Usage:

```c++
#include <EMA.h>

EMA bitrate_ema(4, 2);                       // construct a fast EMA object

void loop (void) {
    float bitrate;
    ....
    bitrate = calculateBitrate();                // get the most recent bitrate
    displayBitrate(bitrate_ema.update(bitrate)); // get the EMA for display purposes
    ....
}
```

### Methods

```c++
/*************************************************************
 * periods: Sets number of periods used in EMA calculations.
 * samples_to_average: Sets the number of samples to average
 * to set the initial EMA value.
 *************************************************************/
EMA (int periods, int samples_to_average);

float update(float sample);     // update the exp. moving average (or simple average
                                //   if still initializing) and return the value
float value(void);              // returns the EMA value again (no update)
float value(float new_value);   // initializes the EMA value and turns off the
                                //   simple averaging initialization phase
bool in_ema_mode(void);         // true if simple averaging initialization is done

int getPeriods(void);		// get the number of EMA periods

int setPeriods(int n_periods);	// set the number of periods to be used in future EMA
				//   calculations. Returns the previous n_periods value.
```

### Run-time Construction

If you want to create your object containing an EMA component using a configurable "periods" attribute, you can use a variable initialization list.

```c++
class Foo {
  Foo (int ema_periods);
  EMA  my_EMA;    // no c'tor args
};

Foo::Foo(int ema_periods) : my_EMA(ema_periods, ema_periods) { ... }
```
### Changing EMA periods

You use the setPeriods() method to change the number of EMA periods as needed.

```c++
EMA myEMA(50, 50);

  // faster response needed now (less damping)
  myEMA.setPeriods(10);
```
