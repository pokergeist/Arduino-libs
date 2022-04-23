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
EMA (int periods, int samples_to_average);

float update(float sample);     // update the exp. moving average (or simple average
                                //   if still initializing) and return the value
float value(void);              // returns the EMA value again (no update)
float value(float new_value);   // initializes the EMA value and turns off the
                                //   simple averaging initialization phase
bool in_ema_mode(void);         // true if simple averaging initialization is done
```
