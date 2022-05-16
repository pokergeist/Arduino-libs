/****************************************************************************
 * Plot EMA values demo.
 *
 * 2022-05-16  John Jordan
 ****************************************************************************/

#include <EMA.h>

#define DONT_CARE 5

EMA slow_ema(100, DONT_CARE); // samples_to_average don't matter - we set init value later
EMA fast_ema(15,  DONT_CARE);
EMA avg(DONT_CARE, 360);      // create an averaging function - ema_periods don't matter,
                              // we're averaging the entire time (360 plot points)

void setup (void) {
  Serial.begin(9600);
  delay(4e3); // give time to launch plotter Ctrl-Shift-L

  // for this plotting example, don't init EMAs
  // by averaging, just set to 0
  slow_ema.value(0);
  fast_ema.value(0);
}

// plot two sine wave cycles, scaled to a +/-5 chart

void loop (void) {
  float slow, fast, average, y;
  for (int i=0 ; i<=720 ; i+=2) {
    y = sin(PI*i/180.0); // convert arg to radians
    // calcutate exponential and simple moving averages
    slow = slow_ema.update(y);
    fast = fast_ema.update(y);
    average = avg.update(y);
    // plot values, place values in plot legend, scale graph
    Serial.print("y=");
    Serial.print(y);
    Serial.print(":");
    Serial.print(4.0*y);

    Serial.print(",slow=");
    Serial.print(slow);
    Serial.print(":");
    Serial.print(4.0*slow);

    Serial.print(",fast=");
    Serial.print(fast);
    Serial.print(":");
    Serial.print(4.0*fast);

    Serial.print(",avg=");
    Serial.print(average);
    Serial.print(":");
    Serial.print(4.0*average);
    Serial.println();
    delay(25);
  }
  while (1) delay(10); // wait for upload with new EMA periods :-)
}
