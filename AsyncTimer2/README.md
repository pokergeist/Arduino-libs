# AsyncTimer2

This is a clone of Luis Llamas' [AsyncTimerLib](https://github.com/luisllamasbinaburo/Arduino-AsyncTimer).

## Intro

This library lets you configure timers and callbacks with an optional parameter. Each time through loop() check your timers. There are a number of good features and options. AsyncTimerLib may be documented better. I use it for concurrent I2C and Serial message processing, blinking LEDs, mode switch timing, and sensor monitoring in one project alone. Handy stuff.

I tinkered with the constructors and default parameters to suit me, and probably renamed a few things - it's been a while. i added and option to pass an integer back to the callback method.

## Usage

This example show how I utilize multiple timers. Some of my software modules manage their own timers and I call their startTimers() and checkTimers() methods.

```c++
#include <AsyncTimer2.h>

// define prototypes for timer callbacks
void scanConsole (void);
void reportTemps (void);
void checkGNSS   (void);
void blinkLED    (int); // pass LED pin number - change via setCookie()

// timers - distribute timewise
AsyncTimer2 timer_Scan (800e3,  TIMER_RESETS, scanConsole); // 800ms
AsyncTimer2 timer_RTMP (30e6,   TIMER_RESETS, reportTemps); // 30s
AsyncTimer2 timer_GNSS (200e3,  TIMER_RESETS, checkGNSS);   // 200ms - GNSS I2C messages (1/sec)
AsyncTimer2 timer_LED  (750e3, blinkLED, LED_BUILTIN, TIMER_RESETS); // 750ms

void setup(void) {
    startTimers());
}

void loop(void) {
    checkTimers();
}

void startTimers (void) {
  timer_Scan.Start();
  timer_GNSS.Start();
  timer_RTMP.Start();
  timer_LED.Start();
  powerMgr.startTimers(); // PowerManager class manages its own timers
}

void checkTimers (void) {
  timer_GNSS.Check();
  timer_Scan.Check();
  timer_RTMP.Check();
  timer_LED.Check();
  powerMgr.checkTimers(); // PowerManager class manages its own timers
}

// callbacks
void scanConsole(void) { ... }
void reportTemps(void) { ... }
void checkGNSS (void) { ... }
void blinkLED(int pinLED) { ... }
```

## Methods

```C++
// callback definitions
typedef void(*AsyncTimerCallback)();
typedef void(*AsyncTimer2Callback)(int);

// constructors
/**************************************************************************
 * microsInterval - timer interval in microseconds
 * onFinish - AsyncTimerCallback callback method with no parameter
 *          - AsyncTimer2Callback - callback method that accepts a cookie
 * cookie - integer parameter returned to AsyncTimer2Callback
 * autoReset - restarts the timer after it expires in the Check() call.
***************************************************************************/
// Note: The parameters are a little mixed up so that unique signatures are
//       created given all of the default parameters.
AsyncTimer2(unsigned long microsInterval, AsyncTimerCallback onFinish, bool autoReset=false);
AsyncTimer2(unsigned long microsInterval, bool autoReset=false, AsyncTimerCallback onFinish=nullptr);
// accepts cookie to pass to callback method
AsyncTimer2(unsigned long microsInterval, AsyncTimer2Callback onFinish2, int cookie=0, bool autoReset=false);

void Start();   // sets active and clears expired
void ConditionalStart(); // start if not already active
void Reset();   // resets start timer but not active and expired states
void Stop();    // sets not active
bool Check();   // checks for expiry
bool CheckAndSwitch(AsyncTimer2 &next);  // check current timer, start next timer if current timer is inactive
                                         // returns status of timer switch
// settimer intervals
void SetIntervalMillis(unsigned long interval);
void SetIntervalMicros(unsigned long interval);

// get timer metrics
unsigned long GetStartTime();
unsigned long GetElapsedTime();
unsigned long GetRemainingTime();

// cookie accessors
void SetCookie(int cookie);
int GetCookie(void);

// get timer mode
bool IsActive() const;  // true as long as started and not stopped or expired w/o reset
bool IsExpired() const; // true once expiry detected until reset or start

// start timer in auto-reset mode
void Every(unsigned long millisInterval, AsyncTimerCallback onFinish);
// start timer in one-shot mode
void In(unsigned long millisInterval, AsyncTimerCallback onFinish);

```

