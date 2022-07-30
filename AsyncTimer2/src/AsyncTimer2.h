/***************************************************
Copyright (c) 2017 Luis Llamas
(www.luisllamas.es)

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License
 ****************************************************/

/******************************************************************************
 * 2020-12-31 John Jordan - Added callback with parameter.
 * 2019       John Jordan - Various mods to funcions, names, heavily
 *                          reformatting and commented.
 ******************************************************************************/

#ifndef _ASYNCTIMER2_H
#define _ASYNCTIMER2_H

#include "Arduino.h"

typedef void(*AsyncTimerCallback)();
typedef void(*AsyncTimer2Callback)(int);

// autoReset aliases
#define TIMER_RESETS    true
#define TIMER_ONE_SHOT  false

class AsyncTimer2 {
 public:
  AsyncTimer2(unsigned long microsInterval, AsyncTimerCallback onFinish, bool autoReset=false);
  AsyncTimer2(unsigned long microsInterval, bool autoReset=false, AsyncTimerCallback onFinish=nullptr);
  AsyncTimer2(unsigned long microsInterval, AsyncTimer2Callback onFinish2, int cookie=0, bool autoReset=false);

  // 32-bit micros() overflow (wrap) every 72.58 minutes. The unsigned integer
  // math still returns the correct exipry result.

  void Start();   // sets active and clears expired
  void ConditionalStart(); // start if not already active
  void Reset();   // resets start timer but not active and expired states
  void Stop();    // sets not active
  bool Check();   // checks for expiry
  bool CheckAndSwitch(AsyncTimer2 &next);  // check current timer, start next timer if current timer is inactive
                                          // returns status of timer switch

  void SetIntervalMillis(unsigned long interval);
  void SetIntervalMicros(unsigned long interval);
  void SetCookie(int cookie);

  unsigned long GetStartTime();
  unsigned long GetElapsedTime();
  unsigned long GetRemainingTime();
  int GetCookie(void);

  bool IsActive() const;  // true as long as started and not stopped or expired w/o reset
  bool IsExpired() const; // true once expiry detected until reset or start

  unsigned long Interval;
  bool AutoReset;         // no accessor

  AsyncTimerCallback  OnFinish;  // no cookie
  AsyncTimer2Callback OnFinish2; // returns cookie

  void Every(unsigned long millisInterval, AsyncTimerCallback onFinish);
  void In(unsigned long millisInterval, AsyncTimerCallback onFinish);

private:
  bool _isActive;     // Started and not expired with no reset
  bool _isExpired;    // timer values checked and found to be expired - may be auto reset
  unsigned long _startTime; // reset by Reset()
  bool SendCookie;
  int  Cookie;
};
#endif
