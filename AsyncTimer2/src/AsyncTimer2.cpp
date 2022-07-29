/***************************************************
Copyright (c) 2017 Luis Llamas
(www.luisllamas.es)

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License
 ****************************************************/

/******************************************************************************
 * 2021-01-30 John Jordan - Moved to AsyncTimer2.
 * 2020-12-31 John Jordan - Added callback with parameter.
 * 2019       John Jordan - Various mods to funcions, names, heavily
 *                          reformatting and commented.
 ******************************************************************************/

#include "AsyncTimer2.h"

AsyncTimer2::AsyncTimer2(unsigned long microsInterval, AsyncTimerCallback onFinish, bool autoReset/*=false*/)
  : AsyncTimer2(microsInterval, autoReset, onFinish) { }

AsyncTimer2::AsyncTimer2(unsigned long microsInterval, bool autoReset/*=false*/,
                         AsyncTimerCallback onFinish/*=nullptr*/) {
  Interval = microsInterval;
  AutoReset = autoReset;
  OnFinish = onFinish;
  SendCookie = false;
}

AsyncTimer2::AsyncTimer2(unsigned long microsInterval, AsyncTimer2Callback onFinish2, int cookie, bool autoReset/*=false*/) {
  Interval  = microsInterval;
  AutoReset = autoReset; 
  OnFinish2 = onFinish2;
  SendCookie = true;
  Cookie = cookie;  // default param 0
}

void AsyncTimer2::Start() {
  Reset();
  _isActive  = true;
  _isExpired = false;
}

void AsyncTimer2::ConditionalStart() {
  if (_isActive) return;
  Start();
}

void AsyncTimer2::Reset() {
  _startTime = micros();
}

void AsyncTimer2::Stop() {
  _isActive = false;
}

bool AsyncTimer2::Check() {   // returns _isActive for "while (Check()) ...
  if (_isActive == false) return false; // returns false if inactive
  if (static_cast<unsigned long>(micros() - _startTime) >= Interval) {
    _isExpired = true;
    _isActive = AutoReset; // set to inactive if no reset
    if (SendCookie and OnFinish2 != nullptr) OnFinish2(Cookie);
    else if (OnFinish != nullptr) OnFinish();
    _isExpired = !AutoReset;
    if (AutoReset)  Reset();
  }
  return _isActive;
}

bool AsyncTimer2::CheckAndSwitch(AsyncTimer2 &next) {
  if (not Check()) {  // current timer not active
    next.Start();     // launch next timer if current timer is inactive
    return true;      // indicate timer switch
  }
  return false;  // returns false if no timer switch
}

void AsyncTimer2::SetIntervalMillis(unsigned long interval) {
  Interval = interval * 1000;
}

void AsyncTimer2::SetIntervalMicros(unsigned long interval) {
  Interval = interval;
}

unsigned long AsyncTimer2::GetStartTime() {
  return _startTime;
}

unsigned long AsyncTimer2::GetElapsedTime() {
  return micros() - _startTime;
}

unsigned long AsyncTimer2::GetRemainingTime() {
  return Interval - micros() + _startTime;
}

void AsyncTimer2::SetCookie (int cookie) {
  Cookie = cookie;
}

int AsyncTimer2::GetCookie (void) {
  return Cookie;
}

bool AsyncTimer2::IsActive() const {
  return _isActive;
}

bool AsyncTimer2::IsExpired() const {
  return _isExpired;
}

void AsyncTimer2::Every(unsigned long millisInterval, AsyncTimerCallback onFinish) {
  this->SetIntervalMillis(millisInterval);
  this->OnFinish = onFinish;
  this->AutoReset = true;
  this->Start();
}

void AsyncTimer2::In(unsigned long millisInterval, AsyncTimerCallback onFinish) {
  this->SetIntervalMillis(millisInterval);
  this->AutoReset = false;
  this->OnFinish = onFinish;
  this->Start();
}
