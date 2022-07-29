/***************************************************
Copyright (c) 2017 Luis Llamas
(www.luisllamas.es)

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License
 ****************************************************/

// 2021-01-30  John Jordan - modified for AsyncTimer2.

#include "AsyncTimer2.h"

void msg (void) {
  // Serial.print(millis());
  Serial.print(1.0*micros()/1000,1);
  Serial.print("ms: ");
  Serial.println("Expired");
}

AsyncTimer2 timer1(2000, TIMER_RESETS, msg); // 2ms

void setup() {
	Serial.begin(9600);
	Serial.println("Starting");
	timer1.Start();
}

void loop() {
	timer1.Check();
}
