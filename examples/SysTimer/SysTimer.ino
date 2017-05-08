/*
Demonstrate the SysTimer library timer abstraction functions on different platforms
This example will run on any of the platforms listed in the switch() statement below

Copyright 2017 Rob Redford
This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 (CC BY-SA 4.0) International License.
To view a copy of this license, visit https://creativecommons.org/licenses/by-sa/4.0

*/
#include "SysTimer.h"

void setup(void) {
   Serial.begin(115200);
}

int counter = 0;

// timer callback function #1
void isr1(int num) {
   counter += num;
}

// timer callback function #2 - callback functions must always have an argument, even if not used
void isr2(int dummy) {
   ++counter;
}

// recursive function to intentionally exhaust available timers
void exhaustTimers(void) {
   SysTimer timer;
   static int count = 2;               // two timers in use when this is called in loop()

   if (timer.valid()) {
      ++count;
      // each successive call will put one more object on the stack, and we will eventually run out of available timers
      exhaustTimers();
   } else {
      //good programming practice: the F() macro places string constants in flash instead of RAM
      Serial.print(F("Exhausted timers after "));
      Serial.print(count);
      Serial.println(F(" timers allocated\n\n"));
   }
}

void loop(void) {
   Platform board;

   SysTimer mytimer;

   board = mytimer.getPlatform();
   switch (board) {
   case Platform::T_AVR:
      Serial.println(F("Testing AVR(Arduino) platform\n"));
      break;

   case Platform::T_SAM:
      Serial.println(F("Testing SAM(Due) platform\n"));
      break;

   case Platform::T_ESP:
      Serial.println(F("Testing ESP8266/32 platform\n"));
      break;
   }

   if (!mytimer.valid()) {
      Serial.println(F("*** 1st timer is invalid ***"));
   } else {
      // one-shot test
      counter = 0;
      Serial.println(F("*** One shot timer (START)***"));
      Serial.print(F("starting value: ")); Serial.println(counter);
      mytimer.setInterval(500);
      // if you are passing any argument other than a (void*) then you need to cast BOTH the function pointer and the argument
      mytimer.attachInterrupt((CallbackArg)isr1, (void*)1);
      mytimer.arm(false);
      delay(5000);              // disarm() is not needed for one-shot timers
      Serial.print(F("Expected: 1, Actual: ")); Serial.println(counter);
      Serial.println("");
      Serial.println(F("*** One shot timer (END)***\n\n"));

      // repeat until disarmed
      counter = 0;
      Serial.println(F("*** Repeating timer 250 msec interval (START)***"));
      Serial.print(F("starting value: ")); Serial.println(counter);
      mytimer.setInterval(250);
      mytimer.attachInterrupt((CallbackArg)isr1, (void*)1);
      mytimer.arm(true);
      delay(5000);
      mytimer.disarm();
      // due to timing considerations, the actual value may be slightly off
      Serial.print(F("Expected: 20+/-, Actual: ")); Serial.println(counter);
      Serial.println(F("*** Repeating timer 250 msec interval (END)***\n\n"));

      // re-arm timer with different interval but the same ISR
      counter = 0;
      Serial.println(F("*** Repeating timer 5 msec interval (START)***"));
      Serial.print(F("starting value: ")); Serial.println(counter);
      mytimer.setInterval(5);
      mytimer.arm(true);
      delay(5000);
      mytimer.disarm();
      Serial.print(F("Expected: 1000+/-, Actual: ")); Serial.println(counter);
      Serial.println(F("*** Repeating timer 5 msec interval (END)***\n\n"));

      // instantiate a new timer and demonstrate some of the status functions
      SysTimer newtimer;

      if (newtimer.valid()) {
         counter = 0;
         Serial.println(F("*** New timer object - one-shot (START)***"));
         Serial.print(F("starting value: ")); Serial.println(counter);
         newtimer.setInterval(1000);
         newtimer.attachInterrupt((CallbackArg)isr2, nullptr);             // must pass a null argument as a placeholder if arg not used in the callback function
         newtimer.arm(false);
         if (!newtimer.isRepeating()) {
            delay(5000);
            Serial.print(F("Expected: 1, Actual: ")); Serial.println(counter);
            Serial.println(F("***New timer object - one-shot (END)***\n\n"));
         } else {
            Serial.println(F("*** failed: one-shot timer should not repeat"));
         }

         // if this define has some value other than -1, then there are a limited number of timers available
         if (SYST_MAX_TIMERS != -1) {
            // intentiaonally exhaust available timers
            Serial.print(F("There are "));
            Serial.print(SYST_MAX_TIMERS);
            Serial.println(F(" timer objects available."));
            exhaustTimers();
         } else {
            Serial.println(F("No hard limit on number of timer objects.\n"));
         }
         if (newtimer.armed()) {
            Serial.println(F("*** failed: one-shot timer should not be armed"));
         }
      } else {
         Serial.println(F("*** cannot instantiate a new timer object"));
      }
   }
   Serial.println(F("\n\n****************** END OF TEST ***********************"));
   // timers are no destroyed, so cannot loop again
   while (true) delay(500);
}