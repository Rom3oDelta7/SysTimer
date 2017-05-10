/*
SysTimer: a timer abstraction library that provides a simple and consistent API across a variety of platforms
https://github.com/Rom3oDelta7/SysTimer

Currently supports:
  * ESP8266
  * AVR platforms (Uno, Mega, Nano, Pro Micro, Teensy, etc.)
  * SAM platforms (Due)

This library utilizes the following libraries for the actual timer implementation:
ESP: internal
DueTimer: https://github.com/ivanseidel/DueTimer
AVR: internal


Copyright 2017 Rob Redford
This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 (CC BY-SA 4.0) International License.
To view a copy of this license, visit https://creativecommons.org/licenses/by-sa/4.0
*/

#include <SysTimer.h>

int8_t SysTimerBase::_index = 0;                    // static class member initialization

#if defined(__SAM3X8E__)
#ifndef USING_SERVO_LIB
   /*
    the DueTimer library pre-instantiates the Timer objects 0:7
    0, 2, 3, 4, and 5 are not available if the Servo library (Servo.h) is used

    table of interrupt handlers for the DueTimer isr function (this one does not accept an argument to the ISR)
   */
   CallbackFunc _SAMCallbackTable[] = { &_isrSAM0, &_isrSAM1, &_isrSAM2, &_isrSAM3, &_isrSAM4, &_isrSAM5, &_isrSAM6, &_isrSAM7, &_isrSAM8 };
#else
   CallbackFunc _SAMCallbackTable[] = { &_isrSAM0, &_isrSAM1, &_isrSAM6, &_isrSAM7, &_isrSAM8 }; // refers to Timer, not Timer0 in DueTimer.cpp, but 0 used here for consistency
#endif

// allows us to emulate use of "this" in the interrupt handlers referenced through the above callback table
SAMTimer*    _SAMTimerTable[SYST_MAX_TIMERS] = { nullptr };

/*
 Shim ISR that associates the interrupt with the initiatiating timer object and the calls the user's callback function
 with the provided (non-optional) argument

 Note that we disable interrupts for the scope of the callback here rather than in the user's function
 just to avoid any timing issues
 */
void _SAMCommonHandler(SAMTimer* that) {
   noInterrupts();
   if (that->_repeating || that->_oneshot) {
      auto callback = std::bind(that->_callback, that->_callbackArg);              // VS2017 IntelliSense complains but gcc accepts this
      callback();
      //that->_callback(that->_callbackArg);                                       // this also works
   }
   if (that->_oneshot) {
      that->_oneshot = false;
      that->disarm(); 
   }
   interrupts();
}

// declare these as static to limit their scope to this exeuction unit (for a "C" function)
static void _isrSAM0 (void) {
   _SAMCommonHandler(_SAMTimerTable[0]);
}

static void _isrSAM1 (void) {
   _SAMCommonHandler(_SAMTimerTable[1]);
}

#ifndef USING_SERVO_LIB
static void _isrSAM2 (void) {
   _SAMCommonHandler(_SAMTimerTable[2]);
}

static void _isrSAM3 (void) {
   _SAMCommonHandler(_SAMTimerTable[3]);
}

static void _isrSAM4 (void) {
   _SAMCommonHandler(_SAMTimerTable[4]);
}

static void _isrSAM5 (void) {
   _SAMCommonHandler(_SAMTimerTable[5]);
}
#endif

static void _isrSAM6 (void) {
   _SAMCommonHandler(_SAMTimerTable[6]);
}

static void _isrSAM7 (void) {
   _SAMCommonHandler(_SAMTimerTable[7]);
}

static void _isrSAM8 (void) {
   _SAMCommonHandler(_SAMTimerTable[8]);
}

#endif
