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


#elif defined(__AVR__)


// macros for register pre-defined symbols  - see iomx8.h for Arduino, iomxx0_1.h for Arduino Mega
#define TIMER_CONTROL(T, S) TCCR ## T ## S
#define TIMER_MASK(T)       TIMSK ## T
#define TIMER_CTC(T)        OCIE ## T ## A
#define TIMER_CMR(T)        OCR ## T ## A

/*
stop a timer by clearing the timer control registers
by default, disables interrupts
*/
void stopTimer (const uint8_t timerNum, const bool disableInterrupts) {
   if (disableInterrupts) cli();
   switch (timerNum) {
   case 0:
      TIMER_CONTROL(1, A) = 0;                 // technically, the timer stops when the CSx bits in segment B are cleared, but clear this too for insurance
      TIMER_CONTROL(1, B) = 0;
      break;
#if SYST_MAX_TIMERS >= 2
   case 1:
      TIMER_CONTROL(3, A) = 0;
      TIMER_CONTROL(3, B) = 0;
      break;
#if SYST_MAX_TIMERS == 4
   case 2:
      TIMER_CONTROL(4, A) = 0;
      TIMER_CONTROL(4, B) = 0;
      break;
   case 3:
      TIMER_CONTROL(5, A) = 0;
      TIMER_CONTROL(5, B) = 0;
      break;
#endif
#endif
   }
   if (disableInterrupts) sei();
}

/*
initialize a timer by setting the timer compare interrupt bit in the timer mask register
*/
void initTimer(const uint8_t timerNum) {
   cli();
   stopTimer(timerNum, false);
   switch (timerNum) {
   case 0:
      TIMER_MASK(1) |= _BV(TIMER_CTC(1));
      break;
#if SYST_MAX_TIMERS >= 2
   case 1:
      TIMER_MASK(3) |= _BV(TIMER_CTC(3));
      break;
#if SYST_MAX_TIMERS == 4
   case 2:
      TIMER_MASK(4) |= _BV(TIMER_CTC(4));
      break;
   case 3:
      TIMER_MASK(5) |= _BV(TIMER_CTC(5));
      break;
#endif
#endif
   }
   sei();
}

/*
start a timer by setting the control bits

uses a fixed prescaler of 1024 (bits CS10 and CS12)
also set WGM12 to enable the cimer compare match mode (CTC)

Setting the control bits starts the timer. Once started, the timer countines to count until stopped
*/
void startTimer(const uint8_t timerNum) {
   cli();
   switch (timerNum) {
   case 0:
      TIMER_CONTROL(1, B) |= (_BV(CS10) | _BV(CS12) | _BV(WGM12));
      break;
#if SYST_MAX_TIMERS >= 2
   case 1:
      TIMER_CONTROL(3, B) |= (_BV(CS10) | _BV(CS12) | _BV(WGM12));
      break;
#if SYST_MAX_TIMERS == 4
   case 2:
      TIMER_CONTROL(4, B) |= (_BV(CS10) | _BV(CS12) | _BV(WGM12));
      break;
   case 3:
      TIMER_CONTROL(5, B) |= (_BV(CS10) | _BV(CS12) | _BV(WGM12));
      break;
#endif
#endif
   }
   sei();
}

/*
use the CTC timer mode (interrupts on timer compare match)

uses a fixed prescaler of 1024 (bits CS10 and CS12) which is used as a divisor of the clock frequency
this means the minimum period for the timer on a 16MHz system (the "resolution") is :
  1/((16 x 10^6) / 1024) = 6.4 x 10^-5 = 0.000064 seconds (64 usec)
and the maximum period is:
  (6.4 x 10^-5) * 65535 = 4.19424 sec
also set WGM12 to enable the timer compare match mode (CTC)

for this mode, we calculate the initial value of the counter as follows:
  count value = (time / resolution) - 1
  note: it is -1 because 0 is counted also
and load this value into the timer compare match register

Returns the set interval, possibly constrained
*/
uint16_t setTimerInterval(const uint8_t timerNum, const uint16_t msec) {
   cli();
   uint16_t maximum = static_cast<uint16_t>((MAX_INTERVAL) * 1000.0);
   uint16_t interval = constrain(msec, 1, maximum);
   double elapsed = static_cast<double>(interval) / 1000.0;
   double temp = (elapsed / static_cast<double>(1024.0/F_CPU)) - 1.0;
   uint16_t counter = static_cast<uint16_t>(temp);

   switch (timerNum) {
   case 0:
      TIMER_CMR(1) = counter;
      break;
#if SYST_MAX_TIMERS >= 2
   case 1:
      TIMER_CMR(3) = counter;
      break;
#if SYST_MAX_TIMERS == 4
   case 2:
      TIMER_CMR(4) = counter;
      break;
   case 3:
      TIMER_CMR(5) = counter;
      break;
#endif
#endif
   }
   sei();
   return interval;
}

// allows us to emulate use of "this" in the interrupt handlers referenced through the above callback table
AVRTimer* _AVRTimerTable[SYST_MAX_TIMERS] = { nullptr };

/*
Shim ISR that associates the interrupt with the initiatiating timer object and the calls the user's callback function
with the provided (non-optional) argument
*/
void _AVRCommonHandler(AVRTimer* that) {
   if (that->_repeating || that->_oneshot) {
      (*(that->_callback))(that->_callbackArg);                                       // std::bind unavailable
   }
   if (that->_oneshot) {
      that->_oneshot = false;
      that->disarm();
   }
}

/*
Function macros for timer interrupt handlers
Notes:
1. for one-shot timers, could clear the timer control register "B" here to stop the timer
   but we instead call disarm() to manage all the other state variables, which calls stopTimer()
2. interrupts are disabled when we enter
*/
ISR(TIMER1_COMPA_vect) {
   _AVRCommonHandler(_AVRTimerTable[0]);
}

#if SYST_MAX_TIMERS >= 2

ISR(TIMER3_COMPA_vect) {
   _AVRCommonHandler(_AVRTimerTable[1]);
}
#if SYST_MAX_TIMERS == 4
ISR(TIMER4_COMPA_vect) {
   _AVRCommonHandler(_AVRTimerTable[2]);
}

ISR(TIMER5_COMPA_vect) {
   _AVRCommonHandler(_AVRTimerTable[3]);
}
#endif
#endif

#endif