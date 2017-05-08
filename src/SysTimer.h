/*
SysTimer: a timer abstraction library that provides a simple and consistent API across a variety of platforms
Currently supports:
  * ESP platforms (ESP8266, ESP32)
  * AVR platforms (ATmega168/328: Uno, Mega, Nano, Teensy, etc.)
  * SAM platforms (Due)

This library utilizes the following libraries for the actual timer implementation:
DueTimer: https://github.com/ivanseidel/DueTimer

To provide a consistent interface, some functions of the supporting timer libraries, especially those that are hardware-specific, are not supported.
The resolution of the timer interval is always in milliseconds, for example.
In other cases, some capabilities are added. A significant example is the ability to pass an argument to the intgerrupt handler (callback).

Copyright 2017 Rob Redford
This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 (CC BY-SA 4.0) International License.
To view a copy of this license, visit https://creativecommons.org/licenses/by-sa/4.0
*/

#ifndef _SysTimer_H_
#define _SysTimer_H_

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#error Older versions of Arduino IDE not supported
#endif

enum class Platform:uint8_t { T_ESP, T_AVR, T_SAM };

typedef void (*CallbackFunc)(void);
typedef void (*CallbackArg)(void*);

class SysTimerBase {
public:
   // need this to make constructor public to prevent a compiler error (cannot reference constructor) in derived classes
   SysTimerBase() {}                                  // note that SysTimerBase() = default; will not work in this case

   void setInterval(uint32_t interval) {
      _interval = interval;
   }

   uint32_t getInterval(void) const {
      return _interval;
   }

   bool valid(void) const {
      return _valid;
   }

   bool armed(void) const {
      return _armed;
   }

   bool isRepeating(void) const {
      return _repeating;
   }

   Platform getPlatform(void) const {
      return _platform;
   }

protected:
   Platform      _platform;
   bool          _valid = false;                      // true if this is a valid (enabled) timer
   volatile bool _armed = false;                      // true when timer is active
   static int8_t _index;                              // counter to manage number of instantiated timers
   uint32_t      _interval = 0;                       // msec interval for timer
   volatile bool _repeating = false;                  // true if the timer continues until stopped
   volatile bool _oneshot = false;                    // control flag for one-shot events
   CallbackArg   _callback = nullptr;                 // timer interrupt user callback function
   void*         _callbackArg = nullptr;              // argument for aforementioned callback function
};

#if defined(ESP8266) or defined(ESP32)

extern "C" {
   #include "user_interface.h"
}
#define SYST_MAX_TIMERS  -1                            // -1 indicates no inherent limit
#define SysTimer ESPTimer

class ESPTimer : public SysTimerBase {
public:
   ESPTimer() { 
      _platform = Platform::T_ESP;
      _valid = true;
   }

   bool attachInterrupt(CallbackArg isr, void* callbackArg) {
      _callback = isr;
      os_timer_setfn(&_timer, static_cast<ETSTimerFunc*>(isr), callbackArg);
      return true;
   }

   bool arm(const bool repeat)  {
      if ((_callback != nullptr) && (_interval > 0)) {
         os_timer_arm(&_timer, _interval >= 5 ? _interval : 5, repeat);
         _repeating = repeat;
         _armed = repeat ? true : false;             // we have no way to clear the flag after the interrupt actually happens, so must do it here
      } else {
         _armed = false;
      }
      return _armed;
   }

   bool disarm(void)  {
      os_timer_disarm(&_timer);
      _armed = false;
      return true;
   }

private:
   os_timer_t    _timer;
};

#elif defined(__SAM3X8E__)

#include <DueTimer.h>
#include <functional>

#define SysTimer SAMTimer

/*
 We need to allow the user to set up a callback function with an argument. However, the DueTimer library does
 not support an argument to the interrupt handler. This would be straightforward if the DueTimer interrupt handler allowed functors, but
 alas, it does not. Instead, we have to have a simple pointer to a function with no arg that returns void (CallbackFunc).
 Thus, we have to set up a function of this type with the additional info in tables so that we can then set up the call we need (CallbackArg).

 A more elegant way would be to use a Lambda function or std::function wrappers but this would require library modifications.
*/
class SAMTimer;                               // forward ref decl

#ifndef USING_SERVO_LIB
   #define SYST_MAX_TIMERS    9                    // timers pre-instantiated in DueTimer library

   static void _isrSAM0 (void);
   static void _isrSAM1 (void);
   static void _isrSAM2 (void);
   static void _isrSAM3 (void);
   static void _isrSAM4 (void);
   static void _isrSAM5 (void);
   static void _isrSAM6 (void);
   static void _isrSAM7 (void);
   static void _isrSAM8 (void);

   /*
    the DueTimer library pre-instantiates the Timer objects 0:7
    0, 2, 3, 4, and 5 are not available if the Servo library (Servo.h) is used

    table of interrupt handlers for the DueTimer isr function (this one does not accept an argument to the ISR)
   */
 
   CallbackFunc _SAMCallbackTable[SYST_MAX_TIMERS] = { &_isrSAM0, &_isrSAM1, &_isrSAM2, &_isrSAM3, &_isrSAM4, &_isrSAM5, &_isrSAM6, &_isrSAM7, &_isrSAM8 };
#else
   #define SYST_MAX_TIMERS    5

   static void _isrSAM0 (void);                 // refers to Timer, not Timer0 in DueTimer.cpp, but 0 used here for consistency
   static void _isrSAM1 (void);
   static void _isrSAM6 (void);
   static void _isrSAM7 (void);
   static void _isrSAM8 (void);

   CallbackFunc _SAMCallbackTable[MAX_TIMERS] = { &_isrSAM0, &_isrSAM1, &_isrSAM6, &_isrSAM7, &_isrSAM8 };
#endif

// allows us to emulate use of "this" in the interrupt handlers referenced through the above callback table
SAMTimer*    _SAMTimerTable[SYST_MAX_TIMERS] = { nullptr };

class SAMTimer : public SysTimerBase {
public:
   SAMTimer() { 
      if (_index + 1 <= SYST_MAX_TIMERS) {
         _platform = Platform::T_SAM;
         _valid = true;
         _current = _index;
         ++_index;
         // save address of this object so we can access state vars from our ISR
         _SAMTimerTable[_current] = this;
      } else {
         // can't return an error from a constructor, so we do this instead - we now have a "zombie" timer
         _valid = false;
      }
   }

   bool attachInterrupt(const CallbackArg isr, void* callbackArg) {
      if (_valid) {
         /*
          save the user's callback and argument in this object, then call our "shim" ISR that will
          construct the function call we actually need
         */
         _callback = isr;
         _callbackArg = callbackArg;
         _DueTimers[_current]->attachInterrupt(_SAMCallbackTable[_current]);
         return true;
      } else {
         return false;
      }
   }

   bool arm(const bool repeat) {
      if (_valid && (_callback != nullptr) && (_interval > 0)) {
         if (repeat) {
            _repeating = true;
            _oneshot = false;
         } else {
            _repeating = false;
            _oneshot = true;                     // will be flipped once we get the first callback
         }
         _DueTimers[_current]->start(static_cast<double>(_interval * 1000));          // msec to usec
         _armed = true;
      } else {
         _armed = false;
      }
      return _armed;
   }

   // stop the timer, but leave the state vars intact, so you just need to rearm it to restart
   bool disarm(void) {
      if (_valid) {
         _DueTimers[_current]->stop();
         _repeating = false;
         _oneshot = false;
         _armed = false;
         return true;
      } else {
         return false;
      }
   }

private:
   int8_t      _current = -1;              // indexes the current timer
   // table of the pre-allocated timer objects in the DueTimer library, which we use directly
#ifndef USING_SERVO_LIB
   DueTimer* _DueTimers[SYST_MAX_TIMERS] = { &Timer0, &Timer1, &Timer2, &Timer3, &Timer4, &Timer5, &Timer6, &Timer7 };

   // allow shim ISR to access the object private parts
   friend  void _SAMCommonHandler(SAMTimer* that);
   friend  void _isrSAM0(void);
   friend  void _isrSAM1(void);
   friend  void _isrSAM2(void);
   friend  void _isrSAM3(void);
   friend  void _isrSAM4(void);
   friend  void _isrSAM5(void);
   friend  void _isrSAM6(void);
   friend  void _isrSAM7(void);
   friend  void _isrSAM8(void);
#else
   DueTimer* _DueTimers[MAX_TIMERS] = { &Timer, &Timer1, &Timer6, &Timer7 };
   friend  void _SAMCommonHandler(SAMTimer* that);
   friend  void _isrSAM0(void);
   friend  void _isrSAM1(void);
   friend  void _isrSAM6(void);
   friend  void _isrSAM7(void);
   friend  void _isrSAM8(void);
#endif
};

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
#include <avr/io.h>

#define SysTimer AVRTimer

/*
AVR timer implementation for ATMega 8/16-bit timers:
Uno (ATmega168/328) :       timer 1
Pro Micro (ATmega16/32U4):  timer 1, 3
Mega (ATMega1280/2560) :    timer 1, 3, 4, 5

Reference: https://arduinodiy.wordpress.com/2012/02/28/timer-interrupts/

Basic strategy:
1. clear the timer control registers (stops timer)
2. enable timer compare on match interrupt mask
3. pre-load the match register with pre-calculated count value
4. set the CS control bits in the timer control register to 1024 pre-scaler value and CRC mode, which also starts the timer
5. The ISR for the overflow timer fires at the end of the interval, and we take the necessary actions in the ISR
e.g. countine counting, stop the timer, etc.

note that timer functions are declared as "static" to limit their scope to this file
*/

// macros for register pre-defined symbols  - see iomx8.h for Arduino, iomxx0_1.h for Arduino Mega
#define TIMER_CONTROL(T, S) TCCR ## T ## S
#define TIMER_MASK(T)       TIMSK ## T
#define TIMER_CTC(T)        OCIE ## T ## A
#define TIMER_CMR(T)        OCR ## T ## A

#define MAX_INTERVAL          ((65535.0 * 1024.0)/(double)F_CPU)          // floating representation of longest timer interval with 16-bit counter and 1024 pre-scaler


// set number of available 16-bit timers
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
   #define SYST_MAX_TIMERS    4 
#elif defined(__AVR_ATmega32u4__) || defined(ARDUINO_AVR_PROMICRO) || defined(__AVR_ATmega16u4__)
   #define SYST_MAX_TIMERS    2
#else
   #define SYST_MAX_TIMERS    1
#endif

class AVRTimer;                               // forward ref decl

/*
stop a timer by clearing the timer control registers
Mega timers: 1, 3, 4, 5
by default, disables interrupts
*/
static void inline stopTimer (const uint8_t timerNum, const bool disableInterrupts = true) {
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
static void inline initTimer(const uint8_t timerNum) {
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
static void inline startTimer(const uint8_t timerNum) {
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
*/
uint16_t inline setTimerInterval(const uint8_t timerNum, const uint16_t msec) {
   cli();
   uint16_t maximum = static_cast<uint16_t>((MAX_INTERVAL) * 1000.0);
   double elapsed = constrain(msec, 1, maximum) / 1000.0;
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
   return counter;
}

// allows us to emulate use of "this" in the interrupt handlers referenced through the above callback table
static AVRTimer* _AVRTimerTable[SYST_MAX_TIMERS] = { nullptr };

void _AVRCommonHandler(AVRTimer* that);

/*
Function macros for timer interrupt handlers
Notes:
1. for one-shot timers, clear the timer control register "B" here to stop the timer
2. interrupts are disabled in this macro
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


class AVRTimer : public SysTimerBase {
public:
   AVRTimer() {
      if (_index + 1 <= SYST_MAX_TIMERS) {
         _platform = Platform::T_AVR;
         _valid = true;
         _current = _index;
         ++_index;
         // save address of this object so we can access state vars from our ISR
         _AVRTimerTable[_current] = this;
         initTimer(_current);
         //Serial.println(F("> CONSTRUCTOR"));
      } else {
         // can't return an error from a constructor, so we do this instead - we now have a "zombie" timer
         _valid = false;
      }
   }

   bool attachInterrupt(const CallbackArg isr, void* callbackArg) {
      if (_valid) {
         // save the user's callback and argument in this object and call from the interrupt handler
         _callback = isr;
         _callbackArg = callbackArg;
         //Serial.println(F(">> ATTACH"));
         return true;
      } else {
         return false;
      }
   }

   bool arm(const bool repeat) {
      if (_valid && (_callback != nullptr) && (_interval > 0)) {
         if (repeat) {
            _repeating = true;
            _oneshot = false;
         } else {
            _repeating = false;
            _oneshot = true;                     // will be flipped once we get the first callback
         }
         setTimerInterval(_current, static_cast<uint16_t>(_interval));
         startTimer(_current);
         _armed = true;
         //Serial.println(F(">>> ARM"));
      } else {
         _armed = false;
      }
      return _armed;
   }

   bool disarm(void) {
      if (_valid) {
         stopTimer(_current);
         _repeating = false;
         _oneshot = false;
         _armed = false;
         //Serial.println(F(">>>> DISARM"));
         return true;
      } else {
         return false;
      }
   }

private:
   int8_t      _current = -1;              // indexes the current timer
   // allow shim ISR to access the object private parts
   friend  void _AVRCommonHandler(AVRTimer* that);
};

/*
Shim ISR that associates the interrupt with the initiatiating timer object and the calls the user's callback function
with the provided (non-optional) argument
*/
void _AVRCommonHandler(AVRTimer* that) {
   if (that->_repeating || that->_oneshot) {
      //auto callback = std::bind(that->_callback, that->_callbackArg);
      (*(that->_callback))(that->_callbackArg);                                       // bind unavailable
   }
   if (that->_oneshot) {
      that->_oneshot = false;
      that->disarm();
   }
}


#endif // architecture

int8_t SysTimerBase::_index = 0;                        // static class member initialization

#endif //header protect
