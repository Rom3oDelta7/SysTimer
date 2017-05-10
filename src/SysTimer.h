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

// base class, not directly used
class SysTimerBase {
public:
   // need this to make constructor public to prevent a compiler error (cannot reference constructor) in derived classes
   SysTimerBase() {}                                  // note that SysTimerBase() = default; will not work in this case

   virtual bool begin(void) const { return _valid; }

   void setInterval(uint32_t interval) { _interval = interval; }

   uint32_t getInterval(void) const {
      return _interval;
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
   static int8_t _index;                              // counter for instantiated objects, initialized in SysTimer_SAM.cpp
   uint32_t      _interval = 0;                       // msec interval for timer
   volatile bool _repeating = false;                  // true if the timer continues until stopped
   volatile bool _oneshot = false;                    // control flag for one-shot events
   CallbackArg   _callback = nullptr;                 // timer interrupt user callback function
   void*         _callbackArg = nullptr;              // argument for aforementioned callback function
};

#if defined(ESP8266)

extern "C" {
   #include <user_interface.h>
}
#define SYST_MAX_TIMERS  -1                            // -1 indicates no inherent limit
#define SysTimer ESPTimer

// ESP8266 class
class ESPTimer : public SysTimerBase {
public:
   ESPTimer() { 
      _platform = Platform::T_ESP;
      _valid = true;
   }

   bool begin(void) const override { return true; }

   bool attachInterrupt(CallbackArg isr, void* callbackArg) {
      _callback = isr;
      os_timer_setfn(&_timer, static_cast<ETSTimerFunc*>(isr), callbackArg);
      return true;
   }

   bool arm(const bool repeat)  {
      if ((_callback != nullptr) && (_interval > 0)) {
         _interval = (_interval >= 5) ? _interval : 5;
         os_timer_arm(&_timer, _interval, repeat);
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
#else
   #define SYST_MAX_TIMERS    5

   static void _isrSAM0 (void);                 // refers to Timer, not Timer0 in DueTimer.cpp, but 0 used here for consistency
   static void _isrSAM1 (void);
   static void _isrSAM6 (void);
   static void _isrSAM7 (void);
   static void _isrSAM8 (void);
#endif

class SAMTimer;

extern SAMTimer*    _SAMTimerTable[];
extern CallbackFunc _SAMCallbackTable[];

// SAM (Due) class
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
         // instantiated but not valid: a zombie timer - user must call begin method to validate
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


#elif defined(__AVR__)
#include <avr/io.h>

#define SysTimer AVRTimer

/*
AVR timer implementation for ATMega 8/16-bit timers:
Uno (ATmega168/328) :       timer 1
Pro Micro (ATmega16/32U4):  timer 1, 3
Mega (ATMega1280/2560) :    timer 1, 3, 4, 5

Note that the Servo.h library uses Timer 1, so we address that with the USING_SERVO_LIB define as we did with the Due

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
   #define SYST_MAX_TIMERS    4               // TImers 1, 3, 4, 5
#elif defined(__AVR_ATmega32u4__) || defined(ARDUINO_AVR_PROMICRO) || defined(__AVR_ATmega16u4__)
   #define SYST_MAX_TIMERS    2               // Timers 1, 3
#elif !defined(USING_SERVO_LIB)
   #define SYST_MAX_TIMERS    1
#else 
   #define SYST_MAX_TIMERS    0               // Timer 1 exists but cannot be used
#endif

class AVRTimer;
extern AVRTimer* _AVRTimerTable[];

extern void      initTimer(const uint8_t timerNum);
extern uint16_t  setTimerInterval(const uint8_t timerNum, const uint16_t msec);
extern void      startTimer(const uint8_t timerNum);
extern void      stopTimer(const uint8_t timerNum, const bool disableInterrupts = true);

// Atmel class: Uno, Mega, etc.
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
         _interval = setTimerInterval(_current, static_cast<uint16_t>(_interval));
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


#endif // architecture

#endif //header protect
