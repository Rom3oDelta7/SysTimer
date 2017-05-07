/*
SysTimer: a timer abstraction library that provides a consistent API across a variety of platforms
Currently supports:
  * ESP platforms (ESP8266, ESP32)
  * AVR platforms (Atmel AVR: Arduino, Mega, Nano, etc.)
  * SAM platforms (Due)

Copyright 2017 Rob Redford
This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 (CC BY-SA 4.0) International License.
To view a copy of this license, visit https://creativecommons.org/licenses/by-sa/4.0
*/

#ifndef _SysTimer_H
#define _SysTimer_H

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#error Older versions of Arduino IDE not supported
#endif

enum class Platform:uint8_t { T_ESP, T_AVR, T_SAM };

typedef void (*CallbackFunc)(void);
typedef void (*CallbackArg)(void*);

template <typename TimerType>
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
   bool          _armed = false;                      // true when timer is active
   static int8_t _count;                              // static ==> shared counter to manage number of instantiated timers
   uint32_t      _interval = 0;                       // msec interval for timer
   bool          _repeating = false;                  // true if the timer continues until stopped
   bool          _oneshot = false;                    // control flag for one-shot events
   CallbackArg   _callback = nullptr;                 // timer interrupt user callback function
   void*         _callbackArg = nullptr;              // argument for aforementioned callback function
   TimerType     _timer;                              // platform-specific timer
};

#if defined(ESP8266) or defined(ESP32)

extern "C" {
   #include "user_interface.h"
}
#define SYST_MAX_TIMERS  -1                            // -1 indicates no inherent limit
#define SysTimer ESPTimer

class ESPTimer : public SysTimerBase<os_timer_t> {
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
};

template<> int8_t SysTimerBase<os_timer_t>::_count = 0;             // template static member initialization

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

   void _isrSAM0 (void);
   void _isrSAM1 (void);
   void _isrSAM2 (void);
   void _isrSAM3 (void);
   void _isrSAM4 (void);
   void _isrSAM5 (void);
   void _isrSAM6 (void);
   void _isrSAM7 (void);
   void _isrSAM8 (void);

   /*
    the DueTimer library pre-instantiates the Timer objects 0:7
    0, 2, 3, 4, and 5 are not available if the Servo library (Servo.h) is used

    table of interrupt handlers for the DueTimer isr function (this one does not accept an argument to the ISR)
   */
 
   CallbackFunc _SAMCallbackTable[SYST_MAX_TIMERS] = { &_isrSAM0, &_isrSAM1, &_isrSAM2, &_isrSAM3, &_isrSAM4, &_isrSAM5, &_isrSAM6, &_isrSAM7, &_isrSAM8 };
#else
   #define SYST_MAX_TIMERS    5

   void _isrSAM0 (void);                 // refers to Timer, not Timer0 in DueTimer.cpp, but 0 used here for consistency
   void _isrSAM1 (void);
   void _isrSAM6 (void);
   void _isrSAM7 (void);
   void _isrSAM8 (void);

   CallbackFunc _SAMCallbackTable[MAX_TIMERS] = { &_isrSAM0, &_isrSAM1, &_isrSAM6, &_isrSAM7, &_isrSAM8 };
#endif

// allows us to emulate use of "this" in the interrupt handlers referenced through the above callback table
SAMTimer*    _SAMTimerTable[SYST_MAX_TIMERS] = { nullptr };

/*
 Becuase we use the pre-instantiated timer objects here, we do not need the DueTimer object in this derived class
 Using <int> as a dummy arg
 */
class SAMTimer : public SysTimerBase<int> {
public:
   SAMTimer() { 
      if (_count + 1 < SYST_MAX_TIMERS) {
         _platform = Platform::T_SAM;
         _valid = true;
         _current = _count;
         ++_count;
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
   friend void _isrCommonHandler(SAMTimer* that);
   friend void _isrSAM0(void);
   friend void _isrSAM1(void);
   friend void _isrSAM2(void);
   friend void _isrSAM3(void);
   friend void _isrSAM4(void);
   friend void _isrSAM5(void);
   friend void _isrSAM6(void);
   friend void _isrSAM7(void);
   friend void _isrSAM8(void);
#else
   DueTimer* _DueTimers[MAX_TIMERS] = { &Timer, &Timer1, &Timer6, &Timer7 };
   friend void _isrCommonHandler(SAMTimer* that);
   friend void _isrSAM0(void);
   friend void _isrSAM1(void);
   friend void _isrSAM6(void);
   friend void _isrSAM7(void);
   friend void _isrSAM8(void);
#endif
};

/*
 Shim ISR that associates the interrupt with the initiatiating timer object and the calls the user's callback function
 with the provided (non-optional) argument

 Note that we disable interrupts for the scope of the callback here rather than in the user's function
 just to avoid any timing issues
 */

void _isrCommonHandler(SAMTimer* that) {
   noInterrupts();
   if (that->_repeating || that->_oneshot) {
      auto callback = std::bind(that->_callback, that->_callbackArg);              // VS2017 IntelliSense complains but gcc accepts this
      callback();
      //that->_callback(that->_callbackArg);                                       // this also works
   }
   if (that->_oneshot) {
      that->_oneshot = false;
      that->disarm();                         // ***TODO*** make sure this is OK to call in an ISR
   }
   interrupts();
}

void _isrSAM0 (void) {
   _isrCommonHandler(_SAMTimerTable[0]);
}

void _isrSAM1 (void) {
   _isrCommonHandler(_SAMTimerTable[1]);
}

#ifndef USING_SERVO_LIB
void _isrSAM2 (void) {
   _isrCommonHandler(_SAMTimerTable[2]);
}

void _isrSAM3 (void) {
   _isrCommonHandler(_SAMTimerTable[3]);
}

void _isrSAM4 (void) {
   _isrCommonHandler(_SAMTimerTable[4]);
}

void _isrSAM5 (void) {
   _isrCommonHandler(_SAMTimerTable[5]);
}
#endif

void _isrSAM6 (void) {
   _isrCommonHandler(_SAMTimerTable[6]);
}

void _isrSAM7 (void) {
   _isrCommonHandler(_SAMTimerTable[7]);
}

void _isrSAM8 (void) {
   _isrCommonHandler(_SAMTimerTable[8]);
}

template<> int8_t SysTimerBase<int>::_count = 0;             // template static member initialization

#endif // architecture
#endif //header protect
