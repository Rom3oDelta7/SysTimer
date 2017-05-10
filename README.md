# SysTimer: Hardware-Based Timers with Support for Multiple Platforms and a Common User Interface
SysTimer is an Arduino IDE linrary for hardware-level timers that simplifies managing hardware timers and writing portable code.
It supports the ESP8266, Atmel AVR-based platforms, and Atmel SAM3X8E ARM Cortex-M3 platforms such as the Due.
Benefits of this library include:

* adds a layer of abstraction, simplifying management of harware timers (e.g. you do not need to know the timer number, for example)
* a common API across all platforms, enabling much more portable code
* added functionality, such as a user-defined parameter to the interrupt handler (callback function)

An example benefit of the use of this library can be seen in the [LEDManager Library], where an LED can be set to blink even when
the processor is running code that blocks. The LEDManager Library works across all of the aforementioned platforms.

## Programming Interface

### Declaring a Timer

All timers, regardless of platform, are delcared as follows:

```C++
SysTimer myTimer;
```

where ```myTimer``` is the name of the SysTimer timer you are declaring.
This will allocate and initialize the next available timer.
The number of available timers depends on the platform you are building the code for.
The preprocessor define ```SYST_MAX_TIMERS``` will be set to the number of available timers
for each platform.
On a platform such as the ESP8266, where the timer functions are implemented using the ESP8266 SDK, 
the number of timers has no inherent limit. 
In this case ```SYST_MAX_TIMERS``` is set to ```-1```.

#### Zombie Timers
One consequence of providing such a simple mechanism for declaring timer objects is that a timer object may be created that is not valid.
This happens when you exceed the number of available hardware timers on your platform.
For example, on the Uno, there is only one timer available.
Thus, it is essential to check the return value of the ```begin``` function (see below) to ensure the timer you declared is actually valid.

### Library Dependencies
The timer functions for the ESP8266 and AVR platforms are implemented in the SysTimer library directly and do not require any other libraries to function.
The SAM (Due) timers are implemented in top of the [DueTimer library], which must be installed prior to using this library.

### Programming Interface

#### Managing Timers

All of the functions in this section are mandatory for each timer you declare.

```C++
bool begin(void);
```

```begin``` returns ```true``` if a timer was available and properly initilized, and ```false``` if not.
In the latter case, any boolean library function that is called through this object will always return ```false``` as well.
_This must be the first function you call once declaraing a timer._

```C++
void setInterval(uint32_t interval);
```
This sets the number of milliseconds between timer interrupts. 
Depending on the platform, the specified interval may be modified to conform to the hardware timer being used.
For example, the AVR interval cannot exceed 4194 msec on a 16MHz processor (this limit is 8388 msec on an 8MHz processor).
To determine if this has occurred, you can call the ```getInterval``` function (see below).

```C++
bool attachInterrupt(CallbackArg isr, void* callbackArg);
```
This attaches your interrupt service routine (ISR) function (or "callback") that is called each time the timer fires.
Here, you can take any action required when a timer event occurs.
You should take care to keep this function as short and simple as possible, and to not perform any complex operations, such as I/O.
Instead, set flags or variables that will be acted upon in ```loop```, for example.
Note that interrupts will be disabled before your function is called, and re-enabled after it returns, so this _should not be done_ in your callback function.
Returns ```false``` if an error occurred, else ```true```.

The ```isr``` argument is a pointer to a function with an argument of type ```void*``` that returns ```void```.
A typedef has been declared to make this easy for you to use in typecasting:
```C++
typedef void (*CallbackArg)(void*);
```
The argument to your ISR function has been declared as a ```void*``` but you can use any argument that is the same size as ```void*``` by typecasting it.
See the example sketch.

```C++
bool arm(const bool repeat);
```
This function arms the given timer and starts it running.
If ```repeat``` is ```true```, then the time will continue to run and generate interrupts until explicitly stopped.
If ```false```, then the timer will fire once and stop (a "one-shot" timer).
Returns ```false``` if an error occurred, else ```true```.

```C++
bool disarm(void);
```
Stops the timer.
All other state information remains in place, so you can restart the timer at any time by calling ```arm``` again.
(Re-arming the timer in this way will reset the interval.)
Returns ```false``` if an error occurred, else ```true```.

#### Timer Status Functions
These functions return information about the state of your timer. 
They are optional.
```C++
uint32_t getInterval(void);
```
Returns the interval you specified in ```setInterval```, with any adjustments applied due to hardware considerations.

```C++
bool armed(void);
```
Returns ```true``` if the timer is curently armed, else ```false```.

```C++
bool isRepeating(void);
```
Returns ```true``` if the timer is set to repeat indefinitely, else ```false```.

```C++
Platform getPlatform(void);
```
Returns an enumerated (enum) constant identifying the hardware platform in use. 
Values are as follows:

|Value|Hardware Platform|
|---|---|
|T_ESP|ESP8266|
|T_AVR|Atmel AVR platforms (Uno, Mega, Nano, Teensy, Pro Micro, etc.)|
|T_SAM|Atmel SAM3X8E ARM Cortex-M3 platforms (Due)|

## Library Interactions

The Arduino [Servo Library] consumes a number of timers.
If you are using this library along with the Servo library, then declare
```C++
#define USING_SERVO_LIB
```
at the top of your sketch, before the line to include the SysTimer library.
This will block the timers in use by the Servo library.
For example:

```C++
#define USING_SERVO_LIB
#include <SysTimer.h>
```

Note that as the Uno only has one timer, and it is used by the Servo library, there are no remaining timers to use with SysTimer.
Interactions with other libraries that use hardware timers are possible if not likely, so check the library code if you suspect a problem.

## Important Caveats
One AVR platforms, there is a bug in the compiler where the class "constructor" is not called when the object
is declared in global space (e.g. outside all functions). 
Thus, the object parameters are not properly initialized and strange behavior results.
The workaround for this is to declare the object inside a function, such as ```loop```.

There is no class "destructor" for the LED objects. 
This means that when they go out of scope (say at the end of ```loop```) they are are not destroyed.
The next time the loop starts, they are allocated anew and you quickly run out of hardware timers.
The workaround is to set your loop up so that it never exits (e.g. create a loop withing the ```loop``` function).

## Examples
The Systimer sketch in the examples folder provides a simple demonstration of declaring and using SysTimer timers.
It will run unaltered on any of the supported hardware platforms.

## Installation

This library is also maintained in GitHub, which has a release mechanism.
The easiest way to install this is to get the [Latest Release] and install it using the Arduino IDE Library Manager.

Of course, you may also clone this repository and manually install the files if you prefer.

## Copyright Notice

Copyright 2017 Rob Redford.
This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License.
To view a copy of this license, visit [CC BY-SA].


[LEDManager Library]: https://github.com/Rom3oDelta7/LEDManager
[DueTimer library]: https://github.com/ivanseidel/DueTimer
[Servo Library]: https://www.arduino.cc/en/reference/servo
[Latest Release]: https://github.com/Rom3oDelta7/SysTimer/releases/latest
[CC BY-SA]: https://creativecommons.org/licenses/by-sa/4.0
