About
=====

The MetaLLL library is a high-level C++ template metaprogramming framework, that at compile-time provides optimized for end-task Standard Peripheral Libraries.
Like C-type Standard Peripheral Libraries, it hides from developer a platform-specific features.
That make doing explict metaprogramming in C++ as easy and enjoyable as possible within the current language and platform.


Features
========

The main difference of MetaLLL from generic a C-like Standard Peripheral Libraries - is that the MCU peripheral modules, their logic and relationships are defined, verified at compile-time.

Extra nice things - that's what most of the test configuration is done at compile time,not at the stage of execution.

Cons architecture:
- Complicates the tracking of dependencies periphery
- It reduces the flexibility of the architecture during execution
- Slightly increases the build time
- Some other

Pros architecture:
- A universal description of the characteristics of different types of MCU modules
- Abstraction of the firmware from the hardware
- Automated collection of module features to perform atomic operations
- Increased application portability to other MCU platforms
- Reduces the volume of service information describing the modules depending
- Reduces the number of parameters called functions
- Reduce the number of checks on the correctness of the input parameters
- Reduce the amount of program memory and data memory
- Increases the speed of execution of the program
- Improves the energy efficiency of the final decision
- Some other


Dependencies
============

- ARM CortexM:
  - uses CMSIS drivers



History
=======

- 1.0.0
  - Initial implementation for STM32F1xx, using Keil uVision v5.10

