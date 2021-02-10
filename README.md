# Licorne Smart Alarm Clock

The firmware is pure Rust using [RTIC](https://rtic.rs/)(Real-Time Interrupt-driven Concurrency).

Objectives:
* allow for small kids to be aware of different day period using a simple color code (ie. 5:00 is sleep time).
* continue to learn embedded Rust
* have fun

## Features

* 2.13'in E-Paper display
* DS3231 RTC for keeping track of time
* BME680 for temperature and COV measures
* TI TM4C123G (Cortex M4F)


## Build log

The project initially started during first French lock-down because we really
needed our sleep time and the ours kids could not make the difference between
5:00 and 8:00. A first [Arduino
prototype](https://github.com/dkm/licorne-smartalarmclock) came to life quickly
and has been used for several months. In parallel, this project started with no
hard deadline with the following goals:
- have a more finished hardware solution (versus previous arduino breadboard)
- use event-based programming using embedded Rust


## PCB

## Software

## End result


