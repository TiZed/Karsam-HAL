<b>Karsam</b> HAL for Machinekit

This a Machinekit HAL written for a CNC project. The CNC is built with a PIC32 based real-time controller and a Raspberry Pi as the application computer. Motion commands are sent by SPI from the RPi to the PIC controller. This HAL manages those communications.

Portions of this code are based on stepgen.c by John Kasunich and [PICnc V2](https://github.com/kinsamanka/PICnc-V2) by GP Orcullo.
