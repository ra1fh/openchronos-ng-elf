GENERAL INFORMATION 
===================

Forked from: [github.com/BenjaminSoelberg/openchronos-ng-elf](https://github.com/BenjaminSoelberg/openchronos-ng-elf)

Changes:
* Added flyback module (see flyback.c for some usage information)
* Added githash module to show githash of current build
* Switched to Python 3
* Compilable with newer MSP430 GCC (9.3.1)

INTRODUCTION
============

Modular opensource firmware for the TI eZ430 Chronos.

openchronos-ng is a major rework of openchronos. Compared to openchronos it has the following features:

* system message bus for system<->module communication.
* hardware RTC timekeeping (no more clock inaccuracy).
* rework of timer and ports drivers.
* implementation of a module system (drop in applications).
* rework of the display routines.
* rework of the menu system.

The firmware code is also conceptually simpler and smaller which leaves room for more modules (applications).

INSTALLATION
============

Compiler and debugger installation
----------------------------------

Download and install the latest MSP430 GCC compiler from TI:

* [https://www.ti.com/tool/MSP430-GCC-OPENSOURCE](https://www.ti.com/tool/MSP430-GCC-OPENSOURCE)

Add the compiler to PATH

Linux Prerequisites
-------------------

Install:

* python-urwid
* mspdebug

Compilation with Make
---------------------

* ```make config```
* ```make clean```
* ```make```

Compilation with CMake
----------------------

* ```MSP430_TI=/usr/local cmake -DCMAKE_TOOLCHAIN_FILE=toolchains/toolchain-msp430-gcc-ti.cmake .```
* ```make config```
* ```make```

Flashing the watch using USB FET
--------------------------------

1) Connect the internal watch module to the USB FET module

2) Reset the watch using the menu or reinserting the battery

3) Program it using mspdebug
```
make usb-install
```
4) Disconnect the watch module and the watch should display BOOT<br>

5) Press any button except DOWN and you should be up and running the new firmware

*Please note that this method is slow but very useful if flashing over wireless fails.*

Flashing the firmware using wireless (RFBSL)
------------------------------------

1) Connect the USB CC11x1 module

2) Reset the watch using the menu or reinserting the battery

3) Program it using ChronosTool.py (Note that sudo might be required. Also repeat this step if it fails)<br>
```
make install
```

4) Press Enter

5) Enter RFBSL by pressing the DOWN button on the watch

6) Press any button except DOWN and you should be up and running the new firmware.

Boot Menu
---------

In openchronos-ng, the watch no longer boots directly into the clock firmware.

To enter the BOOT menu you can either:
* Use the Reset menu
* Reinsert the battery

If the display shows BOOT you have successfully reset the watch and are now in the boot menu.
Press the DOWN button to enter the wireless flash updater (RFBSL). Any other button will run the watch firmware.

Useful links
-------------

* [CC430F6138 Technical Documents](https://www.ti.com/product/CC430F6137)
* [eZ430 Chronos wiki (Wayback Machine)](https://web.archive.org/web/20201112020336/https://processors.wiki.ti.com/index.php/EZ430-Chronos)
* [MSP430™ Programming With the Bootloader (Wayback Machine)](https://web.archive.org/web/20160527090555/http://www.ti.com/lit/ug/slau319l/slau319l.pdf)
* [MSP430™ Programming With the JTAG Interface (Wayback Machine)](https://web.archive.org/web/20161130144328/http://www.ti.com/lit/ug/slau320x/slau320x.pdf)
