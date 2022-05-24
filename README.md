# Games with Morse Code at 940nm

This repository contains designs for two detectors and two transmitters
for Morse Code signals, all working with 940nm infra-red light.

You can read more on [my website](https://mjoldfield.com/atelier/2022/01/ir-fun.html).

If you’re near Cambridge, UK you might enjoy the [geocache](https://www.geocaching.com/geocache/GC86BQY_to-the-birdhouse)
 for which they were designed.

## Building the firmware

There are three steps:

  1. Install a GNU toolchain. You can download this from
	 [ARM](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain),
	 but it is often available as system package in e.g. Debian.

  1. Install [libopencm3](https://libopencm3.org):

		$ git submodule init
		
		$ git submodule upgrade

		$ make -C libopencm3

  1. Compile the firmware

		$ make -C tx-bluepill/firmware

		$ make -C tx-stm32l431/firmware

The libopencm3-rules.mk file contains a number of fake targets to flash the firmware.
