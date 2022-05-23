OBJS = ledcurve.o morseout.o

BINARY = main

LDSCRIPT = ../stm32l431.ld

include ../Makefile.include

morseout.c:	morse.cfg
		morse-tool

ledcurve.c:	../../led-model/ledcurve.c
		cp $< $@

ledcurve.h:	../../led-model/ledcurve.h
		cp $< $@
