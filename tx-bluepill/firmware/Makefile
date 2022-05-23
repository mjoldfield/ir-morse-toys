OBJS = morseout.o

BINARY = main

LDSCRIPT = ../stm32f103x8.ld

include ../Makefile.include

morseout.c:     morse.cfg
		morse-tool
