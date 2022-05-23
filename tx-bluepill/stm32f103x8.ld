/*
 * Memory map for STM32F103x8
 *  
 * See https://www.st.com/resource/en/datasheet/stm32f103c8.pdf
 *
 */ 				 

/*
 * NOTE:
 * The 103x8 parts have  64k of flash
 *     103xB parts have 128k of flash
 */

MEMORY
{
	rom (rx)  : ORIGIN = 0x08000000, LENGTH = 64K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

INCLUDE cortex-m-generic.ld

