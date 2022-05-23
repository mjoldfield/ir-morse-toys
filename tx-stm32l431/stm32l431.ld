/*
 * Memory map for STM32L431
 *  
 * See https://www.st.com/en/microcontrollers-microprocessors/stm32l431rc.html
 *
 */ 				 

/*
 * NOTE:
 * The RAM is really 16k of special parity checked stuff
 * plus 48k of the ordinary sort, but we can treat it as a
 * single 64k chunk, so do that for now.
 */

MEMORY
{
	rom (rx)  : ORIGIN = 0x08000000, LENGTH = 256K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
}

INCLUDE cortex-m-generic.ld

