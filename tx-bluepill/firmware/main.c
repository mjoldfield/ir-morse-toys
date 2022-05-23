/* ex https://github.com/libopencm3/libopencm3-miniblink/blob/master/template_stm32.c */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include <libopencm3/cm3/nvic.h>     // for sys_tick_handler decl
#include <libopencm3/cm3/systick.h>

#include "morseout.h"

// Main IR LED on daughter board
// Use PB9 which is a 5V tolerant pin, and connect
// LEDs between +5V and PB9.
const uint32_t ir_led_pin   = GPIO9;
const uint32_t ir_led_port  = GPIOB;
const uint32_t ir_led_rcc   = RCC_GPIOB;

// Small green LED on Blue pill
const uint32_t grn_led_pin  = GPIO13;
const uint32_t grn_led_port = GPIOC;
const uint32_t grn_led_rcc  = RCC_GPIOC;
const uint32_t grn_led_ms   = 500;

// message select button
const uint32_t button_pin   = GPIO8;
const uint32_t button_port  = GPIOB;
const uint32_t button_rcc   = RCC_GPIOB;

// clock frequency: must match rcc_clock_setup line below
const uint32_t ahb_freq     = 72000000;

// times for Morse
const uint32_t carr_freq    =    40000;
const uint32_t subc_freq    =      800;
const uint32_t dit_t_ms     =       50;
const uint32_t space_t_ms   =      500;

// times for button handler
const uint32_t debounce_ms  =       25;
const uint32_t lng_prss_ms  =    10000;

// implicitly assumed by systick handler
const uint32_t tick_freq    =    carr_freq << 1;

// pre-calculate counts: factor of two because we do two transitions
// for each sub-carrier cycle
const uint32_t spc_cnt_limit  = ((subc_freq * space_t_ms)  / 1000) << 1;
const uint32_t dit_cnt_limit  = ((subc_freq * dit_t_ms)    / 1000) << 1;
const uint32_t debounce_limit = ((subc_freq * debounce_ms) / 1000) << 1;
const uint32_t lng_prss_limit = ((subc_freq * lng_prss_ms) / 1000) << 1;
const uint32_t grn_led_limit  = ((subc_freq * grn_led_ms)  / 1000) << 1;

static inline void real_sys_tick_handler(void);
static inline void read_button(bool *ret_pressed, uint32_t *ret_t_down);

int main(void)
{
  // Use libopencm3's clock routine: it's a bit involved
  // to do by hand.
  // this must match the ahb_freq defined above
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

  // IR LED
  rcc_periph_clock_enable(ir_led_rcc);
  gpio_set_mode(ir_led_port, GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_OPENDRAIN, ir_led_pin);

  // Green LED
  rcc_periph_clock_enable(grn_led_rcc);
  gpio_set_mode(grn_led_port, GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL, grn_led_pin);

  // push button
  rcc_periph_clock_enable(button_rcc);
  gpio_set_mode(button_port, GPIO_MODE_INPUT,
		GPIO_CNF_INPUT_PULL_UPDOWN, button_pin);
  GPIO_ODR(button_port) |= button_pin; // enable pull-up

  // Use SysTick to generate 80kHz interruprs
  // See section 4.5 of PM0056: STM32F Cortex-M3 prog. manual
  // Macro names match libopencm3
  const uint32_t n_systicks = ahb_freq / tick_freq;

  STK_RVR  = n_systicks - 1;
  STK_CVR  = 0;
  STK_CSR |= STK_CSR_CLKSOURCE_AHB | STK_CSR_TICKINT | STK_CSR_ENABLE;

  // Now just sleep: everything of interest is in the sys_tick handler.
  while(1) {
    __asm__("wfi");
  }
}

void sys_tick_handler(void)
{
  // real handler can bail early, so wrap it here to be
  // sure we measure time properly

  //GPIO_BSRR(grn_led_port) = grn_led_pin << 16;
  real_sys_tick_handler();
  //GPIO_BSRR(grn_led_port) = grn_led_pin;
}

static inline void real_sys_tick_handler(void)
{
  // we get here every 1 / 80kHz = 12.5us
  // don't add code with varying runtime before setting
  // GPIO pin below, lest we introduce jitter

  static uint32_t state  = 0;
  static uint32_t msg_id = 0;

  // if all low order bits are clear then turn LED on...
  const bool      led_on = (state & 7) == 0;

  // .. which means clearing the relevant bit
  {
    const uint32_t mask = (led_on) ? (ir_led_pin << 16) : ir_led_pin;
    GPIO_BSRR(ir_led_port) = mask;
  }

  // modulate 40kHz carrier
  state ^= 1; // tick...

  // now modulate with 800Hz sub-carrier
  {
    const uint32_t tone_cnt_limit = carr_freq / subc_freq;
    static uint32_t tone_cnt = 0;
    tone_cnt++;
    if (tone_cnt < tone_cnt_limit)
      return;
    tone_cnt = 0;
  }
  state ^= 2;

  // we get here every 1 / 1600 =  625us
  {
    static uint32_t grn_led_cnt   = 0;
    static bool     grn_led_state = false;
    const  uint32_t cnt_limit     = grn_led_limit;
    if (grn_led_cnt++ == cnt_limit)
      {
	grn_led_cnt   = 0;
	grn_led_state = !grn_led_state;
	const uint32_t mask = (grn_led_state) ? (grn_led_pin << 16) : grn_led_pin;
	GPIO_BSRR(grn_led_port) = mask;
      }
  }

  static bool key_mode = false;

  bool     button_press;
  uint32_t t_button_down;
  read_button(&button_press, &t_button_down);

  if (t_button_down > lng_prss_limit)
    key_mode = true;

  if (key_mode)
    {
      state = (state & ~4) | (t_button_down == 0 ? 4 : 0);
      return;
    }

  // now add data modulation
  static uint32_t data_cnt_limit = spc_cnt_limit;
  static uint32_t data_cnt       = 0;

  static uint32_t msg_i = 0;

  if (button_press)
    {
      msg_id++;
      msg_i = 0;

      state |= 4;
      data_cnt       = 0;
      data_cnt_limit = spc_cnt_limit;
    }

  data_cnt++;
  if (data_cnt < data_cnt_limit)
    return;

  data_cnt = 0;

  // check msg_id lies within correct bounds
  const uint32_t msg_id_min  = 0;
  const uint32_t msg_id_max  = n_morse_messages;
  if (msg_id <  msg_id_min || msg_id >= msg_id_max)
    {
      msg_id = msg_id_min;
    }

  const uint32_t n_dits = morse_get_next_time(msg_id, &msg_i);
  if (n_dits == 0)
    {
      state |= 4;
      data_cnt_limit = spc_cnt_limit;
    }
  else
    {
      state ^= 4;
      data_cnt_limit = dit_cnt_limit * n_dits;
    }
}

static inline void read_button(bool *ret_pressed, uint32_t *ret_t_down)
{
  static uint32_t  t_down  = 0;
  static uint32_t debounce_timer = 0;

  const uint16_t b       = GPIO_IDR(button_port) & button_pin;
  const bool     is_down = (b == 0);

  bool pressed = false;

  if (is_down)
    {
      t_down++;

      if (debounce_timer == 0)
	{
	  pressed = true;
	}

      debounce_timer = debounce_limit;
    }
  else
    {
      t_down = 0;

      if (debounce_timer > 0)
	{
	  debounce_timer--;
	}
    }

  *ret_pressed = pressed;
  *ret_t_down  = t_down;
}
