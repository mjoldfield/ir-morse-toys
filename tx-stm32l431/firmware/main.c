/* ex https://github.com/libopencm3/libopencm3-miniblink/blob/master/template_stm32.c */

#include <stddef.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/flash.h>

#include "morseout.h"
#include "ledcurve.h"

typedef enum _gpio_pin_kind { GPIO_in, GPIO_out, GPIO_alt } gpio_pin_kind;

static const uint8_t gpio_out_set_hi  = 1;
static const uint8_t gpio_out_set_lo  = 2;

typedef struct _GPIOPin {
  uint16_t      pin;
  uint32_t      port;
  gpio_pin_kind kind;
  uint8_t       extra;
} GPIOPin;

// System clock
// 200kHz - 55.8uA, 400kHz - 68.5uA
static const uint32_t master_clk = RCC_CR_MSIRANGE_800KHZ;
static const uint32_t master_f   = 800000L; 

// carr   - 40kHz IR carrier clock
static const uint32_t              carr_freq       = 40000;
static const uint32_t              carr_timer      = TIM15;
static const uint32_t              carr_prescale   = 1;
static const uint32_t              carr_period     = master_f / (carr_freq * carr_prescale);


static const GPIOPin               leds_out = { GPIO2, GPIOA, GPIO_alt, GPIO_AF14 };

// tone_  - 800Hz Morse tone clock
static const uint32_t              tone_freq      = 800;
static const uint32_t              tone_timer     = TIM1;
static const uint32_t              tone_prescale  = 1;
static const uint32_t              tone_period    = master_f / (tone_freq * tone_prescale);

static const GPIOPin               carr_out = { GPIO8, GPIOA, GPIO_alt, GPIO_AF0 };   // MCO

// number of pulses in each burst of carrier, which lasts half the
// period of the tone
static const uint32_t              carr_pulses    = (carr_freq / tone_freq) >> 1;
// select the right master timer to get our trigger from
static const uint8_t               carr_trig_src  = TIM_SMCR_TS_ITR0;

//
// data - call after each symbol so 1,3,7,..  dit times
// 
// NB if this changes from TIM2 then references in the code below
// will have to be changed to match.
//
static const uint32_t              dit_time_ms    = 50;
static const uint32_t              data_timer     = TIM2;
static const uint32_t              data_prescale  = 5000;
static const uint32_t              data_period    = master_f * dit_time_ms /  (1000L * data_prescale);

// ADC for battery sense
static const uint32_t adc_batt      = ADC1;
static const uint32_t adc_batt_port = GPIOC;
static const uint16_t adc_batt_pin  = GPIO0;

static uint32_t v_batt = 0;

// Configuration data: read on boot, then fixed.
static uint32_t config = 0xffffffffL;
// LED power curve
static uint32_t led_curve_id = 0;

static const GPIOPin dbug_int      = { GPIO9, GPIOA, GPIO_out, gpio_out_set_lo };

// Config bits: read them, then set to output and drive LOW
// note the order is MSB first
static const GPIOPin configGPIOs[] = {
				         { GPIO9,   GPIOC, GPIO_in, GPIO_PUPD_PULLUP }
				       , { GPIO8,   GPIOC, GPIO_in, GPIO_PUPD_PULLUP }
				       , { GPIO7,   GPIOC, GPIO_in, GPIO_PUPD_PULLUP }
				       , { GPIO6,   GPIOC, GPIO_in, GPIO_PUPD_PULLUP }
				       , { GPIO15,  GPIOB, GPIO_in, GPIO_PUPD_PULLUP }
				       , { GPIO14,  GPIOB, GPIO_in, GPIO_PUPD_PULLUP }
				       , { GPIO13,  GPIOB, GPIO_in, GPIO_PUPD_PULLUP }
				       , { GPIO12,  GPIOB, GPIO_in, GPIO_PUPD_PULLUP }
					 , { 0, 0, 0, 0 }
};

// Morse code data
// 
// local mutable store for battery data
static const int morse_batt_status_n = 50;
static uint8_t morse_batt_status[50] = { 1,1,1,1,1, 3, 3,1,3,1,3, 3, 1,1,1,1,1, 0 };

static const uint8_t morse_message_pad_time = 20; // in dits

static uint32_t morse_msg_id = 0;


// how often should we update the battery voltage
// measured in complete passes through the message dictionary
//  - in practice this makes very little difference to current
// as configured below sampling takes ~1s, messages take ~30s
// so doing it every three messages means we read for ~1% of
// the time
static const uint32_t n_cycles_for_update = 3;
static volatile bool do_update = true;

//
// function prototypes
//
static void fatal_error(void);

static void init_std_timer(  uint32_t my_timer
			   , uint32_t my_prescale
			   , uint32_t my_period
			   , uint32_t my_period_off
			   );

static void init_gpio(GPIOPin const *my);
static void gpio_pulse_output(GPIOPin const *my);

static enum rcc_periph_clken clk_for_port(uint32_t port);
static enum rcc_periph_rst   rst_for_port(uint32_t port);

static void write_morse_number_check(volatile uint8_t *buff, uint32_t buffsize, uint32_t n);
static void write_morse_digit(volatile uint8_t *b, uint32_t d);

static void read_config(void);

static void reset_gpios(void);
static void reset_gpios_port(const uint32_t port, const uint16_t pins);

static void init_adc(const uint32_t adc
		     , const uint32_t port
		     , const uint16_t pins);


typedef uint8_t async_adc_state;
const async_adc_state async_adc_dormant = 0;
const async_adc_state async_adc_start   = 1;

static async_adc_state async_adc_convert(const   uint32_t adc
					 , const uint8_t  channel
					 , async_adc_state state
					 , uint32_t *result);

static void adc_set_regular_singleton(uint32_t adc, uint8_t channel);

static void set_duty_cycle(void);

static void init_clocks(void);

static void init_watchdog(void);
static void kick_watchdog(void);

static volatile bool int_enabled = false;

//
// here starteth the code
//
int main(void)
{
  rcc_periph_clock_enable(RCC_PWR);

  init_clocks();

  init_watchdog();

  // Read the config jumpers: messes with GPIO clock enables
  // so do this early.
  read_config();

  // Put all GPIOs in analog mode for power reduction
  reset_gpios();

  // explictly turn off LEDs
  {
    GPIOPin leds_tmp = leds_out;
    leds_tmp.kind  = GPIO_out;
    leds_tmp.extra = gpio_out_set_lo;
    init_gpio(&leds_tmp);
  }

  // 5 bits of message selection on 0--4
  // 3 bits of power curve on 5--7
  morse_msg_id = ((config >> 0) & 0x1f) ^ 0x1f;
  led_curve_id =  (config >> 5) & 0x07;

  ///////////////////////////////////////////////////////////////////////
  //
  // Carrier timer (40kHz) setup, taken from page 63 of AN4776
  // where it's called Timer 1. 
  //
  init_std_timer( carr_timer
		  , carr_prescale
		  , carr_period, carr_period >> 1
		  );

  timer_slave_set_trigger(carr_timer, carr_trig_src);
  timer_slave_set_mode(   carr_timer, TIM_SMCR_SMS_TM);

  timer_one_shot_mode(carr_timer);
  timer_set_repetition_counter(carr_timer, carr_pulses - 1);
  
  timer_enable_counter(carr_timer);

  ///////////////////////////////////////////////////////////////////////
  //
  // Tone timer setup (800Hz), taken from page 63 of AN4776
  // where it's called Timer 2
  //
  init_std_timer( tone_timer
		  , tone_prescale
		  , tone_period, tone_period >> 1
		  );

  timer_set_master_mode(tone_timer, TIM_CR2_MMS_UPDATE);
 
  timer_enable_counter(tone_timer);

  init_gpio(&leds_out);
  
  ///////////////////////////////////////////////////////////////////////
  // Status pins 
  //

  init_gpio(&carr_out);
  // put MCO on PA8
  RCC_CFGR |= (RCC_CFGR_MCO_SYSCLK << RCC_CFGR_MCO_SHIFT);
  //RCC_CFGR |= (RCC_CFGR_MCO_LSE << RCC_CFGR_MCO_SHIFT);

  ////////////////////////////////////////////////////////////////////////
  async_adc_state adc_state = async_adc_dormant;
  init_adc(adc_batt, adc_batt_port, adc_batt_pin);

  ///////////////////////////////////////////////////////////////////////
  //
  // Data timer setup: fire every dit time
  // -- enables interrupt so do this last.
  //
  init_std_timer( data_timer
		  , data_prescale
		  , data_period, 0
		  );

  timer_enable_irq(data_timer, TIM_DIER_UIE);
  nvic_enable_irq(NVIC_TIM2_IRQ);
  
  timer_enable_counter(data_timer);

  int_enabled = true;

  //
  // the main loop does very little beyond periodically reading the
  // battery voltage
  //
  // all the morse stuff is handled by hardware timers, updated by
  // the timer2 interrupt handler
  //
  while(1)
    {
      kick_watchdog();

      if (do_update) // flag set in ISR
	{
	  if (adc_state == async_adc_dormant)
	    adc_state = async_adc_start;

	  do_update = false;
	}

      if (adc_state != async_adc_dormant)
	{
	  const uint8_t adc_channel = 1;

	  uint32_t res = 99;
	  adc_state = async_adc_convert(adc_batt, adc_channel, adc_state, &res);

	  if (adc_state == async_adc_dormant)
	    {
	      v_batt = res;
	      write_morse_number_check(morse_batt_status,
				       morse_batt_status_n,
				       v_batt);
	      set_duty_cycle();
	    }
	}

      __asm__("wfi"); // sleep...

    }
}

void tim2_isr(void)
{
  // Assume clock speed 400kHz, so 2.5us per tick
  //
  // unoptimized code which calls libopencm3 takes time:
  //   150us to enable, 200us to disable
  // inlining direct register accesses takes
  //   105us to enable, 135us to disable
  //
  // All times are the measured time for which the dbug_pin
  // goes high.

  GPIO_BSRR(dbug_int.port) = dbug_int.pin;

  TIM2_SR &= ~TIM_SR_UIF;                                        // clear int flag in timer
  NVIC_ICPR(NVIC_TIM2_IRQ >> 5) = (1 << (NVIC_TIM2_IRQ & 0x1f)); // ... and in nvic
    

  static uint32_t phase = 0;
  static uint32_t msg_i = 0;
  const bool isMark = !(msg_i & 1);
    
  uint32_t tau = (phase == 0) ? morse_get_next_time(morse_msg_id + 1, &msg_i)
               : (phase == 1) ? morse_get_next_time(0,                &msg_i)
               :                morse_batt_status[msg_i++]
               ;

  if (tau == 0)
    {
      phase++;
      if (phase > 2)
	phase = 0;

      msg_i = 0;

      tau = morse_message_pad_time;

      static uint32_t i_cycles = 0;
      i_cycles++;
	  
      if (i_cycles == n_cycles_for_update)
	{
	  i_cycles = 0;
	  do_update = true;
	}
    }

    // work out how long next token should last
  const uint32_t period = data_period * tau - 1;
  if (isMark)
    {
      TIM_CR1(carr_timer) |= TIM_CR1_CEN; // enable timers
      TIM_CR1(tone_timer) |= TIM_CR1_CEN;
      
      TIM_ARR(data_timer) = period;
    }
  else
    {
      TIM_CR1(tone_timer) &= ~TIM_CR1_CEN; // disble timers
      TIM_CR1(carr_timer) &= ~TIM_CR1_CEN;
      
      TIM_ARR(data_timer) = period; 
      
      TIM_CNT(tone_timer) = 0;             // reset counts for next time
      TIM_CNT(carr_timer) = 0;
    }
  
  GPIO_BSRR(dbug_int.port) = dbug_int.pin << 16;
}

//
// Initialize a boring upward counting time with specified
// period and prescale value.
//
// If my_period_off is non-zero, also setup the OC1 output
// comparator.
//
static void init_std_timer(  uint32_t my_timer
			   , uint32_t my_prescale
			   , uint32_t my_period
			   , uint32_t my_period_off
			   )
{
  const enum rcc_periph_clken clk = clk_for_port(my_timer);
  const enum rcc_periph_rst   rst = rst_for_port(my_timer);
  
  rcc_periph_clock_enable(clk); 
  rcc_periph_reset_pulse(rst);

  timer_set_mode(my_timer
		 , TIM_CR1_CKD_CK_INT  // unity clock divider
		 , TIM_CR1_CMS_EDGE    // 
		 , TIM_CR1_DIR_UP      // count up
		 );

  timer_set_prescaler(my_timer, my_prescale - 1);
  timer_set_period(my_timer, my_period - 1);
  timer_set_counter(my_timer, 0);

  if (my_period_off != 0)
    {
      timer_set_oc_value(my_timer, TIM_OC1, my_period_off);
      timer_set_oc_mode( my_timer, TIM_OC1,TIM_OCM_PWM1);
      timer_set_oc_polarity_low(my_timer, TIM_OC1);

      timer_enable_oc_output(my_timer, TIM_OC1);
    }
      
  timer_enable_break_main_output(my_timer);
}

//
// Initialize a GPIO pin
//

static void init_gpio(GPIOPin const *my)
{
  enum rcc_periph_clken my_clk = clk_for_port(my->port);
  rcc_periph_clock_enable(my_clk);

  if      (my->kind == GPIO_in)
    {
      gpio_mode_setup(my->port, GPIO_MODE_INPUT, my->extra, my->pin);      
    }
  else if (my->kind == GPIO_out)
    {
      gpio_mode_setup(my->port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, my->pin);

      if      (my->extra == gpio_out_set_hi)
	{
	  gpio_set(my->port, my->pin);
	}
      else if (my->extra == gpio_out_set_lo)
	{
	  gpio_clear(my->port, my->pin);
	}
      else
	{
	  fatal_error();
	}
    }
  else if (my->kind == GPIO_alt)
    {
      gpio_mode_setup(my->port, GPIO_MODE_AF, GPIO_PUPD_NONE, my->pin);
      gpio_set_af(my->port, my->extra, my->pin);
    }
  else
    {
      fatal_error();
    }
}

static void gpio_pulse_output(GPIOPin const *my)
{
  if (my->kind != GPIO_out)
    {
      fatal_error();
    }

  gpio_toggle(my->port, my->pin);
  gpio_toggle(my->port, my->pin);
}


//
// Initialize ADC
//
static void init_adc(const uint32_t adc
		     , const uint32_t port
		     , const uint16_t pins)
{
// ADC clock in AHB
  const enum rcc_periph_clken adc_clk  = clk_for_port(adc);
  rcc_periph_clock_enable(adc_clk);

  RCC_CCIPR   |= (3UL << 28);  // select system clock over PLLADC

  const enum rcc_periph_clken gpio_clk = clk_for_port(port);
  rcc_periph_clock_enable(gpio_clk);

  gpio_mode_setup(port, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, pins);
}

//
// Do the next step in getting a number from the ADC
// - you'll need to thread the state through calls;
// - all bets are off if adc/channel change between calls
//
static async_adc_state async_adc_convert(const   uint32_t adc
					 , const uint8_t  channel
					 , const async_adc_state state
					 , uint32_t *result)
{
  bool finished = false;
  
  switch(state)
    {
    case 0:
      finished = true;

      break;
      
    case 1:
      ADC_CR(adc) &= ~ADC_CR_DEEPPWD;
      ADC_CR(adc) |=  ADC_CR_ADVREGEN;

      break;

    case 2:
      // adc_calibrate_async(adc); <- buggy
      ADC_CR(adc) |= ADC_CR_ADCAL;

      while(adc_is_calibrating(adc));

      adc_power_on(adc);

      adc_set_regular_singleton(adc, channel);
      
      adc_set_sample_time(adc, channel, ADC_SMPR_SMP_12DOT5CYC);

      ADC_CFGR2(adc) |= (3UL << 2) | 1UL; // 16x oversampling
      
      adc_start_conversion_regular(adc);

      break;
    
    case 3:
      
      while (!adc_eoc(adc))
	;

      uint32_t r = adc_read_regular(adc); // max is 16 * 4095

      // 3301 is 2^16 * 3300 / (16 * 4095)
      // 3300 is Vref is mV
      *result = (r * 3301UL) >> 16;

      adc_power_off(adc);

      ADC_CR(adc) |= ADC_CR_DEEPPWD;      
      
      finished = true;

      break;
    }

  const async_adc_state new_state = (finished)
                     ? async_adc_dormant
                     : state + 1;

  return new_state;
}

static void adc_set_regular_singleton(uint32_t adc, uint8_t channel)
{
  ADC_SQR1(adc) = ((uint32_t)channel) << ADC_SQR1_SQ1_SHIFT;
}


//
// look at the status of GPIO lines and write the config
// global byte
//
static void read_config(void)
{
  for(GPIOPin const *p = configGPIOs; p->pin != 0; p++)
    {
      const enum rcc_periph_clken clk = clk_for_port(p->port);

      rcc_periph_clock_enable(clk);

      // read the pin: might be tied low or floating
      init_gpio(p);
      const uint32_t dc = gpio_get(p->port, p->pin) == 0 ? 0 : 1;

      config <<= 1;
      config |=  dc;
    }
}

//
// Put all unused GPIOs into a sensible state.
// - in practice put all GPIOs there and reconfigure as needed
// - leave the SWD/JTAG stuff out so we can easily flash new
//   firmware
//
static void reset_gpios()
{
  reset_gpios_port(GPIOA, GPIO_ALL & !(GPIO13 | GPIO14));
  reset_gpios_port(GPIOB, GPIO_ALL & !(GPIO3));
  reset_gpios_port(GPIOC, GPIO_ALL);
  reset_gpios_port(GPIOD, GPIO2);
  reset_gpios_port(GPIOH, GPIO0 | GPIO1 | GPIO3);
}

static void reset_gpios_port(const uint32_t port, const uint16_t pins)
{
  const enum rcc_periph_clken clk = clk_for_port(port);
  
  rcc_periph_clock_enable(clk);

  //gpio_mode_setup(port, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, pins);
  //gpio_mode_setup(port, GPIO_MODE_ANALOG, GPIO_PUPD_PULLDOWN, pins);
  gpio_mode_setup(port, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, pins);

  rcc_periph_clock_disable(clk);
}

//
// Given a buffer of buffsize write the Morse representation
// of n into it. Pad with leading zeros, and if the number's
// too big show the least significant digits only.
//
// Also add a check digit at the end so that the sum of all
// digits mod 10 is zero.
//
static void write_morse_number_check(volatile uint8_t *buff, uint32_t buffsize, uint32_t n)
{
  const uint32_t n_digits = buffsize / 10 - 1;

  uint8_t eoc = 3; // change to 7 for space after check

  uint32_t csum = 0;
  for(uint32_t i = 0; i < n_digits; i++)
    {
      const uint32_t d  = n % 10;
      n /= 10;

      csum += (10 - d);

      const uint32_t offset = 10 * (n_digits - i - 1);
      
      write_morse_digit(buff + offset, d);
      buff[offset + 9] = eoc;

      eoc = 3; // dah bettween digits
    }

  const uint32_t offset = 10 * n_digits;
  write_morse_digit(buff + offset, csum % 10);
  buff[offset + 9] = 0;
}

static void write_morse_digit(volatile uint8_t *b, uint32_t d)
{
  
  uint8_t  x1st = 1;
  uint8_t  x2nd = 3;
  uint32_t n1st = d;

  if (d >= 5)
    {
      x1st  = 3;
      x2nd  = 1;
      n1st -= 5;
    }

  for(uint32_t i = 0; i < 5; i++)
    {
      *b++ = (i < n1st) ? x1st : x2nd;
      *b++ = 1;
    }
}

//
// Convenience functions to return the clock and reset
// registers for various ports.
//
// It's a failing that we can't easily do something sensible
// if the port isn't handled by the switch.
//
static enum rcc_periph_clken clk_for_port(uint32_t port)
{
  switch(port)
    {
    case GPIOA:
      return RCC_GPIOA;
    case GPIOB:
      return RCC_GPIOB;
    case GPIOC:
      return RCC_GPIOC;
    case GPIOD:
      return RCC_GPIOD;
    case GPIOH:
      return RCC_GPIOH;
    case TIM1:
      return RCC_TIM1;
    case TIM2:
      return RCC_TIM2;
    case TIM15:
      return RCC_TIM15;
    case ADC1:
      return RCC_ADC;
    }
  return 0;
}

static enum rcc_periph_rst rst_for_port(uint32_t port)
{
  switch(port)
    {
    case TIM1:
      return RST_TIM1;
    case TIM2:
      return RST_TIM2;
    case TIM15:
      return RST_TIM15;
    }
  return 0;
}

static void init_clocks(void)
{
  // wait for MSI to be ready --- must do this before changing rate
  while(!(RCC_CR & RCC_CR_MSIRDY))
    ;
  
  // set MSI range
  {
    const uint32_t msi_range = master_clk;
    
    uint32_t rcc_cr = RCC_CR;    
    rcc_cr &= ~(RCC_CR_MSIRANGE_MASK << RCC_CR_MSIRANGE_SHIFT);
    rcc_cr |= msi_range << RCC_CR_MSIRANGE_SHIFT;
    rcc_cr |= RCC_CR_MSIRGSEL;
    RCC_CR = rcc_cr;
  }

  // Set MSI after reset range
  {
    const uint32_t msi_range = master_clk;
    
    uint32_t rcc_csr = RCC_CSR;    
    rcc_csr &= ~(RCC_CSR_MSIRANGE_MASK << RCC_CSR_MSIRANGE_SHIFT);
    rcc_csr |= msi_range << RCC_CSR_MSIRANGE_SHIFT;
    RCC_CSR = rcc_csr;
  }

  // Turn on LSI for IWDG
  RCC_CSR |= RCC_CSR_LSION;
  
  // wait for MSI to be ready
  while(!(RCC_CR & RCC_CR_MSIRDY))
    ;

  // wait for LSI to be ready
  while(!(RCC_CSR & RCC_CSR_LSIRDY))
    ;
  
  // Low power run mode
  PWR_CR1 |= PWR_CR1_LPR;
}

static void init_watchdog(void)  
{
  IWDG_KR = 0xcccc;
  IWDG_KR = 0x5555;
  IWDG_PR = IWDG_PR_DIV16;         // 16x prescale => 2s timeout

  while (IWDG_SR != 0) // wait until writes have finished
    ;

  //
  // The watchdog can be enabled by a bit in the option bytes
  // (FLASH_OPTR_IDWG):
  //    0 -> hardware mode (starts on boot)
  //    1 -> software mode (needs config above)
  //
  // Make sure that the bit's clear so we get start early.
  //

  const bool hw_wdg = (FLASH_OPTR & FLASH_OPTR_IDWG_SW) == 0;



  const uint32_t boot0_mask = 3 << 26;
  const uint32_t boot0_val  = 2 << 26;
  const bool boot_config_ok = (FLASH_OPTR & boot0_mask) == boot0_val;

  if (!hw_wdg || !boot_config_ok)
    {
      // the libopencm3 flash_program_option_bytes() call
      // does odd things (clearing the lowest two bits)
      // This code follows page 83 of the 32L4xxx ref. manual

      // wait until flash operations have finished
      while ((FLASH_SR & FLASH_SR_BSY) == FLASH_SR_BSY)
	;

      // write magic numbers to unlock flash
      FLASH_KEYR    = 0x45670123;
      FLASH_KEYR    = 0xCDEF89AB;
      
      FLASH_OPTKEYR = 0x08192A3B;
      FLASH_OPTKEYR = 0x4C5D6E7F;

      // clear relevant bit for watchdog
      FLASH_OPTR &= ~FLASH_OPTR_IDWG_SW;

      // tweak bits for boot
      FLASH_OPTR &= ~boot0_mask;
      FLASH_OPTR |=  boot0_val;

      // flag we're done
      FLASH_CR   |= FLASH_CR_OPTSTRT;

      // wait until done
      while ((FLASH_SR & FLASH_SR_BSY) == FLASH_SR_BSY)
	;
    }
}

static void set_duty_cycle(void)
{
  // at 800kHz have 20 clock ticks per 40kHz cycle, so sensible
  // off times are 10, 11, .. 19
  // 10 is full-power, 19 is minimal power

  // if we don't have a sane battery voltage, turn on full
  // power and bail early.

  uint32_t carr_period_off = 10;
  
  if (v_batt != 0)
    {
      const uint32_t dc = calc_duty_cycle(1, 10, led_curve_id, v_batt);
      carr_period_off = 20 - dc;
    }

  timer_set_oc_value(carr_timer, TIM_OC1, carr_period_off);
}


static void kick_watchdog(void)
{
  IWDG_KR = 0xaaaa;
}

static void fatal_error(void)
{
  while(1)
    ;
}
