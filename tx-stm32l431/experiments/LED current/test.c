#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "ledcurve.h"


int main(void)
{
  const uint32_t dc_min =  1;
  const uint32_t dc_max = 10;
  
  for(int i = 0; i < n_led_curves; i++)
    {
      printf("Curve %d\n", i);

      uint32_t old_dc  = 0;
      uint16_t v_start = 0;
      bool range_valid = false;

      for(uint16_t v = 5000; v != 0; v--)	
	{
	  const uint32_t dc = calc_duty_cycle(dc_min, dc_max, i, v);

	  if (!range_valid || dc != old_dc)
	    {
	      if (range_valid)
		{
		  printf("  %2d: %5d - %5d\n", old_dc, v_start, v);
		}

	      v_start = v;
	      old_dc  = dc;
	      range_valid = true;
	    }
	}

      if (range_valid)
	{
	  printf("  %2d: %5d - %5s\n", old_dc, v_start, "  -  ");
	}
      
    }

  return(0);
}  
	  

  
