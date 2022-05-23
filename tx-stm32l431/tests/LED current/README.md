# LED characterization

Currrent/Voltage plots for three and four LEDs in series.

The four LED string is more efficient at higher voltages but there's
not enough current when the batteries run down.  So, use three LEDs
and accept the wasted power at high voltages.

For an applied voltage in the range 4.0--6.5V (which covers the range
the battery will supply) the current is accurately modelled by:

	(I / mA) = 55.86 (V / V) - 200.58

Equivalently,

	(V / V) = 17.9 (I / A) + 3.59

Given that the current limiting resistor had a value of 15Ω this
suggests that the LED behaves as a voltage drop of 1.2V in series with
a 1Ω resistor.

# Model generation

The short Haskell program mkModel.hs generates C source and header files
to embed this model in firmware.

test.c is a simple program to test the model.

You can build them thus:

	stack runghc mkModel.hs

	gcc -Wall -std=c99 -pedantic test.c ledcurve.c -o test

	./test
