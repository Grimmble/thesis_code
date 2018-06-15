# Thesis code

This repository contains the code used by 2 MCU's (Arduino Micro and Sparkfun Pro Micro) as part of the main thesis project at University of Agder. The code allows reading a Sparkfun Loadcell Amplifier HX711 unit that samples a Wheatstone bridge strain gauge, filter the readings and process with a PI controller algorithm. The controller output is then transmitted with the I^2C bus to MCU2 which controlles and commutates a BLDC motor based on PPR encoder interpretation.
