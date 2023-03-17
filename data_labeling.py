from machine import Pin, time_pulse_us
from time import sleep

pin_input = Pin(29, mode = Pin.IN) 	#A0 connected to PWM channel 5 of the FMU
pin_label = Pin(28, mode = Pin.OUT) 	#output pin A1 connected to GPIO7 from DevKits

pin_label.high()

#Checks to see if the duty cyle changes. If so, sets the output pin to GND.

while True: 
    value = time_pulse_us(pin_input, 1)
    
    if value > 1100:			
        pin_label.low()
        sleep(1)
    else:
        pin_label.high()
    sleep(0.1)