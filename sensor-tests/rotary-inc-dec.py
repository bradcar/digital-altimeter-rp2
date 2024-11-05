# rotary encoder
# Adjust new_alt_feet by +/- 1ft per dedent, if push button adjust by 25 ft
#
# into lib save rotary_irq_rp2.py
# into lib save rotary.py
#
# by bradcar

import time
from machine import Pin
from rotary_irq_rp2 import RotaryIRQ

rotary = RotaryIRQ(pin_num_clk=6, pin_num_dt=7, min_val=0, reverse=False, \
                range_mode=RotaryIRQ.RANGE_UNBOUNDED)
rotary_switch = Pin(8, Pin.IN, Pin.PULL_UP)

debug = True
new_alt_feet = 365

rotary_multiplier = 1
rotary_old = rotary.value()
while True:
    rotary_new = rotary.value()
    if rotary_switch.value() == 0:
        rotary_multiplier = 1 if rotary_multiplier != 1 else 25
        while rotary_switch.value() == 0:
            continue

    if rotary_old != rotary_new:
        # change in value since last loop then scale by multiplier
        delta = rotary_new - rotary_old 
        new_alt_feet = new_alt_feet + delta*rotary_multiplier
        rotary_old = rotary_new
        if debug: print(f"{new_alt_feet=}")

    time.sleep_ms(50)
