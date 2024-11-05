# rotary encoder
#
# into lib save rotary_irq_rp2.py
# into lib save rotary.py
#
# https://circuitdigest.com/microcontroller-projects/raspberry-pi-pico-interfacing-with-rotary-encoder

import time
from machine import Pin
from rotary_irq_rp2 import RotaryIRQ


SW = Pin(8, Pin.IN, Pin.PULL_UP)
r = RotaryIRQ(pin_num_clk=6,
              pin_num_dt=7,
              min_val=0,
              reverse=False,
              range_mode=RotaryIRQ.RANGE_UNBOUNDED)

val_old = r.value()
while True:
    try:
        value_new = r.value()
        if SW.value() == 0:
            print("Button Pressed")
            print("Selected Number is : ", value_new)

            while SW.value() == 0:
                continue

        if val_old != value_new:
            val_old = value_new
            print(f"{value_new=}")

        time.sleep_ms(50)
    except KeyboardInterrupt:
        break
