# Raspberry Pi Pico 2 project - Potentiometer - Oct 2024
#
# 10K ohms is a good balance of current vs. noise
#
# Potentiometer wiring
# * middle pot pin to Pin 34 adc2 gp28 
# * one side pot to 3v3_en for reference 3.3v
# * other side pot to agnd pin 33 reference ground
#
# See about +/- 0.6% temporal variation for pot at fixed setting
#
#  by bradcar

import time
from os import uname
from sys import implementation
from time import sleep as zzz

from machine import Pin, ADC


# ADC3 signal = gp28:  Pin 33 AGND left , 34 ADC3(middle pin), Vcc right pin
pot = ADC(Pin(28))

led = Pin(25, Pin.OUT)


def read_pot():
    adc_value = pot.read_u16()
    volt = (3.3 / 65535) * adc_value
    percent = (volt / 3.3)
    return percent


# =========================  startup code =========================

print("Starting...")
print("====================================")
print(implementation[0], uname()[3],
      "\nrun on", uname()[4])
print("====================================")


print("start of main loop\n")
while True:
    percent = read_pot()
    print(f"{(percent*100.0)=} %")
            
    # in order to return linear scale between two values ex: 970 to 1040
    # range = (1040-970) = 70
    # offset = 970  aka lowest value
    #
    # (percent * 70.0) + 970  gives values in [970 - 1040] range
    x = (percent * 70.0) + 970.0
    print (f"{x=}\n")

    zzz(.5)

