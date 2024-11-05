# Raspberry Pi Pico 2 project - ArtCar ultrasonic - Oct 2024
#
# Two Button debouncer with efficient interrupts, which don't take CPU cycles!
# Based on one-button
# https://electrocredible.com/raspberry-pi-pico-external-interrupts-button-micropython/
# by bradcar

from machine import Pin
import time
from os import uname
from sys import implementation

interrupt_1_flag=0
interrupt_2_flag=0
debounce_1_time=0
debounce_2_time=0

# === PINS ===
button_1 = Pin(5, Pin.IN, Pin.PULL_UP)  #interrupt cm/in button pins
button_2 = Pin(6, Pin.IN, Pin.PULL_UP)  #interrupt rear/front button pins
led = Pin(25, Pin.OUT)

flag_1 = True
flag_2 = False

# ==== Original 1 button callback implementation ===
# def callback(pin):
#     global interrupt_1_flag, debounce_1_time
#     if (time.ticks_ms()-debounce_1_time) > 500:
#         interrupt_1_flag = 1
#         debounce_1_time=time.ticks_ms()
#         
# button_1.irq(trigger=Pin.IRQ_FALLING, handler=callback)

def callback(pin):
    global interrupt_1_flag, interrupt_2_flag, debounce_1_time, debounce_2_time
    if pin == button_1 and (int(time.ticks_ms()) - debounce_1_time) > 500:
        interrupt_1_flag = 1
        debounce_1_time = time.ticks_ms()
    elif pin == button_2 and (int(time.ticks_ms()) - debounce_2_time) > 500:
        interrupt_2_flag = 1
        debounce_2_time = time.ticks_ms()

button_1.irq(trigger=Pin.IRQ_FALLING, handler=callback)
button_2.irq(trigger=Pin.IRQ_FALLING, handler=callback)

# startup code
print("Starting...")
print("====================================")
print(implementation[0], uname()[3],
      "\nrun on", uname()[4])
print("====================================")

while True:
    # Button 1: flag_1
    if interrupt_1_flag == 1:
        interrupt_1_flag = 0
        print("button 1 Interrupt Detected")
        flag_1 = not flag_1  # Toggle between metric and imperial units
        led.toggle()

    # Button 2: flag_2
    if interrupt_2_flag == 1:
        interrupt_2_flag = 0
        print("button 2 Interrupt Detected")
        flag_3 = not flag_2   # Toggle between rear /front
        led.toggle()
