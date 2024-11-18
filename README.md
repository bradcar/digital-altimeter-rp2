# digital-altimeter-rp2
Digital Altimeter with adjustments for known altitude/sea-level-pressure using Raspberry Pi Pico2 with BMP581(or BM390), BME680, OLED Display

## Features of this Raspberry Pi Pico 2 code:
* BMP581 (I2C1 #47 addr) - latest generation highly accurate air pressure, temp, and altitude.
  * used as primary sensor for air pressure and altitude
  * now uses my driver code for quick switch between bmp585, bmp581, bmp390
    * https://github.com/bradcar/MicroPython_BMP58x
  * BMP581 is on the same I2C bus as BME680
* BME680 (I2C1 #77 addr) - for inside temp & humidity, air pressure, air quality, and altitude.
* Large 20px fonts.
* SSD1309 (SDI) - uses SSD1306 SW, both SDI & I2C code - SDI is faster
  * ssd1306 Framebuffer-based SW for printing text & blit of bitmap images
* button debounce that uses efficient interrupt code (does not use CPU cycles to spin/wait on button debounce, yay!)
  * Has 3 buttons (1. set/adjust alt/sea-level-pressure, 2. F/Celsius, 3. Big Font Summary or detail summary in small font)
* Temperature from on-board RP2350 (no external pins, ADC4)

Deprecated but easy to switch back to:
* BMP390 (I2C1 #76 addr) - previous generation air pressure, temp, and altitude.
  * used as primary sensor for air pressure and altitude
  * on same I2C bus as BME680, solder bump on sensor makes it #76 addr to have code address it distictly from bme680

## Useful sites:
* Powering Pico: https://www.youtube.com/watch?v=3PH9jzRsb5E -- feed, 5v for ultrasonics, and power in my car.
  * TODO get Buck converter 12v car to 5v (to usb-b or micro-usb)
  * For micro-USB and/or external power, best to have MosFET protection
    * recommended DMG-2305ux, but this is surface-mount MosFET
    * investigating RLB8721, that way can hook laptop up in car to update SW

## Project images:
todo
 
## My code is Based on my Parking sensor
* https://github.com/bradcar/artcar-ultrasonic-dist-rp2

## Other useful sites (used in this code):
* MicroPython Fonts of various sizes:  https://github.com/peterhinch/micropython-font-to-py/tree/master
* My driver code for quick switch between bmp585, bmp581, bmp390
    * https://github.com/bradcar/MicroPython_BMP58x
 
## TODOs
* try bmp585
