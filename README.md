# digital-altimeter-rp2
Digital Altimeter with adjustments for known altitude/sea-level-pressure using Raspberry Pi Pico2 with BMP390, BME680, OLED Display

## Features of this Raspberry Pi Pico 2 code:
* BMP390 (I2C1 #76 addr) - highly accurate air pressure, altitude.
  * used as primary sensor for air pressure and altitude
  * on same I2C bus as BME680, solder bump on sensor makes it #76 addr to have code address it distictly from bme680
* BME680 (I2C1 #77 addr) - for inside temp & humidity, air pressure, air quality, and altitude.
* SSD1309 (SDI) - uses SSD1306 SW, both SDI & I2C code - SDI is faster
  * ssd1306 Framebuffer-based SW for printing text & blit of bitmap images
* button debounce that uses efficient interrupt code (does not use CPU cycles with sleep, yay!)
* Temperature from on-board RP2350 (no external pins, ADC4)

## Useful sites:
* Powering Pico: https://www.youtube.com/watch?v=3PH9jzRsb5E -- feed, 5v for ultrasonics, and power in my car.
  * TODO get Buck converter 12v car to 5v (to usb-b or micro-usb)
  * For micro-USB and/or external power, best to have MosFET protection
    * recommended DMG-2305ux, but this is surface-mount MosFET
    * investigating RLB8721, that way can hook laptop up in car to update SW
* maybe going to use Foriot's TXS0108E 8 Channel Level Converter, 5v to Pico 3.3V signals, high speed converter (SDI) https://www.amazon.com/gp/product/B0CFL9KN7L

## Project images:
todo
 
## My code is Based on my Parking sensor
* https://github.com/bradcar/artcar-ultrasonic-dist-rp2

## Other useful sites (used in this code):
* MicroPython Fonts:  https://github.com/peterhinch/micropython-font-to-py/tree/master
* BMP390 driver code I'm using: https://github.com/octaprog7/BMP390
 
## TODOs
* Consider putting air quality status in tiny font
* add 3rd button to show details of all measurments
