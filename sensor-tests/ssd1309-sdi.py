# Raspberry Pi Pico 2 project - ArtCar ultrasonic - Oct 2024
#
# SSD1309 OLED Display - 1306 SW: SDI driver  print, bitmaps, & drawing to frameBuffer
# i2c also included commented out
# by bradcar
#
# https://micropython.org/download/RPI_PICO2/ for latest .uf2 preview
# https://docs.micropython.org/en/latest/esp8266/tutorial/ssd1306.html

from machine import Pin
import machine
from os import uname
from sys import implementation
from time import sleep as zzz

#ic2
#from machine import I2C
#from ssd1306 import SSD1306_I2C

from ssd1306 import SSD1306_SPI
from framebuf import FrameBuffer, MONO_HLSB

# Constants and setup
DISP_WIDTH=128
DISP_HEIGHT=64

# === PINS ===

# https://www.tomshardware.com/how-to/oled-display-raspberry-pi-pico
# i2c=I2C(0,sda=Pin(0), scl=Pin(1), freq=400000)
# oled = SSD1306_I2C(DISP_WIDTH, DISP_HEIGHT, i2c)

# https://coxxect.blogspot.com/2024/10/multi-ssd1306-oled-on-raspberry-pi-pico.html
cs  = machine.Pin(9)    #dummy (any un-used pin), no connection
# scl (SCLK) gp10 
# SDA (MOSI) gp11
res = machine.Pin(12)   # RES (RST)  gp12
dc  = machine.Pin(13)   # DC         gp13

# on-board led
led = Pin(25, Pin.OUT)


oled_spi = machine.SPI(1)
oled = SSD1306_SPI(DISP_WIDTH, DISP_HEIGHT, oled_spi, dc, res, cs)
print("oled_spi:", oled_spi)

# DISPLAY IMAGES
# image2cpp (convert png into C code): https://javl.github.io/image2cpp/
# const unsigned char bitmap_artcar_image[] PROGMEM = {0xc9,0x04,0x52, ...
# can also be bitmap_artcar_image=bytearray(b'\xc9\x04\x59
#
# 'art-car-imag', 56x15px
bitmap_artcar_image_back = bytearray([
  0xc9,0x04,0x52,0x91,0x0c,0x08,0x43,0xc8,0x82,0x8e,0x90,0x93,0x10,0x93,0xe0,0x63, 
  0x08,0x78,0xe0,0xe3,0x07,0xff,0x9f,0xff,0xff,0xff,0xfc,0xff,0xc0,0x00,0x00,0x00, 
  0x00,0x00,0x03,0x40,0x1c,0xf3,0xe3,0x8e,0x78,0x02,0x42,0x22,0x88,0x84,0x51,0x44, 
  0x42,0x42,0x22,0x88,0x84,0x11,0x44,0x42,0x42,0x22,0xf0,0x84,0x11,0x78,0x42,0x22, 
  0x3e,0x88,0x84,0x1f,0x44,0x44,0x10,0x22,0x88,0x84,0x51,0x44,0x08,0x0c,0x22,0x88, 
  0x83,0x91,0x44,0x30,0x03,0x00,0x00,0x00,0x00,0x00,0xc0,0x00,0xff,0xff,0xff,0xff, 
  0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
])

# startup code
print("Starting...")
print("====================================")
print(implementation[0], uname()[3],
      "\nrun on", uname()[4])
print("====================================")

i = 0

# main loop
while True:
    oled.fill(0)
    oled.text(f"i= {i}", 0, 0)
    
    oled.blit(FrameBuffer(bitmap_artcar_image_back,56,15, MONO_HLSB), 16, 9)
    
    # fill_rect(x, y, w, h, color)
    oled.fill_rect(2, 24, 80, 38, 1)
    oled.fill_rect(78, 26, 2, 24, 0)

    # integer convert to string
    int_string = str(56)
    digits = 2
    
    # first draw black retangle, then the number inside
    oled.fill_rect(17, 34-1, 8*digits, 9, 0)
    oled.text(int_string, 17, 34)
    
    # print black text
    oled.text(f"i={i}", 24, 50, 0)

    oled.show()

    # Every loop do this
    i += 1
    print(f"i= {i}")
    led.toggle()
    zzz(1)
