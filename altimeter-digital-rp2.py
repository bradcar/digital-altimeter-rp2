# Raspberry Pi Pico 2: Altimeter = Elevation & sea level pressure adjust - Nov 2024
#
# Sensors used
#    - BMP390 highly accurate pressure & altitude
#    - BME680 temp, humidity, pressure, IAQ, altitude
#    - rotary encoder to adjust alt & pressure
#      use sea level pressure at nearest airport
#    - ssd1309 SDI 128x64 OLED Display (SW is ssd1306)
#    - in/cm F/C changed with button #1
#    - buttons debounced with efficient rp2 interrupts -- nice!
#
# Use nearest airport for sea level pressure
#    Portland updated hourly (7 min before the hour)
#        https://www.weather.gov/wrh/timeseries?site=KPDX
#
#    my home office is
#        365.0 feet elevation, 111.25m
#    my dining room table
#        355 feet elevation (-10')
#    my garage is at <todo> feet elevation
#        339 feet elevation (-26')
#
# Key links
#     https://micropython.org/download/RPI_PICO2/ for latest .uf2 preview
#     https://docs.micropython.org/en/latest/esp8266/tutorial/ssd1306.html
#
# pycharm stubs
#    https://micropython-stubs.readthedocs.io/en/main/packages.html#mp-packages
#
# TODOs
#  * test error code of bmp & bme with either/both disconnected
#
# by bradcar

import time
from math import log
from os import uname
from sys import implementation
from time import sleep as zzz

import bmp390
from sensor_pack.bus_service import I2cAdapter
from bme680 import BME680_I2C
from framebuf import FrameBuffer, MONO_HLSB
from machine import Pin, I2C

# Peter Hinch fonts: https://github.com/peterhinch/micropython-font-to-py
# short_writer Code modified by: Charlotte Swift
import freesans20 as font_20px   # 20px high
from writer_short import Writer
from rotary_irq_rp2 import RotaryIRQ

# ic2
# from ssd1306 import SSD1306_I2C
from ssd1306 import SSD1306_SPI

# Constants for  setup
DISP_WIDTH = 128
DISP_HEIGHT = 64

# === PINS ===
# internal pins
on_pico_temp = machine.ADC(4)

# external pins
uart0 = machine.UART(0, 115200, tx=Pin(0), rx=Pin(1))
button_1 = Pin(2, Pin.IN, Pin.PULL_UP)  # interrupt cm/in button pins
button_2 = Pin(3, Pin.IN, Pin.PULL_UP)  # interrupt set mew alt/pressure
button_3 = Pin(15, Pin.IN, Pin.PULL_UP)  # interrupt detail or summary data


# TODO Bluetooth?
# Pin 4?, 5?

rotary = RotaryIRQ(pin_num_clk=6, pin_num_dt=7, min_val=0, reverse=False,
                range_mode=RotaryIRQ.RANGE_UNBOUNDED)
rotary_switch = Pin(8, Pin.IN, Pin.PULL_UP)

# https://www.tomshardware.com/how-to/oled-display-raspberry-pi-pico
# i2c=I2C(0,sda=Pin(12), scl=Pin(13), freq=400000)
# oled = SSD1306_I2C(DISP_WIDTH, DISP_HEIGHT, i2c)

# ssd1306 SDI SW setup for ssd1309 SDI
cs = machine.Pin(9)  # dummy (any un-used pin), no connection
# scl (SCLK) gp10
# SDA (MOSI) gp11
res = machine.Pin(12)  # RES (RST)  gp12
dc = machine.Pin(13)  # DC         gp13

led = Pin(25, Pin.OUT)
# Pin assignment, for multiple sensors on i2c
# i2c = I2C(id=1, scl=Pin(27), sda=Pin(26))
i2c = I2C(id=1, scl=Pin(27), sda=Pin(26), freq=400_000)
buzzer = Pin(28, Pin.OUT)

error_bme680 = False
error_bmp390 = False
# BME690 #77 address by default, can change addr if SDO pin LOW = 0x76
try: bme = BME680_I2C(i2c=i2c, address=0x77)
except: error_bme680 = True
try: bmp = bmp390.Bmp390(I2cAdapter(i2c), 0x76)
except: error_bmp390 = True

oled_spi = machine.SPI(1)
# print(f"oled_spi:{oled_spi}")
oled = SSD1306_SPI(DISP_WIDTH, DISP_HEIGHT, oled_spi, dc, res, cs)
text_20px = Writer(oled,font_20px, verbose=False)


# === Constants ====
DWELL_MS_LOOP = 300
OVER_TEMP_WARNING = 70.0


# Button debouncer with efficient interrupts, which don't take CPU cycles!
# https://electrocredible.com/raspberry-pi-pico-external-interrupts-button-micropython/
def callback(pin):
    global button_1_pushed, button_2_pushed, button_3_pushed
    global debounce_1_time, debounce_2_time, debounce_3_time
    if pin == button_1 and (int(time.ticks_ms()) - debounce_1_time) > 500:
        button_1_pushed = True
        debounce_1_time = time.ticks_ms()
    elif pin == button_2 and (int(time.ticks_ms()) - debounce_2_time) > 500:
        button_2_pushed = True
        debounce_2_time = time.ticks_ms()
    elif pin == button_3 and (int(time.ticks_ms()) - debounce_3_time) > 500:
        button_3_pushed = True
        debounce_3_time = time.ticks_ms()


button_1.irq(trigger=Pin.IRQ_FALLING, handler=callback)
button_2.irq(trigger=Pin.IRQ_FALLING, handler=callback)
button_3.irq(trigger=Pin.IRQ_FALLING, handler=callback)

# Functions =================================================

def button1():
    global button_1_pushed
    if button_1_pushed:
        button_1_pushed = False
        return True
    else:
        return False


def button2():
    global button_2_pushed
    if button_2_pushed:
        button_2_pushed = False
        return True
    else:
        return False
    
def button3():
    global button_3_pushed
    if button_3_pushed:
        button_3_pushed = False
        return True
    else:
        return False
    

def onboard_temperature():
    """
    pico data pico 2 rp2350 data sheet on page 1068
    data sheet says 27C  is 0.706v, with a slope of -1.721mV per degree
    
    :return: celsius
    """
    adc_value = on_pico_temp.read_u16()
    volt = (3.3 / 65535) * adc_value
    celsius = 27 - (volt - 0.706) / 0.001721
    if debug: print(f"on chip temp = {celsius:.3f}C")
    return celsius


def iaq_quality(iaq_value):
    """
    Calculate text to append to IAQ numerical rating
    IAQ: (0- 50 low, 51-100 ave, 101-150 poor, 151-200 bad, 201-300 VBad, 301-500 danger)

    :param iaq_value: the rating to score
    :return: string to add context to the number
    """
    if iaq_value < 50:
        return "best"
    elif iaq_value < 100:
        return "ave"
    elif iaq_value < 150:
        return "poor"
    elif iaq_value < 200:
        return "bad"
    elif iaq_value < 300:
        return "V Bad"
    else:
        return "DANGER"


def calc_sea_level_pressure(hpa, meters):
    """
    Calculate the sea level pressure from the hpa pressure at a known elevation
    formula from: https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

    slightly different formula at:
       https://www.brisbanehotairballooning.com.au/pressure-and-altitude-conversion/
       https://en.wikipedia.org/wiki/Barometric_formula

    :param hpa: current hpa pressure
    :param meters: meters elevation
    :return: sea level hpa based on known altitude & pressure
    """
    sea_level_pressure = hpa / (1.0 - (meters / 44330.0)) ** 5.255
    return sea_level_pressure


def calc_altitude(hpa, sea_level_pressure):
    """
    Calculate the altitude from sea level pressure and hpa pressure
    formula from: https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

    :param hpa: current hpa pressure
    :param sea_level_pressure: sea level hpa from nearest airport
    :return: meters elevation
    """
    meters = 44330.0 * (1.0 - (hpa / sea_level_pressure) ** (1.0 / 5.255))
    return meters


def bmp390_sensor(sea_level_pressure):
    """
    read temp, pressure from the bmp390 sensor
    measurement takes ~??? ms
    
    Driver code by 2022 Roman Shevchik
    https://github.com/octaprog7/BMP390
    https://github.com/octaprog7/BMP390/discussions/3
    bmp390.py, sensor_pack/base_sensor.py, sensor_pack/bus_service.py
    
    Pressure accuracy:  +/-3 Pa, +/-0.25 meters
    
    note: we are not certain that the values have be updated, OK for this app
    to check that need
    temperature_ready, pressure_ready, cmd_ready = bmp.get_status()
    and wait for both pressure_ready  and  cmd_ready are true

    :param sea_level_pressure: sea level hpa from nearest airport
    :return: celsius, hpa_pressure, meters, error string
    """
    debug = True
    try:
        # note: we are not certain that the values have be updated, OK for this app
        # call temp before pressure - don't know why
        celsius = bmp.get_temperature()
        hpa_pressure = bmp.get_pressure() / 100.0

        # derive altitude from pressure & sea level pressure
        meters = calc_altitude(hpa_pressure, sea_level_pressure)

        if debug:
            print(f"BMP390 Temp °C = {celsius:.2f} C")
            print(f"BMP390 Pressure = {hpa_pressure:.2f} hPA")
            print(f"BMP390 Alt = {meters * 3.28084:.2f} feet \n")

    except OSError as e:
        print("BMP390: Failed to read sensor.")
        return None, None, None, "ERROR_BMP680:" + str(e)

    return celsius, hpa_pressure, meters, None


def bme680_sensor(sea_level_pressure):
    """
    read temp, humidity, pressure, Indoor Air Quality (IAQ) from the BME680 sensor
    measurement takes ~189ms
    
    Pressure accuracy:  +/-12 Pa, +/-1 meter
    
    function iaq_quality(iaq) will show the textual scale
    IAQ: (0- 50 low, 51-100 ave, 101-150 poor, 151-200 bad, 201-300 VBad, 301-500 danger)
    
    need 30min before can trust IAQ

    :param sea_level_pressure: sea level hpa from nearest airport
    :return: celsius, percent_humidity, hpa_pressure, iaq, meters, error string
    """
    debug = True
    try:
        celsius = bme.temperature
        percent_humidity = bme.humidity
        hpa_pressure = bme.pressure
        gas_kohms = bme.gas / 100
        iaq_value = log(gas_kohms) + 0.04 * percent_humidity

        # derive altitude from pressure & sea level pressure
        meters = calc_altitude(hpa_pressure, sea_level_pressure)

        if debug:
            print(f"BME680 Temp °C = {celsius:.2f} C")
            print(f"BME680 Humidity = {percent_humidity:.1f} %")
            print(f"BME680 Pressure = {hpa_pressure:.2f} hPA")
            print(f"BME680 iaq = {iaq_value:.1f} {iaq_quality(iaq_value)}")
            print(f"BME680 Alt = {meters * 3.28084:.2f} feet \n")

    except OSError as e:
        print("BME680: Failed to read sensor.")
        return None, None, None, None, None, "ERROR_BME680:" + str(e)

    return celsius, percent_humidity, hpa_pressure, iaq_value, meters, None


def display_details(buzz):
    """
    display all measurement details in a standard font to the OLED

    param:buzz:  distance if dist = -1.0 then display error
    """
    oled.fill(0)
    if metric:
        oled.text(f"Alt = {altitude_m:.1f}m", 0, 0)
        oled.text(f"Temp = {temp_c:.1f}C", 0, 16)
        oled.text(f"Bar  ={pressure_hpa:.1f} hpa", 0, 28)
    else:
        oled.text(f"Alt = {altitude_m * 3.28084:.0f}\'", 0, 0)
        oled.text(f"Temp = {temp_f:.1f}F", 0, 16)
        oled.text(f"Bar  = {pressure_hpa * 0.02953:.2f}\"", 0, 28)
    
    if humidity:
        oled.text(f"Hum  = {humidity:.1f}%", 0, 40)
    else:
        oled.text(f"Hum  = no sensor", 0, 40)

    if iaq:
        oled.text(f"IAQ  = {iaq:.0f} {iaq_quality(iaq)}", 0, 52)
    else:
        oled.text(f"IAQ  = no sensor", 0, 52)
    oled.show()
    return


def display_big_num(buzz):
    """
    display just alt & hpa readings in a big font to the OLED

    param:buzz:  distance if dist = -1.0 then display error
    """
    global warning_toggle
    # display pressure in hpa only
    oled.fill(0)
    oled.text("Altimeter", 0, 0)
    
    # when iaq quality is poor, flash "iaq" at dwell rate (300ms) in box in upper right
    if iaq and iaq > 150.0:
        oled.fill_rect(128-3*8, 0, 26, 10, 1-warning_toggle)
        oled.text("iaq", 128-3*8-1, 1, warning_toggle)
        warning_toggle = 1 - warning_toggle
    
    # display pressure in hpa only
    if metric:
        chars = " m"
        convert = 1.0
    else:
        chars = "\'"
        convert = 3.28084
    oled.text("Alt", 16, 20)
    text_20px.set_textpos(49, 15)
    text_20px.printstring(f"{(altitude_m*convert):.0f}{chars}") #10px per char? variable space width
    
    # display pressure in hpa only
    oled.text("hPA", 16, 43)
    text_20px.set_textpos(52, 38)
    text_20px.printstring(f"{pressure_hpa:.1f}") #10px per char? variable space width
    oled.show()
    return


def update_settings_display(alt, press):
    """
    update display as the new settings are updated. oled display of alt & press,
    changes ot both altitude & pressure highlighted by white box/black letters

    :params alt: altitude in meters
    :params pressure: pressure in hpa
    """
    oled.fill(0)
    oled.text(f"adjusting...", 0, 0)
    if metric:
        string = f"{alt:.0f}m"
    else:
        string = f"{alt * 3.28084:.0f}\'"
    oled.text(f"Alt = ", 0, 20, 1)
    oled.fill_rect(45, 19, 128 - 45, 9, 1)
    oled.text(f"{string}", 45, 20, 0)

    string = f"{press:.1f} hpa"
    oled.text(f"Sea = ", 0, 36, 1)
    oled.fill_rect(45, 35, 128 - 45, 9, 1)
    oled.text(f"{string}", 45, 36, 0)
    oled.show()
    return


def adjust_altitude_slp(buzz, bmp_update):
    """
    adjust altitude in increments of +/- 1 foot or +/- 25 foot
    show in either feet or meters
    dependent variable is new Sea Level Pressure in hpa

    param:buzz: buzz if move to next input
    param:bmp_update: if bmp_update=True, then update bmp390, else bme680
    """
    global metric, slp_hpa_bme680, slp_hpa_bmp390
    
    new_alt = altitude_m
    new_slp = slp_hpa_bmp390 if bmp_update else slp_hpa_bme680
    print(f"Adjustment start: alt= {new_alt} m, {new_alt * 3.28084} ft")
    print(f"updating: {"bmp390" if bmp_update else "bme680"}")
    print(f"global slp values={slp_hpa_bmp390=}, {slp_hpa_bme680=}\n")
    if buzz:
        buzzer.on()
        zzz(.2)
        buzzer.off()
    update_settings_display(altitude_m, slp_hpa_bmp390 if bmp_update else slp_hpa_bme680)

    #### Increment/decrement New Altitude in feet, calculate new sea level pressure
    rotary_multiplier = 1
    rotary_old = rotary.value()
    while not button2():
        
        # Button 1: toggle between cm/in
        if button1():
            metric = not metric  # Toggle between metric and imperial units

        # adjustments in feet units
        new_alt_feet = new_alt * 3.28084
        
        rotary_new = rotary.value()
        if rotary_switch.value() == 0:
            # when rotary pushed, toggle between 1 and 25
            rotary_multiplier = 1 if rotary_multiplier != 1 else 25
            while rotary_switch.value() == 0:
                continue

        if rotary_old != rotary_new:
            # delta is how much changed since last loop
            delta = rotary_new - rotary_old 
            new_alt_feet = new_alt_feet + delta*rotary_multiplier
            rotary_old = rotary_new
            if debug: print(f"{new_alt_feet=}")
        
        new_alt = new_alt_feet / 3.28084
        new_slp = calc_sea_level_pressure(pressure_hpa, new_alt)
        if debug:
            print (f"{new_alt=}, {new_alt_feet=}")
            print(f"updating: {"bmp390" if bmp_update else "bme680"}")
            print(f"{new_slp=}, {slp_hpa_bmp390=}, {slp_hpa_bme680=}")
        update_settings_display(new_alt, new_slp)

    # upon loop exit beep, and update global slp_hpa_bme680
    if buzz:
        buzzer.on()
        zzz(.2)
        buzzer.off()
    
    print(f"Adjustment end:   alt= {new_alt} m, {new_alt * 3.28084} ft")
    if bmp_update:
        slp_hpa_bmp390 = new_slp
        print(f"updated: bmp390")
    else:
        slp_hpa_bme680 = new_slp
        print(f"updated: bme680")
    print(f"{new_slp=}, {slp_hpa_bmp390=}, {slp_hpa_bme680=}\n")
    return


# =========================  startup code =========================
show_env_details = False
set_known = False
buzzer_sound = True
metric = False
debug = False

button_1_pushed = False
button_2_pushed = False
button_3_pushed = False
debounce_1_time = 0
debounce_2_time = 0
debounce_3_time = 0

# Sea level pressure adjustment is 0.03783 hPA per foot @ 365'
# SLP_CALIBRATION_BMP390 = 0.42
# SLP_CALIBRATION_BME680 = 2.02
# # 7-Nov - mid afternoon
# INIT_SEA_LEVEL_PRESSURE = 1023.30
# SLP_CALIBRATION_BMP390 = 0.8436
# SLP_CALIBRATION_BME680 = 2.3768
# 
# #7-Nov - evening
# INIT_SEA_LEVEL_PRESSURE = 1022.50
# SLP_CALIBRATION_BMP390 = 1.3307
# SLP_CALIBRATION_BME680 = 2.8990
# 
# INIT_SEA_LEVEL_PRESSURE = 1022.30
# SLP_CALIBRATION_BMP390 = 1.2690
# SLP_CALIBRATION_BME680 = 2.8287
# 
# INIT_SEA_LEVEL_PRESSURE = 1022.80
# SLP_CALIBRATION_BMP390 = 1.3036
# SLP_CALIBRATION_BME680 = 2.8791
# 
# INIT_SEA_LEVEL_PRESSURE = 1022.90
# SLP_CALIBRATION_BMP390 = 0.9745
# SLP_CALIBRATION_BME680 = 2.4857
# 
# INIT_SEA_LEVEL_PRESSURE = 1023.20
# SLP_CALIBRATION_BMP390 = 0.9656
# SLP_CALIBRATION_BME680 = 2.5010
# 
# # bmp.set_oversampling(4, 1) # pressure (2=std, 4=ultrahigh), temp (1=2x recomended, 3=8x?)
# 
# INIT_SEA_LEVEL_PRESSURE = 1023.20
# SLP_CALIBRATION_BMP390 = 0.9205
# SLP_CALIBRATION_BME680 = 2.4285
# 
# INIT_SEA_LEVEL_PRESSURE = 1022.40
# SLP_CALIBRATION_BMP390 = 0.6094
# SLP_CALIBRATION_BME680 = 2.0623
# 
# INIT_SEA_LEVEL_PRESSURE = 1022.00
# SLP_CALIBRATION_BMP390 = 1.0148
# SLP_CALIBRATION_BME680 = 2.4430
# 
# INIT_SEA_LEVEL_PRESSURE = 1022.30
# SLP_CALIBRATION_BMP390 = 0.5305
# SLP_CALIBRATION_BME680 = 1.9789
# 
# INIT_SEA_LEVEL_PRESSURE = 1020.70
# SLP_CALIBRATION_BMP390 = 0.3948
# SLP_CALIBRATION_BME680 = 1.8669

# INIT_SEA_LEVEL_PRESSURE = 1017.70
# SLP_CALIBRATION_BMP390 = 1.1712
# SLP_CALIBRATION_BME680 = 2.6701

INIT_SEA_LEVEL_PRESSURE = 1017.40
SLP_CALIBRATION_BMP390 = 0.6595
SLP_CALIBRATION_BME680 = 2.1152

warning_toggle = 0

print("Starting...")
print("====================================")
print(implementation[0], uname()[3],
      "\nrun on", uname()[4])
temp = onboard_temperature()
print(f"onboard Pico 2 temp = {temp:.1f}C")
i2c1_devices = i2c.scan()
if i2c1_devices:
    for d in i2c1_devices: print(f"i2c1 device{d} = {hex(d)}")
else:
    print("ERROR: No i2c1 devices")
print("====================================")
print(f"oled_spi:{oled_spi}\n")

# BMP390 configuration debug 
if debug and not error_bmp390:
    print("BMP390 initialization value:")
    print(f"chip_id: {bmp.get_id()}")
    print(f"pwr mode: {bmp.get_power_mode()}")
    calibration_data = [bmp.get_calibration_data(index) for index in range(14)]
    print(f"Calibration data: {calibration_data}")
    print(f"Event: {bmp.get_event()}; Int status: {bmp.get_int_status()}; FIFO length: {bmp.get_fifo_length()}")    

# initialize BMP390 for continuous measurement mode
if not error_bmp390:
    # https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390-ds002.pdf
    # ultra-high
    bmp.set_oversampling(4, 1) # pressure (4=ultrahigh), temp (1=2x recommended)
    bmp.set_iir_filter(4)  # 4 is coef 7 IIR filter
    # high
#     bmp.set_oversampling(3, 0) # pressure (2=std, 3=high), temp (0=1x recommended)
#     bmp.set_iir_filter(2)  # 2 is coef 3 IIR filter
    bmp.set_sampling_period(5)
    bmp.start_measurement(enable_press=True, enable_temp=True, mode=2)
    print(f"pwr mode(3=continuous): {bmp.get_power_mode(), }\n")

# blank display
oled.fill(0)
oled.text("Starting", 0, 0)
oled.text("altimeter...", 0, 12)
oled.show()

slp_hpa_bmp390 = INIT_SEA_LEVEL_PRESSURE + SLP_CALIBRATION_BMP390
slp_hpa_bme680 = INIT_SEA_LEVEL_PRESSURE + SLP_CALIBRATION_BME680
print(f"Original sea level pressure = {INIT_SEA_LEVEL_PRESSURE:.2f} hpa")
print(f"Calibrated BMP390 Sea level = {slp_hpa_bmp390:.2f} hpa")
print(f"Calibrated BME680 Sea level = {slp_hpa_bme680:.2f} hpa\n")

if buzzer_sound: buzzer.on()
zzz(.2)
buzzer.off()

# main loop
print("start of main loop\n")
first_run = True
time_since_last_temp_update = time.ticks_ms()
try:
    while True:
        dwell = DWELL_MS_LOOP
        loop_time = time.ticks_ms()
        elapsed_time = time.ticks_diff(time.ticks_ms(), time_since_last_temp_update)
        if debug: print(f"Time since last temp ={elapsed_time}")

        # Button 1: cm/in
        if button1():
            metric = not metric  # Toggle between metric and imperial units

        # Button 2: Adjust altitude or sea level hpa to known values
        if button2():
            adjust_altitude_slp(True, bmp_update=not error_bmp390)
        
        # Button 3: detail/Summary
        if button3():
            show_env_details = not show_env_details  # Toggle between summary & details

        #  * BME680 temp & humidity (189ms duration)
        if first_run or elapsed_time > 2000:
            dwell = DWELL_MS_LOOP - 189
            time_since_last_temp_update = time.ticks_ms()

            # Check for over temperature onboard pico
            temp = onboard_temperature()
            if temp > OVER_TEMP_WARNING:
                print(f"WARNING: onboard Pico 2 temp = {temp:.1f}C")
                
            # get bme680 sensor data
            temp_c_bme680,  humidity_bme680, hpa_bme680,  iaq_bme680,  alt_m_bme680, error_bme680  = bme680_sensor(slp_hpa_bme680)
            if error_bme680:
                print(f"No lower-precision Altitude BME680 sensor: {error_bme680}\n")
            first_run = False
            
            # get bmp390 sensor data, overwrite temp_c, temp_f, and altitude_m
            temp_c_bmp390, hpa_bmp390, alt_m_bmp390, error_bmp390 = bmp390_sensor(slp_hpa_bmp390)
            if error_bmp390:
                print(f"No high-precision Altitude BMP390 sensor: {error_bmp390}\n")
            
            temp_c, humidity, pressure_hpa, iaq, altitude_m, temp_f = (None,) * 6
            
            # Check if both sensors have errors, exit loop
            if error_bme680 and error_bmp390:
                oled.fill(0)
                oled.text("Error", 0, 0, 1)
                oled.fill_rect(0, 11, 128, 18, 1)
                oled.text(f"No Alt Sensors:", 5, 12, 0)
                oled.text(f"BMP390 & BME680", 5, 21, 0)
                oled.show()  
                break
            
            # Update BME680 data, if available
            if not error_bme680:
                temp_c = temp_c_bme680
                humidity = humidity_bme680
                altitude_m = alt_m_bme680
                pressure_hpa = hpa_bme680
                iaq = iaq_bme680
            
            # Update BMP390 data, if available and overwrite temperature and altitude
            if not error_bmp390:
                temp_c = temp_c_bmp390
                altitude_m = alt_m_bmp390
                pressure_hpa = hpa_bmp390               
            temp_f = (temp_c * 9.0 / 5.0) + 32.0
            first_run = False

        # Every loop adjust for if sensor read
        time.sleep_ms(dwell)
        led.toggle()
        loop_elapsed_time = time.ticks_diff(time.ticks_ms(), loop_time)
        if debug:
            print(f"loop time with {dwell}ms delay={loop_elapsed_time}")
        
        # update display at uniform timing
        if show_env_details:
            display_details(buzzer_sound)
        else:
            display_big_num(buzzer_sound)


# if control-c at end clean up pico2
except KeyboardInterrupt:
    # blank oled
    oled.fill(0)
    oled.show() 
    print("Exit: ctrl-c")
# except:
#     print ("Other error or exception occurred!")
