# Raspberry Pi Pico 2: Altimeter = Elevation & sea level pressure ajdust - Nov 2024
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
# Use nearest airport for sea level pessure
#    Portland updated hourly (7 min before the hour)
#        https://www.weather.gov/wrh/timeseries?site=KPDX
#
#    my home office is
#        365.0 feet elevation, 111.25m
#        first bme680 says 287.1 feet, 85.51 m
#        correction: +77.9' or + 25.75m correction needed)
#
#    my garage is at <todo> feet elevation
#        ~335 feet elevation, 102.11m  (26' lower than offie)
#
# Key links
#     https://micropython.org/download/RPI_PICO2/ for latest .uf2 preview
#     https://docs.micropython.org/en/latest/esp8266/tutorial/ssd1306.html
#
# pycharm stubs
#    https://micropython-stubs.readthedocs.io/en/main/packages.html#mp-packages
#
# TODOs
#  * right not if bmp390 exists it just overwrite bme680 temp & humidity, need
#    to plan logic if one or the other don't exist
#  * right now calibration adjust is only for bme680 SLP and NOT bmp390 SLP !!!
#  * add digital encoder, use it's button for 10' correction vs else 1' adjust
#  * add button #3 to switch between large font summary & detailed data
#  
#
# by bradcar

import time
from math import log, floor
from os import uname
from sys import implementation
from time import sleep as zzz

import bmp390
from sensor_pack.bus_service import I2cAdapter
from bme680 import BME680_I2C
from framebuf import FrameBuffer, MONO_HLSB
from machine import Pin, I2C, ADC

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
button_2 = Pin(3, Pin.IN, Pin.PULL_UP)  # interrupt rear/front button pins

# TODO Bluetooth?
# Pin 4?, 5?

rotary = RotaryIRQ(pin_num_clk=6, pin_num_dt=7, min_val=0, reverse=False, \
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

# BME690 #77 address by default, can change addr if SDO pin LOW = 0x76
bme = BME680_I2C(i2c=i2c, address=0x77)
bmp = bmp390.Bmp390(I2cAdapter(i2c), 0x76)

oled_spi = machine.SPI(1)
# print(f"oled_spi:{oled_spi}")
oled = SSD1306_SPI(DISP_WIDTH, DISP_HEIGHT, oled_spi, dc, res, cs)
text_20px = Writer(oled,font_20px, verbose=False)


# === Constants ====
DWELL_MS_LOOP = 300
PDX_SLP_1013 = 1009.90
OVER_TEMP_WARNING = 70.0


# Button debouncer with efficient interrupts, which don't take CPU cycles!
# https://electrocredible.com/raspberry-pi-pico-external-interrupts-button-micropython/
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


# Functions =================================================

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
       https://www.mide.com/air-pressure-at-altitude-calculator
       https://www.omnicalculator.com/physics/air-pressure-at-altitude
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
    :param sea_level_pressure: sea level hpa from closest airport
    :return: meters elevation
    """
    meters = 44330.0 * (1.0 - (hpa / sea_level_pressure) ** (1.0 / 5.255))
    return meters


def bmp_temp_hpa_alt(sea_level_pressure):
    """
    read temp, pressure from the bmp390 sensor
    measurement takes ~??? ms
    
    Driver code by 2022 Roman Shevchik   goctaprog@gmail.com
    bmp390.py, sensor_pack/base_sensor.py, sensor_pack/bus_service.py
    
    Pressure accuracy:  +/-3 Pa, +/-0.25 meters
    
    note: we are not certain that the values have be updated, OK for this app
    to check that need
    temperature_ready, pressure_ready, cmd_ready = bmp.get_status()
    and wait for both pressure_ready  and  cmd_ready are true

    :param sea_level_pressure: sea level hpa from closest airport
    :return: celsius, hpa_pressure, meters, error string
    """
    debug = True
    try:
        # note: we are not certain that the values have be updated, OK for this app
        # call temp before pressure - don't know why
        celsius = bmp.get_temperature()
        hpa_pressure = bmp.get_pressure() / 100.0

        # derive altitude from pressure & sea level pressure
        #    meters = 44330.0 * (1.0 - (hpa_pressure/sea_level)**(1.0/5.255) )
        meters = calc_altitude(hpa_pressure, sea_level_pressure)

        if debug:
            print(f"BMP390 Temp °C = {celsius:.2f} C")
            print(f"BMP390 Pressure = {hpa_pressure:.2f} hPA")
            print(f"BMP390 Alt = {meters * 3.28084:.2f} feet \n")

    except OSError as e:
        print("BMP390: Failed to read sensor.")
        return None, None, None, "ERROR_BMP680:" + str(e)

    return celsius, hpa_pressure, meters, None
    return


def bme_temp_humid_hpa_iaq_alt(sea_level_pressure):
    """
    read temp, humidity, pressure, Indoor Air Qualtiy (IAQ) from the BME680 sensor
    measurement takes ~189ms
    
    Pressure accuracy:  +/-12 Pa, +/-1 meters
    
    function iaq_quality(iaq) will show the textual scale
    IAQ: (0- 50 low, 51-100 ave, 101-150 poor, 151-200 bad, 201-300 VBad, 301-500 danger)
    
    need 30min before can trust IAQ

    :param sea_level_pressure: sea level hpa from closest airport
    :return: celsius, percent_humidity, hpa_pressure, iaq, meters, error string
    """
    debug = True
    try:
        celsius = bme.temperature
        percent_humidity = bme.humidity
        hpa_pressure = bme.pressure
        gas_kohms = bme.gas / 100

        # derive altitude from pressure & sea level pressure
        #    meters = 44330.0 * (1.0 - (hpa_pressure/sea_level)**(1.0/5.255) )
        meters = calc_altitude(hpa_pressure, sea_level_pressure)
        iaq_value = log(gas_kohms) + 0.04 * percent_humidity

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


def display_car(celsius, fahrenheit):
    """
    display_car & temp, Need oled.fill(0) before call & oled.show() after call

    :param celsius: temp in C to display
    :param fahrenheit: temp in F to display
    """

    oled.blit(FrameBuffer(bitmap_artcar_image_back, 56, 15, MONO_HLSB), 36, 0)
    oled.blit(FrameBuffer(degree_temp, 24, 10, MONO_HLSB), 104, 0)

    if metric:
        oled.blit(FrameBuffer(bitmap_unit_cm, 24, 10, MONO_HLSB), 0, 0)
        if celsius:
            oled.text(f"{celsius:.0f}", 108, 2)
        else:
            oled.text("xx", 108, 2)
    else:
        oled.blit(FrameBuffer(bitmap_unit_in, 24, 10, MONO_HLSB), 0, 0)
        if fahrenheit:
            oled.text(f"{fahrenheit:.0f}", 108, 2)
        else:
            oled.text("xx", 108, 2)
    return


def display_environment(buzz):
    """
    display just environment readings & car image(for fun)
    No need to oled.fill(0) before or oled.show() after call

    param:buzz:  distance if dist = -1.0 then display error
    """
    oled.fill(0)
    if temp_c:
        if metric:
            oled.text(f"Temp = {temp_c:.1f}C", 0, 0)
        else:
            oled.text(f"Temp = {temp_f:.1f}F", 0, 0)
        if humidity: oled.text(f"Humid= {humidity:.1f}%", 0, 12)
    else:
        oled.text(f"No Temp/Humidity", 0, 10)
        oled.text(f" Sensor Working", 0, 20)

    oled.text(f"IAQ = {iaq:.0f} {iaq_quality(iaq)}", 0, 24)

    if metric:
        oled.text(f"Bar = {pressure_hpa:.1f} hpa", 0, 36)
    else:
        oled.text(f"Bar = {pressure_hpa * 0.02953:.2f}\"", 0, 36)

    if metric:
        oled.text(f"Alt = {altitude_m:.1f}m", 0, 52)
    else:
        oled.text(f"Alt = {altitude_m * 3.28084:.0f}\'", 0, 52)

    #     oled.blit(FrameBuffer(bitmap_artcar_image_back, 56, 15, MONO_HLSB), 22, 36)
    oled.show()
    return


def display_big_num(buzz):
    """
    display just alt & hpa readings
    No need to oled.fill(0) before or oled.show() after call

    param:buzz:  distance if dist = -1.0 then display error
    """
    global warning_toggle
    # display pressure in hpa only
    oled.fill(0)
    oled.text("Altimeter", 0, 0)
    
    # when iaq quality poor, flash "iaq" at dwell rate (300ms) in box in upper right
    if iaq > 150.0:
        oled.fill_rect(128-3*8, 0, 26, 10, 1-warning_toggle)
        oled.text("iaq", 128-3*8-1, 1, warning_toggle)
        warning_toggle = 1 - warning_toggle
    
    # display pressure in hpa only
    if metric:
        chars = " m"
        convert = 1.0
    else:
        chars = "\'"
#         chars = " f"
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


def button2_not_pushed():
    global interrupt_2_flag
    debug = True
    if interrupt_2_flag == 1:
        interrupt_2_flag = 0
        return False
    else:
        return True


def update_numbers(alt, press):
    """
    oled display of alt & press,
    changes ot both altitude & presure hightlighted by white box/black letters

    :params alt: altitude in meters
    :params pressure: pressure in hpa
    """
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


def input_known_values(buzz):
    """
    adjust altitude in increments of +/- 1 foot or +/- 25 foot
    show in either feet or meters
    dependent variable is new Sea Level Pressure in hpa

    param:buzz: buzz if move to next input
    """
    global metric, interrupt_1_flag, slp_bme680_hpa

    err = None
    new_alt = altitude_m
    print(f"adjustment start: alt= {new_alt} m, {new_alt * 3.28084} ft")
    print(f"  sea level pressure = {slp_bme680_hpa} hpa")
    if buzz:
        buzzer.on()
        zzz(.2)
        buzzer.off()
    #
    oled.fill(0)
    oled.text(f"adjusting...", 0, 0)
    update_numbers(altitude_m, pressure_hpa)

    #### Increment/decrement New Altitude in feet, calcule new sea level pressure
    rotary_multiplier = 1
    rotary_old = rotary.value()
    while (button2_not_pushed()):
        new_alt_feet = new_alt * 3.28084

        rotary_new = rotary.value()
        if rotary_switch.value() == 0:
            # toggle between 1 and 25
            rotary_multiplier = 1 if rotary_multiplier != 1 else 25
            while rotary_switch.value() == 0:
                continue

        if rotary_old != rotary_new:
            # change in value since last loop then scale by multiplier
            delta = rotary_new - rotary_old 
            new_alt_feet = new_alt_feet + delta*rotary_multiplier
            rotary_old = rotary_new
            if debug: print(f"{new_alt_feet=}")
        
        new_alt = new_alt_feet / 3.28084
        
        new_slp = calc_sea_level_pressure(pressure_hpa, new_alt)
        if debug:
            print (f"{new_alt=}, {new_alt_feet=}")
            print (f"{new_slp=}, {slp_bme680_hpa=}, {(new_slp - slp_bme680_hpa)=}\n")
                
        # Button 1: cm/in
        if interrupt_1_flag == 1:
            interrupt_1_flag = 0
            if debug: print("button 1 Interrupt Detected: in/cm")
            metric = not metric  # Toggle between metric and imperial units
  
        zzz(.2)
        if debug:
            print(f"{new_alt=},{new_alt_feet=} ")
            print(f"{slp_bme680_hpa=},{new_slp=}")
        update_numbers(new_alt, new_slp)

    # upon loop exit beep, and update global slp_bme680_hpa
    print(f"adjustment end: alt= {new_alt} m, {new_alt * 3.28084} ft")
    print(f"  sea level pressure = {new_slp} hpa\n")
    if buzz:
        buzzer.on()
        zzz(.2)
        buzzer.off()
    slp_bme680_hpa = new_slp

    # return after 5 seconds
    zzz(0.5)
    return err


# =========================  startup code =========================
show_env = True
set_known = False
buzzer_sound = True
metric = True
debug = False

interrupt_1_flag = 0
interrupt_2_flag = 0
debounce_1_time = 0
debounce_2_time = 0

# Sea level pressure adjustment is 0.03783 hPA per foot @ 365'
SLP_BMP390_CALIBRATION = 0.42
SLP_BME680_CALIBRATION = 2.02 
INIT_SEA_LEVEL_PRESSURE = 1025.20

temp_f = None
temp_c = None
humidity = None
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

# BMP390 setup
if debug:
    print(f"chip_id: {bmp.get_id()}")
    print(f"pwr mode: {bmp.get_power_mode()}")
    calibration_data = [bmp.get_calibration_data(index) for index in range(14)]
    print(f"Calibration data: {calibration_data}")
    print(f"Event: {bmp.get_event()}; Int status: {bmp.get_int_status()}; FIFO length: {bmp.get_fifo_length()}")    
#Init for bmp390 continuous measurement mode before loop
bmp.set_oversampling(2, 3)
bmp.set_sampling_period(5)
bmp.set_iir_filter(2) 
bmp.start_measurement(enable_press=True, enable_temp=True, mode=2)
if debug:
    print(f"pwr mode: {bmp.get_power_mode()}")

# blank display
oled.fill(0)
oled.text("Starting", 0, 0)
oled.text("altimeter...", 0, 12)
oled.show()

slp_bmp390_hpa = INIT_SEA_LEVEL_PRESSURE + SLP_BMP390_CALIBRATION
slp_bme680_hpa = INIT_SEA_LEVEL_PRESSURE + SLP_BME680_CALIBRATION
print(f"Actual:   sea level pressure = {INIT_SEA_LEVEL_PRESSURE:.2f} hpa")
print(f"Calibrated BMP390 Sea level   = {slp_bmp390_hpa:.2f} hpa")
print(f"Calibrated BME680 Sea level   = {slp_bme680_hpa:.2f} hpa\n")


if buzzer_sound: buzzer.on()
zzz(.2)
buzzer.off()

print("start of main loop\n")
# main loop
first_run = True
time_since_last_temp_update = time.ticks_ms()

try:
    while True:
        dwell = DWELL_MS_LOOP
        loop_time = time.ticks_ms()
        elapsed_time = time.ticks_diff(time.ticks_ms(), time_since_last_temp_update)
        if debug: print(f"Time since last temp ={elapsed_time}")

        # Button 1: cm/in
        if interrupt_1_flag == 1:
            interrupt_1_flag = 0
            if debug: print("button 1 Interrupt Detected: in/cm")
            metric = not metric  # Toggle between metric and imperial units

        # Button 2: rear sensors or front sensors
        if interrupt_2_flag == 1:
            interrupt_2_flag = 0
            if debug: print("button 2 Interrupt Detected: input known values")
            error = input_known_values(True)
            if error:
                print(f"Error setting known values: {error}")

        #  * BME680 temp & humidity (189ms duration)
        if first_run or elapsed_time > 3000:
            dwell = DWELL_MS_LOOP - 189
            time_since_last_temp_update = time.ticks_ms()

            # check for over temperature onboard pico
            temp = onboard_temperature()
            if temp > OVER_TEMP_WARNING:
                print(f"WARNING: onboard Pico 2 temp = {temp:.1f}C")
                
            # get bme680 sensor data
            temp_c, humidity, pressure_hpa, iaq, altitude_m, error = bme_temp_humid_hpa_iaq_alt(slp_bme680_hpa)
            if error:
                print(f"No Altitude sensor: {error}")
            else:
                if debug: print(f"{temp_c=:.2f}")
                temp_f = (temp_c * 9.0 / 5.0) + 32.0
            first_run = False
            
            # get bmp390 sensor data, overwrite temp_c, temp_f, and altitude_m
            t_c, p_hpa, alt_m, error = bmp_temp_hpa_alt(slp_bmp390_hpa)
            if error:
                print(f"No high-precision Altitude sensor: {error}")
            else:
                if debug: print(f"{temp_c=:.2f}")
                temp_c = t_c
                temp_f = (temp_c * 9.0 / 5.0) + 32.0
                altitude_m = alt_m
                pressure_hpa = p_hpa
            first_run = False

        # Every loop adjust for if sensor read
        time.sleep_ms(dwell)
        led.toggle()
        loop_elapsed_time = time.ticks_diff(time.ticks_ms(), loop_time)
        if debug:
            print(f"loop time with {dwell}ms delay={loop_elapsed_time}")
        
        # update display at uniform timing
        if not error:
#             display_environment(buzzer_sound)
            display_big_num(buzzer_sound)
        else:
            oled.fill(0)
            oled.text("Error", 0, 0, 1)
            oled.fill_rect(0, 11, 128, 9, 1)
            oled.text(f"No Alt Sensor", 13, 12, 0)
            oled.show()  

# if control-c at end clean up pico2
except KeyboardInterrupt:
    # blank oled
    oled.fill(0)
    oled.show() 
    print("Exit: ctrl-c")
# except:
#     print ("Other error or exception occurred!")
