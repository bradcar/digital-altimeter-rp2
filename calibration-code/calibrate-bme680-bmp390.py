# Raspberry Pi Pico 2: calibrate sensors - Nov 2024
#
# Sensors used
#    - BMP390 highly accurate pressure & altitude
#    - BME680 temp, humidity, pressure, IAQ, altitude

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
#  * ERROR adjust_altitude_slp still uses altitude_m and pressure_hpa from bmp390
#          so doesn't really get bme680 adjusted correctly
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


# === Constants ====
DWELL_MS_LOOP = 5000
OVER_TEMP_WARNING = 70.0

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


def bmp390_sensor(sea_level_pressure):
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


def bme680_sensor(sea_level_pressure):
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




# =========================  startup code =========================
show_env = True
buzzer_sound = True
debug = False

# Sea level pressure adjustment is 0.03783 hPA per foot @ 365'
INIT_SEA_LEVEL_PRESSURE = 1022.90

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
oled.text("Calibration...", 0, 12)
oled.show()

slp_bmp390_hpa = INIT_SEA_LEVEL_PRESSURE 
slp_bme680_hpa = INIT_SEA_LEVEL_PRESSURE

print("start of main loop\n")
# main loop
try:
    while True:             
        # get bmp390 sensor data, overwrite temp_c, temp_f, and altitude_m
        bmp390_temp_c, bmp390_hpa, bmp390_alt_m, error = bmp390_sensor(slp_bmp390_hpa)
        
        # get bme680 sensor data
        bme680_temp_c,  bme680_humidity, bme680_hpa,  bme680_iaq,  bme680_alt_m, error = bme680_sensor(slp_bme680_hpa)

        # calc altitude based on pressures
        initial_bmp390_alt = calc_altitude(bmp390_hpa, slp_bmp390_hpa)
        initial_bme680_alt = calc_altitude(bme680_hpa, slp_bme680_hpa)
        print(f"bmp390 initial est {initial_bmp390_alt*3.28084:.1f} feet")
        print(f"bme680 initial est {initial_bme680_alt*3.28084:.1f} feet")
        
        adjust_to_feet = float(input("\nEnter altitude (in feet): "))
        adjust_to_meters = adjust_to_feet / 3.28084
        bmp390_hpa_adj = calc_sea_level_pressure(bmp390_hpa, adjust_to_meters) - slp_bmp390_hpa
        bme680_hpa_adj = calc_sea_level_pressure(bme680_hpa, adjust_to_meters) - slp_bme680_hpa

        print("\nCalibration values:")
        print(f"\nINIT_SEA_LEVEL_PRESSURE = {INIT_SEA_LEVEL_PRESSURE:.2f}")
        print(f"SLP_BMP390_CALIBRATION = {bmp390_hpa_adj:.4f}")
        print(f"SLP_BME680_CALIBRATION = {bme680_hpa_adj:.4f}\n")
        
        _ = input("Enter for next loop: ")

        # Every loop adjust for if sensor read
        led.toggle()

# if control-c at end clean up pico2
except KeyboardInterrupt:
    # blank oled
    oled.fill(0)
    oled.show() 
    print("Exit: ctrl-c")
# except:
#     print ("Other error or exception occurred!")
