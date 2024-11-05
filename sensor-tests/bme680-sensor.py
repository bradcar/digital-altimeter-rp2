# bme680 sensor - temp, humidity, pressure, IAC, altitude
#
# bme680 driver code:
#    https://github.com/robert-hh/BME680-Micropython
#
# IAQ (Indoor Air Quality) calculation:
#    IAQ value between 0 and 500, where lower values represent higher air quality.
#    https://github.com/thstielow/raspi-bme680-iaq
#      IAQ:     0- 50 good
#              51-100 average
#             101-150 poor
#             151-200 bad
#             201-300 worse
#             301-500 very bad
#
# Portland - PDX airport sea level pressure updated every hour
#     https://www.weather.gov/wrh/timeseries?site=KPDX
#
# my home office is ~361 feet elevation, first bme680 says 303.5 feet (+57.5' correction needed)
# my garage is at <todo> feet elevation
#
# by bradcar

from machine import Pin, I2C
from time import sleep
from bme680 import *
from math import log

# Pin assignment  i2c1 
i2c = I2C(id=1, sda=Pin(26),scl=Pin(27))
bme = BME680_I2C(i2c=i2c)

# def bme_temp_humid_hpa_iaq_alt(sea_level):
#     """
#     read temp, humidity, pressure, Indoor Air Qualtiy (IAQ) from the BME680 sensor
#     measurement takes ~189ms
#      IAQ:     0- 50 good
#              51-100 average
#             101-150 poor
#             151-200 bad
#             201-300 worse
#             301-500 very bad
#     todo: add code to not trust IAQ until 300 cycles or about 30mins.
#           https://github.com/thstielow/raspi-bme680-iaq
#     
#     :param :sea_level: sea level hpa from closest airport
#     :returns: temp_c, percent_humidity, hpa_pressure, iaq, meters, error string
#     """
#     # Read sensor data
#     debug = True
#     try:    
#         temp_c = bme.temperature
#         percent_humidity = bme.humidity
#         hpa_pressure = bme.pressure
#         gas_resist = bme.gas/100
#         
#         # derived sensor data
#         meters = 44330.0 * (1.0 - (hpa_pressure/sea_level)**(1.0/5.255) )
#         iaq = log(gas_resist) + 0.04 * percent_humidity
#         
#         if debug:
#             print(f"BME680 Temp °C = {temp_c:.2f} C")
#             print(f"BME680 Humidity = {percent_humidity:.2f} %")
#             print(f"BME680 Pressure = {hpa_pressure:.2f} hPA")
#             print(f"BME680 iaq = {iaq:.2f} IAQ lower better")
#             print(f"BME680 Alt = {meters * 3.28084:.2f} feet \n")
#             
#     except OSError as e:
#         print("BME680: Failed to read sensor.")
#         return None, None, None, None, None, "ERROR_BME680:" + str(e)
#     
#     return temp_c, percent_humidity, hpa_pressure, iaq, meters, None


debug = False
sea_level_pressure_hpa = 1012.90
while True:
    try:
        temp_c = bme.temperature
        humidity = bme.humidity
        pressure_hpa = bme.pressure
        gas_resist = bme.gas/1000
        
        meters = 44330.0 * (1.0 - (pressure_hpa/sea_level_pressure_hpa)**(1.0/5.255) )
        feet = meters * 3.28084
        
        iaq = log(gas_resist) + 0.04 * humidity

        print(f"Temperature= {temp_c:.1f} C")
        print(f"Humidity= {humidity:.1f} %")
        print(f"Pressure= {pressure_hpa:.1f} hPa")
        print(f"Gas restance= {gas_resist:.2f} KOhms")
        print(f"iAQ= {iaq:.1f} lower better [0 to 500]")
        print(f"Altitude= {meters:.1f} meters")
        print(f"Altitude= {feet:.1f} feet")
        print()

    except OSError as e:
        print("Failed to read sensor.")
 
    sleep(5)
