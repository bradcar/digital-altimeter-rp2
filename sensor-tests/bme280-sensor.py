# bme280 sensor test code
#
# the BME280.py library must be installed on Raspberry Pi in order for this to work
# library source:
#    https://github.com/RuiSantosdotme/ESP-MicroPython/blob/master/code/WiFi/HTTP_Client_IFTTT_BME280/BME280.py
#    However it returns strings, so turned this into floats in my version
#
# Portland - PDX airport sea level pressure updated every hour
#     https://www.weather.gov/wrh/timeseries?site=KPDX
#
# my home office is ~361 feet elevation
# my garage is at <todo> feet elevation
#
# by bradcar

from machine import Pin, I2C
from time import sleep
import BME280

# Initialize I2C communication
i2c = I2C(id=1,  sda=Pin(26), scl=Pin(27), freq=10000)

debug = False

def calc_sea_level_pressure(hpa, meters):
    sea_level_pressure_hpa = hpa / (1.0 - (meters / 44330.0)) ** 5.255

    return sea_level_pressure_hpa

def bme_temp_humidity_altitude(sea_level):
    """
    read temp & humidity from the DHT22 sensor, measurement takes ~271ms
    
    :param :sea_level: sea level
    :returns: celsius, percent_humidity, error string
    """
    # Read sensor data
    try:    
        temp_c = bme.temperature
        percent_humidity = bme.humidity
        hpa_pressure = bme.pressure
        meters = 44330.0 * (1.0 - (hpa_pressure/sea_level)**(1.0/5.255) )
        if debug:
            print(f"BME Temp °C : {temp_c:.2f}")
            print(f"BME Humidity: {percent_humidity:.2f}%")
            print(f"BME Pressure: {hpa_pressure:.2f}hPA")
            
    except Exception as e:
        return None, None, None, None, "ERROR_BME:" + str(e)
    return temp_c, percent_humidity, hpa_pressure, meters, None

# Initialize BME280 sensor
bme = BME280.BME280(i2c=i2c)


while True:
    try:       
        # change this to match the pressure (hPa) at sea level, use nearest airport
        # https://community.bosch-sensortec.com/t5/Question-and-answers/How-to-calculate-the-altitude-from-the-pressure-sensor-data/qaq-p/5702
        # https://www.weather.gov/wrh/timeseries?site=KPDX
        # hPa = mB
        sea_level_pressure_hpa = 1011.20
        
        # Read sensor data
        temperature_c, humidity, pressure_hpa, altitude_m, _ = bme_temp_humidity_altitude(sea_level_pressure_hpa)        
        
        # Convert temperature to Fahrenheit
        temperature_f = temperature_c * (9.0/5.0) + 32.0
        
        # calculate altitude
        altitude_f = altitude_m * 3.28084
        
        # Print sensor readings
        print(f"{sea_level_pressure_hpa=}")
        print(f"Temp C = {temperature_c:.1f} C")
        print(f"Temp F = {temperature_f:.1f} F")
        print(f"Humidity = {humidity:.1f} %")
        print(f"Pressure= {pressure_hpa:.2f} hPa")
        print(f"Altitude= {altitude_m:.1f} m")
        print(f"Altitude= {altitude_f:.1f} f")
        
        # from the Altitude, calc sea level pressure
        predict_slp = calc_sea_level_pressure(pressure_hpa, altitude_m)
        print(f"Predict Sea Level pressure = {predict_slp:.1f} hPa\n")
        
    except Exception as e:
        # Handle any exceptions during sensor reading
        print("An error occurred:", e)

    sleep(5)
