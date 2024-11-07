# BMP390 Sensor test code
#
# code and driver from: https://github.com/octaprog7/BMP390
#    Driver code by 2022 Roman Shevchik   goctaprog@gmail.com
#    bmp390.py, sensor_pack/base_sensor.py, sensor_pack/bus_service.py
    
from machine import I2C, Pin
import bmp390
from sensor_pack.bus_service import I2cAdapter
import time
from time import sleep as zzz

# 12C pins
i2c = I2C(id=1, scl=Pin(27), sda=Pin(26), freq=400_000)   # on Raspberry Pi Pico
bmp = bmp390.Bmp390(I2cAdapter(i2c), 0x76)

i2c1_devices = i2c.scan()
if i2c1_devices:
    for d in i2c1_devices: print(f"i2c1 device{d} = {hex(d)}")
else:
    print("ERROR: No i2c1 devices")


def calc_altitude(hpa, sea_level_pressure):
    """
    Calculate the altitude from sea level pressure and hpa pressure 

    :param hpa: current hpa pressure
    :param sea_level_pressure: sea level hpa from closest airport
    :return: meters elevation
    """
    meters = 44330.0 * (1.0 - (hpa / sea_level_pressure) ** (1.0 / 5.255))
    return meters


# Initialization  - If you get exceptions, check all bmp390 connections!
debug = False

if debug:
    print(f"chip_id: {bmp.get_id()}")
    print(f"pwr mode: {bmp.get_power_mode()}")
    calibration_data = [bmp.get_calibration_data(index) for index in range(14)]
    print(f"Calibration data: {calibration_data}")
    print(f"Event: {bmp.get_event()}; Int status: {bmp.get_int_status()}; FIFO length: {bmp.get_fifo_length()}")    

#Init for continuous measurement mode before loop
bmp.set_oversampling(2, 3)
bmp.set_sampling_period(5)
bmp.set_iir_filter(2) 
bmp.start_measurement(enable_press=True, enable_temp=True, mode=2)
if debug:
    print(f"pwr mode: {bmp.get_power_mode()}")
temperature_ready, pressure_ready, cmd_ready = bmp.get_status()

# correction added 
INIT_SEA_LEVEL_PRESSURE = 1025.20 + 0.45

# get first temp & pressure reading
tempc = bmp.get_temperature()
print(f"tempc: {tempc:.2f} °C")
hpa = bmp.get_pressure() / 100.0
print(f"hpa  : {hpa:.2f} hPa\n")

# pressure & temp can be read any time, generally updates are
# every 158ms to 700ms, when both are cmd_ready and pressure_ready both true
# Main Loop -------------------------------------------------------
sense_start = 0
start = time.ticks_ms()
for pressure, temperature in bmp:
    temperature_ready, pressure_ready, cmd_ready = bmp.get_status()
    
    # makes sure that pressure has been updated before reading
    if cmd_ready and pressure_ready:
        read_time = bmp.get_sensor_time()

        pressure_hpa = pressure / 100.0
        print(f"Temp: {temperature:.2f} °C, pressure: {pressure_hpa:.2f} hPa")

        alt = calc_altitude(pressure_hpa, INIT_SEA_LEVEL_PRESSURE)
        print(f"Altitude = {alt:.1f} m, {alt * 3.28084:.1f} ft\n")
