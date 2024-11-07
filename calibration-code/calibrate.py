

def calc_altitude(hpa, sea_level_pressure):
    """
    Calculate the altitude from sea level pressure and hpa pressure 

    :param hpa: current hpa pressure
    :param sea_level_pressure: sea level hpa from closest airport
    :return: meters elevation
    """
    meter = 44330.0 * (1.0 - (hpa / sea_level_pressure) ** (1.0 / 5.255))
    return meter

def calc_sea_level_pressure(hpa, meter):
    """
    Calculate the sea level pressure from the hpa pressure at a known elevation
    slightly different formula at:
       https://www.brisbanehotairballooning.com.au/pressure-and-altitude-conversion/
       https://www.mide.com/air-pressure-at-altitude-calculator
       https://www.omnicalculator.com/physics/air-pressure-at-altitude
       https://en.wikipedia.org/wiki/Barometric_formula

    :param hpa: current hpa pressure
    :param meter: meters elevation
    :return: sea level hpa based on known altitude & pressure
    """
    sea_level_pressure = hpa / (1.0 - (meter / 44330.0)) ** 5.255

    return sea_level_pressure


# Prompt the user for each variable and convert to floating-point format
initial_sea_level_pressure = float(input("Enter initial_sea_level_pressure: "))
feet = float(input("Enter altitude: in feet "))
bme680_hpa = float(input("Enter bme680_hpa: "))
bmp390_hpa = float(input("Enter bmp390_hpa: "))
meters = feet / 3.28084

print(f"\n{feet=:.2f}")
print(f"{meters=:.2f}")

# calc altitude based on pressures
initial_bme680_alt = calc_altitude(bme680_hpa, initial_sea_level_pressure)
initial_bmp390_alt = calc_altitude(bmp390_hpa, initial_sea_level_pressure)
print(f"{initial_bme680_alt*3.28084:.1f} feet")
print(f"{initial_bmp390_alt*3.28084:.1f} feet")

print("\nInitial elevations (un-corrected):")
bme680_hpa_adj = calc_sea_level_pressure(bme680_hpa, meters) - initial_sea_level_pressure
bmp390_hpa_adj = calc_sea_level_pressure(bmp390_hpa, meters) - initial_sea_level_pressure

# Print the entered values
print("\nCalibration values:")

print(f"{bme680_hpa_adj=:.4f}")
print(f"{bmp390_hpa_adj=:.4f}")

print(f"Check TODO {initial_sea_level_pressure=}")

