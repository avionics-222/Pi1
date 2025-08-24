# MAX31865 with PT100 on Raspberry Pi 5 (Two-Wire Configuration)

import spidev
import time

# SPI Configuration
SPI_BUS = 0
CS_PIN = 1  # SPI0.1
MAX_SPEED_HZ = 500000

# RTD and Reference Resistor Values
RREF = 430.0
RNOMINAL = 100.0

# Initialize SPI
spi = spidev.SpiDev()
spi.open(SPI_BUS, CS_PIN)
spi.max_speed_hz = MAX_SPEED_HZ
spi.mode = 0b01

# Read registers from MAX31865
def read_registers(reg, length):
    result = spi.xfer2([reg] + [0x00] * length)
    return result[1:]

# Write to a register
def write_register(reg, data):
    spi.xfer2([reg | 0x80, data])  # Write command: MSB set to 1

# Configure MAX31865 for 2-wire RTD with continuous conversion
def configure_max31865():
    # 0xA2: V_BIAS on, Auto conversion, 2-wire, Fault detection disabled
    write_register(0x00, 0xA2)  

# Trigger a single temperature conversion
def trigger_conversion():
    # Start a new conversion
    write_register(0x00, 0xA2)  # Set V_BIAS on and auto-conversion
    time.sleep(0.1)  # Allow time for conversion to complete

# Calculate temperature from RTD resistance
def calculate_temperature(resistance):
    rtd_ratio = resistance / RNOMINAL
    temperature = ((rtd_ratio - 1) / 0.00385 ) - 3.0   #Calibration Factor //3.3 //3.0
    return temperature

# Read temperature from MAX31865
def read_temperature():
    # Trigger a new conversion
    trigger_conversion()

    # Read RTD MSB and LSB (2 bytes)
    data = read_registers(0x01, 2)
    rtd_adc_code = ((data[0] << 8) | data[1]) >> 1  # 15-bit RTD value
    resistance = (rtd_adc_code * RREF) / 32768.0
    temperature = calculate_temperature(resistance)
    return temperature, resistance

# Main
try:
    print("Configuring MAX31865 module...")
    configure_max31865()
    print("Configuration successful!")

    while True:
        temp, res = read_temperature()
        print(f"Temperature: {temp:.2f} °C | Resistance: {res:.2f} Ω")
        print("----------------------------------------")
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Program terminated.")
finally:
    spi.close()

