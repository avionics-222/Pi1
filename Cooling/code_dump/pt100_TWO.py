# MAX31865 with Two PT100 Sensors on Raspberry Pi 5 (Two-Wire Configuration)

import spidev
import time

# SPI Configuration
SPI_BUS = 0
CS_PINS = [0, 1]  # CS0 (Pin 24) for sensor 1, CS1 (Pin 26) for sensor 2
MAX_SPEED_HZ = 500000

# RTD and Reference Resistor Values
RREF = 430.0
RNOMINAL = 100.0

# Initialize SPI
spi = spidev.SpiDev()
spi.open(SPI_BUS, CS_PINS[0])
spi.max_speed_hz = MAX_SPEED_HZ
spi.mode = 0b01

# Initialize the second SPI instance for the second sensor
spi2 = spidev.SpiDev()
spi2.open(SPI_BUS, CS_PINS[1])
spi2.max_speed_hz = MAX_SPEED_HZ
spi2.mode = 0b01

# Read registers from MAX31865
def read_registers(spi_device, reg, length):
    result = spi_device.xfer2([reg] + [0x00] * length)
    return result[1:]

# Write to a register
def write_register(spi_device, reg, data):
    spi_device.xfer2([reg | 0x80, data])  # Write command: MSB set to 1

# Configure MAX31865 for 2-wire RTD with continuous conversion
def configure_max31865(spi_device):
    write_register(spi_device, 0x00, 0xA2)  # V_BIAS on, Auto conversion, 2-wire

# Trigger a single temperature conversion
def trigger_conversion(spi_device):
    write_register(spi_device, 0x00, 0xA2)
    time.sleep(0.1)

# Calculate temperature from RTD resistance
def calculate_temperature(resistance):
    rtd_ratio = resistance / RNOMINAL
    temperature = ((rtd_ratio - 1) / 0.00385) - 3.0  # Calibration Factor
    return temperature

# Read temperature from MAX31865
def read_temperature(spi_device):
    trigger_conversion(spi_device)
    data = read_registers(spi_device, 0x01, 2)
    rtd_adc_code = ((data[0] << 8) | data[1]) >> 1  # 15-bit RTD value
    resistance = (rtd_adc_code * RREF) / 32768.0
    temperature = calculate_temperature(resistance)
    return temperature, resistance

# Main
try:
    print("Configuring MAX31865 modules...")
    configure_max31865(spi)
    configure_max31865(spi2)
    print("Configuration successful!")

    while True:
        # Read temperature from both sensors
        temp1, res1 = read_temperature(spi)
        temp2, res2 = read_temperature(spi2)

        print(f"Sensor 1 - Temperature: {temp1:.2f} °C")
        print(f"Sensor 2 - Temperature: {temp2:.2f} °C")
        print("----------------------------------------")
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Program terminated.")
finally:
    spi.close()
    spi2.close()
