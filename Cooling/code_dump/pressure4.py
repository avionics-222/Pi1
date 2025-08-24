import spidev
import time

# Initialize SPI (assuming CE0, SPI bus 0, and 8 MHz speed)
spi = spidev.SpiDev()
spi.open(0, 1)  # Bus 0, Device 0 (check your connection for correct bus and device)
spi.max_speed_hz = 1000000  # 1 MHz SPI speed, adjust as needed
spi.mode = 0b01  # SPI Mode 1 (CPOL = 0, CPHA = 1)

# ADS1263 Commands (from datasheet)
# Start with sending a reset or configuration to start continuous conversion
RESET_CMD = [0x06]  # Reset command
START_CMD = [0x08]  # Start continuous conversion command (assuming you're in continuous mode)

# Function to send command to ADS1263
def send_command(cmd):
    response = spi.xfer2(cmd)
    return response

# Function to read 32-bit ADC value (4 bytes)
def read_adc():
    # Send a command to read data (modify according to ADS1263 datasheet)
    response = spi.xfer2([0x01, 0x00, 0x00, 0x00])  # Example, send proper command
    # Combine the 4-byte response to a 32-bit result
    adc_value = (response[1] << 24) | (response[2] << 16) | (response[3] << 8) | response[4]
   
    # If the value is negative (using signed 32-bit data), adjust it
    if adc_value > 2147483647:  # Greater than the 32-bit signed max value
        adc_value -= 4294967296  # Adjust by 2^32 to get the correct signed value

    return adc_value

# Convert ADC value to voltage (assuming 32-bit ADC with 0-5V input range)
def adc_to_voltage(adc_value):
    # For 32-bit ADC (0-4294967295 range), convert to a 0-5V range
    voltage = (adc_value / 4294967295.0) * 5.0

    return voltage

# Convert voltage to pressure (based on 4-20mA -> 1-5V conversion)
def voltage_to_pressure(voltage):
    # Map voltage (1V -> 0 psi, 5V -> 100 psi)
    if voltage < 1.0:
        voltage = 1.0  # Minimum voltage limit
    elif voltage > 5.0:
        voltage = 5.0  # Maximum voltage limit
    pressure = (voltage - 1.0) * 25.0  # 1V corresponds to 0 psi, 5V corresponds to 100 psi
    return pressure

# Main loop
try:
    # Initialize the ADC by sending reset and start commands
    send_command(RESET_CMD)  # Reset ADS1263
    time.sleep(0.5)  # Wait for reset to complete
    send_command(START_CMD)  # Start continuous conversion
   
    while True:
        # Read the raw ADC value
        adc_value = read_adc()
       
        # Convert the raw ADC value to voltage (0-5V range)
        voltage = adc_to_voltage(adc_value)
       
        # Convert the voltage to pressure (in psi)
        pressure = voltage_to_pressure(voltage)
       
        print(f"Voltage: {voltage:.3f} V, Pressure: {pressure:.2f} psi")
       
        time.sleep(1)  # Adjust the reading frequency

except KeyboardInterrupt:
    print("Exiting program.")
finally:
    spi.close()
