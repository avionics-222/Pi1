import time
import math
import spidev
from gpiozero import LED, Button

# Use gpiozero for all GPIO operations (more compatible with Pi 5)
# Configure pins
RST_PIN = 18    # Reset pin
CS_PIN = 22     # Chip select pin
DRDY_PIN = 17   # Data ready pin

# Create pin objects using gpiozero
reset_pin = LED(RST_PIN)
cs_pin = LED(CS_PIN)
drdy_pin = Button(DRDY_PIN, pull_up=True)

# ADS1263 configurations
REF_VOLTAGE = 5.0  # Reference voltage (V)
GAIN = 1           # ADC gain
VREF = 2.5         # Internal reference voltage

# RB21C1 Pressure Transmitter specifications
# Adjust these values according to your specific RB21C1 model
PRESSURE_MIN = 0    # Minimum pressure (bar)
PRESSURE_MAX = 10   # Maximum pressure (bar)
CURRENT_MIN = 4     # Minimum current (mA)
CURRENT_MAX = 20    # Maximum current (mA)
SHUNT_RESISTOR = 250  # Shunt resistor value (ohms)

class ADS1263:
    def __init__(self):
        # Initialize SPI
        self.spi = spidev.SpiDev()
        self.spi.open(0, 1)  # SPI bus 0, device 0
        self.spi.max_speed_hz = 1000000  # 1MHz
        self.spi.mode = 1
       
        # Initialize ADS1263
        self.reset()
        self.config_adc()
   
    def reset(self):
        """Reset the ADS1263"""
        reset_pin.on()
        time.sleep(0.1)
        reset_pin.off()
        time.sleep(0.1)
        reset_pin.on()
        time.sleep(0.1)
   
    def send_command(self, command):
        """Send a command to the ADS1263"""
        cs_pin.off()  # CS low
        self.spi.xfer2([command])
        cs_pin.on()   # CS high
   
    def config_adc(self):
        """Configure the ADS1263 ADC"""
        # Reset to default values
        self.send_command(0x06)  # RESET command
        time.sleep(0.1)
       
        # Set MODE0 register: enable internal reference, 60 SPS, continuous conversion
        cs_pin.off()
        self.spi.xfer2([0x43, 0x00, 0x03])  # Write to MODE0 register
        cs_pin.on()
        time.sleep(0.1)
       
        # Set REF register: use internal reference
        cs_pin.off()
        self.spi.xfer2([0x44, 0x00, 0x03])  # Write to REF register
        cs_pin.on()
        time.sleep(0.1)
       
        # Start conversions
        self.send_command(0x08)  # START command
        time.sleep(0.1)
   
    def read_adc(self, channel):
        """Read a single channel from the ADC"""
        # Set channel in MUX register (select A0 for channel 0)
        cs_pin.off()
        self.spi.xfer2([0x41, 0x00, channel])  # Write to MUX register
        cs_pin.on()
        time.sleep(0.1)
       
        # Wait for DRDY to go low
        while drdy_pin.is_pressed:
            pass
       
        # Read data
        cs_pin.off()
        self.spi.xfer2([0x10])  # RDATA command
        buf = self.spi.xfer2([0x00, 0x00, 0x00, 0x00])  # Read 32 bits
        cs_pin.on()
       
        # Convert to voltage
        raw_value = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3]
        # Handle signed values
        if raw_value > 0x7FFFFFFF:
            raw_value = raw_value - 0x100000000
       
        # Convert to voltage
        voltage = (raw_value / 0x7FFFFFFF) * REF_VOLTAGE * GAIN
       
        return voltage

def read_pressure_sensor(adc):
    """Read pressure from the RB21C1 pressure transmitter"""
    try:
        # Read voltage from ADC (A0 channel)
        voltage = adc.read_adc(0)
       
        # Calculate current through the shunt resistor
        current_ma = (voltage / SHUNT_RESISTOR) * 1000
       
        # Convert current to pressure using linear interpolation
        pressure = ((current_ma - CURRENT_MIN) / (CURRENT_MAX - CURRENT_MIN)) * (PRESSURE_MAX - PRESSURE_MIN) + PRESSURE_MIN
       
        return {
            'voltage': voltage,
            'current_ma': current_ma,
            'pressure_bar': pressure,
            'pressure_psi': pressure * 14.5038  # Convert bar to PSI
        }
    except Exception as e:
        print(f"Error reading pressure sensor: {e}")
        return None

def main():
    try:
        print("Initializing ADS1263...")
        # Initialize ADS1263
        adc = ADS1263()
       
        print("RB21C1 Pressure Transmitter Reader")
        print("Press Ctrl+C to exit")
        print("----------------------------------")
       
        while True:
            # Read pressure sensor
            sensor_data = read_pressure_sensor(adc)
           
            if sensor_data:
                print(f"Voltage: {sensor_data['voltage']:.4f} V")
                print(f"Current: {sensor_data['current_ma']:.2f} mA")
                print(f"Pressure: {sensor_data['pressure_bar']:.2f} bar ({sensor_data['pressure_psi']:.2f} PSI)")
                print("----------------------------------")
           
            # Wait before next reading
            time.sleep(1)
   
    except KeyboardInterrupt:
        print("\nProgram stopped by user")
    finally:
        # gpiozero will clean up automatically
        print("Cleaning up...")

if __name__ == "__main__":
    main()
