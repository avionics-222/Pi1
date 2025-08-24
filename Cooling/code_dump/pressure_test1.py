import spidev
import time
from gpiozero import OutputDevice

class ADS1263:
    def __init__(self, spi_bus=0, spi_device=1, rst_pin=17):
        self.spi = spidev.SpiDev()
        self.spi.open(spi_bus, spi_device)  # SPI bus 0, chip select 0
        self.spi.max_speed_hz = 1000000  # 1MHz
        self.spi.mode = 0b01  # SPI mode 1
        
        # Reset pin
        self.reset_pin = OutputDevice(rst_pin)
        self.reset()
        
    def reset(self):
        self.reset_pin.off()
        time.sleep(0.1)
        self.reset_pin.on()
        time.sleep(0.1)
    
    def write_register(self, reg, value):
        self.spi.xfer2([0x40 | reg, 0x00, value])  # Write command
    
    def read_register(self, reg):
        response = self.spi.xfer2([0x20 | reg, 0x00, 0x00])  # Read command
        return response[2]
    
    def read_adc(self):
        raw_data = self.spi.xfer2([0x12, 0x00, 0x00, 0x00, 0x00])  # Read ADC data command
        adc_value = (raw_data[1] << 24) | (raw_data[2] << 16) | (raw_data[3] << 8) | raw_data[4]
        return adc_value if adc_value < (1 << 31) else adc_value - (1 << 32)  # Convert to signed integer
    
    def close(self):
        self.spi.close()

# Example usage
if __name__ == "__main__":
    adc = ADS1263()
    try:
        while True:
            value = adc.read_adc()
            print(f"ADC Value: {value}")
            time.sleep(1)
    except KeyboardInterrupt:
        adc.close()
        print("ADC closed.")
