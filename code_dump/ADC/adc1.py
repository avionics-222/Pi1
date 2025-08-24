
import time
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

SCL_PIN = 3  # GPIO 3 (Pin 5)
SDA_PIN = 2  # GPIO 2 (Pin 3)

# Initialize I2C
i2c = busio.I2C(SCL_PIN, SDA_PIN)

# Initialize ADS1115
ads = ADS.ADS1115(i2c)

# Create an analog input channel on A0
channel = AnalogIn(ads, ADS.P0)

GF = 2.0           # Gauge Factor provided by the manufacturer

try:
    while True:
        voltage = channel.voltage  # Get voltage reading
        strain = ((voltage / 5.0) / GF )* 10**5
        

        #print(f"Voltage: {voltage:.4f} V")
        print(f"Strain Value: {strain:.3f} µε")
        time.sleep(1)

except KeyboardInterrupt:
    print("Exiting...")
