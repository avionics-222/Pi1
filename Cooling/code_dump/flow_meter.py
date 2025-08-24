from gpiozero import Button
import time
from datetime import datetime

# Define GPIO pin
FLOW_SENSOR_PIN = 23  # GPIO23
pulse_count = 0

# Interrupt-based pulse counter
def count_pulse():
    global pulse_count
    pulse_count += 1

# Initialize flow sensor as a button (falling edge detection)
flow_sensor = Button(FLOW_SENSOR_PIN, pull_up=True)
flow_sensor.when_pressed = count_pulse

def get_flow_rate():
    global pulse_count
    # Convert pulses to flow rate using formula
    flow_rate = (pulse_count / 8.58)  # L/min
    pulse_count = 0  # Reset count after reading
    return round(flow_rate, 2)

try:
    while True:
        time.sleep(1)  # 1-second interval
        flow = get_flow_rate()
        
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # Up to milliseconds
        
        # Print result
        print(f"[{timestamp}] : Flow rate = {flow:.2f} L/min")


except KeyboardInterrupt:
    print("Measurement stopped.")
