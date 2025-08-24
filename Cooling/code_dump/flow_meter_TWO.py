from gpiozero import Button
import time
from datetime import datetime

# Define GPIO pins for two flow sensors
FLOW_SENSOR_1_PIN = 23  # GPIO23
FLOW_SENSOR_2_PIN = 24  # GPIO24

# Pulse counters for each sensor
pulse_count_1 = 0
pulse_count_2 = 0

# Interrupt-based pulse counters
def count_pulse_1():
    global pulse_count_1
    pulse_count_1 += 1

def count_pulse_2():
    global pulse_count_2
    pulse_count_2 += 1

# Initialize flow sensors as buttons (falling edge detection)
flow_sensor_1 = Button(FLOW_SENSOR_1_PIN, pull_up=True)
flow_sensor_2 = Button(FLOW_SENSOR_2_PIN, pull_up=True)

flow_sensor_1.when_pressed = count_pulse_1
flow_sensor_2.when_pressed = count_pulse_2

def get_flow_rates():
    global pulse_count_1, pulse_count_2
    
    # Convert pulses to flow rate using formula
    flow_rate_1 = (pulse_count_1 / 8.58)  # L/min    [3.4 , 5.5, 6, 8.58 ]
    flow_rate_2 = (pulse_count_2 / 8.58)  # L/min
    
    # Reset counters after reading
    pulse_count_1 = 0
    pulse_count_2 = 0
    
    return round(flow_rate_1, 2), round(flow_rate_2, 2)

try:
    while True:
        time.sleep(1)  # 1-second interval
        flow_1, flow_2 = get_flow_rates()
        
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # Up to milliseconds
        
        # Print results
        print(f"[{timestamp}] : Flow rate 1 = {flow_1:.2f} L/min | Flow rate 2 = {flow_2:.2f} L/min")

except KeyboardInterrupt:
    print("Measurement stopped.")
