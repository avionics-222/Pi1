
import time
import ADS1263
import RPi.GPIO as GPIO
from gpiozero import Button
import sys
import csv
from datetime import datetime
import os
import threading
import spidev

# ====================== Configuration ======================

# ADC / Sensor Reference Voltage

REF = 5.00  # ADC and sensor supply reference voltage (5V)



# GPIO pins for flow sensors

FLOW_SENSOR_1_PIN = 23

FLOW_SENSOR_2_PIN = 24



# MAX31865 (RTD Temperature Sensor) SPI config

SPI_BUS = 0

CS_PINS = [0]        # CE0 and CE1 chip selects for two MAX31865 units

MAX_SPEED_HZ = 500000

RREF = 430.0

RNOMINAL = 100.0

test_mode = 0 
if test_mode == 1:
    print("_______________TEST MODE___________")


# =================== Global variables ======================



# Flow & Pulse counts

pulse_count_1 = 0

pulse_count_2 = 0



# Sensor readings (with initial dummy values)

flow_rate_1 = 0.0

pressure_bar_1 = 0.0
pbar_1 = 0.0


flow_rate_2 = 0.0

pressure_bar_2 = 0.0
pbar_2 = 0.0



# Temperature sensor readings

temperature_1 = 0.0

temperature_2 = 0.0



# ADC Voltage (kept internally, no print required)

voltage_1 = 0.0

voltage_2 = 0.0



# Thread locks

data_lock = threading.Lock()



# SPI device handles list

spis = []



# ==================== Sensor location & log file setup ====================

loc_fil = input('Sensor Location: ')

log_dir = "logs_cooling"

os.makedirs(log_dir, exist_ok=True)

filename = os.path.join(

    log_dir, f"{loc_fil}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

)



# =============================== FLOW SENSOR COUNTING ====================

def count_pulse_1():

    global pulse_count_1

    pulse_count_1 += 1



def count_pulse_2():

    global pulse_count_2

    pulse_count_2 += 1



flow_sensor_1 = Button(FLOW_SENSOR_1_PIN, pull_up=True)

flow_sensor_1.when_pressed = count_pulse_1



flow_sensor_2 = Button(FLOW_SENSOR_2_PIN, pull_up=True)

flow_sensor_2.when_pressed = count_pulse_2



def get_flow_rate_1():

    global pulse_count_1

    flow = (pulse_count_1 / 9.9)  # L/min conversion factor

    pulse_count_1 = 0

    return round(flow, 2)



def get_flow_rate_2():

    global pulse_count_2

    flow = (pulse_count_2 / 9.9)  # L/min conversion factor

    pulse_count_2 = 0

    return round(flow, 2)



# ============================= PRESSURE SENSOR THREAD ======================

def pressure_sensor_thread():
    if test_mode == 1:
        return

    global pressure_bar_1, voltage_1, pressure_bar_2, voltage_2

    try:

        ADC = ADS1263.ADS1263()

        if ADC.ADS1263_init_ADC1('ADS1263_50SPS') == -1:

            print("Failed to initialize ADC")

            return

        ADC.ADS1263_SetMode(0)  # Single-ended mode

        channelList = [0, 1]  # Channels for sensor 1 and 2

        while True:

            ADC_Value = ADC.ADS1263_GetAll(channelList)



            # Sensor 1 processing

            raw_value_1 = ADC_Value[0]

            if raw_value_1 & 0x80000000:

                raw_value_1 -= 1 << 32

            current_voltage_1 = (raw_value_1 / 2147483648.0) * REF

            if current_voltage_1 < 0.5:

                pressure_psi_1 = 0.0

            elif current_voltage_1 > 4.5:

                pressure_psi_1 = 100.0

            else:

                pressure_psi_1 = ((current_voltage_1 - 0.5) / 4.0) * 100.0

            pbar_1 = pressure_psi_1 * 0.0689
            current_pressure_bar_1 = (.82 * pbar_1) - 0.017
            
            if(current_pressure_bar_1 < 0):
                current_pressure_bar_1 = 0


            # Sensor 2 processing

            raw_value_2 = ADC_Value[1]

            if raw_value_2 & 0x80000000:

                raw_value_2 -= 1 << 32

            current_voltage_2 = (raw_value_2 / 2147483648.0) * REF

            if current_voltage_2 < 0.5:

                pressure_psi_2 = 0.0

            elif current_voltage_2 > 4.5:

                pressure_psi_2 = 100.0

            else:

                pressure_psi_2 = ((current_voltage_2 - 0.5) / 4.0) * 100.0

            pbar_2 = pressure_psi_2 * 0.0689
            current_pressure_bar_2 = (.82 * pbar_2) - 0.017
            
            if(current_pressure_bar_2 < 0):
                current_pressure_bar_2 = 0



            with data_lock:

                pressure_bar_1 = current_pressure_bar_1

                voltage_1 = current_voltage_1

                pressure_bar_2 = current_pressure_bar_2

                voltage_2 = current_voltage_2



            time.sleep(0.1)

    except Exception as e:

        print(f"Pressure sensor error: {e}")

    finally:

        try:

            ADC.ADS1263_Exit()

        except:

            pass



# ============================ FLOW SENSOR THREAD ===========================

def flow_sensor_thread():
    if test_mode ==1 :
        return

    global flow_rate_1, flow_rate_2

    try:

        while True:

            time.sleep(1)

            current_flow_1 = get_flow_rate_1()

            current_flow_2 = get_flow_rate_2()

            with data_lock:

                flow_rate_1 = current_flow_1

                flow_rate_2 = current_flow_2

    except Exception as e:

        print(f"Flow sensor error: {e}")



# =============================== MAX31865 TEMP SENSOR =====================
def init_spi_devices1():
    global spis
    cs = 0
    #for cs in CS_PINS:

    spi = spidev.SpiDev()

    spi.open(SPI_BUS, cs)

    spi.max_speed_hz = MAX_SPEED_HZ

    spi.mode = 0b01

    spis=[spi]
       # print(spis)



def init_spi_devices2():
    global spis
    cs = 1
    #for cs in CS_PINS:

    spi = spidev.SpiDev()

    spi.open(SPI_BUS, cs)

    spi.max_speed_hz = MAX_SPEED_HZ

    spi.mode = 0b01

    spis=[spi]



def read_registers(spi, reg, length):

    result = spi.xfer2([reg] + [0x00] * length)

    return result[1:]



def write_register(spi, reg, data):

    spi.xfer2([reg | 0x80, data])



def configure_max31865(spi):

    # Configuration: V_BIAS on, Auto convert, 2-wire (adjust if wiring differs)

    write_register(spi, 0x00, 0xD2)



def trigger_conversion(spi):

    write_register(spi, 0x00, 0xA2)

    time.sleep(0.1)



def calculate_temperature(resistance):

    rtd_ratio = resistance / RNOMINAL

    temperature = ((rtd_ratio - 1) / 0.00385)

    return temperature



def read_temperature(spi):

    trigger_conversion(spi)

    data = read_registers(spi, 0x01, 2)

    rtd_adc_code = ((data[0] << 8) | data[1]) >> 1

    resistance = (rtd_adc_code * RREF) / 32768.0

    temperature = calculate_temperature(resistance)

    return temperature, resistance



def temperature_sensor_thread():

    global temperature_1, temperature_2

    try:

        #init_spi_devices1()

        #for spi in spis:

        #configure_max31865(spi)

        while True:
            init_spi_devices1()
            #print(spis)
            for spi in spis:

                configure_max31865(spi)
                #print(12324)

            try:

                temp_1, _ = read_temperature(spi)
                #print(13412)
                

            except Exception:

                temp_1 = float('nan')
            for spi in spis:
                spi.close()
            #spis=[]
            time.sleep(1)
            init_spi_devices2()

            for spi in spis:

                configure_max31865(spi)
            try:

                temp_2, _ = read_temperature(spi)

            except Exception:

                temp_2 = float('nan')
            for spi in spis:
                spi.close()
            #spis=[]
            with data_lock:

                temperature_1 = temp_1 - 4.73

                temperature_2 = temp_2 - 5.00

            time.sleep(1.0)

    except Exception as e:

        print(f"Temperature sensor error: {e}")

    #finally:

        #spi.close()


# ============================= MAIN FUNCTION ==============================

def main():

    global pressure_bar_1, flow_rate_1, temperature_1

    global pressure_bar_2, flow_rate_2, temperature_2



    try:

        # Start threads

        pressure_thread = threading.Thread(target=pressure_sensor_thread, daemon=True)

        flow_thread = threading.Thread(target=flow_sensor_thread, daemon=True)

        temperature_thread = threading.Thread(target=temperature_sensor_thread, daemon=True)

        

        pressure_thread.start()

        

        flow_thread.start()

        

        temperature_thread.start()
        

        time.sleep(2)  # Allow sensors to initialize



        with open(filename, mode='w', newline='') as file:

            writer = csv.writer(file)

            # Write CSV header

            writer.writerow([

                "Timestamp",

                "Pressure_1(Bar)", "Flow_1(L/min)", "Temperature_1(C)",

                "Pressure_2(Bar)", "Flow_2(L/min)", "Temperature_2(C)"

            ])

            print(f"Logging to {filename}...")

            print("Press Ctrl+C to stop.")

            print("Timestamp                           | Sensor 1                   | Sensor 2")

            print("                                   | Press(Bar) | Flow(L/min) | Temp(C) | Press(Bar) | Flow(L/min) | Temp(C)")

            print("-" * 98)



            last_print = time.time()



            while True:

                timestamp = datetime.now()

                timestamp_str = timestamp.isoformat()



                with data_lock:

                    p1 = pressure_bar_1

                    f1 = flow_rate_1

                    t1 = temperature_1

                    p2 = pressure_bar_2

                    f2 = flow_rate_2

                    t2 = temperature_2



                # Write to CSV

                writer.writerow([

                    timestamp_str,

                    f"{p1:.3f}",

                    f"{f1:.2f}",

                    f"{t1:.2f}",

                    f"{p2:.3f}",

                    f"{f2:.2f}",

                    f"{t2:.2f}"

                ])



                # Print formatted output without voltage, with labels and pipe separators

                if time.time() - last_print >= 0.5:

                    print(f"[P1]: {p1:.2f} Bar | [F1]: {f1:.2f} L/min | [T1]: {t1:.2f} °C | "

                          f"[P2]: {p2:.2f} Bar | [F2]: {f2:.2f} L/min | [T2]: {t2:.2f} °C")

                    last_print = time.time()



                file.flush()

                time.sleep(0.1)



    except IOError as e:

        print(f"IOError: {e}")

    except KeyboardInterrupt:

        print("\nCtrl+C: Exiting program...")

    except Exception as e:

        print(f"Unexpected error: {e}")

    finally:

        try:

            flow_sensor_1.close()

            flow_sensor_2.close()

        except:

            pass

        for spi in spis:

            try:

                spi.close()

            except:

                pass

        print("Program terminated.")



if __name__ == "__main__":

    main()

