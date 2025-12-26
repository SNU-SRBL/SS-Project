import csv
import time
from sensor.sensor_FT import SensorFT
from sensor.sensor_srbl import FingerSensor
from sensor.sensor_FT_thread import SensorFTThread
from sensor.sensor_srbl_thread import FingerSensorThread

HZ = 100
DT = 1.0 / HZ

log_id = 0

sensor_ft = SensorFT(port="/dev/ttyUSB1")
rft = SensorFTThread(sensor_ft)
sensor_srbl = FingerSensor(port="/dev/ttyUSB0", force=False)
hall = FingerSensorThread(sensor_srbl)

rft.start()
hall.start()

print("Warming up sensors...")
time.sleep(2.0)  # Allow sensors to initialize
print("Starting data logging...")

with open(f"./calibration/sensor_data_log_{log_id}.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow([
        "time(s)",
        "fx","fy","fz","tx","ty","tz",
        "hall1","hall2","hall3"
    ])

    start_time = time.perf_counter()
    next_time = time.perf_counter()

    try:
        while True:
            now = time.perf_counter()
            if now < next_time:
                time.sleep(next_time - now)
                continue

            t = f"{(now - start_time):.3f}"

            rft_data, ts_rft = rft.get_latest()
            hall_data, ts_hall = hall.get_latest()
            hall_data_0, hall_data_1 = hall_data


            if rft_data is not None and hall_data is not None:

                rft_formatted = [f"{val:.2f}" for val in rft_data]
                hall_formatted = [f"{val:.2f}" for val in hall_data_0]
                row = [t] + rft_formatted + hall_formatted

                writer.writerow(row)
                pass

            next_time += DT

    except KeyboardInterrupt:
        print("Stopping...")
