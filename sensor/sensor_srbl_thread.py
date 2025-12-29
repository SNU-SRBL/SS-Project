#!/usr/bin/env python
# -*- coding: utf-8 -*-

import threading
import time
from sensor.sensor_srbl import FingerSensor

class FingerSensorThread(threading.Thread):
    def __init__(self, sensor: FingerSensor):
        super().__init__(daemon=True)
        self.sensor = sensor
        self.latest = None
        self.latest_ts = None
        self.lock = threading.Lock()
        self.running = True

    def run(self):

        while self.running:
            try:
                s1, s2 = self.sensor.read_raw()
                ts = time.perf_counter_ns()

                with self.lock:
                    self.latest = (s1.copy(), s2.copy())
                    self.latest_ts = ts

            except Exception as e:
                print(f"[FingerSensorThread] Error: {e}")

    def get_latest(self):
        with self.lock:
            return self.latest, self.latest_ts

    def stop(self):
        self.running = False
        time.sleep(0.5)
        self.sensor.close()


if __name__ == "__main__":
    
    fsensor = FingerSensor(port='COM3', force=False)
    sensor_thread = FingerSensorThread(fsensor)
    sensor_thread.start()

    try:
        next_t = time.perf_counter()
        while True:
            now = time.perf_counter()
            if now < next_t:
                time.sleep(0.001)
                continue
            data, ts = sensor_thread.get_latest()
            if data is not None:
                s1, s2 = data
                print(f"Timestamp: {ts}, Sensor1: {s1}, Sensor2: {s2}")
            next_t += 0.01  # 100 Hz
    except KeyboardInterrupt:
        print("Stopping sensor thread...")
    finally:
        sensor_thread.stop()
        sensor_thread.join()