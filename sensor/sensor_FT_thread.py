#!/usr/bin/env python
# -*- coding: utf-8 -*-

import threading
import time
from sensor.sensor_FT import SensorFT

class SensorFTThread(threading.Thread):
    def __init__(self, sensor: SensorFT):
        super().__init__(daemon=True)
        self.sensor = sensor
        self.latest = None
        self.latest_ts = None
        self.lock = threading.Lock()
        self.running = True

    def run(self):

        while self.running:
            try:
                data = self.sensor.read()
                ts = time.perf_counter_ns()

                with self.lock:
                    self.latest = data.copy()
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

    HZ = 100
    DT = 1.0 / HZ
    
    sensor_ft = SensorFT(port="COM5")
    sensor_thread = SensorFTThread(sensor_ft)
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
                Fx, Fy, Fz, Mx, My, Mz = data
                print(f"Timestamp: {ts}, Fx: {Fx:.2f}, Fy: {Fy:.2f}, Fz: {Fz:.2f} | Mx: {Mx:.3f}, My: {My:.3f}, Mz: {Mz:.3f}")
            
            next_t += DT

    except KeyboardInterrupt:
        print("Stopping sensor thread...")
    finally:
        sensor_thread.stop()
        sensor_thread.join()