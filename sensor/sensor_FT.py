import serial
import time
import numpy as np

DF = 50.0                         # Force Divider
DT = 2000.0                       # Torque Divider

class SensorFT:
    def __init__(self, port="COM5", baudrate=921600):
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        time.sleep(1)

        self.df = DF 
        self.dt = DT 

        # Command packet (Read Request)
        self.cmd_get_data = bytes([0x55, 0x0A, 0x02, 0x00, 0x00, 0x00,
                                   0x00, 0x00, 0x00, 0x0C, 0xAA])

        # Sensor init commands
        self.ser.write(bytes([0x55, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0xAA]))  # Stop Output
        time.sleep(0.1)
        self.ser.write(bytes([0x55, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xAA]))  # Output: 200 Hz
        time.sleep(0.1)
        self.ser.write(bytes([0x55, 0x08, 0x01, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0xAA]))  # LPF: 30 Hz
        time.sleep(0.1)
        self.ser.write(bytes([0x55, 0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xAA]))  # Set Bias
        time.sleep(0.5)

        # Clear buffer to remove any old garbage data before starting
        self.ser.reset_input_buffer()

        self.offset_force = np.zeros(3)
        self.offset_torque = np.zeros(3)
        self._calibrate_offset()

        print("Sensor FT Started.")

    def _read_packet_aligned(self):
        """
        Robust read: 
        1. Read 1 byte at a time until Header (0x55) is found.
        2. Read the remaining 18 bytes.
        """
        # 1. Find Header
        while True:
            # Read 1 byte
            header = self.ser.read(1)
            
            # If timeout/empty, return None
            if not header:
                return None
            
            # If header is found, break loop
            if header == b'\x55':
                break
        
        # 2. Read Data Field (18 bytes)
        data_field = self.ser.read(18)
        
        # Check if we got the full 18 bytes
        if len(data_field) != 18:
            return None

        # Return combined bytes to match your parsing indices
        return b'\x55' + data_field

    def _calibrate_offset(self):
        print("Calibrating...")
        force_sum = np.zeros(3)
        torque_sum = np.zeros(3)
        valid = 0
        
        for _ in range(100):
            self.ser.write(self.cmd_get_data) # Request data
            
            data = self._read_packet_aligned() # Use aligned read
            
            if data is None:
                continue
                
            valid += 1
            force_sum += np.array([
                int.from_bytes(data[2:4], byteorder='big', signed=True),
                int.from_bytes(data[4:6], byteorder='big', signed=True),
                int.from_bytes(data[6:8], byteorder='big', signed=True)
            ])
            torque_sum += np.array([
                int.from_bytes(data[8:10], byteorder='big', signed=True),
                int.from_bytes(data[10:12], byteorder='big', signed=True),
                int.from_bytes(data[12:14], byteorder='big', signed=True)
            ])
            time.sleep(0.002) # Small delay

        if valid > 0:
            self.offset_force = force_sum / valid
            self.offset_torque = torque_sum / valid
            print(f"Calibration Done (Samples: {valid})")
        else:
            print("Warning: No valid frames for offset calibration")

    def read(self):
        # 1. Send Request Command
        self.ser.write(self.cmd_get_data)
        
        # 2. Read aligned data (Header + 18 bytes)
        data = self._read_packet_aligned()
        
        if data is None:
            return None 

        # 3. Parse Data
        fx = int.from_bytes(data[2:4], byteorder='big', signed=True)
        fy = int.from_bytes(data[4:6], byteorder='big', signed=True)
        fz = int.from_bytes(data[6:8], byteorder='big', signed=True)
        mx = int.from_bytes(data[8:10], byteorder='big', signed=True)
        my = int.from_bytes(data[10:12], byteorder='big', signed=True)
        mz = int.from_bytes(data[12:14], byteorder='big', signed=True)

        force = np.array([fx, fy, fz]) - self.offset_force
        torque = np.array([mx, my, mz]) - self.offset_torque

        return np.concatenate([force / self.df, torque / self.dt])

    def close(self):
        self.ser.close()


if __name__ == "__main__":
    # Ensure port is correct for your system
    sensor_ft = SensorFT(port="COM5")

    try:
        while True:
            data = sensor_ft.read()
            if data is not None:
                Fx, Fy, Fz, Mx, My, Mz = data
                print(f"Fx: {Fx:.2f}, Fy: {Fy:.2f}, Fz: {Fz:.2f} | Mx: {Mx:.3f}, My: {My:.3f}, Mz: {Mz:.3f}")
            
            # Since we are polling (Request/Response), we control the loop speed here
            time.sleep(0.01)  # ~100 Hz
    except KeyboardInterrupt:
        print("Exiting...")
        sensor_ft.close()