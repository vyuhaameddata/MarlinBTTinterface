import numpy as np
import time
import serial
from decimal import Decimal


class MotionMarlin:
    def __init__(self, motion_params, current_data, queue, timing_manager):
        self.motion_params = motion_params
        self.current_data = current_data
        self.queue = queue
        self.timing_manager = timing_manager
        self.callbacks = []

        self.ser = None
        self.port = "COM7"
        self.baud = 250000
        self.Simulation = motion_params.simulation
        self.device_init = [False, False, False]

    # ---------------- INITIALIZATION ----------------
    def initialize_devices(self):
        try:
            print(f"Connecting to Marlin on {self.port}...")
            self.ser = serial.Serial(self.port, self.baud, timeout=2)
            time.sleep(2)

            self.send_command("M17")
            self.send_command("G90")        # absolute mode
            self.send_command("G21")        # mm
            self.send_command("G92 X0 Y0 Z0")
            self.send_command("M211 S1")

            self.device_init = [True, True, True]
            print("Marlin ready.\n")
            return True
        except Exception as e:
            print("Connection error:", e)
            return False

    # ---------------- BLOCKING SEND ----------------
    def send_command(self, cmd):
        if self.Simulation:
            print("[SIM]", cmd)
            return

        print(">>", cmd)
        self.ser.write((cmd + "\n").encode())

        # wait for ok
        while True:
            line = self.ser.readline().decode().strip()
            if line:
                print("<<", line)
            if "ok" in line:
                break
            if "Error" in line:
                raise RuntimeError(line)

        # HARD wait for motion finish
        print(">> M400")
        self.ser.write(b"M400\n")
        while True:
            line = self.ser.readline().decode().strip()
            if line:
                print("<<", line)
            if "ok" in line:
                break

    # ---------------- MOVE ----------------
    def move(self, x, y, z):
        cmd = f"G1 X{x:.3f} Y{y:.3f} Z{z:.3f} F1800"
        self.send_command(cmd)
        self.current_data.set_x(x)
        self.current_data.set_y(y)
        self.current_data.set_z(z)

    # ---------------- MAIN SCAN ----------------
    def motion_start(self):
        if not any(self.device_init):
            print("Devices not initialized")
            return

        x_start = self.motion_params.x_start
        y_start = self.motion_params.y_start
        z_fixed = self.motion_params.z_start

        xs = [x_start + i * self.motion_params.x_increment
              for i in range(self.motion_params.x_steps)]
        ys = [y_start + i * self.motion_params.y_increment
              for i in range(self.motion_params.y_steps)]

        print("\n--- STARTING SNAKE SCAN ---\n")

        for row, y in enumerate(ys):
            x_iter = xs if row % 2 == 0 else reversed(xs)

            for x in x_iter:
                self.move(x, y, z_fixed)

                if self.callbacks:
                    self.callbacks[0]()

                self.current_data.save_image()
                print(f"Captured at ({x}, {y})\n")

        print("--- SCAN COMPLETE ---")

    # ---------------- SHUTDOWN ----------------
    def disconnect_devices(self):
        if self.ser and self.ser.is_open:
            self.send_command("M18")
            self.ser.close()
            print("Disconnected.")


# ---------------- SUPPORT CLASSES ----------------
class MotionParams:
    def __init__(self):
        self.simulation = False
        self.x_start = -400
        self.y_start = -400
        self.z_start = 0
        self.x_increment = 200
        self.y_increment = 200
        self.x_steps = 5
        self.y_steps = 5


class MockCurrentData:
    def set_x(self, v): pass
    def set_y(self, v): pass
    def set_z(self, v): pass
    def save_image(self): print("   [Image Saved]")


class MockTimingManager:
    def add_move_time(self, v): pass


# ---------------- MAIN ----------------
if __name__ == "__main__":
    params = MotionParams()
    data = MockCurrentData()
    timing = MockTimingManager()
    queue = []

    scanner = MotionMarlin(params, data, queue, timing)

    if scanner.initialize_devices():
        scanner.callbacks.append(lambda: time.sleep(0.5))
        try:
            scanner.motion_start()
        finally:
            scanner.disconnect_devices()
