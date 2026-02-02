import numpy as np
import time
import serial
from decimal import Decimal
from scipy.interpolate import griddata
from parameters import parameters as MotionParams

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
        
        # Tilt and Focus Params
        self.y_tilt_error = float(getattr(motion_params, 'y_tilt_error', 0.0))

    # ---------------- INITIALIZATION ----------------
    def initialize_devices(self):
        if self.Simulation:
            print("[SIM] Marlin Simulation Mode Active.")
            self.device_init = [True, True, True]
            return True
        try:
            print(f"Connecting to Marlin on {self.port}...")
            self.ser = serial.Serial(self.port, self.baud, timeout=2)
            time.sleep(2)

            self.send_command("M17")        # Enable motors
            self.send_command("G90")        # Absolute mode
            self.send_command("G21")        # mm
            self.send_command("G92 X0 Y0 Z0")
            self.send_command("M211 S1")    # Software endstops ON

            self.device_init = [True, True, True]
            print("Marlin ready.\n")
            return True
        except Exception as e:
            print("Connection error:", e)
            return False

    # ---------------- COMMUNICATION ----------------
    def send_command(self, cmd):
        if self.Simulation:
            print("[SIM]", cmd)
            return

        self.ser.write((cmd + "\n").encode())
        # Wait for 'ok' to ensure command received
        while True:
            line = self.ser.readline().decode().strip()
            if "ok" in line: break
            if "Error" in line: raise RuntimeError(line)

        # Ensure motion is actually finished (M400 is wait-for-moves)
        self.ser.write(b"M400\n")
        while True:
            line = self.ser.readline().decode().strip()
            if "ok" in line: break

    def move(self, x, y, z):
        # Format to 3 decimal places for Marlin
        cmd = f"G1 X{float(x):.3f} Y{float(y):.3f} Z{float(z):.3f} F1800"
        self.send_command(cmd)
        self.current_data.set_x(x)
        self.current_data.set_y(y)
        self.current_data.set_z(z)

    # ---------------- FOCUS LOGIC ----------------
    def z_start_optimal(self, x_pos, y_pos):
        """Coarse autofocus sweep using Tamura metric."""
        print(f"Starting coarse sweep at ({x_pos}, {y_pos})")
        prev_tamura = 0
        self.current_data.reset_best_tamura()
        
        z_start = float(self.motion_params.z_start)
        z_range = float(self.motion_params.z_range)
        z_pos = z_start + z_range
        prev_z = z_pos

        while z_pos >= (z_start - z_range):
            if self.current_data.get_stop_acquisition(): break
            
            self.move(x_pos, y_pos, z_pos)
            if self.callbacks: self.callbacks[0]()
            
            current_tamura = self.current_data.get_tamura()
            if current_tamura < prev_tamura: # Peak passed
                break
            
            prev_tamura = current_tamura
            prev_z = z_pos
            z_pos -= 0.05 # Coarse step size
            
        return prev_z

    def z_movement_optimal(self, x_pos, y_pos, is_refinement, z_interp):
        """Fine autofocus using model-predicted displacement."""
        z_curr = float(z_interp) if is_refinement else float(self.motion_params.z_start)
        max_iter = 3 if is_refinement else 5
        
        for i in range(max_iter):
            if self.current_data.get_stop_acquisition(): break
            
            self.move(x_pos, y_pos, z_curr)
            if self.callbacks: self.callbacks[0]()
            
            displacement = self.current_data.get_zmove()
            print(f"Model displacement: {displacement} at Z: {z_curr}")
            
            # Threshold check
            threshold = 0.007 if is_refinement else 0.001
            if abs(displacement) <= threshold:
                return z_curr
            
            z_curr += displacement
            
        # Fallback to coarse if fine search fails
        return self.z_start_optimal(x_pos, y_pos)

    # ---------------- MAIN SCAN ----------------
    def motion_start(self):
        if not any(self.device_init):
            print("Devices not initialized")
            return

        # 1. PRE-CALCULATE GRID
        num_cols = self.motion_params.x_steps
        num_rows = self.motion_params.y_steps
        x_coords = self.motion_params.x_start + np.arange(num_cols) * self.motion_params.x_increment
        y_coords = self.motion_params.y_start + np.arange(num_rows) * self.motion_params.y_increment
        
        # 2. INITIAL FOCUS POINTS (Z-Sensing)
        # Using the first few points to build a Z-map
        x_ref = np.array(self.motion_params.x_values_coordinates)
        y_ref = np.array(self.motion_params.y_values_coordinates)
        z_ref = []
        
        print("Gathering reference Z focus points...")
        for xr, yr in zip(x_ref, y_ref):
            zf = self.z_movement_optimal(xr, yr, False, 0)
            z_ref.append(zf)

        # 3. INTERPOLATE Z-MAP
        grid_x, grid_y = np.meshgrid(x_coords, y_coords)
        points = np.stack((x_ref, y_ref), axis=-1)
        grid_z = griddata(points, z_ref, (grid_x, grid_y), method='cubic')
        
        # Fill NaNs with nearest neighbor
        if np.any(np.isnan(grid_z)):
            grid_z_near = griddata(points, z_ref, (grid_x, grid_y), method='nearest')
            grid_z[np.isnan(grid_z)] = grid_z_near[np.isnan(grid_z)]

        # 4. SNAKE SCAN
        print("\n--- STARTING FOCUS-AWARE SNAKE SCAN ---")
        row_start_y = float(y_coords[0])

        for r in range(num_rows):
            forward = (r % 2 == 0)
            col_range = range(num_cols) if forward else reversed(range(num_cols))
            
            for step_idx, c in enumerate(col_range):
                if self.current_data.get_stop_acquisition(): break
                
                x_target = float(x_coords[c])
                y_target = row_start_y + (step_idx * self.y_tilt_error if forward else -step_idx * self.y_tilt_error)
                z_interp = float(grid_z[r, c])

                # Move to interpolated position
                self.move(x_target, y_target, z_interp)
                if self.callbacks: self.callbacks[0]()

                # Focus Validation
                disp = abs(self.current_data.get_zmove())
                if 0.005 < disp < 0.02: 
                    print(f"Refining Z at Row {r}, Col {c}...")
                    z_final = self.z_movement_optimal(x_target, y_target, True, z_interp)
                    self.move(x_target, y_target, z_final)
                
                self.current_data.save_image()
                print(f"Captured at ({x_target:.2f}, {y_target:.2f})\n")

            # Increment Y for next row
            y_inc = float(self.motion_params.y_increment)
            tilt_span = (num_cols - 1) * self.y_tilt_error
            row_start_y += (y_inc + tilt_span if forward else y_inc - tilt_span)

        print("--- SCAN COMPLETE ---")

    def disconnect_devices(self):
        if self.ser and self.ser.is_open:
            self.send_command("M18") # Disable steppers
            self.ser.close()
            print("Disconnected.")

# ---------------- UPDATED SUPPORT CLASSES ----------------

class MockCurrentData:
    def __init__(self): self.stop = False
    def set_x(self, v): pass
    def set_y(self, v): pass
    def set_z(self, v): pass
    def get_stop_acquisition(self): return self.stop
    def get_tamura(self): return np.random.random() # Mock focus metric
    def reset_best_tamura(self): pass
    def get_zmove(self): return np.random.uniform(-0.01, 0.01) # Mock model prediction
    def save_image(self): print("   [Image Saved]")

class MockTimingManager:
    def add_move_time(self, v): pass

if __name__ == "__main__":
    params = MotionParams()
    data = MockCurrentData()
    timing = MockTimingManager()
    
    scanner = MotionMarlin(params, data, [], timing)

    if scanner.initialize_devices():
        scanner.callbacks.append(lambda: time.sleep(0.1))
        try:
            scanner.motion_start()
        finally:
            scanner.disconnect_devices()