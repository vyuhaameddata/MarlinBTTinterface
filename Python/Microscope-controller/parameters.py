# parameters.py

class MotionParams:
    def __init__(self):
        # --- Connection Settings ---
        self.port = "COM7"
        self.baud = 250000
        self.simulation = False

        # --- Grid Scan Settings ---
        self.x_start = -400
        self.y_start = -400
        self.z_start = 0.0
        
        self.x_increment = 200
        self.y_increment = 200
        
        self.x_steps = 5  # Results in -400, -200, 0, 200, 400
        self.y_steps = 5
        
        # --- Focus & Hardware Tuning ---
        self.z_range = 2.0         # How far to search for focus
        self.y_tilt_error = 0.01    # Adjust for physical bed tilt
        
        # --- Initial Calibration Points ---
        # Points used to build the first Z-Map (Topography)
        self.x_values_coordinates = [-400, 400, -400, 400, 0]
        self.y_values_coordinates = [-400, -400, 400, 400, 0]

        # --- Focus Model Thresholds ---
        self.refinement_threshold = 0.005 # If displacement > this, refine Z