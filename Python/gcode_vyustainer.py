import time
import serial

# -----------------------------
# User Configuration Section
# -----------------------------
PORT = "COM4"
BAUD = 115200

# Safety Z height before moving XY
SAFE_Z = 100        # mm (adjust as needed)

# Pre-start G-code
START_GCODE = [
    "G21",            # set units to mm
    "G90",            # absolute positioning
    "M17",            # enable motors
]

# 12 JAR POSITIONS (editable)
JAR_POSITIONS = {
    1:  (10,  20),
    2:  (30,  20),
    3:  (50,  20),
    4:  (70,  20),
    5:  (90,  20),
    6:  (110, 20),
    7:  (10,  60),
    8:  (30,  60),
    9:  (50,  60),
    10: (70, 60),
    11: (90, 60),
    12: (110,60)
}

# Example soak times (seconds)
JAR_SOAK_TIME = {
    1:  10,
    2:  20,
    3:  10,
    4:  30,
    5:  15,
    6:  25,
    7:  20,
    8:  20,
    9:  10,
    10: 20,
    11: 10,
    12: 30
}

# Configure the ORDER in which jars will run
SEQUENCE = [1,2,5,10,3,7,12]   # you may reorder any way you want


# ----------------------------------------
# Helper Function – Send Gcode
# ----------------------------------------
def send(ser, gcode, wait=True):
    ser.write((gcode + "\n").encode())
    print("→", gcode)
    if wait:
        ser.flush()
        time.sleep(0.1)

# ----------------------------------------
# Move Z Safely
# ----------------------------------------
def move_Z(ser, z, feed=300):
    send(ser, f"G1 Z{z} F{feed}")

# ----------------------------------------
# Move XY but ONLY if Z is safe
# ----------------------------------------
def move_XY_safe(ser, x, y, feed=3000):
    # Query current Z position
    ser.write(b"M114\n")
    time.sleep(0.1)
    response = ser.read_all().decode().strip()

    # Parse Z from M114
    current_Z = SAFE_Z
    for token in response.split(" "):
        if token.startswith("Z:"):
            current_Z = float(token.replace("Z:", ""))
            break

    # Safety lock
    if current_Z < SAFE_Z:
        print("⚠ Z too low! Moving to safe height first.")
        move_Z(ser, SAFE_Z)

    send(ser, f"G1 X{x} Y{y} F{feed}")

# ----------------------------------------
# Perform Dip Sequence (main logic)
# ----------------------------------------
def run_sequence():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)

    print("=== SENDING START GCODE ===")
    for g in START_GCODE:
        send(ser, g)
        time.sleep(0.2)

    print("=== BEGIN SLIDE STAINING SEQUENCE ===")
    for jar in SEQUENCE:
        x, y = JAR_POSITIONS[jar]
        soak_time = JAR_SOAK_TIME[jar]

        print(f"\n--- Moving to JAR {jar} at ({x},{y}) ---")

        # Move XY at safe Z
        move_XY_safe(ser, x, y)

        # Lower into jar
        move_Z(ser, 0, feed=120)   # dip depth (Z=0)

        print(f"Soaking for {soak_time} sec...")
        time.sleep(soak_time)

        # Raise to safe Z
        move_Z(ser, SAFE_Z)

    print("\n=== SEQUENCE COMPLETE ===")

    ser.close()


# -----------------------------
# RUN
# -----------------------------
if __name__ == "__main__":
    run_sequence()
