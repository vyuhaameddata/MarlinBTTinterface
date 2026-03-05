# StallGuard Configuration — Vyuhaa Microscope Stage
**Travel Ranges: X = 12mm | Y = 12mm | Z = 4mm**

---

## Why Small Ranges Change Everything

Standard 3D printer sensorless homing assumes 200–300mm of travel.
Your stage has 12mm XY and 4mm Z — this imposes hard constraints:

| Constraint | Impact |
|---|---|
| StallGuard unreliable below ~20% max speed | Homing speed has a tight window |
| Backoff + bump must fit within range | Max backoff + bump < 40% of travel |
| Acceleration spike can false-trigger | Must use very low acceleration during homing |
| Short travel = less time to stabilize | Motor must be at target speed before hitting end |

---

## Recommended Values

### Motor Currents

```cpp
// X axis — 12mm travel
#define X_CURRENT       600    // mA RMS — enough for reliable SG signal
#define X_CURRENT_HOME  300    // 50% of run: soft stop, no frame grinding

// Y axis — 12mm travel
#define Y_CURRENT       600
#define Y_CURRENT_HOME  300

// Z axis — 4mm travel (focus, lead screw)
#define Z_CURRENT       700    // slightly higher: lead screw friction
#define Z_CURRENT_HOME  400    // 57% — Z needs more torque to move at all
```

> **Why lower homing current?**
> Lower current → lower back-EMF signal → driver triggers stall at lower load.
> This means the stage stops gently against the frame instead of grinding.

---

### Stall Sensitivity

```cpp
// TMC2209 range: 0 (least sensitive) → 255 (most sensitive)
// Higher = triggers more easily

#define X_STALL_SENSITIVITY   70   // 12mm: medium — avoids false triggers on accel
#define Y_STALL_SENSITIVITY   70   // same
#define Z_STALL_SENSITIVITY   55   // 4mm: lower — lead screw has higher baseline load
```

**Tuning ladder (do this per axis):**
```
Start at 120 → G28 X
If triggers before reaching end → lower by 15
If grinds without triggering   → raise by 10
Settle at the point it just triggers cleanly, then subtract 10 as margin
Target: 3 clean homes in a row with no false triggers mid-travel
```

---

### Homing Speeds

Critical: StallGuard needs the motor at a **consistent, fast-enough speed** before hitting the end.
With only 12mm, you need to reach that speed quickly.

```cpp
// In Configuration.h:
// (mm/min = mm/s × 60)

#define HOMING_FEEDRATE_MM_M  { (25*60), (25*60), (8*60) }
//                               X=25mm/s  Y=25mm/s  Z=8mm/s

// Homing acceleration — keep LOW to avoid false stall on ramp-up
// Set in Configuration.h or via M204:
#define HOMING_BOOST_FACTOR   1   // no boost
```

**Why 25 mm/s for XY?**
- At 12mm, starting from 0 the motor needs ~3–4mm to reach speed
- 25 mm/s is well within StallGuard's reliable window for most TMC2209 setups
- Too slow (< 10 mm/s) → StallGuard disabled by TCOOLTHRS, stall never fires
- Too fast (> 50 mm/s) → motor slips before hitting end on a delicate stage

**Why 8 mm/s for Z?**
- 4mm total travel — motor barely has room to accelerate
- Lead screw load is variable; lower speed = more torque headroom
- 8 mm/s gives ~0.5mm ramp, leaving 3.5mm of consistent-speed travel

---

### Backoff and Bump

With 12mm and 4mm ranges, these values must be small:

```cpp
// In Configuration_adv.h:

// Backoff: move away from endstop by this much before declaring home
#define SENSORLESS_BACKOFF_MM  { 2, 2, 0.5 }
//                               X  Y   Z (mm)

// Bump: after backoff, re-approach at slower speed for precise position
#define HOMING_BUMP_MM         { 2, 2, 0.5 }
//                               X  Y   Z (mm)
#define HOMING_BUMP_DIVISOR    { 4, 4,  6  }
// Bump speed = homing speed / divisor
// X/Y bump speed = 25/4 = 6.25 mm/s
// Z bump speed   = 8/6  = 1.33 mm/s
```

**Budget check — does it fit in range?**

| Axis | Range | Backoff | Bump | Total Used | Remaining |
|---|---|---|---|---|---|
| X | 12mm | 2mm | 2mm | 4mm | 8mm ✅ |
| Y | 12mm | 2mm | 2mm | 4mm | 8mm ✅ |
| Z | 4mm  | 0.5mm | 0.5mm | 1mm | 3mm ✅ |

---

### Homing Acceleration

False triggers on short axes almost always come from the acceleration spike at move start.
Set a dedicated low homing acceleration:

```cpp
// In Configuration.h or start G-code:
// M204 H sets homing acceleration

// Suggested: ~20% of normal print acceleration
// If your normal ACCELERATION is 500 mm/s²:
M204 H100   // homing accel = 100 mm/s² (put in start G-code)
```

With 100 mm/s² and 25 mm/s target speed:
- Ramp-up distance = v² / (2a) = 625 / 200 = **3.1mm** — fits in 12mm ✅
- For Z at 8 mm/s: ramp = 64 / 200 = **0.32mm** — fits in 4mm ✅

---

### Chopper Timing

```cpp
// Match to your actual PSU voltage — critical for SG accuracy
#define CHOPPER_TIMING CHOPPER_DEFAULT_24V   // if on 24V
// #define CHOPPER_TIMING CHOPPER_DEFAULT_12V   // if on 12V
```

---

### TMC Debug (enable while tuning, disable in production)

```cpp
#define TMC_DEBUG   // enables M122 — read live sg_result during homing
```

**Use M122 to verify:**
```gcode
M122        ; dump all driver status — check sg_result field
M914        ; show current stall sensitivity values
M914 X70    ; set X sensitivity live without recompile
```

---

## Complete Block (copy into Configuration_adv.h)

```cpp
// ── MOTOR CURRENTS ────────────────────────────────────────────────
#define X_CURRENT        600
#define X_CURRENT_HOME   300
#define X_MICROSTEPS      16
#define X_RSENSE           0.11

#define Y_CURRENT        600
#define Y_CURRENT_HOME   300
#define Y_MICROSTEPS      16
#define Y_RSENSE           0.11

#define Z_CURRENT        700
#define Z_CURRENT_HOME   400
#define Z_MICROSTEPS      16
#define Z_RSENSE           0.11

// ── STEALTHCHOP (quiet during normal movement) ────────────────────
#define STEALTHCHOP_XY
#define STEALTHCHOP_Z
// Marlin auto-switches to SpreadCycle during G28 — no extra config needed

// ── CHOPPER ───────────────────────────────────────────────────────
#define CHOPPER_TIMING CHOPPER_DEFAULT_24V   // change to 12V if on 12V PSU

// ── SENSORLESS HOMING ─────────────────────────────────────────────
#define SENSORLESS_HOMING

#define X_STALL_SENSITIVITY   70
#define Y_STALL_SENSITIVITY   70
#define Z_STALL_SENSITIVITY   55

#define SENSORLESS_BACKOFF_MM { 2, 2, 0.5 }

// ── HOMING BUMP ───────────────────────────────────────────────────
#define HOMING_BUMP_MM      { 2, 2, 0.5 }
#define HOMING_BUMP_DIVISOR { 4, 4, 6   }

// ── DEBUG (disable after tuning) ──────────────────────────────────
#define TMC_DEBUG
```

---

## Start G-code (set homing acceleration)

```gcode
M204 H100        ; low homing acceleration (mm/s²)
G28 Z            ; home Z first (avoid XY collision)
G28 X Y          ; home XY
G92 X0 Y0 Z0     ; set origin
```

---

## Red Flags During Tuning

| Symptom | Likely Cause | Fix |
|---|---|---|
| Stall triggers at move start | Accel too high | Lower M204 H value |
| Stall triggers halfway through | Sensitivity too high | Lower `*_STALL_SENSITIVITY` |
| Never triggers, motor grinds | Sensitivity too low OR speed too low | Raise sensitivity OR raise homing speed |
| Z homes inconsistently | Lead screw backlash + variable load | Lower Z bump speed (higher divisor), raise Z_CURRENT_HOME slightly |
| Works at room temp, fails when warm | Motor resistance changes with heat | Normal — retune sensitivity at operating temp |
