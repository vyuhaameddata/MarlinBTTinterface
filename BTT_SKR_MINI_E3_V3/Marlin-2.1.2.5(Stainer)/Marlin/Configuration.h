
#pragma once


#define CONFIGURATION_H_VERSION 02010205


#define STRING_CONFIG_H_AUTHOR "(none, default config)" // Who made the changes.
#define SHOW_BOOTSCREEN


#ifndef MOTHERBOARD
  #define MOTHERBOARD BOARD_BTT_SKR_MINI_E3_V3_0
#endif


#define SERIAL_PORT -1
#define SERIAL_PORT_2 1


#define BAUDRATE 250000

#define X_DRIVER_TYPE  TMC2209
#define Y_DRIVER_TYPE  TMC2209
#define Z_DRIVER_TYPE  TMC2209
#define E_DRIVER_TYPE  TMC2209


#define E0_DRIVER_TYPE A4988

#ifdef I_DRIVER_TYPE
  #define AXIS4_NAME 'A' // :['A', 'B', 'C', 'U', 'V', 'W']
  #define AXIS4_ROTATES
#endif
#ifdef J_DRIVER_TYPE
  #define AXIS5_NAME 'B' // :['B', 'C', 'U', 'V', 'W']
  #define AXIS5_ROTATES
#endif
#ifdef K_DRIVER_TYPE
  #define AXIS6_NAME 'C' // :['C', 'U', 'V', 'W']
  #define AXIS6_ROTATES
#endif
#ifdef U_DRIVER_TYPE
  #define AXIS7_NAME 'U' // :['U', 'V', 'W']
  //#define AXIS7_ROTATES
#endif
#ifdef V_DRIVER_TYPE
  #define AXIS8_NAME 'V' // :['V', 'W']
  //#define AXIS8_ROTATES
#endif
#ifdef W_DRIVER_TYPE
  #define AXIS9_NAME 'W' // :['W']
  //#define AXIS9_ROTATES
#endif


#define EXTRUDERS 1

#define DEFAULT_NOMINAL_FILAMENT_DIA 1.75
#if ENABLED(SINGLENOZZLE)
  
#endif
#if ENABLED(SWITCHING_EXTRUDER)
  #define SWITCHING_EXTRUDER_SERVO_NR 0
  #define SWITCHING_EXTRUDER_SERVO_ANGLES { 0, 90 } // Angles for E0, E1[, E2, E3]
  #if EXTRUDERS > 3
    #define SWITCHING_EXTRUDER_E23_SERVO_NR 1
  #endif
#endif
#if ENABLED(SWITCHING_NOZZLE)
  #define SWITCHING_NOZZLE_SERVO_NR 0
  //#define SWITCHING_NOZZLE_E1_SERVO_NR 1          // If two servos are used, the index of the second
  #define SWITCHING_NOZZLE_SERVO_ANGLES { 0, 90 }   // A pair of angles for { E0, E1 }.
                                                    // For Dual Servo use two pairs: { { lower, raise }, { lower, raise } }
  #define SWITCHING_NOZZLE_SERVO_DWELL 2500         // Dwell time to wait for servo to make physical move
#endif



#if EITHER(PARKING_EXTRUDER, MAGNETIC_PARKING_EXTRUDER)

  #define PARKING_EXTRUDER_PARKING_X { -78, 184 }     // X positions for parking the extruders
  #define PARKING_EXTRUDER_GRAB_DISTANCE 1            // (mm) Distance to move beyond the parking point to grab the extruder

  #if ENABLED(PARKING_EXTRUDER)

    #define PARKING_EXTRUDER_SOLENOIDS_INVERT           // If enabled, the solenoid is NOT magnetized with applied voltage
    #define PARKING_EXTRUDER_SOLENOIDS_PINS_ACTIVE LOW  // LOW or HIGH pin signal energizes the coil
    #define PARKING_EXTRUDER_SOLENOIDS_DELAY 250        // (ms) Delay for magnetic field. No delay if 0 or not defined.
    //#define MANUAL_SOLENOID_CONTROL                   // Manual control of docking solenoids with M380 S / M381

  #elif ENABLED(MAGNETIC_PARKING_EXTRUDER)

    #define MPE_FAST_SPEED      9000      // (mm/min) Speed for travel before last distance point
    #define MPE_SLOW_SPEED      4500      // (mm/min) Speed for last distance travel to park and couple
    #define MPE_TRAVEL_DISTANCE   10      // (mm) Last distance point
    #define MPE_COMPENSATION       0      // Offset Compensation -1 , 0 , 1 (multiplier) only for coupling

  #endif

#endif
#if ANY(SWITCHING_TOOLHEAD, MAGNETIC_SWITCHING_TOOLHEAD, ELECTROMAGNETIC_SWITCHING_TOOLHEAD)
  #define SWITCHING_TOOLHEAD_Y_POS          235         // (mm) Y position of the toolhead dock
  #define SWITCHING_TOOLHEAD_Y_SECURITY      10         // (mm) Security distance Y axis
  #define SWITCHING_TOOLHEAD_Y_CLEAR         60         // (mm) Minimum distance from dock for unobstructed X axis
  #define SWITCHING_TOOLHEAD_X_POS          { 215, 0 }  // (mm) X positions for parking the extruders
  #if ENABLED(SWITCHING_TOOLHEAD)
    #define SWITCHING_TOOLHEAD_SERVO_NR       2         // Index of the servo connector
    #define SWITCHING_TOOLHEAD_SERVO_ANGLES { 0, 180 }  // (degrees) Angles for Lock, Unlock
  #elif ENABLED(MAGNETIC_SWITCHING_TOOLHEAD)
    #define SWITCHING_TOOLHEAD_Y_RELEASE      5         // (mm) Security distance Y axis
    #define SWITCHING_TOOLHEAD_X_SECURITY   { 90, 150 } // (mm) Security distance X axis (T0,T1)
    //#define PRIME_BEFORE_REMOVE                       // Prime the nozzle before release from the dock
    #if ENABLED(PRIME_BEFORE_REMOVE)
      #define SWITCHING_TOOLHEAD_PRIME_MM           20  // (mm)   Extruder prime length
      #define SWITCHING_TOOLHEAD_RETRACT_MM         10  // (mm)   Retract after priming length
      #define SWITCHING_TOOLHEAD_PRIME_FEEDRATE    300  // (mm/min) Extruder prime feedrate
      #define SWITCHING_TOOLHEAD_RETRACT_FEEDRATE 2400  // (mm/min) Extruder retract feedrate
    #endif
  #elif ENABLED(ELECTROMAGNETIC_SWITCHING_TOOLHEAD)
    #define SWITCHING_TOOLHEAD_Z_HOP          2         // (mm) Z raise for switching
  #endif
#endif
#if ENABLED(MIXING_EXTRUDER)
  #define MIXING_STEPPERS 2        // Number of steppers in your mixing extruder
  #define MIXING_VIRTUAL_TOOLS 16  // Use the Virtual Tool method with M163 and M164
  //#define DIRECT_MIXING_IN_G1    // Allow ABCDHI mix factors in G1 movement commands
  //#define GRADIENT_MIX           // Support for gradient mixing with M166 and LCD
  //#define MIXING_PRESETS         // Assign 8 default V-tool presets for 2 or 3 MIXING_STEPPERS
  #if ENABLED(GRADIENT_MIX)
    //#define GRADIENT_VTOOL       // Add M166 T to use a V-tool index as a Gradient alias
  #endif
#endif
#if ENABLED(PSU_CONTROL)
  #define PSU_ACTIVE_STATE LOW      // Set 'LOW' for ATX, 'HIGH' for X-Box

  #if ENABLED(AUTO_POWER_CONTROL)
    #define AUTO_POWER_FANS           // Turn on PSU for fans
    #define AUTO_POWER_E_FANS         // Turn on PSU for E Fans
    #define AUTO_POWER_CONTROLLERFAN  // Turn on PSU for Controller Fan
    #define AUTO_POWER_CHAMBER_FAN    // Turn on PSU for Chamber Fan
    #define AUTO_POWER_COOLER_FAN     // Turn on PSU for Cooler Fan
    #define POWER_TIMEOUT              30 // (s) Turn off power if the machine is idle for this duration
    //#define POWER_OFF_DELAY          60 // (s) Delay of poweroff after M81 command. Useful to let fans run for extra time.
  #endif
  #if EITHER(AUTO_POWER_CONTROL, POWER_OFF_WAIT_FOR_COOLDOWN)
    
  #endif
#endif

#define TEMP_SENSOR_0 1
#define TEMP_SENSOR_1 0
#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_3 0
#define TEMP_SENSOR_4 0
#define TEMP_SENSOR_5 0
#define TEMP_SENSOR_6 0
#define TEMP_SENSOR_7 0
#define TEMP_SENSOR_BED 1
#define TEMP_SENSOR_PROBE 0
#define TEMP_SENSOR_CHAMBER 0
#define TEMP_SENSOR_COOLER 0
#define TEMP_SENSOR_BOARD 0
#define TEMP_SENSOR_REDUNDANT 0

// Dummy thermistor constant temperature readings, for use with 998 and 999
#define DUMMY_THERMISTOR_998_VALUE  25
#define DUMMY_THERMISTOR_999_VALUE 100

// Resistor values when using MAX31865 sensors (-5) on TEMP_SENSOR_0 / 1
#if TEMP_SENSOR_IS_MAX_TC(0)
  #define MAX31865_SENSOR_OHMS_0      100 // (Ω) Typically 100 or 1000 (PT100 or PT1000)
  #define MAX31865_CALIBRATION_OHMS_0 430 // (Ω) Typically 430 for Adafruit PT100; 4300 for Adafruit PT1000
#endif
#if TEMP_SENSOR_IS_MAX_TC(1)
  #define MAX31865_SENSOR_OHMS_1      100
  #define MAX31865_CALIBRATION_OHMS_1 430
#endif
#if TEMP_SENSOR_IS_MAX_TC(2)
  #define MAX31865_SENSOR_OHMS_2      100
  #define MAX31865_CALIBRATION_OHMS_2 430
#endif

#if HAS_E_TEMP_SENSOR
  #define TEMP_RESIDENCY_TIME         10  // (seconds) Time to wait for hotend to "settle" in M109
  #define TEMP_WINDOW                  1  // (°C) Temperature proximity for the "temperature reached" timer
  #define TEMP_HYSTERESIS              3  // (°C) Temperature proximity considered "close enough" to the target
#endif

#if TEMP_SENSOR_BED
  #define TEMP_BED_RESIDENCY_TIME     10  // (seconds) Time to wait for bed to "settle" in M190
  #define TEMP_BED_WINDOW              1  // (°C) Temperature proximity for the "temperature reached" timer
  #define TEMP_BED_HYSTERESIS          3  // (°C) Temperature proximity considered "close enough" to the target
#endif

#if TEMP_SENSOR_CHAMBER
  #define TEMP_CHAMBER_RESIDENCY_TIME 10  // (seconds) Time to wait for chamber to "settle" in M191
  #define TEMP_CHAMBER_WINDOW          1  // (°C) Temperature proximity for the "temperature reached" timer
  #define TEMP_CHAMBER_HYSTERESIS      3  // (°C) Temperature proximity considered "close enough" to the target
#endif
#if TEMP_SENSOR_REDUNDANT
  #define TEMP_SENSOR_REDUNDANT_SOURCE    E1  // The sensor that will provide the redundant reading.
  #define TEMP_SENSOR_REDUNDANT_TARGET    E0  // The sensor that we are providing a redundant reading for.
  #define TEMP_SENSOR_REDUNDANT_MAX_DIFF  10  // (°C) Temperature difference that will trigger a print abort.
#endif
#define HEATER_0_MINTEMP   5
#define HEATER_1_MINTEMP   5
#define HEATER_2_MINTEMP   5
#define HEATER_3_MINTEMP   5
#define HEATER_4_MINTEMP   5
#define HEATER_5_MINTEMP   5
#define HEATER_6_MINTEMP   5
#define HEATER_7_MINTEMP   5
#define BED_MINTEMP        5
#define CHAMBER_MINTEMP    5

#define HEATER_0_MAXTEMP 275
#define HEATER_1_MAXTEMP 275
#define HEATER_2_MAXTEMP 275
#define HEATER_3_MAXTEMP 275
#define HEATER_4_MAXTEMP 275
#define HEATER_5_MAXTEMP 275
#define HEATER_6_MAXTEMP 275
#define HEATER_7_MAXTEMP 275
#define BED_MAXTEMP      150
#define CHAMBER_MAXTEMP  60
#define HOTEND_OVERSHOOT 15   // (°C) Forbid temperatures over MAXTEMP - OVERSHOOT
#define BED_OVERSHOOT    10   // (°C) Forbid temperatures over MAXTEMP - OVERSHOOT
#define COOLER_OVERSHOOT  2   // (°C) Forbid temperatures closer than OVERSHOOT

#define PIDTEMP           // See the PID Tuning Guide at https://reprap.org/wiki/PID_Tuning
//#define MPCTEMP         // See https://marlinfw.org/docs/features/model_predictive_control.html

#define PID_MAX  255      // Limit hotend current while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current
#define PID_K1     0.95   // Smoothing factor within any PID loop

#if ENABLED(PIDTEMP)
  #if ENABLED(PID_PARAMS_PER_HOTEND)
    // Specify up to one value per hotend here, according to your setup.
    // If there are fewer values, the last one applies to the remaining hotends.
    #define DEFAULT_Kp_LIST {  22.20,  22.20 }
    #define DEFAULT_Ki_LIST {   1.08,   1.08 }
    #define DEFAULT_Kd_LIST { 114.00, 114.00 }
  #else
    #define DEFAULT_Kp  22.20
    #define DEFAULT_Ki   1.08
    #define DEFAULT_Kd 114.00
  #endif
#else
  #define BANG_MAX 255    // Limit hotend current while in bang-bang mode; 255=full current
#endif

#if ENABLED(MPCTEMP)
  #define MPC_MAX 255                                 // (0..255) Current to nozzle while MPC is active.
  #define MPC_HEATER_POWER { 40.0f }                  // (W) Heat cartridge powers.

  #define MPC_INCLUDE_FAN                             // Model the fan speed?

  // Measured physical constants from M306
  #define MPC_BLOCK_HEAT_CAPACITY { 16.7f }           // (J/K) Heat block heat capacities.
  #define MPC_SENSOR_RESPONSIVENESS { 0.22f }         // (K/s per ∆K) Rate of change of sensor temperature from heat block.
  #define MPC_AMBIENT_XFER_COEFF { 0.068f }           // (W/K) Heat transfer coefficients from heat block to room air with fan off.
  #if ENABLED(MPC_INCLUDE_FAN)
    #define MPC_AMBIENT_XFER_COEFF_FAN255 { 0.097f }  // (W/K) Heat transfer coefficients from heat block to room air with fan on full.
  #endif

  // For one fan and multiple hotends MPC needs to know how to apply the fan cooling effect.
  #if ENABLED(MPC_INCLUDE_FAN)
  #endif
  #define FILAMENT_HEAT_CAPACITY_PERMM { 5.6e-3f }    // 0.0056 J/K/mm for 1.75mm PLA (0.0149 J/K/mm for 2.85mm PLA).
                                                      // 0.0036 J/K/mm for 1.75mm PETG (0.0094 J/K/mm for 2.85mm PETG).
                                                      // 0.00515 J/K/mm for 1.75mm ABS (0.0137 J/K/mm for 2.85mm ABS).
                                                      // 0.00522 J/K/mm for 1.75mm Nylon (0.0138 J/K/mm for 2.85mm Nylon).

  // Advanced options
  #define MPC_SMOOTHING_FACTOR 0.5f                   // (0.0...1.0) Noisy temperature sensors may need a lower value for stabilization.
  #define MPC_MIN_AMBIENT_CHANGE 1.0f                 // (K/s) Modeled ambient temperature rate of change, when correcting model inaccuracies.
  #define MPC_STEADYSTATE 0.5f                        // (K/s) Temperature change rate for steady state logic to be enforced.

  #define MPC_TUNING_POS { X_CENTER, Y_CENTER, 1.0f } // (mm) M306 Autotuning position, ideally bed center at first layer height.
  #define MPC_TUNING_END_Z 10.0f                      // (mm) M306 Autotuning final Z position.
#endif

#define MAX_BED_POWER 255 // limits duty cycle to bed; 255=full current

#if ENABLED(PIDTEMPBED)
  #define DEFAULT_bedKp 10.00
  #define DEFAULT_bedKi .023
  #define DEFAULT_bedKd 305.4

  // FIND YOUR OWN: "M303 E-1 C8 S90" to run autotune on the bed at 90 degreesC for 8 cycles.
#else
  //#define BED_LIMIT_SWITCHING   // Keep the bed temperature within BED_HYSTERESIS of the target
#endif
#define MAX_CHAMBER_POWER 255 // limits duty cycle to chamber heater; 255=full current

#if ENABLED(PIDTEMPCHAMBER)
  #define MIN_CHAMBER_POWER 0
  #define DEFAULT_chamberKp  37.04
  #define DEFAULT_chamberKi   1.40
  #define DEFAULT_chamberKd 655.17
  // M309 P37.04 I1.04 D655.17

  // FIND YOUR OWN: "M303 E-2 C8 S50" to run autotune on the chamber at 50 degreesC for 8 cycles.
#endif // PIDTEMPCHAMBER

// @section pid temp

#if ANY(PIDTEMP, PIDTEMPBED, PIDTEMPCHAMBER)
  //#define PID_OPENLOOP          // Puts PID in open loop. M104/M140 sets the output power from 0 to PID_MAX
  //#define SLOW_PWM_HEATERS      // PWM with very low frequency (roughly 0.125Hz=8s) and minimum state time of approximately 1s useful for heaters driven by a relay
  #define PID_FUNCTIONAL_RANGE 10 // If the temperature difference between the target temperature and the actual temperature
  
#endif

#define PREVENT_COLD_EXTRUSION
#define EXTRUDE_MINTEMP 170
#define PREVENT_LENGTHY_EXTRUDE
#define EXTRUDE_MAXLENGTH 200
#define THERMAL_PROTECTION_HOTENDS // Enable thermal protection for all extruders
#define THERMAL_PROTECTION_BED     // Enable thermal protection for the heated bed
#define THERMAL_PROTECTION_CHAMBER // Enable thermal protection for the heated chamber
#define THERMAL_PROTECTION_COOLER  // Enable thermal protection for the laser cooling
#if ENABLED(POLARGRAPH)
  #define POLARGRAPH_MAX_BELT_LEN  1035.0 // (mm) Belt length at full extension. Override with M665 H.
  #define DEFAULT_SEGMENTS_PER_SECOND 5   // Move segmentation based on duration
#endif
#if ENABLED(DELTA)
  #define DEFAULT_SEGMENTS_PER_SECOND 200
  #if ENABLED(DELTA_AUTO_CALIBRATION)
    // Default number of probe points : n*n (1 -> 7)
    #define DELTA_CALIBRATION_DEFAULT_POINTS 4
  #endif

  #if EITHER(DELTA_AUTO_CALIBRATION, DELTA_CALIBRATION_MENU)
    // Step size for paper-test probing
    #define PROBE_MANUALLY_STEP 0.05      // (mm)
  #endif

  // Print surface diameter/2 minus unreachable space (avoid collisions with vertical towers).
  #define DELTA_PRINTABLE_RADIUS 140.0    // (mm)

  // Maximum reachable area
  #define DELTA_MAX_RADIUS       140.0    // (mm)

  // Center-to-center distance of the holes in the diagonal push rods.
  #define DELTA_DIAGONAL_ROD 250.0        // (mm)

  // Distance between bed and nozzle Z home position
  #define DELTA_HEIGHT 250.00             // (mm) Get this value from G33 auto calibrate

  #define DELTA_ENDSTOP_ADJ { 0.0, 0.0, 0.0 } // (mm) Get these values from G33 auto calibrate

  // Horizontal distance bridged by diagonal push rods when effector is centered.
  #define DELTA_RADIUS 124.0              // (mm) Get this value from G33 auto calibrate
  #define DELTA_TOWER_ANGLE_TRIM { 0.0, 0.0, 0.0 } // (mm) Get these values from G33 auto calibrate

#endif
#if EITHER(MORGAN_SCARA, MP_SCARA)
  #define DEFAULT_SEGMENTS_PER_SECOND 200
  #define SCARA_LINKAGE_1 150       // (mm)
  #define SCARA_LINKAGE_2 150       // (mm)
  #define SCARA_OFFSET_X  100       // (mm)
  #define SCARA_OFFSET_Y  -56       // (mm)

  #if ENABLED(MORGAN_SCARA)
    #define SCARA_FEEDRATE_SCALING  // Convert XY feedrate from mm/s to degrees/s on the fly
    #define MIDDLE_DEAD_ZONE_R   0  // (mm)

    #define THETA_HOMING_OFFSET  0  // Calculated from Calibration Guide and M360 / M114. See https://www.morgan3dp.com/morgan-calibration-guide/
    #define PSI_HOMING_OFFSET    0  // Calculated from Calibration Guide and M364 / M114. See https://www.morgan3dp.com/morgan-calibration-guide/

  #elif ENABLED(MP_SCARA)

    #define SCARA_OFFSET_THETA1  12 // degrees
    #define SCARA_OFFSET_THETA2 131 // degrees

  #endif

#endif
#if ENABLED(AXEL_TPARA)
  #define DEBUG_TPARA_KINEMATICS
  #define DEFAULT_SEGMENTS_PER_SECOND 200

  // Length of inner and outer support arms. Measure arm lengths precisely.
  #define TPARA_LINKAGE_1 120     // (mm)
  #define TPARA_LINKAGE_2 120     // (mm)

  #define TPARA_OFFSET_X    0     // (mm)
  #define TPARA_OFFSET_Y    0     // (mm)
  #define TPARA_OFFSET_Z    0     // (mm)

  #define SCARA_FEEDRATE_SCALING  // Convert XY feedrate from mm/s to degrees/s on the fly

  // Radius around the center where the arm cannot reach
  #define MIDDLE_DEAD_ZONE_R   0  // (mm)
#endif
#define USE_XMIN_PLUG
#define USE_YMIN_PLUG
#define USE_ZMIN_PLUG
#define ENDSTOPPULLUPS
#if DISABLED(ENDSTOPPULLUPS)
#endif
#if DISABLED(ENDSTOPPULLDOWNS)

#endif

// Mechanical endstop with COM to ground and NC to Signal uses "false" here (most common setup).
#define X_MIN_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define Y_MIN_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define Z_MIN_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define I_MIN_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define J_MIN_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define K_MIN_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define U_MIN_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define V_MIN_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define W_MIN_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define X_MAX_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define Y_MAX_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define Z_MAX_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define I_MAX_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define J_MAX_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define K_MAX_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define U_MAX_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define V_MAX_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define W_MAX_ENDSTOP_INVERTING false // Set to true to invert the logic of the endstop.
#define Z_MIN_PROBE_ENDSTOP_INVERTING false // Set to true to invert the logic of the probe.
#define DEFAULT_AXIS_STEPS_PER_UNIT   { 80, 80, 400, 500 }
#define DEFAULT_MAX_FEEDRATE          { 2400, 2400, 600, 25 } 
#if ENABLED(LIMITED_MAX_FR_EDITING)
  #define MAX_FEEDRATE_EDIT_VALUES    { 600, 600, 10, 50 } // ...or, set your own edit limits
#endif
#define DEFAULT_MAX_ACCELERATION      { 50, 50, 20, 100 }
#if ENABLED(LIMITED_MAX_ACCEL_EDITING)
  #define MAX_ACCEL_EDIT_VALUES       { 6000, 6000, 200, 20000 } // ...or, set your own edit limits
#endif
#define DEFAULT_ACCELERATION          50
#define DEFAULT_TRAVEL_ACCELERATION   50
#define DEFAULT_RETRACT_ACCELERATION  3000    // E acceleration for retracts
// #define DEFAULT_TRAVEL_ACCELERATION   3000    // X, Y, Z acceleration for travel (non printing) moves
#if ENABLED(CLASSIC_JERK)
  #define DEFAULT_XJERK 1.0
  #define DEFAULT_YJERK 1.0
  #define DEFAULT_ZJERK  0.1
  #if ENABLED(LIMITED_JERK_EDITING)
    #define MAX_JERK_EDIT_VALUES { 20, 20, 0.6, 10 } // ...or, set your own edit limits
  #endif
#endif

#define DEFAULT_EJERK    5.0  // May be used by Linear Advance
#if DISABLED(CLASSIC_JERK)
  #define JUNCTION_DEVIATION_MM 0.013 // (mm) Distance from real junction edge
  #define JD_HANDLE_SMALL_SEGMENTS    // Use curvature estimation instead of just the junction angle
                                      // for small segments (< 1mm) with large junction angles (> 135°).
#endif
#define Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN
#ifdef Z_PROBE_SERVO_NR
  //#define Z_SERVO_ANGLES { 70, 0 }      // Z Servo Deploy and Stow angles
#endif
#if ENABLED(MAGLEV4)
  //#define MAGLEV_TRIGGER_PIN 11     // Set to the connected digital output
  #define MAGLEV_TRIGGER_DELAY 15     // Changing this risks overheating the coil
#endif
#if ENABLED(TOUCH_MI_PROBE)
  #define TOUCH_MI_RETRACT_Z 0.5                  // Height at which the probe retracts
  //#define TOUCH_MI_DEPLOY_XPOS (X_MAX_BED + 2)  // For a magnet on the right side of the bed
  //#define TOUCH_MI_MANUAL_DEPLOY                // For manual deploy (LCD menu)
#endif
#if ENABLED(RACK_AND_PINION_PROBE)
  #define Z_PROBE_DEPLOY_X  X_MIN_POS
  #define Z_PROBE_RETRACT_X X_MAX_POS
#endif
#if ENABLED(MAG_MOUNTED_PROBE)
  #define PROBE_DEPLOY_FEEDRATE (133*60)  // (mm/min) Probe deploy speed
  #define PROBE_STOW_FEEDRATE   (133*60)  // (mm/min) Probe stow speed

  #define MAG_MOUNTED_DEPLOY_1 { PROBE_DEPLOY_FEEDRATE, { 245, 114, 30 } }  // Move to side Dock & Attach probe
  #define MAG_MOUNTED_DEPLOY_2 { PROBE_DEPLOY_FEEDRATE, { 210, 114, 30 } }  // Move probe off dock
  #define MAG_MOUNTED_DEPLOY_3 { PROBE_DEPLOY_FEEDRATE, {   0,   0,  0 } }  // Extra move if needed
  #define MAG_MOUNTED_DEPLOY_4 { PROBE_DEPLOY_FEEDRATE, {   0,   0,  0 } }  // Extra move if needed
  #define MAG_MOUNTED_DEPLOY_5 { PROBE_DEPLOY_FEEDRATE, {   0,   0,  0 } }  // Extra move if needed
  #define MAG_MOUNTED_STOW_1   { PROBE_STOW_FEEDRATE,   { 245, 114, 20 } }  // Move to dock
  #define MAG_MOUNTED_STOW_2   { PROBE_STOW_FEEDRATE,   { 245, 114,  0 } }  // Place probe beside remover
  #define MAG_MOUNTED_STOW_3   { PROBE_STOW_FEEDRATE,   { 230, 114,  0 } }  // Side move to remove probe
  #define MAG_MOUNTED_STOW_4   { PROBE_STOW_FEEDRATE,   { 210, 114, 20 } }  // Side move to remove probe
  #define MAG_MOUNTED_STOW_5   { PROBE_STOW_FEEDRATE,   {   0,   0,  0 } }  // Extra move if needed
#endif
#if ENABLED(DUET_SMART_EFFECTOR)
  #define SMART_EFFECTOR_MOD_PIN  -1  // Connect a GPIO pin to the Smart Effector MOD pin
#endif
#if ENABLED(Z_PROBE_ALLEN_KEY)
  // 2 or 3 sets of coordinates for deploying and retracting the spring loaded touch probe on G29,
  // if servo actuated touch probe is not defined. Uncomment as appropriate for your printer/probe.

  #define Z_PROBE_ALLEN_KEY_DEPLOY_1 { 30.0, DELTA_PRINTABLE_RADIUS, 100.0 }
  #define Z_PROBE_ALLEN_KEY_DEPLOY_1_FEEDRATE XY_PROBE_FEEDRATE

  #define Z_PROBE_ALLEN_KEY_DEPLOY_2 { 0.0, DELTA_PRINTABLE_RADIUS, 100.0 }
  #define Z_PROBE_ALLEN_KEY_DEPLOY_2_FEEDRATE (XY_PROBE_FEEDRATE)/10

  #define Z_PROBE_ALLEN_KEY_DEPLOY_3 { 0.0, (DELTA_PRINTABLE_RADIUS) * 0.75, 100.0 }
  #define Z_PROBE_ALLEN_KEY_DEPLOY_3_FEEDRATE XY_PROBE_FEEDRATE

  #define Z_PROBE_ALLEN_KEY_STOW_1 { -64.0, 56.0, 23.0 } // Move the probe into position
  #define Z_PROBE_ALLEN_KEY_STOW_1_FEEDRATE XY_PROBE_FEEDRATE

  #define Z_PROBE_ALLEN_KEY_STOW_2 { -64.0, 56.0, 3.0 } // Push it down
  #define Z_PROBE_ALLEN_KEY_STOW_2_FEEDRATE (XY_PROBE_FEEDRATE)/10

  #define Z_PROBE_ALLEN_KEY_STOW_3 { -64.0, 56.0, 50.0 } // Move it up to clear
  #define Z_PROBE_ALLEN_KEY_STOW_3_FEEDRATE XY_PROBE_FEEDRATE

  #define Z_PROBE_ALLEN_KEY_STOW_4 { 0.0, 0.0, 50.0 }
  #define Z_PROBE_ALLEN_KEY_STOW_4_FEEDRATE XY_PROBE_FEEDRATE

#endif // Z_PROBE_ALLEN_KEY
#define NOZZLE_TO_PROBE_OFFSET { 10, 10, 0 }
#define PROBING_MARGIN 10
#define XY_PROBE_FEEDRATE (133*60)
#define Z_PROBE_FEEDRATE_FAST (4*60)
#define Z_PROBE_FEEDRATE_SLOW (Z_PROBE_FEEDRATE_FAST / 2)
#if ENABLED(PROBE_ACTIVATION_SWITCH)
  #define PROBE_ACTIVATION_SWITCH_STATE LOW // State indicating probe is active
  //#define PROBE_ACTIVATION_SWITCH_PIN PC6 // Override default pin
#endif
#if ENABLED(PROBE_TARE)
  #define PROBE_TARE_TIME  200    // (ms) Time to hold tare pin
  #define PROBE_TARE_DELAY 200    // (ms) Delay after tare before
  #define PROBE_TARE_STATE HIGH   // State to write pin for tare
  //#define PROBE_TARE_PIN PA5    // Override default pin
  #if ENABLED(PROBE_ACTIVATION_SWITCH)
    //#define PROBE_TARE_ONLY_WHILE_INACTIVE  // Fail to tare/probe if PROBE_ACTIVATION_SWITCH is active
  #endif
#endif
#if ENABLED(PROBE_ENABLE_DISABLE)
  //#define PROBE_ENABLE_PIN -1   // Override the default pin here
#endif
#define Z_CLEARANCE_DEPLOY_PROBE   10 // (mm) Z Clearance for Deploy/Stow
#define Z_CLEARANCE_BETWEEN_PROBES  5 // (mm) Z Clearance between probe points
#define Z_CLEARANCE_MULTI_PROBE     5 // (mm) Z Clearance between multiple probes
#define Z_PROBE_LOW_POINT          -2 // (mm) Farthest distance below the trigger-point to go before stopping
#define Z_PROBE_OFFSET_RANGE_MIN -20
#define Z_PROBE_OFFSET_RANGE_MAX 20
#if ENABLED(PAUSE_BEFORE_DEPLOY_STOW)
#endif
#if ENABLED(PROBING_HEATERS_OFF)
#endif
#if ENABLED(PREHEAT_BEFORE_PROBING)
  #define PROBING_NOZZLE_TEMP 120   // (°C) Only applies to E0 at this time
  #define PROBING_BED_TEMP     50
#endif
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0



#define DISABLE_OTHER_EXTRUDERS

#define INVERT_X_DIR false     // test and flip if needed
#define INVERT_Y_DIR false     // test and flip if needed
#define INVERT_Z_DIR false     // test and flip if needed

#define X_HOME_DIR -1          // if no endstop: may ignore, use G92
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1

// ==== MICROSCOPE MOTION RANGE ====

#define X_MIN_POS -1080
#define X_MAX_POS  1080

#define Y_MIN_POS -1080 
#define Y_MAX_POS  1080

#define Z_MIN_POS  0
#define Z_MAX_POS  3

// #define X_MAX_POS X_BED_SIZE
// #define Y_MAX_POS Y_BED_SIZE

#define MIN_SOFTWARE_ENDSTOPS
#if ENABLED(MIN_SOFTWARE_ENDSTOPS)
  #define MIN_SOFTWARE_ENDSTOP_X
  #define MIN_SOFTWARE_ENDSTOP_Y
  #define MIN_SOFTWARE_ENDSTOP_Z
  #define MIN_SOFTWARE_ENDSTOP_I
  #define MIN_SOFTWARE_ENDSTOP_J
  #define MIN_SOFTWARE_ENDSTOP_K
  #define MIN_SOFTWARE_ENDSTOP_U
  #define MIN_SOFTWARE_ENDSTOP_V
  #define MIN_SOFTWARE_ENDSTOP_W
#endif
#define MAX_SOFTWARE_ENDSTOPS
#if ENABLED(MAX_SOFTWARE_ENDSTOPS)
  #define MAX_SOFTWARE_ENDSTOP_X
  #define MAX_SOFTWARE_ENDSTOP_Y
  #define MAX_SOFTWARE_ENDSTOP_Z
  #define MAX_SOFTWARE_ENDSTOP_I
  #define MAX_SOFTWARE_ENDSTOP_J
  #define MAX_SOFTWARE_ENDSTOP_K
  #define MAX_SOFTWARE_ENDSTOP_U
  #define MAX_SOFTWARE_ENDSTOP_V
  #define MAX_SOFTWARE_ENDSTOP_W
#endif

#if EITHER(MIN_SOFTWARE_ENDSTOPS, MAX_SOFTWARE_ENDSTOPS)
  //#define SOFT_ENDSTOPS_MENU_ITEM  // Enable/Disable software endstops from the LCD
#endif
#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  #define FIL_RUNOUT_ENABLED_DEFAULT true // Enable the sensor on startup. Override with M412 followed by M500.
  #define NUM_RUNOUT_SENSORS   1          // Number of sensors, up to one per extruder. Define a FIL_RUNOUT#_PIN for each.

  #define FIL_RUNOUT_STATE     LOW        // Pin state indicating that filament is NOT present.
  #define FIL_RUNOUT_PULLUP               // Use internal pullup for filament runout pins.
    #define FILAMENT_RUNOUT_SCRIPT "M600"
  #ifdef FILAMENT_RUNOUT_DISTANCE_MM
  #endif
#endif
#if ENABLED(PREHEAT_BEFORE_LEVELING)
  #define LEVELING_NOZZLE_TEMP 120   // (°C) Only applies to E0 at this time
  #define LEVELING_BED_TEMP     50
#endif
#if ANY(MESH_BED_LEVELING, AUTO_BED_LEVELING_UBL, PROBE_MANUALLY)
  // Set a height for the start of manual adjustment
  #define MANUAL_PROBE_START_Z 0.2  // (mm) Comment out to use the last-measured height
#endif

#if ANY(MESH_BED_LEVELING, AUTO_BED_LEVELING_BILINEAR, AUTO_BED_LEVELING_UBL)
  #define ENABLE_LEVELING_FADE_HEIGHT
  #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    #define DEFAULT_LEVELING_FADE_HEIGHT 10.0 // (mm) Default fade height.
  #endif
  #define SEGMENT_LEVELED_MOVES
  #define LEVELED_SEGMENT_LENGTH 5.0 // (mm) Length of all segments (except the last one)
  #if ENABLED(G26_MESH_VALIDATION)
    #define MESH_TEST_NOZZLE_SIZE    0.4  // (mm) Diameter of primary nozzle.
    #define MESH_TEST_LAYER_HEIGHT   0.2  // (mm) Default layer height for G26.
    #define MESH_TEST_HOTEND_TEMP  205    // (°C) Default nozzle temperature for G26.
    #define MESH_TEST_BED_TEMP      60    // (°C) Default bed temperature for G26.
    #define G26_XY_FEEDRATE         20    // (mm/s) Feedrate for G26 XY moves.
    #define G26_XY_FEEDRATE_TRAVEL 100    // (mm/s) Feedrate for G26 XY travel moves.
    #define G26_RETRACT_MULTIPLIER   1.0  // G26 Q (retraction) used by default between mesh test elements.
  #endif

#endif

#if EITHER(AUTO_BED_LEVELING_LINEAR, AUTO_BED_LEVELING_BILINEAR)
  #define GRID_MAX_POINTS_X 3
  #define GRID_MAX_POINTS_Y GRID_MAX_POINTS_X
  #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
    #if ENABLED(ABL_BILINEAR_SUBDIVISION)
      // Number of subdivisions between probe points
      #define BILINEAR_SUBDIVISIONS 3
    #endif

  #endif

#elif ENABLED(AUTO_BED_LEVELING_UBL)
  #define MESH_INSET 1              // Set Mesh bounds as an inset region of the bed
  #define GRID_MAX_POINTS_X 10      // Don't use more than 15 points per axis, implementation limited.
  #define GRID_MAX_POINTS_Y GRID_MAX_POINTS_X
  #define UBL_MESH_EDIT_MOVES_Z     // Sophisticated users prefer no movement of nozzle
  #define UBL_SAVE_ACTIVE_ON_M500   // Save the currently active mesh in the current slot on M500
#elif ENABLED(MESH_BED_LEVELING)
  #define MESH_INSET 10          // Set Mesh bounds as an inset region of the bed
  #define GRID_MAX_POINTS_X 3    // Don't use more than 7 points per axis, implementation limited.
  #define GRID_MAX_POINTS_Y GRID_MAX_POINTS_X
#endif // BED_LEVELING
#if ENABLED(LCD_BED_LEVELING)
  #define MESH_EDIT_Z_STEP  0.025 // (mm) Step size while manually probing Z axis.
  #define LCD_PROBE_Z_RANGE 4     // (mm) Z Range centered on Z_MIN_POS for LCD Z adjustment
  //#define MESH_EDIT_MENU        // Add a menu to edit mesh points
#endif
#if ENABLED(LCD_BED_TRAMMING)
  #define BED_TRAMMING_INSET_LFRB { 30, 30, 30, 30 } // (mm) Left, Front, Right, Back insets
  #define BED_TRAMMING_HEIGHT      0.0        // (mm) Z height of nozzle at tramming points
  #define BED_TRAMMING_Z_HOP       4.0        // (mm) Z raise between tramming points
  //#define BED_TRAMMING_INCLUDE_CENTER       // Move to the center after the last corner
  //#define BED_TRAMMING_USE_PROBE
  #if ENABLED(BED_TRAMMING_USE_PROBE)
    #define BED_TRAMMING_PROBE_TOLERANCE 0.1  // (mm)
    #define BED_TRAMMING_VERIFY_RAISED        // After adjustment triggers the probe, re-probe to verify
    //#define BED_TRAMMING_AUDIO_FEEDBACK
  #endif
  #define BED_TRAMMING_LEVELING_ORDER { LF, RF, RB, LB }
#endif
#if ENABLED(Z_SAFE_HOMING)
  #define Z_SAFE_HOMING_X_POINT X_CENTER  // (mm) X point for Z homing
  #define Z_SAFE_HOMING_Y_POINT Y_CENTER  // (mm) Y point for Z homing
#endif

#define HOMING_FEEDRATE_MM_M { (20*60), (20*60), (2*60) }
#define VALIDATE_HOMING_ENDSTOPS
#if ENABLED(SKEW_CORRECTION)
  // Input all length measurements here:
  #define XY_DIAG_AC 282.8427124746
  #define XY_DIAG_BD 282.8427124746
  #define XY_SIDE_AD 200
  #if ENABLED(SKEW_CORRECTION_FOR_Z)
    #define XZ_DIAG_AC 282.8427124746
    #define XZ_DIAG_BD 282.8427124746
    #define YZ_DIAG_AC 282.8427124746
    #define YZ_DIAG_BD 282.8427124746
    #define YZ_SIDE_AD 200
  #endif
#endif
#define EEPROM_CHITCHAT       // Give feedback on EEPROM commands. Disable to save flash.
#define EEPROM_BOOT_SILENT    // Keep M503 quiet and only give errors during first load
#if ENABLED(EEPROM_SETTINGS)
#endif
#define HOST_KEEPALIVE_FEATURE        // Disable this if your host doesn't like keepalive messages
#define DEFAULT_KEEPALIVE_INTERVAL 2  // Number of seconds between "busy" messages. Set with M113.
#define BUSY_WHILE_HEATING            // Some hosts require "busy" messages even during heating
#define PREHEAT_1_LABEL       "PLA"
#define PREHEAT_1_TEMP_HOTEND 180
#define PREHEAT_1_TEMP_BED     70
#define PREHEAT_1_TEMP_CHAMBER 35
#define PREHEAT_1_FAN_SPEED     0 // Value from 0 to 255

#define PREHEAT_2_LABEL       "ABS"
#define PREHEAT_2_TEMP_HOTEND 240
#define PREHEAT_2_TEMP_BED    110
#define PREHEAT_2_TEMP_CHAMBER 35
#define PREHEAT_2_FAN_SPEED     0 // Value from 0 to 255
#if ENABLED(NOZZLE_PARK_FEATURE)
  // Specify a park position as { X, Y, Z_raise }
  #define NOZZLE_PARK_POINT { (X_MIN_POS + 10), (Y_MAX_POS - 10), 20 }
  #define NOZZLE_PARK_MOVE          0   // Park motion: 0 = XY Move, 1 = X Only, 2 = Y Only, 3 = X before Y, 4 = Y before X
  #define NOZZLE_PARK_Z_RAISE_MIN   2   // (mm) Always raise Z by at least this distance
  #define NOZZLE_PARK_XY_FEEDRATE 100   // (mm/s) X and Y axes feedrate (also used for delta Z axis)
  #define NOZZLE_PARK_Z_FEEDRATE    5   // (mm/s) Z axis feedrate (not used for delta printers)
#endif
#if ENABLED(NOZZLE_CLEAN_FEATURE)
  #define NOZZLE_CLEAN_STROKES  12
  #define NOZZLE_CLEAN_TRIANGLES  3
  #define NOZZLE_CLEAN_START_POINT { {  30, 30, (Z_MIN_POS + 1) } }
  #define NOZZLE_CLEAN_END_POINT   { { 100, 60, (Z_MIN_POS + 1) } }
  #define NOZZLE_CLEAN_CIRCLE_RADIUS 6.5
  #define NOZZLE_CLEAN_CIRCLE_FN 10
  #define NOZZLE_CLEAN_CIRCLE_MIDDLE NOZZLE_CLEAN_START_POINT
  #define NOZZLE_CLEAN_GOBACK
  #define NOZZLE_CLEAN_MIN_TEMP 170
#endif
#define PRINTJOB_TIMER_AUTOSTART
#if ENABLED(PRINTCOUNTER)
  #define PRINTCOUNTER_SAVE_INTERVAL 60 // (minutes) EEPROM save interval during print. A value of 0 will save stats at end of print.
#endif
#if ENABLED(PASSWORD_FEATURE)
  #define PASSWORD_LENGTH 4                 // (#) Number of digits (1-9). 3 or 4 is recommended
  #define PASSWORD_ON_STARTUP
  #define PASSWORD_UNLOCK_GCODE             // Unlock with the M511 P<password> command. Disable to prevent brute-force attack.
  #define PASSWORD_CHANGE_GCODE             // Change the password with M512 P<old> S<new>.
#endif
#define LCD_LANGUAGE en
#define DISPLAY_CHARSET_HD44780 JAPANESE

#define LCD_INFO_SCREEN_STYLE 0
#if ENABLED(ENCODER_NOISE_FILTER)
  #define ENCODER_SAMPLES 10
#endif
#if ENABLED(SAV_3DGLCD)
  #define U8GLIB_SSD1306
#endif
#if ENABLED(DGUS_LCD_UI_MKS)
  #define USE_MKS_GREEN_UI
#endif
#if EITHER(ANYCUBIC_LCD_I3MEGA, ANYCUBIC_LCD_CHIRON)
#endif
#if ENABLED(EXTENSIBLE_UI)
  //#define EXTUI_LOCAL_BEEPER // Enables use of local Beeper pin with external display
#endif
#if ENABLED(TFT_GENERIC)
  #define TFT_DRIVER AUTO
#endif
#if ENABLED(TFT_COLOR_UI)
#endif

#if ENABLED(TFT_LVGL_UI)
#endif
#if ENABLED(TOUCH_SCREEN)
  #define BUTTON_DELAY_EDIT      50 // (ms) Button repeat delay for edit screens
  #define BUTTON_DELAY_MENU     250 // (ms) Button repeat delay for menus
  #define TOUCH_SCREEN_CALIBRATION
  #if BOTH(TOUCH_SCREEN_CALIBRATION, EEPROM_SETTINGS)
    #define TOUCH_CALIBRATION_AUTO_SAVE // Auto save successful calibration values to EEPROM
  #endif

  #if ENABLED(TFT_COLOR_UI)
    
  #endif
#endif
#define SOFT_PWM_SCALE 0
#if EITHER(RGB_LED, RGBW_LED)

#endif

#if ANY(RGB_LED, RGBW_LED, PCA9632)
  #if ENABLED(RGB_STARTUP_TEST)
    #define RGB_STARTUP_TEST_INNER_MS 10  // (ms) Reduce or increase fading speed
  #endif
#endif

#if ENABLED(NEOPIXEL_LED)
  #define NEOPIXEL_TYPE          NEO_GRBW 
  #define NEOPIXEL_PIXELS              30 // Number of LEDs in the strip. (Longest strip when NEOPIXEL2_SEPARATE is disabled.)
  #define NEOPIXEL_IS_SEQUENTIAL          // Sequential display for temperature change - LED by LED. Disable to change all LEDs at once.
  #define NEOPIXEL_BRIGHTNESS         127 // Initial brightness (0-255)
  #if ENABLED(NEOPIXEL2_SEPARATE)
    #define NEOPIXEL2_PIXELS           15 // Number of LEDs in the second strip
    #define NEOPIXEL2_BRIGHTNESS      127 // Initial brightness (0-255)
    #define NEOPIXEL2_STARTUP_TEST        // Cycle through colors at startup
    #define NEOPIXEL_M150_DEFAULT      -1 // Default strip for M150 without 'S'. Use -1 to set all by default.
  #else
  #endif
#endif
#if ANY(BLINKM, RGB_LED, RGBW_LED, PCA9632, PCA9533, NEOPIXEL_LED)
  #define PRINTER_EVENT_LEDS
#endif

#define SERVO_DELAY { 300 }
