/*
pin defs, constants, baud rates, PID gains
*/

#pragma once

#include "Wire.h"

// Inlines make it so that these constexpr are only created once and available in all translation units - generally not a problem, 
// but if these values are sent as references, could lead to weird differing state issues

// GPS
inline constexpr HardwareSerial &GNSS_SERISAL = Serial7; // Serial7: TX7=29, RX7=28
inline constexpr int BOOT_SEQ_DELAY = 5000; // in ms

// pins
// Servos - fins (through raceway to lower stage)
inline constexpr int SERVO1_PIN = 2;
inline constexpr int SERVO2_PIN = 3;
inline constexpr int SERVO3_PIN = 4;
inline constexpr int SERVO4_PIN = 5;
inline constexpr int PYRO_PIN = 6; // gate drive to MOFET, not direct pyro power — TBD later

// Radio
inline constexpr HardwareSerial &RADIO_SERIAL = Serial5; // Serial5: TX5=20, RX5=21
inline constexpr int RADIO_M0_PIN = 22;
inline constexpr int RADIO_M1_PIN = 23;


// I2C buses — avionics bay, one sensor per bus (no contention)
inline constexpr TwoWire &IMU_WIRE = Wire;   // SDA=18, SCL=19 -> LSM6DSO IMU
inline constexpr TwoWire &MAG_WIRE = Wire1;  // SDA=17, SCL=16 -> MLX90393 magnetometer
inline constexpr TwoWire &ALTI_WIRE = Wire2; // SDA=25, SCL=24 -> BMP280 altimeter

// constexpr int STATUS_LED_PIN = 13;  // Basic Debug

// tunable gains
// MENTOR's gains, WILL CHANGE IN FUTURE
inline constexpr double MAX_FIN_DEFLECTION_DEG = 14.0;
inline constexpr double ROLL_KP = 0.5, ROLL_KI = 0.0, ROLL_KD = 0.02;
inline constexpr double PITCH_KP = 2.0, PITCH_KI = 0.0, PITCH_KD = 0.2;
inline constexpr double YAW_KP = 2.0, YAW_KI = 0.0, YAW_KD = 0.2;

// baud rates
inline constexpr long GNSS_BAUD = 38400;
inline constexpr long RADIO_BAUD = 9600; // check your E32 model's default, adjust if different
inline constexpr long DEBUG_BAUD = 115200;

/*
CODE SCRAPS
// I2C buses — avionics bay, one sensor per bus (no contention)
// Wire  (SDA=18, SCL=19) -> LSM6DSO IMU
// Wire1 (SDA=17, SCL=16) -> MLX90393 magnetometer
// Wire2 (SDA=25, SCL=24) -> BMP280 altimeter   // TODO: verify SDA/SCL order against PJRC pinout card before soldering
*/