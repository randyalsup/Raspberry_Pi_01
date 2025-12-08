#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// ========================================
// ESP32-D Robot (RO) Pin Configuration
// ========================================

// === UART Communication (Pi 5 ↔ ESP32) ===
#define PI_RX_PIN       16  // ESP32 RX2 ← Pi GPIO14 (TXD0)
#define PI_TX_PIN       17  // ESP32 TX2 → Pi GPIO15 (RXD0)
#define PI_UART_NUM     2   // UART2
#define PI_BAUD_RATE    115200

// === I2C Bus (Sensors & ADC) ===
#define I2C_SDA_PIN     21
#define I2C_SCL_PIN     22
#define I2C_FREQ        400000  // 400kHz fast mode

// ADS1115 I2C Address: 0x48 (ADDR to GND)
// Channel assignments:
// A0: Battery voltage (12V → 3V via 4:1 divider)
// A1: Motor current (via current sensor → 4:1 divider)
// A2: Reserved (5V rail monitor)
// A3: Reserved (3.3V rail monitor)

// Voltage/Current scaling (4:1 divider)
#define VOLTAGE_DIVIDER_RATIO   4.0
#define CURRENT_DIVIDER_RATIO   4.0
#define ADS1115_VOLTAGE_SCALE   (4.096 / 32768.0)  // 16-bit, ±4.096V range

// === Motor Control (BTS7960 H-Bridge x2) ===
#define MOTOR_LEFT_RPWM   25  // Left motor reverse PWM
#define MOTOR_LEFT_LPWM   26  // Left motor forward PWM
#define MOTOR_RIGHT_RPWM  27  // Right motor reverse PWM
#define MOTOR_RIGHT_LPWM  32  // Right motor forward PWM
#define MOTOR_ENABLE      33  // Enable both motor drivers (active HIGH)

#define MOTOR_PWM_FREQ    1000   // 1kHz PWM frequency
#define MOTOR_PWM_RES     8      // 8-bit resolution (0-255)

// === Ultrasonic Sensors (HC-SR04) ===
#define ULTRASONIC_FRONT_TRIG   18
#define ULTRASONIC_FRONT_ECHO   19
#define ULTRASONIC_REAR_TRIG    23
#define ULTRASONIC_REAR_ECHO    5

#define ULTRASONIC_TIMEOUT_US   30000  // 30ms = ~5m max range

// === Status & Safety ===
#define STATUS_LED_PIN      2   // Onboard LED
#define EMERGENCY_STOP_PIN  4   // E-stop button (INPUT_PULLUP, active LOW)

// === Reserved for Future Use ===
// GPIO12: Wheel encoder left
// GPIO13: Wheel encoder right
// GPIO14: SPI SCLK (if needed)
// GPIO15: SPI MISO (if needed)
// GPIO34: Reserved analog input
// GPIO35: Reserved analog input

// ========================================
// Safety & Operational Limits
// ========================================
#define BATTERY_MIN_VOLTAGE     10.5   // Cutoff voltage (for 3S LiPo)
#define BATTERY_WARN_VOLTAGE    11.1   // Warning voltage
#define BATTERY_MAX_VOLTAGE     12.6   // Fully charged

#define OBSTACLE_STOP_DISTANCE_CM   15  // Hard stop distance
#define OBSTACLE_WARN_DISTANCE_CM   30  // Slow down distance

#define MAX_MOTOR_SPEED         255  // PWM max
#define DEFAULT_MOTOR_SPEED     180  // 70% speed

#endif // PIN_CONFIG_H
