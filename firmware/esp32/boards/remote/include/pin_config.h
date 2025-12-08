#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// ========================================
// ESP32-D Remote Control (RC) Pin Configuration
// ========================================

// === UART Communication (Pi Zero 2W ↔ ESP32) ===
#define PI_RX_PIN       16  // ESP32 RX2 ← Pi GPIO14 (TXD0)
#define PI_TX_PIN       17  // ESP32 TX2 → Pi GPIO15 (RXD0)
#define PI_UART_NUM     2   // UART2
#define PI_BAUD_RATE    115200

// === I2C Bus (Optional Display/Sensors) ===
#define I2C_SDA_PIN     21
#define I2C_SCL_PIN     22
#define I2C_FREQ        400000  // 400kHz fast mode

// === User Input Controls ===
#define BTN_FORWARD     18  // D-pad up / forward
#define BTN_BACKWARD    19  // D-pad down / backward
#define BTN_LEFT        20  // D-pad left
#define BTN_RIGHT       23  // D-pad right

#define BTN_ACTION_A    25  // Action button A (turbo/boost)
#define BTN_ACTION_B    26  // Action button B (mode switch)
#define BTN_EMERGENCY   4   // Emergency stop (INPUT_PULLUP, active LOW)

// === Feedback Output ===
#define STATUS_LED_PIN      2   // Onboard LED
#define BUZZER_PIN          32  // Piezo buzzer for alerts
#define VIBRATION_PIN       33  // Vibration motor (optional haptic)

// === Analog Controls (Optional) ===
#define ANALOG_THROTTLE     34  // ADC input for throttle slider (input-only)
#define ANALOG_STEERING     35  // ADC input for steering knob (input-only)

// === Reserved for Future Use ===
// GPIO12-15: Available for expansion
// GPIO5: Additional sensor/button
// GPIO27: Additional output

// ========================================
// Control Configuration
// ========================================
#define BUTTON_DEBOUNCE_MS      50    // Debounce delay
#define ANALOG_DEADZONE         50    // ADC deadzone (0-4095 scale)
#define VIBRATION_DURATION_MS   200   // Haptic feedback duration

// Alert thresholds (from robot status)
#define ALERT_BATTERY_LOW       11.1  // Trigger buzzer at low voltage
#define ALERT_SIGNAL_LOST_MS    2000  // No heartbeat timeout

#endif // PIN_CONFIG_H
