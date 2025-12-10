#!/usr/bin/env python3
"""
Raspberry Pi 5 Robot Controller
Main control loop for robot with ADS1115 sensors, BTS7960 motors, and ESP32 gateway
"""

import serial
import struct
import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import RPi.GPIO as GPIO
from datetime import datetime

# Try to import I2C LCD library
try:
    from RPLCD.i2c import CharLCD
    LCD_AVAILABLE = True
except ImportError:
    LCD_AVAILABLE = False
    print("Warning: RPLCD not installed. Install with: sudo pip3 install RPLCD")

# ========================================
# Pin Configuration (BCM numbering)
# ========================================
# UART to ESP32
UART_PORT = '/dev/serial0'
UART_BAUD = 115200

# BTS7960 Motor Driver Pins
MOTOR_LEFT_LPWM = 13   # Forward PWM (GPIO13, PWM1)
MOTOR_LEFT_RPWM = 12   # Reverse PWM (GPIO12, PWM0)
MOTOR_RIGHT_LPWM = 19  # Forward PWM (GPIO19, PWM1)
MOTOR_RIGHT_RPWM = 18  # Reverse PWM (GPIO18, PWM0)
MOTOR_ENABLE = 26      # Enable both drivers

# PWM Configuration
PWM_FREQ = 1000  # 1kHz
PWM_RANGE = 1000 # 0-1000 scale

# ========================================
# Protocol Constants
# ========================================
SYNC_BYTE = 0xAA
END_BYTE = 0x55

# Message types (from protocol spec)
CMD_MOTOR = 0x01
CMD_CONFIG = 0x02
CMD_QUERY = 0x03
CMD_UI = 0x04
CMD_MODE = 0x05

STA_TELEMETRY = 0x10
STA_WIRELESS = 0x11
STA_UWB = 0x12
STA_ERROR = 0x13
STA_ACK = 0x14

MSG_HEARTBEAT = 0x20
MSG_DEBUG = 0x21

# ========================================
# CRC-8 Calculation
# ========================================
def crc8(data):
    """Calculate CRC-8 checksum"""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc <<= 1
    return crc & 0xFF

# ========================================
# UART Protocol Handler
# ========================================
class ESP32Gateway:
    def __init__(self, port=UART_PORT, baudrate=UART_BAUD):
        self.ser = serial.Serial(port, baudrate, timeout=0.01)
        self.callbacks = {}
        
    def send_message(self, msg_type, payload):
        """Send framed message to ESP32"""
        length = len(payload)
        crc_data = bytes([msg_type, length]) + payload
        crc = crc8(crc_data)
        
        frame = bytes([SYNC_BYTE, msg_type, length]) + payload + bytes([crc, END_BYTE])
        self.ser.write(frame)
        
    def register_callback(self, msg_type, callback):
        """Register handler for incoming message type"""
        self.callbacks[msg_type] = callback
        
    def process(self):
        """Process incoming messages from ESP32"""
        # State machine for parsing (simplified)
        while self.ser.in_waiting > 0:
            # TODO: Implement full state machine from protocol spec
            # For now, just read and discard to prevent buffer overflow
            self.ser.read(self.ser.in_waiting)

# ========================================
# ADS1115 Sensor Interface
# ========================================
class BatterySensor:
    def __init__(self):
        self.available = False
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.ads = ADS.ADS1115(i2c, address=0x48)
            self.ads.gain = 1  # ±4.096V range
            self.chan0 = AnalogIn(self.ads, ADS.P0)  # 5V/4
            self.chan1 = AnalogIn(self.ads, ADS.P1)  # 5V/4
            self.chan2 = AnalogIn(self.ads, ADS.P2)  # 12V/4
            self.chan3 = AnalogIn(self.ads, ADS.P3)  # Amps
            self.available = True
            print("Battery sensor (ADS1115) initialized")
        except Exception as e:
            print(f"Warning: Battery sensor not available: {e}")
            print("Running without battery monitoring")

    def read_all(self):
        """Read all voltages and amps"""
        if not self.available:
            # Defaults: 5V, 5V, 12V, 1A
            return 5.0, 5.0, 12.0, 1.0
        v1 = self.chan0.voltage * 4.0
        v2 = self.chan1.voltage * 4.0
        v3 = self.chan2.voltage * 4.0
        amps = self.chan3.voltage  # Assume direct reading for amps
        return v1, v2, v3, amps

# ========================================
# Motor Controller (BTS7960)
# ========================================
class MotorController:
    def __init__(self):
        self.available = False
        self.enabled = False
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Setup motor pins
            GPIO.setup(MOTOR_LEFT_LPWM, GPIO.OUT)
            GPIO.setup(MOTOR_LEFT_RPWM, GPIO.OUT)
            GPIO.setup(MOTOR_RIGHT_LPWM, GPIO.OUT)
            GPIO.setup(MOTOR_RIGHT_RPWM, GPIO.OUT)
            GPIO.setup(MOTOR_ENABLE, GPIO.OUT)
            
            # Create PWM objects
            self.left_fwd = GPIO.PWM(MOTOR_LEFT_LPWM, PWM_FREQ)
            self.left_rev = GPIO.PWM(MOTOR_LEFT_RPWM, PWM_FREQ)
            self.right_fwd = GPIO.PWM(MOTOR_RIGHT_LPWM, PWM_FREQ)
            self.right_rev = GPIO.PWM(MOTOR_RIGHT_RPWM, PWM_FREQ)
            
            # Start PWM at 0
            self.left_fwd.start(0)
            self.left_rev.start(0)
            self.right_fwd.start(0)
            self.right_rev.start(0)
            
            # Enable motors
            GPIO.output(MOTOR_ENABLE, GPIO.HIGH)
            
            self.enabled = True
            self.available = True
            print("Motor controller (BTS7960) initialized")
        except Exception as e:
            print(f"Warning: Motor controller not available: {e}")
            print("Running without motor control")
        
    def set_speed(self, left_speed, right_speed):
        """
        Set motor speeds
        Args:
            left_speed: -1000 to +1000 (negative = reverse)
            right_speed: -1000 to +1000
        """
        if not self.available or not self.enabled:
            return
            
        # Clamp values
        left_speed = max(-1000, min(1000, left_speed))
        right_speed = max(-1000, min(1000, right_speed))
        
        # Left motor
        if left_speed >= 0:
            self.left_fwd.ChangeDutyCycle(abs(left_speed) / 10.0)  # 0-100%
            self.left_rev.ChangeDutyCycle(0)
        else:
            self.left_fwd.ChangeDutyCycle(0)
            self.left_rev.ChangeDutyCycle(abs(left_speed) / 10.0)
            
        # Right motor
        if right_speed >= 0:
            self.right_fwd.ChangeDutyCycle(abs(right_speed) / 10.0)
            self.right_rev.ChangeDutyCycle(0)
        else:
            self.right_fwd.ChangeDutyCycle(0)
            self.right_rev.ChangeDutyCycle(abs(right_speed) / 10.0)
            
    def brake(self):
        """Active brake (both directions off)"""
        if not self.available:
            return
        self.left_fwd.ChangeDutyCycle(0)
        self.left_rev.ChangeDutyCycle(0)
        self.right_fwd.ChangeDutyCycle(0)
        self.right_rev.ChangeDutyCycle(0)
        
    def enable(self, state=True):
        """Enable/disable motor drivers"""
        if not self.available:
            return
        self.enabled = state
        GPIO.output(MOTOR_ENABLE, GPIO.HIGH if state else GPIO.LOW)
        if not state:
            self.brake()
            
    def cleanup(self):
        """Stop motors and cleanup GPIO"""
        if not self.available:
            return
        self.brake()
        self.enable(False)
        self.left_fwd.stop()
        self.left_rev.stop()
        self.right_fwd.stop()
        self.right_rev.stop()
        GPIO.cleanup()

# ========================================
# Main Robot Controller
# ========================================
class RobotController:
    def __init__(self):
        print("Initializing Robot Controller...")
        
        self.gateway = ESP32Gateway()
        self.battery = BatterySensor()
        self.motors = MotorController()
        
        # Initialize I2C LCD (20x4 character display)
        self.lcd = None
        if LCD_AVAILABLE:
            try:
                # Try common I2C addresses for 2004A LCD
                for addr in [0x27, 0x3F]:
                    try:
                        self.lcd = CharLCD('PCF8574', addr, cols=20, rows=4)
                        self.lcd.clear()
                        self.lcd.cursor_pos = (0, 0)
                        self.lcd.write_string("Robot v1.0 INIT...")
                        print(f"LCD initialized at I2C address 0x{addr:02X}")
                        break
                    except:
                        continue
                if not self.lcd:
                    print("Warning: Could not find LCD at 0x27 or 0x3F")
            except Exception as e:
                print(f"Warning: Could not initialize LCD: {e}")
        
        # Register message handlers
        # self.gateway.register_callback(CMD_MOTOR, self.handle_motor_command)
        # self.gateway.register_callback(CMD_UI, self.handle_ui_event)
        # self.gateway.register_callback(STA_UWB, self.handle_uwb_data)
        
        # State
        self.last_telemetry = time.time()
        self.last_heartbeat = time.time()
        self.last_lcd_update = time.time()
        self.running = True
        self.left_speed = 0
        self.right_speed = 0
        
        # LCD line cache to avoid unnecessary updates
        self.lcd_lines = ["", "", "", ""]
        self.last_voltages = (None, None, None, None)
        self.last_event_rx = ""
        self.last_event_tx = ""
        
        # Update LCD
        if self.lcd:
            self.lcd.clear()
            self.lcd.cursor_pos = (0, 0)
            self.lcd.write_string("Robot v1.0 ACTIVE")
            time.sleep(1)  # Show for 1 second
            # Log startup messages
            self.log_message("Robot v1.0")
            self.log_message("System READY")
        
        print("Robot Controller Ready")
    
    def log_event_rx(self, event):
        # Abbreviated event received from remote
        event = event[:20]
        if event != self.last_event_rx:
            self.last_event_rx = event
            self.lcd_lines[2] = event

    def log_event_tx(self, event):
        # Abbreviated event sent to remote
        event = event[:20]
        if event != self.last_event_tx:
            self.last_event_tx = event
            self.lcd_lines[3] = event
        
    def handle_motor_command(self, payload):
        """Process motor command from remote"""
        if len(payload) < 6:
            return
        left_speed, right_speed, brake, enable = struct.unpack('<hhBB', payload)
        self.left_speed = left_speed
        self.right_speed = right_speed
        # Log abbreviated event received
        self.log_event_rx(f"RX:L{left_speed:+4d}R{right_speed:+4d}")
        if not enable:
            self.motors.enable(False)
            self.log_event_rx("Motors DISABLED")
        elif brake:
            self.motors.brake()
            self.log_event_rx("Motors BRAKING")
        else:
            self.motors.enable(True)
            self.motors.set_speed(left_speed, right_speed)
            
    def handle_ui_event(self, payload):
        """Process UI event from remote"""
        # Forwarded from remote's touch screen
        # Parse and act on it
        pass
        
    def handle_uwb_data(self, payload):
        """Process UWB ranging data"""
        # Will be used for follow-me mode
        pass
        
    def send_telemetry(self):
        """Send sensor telemetry to remote"""
        v1, v2, v3, amps = self.battery.read_all()
        speed_fps = 0.0
        status_code = 0  # OK
        rssi = -45  # Placeholder (negative dBm value)
        timestamp = int(time.time() * 1000)
        timestamp &= 0xFFFFFFFF
        # Format: 4 floats, 1 unsigned byte (status), 1 signed byte (rssi), 1 unsigned int (timestamp)
        payload = struct.pack('<ffffBbI', v1, v2, v3, amps, status_code, rssi, timestamp)
        self.gateway.send_message(STA_TELEMETRY, payload)
        # Log abbreviated event sent
        self.log_event_tx(f"TX:V1={v1:.2f} V2={v2:.2f} V3={v3:.2f} A={amps:.2f}")
        
    def send_heartbeat(self):
        """Send keepalive to ESP32"""
        uptime = int(time.time() * 1000) & 0xFFFFFFFF
        health = 0  # Healthy
        payload = struct.pack('<IB', uptime, health)
        self.gateway.send_message(MSG_HEARTBEAT, payload)
    
    def update_lcd(self):
        """Update I2C LCD display (20x4 chars) with voltages and event history"""
        if not self.lcd:
            return
        try:
            # Line 0: Voltages 1,2,3
            v1, v2, v3, v4 = self.battery.read_all()
            volt_line = f"1:{v1:.2f} 2:{v2:.2f} 3:{v3:.2f}"
            if volt_line != self.lcd_lines[0]:
                self.lcd_lines[0] = volt_line
                self.lcd.cursor_pos = (0, 0)
                self.lcd.write_string(f"{volt_line:<20}")
            # Line 1: Voltage 4
            volt4_line = f"4:{v4:.2f}"
            if volt4_line != self.lcd_lines[1]:
                self.lcd_lines[1] = volt4_line
                self.lcd.cursor_pos = (1, 0)
                self.lcd.write_string(f"{volt4_line:<20}")
            # Line 2: Abbreviated event received
            if self.lcd_lines[2]:
                self.lcd.cursor_pos = (2, 0)
                self.lcd.write_string(f"{self.lcd_lines[2]:<20}")
            # Line 3: Abbreviated event sent
            if self.lcd_lines[3]:
                self.lcd.cursor_pos = (3, 0)
                self.lcd.write_string(f"{self.lcd_lines[3]:<20}")
        except Exception:
            pass
            pass
    
    def run(self):
        """Main control loop"""
        try:
            while self.running:
                # Process incoming messages
                # self.gateway.process()
                
                # Send telemetry at 10Hz
                now = time.time()
                if now - self.last_telemetry >= 0.1:
                    self.last_telemetry = now
                    self.send_telemetry()
                    
                # Send heartbeat at 1Hz
                if now - self.last_heartbeat >= 1.0:
                    self.last_heartbeat = now
                    self.send_heartbeat()
                
                # Update LCD at 5Hz
                if now - self.last_lcd_update >= 0.2:
                    self.last_lcd_update = now
                    self.update_lcd()
                    
                time.sleep(0.01)  # 100Hz loop
                
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            self.motors.cleanup()
            # self.gateway.ser.close()
            if self.lcd:
                try:
                    self.lcd.clear()
                    self.lcd.write_string("Robot Controller\nShutdown")
                except:
                    pass

APP_VERSION = f"0.001-{datetime.now().strftime('%Y%m%d-%H%M%S')}"
APP_ROLE = "robot"

# ========================================
# Entry Point
# ========================================
if __name__ == '__main__':
    print(f"App Version: {APP_VERSION} | Role: {APP_ROLE}")
    controller = RobotController()
    controller.run()
