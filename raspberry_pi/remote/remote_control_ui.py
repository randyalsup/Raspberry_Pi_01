#!/usr/bin/env python3
"""
Robot Remote Control UI
Pygame-based touch interface for Pi Zero 2W with Waveshare 3.5" LCD
"""

import os
import pygame
import argparse
import sys
import serial
import struct
import math
import time
import mmap
        is_connected = "Connected" in self.connection_status
        status_color = GREEN if is_connected else RED
import select
from threading import Thread, Lock
        if not is_connected:
            waiting_text = self.font_small.render("Waiting for connection...", True, GRAY)
            self.screen.blit(waiting_text, (SCREEN_WIDTH // 2 - waiting_text.get_width() // 2, 70))

# Try to import NumPy for fast RGB565 conversion
        pygame.draw.rect(self.screen, DARK_GRAY, (10, 90, SCREEN_WIDTH - 20, 120), 2)
    import numpy as np 
    NUMPY_AVAILABLE = True
        tele_color = WHITE if is_connected else DARK_GRAY
        battery_text = self.font_small.render(f"Battery: {self.battery_voltage:.1f}V", True, tele_color)
    test_array = np.array([1, 2, 3], dtype=np.uint16)
    test_bytes = test_array.tobytes()
        current_text = self.font_small.render(f"Current: {self.battery_current:.1f}A", True, tele_color)
except Exception as e:
    NUMPY_AVAILABLE = False
    print(f"✗ WARNING: NumPy error: {e}")
        rssi_color = (GREEN if self.rssi > -70 else (YELLOW if self.rssi > -85 else RED)) if is_connected else DARK_GRAY
        rssi_text = self.font_small.render(f"Signal: {self.rssi} dBm", True, rssi_color)

# Try to import I2C LCD library
try:
        left_text = self.font_small.render(f"L: {self.left_speed:+5d}", True, tele_color)
        right_text = self.font_small.render(f"R: {self.right_speed:+5d}", True, tele_color)
except ImportError:
    LCD_AVAILABLE = False
    print("Warning: RPLCD not installed. Install with: sudo pip3 install RPLCD")

        if is_connected:
            self.controller.draw(self.screen)
os.environ['SDL_VIDEODRIVER'] = 'dummy'
        # Instructions
        if is_connected:
            inst_text = self.font_small.render("Touch to control", True, GRAY)
            self.screen.blit(inst_text, (SCREEN_WIDTH // 2 - inst_text.get_width() // 2, SCREEN_HEIGHT - 30))
SCREEN_WIDTH = 320
SCREEN_HEIGHT = 480
FPS = 30  # 30 FPS - requires NumPy for fast RGB565 conversion

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 100, 255)
GRAY = (100, 100, 100)
DARK_GRAY = (50, 50, 50)
YELLOW = (255, 255, 0)

# Message types
CMD_MOTOR = 0x01
STA_TELEMETRY = 0x10
MSG_HEARTBEAT = 0x20

class UARTProtocol:
    """UART protocol handler with CRC-8"""
    
    SYNC = 0xAA
    END = 0x55
    
    def __init__(self, port='/dev/serial0', baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=0.01)
        self.lock = Lock()
        
    def crc8(self, data):
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
    
    def send_motor_cmd(self, left_speed, right_speed, brake=0, enable=1):
        """Send motor command"""
        payload = struct.pack('<hhBB', left_speed, right_speed, brake, enable)
        self._send_message(CMD_MOTOR, payload)
    
    def _send_message(self, msg_type, payload):
        """Send framed message with CRC"""
        with self.lock:
            length = len(payload)
            crc = self.crc8([msg_type, length] + list(payload))
            frame = bytes([self.SYNC, msg_type, length]) + payload + bytes([crc, self.END])
            self.ser.write(frame)
    
    def read_telemetry(self):
        """Read telemetry if available (non-blocking)"""
        # Simple implementation - just check for available data
        if self.ser.in_waiting > 0:
            # TODO: Implement proper framing parser
            pass
        return None
    
    def close(self):
        """Close serial port"""
        self.ser.close()


class ArrowController:
    """Arrow button controller with incremental speed control"""
    
    def __init__(self, x, y, size):
        self.center_x = x
        self.center_y = y
        self.button_size = size  # Size of each arrow button
        self.stop_radius = size // 3  # Stop button in center
        
        # Touch coordinate calibration (will be set by RemoteControlUI)
        self.touch_width = 320
        self.touch_height = 480
        
        # Current speed values (-100 to 100 percent)
        self.forward_back_speed = 0.0  # Positive = forward, negative = backward
        self.left_right_speed = 0.0    # Positive = right, negative = left
        
        # Speed increment per button press (25% = 4 presses to reach 100%)
        self.speed_increment = 25.0
        
        # Track if changed for redraw
        self.changed = False
        
        # Last touch to debounce
        self.last_touch_time = 0
        self.debounce_time = 0.15  # 150ms between presses
        # Cache small font once to avoid per-draw allocations
        self._font_small = pygame.font.Font(None, 20)
        
    def get_button_rects(self):
        """Get the rectangles for each arrow button"""
        bs = self.button_size
        # Up arrow (above center)
        up_rect = (self.center_x - bs//2, self.center_y - bs*1.5, bs, bs)
        # Down arrow (below center)
        down_rect = (self.center_x - bs//2, self.center_y + bs*0.5, bs, bs)
        # Left arrow
        left_rect = (self.center_x - bs*1.5, self.center_y - bs//2, bs, bs)
        # Right arrow
        right_rect = (self.center_x + bs*0.5, self.center_y - bs//2, bs, bs)
        # Stop button (center circle)
        stop_center = (self.center_x, self.center_y)
        
        return {
            'up': up_rect,
            'down': down_rect,
            'left': left_rect,
            'right': right_rect,
            'stop': stop_center
        }
    
    def check_button_press(self, screen_x, screen_y):
        """Check which button was pressed, returns button name or None"""
        now = time.time()
        if now - self.last_touch_time < self.debounce_time:
            return None  # Too soon, debounce
        
        buttons = self.get_button_rects()
        dprint(f"[HIT TEST] Touch at ({screen_x}, {screen_y})")
        # Check stop button first (circular)
        dx = screen_x - buttons['stop'][0]
        dy = screen_y - buttons['stop'][1]
        dist = math.sqrt(dx*dx + dy*dy)
        dprint(f"[HIT TEST] Stop button: center=({buttons['stop'][0]}, {buttons['stop'][1]}) radius={self.stop_radius} distance={dist:.1f}")
        if dist <= self.stop_radius:
            self.last_touch_time = now
            dprint(f"[HIT TEST] -> HIT: stop")
            return 'stop'
                
                if "Connected" not in self.connection_status:
                    # Ignore touch inputs until connected; keep state tidy on release
                    if not touching:
                        self.touch_down_x = None
                        self.touch_down_y = None
                else:
                    if touching:
                        dprint()  # Blank line before touch down
                        # Touch down - only process if we don't already have a stored touch
                        if self.touch_down_x is None:
                            touch_down_time = time.time()
                            dprint(f"[TIMING] Touch DOWN at {touch_down_time:.3f}")
                            dprint(f"[COORD] Raw touch coordinates: x={x}, y={y}")
                            dprint(f"[COORD] Screen is 320x480, buttons centered at y=350")
                            dprint(f"[COORD] If y=0 and you touched UP button, coordinates may be INVERTED")
                            # Debug: Compare with last touch event
                            if self.last_touch_x is not None:
                                dx = x - self.last_touch_x
                                dy = y - self.last_touch_y
                                distance = math.sqrt(dx*dx + dy*dy)
                                time_delta = time.time() - self.last_touch_time if self.last_touch_time else 0
                                dprint(f"[DEBUG] New touch: ({x}, {y})")
                                dprint(f"[DEBUG] Last touch: ({self.last_touch_x}, {self.last_touch_y}) [{self.last_button_pressed}]")
                                dprint(f"[DEBUG] Delta: dx={dx:+d} dy={dy:+d} dist={distance:.1f}px time={time_delta:.3f}s")
                            else:
                                dprint(f"[TOUCH] First touch at ({x}, {y})")
                            
                            self.touch_down_x = x
                            self.touch_down_y = y
                            
                            # Process button immediately on touch down
                            button = self.controller.check_button_press(x, y)
                            if button:
                                self.controller.handle_button_press(button)
                                
                                # Log button press
                                if button == 'stop':
                                    self.log_message("BTN: STOP")
                                    # Emergency stop - reset motor speeds immediately
                                    self.target_left_speed = 0
                                    self.target_right_speed = 0
                                    self.current_left_speed = 0.0
                                    self.current_right_speed = 0.0
                                    self.left_speed = 0
                                    self.right_speed = 0
                                    if self.uart:
                                        try:
                                            self.uart.send_motor_cmd(0, 0, brake=1, enable=0)
                                        except:
                                            pass
                                else:
                                    # Log arrow button press
                                    fb = self.controller.forward_back_speed
                                    lr = self.controller.left_right_speed
                                    self.log_message(f"{button.upper()}:FB{fb:+.0f}LR{lr:+.0f}")
                                
                                # Store this touch for comparison with next touch
                                self.last_touch_x = x
                                self.last_touch_y = y
                                self.last_touch_time = time.time()
                                self.last_button_pressed = button
                            else:
                                dprint(f"[TOUCH] Down - no button at location ({x}, {y})")
                                # Clear the stored touch since no button was pressed
                                self.touch_down_x = None
                                self.touch_down_y = None
                        else:
                            dprint(f"[TOUCH] Down ignored - already have stored touch")
                    else:
                        dprint()  # Blank line before release
                        # Touch release - just reset state
                        touch_up_time = time.time()
                        dprint(f"[TIMING] Touch RELEASE at {touch_up_time:.3f}")
                        dprint(f"[TOUCH] Release - resetting state")
                        self.touch_down_x = None
                        self.touch_down_y = None
        # Convert percentages to -1.0 to 1.0 range
        x = self.left_right_speed / 100.0
        y = self.forward_back_speed / 100.0
        return x, y
    
    def draw(self, screen):
        """Draw arrow buttons and stop button"""
        buttons = self.get_button_rects()
        bs = self.button_size
        
        # Draw UP arrow
        up_rect = buttons['up']
        pygame.draw.rect(screen, DARK_GRAY, up_rect)
        pygame.draw.rect(screen, WHITE, up_rect, 2)
        # Draw triangle pointing up
        points = [(up_rect[0] + bs//2, up_rect[1] + bs//4),
                  (up_rect[0] + bs//4, up_rect[1] + 3*bs//4),
                  (up_rect[0] + 3*bs//4, up_rect[1] + 3*bs//4)]
        pygame.draw.polygon(screen, WHITE, points)
        
        # Draw DOWN arrow
        down_rect = buttons['down']
        pygame.draw.rect(screen, DARK_GRAY, down_rect)
        pygame.draw.rect(screen, WHITE, down_rect, 2)
        # Draw triangle pointing down
        points = [(down_rect[0] + bs//2, down_rect[1] + 3*bs//4),
                  (down_rect[0] + bs//4, down_rect[1] + bs//4),
                  (down_rect[0] + 3*bs//4, down_rect[1] + bs//4)]
        pygame.draw.polygon(screen, WHITE, points)
        
        # Draw LEFT arrow
        left_rect = buttons['left']
        pygame.draw.rect(screen, DARK_GRAY, left_rect)
        pygame.draw.rect(screen, WHITE, left_rect, 2)
        # Draw triangle pointing left
        points = [(left_rect[0] + bs//4, left_rect[1] + bs//2),
                  (left_rect[0] + 3*bs//4, left_rect[1] + bs//4),
                  (left_rect[0] + 3*bs//4, left_rect[1] + 3*bs//4)]
        pygame.draw.polygon(screen, WHITE, points)
        
        # Draw RIGHT arrow
        right_rect = buttons['right']
        pygame.draw.rect(screen, DARK_GRAY, right_rect)
        pygame.draw.rect(screen, WHITE, right_rect, 2)
        # Draw triangle pointing right
        points = [(right_rect[0] + 3*bs//4, right_rect[1] + bs//2),
                  (right_rect[0] + bs//4, right_rect[1] + bs//4),
                  (right_rect[0] + bs//4, right_rect[1] + 3*bs//4)]
        pygame.draw.polygon(screen, WHITE, points)
        
        # Draw STOP button in center (red circle)
        stop_pos = buttons['stop']
        pygame.draw.circle(screen, RED, stop_pos, self.stop_radius)
        pygame.draw.circle(screen, WHITE, stop_pos, self.stop_radius, 2)
        
        # Draw speed indicators as text (use cached font)
        fb_text = self._font_small.render(f"FB:{self.forward_back_speed:+.1f}%", True, YELLOW)
        lr_text = self._font_small.render(f"LR:{self.left_right_speed:+.1f}%", True, YELLOW)
        screen.blit(fb_text, (self.center_x - fb_text.get_width()//2, self.center_y - bs*2))
        screen.blit(lr_text, (self.center_x - lr_text.get_width()//2, self.center_y + bs*1.2))


class TouchInput:
    """Read touch input directly from Linux input device"""
    
    def __init__(self):
        self.touch_x = 0
        self.touch_y = 0
        self.touching = False
        self.last_x = -1
        self.last_y = -1
        self.was_touching = False  # Track previous touch state
        self.last_touch_time = 0  # Track when last touch started
        self.touch_timeout = 0.5  # Timeout in seconds (500ms)
        self.device = None
        self.event_count = 0  # Debug counter
        
        # Find touch input device (goodix touchscreen)
        dprint("DEBUG: Searching for touch device...")
        dprint(f"DEBUG: Running as user: {os.getuid() if hasattr(os, 'getuid') else 'unknown'}")
        
        # Read /proc/bus/input/devices to find the touchscreen
        touchscreen_event = None
        try:
            with open('/proc/bus/input/devices', 'r') as f:
                content = f.read()
                # Look for Goodix touchscreen
                for block in content.split('\n\n'):
                    if 'Goodix' in block or 'Touch' in block or 'touch' in block:
                        # Extract event handler
                        for line in block.split('\n'):
                            if line.startswith('H: Handlers='):
                                # Extract event number
                                import re
                                match = re.search(r'event(\d+)', line)
                                if match:
                                    touchscreen_event = f'/dev/input/event{match.group(1)}'
                                    dprint(f"DEBUG: Found touchscreen at {touchscreen_event}")
                                    dprint(f"DEBUG: Device info: {block.split(chr(10))[1]}")  # Name line
                                    break
                        if touchscreen_event:
                            break
        except Exception as e:
            dprint(f"DEBUG: Could not read /proc/bus/input/devices: {e}")
        
        # If we found the touchscreen, try it first, otherwise fall back to event0
        if touchscreen_event:
            preferred_devices = [touchscreen_event]
        else:
            dprint("DEBUG: Could not find touchscreen, trying event0-2...")
            preferred_devices = ['/dev/input/event0', '/dev/input/event1', '/dev/input/event2']
        
        for device_path in preferred_devices + glob.glob('/dev/input/event*'):
            if device_path in preferred_devices[1:] and self.device:
                continue  # Skip if already found
            try:
                dprint(f"DEBUG: Trying {device_path}...")
                # Try opening for read in non-blocking mode
                import fcntl
                fd = os.open(device_path, os.O_RDONLY | os.O_NONBLOCK)
                self.device = os.fdopen(fd, 'rb')
                self.device_path = device_path
                dprint(f"✓ Opened touch device: {device_path}")
                
                # Flush any pending events from the device buffer
                dprint(f"  Flushing initial events...")
                flushed = 0
                while True:
                    ready, _, _ = select.select([self.device], [], [], 0)
                    if not ready:
                        break
                    try:
                        self.device.read(16)
                        flushed += 1
                    except:
                        break
                if flushed > 0:
                    dprint(f"  Flushed {flushed} stale events")
                # Reset touch state after flush
                self.touching = False
                self.was_touching = False
                
                # Try to get device name
                try:
                    # Get device name using ioctl (if available)
                    device_name = "unknown"
                    dprint(f"  Device: {device_name}")
                except:
                    pass
                    
                break
            except PermissionError:
                dprint(f"DEBUG: Permission denied for {device_path} - need sudo!")
            except Exception as e:
                dprint(f"DEBUG: Failed to open {device_path}: {e}")
                continue
        
        if not self.device:
            dprint("✗ ERROR: Could not open any touch input device!")
            dprint("  Make sure you run this script with sudo:")
            dprint("  sudo python3 remote_control_ui.py")
            dprint("\n  Available devices:")
            import subprocess
            result = subprocess.run(['ls', '-la', '/dev/input/'], capture_output=True, text=True)
            dprint(result.stdout)
    
    def read_event(self):
        """Non-blocking read of touch events - processes one complete touch sequence"""
        if not self.device:
            return None
        
        # Check for timeout - if touch has been held too long, force a release
        import time
        if self.touching and (time.time() - self.last_touch_time) > self.touch_timeout:
            dprint(f"[EVENT] Touch timeout - forcing release after {self.touch_timeout}s")
            self.touching = False
            self.was_touching = False
            return (self.touch_x, self.touch_y, False)
        
        events_processed = 0
        max_events = 50  # Prevent infinite loops
        
        # Process events until we get a complete state change
        while events_processed < max_events:
            # Check if data is available (non-blocking)
            ready, _, _ = select.select([self.device], [], [], 0)
            if not ready:
                if events_processed > 0:
                    dprint(f"[EVENT] No more data after processing {events_processed} events, touching={self.touching}, was={self.was_touching}")
                return None  # No data available
            
            events_processed += 1
            dprint(f"[EVENT] Processing event #{events_processed}, touching={self.touching}, was={self.was_touching}")
            
            try:
                # Read input event structure
                # Try 16 bytes first (32-bit system), then 24 bytes (64-bit system)
                data = self.device.read(16)
                if len(data) < 16:
                    return None
                
                # Parse event structure for 32-bit system
                # struct input_event { timeval (long sec, long usec), unsigned short type, unsigned short code, int value }
                try:
                    sec, usec, ev_type, ev_code, ev_value = struct.unpack('llHHi', data)
                except:
                    # Try reading more for 64-bit
                    extra = self.device.read(8)
                    if len(extra) == 8:
                        data = data + extra
                        sec, usec, ev_type, ev_code, ev_value = struct.unpack('QQHHi', data)
                    else:
                        return None
                
                # EV_ABS (absolute axis events) = 3
                if ev_type == 3:
                    # Handle both single-touch and multi-touch protocols
                    # Single-touch: ABS_X = 0, ABS_Y = 1
                    # Multi-touch: ABS_MT_POSITION_X = 53, ABS_MT_POSITION_Y = 54
                    if ev_code == 0 or ev_code == 53:  # X coordinate (single or multi-touch)
                        self.touch_x = ev_value
                    elif ev_code == 1 or ev_code == 54:  # Y coordinate (single or multi-touch)
                        self.touch_y = ev_value
                    elif ev_code == 24:  # ABS_PRESSURE
                        self.touching = ev_value > 0
                    elif ev_code == 57:  # ABS_MT_TRACKING_ID
                        # -1 means touch released, >= 0 means touch active
                        self.touching = ev_value >= 0
                
                # EV_KEY (button/touch events) = 1
                elif ev_type == 1:
                    # BTN_TOUCH = 330
                    if ev_code == 330:
                        self.touching = ev_value > 0
                
                # EV_SYN (synchronization event) = 0
                elif ev_type == 0:
                    # Return on touch state changes (down or up)
                    if self.touching and not self.was_touching:
                        # Touch down - new press
                        self.was_touching = True
                        self.last_x = self.touch_x
                        self.last_y = self.touch_y
                        self.last_touch_time = time.time()  # Record touch start time
                        dprint(f"[EVENT] Returning touch DOWN at ({self.touch_x}, {self.touch_y})")
                        return (self.touch_x, self.touch_y, True)
                    elif not self.touching and self.was_touching:
                        # Touch up - release
                        self.was_touching = False
                        dprint(f"[EVENT] Returning touch UP at ({self.touch_x}, {self.touch_y})")
                        return (self.touch_x, self.touch_y, False)
                    else:
                        # SYNC with no state change - might indicate we're missing something
                        dprint(f"[EVENT] SYNC #{events_processed} no state change: touching={self.touching}, was={self.was_touching}")
                    # If no state change, continue looping to process next raw event
                
            except Exception as e:
                dprint(f"Error reading touch event: {e}")
                return None
        
        # Hit max events without state change
        dprint(f"[EVENT] WARNING: Processed {max_events} events without state change")
        return None


    
    def close(self):
        if self.device:
            self.device.close()


DEBUG = False

def dprint(*args, **kwargs):
    if DEBUG:
        print(*args, **kwargs)

class RemoteControlUI:
    """Main UI application"""
    
    def __init__(self):
        # Initialize pygame with dummy driver
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("Robot Remote Control")
        self.clock = pygame.time.Clock()
        self.font_large = pygame.font.Font(None, 36)
        self.font_small = pygame.font.Font(None, 24)
        
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
                        self.lcd.write_string("Remote Control v1.0")
                        print(f"LCD initialized at I2C address 0x{addr:02X}")
                        break
                    except:
                        continue
                if not self.lcd:
                    print("Warning: Could not find LCD at 0x27 or 0x3F")
            except Exception as e:
                print(f"Warning: Could not initialize LCD: {e}")
        
        # Open framebuffer device for direct writing
        try:
            self.fb = open('/dev/fb0', 'rb+')
            # ST7796S uses RGB565 format (2 bytes per pixel)
            self.fbmmap = mmap.mmap(self.fb.fileno(), SCREEN_WIDTH * SCREEN_HEIGHT * 2)
            # Pre-allocate conversion buffer for performance
            self.rgb565_buffer = bytearray(SCREEN_WIDTH * SCREEN_HEIGHT * 2)
            print("Framebuffer opened successfully - display will update on LCD")
        except Exception as e:
            print(f"Warning: Could not open framebuffer: {e}")
            print("Running without display output")
            self.fb = None
            self.fbmmap = None
        
        # Touch input
        self.touch = TouchInput()
        
        # UART protocol
        try:
            self.uart = UARTProtocol()
        except Exception as e:
            print(f"Warning: Could not open UART: {e}")
            print("Running in demo mode without UART")
            self.uart = None
        
        # Arrow controller
        self.controller = ArrowController(SCREEN_WIDTH // 2, 350, 60)
        
        # Telemetry data
        self.battery_voltage = 0.0
        self.battery_current = 0.0
        self.rssi = 0
        self.connection_status = "Connecting..."
        
        # Motor speeds (for display)
        self.left_speed = 0
        self.right_speed = 0
        
        # Motor speed ramping (2 second ramp-up time)
        self.target_left_speed = 0
        self.target_right_speed = 0
        self.current_left_speed = 0.0
        self.current_right_speed = 0.0
        self.ramp_time = 2.0  # 2 seconds to reach target speed
        self.last_update_time = time.time()
        
        # Touch state tracking
        self.touch_down_x = None
        self.touch_down_y = None
        
        # Debug: Last touch event tracking
        self.last_touch_x = None
        self.last_touch_y = None
        self.last_touch_time = None
        self.last_button_pressed = None
        
        # Timing
        self.last_motor_cmd = 0
        self.motor_cmd_interval = 0.05  # 20Hz motor updates
        
        # Message history for LCD display (4 lines, newest first)
        self.message_history = ["", "", "", ""]
        
    def differential_steering(self, x, y):
        """Convert joystick X/Y to differential motor speeds"""
        # Y = forward/backward, X = left/right
        # Range: -1.0 to 1.0
        
        # Tank steering algorithm
        left = y + x
        right = y - x
        
        # Normalize to -1.0 to 1.0
        max_val = max(abs(left), abs(right))
        if max_val > 1.0:
            left /= max_val
            right /= max_val
        
        # Convert to motor range (-1000 to 1000)
        left_speed = int(left * 1000)
        right_speed = int(right * 1000)
        
        return left_speed, right_speed
    
    def log_message(self, message):
        """Add a message to the LCD history (newest at top)"""
        # Truncate message to 20 characters (LCD width)
        message = message[:20]
        # Push all messages down and add new one at top
        self.message_history = [message] + self.message_history[:3]
        # Force LCD update
        self.update_lcd()
    
    def update(self):
        """Update logic"""
        # Note: Controller is updated by touch handler in main loop
        # Don't overwrite with pygame mouse which doesn't work with touch
        
        # Get controller values (already updated by touch handler)
        x, y = self.controller.get_values()
        
        # Debug: Print controller state
        print(f"[UPDATE] Controller: FB={self.controller.forward_back_speed:.1f}% LR={self.controller.left_right_speed:.1f}% x={x:.3f} y={y:.3f}")
        
        # Calculate target motor speeds
        self.target_left_speed, self.target_right_speed = self.differential_steering(x, y)
        print(f"[UPDATE] Target speeds: L={self.target_left_speed} R={self.target_right_speed}")
        
        # Calculate time delta for smooth ramping
        now = time.time()
        dt = now - self.last_update_time
        self.last_update_time = now
        
        # Calculate maximum speed change per second
        max_speed_change_per_sec = 500.0 / self.ramp_time  # 1000 = full scale / 4 seconds
        max_change = max_speed_change_per_sec * dt
        
        # Ramp current speeds towards target speeds
        left_diff = self.target_left_speed - self.current_left_speed
        right_diff = self.target_right_speed - self.current_right_speed
        
        # Apply ramping
        if abs(left_diff) <= max_change:
            self.current_left_speed = self.target_left_speed
        else:
            self.current_left_speed += max_change if left_diff > 0 else -max_change
        
        if abs(right_diff) <= max_change:
            self.current_right_speed = self.target_right_speed
        else:
            self.current_right_speed += max_change if right_diff > 0 else -max_change
        
        # Update display speeds (rounded to int)
        self.left_speed = int(self.current_left_speed)
        self.right_speed = int(self.current_right_speed)
        
        # Debug: Print actual displayed speeds
        if abs(self.left_speed) > 0 or abs(self.right_speed) > 0:
            print(f"[UPDATE] Display speeds: L={self.left_speed:+5d} R={self.right_speed:+5d}")
        
        # Send motor command at regular intervals
        if now - self.last_motor_cmd >= self.motor_cmd_interval:
            if self.uart:
                try:
                    self.uart.send_motor_cmd(self.left_speed, self.right_speed, brake=0, enable=1)
                    self.connection_status = "Connected"
                    # Log TX event
                    self.log_message(f"TX:L{self.left_speed:+4d}R{self.right_speed:+4d}")
                except Exception as e:
                    self.connection_status = f"Error: {e}"
                    self.log_message(f"ERR: {str(e)[:15]}")
            self.last_motor_cmd = now
        
        # Update LCD display every frame (not just when sending commands)
        # Note: update_lcd is now called by log_message
    
    def update_lcd(self):
        """Update I2C LCD display (20x4 chars) with scrolling message history"""
        if not self.lcd:
            return
        
        try:
            # Display 4 lines of message history (newest at top)
            for line_num in range(4):
                self.lcd.cursor_pos = (line_num, 0)
                msg = self.message_history[line_num] if line_num < len(self.message_history) else ""
                # Pad to 20 chars to clear any previous text
                self.lcd.write_string(f"{msg:<20}")
        except Exception as e:
            # Don't crash if LCD has issues
            pass
    
    def draw(self):
        """Render UI"""
        self.screen.fill(BLACK)
        
        # Title bar
        title = self.font_large.render("Robot Control", True, WHITE)
        self.screen.blit(title, (SCREEN_WIDTH // 2 - title.get_width() // 2, 10))
        
        # Connection status
        status_color = GREEN if "Connected" in self.connection_status else RED
        status_text = self.font_small.render(self.connection_status, True, status_color)
        self.screen.blit(status_text, (SCREEN_WIDTH // 2 - status_text.get_width() // 2, 50))
        
        # Telemetry box
        pygame.draw.rect(self.screen, DARK_GRAY, (10, 90, SCREEN_WIDTH - 20, 120), 2)
        
        # Battery info
        battery_text = self.font_small.render(f"Battery: {self.battery_voltage:.1f}V", True, WHITE)
        self.screen.blit(battery_text, (20, 100))
        
        current_text = self.font_small.render(f"Current: {self.battery_current:.1f}A", True, WHITE)
        self.screen.blit(current_text, (20, 130))
        
        # Signal strength
        rssi_color = GREEN if self.rssi > -70 else (YELLOW if self.rssi > -85 else RED)
        rssi_text = self.font_small.render(f"Signal: {self.rssi} dBm", True, rssi_color)
        self.screen.blit(rssi_text, (20, 160))
        
        # Motor speeds
        left_text = self.font_small.render(f"L: {self.left_speed:+5d}", True, WHITE)
        right_text = self.font_small.render(f"R: {self.right_speed:+5d}", True, WHITE)
        self.screen.blit(left_text, (20, 190))
        self.screen.blit(right_text, (SCREEN_WIDTH - 120, 190))
        
        # Arrow controller (only when connected)
        if "Connected" in self.connection_status:
            self.controller.draw(self.screen)
            # Instructions
            inst_text = self.font_small.render("Touch to control", True, GRAY)
            self.screen.blit(inst_text, (SCREEN_WIDTH // 2 - inst_text.get_width() // 2, SCREEN_HEIGHT - 30))
        
        # Copy pygame surface to framebuffer (prefer zero-copy surfarray view)
        if self.fbmmap:
            t_start = time.time()
            
            if NUMPY_AVAILABLE:
                # Zero-copy 3D array view into surface pixels (width x height x 3)
                pixels3d = pygame.surfarray.pixels3d(self.screen)
                
                # Flatten channels and convert RGB888 -> RGB565 with minimal temporaries
                r = pixels3d[:, :, 0].astype(np.uint16)
                g = pixels3d[:, :, 1].astype(np.uint16)
                b = pixels3d[:, :, 2].astype(np.uint16)
                rgb565 = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
                # Write to framebuffer (uint16 little-endian)
                self.fbmmap.seek(0)
                self.fbmmap.write(rgb565.astype(np.uint16).tobytes())
                
                elapsed_ms = (time.time() - t_start) * 1000
                if elapsed_ms > 50:  # Only print if slow
                    print(f"⚠ Frame took {elapsed_ms:.0f}ms")
            else:
                # Fallback: slower path. Avoid extra allocation by reusing buffer.
                rgb_data = pygame.image.tostring(self.screen, 'RGB')
                for i in range(0, len(rgb_data), 3):
                    r = rgb_data[i] >> 3      # 5 bits
                    g = rgb_data[i+1] >> 2    # 6 bits
                    b = rgb_data[i+2] >> 3    # 5 bits
                    rgb565 = (r << 11) | (g << 5) | b
                    idx = (i // 3) * 2
                    self.rgb565_buffer[idx] = rgb565 & 0xFF
                    self.rgb565_buffer[idx + 1] = (rgb565 >> 8) & 0xFF
                self.fbmmap.seek(0)
                self.fbmmap.write(self.rgb565_buffer)
        
        pygame.display.flip()
    
    def run(self):
        """Main loop"""
        running = True
        frame_count = 0
        draw_count = 0
        
        # Draw initial screen
        print("Drawing initial screen...")
        self.draw()
        print("Initial screen drawn")
        
        # Log startup message
        self.log_message("Remote Control v1.0")
        self.log_message("Ready")
        
        # Diagnostic info
        print("\n" + "="*60)
        print("TOUCH DEBUG INFO:")
        print(f"Touch device opened: {self.touch.device is not None}")
        if self.touch.device:
            print(f"Touch device path: {getattr(self.touch, 'device_path', 'unknown')}")
            print(f"Touch device fileno: {self.touch.device.fileno()}")
        
        # List all input devices to find the touchscreen
        dprint("\nChecking all input devices:")
        import subprocess
        try:
            # List devices with their names
            result = subprocess.run(['ls', '-la', '/dev/input/by-path/'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                dprint("Devices by path:")
                dprint(result.stdout)
            
            # Try to read device names using cat /proc/bus/input/devices
            result = subprocess.run(['cat', '/proc/bus/input/devices'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                dprint("\nInput devices info:")
                dprint(result.stdout)
        except Exception as e:
            dprint(f"Could not list devices: {e}")
        
        dprint("\nStarting main loop...")
        dprint("="*60 + "\n")
        
        while running:
            frame_count += 1
            
            # Debug every second to show loop is running
        #    if frame_count % 30 == 0:
        #        print(f"[LOOP] Frame {frame_count}, Drew {draw_count} times")
            
            # Read touch events directly from input device
            touch_event = self.touch.read_event()
            if touch_event:
                x, y, touching = touch_event
                dprint(f"[RAW] Touch event: touching={touching} at ({x}, {y})")
                
                if touching:
                    dprint()  # Blank line before touch down
                    # Touch down - only process if we don't already have a stored touch
                    if self.touch_down_x is None:
                        touch_down_time = time.time()
                        dprint(f"[TIMING] Touch DOWN at {touch_down_time:.3f}")
                        dprint(f"[COORD] Raw touch coordinates: x={x}, y={y}")
                        dprint(f"[COORD] Screen is 320x480, buttons centered at y=350")
                        dprint(f"[COORD] If y=0 and you touched UP button, coordinates may be INVERTED")
                        # Debug: Compare with last touch event
                        if self.last_touch_x is not None:
                            dx = x - self.last_touch_x
                            dy = y - self.last_touch_y
                            distance = math.sqrt(dx*dx + dy*dy)
                            time_delta = time.time() - self.last_touch_time if self.last_touch_time else 0
                            dprint(f"[DEBUG] New touch: ({x}, {y})")
                            dprint(f"[DEBUG] Last touch: ({self.last_touch_x}, {self.last_touch_y}) [{self.last_button_pressed}]")
                            dprint(f"[DEBUG] Delta: dx={dx:+d} dy={dy:+d} dist={distance:.1f}px time={time_delta:.3f}s")
                        else:
                            dprint(f"[TOUCH] First touch at ({x}, {y})")
                        
                        self.touch_down_x = x
                        self.touch_down_y = y
                        
                        # Process button immediately on touch down
                        button = self.controller.check_button_press(x, y)
                        if button:
                            self.controller.handle_button_press(button)
                            
                            # Log button press
                            if button == 'stop':
                                self.log_message("BTN: STOP")
                                # Emergency stop - reset motor speeds immediately
                                self.target_left_speed = 0
                                self.target_right_speed = 0
                                self.current_left_speed = 0.0
                                self.current_right_speed = 0.0
                                self.left_speed = 0
                                self.right_speed = 0
                                if self.uart:
                                    try:
                                        self.uart.send_motor_cmd(0, 0, brake=1, enable=0)
                                    except:
                                        pass
                            else:
                                # Log arrow button press
                                fb = self.controller.forward_back_speed
                                lr = self.controller.left_right_speed
                                self.log_message(f"{button.upper()}:FB{fb:+.0f}LR{lr:+.0f}")
                            
                            # Store this touch for comparison with next touch
                            self.last_touch_x = x
                            self.last_touch_y = y
                            self.last_touch_time = time.time()
                            self.last_button_pressed = button
                        else:
                            dprint(f"[TOUCH] Down - no button at location ({x}, {y})")
                            # Clear the stored touch since no button was pressed
                            self.touch_down_x = None
                            self.touch_down_y = None
                    else:
                        dprint(f"[TOUCH] Down ignored - already have stored touch")
                else:
                    dprint()  # Blank line before release
                    # Touch release - just reset state
                    touch_up_time = time.time()
                    dprint(f"[TIMING] Touch RELEASE at {touch_up_time:.3f}")
                    dprint(f"[TOUCH] Release - resetting state")
                    self.touch_down_x = None
                    self.touch_down_y = None
            
            # Handle keyboard events (for ESC to quit)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                        running = False
            
            # Update logic every frame
            if frame_count % 5 == 0:
                self.update()
            
            # Draw only when controller changes or every 2 seconds for LCD updates
            should_draw = self.controller.changed or (frame_count % 60 == 0)
            if should_draw:
                self.draw()
                draw_count += 1
                self.controller.changed = False  # Reset change flag

            
            # Maintain framerate
            self.clock.tick(FPS)
        
        # Cleanup
        self.touch.close()
        if self.uart:
            # Stop motors before exit
            self.uart.send_motor_cmd(0, 0, brake=1, enable=0)
            self.uart.close()
        
        # Clear LCD
        if self.lcd:
            try:
                self.lcd.clear()
                self.lcd.write_string("Remote Control\nShutdown")
            except:
                pass
        
        # Close framebuffer
        if self.fbmmap:
            self.fbmmap.close()
        if self.fb:
            self.fb.close()
        
        pygame.quit()
        sys.exit()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Robot Remote Control UI")
    parser.add_argument("--debug", action="store_true", help="Enable verbose debug logging")
    args = parser.parse_args()
    DEBUG = args.debug
    
    app = RemoteControlUI()
    app.run()
