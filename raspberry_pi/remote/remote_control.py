class UARTProtocol:
    """ESP UART protocol minimal stub."""
    SYNC = 0xAA
    END = 0x55

    def __init__(self, port='/dev/spidev0.1', baudrate=115200):
        self.lock = Lock()
        self.ser = None
        if SERIAL_AVAILABLE:
            try:
                self.ser = serial.Serial(port, baudrate, timeout=0.01)
            except Exception as e:
                print(f"Warning: Could not open UART: {e}")

    def crc8(self, data):
        crc = 0x00
        for b in data:
            crc ^= b
            for _ in range(8):
                crc = ((crc << 1) ^ 0x07) if (crc & 0x80) else (crc << 1)
        return crc & 0xFF

    def send_motor_cmd(self, left_speed, right_speed, brake=0, enable=1):
        if not self.ser:
            return
        payload = struct.pack('<hhBB', int(left_speed), int(right_speed), brake, enable)
        frame_hdr = bytes([self.SYNC, CMD_MOTOR, len(payload)])
        crc = self.crc8(list(frame_hdr[1:]) + list(payload))
        frame = frame_hdr + payload + bytes([crc, self.END])
        with self.lock:
            self.ser.write(frame)

    def close(self):
        if self.ser:
            self.ser.close()

    def receive_telemetry(self, ui=None):
        """Poll for telemetry message and update UI voltages/amps if found."""
        if not self.ser:
            return
        while self.ser.in_waiting >= 27:  # 4 floats, 1B, 1b, 1I + header
            # Read until SYNC byte
            b = self.ser.read(1)
            if b != bytes([self.SYNC]):
                continue
            header = self.ser.read(2)
            if len(header) < 2:
                return
            msg_type, length = header[0], header[1]
            if msg_type != 0x10 or length != 22:
                self.ser.read(length + 2)  # skip payload, crc, end
                continue
            payload = self.ser.read(22)
            crc = self.ser.read(1)
            end = self.ser.read(1)
            if len(payload) < 22 or end != bytes([self.END]):
                continue
            try:
                v1, v2, v3, amps, status, rssi, timestamp = struct.unpack('<ffffBbI', payload)
                if ui:
                    ui.v1 = v1
                    ui.v2 = v2
                    ui.v3 = v3
                    ui.amps = amps
                    ui.rssi = rssi
            except Exception as e:
                print(f"[DEBUG] Telemetry unpack error: {e}")
                continue
class ArrowController:
    """Directional button controller with incremental press behavior."""
    def __init__(self, center_x, center_y, size):
        self.cx = center_x
        self.cy = center_y
        self.size = size
        self.stop_radius = size // 3
        self.debounce_s = 0.15
        self.last_press_ts = 0
        # Percent speeds (-100 .. +100)
        self.fb = 0.0  # forward/back
        self.lr = 0.0  # left/right
        self.changed = False
        print("Button rects:", self.rects())
        print("Stop radius:", self.stop_radius)

    def rects(self):
        s = self.size
        return {
            'up':   (self.cx - s//2, self.cy - int(1.5*s), s, s),
            'down': (self.cx - s//2, self.cy + int(0.5*s), s, s),
            'left': (self.cx - int(1.5*s), self.cy - s//2, s, s),
            'right':(self.cx + int(0.5*s), self.cy - s//2, s, s),
            'stop': (self.cx, self.cy)
        }

    def check(self, x, y):
        now = time.time()
        print(f"Checking button for ({x},{y})")
        print(f"Debounce: {now - self.last_press_ts:.3f}s (threshold {self.debounce_s})")
        if now - self.last_press_ts < self.debounce_s:
            print("Debounced: Ignored")
            return None
        r = self.rects()
        # Stop circle first
        dx = x - r['stop'][0]
        dy = y - r['stop'][1]
        if math.hypot(dx, dy) <= self.stop_radius:
            print("Hit STOP")
            self.last_press_ts = now
            return 'stop'
        for name in ('up', 'down', 'left', 'right'):
            rx, ry, rw, rh = r[name]
            print(f"{name}: rect=({rx},{ry},{rw},{rh})")
            if rx <= x <= rx+rw and ry <= y <= ry+rh:
                print(f"Hit {name}")
                self.last_press_ts = now
                return name
        print("No button hit")
        return None

    def handle(self, btn):
        old_fb, old_lr = self.fb, self.lr
        # fb: forward/back, lr: left/right
        if btn == 'stop':
            self.fb = 0.0
            self.lr = 0.0
        elif btn == 'up':
            # Increase forward speed
            self.fb = min(100.0, self.fb + 25.0)
        elif btn == 'down':
            # Decrease forward speed (or go backward)
            self.fb = max(-100.0, self.fb - 25.0)
        elif btn == 'left':
            # Turn left: decrease left motor (lr negative)
            self.lr = max(-100.0, self.lr - 25.0)
        elif btn == 'right':
            # Turn right: increase right motor (lr positive)
            self.lr = min(100.0, self.lr + 25.0)

        self.changed = (abs(self.fb - old_fb) > 0.01) or (abs(self.lr - old_lr) > 0.01)

    def values(self):
        return self.lr/100.0, self.fb/100.0

    def draw(self, screen):
        r = self.rects()
        s = self.size

        # ...existing code...
        # LEFT
        left = r['left']
        pygame.draw.rect(screen, DARK_GRAY, left)
        pygame.draw.rect(screen, WHITE, left, 2)
        pygame.draw.polygon(screen, WHITE, [
            (left[0]+s//4, left[1]+s//2), (left[0]+3*s//4, left[1]+s//4), (left[0]+3*s//4, left[1]+3*s//4)
        ])
        # RIGHT
        right = r['right']
        pygame.draw.rect(screen, DARK_GRAY, right)
        pygame.draw.rect(screen, WHITE, right, 2)
        pygame.draw.polygon(screen, WHITE, [
            (right[0]+3*s//4, right[1]+s//2), (right[0]+s//4, right[1]+s//4), (right[0]+s//4, right[1]+3*s//4)
        ])
        # STOP
        stop = r['stop']
        pygame.draw.circle(screen, RED, stop, self.stop_radius)
        pygame.draw.circle(screen, WHITE, stop, self.stop_radius, 2)

import os
import sys
import time
import math
import struct
import select
import mmap
import argparse
from threading import Lock
from datetime import datetime

import pygame

try:
    import numpy as np
    NUMPY_AVAILABLE = True
except Exception as e:
    NUMPY_AVAILABLE = False
    print(f"✗ WARNING: NumPy not available: {e}")

try:
    from RPLCD.i2c import CharLCD
    LCD_AVAILABLE = True
except Exception:
    LCD_AVAILABLE = False
    print("Warning: RPLCD not installed. Install with: sudo pip3 install RPLCD")

try:
    import serial
    SERIAL_AVAILABLE = True
except Exception as e:
    SERIAL_AVAILABLE = False
    print(f"Warning: pySerial not available: {e}")

os.environ['SDL_VIDEODRIVER'] = 'dummy'

SCREEN_WIDTH = 320
SCREEN_HEIGHT = 480
FPS = 30

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 100, 255)
GRAY = (100, 100, 100)
DARK_GRAY = (50, 50, 50)
YELLOW = (255, 255, 0)

CMD_MOTOR = 0x01

DEBUG = False

def dprint(*args, **kwargs):
    if DEBUG:
        print(*args, **kwargs)


class UARTProtocol:
    """ESP UART protocol minimal stub."""
    SYNC = 0xAA
    END = 0x55

    def __init__(self, port='/dev/serial0', baudrate=115200):
        self.lock = Lock()
        self.ser = None
        if SERIAL_AVAILABLE:
            try:
                self.ser = serial.Serial(port, baudrate, timeout=0.01)
            except Exception as e:
                print(f"Warning: Could not open UART: {e}")

    def crc8(self, data):
        crc = 0x00
        for b in data:
            crc ^= b
            for _ in range(8):
                crc = ((crc << 1) ^ 0x07) if (crc & 0x80) else (crc << 1)
        return crc & 0xFF

    def send_motor_cmd(self, left_speed, right_speed, brake=0, enable=1):
        if not self.ser:
            return
        payload = struct.pack('<hhBB', int(left_speed), int(right_speed), brake, enable)
        frame_hdr = bytes([self.SYNC, CMD_MOTOR, len(payload)])
        crc = self.crc8(list(frame_hdr[1:]) + list(payload))
        frame = frame_hdr + payload + bytes([crc, self.END])
        with self.lock:
            self.ser.write(frame)

    def close(self):
        if self.ser:
            self.ser.close()


class TouchInput:
    # Non-blocking Linux input reader for touchscreen.
    def __init__(self):
        self.device = None
        self.device_path = None
        self.touch_x = 0
        self.touch_y = 0
        self.touching = False
        self.was_touching = False
        self.last_touch_start = 0
        self.touch_timeout = 0.5

        # Discover device
        preferred = []
        try:
            with open('/proc/bus/input/devices', 'r') as f:
                content = f.read()
            for block in content.split('\n\n'):
                if 'Goodix' in block or 'Touch' in block or 'touch' in block:
                    for line in block.split('\n'):
                        if line.startswith('H: Handlers='):
                            import re
                            m = re.search(r'event(\d+)', line)
                            if m:
                                preferred.append(f"/dev/input/event{m.group(1)}")
                                break
        except Exception:
            pass
        if not preferred:
            preferred = ['/dev/input/event0', '/dev/input/event1', '/dev/input/event2']

        import glob, fcntl
        for path in preferred + glob.glob('/dev/input/event*'):
            try:
                fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
                self.device = os.fdopen(fd, 'rb')
                self.device_path = path
                break
            except Exception:
                continue

    def read_event(self):
        if not self.device:
            return None

        # Timeout release
        if self.touching and (time.time() - self.last_touch_start) > self.touch_timeout:
            print(f"[DEBUG] Timeout release: x={self.touch_x} y={self.touch_y}")
            self.touching = False
            self.was_touching = False
            return (self.touch_x, self.touch_y, False)


        processed = 0
        ready = True
        while ready:
            processed += 1
            data = self.device.read(16)
            if data is None or len(data) < 16:
                return None
            try:
                sec, usec, ev_type, ev_code, ev_val = struct.unpack('llHHi', data)
            except Exception:
                extra = self.device.read(8)
                if len(extra) == 8:
                    sec, usec, ev_type, ev_code, ev_val = struct.unpack('QQHHi', data + extra)
                else:
                    return None

            print(f"[DEBUG] Raw event: type={ev_type} code={ev_code} val={ev_val} x={self.touch_x} y={self.touch_y} touching={self.touching}")

            if ev_type == 3:  # EV_ABS
                if ev_code in (0, 53):
                    self.touch_x = ev_val
                elif ev_code in (1, 54):
                    self.touch_y = ev_val
                elif ev_code == 57:  # MT tracking id
                    self.touching = ev_val >= 0
            elif ev_type == 1 and ev_code == 330:  # BTN_TOUCH
                self.touching = ev_val > 0
            elif ev_type == 0:  # EV_SYN
                print(f"[DEBUG] State change: touching={self.touching} was_touching={self.was_touching} x={self.touch_x} y={self.touch_y}")
                if self.touching and not self.was_touching:
                    self.was_touching = True
                    self.last_touch_start = time.time()
                    print(f"[DEBUG] Touch DOWN returned: x={self.touch_x} y={self.touch_y}")
                    return (self.touch_x, self.touch_y, True)
                if not self.touching and self.was_touching:
                    self.was_touching = False
                    print(f"[DEBUG] Touch UP returned: x={self.touch_x} y={self.touch_y}")
                    return (self.touch_x, self.touch_y, False)

        print(f"[DEBUG] No event returned after {processed} processed")
        return None

    def close(self):
        if self.device:
            self.device.close()

    def handle(self, btn):
        old_fb, old_lr = self.fb, self.lr
        if btn == 'stop':
            self.fb = 0.0
            self.lr = 0.0
        elif btn == 'up':
            # +25% per press up to +100%
            if self.fb < 0:
                self.fb = min(0.0, self.fb + 25.0)
            else:
                self.fb = min(100.0, self.fb + 25.0)
        elif btn == 'down':
            # -25% per press down to -100%
            if self.fb > 0:
                self.fb = max(0.0, self.fb - 25.0)
            else:
                self.fb = max(-100.0, self.fb - 25.0)
        elif btn == 'left':
            # Differential rule: before decrementing, climb toward +100% by +10%; then start decrementing by -10%
            if self.lr < 100.0:
                self.lr = min(100.0, self.lr + 10.0)
            else:
                self.lr = max(-100.0, self.lr - 10.0)
        elif btn == 'right':
            if self.lr < 100.0:
                self.lr = min(100.0, self.lr + 10.0)
            else:
                self.lr = max(-100.0, self.lr - 10.0)

        self.changed = (abs(self.fb - old_fb) > 0.01) or (abs(self.lr - old_lr) > 0.01)

    def values(self):
        return self.lr/100.0, self.fb/100.0

    def draw(self, screen):
        r = self.rects()
        s = self.size
        # UP
        up = r['up']
        pygame.draw.rect(screen, DARK_GRAY, up)
        pygame.draw.rect(screen, WHITE, up, 2)
        pygame.draw.polygon(screen, WHITE, [
            (up[0]+s//2, up[1]+s//4), (up[0]+s//4, up[1]+3*s//4), (up[0]+3*s//4, up[1]+3*s//4)
        ])
        # DOWN
        down = r['down']
        pygame.draw.rect(screen, DARK_GRAY, down)
        pygame.draw.rect(screen, WHITE, down, 2)
        pygame.draw.polygon(screen, WHITE, [
            (down[0]+s//2, down[1]+3*s//4), (down[0]+s//4, down[1]+s//4), (down[0]+3*s//4, down[1]+s//4)
        ])
        # LEFT
        left = r['left']
        pygame.draw.rect(screen, DARK_GRAY, left)
        pygame.draw.rect(screen, WHITE, left, 2)
        pygame.draw.polygon(screen, WHITE, [
            (left[0]+s//4, left[1]+s//2), (left[0]+3*s//4, left[1]+s//4), (left[0]+3*s//4, left[1]+3*s//4)
        ])
        # RIGHT
        right = r['right']
        pygame.draw.rect(screen, DARK_GRAY, right)
        pygame.draw.rect(screen, WHITE, right, 2)
        pygame.draw.polygon(screen, WHITE, [
            (right[0]+3*s//4, right[1]+s//2), (right[0]+s//4, right[1]+s//4), (right[0]+s//4, right[1]+3*s//4)
        ])
        # STOP
        stop = r['stop']
        pygame.draw.circle(screen, RED, stop, self.stop_radius)
        pygame.draw.circle(screen, WHITE, stop, self.stop_radius, 2)


class RemoteControlUI:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("Robot Remote Control")
        self.clock = pygame.time.Clock()
        self.font_large = pygame.font.Font(None, 36)
        self.font_small = pygame.font.Font(None, 24)

        # LCD
        self.lcd = None
        if LCD_AVAILABLE:
            try:
                self.lcd = CharLCD('PCF8574', 0x27, cols=20, rows=4)
                self.lcd.clear()
                self.lcd.write_string("Remote Control v1.0")
            except Exception as e:
                print(f"Warning: LCD init failed: {e}")
                self.lcd = None

        # Framebuffer
        self.fb = None
        self.fbmmap = None
        try:
            self.fb = open('/dev/fb0', 'rb+')
            self.fbmmap = mmap.mmap(self.fb.fileno(), SCREEN_WIDTH*SCREEN_HEIGHT*2)
            self.rgb565_buffer = bytearray(SCREEN_WIDTH*SCREEN_HEIGHT*2)
        except Exception as e:
            print(f"Warning: framebuffer not available: {e}")

        # IO
        self.touch = TouchInput()
        self.uart = UARTProtocol()

        # Controller
        self.controller = ArrowController(SCREEN_WIDTH//2, 350, 60)

        # Telemetry + status
        self.connection_status = "Connecting..."
        self.rssi = -90
        self.left_speed = 0
        self.right_speed = 0
        self.target_left = 0.0
        self.target_right = 0.0
        self.current_left = 0.0
        self.current_right = 0.0
        self.ramp_time = 2.0  # seconds to reach target
        self.last_update = time.time()
        self.last_tx = 0
        self.tx_interval = 0.05

        self.messages = ["", "", "", ""]

    def log(self, msg):
        msg = msg[:20]
        self.messages = [msg] + self.messages[:3]
        if self.lcd:
            try:
                for i in range(4):
                    self.lcd.cursor_pos = (i, 0)
                    self.lcd.write_string(f"{self.messages[i]:<20}")
            except Exception:
                pass

    def differential_steering(self, x, y):
        # Differential drive steering logic:
        # x: left/right (-1.0 to +1.0), y: forward/back (-1.0 to +1.0)
        # Go straight: both motors same speed
        # Turn left: left slower, right faster
        # Turn right: right slower, left faster
        # Spin: left forward, right backward
        left = y + x
        right = y - x
        m = max(abs(left), abs(right))
        if m > 1.0:
            left /= m
            right /= m
        return int(left*1000), int(right*1000)

    def update(self):
        # Poll for telemetry and update voltages/amps
    #    self.uart.receive_telemetry(self)
        x, y = self.controller.values()
        self.target_left, self.target_right = self.differential_steering(x, y)
        now = time.time()
        dt = now - self.last_update
        self.last_update = now
        max_per_sec = 1000.0 / self.ramp_time
        max_change = max_per_sec * dt
        # left
        dl = self.target_left - self.current_left
        if abs(dl) <= max_change:
            self.current_left = self.target_left
        else:
            self.current_left += max_change if dl > 0 else -max_change
        # right
        dr = self.target_right - self.current_right
        if abs(dr) <= max_change:
            self.current_right = self.target_right
        else:
            self.current_right += max_change if dr > 0 else -max_change
        # display
        self.left_speed = int(self.current_left)
        self.right_speed = int(self.current_right)

        # TX
        if now - self.last_tx >= self.tx_interval:
            try:
                self.uart.send_motor_cmd(self.left_speed, self.right_speed, brake=0, enable=1)
                self.connection_status = "Connected"
                self.log(f"TX:L{self.left_speed:+4d}R{self.right_speed:+4d}")
            except Exception as e:
                self.connection_status = f"Error: {e}"
                self.log(f"ERR: {str(e)[:15]}")
            self.last_tx = now

    def draw(self):
        self.screen.fill(BLACK)
        title = self.font_large.render("Robot Control", True, WHITE)
        self.screen.blit(title, (SCREEN_WIDTH//2 - title.get_width()//2, 10))
        is_conn = "Connected" in self.connection_status
        status_color = GREEN if is_conn else RED
        status = self.font_small.render(self.connection_status, True, status_color)
        self.screen.blit(status, (SCREEN_WIDTH//2 - status.get_width()//2, 50))
        if not is_conn:
            waiting = self.font_small.render("Waiting for connection...", True, GRAY)
            self.screen.blit(waiting, (SCREEN_WIDTH//2 - waiting.get_width()//2, 70))

        pygame.draw.rect(self.screen, DARK_GRAY, (10, 90, SCREEN_WIDTH-20, 120), 2)
        tele_color = WHITE if is_conn else DARK_GRAY
        rssi_color = (GREEN if self.rssi > -70 else (YELLOW if self.rssi > -85 else RED)) if is_conn else DARK_GRAY
        bt = self.font_small.render(f"Signal: {self.rssi} dBm", True, rssi_color)
        self.screen.blit(bt, (20, 100))
        lt = self.font_small.render(f"L: {self.left_speed:+5d}", True, tele_color)
        rt = self.font_small.render(f"R: {self.right_speed:+5d}", True, tele_color)
        self.screen.blit(lt, (20, 130))
        self.screen.blit(rt, (SCREEN_WIDTH - 120, 130))

        # Display voltages and amps if connected
        if is_conn:
            # These should be updated from telemetry messages
            v1 = getattr(self, 'v1', 0.0)
            v2 = getattr(self, 'v2', 0.0)
            v3 = getattr(self, 'v3', 0.0)
            amps = getattr(self, 'amps', 0.0)
            # Display voltages on one line with labels '5V:', '5V:', '12V:'
            voltages_text = f"5V: {v1:.2f}V   5V: {v2:.2f}V   12V: {v3:.2f}V"
            voltages_render = self.font_small.render(voltages_text, True, tele_color)
            self.screen.blit(voltages_render, (20, 160))
            # Display amps below voltages
            at = self.font_small.render(f"Amps: {amps:.2f}A", True, tele_color)
            self.screen.blit(at, (20, 185))

        if is_conn:
            self.controller.draw(self.screen)
            inst = self.font_small.render("Touch to control", True, GRAY)
            self.screen.blit(inst, (SCREEN_WIDTH//2 - inst.get_width()//2, SCREEN_HEIGHT - 30))

        # Framebuffer copy
        if self.fbmmap:
            rgb = pygame.image.tostring(self.screen, 'RGB')
            if NUMPY_AVAILABLE:
                arr = np.frombuffer(rgb, dtype=np.uint8).reshape(-1, 3)
                rgb565 = ((arr[:,0].astype(np.uint16)>>3)<<11) | ((arr[:,1].astype(np.uint16)>>2)<<5) | (arr[:,2].astype(np.uint16)>>3)
                self.fbmmap.seek(0)
                self.fbmmap.write(rgb565.astype(np.uint16).tobytes())
            else:
                for i in range(0, len(rgb), 3):
                    r = rgb[i] >> 3
                    g = rgb[i+1] >> 2
                    b = rgb[i+2] >> 3
                    val = (r<<11) | (g<<5) | b
                    idx = (i//3) * 2
                    self.rgb565_buffer[idx] = val & 0xFF
                    self.rgb565_buffer[idx+1] = (val>>8) & 0xFF
                self.fbmmap.seek(0)
                self.fbmmap.write(self.rgb565_buffer)

        pygame.display.flip()

    def run(self):
        running = True
        self.draw()
        self.log("Ready")
        while running:
            # Touch
            ev = self.touch.read_event()
            if ev:
                x, y, touching = ev
                if touching and ("Connected" in self.connection_status):
                    btn = self.controller.check(x, y)
                    fb_before, lr_before = self.controller.fb, self.controller.lr
                    if btn:
                        self.controller.handle(btn)
                        fb_after, lr_after = self.controller.fb, self.controller.lr
                        msg = f"Touch: ({x},{y}) Btn: {btn.upper()} FB:{fb_before:.0f}->{fb_after:.0f} LR:{lr_before:.0f}->{lr_after:.0f}"
                        self.log(msg)
                        if DEBUG:
                            print(msg)
                    else:
                        msg = f"Touch: ({x},{y}) Btn: NONE"
                        self.log(msg)
                        if DEBUG:
                            print(msg)
                elif not touching:
                    # Reset any held-touch state if needed (controller has debounce)
                    pass

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN and (event.key in (pygame.K_ESCAPE, pygame.K_q)):
                    running = False

            self.update()
            self.draw()
            self.clock.tick(FPS)

        # Cleanup
        self.touch.close()
        self.uart.close()
        if self.fbmmap:
            self.fbmmap.close()
        if self.fb:
            self.fb.close()
        pygame.quit()


APP_VERSION = f"0.001-{datetime.now().strftime('%Y%m%d-%H%M%S')}"
APP_ROLE = "remote"

if __name__ == "__main__":
    print(f"App Version: {APP_VERSION} | Role: {APP_ROLE}")
    parser = argparse.ArgumentParser(description='New Remote Control UI')
    parser.add_argument('--debug', action='store_true', help='Enable verbose debug logging')
    args = parser.parse_args()
    DEBUG = args.debug

    app = RemoteControlUI()
    app.run()
