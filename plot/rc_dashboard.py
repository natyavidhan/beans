import sys
import threading
import time

import pygame
import serial
import serial.tools.list_ports

def find_serial_port():
    for p in serial.tools.list_ports.comports():
        return p.device
    return None

PORT = None
BAUD = 115200

# Constants for visual mapping
RC_MIN = 1000
RC_MAX = 2000
RC_MID = 1500

WIDTH = 1000
HEIGHT = 500
FPS_TARGET = 60

# Colors
BACKGROUND = (10, 15, 26)
CYAN = (0, 255, 220)
GREEN = (0, 255, 100)
DARK_GREY = (40, 45, 55)
DIM_GREEN = (0, 120, 60)
RED = (255, 50, 50)
WHITE = (255, 255, 255)

def serial_worker(state, lock, stop_event):
    ser = None
    while not stop_event.is_set():
        if ser is None:
            try:
                p = PORT or find_serial_port()
                if not p:
                    time.sleep(1)
                    continue
                ser = serial.Serial(p, BAUD, timeout=1)
                with lock:
                    state["serial_error"] = False
                    state["port_name"] = p
                    state["connected"] = True
            except Exception:
                with lock:
                    state["serial_error"] = True
                    state["connected"] = False
                stop_event.wait(0.5)
                continue

        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith("RC:"):
                # RC:1500,1500,1000,1500,1000,1500
                parts = line.replace("RC:", "").split(",")
                if len(parts) >= 6:
                    with lock:
                        for i in range(6):
                            state["channels"][i] = int(parts[i])
                        state["last_update"] = time.monotonic()
        except Exception:
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
            ser = None
            with lock:
                state["serial_error"] = True
                state["connected"] = False

    if ser is not None:
        try:
            ser.close()
        except Exception:
            pass

class RCDashboard:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.font_small = pygame.font.SysFont("consolas", 16)
        self.font_med = pygame.font.SysFont("consolas", 22, bold=True)
        self.font_large = pygame.font.SysFont("consolas", 30, bold=True)

        self.labels = ["AILERON", "ELEVATOR", "THROTTLE", "RUDDER", "AUX 1", "AUX 2"]

    def draw(self, screen, state):
        screen.fill(BACKGROUND)
        self.draw_bars(screen, state)
        self.draw_status(screen, state)

    def draw_bars(self, screen, state):
        # 6 Bars centered horizontally
        bar_w = 60
        bar_h = 300
        gap = 80
        total_w = 6 * bar_w + 5 * gap
        start_x = (self.width - total_w) // 2
        y = (self.height - bar_h) // 2

        for i in range(6):
            x = start_x + i * (bar_w + gap)
            val = state["channels"][i]

            # Background Box
            pygame.draw.rect(screen, DARK_GREY, (x - 4, y - 4, bar_w + 8, bar_h + 8), border_radius=6)
            pygame.draw.rect(screen, DIM_GREEN, (x, y, bar_w, bar_h), 2, border_radius=4)
            
            # Constraints
            val_clamped = max(RC_MIN, min(RC_MAX, val))
            
            # Map Value to Fill Height
            pct = (val_clamped - RC_MIN) / (RC_MAX - RC_MIN)
            fill_h = int(bar_h * pct)
            fill_y = y + bar_h - fill_h

            if fill_h > 0:
                pygame.draw.rect(screen, CYAN, (x + 2, fill_y + 2, bar_w - 4, fill_h - 4), border_radius=3)
            
            # Center line (1500us)
            center_y = y + bar_h // 2
            pygame.draw.line(screen, GREEN, (x - 10, center_y), (x + bar_w + 10, center_y), 2)

            # Draw labels
            lbl_surf = self.font_med.render(f"CH {i+1}", True, WHITE)
            screen.blit(lbl_surf, (x + (bar_w - lbl_surf.get_width()) // 2, y - 40))

            func_surf = self.font_small.render(self.labels[i], True, DIM_GREEN)
            screen.blit(func_surf, (x + (bar_w - func_surf.get_width()) // 2, y - 20))

            val_surf = self.font_large.render(str(val), True, CYAN)
            screen.blit(val_surf, (x + (bar_w - val_surf.get_width()) // 2, y + bar_h + 15))


    def draw_status(self, screen, state):
        status_text = f"PORT: {state['port_name']} | "
        if not state["connected"]:
            status_text += "DISCONNECTED"
            color = RED
        elif state["serial_error"]:
            status_text += "ERROR"
            color = RED
        elif time.monotonic() - state["last_update"] > 1.0:
            status_text += "NO DATA"
            color = RED
        else:
            status_text += "CONNECTED"
            color = GREEN
            
        status_surf = self.font_med.render(status_text, True, color)
        screen.blit(status_surf, (20, 20))


def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("RC Debug Dashboard")
    clock = pygame.time.Clock()

    state = {
        "connected": False,
        "serial_error": False,
        "port_name": "AUTO",
        "last_update": 0,
        "channels": [1500, 1500, 1000, 1500, 1000, 1500]  # Initial reasonable values
    }

    state_lock = threading.Lock()
    stop_event = threading.Event()

    serial_thread = threading.Thread(target=serial_worker, args=(state, state_lock, stop_event), daemon=True)
    serial_thread.start()

    dashboard = RCDashboard(WIDTH, HEIGHT)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        with state_lock:
            local_state = {
                "connected": state["connected"],
                "serial_error": state["serial_error"],
                "port_name": state["port_name"],
                "last_update": state["last_update"],
                "channels": list(state["channels"])
            }

        dashboard.draw(screen, local_state)
        pygame.display.flip()
        clock.tick(FPS_TARGET)

    stop_event.set()
    serial_thread.join(timeout=1.0)
    pygame.quit()
    sys.exit(0)

if __name__ == "__main__":
    main()
