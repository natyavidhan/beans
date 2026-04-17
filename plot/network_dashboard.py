import math
import threading
import time
import socket
import json
import pygame
import sys

# ════════════════════════════════════════════
# Configuration - UPDATE THIS IP ADDRESS!
# ════════════════════════════════════════════
ESP_IP = "192.168.1.46"  # Replace with the IP shown in Serial Monitor
TCP_PORT = 8080

WIDTH = 1280
HEIGHT = 720
FPS_TARGET = 60

BACKGROUND = (10, 15, 26)
CYAN = (0, 255, 220)
GREEN = (0, 255, 100)
AMBER = (255, 180, 0)
RED = (255, 50, 50)
SKY_BLUE = (20, 60, 120)
GROUND_BROWN = (80, 50, 20)
WHITE = (255, 255, 255)
DARK_GREY = (40, 45, 55)
DIM_GREEN = (0, 120, 60)

def clamp(value, low, high):
    return max(low, min(high, value))

# ════════════════════════════════════════════
# TCP Worker Thread
# ════════════════════════════════════════════
def tcp_worker(state, lock, stop_event, sock_container):
    while not stop_event.is_set():
        if sock_container[0] is None:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(2.0)
                s.connect((ESP_IP, TCP_PORT))
                s.settimeout(None) # Blocking reads
                sock_container[0] = s
                with lock:
                    state["connected"] = True
            except Exception:
                with lock:
                    state["connected"] = False
                time.sleep(1)
                continue

        try:
            f = sock_container[0].makefile('r')
            while not stop_event.is_set():
                line = f.readline()
                if not line:
                    raise Exception("Disconnected")
                
                try:
                    d = json.loads(line)
                    with lock:
                        state["armed"] = d.get("armed", False)
                        state["rateMode"] = d.get("rateMode", False)
                        state["signal"] = d.get("signal", False)
                        state["altitude"] = d.get("altitude", 0)
                        
                        r, p, y = d.get("roll",0), d.get("pitch",0), d.get("yaw",0)
                        state["raw"]["roll"] = r
                        state["raw"]["pitch"] = p
                        state["raw"]["yaw"] = y
                        
                        state["aileron"] = clamp(r / 45.0, -1.0, 1.0)
                        state["elevator"] = clamp(p / 40.0, -1.0, 1.0)
                        state["rudder"] = clamp((y % 360) / 180.0 - 1.0, -1.0, 1.0)
                        state["heading"] = y % 360.0
                        
                        if "rcRaw" in d: state["rc"] = d["rcRaw"]
                        if "motorUS" in d: state["motors"] = d["motorUS"]
                        
                        # Only update PID if the user isn't currently typing
                        if "pid" in d and state["pid_synced"] == False:
                            state["pids"] = d["pid"]
                            state["pid_synced"] = True

                        state["last_update"] = time.monotonic()
                except Exception:
                    pass
        except Exception:
            if sock_container[0]:
                sock_container[0].close()
                sock_container[0] = None
            with lock:
                state["connected"] = False
            time.sleep(1)

# ════════════════════════════════════════════
# Text Input Class
# ════════════════════════════════════════════
class TextBox:
    def __init__(self, key, text, x, y, w, h):
        self.key = key
        self.text = str(text)
        self.rect = pygame.Rect(x, y, w, h)
        self.active = False
        
    def draw(self, screen, font):
        color = CYAN if self.active else DARK_GREY
        pygame.draw.rect(screen, color, self.rect, 2, border_radius=4)
        txt = font.render(self.text, True, CYAN if self.active else WHITE)
        screen.blit(txt, (self.rect.x + 5, self.rect.y + 6))

class NetworkDashboard:
    def __init__(self, w, h):
        self.w, self.h = w, h
        self.font_s = pygame.font.SysFont("consolas", 14)
        self.font_m = pygame.font.SysFont("consolas", 20, bold=True)
        self.font_l = pygame.font.SysFont("consolas", 28, bold=True)
        
        # PID boxes layout
        self.boxes = []
        start_x = 760
        start_y = 200
        dx = 70
        dy = 35
        
        # Generate Grid mapping
        keys = ['sR_p','sR_i','sR_d','sP_p','sP_i','sP_d','sY_p','sY_i','sY_d',
                'rR_p','rR_i','rR_d','rP_p','rP_i','rP_d','rY_p','rY_i','rY_d']
        
        idx = 0
        for mode in range(2): # 0=Stab, 1=Rate
            bx = start_x + (mode * 260)
            for row in range(3): # R, P, Y
                for col in range(3): # P, I, D
                    self.boxes.append(TextBox(keys[idx], "0.0", bx + (col*dx), start_y + (row*dy), 60, 26))
                    idx += 1
                    
        self.save_rect = pygame.Rect(start_x, start_y + 140, 200, 40)
        self.save_text = "SYNC / SAVE ALL TO DRONE"
        self.save_timer = 0
        
    def draw(self, screen, state, lock):
        screen.fill(BACKGROUND)
        self.draw_horizon(screen, state)
        self.draw_throttle(screen, state)
        self.draw_info(screen, state, lock)
        self.draw_pids(screen, state, lock)
        self.draw_bars(screen, state)
        self.draw_overlay(screen, state)
        
    def draw_horizon(self, screen, state):
        center = (320, self.h // 2)
        radius = 240
        size = radius * 2
        roll_deg = state["raw"]["roll"]
        pitch_offset = state["raw"]["pitch"]

        base = pygame.Surface((size, size), pygame.SRCALPHA)
        horizon_y = radius + (pitch_offset * (radius / 90.0))
        pygame.draw.rect(base, SKY_BLUE, (0, -size, size, horizon_y + size))
        pygame.draw.rect(base, GROUND_BROWN, (0, horizon_y, size, size))
        pygame.draw.aaline(base, WHITE, (0, horizon_y), (size, horizon_y))
        
        for deg in range(-90, 95, 5):
            y = horizon_y - deg * (radius / 90.0)
            if y < 0 or y > size: continue
            len_ = 90 if deg % 10 == 0 else 45
            pygame.draw.aaline(base, CYAN, (radius - len_, y), (radius + len_, y))
            if deg % 10 == 0 and deg != 0:
                txt = self.font_m.render(f"{abs(deg)}", True, CYAN)
                base.blit(txt, (radius + len_ + 8, y - 8))
                base.blit(txt, (radius - len_ - txt.get_width() - 8, y - 8))

        rotated = pygame.transform.rotozoom(base, roll_deg, 1.0)
        horizon = pygame.Surface((size, size), pygame.SRCALPHA)
        horizon.blit(rotated, rotated.get_rect(center=(radius, radius)))
        
        mask = pygame.Surface((size, size), pygame.SRCALPHA)
        pygame.draw.circle(mask, WHITE, (radius, radius), radius)
        horizon.blit(mask, (0, 0), special_flags=pygame.BLEND_RGBA_MULT)
        
        screen.blit(horizon, (center[0] - radius, center[1] - radius))
        pygame.draw.circle(screen, DARK_GREY, center, radius + 4, 8)
        pygame.draw.circle(screen, DIM_GREEN, center, radius, 2)
        
        # Aircraft symbol
        pygame.draw.line(screen, CYAN, (center[0] - 56, center[1]), (center[0] - 14, center[1]), 4)
        pygame.draw.line(screen, CYAN, (center[0] + 14, center[1]), (center[0] + 56, center[1]), 4)
        pygame.draw.circle(screen, CYAN, center, 4)
        
        # Heading marker
        hdg_txt = self.font_l.render(f"HDG {state['heading']:05.1f}*", True, AMBER)
        screen.blit(hdg_txt, (center[0] - hdg_txt.get_width()//2, center[1] + radius + 10))

    def draw_throttle(self, screen, state):
        x, y, w, h = 30, 200, 36, 320
        pygame.draw.rect(screen, DARK_GREY, (x - 2, y - 2, w + 4, h + 4), border_radius=4)
        pygame.draw.rect(screen, DIM_GREEN, (x, y, w, h), 2)
        
        # approximate throttle channel (CH3) mapping
        val = state["rc"][2] if len(state["rc"])>2 else 1000
        pct = clamp((val - 1000) / 1000.0, 0, 1)
        fill_h = int(h * pct)
        if fill_h > 0:
            pygame.draw.rect(screen, GREEN, (x+3, y+h-fill_h, w-6, fill_h))
            
        screen.blit(self.font_m.render("THR", True, CYAN), (x-4, y-26))
        
    def draw_bars(self, screen, state):
        """ Draws RC and Motor bars on the far right """
        x = 1050
        y = 120
        rc_labels = ["AIL", "ELE", "THR", "RUD", "AUX1", "AUX2"]
        
        screen.blit(self.font_l.render("RECEIVER (µs)", True, CYAN), (x, y))
        for i, val in enumerate(state["rc"]):
            pygame.draw.rect(screen, DARK_GREY, (x, y + 40 + i*30, 180, 18))
            pct = clamp((val - 1000)/1000.0, 0, 1)
            pygame.draw.rect(screen, GREEN, (x, y + 40 + i*30, int(180*pct), 18))
            txt = self.font_s.render(f"{rc_labels[i]}: {val}", True, WHITE)
            screen.blit(txt, (x + 5, y + 42 + i*30))
            
        my = y + 250
        screen.blit(self.font_l.render("MOTORS (µs)", True, AMBER), (x, my))
        m_lbls = ["FL", "FR", "RL", "RR"]
        for i, val in enumerate(state["motors"]):
            pygame.draw.rect(screen, DARK_GREY, (x, my + 40 + i*30, 180, 18))
            pct = clamp((val - 1000)/1000.0, 0, 1)
            pygame.draw.rect(screen, AMBER, (x, my + 40 + i*30, int(180*pct), 18))
            txt = self.font_s.render(f"{m_lbls[i]}: {val}", True, WHITE)
            screen.blit(txt, (x + 5, my + 42 + i*30))
            
    def draw_info(self, screen, state, lock):
        # Top-center statuses
        status_x = 640
        pygame.draw.rect(screen, DARK_GREY, (status_x, 40, 600, 50), border_radius=8)
        
        # Signal
        sig_c = GREEN if state["signal"] else RED
        sig_t = "SIGNAL: OK" if state["signal"] else "SIGNAL: LOST"
        screen.blit(self.font_l.render(sig_t, True, sig_c), (status_x + 20, 52))
        
        # Armed
        arm_c = RED if not state["armed"] else GREEN
        arm_t = "ARMED" if state["armed"] else "DISARMED"
        screen.blit(self.font_l.render(arm_t, True, arm_c), (status_x + 240, 52))
        
        # Mode
        mode_c = CYAN if state["rateMode"] else AMBER
        mode_t = "MODE: RATE" if state["rateMode"] else "MODE: STAB"
        screen.blit(self.font_l.render(mode_t, True, mode_c), (status_x + 400, 52))

    def draw_pids(self, screen, state, lock):
        screen.blit(self.font_l.render("LIVE LIVE TUNING", True, CYAN), (640, 130))
        
        screen.blit(self.font_m.render("STABILIZE (Angle)", True, WHITE), (760, 170))
        screen.blit(self.font_m.render("RATE (Acro)", True, WHITE), (1020, 170))
        
        lbls = ["R", "P", "Y"]
        screen.blit(self.font_s.render("P        I        D", True, CYAN), (765, 190))
        
        for i, l in enumerate(lbls):
             screen.blit(self.font_m.render(l, True, WHITE), (730, 203 + i*35))
             
        # Copy backend values to frontend boxes if we just synced
        with lock:
            if state["pid_synced"]:
                # Only overwrite non-active boxes so typing isn't interrupted
                for b in self.boxes:
                    if not b.active and b.key in state["pids"]:
                        # Small formatter
                        val = state["pids"][b.key]
                        b.text = f"{val:.2f}"
                        
        for b in self.boxes:
            b.draw(screen, self.font_m)
            
        # Draw save button
        color = GREEN if time.time() - self.save_timer > 1.0 else CYAN
        pygame.draw.rect(screen, color, self.save_rect, border_radius=6)
        txt = self.font_m.render(self.save_text, True, BACKGROUND)
        screen.blit(txt, (self.save_rect.x + 10, self.save_rect.y + 10))

    def draw_overlay(self, screen, state):
        if not state["connected"]:
            title = self.font_l.render(f"CONNECTING TCP -> {ESP_IP}:{TCP_PORT} ...", True, RED)
            screen.blit(title, (10, 10))
            return
            
        fps = self.font_s.render(f"HUD FPS {state['fps']:.1f}", True, WHITE)
        screen.blit(fps, (10, 10))

def main():
    if len(sys.argv) > 1:
        global ESP_IP
        ESP_IP = sys.argv[1]

    pygame.init()
    pygame.display.set_caption("TCP Raw Flight Cockpit")
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    clock = pygame.time.Clock()

    state = {
        "connected": False,
        "armed": False,
        "rateMode": False,
        "signal": False,
        "altitude": 0.0,
        "raw": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        "heading": 0.0,
        "rc": [1000]*6,
        "motors": [1000]*4,
        "pid_synced": False,
        "pids": {},
        "fps": 0.0,
        "last_update": 0.0
    }

    lock = threading.Lock()
    stop_event = threading.Event()
    sock_container = [None]
    
    thread = threading.Thread(target=tcp_worker, args=(state, lock, stop_event, sock_container), daemon=True)
    thread.start()

    dash = NetworkDashboard(WIDTH, HEIGHT)
    running = True

    while running:
        clock.tick(FPS_TARGET)
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                
            elif event.type == pygame.MOUSEBUTTONDOWN:
                # Check text boxes
                for b in dash.boxes:
                    if b.rect.collidepoint(event.pos):
                        b.active = True
                    else:
                        b.active = False
                        
                # Check SAVE button
                if dash.save_rect.collidepoint(event.pos):
                    # Assemble PIDs
                    payload = {"pid": {}}
                    for b in dash.boxes:
                        try:
                            payload["pid"][b.key] = float(b.text)
                        except ValueError: pass
                    
                    if sock_container[0]:
                        try:
                            # Send massive UDP/TCP blast to update values
                            sock_container[0].sendall((json.dumps(payload) + "\n").encode('utf-8'))
                            dash.save_timer = time.time()
                            dash.save_text = "PIDS SENT!"
                        except Exception as e:
                            print("Send fail:", e)
                            
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                else:
                    for b in dash.boxes:
                        if b.active:
                            if event.key == pygame.K_BACKSPACE:
                                b.text = b.text[:-1]
                            elif event.unicode in "0123456789.-":
                                b.text += event.unicode
                                
        if time.time() - dash.save_timer > 2.0:
            dash.save_text = "SYNC / SAVE ALL TO DRONE"

        with lock:
            state["fps"] = clock.get_fps()
            render_state = dict(state)
            render_state["raw"] = dict(state["raw"])
            render_state["rc"] = list(state["rc"])
            render_state["motors"] = list(state["motors"])
            
        dash.draw(screen, render_state, lock)
        pygame.display.flip()

    stop_event.set()
    if sock_container[0]: sock_container[0].close()
    thread.join(timeout=1.0)
    pygame.quit()

if __name__ == "__main__":
    main()
