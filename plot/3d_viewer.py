import serial
import serial.tools.list_ports
import time
import math
from vpython import *
import threading

def find_serial_port():
    print("Searching for COM ports...")
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found!")
        return None
    for p in ports:
        print(f"Found: {p.device} - {p.description}")
        # Default to first one or adjust here
        return p.device
    return None

PORT = "COM7"

if not PORT:
    print("Please connect your Arduino/ESP32 and try again.")
    exit()

print(f"Connecting to {PORT} at 115200 baud...")
try:
    ser = serial.Serial(PORT, 115200, timeout=1)
except Exception as e:
    print(f"Failed to connect: {e}")
    exit()

# ═══════════════════════════════════════════════════
# VPYTHON 3D SCENE SETUP
# ═══════════════════════════════════════════════════

scene.title = "<b>Drone Flight Controller - 3D Orientation</b>"
scene.width = 800
scene.height = 600
scene.background = color.gray(0.1)

# Drone chassis representing real physical shape
# X = Roll axis, Y = Pitch axis, Z = Yaw axis in VPython world
drone = box(length=6, width=4, height=0.5, color=color.cyan, opacity=0.8)

# Rotors / Motors
m_fl = cylinder(pos=vector(3, 0.25, -2), axis=vector(0, 0.5, 0), radius=0.8, color=color.red)
m_fr = cylinder(pos=vector(3, 0.25, 2), axis=vector(0, 0.5, 0), radius=0.8, color=color.red)
m_rl = cylinder(pos=vector(-3, 0.25, -2), axis=vector(0, 0.5, 0), radius=0.8, color=color.green)
m_rr = cylinder(pos=vector(-3, 0.25, 2), axis=vector(0, 0.5, 0), radius=0.8, color=color.green)

# Direction Labels attached to the chassis
# We group them together so we can rotate the whole assembly easily
board = compound([drone, m_fl, m_fr, m_rl, m_rr])

# Create floating text labels
label_front = label(pos=vector(4, 0, 0), text='FRONT', box=False, color=color.red, xoffset=20, linecolor=color.red)
label_back = label(pos=vector(-4, 0, 0), text='BACK', box=False, color=color.green, xoffset=-20, linecolor=color.green)
label_left = label(pos=vector(0, 0, -3), text='LEFT', box=False, color=color.white, yoffset=20, linecolor=color.white)
label_right = label(pos=vector(0, 0, 3), text='RIGHT', box=False, color=color.white, yoffset=-20, linecolor=color.white)

# Coordinate Axes
arrow(pos=vector(0,0,0), axis=vector(5,0,0), color=color.red, shaftwidth=0.1)
arrow(pos=vector(0,0,0), axis=vector(0,5,0), color=color.green, shaftwidth=0.1)
arrow(pos=vector(0,0,0), axis=vector(0,0,5), color=color.blue, shaftwidth=0.1)
label(pos=vector(5,0,0), text='Roll Axis (X)', box=False, height=10)
label(pos=vector(0,5,0), text='Yaw Axis (Y)', box=False, height=10)
label(pos=vector(0,0,5), text='Pitch Axis (Z)', box=False, height=10)

print("Waiting for IMU Calibration to finish on the board...")

# ═══════════════════════════════════════════════════
# SERIAL LOOP
# ═══════════════════════════════════════════════════

roll = 0.0
pitch = 0.0
yaw = 0.0

def update_graphics():
    global roll, pitch, yaw
    
    # We calculate the up and forward vectors based on roll, pitch, yaw.
    # Vpython coordination system:
    # y is UP
    # x is RIGHT
    # z is OUT OF SCREEN (towards you)
    
    # Let's map hardware (angleX=Roll, angleY=Pitch, angleZ=Yaw) into Vpython
    k = vector(cos(yaw)*cos(pitch), sin(pitch), sin(yaw)*cos(pitch))
    y = vector(-cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll),
                cos(pitch)*sin(roll),
               -sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll))
               
    # Apply to compound object
    board.axis = k
    board.up = y
    
    # Update labels to follow the boards edges
    label_front.pos = board.pos + board.axis * 3.5
    label_back.pos = board.pos - board.axis * 3.5
    
    # Calculate right vector using cross product
    right_vec = cross(board.axis, board.up).norm()
    label_right.pos = board.pos + right_vec * 2.5
    label_left.pos = board.pos - right_vec * 2.5

while True:
    try:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8').strip()
            
            if line.startswith("DATA:"):
                parts = line.replace("DATA:", "").split(",")
                if len(parts) == 3:
                    # Convert degrees to radians for VPython
                    roll_deg = float(parts[0])
                    pitch_deg = float(parts[1])
                    yaw_deg = float(parts[2])
                    
                    roll = math.radians(roll_deg)
                    pitch = math.radians(pitch_deg)
                    yaw = math.radians(yaw_deg)
                    
        # Update 60 times a second
        rate(60)
        update_graphics()
        
    except KeyboardInterrupt:
        print("Closing...")
        break
    except Exception as e:
        pass
        
ser.close()
