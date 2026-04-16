import math
import time

import serial
from vpython import box, canvas, color, label, vector, rate

# Change this to your port.
PORT = "COM7"  # Windows (check Device Manager)
# PORT = "/dev/ttyUSB0"  # Linux
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

scene = canvas(
    title="Real-Time 3D Motion",
    width=1100,
    height=700,
    center=vector(0, 0, 0),
    background=vector(0.08, 0.08, 0.10),
)

body_length = 2.2
sensor_box = box(
    pos=vector(0, 0, 0),
    size=vector(body_length, 0.7, 1.2),
    axis=vector(body_length, 0, 0),
    up=vector(0, 0, 1),
    color=vector(0.2, 0.75, 1.0),
    opacity=0.85,
)

info = label(
    pos=vector(0, -2.8, 0),
    text="Waiting for data...",
    box=False,
    height=14,
    color=color.white,
)


def parse_packet(raw_line: str):
    values = [v.strip() for v in raw_line.split(",") if v.strip() != ""]
    if len(values) < 6:
        return None

    try:
        angle_x, angle_y, acc_x, acc_y, gyro_x, gyro_y = map(float, values[:6])
    except ValueError:
        return None

    if len(values) >= 7:
        try:
            altitude = float(values[6])
        except ValueError:
            altitude = math.nan
    else:
        altitude = math.nan

    return angle_x, angle_y, acc_x, acc_y, gyro_x, gyro_y, altitude


def orientation_vectors(roll_deg: float, pitch_deg: float, yaw_deg: float):
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cx, sx = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw), math.sin(yaw)

    # Rotation matrix R = Rz(yaw) * Ry(pitch) * Rx(roll)
    r00 = cz * cy
    r01 = cz * sy * sx - sz * cx
    r02 = cz * sy * cx + sz * sx

    r10 = sz * cy
    r11 = sz * sy * sx + cz * cx
    r12 = sz * sy * cx - cz * sx

    r20 = -sy
    r21 = cy * sx
    r22 = cy * cx

    forward = vector(r00, r10, r20)
    up = vector(r02, r12, r22)
    return forward, up


yaw_deg = 0.0
last_t = time.time()

# Skip possible header line.
ser.readline()

try:
    while True:
        rate(120)
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue

        packet = parse_packet(line)
        if packet is None:
            continue

        angle_x, angle_y, acc_x, acc_y, gyro_x, gyro_y, altitude = packet

        now = time.time()
        dt = max(now - last_t, 1e-3)
        last_t = now

        # Integrate gyro Y as a basic yaw estimate.
        yaw_deg += gyro_y * dt

        forward, up = orientation_vectors(angle_x, angle_y, yaw_deg)
        sensor_box.axis = forward.hat * body_length
        sensor_box.up = up.hat

        z_target = 0.0 if math.isnan(altitude) else altitude * 0.08
        target_pos = vector(acc_x * 0.12, acc_y * 0.12, z_target)
        sensor_box.pos = sensor_box.pos * 0.90 + target_pos * 0.10

        info.text = (
            f"AngleX: {angle_x:6.2f} deg   AngleY: {angle_y:6.2f} deg   Yaw: {yaw_deg:6.2f} deg\n"
            f"AccX:   {acc_x:6.2f} m/s^2  AccY:   {acc_y:6.2f} m/s^2\n"
            f"GyroX:  {gyro_x:6.2f} deg/s  GyroY:  {gyro_y:6.2f} deg/s   Alt: {altitude}"
        )
except KeyboardInterrupt:
    pass
finally:
    ser.close()