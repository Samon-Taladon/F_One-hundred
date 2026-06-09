import smbus2 as smbus
import time
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

# ==========================================
# ICM20948 SETUP
# ==========================================
bus = smbus.SMBus(7)
IMU_ADDR = 0x68

def select_bank(bank):
    bus.write_byte_data(IMU_ADDR, 0x7F, bank << 4)

# ==========================================
# INITIALIZE IMU
# ==========================================
print("Initializing IMU...")

select_bank(0)

# Reset
bus.write_byte_data(IMU_ADDR, 0x06, 0x80)
time.sleep(0.1)

# Wake up
bus.write_byte_data(IMU_ADDR, 0x06, 0x01)
bus.write_byte_data(IMU_ADDR, 0x07, 0x00)

# Configure sensor
select_bank(2)

# Gyro ±500 dps
bus.write_byte_data(IMU_ADDR, 0x01, 0x04)

# Accel ±4g
bus.write_byte_data(IMU_ADDR, 0x14, 0x04)

select_bank(0)

print("IMU Ready!")

# ==========================================
# READ FUNCTIONS
# ==========================================
def read_word(reg):

    high = bus.read_byte_data(IMU_ADDR, reg)
    low  = bus.read_byte_data(IMU_ADDR, reg + 1)

    value = (high << 8) | low

    if value >= 32768:
        value -= 65536

    return value

def read_accel():

    select_bank(0)

    ax = read_word(0x2D)
    ay = read_word(0x2F)
    az = read_word(0x31)

    # ±4g scale
    ax = ax / 8192.0
    ay = ay / 8192.0
    az = az / 8192.0

    return ax, ay, az

def read_gyro():

    select_bank(0)

    gx = read_word(0x33)
    gy = read_word(0x35)
    gz = read_word(0x37)

    # ±500 dps scale
    gx = gx / 65.5
    gy = gy / 65.5
    gz = gz / 65.5

    return gx, gy, gz

# ==========================================
# CALIBRATION
# ==========================================
print("\n===================================")
print("CALIBRATING IMU")
print("KEEP SENSOR COMPLETELY STILL")
print("===================================\n")

time.sleep(3)

acc_pitch_offset = 0
acc_roll_offset  = 0

gyro_x_offset = 0
gyro_y_offset = 0
gyro_z_offset = 0

samples = 500

for i in range(samples):

    ax, ay, az = read_accel()

    pitch = math.degrees(
        math.atan2(
            ay,
            math.sqrt(ax**2 + az**2)
        )
    )

    roll = math.degrees(
        math.atan2(-ax, az)
    )

    acc_pitch_offset += pitch
    acc_roll_offset  += roll

    gx, gy, gz = read_gyro()

    gyro_x_offset += gx
    gyro_y_offset += gy
    gyro_z_offset += gz

    time.sleep(0.005)

# Average offsets
acc_pitch_offset /= samples
acc_roll_offset  /= samples

gyro_x_offset /= samples
gyro_y_offset /= samples
gyro_z_offset /= samples

# ==========================================
# RESULT
# ==========================================
print("\n========== CALIBRATION RESULT ==========")

print(f"Pitch Offset : {acc_pitch_offset:.3f} deg")
print(f"Roll Offset  : {acc_roll_offset:.3f} deg")

print(f"Gyro X Offset: {gyro_x_offset:.3f} dps")
print(f"Gyro Y Offset: {gyro_y_offset:.3f} dps")
print(f"Gyro Z Offset: {gyro_z_offset:.3f} dps")

print("========================================\n")

# ==========================================
# GRAPH DATA
# ==========================================
MAX_POINTS = 200

pitch_data = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
roll_data  = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
yaw_data   = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)

# ==========================================
# PLOT SETUP
# ==========================================
fig, ax_plot = plt.subplots(figsize=(12,6))

line_pitch, = ax_plot.plot([], [], label="Pitch")
line_roll,  = ax_plot.plot([], [], label="Roll")
line_yaw,   = ax_plot.plot([], [], label="Yaw")

ax_plot.set_xlim(0, MAX_POINTS)
ax_plot.set_ylim(-180, 180)

ax_plot.set_title("Realtime IMU Orientation")
ax_plot.set_xlabel("Samples")
ax_plot.set_ylabel("Degrees")

ax_plot.grid(True)
ax_plot.legend()

# ==========================================
# LOOP VARIABLES
# ==========================================
yaw = 0
prev_time = time.time()

# ==========================================
# UPDATE FUNCTION
# ==========================================
def update(frame):

    global yaw
    global prev_time

    # ----- TIME -----
    current_time = time.time()
    dt = current_time - prev_time
    prev_time = current_time

    # ----- ACCEL -----
    ax, ay, az = read_accel()

    pitch = math.degrees(
        math.atan2(
            ay,
            math.sqrt(ax**2 + az**2)
        )
    ) - acc_pitch_offset

    roll = math.degrees(
        math.atan2(-ax, az)
    ) - acc_roll_offset

    # ----- GYRO -----
    gx, gy, gz = read_gyro()

    gx -= gyro_x_offset
    gy -= gyro_y_offset
    gz -= gyro_z_offset

    # ----- YAW -----
    yaw += gz * dt

    # ----- DEAD BAND -----
    if abs(pitch) < 0.5:
        pitch = 0

    if abs(roll) < 0.5:
        roll = 0

    if abs(yaw) < 0.5:
        yaw = 0

    # ======================================
    # SAVE DATA
    # ======================================
    pitch_data.append(pitch)
    roll_data.append(roll)
    yaw_data.append(yaw)

    # ======================================
    # UPDATE GRAPH
    # ======================================
    line_pitch.set_data(
        range(len(pitch_data)),
        pitch_data
    )

    line_roll.set_data(
        range(len(roll_data)),
        roll_data
    )

    line_yaw.set_data(
        range(len(yaw_data)),
        yaw_data
    )

    # ======================================
    # TERMINAL OUTPUT
    # ======================================
    print(
        f"\rPitch: {pitch:7.2f}° | "
        f"Roll: {roll:7.2f}° | "
        f"Yaw: {yaw:7.2f}°",
        end=""
    )

    return line_pitch, line_roll, line_yaw

# ==========================================
# START GRAPH
# ==========================================
ani = FuncAnimation(
    fig,
    update,
    interval=20,
    cache_frame_data=False
)

plt.tight_layout()
plt.show()