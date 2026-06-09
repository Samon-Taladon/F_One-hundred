import smbus
import time
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

bus = smbus.SMBus(7)
IMU_ADDR = 0x68

def select_bank(bank):
    bus.write_byte_data(IMU_ADDR, 0x7F, bank << 4)

print("Initializing ICM-20948...")
select_bank(0)
bus.write_byte_data(IMU_ADDR, 0x06, 0x80)
time.sleep(0.1)
bus.write_byte_data(IMU_ADDR, 0x06, 0x01)
bus.write_byte_data(IMU_ADDR, 0x07, 0x00)
select_bank(2)
bus.write_byte_data(IMU_ADDR, 0x14, 0x04)
bus.write_byte_data(IMU_ADDR, 0x01, 0x04)
select_bank(0)
time.sleep(0.1)
print("IMU Ready!\n")

def read_word_2c(addr):
    high = bus.read_byte_data(IMU_ADDR, addr)
    low = bus.read_byte_data(IMU_ADDR, addr + 1)
    val = (high << 8) + low
    if val >= 0x8000:
        return -((65535 - val) + 1)
    else:
        return val

class LowPassFilter:
    def __init__(self, alpha=0.5):
        self.alpha = alpha
        self.value = None
    
    def filter(self, new_value):
        if self.value is None:
            self.value = new_value
        else:
            self.value = self.alpha * new_value + (1 - self.alpha) * self.value
        return self.value

lpf_ax = LowPassFilter(alpha=0.2)
lpf_ay = LowPassFilter(alpha=0.2)
lpf_az = LowPassFilter(alpha=0.2)
lpf_gx = LowPassFilter(alpha=0.3)
lpf_gy = LowPassFilter(alpha=0.3)
lpf_gz = LowPassFilter(alpha=0.3)

print("Calibrating (keep still for 1 second)...")
acc_x_offset = 0
acc_y_offset = 0
gyr_x_bias = 0
gyr_y_bias = 0
gyr_z_bias = 0

for i in range(100):
    select_bank(0)
    if i < 20:
        acc_x_offset += read_word_2c(0x2D) / 4096.0
        acc_y_offset += read_word_2c(0x2F) / 4096.0
    gyr_x_bias += read_word_2c(0x33) / 32.8
    gyr_y_bias += read_word_2c(0x35) / 32.8
    gyr_z_bias += read_word_2c(0x37) / 32.8
    time.sleep(0.01)

acc_x_offset /= 20
acc_y_offset /= 20
gyr_x_bias /= 100
gyr_y_bias /= 100
gyr_z_bias /= 100

print(f"Gyro Z bias: {gyr_z_bias:.3f}°/s")
print("Calibration complete!\n")

pitch = 0.0
roll = 0.0
yaw = 0.0
alpha = 0.85
last_time = time.time()

max_points = 200
time_data = deque(maxlen=max_points)
pitch_data = deque(maxlen=max_points)
roll_data = deque(maxlen=max_points)
yaw_data = deque(maxlen=max_points)
start_time = time.time()

fig = plt.figure(figsize=(14, 8))
plot_ax = fig.add_subplot(111)
fig.suptitle('ICM-20948 IMU - Production Ready (with Yaw Deadband)', fontsize=16, fontweight='bold')

line_pitch, = plot_ax.plot([], [], 'r-', label='Pitch (Stable)', linewidth=2.5)
line_roll, = plot_ax.plot([], [], 'g-', label='Roll (Stable)', linewidth=2.5)
line_yaw, = plot_ax.plot([], [], 'b--', label='Yaw (Deadband Active)', linewidth=2, alpha=0.7)

plot_ax.set_xlabel('Time (seconds)', fontsize=12, fontweight='bold')
plot_ax.set_ylabel('Angle (degrees)', fontsize=12, fontweight='bold')
plot_ax.set_ylim(-90, 90)
plot_ax.axhline(y=0, color='k', linestyle='--', alpha=0.5)
plot_ax.legend(loc='upper left', fontsize=11)
plot_ax.grid(True, alpha=0.4)

text_display = plot_ax.text(0.02, 0.98, '', transform=plot_ax.transAxes, 
                             verticalalignment='top', fontsize=11, 
                             bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.9))

frame_count = 0

def on_key(event):
    global yaw
    if event.key == 'r':
        yaw = 0.0
        print(f"\n[RESET] Yaw → 0°\n")

fig.canvas.mpl_connect('key_press_event', on_key)

def update(frame):
    global pitch, roll, yaw, last_time, frame_count
    
    try:
        select_bank(0)
        
        acc_x_raw = read_word_2c(0x2D) / 4096.0 - acc_x_offset
        acc_y_raw = read_word_2c(0x2F) / 4096.0 - acc_y_offset
        acc_z_raw = read_word_2c(0x31) / 4096.0
        gyr_x_raw = read_word_2c(0x33) / 32.8 - gyr_x_bias
        gyr_y_raw = read_word_2c(0x35) / 32.8 - gyr_y_bias
        gyr_z_raw = read_word_2c(0x37) / 32.8 - gyr_z_bias
        
        acc_x = lpf_ax.filter(acc_x_raw)
        acc_y = lpf_ay.filter(acc_y_raw)
        acc_z = lpf_az.filter(acc_z_raw)
        gyr_x = lpf_gx.filter(gyr_x_raw)
        gyr_y = lpf_gy.filter(gyr_y_raw)
        gyr_z = lpf_gz.filter(gyr_z_raw)
        
        # ========================================================
        # [NEW] เริ่มส่วนที่แก้ไข: เพิ่ม Deadband ให้ Gyro Z
        # ========================================================
        # กำหนดค่าความไวในการเริ่มจับการหมุน (องศา/วินาที)
        # ถ้าวางนิ่งๆ แล้ว Yaw ยังไหลอยู่ ให้ลองเพิ่มค่านี้เป็น 0.8 หรือ 1.0
        # แต่ถ้าเพิ่มเยอะไป เวลาหมุนเซนเซอร์ช้าๆ ค่า Yaw จะไม่ขยับ
        DEADBAND_THRESHOLD = 0.5 
        
        if abs(gyr_z) < DEADBAND_THRESHOLD:
            gyr_z = 0.0
        # ========================================================
        # [NEW] จบส่วนที่แก้ไข
        # ========================================================

        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        
        if abs(acc_x) < 0.05 and abs(acc_y) < 0.05 and acc_z < -0.9:
            accel_pitch = 0.0
            accel_roll = 0.0
        else:
            if abs(acc_z) < 0.1:
                acc_z = -0.1 if acc_z < 0 else 0.1
            accel_pitch = math.atan2(acc_y, math.sqrt(acc_x**2 + acc_z**2)) * 180 / math.pi
            accel_roll = math.atan2(-acc_x, acc_z) * 180 / math.pi
        
        gyro_pitch = pitch + gyr_x * dt
        gyro_roll = roll + gyr_y * dt
        yaw += gyr_z * dt
        
        pitch = alpha * gyro_pitch + (1 - alpha) * accel_pitch
        roll = alpha * gyro_roll + (1 - alpha) * accel_roll
        
        if yaw > 180:
            yaw -= 360
        elif yaw < -180:
            yaw += 360
        
        elapsed = time.time() - start_time
        time_data.append(elapsed)
        pitch_data.append(pitch)
        roll_data.append(roll)
        yaw_data.append(yaw)
        
        line_pitch.set_data(list(time_data), list(pitch_data))
        line_roll.set_data(list(time_data), list(roll_data))
        line_yaw.set_data(list(time_data), list(yaw_data))
        
        text_display.set_text(
            f'Pitch: {pitch:6.1f}° ✓\n'
            f'Roll:  {roll:6.1f}° ✓\n'
            f'Yaw:   {yaw:6.1f}° (Deadband)\n'
            f'\n'
            f'Press R to reset Yaw'
        )
        
        if len(time_data) > 0:
            plot_ax.set_xlim(max(0, elapsed - 10), elapsed + 1)
        
        frame_count += 1
        if frame_count % 20 == 0:
            print(f"[{elapsed:.1f}s] Pitch={pitch:6.1f}°  Roll={roll:6.1f}°  Yaw={yaw:6.1f}°")
        
    except Exception as e:
        print(f"Error: {e}")
    
    return line_pitch, line_roll, line_yaw, text_display

ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)

plt.tight_layout()
print("="*70)
print(" ICM-20948 IMU - PRODUCTION VERSION (WITH DEADBAND)")
print("="*70)
print(" ✅ PITCH/ROLL: Stable")
print(" ✅ YAW: Fixed stationary drift using Deadband (0.5 deg/s threshold)")
print("="*70)
plt.show()