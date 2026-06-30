import pandas as pd
import matplotlib.pyplot as plt

# =========================
# โหลดไฟล์ CSV
# =========================
# เปลี่ยนชื่อไฟล์ตามจริง
csv_file = "/home/f1/f1/waypoints.csv"

# อ่านไฟล์
df = pd.read_csv(csv_file, header=None)

# คอลัมน์แรก = x
# คอลัมน์สอง = y
x = df[0]
y = df[1]

# =========================
# Plot
# =========================
plt.figure(figsize=(8, 8))

plt.plot(x, y, marker='o')

plt.xlabel("X")
plt.ylabel("Y")
plt.title("XY Plot from CSV")

plt.grid(True)

# ทำให้สเกลแกนเท่ากัน
plt.axis('equal')

plt.show()