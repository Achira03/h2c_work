from robomaster import robot
import time
import csv

# ====== พารามิเตอร์การควบคุม ======
target_distance = 2.0  # เมตร
kp = 1.0               # ค่าคงที่ของ P controller
dt = 0.1               # ความถี่ในการอัปเดต (วินาที)
max_speed = 0.3        # จำกัดความเร็วสูงสุด (m/s)

# ====== เริ่มต้นการเชื่อมต่อกับหุ่นยนต์ ======
robot_ctrl = robot.Robot()
robot_ctrl.initialize(conn_type="ap")  # เชื่อมต่อ Wi-Fi แบบ STA

chassis = robot_ctrl.chassis

# ====== ตัวแปรจำลองตำแหน่ง ======
position = 0.0
error = target_distance - position

print("เริ่มเคลื่อนที่ไปข้างหน้า 2 เมตร...")

# ====== เตรียมไฟล์ CSV ======
with open("position_log_2.csv", mode="w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["timestamp", "x", "y", "z"])

    # ====== วนลูปควบคุม ======
    while abs(error) > 0.01:
        speed = kp * error
        speed = max(min(speed, max_speed), -max_speed)  # จำกัดความเร็วไม่ให้เกินช่วงปลอดภัย

        chassis.drive_speed(x=speed, y=0, z=0)  # เคลื่อนที่ในแนวแกน x
        time.sleep(dt)

        # จำลองตำแหน่ง (หากไม่มี encoder จริง)
        position += speed * dt
        error = target_distance - position

        # บันทึกข้อมูลลง CSV (timestamp, x, y, z)
        timestamp = time.time()
        writer.writerow([timestamp, round(position, 2), 0.0, 0.0])

        print(f"ตำแหน่ง: {position:.3f} m | error: {error:.3f} m")

# ====== หยุดหุ่นยนต์ ======
chassis.drive_speed(x=0, y=0, z=0)
print("ถึงเป้าหมายแล้ว!")
robot_ctrl.close()

# ====== ปิดการเชื่อมต่อ ======