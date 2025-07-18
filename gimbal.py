import time
import robomaster
from robomaster import robot
import csv
from datetime import datetime

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_gimbal = ep_robot.gimbal

    pitch_val = 0
    yaw_val = 30

    # สร้างชื่อไฟล์ CSV พร้อม timestamp
    filename = datetime.now().strftime("gimbal_data.csv")

    with open(filename, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        # เขียนหัวข้อคอลัมน์ในไฟล์ CSV
        csv_writer.writerow(['timestamp', 'pitch_angle', 'yaw_angle', 'pitch_ground_angle', 'yaw_ground_angle'])

        start_time = time.time()

        # กำหนดฟังก์ชันสำหรับการ subscribe ข้อมูลมุม
        def sub_data_handler(angle_info):
            pitch_angle, yaw_angle, pitch_ground_angle, yaw_ground_angle = angle_info
            timestamp = time.time()
            # บันทึกข้อมูลลงในไฟล์ CSV
            csv_writer.writerow([timestamp, pitch_angle, yaw_angle, pitch_ground_angle, yaw_ground_angle])
            # ยังคงแสดงผลทางคอนโซลเพื่อการตรวจสอบ
            print("gimbal angle: pitch_angle:{0}, yaw_angle:{1}, pitch_ground_angle:{2}, yaw_ground_angle:{3}".format(
                pitch_angle, yaw_angle, pitch_ground_angle, yaw_ground_angle))

        # เริ่มการ subscribe ข้อมูลมุมด้วยความถี่ 5 Hz
        ep_gimbal.sub_angle(freq=5, callback=sub_data_handler)

        # คำสั่งควบคุมการเคลื่อนที่ของ gimbal
        ep_gimbal.moveto(pitch=0, yaw=0).wait_for_completed()
        ep_gimbal.moveto(pitch=0, yaw=90, pitch_speed=50, yaw_speed=30).wait_for_completed()
        ep_gimbal.moveto(pitch=0, yaw=-90, pitch_speed=100, yaw_speed=30).wait_for_completed()
        ep_gimbal.moveto(pitch=0, yaw=0).wait_for_completed()

        # ยกเลิกการ subscribe ข้อมูลมุม
        ep_gimbal.unsub_angle()

    # ปิดการเชื่อมต่อหุ่นยนต์
    ep_robot.close()