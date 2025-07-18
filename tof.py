import time
import robomaster
from robomaster import robot
import csv
from datetime import datetime

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")  # ใช้ conn_type="ap" ตามโค้ด gimbal เดิม

    ep_gimbal = ep_robot.gimbal
    ep_sensor = ep_robot.sensor

    # สร้างชื่อไฟล์ CSV พร้อม timestamp
    filename = datetime.now().strftime("gimbal_tof_data_5.csv")

    # ตัวแปรสำหรับเก็บข้อมูล ToF ล่าสุด
    latest_tof_data = [0, 0, 0, 0]  # ค่าเริ่มต้นสำหรับ tof1, tof2, tof3, tof4

    with open(filename, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        # เขียนหัวข้อคอลัมน์ในไฟล์ CSV
        csv_writer.writerow(['time', 'pitch_angle', 'yaw_angle', 'pitch_ground_angle', 'yaw_ground_angle', 
                            'tof1', 'tof2', 'tof3', 'tof4'])

        start_time = time.time()

        # ฟังก์ชันสำหรับ subscribe ข้อมูลมุมของ gimbal
        def gimbal_data_handler(angle_info):
            pitch_angle, yaw_angle, pitch_ground_angle, yaw_ground_angle = angle_info
            elapsed_time = time.time() - start_time
            # บันทึกข้อมูลมุมและข้อมูล ToF ล่าสุดลงในไฟล์ CSV
            csv_writer.writerow([elapsed_time, pitch_angle, yaw_angle, pitch_ground_angle, yaw_ground_angle,
                                latest_tof_data[0], latest_tof_data[1], latest_tof_data[2], latest_tof_data[3]])
            # แสดงผลทางคอนโซลเพื่อการตรวจสอบ
            print("gimbal: pitch={0}, yaw={1}, pitch_ground={2}, yaw_ground={3}, tof1={4}, tof2={5}, tof3={6}, tof4={7}".format(
                pitch_angle, yaw_angle, pitch_ground_angle, yaw_ground_angle,
                latest_tof_data[0], latest_tof_data[1], latest_tof_data[2], latest_tof_data[3]))

        # ฟังก์ชันสำหรับ subscribe ข้อมูล ToF
        def tof_data_handler(sub_info):
            global latest_tof_data
            latest_tof_data = list(sub_info)  # อัปเดตข้อมูล ToF ล่าสุด (แปลง tuple เป็น list)
            # แสดงผลทางคอนโซลเพื่อการตรวจสอบ
            print("tof1:{0}, tof2:{1}, tof3:{2}, tof4:{3}".format(
                latest_tof_data[0], latest_tof_data[1], latest_tof_data[2], latest_tof_data[3]))

        # เริ่มการ subscribe ข้อมูลมุมและ ToF ด้วยความถี่ 5 Hz
        ep_gimbal.sub_angle(freq=5, callback=gimbal_data_handler)
        ep_sensor.sub_distance(freq=5, callback=tof_data_handler)

        # คำสั่งควบคุมการเคลื่อนที่ของ gimbal (ตามโค้ดเดิม)
        ep_gimbal.moveto(pitch=0, yaw=0).wait_for_completed()
        ep_gimbal.moveto(pitch=0, yaw=90, pitch_speed=50, yaw_speed=10).wait_for_completed()
        ep_gimbal.moveto(pitch=0, yaw=-90, pitch_speed=100, yaw_speed=10).wait_for_completed()
        ep_gimbal.moveto(pitch=0, yaw=0).wait_for_completed()

        # ยกเลิกการ subscribe ข้อมูล
        ep_gimbal.unsub_angle()
        ep_sensor.unsub_distance()

    # ปิดการเชื่อมต่อหุ่นยนต์
    ep_robot.close()