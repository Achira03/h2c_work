# นำเข้าโมดูล
import time  
import robomaster
from robomaster import robot
import csv
from datetime import datetime

# เชื่อมต่อกับหุ่นยนต์
if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    # สร้างตัวควบคุม Gimbal เข้าถึง Object ของ Gimbal จาก Object ของหุ่นยนต์
    ep_gimbal = ep_robot.gimbal


    # สร้างชื่อไฟล์ CSV พร้อม timestamp
    filename = datetime.now().strftime("gimbal_data.csv")

# สร้างชื่อไฟล์ gimbal_data.csv ตามเวลาที่รันโปรแกรม
    with open(filename, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        # เขียนหัวข้อคอลัมน์ในไฟล์ CSV
        csv_writer.writerow(['timestamp', 'pitch_angle', 'yaw_angle', 'pitch_ground_angle', 'yaw_ground_angle'])

        start_time = time.time()

        # กำหนดฟังก์ชันสำหรับการ subscribe ข้อมูลมุม
        # ฟังก์ชัน Callback จะถูกเรียกใช้ทุกครั้งที่ได้รับข้อมูลมุม Gimbal ใหม่จากหุ่นยนต์
        def sub_data_handler(angle_info): 
            pitch_angle, yaw_angle, pitch_ground_angle, yaw_ground_angle = angle_info
            # pitch_angle, yaw_angle, pitch_ground_angle, yaw_ground_angle = angle_info ข้อมูลมุมจะถูกส่งมาในรูปของ tuple angle_info ซึ่งทำการ Unpack ออกเป็นตัวแปรแต่ละตัว
            timestamp = time.time()
            # timestamp จะถูกบันทึกเป็นเวลาปัจจุบันในรูปแบบของ timestamp
            # บันทึกข้อมูลลงในไฟล์ CSV
            csv_writer.writerow([timestamp, pitch_angle, yaw_angle, pitch_ground_angle, yaw_ground_angle])
            # ยังคงแสดงผลทางคอนโซลเพื่อการตรวจสอบ
            print("gimbal angle: pitch_angle:{0}, yaw_angle:{1}, pitch_ground_angle:{2}, yaw_ground_angle:{3}".format(
                pitch_angle, yaw_angle, pitch_ground_angle, yaw_ground_angle))
        
        # คำสั่งควบคุมการเคลื่อนที่ของ gimbal
        ep_gimbal.sub_angle(freq=5, callback=sub_data_handler)

        ep_gimbal.moveto(pitch=0, yaw=0).wait_for_completed()
        ep_gimbal.moveto(pitch=0, yaw=90, pitch_speed=50, yaw_speed=30).wait_for_completed()
        ep_gimbal.moveto(pitch=0, yaw=-90, pitch_speed=100, yaw_speed=30).wait_for_completed()
        ep_gimbal.moveto(pitch=0, yaw=0).wait_for_completed()
        # subscribe ข้อมูลมุมด้วยความถี่ 5 Hz
        # pitch มุม Pitch ที่ต้องการ (หน่วยเป็นองศา) โดยจะกำหนดให้เป็น 0 องศา เพ่ื่อให้ gimbal ขนานกับพื้น
        # yaw มุม Yaw ที่ต้องการ (หน่วยเป็นองศา)
        # pitch_speed ความเร็วในการหมุน Pitch (หน่วยเป็นองศาต่อวินาที)
        # yaw_speed ความเร็วในการหมุน Yaw (หน่วยเป็นองศาต่อวินาที)
        # คำสั่ง moveto จะทำให้ gimbal หมุนไปยังมุม
        # wait_for_completed จะรอให้การเคลื่อนที่เสร็จสิ้นก่อนที่จะดำเนินการต่อไป

        # ยกเลิกการ subscribe ข้อมูลมุม
        ep_gimbal.unsub_angle()

    # ปิดการเชื่อมต่อหุ่นยนต์
    ep_robot.close()
    