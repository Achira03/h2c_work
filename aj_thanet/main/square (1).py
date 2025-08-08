# -*- coding: utf-8 -*-
import time
import csv
import math
from robomaster import robot

#คลาส PID Controller
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self._last_error = 0.0
        self._integral = 0.0
        self._last_time = time.time()

    def update(self, current_value):
        current_time = time.time()
        dt = current_time - self._last_time
        if dt <= 0.0:
            return 0.0
        
        error = self.setpoint - current_value
        
        #จัดการกรณค่ามุมข้าม 180/-180 สำคัญสำหรับ Yaw
        if abs(error) > 180:
            if error > 0:
                error = error - 360
            else:
                error = error + 360

        self._integral += error * dt
        derivative = (error - self._last_error) / dt
        output = (self.Kp * error) + (self.Ki * self._integral) + (self.Kd * derivative)
        
        self._last_error = error
        self._last_time = current_time
        return output

    def reset(self, setpoint=None):
        if setpoint is not None:
            self.setpoint = setpoint
        self._last_error = 0.0
        self._integral = 0.0
        self._last_time = time.time()

#ตัวแปร Global และฟังก์ชันสำหรับ Sensor
current_position = [0, 0, 0]
current_attitude = [0, 0, 0]

def position_handler(info):
    global current_position
    x, y, z = info
    current_position = [x, y, z]

def attitude_handler(info):
    global current_attitude
    yaw, pitch, roll = info
    current_attitude = [yaw, pitch, roll]

#ฟังก์ชันเคลื่อนที่สี่เหลี่ยมพร้อมบันทึกข้อมูล
def move_square_with_pid_and_log(ep_chassis, tile_size=0.6, turn_speed=90):
    # --- ตั้งค่า PID สำหรับ ระยะทาง(Forward)
    FWD_KP = 0.8
    FWD_KI = 0.2
    FWD_KD = 0.15
    TOLERANCE = 0.02
    
    #ตั้งค่า PID สำหรับ ทิศทาง(Yaw)
    YAW_KP = 1.2
    YAW_KI = 0.1
    YAW_KD = 0.5

    pid_forward = PIDController(Kp=FWD_KP, Ki=FWD_KI, Kd=FWD_KD, setpoint=tile_size)
    pid_yaw = PIDController(Kp=YAW_KP, Ki=YAW_KI, Kd=YAW_KD, setpoint=0)

    #ตรียมไฟล์ CSV สำหรับบันทึกข้อมูล
    log_filename = "robot_log_with_yaw.csv"
    log_fieldnames = ["timestamp", "side", "target_distance", "current_distance", "dist_error", "pid_speed_output", "target_yaw", "current_yaw", "yaw_correction_speed"]
    
    with open(log_filename, mode='w', newline='', encoding='utf-8') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=log_fieldnames)
        writer.writeheader()
        print(f"กำลังบันทึกข้อมูลลงในไฟล์ '{log_filename}'...")

        start_time_program = time.time()

        for i in range(4):
            print(f"\n--- ด้านที่ {i+1}/4: เคลื่อนที่ไปข้างหน้า {tile_size} เมตร ---")

            #เคลื่อนที่ไปข้างหน้าพร้อมควบคุมทิศทาง
            start_pos = list(current_position)
            start_yaw_deg = current_attitude[0] 

            pid_forward.reset(setpoint=tile_size)
            pid_yaw.reset(setpoint=start_yaw_deg) 

            while True:
                #คำนวณระยะทาง
                dx = current_position[0] - start_pos[0]
                dy = current_position[1] - start_pos[1]
                start_yaw_rad = math.radians(start_yaw_deg)
                distance_traveled = dx * math.cos(start_yaw_rad) + dy * math.sin(start_yaw_rad)
                
                #คำนวณ PID
                forward_speed = pid_forward.update(distance_traveled)
                forward_speed = max(min(forward_speed, 0.8), -0.8) #จำกัดความเร็วสูงสุด

                current_yaw = current_attitude[0]
                yaw_correction_speed = pid_yaw.update(current_yaw)

                # สั่งให้หุ่นยนต์เคลื่อนที่ไปข้างหน้า (x) พร้อมกับแก้เอียง (z) ไปพร้อมกัน
                ep_chassis.drive_speed(x=forward_speed, y=0, z=yaw_correction_speed)

                # แสดงผลและบันทึกข้อมูล
                dist_error = tile_size - distance_traveled
                current_timestamp = time.time() - start_time_program
                
                print(f"\rSide: {i+1} | Dist: {distance_traveled:.2f}m | Speed: {forward_speed:.2f}m/s | Yaw: {current_yaw:.1f}° | Yaw Correct: {yaw_correction_speed:.1f}°/s", end="")
                
                # เขียนข้อมูลลงไฟล์ CSV
                writer.writerow({
                    "timestamp": current_timestamp,
                    "side": i + 1,
                    "target_distance": tile_size,
                    "current_distance": distance_traveled,
                    "dist_error": dist_error,
                    "pid_speed_output": forward_speed,
                    "target_yaw": start_yaw_deg,
                    "current_yaw": current_yaw,
                    "yaw_correction_speed": yaw_correction_speed
                })

                if abs(dist_error) < TOLERANCE and abs(forward_speed) < 0.05:
                    break
                
                time.sleep(0.02)

            ep_chassis.drive_speed(x=0, y=0, z=0) # หยุดสนิท
            print("\nถึงเป้าหมายแล้ว, หยุดชั่วครู่...")
            time.sleep(1)

            # เลี้ยว 90 องศา
            if i < 3: # ไม่ต้องเลี้ยวในรอบสุดท้าย
                print(f"--- ด้านที่ {i+1}/4: กำลังเลี้ยวขวา 90 องศา ---")
                ep_chassis.move(x=0, y=0, z=-90, z_speed=turn_speed).wait_for_completed()
                time.sleep(1)
        
        # ส่วนของการกระทำสุดท้าย: หันขวาเมื่อสิ้นสุดภารกิจ
        print("\n--- เคลื่อนที่เสร็จสิ้น, กำลังหันขวา 90 องศา ---")
        ep_chassis.move(x=0, y=0, z=-90, z_speed=turn_speed).wait_for_completed()
        time.sleep(1)
        print("หันขวาเรียบร้อยแล้ว!")

        print(f"\nการเคลื่อนที่และการบันทึกข้อมูลเสร็จสมบูรณ์! เช็คไฟล์ '{log_filename}'")


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    
    ep_led = ep_robot.led
    ep_led.set_led(comp="all", r=0, g=255, b=0, effect="on") # ไฟสีเขียว
    
    ep_chassis = ep_robot.chassis
    
    print("กำลัง Subscribe ข้อมูลเซ็นเซอร์สำหรับ PID...")
    ep_chassis.sub_position(freq=50, callback=position_handler)
    ep_chassis.sub_attitude(freq=50, callback=attitude_handler)
    time.sleep(2) # รอให้เซ็นเซอร์เริ่มทำงานนิ่งๆ
    
    # เรียกใช้ฟังก์ชันหลัก
    move_square_with_pid_and_log(ep_chassis, tile_size=0.6, turn_speed=80)
    
    print("\nกำลัง Unsubscribe เซ็นเซอร์ทั้งหมด...")
    ep_chassis.unsub_position()
    ep_chassis.unsub_attitude()
    print("ยกเลิกการ Subscribe เซ็นเซอร์ทั้งหมดแล้ว.")
    
    #ส่วนที่เพิ่มเข้ามา
    print("สิ้นสุดภารกิจ, เปลี่ยนไฟเป็นสีแดง")
    ep_led.set_led(comp="all", r=255, g=0, b=0, effect="on")
    time.sleep(1) # รอเล็กน้อยเพื่อให้แน่ใจว่าคำสั่งเปลี่ยนสีถูกส่งไปแล้ว
    
    
    ep_robot.close()
    print("\nโปรแกรมสิ้นสุดการทำงานแล้ว.")