import time
import csv
from datetime import datetime
from robomaster import robot

# สร้าง Dictionary สำหรับเก็บข้อมูลเซ็นเซอร์แต่ละประเภท
sensor_logs = {
    "position": [],
    "attitude": [],
    "imu": [],
    "esc": [],
    "status": []
}

def log_handler(sensor_type, info):
    """
    ฟังก์ชัน Callback สำหรับการ Subscribe ข้อมูลจากเซ็นเซอร์
    จะทำการเพิ่ม Timestamp และข้อมูลเซ็นเซอร์ลงใน List ที่เกี่ยวข้อง
    ใน Dictionary 'sensor_logs'
    """
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
    log_entry = [timestamp]

    # สำหรับ ESC: info = (speed[4], angle[4], timestamp[4], state[4])
    if sensor_type == "esc" and isinstance(info, (tuple, list)) and len(info) == 4:
        speed, angle, timestamp_internal, state = info
        log_entry.extend([
            list(speed),
            list(angle),
            list(timestamp_internal),
            list(state)
        ])
    elif isinstance(info, (tuple, list)):
        log_entry.extend(info)
    else:
        log_entry.append(info)

    sensor_logs[sensor_type].append(log_entry)
    # สามารถเปิดบรรทัดด้านล่างนี้ได้หากต้องการพิมพ์ข้อมูลทั้งหมดลง Console ทันที
    # print(f"{sensor_type} info: {info}")


def move_square(ep_chassis, tile_size=0.6, speed=1.0, turn_speed=90, decelerate_distance=0.03, slow_speed=0.4):
    """
    สั่งให้หุ่นยนต์เคลื่อนที่ในรูปแบบสี่เหลี่ยม โดยมีการชะลอความเร็วก่อนหยุด
    :param ep_chassis: วัตถุควบคุมแชสซีของหุ่นยนต์
    :param tile_size: ระยะทางของแต่ละด้านของสี่เหลี่ยม (เมตร)
    :param speed: ความเร็วปกติในการเคลื่อนที่ (เมตร/วินาที)
    :param turn_speed: ความเร็วในการหมุน (องศา/วินาที)
    :param decelerate_distance: ระยะทางก่อนถึงจุดหยุดที่หุ่นยนต์จะเริ่มชะลอความเร็ว (เมตร)
    :param slow_speed: ความเร็วที่หุ่นยนต์จะใช้เมื่อชะลอ (เมตร/วินาที)
    """
    print(f"เริ่มการเคลื่อนที่แบบสี่เหลี่ยม: ขนาดด้าน={tile_size} เมตร, ความเร็วปกติ={speed} เมตร/วินาที, "
          f"ความเร็วในการเลี้ยว={turn_speed} องศา/วินาที, ชะลอที่ระยะ={decelerate_distance} เมตร ด้วยความเร็ว={slow_speed} เมตร/วินาที")

    for i in range(4):
        remaining_distance = tile_size
        print(f"กำลังเคลื่อนที่ไปข้างหน้า (ด้านที่ {i+1}/4)...")

        # ส่วนที่ 1: เคลื่อนที่ด้วยความเร็วปกติ
        # ตรวจสอบให้แน่ใจว่าระยะทางที่จะเคลื่อนที่ด้วยความเร็วปกติไม่เป็นค่าติดลบ
        if remaining_distance > decelerate_distance:
            move_distance = tile_size - decelerate_distance
            ep_chassis.move(x=move_distance, y=0, z=0, xy_speed=speed).wait_for_completed()
            remaining_distance -= move_distance
            print(f"   เคลื่อนที่ {move_distance:.2f} เมตร ด้วยความเร็ว {speed} m/s")

        # ส่วนที่ 2: ชะลอความเร็ว
        # ตรวจสอบว่ายังมีระยะทางที่เหลืออยู่ (ส่วนที่จะชะลอ)
        if remaining_distance > 0:
            ep_chassis.move(x=remaining_distance, y=0, z=0, xy_speed=slow_speed).wait_for_completed()
            print(f"   ชะลอความเร็วใน {remaining_distance:.2f} เมตรสุดท้าย ด้วยความเร็ว {slow_speed} m/s")

        time.sleep(1)  # พักครู่หนึ่งหลังจากการเคลื่อนที่ตรงเสร็จสมบูรณ์

        print(f"กำลังเลี้ยวขวา (ด้านที่ {i+1}/4)...")
        ep_chassis.move(x=0, y=0, z=-90, z_speed=turn_speed).wait_for_completed()
        time.sleep(1) # พักครู่หนึ่งหลังจากการเลี้ยว

    print("การเคลื่อนที่แบบสี่เหลี่ยมเสร็จสมบูรณ์")


def save_logs_to_separate_csv(filename_prefix="robot_log"):
    """
    บันทึกข้อมูลเซ็นเซอร์ที่เก็บรวบรวมไว้ลงในไฟล์ CSV แยกตามประเภทเซ็นเซอร์
    """
    print("\n--- กำลังบันทึกข้อมูลเซ็นเซอร์ ---")

    # ข้อมูลตำแหน่ง (Position Data)
    if sensor_logs["position"]:
        with open(f"{filename_prefix}_position.csv", "w", newline='', encoding='utf-8') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Timestamp", "Pos_x (m)", "Pos_y (m)", "Pos_z (m)"])
            writer.writerows(sensor_logs["position"])
        print(f"บันทึกข้อมูลตำแหน่งลงใน {filename_prefix}_position.csv ({len(sensor_logs['position'])} รายการ)")
    else:
        print("ไม่มีข้อมูลตำแหน่งให้บันทึก.")

    # ข้อมูลทัศนคติ (Attitude Data)
    if sensor_logs["attitude"]:
        with open(f"{filename_prefix}_attitude.csv", "w", newline='', encoding='utf-8') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Timestamp", "Roll (deg)", "Pitch (deg)", "Yaw (deg)"])
            writer.writerows(sensor_logs["attitude"])
        print(f"บันทึกข้อมูลทัศนคติลงใน {filename_prefix}_attitude.csv ({len(sensor_logs['attitude'])} รายการ)")
    else:
        print("ไม่มีข้อมูลทัศนคติให้บันทึก.")

    # ข้อมูล IMU (Inertial Measurement Unit)
    if sensor_logs["imu"]:
        with open(f"{filename_prefix}_imu.csv", "w", newline='', encoding='utf-8') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Timestamp", "Acc_X (g)", "Acc_Y (g)", "Acc_Z (g)", "Gyro_X (deg/s)", "Gyro_Y (deg/s)", "Gyro_Z (deg/s)"])
            writer.writerows(sensor_logs["imu"])
        print(f"บันทึกข้อมูล IMU ลงใน {filename_prefix}_imu.csv ({len(sensor_logs['imu'])} รายการ)")
    else:
        print("ไม่มีข้อมูล IMU ให้บันทึก.")

    # ข้อมูล ESC (Electronic Speed Controller)
    # **แก้ไข: ปรับ Header และการบันทึกข้อมูลให้ตรงกับข้อมูลที่ได้รับจาก ESC ของ RoboMaster**
    # **ข้อมูลที่ได้รับจาก sub_esc คือ (speed, angle, timestamp, state)**
    if sensor_logs["esc"]:
        with open(f"{filename_prefix}_esc.csv", "w", newline='', encoding='utf-8') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Timestamp_capture", "Speed", "Angle", "Timestamp_internal", "State"])
            writer.writerows(sensor_logs["esc"])
        print(f"บันทึกข้อมูล ESC ลงใน {filename_prefix}_esc.csv ({len(sensor_logs['esc'])} รายการ)")
    else:
        print("ไม่มีข้อมูล ESC ให้บันทึก.")

    # ข้อมูลสถานะ (Status Data)
    # **สำคัญ: Header นี้สมมติว่า Status ส่งข้อมูลเป็น (Static_Flag, Chassis_Power_State, Error_Status, ...)**
    # **ถ้าข้อมูลที่ได้รับจริงมีจำนวนหรือความหมายต่างกัน โปรดปรับ Header ให้ตรง**
    if sensor_logs["status"]:
        # รวมค่าทุกคอลัมน์หลัง Timestamp เป็น list แล้วเก็บในคอลัมน์เดียวชื่อ 'data'
        status_logs = []
        for row in sensor_logs["status"]:
            # row[0] = Timestamp, row[1:] = ข้อมูล sensor
            status_logs.append([row[0], list(row[1:])])

        with open(f"{filename_prefix}_status.csv", "w", newline='', encoding='utf-8') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Timestamp", "data"])
            writer.writerows(status_logs)
        print(f"บันทึกข้อมูลสถานะลงใน {filename_prefix}_status.csv ({len(sensor_logs['status'])} รายการ)")
    else:
        print("ไม่มีข้อมูลสถานะให้บันทึก.")

    print("บันทึกข้อมูลทั้งหมดที่สามารถบันทึกได้แล้ว.")

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis

    print("กำลัง Subscribe ข้อมูลเซ็นเซอร์...")
    ep_chassis.sub_position(freq=10, callback=lambda info: log_handler("position", info))
    ep_chassis.sub_attitude(freq=10, callback=lambda info: log_handler("attitude", info))
    ep_chassis.sub_imu(freq=10, callback=lambda info: log_handler("imu", info))
    # สำหรับ ESC, ข้อมูลที่ส่งมาคือ (speed, angle, timestamp_internal, state)
    ep_chassis.sub_esc(freq=10, callback=lambda info: log_handler("esc", info))
    ep_chassis.sub_status(freq=10, callback=lambda info: log_handler("status", info))
    print("เปิดใช้งานการ Subscribe เซ็นเซอร์แล้ว.")

    # สั่งให้หุ่นยนต์เคลื่อนที่แบบสี่เหลี่ยม พร้อมพารามิเตอร์การชะลอ
    move_square(ep_chassis, tile_size=0.6, speed=0.8, turn_speed=80,
                decelerate_distance=0.03, slow_speed=0.4)

    print("\nกำลัง Unsubscribe เซ็นเซอร์ทั้งหมด...")
    ep_chassis.unsub_status()
    ep_chassis.unsub_esc()
    ep_chassis.unsub_imu()
    ep_chassis.unsub_attitude()
    ep_chassis.unsub_position()
    print("ยกเลิกการ Subscribe เซ็นเซอร์ทั้งหมดแล้ว.")

    ep_robot.close()

    # บันทึกข้อมูลที่รวบรวมได้ทั้งหมดลงในไฟล์ CSV แยกตามประเภทเซ็นเซอร์
    # save_logs_to_separate_csv("move_square_robot_log_4")

    print("\nโปรแกรมสิ้นสุดการทำงานแล้ว.")