import time
import robomaster
from robomaster import robot
import csv
from datetime import datetime
import threading
from collections import deque
import numpy as np

# --- Global variables for ToF data and control ---
latest_tof_values = [0, 0, 0, 0]
data_lock = threading.Lock() # Protects latest_tof_values
tof_data_buffer = [] # Buffer to store ToF data for CSV
recording_active = threading.Event() # Control recording
recording_active.set() # Start active for recording

# --- Callback function for ToF sensor data ---
def tof_data_handler(sub_info):
    global latest_tof_values, tof_data_buffer
    with data_lock:
        latest_tof_values = list(sub_info)
        # Record to buffer if recording is active
        if recording_active.is_set():
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
            tof_data_buffer.append([timestamp] + list(sub_info)) 
            # Optional: print to console for quick check, but can be verbose
            # print(f"Buffered ToF: {timestamp}, tof1={sub_info[0]}") 

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis
    ep_sensor = ep_robot.sensor

    # --- Configuration (from previous problem) ---
    MOVE_SPEED = 0.25 
    STOP_DISTANCE_CM = 40 
    STOP_DISTANCE_MM = STOP_DISTANCE_CM * 10

    # --- Filter settings ---
    # TECHNIQUE options: 'raw', 'moving_average', 'median_filter', 'ewma'
    # เปลี่ยนค่านี้เพื่อทดสอบแต่ละเทคนิค!
    TECHNIQUE = 'ewma' # <-- ตั้งค่าเป็น 'raw' เพื่อไม่ใช้ฟิลเตอร์

    # ขนาดของหน้าต่างสำหรับ Moving Average/Median Filter (ใช้เมื่อ TECHNIQUE ไม่ใช่ 'raw' หรือ 'ewma')
    WINDOW_SIZE = 10 

    # ค่า Alpha สำหรับ EWMA (0-1) (ใช้เมื่อ TECHNIQUE คือ 'ewma')
    ALPHA = 0.1 

    readings = deque(maxlen=WINDOW_SIZE)
    ewma_value = None

    # Create CSV filename with timestamp, indicating the filter technique
    filename = datetime.now().strftime(f"tof_data_{TECHNIQUE}.csv")

    print("กำลังเตรียมการเชื่อมต่อและเซ็นเซอร์...")

    try:
        # Start subscribing ToF data at 20 Hz
        ep_sensor.sub_distance(freq=20, callback=tof_data_handler)
        print("เริ่มการบันทึกข้อมูล ToF และเตรียมการเคลื่อนที่...")
        
        # Wait for initial data
        time.sleep(1) 

        print(f"\n--- เริ่มต้นการเคลื่อนที่และการบันทึกข้อมูล ---")
        print(f"หุ่นยนต์จะเคลื่อนที่ด้วยความเร็ว: {MOVE_SPEED} m/s")
        print(f"และจะสั่งหยุดเมื่อตรวจพบวัตถุที่ระยะน้อยกว่า: {STOP_DISTANCE_CM} cm ({STOP_DISTANCE_MM} mm)")
        print(f"ใช้เทคนิคฟิลเตอร์: {TECHNIQUE}")
        if TECHNIQUE == 'moving_average' or TECHNIQUE == 'median_filter':
            print(f"  - ขนาด Window: {WINDOW_SIZE}")
        elif TECHNIQUE == 'ewma':
            print(f"  - ค่า Alpha: {ALPHA}")

        ep_chassis.drive_speed(x=MOVE_SPEED, y=0, z=0)
        print("\nหุ่นยนต์เริ่มเคลื่อนที่...")

        # Flag to indicate if stop command was issued
        stop_command_issued = False
        t_detect = None
        t_stop_command = None

        while True:
            with data_lock:
                tof_distance = latest_tof_values[0]

            if tof_distance == 0:
                # print(">> ข้อมูล ToF ไม่ถูกต้อง (ค่า 0), กำลังรอข้อมูลใหม่...")
                time.sleep(0.01) # Short sleep to avoid busy-waiting
                continue

            # Apply selected filter or use raw data
            if TECHNIQUE == 'raw':
                filtered_distance = tof_distance
            else:
                readings.append(tof_distance)
                if TECHNIQUE == 'moving_average':
                    if len(readings) == WINDOW_SIZE:
                        filtered_distance = sum(readings) / len(readings)
                    else:
                        filtered_distance = tof_distance 
                elif TECHNIQUE == 'median_filter':
                    if len(readings) == WINDOW_SIZE:
                        filtered_distance = np.median(list(readings))
                    else:
                        filtered_distance = tof_distance
                elif TECHNIQUE == 'ewma':
                    if ewma_value is None:
                        ewma_value = tof_distance
                    else:
                        ewma_value = ALPHA * tof_distance + (1 - ALPHA) * ewma_value
                    filtered_distance = ewma_value
                else:
                    # Fallback to raw if TECHNIQUE is unrecognized but not 'raw'
                    filtered_distance = tof_distance 

            # Only print if not stopped yet, to reduce console spam
            if not stop_command_issued:
                print(f"ระยะจริง (Raw): {tof_distance} mm, ระยะหลังฟิลเตอร์ ({TECHNIQUE}): {filtered_distance:.2f} mm")

            # Check stop condition
            if filtered_distance < STOP_DISTANCE_MM and filtered_distance > 0 and not stop_command_issued:
                t_detect = time.time()
                ep_chassis.drive_speed(x=0, y=0, z=0)
                t_stop_command = time.time()
                response_time = t_stop_command - t_detect
                
                print("\n--- ตรวจพบวัตถุ ---")
                print(f"*** สั่งหยุดหุ่นยนต์เนื่องจากระยะหลังฟิลเตอร์ ({filtered_distance:.2f} mm) น้อยกว่า {STOP_DISTANCE_MM} mm ***")
                print(f"Response Time (เวลาตั้งแต่ตรวจพบจนถึงสั่งหยุดในโค้ด): {response_time:.6f} วินาที")
                print("\nกรุณาสังเกตและบันทึก 'ระยะหยุดจริง' ของหุ่นยนต์หลังจากการหยุดสนิท")
                stop_command_issued = True # Set flag to true to prevent repeated prints/commands

            # If stop command was issued, wait a bit for robot to actually stop and then exit loop
            if stop_command_issued:
                time.sleep(2) # Give robot some time to physically stop
                break # Exit loop

            time.sleep(0.05) # Normal loop delay

    except KeyboardInterrupt:
        print("\nผู้ใช้งานสั่งหยุดโปรแกรม (Ctrl+C)")
    except Exception as e:
        print(f"เกิดข้อผิดพลาด: {e}")
    finally:
        print("\n--- สิ้นสุดการทดลอง ---")
        # หยุดการบันทึกข้อมูล
        recording_active.clear()
        print("หยุดการบันทึกข้อมูล ToF แล้ว")

        print("กำลังยกเลิกการ Subscribe และปิดการเชื่อมต่อ...")
        ep_sensor.unsub_distance()
        ep_chassis.drive_speed(x=0, y=0, z=0) # Ensure robot stops
        ep_robot.close()
        
        # Write buffer to CSV file
        with open(filename, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(['timestamp', 'tof1', 'tof2', 'tof3', 'tof4'])
            csv_writer.writerows(tof_data_buffer)
        print(f"บันทึกข้อมูลการเคลื่อนที่ (TECHNIQUE: {TECHNIQUE}) ลงในไฟล์ '{filename}' เรียบร้อยแล้ว")
        print("ปิดการเชื่อมต่อ Robomaster แล้ว")