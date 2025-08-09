import time
import cv2
import csv
import datetime
import robomaster
from robomaster import robot, vision, gimbal, blaster, camera

class PIDController:
    """A simple PID controller."""
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()

    def update(self, current_value):
        """Calculates the PID control output."""
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0:
            return 0
        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        self.last_time = current_time
        return output

# Global list to store processed marker objects
g_processed_markers = []

class MarkerObject:
    """A class to hold processed marker information for drawing."""
    def __init__(self, x, y, w, h, info):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        # คำนวณพิกัดสำหรับวาดบนภาพขนาด 1280x720
        self.pt1 = (int((x - w / 2) * 1280), int((y - h / 2) * 720))
        self.pt2 = (int((x + w / 2) * 1280), int((y + h / 2) * 720))
        self.center = (int(x * 1280), int(y * 720))
        self.text = info

def on_detect_marker(marker_info):
    """Callback function that updates the global list of detected markers."""
    global g_processed_markers
    g_processed_markers.clear()
    for m in marker_info:
        g_processed_markers.append(MarkerObject(*m))

# ===== CSV Logging System =====
CSV_FILENAME = "robomaster_laser.csv"

# สร้างไฟล์ CSV พร้อม header
with open(CSV_FILENAME, mode="w", newline="", encoding="utf-8") as f:
    writer = csv.writer(f)
    writer.writerow(["timestamp", "step", "marker_id", "x", "y", "error_x", "error_y", "status"])

def log_event(step, marker_id, x, y, error_x, error_y, status):
    """บันทึกเหตุการณ์ลง CSV"""
    timestamp = datetime.datetime.now().isoformat()
    with open(CSV_FILENAME, mode="a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([timestamp, step, marker_id, x, y, error_x, error_y, status])

# ===== Main Program =====
if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_robot.camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    ep_robot.vision.sub_detect_info(name="marker", callback=on_detect_marker)
    ep_robot.gimbal.recenter().wait_for_completed()

    print("กำลังค้นหา Markers... กรุณาจัดวาง Marker ทั้ง 3 จุดให้อยู่ในมุมกล้อง")
    time.sleep(3)  # รอให้ตรวจจับครบ

    initial_markers = list(g_processed_markers)

    if len(initial_markers) == 3:
        initial_markers.sort(key=lambda m: m.x)
        
        left_marker = initial_markers[0]
        center_marker = initial_markers[1]
        right_marker = initial_markers[2]
        
        target_sequence = [
            left_marker, 
            center_marker, 
            right_marker, 
            center_marker, 
            left_marker
        ]
        
        print("ตรวจพบ Marker 3 จุด! เริ่มลำดับการยิง: ซ้าย -> กลาง -> ขวา -> กลาง -> ซ้าย")

        for i, target_marker in enumerate(target_sequence):
            target_name = f"Marker '{target_marker.text}' (ขั้นตอนที่ {i+1}/{len(target_sequence)})"
            print(f"\n--- เริ่มเล็งเป้าหมาย: {target_name.upper()} ---")
            
            pid_yaw = PIDController(kp=45, ki=0.02, kd=25, setpoint=0.5)
            pid_pitch = PIDController(kp=45, ki=0, kd=25, setpoint=0.5)
            
            locked = False
            LOCK_THRESHOLD = 0.02

            for _ in range(500):
                img = ep_robot.camera.read_cv2_image(strategy="newest", timeout=0.5)
                current_target_info = next((m for m in g_processed_markers if m.text == target_marker.text), None)
                
                if img is not None:
                    for marker in g_processed_markers:
                        color = (0, 0, 255) if current_target_info and marker.text == current_target_info.text else (0, 128, 0)
                        thickness = 4 if current_target_info and marker.text == current_target_info.text else 2
                        cv2.rectangle(img, marker.pt1, marker.pt2, color, thickness)
                        cv2.putText(img, marker.text, marker.center, cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
                    
                    cv2.imshow("Robomaster Camera Feed", img)
                    cv2.waitKey(1)

                if not current_target_info:
                    ep_robot.gimbal.drive_speed(0, 0)
                    print(f"ไม่พบเป้าหมาย {target_name} ในเฟรมปัจจุบัน...")
                    log_event(f"{i+1}/{len(target_sequence)}", None, None, None, None, None, "NOT_FOUND")
                    time.sleep(0.1)
                    continue

                yaw_speed = -pid_yaw.update(current_target_info.x)
                pitch_speed = pid_pitch.update(current_target_info.y)
                ep_robot.gimbal.drive_speed(yaw_speed=yaw_speed, pitch_speed=pitch_speed)
                
                error_x = current_target_info.x - pid_yaw.setpoint
                error_y = current_target_info.y - pid_pitch.setpoint
                print(f"Targeting {target_name}: X_err={error_x:.3f}, Y_err={error_y:.3f}")

                log_event(
                    step=f"{i+1}/{len(target_sequence)}",
                    marker_id=current_target_info.text,
                    x=current_target_info.x,
                    y=current_target_info.y,
                    error_x=error_x,
                    error_y=error_y,
                    status="LOCKED" if abs(error_x) < LOCK_THRESHOLD and abs(error_y) < LOCK_THRESHOLD else "TRACKING"
                )

                if abs(error_x) < LOCK_THRESHOLD and abs(error_y) < LOCK_THRESHOLD:
                    print(f"เป้าหมาย {target_name.upper()} ล็อคแล้ว! 💥 ทำการยิง")
                    ep_robot.gimbal.drive_speed(0, 0)
                    ep_robot.blaster.fire(fire_type=blaster.INFRARED_FIRE, times=1)
                    log_event(f"{i+1}/{len(target_sequence)}", current_target_info.text,
                              current_target_info.x, current_target_info.y, error_x, error_y, "FIRED")
                    time.sleep(2)
                    locked = True
                    break
                
                time.sleep(0.01)

            if not locked:
                print(f"ไม่สามารถล็อคเป้าหมาย {target_name.upper()} ได้ทันเวลา")
                ep_robot.gimbal.drive_speed(0, 0)
                log_event(f"{i+1}/{len(target_sequence)}", target_marker.text, None, None, None, None, "TIMEOUT")
                time.sleep(1)
    
    else:
        print(f"ข้อผิดพลาด: ตรวจพบ Marker {len(initial_markers)} จุด แต่ต้องการ 3 จุดพอดี")
        print("กรุณาจัดวาง Marker 3 จุด แล้วลองใหม่อีกครั้ง")

    print("\n--- ลำดับการทำงานเสร็จสิ้น ---")
    cv2.destroyAllWindows()
    ep_robot.vision.unsub_detect_info(name="marker")
    ep_robot.camera.stop_video_stream()
    ep_robot.close()
