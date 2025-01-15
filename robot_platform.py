#!/usr/bin/env python
import cv2
import threading
import time
import json
import paho.mqtt.client as mqtt

from flask import Flask, render_template, jsonify, request, Response
from pyardrone import ARDroneBase, HelperMixin, at
import pytemi as temi  # 確保 pytemi 模組正確實現了 Robot 類別和連接方法

app = Flask(__name__)

#######################################
# ========== 全域變數定義區 ========== #
#######################################

# ========== Drone 控制相關 (ARDrone) ========== #
class MyARDrone(HelperMixin, ARDroneBase):
    pass

drone = MyARDrone()

navdata = {}

# PID 控制參數（根據需要調整）
Kp_altitude = 0.1
Ki_altitude = 0.01
Kd_altitude = 0.02

Kp_pitch = 0.1
Ki_pitch = 0.01
Kd_pitch = 0.02

Kp_roll = 0.1
Ki_roll = 0.01
Kd_roll = 0.02

integral_altitude = 0
previous_error_altitude = 0

integral_pitch = 0
previous_error_pitch = 0

integral_roll = 0
previous_error_roll = 0

camera_lock = threading.Lock()
cam = None
camera_open = False

# ========== 是否啟用監控 (全域變數) ========== #
monitoring_enabled = False  # True 表示開啟監控, False 表示關閉監控

# ========== Ground Robot 控制相關 ========== #
TEMI_SERIAL = "00120495065"
BATTERY_THRESHOLD_LOW = 20  # [%]
BATTERY_THRESHOLD_CHARGED = 90  # [%]

# MQTT 伺服器參數
MQTT_HOST = 'stevetw.serv00.net'      # 替換為您的 MQTT Broker 地址
MQTT_PORT = 1883
MQTT_USERNAME = 'steve'               # 替換為您的 MQTT 使用者名稱
MQTT_PASSWORD = '062028633'           # 替換為您的 MQTT 密碼

# 地面機器人 IP 攝影機 URL (您可自行修改)
GROUND_CAM_URL = 'http://100.97.133.117:8080/video' # 範例

# 機器人狀態
robot_status = {
    "battery_percentage": None,
    "waypoint_list": [],
    "asr_result": ""
}

# 創建 MQTT 客戶端並連接
mqtt_client = temi.connect(MQTT_HOST, MQTT_PORT, MQTT_USERNAME, MQTT_PASSWORD)
robot = temi.Robot(mqtt_client, TEMI_SERIAL)

# 啟動 MQTT 循環 (處理訊息)
mqtt_client.loop_start()

# 建立鎖以保護 robot_status
robot_status_lock = threading.Lock()

#########################################
# ========== 回調函數定義區 ========== #
#########################################

def _on_status(client, userdata, message):
    payload = message.payload.decode('utf-8')
    data = json.loads(payload)
    # 不再印出 "Status Updated: {...}"
    with robot_status_lock:
        # 將 waypoint_list 寫入 global robot_status
        robot_status['waypoint_list'] = data.get("waypoint_list", [])

def _on_battery(client, userdata, message):
    payload = message.payload.decode('utf-8')
    data = json.loads(payload)
    # 不再印出 "Battery Updated: {...}"
    with robot_status_lock:
        robot_status['battery_percentage'] = data.get("battery_percentage", None)

def _on_asr(client, userdata, message):
    payload = message.payload.decode('utf-8')
    data = json.loads(payload)
    # 不再印出 "ASR Result Updated: {...}"
    with robot_status_lock:
        robot_status['asr_result'] = data.get("asr_result", "")

# 綁定 MQTT 回調
robot.client.message_callback_add(f"temi/{TEMI_SERIAL}/status/info", _on_status)
robot.client.message_callback_add(f"temi/{TEMI_SERIAL}/status/utils/battery", _on_battery)
robot.client.message_callback_add(f"temi/{TEMI_SERIAL}/status/result", _on_asr)

#########################################
# ========== 相機初始化函式 ========== #
#########################################
def initialize_camera():
    global cam, camera_open
    with camera_lock:
        try:
            if cam is None or not cam.isOpened():
                # 修改為無人機相機串流位址，若無實際相機可先關閉
                cam = cv2.VideoCapture('tcp://192.168.1.1:5555')
                if not cam.isOpened():
                    print("Error: Could not open drone video stream.")
                    return False
                camera_open = True
        except cv2.error as e:
            print(f"OpenCV error during camera initialization: {e}")
            return False
    return True

#########################################
# ========== Navdata 函式 ========== #
#########################################
def get_navdata_attribute(drone, attribute, max_attempts=5, delay=1):
    for attempt in range(max_attempts):
        try:
            return getattr(drone.navdata, attribute)
        except AttributeError as e:
            if attempt < max_attempts - 1:
                time.sleep(delay)
            else:
                raise

def update_navdata():
    global navdata
    drone.navdata_ready.wait()
    drone.send(at.CONFIG('general:navdata_demo', True))
    drone.send(at.CONFIG('general:navdata_raw_measures', True))
    drone.send(at.CONFIG('general:navdata_options', 5))

    while True:
        if monitoring_enabled:  # 只有在監控開啟時才更新 navdata
            try:
                demo_data = get_navdata_attribute(drone, 'demo')
                raw_measures = get_navdata_attribute(drone, 'raw_measures')

                navdata = {
                    'Battery': demo_data.vbat_flying_percentage,
                    'theta': demo_data.theta,
                    'phi': demo_data.phi,
                    'psi': demo_data.psi,
                    'altitude': demo_data.altitude,
                    'vx': demo_data.vx,
                    'vy': demo_data.vy,
                    'vz': demo_data.vz,
                    'raw_accs': raw_measures.raw_accs.tolist(),
                    'raw_gyros': raw_measures.raw_gyros.tolist(),
                    'raw_gyros_110': raw_measures.raw_gyros_110.tolist(),
                }
            except AttributeError:
                pass

        time.sleep(0.1)

############################################
# ========== 無人機穩定控制函式 ========== #
############################################
def stabilize_drone():
    global integral_altitude, previous_error_altitude
    global integral_pitch, previous_error_pitch
    global integral_roll, previous_error_roll

    # 假設 navdata 中有 altitude 等資訊
    target_altitude = navdata.get('altitude', 0)
    target_pitch = 0
    target_roll = 0

    while monitoring_enabled:
        current_altitude = navdata.get('altitude', 0)
        current_pitch = navdata.get('theta', 0)
        current_roll = navdata.get('phi', 0)

        error_altitude = target_altitude - current_altitude
        integral_altitude += error_altitude
        derivative_altitude = error_altitude - previous_error_altitude
        output_altitude = (Kp_altitude * error_altitude +
                           Ki_altitude * integral_altitude +
                           Kd_altitude * derivative_altitude)

        error_pitch = target_pitch - current_pitch
        integral_pitch += error_pitch
        derivative_pitch = error_pitch - previous_error_pitch
        output_pitch = (Kp_pitch * error_pitch +
                        Ki_pitch * integral_pitch +
                        Kd_pitch * derivative_pitch)

        error_roll = target_roll - current_roll
        integral_roll += error_roll
        derivative_roll = error_roll - previous_error_roll
        output_roll = (Kp_roll * error_roll +
                       Ki_roll * integral_roll +
                       Kd_roll * derivative_roll)

        drone.move(
            up=max(0, min(output_altitude, 1)) if output_altitude > 0 else 0,
            down=max(0, min(-output_altitude, 1)) if output_altitude < 0 else 0,
            forward=max(0, min(output_pitch, 1)) if output_pitch > 0 else 0,
            backward=max(0, min(-output_pitch, 1)) if output_pitch < 0 else 0,
            left=max(0, min(output_roll, 1)) if output_roll > 0 else 0,
            right=max(0, min(-output_roll, 1)) if output_roll < 0 else 0
        )

        previous_error_altitude = error_altitude
        previous_error_pitch = error_pitch
        previous_error_roll = error_roll

        time.sleep(0.1)

############################################
# ========== Drone Camera 串流函式 =========#
############################################
def gen_drone_frames():
    global cam
    while True:
        with camera_lock:
            if cam is None or not cam.isOpened():
                break
            ret, frame = cam.read()
            if not ret:
                break

            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                break
            frame_bytes = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

############################################
# ========== Ground Robot IP Camera =======#
############################################
ground_cam_lock = threading.Lock()
ground_cam = None
ground_cam_open = False

def initialize_ground_camera():
    global ground_cam, ground_cam_open
    with ground_cam_lock:
        try:
            if ground_cam is None or not ground_cam.isOpened():
                ground_cam = cv2.VideoCapture(GROUND_CAM_URL)
                if not ground_cam.isOpened():
                    print("Error: Could not open ground robot IP camera.")
                    return False
                ground_cam_open = True
        except cv2.error as e:
            print(f"OpenCV error during ground camera initialization: {e}")
            return False
    return True

def gen_ground_frames():
    global ground_cam
    while True:
        with ground_cam_lock:
            if ground_cam is None or not ground_cam.isOpened():
                break
            ret, frame = ground_cam.read()
            if not ret:
                break

            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                break
            frame_bytes = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

###########################################
# ========== Flask 路由與主程式 ========== #
###########################################
@app.route('/')
def index():
    # 您可以將前端模板改成 'index.html' 或 'index1.html'，請自行確認名稱
    return render_template('index1.html')

# ======== Drone Navdata ======== #
@app.route('/navdata', methods=['GET'])
def get_navdata_route():
    global navdata, monitoring_enabled
    if not monitoring_enabled:
        return jsonify({"message": "Monitoring is off"})
    else:
        return jsonify(navdata)

# ======== Drone 相機串流 ======== #
@app.route('/video_feed')
def video_feed():
    return Response(gen_drone_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# ======== Ground Robot 相機串流 ======== #
@app.route('/ground_video_feed')
def ground_video_feed():
    return Response(gen_ground_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# ======== Drone 指令 ======== #
@app.route('/drone_command', methods=['POST'])
def drone_command():
    global integral_altitude, previous_error_altitude
    global integral_pitch, previous_error_pitch
    global integral_roll, previous_error_roll
    global camera_open, monitoring_enabled

    data = request.json
    command = data.get('command', '').lower()
    distance = data.get('distance', 0.3)
    angle = data.get('angle', 90)
    text = data.get('text', '')

    if not command:
        return jsonify({"status": "error", "message": "No command provided"}), 400

    if command == 'takeoff':
        drone.takeoff()
    elif command == 'land':
        drone.land()
        # 重置積分器
        integral_altitude = integral_pitch = integral_roll = 0
        previous_error_altitude = previous_error_pitch = previous_error_roll = 0
    elif command == 'hover':
        drone.hover()
    elif command == 'up':
        drone.move(up=0.3)
    elif command == 'down':
        drone.move(down=0.3)
    elif command == 'forward':
        drone.move(forward=distance)
    elif command == 'backward':
        drone.move(backward=distance)
    elif command == 'left':
        robot.turn_by(-angle)
    elif command == 'right':
        robot.turn_by(angle)
    elif command == 'asr':
        # 自訂語音
        robot.asr(text)
    elif command == 'rotate':
        # 可客製化角度
        robot.turn_by(angle)
    elif command == 'emergency':
        drone.emergency()
    elif command == 'stabilize':
        if not monitoring_enabled:
            monitoring_enabled = True
        stabilize_thread = threading.Thread(target=stabilize_drone, daemon=True)
        stabilize_thread.start()
    elif command == 'open_camera':
        if not initialize_camera():
            pass  # 相機初始化失敗
    elif command == 'close_camera':
        with camera_lock:
            if cam is not None:
                cam.release()
            camera_open = False
    elif command == 'open_monitoring':
        monitoring_enabled = True
    elif command == 'close_monitoring':
        monitoring_enabled = False
    else:
        return jsonify({"status": "error", "message": "Invalid command"}), 400

    return jsonify({"status": "success"}), 200

# ======== Ground Robot 狀態 ======== #
@app.route('/ground_robot_status', methods=['GET'])
def get_ground_robot_status():
    with robot_status_lock:
        return jsonify(robot_status)

# ======== Ground Robot 指令 ======== #
@app.route('/ground_robot_command', methods=['POST'])
def ground_robot_command():
    data = request.json
    command = data.get('command', '').lower()
    parameters = data.get('parameters', {})

    if not command:
        return jsonify({"status": "error", "message": "No command provided"}), 400

    if hasattr(robot, command):
        method = getattr(robot, command)
        try:
            method(**parameters)
            # 不再輸出 "Ground robot command sent: ..."
            return jsonify({"status": "success"}), 200
        except TypeError as e:
            return jsonify({"status": "error", "message": str(e)}), 400
        except Exception as e:
            return jsonify({"status": "error", "message": str(e)}), 500
    else:
        return jsonify({"status": "error", "message": "Invalid command"}), 400

# ======== Ground Robot 開啟/關閉攝影機 ======== #
@app.route('/ground_open_camera', methods=['POST'])
def ground_open_camera():
    if not initialize_ground_camera():
        return jsonify({"status": "error", "message": "Failed to open ground camera"}), 500
    return jsonify({"status": "success"}), 200

@app.route('/ground_close_camera', methods=['POST'])
def ground_close_camera():
    global ground_cam_open, ground_cam
    with ground_cam_lock:
        if ground_cam is not None:
            ground_cam.release()
        ground_cam_open = False
    return jsonify({"status": "success"}), 200

##########################################
# ========== 初始化與啟動程式 =========== #
##########################################
def run_flask_app():
    # 您可以在 production 中使用 gunicorn 或其他 WSGI server
    app.run(host='0.0.0.0', port=10000, threaded=True)

if __name__ == '__main__':
    # 預先初始化相機 (Drone)
    initialize_camera()

    # 預先初始化地面機器人 IP 相機 (若需要)
    initialize_ground_camera()

    # 啟動 Navdata 更新線程
    navdata_thread = threading.Thread(target=update_navdata, daemon=True)
    navdata_thread.start()

    # 啟動 Flask 應用
    flask_thread = threading.Thread(target=run_flask_app, daemon=True)
    flask_thread.start()

    # 主線程保持運行
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()