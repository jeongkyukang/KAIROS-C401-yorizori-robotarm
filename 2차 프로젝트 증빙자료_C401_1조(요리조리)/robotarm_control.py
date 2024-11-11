import cv2
import numpy as np
import threading
from pymycobot.mycobot import MyCobot
import time
from ultralytics import YOLO
import socket
import socketio

model = YOLO('best.pt')

# Socket.IO 클라이언트 생성
sio = socketio.Client()

try:
    sio.connect('http://172.30.1.14:5000')
    print("웹 서버에 연결되었습니다")
except Exception as e:
    print(f"서버 연결 실패: {e}")

# 명령어 매핑 추가
COMMANDS = {
    '1': '대기',
    '2': '블록추적',
    '3': '블록접근',
    '4': '블록잡기',
    '5': '블록들기',
    '6': '블록놓기'
}

# Socket.IO 이벤트 핸들러 추가
@sio.event
def connect():
    print("서버에 연결되었습니다.")

@sio.event
def disconnect():
    print("서버와의 연결이 끊어졌습니다.")

@sio.event
def connect_error(data):
    print(f"연결 오류: {data}")
    
def send_stage_message(stage, color):
    message = COMMANDS[str(stage)] + "," + color.strip()
    try:
        sio.emit('robot_message', message)
        print(f"단계 {stage} 메시지 전송: {message}")
    except Exception as e:
        print(f"메시지 전송 오류: {e}")

# tcp 통신
tcp_ip = '0.0.0.0'  # 서버 IP 주소 (0.0.0.0으로 설정하면 모든 네트워크 인터페이스에서 수신 가능)
tcp_port = 9999  # 서버 포트 번호
tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
tcp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
tcp_socket.bind((tcp_ip, tcp_port))  # 서버 IP와 포트 바인딩
tcp_socket.listen(1)  # 클라이언트 연결 대기

# cobot 관련 변수
z_min, z_max = 100, 380
qy_min, qy_max = -10, 20
xy_threshold = 5
cobot_speed = 30
cx, cy = 0, 0

# 색상 범위 설정 (HSV)
color_ranges = {
    "red": [((0, 80, 116), (10, 255, 255)), ((170, 80, 116), (180, 255, 255))],
    "yellow" : [(18, 114, 94), (35, 255, 255)],
    "navy": [(100, 106, 36), (121, 255, 255)],
    "purple": [(121, 40, 64), (161, 255, 255)],
}

# 물체의 실제 너비 (단위: mm)
actual_object_width_mm = 25
# Camera calibration parameters
camera_matrix = np.array([[561.20581683, 0., 353.85748295],
                          [0., 564.8824937, 221.57289288],
                          [0., 0., 1.]]
                        )
dist_coeffs = np.array([[0.23935691, -0.71647137, -0.01530787, 0.01102398, 0.39847327]])

# MyCobot 초기 설정
mycobot = MyCobot('COM3', 115200)
initial_angles = [98.17, 13.0, -46.23, -53.96, 91.93, 4.83]
mycobot.send_angles(initial_angles, 20)
time.sleep(3)
mycobot.set_gripper_mode(0)
mycobot.init_eletric_gripper()
gripper_state = False # 그리퍼 닫힘
mycobot.set_gripper_state(0,30)
time.sleep(2)

# 전역 변수 및 이벤트 설정
frame = None
frame_ready = threading.Event()
stacking_flag = False
distance_to_object = None
camera_gripper_distance = 25
blocks_cnt = {"red": 0, "yellow": 0, "navy": 0, "purple": 0}
loading_position = {"red": [20, -19.42, -66.35, -0.35, 90, 90],  #[-69.68, -45, -27, -13, 90, 0]
                    "yellow": [0, -19.42, -66.35, -0.35, 90, 90],#[-34.68, -45, -27, -13, 90, 0]
                    "navy": [-20, -19.42, -66.35, -0.35, 90, 90],#[10.32, -45, -27, -13, 90, 0]
                    "purple": [147.56, -11.33, -25.57, -46.66, 92.46, 13.35] 
                    }
loading_position_coords = {"red": [277.6, 7.1, 248.4, 179.85, 3.07, -159.6],  #[-69.68, -45, -27, -13, 90, 0]
                            "yellow": [263.1, -87.6, 246.7, 179.94, 2.63, -179.38],#[-34.68, -45, -27, -13, 90, 0]
                            "navy": [216.5, -171.8, 249.8, 179.94, 3.25, 160.66],#[10.32, -45, -27, -13, 90, 0]
                    }

def send_angles_periodically():
    print(f"서버가 {tcp_ip}:{tcp_port}에서 대기 중입니다...")

    while True:  # Keep the server running
        try:
            client_socket, client_address = tcp_socket.accept()
            print(f"클라이언트 {client_address}와 연결됨")
            # Wait for a client connection

            while True:
                try:
                    angles = mycobot.get_angles()  # Get the current robot angles
                    if gripper_state:  # Gripper is open
                        angles += [0.935]
                    else:
                        angles += [0.0]
                    angles_str = ",".join(map(str, angles))  # Convert to string
                    client_socket.sendall(angles_str.encode())
                    time.sleep(0.1)  # Send every 0.01 seconds

                except Exception as e:
                    print(f"각도값 전송 오류: {e}")
                    # client_socket.close()
                    # break  # Exit the inner loop to wait for a new connection

        except Exception as e:
            print(f"클라이언트 수신 오류: {e}")
            break  # Stop the server if a binding or other major error occurs
    tcp_socket.close()
    print("서버 종료")


def detect_block_hsv():
    global frame
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    largest_contour = None
    for color, ranges in color_ranges.items():
        mask = None
        if color == "red":
            mask1 = cv2.inRange(hsv, ranges[0][0], ranges[0][1])
            mask2 = cv2.inRange(hsv, ranges[1][0], ranges[1][1])
            mask = mask1 | mask2
        else:
            mask = cv2.inRange(hsv, ranges[0], ranges[1])
        mask = cv2.erode(mask, None, iterations=1) # 노이즈 제거 및 경계선 축소
        mask = cv2.dilate(mask, None, iterations=1) # 경계선을 복구하여 객체 형태 유지
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            current_largest_contour = max(contours, key=cv2.contourArea)
            if largest_contour is None or cv2.contourArea(current_largest_contour) > cv2.contourArea(largest_contour) and cv2.contourArea(current_largest_contour) > 2000:
                largest_contour = current_largest_contour
                largest_contour_color = color
    if largest_contour is not None:
        x, y, w, h = cv2.boundingRect(largest_contour)
        cx, cy = x + w // 2, y + h // 2
        rect = cv2.minAreaRect(largest_contour)
        return (cx, cy, w, rect[2], largest_contour_color)
    return None

def detect_block_yolo():
    global cx, cy
    results = model.predict(frame, stream=True)
    
    # 검출된 결과에서 가장 큰 영역의 객체를 찾기 위해 최대 면적 및 관련 정보 초기화
    full_max_area = 2000
    top_max_area = 2000
    full_largest_box = None
    top_largest_box = None
    largest_class_name = ""
    
    # 각 객체의 바운딩 박스를 순회하며 면적 계산
    full_or_top = True
    for result in results:
        for box in result.boxes:
            # 클래스 ID 추출 (텐서에서 정수로 변환)
            class_id = int(box.cls[0].item())
            # 클래스 이름 정의
            if class_id == 0:
                class_name = "navy"
            elif class_id == 1:
                class_name = "navy"
            elif class_id == 2:
                class_name = "purple"
            elif class_id == 3:
                class_name = "purple"
            elif class_id == 4:
                class_name = "red"
            elif class_id == 5:
                class_name = "red"
            elif class_id == 6:
                class_name = "yellow"
            elif class_id == 7:
                class_name = "yellow"
            else:
                continue
            
            if class_id == 0 or class_id == 2 or class_id == 4 or class_id == 6:
                full_or_top = True
            else:
                full_or_top = False
                
            # 바운딩 박스 좌표 추출 및 면적 계산
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            area = (x2 - x1) * (y2 - y1)
            
            # 가장 큰 면적의 바운딩 박스를 업데이트
            if full_or_top:
                if area > full_max_area:
                    full_max_area = area
                    full_largest_box = (x1, y1, x2, y2)
                    largest_class_name = class_name
            else:
                if area > top_max_area:
                    top_max_area = area
                    top_largest_box = (x1, y1, x2, y2)
                    largest_class_name = class_name

    # 가장 큰 영역의 바운딩 박스가 있을 경우 표시
    if top_largest_box:
        x1, y1, x2, y2 = top_largest_box
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        return (cx, cy, x2 - x1, largest_class_name)
    elif full_largest_box:
        x1, y1, x2, y2 = full_largest_box
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        return (cx, cy, x2 - x1, largest_class_name)
    return None

# 블록과 그리퍼 간 거리(높이) 계산 함수
def calculate_distance_to_object(width_in_pixels):
    focal_length = camera_matrix[0, 0]  # Focal length in pixels
    distance = (actual_object_width_mm * focal_length) / width_in_pixels
    return distance

# 물체 인식 및 mycobot 제어 함수 (스레드로 실행)
def robot_arm_control():
    global stacking_flag, gripper_state
    while True:
        frame_ready.wait()  # 메인 함수에서 새 프레임이 준비될 때까지 대기
        frame_ready.clear()
        if frame is None:
            continue
        detect_result_yolo = detect_block_yolo()
        if detect_result_yolo:
            cx, cy, pixel_width, color = detect_result_yolo
            send_stage_message(2, color)
            detect_result = detect_block_hsv()
            if detect_result:
                _, _, _, angle, _ = detect_result
                # 이동할 위치 계산(픽셀 -> mm 단위)
                mm_per_pixel = actual_object_width_mm / pixel_width
                move_x = (cx - frame_width // 2) * mm_per_pixel
                move_y = (cy - frame_height // 2) * mm_per_pixel + camera_gripper_distance + 3 # 카메라, 그리퍼 위치 오차 반영
                # 작동 범위 제한
                coords = mycobot.get_coords()
                print(coords)
                new_z = max(z_min, min(z_max, coords[2]))
                # new_qy = max(qy_min, min(qy_max, coords[4]))
                print(f"move_x: {move_x}, move_y: {move_y}")

                if abs(move_x) > xy_threshold:
                    mycobot.send_coords([coords[0] - move_x, coords[1], new_z, coords[3], coords[4], coords[5]], 20, 1)
                elif abs(move_y) > 3:
                    mycobot.send_coords([coords[0], coords[1] + move_y, new_z, coords[3], coords[4], coords[5]], 20, 1)
                else:
                    print("center~~~")
                    time.sleep(1)
                    # 정규(그리퍼 회전)
                    if angle < 45:
                        angle = angle - 90
                    if abs(angle) > 5:
                        angles = mycobot.get_angles()
                        # 범위 벗어날 때 반대 방향으로 회전
                        new_qz = angles[5] + angle
                        if new_qz > 175:
                            new_qz -= 180
                        elif new_qz < -175:
                            new_qz += 180
                        print("현재 각도: ", angles[5], "계산한 각도 : ", angle,  "이동할 각도: ", new_qz)
                        mycobot.send_angles([angles[0], angles[1], angles[2], angles[3], angles[4], new_qz], 40)
                        time.sleep(2)
                        
                    #그리퍼 정중앙에 블록 위치하게 x, y값 이동
                    coords = mycobot.get_coords()
                    print(coords)
                    mycobot.send_coords([coords[0], coords[1] - camera_gripper_distance + 10, coords[2], coords[3], coords[4], coords[5]], 40, 1)
                    time.sleep(2)
                    
                    detect_result_yolo = detect_block_yolo()
                    if detect_result_yolo:
                        cx, cy, pixel_width, _ = detect_result_yolo
                        mm_per_pixel = actual_object_width_mm / pixel_width
                    move_x = (cy - frame_height // 2) * mm_per_pixel + camera_gripper_distance + 5
                    coords = mycobot.get_coords()
                    print(coords)
                    if abs(move_x) > xy_threshold:
                        if angle > 0:
                            mycobot.send_coords([coords[0] + move_x + 7, coords[1], coords[2], coords[3], coords[4], coords[5]], 40, 1)
                        else :
                            mycobot.send_coords([coords[0] - move_x - 3, coords[1], coords[2], coords[3], coords[4], coords[5]], 40, 1)
                        time.sleep(2)
                        
                    # 우석(실제 거리만큼 내려가기)
                    coords = mycobot.get_coords()
                    print(coords)
                    distance_to_object = calculate_distance_to_object(pixel_width)
                    mycobot.send_coords([coords[0], coords[1], coords[2] - distance_to_object + 45, coords[3], coords[4], coords[5]], 30, 1)
                    time.sleep(2)
                    
                    print("물건 집으러 내려가기")
                    send_stage_message(3, color)
                    # 그리퍼 닫기
                    gripper_state = True
                    mycobot.set_gripper_state(1,80)
                    time.sleep(2)
                    print("그리퍼 닫기")
                    send_stage_message(4, color)
                    
                    # 올라가기
                    mycobot.send_angles(initial_angles, 70)
                    time.sleep(2)
                    print("올라가기")
                    send_stage_message(5, color)
                    # 적재 위치로 이동
                    mycobot.send_angles(loading_position[color], 80)
                    print("적재 위치로 이동")
                    time.sleep(3)
                    gripper_state = False
                    if color == "purple": # 불량품일 경우
                        # 그리퍼 열기
                        mycobot.set_gripper_state(0,50)
                        send_stage_message(6, color)
                        time.sleep(1)
                    else:
                        # 적재 위치에서 내려가기
                        coords = loading_position_coords[color]
                        mycobot.send_coords([coords[0], coords[1], coords[2] - 105 + (27 * blocks_cnt[color]), coords[3], coords[4], coords[5]], 20, 1)
                        time.sleep(3)
                        # 그리퍼 열기
                        mycobot.set_gripper_state(0,70)
                        time.sleep(1)
                        blocks_cnt[color] += 1
                        # 적재 위치 위로 이동
                        mycobot.send_angles(loading_position[color], 80)
                        print("적재 위치 위로 이동")
                        send_stage_message(6, color)
                        time.sleep(2)
                        
                    # 원 위치로 복귀
                    mycobot.send_angles(initial_angles, 80)
                    time.sleep(2)
                    print("원위치 이동")
            time.sleep(1.5)

# TCP 통신 스레드 시작(각도 전송)
tcp_thread = threading.Thread(target=send_angles_periodically, daemon=True)
tcp_thread.start()
print("tcp 스타트")

# MyCobot 제어 스레드 시작
control_thread = threading.Thread(target=robot_arm_control, daemon=True)
control_thread.start()

# 메인 함수: 카메라 읽기 및 프레임 표시
cap = cv2.VideoCapture(0)
frame_width = 640
frame_height = 480
cap.set(3, frame_width)
cap.set(4, frame_height)

while cap.isOpened():

    ret, frame = cap.read()
    if not ret:
        break
    # 제어 스레드에 새 프레임이 준비되었음을 알림
    frame_ready.set()

    cv2.line(frame, (frame_width // 2 - 10, frame_height // 2), (frame_width // 2 + 10, frame_height // 2), (0, 0, 0), 2)  # 수직선
    cv2.line(frame, (frame_width // 2, frame_height // 2 - 10), (frame_width // 2, frame_height // 2 + 10), (0, 0, 0), 2)  # 수평선
    cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
    
    cv2.imshow("Camera Feed", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        if sio.connected:
            sio.disconnect()
        break
cap.release()
cv2.destroyAllWindows()