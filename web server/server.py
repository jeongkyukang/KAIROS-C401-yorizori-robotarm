from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import json
import os
from threading import Lock  # Lock import 추가

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, cors_allowed_origins="*")

# 설정 파일 경로
SETTINGS_FILE = 'robot_settings.json'

# 명령어 매핑
COMMANDS = {
    '1': '대기',
    '2': '블록추적',
    '3': '블록접근',
    '4': '블록잡기',
    '5': '블록들기',
    '6': '블록놓기'
}

# 블록 상태 관리를 위한 Lock 추가
block_count_lock = Lock()

# 블록 상태 관리
block_colors = ["red", "yellow", "navy", "purple"]
block_count = {"red": 1, "yellow": 5, "navy": 9, "purple": 0}
current_block = 0
blocks_filled = False
work_count = 0

# 기본 로봇 설정
default_robot_settings = {
    "1": {"width": 180, "height": 300, "x": 150, "y": 50},
    "2": {"width": 180, "height": 300, "x": 150, "y": 50},
    "3": {"width": 180, "height": 300, "x": 150, "y": 50},
    "4": {"width": 180, "height": 300, "x": 150, "y": 50},
    "5": {"width": 180, "height": 300, "x": 150, "y": 50},
    "6": {"width": 180, "height": 300, "x": 150, "y": 50}  # 추가
}

def load_settings():
    try:
        if os.path.exists(SETTINGS_FILE):
            with open(SETTINGS_FILE, 'r') as f:
                settings = json.load(f)
            # 기본 설정과 병합하여 누락된 로봇 설정 추가
            for key, default in default_robot_settings.items():
                if key not in settings:
                    settings[key] = default
            return settings
    except Exception as e:
        print(f"설정 로드 중 오류 발생: {e}")
    return default_robot_settings

def save_settings(settings):
    try:
        with open(SETTINGS_FILE, 'w') as f:
            json.dump(settings, f, indent=4)
    except Exception as e:
        print(f"설정 저장 중 오류 발생: {e}")

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('connect')
def handle_connect():
    print("웹 클라이언트가 연결되었습니다.")
    # 클라이언트 연결 시 저장된 설정 전송
    settings = load_settings()
    emit('load_settings', settings)
    # 현재 작업량 전송
    emit('update_work_count', {'count': work_count})

@socketio.on('save_settings')
def handle_save_settings(settings):
    print("설정 저장 요청을 받았습니다.")
    save_settings(settings)

# 전역 변수 추가 (파일 상단에)
defect_count = 0  # 여기로 이동

@socketio.on('robot_message')
def handle_robot_message(message):
    global current_block, work_count, blocks_filled, defect_count
    
    print(f"받은 명령어: '{message}'")
    message_parts = message.strip().split(',')
    if len(message_parts) >= 2:
        command = message_parts[0].strip()
        color = message_parts[1].strip()
    else:
        command = message.strip()
        color = ''
    
    print(f"명령어: '{command}', 색상: '{color}'")

    # 명령어에 따른 로봇 이미지 변경
    if command == '대기':
        emit('change_robot_image_specific', {'key': '1'}, broadcast=True)
    elif command == '블록추적':
        emit('change_robot_image_specific', {'key': '2'}, broadcast=True)
    elif command == '블록접근':
        emit('change_robot_image_specific', {'key': '3'}, broadcast=True)
    elif command == '블록잡기':
        emit('change_robot_image_specific', {'key': '4'}, broadcast=True)
    elif command == '블록들기':
        emit('change_robot_image_specific', {'key': '5'}, broadcast=True)
    elif command == '블록놓기':
        emit('change_robot_image_specific', {'key': '6'}, broadcast=True)
        
        if color == "purple":  # 불량품 블록 처리
            if defect_count < 4:
                defect_count += 1
                # 불량품 블록 추가 및 카운터 업데이트 이벤트 발송
                emit('update_defect_count', {'count': defect_count}, broadcast=True)
                # 작업량 카운터 증가 부분 제거
                # work_count = (work_count + 1) % 13  <- 이 부분 삭제
                # emit('update_work_count', {'count': work_count}, broadcast=True)  <- 이 부분 삭제
        else:  # 일반 블록 처리
            if current_block < 12:
                emit('change_block_color', {
                    'blockNumber': block_count[color],
                    'color': color
                }, broadcast=True)
                current_block += 1
                with block_count_lock:
                    block_count[color] += 1
                work_count = (work_count + 1) % 13
                emit('update_work_count', {'count': work_count}, broadcast=True)
            
            if current_block >= 12:
                emit('reset_blocks', broadcast=True)
                current_block = 0
                work_count = 0
                defect_count = 0
                emit('update_work_count', {'count': work_count}, broadcast=True)
                emit('update_defect_count', {'count': 0}, broadcast=True)
    else:
        print("유효하지 않은 명령어입니다.")
    emit('robot_message', message, broadcast=True)

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)
