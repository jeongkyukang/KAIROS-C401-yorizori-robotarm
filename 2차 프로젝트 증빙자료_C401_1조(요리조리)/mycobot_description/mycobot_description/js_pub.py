import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import socket
import threading
import time
import math

class JointStateSubscriberWithTCP(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_angles = [0.0] * 7
        self.get_logger().info("TCP 클라이언트 시작")

        # TCP 설정
        self.tcp_ip = "192.168.56.1"  # 서버 IP 주소를 지정하세요
        self.tcp_port = 9999       # 서버 포트 번호를 지정하세요
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # 서버와 연결
        self.sock.connect((self.tcp_ip, self.tcp_port))
        self.get_logger().info(f"서버 {self.tcp_ip}:{self.tcp_port}에 연결됨")

        # TCP 수신 스레드 시작
        self.thread = threading.Thread(target=self.receive_angles)
        self.thread.daemon = True
        self.thread.start()

        # 타이머 생성
        self.timer = self.create_timer(0.5, self.publish_joint_state)  # 0.5초 주기로 퍼블리시

    def receive_angles(self):
        while rclpy.ok():
            try:
                # 각도 데이터 수신
                data = self.sock.recv(1024)
                if data:
                    angle_values = list(map(float, data.decode().split(',')))
			
                    # 각도 값 업데이트
                    if len(angle_values) == 7:
                        self.joint_angles = [math.radians(angle) for angle in angle_values[:6]] + [angle_values[6]]
                        self.get_logger().info(f"수신된 각도 데이터: {self.joint_angles}")
                    else:
                        self.get_logger().warning("잘못된 데이터 수신")
                    
            except Exception as e:
                self.get_logger().error(f"TCP 수신 오류: {e}")

    def publish_joint_state(self):
        # ROS 2 메시지 생성 및 발행
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [
            'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3', 'joint5_to_joint4', 
            'joint6_to_joint5', 'joint6output_to_joint6', 'gripper_controller', 
        ]
        joint_state.position = self.joint_angles
        self.publisher_.publish(joint_state)
        self.get_logger().info(f"joint state published: {joint_state.position}")

    def destroy_node(self):
        super().destroy_node()
        self.sock.close()  # 소켓 닫기

def main(args=None):
    rclpy.init(args=args)
    joint_state_subscriber = JointStateSubscriberWithTCP()
    rclpy.spin(joint_state_subscriber)
    joint_state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

