#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from std_srvs.srv import Trigger
import serial
import time
import math
import signal
import sys


class ImuData:
    """IMU 데이터를 저장하는 클래스"""
    def __init__(self):
        self.angular_velocity_x = 0.0
        self.angular_velocity_y = 0.0
        self.angular_velocity_z = 0.0

        self.linear_acceleration_x = 0.0
        self.linear_acceleration_y = 0.0
        self.linear_acceleration_z = 0.0

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0


class IahrsDriver(Node):
    """iAHRS IMU 센서 드라이버 (동기 버전)"""

    SERIAL_SPEED = 115200
    COMM_RECV_TIMEOUT_MS = 30

    def __init__(self):
        super().__init__('iahrs_driver')

        # 파라미터 선언
        self.declare_parameter('port', '/dev/iAHRS')
        self.declare_parameter('parent_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('tf_pos_x', 0.0)
        self.declare_parameter('tf_pos_y', 0.0)
        self.declare_parameter('tf_pos_z', 0.2)

        # 파라미터 가져오기
        self.port = self.get_parameter('port').value
        self.parent_frame_id = self.get_parameter('parent_frame_id').value
        self.publish_tf_flag = self.get_parameter('publish_tf').value
        self.tf_pos_x = self.get_parameter('tf_pos_x').value
        self.tf_pos_y = self.get_parameter('tf_pos_y').value
        self.tf_pos_z = self.get_parameter('tf_pos_z').value

        # Publisher와 Broadcaster 초기화
        self.imu_pub = self.create_publisher(Imu, 'iahrs_imu/data', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 서비스 생성
        self.euler_angle_reset_service = self.create_service(
            Trigger,
            '~/euler_angle_reset',
            self.euler_angle_reset_callback
        )

        # IMU 데이터 및 메시지 초기화
        self.imu_data = ImuData()
        self.imu_msg = Imu()
        self.set_covariance_values()

        # 시리얼 포트 초기화
        self.serial_port = None

        self.get_logger().info('IAHRS Driver Node has been initialized.')

    def run(self):
        """드라이버 실행"""
        if not self.open_serial_port():
            self.get_logger().fatal('Failed to open serial port. Shutting down.')
            return False

        # 시작 시 Euler 각도 리셋
        self.reset_device_euler_angles()
        self.get_logger().info('Device Euler angles have been reset.')

        self.print_ascii_art()

        # 메인 루프 (100Hz)
        rate = self.create_rate(100)
        try:
            while rclpy.ok():
                self.process_imu_data()
                rclpy.spin_once(self, timeout_sec=0)
                rate.sleep()
        except KeyboardInterrupt:
            pass
        finally:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()

        return True

    def open_serial_port(self):
        """시리얼 포트 열기"""
        self.get_logger().info(f'Trying to open serial port: {self.port}')

        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.SERIAL_SPEED,
                timeout=self.COMM_RECV_TIMEOUT_MS / 1000.0
            )
            self.get_logger().info(f'{self.port} opened successfully.')
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Unable to open {self.port}: {e}')
            return False

    def send_and_receive(self, command, data_length):
        """명령 전송 및 응답 수신"""
        if not self.serial_port or not self.serial_port.is_open:
            return None

        try:
            # 버퍼 클리어
            self.serial_port.reset_input_buffer()

            # 명령 전송
            self.serial_port.write(command.encode())

            # 응답 수신
            start_time = time.time()
            recv_data = b''

            while True:
                if self.serial_port.in_waiting > 0:
                    chunk = self.serial_port.read(self.serial_port.in_waiting)
                    recv_data += chunk
                    if recv_data.endswith(b'\r') or recv_data.endswith(b'\n'):
                        break

                if (time.time() - start_time) * 1000 > self.COMM_RECV_TIMEOUT_MS:
                    break

                time.sleep(0.001)

            recv_str = recv_data.decode('utf-8', errors='ignore').strip()

            # 에러 체크
            if recv_str.startswith('!'):
                return None

            # 응답 파싱
            if recv_str.startswith(command[:-1]) and '=' in recv_str:
                data_str = recv_str.split('=', 1)[1]
                values = []

                for val in data_str.split(','):
                    val = val.strip()
                    if val.startswith('0x'):
                        values.append(float(int(val, 16)))
                    else:
                        try:
                            values.append(float(val))
                        except ValueError:
                            break

                return values[:data_length]

            return None

        except Exception as e:
            self.get_logger().warn(f'Failed to communicate with serial port: {e}')
            return None

    def euler_angle_reset_callback(self, request, response):
        """Euler 각도 리셋 서비스 콜백"""
        try:
            cmd = "$sc,rst\r\n"
            self.serial_port.write(cmd.encode())
            response.success = True
            response.message = "Euler angle reset command sent."
            self.get_logger().info("Euler angle reset command sent.")
        except Exception as e:
            response.success = False
            response.message = f"Failed to send Euler angle reset command: {e}"
            self.get_logger().error(f"Failed to send Euler angle reset command: {e}")

        return response

    def reset_device_euler_angles(self):
        """디바이스 Euler 각도 리셋"""
        result = self.send_and_receive("ra\n", 10)
        return result is not None

    def process_imu_data(self):
        """IMU 데이터 처리"""
        if not self.serial_port or not self.serial_port.is_open:
            return

        # 자이로스코프 데이터 (각속도)
        gyro_data = self.send_and_receive("g\n", 10)
        if gyro_data and len(gyro_data) >= 3:
            self.imu_data.angular_velocity_x = gyro_data[0] * (math.pi / 180.0)
            self.imu_data.angular_velocity_y = gyro_data[1] * (math.pi / 180.0)
            self.imu_data.angular_velocity_z = gyro_data[2] * (math.pi / 180.0)

        # 가속도계 데이터
        accel_data = self.send_and_receive("a\n", 10)
        if accel_data and len(accel_data) >= 3:
            GRAVITY = 9.80665
            self.imu_data.linear_acceleration_x = accel_data[0] * GRAVITY
            self.imu_data.linear_acceleration_y = accel_data[1] * GRAVITY
            self.imu_data.linear_acceleration_z = accel_data[2] * GRAVITY

        # Euler 각도
        euler_data = self.send_and_receive("e\n", 10)
        if euler_data and len(euler_data) >= 3:
            self.imu_data.roll = euler_data[0] * (math.pi / 180.0)
            self.imu_data.pitch = euler_data[1] * (math.pi / 180.0)
            self.imu_data.yaw = euler_data[2] * (math.pi / 180.0)

        # 메시지 발행
        self.publish_imu_message()

        if self.publish_tf_flag:
            self.publish_tf()

    def publish_imu_message(self):
        """IMU 메시지 발행"""
        # Euler 각도를 Quaternion으로 변환
        cy = math.cos(self.imu_data.yaw * 0.5)
        sy = math.sin(self.imu_data.yaw * 0.5)
        cp = math.cos(self.imu_data.pitch * 0.5)
        sp = math.sin(self.imu_data.pitch * 0.5)
        cr = math.cos(self.imu_data.roll * 0.5)
        sr = math.sin(self.imu_data.roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        # 메시지 작성
        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_msg.header.frame_id = 'imu_link'

        self.imu_msg.orientation.x = qx
        self.imu_msg.orientation.y = qy
        self.imu_msg.orientation.z = qz
        self.imu_msg.orientation.w = qw

        self.imu_msg.angular_velocity.x = self.imu_data.angular_velocity_x
        self.imu_msg.angular_velocity.y = self.imu_data.angular_velocity_y
        self.imu_msg.angular_velocity.z = self.imu_data.angular_velocity_z

        self.imu_msg.linear_acceleration.x = self.imu_data.linear_acceleration_x
        self.imu_msg.linear_acceleration.y = self.imu_data.linear_acceleration_y
        self.imu_msg.linear_acceleration.z = self.imu_data.linear_acceleration_z

        self.imu_pub.publish(self.imu_msg)

    def publish_tf(self):
        """TF 브로드캐스트"""
        # Euler 각도를 Quaternion으로 변환
        cy = math.cos(self.imu_data.yaw * 0.5)
        sy = math.sin(self.imu_data.yaw * 0.5)
        cp = math.cos(self.imu_data.pitch * 0.5)
        sp = math.sin(self.imu_data.pitch * 0.5)
        cr = math.cos(self.imu_data.roll * 0.5)
        sr = math.sin(self.imu_data.roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        # Transform 메시지 작성
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.parent_frame_id
        transform.child_frame_id = 'imu_link'

        transform.transform.translation.x = self.tf_pos_x
        transform.transform.translation.y = self.tf_pos_y
        transform.transform.translation.z = self.tf_pos_z

        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(transform)

    def set_covariance_values(self):
        """Covariance 값 설정"""
        # Linear acceleration covariance
        self.imu_msg.linear_acceleration_covariance[0] = 0.0064
        self.imu_msg.linear_acceleration_covariance[4] = 0.0063
        self.imu_msg.linear_acceleration_covariance[8] = 0.0064

        # Angular velocity covariance
        self.imu_msg.angular_velocity_covariance[0] = 0.032 * (math.pi / 180.0)
        self.imu_msg.angular_velocity_covariance[4] = 0.028 * (math.pi / 180.0)
        self.imu_msg.angular_velocity_covariance[8] = 0.006 * (math.pi / 180.0)

        # Orientation covariance
        self.imu_msg.orientation_covariance[0] = 0.013 * (math.pi / 180.0)
        self.imu_msg.orientation_covariance[4] = 0.011 * (math.pi / 180.0)
        self.imu_msg.orientation_covariance[8] = 0.006 * (math.pi / 180.0)

    def print_ascii_art(self):
        """ASCII 아트 출력"""
        print("                       | Z axis ")
        print("                       | ")
        print("                       |   / X axis ")
        print("                   ____|__/____ ")
        print("      Y axis     / *   | /    /| ")
        print("      _________ /______|/    // ")
        print("               /___________ // ")
        print("              |____iahrs___|/ ")


def main(args=None):
    """메인 함수"""
    rclpy.init(args=args)

    node = IahrsDriver()
    node.run()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
