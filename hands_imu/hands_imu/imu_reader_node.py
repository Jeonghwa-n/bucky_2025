import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import board
import busio
import time

from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
import adafruit_tca9548a


class ImuReaderNode(Node):
    def __init__(self):
        super().__init__('imu_reader_node')

        # 퍼블리시 주기 (기본 50Hz)
        self.rate_hz = self.declare_parameter('rate_hz', 50.0).value
        period = 1.0 / float(self.rate_hz)

        # ============================
        #  I2C / TCA9548A 초기화
        # ============================
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
        except Exception as e:
            self.get_logger().fatal(f'I2C 초기화 실패: {e}')
            raise

        try:
            # 주소 0x70, A0~A2 GND 기준
            self.tca = adafruit_tca9548a.TCA9548A(i2c, address=0x70)
        except Exception as e:
            self.get_logger().fatal(f'TCA9548A 초기화 실패: {e}')
            raise

        # 센서 이름 ↔ 채널 매핑
        self.sensors = [
            {"name": "thumb",  "channel": 0},
            {"name": "index",  "channel": 1},
            {"name": "middle", "channel": 2},
            {"name": "ring",   "channel": 3},
            {"name": "little", "channel": 4},
            {"name": "back",   "channel": 5},
        ]

        # 채널별 BNO08X 객체와 퍼블리셔
        self.imu_devices = {}
        self.imu_publishers = {}

        for s in self.sensors:
            name = s["name"]
            ch = s["channel"]

        # TCA9548A에서 해당 채널의 I2C 라인 얻기
            ch_i2c = self.tca[ch]

            try:
                imu = BNO08X_I2C(ch_i2c, address=0x4B)
                # 회전 벡터(쿼터니언) 리포트 활성화
                imu.enable_feature(BNO_REPORT_ROTATION_VECTOR)
                self.imu_devices[name] = imu
                self.get_logger().info(f'IMU init 성공: {name} (channel {ch})')
            except Exception as e:
                self.get_logger().error(
                    f'IMU init 실패: {name} (channel {ch}) → {e}'
                )
                self.imu_devices[name] = None


            topic_name = f'/hand/imu/{name}'
            self.imu_publishers[name] = self.create_publisher(Imu, topic_name, 10)

        self.start_time = time.time()
        self.timer = self.create_timer(period, self.timer_callback)

    # ============================
    #  주기적으로 6개 센서 읽고 퍼블리시
    # ============================
    def timer_callback(self):
        stamp = self.get_clock().now().to_msg()

        for s in self.sensors:
            name = s["name"]
            imu = self.imu_devices.get(name)
            pub = self.imu_publishers[name]

            if imu is None:
                # 초기화 실패 센서는 스킵
                continue

            try:
                # adafruit_bno08x: quaternion → (w, x, y, z)
                qw, qx, qy, qz = imu.quaternion
            except Exception as e:
                self.get_logger().warn(f'{name} quaternion 읽기 실패: {e}')
                continue

            msg = Imu()
            msg.header.stamp = stamp
            msg.header.frame_id = f'imu_{name}'

            msg.orientation.w = qw
            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz

            # 각속도/가속도는 현재 사용 안 함 (필요 시 추가)
            pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuReaderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()