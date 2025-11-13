#!/usr/bin/env python3
# goal_sender_node.py

import rospy
import struct
import serial
import threading
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class GoalSenderNode:
    def __init__(self):
        rospy.init_node('goal_sender_node')

        # 参数配置
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyACM0')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.control_frequency = rospy.get_param('~control_frequency', 10)  # Hz

        # 控制参数
        self.max_forward_distance = rospy.get_param('~max_forward_distance', 100.0)  # 米
        self.max_lateral_distance = rospy.get_param('~max_lateral_distance', 100.0)  # 米
        self.max_heading_error = rospy.get_param('~max_heading_error', 181.0)  # 度

        # 串口初始化
        self.serial_conn = None
        self.serial_lock = threading.Lock()
        self.init_serial()

        # 统计信息
        self.sent_count = 0
        self.failed_count = 0
        self.consecutive_failures = 0

        # 订阅目标信息话题
        self.goal_info_sub = rospy.Subscriber('/goal_info', Float32MultiArray, self.goal_info_callback)

        # 可选：订阅cmd_vel用于调试
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        # 发布器用于反馈
        self.feedback_pub = rospy.Publisher('/goal_feedback', Float32MultiArray, queue_size=10)

        rospy.loginfo("目标发送节点初始化完成")
        rospy.loginfo("串口: %s, 波特率: %d", self.serial_port, self.baudrate)

    def init_serial(self):
        """初始化串口连接"""
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
                write_timeout=0.1
            )

            if self.serial_conn.is_open:
                rospy.loginfo("成功打开串口设备: %s", self.serial_port)
                # 清空缓冲区
                self.serial_conn.reset_input_buffer()
                self.serial_conn.reset_output_buffer()
            else:
                rospy.logwarn("串口设备打开但未连接")

        except Exception as e:
            rospy.logerr("打开串口设备失败: %s", str(e))
            self.serial_conn = None

    def calculate_checksum(self, data):
        """计算校验和"""
        return sum(data) & 0xFF

    def send_goal_info(self, longitudinal_distance, lateral_distance, heading_error_deg):
        """发送目标信息到C板"""
        if self.serial_conn is None or not self.serial_conn.is_open:
            rospy.logwarn_throttle(5, "串口未连接，无法发送数据")
            self.failed_count += 1
            self.consecutive_failures += 1
            return False

        try:
            # 数据限制
            longitudinal_distance = max(min(longitudinal_distance, self.max_forward_distance), -self.max_forward_distance)
            lateral_distance = max(min(lateral_distance, self.max_lateral_distance), -self.max_lateral_distance)
            heading_error_deg = max(min(heading_error_deg, self.max_heading_error), -self.max_heading_error)

            # 单位转换：米->厘米，度->0.01度
            forward_cm = longitudinal_distance
            lateral_cm = lateral_distance
            yaw_centideg = heading_error_deg

            # 构建数据帧
            frame = bytearray()
            frame.extend([0xAA, 0x55])  # 帧头
            frame.append(8)             # 数据长度
            frame.append(0x01)          # 命令字

            # 数据（小端序）
            frame.extend(struct.pack('<h', forward_cm))    # 前进距离
            frame.extend(struct.pack('<h', lateral_cm))    # 横向偏移
            frame.extend(struct.pack('<h', yaw_centideg))  # 偏航角
            frame.append(0x01)          # 控制模式

            # 校验和（从数据长度开始计算）
            checksum_data = frame[2:]  # 从数据长度开始
            checksum = self.calculate_checksum(checksum_data)
            frame.append(checksum)

            # 帧尾
            frame.extend([0x0D, 0x0A])

            # 发送数据
            with self.serial_lock:
                bytes_written = self.serial_conn.write(frame)
                self.serial_conn.flush()

            if bytes_written == len(frame):
                self.sent_count += 1
                self.consecutive_failures = 0

                # 每100帧打印一次日志
                if self.sent_count % 100 == 0:
                    rospy.loginfo("成功发送 %d 帧数据 [前进:%.2fm, 横向:%.2fm, 航向:%.1f度]",
                                  self.sent_count, longitudinal_distance, lateral_distance, heading_error_deg)
                return True
            else:
                rospy.logwarn("数据发送不完整，期望%d字节，实际%d字节", len(frame), bytes_written)
                self.failed_count += 1
                self.consecutive_failures += 1
                return False

        except Exception as e:
            rospy.logwarn("发送数据失败: %s", str(e))
            self.failed_count += 1
            self.consecutive_failures += 1

            # 如果连续失败多次，尝试重新连接
            if self.consecutive_failures >= 10:
                rospy.logwarn("连续失败次数过多，尝试重新连接串口")
                self.reconnect_serial()

            return False

    def reconnect_serial(self):
        """重新连接串口"""
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()

            rospy.sleep(0.5)
            self.init_serial()
            self.consecutive_failures = 0

        except Exception as e:
            rospy.logerr("重新连接串口失败: %s", str(e))

    def goal_info_callback(self, msg):
        """目标信息回调函数"""
        if len(msg.data) < 3:
            rospy.logerr("接收到的数据格式错误，期望3个元素，实际收到 %d 个", len(msg.data))
            return

        longitudinal_distance = msg.data[0]     # 前进距离 (米)
        lateral_distance = msg.data[1]          # 横向偏移距离 (米)
        heading_error_deg = msg.data[2]         # 航向偏差 (度)

        # 发送到C板
        success = self.send_goal_info(longitudinal_distance, lateral_distance, heading_error_deg)

        # 发布反馈信息
        feedback_msg = Float32MultiArray()
        feedback_msg.data = [
            float(success),
            float(self.sent_count),
            float(self.failed_count)
        ]
        self.feedback_pub.publish(feedback_msg)

    def cmd_vel_callback(self, msg):
        """cmd_vel回调函数（用于调试）"""
        # 将cmd_vel转换为目标信息格式
        # 这里可以根据需要实现不同的控制策略
        longitudinal_distance = msg.linear.x * 0.1  # 简单映射
        lateral_distance = msg.linear.y * 0.1
        heading_error_deg = msg.angular.z * 10.0

        # 发送到C板
        self.send_goal_info(longitudinal_distance, lateral_distance, heading_error_deg)

    def run(self):
        """主运行循环"""
        rate = rospy.Rate(1)  # 1Hz状态监控

        while not rospy.is_shutdown():
            # 定期状态报告
            if self.serial_conn and self.serial_conn.is_open:
                rospy.loginfo_throttle(10, "系统运行正常 - 发送: %d, 失败: %d",
                                       self.sent_count, self.failed_count)
            else:
                rospy.logwarn_throttle(5, "串口设备未连接")
                # 尝试重新连接
                self.reconnect_serial()

            rate.sleep()

if __name__ == '__main__':
    try:
        node = GoalSenderNode()
        node.run()
    except rospy.ROSInterruptException:
        pass