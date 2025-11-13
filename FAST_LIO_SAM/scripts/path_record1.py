#!/usr/bin/env python3
#rate three dog
import rospy
import os
import yaml
import math
import numpy as np
import threading
import sys
import select
import termios
import tty
import time
from scipy.interpolate import splprep, splev
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse
from visualization_msgs.msg import Marker
import tf.transformations as tf_trans
from std_msgs.msg import Float32MultiArray
from fast_lio_sam.srv import AddPoint, AddPointResponse  # 自定义服务


class PathManager:
    def __init__(self):
        rospy.init_node('path_manager')

        # 运行模式参数
        self.mode = rospy.get_param("~mode", "click")  # click或record
        self.auto_load = rospy.get_param("~auto_load", False)
        self.min_distance = rospy.get_param("~min_distance", 0.5)  # 最小记录距离
        self.smooth_factor = rospy.get_param("~smooth_factor", 10.0)
        self.lookahead_distance = rospy.get_param("~lookahead_distance", 1.5)  # 前视距离
        self.goal_threshold = rospy.get_param("~goal_threshold", 0.0)  # 目标点切换阈值
        
        # 路径数据结构
        self.path = Path()
        self.path.header.frame_id = "camera_init"  # 使用相机坐标系
        self.path.header.stamp = rospy.Time.now()
        self.smoothed_path = Path()
        self.smoothed_path.header.frame_id = "camera_init"  # 使用相机坐标系
        self.smoothed_path.header.stamp = rospy.Time.now()
        self.goal_position = None  # 最终目标点
        
        # 键盘输入状态
        self.space_count = 0  # 记录连续按空格的次数
        self.enter_count = 0  # 记录连续按Enter的次数
        self.last_space_time = 0  # 上次按空格的时间
        self.last_enter_time = 0  # 上次按Enter的时间
        self.is_first_point_added = False  # 标记是否已添加第一个点
        
        # 发布器
        self.raw_path_pub = rospy.Publisher('/raw_path', Path, queue_si-ze=10, latch=True)
        self.smooth_path_pub = rospy.Publisher('/smooth_path', Path, queue_size=10, latch=True)
        self.marker_pub = rospy.Publisher('/path_markers', Marker, queue_size=10)
        self.goal_info_pub = rospy.Publisher('/goal_info', Float32MultiArray, queue_size=10)
        
        # 订阅器
        rospy.Subscriber('/clicked_point', PointStamped, self.point_callback)
        rospy.Subscriber('/Odometry', Odometry, self.odom_callback)
        
        # 服务 - 路径管理
        self.save_service = rospy.Service('/save_path', Trigger, self.handle_save_request)
        self.load_service = rospy.Service('/load_path', Trigger, self.handle_load_request)
        self.smooth_service = rospy.Service('/smooth_path', Trigger, self.handle_smooth_request)
        
        # 服务 - 手动设点
        self.add_point_service = rospy.Service('/add_point', AddPoint, self.handle_add_point)
        self.toggle_mode_service = rospy.Service('/toggle_mode', SetBool, self.handle_toggle_mode)
        self.is_recording = False  # 是否处于记录模式
        self.last_recorded_point = None  # 最后记录的点
        self.current_goal_index = 0
        self.robot_position = None  # 机器人当前位置
        
        # 保存路径目录
        self.save_dir = os.path.join(os.path.expanduser("/home/toe/"), "saved_paths")
        os.makedirs(self.save_dir, exist_ok=True)
        
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True  # 设置为守护线程，这样当主线程退出时，该线程也会退出
        self.keyboard_thread.start()
        
        # 自动加载路径
        if self.auto_load:
            self.load_path_from_file()
            
        rospy.loginfo(f"2D雷达路径管理器已启动 (模式: {self.mode})")
        rospy.loginfo(f"最小记录距离: {self.min_distance}m, 前视距离: {self.lookahead_distance}m")
        
        if self.mode == "record":
            self.is_recording = True
            rospy.loginfo("记录模式已启用，使用 /save_path 服务保存记录的路径")

    def keyboard_listener(self):
        try:
            # 尝试设置非阻塞输入
            if sys.stdin.isatty():
                settings = termios.tcgetattr(sys.stdin)
                tty.setcbreak(sys.stdin.fileno())
                use_termios = True
            else:
                use_termios = False
                rospy.logwarn("标准输入不是终端，使用简单输入模式")
        except Exception as e:
            use_termios = False
            rospy.logwarn(f"无法配置终端: {e}, 使用简单输入模式")

        try:
            while not rospy.is_shutdown():
                current_time = time.time()
                
                # 检查是否需要重置计数器（超过1秒未按键）
                if current_time - self.last_space_time > 1.0 and self.space_count > 0:
                    rospy.loginfo("空格输入超时，重置计数器")
                    self.space_count = 0
                
                if current_time - self.last_enter_time > 1.0 and self.enter_count > 0:
                    rospy.loginfo("Enter输入超时,重置计数器")
                    self.enter_count = 0
                
                # 使用select检查输入
                if use_termios:
                    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                    if rlist:
                        key = sys.stdin.read(1)
                        # 检测空格键
                        if key == ' ':
                            self.handle_space_key()
                        # 检测Enter键
                        elif key == '\n' or key == '\r':
                            self.handle_enter_key()
                else:
                    # 简单模式 - 直接检查输入
                    try:
                        line = sys.stdin.readline()
                        if line:
                            # 检查空格键
                            if ' ' in line:
                                self.handle_space_key()
                            # 检查Enter键
                            elif line.endswith('\n') or line.endswith('\r'):
                                self.handle_enter_key()
                    except Exception as e:
                        rospy.logerr_once(f"键盘输入错误: {e}, 停止键盘监听")
                        break
        finally:
            # 恢复终端设置
            if use_termios:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def handle_space_key(self):
        """处理空格键按下事件 - 用于第一次标点"""
        current_time = time.time()
        
        # 如果已添加第一个点，则忽略空格键
        if self.is_first_point_added:
            rospy.loginfo("已添加第一个点，请使用Enter键添加后续点")
            self.space_count = 0
            return
        
        # 如果是第一次按键或超过1秒未按键，重置计数器
        if current_time - self.last_space_time > 1.0:
            self.space_count = 0
        
        self.space_count += 1
        self.last_space_time = current_time
        
        # 显示当前按键次数
        rospy.loginfo(f"检测到空格键按下 ({self.space_count}/3)")
        
        # 检查是否需要添加第一个点
        if self.space_count >= 3:
            self.add_current_position_as_point()
            self.is_first_point_added = True
            self.space_count = 0  # 重置计数器

    def handle_enter_key(self):
        """处理Enter键按下事件 - 用于后续标点"""
        current_time = time.time()
        
        # 如果还没有添加第一个点，则忽略Enter键
        if not self.is_first_point_added:
            rospy.loginfo("请先使用三次空格键添加第一个点")
            self.enter_count = 0
            return
        
        # 如果是第一次按键或超过1秒未按键，重置计数器
        if current_time - self.last_enter_time > 1.0:
            self.enter_count = 0
        
        self.enter_count += 1
        self.last_enter_time = current_time
        
        # 显示当前按键次数
        rospy.loginfo(f"检测到Enter键按下 ({self.enter_count}/3)")
        
        # 检查是否需要添加点
        if self.enter_count >= 3:
            self.add_current_position_as_point()
            self.enter_count = 0  # 重置计数器

    def add_current_position_as_point(self):
        """添加当前机器人位置为路径点"""
        if self.robot_position is None:
            rospy.logwarn("未收到机器人位置，无法添加点")
            return
            
        x, y = self.robot_position
        self.add_point_to_path(x, y)
        rospy.loginfo(f"已添加当前机器人位置为路径点: x={x:.2f}, y={y:.2f}")

    def angle_diff(self, a, b):
        """计算两个角度之间的差值（考虑圆周）"""
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d

    def add_point_to_path(self, x, y, frame_id="map"):
        """添加点到路径"""
        # 如果是第一个点，设置为起点
        if len(self.path.poses) == 0 and self.robot_position:
            start_pose = PoseStamped()
            start_pose.header.frame_id = frame_id
            start_pose.pose.position.y = 0
            start_pose.pose.position.x = 0
            start_pose.pose.position.z = 0.0
            start_pose.pose.orientation.w = 1.0
            self.path.poses.append(start_pose)
            rospy.loginfo(f"已设定路径起点为当前位置: x={self.robot_position[0]:.2f}, y={self.robot_position[1]:.2f}")
            self.current_goal_index = 0 
        
        # 添加目标点
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = frame_id
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        self.path.poses.append(goal_pose)
        self.path.header.stamp = rospy.Time.now()
        self.goal_position = (y, x)  # 更新最终目标点
        self.raw_path_pub.publish(self.path)

        self.visualize_path()
   
    def point_callback(self, msg):
        """处理点击点消息"""
        if self.mode != "click":
            return

        if msg.header.frame_id != "camera_init":
            rospy.logwarn(f"点坐标系 {msg.header.frame_id} 不是'camera_init'，已忽略")
            return

        if not self.robot_position and len(self.path.poses) == 0:
            rospy.logwarn("未收到机器人当前位置，无法设定起点")
            return

        self.add_point_to_path(msg.point.y, msg.point.x, msg.header.frame_id)


    def odom_callback(self, msg):
        """处理里程计消息"""
        # 更新机器人位置
        raw_x = msg.pose.pose.position.y
        raw_y = msg.pose.pose.position.x
        transformed_x = raw_x
        transformed_y = raw_y
        self.robot_position = (transformed_x, transformed_y)        
        
        # 更新偏航角
        quat = (
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        _, _, self.robot_yaw = tf_trans.euler_from_quaternion(quat)
        
        # 记录模式：自动记录路径点
        if self.is_recording:
            current_pos = (transformed_x, transformed_y)
            
            # 检查是否是新点（距离上一个点足够远）
            if self.last_recorded_point:
                dx = current_pos[0] - self.last_recorded_point[0]
                dy = current_pos[1] - self.last_recorded_point[1]
                distance = math.sqrt(dx**2 + dy**2)
                
                if distance < self.min_distance:
                    return  # 距离太小，不记录
            
            # 记录新点
            pose = PoseStamped()
            pose.header.frame_id = "camera_init"  # 使用相机坐标系
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = current_pos[0]
            pose.pose.position.y = current_pos[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            self.path.poses.append(pose)
            self.last_recorded_point = current_pos
            
            # 发布原始路径
            self.path.header.stamp = rospy.Time.now()
            self.raw_path_pub.publish(self.path)
            
            # 更新目标点
            self.goal_position = current_pos
            rospy.loginfo(f"记录路径点: x={current_pos[0]:.2f}, y={current_pos[1]:.2f}")
            self.visualize_path()
        
      
        # 导航信息处理
        if self.path.poses:
            # 确保当前目标索引有效
            if self.current_goal_index >= len(self.path.poses):
                self.current_goal_index = len(self.path.poses) - 1
                rospy.logwarn(f"目标索引超出范围，重置为最后一点: {self.current_goal_index}")

            # 获取当前目标点位置
            current_goal_pose = self.path.poses[self.current_goal_index]
            goal_pos = current_goal_pose.pose.position
            goal_position = (goal_pos.x, goal_pos.y)
            
            # 计算到当前目标点的距离
            goal_dx = self.robot_position[0] - goal_position[0]
            goal_dy = self.robot_position[1] - goal_position[1]
            distance_to_goal = math.hypot(goal_dx, goal_dy)
            
            # 如果接近当前目标点，切换到下一个目标点
            if goal_dx < 0.1:
                if self.current_goal_index < len(self.path.poses) - 1:
                    self.current_goal_index += 1
                    new_goal = self.path.poses[self.current_goal_index].pose.position
                    rospy.loginfo(f"切换到下一个目标点: {self.current_goal_index}/{len(self.path.poses)-1} "
                                f"({new_goal.x:.2f}, {new_goal.y:.2f})")
                else:
                    rospy.loginfo("已到达最终目标点!")
            
            # 获取当前目标点（可能已更新）
            current_goal_pose = self.path.poses[self.current_goal_index]
            goal_pos = current_goal_pose.pose.position
            self.goal_position = (goal_pos.x, goal_pos.y)

            # 重新计算到当前目标点的向量
            goal_dx = self.robot_position[0]- self.goal_position[0]
            goal_dy = self.robot_position[1] - self.goal_position[1]
            distance_to_goal = math.hypot(goal_dx, goal_dy)
            if self.current_goal_index > 0:
                # 正常情况：使用上一个目标点和当前目标点形成的路径段
                prev_goal_pose = self.path.poses[self.current_goal_index - 1]
                A = (prev_goal_pose.pose.position.x, prev_goal_pose.pose.position.y)
                B = self.goal_position
            else:
                # 当处于第一个目标点时：使用原点和当前目标点形成的路径段
                A = (0, 0)  # 原点
                B = self.goal_position
            # 计算路径向量 (从A点到B点)
            path_vector = (B[0] - A[0], B[1] - A[1])
            # 计算从A点到机器人位置的向量
            robot_vector = (self.robot_position[0] - A[0], self.robot_position[1] - A[1])
        
            # 计算路径向量的模长
            path_length = math.hypot(path_vector[0], path_vector[1])
            
            if path_length > 1e-6:  # 避免除以零
                # 使用叉积计算横向偏移 (带符号)
                cross_product = path_vector[0] * robot_vector[1] - path_vector[1] * robot_vector[0]
                lateral_distance = cross_product / path_length
            else:
                lateral_distance = 0.0

            goal_heading = math.atan2(goal_dy, goal_dx)
            
            # 计算航向偏差（机器人当前朝向与目标方向的角度差）            
            goal_heading_error = self.angle_diff(self.robot_yaw,0)
            heading_error_deg = math.degrees(goal_heading_error)
            if heading_error_deg < -180:
                heading_error_deg += 360
            
            rospy.loginfo_throttle(0.1,
                f"当前位置: ({self.robot_position[0]:.2f}, {self.robot_position[1]:.2f}) | "
                f"目标点: ({goal_pos.x:.2f}, {goal_pos.y:.2f}) | "
                f"目标点 {self.current_goal_index}/{len(self.path.poses)-1} → "
                f"距离={goal_dx:.2f}m | 横向偏移={lateral_distance:.2f}m | 航向偏差={heading_error_deg:.1f}°"
                )            
                
            
            # 发布目标点信息
            data = [
                float(goal_dx),           # 到当前目标点的y方向距离
                float(lateral_distance),   # 横向偏移距离
                float(heading_error_deg),  # 航向偏差

            ]
            
            # 创建并发布消息
            array_msg = Float32MultiArray(data=data)
            self.goal_info_pub.publish(array_msg)

    def smooth_path(self):
        """平滑路径"""
        if len(self.path.poses) < 3:
            rospy.logwarn("路径点不足，无法平滑")
            return False
            
        try:
            # 提取路径点
            points = []
            for pose in self.path.poses:
                points.append([pose.pose.position.x, pose.pose.position.y])
            
            # 转换为NumPy数组
            points = np.array(points)
            
            # 样条曲线平滑
            tck, u = splprep(points.T, u=None, s=self.smooth_factor, per=0) 
            u_new = np.linspace(u.min(), u.max(), 100)
            x_new, y_new = splev(u_new, tck, der=0)
            
            # 创建平滑路径
            self.smoothed_path = Path()
            self.smoothed_path.header.frame_id = "camera_init"  # 使用相机坐标系
            self.smoothed_path.header.stamp = rospy.Time.now()
            
            for i in range(len(x_new)):
                pose = PoseStamped()
                pose.header.frame_id = "camera_init"  # 使用相机坐标系
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = x_new[i]
                pose.pose.position.y = y_new[i]
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                self.smoothed_path.poses.append(pose)
            
            # 发布平滑路径
            self.smooth_path_pub.publish(self.smoothed_path)
            rospy.loginfo(f"路径已平滑，原始点: {len(self.path.poses)} → 平滑点: {len(self.smoothed_path.poses)}")
            self.visualize_path()
            return True
        except Exception as e:
            rospy.logerr(f"平滑路径失败: {str(e)}")
            return False

    def visualize_path(self):
        """可视化原始路径和平滑路径"""
        # 可视化原始路径
        if len(self.path.poses) >= 2:
            marker_raw = Marker()
            marker_raw.header.frame_id = "camera_init"  # 使用相机坐标系
            marker_raw.header.stamp = rospy.Time.now()
            marker_raw.ns = "raw_path"
            marker_raw.id = 0
            marker_raw.type = Marker.LINE_STRIP
            marker_raw.action = Marker.ADD
            marker_raw.scale.x = 0.1
            marker_raw.color.r = 1.0
            marker_raw.color.g = 0.5
            marker_raw.color.a = 0.7
            marker_raw.pose.orientation.w = 1.0
            
            # 按顺序添加点
            for pose in self.path.poses:
                point = Point()
                point.x = pose.pose.position.x
                point.y = pose.pose.position.y
                point.z = 0.0
                marker_raw.points.append(point)
                
            self.marker_pub.publish(marker_raw)
        
        # 可视化平滑路径
        if len(self.smoothed_path.poses) >= 2:
            marker_smooth = Marker()
            marker_smooth.header.frame_id = "camera_init"  # 使用相机坐标系
            marker_smooth.header.stamp = rospy.Time.now()
            marker_smooth.ns = "smooth_path"
            marker_smooth.id = 1  # 使用不同的ID
            marker_smooth.type = Marker.LINE_STRIP
            marker_smooth.action = Marker.ADD
            marker_smooth.scale.x = 0.15
            marker_smooth.color.g = 1.0
            marker_smooth.color.r = 0.5
            marker_smooth.color.a = 1.0
            marker_smooth.pose.orientation.w = 1.0
            
            # 按顺序添加点
            for pose in self.smoothed_path.poses:
                point = Point()
                point.x = pose.pose.position.x
                point.y = pose.pose.position.y
                point.z = 0.0
                marker_smooth.points.append(point)
                
            self.marker_pub.publish(marker_smooth)

    def save_path_to_file(self, filename="path.yaml"):
        """保存路径到文件"""
        try:
            filepath = os.path.join(self.save_dir, filename)
            path_data = {
                "frame_id": "map",
                "points": [{"x": p.pose.position.x, "y": p.pose.position.y} for p in self.path.poses],
                "smoothed_points": [{"x": p.pose.position.x, "y": p.pose.position.y} for p in self.smoothed_path.poses],
                "lookahead_distance": self.lookahead_distance,
                "min_distance": self.min_distance,
                "smooth_factor": self.smooth_factor,
                "goal_threshold": self.goal_threshold,
            }
            
            with open(filepath, 'w') as f:
                yaml.dump(path_data, f)
            
            rospy.loginfo(f"路径已保存到: {filepath}")
            return filepath
        except Exception as e:
            rospy.logerr(f"保存路径失败: {str(e)}")
            return None

    def load_path_from_file(self, filename="path.yaml"):
        """从文件加载路径"""
        try:
            filepath = os.path.join(self.save_dir, filename)
            if not os.path.exists(filepath):
                rospy.logerr(f"路径文件不存在: {filepath}")
                return False 
                
            with open(filepath, 'r') as f:
                path_data = yaml.safe_load(f)
            
            # 重置路径
            self.path = Path()
            self.path.header.frame_id = "camera_init"  # 使用相机坐标系
            self.path.header.stamp = rospy.Time.now()
            self.smoothed_path = Path()
            self.smoothed_path.header.frame_id = "camera_init"  # 使用相机坐标系
            self.smoothed_path.header.stamp = rospy.Time.now()
            
            # 加载原始路径点
            for point in path_data["points"]:
                pose = PoseStamped()
                pose.header.frame_id = "camera_init"  # 使用相机坐标系
                pose.pose.position.x = point["x"]
                pose.pose.position.y = point["y"]
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                self.path.poses.append(pose)
            
            # 加载平滑路径点
            if "smoothed_points" in path_data:
                for point in path_data["smoothed_points"]:
                    pose = PoseStamped()
                    pose.header.frame_id = "camera_init"  # 使用相机坐标系
                    pose.header.stamp = rospy.Time.now()
                    pose.pose.position.x = point["x"]
                    pose.pose.position.y = point["y"]
                    pose.pose.position.z = 0.0
                    pose.pose.orientation.w = 1.0
                    self.smoothed_path.poses.append(pose)
            
            # 加载参数
            if "lookahead_distance" in path_data:
                self.lookahead_distance = path_data["lookahead_distance"]
            if "min_distance" in path_data:
                self.min_distance = path_data["min_distance"]
            if "smooth_factor" in path_data:
                self.smooth_factor = path_data["smooth_factor"]
            if "goal_threshold" in path_data:
                self.goal_threshold = path_data["goal_threshold"]
            
            # 重置目标点索引
            self.current_goal_index = 0
            
            # 设置最终目标点
            if self.path.poses:
                last_point = self.path.poses[-1].pose.position
                self.goal_position = (last_point.x, last_point.y)
                rospy.loginfo(f"设置目标点为: ({last_point.x:.2f}, {last_point.y:.2f})")
            
            # 发布路径
            self.path.header.stamp = rospy.Time.now()
            self.raw_path_pub.publish(self.path)
            
            if self.smoothed_path.poses:
                self.smoothed_path.header.stamp = rospy.Time.now()
                self.smooth_path_pub.publish(self.smoothed_path)
            
            rospy.loginfo(f"从 {filepath} 加载了 {len(self.path.poses)} 个原始点和 {len(self.smoothed_path.poses)} 个平滑点")
            self.visualize_path()
            return True
        except Exception as e:
            rospy.logerr(f"加载路径失败: {str(e)}")
            return False

    def handle_save_request(self, req):
        try:
            if self.save_path_to_file():
                return TriggerResponse(True, "路径保存成功")
            return TriggerResponse(False, "路径保存失败")
        except Exception as e:
            return TriggerResponse(False, f"保存失败: {str(e)}")

    def handle_load_request(self, req):
        try:
            if self.load_path_from_file():
                return TriggerResponse(True, "路径加载成功")
            return TriggerResponse(False, "路径加载失败")
        except Exception as e:
            return TriggerResponse(False, f"加载失败: {str(e)}")

    def handle_smooth_request(self, req):
        try:
            if self.smooth_path():
                return TriggerResponse(True, "路径平滑成功")
            return TriggerResponse(False, "路径平滑失败")
        except Exception as e:
            return TriggerResponse(False, f"平滑失败: {str(e)}")
            
    def handle_add_point(self, req):
        """处理手动添加点请求"""
        try:
            self.add_point_to_path(req.y, req.x)
            return AddPointResponse(True, f"点 ({req.x:.2f}, {req.y:.2f}) 添加成功")
        except Exception as e:
            return AddPointResponse(False, f"添加点失败: {str(e)}")
            
    def handle_toggle_mode(self, req):
        """切换记录模式状态"""
        if req.data:
            if not self.is_recording:
                self.is_recording = True
                self.last_recorded_point = None  # 重置最后记录点
                rospy.loginfo("记录模式已启用")
                return SetBoolResponse(True, "记录模式已启用")
            else:
                rospy.logwarn("记录模式已处于启用状态")
                return SetBoolResponse(False, "记录模式已处于启用状态")
        else:
            if self.is_recording:
                self.is_recording = False
                rospy.loginfo("记录模式已禁用")
                return SetBoolResponse(True, "记录模式已禁用")
            else:
                rospy.logwarn("记录模式已处于禁用状态")
                return SetBoolResponse(False, "记录模式已处于禁用状态")

if __name__ == '__main__':
    manager = PathManager()
    rospy.spin()