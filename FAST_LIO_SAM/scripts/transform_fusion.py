#!/usr/bin/env python3
# coding=utf-8

from __future__ import print_function, division, absolute_import

import copy
import threading
import rospy
import tf
import tf.transformations
import numpy as np
import time

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import PoseWithCovarianceStamped

# 全局变量
cur_odom_to_camera = None
cur_map_to_odom = None
last_map_to_odom = None
relocalization_pose = None
enable_relocalization = True

# 线程安全锁
data_lock = threading.Lock()

FREQ_PUB_LOCALIZATION = 50

def pose_to_mat(pose_msg):
    """将Odometry消息转换为4x4变换矩阵"""
    position = pose_msg.pose.pose.position
    orientation = pose_msg.pose.pose.orientation

    translation = tf.transformations.translation_matrix([position.x, position.y, position.z])
    rotation = tf.transformations.quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])

    return np.dot(translation, rotation)

def pose_stamped_to_mat(pose_msg):
    """将PoseStamped消息转换为4x4变换矩阵"""
    position = pose_msg.pose.position
    orientation = pose_msg.pose.orientation

    translation = tf.transformations.translation_matrix([position.x, position.y, position.z])
    rotation = tf.transformations.quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])

    return np.dot(translation, rotation)

def handle_relocalization(pose_msg):
    """处理重定位消息的回调函数"""
    global relocalization_pose, last_map_to_odom
    
    with data_lock:
        rospy.loginfo("Received relocalization pose!")
        relocalization_pose = pose_msg
        
        # 如果已有map_to_odom变换，则更新
        if last_map_to_odom is not None:
            update_map_to_odom()

def update_map_to_odom():
    """根据重定位位姿更新map_to_odom变换"""
    global relocalization_pose, cur_map_to_odom, last_map_to_odom, cur_odom_to_camera
    
    if relocalization_pose is None or cur_odom_to_camera is None:
        return
    
    try:
        # 获取当前odom到camera的变换
        T_odom_to_camera = pose_to_mat(cur_odom_to_camera)
        
        # 获取重定位位姿 (map到camera的变换)
        T_map_to_camera = pose_stamped_to_mat(relocalization_pose)
        
        # 计算新的map到odom的变换: T_map_to_odom = T_map_to_camera * T_camera_to_odom
        T_camera_to_odom = np.linalg.inv(T_odom_to_camera)
        T_map_to_odom = np.dot(T_map_to_camera, T_camera_to_odom)
        
        # 创建Odometry消息
        new_map_to_odom = Odometry()
        new_map_to_odom.header.stamp = rospy.Time.now()
        new_map_to_odom.header.frame_id = "map"
        new_map_to_odom.child_frame_id = "odom"
        
        # 设置位姿
        xyz = tf.transformations.translation_from_matrix(T_map_to_odom)
        quat = tf.transformations.quaternion_from_matrix(T_map_to_odom)
        
        new_map_to_odom.pose.pose.position.x = xyz[0]
        new_map_to_odom.pose.pose.position.y = xyz[1]
        new_map_to_odom.pose.pose.position.z = xyz[2]
        new_map_to_odom.pose.pose.orientation.x = quat[0]
        new_map_to_odom.pose.pose.orientation.y = quat[1]
        new_map_to_odom.pose.pose.orientation.z = quat[2]
        new_map_to_odom.pose.pose.orientation.w = quat[3]
        
        # 更新当前map_to_odom变换
        cur_map_to_odom = new_map_to_odom
        last_map_to_odom = copy.deepcopy(new_map_to_odom)
        
        rospy.loginfo("Updated map_to_odom based on relocalization!")
        
    except Exception as e:
        rospy.logerr(f"Error updating map_to_odom: {str(e)}")

def handle_enable_relocalization(req):
    """启用/禁用重定位服务处理"""
    global enable_relocalization
    enable_relocalization = req.data
    return SetBoolResponse(success=True, message=f"Relocalization {'enabled' if req.data else 'disabled'}")

def transform_fusion():
    """融合变换并发布定位结果"""
    global cur_odom_to_camera, cur_map_to_odom, last_map_to_odom
    
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(FREQ_PUB_LOCALIZATION)
    
    while not rospy.is_shutdown():
        # 使用线程安全锁复制数据
        with data_lock:
            local_odom = copy.deepcopy(cur_odom_to_camera)
            local_map_to_odom = copy.deepcopy(cur_map_to_odom)
        
        T_map_to_odom = np.eye(4)
        
        if local_map_to_odom is not None:
            T_map_to_odom = pose_to_mat(local_map_to_odom)
            last_map_to_odom = copy.deepcopy(local_map_to_odom)
            
            # 发布 map → odom 变换
            translation = tf.transformations.translation_from_matrix(T_map_to_odom)
            rotation = tf.transformations.quaternion_from_matrix(T_map_to_odom)
            br.sendTransform(translation,
                             rotation,
                             rospy.Time.now(),
                             'odom', 'map')
        
        # 发布定位结果
        if local_odom is not None:
            T_odom_to_camera = pose_to_mat(local_odom)
            T_map_to_camera = np.dot(T_map_to_odom, T_odom_to_camera)
            
            xyz = tf.transformations.translation_from_matrix(T_map_to_camera)
            quat = tf.transformations.quaternion_from_matrix(T_map_to_camera)
            
            localization = Odometry()
            localization.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
            localization.twist = local_odom.twist
            localization.header.stamp = local_odom.header.stamp
            localization.header.frame_id = 'camera_init'
            localization.child_frame_id = 'map'
            
            pub_localization.publish(localization)
        
        rate.sleep()

def cb_save_cur_odom(odom_msg):
    """保存当前里程计回调"""
    global cur_odom_to_camera
    with data_lock:
        cur_odom_to_camera = odom_msg

def cb_save_map_to_odom(odom_msg):
    """保存map到odom变换回调"""
    global cur_map_to_odom
    with data_lock:
        cur_map_to_odom = odom_msg

if __name__ == '__main__':
    rospy.init_node('transform_fusion')
    rospy.loginfo('Transform Fusion Node Inited...')
    
    # 订阅里程计和map_to_odom变换
    rospy.Subscriber('/Odometry', Odometry, cb_save_cur_odom, queue_size=10)
    rospy.Subscriber('/map_to_odom', Odometry, cb_save_map_to_odom, queue_size=10)
    
    # 订阅重定位位姿
    rospy.Subscriber('/relocalization_pose', PoseStamped, handle_relocalization, queue_size=1)
    
    # 提供启用/禁用重定位服务
    service = rospy.Service('/enable_relocalization', SetBool, handle_enable_relocalization)
    
    pub_localization = rospy.Publisher('/localization', Odometry, queue_size=10)
    
    # 启动融合线程
    fusion_thread = threading.Thread(target=transform_fusion)
    fusion_thread.daemon = True
    fusion_thread.start()
    
    rospy.spin()