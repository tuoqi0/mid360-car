#!/usr/bin/env python3
# coding=utf8
import rospy
import numpy as np
import threading
import tf
import tf2_ros
import open3d as o3d
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool
import sensor_msgs.point_cloud2 as pc2

# 优化点云处理参数
ICP_PARAMS = {
    "MAP_VOXEL_SIZE": 0.8,  # 增大降采样粒度
    "SCAN_VOXEL_SIZE": 0.4,
    "LOCALIZATION_TH": 0.85,  # 降低匹配阈值要求
    "FOV": 1.6,
    "FOV_FAR": 50,
    "MAX_POINTS": 5000000,  # 最大处理点数
    "MAX_ITER": 15
}

# Global variables
global_map = None  # 存储为numpy数组而非Open3D对象
global_map_downsampled = None  # 降采样后的地图
initial_pose = None
localization_triggered = False
T_map_to_camera = None
cur_scan = None  # 存储为numpy数组

def downsample_points(points, voxel_size):
    """高效降采样点云（避免使用Open3D）"""
    if len(points) == 0:
        return points
    
    # 计算每个点的体素坐标
    voxel_coords = np.floor(points / voxel_size).astype(int)
    
    # 找到唯一体素及其第一个点索引
    _, unique_indices = np.unique(voxel_coords, axis=0, return_index=True)
    
    return points[unique_indices]

def pointcloud2_to_numpy(msg):
    """将PointCloud2转换为numpy数组（高效版本）"""
    points = []
    for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        points.append([p[0], p[1], p[2]])
        # 如果点云太大，提前终止
        if len(points) > ICP_PARAMS["MAX_POINTS"] * 2:
            break
    return np.array(points)

def transform_points(points, transform: TransformStamped):
    """使用变换矩阵转换点云"""
    trans = transform.transform.translation
    rot = transform.transform.rotation
    T = tf.transformations.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
    T[0, 3] = trans.x
    T[1, 3] = trans.y
    T[2, 3] = trans.z
    
    # 高效转换
    points_hom = np.hstack((points, np.ones((points.shape[0], 1))))
    transformed = (T @ points_hom.T).T[:, :3]
    return transformed

def icp_registration(source, target, initial, max_iter=10, tolerance=0.001):
    """
    简易ICP实现（避免使用Open3D）
    参考: http://ais.informatik.uni-freiburg.de/teaching/ws11/robotics2/pdfs/icp.pdf
    """
    prev_error = 0
    transformation = np.copy(initial)
    
    for i in range(max_iter):
        # 找到最近点
        from scipy.spatial import cKDTree
        tree = cKDTree(target)
        dist, indices = tree.query(source)
        
        # 计算对应点
        correspondences = target[indices]
        
        # 计算质心
        source_centroid = np.mean(source, axis=0)
        target_centroid = np.mean(correspondences, axis=0)
        
        # 中心化点云
        source_centered = source - source_centroid
        target_centered = correspondences - target_centroid
        
        # 计算协方差矩阵
        H = source_centered.T @ target_centered
        
        # SVD分解
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        
        # 处理反射情况
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        
        # 计算平移
        t = target_centroid - R @ source_centroid
        
        # 更新变换
        current_trans = np.eye(4)
        current_trans[:3, :3] = R
        current_trans[:3, 3] = t
        transformation = current_trans @ transformation
        
        # 更新源点云
        source = (current_trans[:3, :3] @ source.T).T + current_trans[:3, 3]
        
        # 计算误差
        mean_error = np.mean(dist)
        if abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error
    
    # 计算fitness (1 / (1 + 平均误差))
    fitness = 1.0 / (1.0 + mean_error)
    return transformation, fitness

def crop_global_map_in_FOV(global_map, pose_estimation):
    """裁剪全局地图到当前视野范围（高效版本）"""
    if global_map is None or len(global_map) == 0:
        return np.empty((0, 3))
    
    # 提取变换矩阵
    trans = pose_estimation.transform.translation
    rot = pose_estimation.transform.rotation
    T = tf.transformations.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
    T[0, 3] = trans.x
    T[1, 3] = trans.y
    T[2, 3] = trans.z
    
    # 计算逆变换（地图点->当前坐标系）
    T_inv = np.linalg.inv(T)
    
    # 转换点云
    points_hom = np.hstack((global_map, np.ones((len(global_map), 1))))
    points_in_base = (T_inv @ points_hom.T).T[:, :3]
    
    # 创建筛选条件
    x = points_in_base[:, 0]
    y = points_in_base[:, 1]
    dist = np.sqrt(x**2 + y**2)
    angles = np.abs(np.arctan2(y, x))
    
    if ICP_PARAMS["FOV"] > 3.14:  # 环状LiDAR
        mask = (dist < ICP_PARAMS["FOV_FAR"]) & (angles < ICP_PARAMS["FOV"]/2.0)
    else:  # 前向LiDAR
        mask = (x > 0) & (x < ICP_PARAMS["FOV_FAR"]) & (angles < ICP_PARAMS["FOV"]/2.0)
    
    # 返回裁剪后的点云
    return global_map[mask]

def perform_icp_localization(initial_transform):
    """执行ICP重定位（优化内存使用）"""
    global global_map_downsampled, cur_scan, T_map_to_camera
    
    if global_map_downsampled is None or len(global_map_downsampled) == 0:
        rospy.logwarn("ICP failed: Global map not available")
        return False
    
    if cur_scan is None or len(cur_scan) == 0:
        rospy.logwarn("ICP failed: Current scan not available")
        return False
    
    try:
        rospy.loginfo("Starting memory-efficient ICP localization...")
        
        # 1. 裁剪地图到当前视野
        map_in_fov = crop_global_map_in_FOV(global_map_downsampled, initial_transform)
        
        if len(map_in_fov) < 100:
            rospy.logwarn(f"Only {len(map_in_fov)} points in FOV, skipping ICP")
            return False
        
        # 2. 降采样点云（如果必要）
        if len(cur_scan) > ICP_PARAMS["MAX_POINTS"]:
            cur_scan_down = downsample_points(cur_scan, ICP_PARAMS["SCAN_VOXEL_SIZE"])
        else:
            cur_scan_down = cur_scan
        
        if len(map_in_fov) > ICP_PARAMS["MAX_POINTS"]:
            map_in_fov_down = downsample_points(map_in_fov, ICP_PARAMS["MAP_VOXEL_SIZE"])
        else:
            map_in_fov_down = map_in_fov
        
        rospy.loginfo(f"ICP with {len(cur_scan_down)} scan points and {len(map_in_fov_down)} map points")
        
        # 3. 获取初始变换矩阵
        trans = initial_transform.transform.translation
        rot = initial_transform.transform.rotation
        initial_mat = tf.transformations.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
        initial_mat[0, 3] = trans.x
        initial_mat[1, 3] = trans.y
        initial_mat[2, 3] = trans.z
        
        # 4. 执行ICP
        transformation, fitness = icp_registration(
            cur_scan_down, 
            map_in_fov_down, 
            initial_mat,
            max_iter=ICP_PARAMS["MAX_ITER"]
        )
        
        rospy.loginfo(f"ICP completed with fitness: {fitness:.3f}")
        
        # 5. 检查匹配质量
        if fitness > ICP_PARAMS["LOCALIZATION_TH"]:
            # 更新变换
            trans = tf.transformations.translation_from_matrix(transformation)
            quat = tf.transformations.quaternion_from_matrix(transformation)
            
            T_map_to_camera = TransformStamped()
            T_map_to_camera.header.stamp = rospy.Time.now()
            T_map_to_camera.header.frame_id = "map"
            T_map_to_camera.child_frame_id = "camera_init"
            T_map_to_camera.transform.translation.x = trans[0]
            T_map_to_camera.transform.translation.y = trans[1]
            T_map_to_camera.transform.translation.z = trans[2]
            T_map_to_camera.transform.rotation.x = quat[0]
            T_map_to_camera.transform.rotation.y = quat[1]
            T_map_to_camera.transform.rotation.z = quat[2]
            T_map_to_camera.transform.rotation.w = quat[3]
            
            rospy.loginfo("ICP localization succeeded!")
            return True
        else:
            rospy.logwarn(f"ICP failed: Fitness {fitness:.3f} < threshold {ICP_PARAMS['LOCALIZATION_TH']}")
            return False
            
    except Exception as e:
        rospy.logerr(f"ICP localization failed: {str(e)}")
        return False

def global_localization(T_init):
    """执行全局定位（包含ICP）"""
    if not isinstance(T_init, TransformStamped):
        rospy.logwarn("[GlobalLocalization] Invalid initial pose")
        return
    
    rospy.loginfo("[GlobalLocalization] Starting global localization with ICP...")
    perform_icp_localization(T_init)

def load_global_map(msg: PointCloud2):
    """加载全局地图（优化内存使用）"""
    global global_map, global_map_downsampled
    
    rospy.loginfo("[GlobalLocalization] Loading global map...")
    
    # 1. 转换为numpy数组
    points = pointcloud2_to_numpy(msg)
    rospy.loginfo(f"Original map points: {len(points)}")
    
    # 2. 降采样
    if len(points) > ICP_PARAMS["MAX_POINTS"]:
        global_map = downsample_points(points, ICP_PARAMS["MAP_VOXEL_SIZE"] * 2)
    else:
        global_map = points
    
    # 3. 创建进一步降采样的版本用于ICP
    if len(global_map) > ICP_PARAMS["MAX_POINTS"]:
        global_map_downsampled = downsample_points(global_map, ICP_PARAMS["MAP_VOXEL_SIZE"])
    else:
        global_map_downsampled = global_map
    
    rospy.loginfo(f"[GlobalLocalization] Global map loaded with {len(global_map_downsampled)} points (downsampled).")

def save_current_scan(msg: PointCloud2):
    """保存当前扫描（优化内存使用）"""
    global cur_scan
    
    # 转换为numpy数组并降采样
    points = pointcloud2_to_numpy(msg)
    
    if len(points) > ICP_PARAMS["MAX_POINTS"]:
        cur_scan = downsample_points(points, ICP_PARAMS["SCAN_VOXEL_SIZE"])
    else:
        cur_scan = points
    
    rospy.loginfo(f"Current scan points: {len(cur_scan)}")

def localization_trigger_cb(msg: Bool):
    """重定位触发回调"""
    global localization_triggered
    localization_triggered = msg.data
    rospy.loginfo(f"[GlobalLocalization] Localization triggered: {localization_triggered}")

def thread_localization():
    """定位线程（添加资源检查）"""
    global localization_triggered, initial_pose

    rate = rospy.Rate(0.5)  # 降低定位频率
    while not rospy.is_shutdown():
        if localization_triggered and initial_pose is not None:
            # 检查资源是否可用
            if global_map_downsampled is None or len(global_map_downsampled) == 0:
                rospy.logwarn("Localization skipped: Global map not ready")
            elif cur_scan is None or len(cur_scan) == 0:
                rospy.logwarn("Localization skipped: Current scan not available")
            else:
                rospy.loginfo("[GlobalLocalization] Performing localization...")
                global_localization(initial_pose)
            
            localization_triggered = False
        
        rate.sleep()

def publish_tf():
    """发布TF变换（添加异常处理）"""
    global T_map_to_camera
    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10)  # 降低TF发布频率
    
    while not rospy.is_shutdown():
        try:
            if T_map_to_camera is not None:
                tf_msg = TransformStamped()
                tf_msg.header.stamp = rospy.Time.now()
                tf_msg.header.frame_id = "map"
                tf_msg.child_frame_id = "camera_init"
                tf_msg.transform = T_map_to_camera.transform
                br.sendTransform(tf_msg)
        except Exception as e:
            rospy.logerr(f"TF publishing error: {str(e)}")
        
        rate.sleep()

def initial_pose_cb(msg: PoseWithCovarianceStamped):
    """初始位姿回调（添加有效性检查）"""
    global initial_pose, localization_triggered

    # 检查位姿是否有效
    if not all([abs(msg.pose.pose.position.x) < 1000,
                abs(msg.pose.pose.position.y) < 1000,
                abs(msg.pose.pose.position.z) < 100]):
        rospy.logwarn("Invalid initial pose received, ignoring")
        return

    rospy.loginfo("[GlobalLocalization] Received /initialpose from RViz")

    T = TransformStamped()
    T.header = msg.header
    T.child_frame_id = "camera_init"
    T.transform.translation.x = msg.pose.pose.position.x
    T.transform.translation.y = msg.pose.pose.position.y
    T.transform.translation.z = msg.pose.pose.position.z
    T.transform.rotation = msg.pose.pose.orientation

    initial_pose = T
    localization_triggered = True

def main():
    rospy.init_node("global_localization_node")
    
    # 设置ROS参数（可选）
    rospy.set_param("~map_voxel_size", ICP_PARAMS["MAP_VOXEL_SIZE"])
    rospy.set_param("~scan_voxel_size", ICP_PARAMS["SCAN_VOXEL_SIZE"])
    rospy.set_param("~max_points", ICP_PARAMS["MAX_POINTS"])
    
    # 订阅全局地图、当前扫描和触发信号
    rospy.Subscriber("/global_map", PointCloud2, load_global_map, queue_size=1, buff_size=2**24)  # 增加缓冲区大小
    rospy.Subscriber("/current_scan", PointCloud2, save_current_scan, queue_size=1, buff_size=2**24)
    rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, initial_pose_cb, queue_size=1)
    rospy.Subscriber("/localization_trigger", Bool, localization_trigger_cb, queue_size=1)

    # 启动定位和TF发布线程
    threading.Thread(target=thread_localization, daemon=True).start()
    threading.Thread(target=publish_tf, daemon=True).start()

    rospy.loginfo("Global Localization Node started with memory optimizations")
    rospy.spin()

if __name__ == "__main__":
    main()