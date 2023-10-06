import numpy as np
import quaternion # pip install numpy-quaternion
from geometry_msgs.msg import PoseStamped, Pose

def orientation_2_quaternion(orientation):
    return np.quaternion(orientation.w, orientation.x, orientation.y, orientation.z)

def position_2_array(position):
    return np.array([position.x, position.y, position.z])

def pose_2_transformation(pose: Pose):
    quaternion_orientation = orientation_2_quaternion(pose.orientation)
    translation = position_2_array(pose.position)
    rotation_matrix = quaternion.as_rotation_matrix(quaternion_orientation)
    transformation_matrix = np.identity(4)
    transformation_matrix[0:3, 0:3] = rotation_matrix
    transformation_matrix[0:3, 3] = translation
    return transformation_matrix

def array_quat_2_pose(pos_array, quat):
    pose_st = PoseStamped()
    pose_st.pose.position.x = pos_array[0]
    pose_st.pose.position.y = pos_array[1]
    pose_st.pose.position.z = pos_array[2]
    pose_st.pose.orientation.x = quat.x
    pose_st.pose.orientation.y = quat.y
    pose_st.pose.orientation.z = quat.z
    pose_st.pose.orientation.w = quat.w
    return pose_st

def transformation_2_pose(transformation_matrix):
    pos_array = transformation_matrix[0:3, 3]
    rotation_matrix = transformation_matrix[0:3, 0:3]
    quat = quaternion.from_rotation_matrix(rotation_matrix)
    pose_st = array_quat_2_pose(pos_array, quat)
    return pose_st

def pose_st_2_transformation(pose_st: PoseStamped):
    transformation_matrix = pose_2_transformation(pose_st.pose)
    return transformation_matrix

def transform_pose(pose: PoseStamped, transformation_matrix):
    pose_as_matrix = pose_st_2_transformation(pose)
    transformed_pose_matrix = transformation_matrix @ pose_as_matrix
    transformed_pose = transformation_2_pose(transformed_pose_matrix)
    return transformed_pose

def transform_pos_ori(pos: np.array, ori, transform):
    ori_quat = list_2_quaternion(ori)
    ori_rot_matrix = quaternion.as_rotation_matrix(ori_quat)
    transformed_ori_rot_matrix = transform[:3,:3] @ ori_rot_matrix
    pos = np.hstack((pos, 1))
    transformed_pos = transform @ pos
    transformed_ori_quat = quaternion.from_rotation_matrix(transformed_ori_rot_matrix)
    transformed_ori_array = np.array([transformed_ori_quat.w, transformed_ori_quat.x, transformed_ori_quat.y, transformed_ori_quat.z])
    return transformed_pos[:3], transformed_ori_array

def list_2_quaternion(quaternion_list: list):
    return np.quaternion(quaternion_list[0], quaternion_list[1], quaternion_list[2], quaternion_list[3])

