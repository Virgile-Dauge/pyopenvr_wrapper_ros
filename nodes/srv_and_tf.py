#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""coucou."""
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
import pose_openvr_wrapper
import pprint as pp
from pose_transform import Transform
from pyopenvr_wrapper_ros.srv import GetPoseStamped
from pyopenvr_wrapper_ros.srv import GetAllPoseStamped
from pyopenvr_wrapper_ros.srv import GetAllPoseStampedResponse
from pyopenvr_wrapper_ros.srv import GetRelativePoseStamped
from pyopenvr_wrapper_ros.srv import GetAllRelativePoseStamped
from pyopenvr_wrapper_ros.srv import GetAllRelativePoseStampedResponse

def send_poseStamped(req):
    # We get samples from device
    transform = Transform(
        pyopenvr_wrapper.get_corrected_transformation_matrix(
            target_device_key=req.device,
            samples_count=req.samples,
            sampling_frequency=req.frequency))
    quaternion = transform.quaternion()
    position = transform.position()
    pose_stamped_msg = PoseStamped()
    pose_stamped_msg.header.stamp = rospy.Time.now()
    pose_stamped_msg.header.frame_id = 'map'
    pose_stamped_msg.pose.position.x = position[0]
    pose_stamped_msg.pose.position.y = position[1]
    pose_stamped_msg.pose.position.z = position[2]
    pose_stamped_msg.pose.orientation.x = quaternion.x
    pose_stamped_msg.pose.orientation.y = quaternion.y
    pose_stamped_msg.pose.orientation.z = quaternion.z
    pose_stamped_msg.pose.orientation.w = quaternion.w
    return pose_stamped_msg

def send_all_poseStamped(req):
    # We get samples from device
    dict = pyopenvr_wrapper.get_all_transformation_matrices(
        samples_count=req.samples,
        sampling_frequency=req.frequency)
    keys = []
    poses = []
    for key, matrix in dict.items():
        keys.append(key)
        transform = Transform(matrix)
        quaternion = transform.quaternion()
        position = transform.position()
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.stamp = rospy.Time.now()
        pose_stamped_msg.header.frame_id = 'map'
        pose_stamped_msg.pose.position.x = position[0]
        pose_stamped_msg.pose.position.y = position[1]
        pose_stamped_msg.pose.position.z = position[2]
        pose_stamped_msg.pose.orientation.x = quaternion.x
        pose_stamped_msg.pose.orientation.y = quaternion.y
        pose_stamped_msg.pose.orientation.z = quaternion.z
        pose_stamped_msg.pose.orientation.w = quaternion.w
        poses.append(pose_stamped_msg)

    response = GetAllPoseStampedResponse()
    response.keys = keys
    response.poses = poses
    return response



def send_relative_poseStamped(req):
    # We get samples from device
    transform = Transform(
        pyopenvr_wrapper.get_corrected_transformation_matrix(
            target_device_key=req.device,
            ref_device_key=req.reference,
            samples_count=req.samples,
            sampling_frequency=req.frequency))
    quaternion = transform.quaternion()
    position = transform.position()
    pose_stamped_msg = PoseStamped()
    pose_stamped_msg.header.stamp = rospy.Time.now()
    pose_stamped_msg.header.frame_id = 'map'
    pose_stamped_msg.pose.position.x = position[0]
    pose_stamped_msg.pose.position.y = position[1]
    pose_stamped_msg.pose.position.z = position[2]
    pose_stamped_msg.pose.orientation.x = quaternion.x
    pose_stamped_msg.pose.orientation.y = quaternion.y
    pose_stamped_msg.pose.orientation.z = quaternion.z
    pose_stamped_msg.pose.orientation.w = quaternion.w
    return pose_stamped_msg

def send_all_relative_poseStamped(req):
    # We get samples from device
    dict = pyopenvr_wrapper.get_all_transformation_matrices(
        ref_device_key=req.reference,
        samples_count=req.samples,
        sampling_frequency=req.frequency)
    keys = []
    poses = []
    for key, matrix in dict.items():
        keys.append(key)
        transform = Transform(matrix)
        quaternion = transform.quaternion()
        position = transform.position()
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.stamp = rospy.Time.now()
        pose_stamped_msg.header.frame_id = 'map'
        pose_stamped_msg.pose.position.x = position[0]
        pose_stamped_msg.pose.position.y = position[1]
        pose_stamped_msg.pose.position.z = position[2]
        pose_stamped_msg.pose.orientation.x = quaternion.x
        pose_stamped_msg.pose.orientation.y = quaternion.y
        pose_stamped_msg.pose.orientation.z = quaternion.z
        pose_stamped_msg.pose.orientation.w = quaternion.w
        poses.append(pose_stamped_msg)

    response = GetAllRelativePoseStampedResponse()
    response.keys = keys
    response.poses = poses
    return response

if __name__ == '__main__':

    rospy.init_node('service_pose_node')
    rate = rospy.Rate(10)  # 10hz
    steamVR_is_running = False
    while(not steamVR_is_running):
        try:
            pyopenvr_wrapper = pose_openvr_wrapper.OpenvrWrapper(
                '../cfg/config.json')
            steamVR_is_running = True
        except rospy.ROSInterruptException:
            print('steamVR_not_running')
            rate.sleep()
    print('SteamVR is running, printing dicovered devices :')
    pp.pprint(pyopenvr_wrapper.devices)

    service = rospy.Service(
        'get_poseStamped', GetPoseStamped, send_poseStamped)

    service = rospy.Service(
        'get_all_poseStamped', GetAllPoseStamped, send_all_poseStamped)

    service = rospy.Service(
        'get_relative_poseStamped', GetRelativePoseStamped,
        send_relative_poseStamped)

    service = rospy.Service(
        'get_all_relative_poseStamped', GetAllRelativePoseStamped,
        send_all_relative_poseStamped)

    try:
        broadcaster = tf2_ros.TransformBroadcaster()
        rate = rospy.Rate(250)
        while not rospy.is_shutdown():
            matrices = pyopenvr_wrapper.get_all_transformation_matrices(
                samples_count=1)
            for device, matrix in matrices.items():
                transform = Transform(
                    pyopenvr_wrapper.get_all_transformation_matrices(
                        samples_count=1))
                quaternion = transform.quaternion()
                position = transform.position()
                broadcaster.sendTransform(
                    position,
                    (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                    rospy.Time.now(),
                    device,
                    "local")

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
