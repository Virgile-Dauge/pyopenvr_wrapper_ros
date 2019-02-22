#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""coucou."""
import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
import pose_openvr_wrapper
import pprint as pp
from pose_transform import Transform
from pyopenvr_wrapper_ros.srv import GetPoseStamped, GetRelativePoseStamped

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
        'get_relative_poseStamped', GetRelativePoseStamped,
        send_relative_poseStamped)

    rospy.spin()
