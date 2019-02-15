#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""coucou."""
import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
import pose_openvr_wrapper
import pprint as pp
from pyopenvr_wrapper_ros.srv import GetPoseStamped, GetRelativePoseStamped

def send_poseStamped(req):
    # We get samples from device
    sample = pyopenvr_wrapper.sample(
        req.device, samples_count=req.samples,
        sampling_frequency=req.frequency)
    t = PoseStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = req.device
    t.pose.position.x = np.mean(sample['x'])
    t.pose.position.z = np.mean(sample['y'])
    t.pose.position.y = np.mean(sample['z'])
    t.pose.orientation.w = np.mean(sample['r_w'])
    t.pose.orientation.x = np.mean(sample['r_x'])
    t.pose.orientation.y = np.mean(sample['r_y'])
    t.pose.orientation.z = np.mean(sample['r_z'])
    return t

def send_relative_poseStamped(req):
    # We get samples from device
    sample = pyopenvr_wrapper.sample(
        req.device, samples_count=req.samples,
        sampling_frequency=req.frequency, ref_device_key=req.reference)
    t = PoseStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = req.reference
    t.pose.position.x = np.mean(sample['x'])
    t.pose.position.z = np.mean(sample['y'])
    t.pose.position.y = np.mean(sample['z'])
    t.pose.orientation.w = np.mean(sample['r_w'])
    t.pose.orientation.x = np.mean(sample['r_x'])
    t.pose.orientation.y = np.mean(sample['r_y'])
    t.pose.orientation.z = np.mean(sample['r_z'])
    return t


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
