#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import pose_openvr_wrapper
from pose_transform import Transform
import pprint as pp
import numpy as np
from geometry_msgs.msg import PoseStamped
import math

if __name__ == '__main__':
    try:
        rospy.init_node('publisher_pose_node')
        pyopenvr_wrapper = pose_openvr_wrapper.OpenvrWrapper(
            '../cfg/config.json')
        pp.pprint(pyopenvr_wrapper.devices)
        pubs = {}
        rate = rospy.Rate(10)  # 10hz
        for device in pyopenvr_wrapper.devices:
            pubs[device] = rospy.Publisher(device+'/Pose', PoseStamped, queue_size=10)

        while not rospy.is_shutdown():
            for device, value in pyopenvr_wrapper.devices.items():
                pose_stamped_msg = PoseStamped()
                transform = Transform(
                    pyopenvr_wrapper.get_corrected_transformation_matrix(
                        device, samples_count=1))
                quaternion = transform.quaternion()
                position = transform.position()
                pose_stamped_msg.header.stamp = rospy.Time.now()
                pose_stamped_msg.header.frame_id = 'map'
                pose_stamped_msg.pose.position.x = position[0]
                pose_stamped_msg.pose.position.y = position[1]
                pose_stamped_msg.pose.position.z = position[2]
                pose_stamped_msg.pose.orientation.x = quaternion.x
                pose_stamped_msg.pose.orientation.y = quaternion.y
                pose_stamped_msg.pose.orientation.z = quaternion.z
                pose_stamped_msg.pose.orientation.w = quaternion.w
                pubs[device].publish(pose_stamped_msg)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
