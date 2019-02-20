#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import pose_openvr_wrapper
from pose_transform import Transformation
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
        for device in pyopenvr_wrapper.devices:
            pubs[device] = rospy.Publisher(device+'/Pose', PoseStamped, queue_size=10)
            rate = rospy.Rate(10)  # 10hz

        while not rospy.is_shutdown():
            for device, value in pyopenvr_wrapper.devices.items():
                sample = pyopenvr_wrapper.sample(device, samples_count=1)
                pose_stamped_msg = PoseStamped()
                matrix = np.identity(4)
                matrix[:3, :4] = np.asarray(sample['matrix'][0])
                translation = Transformation(matrix)
                if value['type'] == 'tracker':
                    correction_matrix = np.array([[1, 0, 0, 0],
                                                 [0, 0, -1, 0],
                                                 [0, 1, 0, 0],
                                                 [0, 0, 0, 1]])
                    matrix = correction_matrix.dot(matrix)
                    # recalage du repère du tracker
                    x_rotation = np.array([[1, 0, 0],
                                          [0, 0, 1],
                                          [0, -1, 0]])
                    matrix[:3, :3] = matrix[:3, :3].dot(x_rotation)

                corrected_translation = Transformation(matrix)
                corrected_quaternion = corrected_translation.quaternion()
                corrected_position = corrected_translation.position()
                pose_stamped_msg.header.stamp = rospy.Time.now()
                pose_stamped_msg.header.frame_id = 'vive_world'
                pose_stamped_msg.pose.position.x = corrected_position[0]
                pose_stamped_msg.pose.position.y = corrected_position[1]
                pose_stamped_msg.pose.position.z = corrected_position[2]
                pose_stamped_msg.pose.orientation.x = corrected_quaternion.x
                pose_stamped_msg.pose.orientation.y = corrected_quaternion.y
                pose_stamped_msg.pose.orientation.z = corrected_quaternion.z
                pose_stamped_msg.pose.orientation.w = corrected_quaternion.w
                pubs[device].publish(pose_stamped_msg)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
