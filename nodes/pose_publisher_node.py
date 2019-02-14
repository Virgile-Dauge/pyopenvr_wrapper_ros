#!/usr/bin/env python

import rospy
import pose_openvr_wrapper
import pprint as pp
from geometry_msgs.msg import Pose

if __name__ == '__main__':
    try:
        rospy.init_node('talker', anonymous=True)
        pyopenvr_wrapper = pose_openvr_wrapper.OpenvrWrapper(
            '../cfg/config.json')
        pp.pprint(pyopenvr_wrapper.devices)
        pubs = {}
        for device in pyopenvr_wrapper.devices:
            pubs[device] = rospy.Publisher(device+'/Pose', Pose, queue_size=10)
            rate = rospy.Rate(10)  # 10hz

        while not rospy.is_shutdown():
            for device in pyopenvr_wrapper.devices:
                sample = pyopenvr_wrapper.sample(device, samples_count=1)
                pose = Pose()
                pose.position.x = sample['x'][0]
                pose.position.y = sample['y'][0]
                pose.position.z = sample['z'][0]
                pose.orientation.x = sample['r_x'][0]
                pose.orientation.y = sample['r_y'][0]
                pose.orientation.z = sample['r_z'][0]
                pose.orientation.w = sample['r_w'][0]
                pubs[device].publish(pose)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
