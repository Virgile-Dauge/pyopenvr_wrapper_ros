#!/usr/bin/env python
import rospy

import tf
import pose_openvr_wrapper
from pose_transform import Transform
import pprint as pp

if __name__ == '__main__':
    try:
        rospy.init_node('publisher_tf_node')
        pyopenvr_wrapper = pose_openvr_wrapper.OpenvrWrapper(
            '../cfg/config.json')
        pp.pprint(pyopenvr_wrapper.devices)
        broadcaster = tf.TransformBroadcaster()
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
