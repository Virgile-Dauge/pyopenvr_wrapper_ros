#!/usr/bin/env python3
import os
import sys
import time
import signal
import subprocess

import rospy
import tf
from std_srvs.srv import Trigger, TriggerResponse

import pose_openvr_wrapper
from pose_transform import Transform

class steamvr_process:
    def __init__(self):
        self.steam_runtime = os.path.expanduser('~/.steam/steam/ubuntu12_32/steam-runtime/run.sh')
        self.vr_monitor = os.path.expanduser('~/.steam/steam/steamapps/common/SteamVR/bin/vrmonitor.sh')
        self.proc = None
        signal.signal(signal.SIGINT, self.sigint_handler)

        self.is_running = False

    def sigint_handler(self, signal_received, frame):
        # Handle any cleanup here
        self.kill()
        print('SIGINT or CTRL-C detected. Exiting gracefully')
        sys.exit(0)

    def start(self, waiting_time=0.5):
        self.proc = subprocess.Popen([self.steam_runtime, self.vr_monitor],
                                     stderr=subprocess.DEVNULL,
                                     stdout=subprocess.DEVNULL)
        time.sleep(waiting_time)
        self.is_running = True

    def kill(self):
        self.is_running = False
        self.proc.kill()

    def restart(self):
        self.kill()
        time.sleep(0.1)
        self.start()

class pose_wrapper:
    def __init__(self):
        rospy.init_node('pose_wrapper_node')
        self.up_and_running = False
        self.vr_process = steamvr_process()
        self.broadcaster = tf.TransformBroadcaster()
        self.reset_srv = rospy.Service("/restart_steamvr",
            Trigger, self.restart_handler)
        self.poser = None


    def restart_handler(self, msg=None):
        self.stop()
        self.vr_process.restart()
        res = TriggerResponse()
        if self.vr_process.proc.pid is not None:
            res.success = True
        self.start()
        return res

    def stop(self):
        self.up_and_running = False
        if self.poser is not None:
            self.poser.shutdown()

    def start(self):
        self.vr_process.start()
        self.reset_lighthouse_db()
        rate = rospy.Rate(10)  # 10hz
        steamvr_launch_ok = False
        while not steamvr_launch_ok:
            try:
                self.poser = pose_openvr_wrapper.OpenvrWrapper(
                    '../cfg/config.json')
                steamvr_launch_ok = True
            except rospy.ROSInterruptException:
                #print('steamVR_not_running')
                pass
            rate.sleep()
        self.up_and_running = True
        rospy.loginfo('poser up and running')


    def spin(self, frequency=250):
        rate = rospy.Rate(frequency)  # 10hz
        while not rospy.is_shutdown():
            if self.up_and_running:
                matrices = self.poser.get_all_transformation_matrices(
                    samples_count=1)
                for device, matrix in matrices.items():
                    transform = Transform(matrix)
                    quaternion = transform.quaternion()
                    position = transform.position()
                    self.broadcaster.sendTransform(
                        position,
                        (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                        rospy.Time.now(),
                        device,
                        "local")
            rate.sleep()

    def reset_lighthouse_db(self):
        lighthouse_db = os.path.expanduser('~/.steam/debian-installation/config/lighthouse/lighthousedb.json')
        if os.path.exists(lighthouse_db):
            subprocess.run(['rm', lighthouse_db])

if __name__ == '__main__':
    poser = pose_wrapper()
    poser.start()
    poser.spin()
