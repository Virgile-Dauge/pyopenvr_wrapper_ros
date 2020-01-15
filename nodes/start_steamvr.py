#!/usr/bin/env python3
import os
import sys
import time
import signal
import subprocess

import rospy
from std_srvs.srv import Trigger, TriggerResponse

class steamvr_process:
    def __init__(self):
        self.steam_runtime = os.path.expanduser('~/.steam/steam/ubuntu12_32/steam-runtime/run.sh')
        self.vr_monitor = os.path.expanduser('~/.steam/steam/steamapps/common/SteamVR/bin/vrmonitor.sh')
        self.proc = None
        signal.signal(signal.SIGINT, self.sigint_handler)
        self.reset_service = rospy.Service("/restart_steamvr",
                                           Trigger, self.restart_handler)

    def sigint_handler(self, signal_received, frame):
        # Handle any cleanup here
        self.kill()
        print('SIGINT or CTRL-C detected. Exiting gracefully')
        sys.exit(0)

    def start(self):
        self.proc = subprocess.Popen([self.steam_runtime, self.vr_monitor],
                                     stderr=subprocess.DEVNULL,
                                     stdout=subprocess.DEVNULL)
    def kill(self):
        self.proc.kill()

    def restart(self):
        self.kill()
        time.sleep(0.1)
        self.start()

    def restart_handler(self, msg=None):
        self.restart()
        res = TriggerResponse()
        if self.proc.pid is not None:
            res.success = True
        return res


if __name__ == '__main__':
    rospy.init_node('steamvr_node')
    vr = steamvr_process()
    vr.start()
    rospy.spin()
