#!/usr/bin/env python3
import os
import subprocess

if __name__ == '__main__':
    steam_runtime = os.path.expanduser('~/.steam/steam/ubuntu12_32/steam-runtime/run.sh')
    vr_monitor = os.path.expanduser('~/.steam/steam/steamapps/common/SteamVR/bin/vrmonitor.sh')
    subprocess.run([steam_runtime, vr_monitor],
                   stderr=subprocess.DEVNULL,
                   stdout=subprocess.DEVNULL)
