import sys
sys.path.insert(0, "/home/alois/meta-quest-teleoperate/unitree_sdk2_python")
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

ChannelFactoryInitialize(0, "enx000ec6c3d44a")
c = MotionSwitcherClient()
c.SetTimeout(5.0)
c.Init()
print("before:", c.CheckMode())
code, _ = c.ReleaseMode()
print("release code:", code)
import time; time.sleep(2)
print("after:", c.CheckMode())
