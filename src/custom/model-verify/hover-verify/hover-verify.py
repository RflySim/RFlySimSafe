import os, sys
sys.path.append(os.getcwd())

try:
    import include.PX4MavCtrlV4 as PX4MavCtrl
except ImportError:
    import PX4MavCtrlV4 as PX4MavCtrl    

import time


if __name__ == '__main__':


    # mav = PX4MavCtrl.PX4MavCtrler(Com='COM7:57600')
    mav = PX4MavCtrl.PX4MavCtrler(20100)
    time.sleep(0.5)
    print('Connect mav!')

    mav.InitMavLoop()
    time.sleep(0.5)


    mav.initOffboard()
    time.sleep(0.5)

    mav.SendMavArm(1)
    time.sleep(0.5)
    print('解锁')

    mav.SendPosNED(0,0,-10)
    print('飞至10米')
    time.sleep(25)
    print('等待25s')

    mav.SendPosNED(0,0,0)
    print('降落')
    time.sleep(14)

    mav.SendMavArm(0)
    

