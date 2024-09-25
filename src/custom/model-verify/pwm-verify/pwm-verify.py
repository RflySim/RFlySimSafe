import os, sys
sys.path.append(os.getcwd())

import numpy as np

try:
    import include.PX4MavCtrlV4 as PX4MavCtrl
except ImportError:
    import PX4MavCtrlV4 as PX4MavCtrl
    

import time
import matplotlib.pyplot as plt

from scipy.stats import norm

def cumulative_normal(x):
    mu = 5  # 平均值
    sigma = 1.5  # 标准差
    y = norm.cdf(x, mu, sigma)
    return y


if __name__ == '__main__':

    # mav = PX4MavCtrl.PX4MavCtrler(Com='COM7:57600')
    mav = PX4MavCtrl.PX4MavCtrler(20100)
    time.sleep(0.5)
    print('Connect mav!')

    mav.InitMavLoop()
    time.sleep(0.5)

    x = np.linspace(0,100,100)
    y = cumulative_normal(x)
    y = y * (1800 - 1000) + 1000
    y_ = [int(p) for p in y]

    pwm_x = x[:10]
    pwm_ = y_[0:10]
    xp = np.linspace(0,10,num=40)
    y_new = np.interp(xp, pwm_x, pwm_)
    pwm_y = [int(p) for p in y_new]

    pwms = [1500, 1500, 1000, 1500, 1100, 1100, 1500, 1500]
    mav.SendRCPwms(pwms)
    mav.initRCSendLoop()
    time.sleep(1)

    mav.SendMavArm(1)
    time.sleep(0.5)

    start = time.time()

    for p in pwm_y:
        pwms[2] = p
        mav.SendRCPwms(pwms)
        time.sleep(0.5)
        print(f'Send pwm {pwms[2]}')

    # ctrls = [-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1]
    # modes = 11
    # flags = 4

    # mav.SendHILCtrlMsg(ctrls,modes,flags)
    # time.sleep(0.5)
    # print(f"Start Fault Inject! \n Fault Mode:{modes} \n Fault Flags:{flags} \n Fault Params:{ctrls}")

    # time.sleep(5)

    pwms = [1500, 1500, 900, 1500, 1100, 1100, 1500, 1500]
    mav.SendRCPwms(pwms)
    print('Stop send RC ctrl')

    time.sleep(0.5)
    mav.endRCSendLoop()
    print('Exit Test!')