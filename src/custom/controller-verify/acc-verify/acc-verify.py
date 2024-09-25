import os, sys
sys.path.append(os.getcwd())


import src.custom.block.GenSPO as GenSPO
import src.custom.block.MavAPI as MavAPI
import src.custom.block.SimAPI as SimAPI
    

import time


if __name__ == '__main__':

    # # 生成圆形轨迹
    # radius_circle = 30
    # height_circle = -15
    # num_points = 1000
    # x_circle, y_circle, z_circle = GenSPO.GenSPo().Gen_Circle_SPo(radius_circle, height_circle, num_points)

    # mav = PX4MavCtrl.PX4MavCtrler(Com='COM7:57600')
    # mav = MavAPI.MavAPI(frame='Quadcopter', SimMode='SITL', ID=20100)
    mav = MavAPI.MavAPI(frame='Quadcopter', SimMode='SITL', ID='COM7:57600')
    # mav.Get_SPVo(x_circle, y_circle, z_circle)

    mav.AutoMavRun()

    # ctrls = [-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1]
    # modes = 11
    # flags = 4

    # mav.SendHILCtrlMsg(ctrls,modes,flags)
    # time.sleep(0.5)
    # print(f"Start Fault Inject! \n Fault Mode:{modes} \n Fault Flags:{flags} \n Fault Params:{ctrls}")

    # time.sleep(5)
