import os, sys
sys.path.append(os.getcwd())


import src.custom.block.GenSPO as GenSPO
import src.custom.block.MavAPI as MavAPI
import src.custom.block.SimAPI as SimAPI
    

import time


if __name__ == '__main__':

    
    mav = MavAPI.MavAPI(frame='Quadcopter', SimMode='SITL', ID='COM7:57600')
    # mav = MavAPI.MavAPI(frame='Quadcopter', SimMode='SITL', ID=20100)

    mav.AutoMavRun()

