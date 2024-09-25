import sys,os
sys.path.append(os.getcwd())

try:
    import include.AutoMavCtrl as AutoMavCtrl
    import include.PX4MavCtrlV4 as PX4MavCtrl
except ImportError:
    import AutoMavCtrl
    import PX4MavCtrlV4 as PX4MavCtrl

import time

'''
AutoConf:
    AutoConf[0]: UAV Frame type
        1:  Quadcopter
        2:  Fixedwing

    AutoConf[1]: Software-in-the-loop / Hardware-in-the-loop configuration

    AutoConf[2]: Number of simulated drones
'''

# Multi Frame Multi mav mode
conf = [
    ['Quadcopter', 'SITL', 2],
    ['Fixedwing', 'SITL', 2]
]
'''
    # # Single mav multiple instance mode
    # conf = [
    #     ['Quadcopter', 'SITL', 2]
    # ]
'''
'''
    # # Multi-mav multi-instance mode
    # conf = [
    #     ['Quadcopter', 'SITL', 1],
    #     ['Fixedwing', 'SITL', 2]
    # ]
'''


mav = [
    PX4MavCtrl.PX4MavCtrler(20100),
    PX4MavCtrl.PX4MavCtrler(20102),
    PX4MavCtrl.PX4MavCtrler(20104),
    PX4MavCtrl.PX4MavCtrler(20106)
    ]

'''
    # # In multi-machine mode, if conf configures several drones, configure the corresponding number here.
    # mav = [
    #     PX4MavCtrl.PX4MavCtrler(20100),
    #     PX4MavCtrl.PX4MavCtrler(20102),
    #     PX4MavCtrl.PX4MavCtrler(20104),
    #     ......
    #     ]
'''


ORIGIN_POS_X = 0   # -230
ORIGIN_POS_Y = 0   # 119
ORIGIN_YAW = 0
VEHICLE_INTERVAL = 5
map = [
    'OldFactory', ORIGIN_POS_X, ORIGIN_POS_Y, ORIGIN_YAW, VEHICLE_INTERVAL
]

AutoEnv = AutoMavCtrl.InitMavAutoEnv(mav,conf,map)

# start monitoring aircraft thread
AutoMavCtrl.MavMonitor()

mavAuto1 = AutoMavCtrl.AutoMavCtrler(mav[0],conf[0])
mavAuto1.AutoMavLoopStart()

mavAuto2 = AutoMavCtrl.AutoMavCtrler(mav[1],conf[0])
mavAuto2.AutoMavLoopStart()

mavAuto3 = AutoMavCtrl.AutoMavCtrler(mav[2],conf[1])
mavAuto3.AutoMavLoopStart()

mavAuto4 = AutoMavCtrl.AutoMavCtrler(mav[3],conf[1])
mavAuto4.AutoMavLoopStart()

'''
    # In multi-machine mode, if conf configures several drones, configure the corresponding number here.

    # mavAuto2 = AutoMavCtrl.AutoMavCtrler(mav[1],conf[0])
    # mavAuto2.AutoMavLoopStart()

    # mavAuto3 = AutoMavCtrl.AutoMavCtrler(mav[2],conf[1])
    # mavAuto3.AutoMavLoopStart()

    ......
'''

AutoMavCtrl.SimMonitor()


# while True:
#     print(f'mav1 vel:{mavAuto1.mav.trueVelNED}\t mav1 pos:{mavAuto1.mav.truePosNED}\t mav1 ang:{mavAuto1.mav.trueAccB}')
#     time.sleep(1)

# while True:
#     if mavAuto1.is_alive():
#         print(f'mav1 vel:{mavAuto1.mav.trueVelNED}\t mav1 pos:{mavAuto1.mav.truePosNED}\t mav1 ang:{mavAuto1.mav.trueAccB}')
#         time.sleep(1)