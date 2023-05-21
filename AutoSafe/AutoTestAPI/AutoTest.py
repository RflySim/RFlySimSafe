import AutoMavCtrl
import PX4MavCtrlV4 as PX4MavCtrl

'''
Frame:
    1: Quad

TestBatPath:
    [1,'Quadrotor','QuadModelSITL.bat']

'''

TestBatPath1=[1,'Quadrotor','QuadModelSITL.bat']
mav1 = PX4MavCtrl.PX4MavCtrler(20100)
mav  = [mav1]
TestBatpath = [TestBatPath1]
AutoEnv = AutoMavCtrl.InitMavAutoEnv(mav,TestBatpath,map='NeighborhoodPark')
'''
Map:
    Grasslands 
    NeighborhoodPark
    OldFactory
    ...
'''
mavAuto1 = AutoMavCtrl.AutoMavCtrler(mav1,TestBatPath1)
mavAuto1.AutoMavLoopStart()


