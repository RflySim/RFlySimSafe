import time
import numpy as np
import math
    
class Sleep:
    def __init__(self,mav):
        self.CID = 1
        self.mav = mav
        self.isDone = 0 
        self.WaitFlag = 0
        self.WaitResetFlag = 0
        self.start_time = 0
    
    def Wait(self,times): 
        self.isDone = 0
        if self.WaitFlag == 0:
            print('wait {}s'.format(times[0]))
            self.start_time = time.time() + times[0]
            self.WaitFlag = 1
    
        if self.start_time - time.time() < 0: 
            self.isDone = 1
            self.WaitFlag = 0

class Command:
    def __init__(self,mav):
        self.CID = 2
        self.mav = mav
        self.ARMFLAG = False 
        self.isDone = 0 
        self.RECORDFLAG = False
        self.LANDFLAG = False
        self.LANDFLAGTAG = False
        self.silInt = np.zeros(8).astype(int).tolist()
        self.silFloats = np.zeros(20).astype(float).tolist()
        self.INJECTFLAG = False
        self.FAULTID = 0
        self.isInitOff = 0

    def Arm(self): 
        self.isDone = 0
        self.mav.SendMavArm(1)
        print('Armed')
        self.mav.sendUE4Cmd(b'RflyShowTextTime "Armed" 10')
        self.ARMFLAG = True
        self.isDone = 1
        self.RECORDFLAG = True

    def DisArm(self): 
        self.isDone = 0
        self.mav.SendMavArm(0) 
        print('DisArmed') 
        self.mav.sendUE4Cmd(b'RflyShowTextTime "DisArmed" 10')
        self.isDone = 1

    def QuadPos(self,pos):
        self.isDone = 0
        self.mav.SendMavArm(1)
        self.mav.SendPosNED(pos[0],pos[1],pos[2])
        print('Send Pos {}'.format(pos))
        self.mav.sendUE4Cmd(b'RflyShowTextTime "Send Pos Cmd:%.2f %.2f %.2f" 10'%(pos[0],pos[1],pos[2]))
        self.isDone = 1

    def QuadVel(self,vel): 
        self.isDone = 0
        self.mav.SendVelNED(vel[0],vel[1],vel[2])
        print('Send Vel {}'.format(vel))
        self.mav.sendUE4Cmd(b'RflyShowTextTime "Send Vel Cmd:%.2f %.2f %.2f" 10'%(vel[0],vel[1],vel[2]))
        self.isDone = 1

    def UAVLand(self,pos): 
        self.isDone = 0
        self.LANDFLAG = True
        if self.LANDFLAGTAG == False:
            self.mav.sendMavLand(pos[0],pos[1],pos[2])
            print('Start Landing')
            self.mav.sendUE4Cmd(b'RflyShowTextTime "Start Landing:%.2f %.2f %.2f" 10'%(pos[0],pos[1],pos[2]))
            self.LANDFLAGTAG = True
        if abs(self.mav.truePosNED[2]) < 1.5:
            print('Landed')
            self.isDone = 1
            self.LANDFLAGTAG = False
        
    def FaultInject(self,param): 
        self.isDone = 0
        self.inInts = np.array([])
        self.inFloats = np.array([])
        for i in range(len(param)):
            if param[i] >= 123450:
                self.inInts = np.append(self.inInts,param[i])
            else:
                self.inFloats = np.append(self.inFloats,param[i])
        
        for i in range(len(self.inInts)):
            self.silInt[i] = self.inInts[i].astype(int)
        for i in range(len(self.inFloats)):
            self.silFloats[i] = self.inFloats[i].astype(np.double)

        if self.silInt[0] == 123450 or self.silInt[0] == 123451:
            self.mav.sendUE4Cmd(b'RflySetActuatorPWMsExt 1 1')

        print('Start Inject Fault')
        self.mav.sendUE4Cmd(b'RflyShowTextTime "Fault Params: %.2f %.2f %.2f" 10'%(self.silFloats[0],self.silFloats[1],self.silFloats[2]))
        self.mav.sendUE4Cmd(b'RflyShowTextTime "Start Inject %d Fault" 10'%(self.silInt[0]))
        self.mav.sendSILIntFloat(self.silInt,self.silFloats)
        self.mav.SendMavCmdLong(183,999,999,999,999,999,999,999)
        self.FAULTID = self.silInt[0]
        self.isDone = 1
        self.INJECTFLAG = True

class CmdCtrl:
    def __init__(self,mav,frame):
        self.mav = mav
        self.frame = frame
        self.CID = {
        '1':Sleep(mav),
        '2':Command(mav)
        }
        self.CID1 = self.CID['1']
        self.CID2 = self.CID['2']
        self.FID = 0
    
    def GetWaitseq(self):
        Waitseq = {
                '1':self.CID1.Wait,
        }
        return Waitseq

    def GetCmdseq(self):
        if self.frame == 1: 
            Cmdseq = {
                '1':self.CID2.Arm,
                '2':self.CID2.DisArm,
                '3':self.CID2.QuadPos,
                '4':self.CID2.QuadVel,
                '5':self.CID2.UAVLand,
                '6':self.CID2.FaultInject
            }
        return Cmdseq

    def FIDPro(self,cmdCID):
        if cmdCID == '1':
            self.FID = CmdCtrl.GetWaitseq(self)
        elif cmdCID == '2':
            self.FID = CmdCtrl.GetCmdseq(self)
        return self.FID
