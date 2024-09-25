import numpy as np
import re
import time
import AutoMavDB
import sys,os,subprocess
import threading
import AutoMavCmd
import cv2
import MavProfustSafty
import tkinter
import PX4MavCtrlV4 as PX4MavCtrl
from tkinter.messagebox import *
import AutoREG

class InitMavAutoEnv:
    def __init__(self,mav,TestBatPath,map='OldFactory',Interval=5):
        self.mav = mav
        self.map = map
        self.Interval = Interval
        self.TestBatPath = TestBatPath
        AutoREG.REG.MAV_ALL_NUM = len(mav)
        self.BInfo()
        self.BReset()
        self.BatEnvConf()

    def BInfo(self):
        self.QuadMavNum = 0
        self.FixdwingMavNum = 0
        self.USVMavNum = 0
        for i in range(len(self.TestBatPath)):
            AutoREG.REG.ALL_MAV_FRAME.append(self.TestBatPath[i][0])
            if self.TestBatPath[i][0] == 1:
                self.QuadMavNum += 1
            elif self.TestBatPath[i][0] == 2:
                self.FixdwingMavNum += 1
            elif self.TestBatPath[i][0] == 3:
                self.USVMavNum += 1
        AutoREG.REG.FRAME_TYPE = list(set(AutoREG.REG.ALL_MAV_FRAME)) 
        AutoREG.REG.START_INDEX = [INDEX+1 for INDEX in range(AutoREG.REG.MAV_ALL_NUM)] 
        
        if len(AutoREG.REG.FRAME_TYPE) == 1 and AutoREG.REG.MAV_ALL_NUM == 1:
            AutoREG.REG.TEST_MODE = 1 # Single model single instance
        elif len(AutoREG.REG.FRAME_TYPE) == 1 and AutoREG.REG.MAV_ALL_NUM > 1:
            AutoREG.REG.TEST_MODE = 2 # Single model multi-instance
        elif len(AutoREG.REG.FRAME_TYPE) > 1 and AutoREG.REG.MAV_ALL_NUM == len(AutoREG.REG.FRAME_TYPE):
            AutoREG.REG.TEST_MODE = 3 # Multiple models and single instance
        else:
            AutoREG.REG.TEST_MODE = 4 # Multiple models and instances

    def BReset(self):
        for i in range(len(AutoREG.REG.FRAME_TYPE)):
            if AutoREG.REG.FRAME_TYPE[i] == 1:
                BatPath = self.GetPath(1)
                self.BSingle(BatPath)
            elif AutoREG.REG.FRAME_TYPE[i] == 2: 
                BatPath = self.GetPath(2)
                self.BSingle(BatPath)
            elif AutoREG.REG.FRAME_TYPE[i] == 3:
                BatPath = self.GetPath(3)
                self.BSingle(BatPath)
    
    def BSingle(self,Path):
        newdata = ''
        with open(Path,mode='r',encoding='UTF-8') as fline:
            for line in fline:
                if  line.find('SET /a START_INDEX=')!= -1:
                    line = line.replace(line,'SET /a START_INDEX=1 \n')
                if  line.find('SET /A VehicleNum=')!= -1:
                    line = line.replace(line,'SET /A VehicleNum=1 \n')
                if  line.find('REM tasklist|find /i "QGroundControl.exe" || start %PSP_PATH%\QGroundControl\QGroundControl.exe')!= -1:
                    line = line.replace(line,'tasklist|find /i "QGroundControl.exe" || start %PSP_PATH%\QGroundControl\QGroundControl.exe \n')
                if  line.find('REM tasklist|find /i "RflySim3D.exe" || start %PSP_PATH%\RflySim3D\RflySim3D.exe')!= -1:
                    line = line.replace(line,'tasklist|find /i "RflySim3D.exe" || start %PSP_PATH%\RflySim3D\RflySim3D.exe \n')
                if  line.find('REM tasklist|find /i "CopterSim.exe" && taskkill /im "CopterSim.exe"')!= -1:
                    line = line.replace(line,'tasklist|find /i "CopterSim.exe" && taskkill /im "CopterSim.exe" \n')
                if  line.find('REM tasklist|find /i "CopterSim.exe" && taskkill /im "CopterSim.exe"')!= -1:
                    line = line.replace(line,'tasklist|find /i "CopterSim.exe" && taskkill /im "CopterSim.exe" \n')
                if  line.find('REM tasklist|find /i "QGroundControl.exe" && taskkill /f /im "QGroundControl.exe"')!= -1:
                    line = line.replace(line,'tasklist|find /i "QGroundControl.exe" && taskkill /f /im "QGroundControl.exe" \n')
                if  line.find('REM tasklist|find /i "RflySim3D.exe" && taskkill /f /im "RflySim3D.exe"')!= -1:
                    line = line.replace(line,'tasklist|find /i "RflySim3D.exe" && taskkill /f /im "RflySim3D.exe" \n')
                newdata += line

        with open(Path,mode='w',encoding='UTF-8') as f:
            f.write(newdata)
    
    def BatEnvConf(self):
        # In the case of multiple aircraft, it is necessary to modify the start of the Bat environment, the index and the number of aircraft
        # 1、Keep the start of only one Bat environment + START_INDEX + VehicleNum
        START_INDEX = 1
        IsFirst = True
        LastVech = 0
        AutoREG.REG.START_INDEX = [1]
        for i in range(len(AutoREG.REG.FRAME_TYPE)):
            if AutoREG.REG.FRAME_TYPE[i] == 1: # Quad
                if IsFirst:
                    BatPath = self.GetPath(1)
                    self.BVNum(BatPath,self.QuadMavNum)
                    LastVech = 1
                    IsFirst = False
            elif AutoREG.REG.FRAME_TYPE[i] == 2: # Fixed
                if IsFirst:
                    BatPath = self.GetPath(2)
                    self.BVNum(BatPath,self.FixdwingMavNum)
                    LastVech = 2
                    IsFirst = False
                elif LastVech == 1:
                    START_INDEX += self.QuadMavNum
                    BatPath = self.GetPath(2)
                    self.BEnv(BatPath,START_INDEX,self.FixdwingMavNum)
                    LastVech = 2
            elif AutoREG.REG.FRAME_TYPE[i] == 3: # USV
                if IsFirst:
                    BatPath = self.GetPath(3)
                    self.BVNum(BatPath,self.USVMavNum)
                    LastVech = 3
                    IsFirst = False
                elif LastVech == 2:
                    START_INDEX += self.FixdwingMavNum
                    BatPath = self.GetPath(3)
                    self.BEnv(BatPath,START_INDEX,self.USVMavNum)
                    LastVech = 3
        
        # Synchronous scene + initial position + interval
        for i in range(len(AutoREG.REG.FRAME_TYPE)):
            BatPath = self.GetPath(AutoREG.REG.FRAME_TYPE[i])
            '''
            Grasslands 
            NeighborhoodPark
            OldFactory
            RealForestBLake
            '''
            self.BMap(BatPath)

    def BVNum(self,path,num):
        newdata = ''
        with open(path,mode='r',encoding='UTF-8') as fline:
            for line in fline:
                if  line.find('SET /A VehicleNum=')!= -1:
                    line = line.replace(line,'SET /A VehicleNum={} \n'.format(num))
                newdata += line

        with open(path,mode='w',encoding='UTF-8') as f:
            f.write(newdata)

    def BMap(self,path):
        '''
        Grasslands
        OldFactory
        RealForestBLake
        NeighborhoodPark
        '''
        newdata = ''
        with open(path,mode='r',encoding='UTF-8') as fline:
            for line in fline:
                if  line.find('SET UE4_MAP=')!= -1:
                    line = line.replace(line,'SET UE4_MAP={} \n'.format(self.map))
                if  line.find('SET /a VEHICLE_INTERVAL=')!= -1:
                    line = line.replace(line,'SET /a VEHICLE_INTERVAL={} \n'.format(self.Interval))
                newdata += line

        with open(path,mode='w',encoding='UTF-8') as f:
            f.write(newdata)

    def BEnv(self,Path,Index,Num):
        newdata = ''
        with open(Path,mode='r',encoding='UTF-8') as fline:
            for line in fline:
                if  line.find('SET /a START_INDEX=')!= -1:
                    line = line.replace(line,'SET /a START_INDEX={} \n'.format(Index))
                if  line.find('SET /A VehicleNum=')!= -1:
                    line = line.replace(line,'SET /A VehicleNum={} \n'.format(Num))
                if  line.find('tasklist|find /i "QGroundControl.exe" || start %PSP_PATH%\QGroundControl\QGroundControl.exe')!= -1:
                    line = line.replace(line,'REM tasklist|find /i "QGroundControl.exe" || start %PSP_PATH%\QGroundControl\QGroundControl.exe \n')
                if  line.find('tasklist|find /i "RflySim3D.exe" || start %PSP_PATH%\RflySim3D\RflySim3D.exe')!= -1:
                    line = line.replace(line,'REM tasklist|find /i "RflySim3D.exe" || start %PSP_PATH%\RflySim3D\RflySim3D.exe \n')
                if  line.find('tasklist|find /i "CopterSim.exe" && taskkill /im "CopterSim.exe"')!= -1:
                    line = line.replace(line,'REM tasklist|find /i "CopterSim.exe" && taskkill /im "CopterSim.exe" \n')
                if  line.find('tasklist|find /i "CopterSim.exe" && taskkill /im "CopterSim.exe"')!= -1:
                    line = line.replace(line,'REM tasklist|find /i "CopterSim.exe" && taskkill /im "CopterSim.exe" \n')
                if  line.find('tasklist|find /i "QGroundControl.exe" && taskkill /f /im "QGroundControl.exe"')!= -1:
                    line = line.replace(line,'REM tasklist|find /i "QGroundControl.exe" && taskkill /f /im "QGroundControl.exe" \n')
                if  line.find('tasklist|find /i "RflySim3D.exe" && taskkill /f /im "RflySim3D.exe"')!= -1:
                    line = line.replace(line,'REM tasklist|find /i "RflySim3D.exe" && taskkill /f /im "RflySim3D.exe" \n')
                newdata += line
        
        with open(Path,mode='w',encoding='UTF-8') as f:
            f.write(newdata)

    def GetPath(self,Frame):
        for i in range(len(AutoREG.REG.ALL_MAV_FRAME)):
            if self.TestBatPath[i][0] == Frame:
                path = os.path.join(sys.path[0],'..','Model',self.TestBatPath[i][1],self.TestBatPath[i][2])
                return path

class ThreadManage:
    def __init__(self):
        pass
    
    def SimProcessStartManger(self):
        self.SimProcessStartContainer = sorted(MAVREG.SimProcessStartContainer,key=(lambda x:x[2][0])) 
        self.MultiSimProcessStartContainer = self.GetOpenBat(self.SimProcessStartContainer)
        if len(AutoREG.REG.FRAME_TYPE) >= 2: 
            while True:
                MAVREG.Lock.acquire() 
                if len(self.MultiSimProcessStartContainer)>=1 and MAVREG.SIMSTARTFLAG == False:
                    MAVREG.SIMSTARTFLAG = True
                    MAVREG.Lock.release()
                    for i in range(len(self.MultiSimProcessStartContainer)):
                        self.MultiSimProcessStartContainer[i][1].start()
                        time.sleep(1)
                    AutoMavDB.ClearInd()
                    break
                else:
                    MAVREG.Lock.release()
                    break

            if MAVREG.SIMSTARTFLAG == True:
                sleep_time = len(self.MultiSimProcessStartContainer) * 1.5
                time.sleep(sleep_time) 
                ISBREAK = 0
                while True:
                    for i in range(len(self.MultiSimProcessStartContainer)):
                        if not self.MultiSimProcessStartContainer[i][1].is_alive():
                            ISBREAK = 1
                        else:
                            ISBREAK = 0
                    if ISBREAK == 1:
                      
                        break
                    else:
                        pass
        else:
            while True:
                MAVREG.Lock.acquire() 
                if len(MAVREG.SimProcessStartContainer)>=1 and MAVREG.SIMSTARTFLAG == False:
                    MAVREG.SIMSTARTFLAG = True
                    MAVREG.Lock.release()
                    self.SimProcessStartContainer[0][1].start() 
                    AutoMavDB.ClearInd() 
                    break
                else:
                    MAVREG.Lock.release()
                    break

            if MAVREG.SIMSTARTFLAG == True:
                time.sleep(0.5)
                # if len(MAVREG.SimProcessStartContainer) > 1:
                #     for i in range(1,len(MAVREG.SimProcessStartContainer)):
                #         print(f'mav{i} Starting hardware in the loop simulation software')
                while True:
                    if not self.SimProcessStartContainer[0][1].is_alive():
                        break
                    else:
                        pass
    
    def SimProcessEndManger(self):
        self.SimProcessEndContainer = sorted(MAVREG.SimProcessEndContainer,key=(lambda x:x[2][0])) 
        while True: 
            MAVREG.Lock.acquire() 
            if len(self.SimProcessEndContainer)>=1 and MAVREG.SIMENDFLAG == False and len(MAVREG.SimProcessOverContainer) == len(MAVREG.MavContainer):
                MAVREG.SIMENDFLAG = True
                MAVREG.Lock.release()
                self.SimProcessEndContainer[0][1].start()
                break
            else:
                MAVREG.Lock.release()
                break

        if MAVREG.SIMENDFLAG == True:
            time.sleep(0.5)
            while True:
                if not self.SimProcessEndContainer[0][1].is_alive():
                    break
                else:
                    pass

    def AutoMavLoopStartManger(self):
        self.AutoMavThreadsContainer = sorted(MAVREG.AutoMavThreadsContainer,key=(lambda x:x[2][0])) 
        '''
        waitTime = [(val+1)/len(self.AutoMavThreadsContainer) for val in range(len(self.AutoMavThreadsContainer))]
        waitTime.sort(reverse=True)
        '''
        MAVREG.Lock.acquire()
        if MAVREG.MAVSTARTFLAG == False:
            MAVREG.MAVSTARTFLAG = True
            MAVREG.Lock.release()
            for i in range(len(self.AutoMavThreadsContainer)):
                self.AutoMavThreadsContainer[i][1].start()
        else:
            MAVREG.Lock.release()
            time.sleep(0.01) 

    def DataRecordManger(self):
        self.DataRecordThreads = sorted(MAVREG.DataRecordThreadsContainer,key=(lambda x:x[2][0])) 
        WaitFlag = False

        if len(MAVREG.MavContainer) >= 2:
            MAVREG.Lock.acquire()
            if MAVREG.RECORDSTART == False:
                for i in range(len(self.DataRecordThreads)):
                    self.DataRecordThreads[i][1].start()
                for i in range(len(self.DataRecordThreads)):
                    self.DataRecordThreads[i][1].join() 
                MAVREG.RECORDSTART = True
                MAVREG.Lock.release()
            else:
                MAVREG.Lock.release()
                while True:
                    if MAVREG.RECORDSTART == True:
                        self.ClearThreadReg()
                        break
                    else:
                        pass
        else:
            if MAVREG.RECORDSTART == False:
                for i in range(len(self.DataRecordThreads)):
                    self.DataRecordThreads[i][1].start()
                for i in range(len(self.DataRecordThreads)):
                    self.DataRecordThreads[i][1].join()
                self.ClearThreadReg()

    def ClearThreadReg(self):
        MAVREG.SimProcessStartContainer = []
        MAVREG.SimProcessEndContainer = []
        MAVREG.DataRecordThreadsContainer = []
        MAVREG.RECORDSTART = False
        MAVREG.SimProcessOverContainer = []
        MAVREG.SIMSTARTFLAG = False
        MAVREG.SIMENDFLAG = False

            
    def GetOpenBat(self,SimProcessStartContainer):
        MultiSimProcessStartContainer = []
        QuadFlag = 0
        FixedFlag = 0
        USVFlag = 0
        for i in range(len(SimProcessStartContainer)):
            if SimProcessStartContainer[i][2][0] == 1 and QuadFlag == 0:
                MultiSimProcessStartContainer.append(SimProcessStartContainer[i])
                QuadFlag = 1
            if SimProcessStartContainer[i][2][0] == 2 and FixedFlag == 0:
                MultiSimProcessStartContainer.append(SimProcessStartContainer[i])
                FixedFlag = 1
            if SimProcessStartContainer[i][2][0] == 3 and USVFlag == 0:
                MultiSimProcessStartContainer.append(SimProcessStartContainer[i])
                USVFlag = 1
        return MultiSimProcessStartContainer

class MAVREG():
    ThreadOBJ = 0
    MavContainer = [] 
    AutoMavThreadsContainer = [] 
    Lock = threading.Lock() 
    SimProcessStartContainer = []
    SimProcessEndContainer = []
    SimProcessOverContainer = []
    DataRecordThreadsContainer = []
    MAVSTARTFLAG = False
    SIMSTARTFLAG = False
    SIMENDFLAG = False
    RECORDSTART = False
    CLEARFLAG = False

class AutoMavCtrler():
    def __init__(self,mav,TestBatPath):
        self.mav = mav
        self.port = self.mav.port
        self.MAVID = int((self.port-20100)/2+1)
        self.TestBatPath = TestBatPath
        self.InitMavEnvCount = 0
        self.TESTOVER = False
        MAVREG.MavContainer.append(self.MAVID)
        self.InitModelConf()
    
    def InitModelConf(self):
        '''
        Custom model frame and configuration path

        Frame:
        1:                 Quadrotor
        2:                 Fixedwing
        3:                 USV

        Config Path:
        [1,'Quadrotor','QuadModelSITL.bat']
        [2,'Fixedwing','FixedwingModelSITL.bat']
        [3,'USV','USVModelSITL.bat']

        '''
        # Create Mav database objects, and synchronize the json file test data to the corresponding model's test case table
        self.MAVDBOBJ = AutoMavDB.mavdb(self.TestBatPath)
        self.MAVCASELISTID = self.GetMavCase() 
        self.MAVCASEIND = 0

    def InitMavEnv(self):
        MAVREG.CLEARFLAG == False
        self.InitMavEnvCount += 1

        # Start hardware in the loop simulation
        if self.InitMavEnvCount >= 2:
            self.mav = PX4MavCtrl.PX4MavCtrler(self.port)
        self.SimProcessLoopStart()
        AutoMavCtrler.InitMavComm(self)

        # Initialize control sequence class object
        self.CFID = AutoMavCmd.CmdCtrl(self.mav,self.TestBatPath[0])
        self.CID1OBJ = self.CFID.CID1
        self.CID2OBJ = self.CFID.CID2

        # Initialize flight truth data
        self.WCSVel = np.array([0,0,0]) 
        self.WCSAng = np.array([0,0,0]) 
        self.WCSAcc = np.array([0,0,0]) 
        self.WCSEular = np.array([0,0,0]) 
        self.WCSPos = np.array([0,0,0]) 
        self.MotorRPM = np.array([0,0,0,0,0,0,0,0]) 
        self.FallVEL = 0 
        self.FallEnergy = 0 
        self.m = 1.515 

        # Initialize Flight index variable
        self.FLYIND = 0 
        self.RECORDIND = 0 
        self.RECORDFLAG = False
        self.RECORDTIME = 0
        self.EXITUND = 0 
        self.EXITFLAG = False
        self.EXITTIME = 0
        self.INJECTIND = 0 
        self.INJECTFLAG = False
        self.INJECTTIME = 0
        self.FALLIND = 0 
        self.FALLFLAG = False
        self.FALLTIME = 0
        self.LANDIND = 0 
        self.LANDFLAG = False
        self.LANDTIME = 0

        # Initialize database parameters
        self.FaultTestType = np.array([])
        self.FaultParameters = np.array([])

        # Initialize Parameters of safety assessment results
        self.ProfustRisk = 0
        self.ProfustLevel = 0
        self.AssessmentCount = 0
        self.IndexName = 0
        self.ProfustData = 0

        # Fault test status parameters
        self.ARMEDERROR = False

        # Start time and end time (unlock after startup to prevent the ground station from not starting timing)
        self.startTime = time.time()
        self.endTime = time.time()
        self.lastTime = time.time()
        self.mav.SendMavArm(1)
        self.mav.sendUE4Cmd(b'RflyShowTextTime "Sim Start" 10')
        print(f'mav{self.MAVID} Sim start')

        # Initialize Control instruction sequence index
        self.MAVCMDIND = 0
        self.CASEID = self.MAVCASELISTID[self.MAVCASEIND]
        self.MAVCMD = self.MAVDBOBJ.GET_MAVCMD(self.CASEID)
        self.MAVCMDNUM = len(self.MAVCMD)
        print(f'Start mav{self.MAVID} caseID {self.CASEID}')
        print(f'mav{self.MAVID} cmd',self.MAVCMD)
    
    def SimProcessLoopStart(self):
        self.tSimStart = threading.Thread(target=self.StartSimProcess, args=())
        Info = [self.MAVID,self.tSimStart,self.TestBatPath]
        MAVREG.SimProcessStartContainer.append(Info)
        while True:
            if len(MAVREG.SimProcessStartContainer) == len(MAVREG.MavContainer):
                break
        ThreadManage().SimProcessStartManger()
    
    def SimProcessLoopEnd(self):
        self.tSimEnd = threading.Thread(target=self.EndSimProcess, args=())
        Info = [self.MAVID,self.tSimEnd,self.TestBatPath]
        MAVREG.SimProcessEndContainer.append(Info)
        while True:
            if len(MAVREG.SimProcessEndContainer) == len(MAVREG.MavContainer):
                break
        ThreadManage().SimProcessEndManger()

    def StartSimProcess(self):
        pathStr = os.path.join(sys.path[0],'..','Model',self.TestBatPath[1],self.TestBatPath[2])
        self.child = subprocess.Popen(pathStr,shell=True,stdout=subprocess.PIPE)
        MAVREG.ThreadOBJ = self.child
        print(f'mav{self.MAVID} Starting hardware in the loop simulation software')
        time.sleep(25)

    def EndSimProcess(self):
        # Exit the simulation software
        print(f'mav{self.MAVID} Exit hardware in the loop simulation software')
        os.system('tasklist|find /i "CopterSim.exe" && taskkill /im "CopterSim.exe"')
        os.system('tasklist|find /i "QGroundControl.exe" && taskkill /f /im "QGroundControl.exe"')
        os.system('tasklist|find /i "RflySim3D.exe" && taskkill /f /im "RflySim3D.exe"')
        MAVREG.ThreadOBJ.terminate()
        MAVREG.ThreadOBJ.kill()
        print('All closed')
        time.sleep(5)

    def AutoMavLoopStart(self):
        self.tS = threading.Thread(target=self.AutoCreatMavThread,args=())
        self.tS.start()

    def AutoCreatMavThread(self):
        self.tAutoRun = threading.Thread(target=self.AutoMavRun, args=())
        TMavInfo = [self.MAVID,self.tAutoRun,self.TestBatPath]
        MAVREG.AutoMavThreadsContainer.append(TMavInfo) 
        while True:
            if len(MAVREG.AutoMavThreadsContainer) == AutoREG.REG.MAV_ALL_NUM :
                break
        ThreadManage().AutoMavLoopStartManger()
    
    def DataRecordLoop(self):
        self.tData = threading.Thread(target=self.DataRecord,args=())
        mavDataThreadInfo = [self.MAVID,self.tData,self.TestBatPath]
        MAVREG.DataRecordThreadsContainer.append(mavDataThreadInfo)
        while True:
            if len(MAVREG.DataRecordThreadsContainer) == len(MAVREG.MavContainer):
                break
            else:
                pass
        ThreadManage().DataRecordManger()

    def DataRecord(self):
        TestNum = self.InitMavEnvCount
        mavInfo = [len(MAVREG.MavContainer),TestNum]
        Data = [self.WCSVel,self.WCSAng,self.WCSPos]
        print(f"mav{int((self.port-20100)/2+1)}:Start data processing")
        DataObj = AutoMavDB.DataAPI(self.CASEID,self.TestBatPath,mavInfo,Data)
        # DataObj.ProfustSaftyScoreRecord(self.AssessmentCount,self.IndexName,self.ProfustData)
        print(f"mav{int((self.port-20100)/2+1)}:Data processing Finished")
        
    def ShowWinLoop(self):
        self.tShow = threading.Thread(target=self.ShowWin, args=())
        self.tShow.start()
        self.tShow.join()

    def ShowWin(self):
        MAVREG.Lock.acquire()
        ProfustLevel=str(self.ProfustLevel)
        ID = self.CASEID
        window = tkinter.Tk()
        window.attributes("-topmost",1)
        window.withdraw()  
        ProfustSafty="%.1f" % self.ProfustRisk
        Healthscore="%.1f" % 0.021
        myText="Test task:"+"Wind disturbance\n"+'Test case ID:'+str(ID)+"\nProfust Safty Level:"+ProfustLevel+"\nProfustSafty:"+ProfustSafty+"\nFault diagnosis is triggered correctly:"+"Yes"
        myText=myText+'\n\nClick "OK" to archive the data and start the next test'
        window.after(3000, window.destroy)
        result = showinfo('prompt',myText)
        window.mainloop() 
        MAVREG.Lock.release()

    def InitMavComm(self):
        self.mav.InitMavLoop()
        time.sleep(0.5)
        
        self.mav.InitTrueDataLoop()
        time.sleep(0.5)
        
        if self.TestBatPath[0] == 2:
            self.mav.enFixedWRWTO()
        else:
            self.mav.initOffboard()
            time.sleep(0.5)
    
    def EndMavComm(self):
        self.mav.EndTrueDataLoop()
        time.sleep(0.5)
        self.mav.endMavLoop() 
        time.sleep(0.5)

        time.sleep(5)

    def GetMavCase(self):
        CaseList = self.MAVDBOBJ.GET_CASEID() 
        print(f'mav{self.MAVID} AllCaseList',CaseList)
        return CaseList

    def AutoMavRun(self):
        while True: 
            if self.MAVCASEIND >= len(self.MAVCASELISTID):
                AutoREG.ALLSTATE.ALL_DOWN = True
                print('All case test finish!')
                break
            
            AutoMavCtrler.InitMavEnv(self)

            while True:
                # 250HZ receiving data
                self.lastTime = self.lastTime + (1.0/250)
                sleepTime = self.lastTime - time.time()
                if sleepTime > 0:
                    time.sleep(sleepTime)
                else:
                    self.lastTime = time.time()

                if self.CID2OBJ.INJECTFLAG == True and (self.CID2OBJ.FAULTID == 123549 or self.CID2OBJ.FAULTID == 125340 or self.CID2OBJ.FAULTID == 125341):
                    self.POD.Podfault(self.CID2OBJ.FAULTID) # Trigger pod fault
                else:
                    if self.MAVDBOBJ.VISIONFLAG == True:
                        self.POD.Podfault(0)

                # Start recording data after the unlocking command is issued
                if self.CID2OBJ.ARMFLAG == True:
                    VEL = np.array(self.mav.trueVelNED)
                    ANGRATE = np.array(self.mav.trueAngRate) 
                    ACC = np.array(self.mav.trueAccB)
                    MOTORRPM = np.array(self.mav.trueMotorRPMS)
                    EULAR = np.array(self.mav.trueAngEular)
                    POS = np.array(self.mav.truePosNED)
                    self.FLYIND = self.FLYIND + 1
                    self.WCSVel = np.row_stack((self.WCSVel,VEL)) 
                    self.WCSAng = np.row_stack((self.WCSAng,ANGRATE)) 
                    self.WCSAcc = np.row_stack((self.WCSAcc,ACC)) 
                    self.WCSEular = np.row_stack((self.WCSEular,EULAR)) 
                    self.WCSPos =  np.row_stack((self.WCSPos,POS)) 
                    self.MotorRPM =  np.row_stack((self.MotorRPM,MOTORRPM)) 

                # Processing instruction sequence
                AutoMavCtrler.TRIGGERMAVCMD(self,self.MAVCMD[self.MAVCMDIND])
                # If one instruction sequence is completed, the next instruction is processed
                if re.findall(r'-?\d+\.?[0-9]*',self.MAVCMD[self.MAVCMDIND])[0] == '1' and self.CID1OBJ.isDone == 1 or re.findall(r'-?\d+\.?[0-9]*',self.MAVCMD[self.MAVCMDIND])[0] == '2' and self.CID2OBJ.isDone == 1:
                    self.MAVCMDIND = self.MAVCMDIND + 1

                if self.CID2OBJ.RECORDFLAG == True and self.RECORDFLAG == False:
                    self.RECORDIND = self.FLYIND 
                    self.RECORDTIME = round(time.time() - self.startTime)
                    self.RECORDFLAG = True
                
                if self.CID2OBJ.INJECTFLAG == True and self.INJECTFLAG == False:
                    self.INJECTIND = self.FLYIND 
                    self.INJECTTIME = round(time.time() - self.startTime)
                    self.INJECTFLAG = True
                
                if self.CID2OBJ.LANDFLAG == True and self.LANDFLAG == False:
                    self.LANDIND = self.FLYIND 
                    self.LANDTIME = round(time.time() - self.startTime)
                    self.LANDFLAG = True

                if self.MAVCMDIND >= self.MAVCMDNUM and self.EXITFLAG == False:
                    self.EXITUND = self.FLYIND 
                    self.EXITTIME = round(time.time() - self.startTime)
                    self.EXITFLAG = True
                    print(f'mav{self.MAVID}: CaseID {self.CASEID} test completed')
                    self.mav.sendUE4Cmd(b'RflyShowTextTime "CaseID %d test completed!" 10'%(self.CASEID))
                    self.endTime = time.time()
                    break

                # Judgment of crash, if the landing speed is greater than 3.5, the aircraft is considered to have crashed
                if  abs(np.array(self.mav.truePosNED[2])) < 1.2 and self.FALLFLAG == False  and abs(np.array(self.mav.trueVelNED[2])) > 3.5: 
                    self.FALLTIME =  round(time.time() - self.startTime)
                    self.FALLIND = self.FLYIND
                    self.FALLFLAG = True
                    self.FallVEL = round(np.max(self.WCSVel),2) 
                    self.FallEnergy = round(0.5*self.m*(self.FallVEL**2),2) 
                    self.EXITUND = self.FLYIND 
                    self.EXITTIME = round(time.time() - self.startTime)
                    self.EXITFLAG = True
                    self.endTime = time.time()
                    print("{}s,Crash! Exit the test".format(self.EXITTIME))
                    self.mav.sendUE4Cmd(b'RflyShowTextTime "Crash!" 10')
                    break

                # If the unlocking is abnormal, exit the test and start a new test again
                if self.mav.ARMEDERROR == True:
                    self.ARMEDERROR = True
                    break
            
            if self.ARMEDERROR == False:
                if self.CID2OBJ.RECORDFLAG == True:
                    '''
                    SaftyAssessment(Index,EvalName,EvalData,EvalDim,EvalParam,CtrlCmd)
                    Index is the list of index entries:start_index、end_index、fall_index
                    EvalName is the name of the data
                    EvalData is a data  item list:Ang、Vel、Pos、...
                    EvalDim is a dimension item list: Represents the specific dimension of Data
                    EvalParam is a parameter item list: Including: data frequency (number of data in 1s), ground kinetic energy, index weight
                    CtrlCmd is a Control command item list: Target command representing position and speed
                    '''
                    DataFreq = round(len(self.WCSVel)/(self.endTime - self.startTime))
                    FallEnergy = self.FallEnergy
                    VelCmd = [0,0,0]
                    PosCmd = [0,0,0]

                    StartIndex = self.INJECTIND
                    EndIndex = self.EXITUND
                    FallIndex = self.FALLIND
                    Index = [StartIndex,EndIndex,FallIndex]
                    EvalName = ['Vel']
                    EvalData = [self.WCSVel]
                    EvalDim = [1]
                    EvalWeight = [1]

                    EvalParam = [DataFreq,FallEnergy,EvalWeight]
                    CtrlCmd = [PosCmd,VelCmd]
                    MavProfustSafty.SaftyAssessment(Index,EvalName,EvalData,EvalDim,EvalParam,CtrlCmd)
                    self.AssessmentCount += 1
                    self.ProfustRisk = MavProfustSafty.ProfustSaftyScoreUAV
                    self.ProfustLevel = MavProfustSafty.ProfustSaftyLevelUAV
                    
                    self.IndexName = EvalName
                    self.ProfustData = MavProfustSafty.RealTimeProfustSaftyScore

                    mystr = '"Profust Safty Level:{}" 10'.format(self.ProfustLevel)
                    strcmd = 'RflyShowTextTime '+ mystr 
                    self.mav.sendUE4Cmd(strcmd.encode())
                    self.mav.sendUE4Cmd(b'RflyShowTextTime "Profust Safty Score:%.3f" 10'%(self.ProfustRisk))

                    print(f"mav{self.MAVID} ProfustRisk:",self.ProfustRisk)
                    print(f"mav{self.MAVID} ProfustLevel:",self.ProfustLevel)

                    # Data Base Pro
                    self.MAVDBOBJ.RESETR_DB(self.CASEID)
                    ID = self.CASEID
                    self.FaultTestType = str(self.CID2OBJ.silInt) 
                    self.FaultParameters = str(self.CID2OBJ.silFloats) 
                    result_data = [ID,self.FaultTestType,self.FaultParameters,'Finished','Fault risk:{} Fault level:{}'.format(self.ProfustRisk,self.ProfustLevel)]
                    self.MAVDBOBJ.RESULT_DBPro(result_data)
                    self.MAVDBOBJ.TEST_STATEPro(self.CASEID)

                    # JSON Pro
                    MAVREG.Lock.acquire() 
                    self.MAVDBOBJ.MAV_JSONPro(self.CASEID)
                    MAVREG.Lock.release()

                    self.ShowWinLoop()

                print(f'mav{self.MAVID} Sim end')
                self.mav.sendUE4Cmd(b'RflyShowTextTime "Sim end" 10')

                if self.MAVDBOBJ.VISIONFLAG == True:
                    cv2.destroyAllWindows() 
                MAVREG.SimProcessOverContainer.append(self.EXITFLAG)
                AutoMavCtrler.EndMavComm(self)
                self.SimProcessLoopEnd()
                self.MAVCASEIND += 1
            else:
                print(f'mav{self.MAVID} Armed exception,retest!')
                time.sleep(5)
                print(f'mav{self.MAVID} Sim end')
                AutoMavCtrler.EndMavComm(self)
                self.SimProcessLoopEnd()
            
            time.sleep(5)

            # Data Record
            self.TESTOVER = True
            AutoMavCtrler.DataRecordLoop(self)

    def TRIGGERMAVCMD(self,ctrlseq):
        cmdseq = ctrlseq # '2,3,0,0,-20'
        cmdseq = re.findall(r'-?\d+\.?[0-9]*',cmdseq) # ['2', '3', '0', '0', '-20']
        cmdCID = cmdseq[0]
        if  cmdCID in self.CFID.CID:
            FID = self.CFID.FIDPro(cmdCID)
            if len(cmdseq) > 2:
                param = cmdseq[2:len(cmdseq)]
                param = [float(val) for val in param]
                FID[cmdseq[1]](param)

            else:
                FID[cmdseq[1]]()
        else:
            print(f'mav{self.MAVID} Command input error, please re-enter')
    

        
