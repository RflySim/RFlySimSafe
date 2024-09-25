import threading
import numpy as np
import re
import sqlite3
import os
import time
import sys
import csv
import json
from fnmatch import fnmatch
import lxml.html as lh
import matplotlib.pyplot as plt
import shutil
import pandas as pd

try:
    import include.AutoREG as AutoREG
    import include.QGCCtrlAPI as QGCCtrlAPI
except ImportError:
    import AutoREG 
    import QGCCtrlAPI



def dict_factory(cursor, row):  
    d = {}  
    for idx, col in enumerate(cursor.description):  
        d[col[0]] = row[idx]  
    return d 

class MAVDB:
    def __init__(self, conf): # conf_eg: ['Quadcopter', 'SITL', 1]
        self.cursor = None
        self.mydb = None
        self.conf = conf
        self.is_tested = 0
        self.count = 0
        model_path = os.path.join(os.getcwd(),'src','model',AutoREG.MAV_FRAME_DICT[AutoREG.RFLYSIM_FRAME[conf[0]]][0])
        '''
        conf[0]                      : Quadcopter
        RFLYSIM_FRAME[Quadcopter]    : 1
        AutoREG.MAV_FRAME_DICT[1][0] : Quadcopter
        '''
        dbf = 'db.json'
        dbp = [filename for filename in os.listdir(model_path) if dbf in filename]
        self.jsonpath = os.path.join(model_path, dbp[0])
        MAVDB.JSON_TO_SQL(self)
        self.VISIONFLAG = MAVDB.VISION(self)

    def VISION(self):
        jsonpath = self.jsonpath
        with open(jsonpath, "r",encoding='utf-8') as f:
            json_data = json.load(f)
        isVision = json_data.get('VISION') 
        if isVision == 'On':
            return True
        else:
            return False
    
    def JSON_TO_SQL(self):
        '''
        Synchronize test cases in json files and database
        '''
        path = self.jsonpath
        
        with open(path, "r",encoding='utf-8') as f:
            json_data = json.load(f)

        json_case = json_data.get('FAULT_CASE')
        '''
        db_case_eg:
        [
         {'CaseID': 1, 'Subsystem': 'Power', 'Component': 'Motor', 'FaultID': '123450', 'FaultType': 'Motor Fault', 'FaultMode': 'Decreased efficiency of motor execution', 'FaultParams': '0', 'ControlSequence': '2,1;1,1,5;2,3,150,0,-30;1,1,10;2,4,1000,0,-30;1,1,10;2,6,123450,0;1,1,10', 'TestStatus': 'Finished'}
        ]
        
        json_case_eg:
        [
         {'CaseID': 1, 'Subsystem': 'Power', 'Component': 'Motor', 'FaultID': '123450', 'FaultType': 'Motor Fault', 'FaultMode': 'Decreased efficiency of motor execution', 'FaultParams': '0', 'ControlSequence': '2,1;1,1,5;2,3,150,0,-30;1,1,10;2,4,1000,0,-30;1,1,10;2,6,123450,0;1,1,10', 'TestStatus': 'Finished'}, 
         {'CaseID': 2, 'Subsystem': 'Sensor', 'Component': 'Magnetometer', 'FaultID': '123544', 'FaultType': 'Magnetometer noise', 'FaultMode': 'Magnetometer noise interference', 'FaultParams': '0', 'ControlSequence': '2,1;1,1,5;2,3,150,0,-30;1,1,10;2,4,1000,0,-30;1,1,10;2,6,123544,0;1,1,10', 'TestStatus': 'Finished'}, 
         {'CaseID': 3, 'Subsystem': 'Environment', 'Component': 'ConstWind', 'FaultID': '123458', 'FaultType': 'ConstWind interference', 'FaultMode': 'Flight encounters constant winds', 'FaultParams': '15,15,14', 'ControlSequence': '2,1;1,1,5;2,3,150,0,-30;1,1,10;2,4,1000,0,-30;1,1,10;2,6,123458,123458,15,15,14;1,1,10', 'TestStatus': 'Finished'}, 
         {'CaseID': 4, 'Subsystem': 'Sensor', 'Component': 'Accelerometer', 'FaultID': '123542', 'FaultType': 'Accelerometer noise', 'FaultMode': 'Accelerometer noise interference', 'FaultParams': '1', 'ControlSequence': '2,1;1,1,5;2,3,150,0,-30;1,1,10;2,4,1000,0,-30;1,1,10;2,6,123542,1;1,1,10', 'TestStatus': 'Finished'}
        ]
        '''
        '''
        1. Open JSON file and database connection
        '''
        MAVDB.GET_CURSOR(self)
        for item in json_case:
            '''
            2. Traverse JSON data
            '''
            self.cursor.execute("SELECT * FROM TEST_CASE WHERE CaseID=?", (item['CaseID'],))
            '''
            3. Check if the ID exists
            '''
            existing_data = self.cursor.fetchone()
            if existing_data:
                '''
                4. If the ID already exists, update the data in the database using JSON data
                '''
                # print('The same caseID exists, replace')
                self.cursor.execute("UPDATE TEST_CASE SET Subsystem=?, Component=?, FaultID=?, FaultType=?, FaultMode=?, FaultParams=?, ControlSequence=?, TestStatus=? WHERE CaseID=?",
                            (item['Subsystem'], item['Component'], item['FaultID'], item['FaultType'], item['FaultMode'], item['FaultParams'], item['ControlSequence'], item['TestStatus'], item['CaseID']))
            else:
                '''
                5. If the ID does not exist, insert the JSON data into the database
                '''
                # print('The same caseID does not exist, add')
                self.cursor.execute("INSERT INTO TEST_CASE (CaseID, Subsystem, Component, FaultID, FaultType, FaultMode, FaultParams, ControlSequence, TestStatus) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
                            (item['CaseID'], item['Subsystem'], item['Component'], item['FaultID'], item['FaultType'], item['FaultMode'], item['FaultParams'], item['ControlSequence'], item['TestStatus']))

            self.mydb.commit()

    def GET_CURSOR(self):
        frame = AutoREG.MAV_FRAME_DICT[AutoREG.RFLYSIM_FRAME[self.conf[0]]][0] + '.db'
        imPath = os.path.join(os.getcwd(),'case',frame)
        self.mydb = sqlite3.connect(imPath)
        self.mydb.row_factory = dict_factory
        self.cursor=self.mydb.cursor()
    
    def GET_FAULT_CASE(self): 
        '''
        Obtain fault test cases
        '''
        MAVDB.GET_CURSOR(self)
        sql='''
            select   *
            from     TEST_CASE
            '''
        self.cursor.execute(sql)
        result=self.cursor.fetchall()
        return result
    
    def GET_CASEINFO(self, case_id):
        path = self.jsonpath

        with open(path, "r",encoding='utf-8') as f:
            json_data = json.load(f)
        
        caseInfo = json_data['FAULT_CASE']

        for fault_case in caseInfo:
            if fault_case['CaseID'] == case_id:
                return fault_case

        return None
    
    def GET_CASEINFO_P(self, case_id, path):

        with open(path, "r",encoding='utf-8') as f:
            json_data = json.load(f)
        
        caseInfo = json_data['FAULT_CASE']

        for fault_case in caseInfo:
            if fault_case['CaseID'] == case_id:
                return fault_case

        return None

    
    def GET_CASEID(self): 
        '''
        Obtain the list of fault test case IDs
        '''
        result = MAVDB.GET_FAULT_CASE(self)
        '''
        result_eg:
        [
         {'CaseID': 1, 'Subsystem': 'Power', 'Component': 'Motor', 'FaultID': '123450', 'FaultType': 'Motor Fault', 'FaultMode': 'Decreased efficiency of motor execution', 'FaultParams': '0', 'ControlSequence': '2,1;1,1,5;2,3,150,0,-30;1,1,10;2,4,1000,0,-30;1,1,10;2,6,123450,0;1,1,10', 'TestStatus': 'Finished'}, 
         {'CaseID': 2, 'Subsystem': 'Sensor', 'Component': 'Magnetometer', 'FaultID': '123544', 'FaultType': 'Magnetometer noise', 'FaultMode': 'Magnetometer noise interference', 'FaultParams': '0', 'ControlSequence': '2,1;1,1,5;2,3,150,0,-30;1,1,10;2,4,1000,0,-30;1,1,10;2,6,123544,0;1,1,10', 'TestStatus': 'Finished'}, 
         {'CaseID': 3, 'Subsystem': 'Environment', 'Component': 'ConstWind', 'FaultID': '123458', 'FaultType': 'ConstWind interference', 'FaultMode': 'Flight encounters constant winds', 'FaultParams': '15,15,14', 'ControlSequence': '2,1;1,1,5;2,3,150,0,-30;1,1,10;2,4,1000,0,-30;1,1,10;2,6,123458,123458,15,15,14;1,1,10', 'TestStatus': 'Finished'}, 
         {'CaseID': 4, 'Subsystem': 'Sensor', 'Component': 'Accelerometer', 'FaultID': '123542', 'FaultType': 'Accelerometer noise', 'FaultMode': 'Accelerometer noise interference', 'FaultParams': '1', 'ControlSequence': '2,1;1,1,5;2,3,150,0,-30;1,1,10;2,4,1000,0,-30;1,1,10;2,6,123542,1;1,1,10', 'TestStatus': 'Finished'}
        ]
        '''
        path = self.jsonpath

        with open(path, "r",encoding='utf-8') as f:
            json_data = json.load(f)

        global caselist
        if json_data.get('TEST_CASE') == 'All':
            caselist = []
            casedata = result
            for data in casedata:
                ID = data['CaseID']
                caselist.append(ID)
            return caselist
        else:
            if self.conf[2] > 1: # Multi Instance
                case = json_data.get('TEST_CASE')
                '''
                case_eg:
                "1,2,5;2,1,1;4,5,6;"
                '''
                
                len_seg = len(re.findall(r';',case))
                if len_seg:
                    mcase = re.findall(r'-?\d+', case.split(";")[0]) # ['1,2', '2,6']
                    '''['1', '2']'''
                    len_case = len(mcase)
                    if len_case > 1:
                        ''' Multi-machine mode multiple test cases, eg: "1,2;3,5" '''
                        segments = case.split(";")
                        '''
                        ['1,2,5', '2,1,1', '4,5,6', '']
                        '''
                        segment_lists = [list(filter(None, segment.split(','))) for segment in segments]
                        '''
                        [['1', '2', '5'], ['2', '1', '1'], ['4', '5', '6'], []]
                        '''
                        max_length = max(len(segment) for segment in segment_lists)

                        result = []
                        for i in range(max_length):
                            result.append([int(segment[i]) for segment in segment_lists if i < len(segment)])
                        '''
                        [[1, 2, 4], [2, 1, 5], [5, 1, 6]]
                        '''
                        return result
                    else:
                        ''' Multi-machine mode single test case, eg:"1;2" ''' 
                        segments = case.split(";")
                        ''' ['1', '2'] '''
                        result = []
                        
                        for segment in segments:
                            elements = list(map(int, segment.split(',')))
                            result.append(elements)

                        return result
                else:
                    warn = 'Sorry, your configuration is wrong, please check as follows:\n1) Whether your UAV num is greater than 1; \n2) Did you forget to add a semicolon(;) in the "TEST_CASE"  of your json file to switch to multi-machine mode?\n'
                    print(warn)
                    sys.exit(0)

            else:
                len_seg = len(re.findall(r';',json_data.get('TEST_CASE')))
                if len_seg:
                    warn = 'Sorry, your configuration is wrong, please check as follows:\n1) Whether your UAV num is greater than 1; \n2) Did you forget to add a semicolon(;) in the "TEST_CASE"  of your json file to switch to multi-machine mode?\n'
                    print(warn)
                    sys.exit(0)
                else:
                    result = [int(val) for val in re.split(',',json_data.get('TEST_CASE'))]
                    '''[1, 2, 5]'''
                    return result

    def GET_MAVCMD(self,case_id): 
        '''
        Process Control Sequence
        '''
        MAVDB.GET_CURSOR(self)
        sql = '''
            select * from TEST_CASE
            where CaseID = {}
            '''.format(case_id)
        self.cursor.execute(sql)
        data = self.cursor.fetchall()
        case_sequence = data[0].get('ControlSequence')
        case = re.split(';',case_sequence)
        cmd = np.array([])
        for i in range(len(case)):
            cmd = np.append(cmd,case[i])
        '''
        ['2,1' '1,1,5' '2,3,150,0,-30' '1,1,10' '2,4,1000,0,-30' '1,1,10' '2,6,123450,0' '1,1,10']
        '''  
        return cmd   

    def RESULT_DBPro(self,data): 
        '''
        Process test result library (add test results)
        '''
        MAVDB.GET_CURSOR(self)
        sql = '''
            insert into TEST_RESULT
            (CaseID, FaultID, CaseDescription, FaultMode, ControlSequence, TestResult)
            values(?, ?, ?, ?, ?, ?) 
            '''
        value = (data[0],data[1],data[2],data[3],data[4],data[5])
        self.cursor.execute(sql,value)
        self.mydb.commit()

    def TEST_STATEPro(self,case_id): 
        '''
        Process test case library (change test status)
        '''
        MAVDB.GET_CURSOR(self)
        sql = '''
            update TEST_CASE
            set TestStatus = 'Finished'
            where CaseID = {}
            '''.format(case_id)
        self.cursor.execute(sql)
        self.mydb.commit()

    def IS_TESTEDPro(self,case_id): 
        '''
        Judge whether it is a tested case
        '''
        MAVDB.GET_CURSOR(self)
        sql = '''
            select * from TEST_CASE
            where CaseID = {}
            '''.format(case_id)
    
        self.cursor.execute(sql)
        data = self.cursor.fetchall()

        if data[0].get('TestStatus') == 'Finished':
            self.is_tested = 1
        else:
            self.is_tested = 0
        return self.is_tested

    def RESETR_DB(self,case_id): 
        '''
        Handle the result library of repeated use case test
        '''
        MAVDB.GET_CURSOR(self)
        sql = '''
            DELETE FROM TEST_CASE
            WHERE CaseID = {}
            '''.format(case_id)
    
        self.cursor.execute(sql)

    def MAV_JSONPro(self,case_id): 
        '''
        Change the test status information of json file
        '''
         
        path = self.jsonpath
        with open(path, "r",encoding='utf-8') as f:
            db_data = json.load(f)
        json_case = db_data.get('FAULT_CASE')
        '''
        [
        {'CaseID': 1, 'Subsystem': 'Power', 'Component': 'Motor', 'FaultID': '123450', 'FaultType': 'Motor Fault', 'FaultMode': 'Decreased efficiency of motor execution', 'FaultParams': '0', 'ControlSequence': '2,1;1,1,5;2,3,150,0,-30;1,1,10;2,4,1000,0,-30;1,1,10;2,6,123450,0;1,1,10', 'TestStatus': 'Finished'}
        ]
        '''
        if len(json_case) >= 1:
            path = self.jsonpath

            with open(path, "r",encoding='utf-8') as f:
                db_data = json.load(f)
                db_data['FAULT_CASE'][case_id-1]['TestStatus'] = 'Finished'
                data = db_data
            f.close()

            with open(path, "w",encoding='utf-8') as w:
                json.dump(data,w,indent=4) 
            w.close()
    
    def MAV_JSONPro_P(self, case_id, path): 
        '''
        Change the test status information of json file
        '''

        with open(path, "r",encoding='utf-8') as f:
            db_data = json.load(f)
        json_case = db_data.get('FAULT_CASE')
        '''
        [
        {'CaseID': 1, 'Subsystem': 'Power', 'Component': 'Motor', 'FaultID': '123450', 'FaultType': 'Motor Fault', 'FaultMode': 'Decreased efficiency of motor execution', 'FaultParams': '0', 'ControlSequence': '2,1;1,1,5;2,3,150,0,-30;1,1,10;2,4,1000,0,-30;1,1,10;2,6,123450,0;1,1,10', 'TestStatus': 'Finished'}
        ]
        '''
        if len(json_case) >= 1:
            path = self.jsonpath

            with open(path, "r",encoding='utf-8') as f:
                db_data = json.load(f)
                db_data['FAULT_CASE'][case_id-1]['TestStatus'] = 'Finished'
                data = db_data
            f.close()

            with open(path, "w",encoding='utf-8') as w:
                json.dump(data,w,indent=4) 
            w.close()

class DataREG():
    
    lock = threading.Lock()
    isDeleted = False
    MavNum = 0
    Mode2mavP = []
    Mode2caseP = []
    Mode3frameP = []
    Mode3caseP = []
    Mode4frameP = []
    Mode4caseP = []
    Mode4mavREG = {
        'Quadcopter' : [],
        'Fixedwing'  : [],
        'Vtol'       :[]
    }
    html_file_path = os.path.abspath(os.path.join(sys.path[0],'..','..','data','TestInfo.html'))
    TestResult_html_file_path = os.path.abspath(os.path.join(sys.path[0],'..','..','data','TestResult.html'))
    bk_path = os.path.abspath(os.path.join(sys.path[0],'..','..','conf','material','bk3.png'))
    bk_path = bk_path.replace('\\','//')
    bk_path2 = os.path.abspath(os.path.join(sys.path[0],'..','..','conf','material','bk2.png'))
    bk_path2 = bk_path2.replace('\\','//')
    dataPollNum = 0
    Finally_Path_for_Multi = None


class DATAAPI():
    def __init__(self,CaseID,conf,Data,Info):
        # conf_eg: ['Quadcopter', 'SITL', 1]
        self.Data = Data
        self.conf = conf
        self.CaseID = CaseID
        self.Info = Info

        model_path = os.path.join(sys.path[0],'..','model',AutoREG.MAV_FRAME_DICT[AutoREG.RFLYSIM_FRAME[conf[0]]][0])
        dbf = 'db.json'
        dbp = [filename for filename in os.listdir(model_path) if dbf in filename]
        self.jsonpath = os.path.join(model_path, dbp[0])

        conf = self.conf[1] + '.bat'  
        batp = [filename for filename in os.listdir(model_path) if conf in filename]  
        self.batpath = os.path.join(model_path, batp[0])

        self.isTrueDataRecordOver = 0
        self.MacVechileNum = AutoREG.MAV_DATA_FOLDER_REG[AutoREG.RFLYSIM_FRAME[self.conf[0]]]
    
        self.PlatFormpath = self.PX4Path() # 'C:\\PX4PSP'
        self.DataPath = sys.path[0] + '/..' + '/..' + '/data'
        self.MavFrameDataPath = self.DataPath + f'/{self.conf[0]}'

        self.ModePath = None
        if self.conf[1] == "SITL":
            self.ModePath = self.DataPath + '/SITL'
        elif self.conf[1] == "HITL":
            self.ModePath = self.DataPath + '/HITL'

        
        self.mode1sInsp = self.ModePath + '/single' + '/sInstance' 
        self.mode2mInsp = self.ModePath + '/single' + '/mInstance'
        self.mode3sInsp = self.ModePath + '/multi' + '/sInstance'
        self.mode4mInsp = self.ModePath + '/multi' + '/mInstance'
    
        self.DP()
    
    def DP(self):
        DataREG.lock.acquire()
        self.MKDPath()
        self.DataPro()
        self.TruedataRecord()
        self.RName()
        DataREG.lock.release()
        self.InfoDown()
        self.Test_result_Record()
    
    def MKDPath(self):
        DataREG.MavNum += 1

        if AutoREG.TEST_MODE == 1:
            '''Single Instance. Folder struct named with mode/frame/TestCase_''' 
            fp = self.mode1sInsp + f'/{self.conf[0]}'
            cp = fp + f'/TestCase_{self.CaseID}'

            self.Mode1InsP = cp

            if os.path.exists(cp): 
                shutil.rmtree(cp)
            self.logp = cp + '/log' 
            self.truep = cp + '/true'
            self.MFolder(self.logp, self.truep)

        elif AutoREG.TEST_MODE == 2:
            '''Single frame Multi Instance. Folder struct named with mode/frame/TestCase_*_*/'''
            fp = self.mode2mInsp + f'/{self.conf[0]}'
            self.tempCaseP = fp + '/TestCase'

            mavP = self.tempCaseP + f'/mav{AutoREG.DIND_REG[self.conf[0]]+1}'
            self.mavTestID = AutoREG.DIND_REG[self.conf[0]]+1

            DataREG.Mode2mavP.append(f'/mav{AutoREG.DIND_REG[self.conf[0]]+1}')
            DataREG.Mode2caseP.append(self.CaseID)

            AutoREG.DIND_REG[self.conf[0]] += 1

            self.logp = mavP + '/log'
            self.truep = mavP + '/true'
            self.MFolder(self.logp, self.truep)
 
        elif AutoREG.TEST_MODE == 3:
            '''Multi frame Single Instance. Folder struct named with mode/TestCase_/frame/'''
            self.tempCaseP = self.mode3sInsp + f'/TestCase'

            frameP = self.tempCaseP + f'/{self.conf[0]}'

            AutoREG.DIND_REG[self.conf[0]] += 1

            DataREG.Mode3frameP.append(f'/{self.conf[0]}')
            DataREG.Mode3caseP.append(self.CaseID)

            self.logp = frameP + '/log'
            self.truep = frameP + '/true'
            self.MFolder(self.logp, self.truep)

        elif AutoREG.TEST_MODE == 4:
            '''Multi frame multi Instance. Folder struct named with mode/TestCase_/frame/'''
            self.tempCaseP = self.mode4mInsp + '/TestCase'

            frameP = self.tempCaseP + f'/{self.conf[0]}'
            mavP = frameP + f'/mav{AutoREG.DIND_REG[self.conf[0]]+1}'
            self.mavTestID = AutoREG.DIND_REG[self.conf[0]]+1

            DataREG.Mode4mavREG[self.conf[0]].append(f'/mav{AutoREG.DIND_REG[self.conf[0]]+1}')
            DataREG.Mode4frameP.append(f'/{self.conf[0]}')
            DataREG.Mode4caseP.append(self.CaseID)

            AutoREG.DIND_REG[self.conf[0]] += 1

            self.logp = mavP + '/log'
            self.truep = mavP + '/true'
            self.MFolder(self.logp, self.truep) 
  
    def PX4Path(self):
        PX4Path='C:\\PX4PSP'
        with open(self.batpath, 'r',encoding='UTF-8') as file:
            for line in file:
                if line.find('SET PSP_PATH=')!=-1:
                    PX4Path=line.replace('SET PSP_PATH=','')
                    PX4Path=PX4Path.strip()
                    PX4Path=PX4Path.replace('\\','/')
                    break
        
        return PX4Path

    def MFolder(self, TargetFilder_log, TargetFilder_truedata):
        os.makedirs(TargetFilder_log) 
        os.makedirs(TargetFilder_truedata)

    def DataPro(self):
        qgc = QGCCtrlAPI.QGCCtrlAPI()
        logName = qgc.ReqQgcLog(AutoREG.LOG_TIMEOUT, self.Info[2])
        if logName!='': 
            shutil.copyfile(qgc.QGCPath+'\\'+logName,self.logp +'\\'+logName)
            print('Download log '+logName+f' for mav{self.Info[2]} successfully.')

        '''
        # Old Mode
        # 1、List the directories under the file
        # SITL Mode:
        log_path = self.PlatFormpath + f'/Firmware/build/px4_sitl_default/instance_{DataREG.MavNum}/log'

        PlatForm_log_dirs = os.listdir(log_path) 
        log_data = PlatForm_log_dirs[len(PlatForm_log_dirs)-1]
        self.path = os.path.join(log_path,log_data) 
        dirs = os.listdir(self.path) 

        # 2、Get the latest ulg file
        ulg = dirs[len(dirs)-1]
        ulgPath = os.path.join(self.path,ulg) 

        
        # 3、Copy the ulg file to the log folder
        TargetPath_log = self.logp + '/{}'.format(ulg)
        shutil.copyfile(ulgPath, TargetPath_log) 
        '''
            
    def TruedataRecord(self):
            truedata_v_csv_path = self.truep + '//truedata_vel.csv' 
            with open(truedata_v_csv_path, 'w', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)
                for row in self.Data[0]:
                    writer.writerow(row)

            truedata_ang_csv_path = self.truep + '//truedata_ang.csv' 
            with open(truedata_ang_csv_path, 'w', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)
                for row in self.Data[1]:
                    writer.writerow(row)
            
            truedata_pos_csv_path = self.truep + '//truedata_pos.csv' 
            with open(truedata_pos_csv_path, 'w', newline='', encoding='utf-8') as csvfile:
                writer = csv.writer(csvfile)
                for row in self.Data[2]:
                    writer.writerow(row)
                
    
    def InfoRecord(self):
        infop = os.path.abspath(os.path.join(sys.path[0],'..','..','data','TestInfo.csv'))
        if not os.path.exists(infop):
            header = [
                ['Date','Frame', 'CaseID', 'S/HITL','TestInfo','DataPath','Normal?']
            ]

            df = pd.DataFrame(header)

            df.to_csv(infop, index=False, header=False)

        date = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
        frame = self.conf[0]
        caseID = self.CaseID
        SHITL = self.conf[1]
        testinfo = self.CMDAna(self.Info[0])
        datapath = self.GetInfoP()
        warnInfo = 'Yes'

        if  AutoREG.ARMED_WARN:
            warnInfo = 'No! Armed exception, recommend retest!'
        
        self.dataInfo = [
            [date, frame, caseID, SHITL, testinfo, datapath, warnInfo]
        ]

        ff = pd.DataFrame(self.dataInfo)
        ff.to_csv(infop, mode='a', index=False, header=False)

        self.to_html(self.dataInfo, DataREG.html_file_path, DataREG.bk_path)
    
    def Test_result_Record(self):
        infop = os.path.abspath(os.path.join(sys.path[0],'..','..','data','TestResultInfo.csv'))
        if not os.path.exists(infop):
            header = [
                ['CaseID','FaultID', 'CaseDescription', 'FaultMode','ControlSequence','TestResult']
            ]

            df = pd.DataFrame(header)

            df.to_csv(infop, index=False, header=False)

        
        TestResultInfo = [
            self.Info[1]
        ]

        ff = pd.DataFrame(TestResultInfo)
        ff.to_csv(infop, mode='a', index=False, header=False)

        DataREG.lock.acquire()
        self.test_result_to_html(TestResultInfo, DataREG.TestResult_html_file_path, DataREG.bk_path2)
        DataREG.lock.release()

    def CMDAna(self,cmd):
        subcmd = ';'.join(cmd)
        subcmd = subcmd.split(';')

        mtime = 0
        info = ''

        if subcmd[0][0] == '2':
            for index, cmd in enumerate(subcmd):
                cmd = cmd.split(',')
                if cmd[0] == '2':
                    info += f'{mtime}s: '
                    if cmd[1] == '1' or cmd[1] == '2':
                        info += AutoREG.RFLYSIM_CMD[self.conf[0]][cmd[1]]
                    elif cmd[1] == '3' or cmd[1] == '4' or cmd[1] == '5':
                        if cmd[2] == 'SPo' or cmd[2] == 'SVo':
                            info += cmd[2]
                        else:
                            info += AutoREG.RFLYSIM_CMD[self.conf[0]][cmd[1]].format(cmd[2],cmd[3],cmd[4])
                    elif cmd[1] == '6':
                        fp = [float(str) for str in cmd[3:] if str != cmd[2]]
                        info += AutoREG.RFLYSIM_CMD[self.conf[0]][cmd[1]].format(cmd[2],fp)
                    info += ' \n'

                if cmd[0] == '1':
                    mtime += int(float(cmd[2]))
                    if index == len(subcmd) - 1:
                        info += f'{mtime}s: Exit test!'
                
        elif subcmd[0][0] == '1':
            index = 0
            for i in range(len(subcmd)):
                if subcmd[i][0] != '1':
                    index = i
                    break
            
            for index, cmd in enumerate(subcmd[index:], start=index):
                cmd = cmd.split(',')
                if cmd[0] == '2':
                    info += f'{mtime}s: '
                    if cmd[1] == '1' or cmd[1] == '2':
                        info += AutoREG.RFLYSIM_CMD[self.conf[0]][cmd[1]]
                    elif cmd[1] == '3' or cmd[1] == '4' or cmd[1] == '5':
                        if cmd[2] == 'SPo' or cmd[2] == 'SVo':
                            info += cmd[2]
                        else:
                            info += AutoREG.RFLYSIM_CMD[self.conf[0]][cmd[1]].format(cmd[2],cmd[3],cmd[4])
                    elif cmd[1] == '6':
                        fp = [float(str) for str in cmd[3:] if str != cmd[2]]
                        info += AutoREG.RFLYSIM_CMD[self.conf[0]][cmd[1]].format(cmd[2],fp)
                    info += ' \n'

                if cmd[0] == '1':
                    mtime += int(float(cmd[2]))
                    if index == len(subcmd) - 1:
                        info += f'{mtime}s: Exit test!'
        return info

    def GetInfoP(self):
        path = None
        if AutoREG.TEST_MODE == 1:
            path = self.Mode1InsP
        elif AutoREG.TEST_MODE == 2:
            path = self.Mode2InsP
        elif AutoREG.TEST_MODE == 3:
            path = self.Mode3InsP
        elif AutoREG.TEST_MODE == 4:
            path = self.Mode4InsP
        
        path = os.path.abspath(path)
        splitstr = os.path.basename(os.path.abspath(os.path.join(sys.path[0], '..', '..')))
        '''splitstr: BETA'''

        spath = path.split(splitstr)[1]
        spath = spath[1:]

        return spath

    def RName(self):
        if AutoREG.TEST_MODE == 1:
            self.SExecute(self.Mode1InsP)
        if AutoREG.TEST_MODE == 2:
            '''Single frame Multi Instance. Folder struct named with mode/frame/TestCase_*_*/'''
            self.Mode2InsP = self.tempCaseP
            if len(DataREG.Mode2mavP) == AutoREG.MAV_NUM:
                for i in range(len(DataREG.Mode2caseP)):
                    self.Mode2InsP += f'_{DataREG.Mode2caseP[i]}'

                DataREG.Finally_Path_for_Multi = self.Mode2InsP

                if not DataREG.isDeleted:
                    if os.path.exists(self.Mode2InsP): 
                        shutil.rmtree(self.Mode2InsP)
                    DataREG.isDeleted = True

                os.rename(self.tempCaseP,self.Mode2InsP)
                self.SExecute(self.Mode2InsP)
            
        elif AutoREG.TEST_MODE == 3:
            '''Multi frame Single Instance. Folder struct named with mode/TestCase_/frame/'''
            self.Mode3InsP = self.tempCaseP
            if len(DataREG.Mode3frameP) == AutoREG.MAV_NUM:
                for i in range(len(DataREG.Mode3caseP)):
                    self.Mode3InsP += f'_{DataREG.Mode3caseP[i]}'

                DataREG.Finally_Path_for_Multi = self.Mode3InsP

                if not DataREG.isDeleted:
                    if os.path.exists(self.Mode3InsP): 
                        shutil.rmtree(self.Mode3InsP)
                    DataREG.isDeleted = True
                
                os.rename(self.tempCaseP,self.Mode3InsP)
                self.SExecute(self.Mode3InsP)
        
        elif AutoREG.TEST_MODE == 4:
            '''Multi frame multi Instance. Folder struct named with mode/TestCase_/frame/'''
            self.Mode4InsP = self.tempCaseP
            if len(DataREG.Mode4caseP) == AutoREG.MAV_NUM:
                for i in range(len(DataREG.Mode4caseP)):
                    self.Mode4InsP += f'_{DataREG.Mode4caseP[i]}'

                DataREG.Finally_Path_for_Multi = self.Mode4InsP
                
                if not DataREG.isDeleted:
                    # If the destination folder exists, delete the reconstruction
                    if os.path.exists(self.Mode4InsP): 
                        shutil.rmtree(self.Mode4InsP)
                    DataREG.isDeleted = True
                
                os.rename(self.tempCaseP,self.Mode4InsP)
                self.SExecute(self.Mode4InsP)
        
    def SExecute(self,start_dir):
        for root, dirs, files in os.walk(start_dir):
            for file in files:
                if file.endswith('.ulg'):
                    current_dir = os.path.abspath(root)
                    os.chdir(current_dir)
                    cmd = "for %i in (*); do ulog2csv %i"
                    os.system(cmd)

    def convert_newlines_to_br(self, text):
        return re.sub(r'\n', '<br />', text)

    def generate_html_with_style(self, html_table, bk):
        html = f"""
            <html>
            <head>
            <style>
            body {{
                background-image: url('{bk}');
                background-repeat: no-repeat;
                background-size: cover;
            }}
            .d2 {{
                line-height: 50px;
                text-align: center;
                background: rgba(255,255,255,0.5);
                font-weight: bold;
                font-size: 5pt; 
                font-family: sans-serif;
                border-collapse: collapse; 
                border: 1px solid silver;
            }}
            table {{
                table-layout: fixed;
                width: 100%;
                border-collapse: collapse;
                font-family: sans-serif;
                font-size: 12pt;
            }}
            table,th {{
                padding: 10px;
                padding-left:0px;
                font-weight: bold;
                background-color: rgba(0,0,255,0.1);
                color: white;
            }}
            table,td{{
                background-color: rgba(255,0,0,0.2);
                color: white;
                white-space:pre-wrap; 
                word-wrap: break-word;
            }}
            </style>
            </head>
            <body>
            
            <div class="d2"> 
                {html_table}
            </div>

            </body>
            </html>
        """
        return html
    
    def generate_test_result_html_with_style(self, html_table, bk):
        html = f"""
            <html>
            <head>
            <style>
            body {{
                background-image: url('{bk}');
                background-repeat: no-repeat;
                background-size: cover;
            }}
            .d2 {{
                line-height: 50px;
                text-align: center;
                background: rgba(255,255,255,0.5);
                font-weight: bold;
                font-size: 5pt; 
                font-family: sans-serif;
                border-collapse: collapse; 
                border: 1px solid silver;
            }}
            table {{
                table-layout: fixed;
                width: 100%;
                border-collapse: collapse;
                font-family: sans-serif;
                font-size: 12pt;
            }}
            table,th {{
                padding: 10px;
                padding-left:0px;
                font-weight: bold;
                background-color: rgba(0,0,255,0.1);
                color: white;
            }}
            table,td{{
                background-color: rgba(255,0,0,0.2);
                color: white;
                white-space:pre-wrap; 
                word-wrap: break-word;
            }}
            </style>
            </head>
            <body>
            
            <div class="d2"> 
                {html_table}
            </div>

            </body>
            </html>
        """
        return html

    def read_data_from_html_tolist(self, html_file):
        # Read data from HTML file
        doc = lh.parse(html_file)
        tables = doc.xpath('//table')

        # Extract data from all tables
        data = []
        for table in tables:
            rows = []
            for tr in table.xpath('.//tr'):
                row = [td.text_content().strip() for td in tr.xpath('.//td')]
                rows.append(row)
            data.extend(rows)
        return data

    def generate_header(self, html_file, bk_path):

        if os.path.exists(html_file):
            return True

        header = [
            ['Date','Frame', 'CaseID', 'S/HITL','TestInfo','DataPath','Normal?']
        ]

        df = pd.DataFrame(header)
        html_table = df.to_html(index=False, header=False, escape=False)
        header = self.generate_html_with_style(html_table, bk_path)
        with open(html_file, 'w') as file:
            file.write(header)

        '''
        Mode2:
        header_html = '<tr><td><font color="white"><b>Frame</b></font></td>  <td><font color="white"><b>CaseID</b></font></td>  <td><font color="white"><b>S/HITL</b></font></td>  <td><font color="white"><b>TestInfo</b></font></td>  <td><font color="white"><b>DataPath</b></font></td> <td><font color="white"><b>Normal?</b></font></td>  </tr>'
        frame_html = '<table border="1" class="dataframe"><tbody>{}</tbody></table>'.format(header_html)
        header = self.generate_html_with_style(frame_html, bk_path)
        with open(html_file, 'w') as file:
            file.write(header)
        '''
        
        return False
    
    def generate_test_result_header(self, html_file, bk_path):

        if os.path.exists(html_file):
            return True

        header = [
            ['CaseID','FaultID', 'CaseDescription', 'FaultMode','ControlSequence','TestResult']
        ]

        df = pd.DataFrame(header)
        html_table = df.to_html(index=False, header=False, escape=False)
        header = self.generate_test_result_html_with_style(html_table, bk_path)
        with open(html_file, 'w') as file:
            file.write(header)
        
        return False

    def add_br_to_html(self, new_html_data_list):
        for index, data in enumerate(new_html_data_list):

            testInfo = data[4]
            new_html_data_list[index][4] = self.add_ControlSequence_br(testInfo)

        return new_html_data_list

    def to_html(self, dataInfo, html_file_path, bk_path):
        has_header = self.generate_header(html_file_path, bk_path)

        # 1、Get new data and replace newline tags with spaces for comparison
        sub_br = re.sub(r'\n', '', dataInfo[0][4])
        nd_sub_br = dataInfo[0][:]
        nd_sub_br[4] = sub_br[:]

        # 2、Read data in html
        html_data_list = self.read_data_from_html_tolist(html_file_path)
        dataLen = len(html_data_list)

        # 3、Compare row by row, if the same data is found, replace
        frame, caseID, SHITL = nd_sub_br[1], str(nd_sub_br[2]), nd_sub_br[3]
        # print(f'frame:{frame} caseID:{caseID} SHITL:{SHITL} testInfo:{testInfo}')
    
        if dataLen >=2:
            # 3.1、If there is data, first compare the new data and the original data to see if they are the same. If they are the same, replace them.，
            samekey = False
            for index, new in enumerate(html_data_list):
                if new[1] == frame and new[2] == caseID and new[3] == SHITL:
                    # print('The same data exists, start replacing new data')
                    html_data_list[index] = nd_sub_br[:]
                    samekey = True
                    break
            
            # If different, append and write
            if not samekey:
                html_data_list.append(nd_sub_br)

            # 3.2、For all data, add html line break tags
            new_html_data_list = self.add_br_to_html(html_data_list)

            # 3.3、Rewrite html
            df = pd.DataFrame(new_html_data_list)
            html_table = df.to_html(index=False, header=False, escape=False)
            new_html_table = self.generate_html_with_style(html_table, bk_path)
            with open(html_file_path, 'w') as file:
                file.write(new_html_table)
        else:
            # Write the first data
            dataInfo[0][4] = self.convert_newlines_to_br(dataInfo[0][4])
            df = pd.DataFrame(dataInfo)
            html_table = df.to_html(index=False, header=False, escape=False)
            new_html_table = self.generate_html_with_style(html_table, bk_path)
            with open(html_file_path, 'a') as file:
                file.write(new_html_table)

    def test_result_to_html(self, testresultdata, html_file_path, bk_path):
        has_header = self.generate_test_result_header(html_file_path, bk_path)

        sub_br_CaseDescription = re.sub(r'\n', '', testresultdata[0][2])
        sub_br_ControlSequence = re.sub(r'\n', '', testresultdata[0][4])
        sub_br_TestResult = re.sub(r'\n', '', testresultdata[0][5])

        # 1、Get new data
        nd_sub_br = testresultdata[0][:]
        nd_sub_br[2] = sub_br_CaseDescription[:]
        nd_sub_br[4] = sub_br_ControlSequence[:]
        nd_sub_br[5] = sub_br_TestResult[:]

        # 2、Read data in html
        html_data_list = self.read_data_from_html_tolist(html_file_path)
        dataLen = len(html_data_list)

        # 3、Compare row by row, if the same data is found, replace
        frame, caseID, SHITL = self.conf[0], str(self.CaseID), self.conf[1]
        # print(f'Frame: {frame} caseID: {caseID} SHITL: {SHITL}')

        if dataLen >=2:
            # 3.1、If there is data, first compare the new data and the original data to see if they are the same. If they are the same, replace them.，
            samekey = False
            for index, new in enumerate(html_data_list):
                new_frame = self.Get_test_result_frame(new[2])
                new_caseID = new[0]
                new_SHITL = self.Get_test_result_shitl(new[2])
                if new_frame == frame and new_caseID == caseID and new_SHITL == SHITL:
                    print('The same data exists, start replacing new data')
                    html_data_list[index] = nd_sub_br[:]
                    samekey = True
                    break
            
            # If different, append and write
            if not samekey:
                html_data_list.append(nd_sub_br)

            # 3.2、For all data, add html line break tags
            new_html_data_list = self.add_br_to_test_result_html(html_data_list)

            # 3.3、Rewrite html
            df = pd.DataFrame(new_html_data_list)
            html_table = df.to_html(index=False, header=False, escape=False)
            new_html_table = self.generate_test_result_html_with_style(html_table, bk_path)
            with open(html_file_path, 'w') as file:
                file.write(new_html_table)
        else:
            # Write the first data
            testresultdata[0][2] = self.convert_newlines_to_br(testresultdata[0][2])
            testresultdata[0][4] = self.convert_newlines_to_br(testresultdata[0][4])
            testresultdata[0][5] = self.convert_newlines_to_br(testresultdata[0][5])

            df = pd.DataFrame(testresultdata)
            html_table = df.to_html(index=False, header=False, escape=False)
            new_html_table = self.generate_test_result_html_with_style(html_table, bk_path)
            with open(html_file_path, 'a') as file:
                file.write(new_html_table)
    
    def add_br_to_test_result_html(self, new_html_data_list):
        for index, data in enumerate(new_html_data_list[1:],1):

            CaseDescription = data[2]
            ControlSequence = data[4]
            TestResult = data[5]

            need_add_str = [CaseDescription, ControlSequence, TestResult]

            for ind, add_str in enumerate(need_add_str):
                if ind == 0:
                    variables = [self.Get_test_result_frame(add_str), self.Get_test_result_shitl(add_str)]
                    new_html_data_list[index][2] = self.add_CaseDescription_br(add_str, variables)
                    pass

                if ind == 1:
                    new_html_data_list[index][4] = self.add_ControlSequence_br(add_str)
                    pass

                if ind == 2:
                    new_html_data_list[index][5] = self.add_TestResult_br(add_str)
                    pass
            
        return new_html_data_list

    def add_CaseDescription_br(self, caseDescription, variables):
        for variable in variables:
            pattern = re.compile(rf"({variable}\s*)")
            caseDescription = re.sub(pattern, r"\1<br />", caseDescription)
        
        return caseDescription

    def add_ControlSequence_br(self, testInfo):
        # Prepend a newline character before all times except the first
        pattern = r'(\d+s:)'
        matches = re.findall(pattern, testInfo)
        if matches:
            for match in matches[1:]:
                # Handle the case of mistaken addition
                if re.findall(r'(\d+)\<br />(\d+s)', testInfo):
                    err = re.findall(r'\b(\d+)\<br />(\d+s)\b', testInfo)
                    for er in err:
                        testInfo = testInfo.replace(er[0]+'<br />'+er[1], er[0]+er[1])

                testInfo = testInfo.replace(match, '<br />' + match)
        
        # Add <br> string before Fault injection type and Fault injection parameters
        if 'Fault injection type' in testInfo:
            testInfo = testInfo.replace('Fault injection type:', '<br />Fault injection type:')
        if 'Fault injection parameters' in testInfo:
            testInfo = testInfo.replace('Fault injection parameters:', '<br />Fault injection parameters:')

        return testInfo[:]

    def add_TestResult_br(self, testresult):
        keys = [key for key in AutoREG.TEST_RESULT.keys() if key != 'Is_Fall']
        for key in keys:
            testresult = testresult.replace(key, "<br />" + key)
        
        return testresult
        
    
    def Get_test_result_frame(self, text):
        pattern = r"Frame:\s*(Quadcopter|Fixedwing|Vtol)"
        match = re.search(pattern, text)

        if match:
            return match.group(1)

        return [None]

    def Get_test_result_shitl(self, text):
        pattern = r"S/HITL:\s*(SITL|HITL)"
        match = re.search(pattern, text)

        if match:
            return match.group(1)

        return [None]

    def InfoDown(self):
        DataREG.dataPollNum += 1
        # Wait for all processes to modify the path name
        while True:
            if DataREG.dataPollNum == AutoREG.MAV_NUM:
                break
        
        # Update the latest path
        if AutoREG.TEST_MODE == 2:
            self.Mode2InsP = DataREG.Finally_Path_for_Multi + f'/mav{self.mavTestID}'
        elif AutoREG.TEST_MODE == 3:
            self.Mode3InsP = DataREG.Finally_Path_for_Multi + f'/{self.conf[0]}'
        elif AutoREG.TEST_MODE == 4:
            self.Mode4InsP = DataREG.Finally_Path_for_Multi + f'/mav{self.mavTestID}'
        
        DataREG.lock.acquire()
        # Start writing to html and csv
        self.InfoRecord()
        DataREG.lock.release()






