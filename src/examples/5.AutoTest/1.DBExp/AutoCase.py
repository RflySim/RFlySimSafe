import pandas as pd
import os,sys,json
import sqlite3

class mavdb:
    def __init__(self,db,json,mode):
        self.db = db
        self.json = json
        self.cursor = 0
        self.mydb = 0
        self.mode = mode

    def DICT_FACTORY(self,cursor,row):  
        d = {}  
        for idx, col in enumerate(cursor.description):  
            d[col[0]] = row[idx]  
        return d

    def GET_CURSOR(self):
        imPath = os.path.join(sys.path[0],self.db)
        self.mydb = sqlite3.connect(imPath)
        self.mydb.row_factory = self.DICT_FACTORY
        self.cursor=self.mydb.cursor()

    def FaultIDIte(self,ID):
        return list(set(ID))

    def AutoCase(self,StartID,Subsystem,Component,FaultID,FaultType,FaultMode,ControlSequence,FaultParamStart,FaultParamEnd,Step):
        Num = StartID
        AllCase = {}
        ParamNum = len(FaultParamStart)
        FaultIDItem = ''.join([str(val) for val in self.FaultIDIte(FaultID)])
        for i in range(ParamNum):  
            start = FaultParamStart[i]
            if i >= 1:
                FaultParamStart[i] = round(FaultParamStart[i] + Step,2)
            while FaultParamStart[i] <= FaultParamEnd[i]:
                FaultIDD = [str(val) for val in FaultID] 
                FaultIDD = ','.join(FaultIDD) + ',' 
                FaultParam = [str(val) for val in FaultParamStart] # 转字符串
                FaultParam = ','.join(FaultParam) # 转字符串
                CS = [ControlSequence,FaultIDD,FaultParam,';1,1,10']
                CSD = ''.join(CS)
                data = {
                "CaseID": StartID,
                "Subsystem": Subsystem,
                "Component": Component,
                "FaultID": FaultIDItem,
                "FaultType": FaultType,
                "FaultMode": FaultMode,
                "FaultParams": FaultParam,
                "ControlSequence": CSD,
                "TestStatus": "Finished"
                }
                AllCase[Num] = data
                Num += 1
                StartID += 1
                FaultParamStart[i] = round(FaultParamStart[i] + Step,2)
            FaultParamStart[i] = FaultParamEnd[i]
        AllCase = json.dumps(AllCase,indent=4)
        with open(self.json,self.mode) as f:
            f.write(AllCase)

    def GetJson(self):
        f = open(self.json, 'r')
        content = f.read()
        jsoncase = json.loads(content)
        f.close()
        return jsoncase

    def ISSAMEID(self,ID):
        self.GET_CURSOR()
        sql = '''
        select * from TEST_CASE
        where CaseID = {}
        '''.format(ID)
        self.cursor.execute(sql)
        data = self.cursor.fetchall()
        if len(data) != 0:
            sql2 = '''
            delete from TEST_CASE
            where CaseID = {}
            '''.format(ID)
            self.cursor.execute(sql2)
            self.mydb.commit()

    def JsonToSql(self):
        self.GET_CURSOR()
        case = self.GetJson()
        s = int(list(case.keys())[0])
        for l in range(s,len(case)+s):
            Testcase = []
            for d in case[f"{l}"].items():
                Testcase.append(d[1])
            sql = '''
                    insert into TEST_CASE
                    (CaseID,Subsystem,Component,FaultID,FaultType,FaultMode,FaultParams,ControlSequence,TestStatus)
                    values(?,?,?,?,?,?,?,?,?)
                '''
            Testcase = [str(var) for var in Testcase]
            values = Testcase
            self.ISSAMEID(values[0])
            self.cursor.execute(sql,values)
            self.mydb.commit()

basepath = os.path.dirname(__file__)

mavd1 = mavdb('Quadcopter.db', basepath + '/123542.json','w')
mavd1.AutoCase(346,'Sensor','Accelerometer',[123542],"Accelerometer noise","Accelerometer noise interference","2,1;1,1,5;2,3,150,0,-30;1,1,10;2,4,1000,0,-30;1,1,10;2,6,",[0],[30],1)

mavd2 = mavdb('Quadcopter.db', basepath +'/123543.json','w')
mavd2.AutoCase(558,'Sensor','Gyroscope',[123543],"Gyroscope noise","Gyroscope noise interference","2,1;1,1,5;2,3,150,0,-30;1,1,10;2,4,1000,0,-30;1,1,10;2,6,",[0],[30],1)

mavd3 = mavdb('Quadcopter.db', basepath +'/123544.json','w')
mavd3.AutoCase(891,'Sensor','Magnetometer',[123544],"Magnetometer noise","Magnetometer noise interference","2,1;1,1,5;2,3,150,0,-30;1,1,10;2,4,1000,0,-30;1,1,10;2,6,",[0],[30],1)

mavd4 = mavdb('Quadcopter.db', basepath +'/123545.json','w')
mavd4.AutoCase(922,'Sensor','Barometer',[123545],"Barometer noise","Barometer noise interference","2,1;1,1,5;2,3,150,0,-30;1,1,10;2,4,1000,0,-30;1,1,10;2,6,",[0],[30],1)

mavd5 = mavdb('Quadcopter.db',basepath + '/123546.json','w')
mavd5.AutoCase(953,'Sensor','GPS',[123546,123546],"GPS noise","GPS noise interference","2,1;1,1,5;2,3,150,0,-30;1,1,10;2,4,1000,0,-30;1,1,10;2,6,",[0,3,10],[250,3,10],5)

mavd = [mavd1, mavd2, mavd3, mavd4, mavd5]
for mavdd in mavd:
    mavdd.JsonToSql()

        
 

            

    
