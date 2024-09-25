import threading

class MavDataRec:
    # constructor function
    def __init__(self, mav):
        self.mav = mav
        # 在这里添加列表变量

        self.msgDic={}
        self.NameList=['']
        self.LenList=0


    # 开始监听消息的函数
    def startRecMsg(self,NameList=['HIGHRES_IMU'],LenList=[10]):
        if len(NameList) != len(LenList):
            disp('Error: the input length not match!')
            return

        self.NameList = NameList
        self.LenList = LenList

        for i in range(len(self.NameList)):
            self.msgDic[self.NameList[i]]=[] # 初始化为列表

        # 创建线程并开始监听
        self.t1 = threading.Thread(target=self.getMavMsg, args=())
        self.t1Stop=False # 停止标志位
        self.t1.start()



    # 开始监听消息的函数
    def stopRecMsg(self):
        self.t1Stop=True # 停止标志位
        self.t1.join()        

    def getMavMsg(self):
        while(True):
            if self.t1Stop: # 如果停止标志位启用，就跳出循环
                break

            # 阻塞，直到受到Mavlink消息
            self.mav.uavEvent.wait()
            msg = self.mav.uavMsg
            # 重新使能阻塞状态
            self.mav.uavEvent.clear()

            for i in range(len(self.NameList)):
                if msg.get_type() == self.NameList[i]:
                    self.msgDic[self.NameList[i]].append(msg) # 存入一个数据
                    if len(self.msgDic[self.NameList[i]])>=self.LenList[i]+1:
                        del self.msgDic[self.NameList[i]][0] # 如果大于指定数量，就删除最前面的一个


