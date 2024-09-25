
from email import header
from re import A
import socket
import threading
import time
import cv2
import numpy as np
import struct
import mmap
import json
import sys
import os
import math
import copy
import platform
import include.AutoREG as AutoREG

isLinux = False
if platform.system().lower() == 'linux':
    isLinux = True
    try:
        from logging import exception
        from typing import Any
        from xml.etree.ElementTree import tostring
        import yaml
        import rospy
        import sensor_msgs.msg as sensor
        import std_msgs.msg as std_msg

    except ImportError:
        print('Faild to load ROS labs')

# 是否启用ROS图像转发功能
# IsEnable ROS image topic forwarding
isEnableRosTrans = False

# 加入其他的ROS库


class Queue:
    """pacth
    """

    def __init__(self):
        self.items = []

    def enqueue(self, item):
        self.items.insert(0, item)

    def dequeue(self):
        return self.items.pop()

    def is_empty(self):
        return self.items == []

    def size(self):
        return len(self.items)


# 注意:本条消息会发送给指定远端电脑的端口20005
# struct RflyTimeStmp{
#     int checksum; //校验位，取123456789
#     int copterID; //当前飞机的ID号
#     long long SysStartTime; //Windows下的开始仿真时的时间戳（单位毫秒，格林尼治标准起点）
#     long long SysCurrentTime;//Windows下的当前时间戳（单位毫秒，格林尼治标准起点）
#     long long HeartCount; //心跳包的计数器
# } 2i3q
class RflyTimeStmp:
    def __init__(self):
        self.checksum = 1234567897
        self.copterID = 0
        self.SysStartTime = 0
        self.SysCurrentTime = 0
        self.HeartCount = 0


class VisionSensorReq:
    """ This is a class (C++ struct) that sent to UE4 to request and set camera parameters.
        # struct VisionSensorReq {
        # 	uint16 checksum; //数据校验位，12345
        # 	uint16 SeqID; //内存序号ID
        # 	uint16 TypeID; //传感器类型ID
        # 	uint16 TargetCopter; //绑定的目标飞机     //可改变
        # 	uint16 TargetMountType; //绑定的类型    //可改变
        # 	uint16 DataWidth;   //数据或图像宽度
        # 	uint16 DataHeight; //数据或图像高度
        # 	uint16 DataCheckFreq; //检查数据更新频率
        # 	uint16 SendProtocol[8]; //传输类型（共享内存、UDP传输无压缩、UDP视频串流），IP地址，端口号，...
        # 	float CameraFOV;  //相机视场角（仅限视觉类传感器）  //可改变
        # 	float SensorPosXYZ[3]; // 传感器安装位置    //可改变
        # 	float SensorAngEular[3]; //传感器安装角度   //可改变
        # 	float otherParams[8]; //预留的八位数据位
        # }16H15f
    """

    def __init__(self):
        self.checksum = 12345
        self.SeqID = 0
        self.TypeID = 1
        self.TargetCopter = 1
        self.TargetMountType = 0
        self.DataWidth = 0
        self.DataHeight = 0
        self.DataCheckFreq = 0
        self.SendProtocol = [0, 0, 0, 0, 0, 0, 0, 0]
        self.CameraFOV = 90
        self.SensorPosXYZ = [0, 0, 0]
        self.SensorAngEular = [0, 0, 0]
        self.otherParams = [0, 0, 0, 0, 0, 0, 0, 0]


class imuDataCopter:
    """ This is a class (C++ struct) for IMU data receive from CopterSim
        # struct imuDataCopter{
        #     int checksum; //数据校验位1234567898
        #     int seq; //消息序号
        #     double timestmp;//时间戳
        #     float acc[3];
        #     float rate[3];
        # }   //2i1d6f    
    """

    def __init__(self):
        self.checksum = 1234567898
        self.seq = 0
        self.timestmp = 0
        self.acc = [0, 0, 0]
        self.rate = [0, 0, 0]
        if isEnableRosTrans:
            self.time_record = -1
            self.isUseTimeAlign = True  # 是否使用与图像时间对其的方式发布数据
            self.rostime = rospy.Time.now()
            self.imu_pub = rospy.Publisher(
                "/rflysim/imu", sensor.Imu, queue_size=1)
            self.time_queue = Queue()
            self.newest_time_img = -1
            self.test_imu_time = 0
            self.test_sum = 0
            self.count = 0
            self.ros_imu = sensor.Imu()
            self.imu_frame_id = "imu"
            self.ros_imu.header.frame_id = self.imu_frame_id

    def AlignTime(self, img_time):  # 原子操作不需要用锁，用了反而较低下率
        self.newest_time_img = img_time
        # print("queue size: ",self.time_queue.size())
        # print("current <img:%f,imu:%f>"% (img_time,self.test_imu_time))
        # self.test_sum += abs(self.newest_time_img - self.test_imu_time)
        # self.count +=1
        # if(self.count == 10):
        #     print("====",self.test_sum/self.count)
        #     self.count = 0
        #     self.test_sum = 0

        pass

    def Imu2ros(self):
        if isEnableRosTrans:
            # ros_imu = sensor.Imu()
            acc = [0, 0, 0]
            rate = [0, 0, 0]
            if(self.isUseTimeAlign):
                self.time_queue.enqueue((self.timestmp, [self.acc, self.rate]))
                if(self.newest_time_img == -1):
                    # 为了保证Imu数据发布过程中保持稳定频率发布，不受图像发布频率不稳定干扰，采用的方式数据程序启动前对齐时间，数据发布过程不做对齐
                    # 因此，对于不是十分稳定数据发布处理，需要在应用端处理
                    return
                if(self.time_record == -1):
                    imu = self.time_queue.dequeue()
                    self.time_record = imu[0]
                    self.rostime = rospy.Time.now()
                    # return
                imu = self.time_queue.dequeue()
                self.test_imu_time = imu[0]
                self.ros_imu.header.stamp = self.rostime + \
                    rospy.Duration(imu[0] - self.time_record)
                acc[0] = imu[1][0][0]
                acc[1] = imu[1][0][1]
                acc[2] = imu[1][0][2]
                rate[0] = imu[1][1][0]
                rate[1] = imu[1][1][1]
                rate[2] = imu[1][1][2]
            else:
                if(self.time_record == -1):
                    self.time_record = self.timestmp
                    self.rostime = rospy.Time.now()
                self.ros_imu.header.stamp = self.rostime + \
                    rospy.Duration(self.timestmp - self.time_record)
                acc[0] = self.acc[0]
                acc[1] = self.acc[1]
                acc[2] = self.acc[2]
                rate[0] = self.rate[0]
                rate[1] = self.rate[1]
                rate[2] = self.rate[2]
            self.ros_imu.orientation.w = 0
            self.ros_imu.orientation.x = 0
            self.ros_imu.orientation.y = 0
            self.ros_imu.orientation.z = 0
            self.ros_imu.orientation_covariance[0] = -1
            self.ros_imu.orientation_covariance[1] = -1
            self.ros_imu.orientation_covariance[2] = -1
            self.ros_imu.orientation_covariance[3] = -1
            self.ros_imu.orientation_covariance[4] = -1
            self.ros_imu.orientation_covariance[5] = -1
            self.ros_imu.orientation_covariance[6] = -1
            self.ros_imu.orientation_covariance[7] = -1
            self.ros_imu.orientation_covariance[8] = -1
            self.ros_imu.linear_acceleration.x = acc[0]
            self.ros_imu.linear_acceleration.y = acc[1]
            self.ros_imu.linear_acceleration.z = acc[2]
            self.ros_imu.angular_velocity.x = rate[0]
            self.ros_imu.angular_velocity.y = rate[1]
            self.ros_imu.angular_velocity.z = rate[2]
            self.ros_imu.angular_velocity_covariance[0] = 0.001
            self.ros_imu.angular_velocity_covariance[4] = 0.001
            self.ros_imu.angular_velocity_covariance[8] = 0.001
            # self.ros_imu.header.stamp.secs = self.timestmp
            self.imu_pub.publish(self.ros_imu)


class SensorReqCopterSim:
    """This is a class (C++ struct) that sent to UE4 to request sensor data.
        # struct SensorReqCopterSim{
        #     uint16_t checksum;
        #     uint16_t sensorType;
        #     uint16_t updateFreq;
        #     uint16_t port;
        #     uint8_t IP[4];
        #     float Params[6];
        # } //4H4B6f    
    """

    def __init__(self):
        self.checksum = 12345
        self.sensorType = 0
        self.updateFreq = 100
        self.port = 9998
        self.IP = [127, 0, 0, 1]
        self.Params = [0, 0, 0, 0, 0, 0]


# struct reqVeCrashData {
# 	int checksum; //数据包校验码1234567897
# 	int copterID; //当前飞机的ID号
# 	int vehicleType; //当前飞机的样式
# 	int CrashType;//碰撞物体类型，-2表示地面，-1表示场景静态物体，0表示无碰撞，1以上表示被碰飞机的ID号
# 	double runnedTime; //当前飞机的时间戳
# 	float VelE[3]; // 当前飞机的速度
# 	float PosE[3]; //当前飞机的位置
# 	float CrashPos[3];//碰撞点的坐标
# 	float targetPos[3];//被碰物体的中心坐标
# 	float AngEuler[3]; //当前飞机的欧拉角
# 	float MotorRPMS[8]; //当前飞机的电机转速
#   float ray[6]; //飞机的前后左右上下扫描线
# 	char CrashedName[16];//被碰物体的名字
#  } 4i1d29f20s

class reqVeCrashData:
    def __init__(self):
        self.checksum = 1234567897
        self.copterID = 0
        self.vehicleType = 0
        self.CrashType = 0
        self.runnedTime = 0
        self.VelE = [0, 0, 0]
        self.PosE = [0, 0, 0]
        self.CrashPos = [0, 0, 0]
        self.targetPos = [0, 0, 0]
        self.AngEuler = [0, 0, 0]
        self.MotorRPMS = [0, 0, 0, 0, 0, 0, 0, 0]
        self.ray = [0, 0, 0, 0, 0, 0]
        self.CrashedName = ''

    def __init__(self, iv):
        self.checksum = iv[0]
        self.copterID = iv[1]
        self.vehicleType = iv[2]
        self.CrashType = iv[3]
        self.runnedTime = iv[4]
        self.VelE = iv[5:8]
        self.PosE = iv[8:11]
        self.CrashPos = iv[11:14]
        self.targetPos = iv[14:17]
        self.AngEuler = iv[17:20]
        self.MotorRPMS = iv[20:28]
        self.ray = iv[28:34]
        self.CrashedName = iv[34].decode('UTF-8')


class PX4SILIntFloat:
    # //输出到CopterSim DLL模型的SILints和SILFloats数据
    # struct PX4SILIntFloat{
    #     int checksum;//1234567897
    #     int CopterID;
    #     int inSILInts[8];
    #     float inSILFLoats[20];
    # };
    # struct.pack 10i20f
    def __init__(self):
        self.checksum = 0
        self.CopterID = 0
        self.inSILInts = [0, 0, 0, 0, 0, 0, 0, 0]
        self.inSILFLoats = [0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    def __init__(self, iv):
        self.checksum = iv[0]
        self.CopterID = iv[1]
        self.inSILInts = iv[2:10]
        self.inSILFLoats = iv[10:30]


class VisionCaptureApi:
    """ This is the API class for python to request image from UE4
    """

    def __init__(self):
        if isEnableRosTrans:
            rospy.init_node("RecvRFlySim3DData", anonymous=True)
            self.time_record = Any
            self.rostime = Any
            # 加入ROS节点的初始化工作
        self.udp_socket = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM)  # 创建套接字
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.udp_imu = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM)  # 创建套接字
        self.VisSensor = []
        self.Img = []
        self.ImgData = []
        self.hasData = []
        self.timeStmp = []
        self.IpList = []
        self.portList = []
        self.hasReqUE4 = False
        self.sleepCheck = 0.005
        self.ip = '127.0.0.1'
        self.isRemoteSend = False
        self.RemotSendIP = ''
        self.isUE4DirectUDP = False
        self.imu = imuDataCopter()
        self.hasIMUData = False
        self.RflyTime = RflyTimeStmp()
        self.inSilVect = []
        self.inReqVect = []
        self.inReqUpdateVect = []
        self.stopFlagUE4 = True
        self.startTime = time.time()
        self.isPrintTime = False
        self.lastIMUTime = time.time()
        self.sensors_num = 0
        if isEnableRosTrans:
            self.sensor_frame_id = ["map"]
            self.imu_frame_id = "imu"
            try:
                file = open(r"tf_cfg.yaml")
                y = yaml.safe_load(file)
                self.imu.imu_frame_id = y["imu_frame_id"]
                self.sensors_frame_id = y["sensors_frame_id"]
                self.sensors_num = y["sensors_num"]
            except IOError:
                print("使用默认的全局坐标系下的frame_id:map")

        # self.lock = threading.Lock() #应用到IMU和图像数据时间对齐处理

    def addVisSensor(self, vsr=VisionSensorReq()):
        """ Add a new VisionSensorReq struct to the list
        """
        if isinstance(vsr, VisionSensorReq):
            self.VisSensor = self.VisSensor + [copy.deepcopy(vsr)]
        else:
            raise Exception("Wrong data input to addVisSensor()")

    def sendUE4Cmd(self, cmd, windowID=-1):
        """send command to control the display style of RflySim3D
            The available command are list as follows, the command string is as b'RflyShowTextTime txt time'
            RflyShowTextTime(String txt, float time)\\ let UE4 show txt with time second
            RflyShowText(String txt)\\  let UE4 show txt 5 second
            RflyChangeMapbyID(int id)\\ Change the map to ID (int number)
            RflyChangeMapbyName(String txt)\\ Change to map with name txt
            RflyChangeViewKeyCmd(String key, int num) \\ the same as press key + num on UE4
            RflyCameraPosAngAdd(float x, float y, float z,float roll,float pitch,float yaw) \\ move the camera with x-y-z(m) and roll-pitch-yaw(degree) related to current pos
            RflyCameraPosAng(float x, float y, float z, float roll, float pitch, float yaw) \\ set the camera with x-y-z(m) and roll-pitch-yaw(degree) related to UE origin
            RflyCameraFovDegrees(float degrees) \\ change the cameras fov (degree)
            RflyChange3DModel(int CopterID, int veTypes=0) \\ change the vehicle 3D model to ID
            RflyChangeVehicleSize(int CopterID, float size=0) \\change vhielce's size
            RflyMoveVehiclePosAng(int CopterID, int isFitGround, float x, float y, float z, float roll, float pitch, float yaw) \\ move the vehicle's  x-y-z(m) and roll-pitch-yaw(degree) related to current pos
            RflySetVehiclePosAng(int CopterID, int isFitGround, float x, float y, float z, float roll, float pitch, float yaw) \\ set the vehilce's x-y-z(m) and roll-pitch-yaw(degree) related to UE origin
            RflyScanTerrainH(float xLeftBottom(m), float yLeftBottom(m), float xRightTop(m), float yRightTop(m), float scanHeight(m), float scanInterval(m)) \\ send command to let UE4 scan the map to generate png and txt files
            RflyCesiumOriPos(double lat, double lon, double Alt) \\ change the lat, lon, Alt (degrees) of the Cesium map origin
            RflyClearCapture \\ clear the image capture unit
            struct Ue4CMD{
                int checksum;
                char data[52];
            }
        """
        buf = struct.pack("i52s", 1234567890, cmd)
        if windowID < 0:
            if self.ip == '127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                # multicast address, send to all RflySim3Ds on all PC in LAN
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009))
        else:
            if self.ip != '127.0.0.1' and self.ip != '255.255.255.255':
                # ensure this PC can reciver message under specify IP mode
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID))
            # specify PC's IP to send
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID))

    def sendUE4Attatch(self, CopterIDs, AttatchIDs, AttatchTypes, windowID=-1):
        """ Send msg to UE4 to attach a vehicle to another (25 vehicles);
        CopterIDs,AttatchIDs,AttatchTypes can be a list with max len 25
        """
        # change the 1D variable to 1D list
        if isinstance(CopterIDs, int):
            CopterIDs = [CopterIDs]

        if isinstance(AttatchIDs, int):
            AttatchIDs = [AttatchIDs]

        if isinstance(AttatchTypes, int):
            AttatchTypes = [AttatchTypes]

        if not isinstance(CopterIDs, list) or not isinstance(AttatchIDs, list) or not isinstance(AttatchTypes, list):
            print('Error: Wrong sendUE4Attatch input Type')
            return

        if len(CopterIDs) != len(AttatchIDs) or len(CopterIDs) != len(AttatchTypes) or len(CopterIDs) > 25:
            print('Error: Wrong sendUE4Attatch input dimension')
            return

        vLen = len(CopterIDs)
        if vLen < 25:  # Extend the IDs to 25D
            CopterIDs = CopterIDs + [0]*(25-vLen)
            AttatchIDs = AttatchIDs + [0]*(25-vLen)
            AttatchTypes = AttatchTypes + [0]*(25-vLen)

        if vLen > 25:
            CopterIDs = CopterIDs[0:25]
            AttatchIDs = AttatchIDs[0:25]
            AttatchTypes = AttatchTypes[0:25]

        # struct VehicleAttatch25 {
        # 	int checksum;//1234567892
        # 	int CopterIDs[25];
        # 	int AttatchIDs[25];
        # 	int AttatchTypes[25];//0：正常模式，1：相对位置不相对姿态，2：相对位置+偏航（不相对俯仰和滚转），3：相对位置+全姿态（俯仰滚转偏航）
        # }i25i25i25i

        buf = struct.pack("i25i25i25i", 1234567892, *
                          CopterIDs, *AttatchIDs, *AttatchTypes)
        if windowID < 0:
            if self.ip == '127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                # multicast address, send to all RflySim3Ds on all PC in LAN
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009))
        else:
            if self.ip != '127.0.0.1' and self.ip != '255.255.255.255':
                # ensure this PC can reciver message under specify IP mode
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID))
            # specify PC's IP to send
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID))

    def sendUE4Pos(self, copterID=1, vehicleType=3, MotorRPMSMean=0, PosE=[0, 0, 0], AngEuler=[0, 0, 0], windowID=-1):
        """send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
        """
        VelE = [0, 0, 0]
        self.sendUE4PosNew(copterID, vehicleType, PosE, AngEuler, VelE, [
                           MotorRPMSMean]*8, -1, windowID)

    # send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
    def sendUE4Pos2Ground(self, copterID=1, vehicleType=3, MotorRPMSMean=0, PosE=[0, 0, 0], AngEuler=[0, 0, 0], windowID=-1):
        """send the position & angle information to RflySim3D to create a new 3D model or update the old model's states on the ground
            checksum =1234567891 is adopted here to tell UE4 to genearete a object always fit the ground
            struct SOut2SimulatorSimple {
                int checkSum; //1234567890 for normal object, 1234567891 for fit ground object
                int copterID;  //Vehicle ID
                int vehicleType;  //Vehicle type
                float MotorRPMSMean; // mean motor speed
                float PosE[3];   //NED vehicle position in earth frame (m)
                float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
            }  3i7f
        """
        buf = struct.pack("3i7f", 1234567891, copterID, vehicleType, MotorRPMSMean,
                          PosE[0], PosE[1], PosE[2], AngEuler[0], AngEuler[1], AngEuler[2])
        if windowID < 0:
            if self.ip == '127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                # multicast address, send to all RflySim3Ds on all PC in LAN
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009))
        else:
            if self.ip != '127.0.0.1' and self.ip != '255.255.255.255':
                # ensure this PC can reciver message under specify IP mode
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID))
            # specify PC's IP to send
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID))

    def sendUE4PosScale(self, copterID=1, vehicleType=3, MotorRPMSMean=0, PosE=[0, 0, 0], AngEuler=[0, 0, 0], Scale=[1, 1, 1], windowID=-1):
        """send the position & angle information to RflySim3D to create a new 3D model or update the old model's states with changing the scale
            struct SOut2SimulatorSimple1 {
                int checkSum; //1234567890 for normal object, 1234567891 for fit ground object
                int copterID;  //Vehicle ID
                int vehicleType;  //Vehicle type
                float MotorRPMSMean; // mean motor speed
                float PosE[3];   //NED vehicle position in earth frame (m)
                float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
                float Scale[3];
            }  3i10f
        """
        buf = struct.pack("3i10f", 1234567890, copterID, vehicleType, MotorRPMSMean,
                          PosE[0], PosE[1], PosE[2], AngEuler[0], AngEuler[1], AngEuler[2], Scale[0], Scale[1], Scale[2])
        if windowID < 0:
            if self.ip == '127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                # multicast address, send to all RflySim3Ds on all PC in LAN
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009))
        else:
            if self.ip != '127.0.0.1' and self.ip != '255.255.255.255':
                # ensure this PC can reciver message under specify IP mode
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID))
            # specify PC's IP to send
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID))

    def sendUE4PosScale2Ground(self, copterID=1, vehicleType=3, MotorRPMSMean=0, PosE=[0, 0, 0], AngEuler=[0, 0, 0], Scale=[1, 1, 1], windowID=-1):
        """send the position & angle information to RflySim3D to create a new 3D model or update the old model's states with changing the scale
            checksum =1234567891 is adopted here to tell UE4 to genearete a object always fit the ground
            struct SOut2SimulatorSimple1 {
                int checkSum; //1234567890 for normal object, 1234567891 for fit ground object
                int copterID;  //Vehicle ID
                int vehicleType;  //Vehicle type
                float MotorRPMSMean; // mean motor speed
                float PosE[3];   //NED vehicle position in earth frame (m)
                float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
                float Scale[3];
            }  3i10f
        """
        buf = struct.pack("3i10f", 1234567891, copterID, vehicleType, MotorRPMSMean,
                          PosE[0], PosE[1], PosE[2], AngEuler[0], AngEuler[1], AngEuler[2], Scale[0], Scale[1], Scale[2])
        if windowID < 0:
            if self.ip == '127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                # multicast address, send to all RflySim3Ds on all PC in LAN
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009))
        else:
            if self.ip != '127.0.0.1' and self.ip != '255.255.255.255':
                # ensure this PC can reciver message under specify IP mode
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID))
            # specify PC's IP to send
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID))

    # send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
    def sendUE4PosFull(self, copterID, vehicleType, MotorRPMS, VelE, PosE, RateB, AngEuler, windowID=-1):
        """send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
            # struct SOut2Simulator {
            #     int copterID;  //Vehicle ID
            #     int vehicleType;  //Vehicle type
            #     double runnedTime; //Current Time stamp (s)
            #     float VelE[3];   //NED vehicle velocity in earth frame (m/s)
            #     float PosE[3];   //NED vehicle position in earth frame (m)
            #     float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
            #     float AngQuatern[4]; //Vehicle attitude in Quaternion
            #     float MotorRPMS[8];  //Motor rotation speed (RPM)
            #     float AccB[3];       //Vehicle acceleration in body frame x y z (m/s/s)
            #     float RateB[3];      //Vehicle angular speed in body frame x y z (rad/s)
            #     double PosGPS[3];    //vehicle longitude, latitude and altitude (degree,degree,m)

            # }
            # typedef struct _netDataShort {
            #     int tg;
            #     int        len;
            #     char       payload[192];
            # }netDataShort;

        """
        runnedTime = -1
        # VelE=[0,0,0]
        AngQuatern = [0, 0, 0, 0]
        AccB = [0, 0, 0]
        # RateB=[0,0,0]
        PosGPS = PosE
        # buf for SOut2Simulator, len=152
        buf0 = struct.pack("2i1d27f3d", copterID, vehicleType, runnedTime, *VelE,
                           *PosE, *AngEuler, *AngQuatern, *MotorRPMS, *AccB, *RateB, *PosGPS)
        # buf for remaining 192-152=40bytes of payload[192] of netDataShort
        buf1 = bytes([0]*(192-len(buf0)))
        # buf for tg and len in netDataShort
        buf2 = struct.pack("2i", 2, len(buf0))
        # buf for netDataShort
        buf = buf2+buf0+buf1
        # print(len(buf))
        if windowID < 0:
            if self.ip == '127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                # multicast address, send to all RflySim3Ds on all PC in LAN
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009))
        else:
            if self.ip != '127.0.0.1' and self.ip != '255.255.255.255':
                # ensure this PC can reciver message under specify IP mode
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID))
            # specify PC's IP to send
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID))
        #print('Message Send')

    def sendUE4ExtAct(self, copterID=1, ActExt=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], windowID=-1):
        # struct Ue4ExtMsg {
        #     int checksum;//1234567894
        #     int CopterID;
        #     double runnedTime; //Current  stamp (s)
        #     float ExtToUE4[16];
        # }
        # struct.pack 2i1d16f
        runnedTime = time.time()-self.startTime
        checkSum = 1234567894
        buf = struct.pack("2i1d16f", checkSum, copterID, runnedTime, *ActExt)
        if windowID < 0:
            if self.ip == '127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                # multicast address, send to all RflySim3Ds on all PC in LAN
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009))
        else:
            if self.ip != '127.0.0.1' and self.ip != '255.255.255.255':
                # ensure this PC can reciver message under specify IP mode
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID))
            # specify PC's IP to send
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID))
        #print('Message Send')

    def sendUE4PosSimple(self, copterID, vehicleType, PWMs, VelE, PosE, AngEuler, runnedTime=-1, windowID=-1):
        """ send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
            # //输出到模拟器的数据
            # struct SOut2SimulatorSimpleTime {
            #     int checkSum;
            #     int copterID;  //Vehicle ID
            #     int vehicleType;  //Vehicle type
            #     float PWMs[8];
            #     float PosE[3];   //NED vehicle position in earth frame (m)
            #     float VelE[3];
            #     float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
            #     double runnedTime; //Current Time stamp (s)
            # };
            #struct.pack 3i17f1d
        """
        # if runnedTime<0:
        #     runnedTime = time.time()-self.startTime
        checkSum = 1234567890
        # pack for SOut2SimulatorSimpleTime
        buf = struct.pack("3i17f1d", checkSum, copterID, vehicleType,
                          *PWMs, *PosE, *VelE, *AngEuler, runnedTime)
        # print(len(buf))
        if windowID < 0:
            if self.ip == '127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                # multicast address, send to all RflySim3Ds on all PC in LAN
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009))
        else:
            if self.ip != '127.0.0.1' and self.ip != '255.255.255.255':
                # ensure this PC can reciver message under specify IP mode
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID))
            # specify PC's IP to send
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID))
        #print('Message Send')

    def sendUE4PosNew(self, copterID=1, vehicleType=3, PosE=[0, 0, 0], AngEuler=[0, 0, 0], VelE=[0, 0, 0], PWMs=[0]*8, runnedTime=-1, windowID=-1):
        """ send the position & angle information to RflySim3D to create a new 3D model or update the old model's states
            # //输出到模拟器的数据
            # struct SOut2SimulatorSimpleTime {
            #     int checkSum; //1234567890
            #     int copterID;  //Vehicle ID
            #     int vehicleType;  //Vehicle type
            #     float PWMs[8];
            #     float VelE[3];
            #     float AngEuler[3];  //Vehicle Euler angle roll pitch yaw (rad) in x y z
            #     double PosE[3];   //NED vehicle position in earth frame (m)
            #     double runnedTime; //Current Time stamp (s)
            # };
            #struct.pack 3i14f4d
        """
        # if runnedTime<0:
        #     runnedTime = time.time()-self.startTime
        checkSum = 1234567890
        # pack for SOut2SimulatorSimpleTime
        buf = struct.pack("3i14f4d", checkSum, copterID, vehicleType,
                          *PWMs, *VelE, *AngEuler, *PosE, runnedTime)
        # print(len(buf))
        if windowID < 0:
            if self.ip == '127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                # multicast address, send to all RflySim3Ds on all PC in LAN
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009))
        else:
            if self.ip != '127.0.0.1' and self.ip != '255.255.255.255':
                # ensure this PC can reciver message under specify IP mode
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID))
            # specify PC's IP to send
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID))
        #print('Message Send')

    # //输出到CopterSim DLL模型的SILints和SILFloats数据
    # struct PX4SILIntFloat{
    #     int checksum;//1234567897
    #     int CopterID;
    #     int inSILInts[8];
    #     float inSILFLoats[20];
    # };
    # struct.pack 10i20f

    def sendUE4PosScale100(self, copterID, vehicleType, PosE, AngEuler, MotorRPMSMean, Scale, isFitGround=False, windowID=-1):
        """send the position & angle information to RflySim3D to create 100 vehicles once
            #struct Multi3DData100New {
            #    int checksum;
            #    uint16 copterID[100];
            #    uint16 vehicleType[100];
            #    float PosE[300];
            #    float AngEuler[300];
            #    uint16 Scale[300];
            #    float MotorRPMSMean[100];
            #}
            #checksum是数据校验位，1234567890表示正常数据，1234567891表示飞机始终贴合地面
            #copterID是100维整型数组，飞机ID，这个从1给到100即可,如果ID给0则不显示次飞机
            #vehicleType是100维整型数组，飞机类型，四旋翼给3即可
            #MotorRPMSMean是100维数组，飞机螺旋桨转速，单位RPM，四旋翼默认给1000即可
            #PosE是300维数组，飞机位置，可以随机生成，单位是m
            #AngEuler是300维数组，飞机姿态角，单位弧度，默认都给0，表示正常放置
            #Scale是300维的数组，数据是xyz显示的尺寸*100，默认都给100表示实际大小即1倍
        """

        checksum = 1234567890
        if isFitGround:
            checksum = 1234567891
        buf = struct.pack("i200H600f300H100f", checksum, *copterID,
                          *vehicleType, *PosE, *AngEuler, *Scale, *MotorRPMSMean)
        if windowID < 0:
            if self.ip == '127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                # multicast address, send to all RflySim3Ds on all PC in LAN
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009))
        else:
            if self.ip != '127.0.0.1' and self.ip != '255.255.255.255':
                # ensure this PC can reciver message under specify IP mode
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID))
            # specify PC's IP to send
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID))

    def sendUE4PosScalePwm20(self, copterID, vehicleType, PosE, AngEuler, Scale, PWMs, isFitGround=False, windowID=-1):
        """send the position & angle information to RflySim3D to create 20 vehicles once
            # struct Multi3DData2New {
            #   int checksum;
            # 	uint16 copterID[20];
            # 	uint16 vehicleType[20];
            # 	float PosE[60];
            # 	float AngEuler[60];
            # 	uint16 Scale[60];
            # 	float PWMs[160];
            # }
            #checksum是数据校验位，1234567890表示正常数据，1234567891表示飞机始终贴合地面
            #copterID是20维整型数组，飞机ID，这个从1给到20即可,如果ID给0则不显示次飞机
            #vehicleType是20维整型数组，飞机类型，四旋翼给3即可
            #PosE是60维数组，飞机位置，可以随机生成，单位是m
            #AngEuler是60维数组，飞机姿态角，单位弧度，默认都给0，表示正常放置
            #Scale是300维的数组，数据是xyz显示的尺寸*100，默认都给100表示实际大小即1倍
            #PWMs是160维数组，对应20个飞机各8个旋翼转速，单位RPM，四旋翼默认给1000即可
        """
        checksum = 1234567890
        if isFitGround:
            checksum = 1234567891
        buf = struct.pack("i40H120f60H160f", checksum, *copterID,
                          *vehicleType, *PosE, *AngEuler, *Scale, *PWMs)
        if windowID < 0:
            if self.ip == '127.0.0.1':
                for i in range(6):
                    self.udp_socket.sendto(buf, (self.ip, 20010+i))
            else:
                # multicast address, send to all RflySim3Ds on all PC in LAN
                self.udp_socket.sendto(buf, ('224.0.0.10', 20009))
        else:
            if self.ip != '127.0.0.1' and self.ip != '255.255.255.255':
                # ensure this PC can reciver message under specify IP mode
                self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windowID))
            # specify PC's IP to send
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID))

    def sendReqToCopterSim(self, srcs=SensorReqCopterSim(), copterID=1):
        """ send UDP message SensorReqCopterSim to CopterSim to request a sensor data
        the copterID specify the index of CopterSim to request
        """
        if type(srcs).__name__ != 'SensorReqCopterSim':
            print('Error: input is not SensorReqCopterSim class')
            return
        u16Value = [srcs.checksum, srcs.sensorType, srcs.updateFreq, srcs.port]
        u8Value = srcs.IP
        fValue = srcs.Params
        buf = struct.pack("4H4B6f", *u16Value, *u8Value, *fValue)
        self.udp_socket.sendto(buf, ('255.255.255.255', 30100+(copterID-1)*2))

    def sendImuReqCopterSim(self, copterID=1, IP='', port=31000, freq=200):
        """send command to CopterSim to request IMU data
        copterID is the CopterID
        IP is the IP that copterSim send back to
        port is the port that CopterSim send to 
        freq is the frequency of the send data
        This function will init a thread to listen IMU data
        """
        self.sendImuReqClient(copterID, IP, port, freq)
        self.sendImuReqServe(copterID, port)

    def sendImuReqClient(self, copterID=1, IP='', port=31000, freq=200):
        """send command to CopterSim to request IMU data
        copterID is the CopterID
        IP is the IP that copterSim send back to
        port is the port that CopterSim send to 
        freq is the frequency of the send data
        """

        # if RemotSendIP has been set, the IMU rev IP will be RemotSendIP
        # else use local IP address 127.0.0.1
        if IP == '' and self.RemotSendIP != '':
            IP = self.RemotSendIP
        elif IP == '':
            IP = '127.0.0.1'

        srcs = SensorReqCopterSim()
        srcs.sensorType = 0  # IMU传感器数据
        srcs.updateFreq = freq
        if IP != '':
            cList = IP.split('.')
            if len(cList) == 4:
                srcs.IP[0] = int(cList[0])
                srcs.IP[1] = int(cList[1])
                srcs.IP[2] = int(cList[2])
                srcs.IP[3] = int(cList[3])
        srcs.port = port+copterID-1
        self.sendReqToCopterSim(srcs, copterID)  # 发送消息请求IMU数据

    def sendImuReqServe(self, copterID=1, port=31000):
        """send command to CopterSim to request IMU data
        copterID is the CopterID
        port is the port that CopterSim send to 
        This function will init a thread to listen IMU data
        """
        self.udp_imu.bind(('0.0.0.0', port))
        self.tIMU = threading.Thread(target=self.getIMUDataLoop, args=())
        self.tIMU.start()

    def getIMUDataLoop(self):
        print("Start lisening to IMU Msg")
        while True:
            try:
                buf, addr = self.udp_imu.recvfrom(65500)
                if len(buf) == 40:
                    # print(len(buf[0:12]))
                    IMUData = struct.unpack('2i1d6f', buf)
                    if IMUData[0] == 1234567898:
                        self.imu.checksum = IMUData[0]
                        self.imu.seq = IMUData[1]
                        self.imu.timestmp = IMUData[2]
                        if self.isPrintTime:
                            self.lastIMUTime = time.time()
                            print('IMU:', self.imu.timestmp)
                        self.imu.acc[0] = IMUData[3]
                        self.imu.acc[1] = IMUData[4]
                        self.imu.acc[2] = IMUData[5]
                        self.imu.rate[0] = IMUData[6]
                        self.imu.rate[1] = IMUData[7]
                        self.imu.rate[2] = IMUData[8]
                        if not self.hasIMUData:
                            self.hasIMUData = True
                            print("Got CopterSim IMU Msg!")
                        if isEnableRosTrans and self.hasIMUData:
                            self.imu.Imu2ros()
                            # 将IMU消息，推送到ROS消息中

            except:
                print("Error to listen to IMU Msg!")
                sys.exit(0)

    def StartTimeStmplisten(self, cpID=1):
        """Start to listen to 20005 port to get RflyTimeStmp of CopterID
        if cpID == 0 then only current CopterID will be listened.
        if cpID >0 then timestmp of desired CopterID will be listened.
        """
        self.udp_time = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM)  # 创建套接字
        self.udp_time.bind(('0.0.0.0', 20005))
        self.tTimeStmp = threading.Thread(target=self.TimeStmploop, args=())
        self.cpID = cpID
        self.tTimeStmp.start()

    def TimeStmploop(self):
        print("Start lisening to timeStmp Msg")
        while True:
            try:
                buf, addr = self.udp_time.recvfrom(65500)
                if len(buf) == 32:
                    # print(len(buf[0:12]))
                    TimeData = struct.unpack('2i3q', buf)
                    if TimeData[0] == 123456789:

                        cpIDTmp = TimeData[1]
                        if cpIDTmp == self.cpID:
                            self.RflyTime.checksum = TimeData[0]
                            self.RflyTime.copterID = TimeData[1]
                            self.RflyTime.SysStartTime = TimeData[2]
                            self.RflyTime.SysCurrentTime = TimeData[3]
                            self.RflyTime.HeartCount = TimeData[4]

            except:
                print("Error to listen to Time Msg!")
                sys.exit(0)

    def sendUpdateUEImage(self, vs=VisionSensorReq(), windID=0, IP='127.0.0.1'):
        if not isinstance(vs, VisionSensorReq):
            raise Exception("Wrong data input to addVisSensor()")
        intValue = [vs.checksum, vs.SeqID, vs.TypeID, vs.TargetCopter, vs.TargetMountType,
                    vs.DataWidth, vs.DataHeight, vs.DataCheckFreq]+vs.SendProtocol
        floValue = [vs.CameraFOV] + vs.SensorPosXYZ + \
            vs.SensorAngEular+vs.otherParams
        buf = struct.pack("16H15f", *intValue, *floValue)
        self.udp_socket.sendto(buf, (IP, 20010+windID))
        if self.RemotSendIP != '' and self.RemotSendIP != '127.0.0.1':
            self.udp_socket.sendto(buf, (self.RemotSendIP, 20010+windID))

    def sendReqToUE4(self, windID=0):
        """ send VisSensor list to RflySim3D to request image
        windID specify the index of RflySim3D window to send image
        """

        if len(self.VisSensor) <= 0:
            print('Error: No sensor is obtained.')
            return False

        # EmptyMem = np.zeros(66,dtype=np.int).tolist()
        # buf = struct.pack("66i",*EmptyMem)
        # self.mm0.seek(0)
        # self.mm0.write(buf)
        # self.mm0.seek(0)
        contSeq0 = False
        if self.isUE4DirectUDP or self.RemotSendIP != '':
            for i in range(len(self.VisSensor)):
                if self.VisSensor[i].SendProtocol[0] == 0:  # 如果之前设置的是共享内存方式，则强制转化为UDP直发
                    self.VisSensor[i].SendProtocol[0] = 1
                if self.RemotSendIP != '' and (self.VisSensor[i].SendProtocol[0] == 1 or self.VisSensor[i].SendProtocol[0] == 2 or self.VisSensor[i].SendProtocol[0] == 3):
                    cList = self.RemotSendIP.split('.')
                    if len(cList) == 4:
                        self.VisSensor[i].SendProtocol[1] = int(cList[0])
                        self.VisSensor[i].SendProtocol[2] = int(cList[1])
                        self.VisSensor[i].SendProtocol[3] = int(cList[2])
                        self.VisSensor[i].SendProtocol[4] = int(cList[3])
                if self.VisSensor[i].SeqID == 0:
                    contSeq0 = True

        if contSeq0:
            self.sendUE4Cmd(b'RflyClearCapture', windID)

        for i in range(len(self.VisSensor)):
            # struct VisionSensorReq {
            # 	uint16 checksum; //数据校验位，12345
            # 	uint16 SeqID; //内存序号ID
            # 	uint16 TypeID; //传感器类型ID
            # 	uint16 TargetCopter; //绑定的目标飞机     //可改变
            # 	uint16 TargetMountType; //绑定的类型    //可改变
            # 	uint16 DataWidth;   //数据或图像宽度
            # 	uint16 DataHeight; //数据或图像高度
            # 	uint16 DataCheckFreq; //检查数据更新频率
            # 	uint16 SendProtocol[8]; //传输类型（共享内存、UDP传输无压缩、UDP视频串流），IP地址，端口号，...
            # 	float CameraFOV;  //相机视场角（仅限视觉类传感器）  //可改变
            # 	float SensorPosXYZ[3]; // 传感器安装位置    //可改变
            # 	float SensorAngEular[3]; //传感器安装角度   //可改变
            # 	float otherParams[8]; //预留的八位数据位
            # }16H15f
            vs = self.VisSensor[i]
            intValue = [vs.checksum, vs.SeqID, vs.TypeID, vs.TargetCopter, vs.TargetMountType,
                        vs.DataWidth, vs.DataHeight, vs.DataCheckFreq]+vs.SendProtocol
            floValue = [vs.CameraFOV] + vs.SensorPosXYZ + \
                vs.SensorAngEular+vs.otherParams
            buf = struct.pack("16H15f", *intValue, *floValue)
            self.udp_socket.sendto(buf, ('127.0.0.1', 20010+windID))

        time.sleep(1)

        # struct UE4CommMemData {
        # 	int Checksum;//校验位，设置为1234567890
        # 	int totalNum;//最大传感器数量
        # 	int WidthHeigh[64];//分辨率宽高序列，包含最多32个传感器的
        # }
        if isLinux:  # Linux下共享内存代码
            # Linux mmap
            # SHARE_MEMORY_FILE_SIZE_BYTES = 66*4
            f = open('/dev/shm/UE4CommMemData', 'r+b')
            fd = f.fileno()
            self.mm0 = mmap.mmap(fd, 66*4)
        else:  # Windows下共享内存代码
            self.mm0 = mmap.mmap(0, 66*4, 'UE4CommMemData')  # 公共区

        Data = np.frombuffer(self.mm0, dtype=np.int32)
        checksum = Data[0]
        totalNum = Data[1]
        ckCheck = False
        # print(Data)
        if checksum == 1234567890:
            ckCheck = True
            for i in range(len(self.VisSensor)):
                isSucc = False
                vs = self.VisSensor[i]
                idx = vs.SeqID
                width = Data[2+idx*2]
                height = Data[2+idx*2+1]
                if width == vs.DataWidth and height == vs.DataHeight:
                    if idx <= totalNum:
                        isSucc = True
                if not isSucc:
                    ckCheck = False
                    break
        if not ckCheck:
            print('Error: Sensor req failed from UE4.')
            sys.exit(0)
            return False
        print('Sensor req success from UE4.')
        self.hasReqUE4 = True
        return True

    def img_udp_thrdNew(self, udpSok, idx, typeID):
        CheckSum = 1234567890
        fhead_size = struct.calcsize('4i1d')
        imgPackUnit = 60000

        seqList = []
        dataList = []
        timeList = []
        recPackNum = 0
        timeStmpStore = 0

        while True:
            buf, addr = udpSok.recvfrom(imgPackUnit+2000)  # 加一些余量，确保包头数据考虑在内
            # print(len(buf))
            if len(buf) < fhead_size:  # 如果数据包还没包头长，数据错误
                continue
            dd = struct.unpack('4i1d', buf[0:fhead_size])  # 校验，包长度，包序号，总包数，时间戳
            # print(dd)
            if dd[0] != CheckSum or dd[1] != len(buf):  # 校验位不对或者长度不对
                print('Wrong Data!')
                continue
            packSeq = dd[2]  # 包序号
            if packSeq == 0:  # 如果是第一个包
                seqList = []  # 清空数据序号列表
                dataList = []  # 清空数据缓存列表
                seqList = seqList + [packSeq]  # 提取序号
                dataList = dataList+[buf[fhead_size:]]  # 提取包头剩余数据
                timeStmpStore = dd[4]  # 提取时间戳
                recPackNum = dd[3]  # 以包头定义的总包作为接收结束标志
            else:  # 如果不是包头，直接将其存入列表
                if recPackNum == 0:
                    continue

                # 如果时间戳不一致
                if not math.isclose(timeStmpStore, dd[4], rel_tol=0.00001):
                    continue  # 跳过这个包
                seqList = seqList + [packSeq]  # 提取序号
                dataList = dataList+[buf[fhead_size:]]  # 提取包头剩余数据
            # if typeID==2:
                # print(seqList,recPackNum,len(dataList))
            if len(seqList) == recPackNum:  # 如果收到的包达到总数了，开始处理图像
                recPackNum = 0
                #print('Start Img Cap')
                data_total = b''
                dataOk = True
                for i in range(len(seqList)):
                    if seqList.count(i) < 1:
                        dataOk = False  # 如果某序号不在包中，报错
                        print('Failed to process img pack')
                        break
                    idx0 = seqList.index(i)  # 按次序搜索包序号
                    data_total = data_total+dataList[idx0]
                # if typeID==2:
                #    print(len(data_total))
                if dataOk:  # 如果数据都没问题，开始处理图像
                    # if typeID==2:
                    #    print('Start img cap',self.VisSensor[idx].SendProtocol[0])
                    if self.VisSensor[idx].SendProtocol[0] == 1 or self.VisSensor[idx].SendProtocol[0] == 3:
                        if self.VisSensor[idx].TypeID == 1 or self.VisSensor[idx].TypeID == 2 or self.VisSensor[idx].TypeID == 3:
                            nparr = np.frombuffer(data_total, np.uint8)
                            colorType = cv2.IMREAD_COLOR
                            if typeID == 2:
                                colorType = cv2.IMREAD_ANYDEPTH
                            elif typeID == 3:
                                colorType = cv2.IMREAD_GRAYSCALE
                            self.Img[idx] = cv2.imdecode(nparr, colorType)
                            if self.Img[idx] is None:
                                print('Wrong Img decode!')
                                self.hasData[idx] = False
                            else:
                                self.hasData[idx] = True
                                self.timeStmp[idx] = timeStmpStore
                                if self.isPrintTime:
                                    dTime = time.time()-self.lastIMUTime
                                    print('Img', idx, ':', '{:.5f}'.format(
                                        timeStmpStore), ', dTimeIMU: ', dTime)

                        if self.VisSensor[idx].SendProtocol[0] == 1 and (self.VisSensor[idx].TypeID == 4 or self.VisSensor[idx].TypeID == 5 or self.VisSensor[idx].TypeID == 6):
                            # print('')
                            posAng = np.frombuffer(
                                data_total, dtype=np.float32, count=6)  # pos ang
                            PointNum = np.frombuffer(
                                data_total, dtype=np.int32, count=1, offset=4*6)  # num
                            PointNum = PointNum[0]
                            #print('PointNum: ',PointNum)
                            #print('posAng: ', posAng)
                            self.ImgData[idx] = posAng.tolist() + [PointNum]

                            L = np.frombuffer(
                                data_total, dtype=np.int16, count=PointNum*3, offset=4*7)  # cloud
                            # reshape array to 4 channel image array H X W X 4
                            self.Img[idx] = L.reshape(PointNum, 3)
                            self.Img[idx] = self.Img[idx] / 32767.0 * \
                                self.VisSensor[idx].otherParams[0]
                            self.hasData[idx] = True
                            self.timeStmp[idx] = timeStmpStore

                    elif self.VisSensor[idx].SendProtocol[0] == 2:
                        dtyp = np.uint8
                        dim = 3
                        if(typeID == 1):
                            dtyp = np.uint8
                            dim = 3
                        elif(typeID == 2):
                            dtyp = np.uint16
                            dim = 1
                        elif(typeID == 3):
                            dtyp = np.uint8
                            dim = 1
                        DataWidth = self.VisSensor[idx].DataWidth
                        DataHeight = self.VisSensor[idx].DataHeight
                        L = np.frombuffer(data_total, dtype=dtyp)
                        # colorType=cv2.IMREAD_COLOR
                        # if typeID==2 or typeID==3:
                        #     colorType=cv2.IMREAD_GRAYSCALE
                        # self.Img[idx] = cv2.imdecode(nparr, colorType)
                        self.Img[idx] = L.reshape(DataHeight, DataWidth, dim)
                        self.hasData[idx] = True
                        self.timeStmp[idx] = timeStmpStore
                        if self.isPrintTime:
                            dTime = time.time()-self.lastIMUTime
                            print('Img', idx, ':', timeStmpStore,
                                  ', dTimeIMU: ', dTime)

                    if isEnableRosTrans and self.hasData[idx]:  # 如果需要发布ros消息
                        if(self.VisSensor[idx].TypeID >= 1):  # 目前任务所有取图操作都同时进行
                            self.imu.AlignTime(timeStmpStore)  # 发送时间戳到imu发布线程
                        topic_name = "/rflysim/sensor" + \
                            str(self.VisSensor[idx].SeqID)
                        frame_id = "map"  # 为了方便可视化，使用默认frame_id，在算法里使用时，需要根据实际情况修改
                        # 为了方便可视化，使用默认frame_id，在算法里使用时，需要根据实际情况修改
                        if(len(self.VisSensor) == self.sensors_num):
                            frame_id = self.sensors_frame_id[idx]
                        header = std_msg.Header()
                        header.frame_id = frame_id
                        if(self.time_record[idx] < 0.0000001):
                            self.time_record[idx] = timeStmpStore
                            self.rostime[idx] = rospy.Time.now()
                            continue
                        header.stamp = self.rostime[idx] + rospy.Duration(
                            timeStmpStore-self.time_record[idx])
                        type_id = self.VisSensor[idx].TypeID

                        # print('Img',idx,':',header.stamp.to_sec())

                        type = Any
                        msg = Any
                        if(type_id == 1 or type_id == 2 or type_id == 3):
                            encoding_ = "bgr8"
                            type = sensor.Image
                            msg = sensor.Image()
                            byte_num = 1
                            msg.header = header
                            if(type_id == 1):
                                topic_name += "/img_rgb"
                                # msg.encoding = "bgr8"
                                byte_num = 3
                            elif(type_id == 2):
                                encoding_ = "mono16"
                                topic_name += "/img_depth"
                                byte_num = 2
                            else:
                                encoding_ = "mono8"
                                topic_name += "/img_gray"
                            msg.height = self.Img[idx].shape[0]
                            msg.width = self.Img[idx].shape[1]
                            msg.encoding = encoding_
                            msg.data = self.Img[idx].tostring()
                            msg.step = msg.width * byte_num
                            # print(encoding_)
                        if(type_id == 4 or type_id == 5 or type_id == 6):
                            type = sensor.PointCloud2
                            msg = sensor.PointCloud2()
                            msg.header = header
                            if(type_id == 4):
                                topic_name += "/vehicle_lidar"
                            if(type_id == 5):
                                topic_name += "/global_lidar"
                            if(type_id == 6):
                                topic_name += "/livox_lidar"
                            msg.height = 1
                            msg.width = self.Img[idx].shape[0]
                            msg.fields = [
                                sensor.PointField(
                                    'x', 0, sensor.PointField.FLOAT32, 1),
                                sensor.PointField(
                                    'y', 4, sensor.PointField.FLOAT32, 1),
                                sensor.PointField(
                                    'z', 8, sensor.PointField.FLOAT32, 1)
                            ]
                            msg.is_bigendian = False
                            msg.point_step = 12
                            msg.row_step = msg.point_step * \
                                self.Img[idx].shape[0]
                            msg.is_dense = False
                            msg.data = np.asarray(
                                self.Img[idx], np.float32).tostring()
                        pub = rospy.Publisher(topic_name, type, queue_size=10)
                        pub.publish(msg)

    def img_mem_thrd(self, idxList):
        mmList = []
        for i in range(len(idxList)):
            idx = idxList[i]
            SeqID = self.VisSensor[idx].SeqID
            DataWidth = self.VisSensor[idx].DataWidth
            DataHeight = self.VisSensor[idx].DataHeight
            typeID = self.VisSensor[idx].TypeID
            dim = 3
            dimSize = 1
            otherSize = 0
            if(typeID == 1):
                dim = 3
                dimSize = 1
            elif(typeID == 2):
                dim = 1
                dimSize = 2
            elif(typeID == 3):
                dim = 1
                dimSize = 1
            elif(typeID == 4 or typeID == 5 or typeID == 6):
                dim = 3
                dimSize = 2
                otherSize = 4*7
            if isLinux:
                # Linux
                dataLen = DataWidth*DataHeight*dim*dimSize+1+8+otherSize
                f = open("/dev/shm/" + 'RflySim3DImg_' + str(SeqID), 'r+b')
                fd = f.fileno()
                mm = mmap.mmap(fd, dataLen)
                #mm = mmap_file.read(SHARE_MEMORY_FILE_SIZE_BYTES)
            else:
                mm = mmap.mmap(0, DataWidth*DataHeight*dim*dimSize +
                               1+8+otherSize, 'RflySim3DImg_'+str(SeqID))
            mmList = mmList+[mm]
        # cv2.IMWRITE_PAM_FORMAT_GRAYSCALE
        while True:
            if AutoREG.SIM_END_PRO_ALL_DOWM_KEY == True:
                sys.exit(0)
            for kk in range(len(idxList)):
                mm = mmList[kk]
                idx = idxList[kk]
                DataWidth = self.VisSensor[idx].DataWidth
                DataHeight = self.VisSensor[idx].DataHeight
                typeID = self.VisSensor[idx].TypeID
                dtyp = np.uint8
                dim = 3
                if(typeID == 1):
                    dtyp = np.uint8
                    dim = 3
                elif(typeID == 2):
                    dtyp = np.uint16
                    dim = 1
                elif(typeID == 3):
                    dtyp = np.uint8
                    dim = 1
                elif(typeID == 4 or typeID == 5 or typeID == 6):
                    dtyp = np.int16
                    dim = 3
                for ii in range(3):  # 尝试读取三次内存区域
                    flag = np.frombuffer(mm, dtype=np.uint8, count=1)
                    # print(flag[0])
                    if(flag[0] == 2):  # 图像已写入完成
                        # print(flag[0])
                        mm.seek(0)
                        mm.write_byte(3)  # 进入读取状态

                        # 开始读取图片
                        #L=np.frombuffer(mm,dtype = np.uint8)
                        # struct.unpack('d',L[1:9]) #获得时间戳
                        self.timeStmp[idx] = np.frombuffer(
                            mm, dtype=np.float64, count=1, offset=1)
                        if self.isPrintTime:
                            dTime = time.time()-self.lastIMUTime
                            print('Img', idx, ':',
                                  self.timeStmp[idx], ', dTimeIMU: ', dTime)

                        mm.seek(0)
                        mm.write_byte(4)  # 进入读取完成状态
                        if typeID == 1 or typeID == 2 or typeID == 3:
                            L = np.frombuffer(mm, dtype=dtyp, offset=9)
                            # reshape array to 4 channel image array H X W X 4
                            self.Img[idx] = L.reshape(
                                DataHeight, DataWidth, dim)
                            if len(self.Img[idx]) > 0:
                                self.hasData[idx] = True
                            if self.isRemoteSend:
                                self.sendImgUDPNew(idx)

                        elif typeID == 4 or typeID == 5 or typeID == 6:
                            posAng = np.frombuffer(
                                mm, dtype=np.float32, count=6, offset=9)  # pos ang
                            PointNum = np.frombuffer(
                                mm, dtype=np.int32, count=1, offset=9+4*6)  # num
                            PointNum = PointNum[0]
                            #print('PointNum: ',PointNum)
                            #print('posAng: ', posAng)
                            self.ImgData[idx] = posAng.tolist() + [PointNum]

                            L = np.frombuffer(
                                mm, dtype=dtyp, count=PointNum*dim, offset=9+4*7)  # cloud
                            # reshape array to 4 channel image array H X W X 4
                            self.Img[idx] = L.reshape(PointNum, dim)
                            self.Img[idx] = self.Img[idx] / 32767.0 * \
                                self.VisSensor[idx].otherParams[0]
                            if len(self.Img[idx]) > 0:
                                self.hasData[idx] = True
                            if self.isRemoteSend:
                                # pos ang num cloud
                                L = np.frombuffer(
                                    mm, dtype=np.uint8, count=PointNum*dim*2+4*7, offset=9)
                                self.sendImgBuffer(idx, L.tostring())

                        # 读取到图片后就退出for循环
                        # print("readImg"+str(idx))
                        break
            time.sleep(0.001)

    def startImgCap(self, isRemoteSend=False):
        """ start loop to receive image from UE4,
        isRemoteSend=true will forward image from memory to UDP port
        """
        self.isRemoteSend = isRemoteSend
        memList = []
        udpList = []
        if isEnableRosTrans:
            self.time_record = np.zeros(len(self.VisSensor))
            self.rostime = np.ndarray(len(self.time_record), dtype=rospy.Time)

        for i in range(len(self.VisSensor)):
            self.Img = self.Img + [0]
            self.ImgData = self.ImgData + [0]
            self.hasData = self.hasData + [False]
            self.timeStmp = self.timeStmp + [0]
            IP = str(self.VisSensor[i].SendProtocol[1])+'.'+str(self.VisSensor[i].SendProtocol[2])+'.'+str(
                self.VisSensor[i].SendProtocol[3])+'.'+str(self.VisSensor[i].SendProtocol[4])
            if IP == '0.0.0.0':
                IP = '127.0.0.1'
            if self.RemotSendIP != '':
                IP = self.RemotSendIP
            self.IpList = self.IpList + [IP]
            self.portList = self.portList + [self.VisSensor[i].SendProtocol[5]]
            if self.VisSensor[i].SendProtocol[0] == 0:
                memList = memList + [i]
            else:
                udpList = udpList + [i]

        if len(memList) > 0:
            self.t_menRec = threading.Thread(
                target=self.img_mem_thrd, args=(memList,))
            self.t_menRec.start()

        if len(udpList) > 0:
            #print('Enter UDP capture')
            for i in range(len(udpList)):
                udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                udp.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 60000*100)
                udp.bind(('0.0.0.0', self.portList[udpList[i]]))
                typeID = self.VisSensor[udpList[i]].TypeID
                t_udpRec = threading.Thread(
                    target=self.img_udp_thrdNew, args=(udp, udpList[i], typeID,))
                t_udpRec.start()

    def sendImgUDPNew(self, idx):
        img_encode = cv2.imencode('.png', self.Img[idx])[1]
        data_encode = np.array(img_encode)
        data = data_encode.tostring()
        self.sendImgBuffer(idx, data)

    def sendImgBuffer(self, idx, data):
        imgPackUnit = 60000
        imgLen = len(data)
        imgpackNum = imgLen//imgPackUnit+1
        IP = self.IpList[idx]
        if self.RemotSendIP != '':
            IP = self.RemotSendIP

        CheckSum = 1234567890
        timeStmpSend = self.timeStmp[idx]

        # 循环发送图片码流
        for i in range(imgpackNum):
            dataSend = []
            if imgPackUnit*(i+1) > len(data):  # 末尾包数据提取
                dataSend = data[imgPackUnit*i:]
            else:  # 前面数据包直接提取60000数据
                dataSend = data[imgPackUnit*i:imgPackUnit*(i+1)]
            PackLen = 4*4+8*1+len(dataSend)  # fhead的4i1d长度，加上图片数据长度
            fhead = struct.pack('4i1d', CheckSum, PackLen, i,
                                imgpackNum, timeStmpSend)  # 校验，包长度，包序号，总包数，时间戳
            dataSend = fhead+dataSend  # 包头加上图像数据
            self.udp_socket.sendto(dataSend, (IP, self.portList[idx]))  # 发送出去

    def jsonLoad(self, ChangeMode=-1, jsonPath=''):
        """ load config.json file to create camera list for image capture,
        if ChangeMode>=0, then the SendProtocol[0] will be set to ChangeMode to change the transfer style
        """
        if len(jsonPath) == 0:
            jsonPath = sys.path[0]+'/Config.json'
        else:
            if jsonPath.find(':')==-1:
                jsonPath = sys.path[0]+'/'+jsonPath
        if not os.path.exists(jsonPath):
            print("The json file does not exist!")
            return False
        with open(jsonPath, "r", encoding='utf-8') as f:
            jsData = json.loads(f.read())
            if len(jsData["VisionSensors"]) <= 0:
                print("No sensor data is found!")
                return False
            for i in range(len(jsData["VisionSensors"])):
                visSenStruct = VisionSensorReq()
                if isinstance(jsData["VisionSensors"][i]["SeqID"], int):
                    visSenStruct.SeqID = jsData["VisionSensors"][i]["SeqID"]
                else:
                    print("Json data format is wrong!")
                    continue

                if isinstance(jsData["VisionSensors"][i]["TypeID"], int):
                    visSenStruct.TypeID = jsData["VisionSensors"][i]["TypeID"]
                else:
                    print("Json data format is wrong!")
                    continue

                if isinstance(jsData["VisionSensors"][i]["TargetCopter"], int):
                    visSenStruct.TargetCopter = jsData["VisionSensors"][i]["TargetCopter"]
                else:
                    print("Json data format is wrong!")
                    continue

                if isinstance(jsData["VisionSensors"][i]["TargetMountType"], int):
                    visSenStruct.TargetMountType = jsData["VisionSensors"][i]["TargetMountType"]
                else:
                    print("Json data format is wrong!")
                    continue

                if isinstance(jsData["VisionSensors"][i]["DataWidth"], int):
                    visSenStruct.DataWidth = jsData["VisionSensors"][i]["DataWidth"]
                else:
                    print("Json data format is wrong!")
                    continue

                if isinstance(jsData["VisionSensors"][i]["DataHeight"], int):
                    visSenStruct.DataHeight = jsData["VisionSensors"][i]["DataHeight"]
                else:
                    print("Json data format is wrong!")
                    continue

                if isinstance(jsData["VisionSensors"][i]["DataCheckFreq"], int):
                    visSenStruct.DataCheckFreq = jsData["VisionSensors"][i]["DataCheckFreq"]
                else:
                    print("Json data format is wrong!")
                    continue

                if isinstance(jsData["VisionSensors"][i]["CameraFOV"], float) or isinstance(jsData["VisionSensors"][i]["CameraFOV"], int):
                    visSenStruct.CameraFOV = jsData["VisionSensors"][i]["CameraFOV"]
                else:
                    print("Json data format is wrong!")
                    continue

                if len(jsData["VisionSensors"][i]["SendProtocol"]) == 8:
                    visSenStruct.SendProtocol = jsData["VisionSensors"][i]["SendProtocol"]
                    if ChangeMode != -1:
                        # 如果是远程接收模式，那么读图这里需要配置为UDP接收
                        visSenStruct.SendProtocol[0] = ChangeMode
                else:
                    print("Json data format is wrong!")
                    continue

                if len(jsData["VisionSensors"][i]["SensorPosXYZ"]) == 3:
                    visSenStruct.SensorPosXYZ = jsData["VisionSensors"][i]["SensorPosXYZ"]
                else:
                    print("Json data format is wrong!")
                    continue

                if len(jsData["VisionSensors"][i]["SensorAngEular"]) == 3:
                    visSenStruct.SensorAngEular = jsData["VisionSensors"][i]["SensorAngEular"]
                else:
                    print("Json data format is wrong!")
                    continue

                if len(jsData["VisionSensors"][i]["otherParams"]) == 8:
                    visSenStruct.otherParams = jsData["VisionSensors"][i]["otherParams"]
                else:
                    print("Json data format is wrong!")
                    continue
                self.VisSensor = self.VisSensor+[visSenStruct]
        if(len(self.VisSensor)) <= 0:
            print("No sensor is obtained.")
            return False
        print('Got', len(self.VisSensor), 'vision sensors from json')
        return True

    def initUE4MsgRec(self):
        """ Initialize the UDP data linsening from UE4,
        currently, the crash data is listened
        """
        self.stopFlagUE4 = False
        self.inSilVect = []
        self.inReqVect = []
        MYPORT = 20006
        MYGROUP = '224.0.0.10'
        ANY = '0.0.0.0'
        sock = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.udp_socketUE4.setsockopt(
            socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_socketUE4.bind((ANY, MYPORT))
        status = self.udp_socketUE4.setsockopt(socket.IPPROTO_IP,
                                               socket.IP_ADD_MEMBERSHIP,
                                               socket.inet_aton(MYGROUP) + socket.inet_aton(ANY))
        self.t4 = threading.Thread(target=self.UE4MsgRecLoop, args=())
        self.t4.start()

    def endUE4MsgRec(self):
        """ End UE4 message listening
        """
        self.stopFlagUE4 = True
        time.sleep(0.5)
        self.t4.join()
        self.udp_socketUE4.close()

    def UE4MsgRecLoop(self):
        """ UE4 message listening dead loop
        """
        lastTime = time.time()
        while True:
            if self.stopFlagUE4:
                break
            lastTime = lastTime + 0.01
            sleepTime = lastTime - time.time()
            if sleepTime > 0:
                time.sleep(sleepTime)
            else:
                lastTime = time.time()
            # print(time.time())

            # struct CopterSimCrash {
            # 	int checksum;
            # 	int CopterID;
            # 	int TargetID;
            # }
            while True:
                if self.stopFlagUE4:
                    break

                try:
                    buf, addr = self.udp_socketUE4.recvfrom(65500)
                    #print('Data Received!')
                    if len(buf) == 12:
                        checksum, CopterID, targetID = struct.unpack(
                            'iii', buf[0:12])
                        if checksum == 1234567890:
                            if targetID > -0.5 and CopterID == self.CopterID:
                                self.isVehicleCrash = True
                                self.isVehicleCrashID = targetID
                            print('Vehicle #', CopterID,
                                  ' Crashed with vehicle #', targetID)

                    if len(buf) == 120:
                        iValue = struct.unpack('10i20f', buf[0:120])
                        if iValue[0] == 1234567897:
                            isCopterExist = False
                            for i in range(len(self.inSilVect)):  # 遍历数据列表，飞机ID有没有出现过
                                if self.inSilVect[i].CopterID == iValue[1]:  # 如果出现过，就直接更新数据
                                    isCopterExist = True
                                    self.inSilVect[i].checksum = iValue[0]
                                    self.inSilVect[i].inSILInts = iValue[2:10]
                                    self.inSilVect[i].inSILFLoats = iValue[10:30]
                                    break
                            if not isCopterExist:  # 如果没有出现过，就创建一个结构体
                                vsr = PX4SILIntFloat(iValue)
                                self.inSilVect = self.inSilVect + \
                                    [copy.deepcopy(vsr)]  # 扩充列表，增加一个元素

                    if len(buf) == 160:

                        isCopterExist = False
                        iValue = struct.unpack('4i1d29f20s', buf[0:160])
                        if(iValue[0] == 1234567897):
                            vsr = reqVeCrashData(iValue)
                            # print(vsr.copterID,vsr.vehicleType)
                            for i in range(len(self.inReqVect)):  # 遍历数据列表，飞机ID有没有出现过
                                if self.inReqVect[i].copterID == iValue[1]:  # 如果出现过，就直接更新数据
                                    isCopterExist = True
                                    self.inReqVect[i] = copy.deepcopy(vsr)
                                    self.inReqUpdateVect[i] = True
                                    break
                            if not isCopterExist:  # 如果没有出现过，就创建一个结构体
                                self.inReqVect = self.inReqVect + \
                                    [copy.deepcopy(vsr)]  # 扩充列表，增加一个元素
                                self.inReqUpdateVect = self.inReqUpdateVect + \
                                    [True]

                except:
                    self.stopFlagUE4 = True
                    print('Error Data')

                    break

    def getUE4Pos(self, CopterID=1):
        if self.stopFlagUE4:  # 如果没有启用监听程序
            self.initUE4MsgRec()
            time.sleep(1)

        for i in range(len(self.inReqVect)):  # 遍历数据列表，飞机ID有没有出现过
            if self.inReqVect[i].copterID == CopterID:
                posE = self.inReqVect[i].PosE
                return [posE[0], posE[1], posE[2], 1]
        return [0, 0, 0, 0]

    def getUE4Data(self, CopterID=1):
        if self.stopFlagUE4:  # 如果没有启用监听程序
            self.initUE4MsgRec()
            time.sleep(1)

        for i in range(len(self.inReqVect)):  # 遍历数据列表，飞机ID有没有出现过
            if self.inReqVect[i].copterID == CopterID:
                return self.inReqVect[i]
        return 0
