改动：
1、使用了封装方法，将故障注入模块形成了一个总的库slx文件，通过修改库FaultModelLib的模块即能同步到模型。
2、使用了私有参数（提前定义好的，如故障ID等）和公共参数（主要是InParam参数，时Ints和Floats参数无法涉及到的参数）

3、此文件中的GenerateModelDLLFile.p支持将模型真值输出。

4、如果遇到初始化未变成Offboard模式，重新插拔飞控





MotorFaultTemp.FaultID=123450;

MotorFaultTemp,NoiseFaultID=111111;

MotorFaultTemp.MotorNum=int32(4);

% Motor Fault Struct
MotorFault1=MotorFaultTemp;

%Prop Fault Stuct
PropFault.FaultID = 123451;
PropFault.PropNum = int32(4);

%Battery Fault Struct
BatteryFault.PowOffFaultID = 123452;

BatteryFault.LowVoltagefaultID = 123453;

BatteryFault.LowCapacityFaultID = 123454;

%Load Fault Struct
LoadFault.LoadFal1FaultID = 123455;

LoadFault.LoadShiftFaultID = 123456;

LoadFault.LoadLeakFaultID = 123457;

%Wind Fault struct
WindFault.ConstwindFaultID = 123458;

WindFault.GustwindFaultID = 123459;

WindFault.TurbwindFaultID = 123540;

WindFault.SheerWindFaultID = 123541;

%Sensor Fault Struct
SensorFault.AccNoiseFaultID = 123542;

SensorFault.GyroNoiseFaultID = 123543;

SensorFault.MagNoiseFaultID = 123544;

SensorFault.BaroNoiseFaultID = 123545

SensorFault.GPSNoiseFaultID = 123546;