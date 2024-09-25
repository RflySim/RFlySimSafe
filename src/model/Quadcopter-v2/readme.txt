MotorFault1=MotorFaultTemp;
PropFault.FaultID = 123451;
PropFault.PropNum = int32(4);
%Battery Fault Struct
BatteryFault.UseCusTomHoverTimeFaultID = 123452;
BatteryFault.PowOffFaultID = 123453;
BatteryFault.LowVoltageFaultID = 123454;
BatteryFault.LowCapacityFaultID = 123455;
%Load Fault Struct
LoadFault.LoadFallFaultID = 123456;
LoadFault.LoadShiftFaultID = 123457;
LoadFault.LoadLeakFaultID = 123458;
%Wind Fault Struct
WindFault.ConstWindFaultID = 123459;
WindFault.GustWindFaultID = 123540;
WindFault.TurbWindFaultID = 123541;
WindFault.SheerWindFaultID = 123542;
WindFault.NoiseWindFaultID = 123543;
%Sensor Fault Struct
SensorFault.AccFaultID = 123544;
SensorFault.GyroFaultID = 123545;
SensorFault.MagFaultID = 123546;
SensorFault.BaroFaultID = 123547;
SensorFault.GPSFaultID = 123548;


FaultParamAPI.FaultInParams(4)=3; %Turb Wind speed at 6 m defines the low-altitude intensity (m/s),FaultID = 123541
FaultParamAPI.FaultInParams(5)=1; %Turb Wind direction at 6 m (degrees clockwise from north)，FaultID = 123541
FaultParamAPI.FaultInParams(6)=3; %Sheer Wind speed at 6 m altitude (m/s)，FaultID = 123542
FaultParamAPI.FaultInParams(7)=1; %Sheer Wind direction at 6 m altitude (degrees clockwise from north)，FaultID = 123542
FaultParamAPI.FaultInParams(8)=2; %Gust Wind speed at 6 m defines the low-altitude intensity (m/s)，FaultID = 123540
FaultParamAPI.FaultInParams(9)=1; %Gust Wind direction at 6 m (degrees clockwise from north)，FaultID = 123540


FaultParamAPI.FaultInParams(10)=0.9; %Accelerometer damping ratio
FaultParamAPI.FaultInParams(11)=0; % Accelerometer measurement bias-x
FaultParamAPI.FaultInParams(12)=0; % Accelerometer measurement bias-y
FaultParamAPI.FaultInParams(13)=0; % Accelerometer measurement bias-z
FaultParamAPI.FaultInParams(14)=0.707; % Gyro damping ratio
FaultParamAPI.FaultInParams(15)=0; % Gyro measurement bias-x
FaultParamAPI.FaultInParams(16)=0; % Gyro measurement bias-y
FaultParamAPI.FaultInParams(17)=0; % Gyro measurement bias-z

参数为-1时表示继承原来的正常输出

电机故障	
FaultParams(1)为flag
flags:
   1)1:覆盖模式，直接将输出值替换成故障注入中的值，参数为-1时表示继承原来的正常输出
   2)2:叠加模式，直接将输出值替换成故障注入中的值和传感器自身值的和，参数为-1时表示继承原来的正常输出
   3)3:噪声模式，FaultParams(2)、FaultParams(3)分别为均值和方差，参数为-1时表示继承原来的正常输出
   4)4:改变尺度因子，参数为-1时表示继承原来的正常输出

GPS故障	

flags:
   1)1:覆盖模式，直接将输出值替换成故障注入中的值，参数为-1时表示继承原来的正常输出
   2)2:叠加模式，直接将输出值替换成故障注入中的值和传感器自身值的和，参数为-1时表示继承原来的正常输出
   3)3:噪声模式，FaultParams(2)、FaultParams(3)分别为均值和方差，参数为-1时表示继承原来的正常输出
   4)4:改变尺度因子，参数为-1时表示继承原来的正常输出
   5)5:改变搜星数
   6)6:改变fix_type
"
加速度计 	

flags:
   1)1:覆盖模式，直接将输出值替换成故障注入中的值，参数为-1时表示继承原来的正常输出
   2)2:叠加模式，直接将输出值替换成故障注入中的值和传感器自身值的和，参数为-1时表示继承原来的正常输出
   3)3:噪声模式，FaultParams(2)、FaultParams(3)分别为均值和方差，参数为-1时表示继承原来的正常输出
   4)4:改变尺度因子，参数为-1时表示继承原来的正常输出


气压计	

flags:
   1)1:覆盖模式，直接将输出值替换成故障注入中的值，参数为-1时表示继承原来的正常输出
   2)2:叠加模式，直接将输出值替换成故障注入中的值和传感器自身值的和，参数为-1时表示继承原来的正常输出
   3)3:噪声模式，FaultParams(2)、FaultParams(3)分别为均值和方差，参数为-1时表示继承原来的正常输出
   4)4:改变尺度因子，参数为-1时表示继承原来的正常输出


磁力计	

flags:
   1)1:覆盖模式，直接将输出值替换成故障注入中的值，参数为-1时表示继承原来的正常输出
   2)2:叠加模式，直接将输出值替换成故障注入中的值和传感器自身值的和，参数为-1时表示继承原来的正常输出
   3)3:噪声模式，FaultParams(2)、FaultParams(3)分别为均值和方差，参数为-1时表示继承原来的正常输出
   4)4:改变尺度因子，参数为-1时表示继承原来的正常输出

陀螺仪	

flags:
   1)1:覆盖模式，直接将输出值替换成故障注入中的值，参数为-1时表示继承原来的正常输出
   2)2:叠加模式，直接将输出值替换成故障注入中的值和传感器自身值的和，参数为-1时表示继承原来的正常输出
   3)3:噪声模式，FaultParams(2)、FaultParams(3)分别为均值和方差，参数为-1时表示继承原来的正常输出
   4)4:改变尺度因子，参数为-1时表示继承原来的正常输出
















