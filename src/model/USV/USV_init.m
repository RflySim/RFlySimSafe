%InitFunction
load MavLinkStruct;
%Initial condition

% Define the 32-D ModelInParams vector for external modification
FaultParamAPI.FaultInParams = zeros(32,1);

MotorFaultTemp.FaultID=123450;
MotorFaultTemp.MotorNum=int32(2);

% Motor Fault Struct
MotorFault1=MotorFaultTemp;
%ESC Fault Stuct
ESCFault.FaultID = 123451;
ESCFault.ESCNum = int32(4);
% %Battery Fault Struct
% BatteryFault.UseCusTomHoverTimeFaultID = 123452;
% BatteryFault.PowOffFaultID = 123453;
% BatteryFault.LowVoltageFaultID = 123454;
% BatteryFault.LowCapacityFaultID = 123455;
% %Load Fault Struct
% LoadFault.LoadFallFaultID = 123456;
% LoadFault.LoadShiftFaultID = 123457;
% LoadFault.LoadLeakFaultID = 123458;
%Wind Fault Struct
WindFault.ConstWindFaultID = 123459;
WindFault.GustWindFaultID = 123540;
WindFault.TurbWindFaultID = 123541;
WindFault.SheerWindFaultID = 123542;
WindFault.NoiseWindFaultID = 123543;
%Sensor Fault Struct
SensorFault.AccNoiseFaultID = 123544;
SensorFault.GyroNoiseFaultID = 123545;
SensorFault.MagNoiseFaultID = 123546;
SensorFault.BaroNoiseFaultID = 123547;
SensorFault.GPSNoiseFaultID = 123548;

FaultParamAPI.FaultInParams(1)=1; % Band-Limited White Noise,FaultID = 123450
FaultParamAPI.FaultInParams(2)=2;
FaultParamAPI.FaultInParams(3)=2;
FaultParamAPI.FaultInParams(4)=3; %Wind speed at 6 m defines the low-altitude intensity (m/s),FaultID = 123541
FaultParamAPI.FaultInParams(5)=1; %Wind direction at 6 m (degrees clockwise from north)��FaultID = 123541
FaultParamAPI.FaultInParams(6)=3; %Wind speed at 6 m altitude (m/s)��FaultID = 123542
FaultParamAPI.FaultInParams(7)=1; %Wind direction at 6 m altitude (degrees clockwise from north)��FaultID = 123542
FaultParamAPI.FaultInParams(8)=2; %Wind speed at 6 m defines the low-altitude intensity (m/s)��FaultID = 123540
FaultParamAPI.FaultInParams(9)=1; %Wind direction at 6 m (degrees clockwise from north)��FaultID = 123540
FaultParamAPI.FaultInParams(10)=0.707; %Accelerometer damping ratio
FaultParamAPI.FaultInParams(11)=0; % Accelerometer measurement bias-x
FaultParamAPI.FaultInParams(12)=0; % Accelerometer measurement bias-y
FaultParamAPI.FaultInParams(13)=0; % Accelerometer measurement bias-z
FaultParamAPI.FaultInParams(14)=0.707; % Gyro damping ratio
FaultParamAPI.FaultInParams(15)=0; % Gyro measurement bias-x
FaultParamAPI.FaultInParams(16)=0; % Gyro measurement bias-y
FaultParamAPI.FaultInParams(17)=0; % Gyro measurement bias-z


% �ɻ��ĳ�ʼλ��
ModelInit_PosE=[0,0,0];
ModelInit_VelB=[0,0,0];
ModelInit_AngEuler=[0,0,0];
ModelInit_RateB=[0,0,0];
ModelInit_Inputs = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];

%Model Parameter
% USV TYPE.
ModelParam_uavType = int16(608); %���������˴�������άģ�͵�ClassID��Ӧ 608��609���ֱ��������û��
ModelParam_uavMotNumbs = int8(2); % �������

ModelParam_uavMass=227;
ModelParam_uavJxx =181.42; %ת������
ModelParam_uavJyy = 408.203;
ModelParam_uavJzz = 495.037;
ModelParam_uavJ= [ModelParam_uavJxx,0,0;0,ModelParam_uavJyy,0;0,0,ModelParam_uavJzz];
ModelPara_coborigin=[0;0;0];


BoatParam_waterDensity = 1000; %ˮ���ܶȣ�����Ϊ��
BoatParam_cf = 0.1; % ���ĵ�λ��
BoatParam_cg =0.1; % ����λ��
BoatParam_width = 0.8; %���Ŀ�ȣ���ˮ�洦��
BoatParam_widSlope=pi/4; % ������ص�б�ʣ���ˮ�洦��
BoatParam_len = 1.5; %���ĳ��ȣ���ˮ�洦��
BoatParam_lenSlope=pi/6; % ������ص�б�ʣ���ˮ�洦��


ModelParam_uavCd = [2 200 200];
ModelParam_uavCCm = [1 1 1]*200;
ModelParam_uavDearo = -0;%%unit m


% �������ģ��
ESCTmp.isEnable=false; %�Ƿ����ñ������Ĭ�ϲ�����
ESCTmp.input_offset = 0;
ESCTmp.input_scaling = 100;
ESCTmp.zero_position_disarmed = 0;
ESCTmp.zero_position_armed =0;
ESCTmp.joint_control_type=0; %0:velocity, 1:position,...

ESC=[];%��������ݽṹ��
for i=1:8 %�������ģ�����֧��8�����
    if i<=ModelParam_uavMotNumbs %������Ļ���������4
        tmp=ESCTmp; %����һ��ģ��
        tmp.isEnable=true;%���õ��ģ��
        ESC=[ESC,tmp];%����ά��
    else
        ESC=[ESC,ESCTmp]; %ֱ����Ĭ��ģ����䣬�����õ��ģ��
    end
end


% ���ͨ��ģ��
motorTmp.isEnable=false; %�Ƿ����ñ������Ĭ�ϲ�����
motorTmp.input_index = 1;%û���ã���ûɾ
motorTmp.pose=[-2.65193 1.02713 0 0 1.57079 0];%û���ã���ûɾ

motorTmp.reversible=0;                        
motorTmp.turningDirection=-1; %���������ת��˳��CW����-1 �棨CCW����1
motorTmp.timeConstantUp = 0.0125; %�˲������ϵ��
motorTmp.timeConstantDown = 0.025; %�˲������ϵ��
motorTmp.maxRotVelocity = 1100; %������ת��
motorTmp.motorConstant = 4.54858e-02; %��ﳣ��
motorTmp.momentConstant = 0.01; %�㶨����
motorTmp.rotorDragCoefficient = 0.000806428; %ת����קϵ��
motorTmp.rollingMomentCoefficient = 1e-06; %��������ϵ��
motorTmp.rotorVelocitySlowdownSim=10; %ת�Ӽ���ϵ��

pose = [... %ǰ��λ�����ƽ������������λ�ã�����λ��ʾŷ����-0.2
[-2.65193 1.02713 -0.06 0 1.57079 0];...
[-2.65193 -1.02713 -0.06 0 1.57079 0];...
];

% ���ת��  ˳��CW����-1  �棨CCW����1
turningDirection = [1; 1];

motor=[];%��������ݽṹ��
for i=1:8 %�������ģ�����֧��8�����
    if i<=ModelParam_uavMotNumbs %������Ļ���������4
        tmp=motorTmp; %����һ��ģ��
        tmp.isEnable=true;%���õ��ģ��
        tmp.pose=pose(i,:); %���õ��λ��
        tmp.reversible=1;
        tmp.turningDirection=turningDirection(i); %���õ��ת��
        motor=[motor,tmp];%����ά��
    else
        motor=[motor,motorTmp]; %ֱ����Ĭ��ģ����䣬�����õ��ģ��
    end
end
% USV��������ṹ��      
USVPara.Pose_futong_left = [-0.4; 1.03; 0.2];   
USVPara.Pose_futong_right = [-0.4; -1.03; 0.2];
USVPara.zw=562.1;
USVPara.HullRadius=0.213;
USVPara.z_cb=0.2;%ԲͲ����ڴ�������ĵĸ߶�
USVPara.BoatLength=4.9;
USVPara.BoatWidth=2.4;
USVPara.LengthN=1;
USVPara.WaterDensity=997.8;
USVPara.Ma=[...
    [5 0 0 0 0 0 ];... 
    [0 5 0 0 0 0 ];...    
    [0 0 0.1 0 0 0 ];... 
    [0 0 0 0.1 0 0 ];... 
    [0 0 0 0 0.1 0 ];... 
    [0 0 0 0 0 1 ];... 
];
USVPara.Dmat=[...
    [30 0 0 0 0 0 ];... 
    [0 300 0 0 0 0 ];...    
    [0 0 300 0 0 0 ];... 
    [0 0 0 600 0 0 ];... 
    [0 0 0 0 600 0 ];... 
    [0 0 0 0 0 300 ];... 
];

%-----------------------------------------
%ModelParam_uavCtrlEn = int8(0);
%ModelParam_ControlMode = int8(1); %���� 1��ʾAutoģʽ��0��ʾManualģʽ
ModelParam_motorMinThr=0.05;
ModelParam_motorCr=842.1;
ModelParam_motorWb=22.83;
ModelParam_motorT= 0.0214;%0.0261;
ModelParam_motorJm =0.0001287;
ModelParam_rotorCm=2.783e-07;
ModelParam_rotorCt=1.681e-05;

ModelParam_noisePowerIMU=0.0003394;
ModelParam_noisePowerMag=0.0004;
ModelParam_timeSampBaro = 0.02;
ModelParam_noiseSampleTimeMag = 0.01;



ModelParam_GlobalNoiseGainSwitch =0.4;

%Environment Parameter
ModelParam_envLongitude = 116.259368300000;
ModelParam_envLatitude = 40.1540302;
ModelParam_GPSLatLong = [ModelParam_envLatitude ModelParam_envLongitude];
ModelParam_envAltitude = -600;     %�ο��߶ȣ���ֵ


%SimParam.timeACC=1;
%SimParam.timeStep=0.0001;
% SimParam.sonarSamp=0.1;
% SimParam.AngEularSamp=0.004;
% SimParam.AngRateSamp=0.004;
% SimParam.AccSensSamp=0.004;
% SimParam.GyroSensSamp=0.004;
% SimParam.AngQuaternSamp=0.004;
% SimParam.BaroSamp=0.008;
% SimParam.AngEulerSamp=0.004;
%ModelParam_timeSampBaro = 0.01;             %��ע�͵���
ModelParam_timeSampTurbWind = 0.01;
ModelParam_BusSampleRate = 0.001;



%%%ModelParam_BattModelEnable=int8(0);
%ModelParam_BattAuxCurrent=0.5;
%ModelParam_BattCells=3;
%ModelParam_BattCapacity=0.55;   %%��һ���ģ�����ý��洫�����
ModelParam_BattHoverMinutes=18;
ModelParam_BattHoverThr=0.609;

%GPS Parameter
ModelParam_GPSEphFinal=0.3;
ModelParam_GPSEpvFinal=0.4;
ModelParam_GPSFix3DFix=3;
ModelParam_GPSSatsVisible=10;


%Noise Parameter
ModelParam_noisePowerAccel = [0.001,0.001,0.003];%˳�� xyz ��ͬ  ��Ҫ�޸�����
ModelParam_noiseSampleTimeAccel = 0.001;
%ModelParam_noiseLowPassFilterCoeAccel = 0.0001;
ModelParam_noisePowerOffGainAccel = 0.04;
ModelParam_noisePowerOffGainAccelZ = 0.03;
ModelParam_noisePowerOnGainAccel = 0.8;
ModelParam_noisePowerOnGainAccelZ = 4.5;


ModelParam_noisePowerGyro = [0.00001,0.00001,0.00001];%��Ҫ�޸�����
ModelParam_noiseSampleTimeGyro = 0.001;
%ModelParam_noiseLowPassFilterCoeGyro = 0.0001;
ModelParam_noisePowerOffGainGyro = 0.02;
ModelParam_noisePowerOffGainGyroZ = 0.025;
ModelParam_noisePowerOnGainGyro = 2;%3.2;
ModelParam_noisePowerOnGainGyroZ = 1;



%ModelParam_noisePowerMag = [0.00001,0.00001,0.00001];%��Ҫ�޸�����  ��ע�͵���
%ModelParam_noiseSampleTimeMag = 0.01;          ��ע�͵���
%ModelParam_noiseLowPassFilterCoeMag = 0.02;%��ʱû��ʹ��
ModelParam_noisePowerOffGainMag = 0.02;
ModelParam_noisePowerOffGainMagZ = 0.035;
ModelParam_noisePowerOnGainMag = 0.025;
ModelParam_noisePowerOnGainMagZ = 0.05;



%ModelParam_noisePowerIMU=0;%IMU�����������ǰ������������Ǿ�����һ��  ��ע�͵���

ModelParam_noiseUpperGPS=0.5;  %GPS��λ�������������������������x,y,z�Ĳ������ޣ���λ��m
ModelParam_noiseGPSSampTime=0.2;%Ĭ��0.05

ModelParam_noiseUpperBaro=0; %��ѹ������������������������߶ȵĲ������ޣ���λ��m
ModelParam_noiseBaroSampTime=0.5;%��ѹ����������Ƶ�ʣ���Ĭ��0.05
ModelParam_noiseBaroCoupleWithSpeed=0;%��ѹ�Ʋ����߶��붯ѹ��ϵ��Ҳ���Ƿ�������ѹ�Ƶ���ģ�͵�ϵ������ǰ����0.008�ɻ�10m/s����1m

ModelParam_noiseUpperWindBodyRatio=0;%�粨��ϵ��������*(1+��ϵ��)
ModelParam_noiseWindSampTime=0.001;


%%ModelParam_envGravityAcc = 9.81;
ModelParam_envAirDensity = 1.225;    %��û���õ�
ModelParam_envDiffPressure = 0; % Differential pressure (airspeed) in millibar
ModelParam_noiseTs = 0.001;

%ModelParam_FailModelStartT = 5;
%ModelParam_FailModelLastT = 5;