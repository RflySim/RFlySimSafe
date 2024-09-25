//
// File: QuadModelv.cpp
//
// Code generated for Simulink model 'QuadModelv'.
//
// Model version                  : 12.1
// Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
// C/C++ source code generated on : Thu Jun 13 16:02:48 2024
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "QuadModelv.h"
#include "rtwtypes.h"
#include <cmath>
#include <cstring>
#include <cfloat>
#include <stddef.h>
#define NumBitsPerChar                 8U

// Private macros used by the generated code to access rtModel
#ifndef rtmSetFirstInitCond
#define rtmSetFirstInitCond(rtm, val)  ((rtm)->Timing.firstInitCondFlag = (val))
#endif

#ifndef rtmIsFirstInitCond
#define rtmIsFirstInitCond(rtm)        ((rtm)->Timing.firstInitCondFlag)
#endif

#ifndef rtmIsMajorTimeStep
#define rtmIsMajorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
#define rtmIsMinorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTPtr
#define rtmSetTPtr(rtm, val)           ((rtm)->Timing.t = (val))
#endif

#ifndef GRAVITY2_TYPEDEF

typedef enum { WGS84TAYLORSERIES = 1, WGS84CLOSEAPPROX,
  WGS84EXACT } GravityTypeIdx;

typedef enum { METRIC = 1, ENGLISH } UnitIdx;

typedef enum { JANUARY = 1, FEBRUARY, MARCH, APRIL, MAY, JUNE, JULY,
  AUGUST, SEPTEMBER, OCTOBER, NOVEMBER, DECEMBER } MonthIdx;

#define GRAVITY2_TYPEDEF
#endif                                 // GRAVITY2_TYPEDEF

#ifndef WGS84_DEFINE
#define WGS84_A                        6378137.0                 // Semi-major Axis (m) 
#define WGS84_INV_F                    298.257223563             // Reciprocal of Flattening 
#define WGS84_W_DEF                    7292115.0e-11             // WGS 84 Angular Velocity of Earth (rad/sec)
#define WGS84_GM_DEF                   3986004.418e+8            // Earth's Gravitational Const. (m^3/sec^2) 
#define WGS84_GM_PRM                   3986000.9e+8              // Earth's Grav. Const. (m^3/sec^2) [no atmos]
#define WGS84_W_PRM                    7292115.1467e-11          // IAU Angular Velocity of Earth (rad/sec) 
#define WGS84_G_E                      9.7803253359              // Theoretical (Normal) Gravity at the Equator
                                                                 // (on the Ellipsoid) (m/s^2) 
#define WGS84_K                        0.00193185265241          // Theoretical (Normal) Gravity Formula Const.
#define WGS84_E_2                      6.69437999014e-3          // First Eccentricity Squared 
#define WGS84_EL                       5.2185400842339e+5        // Linear Eccentricity 
#define WGS84_B                        6356752.3142              // Semi-minor Axis (m) 
#define WGS84_B_A                      0.996647189335            // Axis Ratio
#define M2FT                           1.0/0.3048
#define AERO_PI                        3.14159265358979323846
#define DEG2RAD                        AERO_PI/180.0
#define YEAR2000                       2000
#define WGS84_DEFINE
#endif                                 // WGS84_DEFINE

// Block parameters (default storage)
P_QuadModelv_T MulticopterModelClass::QuadModelv_P = {
  // Variable: FaultParamAPI
  //  Referenced by:
  //    '<S510>/Constant'
  //    '<S510>/Gain'
  //    '<S94>/Wind direction'
  //    '<S94>/Windspeed at 20ft (6m)'
  //    '<S95>/Wind direction'
  //    '<S95>/Windspeed at 20ft (6m)'
  //    '<S105>/Wind Direction'
  //    '<S105>/Wind speed at reference height'
  //    '<S512>/Gust start time'
  //    '<S421>/2*zeta * wn'
  //    '<S432>/2*zeta * wn'

  {
    { 0.0, 0.0, 0.0, 3.0, 1.0, 3.0, 1.0, 2.0, 1.0, 0.9, 0.0, 0.0, 0.0, 0.707,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0 }
  },

  // Variable: PropFault
  //  Referenced by: '<S510>/FaultID1'

  {
    123451.0,
    4
  },

  // Variable: MotorFault1
  //  Referenced by: '<S293>/MotorNum'

  {
    123450.0,
    4
  },

  // Variable: ModelFailEnable
  //  Referenced by: '<S12>/ModelFailEnable'

  0.0,

  // Variable: ModelInit_AngEuler
  //  Referenced by: '<S15>/Initial Euler Angles'

  { 0.0, 0.0, 0.0 },

  // Variable: ModelInit_PosE
  //  Referenced by: '<S14>/xe,ye,ze'

  { 0.0, 0.0, 0.0 },

  // Variable: ModelInit_RPM
  //  Referenced by: '<S294>/ModelInit_RPM'

  0.0,

  // Variable: ModelInit_RateB
  //  Referenced by: '<S14>/p,q,r '

  { 0.0, 0.0, 0.0 },

  // Variable: ModelInit_VelB
  //  Referenced by: '<S14>/ub,vb,wb'

  { 0.0, 0.0, 0.0 },

  // Variable: ModelParam_BattHoverMinutes
  //  Referenced by: '<S2>/ModelParam.BattHoverMinutes'

  18.0,

  // Variable: ModelParam_BattHoverThr
  //  Referenced by: '<S2>/ModelParam.BattHoverThr'

  0.609,

  // Variable: ModelParam_GPSEphFinal
  //  Referenced by: '<S322>/ModelParam.GPSEphFinal'

  0.3,

  // Variable: ModelParam_GPSEpvFinal
  //  Referenced by: '<S322>/ModelParam.GPSEpvFinal'

  0.4,

  // Variable: ModelParam_GPSLatLong
  //  Referenced by:
  //    '<S63>/ref_position'
  //    '<S371>/ref_position'

  { 40.1540302, 116.2593683 },

  // Variable: ModelParam_envAltitude
  //  Referenced by:
  //    '<S5>/ModelParam.envAltitude'
  //    '<S366>/ModelParam.envAltitude'

  -50.0,

  // Variable: ModelParam_motorCr
  //  Referenced by: '<S294>/ModelParam.motorCr'

  842.1,

  // Variable: ModelParam_motorJm
  //  Referenced by: '<S7>/ModelParam.motorJm'

  0.0001287,

  // Variable: ModelParam_motorMinThr
  //  Referenced by:
  //    '<S305>/Constant'
  //    '<S307>/Constant'
  //    '<S309>/Constant'
  //    '<S311>/Constant'
  //    '<S313>/Constant'
  //    '<S315>/Constant'
  //    '<S317>/Constant'
  //    '<S319>/Constant'

  0.05,

  // Variable: ModelParam_motorT
  //  Referenced by: '<S294>/ModelParam.motorT'

  0.0214,

  // Variable: ModelParam_motorWb
  //  Referenced by: '<S294>/ModelParam.motorWb'

  22.83,

  // Variable: ModelParam_noisePowerOffGainAccel
  //  Referenced by: '<S394>/AccelNoiseGainFunction'

  0.04,

  // Variable: ModelParam_noisePowerOffGainAccelZ
  //  Referenced by: '<S394>/AccelNoiseGainFunction'

  0.03,

  // Variable: ModelParam_noisePowerOffGainGyro
  //  Referenced by: '<S394>/GyroNoiseGainFunction'

  0.02,

  // Variable: ModelParam_noisePowerOffGainGyroZ
  //  Referenced by: '<S394>/GyroNoiseGainFunction'

  0.025,

  // Variable: ModelParam_noisePowerOffGainMag
  //  Referenced by: '<S394>/MagNoiseGainFunction'

  0.02,

  // Variable: ModelParam_noisePowerOffGainMagZ
  //  Referenced by: '<S394>/MagNoiseGainFunction'

  0.035,

  // Variable: ModelParam_noisePowerOnGainAccel
  //  Referenced by: '<S394>/AccelNoiseGainFunction'

  0.8,

  // Variable: ModelParam_noisePowerOnGainAccelZ
  //  Referenced by: '<S394>/AccelNoiseGainFunction'

  4.5,

  // Variable: ModelParam_noisePowerOnGainGyro
  //  Referenced by: '<S394>/GyroNoiseGainFunction'

  2.0,

  // Variable: ModelParam_noisePowerOnGainGyroZ
  //  Referenced by: '<S394>/GyroNoiseGainFunction'

  1.0,

  // Variable: ModelParam_noisePowerOnGainMag
  //  Referenced by: '<S394>/MagNoiseGainFunction'

  0.025,

  // Variable: ModelParam_noisePowerOnGainMagZ
  //  Referenced by: '<S394>/MagNoiseGainFunction'

  0.05,

  // Variable: ModelParam_rotorCm
  //  Referenced by: '<S7>/ModelParam.rotorCm'

  2.783E-7,

  // Variable: ModelParam_rotorCt
  //  Referenced by: '<S7>/ModelParam.rotorCt'

  1.681E-5,

  // Variable: ModelParam_uavCCm
  //  Referenced by: '<S7>/ModelParam.uavCCm'

  { 0.0035, 0.0039, 0.0034 },

  // Variable: ModelParam_uavCd
  //  Referenced by: '<S7>/ModelParam.uavCd'

  0.055,

  // Variable: ModelParam_uavDearo
  //  Referenced by: '<S7>/ModelParam.uavDearo'

  0.12,

  // Variable: ModelParam_uavJ
  //  Referenced by: '<S6>/uavJ'

  { 0.0211, 0.0, 0.0, 0.0, 0.0219, 0.0, 0.0, 0.0, 0.0366 },

  // Variable: ModelParam_uavMass
  //  Referenced by: '<S6>/ModelParam.uavMass'

  1.515,

  // Variable: ModelParam_uavR
  //  Referenced by: '<S6>/ModelParam.uavR'

  0.225,

  // Variable: ModelParam_uavMotNumbs
  //  Referenced by: '<S2>/ModelParam.uavMotNumbs'

  4,

  // Variable: ModelParam_uavType
  //  Referenced by:
  //    '<S7>/ModelParam.uavType'
  //    '<S12>/UAVType'

  3,

  // Mask Parameter: BandLimitedWhiteNoise_Cov
  //  Referenced by: '<S511>/Output'

  1.0,

  // Mask Parameter: DrydenWindTurbulenceModelDiscre
  //  Referenced by: '<S183>/Medium//High Altitude'

  533.4,

  // Mask Parameter: DrydenWindTurbulenceModelDisc_m
  //  Referenced by: '<S144>/Medium//High Altitude'

  533.4,

  // Mask Parameter: DrydenWindTurbulenceModelDisc_n
  //  Referenced by: '<S157>/White Noise'

  { 23301.0, 23542.0, 82443.0, 33344.0 },

  // Mask Parameter: DrydenWindTurbulenceModelDisc_i
  //  Referenced by: '<S118>/White Noise'

  { 23341.0, 23342.0, 23343.0, 23344.0 },

  // Mask Parameter: DrydenWindTurbulenceModelDisc_g
  //  Referenced by:
  //    '<S147>/Constant1'
  //    '<S147>/Constant2'
  //    '<S147>/Constant3'
  //    '<S148>/Constant3'

  1.0,

  // Mask Parameter: DrydenWindTurbulenceModelDisc_k
  //  Referenced by:
  //    '<S108>/Constant1'
  //    '<S108>/Constant2'
  //    '<S108>/Constant3'
  //    '<S109>/Constant3'

  1.0,

  // Mask Parameter: WhiteNoise_Ts
  //  Referenced by: '<S157>/Constant1'

  0.01,

  // Mask Parameter: WhiteNoise_Ts_a
  //  Referenced by: '<S118>/Constant1'

  0.01,

  // Mask Parameter: DrydenWindTurbulenceModelDis_n3
  //  Referenced by: '<S164>/Probability of  Exceedance'

  3.0,

  // Mask Parameter: DrydenWindTurbulenceModelDis_gr
  //  Referenced by: '<S125>/Probability of  Exceedance'

  4.0,

  // Mask Parameter: DrydenWindTurbulenceModelDis_np
  //  Referenced by: '<S94>/Wingspan'

  1.0,

  // Mask Parameter: DrydenWindTurbulenceModelDisc_d
  //  Referenced by: '<S95>/Wingspan'

  1.0,

  // Mask Parameter: ThreeaxisInertialMeasurementUni
  //  Referenced by: '<S415>/Measurement bias'

  { 0.0, 0.0, 0.0 },

  // Mask Parameter: ThreeaxisInertialMeasurementU_h
  //  Referenced by: '<S415>/Scale factors & Cross-coupling  errors'

  { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 },

  // Mask Parameter: DirectionCosineMatrixtoQuaterni
  //  Referenced by:
  //    '<S203>/If Warning//Error'
  //    '<S203>/Constant'

  1.0,

  // Mask Parameter: DirectionCosineMatrixtoQuater_p
  //  Referenced by:
  //    '<S241>/If Warning//Error'
  //    '<S241>/Constant'

  1.0,

  // Mask Parameter: DirectionCosineMatrixtoQuater_c
  //  Referenced by:
  //    '<S329>/If Warning//Error'
  //    '<S329>/Constant'

  1.0,

  // Mask Parameter: CompareToConstant_const
  //  Referenced by: '<S82>/Constant'

  180.0,

  // Mask Parameter: CompareToConstant_const_i
  //  Referenced by: '<S80>/Constant'

  90.0,

  // Mask Parameter: CompareToConstant_const_j
  //  Referenced by: '<S83>/Constant'

  180.0,

  // Mask Parameter: CompareToConstant_const_ji
  //  Referenced by: '<S76>/Constant'

  180.0,

  // Mask Parameter: CompareToConstant_const_k
  //  Referenced by: '<S74>/Constant'

  90.0,

  // Mask Parameter: CompareToConstant_const_f
  //  Referenced by: '<S77>/Constant'

  180.0,

  // Mask Parameter: CompareToConstant_const_n
  //  Referenced by: '<S448>/Constant'

  180.0,

  // Mask Parameter: CompareToConstant_const_a
  //  Referenced by: '<S446>/Constant'

  90.0,

  // Mask Parameter: CompareToConstant_const_e
  //  Referenced by: '<S449>/Constant'

  180.0,

  // Mask Parameter: CompareToConstant_const_b
  //  Referenced by: '<S386>/Constant'

  180.0,

  // Mask Parameter: CompareToConstant_const_g
  //  Referenced by: '<S384>/Constant'

  90.0,

  // Mask Parameter: CompareToConstant_const_iz
  //  Referenced by: '<S387>/Constant'

  180.0,

  // Mask Parameter: CompareToConstant_const_bz
  //  Referenced by: '<S380>/Constant'

  180.0,

  // Mask Parameter: CompareToConstant_const_d
  //  Referenced by: '<S378>/Constant'

  90.0,

  // Mask Parameter: CompareToConstant_const_ex
  //  Referenced by: '<S381>/Constant'

  180.0,

  // Mask Parameter: Distanceintogustx_d_m
  //  Referenced by: '<S515>/Distance into Gust (x) (Limited to gust length d)'

  120.0,

  // Mask Parameter: Distanceintogusty_d_m
  //  Referenced by: '<S512>/Distance into gust (y)'

  120.0,

  // Mask Parameter: Distanceintogustz_d_m
  //  Referenced by: '<S512>/Distance into gust (z)'

  80.0,

  // Mask Parameter: DiscreteWindGustModel_d_m
  //  Referenced by: '<S512>/pi//Gust length'

  { 120.0, 120.0, 80.0 },

  // Mask Parameter: ThreeaxisInertialMeasurementU_b
  //  Referenced by: '<S416>/Measurement bias'

  { 0.0, 0.0, 0.0 },

  // Mask Parameter: ThreeaxisInertialMeasurementU_c
  //  Referenced by: '<S416>/g-sensitive bias'

  { 0.0, 0.0, 0.0 },

  // Mask Parameter: ThreeaxisInertialMeasurementU_m
  //  Referenced by: '<S416>/Scale factors & Cross-coupling  errors '

  { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 },

  // Mask Parameter: ThreeaxisInertialMeasurementU_e
  //  Referenced by: '<S415>/wl_ins'

  { 0.0, 0.0, 0.0 },

  // Mask Parameter: CustomVariableMass6DOFQuaternio
  //  Referenced by: '<S26>/High Gain Quaternion Normalization'

  1.0,

  // Mask Parameter: CheckAltitude_max
  //  Referenced by: '<S433>/max_val'

  850000.0,

  // Mask Parameter: CheckLatitude_max
  //  Referenced by: '<S434>/max_val'

  90.0,

  // Mask Parameter: CheckLongitude_max
  //  Referenced by: '<S435>/max_val'

  180.0,

  // Mask Parameter: Istimewithinmodellimits_max
  //  Referenced by: '<S437>/max_val'

  5.0,

  // Mask Parameter: CheckAltitude_min
  //  Referenced by: '<S433>/min_val'

  -1000.0,

  // Mask Parameter: CheckLatitude_min
  //  Referenced by: '<S434>/min_val'

  -90.0,

  // Mask Parameter: CheckLongitude_min
  //  Referenced by: '<S435>/min_val'

  -180.0,

  // Mask Parameter: Istimewithinmodellimits_min
  //  Referenced by: '<S437>/min_val'

  0.0,

  // Mask Parameter: FlatEarthtoLLA_psi
  //  Referenced by: '<S63>/ref_rotation'

  0.0,

  // Mask Parameter: FlatEarthtoLLA_psi_e
  //  Referenced by: '<S371>/ref_rotation'

  0.0,

  // Mask Parameter: WhiteNoise_pwr
  //  Referenced by: '<S157>/Constant'

  { 0.01, 0.01, 0.01, 0.01 },

  // Mask Parameter: WhiteNoise_pwr_d
  //  Referenced by: '<S118>/Constant'

  { 0.01, 0.01, 0.01, 0.01 },

  // Mask Parameter: BandLimitedWhiteNoise_seed
  //  Referenced by: '<S511>/White Noise'

  23341.0,

  // Mask Parameter: DirectionCosineMatrixtoQuater_i
  //  Referenced by: '<S203>/If Warning//Error'

  4.4408920985006262E-16,

  // Mask Parameter: DirectionCosineMatrixtoQuate_i5
  //  Referenced by: '<S241>/If Warning//Error'

  4.4408920985006262E-16,

  // Mask Parameter: DirectionCosineMatrixtoQuater_b
  //  Referenced by: '<S329>/If Warning//Error'

  4.4408920985006262E-16,

  // Mask Parameter: DiscreteWindGustModel_v_m
  //  Referenced by: '<S512>/Gust magnitude//2.0'

  { 3.5, 3.5, 3.0 },

  // Mask Parameter: ThreeaxisInertialMeasurement_c1
  //  Referenced by:
  //    '<S421>/2*zeta * wn'
  //    '<S421>/wn^2'

  190.0,

  // Mask Parameter: ThreeaxisInertialMeasurementU_i
  //  Referenced by:
  //    '<S432>/2*zeta * wn'
  //    '<S432>/wn^2'

  190.0,

  // Mask Parameter: DiscreteWindGustModel_Gx
  //  Referenced by: '<S512>/Constant'

  true,

  // Mask Parameter: DiscreteWindGustModel_Gy
  //  Referenced by: '<S512>/Constant1'

  true,

  // Mask Parameter: DiscreteWindGustModel_Gz
  //  Referenced by: '<S512>/Constant2'

  true,

  // Expression: 1
  //  Referenced by: '<S43>/Constant'

  1.0,

  // Expression: 1
  //  Referenced by: '<S44>/Constant'

  1.0,

  // Expression: -90
  //  Referenced by: '<S72>/Bias'

  -90.0,

  // Expression: -1
  //  Referenced by: '<S72>/Gain'

  -1.0,

  // Expression: +90
  //  Referenced by: '<S72>/Bias1'

  90.0,

  // Expression: 180
  //  Referenced by: '<S75>/Bias'

  180.0,

  // Expression: -180
  //  Referenced by: '<S75>/Bias1'

  -180.0,

  // Expression: 180
  //  Referenced by: '<S73>/Bias'

  180.0,

  // Expression: -180
  //  Referenced by: '<S73>/Bias1'

  -180.0,

  // Expression: -90
  //  Referenced by: '<S78>/Bias'

  -90.0,

  // Expression: -1
  //  Referenced by: '<S78>/Gain'

  -1.0,

  // Expression: +90
  //  Referenced by: '<S78>/Bias1'

  90.0,

  // Expression: 360
  //  Referenced by: '<S81>/Constant2'

  360.0,

  // Expression: 180
  //  Referenced by: '<S81>/Bias'

  180.0,

  // Expression: -180
  //  Referenced by: '<S81>/Bias1'

  -180.0,

  // Expression: 180
  //  Referenced by: '<S69>/Constant'

  180.0,

  // Expression: 0
  //  Referenced by: '<S69>/Constant1'

  0.0,

  // Expression: 360
  //  Referenced by: '<S79>/Constant2'

  360.0,

  // Expression: 180
  //  Referenced by: '<S79>/Bias'

  180.0,

  // Expression: -180
  //  Referenced by: '<S79>/Bias1'

  -180.0,

  // Expression: 1
  //  Referenced by: '<S129>/Gain'

  1.0,

  // Expression: 1
  //  Referenced by: '<S137>/Gain'

  1.0,

  // Expression: 1
  //  Referenced by: '<S168>/Gain'

  1.0,

  // Expression: 1
  //  Referenced by: '<S176>/Gain'

  1.0,

  // Expression: -90
  //  Referenced by: '<S376>/Bias'

  -90.0,

  // Expression: -1
  //  Referenced by: '<S376>/Gain'

  -1.0,

  // Expression: +90
  //  Referenced by: '<S376>/Bias1'

  90.0,

  // Expression: 180
  //  Referenced by: '<S379>/Bias'

  180.0,

  // Expression: -180
  //  Referenced by: '<S379>/Bias1'

  -180.0,

  // Expression: 180
  //  Referenced by: '<S377>/Bias'

  180.0,

  // Expression: -180
  //  Referenced by: '<S377>/Bias1'

  -180.0,

  // Expression: -90
  //  Referenced by: '<S382>/Bias'

  -90.0,

  // Expression: -1
  //  Referenced by: '<S382>/Gain'

  -1.0,

  // Expression: +90
  //  Referenced by: '<S382>/Bias1'

  90.0,

  // Expression: 360
  //  Referenced by: '<S385>/Constant2'

  360.0,

  // Expression: 180
  //  Referenced by: '<S385>/Bias'

  180.0,

  // Expression: -180
  //  Referenced by: '<S385>/Bias1'

  -180.0,

  // Expression: 180
  //  Referenced by: '<S373>/Constant'

  180.0,

  // Expression: 0
  //  Referenced by: '<S373>/Constant1'

  0.0,

  // Expression: 360
  //  Referenced by: '<S383>/Constant2'

  360.0,

  // Expression: 180
  //  Referenced by: '<S383>/Bias'

  180.0,

  // Expression: -180
  //  Referenced by: '<S383>/Bias1'

  -180.0,

  // Expression: -90
  //  Referenced by: '<S444>/Bias'

  -90.0,

  // Expression: -1
  //  Referenced by: '<S444>/Gain'

  -1.0,

  // Expression: +90
  //  Referenced by: '<S444>/Bias1'

  90.0,

  // Expression: 180
  //  Referenced by: '<S447>/Bias'

  180.0,

  // Expression: -180
  //  Referenced by: '<S447>/Bias1'

  -180.0,

  // Expression: 180
  //  Referenced by: '<S445>/Bias'

  180.0,

  // Expression: -180
  //  Referenced by: '<S445>/Bias1'

  -180.0,

  // Expression: ones(1,maxdef+1)
  //  Referenced by: '<S466>/pp[13]'

  { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 },

  // Expression: 1
  //  Referenced by: '<S466>/Constant'

  1.0,

  // Expression: ones(1,maxdef+1)
  //  Referenced by: '<S467>/pp[13]'

  { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 },

  // Expression: k
  //  Referenced by: '<S467>/k[13][13]'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.33333333333333331,
    0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.26666666666666666, 0.2, 0.0, -0.33333333333333331, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.25714285714285712, 0.22857142857142856,
    0.14285714285714285, 0.0, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.25396825396825395, 0.23809523809523808, 0.19047619047619047,
    0.1111111111111111, 0.0, -0.14285714285714285, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.25252525252525254, 0.24242424242424243, 0.21212121212121213,
    0.16161616161616163, 0.090909090909090912, 0.0, -0.1111111111111111, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.25174825174825177, 0.24475524475524477,
    0.22377622377622378, 0.1888111888111888, 0.13986013986013987,
    0.076923076923076927, 0.0, -0.090909090909090912, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.25128205128205128, 0.24615384615384617, 0.23076923076923078,
    0.20512820512820512, 0.16923076923076924, 0.12307692307692308,
    0.066666666666666666, 0.0, -0.076923076923076927, 0.0, 0.0, 0.0, 0.0,
    0.25098039215686274, 0.24705882352941178, 0.23529411764705882,
    0.21568627450980393, 0.18823529411764706, 0.15294117647058825,
    0.10980392156862745, 0.058823529411764705, 0.0, -0.066666666666666666, 0.0,
    0.0, 0.0, 0.25077399380804954, 0.24767801857585139, 0.23839009287925697,
    0.22291021671826625, 0.20123839009287925, 0.17337461300309598,
    0.13931888544891641, 0.099071207430340563, 0.052631578947368418, 0.0,
    -0.058823529411764705, 0.0, 0.0, 0.25062656641604009, 0.24812030075187969,
    0.24060150375939848, 0.22807017543859648, 0.21052631578947367,
    0.18796992481203006, 0.16040100250626566, 0.12781954887218044,
    0.090225563909774431, 0.047619047619047616, 0.0, -0.052631578947368418, 0.0,
    0.25051759834368531, 0.2484472049689441, 0.24223602484472051,
    0.2318840579710145, 0.21739130434782608, 0.19875776397515527,
    0.17598343685300208, 0.14906832298136646, 0.11801242236024845,
    0.082815734989648032, 0.043478260869565216, 0.0, -0.047619047619047616 },

  // Expression: 0
  //  Referenced by: '<S462>/bpp'

  0.0,

  // Expression: fm(2)
  //  Referenced by: '<S462>/Constant'

  1.0,

  // Expression: 1
  //  Referenced by: '<S462>/Constant1'

  1.0,

  // Expression: ones(1,maxdef+1)
  //  Referenced by: '<S462>/Unit Delay1'

  { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 },

  // Expression: 1
  //  Referenced by: '<S470>/Constant'

  1.0,

  // Expression: 1
  //  Referenced by: '<S470>/Gain1'

  1.0,

  // Expression: 1
  //  Referenced by: '<S470>/Gain2'

  1.0,

  // Expression: 1
  //  Referenced by: '<S472>/Constant'

  1.0,

  // Expression: 1
  //  Referenced by: '<S473>/Constant'

  1.0,

  // Expression: 0
  //  Referenced by: '<S476>/Constant1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S476>/Constant'

  0.0,

  // Expression: k
  //  Referenced by: '<S476>/k[13][13]'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.33333333333333331,
    0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.26666666666666666, 0.2, 0.0, -0.33333333333333331, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.25714285714285712, 0.22857142857142856,
    0.14285714285714285, 0.0, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.25396825396825395, 0.23809523809523808, 0.19047619047619047,
    0.1111111111111111, 0.0, -0.14285714285714285, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.25252525252525254, 0.24242424242424243, 0.21212121212121213,
    0.16161616161616163, 0.090909090909090912, 0.0, -0.1111111111111111, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.25174825174825177, 0.24475524475524477,
    0.22377622377622378, 0.1888111888111888, 0.13986013986013987,
    0.076923076923076927, 0.0, -0.090909090909090912, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.25128205128205128, 0.24615384615384617, 0.23076923076923078,
    0.20512820512820512, 0.16923076923076924, 0.12307692307692308,
    0.066666666666666666, 0.0, -0.076923076923076927, 0.0, 0.0, 0.0, 0.0,
    0.25098039215686274, 0.24705882352941178, 0.23529411764705882,
    0.21568627450980393, 0.18823529411764706, 0.15294117647058825,
    0.10980392156862745, 0.058823529411764705, 0.0, -0.066666666666666666, 0.0,
    0.0, 0.0, 0.25077399380804954, 0.24767801857585139, 0.23839009287925697,
    0.22291021671826625, 0.20123839009287925, 0.17337461300309598,
    0.13931888544891641, 0.099071207430340563, 0.052631578947368418, 0.0,
    -0.058823529411764705, 0.0, 0.0, 0.25062656641604009, 0.24812030075187969,
    0.24060150375939848, 0.22807017543859648, 0.21052631578947367,
    0.18796992481203006, 0.16040100250626566, 0.12781954887218044,
    0.090225563909774431, 0.047619047619047616, 0.0, -0.052631578947368418, 0.0,
    0.25051759834368531, 0.2484472049689441, 0.24223602484472051,
    0.2318840579710145, 0.21739130434782608, 0.19875776397515527,
    0.17598343685300208, 0.14906832298136646, 0.11801242236024845,
    0.082815734989648032, 0.043478260869565216, 0.0, -0.047619047619047616 },

  // Expression: zeros(maxdef+1,maxdef+1)
  //  Referenced by: '<S460>/dp[13][13]'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0 },

  // Expression: snorm
  //  Referenced by: '<S460>/snorm[169]'

  { 1.0, 1.0, 1.5, 2.5, 4.375, 7.875, 14.4375, 26.8125, 50.2734375, 94.9609375,
    180.42578125, 344.44921875, 660.1943359375, 0.0, 1.0, 1.7320508075688772,
    3.0618621784789726, 5.5339859052946636, 10.16658128379447,
    18.903124741692839, 35.469603513959669, 67.03125, 127.40346687426536,
    243.28607380714598, 466.38644692864216, 897.027461585248, 0.0, 0.0,
    0.8660254037844386, 1.9364916731037085, 3.9131189606246322,
    7.685213074469698, 14.944232269507859, 28.960809996010127,
    56.082367403612253, 108.65004161512664, 210.69192030396434,
    409.04797337487776, 795.12986069746626, 0.0, 0.0, 0.0, 0.79056941504209488,
    2.0916500663351889, 4.7062126492541738, 9.9628215130052382,
    20.478385136833914, 41.41957332816817, 82.982839993569982,
    165.28034045942309, 327.9680080977904, 649.22081265302029, 0.0, 0.0, 0.0,
    0.0, 0.739509972887452, 2.2185299186623553, 5.4568620790707172,
    12.348930874776167, 26.736219617835371, 56.375738371688975,
    116.87084953567937, 239.5139682335957, 486.91560948976519, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.7015607600201138, 2.3268138086232852, 6.1744654373880836,
    14.830586268334102, 33.69094768709671, 73.915615322315773,
    158.42359886807964, 334.02135244518831, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.671693289381396, 2.4218245962496963, 6.8652274293172546, 17.39793057467611,
    41.320085114855779, 94.117642301250768, 208.29891011946015, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.6472598492877496, 2.5068266169601756,
    7.5335249254737544, 20.043185339772048, 49.604352946160631,
    117.05388227149012, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.6267066542400439, 2.5839777317091466, 8.1825961504123, 22.760038068635609,
    58.526941135745062, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.60904939217552367, 2.65478475211798, 8.8149248398872544,
    25.543251233216804, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.59362791713657326, 2.72034486491732, 9.4324706362690076, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.579979473934679,
    2.7814838439702596, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.56776801212685635 },

  // Expression: zeros(maxdef+1,maxdef+1)
  //  Referenced by: '<S460>/Unit Delay'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0 },

  // Expression: snorm
  //  Referenced by: '<S460>/Unit Delay1'

  { 1.0, 1.0, 1.5, 2.5, 4.375, 7.875, 14.4375, 26.8125, 50.2734375, 94.9609375,
    180.42578125, 344.44921875, 660.1943359375, 0.0, 1.0, 1.7320508075688772,
    3.0618621784789726, 5.5339859052946636, 10.16658128379447,
    18.903124741692839, 35.469603513959669, 67.03125, 127.40346687426536,
    243.28607380714598, 466.38644692864216, 897.027461585248, 0.0, 0.0,
    0.8660254037844386, 1.9364916731037085, 3.9131189606246322,
    7.685213074469698, 14.944232269507859, 28.960809996010127,
    56.082367403612253, 108.65004161512664, 210.69192030396434,
    409.04797337487776, 795.12986069746626, 0.0, 0.0, 0.0, 0.79056941504209488,
    2.0916500663351889, 4.7062126492541738, 9.9628215130052382,
    20.478385136833914, 41.41957332816817, 82.982839993569982,
    165.28034045942309, 327.9680080977904, 649.22081265302029, 0.0, 0.0, 0.0,
    0.0, 0.739509972887452, 2.2185299186623553, 5.4568620790707172,
    12.348930874776167, 26.736219617835371, 56.375738371688975,
    116.87084953567937, 239.5139682335957, 486.91560948976519, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.7015607600201138, 2.3268138086232852, 6.1744654373880836,
    14.830586268334102, 33.69094768709671, 73.915615322315773,
    158.42359886807964, 334.02135244518831, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.671693289381396, 2.4218245962496963, 6.8652274293172546, 17.39793057467611,
    41.320085114855779, 94.117642301250768, 208.29891011946015, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.6472598492877496, 2.5068266169601756,
    7.5335249254737544, 20.043185339772048, 49.604352946160631,
    117.05388227149012, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.6267066542400439, 2.5839777317091466, 8.1825961504123, 22.760038068635609,
    58.526941135745062, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.60904939217552367, 2.65478475211798, 8.8149248398872544,
    25.543251233216804, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.59362791713657326, 2.72034486491732, 9.4324706362690076, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.579979473934679,
    2.7814838439702596, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.56776801212685635 },

  // Expression: 0
  //  Referenced by: '<S460>/Merge1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S460>/Merge'

  0.0,

  // Expression: 1
  //  Referenced by: '<S489>/Gain'

  1.0,

  // Expression: zeros(maxdef+1,maxdef+1)
  //  Referenced by: '<S488>/zeros(maxdef+1,maxdef+1)'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0 },

  // Expression: zeros(maxdef+1,maxdef+1)
  //  Referenced by: '<S461>/tc[13][13]'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0 },

  // Expression: zeros(maxdef+1,maxdef+1)
  //  Referenced by: '<S461>/Unit Delay'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0 },

  // Expression: c
  //  Referenced by: '<S461>/c[maxdef][maxdef]'

  { 0.0, 4796.3, -4923.1812154337767, -348.13372969305919, 1567.7782069699783,
    476.81266220996059, -379.95280730802608, -1925.9994708080098, 677.015625,
    -2777.3955778589848, 802.84404356358175, -0.0, -897.027461585248, -29438.2,
    -1493.5, -553.21702793749932, 477.34519742006415, -738.01423597380563,
    1510.1443691332956, 490.17081843985773, -564.73579492219744,
    -1026.3073234861042, 1162.5554452818549, -84.276768121585746,
    859.00074408724333, 238.53895820923987, -3666.75, 5221.6135695778939,
    1454.0566529540724, -424.8520036436218, 377.9611669867686,
    -564.2748966455755, 588.80275141860955, 122.87031082100349,
    550.88032526463667, 979.19751192412582, 760.28956611334615,
    -196.78080485867423, 1168.5974627754365, 3379.5, -7200.2750989111519,
    2369.4912112096977, 460.34857037901179, -244.03829105285917,
    35.496478698597684, -366.1554455056451, 302.54880643201608,
    -387.67518445861288, -383.35502092748504, 514.23173795698926,
    -263.46536505695531, -1071.2143408774834, 3970.3125, 4509.091715634092,
    460.96541356158167, -701.95776226208943, 51.543845110255411,
    70.577012458023447, 18.84719184984861, 21.610629030858291,
    240.25549754701245, -232.4675390409673, -583.93336104629464,
    110.89651920765574, 100.20640573355649, -1834.0875, 3660.9859202943885,
    1473.2553463758411, -664.98784733961486, -348.75290321372222,
    5.4020178521548763, 41.577814612708416, -67.084541316116585,
    41.191364575903528, 137.44365153994127, -24.792051068913466,
    -18.823528460250156, 145.80923708362209, 1001.9625000000001,
    1279.7415450126052, 1080.4679930854181, -1286.2002573289762,
    -154.97488304560835, 31.644667797276679, -47.220038243512143,
    -1.8770535629344738, -23.062804876033614, 7.5335249254737544,
    -84.1813784270426, -104.16914118693732, -11.705388227149014, 2190.58125,
    -2692.1429067095391, -205.6217509716719, 1068.9717041427305,
    185.23396312164252, 56.187635480231556, -7.2654737887490892,
    3.8188331107977227, 1.5040959701761054, -10.077513153665672,
    -23.729528836195669, -34.140057102953413, 17.558082340723519, 1216.6171875,
    596.578125, -947.792009121047, -128.40067731732134, -553.43974608919211,
    197.24679736884357, 79.636638180080155, -40.861273856450865,
    -1.3160839739040922, 5.1769198334919508, -2.9202632273297784,
    -22.918804583706862, 5.1086502466433608, 522.28515625, 1121.1505084935352,
    325.95012484537995, -265.54508797942395, 33.825443023013385,
    -444.72050946967653, -1.739793057467611, 65.541666851621656,
    -23.514197358553233, -6.3341136786254468, -5.2239256708018447,
    -5.44068972983464, -8.489223572642107, -360.8515625, -1484.0450502235904,
    42.138384060792873, 99.168204275653849, -58.435424767839685,
    133.04810758016839, -28.924059580399042, 44.095007747498506,
    19.638230760989519, -4.7786125538123647, -2.1370605016916637,
    -1.3339527900497614, -0.556296768794052, 1033.34765625, -652.941025700099,
    -940.81033876221875, 688.73281700535983, -191.61117458687659,
    95.054159320847774, -65.882349610875536, 4.9604352946160635,
    38.692064716680534, -1.762984967977451, 1.088137945966928,
    2.0299281587713764, 0.45421440970148508, -1320.388671875, -89.7027461585248,
    397.56493034873313, 779.06497518362437, -438.22404854078866,
    300.61921720066948, 20.829891011946017, 70.232329362894077,
    -23.410776454298027, -12.771625616608402, 1.8864941272538016,
    -2.5033354595732336, -0.0 },

  // Expression: cd
  //  Referenced by: '<S461>/cd[maxdef][maxdef]'

  { 0.0, -30.2, -51.268703904038766, 19.902104160113321, -2.2135943621178655,
    2.0333162567588938, 5.6709374225078513, 21.2817621083758, -26.8125,
    -38.221040062279606, 0.0, 0.0, -0.0, 7.0, 9.0, -14.982239485470789,
    -1.5491933384829668, 22.696089971622865, 17.675990071280303,
    -22.41634840426179, 14.480404998005064, 33.649420442167347,
    10.865004161512665, 21.069192030396437, 40.904797337487778, 0.0, -16.5,
    -10.738715006927039, 0.25980762113533157, -1.5811388300841898,
    7.9482702520737174, -0.0, -11.955385815606286, -16.382708109467131,
    -4.1419573328168173, -33.193135997427994, -33.056068091884619, 0.0,
    -64.922081265302026, 6.0, -17.452614417330143, 3.872983346207417,
    -8.6962635654630436, -2.5882849051060819, 7.3211487315857724,
    2.1827448316282871, -2.4697861749552334, 16.041731770701222,
    16.912721511506692, 11.687084953567938, 23.951396823359573,
    48.691560948976523, -3.5, -4.9805873147651978, -25.435273244060109,
    10.876580344942983, -2.9580398915498081, -0.42093645601206825,
    0.46536276172465707, -6.7919119811268924, -2.9661172536668206,
    3.3690947687096711, -7.3915615322315773, -0.0, -0.0, -2.3625,
    6.0999487702766819, -6.1481704595757591, 0.47062126492541739,
    2.6622359023948263, 0.9821850640281593, 0.87320127619581489,
    0.24218245962496965, -3.4326137146586273, -0.0, 4.1320085114855782, -0.0,
    0.0, -11.55, -9.45156237084642, -1.494423226950786, 15.940514420808382,
    -8.7309793265131486, 0.0, 0.80603194725767524, 0.12945196985754992,
    1.2534133084800878, -0.75335249254737546, -0.0, 4.9604352946160635, -0.0,
    -8.04375, -7.0939207027919338, -8.6882429988030374, 18.430546623150523,
    1.2348930874776167, -3.70467926243285, -2.1796421366247265,
    0.45308189450142466, 0.062670665424004388, 1.2919888658545733,
    -0.81825961504123, -0.0, 0.0, -5.02734375, 13.40625, -11.216473480722451,
    20.709786664084085, -2.6736219617835371, 5.9322345073336411,
    2.7460909717269022, -0.25068266169601755, 0.25068266169601755,
    0.12180987843510474, 0.530956950423596, -0.88149248398872548, 0.0,
    -9.49609375, -12.740346687426538, -0.0, 33.193135997427994,
    -22.55029534867559, 0.0, 5.2193791724028324, 0.0, -0.0, -0.1827148176526571,
    -0.0, -0.0, -0.0, 0.0, -0.0, -21.069192030396437, 33.056068091884619,
    -11.687084953567938, -14.783123064463155, -0.0, -2.0043185339772047,
    -1.63651923008246, -0.265478475211798, -0.0, -0.0579979473934679, 0.0, -0.0,
    0.0, -0.0, 0.0, -0.0, -15.842359886807964, 0.0, -0.0, -0.0,
    -0.88149248398872548, -0.0, -0.0579979473934679, -0.056776801212685635, 0.0,
    0.0, -0.0, 0.0, -48.691560948976523, -0.0, 0.0, -0.0, 0.0, -0.0, -0.0, -0.0,
    -0.056776801212685635 },

  // Expression: zeros(maxdef+1,maxdef+1)
  //  Referenced by: '<S488>/Unit Delay'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0 },

  // Expression: 0
  //  Referenced by: '<S458>/bt'

  0.0,

  // Expression: 0
  //  Referenced by: '<S458>/bp'

  0.0,

  // Expression: 0
  //  Referenced by: '<S458>/br'

  0.0,

  // Expression: 0
  //  Referenced by: '<S458>/bpp'

  0.0,

  // Expression: 1
  //  Referenced by: '<S464>/Constant1'

  1.0,

  // Expression: 0
  //  Referenced by: '<S464>/Merge'

  0.0,

  // Expression: fm
  //  Referenced by: '<S459>/fm'

  { 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0 },

  // Expression: 0
  //  Referenced by: '<S464>/Merge1'

  0.0,

  // Expression: fn
  //  Referenced by: '<S459>/fn'

  { 0.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0 },

  // Expression: 0
  //  Referenced by: '<S465>/Constant1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S459>/Unit Delay1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S459>/Unit Delay3'

  0.0,

  // Expression: 0
  //  Referenced by: '<S459>/Unit Delay2'

  0.0,

  // Expression: 0
  //  Referenced by: '<S459>/Unit Delay4'

  0.0,

  // Expression: [0 0 0 0]
  //  Referenced by: '<S450>/bt,bp,br,bpp'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: 0
  //  Referenced by: '<S450>/Unit Delay'

  0.0,

  // Expression: [0 0 0 0]
  //  Referenced by: '<S450>/Unit Delay2'

  { 0.0, 0.0, 0.0, 0.0 },

  // Expression: 6378.137
  //  Referenced by: '<S451>/r'

  6378.137,

  // Expression: 1
  //  Referenced by: '<S451>/ct'

  1.0,

  // Expression: 0
  //  Referenced by: '<S451>/st'

  0.0,

  // Expression: 0
  //  Referenced by: '<S451>/sa'

  0.0,

  // Expression: 0
  //  Referenced by: '<S451>/ca'

  0.0,

  // Expression: 6356.7523142
  //  Referenced by: '<S451>/b'

  6356.7523142,

  // Expression: 6378.137
  //  Referenced by: '<S451>/a'

  6378.137,

  // Expression: 2
  //  Referenced by: '<S496>/Gain'

  2.0,

  // Expression: 1
  //  Referenced by: '<S498>/Constant'

  1.0,

  // Expression: (1:(maxdef-1))
  //  Referenced by: '<S499>/sp[11]'

  { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0 },

  // Expression: (1:(maxdef-1))
  //  Referenced by: '<S499>/cp[11]'

  { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0 },

  // Expression: maxdef-1
  //  Referenced by: '<S499>/For Iterator'

  11.0,

  // Expression: [1:maxdef-1]
  //  Referenced by: '<S499>/Constant'

  { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0 },

  // Expression: 0
  //  Referenced by: '<S499>/Unit Delay1'

  0.0,

  // Expression: 1
  //  Referenced by: '<S499>/cp[m-1] sp[m-1]'

  1.0,

  // Expression: [1:maxdef-1]
  //  Referenced by: '<S499>/Constant1'

  { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0 },

  // Expression: [0 0 (1:(maxdef-1))]
  //  Referenced by: '<S452>/sp[13]'

  { 0.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0 },

  // Expression: [1 1 (1:(maxdef-1))]
  //  Referenced by: '<S452>/cp[13]'

  { 1.0, 1.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0 },

  // Expression: 1
  //  Referenced by: '<S452>/Gain'

  1.0,

  // Expression: 1
  //  Referenced by: '<S452>/Gain1'

  1.0,

  // Expression: 1
  //  Referenced by: '<S452>/cp[1]'

  1.0,

  // Expression: 0
  //  Referenced by: '<S452>/sp[1]'

  0.0,

  // Expression: [0]
  //  Referenced by: '<S515>/x'

  0.0,

  // Expression: 0
  //  Referenced by: '<S515>/Distance into Gust (x) (Limited to gust length d)'

  0.0,

  // Expression: 0
  //  Referenced by: '<S515>/Distance into Gust (x) (Limited to gust length d)'

  0.0,

  // Expression: 1E6
  //  Referenced by: '<S12>/Gain_clock'

  1.0E+6,

  // Expression: 0
  //  Referenced by: '<S421>/Integrator, Second-Order Limited'

  0.0,

  // Expression: 0
  //  Referenced by: '<S421>/Integrator, Second-Order Limited'

  0.0,

  // Expression: dtype_a
  //  Referenced by: '<S417>/Constant'

  1.0,

  // Expression: 0.5
  //  Referenced by: '<S25>/1//2'

  0.5,

  // Expression: 2
  //  Referenced by: '<S30>/Gain'

  2.0,

  // Expression: 2
  //  Referenced by: '<S33>/Gain'

  2.0,

  // Expression: 2
  //  Referenced by: '<S28>/Gain'

  2.0,

  // Expression: 2
  //  Referenced by: '<S34>/Gain'

  2.0,

  // Expression: 2
  //  Referenced by: '<S29>/Gain'

  2.0,

  // Expression: 2
  //  Referenced by: '<S32>/Gain'

  2.0,

  // Expression: [123544 123544 123544 0 0 0 0 0]
  //  Referenced by: '<Root>/Constant'

  { 123544.0, 123544.0, 123544.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  // Expression: [3 0 5 1 -1 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  Referenced by: '<Root>/Constant1'

  { 3.0, 0.0, 5.0, 1.0, -1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0 },

  // Expression: 1
  //  Referenced by: '<S84>/Constant2'

  1.0,

  // Expression: R
  //  Referenced by: '<S84>/Constant1'

  6.378137E+6,

  // Expression: 1
  //  Referenced by: '<S87>/Constant'

  1.0,

  // Expression: 1
  //  Referenced by: '<S89>/Constant'

  1.0,

  // Expression: F
  //  Referenced by: '<S88>/Constant'

  0.0033528106647474805,

  // Expression: 1
  //  Referenced by: '<S88>/f'

  1.0,

  // Expression: 1
  //  Referenced by: '<S84>/Constant'

  1.0,

  // Expression: 1
  //  Referenced by: '<S84>/Constant3'

  1.0,

  // Expression: 360
  //  Referenced by: '<S75>/Constant2'

  360.0,

  // Expression: 180
  //  Referenced by: '<S68>/Constant'

  180.0,

  // Expression: 0
  //  Referenced by: '<S68>/Constant1'

  0.0,

  // Expression: 360
  //  Referenced by: '<S73>/Constant2'

  360.0,

  // Expression: 100000
  //  Referenced by: '<S5>/Saturation_1'

  100000.0,

  // Expression: 0
  //  Referenced by: '<S5>/Saturation_1'

  0.0,

  // Expression: juliandate(year,month,day)
  //  Referenced by: '<S66>/Julian Date'

  2.4532885E+6,

  // Expression: [1 0 0 0]
  //  Referenced by: '<S194>/Merge'

  { 1.0, 0.0, 0.0, 0.0 },

  // Expression: [1 0 0 0]
  //  Referenced by: '<S193>/Merge'

  { 1.0, 0.0, 0.0, 0.0 },

  // Expression: 0
  //  Referenced by: '<Root>/Memory'

  0.0,

  // Expression: 1/4.2
  //  Referenced by: '<S2>/MotorFault3'

  0.23809523809523808,

  // Expression: 0
  //  Referenced by: '<S511>/White Noise'

  0.0,

  // Computed Parameter: WhiteNoise_StdDev
  //  Referenced by: '<S511>/White Noise'

  1.0,

  // Expression: 1.0
  //  Referenced by: '<S512>/2'

  1.0,

  // Expression: [1,1,1]
  //  Referenced by: '<S67>/Constant_[1 1 1]'

  { 1.0, 1.0, 1.0 },

  // Expression: [-1,-1,-0.5]
  //  Referenced by: '<S67>/Uniform Random Number'

  { -1.0, -1.0, -0.5 },

  // Expression: [1,1,0.5]
  //  Referenced by: '<S67>/Uniform Random Number'

  { 1.0, 1.0, 0.5 },

  // Expression: [564565,6846798,46545]
  //  Referenced by: '<S67>/Uniform Random Number'

  { 564565.0, 6.846798E+6, 46545.0 },

  // Expression: -1
  //  Referenced by: '<S67>/Gain_-1'

  -1.0,

  // Expression: 10000
  //  Referenced by: '<S67>/Saturation_2'

  10000.0,

  // Expression: 0
  //  Referenced by: '<S67>/Saturation_2'

  0.0,

  // Expression: 5
  //  Referenced by: '<S67>/Constant_V'

  5.0,

  // Expression: max_height_low
  //  Referenced by: '<S182>/Limit Function 10ft to 1000ft'

  1000.0,

  // Expression: 10
  //  Referenced by: '<S182>/Limit Function 10ft to 1000ft'

  10.0,

  // Expression: max_height_low
  //  Referenced by: '<S165>/Limit Height h<1000ft'

  1000.0,

  // Expression: 0
  //  Referenced by: '<S165>/Limit Height h<1000ft'

  0.0,

  // Expression: 0.1
  //  Referenced by: '<S165>/sigma_wg '

  0.1,

  // Expression: h_vec
  //  Referenced by: '<S164>/PreLook-Up Index Search  (altitude)'

  { 500.0, 1750.0, 3750.0, 7500.0, 15000.0, 25000.0, 35000.0, 45000.0, 55000.0,
    65000.0, 75000.0, 80000.0 },

  // Expression: [1:7]
  //  Referenced by: '<S164>/PreLook-Up Index Search  (prob of exceed)'

  { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0 },

  // Expression: sigma_vec'
  //  Referenced by: '<S164>/Medium//High Altitude Intensity'

  { 3.2, 2.2, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.2, 3.6, 3.3,
    1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.6, 6.9, 7.4, 6.7, 4.6, 2.7,
    0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 8.6, 9.6, 10.6, 10.1, 8.0, 6.6, 5.0, 4.2, 2.7,
    0.0, 0.0, 0.0, 11.8, 13.0, 16.0, 15.1, 11.6, 9.7, 8.1, 8.2, 7.9, 4.9, 3.2,
    2.1, 15.6, 17.6, 23.0, 23.6, 22.1, 20.0, 16.0, 15.1, 12.1, 7.9, 6.2, 5.1,
    18.7, 21.5, 28.4, 30.2, 30.7, 31.0, 25.2, 23.1, 17.5, 10.7, 8.4, 7.2 },

  // Expression: 0
  //  Referenced by: '<S157>/White Noise'

  0.0,

  // Computed Parameter: WhiteNoise_StdDev_j
  //  Referenced by: '<S157>/White Noise'

  1.0,

  // Expression: 1
  //  Referenced by: '<S154>/Lv'

  1.0,

  // Expression: 1
  //  Referenced by: '<S154>/Lw'

  1.0,

  // Expression: eye(3)
  //  Referenced by: '<S67>/Constant_DCM'

  { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 },

  // Expression: max_height_low
  //  Referenced by: '<S143>/Limit Function 10ft to 1000ft'

  1000.0,

  // Expression: 10
  //  Referenced by: '<S143>/Limit Function 10ft to 1000ft'

  10.0,

  // Expression: max_height_low
  //  Referenced by: '<S126>/Limit Height h<1000ft'

  1000.0,

  // Expression: 0
  //  Referenced by: '<S126>/Limit Height h<1000ft'

  0.0,

  // Expression: 0.1
  //  Referenced by: '<S126>/sigma_wg '

  0.1,

  // Expression: h_vec
  //  Referenced by: '<S125>/PreLook-Up Index Search  (altitude)'

  { 500.0, 1750.0, 3750.0, 7500.0, 15000.0, 25000.0, 35000.0, 45000.0, 55000.0,
    65000.0, 75000.0, 80000.0 },

  // Expression: [1:7]
  //  Referenced by: '<S125>/PreLook-Up Index Search  (prob of exceed)'

  { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0 },

  // Expression: sigma_vec'
  //  Referenced by: '<S125>/Medium//High Altitude Intensity'

  { 3.2, 2.2, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.2, 3.6, 3.3,
    1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.6, 6.9, 7.4, 6.7, 4.6, 2.7,
    0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 8.6, 9.6, 10.6, 10.1, 8.0, 6.6, 5.0, 4.2, 2.7,
    0.0, 0.0, 0.0, 11.8, 13.0, 16.0, 15.1, 11.6, 9.7, 8.1, 8.2, 7.9, 4.9, 3.2,
    2.1, 15.6, 17.6, 23.0, 23.6, 22.1, 20.0, 16.0, 15.1, 12.1, 7.9, 6.2, 5.1,
    18.7, 21.5, 28.4, 30.2, 30.7, 31.0, 25.2, 23.1, 17.5, 10.7, 8.4, 7.2 },

  // Expression: 0
  //  Referenced by: '<S118>/White Noise'

  0.0,

  // Computed Parameter: WhiteNoise_StdDev_k
  //  Referenced by: '<S118>/White Noise'

  1.0,

  // Expression: 1
  //  Referenced by: '<S115>/Lv'

  1.0,

  // Expression: 1
  //  Referenced by: '<S115>/Lw'

  1.0,

  // Expression: inf
  //  Referenced by: '<S105>/3ft-->inf'

  0.0,

  // Expression: 3
  //  Referenced by: '<S105>/3ft-->inf'

  3.0,

  // Expression: 1/z0
  //  Referenced by: '<S105>/h//z0'

  6.666666666666667,

  // Expression: 20/z0
  //  Referenced by: '<S105>/ref_height//z0'

  133.33333333333334,

  // Expression: 0
  //  Referenced by: '<S105>/Wdeg1'

  0.0,

  // Expression: 1
  //  Referenced by: '<S415>/Zero-Order Hold1'

  1.0,

  // Expression: 1
  //  Referenced by: '<S415>/Zero-Order Hold2'

  1.0,

  // Expression: 1
  //  Referenced by: '<S415>/Zero-Order Hold'

  1.0,

  // Expression: [0 0 0]
  //  Referenced by: '<S398>/center of gravity'

  { 0.0, 0.0, 0.0 },

  // Expression: 1
  //  Referenced by: '<S415>/Zero-Order Hold4'

  1.0,

  // Expression: [1 -1 1]
  //  Referenced by: '<S415>/Gain'

  { 1.0, -1.0, 1.0 },

  // Expression: zeros(3,3)
  //  Referenced by: '<S1>/Constant_dI//dt'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  // Expression: 1
  //  Referenced by: '<S415>/Zero-Order Hold3'

  1.0,

  // Expression: 0.5
  //  Referenced by: '<S417>/Switch'

  0.5,

  // Expression: a_sath
  //  Referenced by: '<S415>/Saturation'

  { 160.0, 160.0, 160.0 },

  // Expression: a_satl
  //  Referenced by: '<S415>/Saturation'

  { -160.0, -160.0, -160.0 },

  // Expression: 0
  //  Referenced by: '<S432>/Integrator, Second-Order Limited'

  0.0,

  // Expression: 0
  //  Referenced by: '<S432>/Integrator, Second-Order Limited'

  0.0,

  // Expression: dtype_g
  //  Referenced by: '<S430>/Constant'

  1.0,

  // Expression: 1
  //  Referenced by: '<S416>/Zero-Order Hold'

  1.0,

  // Expression: 1
  //  Referenced by: '<S416>/Zero-Order Hold1'

  1.0,

  // Expression: 0.5
  //  Referenced by: '<S430>/Switch'

  0.5,

  // Expression: g_sath
  //  Referenced by: '<S416>/Saturation'

  { 4.36, 4.36, 4.36 },

  // Expression: g_satl
  //  Referenced by: '<S416>/Saturation'

  { -4.36, -4.36, -4.36 },

  // Expression: epoch
  //  Referenced by: '<S442>/epoch'

  2015.0,

  // Expression: dyear
  //  Referenced by: '<S411>/Decimal Year'

  2017.8383561643836,

  // Expression: -1000
  //  Referenced by: '<S456>/otime'

  -1000.0,

  // Expression: 180
  //  Referenced by: '<S438>/Constant'

  180.0,

  // Expression: 360
  //  Referenced by: '<S447>/Constant2'

  360.0,

  // Expression: 0
  //  Referenced by: '<S438>/Constant1'

  0.0,

  // Expression: 360
  //  Referenced by: '<S445>/Constant2'

  360.0,

  // Expression: -1000
  //  Referenced by: '<S455>/olon'

  -1000.0,

  // Expression: -1000
  //  Referenced by: '<S454>/olat'

  -1000.0,

  // Expression: 0.001
  //  Referenced by: '<S411>/Gain'

  0.001,

  // Expression: -1000
  //  Referenced by: '<S454>/oalt'

  -1000.0,

  // Expression: 6371.2
  //  Referenced by: '<S442>/re'

  6371.2,

  // Expression: 1
  //  Referenced by: '<S398>/Gain_Mag'

  1.0,

  // Expression: 1E-5
  //  Referenced by: '<S398>/nT2Gauss'

  1.0E-5,

  // Expression: T0
  //  Referenced by: '<S408>/Sea Level  Temperature'

  288.15,

  // Expression: T0
  //  Referenced by: '<S64>/Sea Level  Temperature'

  288.15,

  // Expression: h_trop
  //  Referenced by: '<S64>/Limit  altitude  to troposhere'

  11000.0,

  // Expression: h0
  //  Referenced by: '<S64>/Limit  altitude  to troposhere'

  0.0,

  // Expression: L
  //  Referenced by: '<S64>/Lapse Rate'

  0.0065,

  // Expression: 1/T0
  //  Referenced by: '<S64>/1//T0'

  0.00347041471455839,

  // Expression: g/(L*R)
  //  Referenced by: '<S64>/Constant'

  5.2558756014667134,

  // Expression: rho0
  //  Referenced by: '<S64>/rho0'

  1.225,

  // Expression: h_trop
  //  Referenced by: '<S64>/Altitude of Troposphere'

  11000.0,

  // Expression: 0
  //  Referenced by: '<S64>/Limit  altitude  to Stratosphere'

  0.0,

  // Expression: h_trop-h_strat
  //  Referenced by: '<S64>/Limit  altitude  to Stratosphere'

  -9000.0,

  // Expression: g/R
  //  Referenced by: '<S64>/g//R'

  0.034163191409533639,

  // Expression: 1/2
  //  Referenced by: '<S400>/1//2rhoV^2'

  0.5,

  // Expression: -2
  //  Referenced by: '<S398>/Uniform Random Number'

  -2.0,

  // Expression: 2
  //  Referenced by: '<S398>/Uniform Random Number'

  2.0,

  // Expression: 15634
  //  Referenced by: '<S398>/Uniform Random Number'

  15634.0,

  // Expression: 0.2
  //  Referenced by: '<S398>/Gain5'

  0.2,

  // Expression: 0.3
  //  Referenced by: '<S406>/Constant2'

  0.3,

  // Expression: h_trop
  //  Referenced by: '<S408>/Limit  altitude  to troposhere'

  11000.0,

  // Expression: h0
  //  Referenced by: '<S408>/Limit  altitude  to troposhere'

  0.0,

  // Expression: L
  //  Referenced by: '<S408>/Lapse Rate'

  0.0065,

  // Expression: 1/T0
  //  Referenced by: '<S408>/1//T0'

  0.00347041471455839,

  // Expression: g/(L*R)
  //  Referenced by: '<S408>/Constant'

  5.2558756014667134,

  // Expression: P0
  //  Referenced by: '<S408>/P0'

  101325.0,

  // Expression: h_trop
  //  Referenced by: '<S408>/Altitude of Troposphere'

  11000.0,

  // Expression: 0
  //  Referenced by: '<S408>/Limit  altitude  to Stratosphere'

  0.0,

  // Expression: h_trop-h_strat
  //  Referenced by: '<S408>/Limit  altitude  to Stratosphere'

  -9000.0,

  // Expression: g/R
  //  Referenced by: '<S408>/g//R'

  0.034163191409533639,

  // Expression: 0.01
  //  Referenced by: '<S398>/Gain'

  0.01,

  // Expression: -1
  //  Referenced by: '<S398>/Uniform Random Number4'

  -1.0,

  // Expression: 1
  //  Referenced by: '<S398>/Uniform Random Number4'

  1.0,

  // Expression: 25634
  //  Referenced by: '<S398>/Uniform Random Number4'

  25634.0,

  // Expression: 0.01
  //  Referenced by: '<S398>/Gain9'

  0.01,

  // Expression: 0.5
  //  Referenced by: '<S407>/Constant'

  0.5,

  // Expression: 0.7
  //  Referenced by: '<S407>/Gain2'

  0.7,

  // Expression: 0.3
  //  Referenced by: '<S407>/Constant2'

  0.3,

  // Expression: 0.01
  //  Referenced by: '<S398>/Gain1'

  0.01,

  // Expression: 10
  //  Referenced by: '<S5>/Constant1'

  10.0,

  // Expression: 8191
  //  Referenced by: '<S394>/Constant'

  8191.0,

  // Expression: [1 0 0 0]
  //  Referenced by: '<S321>/Merge'

  { 1.0, 0.0, 0.0, 0.0 },

  // Expression: 1E7
  //  Referenced by: '<S324>/Gain'

  1.0E+7,

  // Expression: 1E7
  //  Referenced by: '<S324>/Gain1'

  1.0E+7,

  // Expression: 1E3
  //  Referenced by: '<S324>/Gain3'

  1000.0,

  // Expression: 100
  //  Referenced by: '<S324>/Gain4'

  100.0,

  // Expression: 1/2
  //  Referenced by: '<S507>/1//2rhoV^2'

  0.5,

  // Expression: 2/1.225
  //  Referenced by: '<S324>/Gain2'

  1.6326530612244896,

  // Expression: 100
  //  Referenced by: '<S324>/Gain7'

  100.0,

  // Expression: 1
  //  Referenced by: '<S388>/Constant2'

  1.0,

  // Expression: R
  //  Referenced by: '<S388>/Constant1'

  6.378137E+6,

  // Expression: 1
  //  Referenced by: '<S391>/Constant'

  1.0,

  // Expression: 1
  //  Referenced by: '<S393>/Constant'

  1.0,

  // Expression: F
  //  Referenced by: '<S392>/Constant'

  0.0033528106647474805,

  // Expression: 1
  //  Referenced by: '<S392>/f'

  1.0,

  // Expression: 1
  //  Referenced by: '<S388>/Constant'

  1.0,

  // Expression: 1
  //  Referenced by: '<S388>/Constant3'

  1.0,

  // Expression: 360
  //  Referenced by: '<S379>/Constant2'

  360.0,

  // Expression: 180
  //  Referenced by: '<S372>/Constant'

  180.0,

  // Expression: 0
  //  Referenced by: '<S372>/Constant1'

  0.0,

  // Expression: 360
  //  Referenced by: '<S377>/Constant2'

  360.0,

  // Expression: 100000
  //  Referenced by: '<S366>/Saturation'

  100000.0,

  // Expression: 0
  //  Referenced by: '<S366>/Saturation'

  0.0,

  // Expression: 1E7
  //  Referenced by: '<S322>/latScale'

  1.0E+7,

  // Expression: 1E7
  //  Referenced by: '<S322>/lonScale'

  1.0E+7,

  // Expression: 1E3
  //  Referenced by: '<S322>/altScale'

  1000.0,

  // Expression: 100
  //  Referenced by: '<S322>/Gain6'

  100.0,

  // Expression: 100
  //  Referenced by: '<S322>/Gain8'

  100.0,

  // Computed Parameter: TransferFcn4_A
  //  Referenced by: '<S368>/Transfer Fcn4'

  -20.0,

  // Computed Parameter: TransferFcn4_C
  //  Referenced by: '<S368>/Transfer Fcn4'

  20.0,

  // Computed Parameter: TransferFcn1_A
  //  Referenced by: '<S368>/Transfer Fcn1'

  -20.0,

  // Computed Parameter: TransferFcn1_C
  //  Referenced by: '<S368>/Transfer Fcn1'

  20.0,

  // Computed Parameter: TransferFcn2_A
  //  Referenced by: '<S368>/Transfer Fcn2'

  -20.0,

  // Computed Parameter: TransferFcn2_C
  //  Referenced by: '<S368>/Transfer Fcn2'

  20.0,

  // Expression: 1E2
  //  Referenced by: '<S322>/VelScale'

  100.0,

  // Expression: 1E2
  //  Referenced by: '<S322>/VeScale'

  100.0,

  // Expression: 1E2
  //  Referenced by: '<S322>/AngleScale'

  100.0,

  // Expression: 1E3
  //  Referenced by: '<S322>/Gain1'

  1000.0,

  // Expression: 1E3
  //  Referenced by: '<S322>/Gain3'

  1000.0,

  // Expression: 1E3
  //  Referenced by: '<S322>/VeScale1'

  1000.0,

  // Expression: 1E3
  //  Referenced by: '<S322>/VelScale1'

  1000.0,

  // Expression: 1E5
  //  Referenced by: '<S322>/headMotAngle'

  100000.0,

  // Expression: 1E5
  //  Referenced by: '<S322>/headVehAngleScale'

  100000.0,

  // Expression: 100
  //  Referenced by: '<S398>/magDecGain'

  100.0,

  // Expression: 60/2/pi
  //  Referenced by: '<S12>/Gain'

  9.5492965855137211,

  // Expression: 1
  //  Referenced by: '<S12>/CopterID'

  1.0,

  // Expression: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
  //  Referenced by: '<S8>/Constant'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  // Expression: 1
  //  Referenced by: '<S26>/Constant'

  1.0,

  // Expression: [-1,-1,-2]
  //  Referenced by: '<S322>/Uniform Random Number4'

  { -1.0, -1.0, -2.0 },

  // Expression: [1,1,2]
  //  Referenced by: '<S322>/Uniform Random Number4'

  { 1.0, 1.0, 2.0 },

  // Expression: [5445,45433,33433]
  //  Referenced by: '<S322>/Uniform Random Number4'

  { 5445.0, 45433.0, 33433.0 },

  // Expression: 0.1
  //  Referenced by: '<S322>/BiasGain1'

  0.1,

  // Computed Parameter: Constant1_Value_l
  //  Referenced by: '<S468>/Constant1'

  1,

  // Computed Parameter: Constant_Value_op
  //  Referenced by: '<S469>/Constant'

  1,

  // Computed Parameter: Constant_Value_mh
  //  Referenced by: '<S467>/Constant'

  1,

  // Computed Parameter: Constant_Value_os
  //  Referenced by: '<S478>/Constant'

  1,

  // Computed Parameter: Gain_Gain_bd
  //  Referenced by: '<S478>/Gain'

  13,

  // Computed Parameter: Constant_Value_es
  //  Referenced by: '<S481>/Constant'

  1,

  // Computed Parameter: Gain_Gain_hz
  //  Referenced by: '<S480>/Gain'

  13,

  // Computed Parameter: Constant_Value_py
  //  Referenced by: '<S484>/Constant'

  1,

  // Computed Parameter: Constant1_Value_c
  //  Referenced by: '<S484>/Constant1'

  1,

  // Computed Parameter: Constant1_Value_fy
  //  Referenced by: '<S485>/Constant1'

  2,

  // Computed Parameter: Constant_Value_g2
  //  Referenced by: '<S483>/Constant'

  1,

  // Computed Parameter: Constant1_Value_m
  //  Referenced by: '<S482>/Constant1'

  1,

  // Computed Parameter: Gain_Gain_aa
  //  Referenced by: '<S482>/Gain'

  13,

  // Computed Parameter: Constant1_Value_hs
  //  Referenced by: '<S486>/Constant1'

  2,

  // Computed Parameter: Constant_Value_fi
  //  Referenced by: '<S460>/Constant'

  1,

  // Computed Parameter: Constant_Value_lp
  //  Referenced by: '<S477>/Constant'

  1,

  // Computed Parameter: Gain_Gain_ob
  //  Referenced by: '<S477>/Gain'

  13,

  // Computed Parameter: Constant_Value_g0
  //  Referenced by: '<S487>/Constant'

  1,

  // Computed Parameter: Constant1_Value_g
  //  Referenced by: '<S487>/Constant1'

  1,

  // Computed Parameter: Constant_Value_nf
  //  Referenced by: '<S489>/Constant'

  1,

  // Computed Parameter: tc_old_Threshold
  //  Referenced by: '<S488>/tc_old'

  1,

  // Computed Parameter: Constant_Value_j
  //  Referenced by: '<S459>/Constant'

  1,

  // Computed Parameter: Constant1_Value_ja
  //  Referenced by: '<S459>/Constant1'

  1,

  // Computed Parameter: Constant_Value_cy
  //  Referenced by: '<S458>/Constant'

  1,

  // Computed Parameter: Constant_Value_pe
  //  Referenced by: '<S463>/Constant'

  1,

  // Computed Parameter: Gain_Gain_ij
  //  Referenced by: '<S463>/Gain'

  13,

  // Computed Parameter: Constant_Value_a
  //  Referenced by: '<S465>/Constant'

  1,

  // Computed Parameter: ForIterator_IterationLimit_h
  //  Referenced by: '<S450>/For Iterator'

  12,

  // Computed Parameter: Constant_Value_lq
  //  Referenced by: '<S450>/Constant'

  1,

  // Computed Parameter: arn_Threshold
  //  Referenced by: '<S450>/ar(n)'

  1,

  // Expression: FaultParamStruct.LoadFallFaultID
  //  Referenced by: '<S187>/FaultID'

  123456,

  // Expression: FaultParamStruct.LoadShiftFaultID
  //  Referenced by: '<S187>/FaultID1'

  123457,

  // Expression: FaultParamStruct.LoadLeakFaultID
  //  Referenced by: '<S187>/FaultID2'

  123458,

  // Expression: FaultParamStruct.FaultID
  //  Referenced by: '<S510>/FaultID'

  123451,

  // Expression: FaultParamStruct.UseCusTomHoverTimeFaultID
  //  Referenced by: '<S57>/FaultID'

  123452,

  // Expression: FaultParamStruct.PowOffFaultID
  //  Referenced by: '<S57>/FaultID1'

  123453,

  // Expression: FaultParamStruct.LowVoltageFaultID
  //  Referenced by: '<S57>/FaultID2'

  123454,

  // Expression: FaultParamStruct.LowCapacityFaultID
  //  Referenced by: '<S57>/FaultID3'

  123455,

  // Expression: FaultParamStruct.FaultID
  //  Referenced by: '<S293>/FaultID'

  123450,

  // Expression: FaultParamStruct.NoiseWindFaultID
  //  Referenced by: '<S67>/FaultID4'

  123543,

  // Expression: FaultParamStruct.ConstWindFaultID
  //  Referenced by: '<S67>/FaultID'

  123459,

  // Expression: FaultParamStruct.GustWindFaultID
  //  Referenced by: '<S67>/FaultID1'

  123540,

  // Expression: FaultParamStruct.TurbWindFaultID
  //  Referenced by: '<S67>/FaultID2'

  123541,

  // Expression: FaultParamStruct.SheerWindFaultID
  //  Referenced by: '<S67>/FaultID3'

  123542,

  // Expression: FaultParamStruct.AccFaultID
  //  Referenced by: '<S398>/FaultID'

  123544,

  // Expression: FaultParamStruct.GyroFaultID
  //  Referenced by: '<S398>/FaultID1'

  123545,

  // Expression: FaultParamStruct.MagFaultID
  //  Referenced by: '<S398>/FaultID2'

  123546,

  // Expression: FaultParamStruct.BaroFaultID
  //  Referenced by: '<S398>/FaultID3'

  123547,

  // Expression: FaultParamStruct.GPSFaultID
  //  Referenced by: '<S366>/FaultID'

  123548,

  // Computed Parameter: Gain5_Gain_p
  //  Referenced by: '<S324>/Gain5'

  101.971619F,

  // Computed Parameter: MediumHighAltitudeIntensity_max
  //  Referenced by: '<S164>/Medium//High Altitude Intensity'

  { 11U, 6U },

  // Computed Parameter: MediumHighAltitudeIntensity_m_i
  //  Referenced by: '<S125>/Medium//High Altitude Intensity'

  { 11U, 6U },

  // Start of '<S512>/Distance into gust (z)'
  {
    // Expression: [0]
    //  Referenced by: '<S517>/x'

    0.0,

    // Expression: 0
    //  Referenced by: '<S517>/Distance into Gust (x) (Limited to gust length d) '

    0.0,

    // Expression: 0
    //  Referenced by: '<S517>/Distance into Gust (x) (Limited to gust length d) '

    0.0
  }
  ,

  // End of '<S512>/Distance into gust (z)'

  // Start of '<S512>/Distance into gust (y)'
  {
    // Expression: [0]
    //  Referenced by: '<S516>/x'

    0.0,

    // Expression: 0
    //  Referenced by: '<S516>/Distance into Gust (x) (Limited to gust length d) '

    0.0,

    // Expression: 0
    //  Referenced by: '<S516>/Distance into Gust (x) (Limited to gust length d) '

    0.0
  }
  ,

  // End of '<S512>/Distance into gust (y)'

  // Start of '<S329>/If Warning//Error'
  {
    // Expression: 0
    //  Referenced by: '<S355>/Constant1'

    0.0,

    // Expression: 0
    //  Referenced by: '<S354>/Constant1'

    0.0,

    // Expression: -1
    //  Referenced by: '<S357>/Bias'

    -1.0,

    // Expression: -eye(3)
    //  Referenced by: '<S356>/Bias1'

    { -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0 }
  }
  ,

  // End of '<S329>/If Warning//Error'

  // Start of '<S321>/Negative Trace'
  {
    // Expression: 1
    //  Referenced by: '<S344>/Constant'

    1.0,

    // Expression: 0.5
    //  Referenced by: '<S332>/Gain'

    0.5,

    // Expression: 0.5
    //  Referenced by: '<S343>/Constant1'

    0.5,

    // Expression: [0 1]
    //  Referenced by: '<S343>/Constant2'

    { 0.0, 1.0 },

    // Expression: 1
    //  Referenced by: '<S332>/Gain1'

    1.0,

    // Expression: 1
    //  Referenced by: '<S332>/Gain3'

    1.0,

    // Expression: 1
    //  Referenced by: '<S332>/Gain4'

    1.0,

    // Expression: 1
    //  Referenced by: '<S349>/Constant'

    1.0,

    // Expression: 0.5
    //  Referenced by: '<S333>/Gain'

    0.5,

    // Expression: 0.5
    //  Referenced by: '<S348>/Constant1'

    0.5,

    // Expression: [0 1]
    //  Referenced by: '<S348>/Constant2'

    { 0.0, 1.0 },

    // Expression: 1
    //  Referenced by: '<S333>/Gain1'

    1.0,

    // Expression: 1
    //  Referenced by: '<S333>/Gain2'

    1.0,

    // Expression: 1
    //  Referenced by: '<S333>/Gain3'

    1.0,

    // Expression: 1
    //  Referenced by: '<S339>/Constant'

    1.0,

    // Expression: 0.5
    //  Referenced by: '<S331>/Gain'

    0.5,

    // Expression: 0.5
    //  Referenced by: '<S338>/Constant1'

    0.5,

    // Expression: [0 1]
    //  Referenced by: '<S338>/Constant2'

    { 0.0, 1.0 },

    // Expression: 1
    //  Referenced by: '<S331>/Gain1'

    1.0,

    // Expression: 1
    //  Referenced by: '<S331>/Gain2'

    1.0,

    // Expression: 1
    //  Referenced by: '<S331>/Gain3'

    1.0
  }
  ,

  // End of '<S321>/Negative Trace'

  // Start of '<S321>/Positive Trace'
  {
    // Expression: 1
    //  Referenced by: '<S328>/Constant'

    1.0,

    // Expression: 0.5
    //  Referenced by: '<S328>/Gain'

    0.5,

    // Expression: 2
    //  Referenced by: '<S328>/Gain1'

    2.0
  }
  ,

  // End of '<S321>/Positive Trace'

  // Start of '<S294>/MotorNonlinearDynamic8'
  {
    // Expression: 0.95
    //  Referenced by: '<S304>/Signal_Saturation'

    0.95,

    // Expression: 0
    //  Referenced by: '<S304>/Signal_Saturation'

    0.0
  }
  ,

  // End of '<S294>/MotorNonlinearDynamic8'

  // Start of '<S294>/MotorNonlinearDynamic7'
  {
    // Expression: 0.95
    //  Referenced by: '<S303>/Signal_Saturation'

    0.95,

    // Expression: 0
    //  Referenced by: '<S303>/Signal_Saturation'

    0.0
  }
  ,

  // End of '<S294>/MotorNonlinearDynamic7'

  // Start of '<S294>/MotorNonlinearDynamic6'
  {
    // Expression: 0.95
    //  Referenced by: '<S302>/Signal_Saturation'

    0.95,

    // Expression: 0
    //  Referenced by: '<S302>/Signal_Saturation'

    0.0
  }
  ,

  // End of '<S294>/MotorNonlinearDynamic6'

  // Start of '<S294>/MotorNonlinearDynamic5'
  {
    // Expression: 0.95
    //  Referenced by: '<S301>/Signal_Saturation'

    0.95,

    // Expression: 0
    //  Referenced by: '<S301>/Signal_Saturation'

    0.0
  }
  ,

  // End of '<S294>/MotorNonlinearDynamic5'

  // Start of '<S294>/MotorNonlinearDynamic4'
  {
    // Expression: 0.95
    //  Referenced by: '<S300>/Signal_Saturation'

    0.95,

    // Expression: 0
    //  Referenced by: '<S300>/Signal_Saturation'

    0.0
  }
  ,

  // End of '<S294>/MotorNonlinearDynamic4'

  // Start of '<S294>/MotorNonlinearDynamic3'
  {
    // Expression: 0.95
    //  Referenced by: '<S299>/Signal_Saturation'

    0.95,

    // Expression: 0
    //  Referenced by: '<S299>/Signal_Saturation'

    0.0
  }
  ,

  // End of '<S294>/MotorNonlinearDynamic3'

  // Start of '<S294>/MotorNonlinearDynamic2'
  {
    // Expression: 0.95
    //  Referenced by: '<S298>/Signal_Saturation'

    0.95,

    // Expression: 0
    //  Referenced by: '<S298>/Signal_Saturation'

    0.0
  }
  ,

  // End of '<S294>/MotorNonlinearDynamic2'

  // Start of '<S294>/MotorNonlinearDynamic1'
  {
    // Expression: 0.95
    //  Referenced by: '<S297>/Signal_Saturation'

    0.95,

    // Expression: 0
    //  Referenced by: '<S297>/Signal_Saturation'

    0.0
  }
  ,

  // End of '<S294>/MotorNonlinearDynamic1'

  // Start of '<S241>/If Warning//Error'
  {
    // Expression: 0
    //  Referenced by: '<S267>/Constant1'

    0.0,

    // Expression: 0
    //  Referenced by: '<S266>/Constant1'

    0.0,

    // Expression: -1
    //  Referenced by: '<S269>/Bias'

    -1.0,

    // Expression: -eye(3)
    //  Referenced by: '<S268>/Bias1'

    { -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0 }
  }
  ,

  // End of '<S241>/If Warning//Error'

  // Start of '<S194>/Negative Trace'
  {
    // Expression: 1
    //  Referenced by: '<S256>/Constant'

    1.0,

    // Expression: 0.5
    //  Referenced by: '<S244>/Gain'

    0.5,

    // Expression: 0.5
    //  Referenced by: '<S255>/Constant1'

    0.5,

    // Expression: [0 1]
    //  Referenced by: '<S255>/Constant2'

    { 0.0, 1.0 },

    // Expression: 1
    //  Referenced by: '<S244>/Gain1'

    1.0,

    // Expression: 1
    //  Referenced by: '<S244>/Gain3'

    1.0,

    // Expression: 1
    //  Referenced by: '<S244>/Gain4'

    1.0,

    // Expression: 1
    //  Referenced by: '<S261>/Constant'

    1.0,

    // Expression: 0.5
    //  Referenced by: '<S245>/Gain'

    0.5,

    // Expression: 0.5
    //  Referenced by: '<S260>/Constant1'

    0.5,

    // Expression: [0 1]
    //  Referenced by: '<S260>/Constant2'

    { 0.0, 1.0 },

    // Expression: 1
    //  Referenced by: '<S245>/Gain1'

    1.0,

    // Expression: 1
    //  Referenced by: '<S245>/Gain2'

    1.0,

    // Expression: 1
    //  Referenced by: '<S245>/Gain3'

    1.0,

    // Expression: 1
    //  Referenced by: '<S251>/Constant'

    1.0,

    // Expression: 0.5
    //  Referenced by: '<S243>/Gain'

    0.5,

    // Expression: 0.5
    //  Referenced by: '<S250>/Constant1'

    0.5,

    // Expression: [0 1]
    //  Referenced by: '<S250>/Constant2'

    { 0.0, 1.0 },

    // Expression: 1
    //  Referenced by: '<S243>/Gain1'

    1.0,

    // Expression: 1
    //  Referenced by: '<S243>/Gain2'

    1.0,

    // Expression: 1
    //  Referenced by: '<S243>/Gain3'

    1.0
  }
  ,

  // End of '<S194>/Negative Trace'

  // Start of '<S194>/Positive Trace'
  {
    // Expression: 1
    //  Referenced by: '<S240>/Constant'

    1.0,

    // Expression: 0.5
    //  Referenced by: '<S240>/Gain'

    0.5,

    // Expression: 2
    //  Referenced by: '<S240>/Gain1'

    2.0
  }
  ,

  // End of '<S194>/Positive Trace'

  // Start of '<S203>/If Warning//Error'
  {
    // Expression: 0
    //  Referenced by: '<S229>/Constant1'

    0.0,

    // Expression: 0
    //  Referenced by: '<S228>/Constant1'

    0.0,

    // Expression: -1
    //  Referenced by: '<S231>/Bias'

    -1.0,

    // Expression: -eye(3)
    //  Referenced by: '<S230>/Bias1'

    { -1.0, -0.0, -0.0, -0.0, -1.0, -0.0, -0.0, -0.0, -1.0 }
  }
  ,

  // End of '<S203>/If Warning//Error'

  // Start of '<S193>/Negative Trace'
  {
    // Expression: 1
    //  Referenced by: '<S218>/Constant'

    1.0,

    // Expression: 0.5
    //  Referenced by: '<S206>/Gain'

    0.5,

    // Expression: 0.5
    //  Referenced by: '<S217>/Constant1'

    0.5,

    // Expression: [0 1]
    //  Referenced by: '<S217>/Constant2'

    { 0.0, 1.0 },

    // Expression: 1
    //  Referenced by: '<S206>/Gain1'

    1.0,

    // Expression: 1
    //  Referenced by: '<S206>/Gain3'

    1.0,

    // Expression: 1
    //  Referenced by: '<S206>/Gain4'

    1.0,

    // Expression: 1
    //  Referenced by: '<S223>/Constant'

    1.0,

    // Expression: 0.5
    //  Referenced by: '<S207>/Gain'

    0.5,

    // Expression: 0.5
    //  Referenced by: '<S222>/Constant1'

    0.5,

    // Expression: [0 1]
    //  Referenced by: '<S222>/Constant2'

    { 0.0, 1.0 },

    // Expression: 1
    //  Referenced by: '<S207>/Gain1'

    1.0,

    // Expression: 1
    //  Referenced by: '<S207>/Gain2'

    1.0,

    // Expression: 1
    //  Referenced by: '<S207>/Gain3'

    1.0,

    // Expression: 1
    //  Referenced by: '<S213>/Constant'

    1.0,

    // Expression: 0.5
    //  Referenced by: '<S205>/Gain'

    0.5,

    // Expression: 0.5
    //  Referenced by: '<S212>/Constant1'

    0.5,

    // Expression: [0 1]
    //  Referenced by: '<S212>/Constant2'

    { 0.0, 1.0 },

    // Expression: 1
    //  Referenced by: '<S205>/Gain1'

    1.0,

    // Expression: 1
    //  Referenced by: '<S205>/Gain2'

    1.0,

    // Expression: 1
    //  Referenced by: '<S205>/Gain3'

    1.0
  }
  ,

  // End of '<S193>/Negative Trace'

  // Start of '<S193>/Positive Trace'
  {
    // Expression: 1
    //  Referenced by: '<S202>/Constant'

    1.0,

    // Expression: 0.5
    //  Referenced by: '<S202>/Gain'

    0.5,

    // Expression: 2
    //  Referenced by: '<S202>/Gain1'

    2.0
  }
  ,

  // End of '<S193>/Positive Trace'

  // Start of '<S153>/Interpolate  velocities'
  {
    // Expression: max_height_low
    //  Referenced by: '<S174>/max_height_low'

    1000.0,

    // Expression: min_height_high
    //  Referenced by: '<S174>/min_height_high'

    2000.0
  }
  ,

  // End of '<S153>/Interpolate  velocities'

  // Start of '<S152>/Interpolate  rates'
  {
    // Expression: max_height_low
    //  Referenced by: '<S166>/max_height_low'

    1000.0,

    // Expression: min_height_high
    //  Referenced by: '<S166>/min_height_high'

    2000.0
  }
  ,

  // End of '<S152>/Interpolate  rates'

  // Start of '<S148>/Hwgw(z)'
  {
    // Expression: 0
    //  Referenced by: '<S163>/wgw'

    0.0,

    // Expression: 2*dt
    //  Referenced by: '<S163>/2'

    0.02,

    // Expression: 1
    //  Referenced by: '<S163>/Constant'

    1.0,

    // Expression: dt
    //  Referenced by: '<S163>/dt'

    0.01,

    // Expression: 0
    //  Referenced by: '<S163>/Unit Delay'

    0.0
  }
  ,

  // End of '<S148>/Hwgw(z)'

  // Start of '<S148>/Hvgw(z)'
  {
    // Expression: 0
    //  Referenced by: '<S162>/vgw'

    0.0,

    // Expression: 2*dt
    //  Referenced by: '<S162>/2'

    0.02,

    // Expression: 1
    //  Referenced by: '<S162>/Constant'

    1.0,

    // Expression: dt
    //  Referenced by: '<S162>/dt'

    0.01,

    // Expression: 0
    //  Referenced by: '<S162>/Unit Delay'

    0.0
  }
  ,

  // End of '<S148>/Hvgw(z)'

  // Start of '<S148>/Hugw(z)'
  {
    // Expression: 0
    //  Referenced by: '<S161>/ugw'

    0.0,

    // Expression: 2*dt
    //  Referenced by: '<S161>/2'

    0.02,

    // Expression: 1
    //  Referenced by: '<S161>/Constant'

    1.0,

    // Expression: dt
    //  Referenced by: '<S161>/dt'

    0.01,

    // Expression: 0
    //  Referenced by: '<S161>/Unit Delay'

    0.0
  }
  ,

  // End of '<S148>/Hugw(z)'

  // Start of '<S147>/Hrgw'
  {
    // Expression: 0
    //  Referenced by: '<S160>/rgw'

    0.0,

    // Expression: 1
    //  Referenced by: '<S160>/Constant'

    1.0,

    // Expression: 3/pi
    //  Referenced by: '<S160>/dt1'

    0.954929658551372,

    // Expression: dt
    //  Referenced by: '<S160>/dt'

    0.01,

    // Expression: 0
    //  Referenced by: '<S160>/Unit Delay'

    0.0,

    // Expression: 0
    //  Referenced by: '<S160>/Unit Delay1'

    0.0
  }
  ,

  // End of '<S147>/Hrgw'

  // Start of '<S147>/Hqgw'
  {
    // Expression: 0
    //  Referenced by: '<S159>/qgw'

    0.0,

    // Expression: 1
    //  Referenced by: '<S159>/Constant'

    1.0,

    // Expression: 4/pi
    //  Referenced by: '<S159>/dt1'

    1.2732395447351628,

    // Expression: dt
    //  Referenced by: '<S159>/dt'

    0.01,

    // Expression: 0
    //  Referenced by: '<S159>/Unit Delay'

    0.0,

    // Expression: 0
    //  Referenced by: '<S159>/Unit Delay1'

    0.0
  }
  ,

  // End of '<S147>/Hqgw'

  // Start of '<S147>/Hpgw'
  {
    // Expression: 0
    //  Referenced by: '<S158>/pgw'

    0.0,

    // Expression: 2.6
    //  Referenced by: '<S158>/Constant2'

    2.6,

    // Expression: 2*dt
    //  Referenced by: '<S158>/2'

    0.02,

    // Expression: 1
    //  Referenced by: '<S158>/Constant'

    1.0,

    // Expression: 0.95
    //  Referenced by: '<S158>/Constant1'

    0.95,

    // Expression: 1/3
    //  Referenced by: '<S158>/Constant3'

    0.33333333333333331,

    // Expression: dt
    //  Referenced by: '<S158>/dt'

    0.01,

    // Expression: 0
    //  Referenced by: '<S158>/Unit Delay'

    0.0
  }
  ,

  // End of '<S147>/Hpgw'

  // Start of '<S114>/Interpolate  velocities'
  {
    // Expression: max_height_low
    //  Referenced by: '<S135>/max_height_low'

    1000.0,

    // Expression: min_height_high
    //  Referenced by: '<S135>/min_height_high'

    2000.0
  }
  ,

  // End of '<S114>/Interpolate  velocities'

  // Start of '<S113>/Interpolate  rates'
  {
    // Expression: max_height_low
    //  Referenced by: '<S127>/max_height_low'

    1000.0,

    // Expression: min_height_high
    //  Referenced by: '<S127>/min_height_high'

    2000.0
  }
  ,

  // End of '<S113>/Interpolate  rates'

  // Start of '<S109>/Hwgw(z)'
  {
    // Expression: 0
    //  Referenced by: '<S124>/wgw'

    0.0,

    // Expression: 2*dt
    //  Referenced by: '<S124>/2'

    0.02,

    // Expression: 1
    //  Referenced by: '<S124>/Constant'

    1.0,

    // Expression: dt
    //  Referenced by: '<S124>/dt'

    0.01,

    // Expression: 0
    //  Referenced by: '<S124>/Unit Delay'

    0.0
  }
  ,

  // End of '<S109>/Hwgw(z)'

  // Start of '<S109>/Hvgw(z)'
  {
    // Expression: 0
    //  Referenced by: '<S123>/vgw'

    0.0,

    // Expression: 2*dt
    //  Referenced by: '<S123>/2'

    0.02,

    // Expression: 1
    //  Referenced by: '<S123>/Constant'

    1.0,

    // Expression: dt
    //  Referenced by: '<S123>/dt'

    0.01,

    // Expression: 0
    //  Referenced by: '<S123>/Unit Delay'

    0.0
  }
  ,

  // End of '<S109>/Hvgw(z)'

  // Start of '<S109>/Hugw(z)'
  {
    // Expression: 0
    //  Referenced by: '<S122>/ugw'

    0.0,

    // Expression: 2*dt
    //  Referenced by: '<S122>/2'

    0.02,

    // Expression: 1
    //  Referenced by: '<S122>/Constant'

    1.0,

    // Expression: dt
    //  Referenced by: '<S122>/dt'

    0.01,

    // Expression: 0
    //  Referenced by: '<S122>/Unit Delay'

    0.0
  }
  ,

  // End of '<S109>/Hugw(z)'

  // Start of '<S108>/Hrgw'
  {
    // Expression: 0
    //  Referenced by: '<S121>/rgw'

    0.0,

    // Expression: 1
    //  Referenced by: '<S121>/Constant'

    1.0,

    // Expression: 3/pi
    //  Referenced by: '<S121>/dt1'

    0.954929658551372,

    // Expression: dt
    //  Referenced by: '<S121>/dt'

    0.01,

    // Expression: 0
    //  Referenced by: '<S121>/Unit Delay'

    0.0,

    // Expression: 0
    //  Referenced by: '<S121>/Unit Delay1'

    0.0
  }
  ,

  // End of '<S108>/Hrgw'

  // Start of '<S108>/Hqgw'
  {
    // Expression: 0
    //  Referenced by: '<S120>/qgw'

    0.0,

    // Expression: 1
    //  Referenced by: '<S120>/Constant'

    1.0,

    // Expression: 4/pi
    //  Referenced by: '<S120>/dt1'

    1.2732395447351628,

    // Expression: dt
    //  Referenced by: '<S120>/dt'

    0.01,

    // Expression: 0
    //  Referenced by: '<S120>/Unit Delay'

    0.0,

    // Expression: 0
    //  Referenced by: '<S120>/Unit Delay1'

    0.0
  }
  ,

  // End of '<S108>/Hqgw'

  // Start of '<S108>/Hpgw'
  {
    // Expression: 0
    //  Referenced by: '<S119>/pgw'

    0.0,

    // Expression: 2.6
    //  Referenced by: '<S119>/Constant2'

    2.6,

    // Expression: 2*dt
    //  Referenced by: '<S119>/2'

    0.02,

    // Expression: 1
    //  Referenced by: '<S119>/Constant'

    1.0,

    // Expression: 0.95
    //  Referenced by: '<S119>/Constant1'

    0.95,

    // Expression: 1/3
    //  Referenced by: '<S119>/Constant3'

    0.33333333333333331,

    // Expression: dt
    //  Referenced by: '<S119>/dt'

    0.01,

    // Expression: 0
    //  Referenced by: '<S119>/Unit Delay'

    0.0
  }
  // End of '<S108>/Hpgw'
};

extern real_T rt_powd_snf(real_T u0, real_T u1);
extern real_T rt_modd_snf(real_T u0, real_T u1);
extern real_T rt_atan2d_snf(real_T u0, real_T u1);
extern void rt_mrdivide_U1d1x3_U2d_9vOrDY9Z(const real_T u0[3], const real_T u1
  [9], real_T y[3]);
extern real_T rt_roundd_snf(real_T u);
extern real32_T rt_roundf_snf(real32_T u);
extern real_T rt_urand_Upu32_Yd_f_pw_snf(uint32_T *u);
extern real_T rt_nrand_Upu32_Yd_f_pw_snf(uint32_T *u);
void wgs84_taylor_series(real_T *h, real_T *phi, real_T opt_m2ft, real_T *y,
  int_T k);
static uint32_T plook_bincpa(real_T u, const real_T bp[], uint32_T maxIndex,
  real_T *fraction, uint32_T *prevIndex);
static real_T intrp2d_la_pw(const uint32_T bpIndex[], const real_T frac[], const
  real_T table[], const uint32_T stride, const uint32_T maxIndex[]);
static uint32_T binsearch_u32d_prevIdx(real_T u, const real_T bp[], uint32_T
  startIndex, uint32_T maxIndex);
static void mul_wide_s32(int32_T in0, int32_T in1, uint32_T *ptrOutBitsHi,
  uint32_T *ptrOutBitsLo);
static int32_T mul_s32_sat(int32_T a, int32_T b);

// private model entry point functions
extern void QuadModelv_derivatives();
static void rate_scheduler(RT_MODEL_QuadModelv_T *const QuadModelv_M);

//===========*
//  Constants *
// ===========
#define RT_PI                          3.14159265358979323846
#define RT_PIF                         3.1415927F
#define RT_LN_10                       2.30258509299404568402
#define RT_LN_10F                      2.3025851F
#define RT_LOG10E                      0.43429448190325182765
#define RT_LOG10EF                     0.43429449F
#define RT_E                           2.7182818284590452354
#define RT_EF                          2.7182817F

//
//  UNUSED_PARAMETER(x)
//    Used to specify that a function parameter (argument) is required but not
//    accessed by the function body.

#ifndef UNUSED_PARAMETER
#if defined(__LCC__)
#define UNUSED_PARAMETER(x)                                      // do nothing
#else

//
//  This is the semi-ANSI standard way of indicating that an
//  unused function parameter is required.

#define UNUSED_PARAMETER(x)            (void) (x)
#endif
#endif

extern "C"
{
  real_T rtInf;
  real_T rtMinusInf;
  real_T rtNaN;
  real32_T rtInfF;
  real32_T rtMinusInfF;
  real32_T rtNaNF;
}

//=========*
//  Asserts *
// =========
#ifndef utAssert
#if defined(DOASSERTS)
#if !defined(PRINT_ASSERTS)
#include <assert.h>
#define utAssert(exp)                  assert(exp)
#else
#include <stdio.h>

static void _assert(char_T *statement, char_T *file, int line)
{
  printf("%s in %s on line %d\n", statement, file, line);
}

#define utAssert(_EX)                  ((_EX) ? (void)0 : _assert(#_EX, __FILE__, __LINE__))
#endif

#else
#define utAssert(exp)                                            // do nothing
#endif
#endif

extern "C"
{
  //
  // Initialize rtInf needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real_T rtGetInf(void)
  {
    size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
    real_T inf = 0.0;
    if (bitsPerReal == 32U) {
      inf = rtGetInfF();
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0x7FF00000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      inf = tmpVal.fltVal;
    }

    return inf;
  }

  //
  // Initialize rtInfF needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real32_T rtGetInfF(void)
  {
    IEEESingle infF;
    infF.wordL.wordLuint = 0x7F800000U;
    return infF.wordL.wordLreal;
  }

  //
  // Initialize rtMinusInf needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real_T rtGetMinusInf(void)
  {
    size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
    real_T minf = 0.0;
    if (bitsPerReal == 32U) {
      minf = rtGetMinusInfF();
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0xFFF00000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      minf = tmpVal.fltVal;
    }

    return minf;
  }

  //
  // Initialize rtMinusInfF needed by the generated code.
  // Inf is initialized as non-signaling. Assumes IEEE.
  //
  static real32_T rtGetMinusInfF(void)
  {
    IEEESingle minfF;
    minfF.wordL.wordLuint = 0xFF800000U;
    return minfF.wordL.wordLreal;
  }
}

extern "C"
{
  //
  // Initialize rtNaN needed by the generated code.
  // NaN is initialized as non-signaling. Assumes IEEE.
  //
  static real_T rtGetNaN(void)
  {
    size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
    real_T nan = 0.0;
    if (bitsPerReal == 32U) {
      nan = rtGetNaNF();
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.bitVal.words.wordH = 0xFFF80000U;
      tmpVal.bitVal.words.wordL = 0x00000000U;
      nan = tmpVal.fltVal;
    }

    return nan;
  }

  //
  // Initialize rtNaNF needed by the generated code.
  // NaN is initialized as non-signaling. Assumes IEEE.
  //
  static real32_T rtGetNaNF(void)
  {
    IEEESingle nanF = { { 0.0F } };

    nanF.wordL.wordLuint = 0xFFC00000U;
    return nanF.wordL.wordLreal;
  }
}

extern "C"
{
  //
  // Initialize the rtInf, rtMinusInf, and rtNaN needed by the
  // generated code. NaN is initialized as non-signaling. Assumes IEEE.
  //
  static void rt_InitInfAndNaN(size_t realSize)
  {
    (void) (realSize);
    rtNaN = rtGetNaN();
    rtNaNF = rtGetNaNF();
    rtInf = rtGetInf();
    rtInfF = rtGetInfF();
    rtMinusInf = rtGetMinusInf();
    rtMinusInfF = rtGetMinusInfF();
  }

  // Test if value is infinite
  static boolean_T rtIsInf(real_T value)
  {
    return (boolean_T)((value==rtInf || value==rtMinusInf) ? 1U : 0U);
  }

  // Test if single-precision value is infinite
  static boolean_T rtIsInfF(real32_T value)
  {
    return (boolean_T)(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
  }

  // Test if value is not a number
  static boolean_T rtIsNaN(real_T value)
  {
    boolean_T result = (boolean_T) 0;
    size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
    if (bitsPerReal == 32U) {
      result = rtIsNaNF((real32_T)value);
    } else {
      union {
        LittleEndianIEEEDouble bitVal;
        real_T fltVal;
      } tmpVal;

      tmpVal.fltVal = value;
      result = (boolean_T)((tmpVal.bitVal.words.wordH & 0x7FF00000) ==
                           0x7FF00000 &&
                           ( (tmpVal.bitVal.words.wordH & 0x000FFFFF) != 0 ||
                            (tmpVal.bitVal.words.wordL != 0) ));
    }

    return result;
  }

  // Test if single-precision value is not a number
  static boolean_T rtIsNaNF(real32_T value)
  {
    IEEESingle tmp;
    tmp.wordL.wordLreal = value;
    return (boolean_T)( (tmp.wordL.wordLuint & 0x7F800000) == 0x7F800000 &&
                       (tmp.wordL.wordLuint & 0x007FFFFF) != 0 );
  }
}

//
//         Taylor Series expansion approximation of WGS84 model of
//         ellipsoid normal gravity
//
void wgs84_taylor_series(real_T *h, real_T *phi, real_T opt_m2ft, real_T *y,
  int_T k)
{
  real_T gamma_ts, m, sinphi, sin2phi;
  int_T i;
  for (i = 0; i < k; i++ ) {
    sinphi = std::sin(phi[i]);
    sin2phi = sinphi*sinphi;

    // Calculate theoretical normal gravity (gamma) /eq. 4-1/
    gamma_ts = (WGS84_G_E)*( 1.0 + (WGS84_K)*sin2phi )/( std::sqrt(1.0 -
      (WGS84_E_2)*sin2phi) );
    m = (WGS84_A)*(WGS84_A)*(WGS84_B)*(WGS84_W_DEF)*(WGS84_W_DEF)/
      (WGS84_GM_DEF);

    // Return normal gravity as the output /eq. 4-3/
    y[i] = opt_m2ft*gamma_ts*( 1.0 - 2.0*( 1.0 + 1.0/(WGS84_INV_F) + m -
      2.0*sin2phi/(WGS84_INV_F) )*h[i]/(WGS84_A) + 3.0*h[i]*h[i]/(
      (WGS84_A)*(WGS84_A)) );
  }
}

static uint32_T plook_bincpa(real_T u, const real_T bp[], uint32_T maxIndex,
  real_T *fraction, uint32_T *prevIndex)
{
  uint32_T bpIndex;

  // Prelookup - Index and Fraction
  // Index Search method: 'binary'
  // Extrapolation method: 'Clip'
  // Use previous index: 'on'
  // Use last breakpoint for index at or above upper limit: 'on'
  // Remove protection against out-of-range input in generated code: 'off'

  if (u <= bp[0U]) {
    bpIndex = 0U;
    *fraction = 0.0;
  } else if (u < bp[maxIndex]) {
    bpIndex = binsearch_u32d_prevIdx(u, bp, *prevIndex, maxIndex);
    *fraction = (u - bp[bpIndex]) / (bp[bpIndex + 1U] - bp[bpIndex]);
  } else {
    bpIndex = maxIndex;
    *fraction = 0.0;
  }

  *prevIndex = bpIndex;
  return bpIndex;
}

static real_T intrp2d_la_pw(const uint32_T bpIndex[], const real_T frac[], const
  real_T table[], const uint32_T stride, const uint32_T maxIndex[])
{
  real_T y;
  real_T yL_0d0;
  uint32_T offset_1d;

  // Column-major Interpolation 2-D
  // Interpolation method: 'Linear point-slope'
  // Use last breakpoint for index at or above upper limit: 'on'
  // Overflow mode: 'portable wrapping'

  offset_1d = bpIndex[1U] * stride + bpIndex[0U];
  if (bpIndex[0U] == maxIndex[0U]) {
    y = table[offset_1d];
  } else {
    yL_0d0 = table[offset_1d];
    y = (table[offset_1d + 1U] - yL_0d0) * frac[0U] + yL_0d0;
  }

  if (bpIndex[1U] == maxIndex[1U]) {
  } else {
    offset_1d += stride;
    if (bpIndex[0U] == maxIndex[0U]) {
      yL_0d0 = table[offset_1d];
    } else {
      yL_0d0 = table[offset_1d];
      yL_0d0 += (table[offset_1d + 1U] - yL_0d0) * frac[0U];
    }

    y += (yL_0d0 - y) * frac[1U];
  }

  return y;
}

static uint32_T binsearch_u32d_prevIdx(real_T u, const real_T bp[], uint32_T
  startIndex, uint32_T maxIndex)
{
  uint32_T bpIndex;
  uint32_T found;
  uint32_T iLeft;
  uint32_T iRght;

  // Binary Search using Previous Index
  bpIndex = startIndex;
  iLeft = 0U;
  iRght = maxIndex;
  found = 0U;
  while (found == 0U) {
    if (u < bp[bpIndex]) {
      iRght = bpIndex - 1U;
      bpIndex = ((bpIndex + iLeft) - 1U) >> 1U;
    } else if (u < bp[bpIndex + 1U]) {
      found = 1U;
    } else {
      iLeft = bpIndex + 1U;
      bpIndex = ((bpIndex + iRght) + 1U) >> 1U;
    }
  }

  return bpIndex;
}

static void mul_wide_s32(int32_T in0, int32_T in1, uint32_T *ptrOutBitsHi,
  uint32_T *ptrOutBitsLo)
{
  uint32_T absIn0;
  uint32_T absIn1;
  uint32_T in0Hi;
  uint32_T in0Lo;
  uint32_T in1Hi;
  uint32_T productHiLo;
  uint32_T productLoHi;
  absIn0 = in0 < 0 ? ~static_cast<uint32_T>(in0) + 1U : static_cast<uint32_T>
    (in0);
  absIn1 = in1 < 0 ? ~static_cast<uint32_T>(in1) + 1U : static_cast<uint32_T>
    (in1);
  in0Hi = absIn0 >> 16U;
  in0Lo = absIn0 & 65535U;
  in1Hi = absIn1 >> 16U;
  absIn0 = absIn1 & 65535U;
  productHiLo = in0Hi * absIn0;
  productLoHi = in0Lo * in1Hi;
  absIn0 *= in0Lo;
  absIn1 = 0U;
  in0Lo = (productLoHi << /*MW:OvBitwiseOk*/ 16U) + /*MW:OvCarryOk*/ absIn0;
  if (in0Lo < absIn0) {
    absIn1 = 1U;
  }

  absIn0 = in0Lo;
  in0Lo += /*MW:OvCarryOk*/ productHiLo << /*MW:OvBitwiseOk*/ 16U;
  if (in0Lo < absIn0) {
    absIn1++;
  }

  absIn0 = (((productLoHi >> 16U) + (productHiLo >> 16U)) + in0Hi * in1Hi) +
    absIn1;
  if (static_cast<int32_T>((in0 != 0) && ((in1 != 0) && ((in0 > 0) != (in1 > 0)))))
  {
    absIn0 = ~absIn0;
    in0Lo = ~in0Lo;
    in0Lo++;
    if (in0Lo == 0U) {
      absIn0++;
    }
  }

  *ptrOutBitsHi = absIn0;
  *ptrOutBitsLo = in0Lo;
}

static int32_T mul_s32_sat(int32_T a, int32_T b)
{
  int32_T result;
  uint32_T u32_chi;
  uint32_T u32_clo;
  mul_wide_s32(a, b, &u32_chi, &u32_clo);
  if ((static_cast<int32_T>(u32_chi) > 0) || ((u32_chi == 0U) && (u32_clo >=
        2147483648U))) {
    result = MAX_int32_T;
  } else if ((static_cast<int32_T>(u32_chi) < -1) || ((static_cast<int32_T>
               (u32_chi) == -1) && (u32_clo < 2147483648U))) {
    result = MIN_int32_T;
  } else {
    result = static_cast<int32_T>(u32_clo);
  }

  return result;
}

//
//         This function updates active task flag for each subrate.
//         The function is called at model base rate, hence the
//         generated code self-manages all its subrates.
//
static void rate_scheduler(RT_MODEL_QuadModelv_T *const QuadModelv_M)
{
  // Compute which subrates run during the next base time step.  Subrates
  //  are an integer multiple of the base rate counter.  Therefore, the subtask
  //  counter is reset when it reaches its limit (zero means run).

  (QuadModelv_M->Timing.TaskCounters.TID[2])++;
  if ((QuadModelv_M->Timing.TaskCounters.TID[2]) > 2) {// Sample time: [0.003s, 0.0s] 
    QuadModelv_M->Timing.TaskCounters.TID[2] = 0;
  }

  (QuadModelv_M->Timing.TaskCounters.TID[3])++;
  if ((QuadModelv_M->Timing.TaskCounters.TID[3]) > 9) {// Sample time: [0.01s, 0.0s] 
    QuadModelv_M->Timing.TaskCounters.TID[3] = 0;
  }

  (QuadModelv_M->Timing.TaskCounters.TID[4])++;
  if ((QuadModelv_M->Timing.TaskCounters.TID[4]) > 99) {// Sample time: [0.1s, 0.0s] 
    QuadModelv_M->Timing.TaskCounters.TID[4] = 0;
  }

  (QuadModelv_M->Timing.TaskCounters.TID[5])++;
  if ((QuadModelv_M->Timing.TaskCounters.TID[5]) > 199) {// Sample time: [0.2s, 0.0s] 
    QuadModelv_M->Timing.TaskCounters.TID[5] = 0;
  }
}

//
// This function updates continuous states using the ODE4 fixed-step
// solver algorithm
//
void MulticopterModelClass::rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE4_IntgData *id = static_cast<ODE4_IntgData *>(rtsiGetSolverData(si));
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T *f3 = id->f[3];
  real_T temp;
  int_T i;
  int_T nXc = 39;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  // Save the state values at time t in y, we'll use x as ynew.
  (void) std::memcpy(y, x,
                     static_cast<uint_T>(nXc)*sizeof(real_T));

  // Assumes that rtsiSetT and ModelOutputs are up-to-date
  // f0 = f(t,y)
  rtsiSetdX(si, f0);
  QuadModelv_derivatives();

  // f1 = f(t + (h/2), y + (h/2)*f0)
  temp = 0.5 * h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f0[i]);
  }

  rtsiSetT(si, t + temp);
  rtsiSetdX(si, f1);
  this->step();
  QuadModelv_derivatives();

  // f2 = f(t + (h/2), y + (h/2)*f1)
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f1[i]);
  }

  rtsiSetdX(si, f2);
  this->step();
  QuadModelv_derivatives();

  // f3 = f(t + h, y + h*f2)
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f2[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f3);
  this->step();
  QuadModelv_derivatives();

  // tnew = t + h
  // ynew = y + (h/6)*(f0 + 2*f1 + 2*f2 + 2*f3)
  temp = h / 6.0;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + temp*(f0[i] + 2.0*f1[i] + 2.0*f2[i] + f3[i]);
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    real_T tmp;
    real_T tmp_0;
    tmp = std::abs(u0);
    tmp_0 = std::abs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = (rtNaN);
    } else {
      y = std::pow(u0, u1);
    }
  }

  return y;
}

//
// System initialize for enable system:
//    '<S108>/Hpgw'
//    '<S147>/Hpgw'
//
void MulticopterModelClass::QuadModelv_Hpgw_Init(B_Hpgw_QuadModelv_T *localB,
  DW_Hpgw_QuadModelv_T *localDW, P_Hpgw_QuadModelv_T *localP)
{
  // InitializeConditions for UnitDelay: '<S119>/Unit Delay'
  localDW->UnitDelay_DSTATE[0] = localP->UnitDelay_InitialCondition;

  // SystemInitialize for Sum: '<S119>/Sum' incorporates:
  //   Outport: '<S119>/pgw'

  localB->Sum[0] = localP->pgw_Y0;

  // InitializeConditions for UnitDelay: '<S119>/Unit Delay'
  localDW->UnitDelay_DSTATE[1] = localP->UnitDelay_InitialCondition;

  // SystemInitialize for Sum: '<S119>/Sum' incorporates:
  //   Outport: '<S119>/pgw'

  localB->Sum[1] = localP->pgw_Y0;
}

//
// System reset for enable system:
//    '<S108>/Hpgw'
//    '<S147>/Hpgw'
//
void MulticopterModelClass::QuadModelv_Hpgw_Reset(DW_Hpgw_QuadModelv_T *localDW,
  P_Hpgw_QuadModelv_T *localP)
{
  // InitializeConditions for UnitDelay: '<S119>/Unit Delay'
  localDW->UnitDelay_DSTATE[0] = localP->UnitDelay_InitialCondition;
  localDW->UnitDelay_DSTATE[1] = localP->UnitDelay_InitialCondition;
}

//
// Disable for enable system:
//    '<S108>/Hpgw'
//    '<S147>/Hpgw'
//
void MulticopterModelClass::QuadModelv_Hpgw_Disable(B_Hpgw_QuadModelv_T *localB,
  DW_Hpgw_QuadModelv_T *localDW, P_Hpgw_QuadModelv_T *localP)
{
  // Disable for Sum: '<S119>/Sum' incorporates:
  //   Outport: '<S119>/pgw'

  localB->Sum[0] = localP->pgw_Y0;
  localB->Sum[1] = localP->pgw_Y0;
  localDW->Hpgw_MODE = false;
}

//
// Outputs for enable system:
//    '<S108>/Hpgw'
//    '<S147>/Hpgw'
//
void MulticopterModelClass::QuadModelv_Hpgw(real_T rtu_Enable, const real_T
  rtu_L_wg[2], real_T rtu_sigma_wg, real_T rtu_sigma_wg_e, real_T rtu_Noise,
  real_T rtu_wingspan, B_Hpgw_QuadModelv_T *localB, DW_Hpgw_QuadModelv_T
  *localDW, P_Hpgw_QuadModelv_T *localP)
{
  real_T rtb_ap_idx_0;
  real_T rtb_ap_idx_1;
  real_T rtb_sp_0;
  real_T rtb_sp_idx_0;
  real_T u0;

  // Outputs for Enabled SubSystem: '<S108>/Hpgw' incorporates:
  //   EnablePort: '<S119>/Enable'

  if ((rtmIsMajorTimeStep((&QuadModelv_M)) &&
       (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) &&
      rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    if (rtu_Enable > 0.0) {
      if (!localDW->Hpgw_MODE) {
        QuadModelv_Hpgw_Reset(localDW, localP);
        localDW->Hpgw_MODE = true;
      }
    } else if (localDW->Hpgw_MODE) {
      QuadModelv_Hpgw_Disable(localB, localDW, localP);
    }
  }

  if (localDW->Hpgw_MODE) {
    // Product: '<S119>/w2'
    rtb_sp_0 = rtu_L_wg[0] * rtu_wingspan;
    rtb_sp_idx_0 = rtb_sp_0;

    // Product: '<S119>/w1' incorporates:
    //   Constant: '<S119>/Constant2'
    //   Sqrt: '<S119>/sqrt'

    rtb_ap_idx_0 = localP->Constant2_Value / std::sqrt(rtb_sp_0);

    // Product: '<S119>/w2'
    rtb_sp_0 = rtu_L_wg[1] * rtu_wingspan;

    // Product: '<S119>/w1' incorporates:
    //   Constant: '<S119>/Constant2'
    //   Sqrt: '<S119>/sqrt'

    rtb_ap_idx_1 = localP->Constant2_Value / std::sqrt(rtb_sp_0);
    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0) {
      // UnitDelay: '<S119>/Unit Delay'
      localB->UnitDelay[0] = localDW->UnitDelay_DSTATE[0];
      localB->UnitDelay[1] = localDW->UnitDelay_DSTATE[1];
    }

    // Product: '<S119>/w4'
    u0 = rtb_sp_idx_0 * rtu_wingspan;

    // Math: '<S119>/Math Function' incorporates:
    //   Constant: '<S119>/Constant3'

    rtb_sp_idx_0 = std::floor(localP->Constant3_Value);
    if ((u0 < 0.0) && (localP->Constant3_Value > rtb_sp_idx_0)) {
      u0 = -rt_powd_snf(-u0, localP->Constant3_Value);
    } else {
      u0 = rt_powd_snf(u0, localP->Constant3_Value);
    }

    // Sum: '<S119>/Sum' incorporates:
    //   Constant: '<S119>/Constant'
    //   Constant: '<S119>/Constant1'
    //   Gain: '<S119>/2'
    //   Gain: '<S119>/dt'
    //   Math: '<S119>/Math Function'
    //   Product: '<S119>/Lug//V1'
    //   Product: '<S119>/Lug//V2'
    //   Product: '<S119>/w3'
    //   Sqrt: '<S119>/sqrt1'
    //   Sum: '<S119>/Sum1'

    localB->Sum[0] = localP->Constant1_Value / u0 * rtu_sigma_wg * std::sqrt
      (localP->u_Gain * rtb_ap_idx_0) * rtu_Noise + (localP->Constant_Value -
      localP->dt_Gain * rtb_ap_idx_0) * localB->UnitDelay[0];

    // Product: '<S119>/w4' incorporates:
    //   Product: '<S119>/w2'

    u0 = rtb_sp_0 * rtu_wingspan;

    // Math: '<S119>/Math Function' incorporates:
    //   Constant: '<S119>/Constant3'

    if ((u0 < 0.0) && (localP->Constant3_Value > rtb_sp_idx_0)) {
      rtb_sp_idx_0 = -rt_powd_snf(-u0, localP->Constant3_Value);
    } else {
      rtb_sp_idx_0 = rt_powd_snf(u0, localP->Constant3_Value);
    }

    // Sum: '<S119>/Sum' incorporates:
    //   Constant: '<S119>/Constant'
    //   Constant: '<S119>/Constant1'
    //   Gain: '<S119>/2'
    //   Gain: '<S119>/dt'
    //   Math: '<S119>/Math Function'
    //   Product: '<S119>/Lug//V1'
    //   Product: '<S119>/Lug//V2'
    //   Product: '<S119>/w3'
    //   Sqrt: '<S119>/sqrt1'
    //   Sum: '<S119>/Sum1'

    localB->Sum[1] = localP->Constant1_Value / rtb_sp_idx_0 * rtu_sigma_wg_e *
      std::sqrt(localP->u_Gain * rtb_ap_idx_1) * rtu_Noise +
      (localP->Constant_Value - localP->dt_Gain * rtb_ap_idx_1) *
      localB->UnitDelay[1];
  }

  // End of Outputs for SubSystem: '<S108>/Hpgw'
}

//
// Update for enable system:
//    '<S108>/Hpgw'
//    '<S147>/Hpgw'
//
void MulticopterModelClass::QuadModelv_Hpgw_Update(B_Hpgw_QuadModelv_T *localB,
  DW_Hpgw_QuadModelv_T *localDW)
{
  // Update for Enabled SubSystem: '<S108>/Hpgw' incorporates:
  //   EnablePort: '<S119>/Enable'

  if (localDW->Hpgw_MODE && (rtmIsMajorTimeStep((&QuadModelv_M)) &&
       (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0)) {
    // Update for UnitDelay: '<S119>/Unit Delay'
    localDW->UnitDelay_DSTATE[0] = localB->Sum[0];
    localDW->UnitDelay_DSTATE[1] = localB->Sum[1];
  }

  // End of Update for SubSystem: '<S108>/Hpgw'
}

//
// System initialize for enable system:
//    '<S108>/Hqgw'
//    '<S147>/Hqgw'
//
void MulticopterModelClass::QuadModelv_Hqgw_Init(B_Hqgw_QuadModelv_T *localB,
  DW_Hqgw_QuadModelv_T *localDW, P_Hqgw_QuadModelv_T *localP)
{
  // InitializeConditions for UnitDelay: '<S120>/Unit Delay'
  localDW->UnitDelay_DSTATE[0] = localP->UnitDelay_InitialCondition;

  // InitializeConditions for UnitDelay: '<S120>/Unit Delay1'
  localDW->UnitDelay1_DSTATE[0] = localP->UnitDelay1_InitialCondition;

  // SystemInitialize for Sum: '<S120>/Sum1' incorporates:
  //   Outport: '<S120>/qgw'

  localB->Sum1[0] = localP->qgw_Y0;

  // InitializeConditions for UnitDelay: '<S120>/Unit Delay'
  localDW->UnitDelay_DSTATE[1] = localP->UnitDelay_InitialCondition;

  // InitializeConditions for UnitDelay: '<S120>/Unit Delay1'
  localDW->UnitDelay1_DSTATE[1] = localP->UnitDelay1_InitialCondition;

  // SystemInitialize for Sum: '<S120>/Sum1' incorporates:
  //   Outport: '<S120>/qgw'

  localB->Sum1[1] = localP->qgw_Y0;
}

//
// System reset for enable system:
//    '<S108>/Hqgw'
//    '<S147>/Hqgw'
//
void MulticopterModelClass::QuadModelv_Hqgw_Reset(DW_Hqgw_QuadModelv_T *localDW,
  P_Hqgw_QuadModelv_T *localP)
{
  // InitializeConditions for UnitDelay: '<S120>/Unit Delay'
  localDW->UnitDelay_DSTATE[0] = localP->UnitDelay_InitialCondition;

  // InitializeConditions for UnitDelay: '<S120>/Unit Delay1'
  localDW->UnitDelay1_DSTATE[0] = localP->UnitDelay1_InitialCondition;

  // InitializeConditions for UnitDelay: '<S120>/Unit Delay'
  localDW->UnitDelay_DSTATE[1] = localP->UnitDelay_InitialCondition;

  // InitializeConditions for UnitDelay: '<S120>/Unit Delay1'
  localDW->UnitDelay1_DSTATE[1] = localP->UnitDelay1_InitialCondition;
}

//
// Disable for enable system:
//    '<S108>/Hqgw'
//    '<S147>/Hqgw'
//
void MulticopterModelClass::QuadModelv_Hqgw_Disable(B_Hqgw_QuadModelv_T *localB,
  DW_Hqgw_QuadModelv_T *localDW, P_Hqgw_QuadModelv_T *localP)
{
  // Disable for Sum: '<S120>/Sum1' incorporates:
  //   Outport: '<S120>/qgw'

  localB->Sum1[0] = localP->qgw_Y0;
  localB->Sum1[1] = localP->qgw_Y0;
  localDW->Hqgw_MODE = false;
}

//
// Outputs for enable system:
//    '<S108>/Hqgw'
//    '<S147>/Hqgw'
//
void MulticopterModelClass::QuadModelv_Hqgw(real_T rtu_Enable, real_T rtu_V,
  const real_T rtu_wg[2], real_T rtu_wingspan, B_Hqgw_QuadModelv_T *localB,
  DW_Hqgw_QuadModelv_T *localDW, P_Hqgw_QuadModelv_T *localP)
{
  // Outputs for Enabled SubSystem: '<S108>/Hqgw' incorporates:
  //   EnablePort: '<S120>/Enable'

  if ((rtmIsMajorTimeStep((&QuadModelv_M)) &&
       (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) &&
      rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    if (rtu_Enable > 0.0) {
      if (!localDW->Hqgw_MODE) {
        QuadModelv_Hqgw_Reset(localDW, localP);
        localDW->Hqgw_MODE = true;
      }
    } else if (localDW->Hqgw_MODE) {
      QuadModelv_Hqgw_Disable(localB, localDW, localP);
    }
  }

  if (localDW->Hqgw_MODE) {
    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
      // Gain: '<S120>/dt1'
      localB->dt1 = localP->dt1_Gain * rtu_wingspan;

      // Sum: '<S120>/Sum2' incorporates:
      //   Constant: '<S120>/Constant'
      //   Gain: '<S120>/dt'
      //   Product: '<S120>/w1'

      localB->Sum2 = localP->Constant_Value - rtu_V / localB->dt1 *
        localP->dt_Gain;
    }

    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0) {
      // Product: '<S120>/Lug//V2' incorporates:
      //   UnitDelay: '<S120>/Unit Delay'

      localB->LugV2[0] = localB->Sum2 * localDW->UnitDelay_DSTATE[0];

      // UnitDelay: '<S120>/Unit Delay1'
      localB->UnitDelay1[0] = localDW->UnitDelay1_DSTATE[0];

      // Product: '<S120>/Lug//V2' incorporates:
      //   UnitDelay: '<S120>/Unit Delay'

      localB->LugV2[1] = localB->Sum2 * localDW->UnitDelay_DSTATE[1];

      // UnitDelay: '<S120>/Unit Delay1'
      localB->UnitDelay1[1] = localDW->UnitDelay1_DSTATE[1];
    }

    // Sum: '<S120>/Sum1' incorporates:
    //   Product: '<S120>/w2'
    //   Sum: '<S120>/Sum3'

    localB->Sum1[0] = localB->LugV2[0] - (rtu_wg[0] - localB->UnitDelay1[0]) /
      localB->dt1;
    localB->Sum1[1] = localB->LugV2[1] - (rtu_wg[1] - localB->UnitDelay1[1]) /
      localB->dt1;
  }

  // End of Outputs for SubSystem: '<S108>/Hqgw'
}

//
// Update for enable system:
//    '<S108>/Hqgw'
//    '<S147>/Hqgw'
//
void MulticopterModelClass::QuadModelv_Hqgw_Update(const real_T rtu_wg[2],
  B_Hqgw_QuadModelv_T *localB, DW_Hqgw_QuadModelv_T *localDW)
{
  // Update for Enabled SubSystem: '<S108>/Hqgw' incorporates:
  //   EnablePort: '<S120>/Enable'

  if (localDW->Hqgw_MODE && (rtmIsMajorTimeStep((&QuadModelv_M)) &&
       (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0)) {
    // Update for UnitDelay: '<S120>/Unit Delay'
    localDW->UnitDelay_DSTATE[0] = localB->Sum1[0];

    // Update for UnitDelay: '<S120>/Unit Delay1'
    localDW->UnitDelay1_DSTATE[0] = rtu_wg[0];

    // Update for UnitDelay: '<S120>/Unit Delay'
    localDW->UnitDelay_DSTATE[1] = localB->Sum1[1];

    // Update for UnitDelay: '<S120>/Unit Delay1'
    localDW->UnitDelay1_DSTATE[1] = rtu_wg[1];
  }

  // End of Update for SubSystem: '<S108>/Hqgw'
}

//
// System initialize for enable system:
//    '<S108>/Hrgw'
//    '<S147>/Hrgw'
//
void MulticopterModelClass::QuadModelv_Hrgw_Init(B_Hrgw_QuadModelv_T *localB,
  DW_Hrgw_QuadModelv_T *localDW, P_Hrgw_QuadModelv_T *localP)
{
  // InitializeConditions for UnitDelay: '<S121>/Unit Delay'
  localDW->UnitDelay_DSTATE[0] = localP->UnitDelay_InitialCondition;

  // InitializeConditions for UnitDelay: '<S121>/Unit Delay1'
  localDW->UnitDelay1_DSTATE[0] = localP->UnitDelay1_InitialCondition;

  // SystemInitialize for Sum: '<S121>/Sum1' incorporates:
  //   Outport: '<S121>/rgw'

  localB->Sum1[0] = localP->rgw_Y0;

  // InitializeConditions for UnitDelay: '<S121>/Unit Delay'
  localDW->UnitDelay_DSTATE[1] = localP->UnitDelay_InitialCondition;

  // InitializeConditions for UnitDelay: '<S121>/Unit Delay1'
  localDW->UnitDelay1_DSTATE[1] = localP->UnitDelay1_InitialCondition;

  // SystemInitialize for Sum: '<S121>/Sum1' incorporates:
  //   Outport: '<S121>/rgw'

  localB->Sum1[1] = localP->rgw_Y0;
}

//
// System reset for enable system:
//    '<S108>/Hrgw'
//    '<S147>/Hrgw'
//
void MulticopterModelClass::QuadModelv_Hrgw_Reset(DW_Hrgw_QuadModelv_T *localDW,
  P_Hrgw_QuadModelv_T *localP)
{
  // InitializeConditions for UnitDelay: '<S121>/Unit Delay'
  localDW->UnitDelay_DSTATE[0] = localP->UnitDelay_InitialCondition;

  // InitializeConditions for UnitDelay: '<S121>/Unit Delay1'
  localDW->UnitDelay1_DSTATE[0] = localP->UnitDelay1_InitialCondition;

  // InitializeConditions for UnitDelay: '<S121>/Unit Delay'
  localDW->UnitDelay_DSTATE[1] = localP->UnitDelay_InitialCondition;

  // InitializeConditions for UnitDelay: '<S121>/Unit Delay1'
  localDW->UnitDelay1_DSTATE[1] = localP->UnitDelay1_InitialCondition;
}

//
// Disable for enable system:
//    '<S108>/Hrgw'
//    '<S147>/Hrgw'
//
void MulticopterModelClass::QuadModelv_Hrgw_Disable(B_Hrgw_QuadModelv_T *localB,
  DW_Hrgw_QuadModelv_T *localDW, P_Hrgw_QuadModelv_T *localP)
{
  // Disable for Sum: '<S121>/Sum1' incorporates:
  //   Outport: '<S121>/rgw'

  localB->Sum1[0] = localP->rgw_Y0;
  localB->Sum1[1] = localP->rgw_Y0;
  localDW->Hrgw_MODE = false;
}

//
// Outputs for enable system:
//    '<S108>/Hrgw'
//    '<S147>/Hrgw'
//
void MulticopterModelClass::QuadModelv_Hrgw(real_T rtu_Enable, real_T rtu_V,
  const real_T rtu_vg[2], real_T rtu_wingspan, B_Hrgw_QuadModelv_T *localB,
  DW_Hrgw_QuadModelv_T *localDW, P_Hrgw_QuadModelv_T *localP)
{
  // Outputs for Enabled SubSystem: '<S108>/Hrgw' incorporates:
  //   EnablePort: '<S121>/Enable'

  if ((rtmIsMajorTimeStep((&QuadModelv_M)) &&
       (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) &&
      rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    if (rtu_Enable > 0.0) {
      if (!localDW->Hrgw_MODE) {
        QuadModelv_Hrgw_Reset(localDW, localP);
        localDW->Hrgw_MODE = true;
      }
    } else if (localDW->Hrgw_MODE) {
      QuadModelv_Hrgw_Disable(localB, localDW, localP);
    }
  }

  if (localDW->Hrgw_MODE) {
    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
      // Gain: '<S121>/dt1'
      localB->dt1 = localP->dt1_Gain * rtu_wingspan;

      // Sum: '<S121>/Sum2' incorporates:
      //   Constant: '<S121>/Constant'
      //   Gain: '<S121>/dt'
      //   Product: '<S121>/w1'

      localB->Sum2 = localP->Constant_Value - rtu_V / localB->dt1 *
        localP->dt_Gain;
    }

    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0) {
      // Product: '<S121>/Lug//V2' incorporates:
      //   UnitDelay: '<S121>/Unit Delay'

      localB->LugV2[0] = localB->Sum2 * localDW->UnitDelay_DSTATE[0];

      // UnitDelay: '<S121>/Unit Delay1'
      localB->UnitDelay1[0] = localDW->UnitDelay1_DSTATE[0];

      // Product: '<S121>/Lug//V2' incorporates:
      //   UnitDelay: '<S121>/Unit Delay'

      localB->LugV2[1] = localB->Sum2 * localDW->UnitDelay_DSTATE[1];

      // UnitDelay: '<S121>/Unit Delay1'
      localB->UnitDelay1[1] = localDW->UnitDelay1_DSTATE[1];
    }

    // Sum: '<S121>/Sum1' incorporates:
    //   Product: '<S121>/w2'
    //   Sum: '<S121>/Sum3'

    localB->Sum1[0] = (rtu_vg[0] - localB->UnitDelay1[0]) / localB->dt1 +
      localB->LugV2[0];
    localB->Sum1[1] = (rtu_vg[1] - localB->UnitDelay1[1]) / localB->dt1 +
      localB->LugV2[1];
  }

  // End of Outputs for SubSystem: '<S108>/Hrgw'
}

//
// Update for enable system:
//    '<S108>/Hrgw'
//    '<S147>/Hrgw'
//
void MulticopterModelClass::QuadModelv_Hrgw_Update(const real_T rtu_vg[2],
  B_Hrgw_QuadModelv_T *localB, DW_Hrgw_QuadModelv_T *localDW)
{
  // Update for Enabled SubSystem: '<S108>/Hrgw' incorporates:
  //   EnablePort: '<S121>/Enable'

  if (localDW->Hrgw_MODE && (rtmIsMajorTimeStep((&QuadModelv_M)) &&
       (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0)) {
    // Update for UnitDelay: '<S121>/Unit Delay'
    localDW->UnitDelay_DSTATE[0] = localB->Sum1[0];

    // Update for UnitDelay: '<S121>/Unit Delay1'
    localDW->UnitDelay1_DSTATE[0] = rtu_vg[0];

    // Update for UnitDelay: '<S121>/Unit Delay'
    localDW->UnitDelay_DSTATE[1] = localB->Sum1[1];

    // Update for UnitDelay: '<S121>/Unit Delay1'
    localDW->UnitDelay1_DSTATE[1] = rtu_vg[1];
  }

  // End of Update for SubSystem: '<S108>/Hrgw'
}

//
// System initialize for enable system:
//    '<S109>/Hugw(z)'
//    '<S148>/Hugw(z)'
//
void MulticopterModelClass::QuadModelv_Hugwz_Init(B_Hugwz_QuadModelv_T *localB,
  DW_Hugwz_QuadModelv_T *localDW, P_Hugwz_QuadModelv_T *localP)
{
  // InitializeConditions for UnitDelay: '<S122>/Unit Delay'
  localDW->UnitDelay_DSTATE[0] = localP->UnitDelay_InitialCondition;

  // SystemInitialize for Sum: '<S122>/Sum' incorporates:
  //   Outport: '<S122>/ugw'

  localB->Sum[0] = localP->ugw_Y0;

  // InitializeConditions for UnitDelay: '<S122>/Unit Delay'
  localDW->UnitDelay_DSTATE[1] = localP->UnitDelay_InitialCondition;

  // SystemInitialize for Sum: '<S122>/Sum' incorporates:
  //   Outport: '<S122>/ugw'

  localB->Sum[1] = localP->ugw_Y0;
}

//
// System reset for enable system:
//    '<S109>/Hugw(z)'
//    '<S148>/Hugw(z)'
//
void MulticopterModelClass::QuadModelv_Hugwz_Reset(DW_Hugwz_QuadModelv_T
  *localDW, P_Hugwz_QuadModelv_T *localP)
{
  // InitializeConditions for UnitDelay: '<S122>/Unit Delay'
  localDW->UnitDelay_DSTATE[0] = localP->UnitDelay_InitialCondition;
  localDW->UnitDelay_DSTATE[1] = localP->UnitDelay_InitialCondition;
}

//
// Disable for enable system:
//    '<S109>/Hugw(z)'
//    '<S148>/Hugw(z)'
//
void MulticopterModelClass::QuadModelv_Hugwz_Disable(B_Hugwz_QuadModelv_T
  *localB, DW_Hugwz_QuadModelv_T *localDW, P_Hugwz_QuadModelv_T *localP)
{
  // Disable for Sum: '<S122>/Sum' incorporates:
  //   Outport: '<S122>/ugw'

  localB->Sum[0] = localP->ugw_Y0;
  localB->Sum[1] = localP->ugw_Y0;
  localDW->Hugwz_MODE = false;
}

//
// Outputs for enable system:
//    '<S109>/Hugw(z)'
//    '<S148>/Hugw(z)'
//
void MulticopterModelClass::QuadModelv_Hugwz(real_T rtu_Enable, real_T rtu_V,
  real_T rtu_L_ug, real_T rtu_L_ug_i, real_T rtu_sigma_ug, real_T rtu_sigma_ug_j,
  real_T rtu_Noise, B_Hugwz_QuadModelv_T *localB, DW_Hugwz_QuadModelv_T *localDW,
  P_Hugwz_QuadModelv_T *localP)
{
  real_T rtb_VLug_idx_0;
  real_T rtb_VLug_idx_1;

  // Outputs for Enabled SubSystem: '<S109>/Hugw(z)' incorporates:
  //   EnablePort: '<S122>/Enable'

  if ((rtmIsMajorTimeStep((&QuadModelv_M)) &&
       (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) &&
      rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    if (rtu_Enable > 0.0) {
      if (!localDW->Hugwz_MODE) {
        QuadModelv_Hugwz_Reset(localDW, localP);
        localDW->Hugwz_MODE = true;
      }
    } else if (localDW->Hugwz_MODE) {
      QuadModelv_Hugwz_Disable(localB, localDW, localP);
    }
  }

  if (localDW->Hugwz_MODE) {
    // Product: '<S122>/V//Lug'
    rtb_VLug_idx_0 = rtu_V / rtu_L_ug;
    rtb_VLug_idx_1 = rtu_V / rtu_L_ug_i;
    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0) {
      // UnitDelay: '<S122>/Unit Delay'
      localB->UnitDelay[0] = localDW->UnitDelay_DSTATE[0];
      localB->UnitDelay[1] = localDW->UnitDelay_DSTATE[1];
    }

    // Sum: '<S122>/Sum' incorporates:
    //   Constant: '<S122>/Constant'
    //   Gain: '<S122>/2'
    //   Gain: '<S122>/dt'
    //   Product: '<S122>/Lug//V1'
    //   Product: '<S122>/Lug//V2'
    //   Sqrt: '<S122>/sqrt'
    //   Sum: '<S122>/Sum1'

    localB->Sum[0] = (localP->Constant_Value - localP->dt_Gain * rtb_VLug_idx_0)
      * localB->UnitDelay[0] + std::sqrt(localP->u_Gain * rtb_VLug_idx_0) *
      rtu_Noise * rtu_sigma_ug;
    localB->Sum[1] = (localP->Constant_Value - localP->dt_Gain * rtb_VLug_idx_1)
      * localB->UnitDelay[1] + std::sqrt(localP->u_Gain * rtb_VLug_idx_1) *
      rtu_Noise * rtu_sigma_ug_j;
  }

  // End of Outputs for SubSystem: '<S109>/Hugw(z)'
}

//
// Update for enable system:
//    '<S109>/Hugw(z)'
//    '<S148>/Hugw(z)'
//
void MulticopterModelClass::QuadModelv_Hugwz_Update(B_Hugwz_QuadModelv_T *localB,
  DW_Hugwz_QuadModelv_T *localDW)
{
  // Update for Enabled SubSystem: '<S109>/Hugw(z)' incorporates:
  //   EnablePort: '<S122>/Enable'

  if (localDW->Hugwz_MODE && (rtmIsMajorTimeStep((&QuadModelv_M)) &&
       (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0)) {
    // Update for UnitDelay: '<S122>/Unit Delay'
    localDW->UnitDelay_DSTATE[0] = localB->Sum[0];
    localDW->UnitDelay_DSTATE[1] = localB->Sum[1];
  }

  // End of Update for SubSystem: '<S109>/Hugw(z)'
}

//
// System initialize for enable system:
//    '<S109>/Hvgw(z)'
//    '<S148>/Hvgw(z)'
//
void MulticopterModelClass::QuadModelv_Hvgwz_Init(B_Hvgwz_QuadModelv_T *localB,
  DW_Hvgwz_QuadModelv_T *localDW, P_Hvgwz_QuadModelv_T *localP)
{
  // InitializeConditions for UnitDelay: '<S123>/Unit Delay'
  localDW->UnitDelay_DSTATE[0] = localP->UnitDelay_InitialCondition;

  // SystemInitialize for Sum: '<S123>/Sum' incorporates:
  //   Outport: '<S123>/vgw'

  localB->Sum[0] = localP->vgw_Y0;

  // InitializeConditions for UnitDelay: '<S123>/Unit Delay'
  localDW->UnitDelay_DSTATE[1] = localP->UnitDelay_InitialCondition;

  // SystemInitialize for Sum: '<S123>/Sum' incorporates:
  //   Outport: '<S123>/vgw'

  localB->Sum[1] = localP->vgw_Y0;
}

//
// System reset for enable system:
//    '<S109>/Hvgw(z)'
//    '<S148>/Hvgw(z)'
//
void MulticopterModelClass::QuadModelv_Hvgwz_Reset(DW_Hvgwz_QuadModelv_T
  *localDW, P_Hvgwz_QuadModelv_T *localP)
{
  // InitializeConditions for UnitDelay: '<S123>/Unit Delay'
  localDW->UnitDelay_DSTATE[0] = localP->UnitDelay_InitialCondition;
  localDW->UnitDelay_DSTATE[1] = localP->UnitDelay_InitialCondition;
}

//
// Disable for enable system:
//    '<S109>/Hvgw(z)'
//    '<S148>/Hvgw(z)'
//
void MulticopterModelClass::QuadModelv_Hvgwz_Disable(B_Hvgwz_QuadModelv_T
  *localB, DW_Hvgwz_QuadModelv_T *localDW, P_Hvgwz_QuadModelv_T *localP)
{
  // Disable for Sum: '<S123>/Sum' incorporates:
  //   Outport: '<S123>/vgw'

  localB->Sum[0] = localP->vgw_Y0;
  localB->Sum[1] = localP->vgw_Y0;
  localDW->Hvgwz_MODE = false;
}

//
// Outputs for enable system:
//    '<S109>/Hvgw(z)'
//    '<S148>/Hvgw(z)'
//
void MulticopterModelClass::QuadModelv_Hvgwz(real_T rtu_Enable, real_T
  rtu_sigma_vg, real_T rtu_sigma_vg_e, const real_T rtu_L_vg[2], real_T rtu_V,
  real_T rtu_Noise, B_Hvgwz_QuadModelv_T *localB, DW_Hvgwz_QuadModelv_T *localDW,
  P_Hvgwz_QuadModelv_T *localP)
{
  real_T rtb_VLvg_idx_0;
  real_T rtb_VLvg_idx_1;

  // Outputs for Enabled SubSystem: '<S109>/Hvgw(z)' incorporates:
  //   EnablePort: '<S123>/Enable'

  if ((rtmIsMajorTimeStep((&QuadModelv_M)) &&
       (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) &&
      rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    if (rtu_Enable > 0.0) {
      if (!localDW->Hvgwz_MODE) {
        QuadModelv_Hvgwz_Reset(localDW, localP);
        localDW->Hvgwz_MODE = true;
      }
    } else if (localDW->Hvgwz_MODE) {
      QuadModelv_Hvgwz_Disable(localB, localDW, localP);
    }
  }

  if (localDW->Hvgwz_MODE) {
    // Product: '<S123>/V//Lvg'
    rtb_VLvg_idx_0 = rtu_V / rtu_L_vg[0];
    rtb_VLvg_idx_1 = rtu_V / rtu_L_vg[1];
    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0) {
      // UnitDelay: '<S123>/Unit Delay'
      localB->UnitDelay[0] = localDW->UnitDelay_DSTATE[0];
      localB->UnitDelay[1] = localDW->UnitDelay_DSTATE[1];
    }

    // Sum: '<S123>/Sum' incorporates:
    //   Constant: '<S123>/Constant'
    //   Gain: '<S123>/2'
    //   Gain: '<S123>/dt'
    //   Product: '<S123>/Lug//V1'
    //   Product: '<S123>/Lug//V2'
    //   Sqrt: '<S123>/sqrt'
    //   Sum: '<S123>/Sum1'

    localB->Sum[0] = (localP->Constant_Value - localP->dt_Gain * rtb_VLvg_idx_0)
      * localB->UnitDelay[0] + std::sqrt(localP->u_Gain * rtb_VLvg_idx_0) *
      rtu_Noise * rtu_sigma_vg;
    localB->Sum[1] = (localP->Constant_Value - localP->dt_Gain * rtb_VLvg_idx_1)
      * localB->UnitDelay[1] + std::sqrt(localP->u_Gain * rtb_VLvg_idx_1) *
      rtu_Noise * rtu_sigma_vg_e;
  }

  // End of Outputs for SubSystem: '<S109>/Hvgw(z)'
}

//
// Update for enable system:
//    '<S109>/Hvgw(z)'
//    '<S148>/Hvgw(z)'
//
void MulticopterModelClass::QuadModelv_Hvgwz_Update(B_Hvgwz_QuadModelv_T *localB,
  DW_Hvgwz_QuadModelv_T *localDW)
{
  // Update for Enabled SubSystem: '<S109>/Hvgw(z)' incorporates:
  //   EnablePort: '<S123>/Enable'

  if (localDW->Hvgwz_MODE && (rtmIsMajorTimeStep((&QuadModelv_M)) &&
       (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0)) {
    // Update for UnitDelay: '<S123>/Unit Delay'
    localDW->UnitDelay_DSTATE[0] = localB->Sum[0];
    localDW->UnitDelay_DSTATE[1] = localB->Sum[1];
  }

  // End of Update for SubSystem: '<S109>/Hvgw(z)'
}

//
// System initialize for enable system:
//    '<S109>/Hwgw(z)'
//    '<S148>/Hwgw(z)'
//
void MulticopterModelClass::QuadModelv_Hwgwz_Init(B_Hwgwz_QuadModelv_T *localB,
  DW_Hwgwz_QuadModelv_T *localDW, P_Hwgwz_QuadModelv_T *localP)
{
  // InitializeConditions for UnitDelay: '<S124>/Unit Delay'
  localDW->UnitDelay_DSTATE[0] = localP->UnitDelay_InitialCondition;

  // SystemInitialize for Sum: '<S124>/Sum' incorporates:
  //   Outport: '<S124>/wgw'

  localB->Sum[0] = localP->wgw_Y0;

  // InitializeConditions for UnitDelay: '<S124>/Unit Delay'
  localDW->UnitDelay_DSTATE[1] = localP->UnitDelay_InitialCondition;

  // SystemInitialize for Sum: '<S124>/Sum' incorporates:
  //   Outport: '<S124>/wgw'

  localB->Sum[1] = localP->wgw_Y0;
}

//
// System reset for enable system:
//    '<S109>/Hwgw(z)'
//    '<S148>/Hwgw(z)'
//
void MulticopterModelClass::QuadModelv_Hwgwz_Reset(DW_Hwgwz_QuadModelv_T
  *localDW, P_Hwgwz_QuadModelv_T *localP)
{
  // InitializeConditions for UnitDelay: '<S124>/Unit Delay'
  localDW->UnitDelay_DSTATE[0] = localP->UnitDelay_InitialCondition;
  localDW->UnitDelay_DSTATE[1] = localP->UnitDelay_InitialCondition;
}

//
// Disable for enable system:
//    '<S109>/Hwgw(z)'
//    '<S148>/Hwgw(z)'
//
void MulticopterModelClass::QuadModelv_Hwgwz_Disable(B_Hwgwz_QuadModelv_T
  *localB, DW_Hwgwz_QuadModelv_T *localDW, P_Hwgwz_QuadModelv_T *localP)
{
  // Disable for Sum: '<S124>/Sum' incorporates:
  //   Outport: '<S124>/wgw'

  localB->Sum[0] = localP->wgw_Y0;
  localB->Sum[1] = localP->wgw_Y0;
  localDW->Hwgwz_MODE = false;
}

//
// Outputs for enable system:
//    '<S109>/Hwgw(z)'
//    '<S148>/Hwgw(z)'
//
void MulticopterModelClass::QuadModelv_Hwgwz(real_T rtu_Enable, real_T rtu_V,
  const real_T rtu_L_wg[2], real_T rtu_sigma_wg, real_T rtu_sigma_wg_j, real_T
  rtu_Noise, B_Hwgwz_QuadModelv_T *localB, DW_Hwgwz_QuadModelv_T *localDW,
  P_Hwgwz_QuadModelv_T *localP)
{
  real_T rtb_VLwg_idx_0;
  real_T rtb_VLwg_idx_1;

  // Outputs for Enabled SubSystem: '<S109>/Hwgw(z)' incorporates:
  //   EnablePort: '<S124>/Enable'

  if ((rtmIsMajorTimeStep((&QuadModelv_M)) &&
       (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) &&
      rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    if (rtu_Enable > 0.0) {
      if (!localDW->Hwgwz_MODE) {
        QuadModelv_Hwgwz_Reset(localDW, localP);
        localDW->Hwgwz_MODE = true;
      }
    } else if (localDW->Hwgwz_MODE) {
      QuadModelv_Hwgwz_Disable(localB, localDW, localP);
    }
  }

  if (localDW->Hwgwz_MODE) {
    // Product: '<S124>/V//Lwg'
    rtb_VLwg_idx_0 = rtu_V / rtu_L_wg[0];
    rtb_VLwg_idx_1 = rtu_V / rtu_L_wg[1];
    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0) {
      // UnitDelay: '<S124>/Unit Delay'
      localB->UnitDelay[0] = localDW->UnitDelay_DSTATE[0];
      localB->UnitDelay[1] = localDW->UnitDelay_DSTATE[1];
    }

    // Sum: '<S124>/Sum' incorporates:
    //   Constant: '<S124>/Constant'
    //   Gain: '<S124>/2'
    //   Gain: '<S124>/dt'
    //   Product: '<S124>/Lug//V1'
    //   Product: '<S124>/Lug//V2'
    //   Sqrt: '<S124>/sqrt'
    //   Sum: '<S124>/Sum1'

    localB->Sum[0] = (localP->Constant_Value - localP->dt_Gain * rtb_VLwg_idx_0)
      * localB->UnitDelay[0] + std::sqrt(localP->u_Gain * rtb_VLwg_idx_0) *
      rtu_Noise * rtu_sigma_wg;
    localB->Sum[1] = (localP->Constant_Value - localP->dt_Gain * rtb_VLwg_idx_1)
      * localB->UnitDelay[1] + std::sqrt(localP->u_Gain * rtb_VLwg_idx_1) *
      rtu_Noise * rtu_sigma_wg_j;
  }

  // End of Outputs for SubSystem: '<S109>/Hwgw(z)'
}

//
// Update for enable system:
//    '<S109>/Hwgw(z)'
//    '<S148>/Hwgw(z)'
//
void MulticopterModelClass::QuadModelv_Hwgwz_Update(B_Hwgwz_QuadModelv_T *localB,
  DW_Hwgwz_QuadModelv_T *localDW)
{
  // Update for Enabled SubSystem: '<S109>/Hwgw(z)' incorporates:
  //   EnablePort: '<S124>/Enable'

  if (localDW->Hwgwz_MODE && (rtmIsMajorTimeStep((&QuadModelv_M)) &&
       (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0)) {
    // Update for UnitDelay: '<S124>/Unit Delay'
    localDW->UnitDelay_DSTATE[0] = localB->Sum[0];
    localDW->UnitDelay_DSTATE[1] = localB->Sum[1];
  }

  // End of Update for SubSystem: '<S109>/Hwgw(z)'
}

//
// Output and update for action system:
//    '<S113>/Low altitude  rates'
//    '<S152>/Low altitude  rates'
//
void MulticopterModelClass::QuadModelv_Lowaltituderates(const real_T rtu_DCM[9],
  const real_T rtu_pgw_hl[2], const real_T rtu_qgw_hl[2], const real_T
  rtu_rgw_hl[2], real_T rtu_Winddirection, real_T rty_pgwqgwrgw[3])
{
  real_T rtb_TrigonometricFunction1_o1;
  real_T rtb_TrigonometricFunction1_o2;
  real_T rtb_VectorConcatenate_idx_0;
  real_T rtb_VectorConcatenate_idx_2;

  // SignalConversion generated from: '<S133>/Vector Concatenate'
  rtb_VectorConcatenate_idx_2 = rtu_rgw_hl[0];

  // Trigonometry: '<S134>/Trigonometric Function1'
  rtb_TrigonometricFunction1_o1 = std::sin(rtu_Winddirection);
  rtb_TrigonometricFunction1_o2 = std::cos(rtu_Winddirection);

  // Sum: '<S134>/Sum' incorporates:
  //   Product: '<S134>/Product1'
  //   Product: '<S134>/Product2'

  rtb_VectorConcatenate_idx_0 = rtu_pgw_hl[0] * rtb_TrigonometricFunction1_o2 -
    rtb_TrigonometricFunction1_o1 * rtu_qgw_hl[0];

  // Sum: '<S134>/Sum1' incorporates:
  //   Product: '<S134>/Product1'
  //   Product: '<S134>/Product2'

  rtb_TrigonometricFunction1_o1 = rtb_TrigonometricFunction1_o1 * rtu_pgw_hl[0]
    + rtu_qgw_hl[0] * rtb_TrigonometricFunction1_o2;

  // Reshape: '<S133>/Reshape1' incorporates:
  //   Concatenate: '<S133>/Vector Concatenate'
  //   Product: '<S133>/Product'

  for (int32_T i = 0; i < 3; i++) {
    rty_pgwqgwrgw[i] = (rtu_DCM[i + 3] * rtb_TrigonometricFunction1_o1 +
                        rtu_DCM[i] * rtb_VectorConcatenate_idx_0) + rtu_DCM[i +
      6] * rtb_VectorConcatenate_idx_2;
  }

  // End of Reshape: '<S133>/Reshape1'
}

//
// Output and update for action system:
//    '<S113>/Interpolate  rates'
//    '<S152>/Interpolate  rates'
//
void MulticopterModelClass::QuadModelv_Interpolaterates(const real_T rtu_pgw_hl
  [2], const real_T rtu_qgw_hl[2], const real_T rtu_rgw_hl[2], const real_T
  rtu_DCM[9], real_T rtu_Winddirection, real_T rtu_Altitude, real_T
  rty_pgwqgwrgw[3], P_Interpolaterates_QuadModelv_T *localP)
{
  real_T rtb_Product_fi[3];
  real_T rtb_TrigonometricFunction_o1;
  real_T rtb_TrigonometricFunction_o2;
  real_T rtb_VectorConcatenate_p_idx_0;

  // Trigonometry: '<S132>/Trigonometric Function'
  rtb_TrigonometricFunction_o1 = std::sin(rtu_Winddirection);
  rtb_TrigonometricFunction_o2 = std::cos(rtu_Winddirection);

  // Sum: '<S132>/Sum' incorporates:
  //   Product: '<S132>/Product1'
  //   Product: '<S132>/Product2'

  rtb_VectorConcatenate_p_idx_0 = rtu_pgw_hl[0] * rtb_TrigonometricFunction_o2 -
    rtb_TrigonometricFunction_o1 * rtu_qgw_hl[0];

  // Sum: '<S132>/Sum1' incorporates:
  //   Product: '<S132>/Product1'
  //   Product: '<S132>/Product2'

  rtb_TrigonometricFunction_o1 = rtb_TrigonometricFunction_o1 * rtu_pgw_hl[0] +
    rtu_qgw_hl[0] * rtb_TrigonometricFunction_o2;

  // SignalConversion generated from: '<S131>/Vector Concatenate'
  rtb_TrigonometricFunction_o2 = rtu_rgw_hl[0];

  // Product: '<S131>/Product'
  for (int32_T i = 0; i < 3; i++) {
    // Product: '<S131>/Product' incorporates:
    //   Concatenate: '<S131>/Vector Concatenate'

    rtb_Product_fi[i] = (rtu_DCM[i + 3] * rtb_TrigonometricFunction_o1 +
                         rtu_DCM[i] * rtb_VectorConcatenate_p_idx_0) + rtu_DCM[i
      + 6] * rtb_TrigonometricFunction_o2;
  }

  // End of Product: '<S131>/Product'

  // Sum: '<S127>/Sum1' incorporates:
  //   Constant: '<S127>/max_height_low'

  rtb_TrigonometricFunction_o1 = rtu_Altitude - localP->max_height_low_Value;

  // Sum: '<S127>/Sum' incorporates:
  //   Constant: '<S127>/max_height_low'
  //   Constant: '<S127>/min_height_high'

  rtb_TrigonometricFunction_o2 = localP->min_height_high_Value -
    localP->max_height_low_Value;

  // Sum: '<S127>/Sum3' incorporates:
  //   Product: '<S127>/Product1'
  //   Sum: '<S127>/Sum2'

  rty_pgwqgwrgw[0] = (rtu_pgw_hl[1] - rtb_Product_fi[0]) *
    rtb_TrigonometricFunction_o1 / rtb_TrigonometricFunction_o2 +
    rtb_Product_fi[0];
  rty_pgwqgwrgw[1] = (rtu_qgw_hl[1] - rtb_Product_fi[1]) *
    rtb_TrigonometricFunction_o1 / rtb_TrigonometricFunction_o2 +
    rtb_Product_fi[1];
  rty_pgwqgwrgw[2] = (rtu_rgw_hl[1] - rtb_Product_fi[2]) *
    rtb_TrigonometricFunction_o1 / rtb_TrigonometricFunction_o2 +
    rtb_Product_fi[2];
}

//
// Output and update for action system:
//    '<S114>/Interpolate  velocities'
//    '<S153>/Interpolate  velocities'
//
void MulticopterModelClass::QuadModel_Interpolatevelocities(const real_T
  rtu_ugw_hl[2], const real_T rtu_vgw_hl[2], const real_T rtu_wgw_hl[2], const
  real_T rtu_DCM[9], real_T rtu_Winddirection, real_T rtu_Altitude, real_T
  rty_ugwvgwwgw[3], P_Interpolatevelocities_QuadM_T *localP)
{
  real_T rtb_Product_f2[3];
  real_T rtb_TrigonometricFunction_o1;
  real_T rtb_TrigonometricFunction_o2;
  real_T rtb_VectorConcatenate_f_idx_0;

  // Trigonometry: '<S140>/Trigonometric Function'
  rtb_TrigonometricFunction_o1 = std::sin(rtu_Winddirection);
  rtb_TrigonometricFunction_o2 = std::cos(rtu_Winddirection);

  // Sum: '<S140>/Sum' incorporates:
  //   Product: '<S140>/Product1'
  //   Product: '<S140>/Product2'

  rtb_VectorConcatenate_f_idx_0 = rtu_ugw_hl[0] * rtb_TrigonometricFunction_o2 -
    rtb_TrigonometricFunction_o1 * rtu_vgw_hl[0];

  // Sum: '<S140>/Sum1' incorporates:
  //   Product: '<S140>/Product1'
  //   Product: '<S140>/Product2'

  rtb_TrigonometricFunction_o1 = rtb_TrigonometricFunction_o1 * rtu_ugw_hl[0] +
    rtu_vgw_hl[0] * rtb_TrigonometricFunction_o2;

  // SignalConversion generated from: '<S139>/Vector Concatenate'
  rtb_TrigonometricFunction_o2 = rtu_wgw_hl[0];

  // Product: '<S139>/Product'
  for (int32_T i = 0; i < 3; i++) {
    // Product: '<S139>/Product' incorporates:
    //   Concatenate: '<S139>/Vector Concatenate'

    rtb_Product_f2[i] = (rtu_DCM[i + 3] * rtb_TrigonometricFunction_o1 +
                         rtu_DCM[i] * rtb_VectorConcatenate_f_idx_0) + rtu_DCM[i
      + 6] * rtb_TrigonometricFunction_o2;
  }

  // End of Product: '<S139>/Product'

  // Sum: '<S135>/Sum1' incorporates:
  //   Constant: '<S135>/max_height_low'

  rtb_TrigonometricFunction_o1 = rtu_Altitude - localP->max_height_low_Value;

  // Sum: '<S135>/Sum' incorporates:
  //   Constant: '<S135>/max_height_low'
  //   Constant: '<S135>/min_height_high'

  rtb_TrigonometricFunction_o2 = localP->min_height_high_Value -
    localP->max_height_low_Value;

  // Sum: '<S135>/Sum3' incorporates:
  //   Product: '<S135>/Product1'
  //   Sum: '<S135>/Sum2'

  rty_ugwvgwwgw[0] = (rtu_ugw_hl[1] - rtb_Product_f2[0]) *
    rtb_TrigonometricFunction_o1 / rtb_TrigonometricFunction_o2 +
    rtb_Product_f2[0];
  rty_ugwvgwwgw[1] = (rtu_vgw_hl[1] - rtb_Product_f2[1]) *
    rtb_TrigonometricFunction_o1 / rtb_TrigonometricFunction_o2 +
    rtb_Product_f2[1];
  rty_ugwvgwwgw[2] = (rtu_wgw_hl[1] - rtb_Product_f2[2]) *
    rtb_TrigonometricFunction_o1 / rtb_TrigonometricFunction_o2 +
    rtb_Product_f2[2];
}

//
// Output and update for action system:
//    '<S193>/Positive Trace'
//    '<S194>/Positive Trace'
//    '<S321>/Positive Trace'
//
void MulticopterModelClass::QuadModelv_PositiveTrace(real_T rtu_traceDCM, const
  real_T rtu_DCM[9], real_T *rty_qwqxqyqz, real_T rty_qwqxqyqz_a[3],
  P_PositiveTrace_QuadModelv_T *localP)
{
  real_T rtb_Gain1_o;

  // Sqrt: '<S202>/sqrt' incorporates:
  //   Constant: '<S202>/Constant'
  //   Sum: '<S202>/Sum'

  rtb_Gain1_o = std::sqrt(rtu_traceDCM + localP->Constant_Value);

  // Gain: '<S202>/Gain'
  *rty_qwqxqyqz = localP->Gain_Gain * rtb_Gain1_o;

  // Gain: '<S202>/Gain1'
  rtb_Gain1_o *= localP->Gain1_Gain;

  // Product: '<S202>/Product' incorporates:
  //   Sum: '<S224>/Add'
  //   Sum: '<S225>/Add'
  //   Sum: '<S226>/Add'

  rty_qwqxqyqz_a[0] = (rtu_DCM[7] - rtu_DCM[5]) / rtb_Gain1_o;
  rty_qwqxqyqz_a[1] = (rtu_DCM[2] - rtu_DCM[6]) / rtb_Gain1_o;
  rty_qwqxqyqz_a[2] = (rtu_DCM[3] - rtu_DCM[1]) / rtb_Gain1_o;
}

//
// System initialize for action system:
//    '<S193>/Negative Trace'
//    '<S194>/Negative Trace'
//    '<S321>/Negative Trace'
//
void MulticopterModelClass::QuadModelv_NegativeTrace_Init
  (DW_NegativeTrace_QuadModelv_T *localDW)
{
  // Start for If: '<S201>/Find Maximum Diagonal Value'
  localDW->FindMaximumDiagonalValue_Active = -1;
}

//
// Output and update for action system:
//    '<S193>/Negative Trace'
//    '<S194>/Negative Trace'
//    '<S321>/Negative Trace'
//
void MulticopterModelClass::QuadModelv_NegativeTrace(const real_T rtu_DCM[9],
  real_T rty_qwqxqyqz[4], DW_NegativeTrace_QuadModelv_T *localDW,
  P_NegativeTrace_QuadModelv_T *localP)
{
  real_T rtb_Product_a;
  real_T rtb_Switch_g_idx_0;
  int8_T rtAction;

  // If: '<S201>/Find Maximum Diagonal Value'
  if (rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    if ((rtu_DCM[4] > rtu_DCM[0]) && (rtu_DCM[4] > rtu_DCM[8])) {
      rtAction = 0;
    } else if (rtu_DCM[8] > rtu_DCM[0]) {
      rtAction = 1;
    } else {
      rtAction = 2;
    }

    localDW->FindMaximumDiagonalValue_Active = rtAction;
  } else {
    rtAction = localDW->FindMaximumDiagonalValue_Active;
  }

  switch (rtAction) {
   case 0:
    // Outputs for IfAction SubSystem: '<S201>/Maximum Value at DCM(2,2)' incorporates:
    //   ActionPort: '<S206>/Action Port'

    // Sqrt: '<S206>/sqrt' incorporates:
    //   Constant: '<S218>/Constant'
    //   Sum: '<S218>/Add'

    rtb_Product_a = std::sqrt(((rtu_DCM[4] - rtu_DCM[0]) - rtu_DCM[8]) +
      localP->Constant_Value);

    // Gain: '<S206>/Gain'
    rty_qwqxqyqz[2] = localP->Gain_Gain * rtb_Product_a;

    // Switch: '<S217>/Switch' incorporates:
    //   Constant: '<S217>/Constant1'
    //   Constant: '<S217>/Constant2'

    if (rtb_Product_a != 0.0) {
      rtb_Switch_g_idx_0 = localP->Constant1_Value;
    } else {
      rtb_Switch_g_idx_0 = localP->Constant2_Value[0];
      rtb_Product_a = localP->Constant2_Value[1];
    }

    // End of Switch: '<S217>/Switch'

    // Product: '<S217>/Product'
    rtb_Product_a = rtb_Switch_g_idx_0 / rtb_Product_a;

    // Gain: '<S206>/Gain1' incorporates:
    //   Product: '<S206>/Product'
    //   Sum: '<S216>/Add'

    rty_qwqxqyqz[1] = (rtu_DCM[1] + rtu_DCM[3]) * rtb_Product_a *
      localP->Gain1_Gain;

    // Gain: '<S206>/Gain3' incorporates:
    //   Product: '<S206>/Product'
    //   Sum: '<S215>/Add'

    rty_qwqxqyqz[3] = (rtu_DCM[5] + rtu_DCM[7]) * rtb_Product_a *
      localP->Gain3_Gain;

    // Gain: '<S206>/Gain4' incorporates:
    //   Product: '<S206>/Product'
    //   Sum: '<S214>/Add'

    rty_qwqxqyqz[0] = (rtu_DCM[2] - rtu_DCM[6]) * rtb_Product_a *
      localP->Gain4_Gain;

    // End of Outputs for SubSystem: '<S201>/Maximum Value at DCM(2,2)'
    break;

   case 1:
    // Outputs for IfAction SubSystem: '<S201>/Maximum Value at DCM(3,3)' incorporates:
    //   ActionPort: '<S207>/Action Port'

    // Sqrt: '<S207>/sqrt' incorporates:
    //   Constant: '<S223>/Constant'
    //   Sum: '<S223>/Add'

    rtb_Product_a = std::sqrt(((rtu_DCM[8] - rtu_DCM[0]) - rtu_DCM[4]) +
      localP->Constant_Value_n);

    // Gain: '<S207>/Gain'
    rty_qwqxqyqz[3] = localP->Gain_Gain_c * rtb_Product_a;

    // Switch: '<S222>/Switch' incorporates:
    //   Constant: '<S222>/Constant1'
    //   Constant: '<S222>/Constant2'

    if (rtb_Product_a != 0.0) {
      rtb_Switch_g_idx_0 = localP->Constant1_Value_a;
    } else {
      rtb_Switch_g_idx_0 = localP->Constant2_Value_c[0];
      rtb_Product_a = localP->Constant2_Value_c[1];
    }

    // End of Switch: '<S222>/Switch'

    // Product: '<S222>/Product'
    rtb_Product_a = rtb_Switch_g_idx_0 / rtb_Product_a;

    // Gain: '<S207>/Gain1' incorporates:
    //   Product: '<S207>/Product'
    //   Sum: '<S219>/Add'

    rty_qwqxqyqz[1] = (rtu_DCM[2] + rtu_DCM[6]) * rtb_Product_a *
      localP->Gain1_Gain_n;

    // Gain: '<S207>/Gain2' incorporates:
    //   Product: '<S207>/Product'
    //   Sum: '<S220>/Add'

    rty_qwqxqyqz[2] = (rtu_DCM[5] + rtu_DCM[7]) * rtb_Product_a *
      localP->Gain2_Gain;

    // Gain: '<S207>/Gain3' incorporates:
    //   Product: '<S207>/Product'
    //   Sum: '<S221>/Add'

    rty_qwqxqyqz[0] = (rtu_DCM[3] - rtu_DCM[1]) * rtb_Product_a *
      localP->Gain3_Gain_m;

    // End of Outputs for SubSystem: '<S201>/Maximum Value at DCM(3,3)'
    break;

   default:
    // Outputs for IfAction SubSystem: '<S201>/Maximum Value at DCM(1,1)' incorporates:
    //   ActionPort: '<S205>/Action Port'

    // Sqrt: '<S205>/sqrt' incorporates:
    //   Constant: '<S213>/Constant'
    //   Sum: '<S213>/Add'

    rtb_Product_a = std::sqrt(((rtu_DCM[0] - rtu_DCM[4]) - rtu_DCM[8]) +
      localP->Constant_Value_m);

    // Gain: '<S205>/Gain'
    rty_qwqxqyqz[1] = localP->Gain_Gain_l * rtb_Product_a;

    // Switch: '<S212>/Switch' incorporates:
    //   Constant: '<S212>/Constant1'
    //   Constant: '<S212>/Constant2'

    if (rtb_Product_a != 0.0) {
      rtb_Switch_g_idx_0 = localP->Constant1_Value_i;
    } else {
      rtb_Switch_g_idx_0 = localP->Constant2_Value_b[0];
      rtb_Product_a = localP->Constant2_Value_b[1];
    }

    // End of Switch: '<S212>/Switch'

    // Product: '<S212>/Product'
    rtb_Product_a = rtb_Switch_g_idx_0 / rtb_Product_a;

    // Gain: '<S205>/Gain1' incorporates:
    //   Product: '<S205>/Product'
    //   Sum: '<S211>/Add'

    rty_qwqxqyqz[2] = (rtu_DCM[1] + rtu_DCM[3]) * rtb_Product_a *
      localP->Gain1_Gain_b;

    // Gain: '<S205>/Gain2' incorporates:
    //   Product: '<S205>/Product'
    //   Sum: '<S209>/Add'

    rty_qwqxqyqz[3] = (rtu_DCM[2] + rtu_DCM[6]) * rtb_Product_a *
      localP->Gain2_Gain_d;

    // Gain: '<S205>/Gain3' incorporates:
    //   Product: '<S205>/Product'
    //   Sum: '<S210>/Add'

    rty_qwqxqyqz[0] = (rtu_DCM[7] - rtu_DCM[5]) * rtb_Product_a *
      localP->Gain3_Gain_n;

    // End of Outputs for SubSystem: '<S201>/Maximum Value at DCM(1,1)'
    break;
  }

  // End of If: '<S201>/Find Maximum Diagonal Value'
}

//
// Output and update for action system:
//    '<S203>/If Warning//Error'
//    '<S241>/If Warning//Error'
//    '<S329>/If Warning//Error'
//
void MulticopterModelClass::QuadModelv_IfWarningError(const real_T rtu_dcm[9],
  real_T rtp_action, real_T rtp_tolerance, P_IfWarningError_QuadModelv_T *localP)
{
  real_T rtu_dcm_0[9];
  boolean_T rtb_Compare_k[9];
  boolean_T tmp;

  // Bias: '<S230>/Bias1' incorporates:
  //   Math: '<S230>/Math Function'
  //   Product: '<S230>/Product'

  for (int32_T i = 0; i < 3; i++) {
    for (int32_T i_0 = 0; i_0 < 3; i_0++) {
      int32_T rtu_dcm_tmp;
      rtu_dcm_tmp = 3 * i_0 + i;
      rtu_dcm_0[rtu_dcm_tmp] = ((rtu_dcm[3 * i + 1] * rtu_dcm[3 * i_0 + 1] +
        rtu_dcm[3 * i] * rtu_dcm[3 * i_0]) + rtu_dcm[3 * i + 2] * rtu_dcm[3 *
        i_0 + 2]) + localP->Bias1_Bias[rtu_dcm_tmp];
    }
  }

  // End of Bias: '<S230>/Bias1'

  // RelationalOperator: '<S236>/Compare' incorporates:
  //   Abs: '<S230>/Abs2'
  //   Constant: '<S236>/Constant'

  for (int32_T i = 0; i < 9; i++) {
    rtb_Compare_k[i] = (std::abs(rtu_dcm_0[i]) > rtp_tolerance);
  }

  // End of RelationalOperator: '<S236>/Compare'

  // Logic: '<S230>/Logical Operator1' incorporates:
  //   RelationalOperator: '<S236>/Compare'

  tmp = rtb_Compare_k[0];
  for (int32_T i = 0; i < 8; i++) {
    tmp = (tmp || rtb_Compare_k[i + 1]);
  }

  // If: '<S227>/If' incorporates:
  //   Abs: '<S231>/Abs1'
  //   Bias: '<S231>/Bias'
  //   Constant: '<S238>/Constant'
  //   Logic: '<S230>/Logical Operator1'
  //   Product: '<S237>/Product'
  //   Product: '<S237>/Product1'
  //   Product: '<S237>/Product2'
  //   Product: '<S237>/Product3'
  //   Product: '<S237>/Product4'
  //   Product: '<S237>/Product5'
  //   RelationalOperator: '<S238>/Compare'
  //   Reshape: '<S237>/Reshape'
  //   Sum: '<S237>/Sum'

  if (std::abs((((((rtu_dcm[0] * rtu_dcm[4] * rtu_dcm[8] - rtu_dcm[0] * rtu_dcm
                    [5] * rtu_dcm[7]) - rtu_dcm[1] * rtu_dcm[3] * rtu_dcm[8]) +
                  rtu_dcm[2] * rtu_dcm[3] * rtu_dcm[7]) + rtu_dcm[1] * rtu_dcm[5]
                 * rtu_dcm[6]) - rtu_dcm[2] * rtu_dcm[4] * rtu_dcm[6]) +
               localP->Bias_Bias) > rtp_tolerance) {
    // Outputs for IfAction SubSystem: '<S227>/If Not Proper' incorporates:
    //   ActionPort: '<S229>/Action Port'

    // If: '<S229>/If' incorporates:
    //   Constant: '<S229>/Constant'

    if (rtp_action == 2.0) {
      // Outputs for IfAction SubSystem: '<S229>/Warning' incorporates:
      //   ActionPort: '<S235>/Action Port'

      // Assertion: '<S235>/Assertion' incorporates:
      //   Constant: '<S229>/Constant1'

      utAssert(localP->Constant1_Value != 0.0);

      // End of Outputs for SubSystem: '<S229>/Warning'
    } else if (rtp_action == 3.0) {
      // Outputs for IfAction SubSystem: '<S229>/Error' incorporates:
      //   ActionPort: '<S234>/Action Port'

      // Assertion: '<S234>/Assertion' incorporates:
      //   Constant: '<S229>/Constant1'

      utAssert(localP->Constant1_Value != 0.0);

      // End of Outputs for SubSystem: '<S229>/Error'
    }

    // End of If: '<S229>/If'
    // End of Outputs for SubSystem: '<S227>/If Not Proper'
  } else if (tmp) {
    // Outputs for IfAction SubSystem: '<S227>/Else If Not Orthogonal' incorporates:
    //   ActionPort: '<S228>/Action Port'

    // If: '<S228>/If' incorporates:
    //   Constant: '<S228>/Constant'

    if (rtp_action == 2.0) {
      // Outputs for IfAction SubSystem: '<S228>/Warning' incorporates:
      //   ActionPort: '<S233>/Action Port'

      // Assertion: '<S233>/Assertion' incorporates:
      //   Constant: '<S228>/Constant1'

      utAssert(localP->Constant1_Value_k != 0.0);

      // End of Outputs for SubSystem: '<S228>/Warning'
    } else if (rtp_action == 3.0) {
      // Outputs for IfAction SubSystem: '<S228>/Error' incorporates:
      //   ActionPort: '<S232>/Action Port'

      // Assertion: '<S232>/Assertion' incorporates:
      //   Constant: '<S228>/Constant1'

      utAssert(localP->Constant1_Value_k != 0.0);

      // End of Outputs for SubSystem: '<S228>/Error'
    }

    // End of If: '<S228>/If'
    // End of Outputs for SubSystem: '<S227>/Else If Not Orthogonal'
  }

  // End of If: '<S227>/If'
}

//
// System initialize for atomic system:
//    '<S297>/Motor_Dynamics'
//    '<S298>/Motor_Dynamics'
//    '<S299>/Motor_Dynamics'
//    '<S300>/Motor_Dynamics'
//    '<S301>/Motor_Dynamics'
//    '<S302>/Motor_Dynamics'
//    '<S303>/Motor_Dynamics'
//    '<S304>/Motor_Dynamics'
//
void MulticopterModelClass::QuadModelv_Motor_Dynamics_Init
  (DW_Motor_Dynamics_QuadModelv_T *localDW, X_Motor_Dynamics_QuadModelv_T
   *localX) const
{
  // InitializeConditions for Integrator: '<S306>/Integrator'
  if (rtmIsFirstInitCond((&QuadModelv_M))) {
    localX->Integrator_CSTATE = 0.0;
  }

  localDW->Integrator_IWORK = 1;

  // End of InitializeConditions for Integrator: '<S306>/Integrator'
}

//
// Outputs for atomic system:
//    '<S297>/Motor_Dynamics'
//    '<S298>/Motor_Dynamics'
//    '<S299>/Motor_Dynamics'
//    '<S300>/Motor_Dynamics'
//    '<S301>/Motor_Dynamics'
//    '<S302>/Motor_Dynamics'
//    '<S303>/Motor_Dynamics'
//    '<S304>/Motor_Dynamics'
//
void MulticopterModelClass::QuadModelv_Motor_Dynamics(real_T rtu_motor_rate_d,
  real_T rtu_ModelInit_RPM, real_T rtu_motorT, B_Motor_Dynamics_QuadModelv_T
  *localB, DW_Motor_Dynamics_QuadModelv_T *localDW,
  X_Motor_Dynamics_QuadModelv_T *localX)
{
  // Integrator: '<S306>/Integrator'
  if (localDW->Integrator_IWORK != 0) {
    localX->Integrator_CSTATE = rtu_ModelInit_RPM;
  }

  // Integrator: '<S306>/Integrator'
  localB->x = localX->Integrator_CSTATE;

  // Product: '<S306>/Divide' incorporates:
  //   Sum: '<S306>/Add'

  localB->dratedt = (rtu_motor_rate_d - localB->x) / rtu_motorT;
}

//
// Update for atomic system:
//    '<S297>/Motor_Dynamics'
//    '<S298>/Motor_Dynamics'
//    '<S299>/Motor_Dynamics'
//    '<S300>/Motor_Dynamics'
//    '<S301>/Motor_Dynamics'
//    '<S302>/Motor_Dynamics'
//    '<S303>/Motor_Dynamics'
//    '<S304>/Motor_Dynamics'
//
void MulticopterModelClass::QuadModel_Motor_Dynamics_Update
  (DW_Motor_Dynamics_QuadModelv_T *localDW)
{
  // Update for Integrator: '<S306>/Integrator'
  localDW->Integrator_IWORK = 0;
}

//
// Derivatives for atomic system:
//    '<S297>/Motor_Dynamics'
//    '<S298>/Motor_Dynamics'
//    '<S299>/Motor_Dynamics'
//    '<S300>/Motor_Dynamics'
//    '<S301>/Motor_Dynamics'
//    '<S302>/Motor_Dynamics'
//    '<S303>/Motor_Dynamics'
//    '<S304>/Motor_Dynamics'
//
void MulticopterModelClass::QuadModelv_Motor_Dynamics_Deriv
  (B_Motor_Dynamics_QuadModelv_T *localB, XDot_Motor_Dynamics_QuadModel_T
   *localXdot)
{
  // Derivatives for Integrator: '<S306>/Integrator'
  localXdot->Integrator_CSTATE = localB->dratedt;
}

//
// System initialize for atomic system:
//    '<S294>/MotorNonlinearDynamic1'
//    '<S294>/MotorNonlinearDynamic2'
//    '<S294>/MotorNonlinearDynamic3'
//    '<S294>/MotorNonlinearDynamic4'
//    '<S294>/MotorNonlinearDynamic5'
//    '<S294>/MotorNonlinearDynamic6'
//    '<S294>/MotorNonlinearDynamic7'
//    '<S294>/MotorNonlinearDynamic8'
//
void MulticopterModelClass::Qua_MotorNonlinearDynamic1_Init
  (DW_MotorNonlinearDynamic1_Qua_T *localDW, X_MotorNonlinearDynamic1_Quad_T
   *localX) const
{
  // SystemInitialize for Atomic SubSystem: '<S297>/Motor_Dynamics'
  QuadModelv_Motor_Dynamics_Init(&localDW->Motor_Dynamics,
    &localX->Motor_Dynamics);

  // End of SystemInitialize for SubSystem: '<S297>/Motor_Dynamics'
}

//
// Outputs for atomic system:
//    '<S294>/MotorNonlinearDynamic1'
//    '<S294>/MotorNonlinearDynamic2'
//    '<S294>/MotorNonlinearDynamic3'
//    '<S294>/MotorNonlinearDynamic4'
//    '<S294>/MotorNonlinearDynamic5'
//    '<S294>/MotorNonlinearDynamic6'
//    '<S294>/MotorNonlinearDynamic7'
//    '<S294>/MotorNonlinearDynamic8'
//
void MulticopterModelClass::QuadMode_MotorNonlinearDynamic1(real_T rtu_Wb,
  real_T rtu_Cr, real_T rtu_motorT, real_T rtu_ModelInit_RPM, real_T rtu_PWM,
  B_MotorNonlinearDynamic1_Quad_T *localB, DW_MotorNonlinearDynamic1_Qua_T
  *localDW, P_MotorNonlinearDynamic1_Quad_T *localP, P_QuadModelv_T
  *QuadModelv_P, X_MotorNonlinearDynamic1_Quad_T *localX)
{
  real_T rtb_Sum1_i;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // Saturate: '<S297>/Signal_Saturation'
    if (rtu_PWM > localP->Signal_Saturation_UpperSat) {
      rtb_Sum1_i = localP->Signal_Saturation_UpperSat;
    } else if (rtu_PWM < localP->Signal_Saturation_LowerSat) {
      rtb_Sum1_i = localP->Signal_Saturation_LowerSat;
    } else {
      rtb_Sum1_i = rtu_PWM;
    }

    // End of Saturate: '<S297>/Signal_Saturation'

    // Product: '<S297>/Product' incorporates:
    //   Constant: '<S305>/Constant'
    //   Product: '<S297>/Product1'
    //   RelationalOperator: '<S305>/Compare'
    //   Sum: '<S297>/Sum1'

    localB->Product = (rtu_Cr * rtb_Sum1_i + rtu_Wb) * static_cast<real_T>
      (rtb_Sum1_i >= QuadModelv_P->ModelParam_motorMinThr);
  }

  // Outputs for Atomic SubSystem: '<S297>/Motor_Dynamics'
  QuadModelv_Motor_Dynamics(localB->Product, rtu_ModelInit_RPM, rtu_motorT,
    &localB->Motor_Dynamics, &localDW->Motor_Dynamics, &localX->Motor_Dynamics);

  // End of Outputs for SubSystem: '<S297>/Motor_Dynamics'
}

//
// Update for atomic system:
//    '<S294>/MotorNonlinearDynamic1'
//    '<S294>/MotorNonlinearDynamic2'
//    '<S294>/MotorNonlinearDynamic3'
//    '<S294>/MotorNonlinearDynamic4'
//    '<S294>/MotorNonlinearDynamic5'
//    '<S294>/MotorNonlinearDynamic6'
//    '<S294>/MotorNonlinearDynamic7'
//    '<S294>/MotorNonlinearDynamic8'
//
void MulticopterModelClass::Q_MotorNonlinearDynamic1_Update
  (DW_MotorNonlinearDynamic1_Qua_T *localDW)
{
  // Update for Atomic SubSystem: '<S297>/Motor_Dynamics'
  QuadModel_Motor_Dynamics_Update(&localDW->Motor_Dynamics);

  // End of Update for SubSystem: '<S297>/Motor_Dynamics'
}

//
// Derivatives for atomic system:
//    '<S294>/MotorNonlinearDynamic1'
//    '<S294>/MotorNonlinearDynamic2'
//    '<S294>/MotorNonlinearDynamic3'
//    '<S294>/MotorNonlinearDynamic4'
//    '<S294>/MotorNonlinearDynamic5'
//    '<S294>/MotorNonlinearDynamic6'
//    '<S294>/MotorNonlinearDynamic7'
//    '<S294>/MotorNonlinearDynamic8'
//
void MulticopterModelClass::Qu_MotorNonlinearDynamic1_Deriv
  (B_MotorNonlinearDynamic1_Quad_T *localB, XDot_MotorNonlinearDynamic1_Q_T
   *localXdot)
{
  // Derivatives for Atomic SubSystem: '<S297>/Motor_Dynamics'
  QuadModelv_Motor_Dynamics_Deriv(&localB->Motor_Dynamics,
    &localXdot->Motor_Dynamics);

  // End of Derivatives for SubSystem: '<S297>/Motor_Dynamics'
}

//
// System initialize for enable system:
//    '<S512>/Distance into gust (y)'
//    '<S512>/Distance into gust (z)'
//
void MulticopterModelClass::QuadMode_Distanceintogusty_Init
  (B_Distanceintogusty_QuadModel_T *localB, P_Distanceintogusty_QuadModel_T
   *localP, X_Distanceintogusty_QuadModel_T *localX)
{
  // InitializeConditions for Integrator: '<S516>/Distance into Gust (x) (Limited to gust length d) ' 
  localX->DistanceintoGustxLimitedtogustl =
    localP->DistanceintoGustxLimitedtogustl;

  // SystemInitialize for Integrator: '<S516>/Distance into Gust (x) (Limited to gust length d) ' incorporates:
  //   Outport: '<S516>/x'

  localB->DistanceintoGustxLimitedtogustl = localP->x_Y0;
}

//
// System reset for enable system:
//    '<S512>/Distance into gust (y)'
//    '<S512>/Distance into gust (z)'
//
void MulticopterModelClass::QuadMod_Distanceintogusty_Reset
  (P_Distanceintogusty_QuadModel_T *localP, X_Distanceintogusty_QuadModel_T
   *localX)
{
  // InitializeConditions for Integrator: '<S516>/Distance into Gust (x) (Limited to gust length d) ' 
  localX->DistanceintoGustxLimitedtogustl =
    localP->DistanceintoGustxLimitedtogustl;
}

//
// Disable for enable system:
//    '<S512>/Distance into gust (y)'
//    '<S512>/Distance into gust (z)'
//
void MulticopterModelClass::QuadM_Distanceintogusty_Disable
  (DW_Distanceintogusty_QuadMode_T *localDW)
{
  localDW->Distanceintogusty_MODE = false;
}

//
// Outputs for enable system:
//    '<S512>/Distance into gust (y)'
//    '<S512>/Distance into gust (z)'
//
void MulticopterModelClass::QuadModelv_Distanceintogusty(boolean_T rtu_Enable,
  real_T rtp_d_m, B_Distanceintogusty_QuadModel_T *localB,
  DW_Distanceintogusty_QuadMode_T *localDW, P_Distanceintogusty_QuadModel_T
  *localP, X_Distanceintogusty_QuadModel_T *localX)
{
  // Outputs for Enabled SubSystem: '<S512>/Distance into gust (y)' incorporates:
  //   EnablePort: '<S516>/Enable'

  if ((rtmIsMajorTimeStep((&QuadModelv_M)) &&
       (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) &&
      rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    if (rtu_Enable) {
      if (!localDW->Distanceintogusty_MODE) {
        QuadMod_Distanceintogusty_Reset(localP, localX);
        localDW->Distanceintogusty_MODE = true;
      }
    } else if (localDW->Distanceintogusty_MODE) {
      QuadM_Distanceintogusty_Disable(localDW);
    }
  }

  if (localDW->Distanceintogusty_MODE) {
    // Integrator: '<S516>/Distance into Gust (x) (Limited to gust length d) '
    // Limited  Integrator
    if (localX->DistanceintoGustxLimitedtogustl >= rtp_d_m) {
      localX->DistanceintoGustxLimitedtogustl = rtp_d_m;
    } else if (localX->DistanceintoGustxLimitedtogustl <=
               localP->DistanceintoGustxLimitedtogus_e) {
      localX->DistanceintoGustxLimitedtogustl =
        localP->DistanceintoGustxLimitedtogus_e;
    }

    // Integrator: '<S516>/Distance into Gust (x) (Limited to gust length d) '
    localB->DistanceintoGustxLimitedtogustl =
      localX->DistanceintoGustxLimitedtogustl;
  }

  // End of Outputs for SubSystem: '<S512>/Distance into gust (y)'
}

//
// Derivatives for enable system:
//    '<S512>/Distance into gust (y)'
//    '<S512>/Distance into gust (z)'
//
void MulticopterModelClass::QuadMod_Distanceintogusty_Deriv(real_T rtu_V, real_T
  rtp_d_m, DW_Distanceintogusty_QuadMode_T *localDW,
  P_Distanceintogusty_QuadModel_T *localP, X_Distanceintogusty_QuadModel_T
  *localX, XDot_Distanceintogusty_QuadMo_T *localXdot)
{
  boolean_T lsat;
  boolean_T usat;
  if (localDW->Distanceintogusty_MODE) {
    // Derivatives for Integrator: '<S516>/Distance into Gust (x) (Limited to gust length d) ' 
    lsat = (localX->DistanceintoGustxLimitedtogustl <=
            localP->DistanceintoGustxLimitedtogus_e);
    usat = (localX->DistanceintoGustxLimitedtogustl >= rtp_d_m);
    if (((!lsat) && (!usat)) || (lsat && (rtu_V > 0.0)) || (usat && (rtu_V < 0.0)))
    {
      localXdot->DistanceintoGustxLimitedtogustl = rtu_V;
    } else {
      // in saturation
      localXdot->DistanceintoGustxLimitedtogustl = 0.0;
    }

    // End of Derivatives for Integrator: '<S516>/Distance into Gust (x) (Limited to gust length d) ' 
  } else {
    localXdot->DistanceintoGustxLimitedtogustl = 0.0;
  }
}

real_T rt_modd_snf(real_T u0, real_T u1)
{
  real_T y;
  y = u0;
  if (u1 == 0.0) {
    if (u0 == 0.0) {
      y = u1;
    }
  } else if (rtIsNaN(u0) || rtIsNaN(u1) || rtIsInf(u0)) {
    y = (rtNaN);
  } else if (u0 == 0.0) {
    y = 0.0 / u1;
  } else if (rtIsInf(u1)) {
    if ((u1 < 0.0) != (u0 < 0.0)) {
      y = u1;
    }
  } else {
    boolean_T yEq;
    y = std::fmod(u0, u1);
    yEq = (y == 0.0);
    if ((!yEq) && (u1 > std::floor(u1))) {
      real_T q;
      q = std::abs(u0 / u1);
      yEq = !(std::abs(q - std::floor(q + 0.5)) > DBL_EPSILON * q);
    }

    if (yEq) {
      y = u1 * 0.0;
    } else if ((u0 < 0.0) != (u1 < 0.0)) {
      y += u1;
    }
  }

  return y;
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = std::atan2(static_cast<real_T>(tmp), static_cast<real_T>(tmp_0));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = std::atan2(u0, u1);
  }

  return y;
}

// Function for MATLAB Function: '<Root>/CollisionDetection'
real_T MulticopterModelClass::QuadModelv_eml_rand_mt19937ar(uint32_T state[625])
{
  real_T r;
  uint32_T u[2];

  // ========================= COPYRIGHT NOTICE ============================
  //  This is a uniform (0,1) pseudorandom number generator based on:
  //
  //  A C-program for MT19937, with initialization improved 2002/1/26.
  //  Coded by Takuji Nishimura and Makoto Matsumoto.
  //
  //  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,
  //  All rights reserved.
  //
  //  Redistribution and use in source and binary forms, with or without
  //  modification, are permitted provided that the following conditions
  //  are met:
  //
  //    1. Redistributions of source code must retain the above copyright
  //       notice, this list of conditions and the following disclaimer.
  //
  //    2. Redistributions in binary form must reproduce the above copyright
  //       notice, this list of conditions and the following disclaimer
  //       in the documentation and/or other materials provided with the
  //       distribution.
  //
  //    3. The names of its contributors may not be used to endorse or
  //       promote products derived from this software without specific
  //       prior written permission.
  //
  //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  //  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  //  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  //  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
  //  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  //  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  //  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  //  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  //  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  //  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  //  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  //
  // =============================   END   =================================
  int32_T exitg1;
  do {
    int32_T k;
    uint32_T mti;
    exitg1 = 0;
    for (k = 0; k < 2; k++) {
      uint32_T y;
      mti = state[624] + 1U;
      if (state[624] + 1U >= 625U) {
        for (int32_T kk = 0; kk < 227; kk++) {
          mti = (state[kk + 1] & 2147483647U) | (state[kk] & 2147483648U);
          if ((mti & 1U) == 0U) {
            mti >>= 1U;
          } else {
            mti = mti >> 1U ^ 2567483615U;
          }

          state[kk] = state[kk + 397] ^ mti;
        }

        for (int32_T kk = 0; kk < 396; kk++) {
          mti = (state[kk + 227] & 2147483648U) | (state[kk + 228] & 2147483647U);
          if ((mti & 1U) == 0U) {
            mti >>= 1U;
          } else {
            mti = mti >> 1U ^ 2567483615U;
          }

          state[kk + 227] = state[kk] ^ mti;
        }

        mti = (state[623] & 2147483648U) | (state[0] & 2147483647U);
        if ((mti & 1U) == 0U) {
          mti >>= 1U;
        } else {
          mti = mti >> 1U ^ 2567483615U;
        }

        state[623] = state[396] ^ mti;
        mti = 1U;
      }

      y = state[static_cast<int32_T>(mti) - 1];
      state[624] = mti;
      y ^= y >> 11U;
      y ^= y << 7U & 2636928640U;
      y ^= y << 15U & 4022730752U;
      u[k] = y >> 18U ^ y;
    }

    r = (static_cast<real_T>(u[0] >> 5U) * 6.7108864E+7 + static_cast<real_T>(u
          [1] >> 6U)) * 1.1102230246251565E-16;
    if (r == 0.0) {
      boolean_T b_isvalid;
      b_isvalid = ((state[624] >= 1U) && (state[624] < 625U));
      if (b_isvalid) {
        boolean_T exitg2;
        b_isvalid = false;
        k = 1;
        exitg2 = false;
        while ((!exitg2) && (k < 625)) {
          if (state[k - 1] == 0U) {
            k++;
          } else {
            b_isvalid = true;
            exitg2 = true;
          }
        }
      }

      if (!b_isvalid) {
        mti = 5489U;
        state[0] = 5489U;
        for (k = 0; k < 623; k++) {
          mti = ((mti >> 30U ^ mti) * 1812433253U + static_cast<uint32_T>(k)) +
            1U;
          state[k + 1] = mti;
        }

        state[624] = 624U;
      }
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return r;
}

// Function for MATLAB Function: '<Root>/CollisionDetection'
real_T MulticopterModelClass::QuadModelv_eml_rand_mcg16807(uint32_T *state)
{
  real_T r;
  int32_T hi;
  uint32_T a;
  uint32_T b;
  hi = static_cast<int32_T>(*state / 127773U);
  a = (*state - static_cast<uint32_T>(hi) * 127773U) * 16807U;
  b = 2836U * static_cast<uint32_T>(hi);
  if (a < b) {
    a = ~(b - a) & 2147483647U;
  } else {
    a -= b;
  }

  r = static_cast<real_T>(a) * 4.6566128752457969E-10;
  *state = a;
  return r;
}

// Function for MATLAB Function: '<Root>/CollisionDetection'
real_T MulticopterModelClass::QuadModelv_rand(void)
{
  real_T r;
  uint32_T b;
  uint32_T c;
  switch (QuadModelv_DW.method_do) {
   case 4U:
    r = QuadModelv_eml_rand_mcg16807(&QuadModelv_DW.state_g3);
    break;

   case 5U:
    b = 69069U * QuadModelv_DW.state_pv[0] + 1234567U;
    c = QuadModelv_DW.state_pv[1] << 13 ^ QuadModelv_DW.state_pv[1];
    c ^= c >> 17;
    c ^= c << 5;
    QuadModelv_DW.state_pv[0] = b;
    QuadModelv_DW.state_pv[1] = c;
    r = static_cast<real_T>(b + c) * 2.328306436538696E-10;
    break;

   default:
    r = QuadModelv_eml_rand_mt19937ar(QuadModelv_DW.state_jn);
    break;
  }

  return r;
}

// Function for MATLAB Function: '<Root>/CollisionDetection'
void MulticopterModelClass::QuadModelv_rand_a(real_T r[3])
{
  uint32_T b;
  uint32_T c;
  switch (QuadModelv_DW.method_do) {
   case 4U:
    r[0] = QuadModelv_eml_rand_mcg16807(&QuadModelv_DW.state_g3);
    r[1] = QuadModelv_eml_rand_mcg16807(&QuadModelv_DW.state_g3);
    r[2] = QuadModelv_eml_rand_mcg16807(&QuadModelv_DW.state_g3);
    break;

   case 5U:
    b = 69069U * QuadModelv_DW.state_pv[0] + 1234567U;
    c = QuadModelv_DW.state_pv[1] << 13 ^ QuadModelv_DW.state_pv[1];
    c ^= c >> 17;
    c ^= c << 5;
    QuadModelv_DW.state_pv[0] = b;
    QuadModelv_DW.state_pv[1] = c;
    r[0] = static_cast<real_T>(b + c) * 2.328306436538696E-10;
    b = 69069U * QuadModelv_DW.state_pv[0] + 1234567U;
    c = QuadModelv_DW.state_pv[1] << 13 ^ QuadModelv_DW.state_pv[1];
    c ^= c >> 17;
    c ^= c << 5;
    QuadModelv_DW.state_pv[0] = b;
    QuadModelv_DW.state_pv[1] = c;
    r[1] = static_cast<real_T>(b + c) * 2.328306436538696E-10;
    b = 69069U * QuadModelv_DW.state_pv[0] + 1234567U;
    c = QuadModelv_DW.state_pv[1] << 13 ^ QuadModelv_DW.state_pv[1];
    c ^= c >> 17;
    c ^= c << 5;
    QuadModelv_DW.state_pv[0] = b;
    QuadModelv_DW.state_pv[1] = c;
    r[2] = static_cast<real_T>(b + c) * 2.328306436538696E-10;
    break;

   default:
    r[0] = QuadModelv_eml_rand_mt19937ar(QuadModelv_DW.state_jn);
    r[1] = QuadModelv_eml_rand_mt19937ar(QuadModelv_DW.state_jn);
    r[2] = QuadModelv_eml_rand_mt19937ar(QuadModelv_DW.state_jn);
    break;
  }
}

// Function for MATLAB Function: '<Root>/Crash_Motor_Fault'
real_T MulticopterModelClass::QuadModelv_rand_g(void)
{
  real_T r;
  uint32_T u[2];
  switch (QuadModelv_DW.method_b) {
   case 4U:
    {
      int32_T k;
      uint32_T mti;
      uint32_T y;
      k = static_cast<int32_T>(QuadModelv_DW.state_h / 127773U);
      mti = (QuadModelv_DW.state_h - static_cast<uint32_T>(k) * 127773U) *
        16807U;
      y = 2836U * static_cast<uint32_T>(k);
      if (mti < y) {
        mti = ~(y - mti) & 2147483647U;
      } else {
        mti -= y;
      }

      r = static_cast<real_T>(mti) * 4.6566128752457969E-10;
      QuadModelv_DW.state_h = mti;
    }
    break;

   case 5U:
    {
      uint32_T mti;
      uint32_T y;
      mti = 69069U * QuadModelv_DW.state_ef[0] + 1234567U;
      y = QuadModelv_DW.state_ef[1] << 13 ^ QuadModelv_DW.state_ef[1];
      y ^= y >> 17;
      y ^= y << 5;
      QuadModelv_DW.state_ef[0] = mti;
      QuadModelv_DW.state_ef[1] = y;
      r = static_cast<real_T>(mti + y) * 2.328306436538696E-10;
    }
    break;

   default:
    {
      // ========================= COPYRIGHT NOTICE ============================ 
      //  This is a uniform (0,1) pseudorandom number generator based on:        
      //                                                                         
      //  A C-program for MT19937, with initialization improved 2002/1/26.       
      //  Coded by Takuji Nishimura and Makoto Matsumoto.                        
      //                                                                         
      //  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      
      //  All rights reserved.                                                   
      //                                                                         
      //  Redistribution and use in source and binary forms, with or without     
      //  modification, are permitted provided that the following conditions     
      //  are met:                                                               
      //                                                                         
      //    1. Redistributions of source code must retain the above copyright    
      //       notice, this list of conditions and the following disclaimer.     
      //                                                                         
      //    2. Redistributions in binary form must reproduce the above copyright 
      //       notice, this list of conditions and the following disclaimer      
      //       in the documentation and/or other materials provided with the     
      //       distribution.                                                     
      //                                                                         
      //    3. The names of its contributors may not be used to endorse or       
      //       promote products derived from this software without specific      
      //       prior written permission.                                         
      //                                                                         
      //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    
      //  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      
      //  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  
      //  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  
      //  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  
      //  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       
      //  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  
      //  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  
      //  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    
      //  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
      //  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
      //                                                                         
      // =============================   END   ================================= 
      int32_T exitg1;
      do {
        int32_T k;
        uint32_T mti;
        exitg1 = 0;
        for (k = 0; k < 2; k++) {
          uint32_T y;
          mti = QuadModelv_DW.state_m[624] + 1U;
          if (QuadModelv_DW.state_m[624] + 1U >= 625U) {
            for (int32_T kk = 0; kk < 227; kk++) {
              mti = (QuadModelv_DW.state_m[kk + 1] & 2147483647U) |
                (QuadModelv_DW.state_m[kk] & 2147483648U);
              if ((mti & 1U) == 0U) {
                mti >>= 1U;
              } else {
                mti = mti >> 1U ^ 2567483615U;
              }

              QuadModelv_DW.state_m[kk] = QuadModelv_DW.state_m[kk + 397] ^ mti;
            }

            for (int32_T kk = 0; kk < 396; kk++) {
              mti = (QuadModelv_DW.state_m[kk + 227] & 2147483648U) |
                (QuadModelv_DW.state_m[kk + 228] & 2147483647U);
              if ((mti & 1U) == 0U) {
                mti >>= 1U;
              } else {
                mti = mti >> 1U ^ 2567483615U;
              }

              QuadModelv_DW.state_m[kk + 227] = QuadModelv_DW.state_m[kk] ^ mti;
            }

            mti = (QuadModelv_DW.state_m[623] & 2147483648U) |
              (QuadModelv_DW.state_m[0] & 2147483647U);
            if ((mti & 1U) == 0U) {
              mti >>= 1U;
            } else {
              mti = mti >> 1U ^ 2567483615U;
            }

            QuadModelv_DW.state_m[623] = QuadModelv_DW.state_m[396] ^ mti;
            mti = 1U;
          }

          y = QuadModelv_DW.state_m[static_cast<int32_T>(mti) - 1];
          QuadModelv_DW.state_m[624] = mti;
          y ^= y >> 11U;
          y ^= y << 7U & 2636928640U;
          y ^= y << 15U & 4022730752U;
          u[k] = y >> 18U ^ y;
        }

        r = (static_cast<real_T>(u[0] >> 5U) * 6.7108864E+7 + static_cast<real_T>
             (u[1] >> 6U)) * 1.1102230246251565E-16;
        if (r == 0.0) {
          boolean_T b_isvalid;
          b_isvalid = ((QuadModelv_DW.state_m[624] >= 1U) &&
                       (QuadModelv_DW.state_m[624] < 625U));
          if (b_isvalid) {
            boolean_T exitg2;
            b_isvalid = false;
            k = 1;
            exitg2 = false;
            while ((!exitg2) && (k < 625)) {
              if (QuadModelv_DW.state_m[k - 1] == 0U) {
                k++;
              } else {
                b_isvalid = true;
                exitg2 = true;
              }
            }
          }

          if (!b_isvalid) {
            mti = 5489U;
            QuadModelv_DW.state_m[0] = 5489U;
            for (k = 0; k < 623; k++) {
              mti = ((mti >> 30U ^ mti) * 1812433253U + static_cast<uint32_T>(k))
                + 1U;
              QuadModelv_DW.state_m[k + 1] = mti;
            }

            QuadModelv_DW.state_m[624] = 624U;
          }
        } else {
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    break;
  }

  return r;
}

// Function for MATLAB Function: '<S293>/MotorFaultModel'
real_T MulticopterModelClass::QuadModelv_rand_f(void)
{
  real_T r;
  uint32_T u[2];
  switch (QuadModelv_DW.method_l1) {
   case 4U:
    {
      int32_T k;
      uint32_T mti;
      uint32_T y;
      k = static_cast<int32_T>(QuadModelv_DW.state_ib / 127773U);
      mti = (QuadModelv_DW.state_ib - static_cast<uint32_T>(k) * 127773U) *
        16807U;
      y = 2836U * static_cast<uint32_T>(k);
      if (mti < y) {
        mti = ~(y - mti) & 2147483647U;
      } else {
        mti -= y;
      }

      r = static_cast<real_T>(mti) * 4.6566128752457969E-10;
      QuadModelv_DW.state_ib = mti;
    }
    break;

   case 5U:
    {
      uint32_T mti;
      uint32_T y;
      mti = 69069U * QuadModelv_DW.state_po[0] + 1234567U;
      y = QuadModelv_DW.state_po[1] << 13 ^ QuadModelv_DW.state_po[1];
      y ^= y >> 17;
      y ^= y << 5;
      QuadModelv_DW.state_po[0] = mti;
      QuadModelv_DW.state_po[1] = y;
      r = static_cast<real_T>(mti + y) * 2.328306436538696E-10;
    }
    break;

   default:
    {
      // ========================= COPYRIGHT NOTICE ============================ 
      //  This is a uniform (0,1) pseudorandom number generator based on:        
      //                                                                         
      //  A C-program for MT19937, with initialization improved 2002/1/26.       
      //  Coded by Takuji Nishimura and Makoto Matsumoto.                        
      //                                                                         
      //  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      
      //  All rights reserved.                                                   
      //                                                                         
      //  Redistribution and use in source and binary forms, with or without     
      //  modification, are permitted provided that the following conditions     
      //  are met:                                                               
      //                                                                         
      //    1. Redistributions of source code must retain the above copyright    
      //       notice, this list of conditions and the following disclaimer.     
      //                                                                         
      //    2. Redistributions in binary form must reproduce the above copyright 
      //       notice, this list of conditions and the following disclaimer      
      //       in the documentation and/or other materials provided with the     
      //       distribution.                                                     
      //                                                                         
      //    3. The names of its contributors may not be used to endorse or       
      //       promote products derived from this software without specific      
      //       prior written permission.                                         
      //                                                                         
      //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    
      //  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      
      //  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  
      //  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  
      //  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  
      //  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       
      //  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  
      //  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  
      //  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    
      //  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
      //  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
      //                                                                         
      // =============================   END   ================================= 
      int32_T exitg1;
      do {
        int32_T k;
        uint32_T mti;
        exitg1 = 0;
        for (k = 0; k < 2; k++) {
          uint32_T y;
          mti = QuadModelv_DW.state_jf[624] + 1U;
          if (QuadModelv_DW.state_jf[624] + 1U >= 625U) {
            for (int32_T kk = 0; kk < 227; kk++) {
              mti = (QuadModelv_DW.state_jf[kk + 1] & 2147483647U) |
                (QuadModelv_DW.state_jf[kk] & 2147483648U);
              if ((mti & 1U) == 0U) {
                mti >>= 1U;
              } else {
                mti = mti >> 1U ^ 2567483615U;
              }

              QuadModelv_DW.state_jf[kk] = QuadModelv_DW.state_jf[kk + 397] ^
                mti;
            }

            for (int32_T kk = 0; kk < 396; kk++) {
              mti = (QuadModelv_DW.state_jf[kk + 227] & 2147483648U) |
                (QuadModelv_DW.state_jf[kk + 228] & 2147483647U);
              if ((mti & 1U) == 0U) {
                mti >>= 1U;
              } else {
                mti = mti >> 1U ^ 2567483615U;
              }

              QuadModelv_DW.state_jf[kk + 227] = QuadModelv_DW.state_jf[kk] ^
                mti;
            }

            mti = (QuadModelv_DW.state_jf[623] & 2147483648U) |
              (QuadModelv_DW.state_jf[0] & 2147483647U);
            if ((mti & 1U) == 0U) {
              mti >>= 1U;
            } else {
              mti = mti >> 1U ^ 2567483615U;
            }

            QuadModelv_DW.state_jf[623] = QuadModelv_DW.state_jf[396] ^ mti;
            mti = 1U;
          }

          y = QuadModelv_DW.state_jf[static_cast<int32_T>(mti) - 1];
          QuadModelv_DW.state_jf[624] = mti;
          y ^= y >> 11U;
          y ^= y << 7U & 2636928640U;
          y ^= y << 15U & 4022730752U;
          u[k] = y >> 18U ^ y;
        }

        r = (static_cast<real_T>(u[0] >> 5U) * 6.7108864E+7 + static_cast<real_T>
             (u[1] >> 6U)) * 1.1102230246251565E-16;
        if (r == 0.0) {
          boolean_T b_isvalid;
          b_isvalid = ((QuadModelv_DW.state_jf[624] >= 1U) &&
                       (QuadModelv_DW.state_jf[624] < 625U));
          if (b_isvalid) {
            boolean_T exitg2;
            b_isvalid = false;
            k = 1;
            exitg2 = false;
            while ((!exitg2) && (k < 625)) {
              if (QuadModelv_DW.state_jf[k - 1] == 0U) {
                k++;
              } else {
                b_isvalid = true;
                exitg2 = true;
              }
            }
          }

          if (!b_isvalid) {
            mti = 5489U;
            QuadModelv_DW.state_jf[0] = 5489U;
            for (k = 0; k < 623; k++) {
              mti = ((mti >> 30U ^ mti) * 1812433253U + static_cast<uint32_T>(k))
                + 1U;
              QuadModelv_DW.state_jf[k + 1] = mti;
            }

            QuadModelv_DW.state_jf[624] = 624U;
          }
        } else {
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    break;
  }

  return r;
}

// Function for MATLAB Function: '<S67>/MATLAB Function'
real_T MulticopterModelClass::QuadModelv_rand_g3(void)
{
  real_T r;
  uint32_T u[2];
  switch (QuadModelv_DW.method_e) {
   case 4U:
    {
      int32_T k;
      uint32_T mti;
      uint32_T y;
      k = static_cast<int32_T>(QuadModelv_DW.state_ny / 127773U);
      mti = (QuadModelv_DW.state_ny - static_cast<uint32_T>(k) * 127773U) *
        16807U;
      y = 2836U * static_cast<uint32_T>(k);
      if (mti < y) {
        mti = ~(y - mti) & 2147483647U;
      } else {
        mti -= y;
      }

      r = static_cast<real_T>(mti) * 4.6566128752457969E-10;
      QuadModelv_DW.state_ny = mti;
    }
    break;

   case 5U:
    {
      uint32_T mti;
      uint32_T y;
      mti = 69069U * QuadModelv_DW.state_km[0] + 1234567U;
      y = QuadModelv_DW.state_km[1] << 13 ^ QuadModelv_DW.state_km[1];
      y ^= y >> 17;
      y ^= y << 5;
      QuadModelv_DW.state_km[0] = mti;
      QuadModelv_DW.state_km[1] = y;
      r = static_cast<real_T>(mti + y) * 2.328306436538696E-10;
    }
    break;

   default:
    {
      // ========================= COPYRIGHT NOTICE ============================ 
      //  This is a uniform (0,1) pseudorandom number generator based on:        
      //                                                                         
      //  A C-program for MT19937, with initialization improved 2002/1/26.       
      //  Coded by Takuji Nishimura and Makoto Matsumoto.                        
      //                                                                         
      //  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      
      //  All rights reserved.                                                   
      //                                                                         
      //  Redistribution and use in source and binary forms, with or without     
      //  modification, are permitted provided that the following conditions     
      //  are met:                                                               
      //                                                                         
      //    1. Redistributions of source code must retain the above copyright    
      //       notice, this list of conditions and the following disclaimer.     
      //                                                                         
      //    2. Redistributions in binary form must reproduce the above copyright 
      //       notice, this list of conditions and the following disclaimer      
      //       in the documentation and/or other materials provided with the     
      //       distribution.                                                     
      //                                                                         
      //    3. The names of its contributors may not be used to endorse or       
      //       promote products derived from this software without specific      
      //       prior written permission.                                         
      //                                                                         
      //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    
      //  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      
      //  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  
      //  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  
      //  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  
      //  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       
      //  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  
      //  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  
      //  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    
      //  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
      //  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
      //                                                                         
      // =============================   END   ================================= 
      int32_T exitg1;
      do {
        int32_T k;
        uint32_T mti;
        exitg1 = 0;
        for (k = 0; k < 2; k++) {
          uint32_T y;
          mti = QuadModelv_DW.state_iu[624] + 1U;
          if (QuadModelv_DW.state_iu[624] + 1U >= 625U) {
            for (int32_T kk = 0; kk < 227; kk++) {
              mti = (QuadModelv_DW.state_iu[kk + 1] & 2147483647U) |
                (QuadModelv_DW.state_iu[kk] & 2147483648U);
              if ((mti & 1U) == 0U) {
                mti >>= 1U;
              } else {
                mti = mti >> 1U ^ 2567483615U;
              }

              QuadModelv_DW.state_iu[kk] = QuadModelv_DW.state_iu[kk + 397] ^
                mti;
            }

            for (int32_T kk = 0; kk < 396; kk++) {
              mti = (QuadModelv_DW.state_iu[kk + 227] & 2147483648U) |
                (QuadModelv_DW.state_iu[kk + 228] & 2147483647U);
              if ((mti & 1U) == 0U) {
                mti >>= 1U;
              } else {
                mti = mti >> 1U ^ 2567483615U;
              }

              QuadModelv_DW.state_iu[kk + 227] = QuadModelv_DW.state_iu[kk] ^
                mti;
            }

            mti = (QuadModelv_DW.state_iu[623] & 2147483648U) |
              (QuadModelv_DW.state_iu[0] & 2147483647U);
            if ((mti & 1U) == 0U) {
              mti >>= 1U;
            } else {
              mti = mti >> 1U ^ 2567483615U;
            }

            QuadModelv_DW.state_iu[623] = QuadModelv_DW.state_iu[396] ^ mti;
            mti = 1U;
          }

          y = QuadModelv_DW.state_iu[static_cast<int32_T>(mti) - 1];
          QuadModelv_DW.state_iu[624] = mti;
          y ^= y >> 11U;
          y ^= y << 7U & 2636928640U;
          y ^= y << 15U & 4022730752U;
          u[k] = y >> 18U ^ y;
        }

        r = (static_cast<real_T>(u[0] >> 5U) * 6.7108864E+7 + static_cast<real_T>
             (u[1] >> 6U)) * 1.1102230246251565E-16;
        if (r == 0.0) {
          boolean_T b_isvalid;
          b_isvalid = ((QuadModelv_DW.state_iu[624] >= 1U) &&
                       (QuadModelv_DW.state_iu[624] < 625U));
          if (b_isvalid) {
            boolean_T exitg2;
            b_isvalid = false;
            k = 1;
            exitg2 = false;
            while ((!exitg2) && (k < 625)) {
              if (QuadModelv_DW.state_iu[k - 1] == 0U) {
                k++;
              } else {
                b_isvalid = true;
                exitg2 = true;
              }
            }
          }

          if (!b_isvalid) {
            mti = 5489U;
            QuadModelv_DW.state_iu[0] = 5489U;
            for (k = 0; k < 623; k++) {
              mti = ((mti >> 30U ^ mti) * 1812433253U + static_cast<uint32_T>(k))
                + 1U;
              QuadModelv_DW.state_iu[k + 1] = mti;
            }

            QuadModelv_DW.state_iu[624] = 624U;
          }
        } else {
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    break;
  }

  return r;
}

void rt_mrdivide_U1d1x3_U2d_9vOrDY9Z(const real_T u0[3], const real_T u1[9],
  real_T y[3])
{
  real_T A[9];
  real_T a21;
  real_T maxval;
  int32_T r1;
  int32_T r2;
  int32_T r3;
  std::memcpy(&A[0], &u1[0], 9U * sizeof(real_T));
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = std::abs(u1[0]);
  a21 = std::abs(u1[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if (std::abs(u1[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  A[r2] = u1[r2] / u1[r1];
  A[r3] /= A[r1];
  A[r2 + 3] -= A[r1 + 3] * A[r2];
  A[r3 + 3] -= A[r1 + 3] * A[r3];
  A[r2 + 6] -= A[r1 + 6] * A[r2];
  A[r3 + 6] -= A[r1 + 6] * A[r3];
  if (std::abs(A[r3 + 3]) > std::abs(A[r2 + 3])) {
    int32_T rtemp;
    rtemp = r2 + 1;
    r2 = r3;
    r3 = rtemp - 1;
  }

  A[r3 + 3] /= A[r2 + 3];
  A[r3 + 6] -= A[r3 + 3] * A[r2 + 6];
  y[r1] = u0[0] / A[r1];
  y[r2] = u0[1] - A[r1 + 3] * y[r1];
  y[r3] = u0[2] - A[r1 + 6] * y[r1];
  y[r2] /= A[r2 + 3];
  y[r3] -= A[r2 + 6] * y[r2];
  y[r3] /= A[r3 + 6];
  y[r2] -= A[r3 + 3] * y[r3];
  y[r1] -= y[r3] * A[r3];
  y[r1] -= y[r2] * A[r2];
}

// Function for MATLAB Function: '<S398>/Acc NoiseFun'
real_T MulticopterModelClass::QuadModelv_rand_e(void)
{
  real_T r;
  uint32_T u[2];
  switch (QuadModelv_DW.method_j) {
   case 4U:
    {
      int32_T k;
      uint32_T mti;
      uint32_T y;
      k = static_cast<int32_T>(QuadModelv_DW.state_d / 127773U);
      mti = (QuadModelv_DW.state_d - static_cast<uint32_T>(k) * 127773U) *
        16807U;
      y = 2836U * static_cast<uint32_T>(k);
      if (mti < y) {
        mti = ~(y - mti) & 2147483647U;
      } else {
        mti -= y;
      }

      r = static_cast<real_T>(mti) * 4.6566128752457969E-10;
      QuadModelv_DW.state_d = mti;
    }
    break;

   case 5U:
    {
      uint32_T mti;
      uint32_T y;
      mti = 69069U * QuadModelv_DW.state_k[0] + 1234567U;
      y = QuadModelv_DW.state_k[1] << 13 ^ QuadModelv_DW.state_k[1];
      y ^= y >> 17;
      y ^= y << 5;
      QuadModelv_DW.state_k[0] = mti;
      QuadModelv_DW.state_k[1] = y;
      r = static_cast<real_T>(mti + y) * 2.328306436538696E-10;
    }
    break;

   default:
    {
      // ========================= COPYRIGHT NOTICE ============================ 
      //  This is a uniform (0,1) pseudorandom number generator based on:        
      //                                                                         
      //  A C-program for MT19937, with initialization improved 2002/1/26.       
      //  Coded by Takuji Nishimura and Makoto Matsumoto.                        
      //                                                                         
      //  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      
      //  All rights reserved.                                                   
      //                                                                         
      //  Redistribution and use in source and binary forms, with or without     
      //  modification, are permitted provided that the following conditions     
      //  are met:                                                               
      //                                                                         
      //    1. Redistributions of source code must retain the above copyright    
      //       notice, this list of conditions and the following disclaimer.     
      //                                                                         
      //    2. Redistributions in binary form must reproduce the above copyright 
      //       notice, this list of conditions and the following disclaimer      
      //       in the documentation and/or other materials provided with the     
      //       distribution.                                                     
      //                                                                         
      //    3. The names of its contributors may not be used to endorse or       
      //       promote products derived from this software without specific      
      //       prior written permission.                                         
      //                                                                         
      //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    
      //  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      
      //  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  
      //  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  
      //  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  
      //  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       
      //  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  
      //  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  
      //  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    
      //  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
      //  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
      //                                                                         
      // =============================   END   ================================= 
      int32_T exitg1;
      do {
        int32_T k;
        uint32_T mti;
        exitg1 = 0;
        for (k = 0; k < 2; k++) {
          uint32_T y;
          mti = QuadModelv_DW.state_f[624] + 1U;
          if (QuadModelv_DW.state_f[624] + 1U >= 625U) {
            for (int32_T kk = 0; kk < 227; kk++) {
              mti = (QuadModelv_DW.state_f[kk + 1] & 2147483647U) |
                (QuadModelv_DW.state_f[kk] & 2147483648U);
              if ((mti & 1U) == 0U) {
                mti >>= 1U;
              } else {
                mti = mti >> 1U ^ 2567483615U;
              }

              QuadModelv_DW.state_f[kk] = QuadModelv_DW.state_f[kk + 397] ^ mti;
            }

            for (int32_T kk = 0; kk < 396; kk++) {
              mti = (QuadModelv_DW.state_f[kk + 227] & 2147483648U) |
                (QuadModelv_DW.state_f[kk + 228] & 2147483647U);
              if ((mti & 1U) == 0U) {
                mti >>= 1U;
              } else {
                mti = mti >> 1U ^ 2567483615U;
              }

              QuadModelv_DW.state_f[kk + 227] = QuadModelv_DW.state_f[kk] ^ mti;
            }

            mti = (QuadModelv_DW.state_f[623] & 2147483648U) |
              (QuadModelv_DW.state_f[0] & 2147483647U);
            if ((mti & 1U) == 0U) {
              mti >>= 1U;
            } else {
              mti = mti >> 1U ^ 2567483615U;
            }

            QuadModelv_DW.state_f[623] = QuadModelv_DW.state_f[396] ^ mti;
            mti = 1U;
          }

          y = QuadModelv_DW.state_f[static_cast<int32_T>(mti) - 1];
          QuadModelv_DW.state_f[624] = mti;
          y ^= y >> 11U;
          y ^= y << 7U & 2636928640U;
          y ^= y << 15U & 4022730752U;
          u[k] = y >> 18U ^ y;
        }

        r = (static_cast<real_T>(u[0] >> 5U) * 6.7108864E+7 + static_cast<real_T>
             (u[1] >> 6U)) * 1.1102230246251565E-16;
        if (r == 0.0) {
          boolean_T b_isvalid;
          b_isvalid = ((QuadModelv_DW.state_f[624] >= 1U) &&
                       (QuadModelv_DW.state_f[624] < 625U));
          if (b_isvalid) {
            boolean_T exitg2;
            b_isvalid = false;
            k = 1;
            exitg2 = false;
            while ((!exitg2) && (k < 625)) {
              if (QuadModelv_DW.state_f[k - 1] == 0U) {
                k++;
              } else {
                b_isvalid = true;
                exitg2 = true;
              }
            }
          }

          if (!b_isvalid) {
            mti = 5489U;
            QuadModelv_DW.state_f[0] = 5489U;
            for (k = 0; k < 623; k++) {
              mti = ((mti >> 30U ^ mti) * 1812433253U + static_cast<uint32_T>(k))
                + 1U;
              QuadModelv_DW.state_f[k + 1] = mti;
            }

            QuadModelv_DW.state_f[624] = 624U;
          }
        } else {
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    break;
  }

  return r;
}

// Function for MATLAB Function: '<S398>/Gyro NoiseFun'
real_T MulticopterModelClass::QuadModelv_rand_i(void)
{
  real_T r;
  uint32_T u[2];
  switch (QuadModelv_DW.method_l) {
   case 4U:
    {
      int32_T k;
      uint32_T mti;
      uint32_T y;
      k = static_cast<int32_T>(QuadModelv_DW.state_e / 127773U);
      mti = (QuadModelv_DW.state_e - static_cast<uint32_T>(k) * 127773U) *
        16807U;
      y = 2836U * static_cast<uint32_T>(k);
      if (mti < y) {
        mti = ~(y - mti) & 2147483647U;
      } else {
        mti -= y;
      }

      r = static_cast<real_T>(mti) * 4.6566128752457969E-10;
      QuadModelv_DW.state_e = mti;
    }
    break;

   case 5U:
    {
      uint32_T mti;
      uint32_T y;
      mti = 69069U * QuadModelv_DW.state_c[0] + 1234567U;
      y = QuadModelv_DW.state_c[1] << 13 ^ QuadModelv_DW.state_c[1];
      y ^= y >> 17;
      y ^= y << 5;
      QuadModelv_DW.state_c[0] = mti;
      QuadModelv_DW.state_c[1] = y;
      r = static_cast<real_T>(mti + y) * 2.328306436538696E-10;
    }
    break;

   default:
    {
      // ========================= COPYRIGHT NOTICE ============================ 
      //  This is a uniform (0,1) pseudorandom number generator based on:        
      //                                                                         
      //  A C-program for MT19937, with initialization improved 2002/1/26.       
      //  Coded by Takuji Nishimura and Makoto Matsumoto.                        
      //                                                                         
      //  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      
      //  All rights reserved.                                                   
      //                                                                         
      //  Redistribution and use in source and binary forms, with or without     
      //  modification, are permitted provided that the following conditions     
      //  are met:                                                               
      //                                                                         
      //    1. Redistributions of source code must retain the above copyright    
      //       notice, this list of conditions and the following disclaimer.     
      //                                                                         
      //    2. Redistributions in binary form must reproduce the above copyright 
      //       notice, this list of conditions and the following disclaimer      
      //       in the documentation and/or other materials provided with the     
      //       distribution.                                                     
      //                                                                         
      //    3. The names of its contributors may not be used to endorse or       
      //       promote products derived from this software without specific      
      //       prior written permission.                                         
      //                                                                         
      //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    
      //  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      
      //  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  
      //  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  
      //  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  
      //  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       
      //  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  
      //  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  
      //  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    
      //  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
      //  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
      //                                                                         
      // =============================   END   ================================= 
      int32_T exitg1;
      do {
        int32_T k;
        uint32_T mti;
        exitg1 = 0;
        for (k = 0; k < 2; k++) {
          uint32_T y;
          mti = QuadModelv_DW.state_n[624] + 1U;
          if (QuadModelv_DW.state_n[624] + 1U >= 625U) {
            for (int32_T kk = 0; kk < 227; kk++) {
              mti = (QuadModelv_DW.state_n[kk + 1] & 2147483647U) |
                (QuadModelv_DW.state_n[kk] & 2147483648U);
              if ((mti & 1U) == 0U) {
                mti >>= 1U;
              } else {
                mti = mti >> 1U ^ 2567483615U;
              }

              QuadModelv_DW.state_n[kk] = QuadModelv_DW.state_n[kk + 397] ^ mti;
            }

            for (int32_T kk = 0; kk < 396; kk++) {
              mti = (QuadModelv_DW.state_n[kk + 227] & 2147483648U) |
                (QuadModelv_DW.state_n[kk + 228] & 2147483647U);
              if ((mti & 1U) == 0U) {
                mti >>= 1U;
              } else {
                mti = mti >> 1U ^ 2567483615U;
              }

              QuadModelv_DW.state_n[kk + 227] = QuadModelv_DW.state_n[kk] ^ mti;
            }

            mti = (QuadModelv_DW.state_n[623] & 2147483648U) |
              (QuadModelv_DW.state_n[0] & 2147483647U);
            if ((mti & 1U) == 0U) {
              mti >>= 1U;
            } else {
              mti = mti >> 1U ^ 2567483615U;
            }

            QuadModelv_DW.state_n[623] = QuadModelv_DW.state_n[396] ^ mti;
            mti = 1U;
          }

          y = QuadModelv_DW.state_n[static_cast<int32_T>(mti) - 1];
          QuadModelv_DW.state_n[624] = mti;
          y ^= y >> 11U;
          y ^= y << 7U & 2636928640U;
          y ^= y << 15U & 4022730752U;
          u[k] = y >> 18U ^ y;
        }

        r = (static_cast<real_T>(u[0] >> 5U) * 6.7108864E+7 + static_cast<real_T>
             (u[1] >> 6U)) * 1.1102230246251565E-16;
        if (r == 0.0) {
          boolean_T b_isvalid;
          b_isvalid = ((QuadModelv_DW.state_n[624] >= 1U) &&
                       (QuadModelv_DW.state_n[624] < 625U));
          if (b_isvalid) {
            boolean_T exitg2;
            b_isvalid = false;
            k = 1;
            exitg2 = false;
            while ((!exitg2) && (k < 625)) {
              if (QuadModelv_DW.state_n[k - 1] == 0U) {
                k++;
              } else {
                b_isvalid = true;
                exitg2 = true;
              }
            }
          }

          if (!b_isvalid) {
            mti = 5489U;
            QuadModelv_DW.state_n[0] = 5489U;
            for (k = 0; k < 623; k++) {
              mti = ((mti >> 30U ^ mti) * 1812433253U + static_cast<uint32_T>(k))
                + 1U;
              QuadModelv_DW.state_n[k + 1] = mti;
            }

            QuadModelv_DW.state_n[624] = 624U;
          }
        } else {
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    break;
  }

  return r;
}

// Function for MATLAB Function: '<S398>/Mag NoiseFun'
real_T MulticopterModelClass::QuadModelv_rand_ah(void)
{
  real_T r;
  uint32_T u[2];
  switch (QuadModelv_DW.method_d) {
   case 4U:
    {
      int32_T k;
      uint32_T mti;
      uint32_T y;
      k = static_cast<int32_T>(QuadModelv_DW.state_g / 127773U);
      mti = (QuadModelv_DW.state_g - static_cast<uint32_T>(k) * 127773U) *
        16807U;
      y = 2836U * static_cast<uint32_T>(k);
      if (mti < y) {
        mti = ~(y - mti) & 2147483647U;
      } else {
        mti -= y;
      }

      r = static_cast<real_T>(mti) * 4.6566128752457969E-10;
      QuadModelv_DW.state_g = mti;
    }
    break;

   case 5U:
    {
      uint32_T mti;
      uint32_T y;
      mti = 69069U * QuadModelv_DW.state_j[0] + 1234567U;
      y = QuadModelv_DW.state_j[1] << 13 ^ QuadModelv_DW.state_j[1];
      y ^= y >> 17;
      y ^= y << 5;
      QuadModelv_DW.state_j[0] = mti;
      QuadModelv_DW.state_j[1] = y;
      r = static_cast<real_T>(mti + y) * 2.328306436538696E-10;
    }
    break;

   default:
    {
      // ========================= COPYRIGHT NOTICE ============================ 
      //  This is a uniform (0,1) pseudorandom number generator based on:        
      //                                                                         
      //  A C-program for MT19937, with initialization improved 2002/1/26.       
      //  Coded by Takuji Nishimura and Makoto Matsumoto.                        
      //                                                                         
      //  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      
      //  All rights reserved.                                                   
      //                                                                         
      //  Redistribution and use in source and binary forms, with or without     
      //  modification, are permitted provided that the following conditions     
      //  are met:                                                               
      //                                                                         
      //    1. Redistributions of source code must retain the above copyright    
      //       notice, this list of conditions and the following disclaimer.     
      //                                                                         
      //    2. Redistributions in binary form must reproduce the above copyright 
      //       notice, this list of conditions and the following disclaimer      
      //       in the documentation and/or other materials provided with the     
      //       distribution.                                                     
      //                                                                         
      //    3. The names of its contributors may not be used to endorse or       
      //       promote products derived from this software without specific      
      //       prior written permission.                                         
      //                                                                         
      //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    
      //  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      
      //  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  
      //  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  
      //  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  
      //  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       
      //  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  
      //  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  
      //  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    
      //  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
      //  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
      //                                                                         
      // =============================   END   ================================= 
      int32_T exitg1;
      do {
        int32_T k;
        uint32_T mti;
        exitg1 = 0;
        for (k = 0; k < 2; k++) {
          uint32_T y;
          mti = QuadModelv_DW.state_b[624] + 1U;
          if (QuadModelv_DW.state_b[624] + 1U >= 625U) {
            for (int32_T kk = 0; kk < 227; kk++) {
              mti = (QuadModelv_DW.state_b[kk + 1] & 2147483647U) |
                (QuadModelv_DW.state_b[kk] & 2147483648U);
              if ((mti & 1U) == 0U) {
                mti >>= 1U;
              } else {
                mti = mti >> 1U ^ 2567483615U;
              }

              QuadModelv_DW.state_b[kk] = QuadModelv_DW.state_b[kk + 397] ^ mti;
            }

            for (int32_T kk = 0; kk < 396; kk++) {
              mti = (QuadModelv_DW.state_b[kk + 227] & 2147483648U) |
                (QuadModelv_DW.state_b[kk + 228] & 2147483647U);
              if ((mti & 1U) == 0U) {
                mti >>= 1U;
              } else {
                mti = mti >> 1U ^ 2567483615U;
              }

              QuadModelv_DW.state_b[kk + 227] = QuadModelv_DW.state_b[kk] ^ mti;
            }

            mti = (QuadModelv_DW.state_b[623] & 2147483648U) |
              (QuadModelv_DW.state_b[0] & 2147483647U);
            if ((mti & 1U) == 0U) {
              mti >>= 1U;
            } else {
              mti = mti >> 1U ^ 2567483615U;
            }

            QuadModelv_DW.state_b[623] = QuadModelv_DW.state_b[396] ^ mti;
            mti = 1U;
          }

          y = QuadModelv_DW.state_b[static_cast<int32_T>(mti) - 1];
          QuadModelv_DW.state_b[624] = mti;
          y ^= y >> 11U;
          y ^= y << 7U & 2636928640U;
          y ^= y << 15U & 4022730752U;
          u[k] = y >> 18U ^ y;
        }

        r = (static_cast<real_T>(u[0] >> 5U) * 6.7108864E+7 + static_cast<real_T>
             (u[1] >> 6U)) * 1.1102230246251565E-16;
        if (r == 0.0) {
          boolean_T b_isvalid;
          b_isvalid = ((QuadModelv_DW.state_b[624] >= 1U) &&
                       (QuadModelv_DW.state_b[624] < 625U));
          if (b_isvalid) {
            boolean_T exitg2;
            b_isvalid = false;
            k = 1;
            exitg2 = false;
            while ((!exitg2) && (k < 625)) {
              if (QuadModelv_DW.state_b[k - 1] == 0U) {
                k++;
              } else {
                b_isvalid = true;
                exitg2 = true;
              }
            }
          }

          if (!b_isvalid) {
            mti = 5489U;
            QuadModelv_DW.state_b[0] = 5489U;
            for (k = 0; k < 623; k++) {
              mti = ((mti >> 30U ^ mti) * 1812433253U + static_cast<uint32_T>(k))
                + 1U;
              QuadModelv_DW.state_b[k + 1] = mti;
            }

            QuadModelv_DW.state_b[624] = 624U;
          }
        } else {
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    break;
  }

  return r;
}

// Function for MATLAB Function: '<S398>/baro NoiseFun'
real_T MulticopterModelClass::QuadModelv_rand_l(void)
{
  real_T r;
  uint32_T u[2];
  switch (QuadModelv_DW.method) {
   case 4U:
    {
      int32_T k;
      uint32_T mti;
      uint32_T y;
      k = static_cast<int32_T>(QuadModelv_DW.state / 127773U);
      mti = (QuadModelv_DW.state - static_cast<uint32_T>(k) * 127773U) * 16807U;
      y = 2836U * static_cast<uint32_T>(k);
      if (mti < y) {
        mti = ~(y - mti) & 2147483647U;
      } else {
        mti -= y;
      }

      r = static_cast<real_T>(mti) * 4.6566128752457969E-10;
      QuadModelv_DW.state = mti;
    }
    break;

   case 5U:
    {
      uint32_T mti;
      uint32_T y;
      mti = 69069U * QuadModelv_DW.state_a[0] + 1234567U;
      y = QuadModelv_DW.state_a[1] << 13 ^ QuadModelv_DW.state_a[1];
      y ^= y >> 17;
      y ^= y << 5;
      QuadModelv_DW.state_a[0] = mti;
      QuadModelv_DW.state_a[1] = y;
      r = static_cast<real_T>(mti + y) * 2.328306436538696E-10;
    }
    break;

   default:
    {
      // ========================= COPYRIGHT NOTICE ============================ 
      //  This is a uniform (0,1) pseudorandom number generator based on:        
      //                                                                         
      //  A C-program for MT19937, with initialization improved 2002/1/26.       
      //  Coded by Takuji Nishimura and Makoto Matsumoto.                        
      //                                                                         
      //  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      
      //  All rights reserved.                                                   
      //                                                                         
      //  Redistribution and use in source and binary forms, with or without     
      //  modification, are permitted provided that the following conditions     
      //  are met:                                                               
      //                                                                         
      //    1. Redistributions of source code must retain the above copyright    
      //       notice, this list of conditions and the following disclaimer.     
      //                                                                         
      //    2. Redistributions in binary form must reproduce the above copyright 
      //       notice, this list of conditions and the following disclaimer      
      //       in the documentation and/or other materials provided with the     
      //       distribution.                                                     
      //                                                                         
      //    3. The names of its contributors may not be used to endorse or       
      //       promote products derived from this software without specific      
      //       prior written permission.                                         
      //                                                                         
      //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    
      //  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      
      //  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  
      //  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  
      //  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  
      //  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       
      //  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  
      //  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  
      //  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    
      //  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
      //  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
      //                                                                         
      // =============================   END   ================================= 
      int32_T exitg1;
      do {
        int32_T k;
        uint32_T mti;
        exitg1 = 0;
        for (k = 0; k < 2; k++) {
          uint32_T y;
          mti = QuadModelv_DW.state_i[624] + 1U;
          if (QuadModelv_DW.state_i[624] + 1U >= 625U) {
            for (int32_T kk = 0; kk < 227; kk++) {
              mti = (QuadModelv_DW.state_i[kk + 1] & 2147483647U) |
                (QuadModelv_DW.state_i[kk] & 2147483648U);
              if ((mti & 1U) == 0U) {
                mti >>= 1U;
              } else {
                mti = mti >> 1U ^ 2567483615U;
              }

              QuadModelv_DW.state_i[kk] = QuadModelv_DW.state_i[kk + 397] ^ mti;
            }

            for (int32_T kk = 0; kk < 396; kk++) {
              mti = (QuadModelv_DW.state_i[kk + 227] & 2147483648U) |
                (QuadModelv_DW.state_i[kk + 228] & 2147483647U);
              if ((mti & 1U) == 0U) {
                mti >>= 1U;
              } else {
                mti = mti >> 1U ^ 2567483615U;
              }

              QuadModelv_DW.state_i[kk + 227] = QuadModelv_DW.state_i[kk] ^ mti;
            }

            mti = (QuadModelv_DW.state_i[623] & 2147483648U) |
              (QuadModelv_DW.state_i[0] & 2147483647U);
            if ((mti & 1U) == 0U) {
              mti >>= 1U;
            } else {
              mti = mti >> 1U ^ 2567483615U;
            }

            QuadModelv_DW.state_i[623] = QuadModelv_DW.state_i[396] ^ mti;
            mti = 1U;
          }

          y = QuadModelv_DW.state_i[static_cast<int32_T>(mti) - 1];
          QuadModelv_DW.state_i[624] = mti;
          y ^= y >> 11U;
          y ^= y << 7U & 2636928640U;
          y ^= y << 15U & 4022730752U;
          u[k] = y >> 18U ^ y;
        }

        r = (static_cast<real_T>(u[0] >> 5U) * 6.7108864E+7 + static_cast<real_T>
             (u[1] >> 6U)) * 1.1102230246251565E-16;
        if (r == 0.0) {
          boolean_T b_isvalid;
          b_isvalid = ((QuadModelv_DW.state_i[624] >= 1U) &&
                       (QuadModelv_DW.state_i[624] < 625U));
          if (b_isvalid) {
            boolean_T exitg2;
            b_isvalid = false;
            k = 1;
            exitg2 = false;
            while ((!exitg2) && (k < 625)) {
              if (QuadModelv_DW.state_i[k - 1] == 0U) {
                k++;
              } else {
                b_isvalid = true;
                exitg2 = true;
              }
            }
          }

          if (!b_isvalid) {
            mti = 5489U;
            QuadModelv_DW.state_i[0] = 5489U;
            for (k = 0; k < 623; k++) {
              mti = ((mti >> 30U ^ mti) * 1812433253U + static_cast<uint32_T>(k))
                + 1U;
              QuadModelv_DW.state_i[k + 1] = mti;
            }

            QuadModelv_DW.state_i[624] = 624U;
          }
        } else {
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    break;
  }

  return r;
}

real_T rt_roundd_snf(real_T u)
{
  real_T y;
  if (std::abs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = std::floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = std::ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

real32_T rt_roundf_snf(real32_T u)
{
  real32_T y;
  if (std::abs(u) < 8.388608E+6F) {
    if (u >= 0.5F) {
      y = std::floor(u + 0.5F);
    } else if (u > -0.5F) {
      y = u * 0.0F;
    } else {
      y = std::ceil(u - 0.5F);
    }
  } else {
    y = u;
  }

  return y;
}

// Function for MATLAB Function: '<S366>/Acc NoiseFun'
real_T MulticopterModelClass::QuadModelv_rand_d(void)
{
  real_T r;
  uint32_T u[2];
  switch (QuadModelv_DW.method_dt) {
   case 4U:
    {
      int32_T k;
      uint32_T mti;
      uint32_T y;
      k = static_cast<int32_T>(QuadModelv_DW.state_ee / 127773U);
      mti = (QuadModelv_DW.state_ee - static_cast<uint32_T>(k) * 127773U) *
        16807U;
      y = 2836U * static_cast<uint32_T>(k);
      if (mti < y) {
        mti = ~(y - mti) & 2147483647U;
      } else {
        mti -= y;
      }

      r = static_cast<real_T>(mti) * 4.6566128752457969E-10;
      QuadModelv_DW.state_ee = mti;
    }
    break;

   case 5U:
    {
      uint32_T mti;
      uint32_T y;
      mti = 69069U * QuadModelv_DW.state_cw[0] + 1234567U;
      y = QuadModelv_DW.state_cw[1] << 13 ^ QuadModelv_DW.state_cw[1];
      y ^= y >> 17;
      y ^= y << 5;
      QuadModelv_DW.state_cw[0] = mti;
      QuadModelv_DW.state_cw[1] = y;
      r = static_cast<real_T>(mti + y) * 2.328306436538696E-10;
    }
    break;

   default:
    {
      // ========================= COPYRIGHT NOTICE ============================ 
      //  This is a uniform (0,1) pseudorandom number generator based on:        
      //                                                                         
      //  A C-program for MT19937, with initialization improved 2002/1/26.       
      //  Coded by Takuji Nishimura and Makoto Matsumoto.                        
      //                                                                         
      //  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      
      //  All rights reserved.                                                   
      //                                                                         
      //  Redistribution and use in source and binary forms, with or without     
      //  modification, are permitted provided that the following conditions     
      //  are met:                                                               
      //                                                                         
      //    1. Redistributions of source code must retain the above copyright    
      //       notice, this list of conditions and the following disclaimer.     
      //                                                                         
      //    2. Redistributions in binary form must reproduce the above copyright 
      //       notice, this list of conditions and the following disclaimer      
      //       in the documentation and/or other materials provided with the     
      //       distribution.                                                     
      //                                                                         
      //    3. The names of its contributors may not be used to endorse or       
      //       promote products derived from this software without specific      
      //       prior written permission.                                         
      //                                                                         
      //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    
      //  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      
      //  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  
      //  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  
      //  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  
      //  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       
      //  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  
      //  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  
      //  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    
      //  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
      //  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
      //                                                                         
      // =============================   END   ================================= 
      int32_T exitg1;
      do {
        int32_T k;
        uint32_T mti;
        exitg1 = 0;
        for (k = 0; k < 2; k++) {
          uint32_T y;
          mti = QuadModelv_DW.state_p[624] + 1U;
          if (QuadModelv_DW.state_p[624] + 1U >= 625U) {
            for (int32_T kk = 0; kk < 227; kk++) {
              mti = (QuadModelv_DW.state_p[kk + 1] & 2147483647U) |
                (QuadModelv_DW.state_p[kk] & 2147483648U);
              if ((mti & 1U) == 0U) {
                mti >>= 1U;
              } else {
                mti = mti >> 1U ^ 2567483615U;
              }

              QuadModelv_DW.state_p[kk] = QuadModelv_DW.state_p[kk + 397] ^ mti;
            }

            for (int32_T kk = 0; kk < 396; kk++) {
              mti = (QuadModelv_DW.state_p[kk + 227] & 2147483648U) |
                (QuadModelv_DW.state_p[kk + 228] & 2147483647U);
              if ((mti & 1U) == 0U) {
                mti >>= 1U;
              } else {
                mti = mti >> 1U ^ 2567483615U;
              }

              QuadModelv_DW.state_p[kk + 227] = QuadModelv_DW.state_p[kk] ^ mti;
            }

            mti = (QuadModelv_DW.state_p[623] & 2147483648U) |
              (QuadModelv_DW.state_p[0] & 2147483647U);
            if ((mti & 1U) == 0U) {
              mti >>= 1U;
            } else {
              mti = mti >> 1U ^ 2567483615U;
            }

            QuadModelv_DW.state_p[623] = QuadModelv_DW.state_p[396] ^ mti;
            mti = 1U;
          }

          y = QuadModelv_DW.state_p[static_cast<int32_T>(mti) - 1];
          QuadModelv_DW.state_p[624] = mti;
          y ^= y >> 11U;
          y ^= y << 7U & 2636928640U;
          y ^= y << 15U & 4022730752U;
          u[k] = y >> 18U ^ y;
        }

        r = (static_cast<real_T>(u[0] >> 5U) * 6.7108864E+7 + static_cast<real_T>
             (u[1] >> 6U)) * 1.1102230246251565E-16;
        if (r == 0.0) {
          boolean_T b_isvalid;
          b_isvalid = ((QuadModelv_DW.state_p[624] >= 1U) &&
                       (QuadModelv_DW.state_p[624] < 625U));
          if (b_isvalid) {
            boolean_T exitg2;
            b_isvalid = false;
            k = 1;
            exitg2 = false;
            while ((!exitg2) && (k < 625)) {
              if (QuadModelv_DW.state_p[k - 1] == 0U) {
                k++;
              } else {
                b_isvalid = true;
                exitg2 = true;
              }
            }
          }

          if (!b_isvalid) {
            mti = 5489U;
            QuadModelv_DW.state_p[0] = 5489U;
            for (k = 0; k < 623; k++) {
              mti = ((mti >> 30U ^ mti) * 1812433253U + static_cast<uint32_T>(k))
                + 1U;
              QuadModelv_DW.state_p[k + 1] = mti;
            }

            QuadModelv_DW.state_p[624] = 624U;
          }
        } else {
          exitg1 = 1;
        }
      } while (exitg1 == 0);
    }
    break;
  }

  return r;
}

// Function for MATLAB Function: '<S12>/Model Fail Assessment'
real_T MulticopterModelClass::QuadModelv_norm(const real_T x[3])
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  scale = 3.3121686421112381E-170;
  absxk = std::abs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = std::abs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = std::abs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * std::sqrt(y);
}

real_T rt_urand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  uint32_T hi;
  uint32_T lo;

  // Uniform random number generator (random number between 0 and 1)

  // #define IA      16807                      magic multiplier = 7^5
  // #define IM      2147483647                 modulus = 2^31-1
  // #define IQ      127773                     IM div IA
  // #define IR      2836                       IM modulo IA
  // #define S       4.656612875245797e-10      reciprocal of 2^31-1
  // test = IA * (seed % IQ) - IR * (seed/IQ)
  // seed = test < 0 ? (test + IM) : test
  // return (seed*S)

  lo = *u % 127773U * 16807U;
  hi = *u / 127773U * 2836U;
  if (lo < hi) {
    *u = 2147483647U - (hi - lo);
  } else {
    *u = lo - hi;
  }

  return static_cast<real_T>(*u) * 4.6566128752457969E-10;
}

real_T rt_nrand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  real_T si;
  real_T sr;
  real_T y;

  // Normal (Gaussian) random number generator
  do {
    sr = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = sr * sr + si * si;
  } while (si > 1.0);

  y = std::sqrt(-2.0 * std::log(si) / si) * sr;
  return y;
}

// Model step function
void MulticopterModelClass::step()
{
  // local block i/o variables
  real_T rtb_DataTypeConversion2[3];

  // local scratch DWork variables
  int32_T ForEach_itr;
  real_T rtb_tc_old[169];
  real_T LoadLeakParams[20];
  real_T LoadShiftParams[20];
  real_T fParamTmp[20];
  real_T rtb_FaultParam[20];
  real_T rtb_MatrixConcatenation[18];
  real_T rtb_TmpSignalConversionAtppnInp[13];
  real_T rtb_Assignment[11];
  real_T rtb_Assignment1[11];
  real_T VectorConcatenate[9];
  real_T VectorConcatenate_p[9];
  real_T rtb_uavlostJ[9];
  real_T rtb_Gain_ok[8];
  real_T ddm[6];
  real_T Product1[3];
  real_T VectorConcatenate_0[3];
  real_T rtb_DataTypeConversion1[3];
  real_T rtb_Mb[3];
  real_T rtb_Sum1_ki[3];
  real_T rtb_sincos_o1_f[3];
  real_T rtb_wind[3];
  real_T tmp[3];
  real_T frac[2];
  real_T frac_0[2];
  real_T rtb_Sum_d[2];
  real_T LoadLeakRatio;
  real_T Merge1;
  real_T U1;
  real_T VectorConcatenate_n_idx_0;
  real_T VectorConcatenate_n_idx_1;
  real_T VectorConcatenate_n_idx_2;
  real_T VectorConcatenate_tmp;
  real_T VectorConcatenate_tmp_0;
  real_T X;
  real_T Z;
  real_T discharge_capacity;
  real_T hover_current;
  real_T rtb_Add;
  real_T rtb_Clock1_tmp;
  real_T rtb_Product1_d;
  real_T rtb_Product2_kl;
  real_T rtb_Product3_o;
  real_T rtb_Rn;
  real_T rtb_Saturation_1;
  real_T rtb_Sum_o1;
  real_T rtb_TmpSignalConversionAtSFun_0;
  real_T rtb_TmpSignalConversionAtSFun_1;
  real_T rtb_q2dot;
  real_T rtb_q3dot;
  real_T rtb_sincos_o1_idx_1;
  real_T tz_idx_0;
  real_T tz_idx_1;
  real_T tz_idx_2;
  real_T tz_idx_3;
  real_T uavlostJyz;
  int32_T fParamTmp_tmp;
  int32_T i;
  int32_T j;
  int32_T qY;
  int32_T qY_0;
  int32_T qY_1;
  int32_T r;
  int32_T rtb_Sum1_i;
  int32_T s458_iter;
  int32_T tmp_1;
  int32_T tmp_2;
  int32_T tmp_3;
  real32_T tmp_0;
  uint32_T bpIndex[2];
  uint32_T bpIndex_0[2];
  uint32_T rtb_time_usec;
  uint32_T seed;
  int8_T c;
  int8_T rtAction;
  int8_T rtb_landed;
  uint8_T ForIterator_IterationMarker[6];
  boolean_T hFaultTmp;
  boolean_T rtb_RelationalOperator;
  boolean_T rtb_hasFault;
  boolean_T tmp_4;
  static const int8_T d[10] = { 3, 3, 4, 4, 6, 6, 6, 8, 8, 8 };

  static const real_T e[80] = { 1.0471975511965976, 1.0471975511965976,
    0.78539816339744828, 1.5707963267948966, 1.5707963267948966, 0.0,
    1.0471975511965976, 0.39269908169872414, 0.0, 0.78539816339744828,
    3.1415926535897931, 3.1415926535897931, 3.9269908169872414, 4.71238898038469,
    4.71238898038469, 3.1415926535897931, 1.0471975511965976, 3.5342917352885173,
    3.1415926535897931, -0.78539816339744828, -1.0471975511965976,
    -1.0471975511965976, 5.497787143782138, 0.0, 5.7595865315812871,
    4.1887902047863905, 3.1415926535897931, 1.1780972450961724,
    0.78539816339744828, 3.9269908169872414, 0.0, 0.0, 2.3561944901923448,
    3.1415926535897931, 2.617993877991494, 1.0471975511965976,
    3.1415926535897931, 2.748893571891069, 2.3561944901923448,
    2.3561944901923448, 0.0, 0.0, 0.0, 0.0, 0.52359877559829893,
    -1.0471975511965976, -1.0471975511965976, -0.39269908169872414,
    -0.78539816339744828, -0.78539816339744828, 0.0, 0.0, 0.0, 0.0,
    3.6651914291880923, 2.0943951023931957, -1.0471975511965976,
    4.3196898986859651, 3.9269908169872414, 0.78539816339744828, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 5.1050880620834143, 4.71238898038469, 2.3561944901923448,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.9634954084936207, 1.5707963267948966,
    3.9269908169872414 };

  static const int8_T c_0[80] = { -1, 1, -1, -1, 1, 1, 1, 1, 1, 1, -1, 1, -1, -1,
    -1, -1, -1, 1, 1, -1, -1, 1, 1, 1, 1, 1, 1, -1, -1, 1, 0, 0, 1, 1, -1, -1,
    -1, -1, -1, -1, 0, 0, 0, 0, -1, -1, 1, -1, -1, 1, 0, 0, 0, 0, 1, 1, -1, -1,
    -1, -1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, -1 };

  real_T *Memory;
  if (rtmIsMajorTimeStep((&QuadModelv_M))) {
    // set solver stop time
    rtsiSetSolverStopTime(&(&QuadModelv_M)->solverInfo,(((&QuadModelv_M)
      ->Timing.clockTick0+1)*(&QuadModelv_M)->Timing.stepSize0));
  }                                    // end MajorTimeStep

  // Update absolute time of base rate at minor time step
  if (rtmIsMinorTimeStep((&QuadModelv_M))) {
    (&QuadModelv_M)->Timing.t[0] = rtsiGetT(&(&QuadModelv_M)->solverInfo);
  }

  // Clock: '<S12>/Clock1' incorporates:
  //   Clock: '<Root>/Clock'
  //   Clock: '<S2>/Clock1'
  //   Clock: '<S512>/Time'
  //   Clock: '<S67>/Clock1'
  //   Clock: '<S6>/Clock'

  rtb_Clock1_tmp = (&QuadModelv_M)->Timing.t[0];

  // DataTypeConversion: '<S12>/Data Type Conversion1' incorporates:
  //   Clock: '<S12>/Clock1'
  //   Gain: '<S12>/Gain_clock'

  rtb_sincos_o1_idx_1 = std::floor(QuadModelv_P.Gain_clock_Gain * rtb_Clock1_tmp);
  if (rtIsNaN(rtb_sincos_o1_idx_1) || rtIsInf(rtb_sincos_o1_idx_1)) {
    rtb_sincos_o1_idx_1 = 0.0;
  } else {
    rtb_sincos_o1_idx_1 = std::fmod(rtb_sincos_o1_idx_1, 4.294967296E+9);
  }

  rtb_time_usec = rtb_sincos_o1_idx_1 < 0.0 ? static_cast<uint32_T>(-
    static_cast<int32_T>(static_cast<uint32_T>(-rtb_sincos_o1_idx_1))) :
    static_cast<uint32_T>(rtb_sincos_o1_idx_1);

  // End of DataTypeConversion: '<S12>/Data Type Conversion1'
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // Gain: '<S25>/1//2' incorporates:
    //   Constant: '<S15>/Initial Euler Angles'

    rtb_sincos_o1_f[0] = QuadModelv_P.u2_Gain * QuadModelv_P.ModelInit_AngEuler
      [2];
    rtb_sincos_o1_f[1] = QuadModelv_P.u2_Gain * QuadModelv_P.ModelInit_AngEuler
      [1];
    rtb_sincos_o1_f[2] = QuadModelv_P.u2_Gain * QuadModelv_P.ModelInit_AngEuler
      [0];

    // Trigonometry: '<S25>/sincos'
    rtb_TmpSignalConversionAtSFun_0 = std::cos(rtb_sincos_o1_f[0]);
    rtb_Product2_kl = std::sin(rtb_sincos_o1_f[0]);
    rtb_TmpSignalConversionAtSFun_1 = std::cos(rtb_sincos_o1_f[1]);
    rtb_Saturation_1 = std::sin(rtb_sincos_o1_f[1]);
    rtb_Rn = std::cos(rtb_sincos_o1_f[2]);
    Z = std::sin(rtb_sincos_o1_f[2]);

    // Fcn: '<S25>/q0' incorporates:
    //   Fcn: '<S25>/q1'
    //   Trigonometry: '<S25>/sincos'

    LoadLeakRatio = rtb_TmpSignalConversionAtSFun_0 *
      rtb_TmpSignalConversionAtSFun_1;
    rtb_Product3_o = rtb_Product2_kl * rtb_Saturation_1;

    // Fcn: '<S25>/q0' incorporates:
    //   Trigonometry: '<S25>/sincos'

    QuadModelv_B.q0 = LoadLeakRatio * rtb_Rn + rtb_Product3_o * Z;

    // Fcn: '<S25>/q1' incorporates:
    //   Trigonometry: '<S25>/sincos'

    QuadModelv_B.q1 = LoadLeakRatio * Z - rtb_Product3_o * rtb_Rn;

    // Fcn: '<S25>/q2' incorporates:
    //   Fcn: '<S25>/q3'
    //   Trigonometry: '<S25>/sincos'

    rtb_Product2_kl *= rtb_TmpSignalConversionAtSFun_1;
    rtb_Saturation_1 *= rtb_TmpSignalConversionAtSFun_0;

    // Fcn: '<S25>/q2' incorporates:
    //   Trigonometry: '<S25>/sincos'

    QuadModelv_B.q2 = rtb_Saturation_1 * rtb_Rn + rtb_Product2_kl * Z;

    // Fcn: '<S25>/q3' incorporates:
    //   Trigonometry: '<S25>/sincos'

    QuadModelv_B.q3 = rtb_Product2_kl * rtb_Rn - rtb_Saturation_1 * Z;
  }

  // Integrator: '<S15>/q0 q1 q2 q3' incorporates:
  //   SignalConversion generated from: '<S15>/q0 q1 q2 q3'

  if (QuadModelv_DW.q0q1q2q3_IWORK != 0) {
    QuadModelv_X.q0q1q2q3_CSTATE[0] = QuadModelv_B.q0;
    QuadModelv_X.q0q1q2q3_CSTATE[1] = QuadModelv_B.q1;
    QuadModelv_X.q0q1q2q3_CSTATE[2] = QuadModelv_B.q2;
    QuadModelv_X.q0q1q2q3_CSTATE[3] = QuadModelv_B.q3;
  }

  // Sqrt: '<S38>/sqrt' incorporates:
  //   Integrator: '<S15>/q0 q1 q2 q3'
  //   Product: '<S39>/Product'
  //   Product: '<S39>/Product1'
  //   Product: '<S39>/Product2'
  //   Product: '<S39>/Product3'
  //   Sum: '<S39>/Sum'

  rtb_Product2_kl = std::sqrt(((QuadModelv_X.q0q1q2q3_CSTATE[0] *
    QuadModelv_X.q0q1q2q3_CSTATE[0] + QuadModelv_X.q0q1q2q3_CSTATE[1] *
    QuadModelv_X.q0q1q2q3_CSTATE[1]) + QuadModelv_X.q0q1q2q3_CSTATE[2] *
    QuadModelv_X.q0q1q2q3_CSTATE[2]) + QuadModelv_X.q0q1q2q3_CSTATE[3] *
    QuadModelv_X.q0q1q2q3_CSTATE[3]);

  // Product: '<S37>/Product' incorporates:
  //   Integrator: '<S15>/q0 q1 q2 q3'

  Z = QuadModelv_X.q0q1q2q3_CSTATE[0] / rtb_Product2_kl;

  // Product: '<S37>/Product1' incorporates:
  //   Integrator: '<S15>/q0 q1 q2 q3'

  rtb_Saturation_1 = QuadModelv_X.q0q1q2q3_CSTATE[1] / rtb_Product2_kl;

  // Product: '<S37>/Product2' incorporates:
  //   Integrator: '<S15>/q0 q1 q2 q3'

  rtb_Product3_o = QuadModelv_X.q0q1q2q3_CSTATE[2] / rtb_Product2_kl;

  // Product: '<S37>/Product3' incorporates:
  //   Integrator: '<S15>/q0 q1 q2 q3'

  rtb_Product2_kl = QuadModelv_X.q0q1q2q3_CSTATE[3] / rtb_Product2_kl;

  // Product: '<S27>/Product3' incorporates:
  //   Product: '<S31>/Product3'

  LoadLeakRatio = Z * Z;

  // Product: '<S27>/Product2' incorporates:
  //   Product: '<S31>/Product2'

  VectorConcatenate_n_idx_0 = rtb_Saturation_1 * rtb_Saturation_1;

  // Product: '<S27>/Product1' incorporates:
  //   Product: '<S31>/Product1'
  //   Product: '<S35>/Product1'

  VectorConcatenate_n_idx_1 = rtb_Product3_o * rtb_Product3_o;

  // Product: '<S27>/Product' incorporates:
  //   Product: '<S31>/Product'
  //   Product: '<S35>/Product'

  VectorConcatenate_n_idx_2 = rtb_Product2_kl * rtb_Product2_kl;

  // Sum: '<S27>/Sum' incorporates:
  //   Concatenate: '<S36>/Vector Concatenate'
  //   Product: '<S27>/Product'
  //   Product: '<S27>/Product1'
  //   Product: '<S27>/Product2'
  //   Product: '<S27>/Product3'

  VectorConcatenate[0] = ((LoadLeakRatio + VectorConcatenate_n_idx_0) -
    VectorConcatenate_n_idx_1) - VectorConcatenate_n_idx_2;

  // Product: '<S30>/Product3' incorporates:
  //   Product: '<S28>/Product3'

  rtb_Add = rtb_Product2_kl * Z;

  // Product: '<S30>/Product2' incorporates:
  //   Product: '<S28>/Product2'

  rtb_TmpSignalConversionAtSFun_0 = rtb_Saturation_1 * rtb_Product3_o;

  // Gain: '<S30>/Gain' incorporates:
  //   Concatenate: '<S36>/Vector Concatenate'
  //   Product: '<S30>/Product2'
  //   Product: '<S30>/Product3'
  //   Sum: '<S30>/Sum'

  VectorConcatenate[1] = (rtb_TmpSignalConversionAtSFun_0 - rtb_Add) *
    QuadModelv_P.Gain_Gain_pu;

  // Product: '<S33>/Product2' incorporates:
  //   Product: '<S29>/Product2'

  VectorConcatenate_tmp = rtb_Saturation_1 * rtb_Product2_kl;

  // Product: '<S33>/Product1' incorporates:
  //   Product: '<S29>/Product1'

  VectorConcatenate_tmp_0 = Z * rtb_Product3_o;

  // Gain: '<S33>/Gain' incorporates:
  //   Concatenate: '<S36>/Vector Concatenate'
  //   Product: '<S33>/Product1'
  //   Product: '<S33>/Product2'
  //   Sum: '<S33>/Sum'

  VectorConcatenate[2] = (VectorConcatenate_tmp_0 + VectorConcatenate_tmp) *
    QuadModelv_P.Gain_Gain_o;

  // Gain: '<S28>/Gain' incorporates:
  //   Concatenate: '<S36>/Vector Concatenate'
  //   Sum: '<S28>/Sum'

  VectorConcatenate[3] = (rtb_Add + rtb_TmpSignalConversionAtSFun_0) *
    QuadModelv_P.Gain_Gain_n;

  // Sum: '<S31>/Sum' incorporates:
  //   Concatenate: '<S36>/Vector Concatenate'
  //   Sum: '<S35>/Sum'

  LoadLeakRatio -= VectorConcatenate_n_idx_0;
  VectorConcatenate[4] = (LoadLeakRatio + VectorConcatenate_n_idx_1) -
    VectorConcatenate_n_idx_2;

  // Product: '<S34>/Product1' incorporates:
  //   Product: '<S32>/Product1'

  VectorConcatenate_n_idx_0 = Z * rtb_Saturation_1;

  // Product: '<S34>/Product2' incorporates:
  //   Product: '<S32>/Product2'

  rtb_Add = rtb_Product3_o * rtb_Product2_kl;

  // Gain: '<S34>/Gain' incorporates:
  //   Concatenate: '<S36>/Vector Concatenate'
  //   Product: '<S34>/Product1'
  //   Product: '<S34>/Product2'
  //   Sum: '<S34>/Sum'

  VectorConcatenate[5] = (rtb_Add - VectorConcatenate_n_idx_0) *
    QuadModelv_P.Gain_Gain_nd;

  // Gain: '<S29>/Gain' incorporates:
  //   Concatenate: '<S36>/Vector Concatenate'
  //   Sum: '<S29>/Sum'

  VectorConcatenate[6] = (VectorConcatenate_tmp - VectorConcatenate_tmp_0) *
    QuadModelv_P.Gain_Gain_af;

  // Gain: '<S32>/Gain' incorporates:
  //   Concatenate: '<S36>/Vector Concatenate'
  //   Sum: '<S32>/Sum'

  VectorConcatenate[7] = (VectorConcatenate_n_idx_0 + rtb_Add) *
    QuadModelv_P.Gain_Gain_c;

  // Sum: '<S35>/Sum' incorporates:
  //   Concatenate: '<S36>/Vector Concatenate'

  VectorConcatenate[8] = (LoadLeakRatio - VectorConcatenate_n_idx_1) +
    VectorConcatenate_n_idx_2;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    for (i = 0; i < 8; i++) {
      // DataTypeConversion: '<Root>/Data Type Conversion3' incorporates:
      //   Constant: '<Root>/Constant'

      rtb_sincos_o1_idx_1 = std::floor(QuadModelv_P.Constant_Value_b[i]);
      if (rtIsNaN(rtb_sincos_o1_idx_1) || rtIsInf(rtb_sincos_o1_idx_1)) {
        rtb_sincos_o1_idx_1 = 0.0;
      } else {
        rtb_sincos_o1_idx_1 = std::fmod(rtb_sincos_o1_idx_1, 4.294967296E+9);
      }

      // DataTypeConversion: '<Root>/Data Type Conversion3'
      QuadModelv_B.inSILInts[i] = rtb_sincos_o1_idx_1 < 0.0 ?
        -static_cast<int32_T>(static_cast<uint32_T>(-rtb_sincos_o1_idx_1)) :
        static_cast<int32_T>(static_cast<uint32_T>(rtb_sincos_o1_idx_1));
    }

    // MATLAB Function: '<S187>/FaultParamsExtract3' incorporates:
    //   Constant: '<Root>/Constant1'
    //   Constant: '<S187>/FaultID'

    // MATLAB Function 'LoadFault/FaultParamsExtract3': '<S191>:1'
    // '<S191>:1:5' if isempty(hFault)
    // '<S191>:1:8' if isempty(fParam)
    // '<S191>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S191>:1:13' fParamTmp=zeros(20,1);
    std::memset(&fParamTmp[0], 0, 20U * sizeof(real_T));

    // '<S191>:1:14' j=1;
    j = 1;

    // '<S191>:1:15' for i=1:8
    for (r = 0; r < 8; r++) {
      // '<S191>:1:16' if inInts(i) == FaultID
      if (QuadModelv_B.inSILInts[r] == QuadModelv_P.FaultID_Value) {
        // '<S191>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S191>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        s458_iter = (r + 1) << 1;
        fParamTmp_tmp = j << 1;
        fParamTmp[fParamTmp_tmp - 2] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 2];

        // '<S191>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[fParamTmp_tmp - 1] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 1];

        // '<S191>:1:20' j=j+1;
        j++;
      }
    }

    // '<S191>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S191>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_j = true;

      // '<S191>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = QuadModelv_P.Constant1_Value_a[16];
      fParamTmp[17] = QuadModelv_P.Constant1_Value_a[17];
      fParamTmp[18] = QuadModelv_P.Constant1_Value_a[18];
      fParamTmp[19] = QuadModelv_P.Constant1_Value_a[19];

      // '<S191>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_p1[0], &fParamTmp[0], 20U * sizeof
                  (real_T));
    }

    // MATLAB Function: '<S187>/FaultParamsExtract1' incorporates:
    //   Constant: '<Root>/Constant1'
    //   Constant: '<S187>/FaultID1'

    // '<S191>:1:29' hasFault_isloadFall=hFault;
    // '<S191>:1:30' LoadFallFaultParam=fParam;
    // MATLAB Function 'LoadFault/FaultParamsExtract1': '<S189>:1'
    // '<S189>:1:5' if isempty(hFault)
    // '<S189>:1:8' if isempty(fParam)
    // '<S189>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S189>:1:13' fParamTmp=zeros(20,1);
    std::memset(&fParamTmp[0], 0, 20U * sizeof(real_T));

    // '<S189>:1:14' j=1;
    j = 1;

    // '<S189>:1:15' for i=1:8
    for (r = 0; r < 8; r++) {
      // '<S189>:1:16' if inInts(i) == FaultID
      if (QuadModelv_B.inSILInts[r] == QuadModelv_P.FaultID1_Value) {
        // '<S189>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S189>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        s458_iter = (r + 1) << 1;
        fParamTmp_tmp = j << 1;
        fParamTmp[fParamTmp_tmp - 2] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 2];

        // '<S189>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[fParamTmp_tmp - 1] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 1];

        // '<S189>:1:20' j=j+1;
        j++;
      }
    }

    // '<S189>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S189>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_om = true;

      // '<S189>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = QuadModelv_P.Constant1_Value_a[16];
      fParamTmp[17] = QuadModelv_P.Constant1_Value_a[17];
      fParamTmp[18] = QuadModelv_P.Constant1_Value_a[18];
      fParamTmp[19] = QuadModelv_P.Constant1_Value_a[19];

      // '<S189>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_ai[0], &fParamTmp[0], 20U * sizeof
                  (real_T));
    }

    // MATLAB Function: '<S187>/FaultParamsExtract2' incorporates:
    //   Constant: '<Root>/Constant1'
    //   Constant: '<S187>/FaultID2'

    // '<S189>:1:29' hasFault_isloadShift=hFault;
    // '<S189>:1:30' LoadShiftFaultParam=fParam;
    // MATLAB Function 'LoadFault/FaultParamsExtract2': '<S190>:1'
    // '<S190>:1:5' if isempty(hFault)
    // '<S190>:1:8' if isempty(fParam)
    // '<S190>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S190>:1:13' fParamTmp=zeros(20,1);
    std::memset(&fParamTmp[0], 0, 20U * sizeof(real_T));

    // '<S190>:1:14' j=1;
    j = 1;

    // '<S190>:1:15' for i=1:8
    for (r = 0; r < 8; r++) {
      // '<S190>:1:16' if inInts(i) == FaultID
      if (QuadModelv_B.inSILInts[r] == QuadModelv_P.FaultID2_Value) {
        // '<S190>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S190>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        s458_iter = (r + 1) << 1;
        fParamTmp_tmp = j << 1;
        fParamTmp[fParamTmp_tmp - 2] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 2];

        // '<S190>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[fParamTmp_tmp - 1] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 1];

        // '<S190>:1:20' j=j+1;
        j++;
      }
    }

    // '<S190>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S190>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_n = true;

      // '<S190>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = QuadModelv_P.Constant1_Value_a[16];
      fParamTmp[17] = QuadModelv_P.Constant1_Value_a[17];
      fParamTmp[18] = QuadModelv_P.Constant1_Value_a[18];
      fParamTmp[19] = QuadModelv_P.Constant1_Value_a[19];

      // '<S190>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_an[0], &fParamTmp[0], 20U * sizeof
                  (real_T));
    }

    // MATLAB Function: '<S6>/Model Failure' incorporates:
    //   Concatenate: '<S36>/Vector Concatenate'
    //   Constant: '<S6>/ModelParam.uavMass'
    //   Constant: '<S6>/ModelParam.uavR'
    //   MATLAB Function: '<S187>/FaultParamsExtract1'
    //   MATLAB Function: '<S187>/FaultParamsExtract2'
    //   MATLAB Function: '<S187>/FaultParamsExtract3'

    // '<S190>:1:29' hasFault_isloadLeak=hFault;
    // '<S190>:1:30' LoadleakFaultParam=fParam;
    std::memcpy(&fParamTmp[0], &QuadModelv_DW.fParam_p1[0], 20U * sizeof(real_T));
    std::memcpy(&LoadShiftParams[0], &QuadModelv_DW.fParam_ai[0], 20U * sizeof
                (real_T));
    std::memcpy(&LoadLeakParams[0], &QuadModelv_DW.fParam_an[0], 20U * sizeof
                (real_T));

    //  This function aims to calculate the variation of mass and moment of
    //  inertia caused by model failure including LoadFall, LoadShift, and
    //  LoadLeak.
    // MATLAB Function 'Fail Model/Model Failure': '<S188>:1'
    // '<S188>:1:7' if isempty(tOld)
    // '<S188>:1:11' deltaM=[0;0;0];
    QuadModelv_B.deltaM[0] = 0.0;
    QuadModelv_B.deltaM[1] = 0.0;
    QuadModelv_B.deltaM[2] = 0.0;

    // '<S188>:1:13' if isLoadFall
    if (QuadModelv_DW.hFault_j) {
      // '<S188>:1:14' if LoadFallParams(1)>0.95
      if (QuadModelv_DW.fParam_p1[0] > 0.95) {
        // '<S188>:1:15' LoadFallParams(1)=0.95;
        fParamTmp[0] = 0.95;
      } else if (QuadModelv_DW.fParam_p1[0] < 0.0) {
        // '<S188>:1:16' elseif LoadFallParams(1)<0
        // '<S188>:1:17' LoadFallParams(1)=0;
        fParamTmp[0] = 0.0;
      }

      // '<S188>:1:19' uavlostMass=LoadFallParams(1)*uavMass;
      QuadModelv_B.uavlostMass = fParamTmp[0] * QuadModelv_P.ModelParam_uavMass;

      // '<S188>:1:20' uavlostJ=zeros(3,3);
      std::memset(&rtb_uavlostJ[0], 0, 9U * sizeof(real_T));
    } else if (QuadModelv_DW.hFault_om) {
      // '<S188>:1:21' elseif isLoadShift
      // '<S188>:1:22' if LoadShiftParams(1)>0.95
      if (QuadModelv_DW.fParam_ai[0] > 0.95) {
        // '<S188>:1:23' LoadShiftParams(1)=0.95;
        LoadShiftParams[0] = 0.95;
      } else if (QuadModelv_DW.fParam_ai[0] < 0.0) {
        // '<S188>:1:24' elseif LoadShiftParams(1)<0
        // '<S188>:1:25' LoadShiftParams(1)=0;
        LoadShiftParams[0] = 0.0;
      }

      // '<S188>:1:28' uavlostMass= 0;
      QuadModelv_B.uavlostMass = 0.0;

      // '<S188>:1:29' loadMoveMass= LoadShiftParams(1)*uavMass;
      LoadLeakRatio = LoadShiftParams[0] * QuadModelv_P.ModelParam_uavMass;

      // '<S188>:1:31' X=uavR*LoadShiftParams(2);
      X = QuadModelv_P.ModelParam_uavR * QuadModelv_DW.fParam_ai[1];

      // '<S188>:1:32' Y=uavR*LoadShiftParams(3);
      rtb_Saturation_1 = QuadModelv_P.ModelParam_uavR * QuadModelv_DW.fParam_ai
        [2];

      // '<S188>:1:33' Z=uavR*LoadShiftParams(4);
      Z = QuadModelv_P.ModelParam_uavR * QuadModelv_DW.fParam_ai[3];

      // '<S188>:1:35' uavlostJxx= loadMoveMass*(Y^2+Z^2);
      // '<S188>:1:36' uavlostJyy= loadMoveMass*(X^2+Z^2);
      // '<S188>:1:37' uavlostJzz= loadMoveMass*(X^2+Y^2);
      // '<S188>:1:38' uavlostJyz= loadMoveMass*Y*Z;
      uavlostJyz = LoadLeakRatio * rtb_Saturation_1 * Z;

      // '<S188>:1:39' uavlostJzx= loadMoveMass*Z*X;
      rtb_Product2_kl = LoadLeakRatio * Z * X;

      // '<S188>:1:40' uavlostJxy= loadMoveMass*X*Y;
      rtb_Product3_o = LoadLeakRatio * X * rtb_Saturation_1;

      // '<S188>:1:41' uavlostJ=-[uavlostJxx,-uavlostJxy,-uavlostJzx;-uavlostJxy,uavlostJyy,-uavlostJyz;-uavlostJzx,-uavlostJyz,uavlostJzz]; 
      VectorConcatenate_n_idx_0 = Z * Z;
      VectorConcatenate_n_idx_1 = rtb_Saturation_1 * rtb_Saturation_1;
      rtb_uavlostJ[0] = -((VectorConcatenate_n_idx_1 + VectorConcatenate_n_idx_0)
                          * LoadLeakRatio);
      rtb_uavlostJ[3] = rtb_Product3_o;
      rtb_uavlostJ[6] = rtb_Product2_kl;
      rtb_uavlostJ[1] = rtb_Product3_o;
      rtb_Product3_o = X * X;
      rtb_uavlostJ[4] = -((rtb_Product3_o + VectorConcatenate_n_idx_0) *
                          LoadLeakRatio);
      rtb_uavlostJ[7] = uavlostJyz;
      rtb_uavlostJ[2] = rtb_Product2_kl;
      rtb_uavlostJ[5] = uavlostJyz;
      rtb_uavlostJ[8] = -((rtb_Product3_o + VectorConcatenate_n_idx_1) *
                          LoadLeakRatio);

      // '<S188>:1:43' deltaF = dcm*loadMoveMass*[0;0;9.8];
      for (i = 0; i < 3; i++) {
        rtb_sincos_o1_f[i] = (VectorConcatenate[i + 3] * LoadLeakRatio * 0.0 +
                              VectorConcatenate[i] * LoadLeakRatio * 0.0) +
          VectorConcatenate[i + 6] * LoadLeakRatio * 9.8;
      }

      // '<S188>:1:45' deltaM = cross([X;Y;Z],deltaF);
      QuadModelv_B.deltaM[0] = rtb_Saturation_1 * rtb_sincos_o1_f[2] -
        rtb_sincos_o1_f[1] * Z;
      QuadModelv_B.deltaM[1] = rtb_sincos_o1_f[0] * Z - X * rtb_sincos_o1_f[2];
      QuadModelv_B.deltaM[2] = X * rtb_sincos_o1_f[1] - rtb_sincos_o1_f[0] *
        rtb_Saturation_1;

      //  变化的力矩
    } else if (QuadModelv_DW.hFault_n) {
      // '<S188>:1:47' elseif isLoadLeak
      // '<S188>:1:48' uavlostJ=zeros(3,3);
      std::memset(&rtb_uavlostJ[0], 0, 9U * sizeof(real_T));

      // '<S188>:1:49' if LoadLeakParams(1)>0.99
      if (QuadModelv_DW.fParam_an[0] > 0.99) {
        // '<S188>:1:50' LoadLeakParams(1)=0.99;
        LoadLeakParams[0] = 0.99;
      } else if (QuadModelv_DW.fParam_an[0] < 0.0) {
        // '<S188>:1:51' elseif LoadLeakParams(1)<0
        // '<S188>:1:52' LoadLeakParams(1)=0;
        LoadLeakParams[0] = 0.0;
      }

      // '<S188>:1:54' loadmass0=LoadLeakParams(1)*uavMass;
      // '<S188>:1:56' if tOld<0.01
      if (QuadModelv_DW.tOld < 0.01) {
        // '<S188>:1:57' tOld=T;
        QuadModelv_DW.tOld = rtb_Clock1_tmp;
      }

      // '<S188>:1:60' dt=T-tOld;
      // '<S188>:1:62' LoadLeakRatio = LoadLeakParams(2)*dt;
      LoadLeakRatio = (rtb_Clock1_tmp - QuadModelv_DW.tOld) *
        QuadModelv_DW.fParam_an[1];

      // '<S188>:1:63' if LoadLeakRatio > 0.99
      if (LoadLeakRatio > 0.99) {
        // '<S188>:1:64' LoadLeakRatio = 0.99;
        LoadLeakRatio = 0.99;
      }

      // '<S188>:1:66' uavlostMass = LoadLeakRatio*loadmass0;
      QuadModelv_B.uavlostMass = LoadLeakParams[0] *
        QuadModelv_P.ModelParam_uavMass * LoadLeakRatio;
    } else {
      // '<S188>:1:68' else
      // '<S188>:1:69' uavlostMass=0;
      QuadModelv_B.uavlostMass = 0.0;

      // '<S188>:1:70' uavlostJ=zeros(3,3);
      std::memset(&rtb_uavlostJ[0], 0, 9U * sizeof(real_T));

      // '<S188>:1:71' tOld=0;
      QuadModelv_DW.tOld = 0.0;
    }

    // '<S188>:1:74' if ~isLoadLeak
    if (!QuadModelv_DW.hFault_n) {
      // reset
      // '<S188>:1:75' tOld=0;
      QuadModelv_DW.tOld = 0.0;
    }

    // End of MATLAB Function: '<S6>/Model Failure'

    // Sum: '<S6>/Add5' incorporates:
    //   Constant: '<S6>/ModelParam.uavMass'

    QuadModelv_B.m = QuadModelv_P.ModelParam_uavMass - QuadModelv_B.uavlostMass;

    // UnitConversion: '<S85>/Unit Conversion' incorporates:
    //   Constant: '<S63>/ref_rotation'

    // Unit Conversion - from: deg to: rad
    // Expression: output = (0.0174533*input) + (0)
    rtb_Rn = 0.017453292519943295 * QuadModelv_P.FlatEarthtoLLA_psi;

    // Trigonometry: '<S70>/SinCos'
    QuadModelv_B.SinCos_o1 = std::sin(rtb_Rn);

    // Trigonometry: '<S70>/SinCos'
    QuadModelv_B.SinCos_o2 = std::cos(rtb_Rn);

    // Sum: '<S88>/Sum' incorporates:
    //   Constant: '<S88>/Constant'
    //   Constant: '<S88>/f'

    rtb_Rn = QuadModelv_P.f_Value - QuadModelv_P.Constant_Value_c;

    // Sqrt: '<S89>/sqrt' incorporates:
    //   Constant: '<S89>/Constant'
    //   Product: '<S89>/Product1'
    //   Sum: '<S89>/Sum1'

    rtb_Rn = std::sqrt(QuadModelv_P.Constant_Value_n5 - rtb_Rn * rtb_Rn);

    // Switch: '<S81>/Switch' incorporates:
    //   Abs: '<S81>/Abs'
    //   Bias: '<S81>/Bias'
    //   Bias: '<S81>/Bias1'
    //   Constant: '<S63>/ref_position'
    //   Constant: '<S81>/Constant2'
    //   Constant: '<S82>/Constant'
    //   Math: '<S81>/Math Function1'
    //   RelationalOperator: '<S82>/Compare'

    if (std::abs(QuadModelv_P.ModelParam_GPSLatLong[0]) >
        QuadModelv_P.CompareToConstant_const) {
      LoadLeakRatio = rt_modd_snf(QuadModelv_P.ModelParam_GPSLatLong[0] +
        QuadModelv_P.Bias_Bias_m, QuadModelv_P.Constant2_Value) +
        QuadModelv_P.Bias1_Bias_h;
    } else {
      LoadLeakRatio = QuadModelv_P.ModelParam_GPSLatLong[0];
    }

    // End of Switch: '<S81>/Switch'

    // Abs: '<S78>/Abs1'
    rtb_Saturation_1 = std::abs(LoadLeakRatio);

    // RelationalOperator: '<S80>/Compare' incorporates:
    //   Constant: '<S80>/Constant'

    hFaultTmp = (rtb_Saturation_1 > QuadModelv_P.CompareToConstant_const_i);

    // Switch: '<S78>/Switch'
    if (hFaultTmp) {
      // Signum: '<S78>/Sign1'
      if (rtIsNaN(LoadLeakRatio)) {
        rtb_sincos_o1_idx_1 = (rtNaN);
      } else if (LoadLeakRatio < 0.0) {
        rtb_sincos_o1_idx_1 = -1.0;
      } else {
        rtb_sincos_o1_idx_1 = (LoadLeakRatio > 0.0);
      }

      // Switch: '<S78>/Switch' incorporates:
      //   Bias: '<S78>/Bias'
      //   Bias: '<S78>/Bias1'
      //   Gain: '<S78>/Gain'
      //   Product: '<S78>/Divide1'
      //   Signum: '<S78>/Sign1'

      QuadModelv_B.Switch = ((rtb_Saturation_1 + QuadModelv_P.Bias_Bias_g) *
        QuadModelv_P.Gain_Gain_m + QuadModelv_P.Bias1_Bias_g) *
        rtb_sincos_o1_idx_1;
    } else {
      // Switch: '<S78>/Switch'
      QuadModelv_B.Switch = LoadLeakRatio;
    }

    // End of Switch: '<S78>/Switch'

    // UnitConversion: '<S86>/Unit Conversion'
    // Unit Conversion - from: deg to: rad
    // Expression: output = (0.0174533*input) + (0)
    rtb_TmpSignalConversionAtSFun_1 = 0.017453292519943295 * QuadModelv_B.Switch;

    // Trigonometry: '<S87>/Trigonometric Function1'
    rtb_Product1_d = std::sin(rtb_TmpSignalConversionAtSFun_1);

    // Product: '<S87>/Product1' incorporates:
    //   Product: '<S84>/Product2'

    Z = rtb_Rn * rtb_Rn;

    // Sum: '<S87>/Sum1' incorporates:
    //   Constant: '<S87>/Constant'
    //   Product: '<S87>/Product1'

    rtb_Product1_d = QuadModelv_P.Constant_Value_kx - Z * rtb_Product1_d *
      rtb_Product1_d;

    // Product: '<S84>/Product1' incorporates:
    //   Constant: '<S84>/Constant1'
    //   Sqrt: '<S84>/sqrt'

    uavlostJyz = QuadModelv_P.Constant1_Value_d / std::sqrt(rtb_Product1_d);

    // Trigonometry: '<S84>/Trigonometric Function1' incorporates:
    //   Constant: '<S84>/Constant'
    //   Constant: '<S84>/Constant2'
    //   Product: '<S84>/Product3'
    //   Sum: '<S84>/Sum1'

    QuadModelv_B.TrigonometricFunction1 = rt_atan2d_snf
      (QuadModelv_P.Constant2_Value_mm, (QuadModelv_P.Constant_Value_lo - Z) *
       uavlostJyz / rtb_Product1_d);

    // Trigonometry: '<S84>/Trigonometric Function2' incorporates:
    //   Constant: '<S84>/Constant3'
    //   Product: '<S84>/Product4'
    //   Trigonometry: '<S84>/Trigonometric Function'

    QuadModelv_B.TrigonometricFunction2 = rt_atan2d_snf
      (QuadModelv_P.Constant3_Value, uavlostJyz * std::cos
       (rtb_TmpSignalConversionAtSFun_1));

    // Switch: '<S69>/Switch1' incorporates:
    //   Constant: '<S69>/Constant'
    //   Constant: '<S69>/Constant1'

    if (hFaultTmp) {
      rtb_sincos_o1_idx_1 = QuadModelv_P.Constant_Value_n;
    } else {
      rtb_sincos_o1_idx_1 = QuadModelv_P.Constant1_Value;
    }

    // Sum: '<S69>/Sum' incorporates:
    //   Constant: '<S63>/ref_position'
    //   Switch: '<S69>/Switch1'

    LoadLeakRatio = rtb_sincos_o1_idx_1 + QuadModelv_P.ModelParam_GPSLatLong[1];

    // Switch: '<S79>/Switch' incorporates:
    //   Abs: '<S79>/Abs'
    //   Constant: '<S83>/Constant'
    //   RelationalOperator: '<S83>/Compare'

    if (std::abs(LoadLeakRatio) > QuadModelv_P.CompareToConstant_const_j) {
      // Switch: '<S79>/Switch' incorporates:
      //   Bias: '<S79>/Bias'
      //   Bias: '<S79>/Bias1'
      //   Constant: '<S79>/Constant2'
      //   Math: '<S79>/Math Function1'

      QuadModelv_B.Switch_a = rt_modd_snf(LoadLeakRatio +
        QuadModelv_P.Bias_Bias_c, QuadModelv_P.Constant2_Value_b) +
        QuadModelv_P.Bias1_Bias_n;
    } else {
      // Switch: '<S79>/Switch'
      QuadModelv_B.Switch_a = LoadLeakRatio;
    }

    // End of Switch: '<S79>/Switch'
  }

  // Sum: '<S63>/Sum' incorporates:
  //   Integrator: '<S14>/xe,ye,ze'
  //   Product: '<S70>/rad lat'
  //   Product: '<S70>/x*cos'
  //   Product: '<S70>/y*sin'
  //   Sum: '<S70>/Sum'
  //   UnitConversion: '<S71>/Unit Conversion'

  // Unit Conversion - from: rad to: deg
  // Expression: output = (57.2958*input) + (0)
  rtb_Sum_d[0] = (QuadModelv_X.xeyeze_CSTATE[0] * QuadModelv_B.SinCos_o2 -
                  QuadModelv_X.xeyeze_CSTATE[1] * QuadModelv_B.SinCos_o1) *
    QuadModelv_B.TrigonometricFunction1 * 57.295779513082323 +
    QuadModelv_B.Switch;

  // Switch: '<S75>/Switch' incorporates:
  //   Abs: '<S75>/Abs'
  //   Bias: '<S75>/Bias'
  //   Bias: '<S75>/Bias1'
  //   Constant: '<S75>/Constant2'
  //   Constant: '<S76>/Constant'
  //   Math: '<S75>/Math Function1'
  //   RelationalOperator: '<S76>/Compare'

  if (std::abs(rtb_Sum_d[0]) > QuadModelv_P.CompareToConstant_const_ji) {
    LoadLeakRatio = rt_modd_snf(rtb_Sum_d[0] + QuadModelv_P.Bias_Bias_i,
      QuadModelv_P.Constant2_Value_n) + QuadModelv_P.Bias1_Bias_o;
  } else {
    LoadLeakRatio = rtb_Sum_d[0];
  }

  // End of Switch: '<S75>/Switch'

  // Abs: '<S72>/Abs1'
  rtb_Saturation_1 = std::abs(LoadLeakRatio);

  // Switch: '<S72>/Switch' incorporates:
  //   Bias: '<S72>/Bias'
  //   Bias: '<S72>/Bias1'
  //   Constant: '<S68>/Constant'
  //   Constant: '<S68>/Constant1'
  //   Constant: '<S74>/Constant'
  //   Gain: '<S72>/Gain'
  //   Product: '<S72>/Divide1'
  //   RelationalOperator: '<S74>/Compare'
  //   Signum: '<S72>/Sign1'
  //   Switch: '<S68>/Switch1'

  if (rtb_Saturation_1 > QuadModelv_P.CompareToConstant_const_k) {
    // Signum: '<S72>/Sign1'
    if (rtIsNaN(LoadLeakRatio)) {
      rtb_sincos_o1_idx_1 = (rtNaN);
    } else if (LoadLeakRatio < 0.0) {
      rtb_sincos_o1_idx_1 = -1.0;
    } else {
      rtb_sincos_o1_idx_1 = (LoadLeakRatio > 0.0);
    }

    LoadLeakRatio = ((rtb_Saturation_1 + QuadModelv_P.Bias_Bias) *
                     QuadModelv_P.Gain_Gain + QuadModelv_P.Bias1_Bias) *
      rtb_sincos_o1_idx_1;
    rtb_sincos_o1_idx_1 = QuadModelv_P.Constant_Value_gw;
  } else {
    rtb_sincos_o1_idx_1 = QuadModelv_P.Constant1_Value_nq;
  }

  // End of Switch: '<S72>/Switch'

  // Sum: '<S68>/Sum' incorporates:
  //   Integrator: '<S14>/xe,ye,ze'
  //   Product: '<S70>/rad long '
  //   Product: '<S70>/x*sin'
  //   Product: '<S70>/y*cos'
  //   Sum: '<S63>/Sum'
  //   Sum: '<S70>/Sum1'
  //   Switch: '<S68>/Switch1'
  //   UnitConversion: '<S71>/Unit Conversion'

  Z = ((QuadModelv_X.xeyeze_CSTATE[0] * QuadModelv_B.SinCos_o1 +
        QuadModelv_X.xeyeze_CSTATE[1] * QuadModelv_B.SinCos_o2) *
       QuadModelv_B.TrigonometricFunction2 * 57.295779513082323 +
       QuadModelv_B.Switch_a) + rtb_sincos_o1_idx_1;

  // Switch: '<S73>/Switch' incorporates:
  //   Abs: '<S73>/Abs'
  //   Bias: '<S73>/Bias'
  //   Bias: '<S73>/Bias1'
  //   Constant: '<S73>/Constant2'
  //   Constant: '<S77>/Constant'
  //   Math: '<S73>/Math Function1'
  //   RelationalOperator: '<S77>/Compare'

  if (std::abs(Z) > QuadModelv_P.CompareToConstant_const_f) {
    Z = rt_modd_snf(Z + QuadModelv_P.Bias_Bias_l, QuadModelv_P.Constant2_Value_o)
      + QuadModelv_P.Bias1_Bias_c;
  }

  // End of Switch: '<S73>/Switch'

  // Sum: '<S63>/Sum1' incorporates:
  //   Constant: '<S5>/ModelParam.envAltitude'
  //   Integrator: '<S14>/xe,ye,ze'
  //   UnaryMinus: '<S63>/Ze2height'

  rtb_Saturation_1 = -QuadModelv_X.xeyeze_CSTATE[2] -
    QuadModelv_P.ModelParam_envAltitude;

  // Saturate: '<S5>/Saturation_1'
  if (rtb_Saturation_1 > QuadModelv_P.Saturation_1_UpperSat) {
    rtb_Saturation_1 = QuadModelv_P.Saturation_1_UpperSat;
  } else if (rtb_Saturation_1 < QuadModelv_P.Saturation_1_LowerSat) {
    rtb_Saturation_1 = QuadModelv_P.Saturation_1_LowerSat;
  }

  // End of Saturate: '<S5>/Saturation_1'

  // UnitConversion: '<S91>/Unit Conversion' incorporates:
  //   SignalConversion generated from: '<Root>/MavVehile3DInfo'

  // Unit Conversion - from: deg to: rad
  // Expression: output = (0.0174533*input) + (0)
  rtb_Product3_o = 0.017453292519943295 * LoadLeakRatio;

  // UnitConversion: '<S93>/Unit Conversion' incorporates:
  //   SignalConversion generated from: '<Root>/MavVehile3DInfo'

  // Unit Conversion - from: deg to: rad
  // Expression: output = (0.0174533*input) + (0)
  rtb_Product2_kl = 0.017453292519943295 * Z;

  // Selector: '<S66>/Selector' incorporates:
  //   SignalConversion generated from: '<Root>/MavVehile3DInfo'

  rtb_TmpSignalConversionAtSFun_1 = rtb_Saturation_1;

  // S-Function (saerogravity2): '<S66>/WGS84 Gravity S-Function' incorporates:
  //   Constant: '<S66>/Julian Date'


  // S-Function Block: <S66>/WGS84 Gravity S-Function
  {
    int_T i;
    real_T GM, opt_m2ft, deg2rad;
    real_T *phi_ptr, *height_ptr;
    boolean_T phi_wrapped = false;
    real_T *phi = (real_T *) &QuadModelv_DW.WGS84GravitySFunction_phi;
    real_T *h = (real_T *) &QuadModelv_DW.WGS84GravitySFunction_h;
    height_ptr = (real_T *) &rtb_TmpSignalConversionAtSFun_1;
    phi_ptr = (real_T *) &rtb_Product3_o;

    // get unit conversion factor
    opt_m2ft = 1.0;
    deg2rad = 1.0;

    // Use Earth's Atmosphere in Gravitational Const?
    GM = ( 1.0 == 0 ) ? WGS84_GM_DEF : WGS84_GM_PRM;

    // WGS84TAYLORSERIES
    {
      for (i = 0; i < 1; i++ ) {
        real_T fphi, pi_2;

        // create short variables for latitude (phi) and height (h)
        phi[i] = phi_ptr[i] * deg2rad;
        h[i] = height_ptr[i] / opt_m2ft;
        if (phi[i] > AERO_PI || phi[i] < -AERO_PI) {
          phi[i] = std::fmod(phi[i]+AERO_PI, 2.0*AERO_PI) - AERO_PI;
        }

        fphi = std::abs(phi[i]);
        pi_2 = AERO_PI/2.0;
        if (fphi > pi_2 ) {
          real_T sign = phi[i]/fphi;
          phi_wrapped = true;
          phi[i] = sign*(pi_2 - (fphi - pi_2));
        } else {
          phi_wrapped = false;
        }
      }

      wgs84_taylor_series(h, phi, opt_m2ft, &QuadModelv_B.MatrixConcatenate[2],
                          1);

      // north and east components are zero
      QuadModelv_B.MatrixConcatenate[0] =
        QuadModelv_DW.WGS84GravitySFunction_gamma_phi;
      QuadModelv_B.MatrixConcatenate[1] =
        QuadModelv_DW.WGS84GravitySFunction_gamma_phi;
    }
  }

  // Integrator: '<S14>/ub,vb,wb' incorporates:
  //   Math: '<S14>/Transpose'

  rtb_Product2_kl = QuadModelv_X.ubvbwb_CSTATE[1];
  rtb_Product3_o = QuadModelv_X.ubvbwb_CSTATE[0];
  VectorConcatenate_n_idx_0 = QuadModelv_X.ubvbwb_CSTATE[2];
  for (i = 0; i < 3; i++) {
    // Product: '<S7>/Product1'
    Product1[i] = QuadModelv_B.m * QuadModelv_B.MatrixConcatenate[i];

    // Product: '<S22>/Product' incorporates:
    //   Concatenate: '<S36>/Vector Concatenate'
    //   Integrator: '<S14>/ub,vb,wb'
    //   Math: '<S14>/Transpose'

    QuadModelv_B.Product[i] = (VectorConcatenate[3 * i + 1] * rtb_Product2_kl +
      VectorConcatenate[3 * i] * rtb_Product3_o) + VectorConcatenate[3 * i + 2] *
      VectorConcatenate_n_idx_0;
  }

  // Product: '<S47>/Product' incorporates:
  //   Integrator: '<S15>/q0 q1 q2 q3'

  rtb_TmpSignalConversionAtSFun_1 = QuadModelv_X.q0q1q2q3_CSTATE[0] *
    QuadModelv_X.q0q1q2q3_CSTATE[0];

  // Sqrt: '<S46>/sqrt' incorporates:
  //   Integrator: '<S15>/q0 q1 q2 q3'
  //   Product: '<S47>/Product1'
  //   Product: '<S47>/Product2'
  //   Product: '<S47>/Product3'
  //   Sum: '<S47>/Sum'

  rtb_TmpSignalConversionAtSFun_1 = std::sqrt(((QuadModelv_X.q0q1q2q3_CSTATE[1] *
    QuadModelv_X.q0q1q2q3_CSTATE[1] + rtb_TmpSignalConversionAtSFun_1) +
    QuadModelv_X.q0q1q2q3_CSTATE[2] * QuadModelv_X.q0q1q2q3_CSTATE[2]) +
    QuadModelv_X.q0q1q2q3_CSTATE[3] * QuadModelv_X.q0q1q2q3_CSTATE[3]);

  // Product: '<S41>/Product' incorporates:
  //   Integrator: '<S15>/q0 q1 q2 q3'

  rtb_q3dot = QuadModelv_X.q0q1q2q3_CSTATE[0] / rtb_TmpSignalConversionAtSFun_1;

  // Product: '<S41>/Product1' incorporates:
  //   Integrator: '<S15>/q0 q1 q2 q3'

  rtb_Product3_o = QuadModelv_X.q0q1q2q3_CSTATE[1] /
    rtb_TmpSignalConversionAtSFun_1;

  // Product: '<S41>/Product2' incorporates:
  //   Integrator: '<S15>/q0 q1 q2 q3'

  rtb_Product2_kl = QuadModelv_X.q0q1q2q3_CSTATE[2] /
    rtb_TmpSignalConversionAtSFun_1;

  // Product: '<S41>/Product3' incorporates:
  //   Integrator: '<S15>/q0 q1 q2 q3'

  rtb_TmpSignalConversionAtSFun_1 = QuadModelv_X.q0q1q2q3_CSTATE[3] /
    rtb_TmpSignalConversionAtSFun_1;

  // Fcn: '<S24>/fcn2' incorporates:
  //   Fcn: '<S24>/fcn5'

  VectorConcatenate_n_idx_2 = rtb_q3dot * rtb_q3dot;
  rtb_Add = rtb_Product3_o * rtb_Product3_o;
  rtb_TmpSignalConversionAtSFun_0 = rtb_Product2_kl * rtb_Product2_kl;
  VectorConcatenate_tmp = rtb_TmpSignalConversionAtSFun_1 *
    rtb_TmpSignalConversionAtSFun_1;

  // Trigonometry: '<S40>/Trigonometric Function1' incorporates:
  //   Concatenate: '<S40>/Vector Concatenate'
  //   Fcn: '<S24>/fcn1'
  //   Fcn: '<S24>/fcn2'

  VectorConcatenate_n_idx_0 = rt_atan2d_snf((rtb_Product3_o * rtb_Product2_kl +
    rtb_q3dot * rtb_TmpSignalConversionAtSFun_1) * 2.0,
    ((VectorConcatenate_n_idx_2 + rtb_Add) - rtb_TmpSignalConversionAtSFun_0) -
    VectorConcatenate_tmp);

  // Fcn: '<S24>/fcn3'
  rtb_Rn = (rtb_Product3_o * rtb_TmpSignalConversionAtSFun_1 - rtb_q3dot *
            rtb_Product2_kl) * -2.0;

  // If: '<S42>/If'
  if (rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    if (rtb_Rn > 1.0) {
      rtAction = 0;
    } else if (rtb_Rn < -1.0) {
      rtAction = 1;
    } else {
      rtAction = 2;
    }

    QuadModelv_DW.If_ActiveSubsystem = rtAction;
  } else {
    rtAction = QuadModelv_DW.If_ActiveSubsystem;
  }

  switch (rtAction) {
   case 0:
    // Outputs for IfAction SubSystem: '<S42>/If Action Subsystem' incorporates:
    //   ActionPort: '<S43>/Action Port'

    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
      // Merge: '<S42>/Merge' incorporates:
      //   Constant: '<S43>/Constant'

      QuadModelv_B.Merge_e2 = QuadModelv_P.Constant_Value;
    }

    // End of Outputs for SubSystem: '<S42>/If Action Subsystem'
    break;

   case 1:
    // Outputs for IfAction SubSystem: '<S42>/If Action Subsystem1' incorporates:
    //   ActionPort: '<S44>/Action Port'

    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
      // Merge: '<S42>/Merge' incorporates:
      //   Constant: '<S44>/Constant'

      QuadModelv_B.Merge_e2 = QuadModelv_P.Constant_Value_l;
    }

    // End of Outputs for SubSystem: '<S42>/If Action Subsystem1'
    break;

   default:
    // Outputs for IfAction SubSystem: '<S42>/If Action Subsystem2' incorporates:
    //   ActionPort: '<S45>/Action Port'

    // Merge: '<S42>/Merge' incorporates:
    //   SignalConversion generated from: '<S45>/In'

    QuadModelv_B.Merge_e2 = rtb_Rn;

    // End of Outputs for SubSystem: '<S42>/If Action Subsystem2'
    break;
  }

  // End of If: '<S42>/If'

  // Trigonometry: '<S40>/trigFcn' incorporates:
  //   Concatenate: '<S40>/Vector Concatenate'

  if (QuadModelv_B.Merge_e2 > 1.0) {
    rtb_sincos_o1_idx_1 = 1.0;
  } else if (QuadModelv_B.Merge_e2 < -1.0) {
    rtb_sincos_o1_idx_1 = -1.0;
  } else {
    rtb_sincos_o1_idx_1 = QuadModelv_B.Merge_e2;
  }

  VectorConcatenate_n_idx_1 = std::asin(rtb_sincos_o1_idx_1);

  // End of Trigonometry: '<S40>/trigFcn'

  // Trigonometry: '<S40>/Trigonometric Function3' incorporates:
  //   Concatenate: '<S40>/Vector Concatenate'
  //   Fcn: '<S24>/fcn4'
  //   Fcn: '<S24>/fcn5'

  VectorConcatenate_n_idx_2 = rt_atan2d_snf((rtb_Product2_kl *
    rtb_TmpSignalConversionAtSFun_1 + rtb_q3dot * rtb_Product3_o) * 2.0,
    ((VectorConcatenate_n_idx_2 - rtb_Add) - rtb_TmpSignalConversionAtSFun_0) +
    VectorConcatenate_tmp);

  // Trigonometry: '<S195>/sincos' incorporates:
  //   DataTypeConversion: '<S8>/Data Type Conversion2'
  //   MATLAB Function: '<S7>/OnGroundFaceup'
  //   SignalConversion generated from: '<S197>/ SFunction '

  // MATLAB Function 'Force and Moment Model/OnGroundFaceup': '<S197>:1'
  // '<S197>:1:3' targetRoll = 0;
  //  if cos(EulerAng(1))<0
  //      targetRoll=pi;
  //  end
  // '<S197>:1:8' targetPitch = 0;
  //  if cos(EulerAng(2))<0
  //          targetPitch=pi;
  //  end
  // '<S197>:1:13' y = [targetRoll;targetPitch;EulerAng(3)];
  rtb_Sum1_ki[2] = std::sin(VectorConcatenate_n_idx_0);
  rtb_DataTypeConversion2[2] = std::cos(VectorConcatenate_n_idx_0);

  // Product: '<S277>/u(5)*u(6)' incorporates:
  //   Concatenate: '<S286>/Vector Concatenate'

  VectorConcatenate_p[0] = rtb_DataTypeConversion2[2];

  // Sum: '<S280>/Sum' incorporates:
  //   Concatenate: '<S286>/Vector Concatenate'
  //   Product: '<S280>/u(3)*u(4)'
  //   Product: '<S280>/u(6)*u(1)*u(2)'

  VectorConcatenate_p[1] = 0.0 * rtb_DataTypeConversion2[2] * 0.0 - rtb_Sum1_ki
    [2];

  // Sum: '<S283>/Sum' incorporates:
  //   Concatenate: '<S286>/Vector Concatenate'
  //   Product: '<S283>/u(1)*u(3)'
  //   Product: '<S283>/u(2)*u(4)*u(6)'

  VectorConcatenate_p[2] = 0.0 * rtb_Sum1_ki[2] + 0.0 * rtb_DataTypeConversion2
    [2];

  // Product: '<S278>/u(3)*u(5)' incorporates:
  //   Concatenate: '<S286>/Vector Concatenate'

  VectorConcatenate_p[3] = rtb_Sum1_ki[2];

  // Sum: '<S281>/Sum' incorporates:
  //   Concatenate: '<S286>/Vector Concatenate'
  //   Product: '<S281>/u(1)*u(2)*u(3)'
  //   Product: '<S281>/u(4)*u(6)'

  VectorConcatenate_p[4] = 0.0 * rtb_Sum1_ki[2] + rtb_DataTypeConversion2[2];

  // Sum: '<S284>/Sum' incorporates:
  //   Concatenate: '<S286>/Vector Concatenate'
  //   Product: '<S284>/u(1)*u(6)'
  //   Product: '<S284>/u(2)*u(3)*u(4)'

  VectorConcatenate_p[5] = 0.0 * rtb_Sum1_ki[2] - 0.0 * rtb_DataTypeConversion2
    [2];

  // UnaryMinus: '<S279>/Unary Minus' incorporates:
  //   Concatenate: '<S286>/Vector Concatenate'

  VectorConcatenate_p[6] = -0.0;

  // Product: '<S282>/u(1)*u(5)' incorporates:
  //   Concatenate: '<S286>/Vector Concatenate'

  VectorConcatenate_p[7] = 0.0;

  // Product: '<S285>/u(4)*u(5)' incorporates:
  //   Concatenate: '<S286>/Vector Concatenate'

  VectorConcatenate_p[8] = 1.0;

  // Sum: '<S242>/Add' incorporates:
  //   Product: '<S277>/u(5)*u(6)'

  rtb_Add = rtb_DataTypeConversion2[2] + VectorConcatenate_p[4];

  // If: '<S194>/If' incorporates:
  //   Merge: '<S194>/Merge'
  //   Sum: '<S242>/Add'

  if (rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    rtAction = static_cast<int8_T>(!(rtb_Add + 1.0 > 0.0));
    QuadModelv_DW.If_ActiveSubsystem_h = rtAction;
  } else {
    rtAction = QuadModelv_DW.If_ActiveSubsystem_h;
  }

  if (rtAction == 0) {
    // Outputs for IfAction SubSystem: '<S194>/Positive Trace' incorporates:
    //   ActionPort: '<S240>/Action Port'

    QuadModelv_PositiveTrace(rtb_Add + 1.0, VectorConcatenate_p,
      &QuadModelv_B.Merge[0], &QuadModelv_B.Merge[1],
      &QuadModelv_P.PositiveTrace_d);

    // End of Outputs for SubSystem: '<S194>/Positive Trace'
  } else {
    // Outputs for IfAction SubSystem: '<S194>/Negative Trace' incorporates:
    //   ActionPort: '<S239>/Action Port'

    QuadModelv_NegativeTrace(VectorConcatenate_p, QuadModelv_B.Merge,
      &QuadModelv_DW.NegativeTrace_d, &QuadModelv_P.NegativeTrace_d);

    // End of Outputs for SubSystem: '<S194>/Negative Trace'
  }

  // End of If: '<S194>/If'

  // Product: '<S288>/Product3'
  rtb_TmpSignalConversionAtSFun_1 = QuadModelv_B.Merge[3] * QuadModelv_B.Merge[3];

  // Sum: '<S288>/Sum' incorporates:
  //   Product: '<S288>/Product'
  //   Product: '<S288>/Product1'
  //   Product: '<S288>/Product2'

  rtb_q3dot = ((QuadModelv_B.Merge[0] * QuadModelv_B.Merge[0] +
                QuadModelv_B.Merge[1] * QuadModelv_B.Merge[1]) +
               QuadModelv_B.Merge[2] * QuadModelv_B.Merge[2]) +
    rtb_TmpSignalConversionAtSFun_1;

  // Product: '<S199>/Divide'
  rtb_q2dot = QuadModelv_B.Merge[0] / rtb_q3dot;

  // Sum: '<S204>/Add' incorporates:
  //   Sum: '<S330>/Add'

  rtb_TmpSignalConversionAtSFun_0 = (VectorConcatenate[0] + VectorConcatenate[4])
    + VectorConcatenate[8];

  // If: '<S193>/If' incorporates:
  //   Merge: '<S193>/Merge'
  //   Sum: '<S204>/Add'

  if (rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    rtAction = static_cast<int8_T>(!(rtb_TmpSignalConversionAtSFun_0 > 0.0));
    QuadModelv_DW.If_ActiveSubsystem_l = rtAction;
  } else {
    rtAction = QuadModelv_DW.If_ActiveSubsystem_l;
  }

  if (rtAction == 0) {
    // Outputs for IfAction SubSystem: '<S193>/Positive Trace' incorporates:
    //   ActionPort: '<S202>/Action Port'

    QuadModelv_PositiveTrace(rtb_TmpSignalConversionAtSFun_0, VectorConcatenate,
      &QuadModelv_B.Merge_c[0], &QuadModelv_B.Merge_c[1],
      &QuadModelv_P.PositiveTrace);

    // End of Outputs for SubSystem: '<S193>/Positive Trace'
  } else {
    // Outputs for IfAction SubSystem: '<S193>/Negative Trace' incorporates:
    //   ActionPort: '<S201>/Action Port'

    QuadModelv_NegativeTrace(VectorConcatenate, QuadModelv_B.Merge_c,
      &QuadModelv_DW.NegativeTrace, &QuadModelv_P.NegativeTrace);

    // End of Outputs for SubSystem: '<S193>/Negative Trace'
  }

  // End of If: '<S193>/If'

  // UnaryMinus: '<S287>/Unary Minus'
  rtb_TmpSignalConversionAtSFun_1 = -QuadModelv_B.Merge[1];

  // Product: '<S199>/Divide1'
  rtb_TmpSignalConversionAtSFun_1 /= rtb_q3dot;

  // Product: '<S199>/Divide2' incorporates:
  //   UnaryMinus: '<S287>/Unary Minus1'

  rtb_Product2_kl = -QuadModelv_B.Merge[2] / rtb_q3dot;

  // Product: '<S199>/Divide3' incorporates:
  //   UnaryMinus: '<S287>/Unary Minus2'

  rtb_Rn = -QuadModelv_B.Merge[3] / rtb_q3dot;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    for (i = 0; i < 20; i++) {
      // DataTypeConversion: '<Root>/Data Type Conversion2' incorporates:
      //   Inport: '<Root>/inFloatsCollision'

      QuadModelv_B.DataTypeConversion2[i] = QuadModelv_U.inFloatsCollision[i];
    }

    // MATLAB Function: '<Root>/CollisionDetection' incorporates:
    //   Concatenate: '<S36>/Vector Concatenate'

    //  vect=[1;1;1;1;1;1;1;1];
    // 是否碰撞标志位
    // MATLAB Function 'CollisionDetection': '<S3>:1'
    // '<S3>:1:6' if isempty(isCol)
    // 输出力和力矩暂存向量
    // '<S3>:1:12' if isempty(fOut)
    // 碰撞冲量矩阵
    // '<S3>:1:18' if isempty(mv0)
    // 碰撞时间
    // '<S3>:1:24' if isempty(tColi)
    // 如果没有发送碰撞
    // '<S3>:1:29' if isCol<0.5
    if ((QuadModelv_DW.isCol < 0.5) && (std::abs
         (QuadModelv_B.DataTypeConversion2[0] - 12345.0) < 1.0) &&
        (QuadModelv_B.DataTypeConversion2[1] > 0.5)) {
      // 如果收到碰撞消息，且被碰撞物体为编号飞机（带CopterID的值大于0）
      // '<S3>:1:31' if abs(uFloats(1)-12345)<1 && uFloats(2)>0.5
      // 如果收到碰撞消息，12345是数据校验位，uFloats(2)是碰撞到飞机的ID
      // '<S3>:1:32' isCol=int8(1);
      QuadModelv_DW.isCol = 1;

      // 设置为碰撞模式
      // 被碰撞物体是活动的还是固定的
      // '<S3>:1:34' if uFloats(3)>0
      if (QuadModelv_B.DataTypeConversion2[2] > 0.0) {
        // 被碰撞物与本物体的质量比有正值，说明碰撞物也是飞机，需要用冲量定理
        // '<S3>:1:35' massOb=mass*uFloats(3)^2;
        rtb_Product3_o = QuadModelv_B.DataTypeConversion2[2] *
          QuadModelv_B.DataTypeConversion2[2] * QuadModelv_B.m;

        // '<S3>:1:36' veOb = uFloats(7:9);
        // '<S3>:1:37' veNew=(mass*ve+massOb*veOb)/(mass+massOb);
        rtb_Add = QuadModelv_B.m + rtb_Product3_o;
        QuadModelv_DW.mv0[0] = (QuadModelv_B.m * QuadModelv_B.Product[0] +
          rtb_Product3_o * QuadModelv_B.DataTypeConversion2[6]) / rtb_Add;
        QuadModelv_DW.mv0[1] = (QuadModelv_B.m * QuadModelv_B.Product[1] +
          rtb_Product3_o * QuadModelv_B.DataTypeConversion2[7]) / rtb_Add;
        QuadModelv_DW.mv0[2] = (QuadModelv_B.m * QuadModelv_B.Product[2] +
          rtb_Product3_o * QuadModelv_B.DataTypeConversion2[8]) / rtb_Add;
      } else {
        // '<S3>:1:38' else
        // 被碰撞物为树、房子等固定物体，质量无穷大，直接反弹回去（1/10的速度）
        // '<S3>:1:39' veNew=-ve/10.0;
        QuadModelv_DW.mv0[0] = -QuadModelv_B.Product[0] / 10.0;
        QuadModelv_DW.mv0[1] = -QuadModelv_B.Product[1] / 10.0;
        QuadModelv_DW.mv0[2] = -QuadModelv_B.Product[2] / 10.0;
      }

      // '<S3>:1:42' mv0=mass*(veNew-ve);
      QuadModelv_DW.mv0[0] = (QuadModelv_DW.mv0[0] - QuadModelv_B.Product[0]) *
        QuadModelv_B.m;
      QuadModelv_DW.mv0[1] = (QuadModelv_DW.mv0[1] - QuadModelv_B.Product[1]) *
        QuadModelv_B.m;
      QuadModelv_DW.mv0[2] = (QuadModelv_DW.mv0[2] - QuadModelv_B.Product[2]) *
        QuadModelv_B.m;

      // '<S3>:1:43' tColi=t;
      QuadModelv_DW.tColi = rtb_Clock1_tmp;

      // 记录碰撞时间
    }

    // 如果已经发生碰撞
    // '<S3>:1:48' if isCol>=0.5
    if (QuadModelv_DW.isCol >= 0.5) {
      // '<S3>:1:49' if (t-tColi)>0.05
      if (rtb_Clock1_tmp - QuadModelv_DW.tColi > 0.05) {
        // '<S3>:1:50' fOut=[0;0;0;0;0;0];
        for (i = 0; i < 6; i++) {
          QuadModelv_DW.fOut[i] = 0.0;
        }
      } else {
        // '<S3>:1:51' else
        // 用瞬时的作用力模拟冲量作用（0.05s）
        // '<S3>:1:53' mv=DCM*mv0;
        rtb_Product3_o = QuadModelv_DW.mv0[1];
        rtb_Add = QuadModelv_DW.mv0[0];
        VectorConcatenate_tmp = QuadModelv_DW.mv0[2];
        for (i = 0; i < 3; i++) {
          rtb_Sum1_ki[i] = (VectorConcatenate[i + 3] * rtb_Product3_o +
                            VectorConcatenate[i] * rtb_Add) +
            VectorConcatenate[i + 6] * VectorConcatenate_tmp;
        }

        // '<S3>:1:54' mv(1)=mv(1)*(0.7+0.3*rand());
        rtb_Sum1_ki[0] *= 0.3 * QuadModelv_rand() + 0.7;

        // '<S3>:1:55' mv(2)=mv(2)*(0.7+0.3*rand());
        rtb_Sum1_ki[1] *= 0.3 * QuadModelv_rand() + 0.7;

        // '<S3>:1:56' mv(3)=mv(3)*(0.7+0.3*rand());
        rtb_Sum1_ki[2] *= 0.3 * QuadModelv_rand() + 0.7;

        // '<S3>:1:57' fOut(1:3) = mv/0.05;
        QuadModelv_DW.fOut[0] = rtb_Sum1_ki[0] / 0.05;
        QuadModelv_DW.fOut[1] = rtb_Sum1_ki[1] / 0.05;
        QuadModelv_DW.fOut[2] = rtb_Sum1_ki[2] / 0.05;

        // 冲量除以时间得到力
        // '<S3>:1:58' fOut(4:6) = rand(3,1)*mass;
        QuadModelv_rand_a(rtb_Sum1_ki);
        QuadModelv_DW.fOut[3] = rtb_Sum1_ki[0] * QuadModelv_B.m;
        QuadModelv_DW.fOut[4] = rtb_Sum1_ki[1] * QuadModelv_B.m;
        QuadModelv_DW.fOut[5] = rtb_Sum1_ki[2] * QuadModelv_B.m;
      }
    }

    // 下面处理飞机与场景固定物体之间的碰撞
    // '<S3>:1:63' fm = fOut;
    // tz分别表示距地面高度和飞机XYZ坐标
    // '<S3>:1:65' tz=[100;0;0;0];
    tz_idx_0 = 100.0;
    tz_idx_3 = 0.0;

    // '<S3>:1:66' ddm=[0;0;0;0;0;0];
    for (i = 0; i < 6; i++) {
      ddm[i] = 0.0;
    }

    // '<S3>:1:67' if abs(uFloats(1)-12345)<1
    if (std::abs(QuadModelv_B.DataTypeConversion2[0] - 12345.0) < 1.0) {
      // 如果收到碰撞消息
      // '<S3>:1:68' kRa=40;
      // '<S3>:1:69' kRaVe=5;
      // '<S3>:1:70' if uFloats(2)<0.5
      if (QuadModelv_B.DataTypeConversion2[1] < 0.5) {
        // 如果碰撞的是场景固定物体
        // 向下照射射线返回长度
        // '<S3>:1:72' tz(1)=uFloats(15);
        tz_idx_0 = QuadModelv_B.DataTypeConversion2[14];

        // 飞机在UE4中的坐标
        // '<S3>:1:74' tz(2:4)=uFloats(4:6);
        tz_idx_3 = QuadModelv_B.DataTypeConversion2[5];
      } else {
        // '<S3>:1:75' else
        // '<S3>:1:76' kRa=20;
      }

      // 前后左右是以地面坐标系的北东地为准
      // 前障碍物碰撞反馈力
      // '<S3>:1:80' if uFloats(10)<0
      if (QuadModelv_B.DataTypeConversion2[9] < 0.0) {
        // 距前墙面距离，小于0说明已经接触
        // '<S3>:1:81' z=uFloats(10);
        // '<S3>:1:82' if z>-0.01
        if (QuadModelv_B.DataTypeConversion2[9] > -0.01) {
          // '<S3>:1:83' ddm(1)=-10*ve(1);
          ddm[0] = -10.0 * QuadModelv_B.Product[0];
        } else if (QuadModelv_B.DataTypeConversion2[9] > -0.04) {
          // '<S3>:1:84' elseif z>-0.04
          // '<S3>:1:85' ddm(1)=200*z-20*ve(1);
          ddm[0] = 200.0 * QuadModelv_B.DataTypeConversion2[9] - 20.0 *
            QuadModelv_B.Product[0];
        } else {
          // '<S3>:1:86' else
          // '<S3>:1:87' ddm(1)=500*z-50*ve(1);
          ddm[0] = 500.0 * QuadModelv_B.DataTypeConversion2[9] - 50.0 *
            QuadModelv_B.Product[0];
        }
      }

      // 后障碍物碰撞反馈力
      // '<S3>:1:91' if uFloats(11)<0
      if (QuadModelv_B.DataTypeConversion2[10] < 0.0) {
        //          ddm(1)=-uFloats(11)*mass*kRa;
        //          if ve(1)<0
        //              ddm(1)=ddm(1)-ve(1)*mass*kRaVe;
        //          end
        // '<S3>:1:96' z=uFloats(11);
        // '<S3>:1:97' if z>-0.01
        if (QuadModelv_B.DataTypeConversion2[10] > -0.01) {
          // '<S3>:1:98' ddm(1)=-10*ve(1);
          ddm[0] = -10.0 * QuadModelv_B.Product[0];
        } else if (QuadModelv_B.DataTypeConversion2[10] > -0.04) {
          // '<S3>:1:99' elseif z>-0.04
          // '<S3>:1:100' ddm(1)=-200*z-20*ve(1);
          ddm[0] = -200.0 * QuadModelv_B.DataTypeConversion2[10] - 20.0 *
            QuadModelv_B.Product[0];
        } else {
          // '<S3>:1:101' else
          // '<S3>:1:102' ddm(1)=-500*z-50*ve(1);
          ddm[0] = -500.0 * QuadModelv_B.DataTypeConversion2[10] - 50.0 *
            QuadModelv_B.Product[0];
        }
      }

      // 左障碍物碰撞反馈力
      // '<S3>:1:107' if uFloats(12)<0
      if (QuadModelv_B.DataTypeConversion2[11] < 0.0) {
        //          ddm(2)=-uFloats(12)*mass*kRa;
        //          if ve(2)<0
        //              ddm(2)=ddm(2)-ve(2)*mass*kRaVe;
        //          end
        // '<S3>:1:112' z=uFloats(12);
        // '<S3>:1:113' if z>-0.01
        if (QuadModelv_B.DataTypeConversion2[11] > -0.01) {
          // '<S3>:1:114' ddm(2)=-10*ve(2);
          ddm[1] = -10.0 * QuadModelv_B.Product[1];
        } else if (QuadModelv_B.DataTypeConversion2[11] > -0.04) {
          // '<S3>:1:115' elseif z>-0.04
          // '<S3>:1:116' ddm(2)=-200*z-20*ve(2);
          ddm[1] = -200.0 * QuadModelv_B.DataTypeConversion2[11] - 20.0 *
            QuadModelv_B.Product[1];
        } else {
          // '<S3>:1:117' else
          // '<S3>:1:118' ddm(2)=-500*z-50*ve(2);
          ddm[1] = -500.0 * QuadModelv_B.DataTypeConversion2[11] - 50.0 *
            QuadModelv_B.Product[1];
        }
      }

      // 右障碍物碰撞反馈力
      // '<S3>:1:122' if uFloats(13)<0
      if (QuadModelv_B.DataTypeConversion2[12] < 0.0) {
        //          ddm(2)=uFloats(13)*mass*kRa;
        //          if ve(2)>0
        //              ddm(2)=ddm(2)-ve(2)*mass*kRaVe;
        //          end
        // '<S3>:1:127' z=uFloats(13);
        // '<S3>:1:128' if z>-0.01
        if (QuadModelv_B.DataTypeConversion2[12] > -0.01) {
          // '<S3>:1:129' ddm(2)=-10*ve(2);
          ddm[1] = -10.0 * QuadModelv_B.Product[1];
        } else if (QuadModelv_B.DataTypeConversion2[12] > -0.04) {
          // '<S3>:1:130' elseif z>-0.04
          // '<S3>:1:131' ddm(2)=200*z-20*ve(2);
          ddm[1] = 200.0 * QuadModelv_B.DataTypeConversion2[12] - 20.0 *
            QuadModelv_B.Product[1];
        } else {
          // '<S3>:1:132' else
          // '<S3>:1:133' ddm(2)=500*z-50*ve(2);
          ddm[1] = 500.0 * QuadModelv_B.DataTypeConversion2[12] - 50.0 *
            QuadModelv_B.Product[1];
        }
      }

      // 上障碍物碰撞反馈力
      // '<S3>:1:138' if uFloats(14)<0
      if (QuadModelv_B.DataTypeConversion2[13] < 0.0) {
        //          ddm(3)=-uFloats(14)*mass*kRa;
        //          if ve(3)<0
        //              ddm(3)=ddm(3)-ve(3)*mass*kRaVe;
        //          end
        // '<S3>:1:144' z=uFloats(14);
        // '<S3>:1:145' if z>-0.01
        if (QuadModelv_B.DataTypeConversion2[13] > -0.01) {
          // '<S3>:1:146' ddm(3)=-10*ve(3);
          ddm[2] = -10.0 * QuadModelv_B.Product[2];
        } else if (QuadModelv_B.DataTypeConversion2[13] > -0.04) {
          // '<S3>:1:147' elseif z>-0.04
          // '<S3>:1:148' ddm(3)=-200*z-20*ve(3);
          ddm[2] = -200.0 * QuadModelv_B.DataTypeConversion2[13] - 20.0 *
            QuadModelv_B.Product[2];
        } else {
          // '<S3>:1:149' else
          // '<S3>:1:150' ddm(3)=-500*z-50*ve(3);
          ddm[2] = -500.0 * QuadModelv_B.DataTypeConversion2[13] - 50.0 *
            QuadModelv_B.Product[2];
        }
      }
    }

    // 力从地面坐标系映射机体坐标系
    // '<S3>:1:155' ddmm=DCM*ddm(1:3);
    // '<S3>:1:156' fm=fm+[ddmm;0;0;0];
    rtb_Product3_o = ddm[1];
    rtb_Add = ddm[0];
    VectorConcatenate_tmp = ddm[2];
    for (i = 0; i < 3; i++) {
      QuadModelv_B.fm[i] = ((VectorConcatenate[i + 3] * rtb_Product3_o +
        VectorConcatenate[i] * rtb_Add) + VectorConcatenate[i + 6] *
                            VectorConcatenate_tmp) + QuadModelv_DW.fOut[i];
    }

    QuadModelv_B.fm[3] = QuadModelv_DW.fOut[3];
    QuadModelv_B.fm[4] = QuadModelv_DW.fOut[4];
    QuadModelv_B.fm[5] = QuadModelv_DW.fOut[5];

    // '<S3>:1:158' isCrash=isCol;
    QuadModelv_B.isCrash = QuadModelv_DW.isCol;

    // End of MATLAB Function: '<Root>/CollisionDetection'

    // MATLAB Function: '<Root>/MATLAB Function' incorporates:
    //   Inport: '<Root>/TerrainZ'

    QuadModelv_B.y_m = QuadModelv_U.TerrainZ;

    // MATLAB Function 'MATLAB Function': '<S9>:1'
    // '<S9>:1:2' y=TerrainZ;
    // '<S9>:1:3' if tz(1)<=99
    if (tz_idx_0 <= 99.0) {
      // '<S9>:1:4' y=tz(4)+tz(1);
      QuadModelv_B.y_m = tz_idx_0 + tz_idx_3;
    }

    // End of MATLAB Function: '<Root>/MATLAB Function'

    // MATLAB Function: '<S7>/Ground Model' incorporates:
    //   Integrator: '<S14>/p,q,r '
    //   Integrator: '<S14>/xe,ye,ze'
    //   Product: '<S290>/Product'
    //   Product: '<S290>/Product1'
    //   Product: '<S290>/Product2'
    //   Product: '<S290>/Product3'
    //   Product: '<S291>/Product'
    //   Product: '<S291>/Product1'
    //   Product: '<S291>/Product2'
    //   Product: '<S291>/Product3'
    //   Product: '<S292>/Product'
    //   Product: '<S292>/Product1'
    //   Product: '<S292>/Product2'
    //   Product: '<S292>/Product3'
    //   Sum: '<S290>/Sum'
    //   Sum: '<S291>/Sum'
    //   Sum: '<S292>/Sum'

    //  persistent last;
    //  if isempty(last)
    //      last=TerrainZ;
    //  end
    //  y=TerrainZ;
    //  if tz(1)<10
    //      if tz(1)<0
    //          y=last;
    //      else
    //          y=tz(3)+tz(1);
    //      end
    //  end
    //  This is a ground model that can provides reaction force and reaction torque 
    // MATLAB Function 'Force and Moment Model/Ground Model': '<S196>:1'
    //  persistent z0;
    //  if isempty(z0)
    //       z0=0;
    //  end
    // '<S196>:1:10' if isempty(takeoffFlag)
    // '<S196>:1:15' if isempty(landFlag)
    // '<S196>:1:19' z=Xe(3)-terrainZ;
    rtb_Product3_o = QuadModelv_X.xeyeze_CSTATE[2] - QuadModelv_B.y_m;

    // '<S196>:1:21' if (z<-0.3)&&(Ve(3)<-0.1)&&(takeoffFlag<0.5)
    if ((rtb_Product3_o < -0.3) && (QuadModelv_B.Product[2] < -0.1) &&
        (QuadModelv_DW.takeoffFlag < 0.5)) {
      // '<S196>:1:22' takeoffFlag=int8(1);
      QuadModelv_DW.takeoffFlag = 1;

      // '<S196>:1:23' landFlag=int8(0);
      QuadModelv_DW.landFlag = 0;
    }

    // '<S196>:1:26' if (z>-0.05)&&(Ve(3)>0.1)&&(landFlag<0.5)
    if ((rtb_Product3_o > -0.05) && (QuadModelv_B.Product[2] > 0.1) &&
        (QuadModelv_DW.landFlag < 0.5)) {
      // '<S196>:1:27' landFlag=int8(1);
      QuadModelv_DW.landFlag = 1;

      // '<S196>:1:28' takeoffFlag=int8(0);
      QuadModelv_DW.takeoffFlag = 0;
    }

    // '<S196>:1:31' takeoff = takeoffFlag;
    // '<S196>:1:32' landed = landFlag;
    rtb_landed = QuadModelv_DW.landFlag;

    // '<S196>:1:35' F1=[0;0;0];
    // '<S196>:1:36' M1=[0;0;0];
    QuadModelv_B.F1[0] = 0.0;
    QuadModelv_B.M1[0] = 0.0;
    QuadModelv_B.F1[1] = 0.0;
    QuadModelv_B.M1[1] = 0.0;
    QuadModelv_B.F1[2] = 0.0;
    QuadModelv_B.M1[2] = 0.0;

    // '<S196>:1:38' if z>=0
    if (rtb_Product3_o >= 0.0) {
      //      if z>=0.3
      //          if Ve(3)>=5
      //              F1(3)=-mg(3)-1000*Ve(3);
      //          else
      //              F1(3)=-mg(3)-500*z-100*Ve(3);
      //          end
      //      elseif z>=0.1
      //          if Ve(3)>=2
      //              F1(3)=-mg(3)-100*Ve(3);
      //          else
      //              F1(3)=-mg(3)-50*z-100*Ve(3);
      //          end
      //      else
      //           if Ve(3)>=1
      //              F1(3)=-mg(3)-50*Ve(3);
      //          else
      //              F1(3)=-mg(3)-5*z-50*Ve(3);
      //          end
      //      end
      // '<S196>:1:58' if z<0.05
      if (rtb_Product3_o < 0.05) {
        // '<S196>:1:59' F1(3)=-mg(3)-10*Ve(3);
        QuadModelv_B.F1[2] = -Product1[2] - 10.0 * QuadModelv_B.Product[2];
      } else if (rtb_Product3_o < 0.1) {
        // '<S196>:1:60' elseif z<0.1
        // '<S196>:1:61' F1(3)=-mg(3)-0.5*z-20*Ve(3);
        QuadModelv_B.F1[2] = (-Product1[2] - 0.5 * rtb_Product3_o) - 20.0 *
          QuadModelv_B.Product[2];
      } else {
        // '<S196>:1:62' else
        // '<S196>:1:63' F1(3)=-mg(3)-1*z-50*Ve(3);
        QuadModelv_B.F1[2] = (-Product1[2] - rtb_Product3_o) - 50.0 *
          QuadModelv_B.Product[2];
      }

      // '<S196>:1:66' F1(1)=-50*Ve(1);
      QuadModelv_B.F1[0] = -50.0 * QuadModelv_B.Product[0];

      // '<S196>:1:67' F1(2)=-50*Ve(2);
      QuadModelv_B.F1[1] = -50.0 * QuadModelv_B.Product[1];

      //      if ~(isnan(Quet(2))||isnan(Quet(3))||isnan(Quet(4)))
      //          M1=-200*[Quet(2);Quet(3);Quet(4)]-10*wb;
      //      end
      // '<S196>:1:73' M1=-20*[Quet(2);Quet(3);Quet(4)]-5*wb;
      // '<S196>:1:74' M1 = M1 * abs(mg(3))*0.0333;
      rtb_Product3_o = std::abs(Product1[2]);
      QuadModelv_B.M1[0] = ((((rtb_q2dot * QuadModelv_B.Merge_c[1] +
        rtb_TmpSignalConversionAtSFun_1 * QuadModelv_B.Merge_c[0]) +
        rtb_Product2_kl * QuadModelv_B.Merge_c[3]) - rtb_Rn *
        QuadModelv_B.Merge_c[2]) * -20.0 - 5.0 * QuadModelv_X.pqr_CSTATE[0]) *
        rtb_Product3_o * 0.0333;
      QuadModelv_B.M1[1] = ((((rtb_q2dot * QuadModelv_B.Merge_c[2] -
        rtb_TmpSignalConversionAtSFun_1 * QuadModelv_B.Merge_c[3]) +
        rtb_Product2_kl * QuadModelv_B.Merge_c[0]) + rtb_Rn *
        QuadModelv_B.Merge_c[1]) * -20.0 - 5.0 * QuadModelv_X.pqr_CSTATE[1]) *
        rtb_Product3_o * 0.0333;
      QuadModelv_B.M1[2] = ((((rtb_q2dot * QuadModelv_B.Merge_c[3] +
        rtb_TmpSignalConversionAtSFun_1 * QuadModelv_B.Merge_c[2]) -
        rtb_Product2_kl * QuadModelv_B.Merge_c[1]) + rtb_Rn *
        QuadModelv_B.Merge_c[0]) * -20.0 - 5.0 * QuadModelv_X.pqr_CSTATE[2]) *
        rtb_Product3_o * 0.0333;
    }

    // End of MATLAB Function: '<S7>/Ground Model'
  }

  // DataTypeConversion: '<S8>/Data Type Conversion2' incorporates:
  //   Sum: '<S7>/Add4'

  rtb_DataTypeConversion2[0] = QuadModelv_B.F1[0] + Product1[0];
  rtb_DataTypeConversion2[1] = QuadModelv_B.F1[1] + Product1[1];
  rtb_DataTypeConversion2[2] = QuadModelv_B.F1[2] + Product1[2];

  // Product: '<S7>/Product' incorporates:
  //   Concatenate: '<S36>/Vector Concatenate'

  VectorConcatenate_tmp_0 = rtb_DataTypeConversion2[1];
  rtb_TmpSignalConversionAtSFun_1 = rtb_DataTypeConversion2[0];
  rtb_Rn = rtb_DataTypeConversion2[2];
  for (i = 0; i < 3; i++) {
    rtb_Sum1_ki[i] = (VectorConcatenate[i + 3] * VectorConcatenate_tmp_0 +
                      VectorConcatenate[i] * rtb_TmpSignalConversionAtSFun_1) +
      VectorConcatenate[i + 6] * rtb_Rn;
  }

  // End of Product: '<S7>/Product'
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // MATLAB Function: '<S510>/FaultParamsExtract' incorporates:
    //   Constant: '<Root>/Constant1'
    //   Constant: '<S510>/FaultID'

    // MATLAB Function 'PropFault/FaultParamsExtract': '<S513>:1'
    // '<S513>:1:5' if isempty(hFault)
    // '<S513>:1:8' if isempty(fParam)
    // '<S513>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S513>:1:13' fParamTmp=zeros(20,1);
    std::memset(&fParamTmp[0], 0, 20U * sizeof(real_T));

    // '<S513>:1:14' j=1;
    j = 1;

    // '<S513>:1:15' for i=1:8
    for (r = 0; r < 8; r++) {
      // '<S513>:1:16' if inInts(i) == FaultID
      if (QuadModelv_B.inSILInts[r] == QuadModelv_P.FaultID_Value_d) {
        // '<S513>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S513>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        s458_iter = (r + 1) << 1;
        fParamTmp_tmp = j << 1;
        fParamTmp[fParamTmp_tmp - 2] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 2];

        // '<S513>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[fParamTmp_tmp - 1] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 1];

        // '<S513>:1:20' j=j+1;
        j++;
      }
    }

    // '<S513>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S513>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault = true;

      // '<S513>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = QuadModelv_P.Constant1_Value_a[16];
      fParamTmp[17] = QuadModelv_P.Constant1_Value_a[17];
      fParamTmp[18] = QuadModelv_P.Constant1_Value_a[18];
      fParamTmp[19] = QuadModelv_P.Constant1_Value_a[19];

      // '<S513>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam[0], &fParamTmp[0], 20U * sizeof(real_T));
    }

    // '<S513>:1:29' hasFault=hFault;
    rtb_hasFault = QuadModelv_DW.hFault;

    // '<S513>:1:30' FaultParam=fParam;
    std::memcpy(&rtb_FaultParam[0], &QuadModelv_DW.fParam[0], 20U * sizeof
                (real_T));

    // End of MATLAB Function: '<S510>/FaultParamsExtract'

    // Constant: '<S294>/ModelInit_RPM'
    QuadModelv_B.ModelInit_RPM = QuadModelv_P.ModelInit_RPM;

    // Memory: '<Root>/Memory'
    // MATLAB Function 'Crash_Motor_Fault': '<S4>:1'
    // '<S4>:1:2' y = u;
    Memory = &QuadModelv_DW.Memory_PreviousInput[0];

    // MATLAB Function: '<Root>/Crash_Motor_Fault' incorporates:
    //   Memory: '<Root>/Memory'

    std::memcpy(&QuadModelv_B.y_l[0], &QuadModelv_DW.Memory_PreviousInput[0],
                sizeof(real_T) << 3U);

    // 如果飞机相撞则损坏三个电机
    // '<S4>:1:7' if isCrash>0.5
    if (QuadModelv_B.isCrash > 0.5) {
      // '<S4>:1:8' y(1)=rand()/2.0;
      QuadModelv_B.y_l[0] = QuadModelv_rand_g() / 2.0;

      // '<S4>:1:9' y(2)=rand()/5.0;
      QuadModelv_B.y_l[1] = QuadModelv_rand_g() / 5.0;

      // '<S4>:1:10' y(3)=rand()/10.0+u(3)*0.9;
      QuadModelv_B.y_l[2] = QuadModelv_rand_g() / 10.0 + Memory[2] * 0.9;
    }

    // End of MATLAB Function: '<Root>/Crash_Motor_Fault'
  }

  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0) {
    // MATLAB Function: '<S57>/FaultParamsExtract' incorporates:
    //   Constant: '<Root>/Constant1'
    //   Constant: '<S57>/FaultID'

    // MATLAB Function 'BatteryFault/FaultParamsExtract': '<S59>:1'
    // '<S59>:1:5' if isempty(hFault)
    // '<S59>:1:8' if isempty(fParam)
    // '<S59>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S59>:1:13' fParamTmp=zeros(20,1);
    std::memset(&fParamTmp[0], 0, 20U * sizeof(real_T));

    // '<S59>:1:14' j=1;
    j = 1;

    // '<S59>:1:15' for i=1:8
    for (r = 0; r < 8; r++) {
      // '<S59>:1:16' if inInts(i) == FaultID
      if (QuadModelv_B.inSILInts[r] == QuadModelv_P.FaultID_Value_h) {
        // '<S59>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S59>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        s458_iter = (r + 1) << 1;
        fParamTmp_tmp = j << 1;
        fParamTmp[fParamTmp_tmp - 2] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 2];

        // '<S59>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[fParamTmp_tmp - 1] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 1];

        // '<S59>:1:20' j=j+1;
        j++;
      }
    }

    // '<S59>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S59>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_i0 = true;

      // '<S59>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = QuadModelv_P.Constant1_Value_a[16];
      fParamTmp[17] = QuadModelv_P.Constant1_Value_a[17];
      fParamTmp[18] = QuadModelv_P.Constant1_Value_a[18];
      fParamTmp[19] = QuadModelv_P.Constant1_Value_a[19];

      // '<S59>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_j[0], &fParamTmp[0], 20U * sizeof(real_T));
    }

    // MATLAB Function: '<S57>/FaultParamsExtract1' incorporates:
    //   Constant: '<S57>/FaultID1'

    // '<S59>:1:29' hasFault_isUseCustomHovTime=hFault;
    // '<S59>:1:30' UserCustomFaultParam=fParam;
    // MATLAB Function 'BatteryFault/FaultParamsExtract1': '<S60>:1'
    // '<S60>:1:5' if isempty(hFault)
    // '<S60>:1:8' if isempty(fParam)
    // '<S60>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S60>:1:13' fParamTmp=zeros(20,1);
    // '<S60>:1:14' j=1;
    // '<S60>:1:15' for i=1:8
    for (r = 0; r < 8; r++) {
      // '<S60>:1:16' if inInts(i) == FaultID
      if (QuadModelv_B.inSILInts[r] == QuadModelv_P.FaultID1_Value_h) {
        // '<S60>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S60>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        // '<S60>:1:19' fParamTmp(2*j)=inFloats(2*i);
        // '<S60>:1:20' j=j+1;
      }
    }

    // '<S60>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S60>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_g5 = true;

      // '<S60>:1:25' fParamTmp(17:20) = inFloats(17:20);
      // '<S60>:1:26' fParam=fParamTmp;
    }

    // MATLAB Function: '<S57>/FaultParamsExtract2' incorporates:
    //   Constant: '<Root>/Constant1'
    //   Constant: '<S57>/FaultID2'

    // '<S60>:1:29' hasFault_isPowOff=hFault;
    // '<S60>:1:30' FaultParam=fParam;
    // MATLAB Function 'BatteryFault/FaultParamsExtract2': '<S61>:1'
    // '<S61>:1:5' if isempty(hFault)
    // '<S61>:1:8' if isempty(fParam)
    // '<S61>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S61>:1:13' fParamTmp=zeros(20,1);
    std::memset(&fParamTmp[0], 0, 20U * sizeof(real_T));

    // '<S61>:1:14' j=1;
    j = 1;

    // '<S61>:1:15' for i=1:8
    for (r = 0; r < 8; r++) {
      // '<S61>:1:16' if inInts(i) == FaultID
      if (QuadModelv_B.inSILInts[r] == QuadModelv_P.FaultID2_Value_m) {
        // '<S61>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S61>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        s458_iter = (r + 1) << 1;
        fParamTmp_tmp = j << 1;
        fParamTmp[fParamTmp_tmp - 2] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 2];

        // '<S61>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[fParamTmp_tmp - 1] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 1];

        // '<S61>:1:20' j=j+1;
        j++;
      }
    }

    // '<S61>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S61>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_mv = true;

      // '<S61>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = QuadModelv_P.Constant1_Value_a[16];
      fParamTmp[17] = QuadModelv_P.Constant1_Value_a[17];
      fParamTmp[18] = QuadModelv_P.Constant1_Value_a[18];
      fParamTmp[19] = QuadModelv_P.Constant1_Value_a[19];

      // '<S61>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_iy[0], &fParamTmp[0], 20U * sizeof
                  (real_T));
    }

    // MATLAB Function: '<S57>/FaultParamsExtract3' incorporates:
    //   Constant: '<Root>/Constant1'
    //   Constant: '<S57>/FaultID3'

    // '<S61>:1:29' hasFault_isLowVoltage=hFault;
    // '<S61>:1:30' LowVoltageFaultParam=fParam;
    // MATLAB Function 'BatteryFault/FaultParamsExtract3': '<S62>:1'
    // '<S62>:1:5' if isempty(hFault)
    // '<S62>:1:8' if isempty(fParam)
    // '<S62>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S62>:1:13' fParamTmp=zeros(20,1);
    std::memset(&fParamTmp[0], 0, 20U * sizeof(real_T));

    // '<S62>:1:14' j=1;
    j = 1;

    // '<S62>:1:15' for i=1:8
    for (r = 0; r < 8; r++) {
      // '<S62>:1:16' if inInts(i) == FaultID
      if (QuadModelv_B.inSILInts[r] == QuadModelv_P.FaultID3_Value) {
        // '<S62>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S62>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        s458_iter = (r + 1) << 1;
        fParamTmp_tmp = j << 1;
        fParamTmp[fParamTmp_tmp - 2] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 2];

        // '<S62>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[fParamTmp_tmp - 1] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 1];

        // '<S62>:1:20' j=j+1;
        j++;
      }
    }

    // '<S62>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S62>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_d = true;

      // '<S62>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = QuadModelv_P.Constant1_Value_a[16];
      fParamTmp[17] = QuadModelv_P.Constant1_Value_a[17];
      fParamTmp[18] = QuadModelv_P.Constant1_Value_a[18];
      fParamTmp[19] = QuadModelv_P.Constant1_Value_a[19];

      // '<S62>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_f[0], &fParamTmp[0], 20U * sizeof(real_T));
    }

    // MATLAB Function: '<S2>/MATLAB Function' incorporates:
    //   Constant: '<S2>/ModelParam.BattHoverMinutes'
    //   Constant: '<S2>/ModelParam.BattHoverThr'
    //   Constant: '<S2>/ModelParam.uavMotNumbs'
    //   MATLAB Function: '<S57>/FaultParamsExtract'
    //   MATLAB Function: '<S57>/FaultParamsExtract1'
    //   MATLAB Function: '<S57>/FaultParamsExtract2'
    //   MATLAB Function: '<S57>/FaultParamsExtract3'

    // '<S62>:1:29' hasFault_islowCapacity=hFault;
    // '<S62>:1:30' LowCapFaultParam=fParam;
    hover_current = QuadModelv_P.ModelParam_BattHoverMinutes;
    hFaultTmp = QuadModelv_DW.hFault_i0;
    std::memcpy(&fParamTmp[0], &QuadModelv_DW.fParam_iy[0], 20U * sizeof(real_T));

    //  battery model
    // ================================Constants================================= 
    // MATLAB Function 'Battery FaultModel/MATLAB Function': '<S58>:1'
    // '<S58>:1:5' delta_t = 0.01;
    // max_current = 5.500;                % maximum current per motor             (A) 
    // aux_current = 0.100;                % current of auxillary components       (A)         
    // capacity = 1.000;                   % LiPo current capacity                 (Ah)                                  
    // cells = 2;                          % LiPo cell count
    // ========================================================================== 
    // '<S58>:1:14' if isempty(discharge)
    // '<S58>:1:19' if isempty(startTime)
    // '<S58>:1:24' if isempty(reset)
    // '<S58>:1:28' discharge_capacity=0;
    discharge_capacity = 0.0;

    //  低电量故障
    // '<S58>:1:31' if islowCapacity
    if (QuadModelv_DW.hFault_d) {
      // '<S58>:1:32' hover_time = hover_time*remainCapacityRatio(1);
      hover_current = QuadModelv_P.ModelParam_BattHoverMinutes *
        QuadModelv_DW.fParam_f[0];

      // '<S58>:1:33' isUseCustomHovTime=false;
      hFaultTmp = false;
    }

    // '<S58>:1:37' if ~isUseCustomHovTime
    tmp_4 = !hFaultTmp;
    if (tmp_4) {
      // '<S58>:1:38' capacity=1;
      // '<S58>:1:40' if hover_time<0.01
      if (hover_current < 0.01) {
        // '<S58>:1:41' hover_time=0.01;
        hover_current = 0.01;
      }

      // '<S58>:1:44' hover_current=1/(hover_time*1.4/60);
      hover_current = 1.0 / (hover_current * 1.4 / 60.0);

      // '<S58>:1:45' current=0;
      // '<S58>:1:46' for i=1:motorNumbs
      j = QuadModelv_P.ModelParam_uavMotNumbs;
      for (r = 0; r < j; r++) {
        // '<S58>:1:47' current=current+hover_current*(load(i)/hover_thr)^2;
        rtb_Product3_o = QuadModelv_B.y_l[r] /
          QuadModelv_P.ModelParam_BattHoverThr;
        discharge_capacity += rtb_Product3_o * rtb_Product3_o * hover_current;
      }

      // '<S58>:1:49' aux_current=0.5;
      // current = sum(hover_current .* (load/hover_thr).^2) + aux_current;
      // '<S58>:1:51' current =(current+aux_current)/double(motorNumbs);
      // '<S58>:1:52' discharge = current * (delta_t / 3600) + discharge;
      QuadModelv_DW.discharge += (discharge_capacity + 0.5) / static_cast<real_T>
        (QuadModelv_P.ModelParam_uavMotNumbs) * 2.7777777777777779E-6;

      // '<S58>:1:53' discharge_capacity = discharge / capacity;
      discharge_capacity = QuadModelv_DW.discharge;
    }

    // 如果启用电池模型且使用自定义续航时间
    // '<S58>:1:60' if isUseCustomHovTime
    if (hFaultTmp) {
      // '<S58>:1:61' hover_time=CustomHovTime(1)*1.4*60;
      hover_current = QuadModelv_DW.fParam_j[0] * 1.4 * 60.0;

      // '<S58>:1:62' if reset<0.5
      if (QuadModelv_DW.reset < 0.5) {
        // '<S58>:1:63' startTime=time;
        QuadModelv_DW.startTime = rtb_Clock1_tmp;

        // '<S58>:1:64' reset=1;
        QuadModelv_DW.reset = 1.0;
      }

      // '<S58>:1:66' if hover_time<0.01
      if (hover_current < 0.01) {
        // '<S58>:1:67' hover_time=0.01;
        hover_current = 0.01;
      }

      // '<S58>:1:69' discharge_capacity=(time-startTime)/hover_time;
      discharge_capacity = (rtb_Clock1_tmp - QuadModelv_DW.startTime) /
        hover_current;
    } else {
      // '<S58>:1:70' else
      // 使用默认的续航算法
      // '<S58>:1:72' reset=0;
      QuadModelv_DW.reset = 0.0;
    }

    // '<S58>:1:76' if discharge_capacity<0
    if (discharge_capacity < 0.0) {
      // '<S58>:1:77' discharge_capacity=0;
      discharge_capacity = 0.0;
    }

    // '<S58>:1:79' if discharge_capacity>1
    if (discharge_capacity > 1.0) {
      // '<S58>:1:80' discharge_capacity=1;
      discharge_capacity = 1.0;
    }

    // 电量与电压的曲线
    // '<S58>:1:85' if (0 <= discharge_capacity) && (discharge_capacity <=0.20)
    if ((discharge_capacity >= 0.0) && (discharge_capacity <= 0.2)) {
      // '<S58>:1:86' cell_voltage = -14.029*discharge_capacity^3 + 16.975*discharge_capacity^2 - 5.3339*discharge_capacity + 4.2; 
      hover_current = ((discharge_capacity * discharge_capacity * 16.975 +
                        -14.029 * rt_powd_snf(discharge_capacity, 3.0)) - 5.3339
                       * discharge_capacity) + 4.2;
    } else if ((discharge_capacity > 0.2) && (discharge_capacity < 0.7)) {
      // '<S58>:1:87' elseif (0.20 < discharge_capacity) && (discharge_capacity < 0.70) 
      // '<S58>:1:88' cell_voltage = -0.2*discharge_capacity + 3.74;
      hover_current = -0.2 * discharge_capacity + 3.74;
    } else {
      // '<S58>:1:89' else
      // '<S58>:1:90' cell_voltage = -48*discharge_capacity^3 + 89.6*discharge_capacity^2 - 55.08*discharge_capacity + 14.716; 
      hover_current = ((discharge_capacity * discharge_capacity * 89.6 + -48.0 *
                        rt_powd_snf(discharge_capacity, 3.0)) - 55.08 *
                       discharge_capacity) + 14.716;
    }

    // '<S58>:1:93' if cell_voltage < 0.5
    if (hover_current < 0.5) {
      // '<S58>:1:94' cell_voltage = 0;
      hover_current = 0.0;
    }

    //  电池失效故障
    // '<S58>:1:98' if isPowOff
    if (QuadModelv_DW.hFault_g5) {
      // '<S58>:1:99' cell_voltage = 0;
      hover_current = 0.0;
    }

    //  低电压故障
    // '<S58>:1:103' if isLowVoltage
    if (QuadModelv_DW.hFault_mv) {
      // '<S58>:1:104' if remainVoltageRatio(1)>1
      if (QuadModelv_DW.fParam_iy[0] > 1.0) {
        // '<S58>:1:105' remainVoltageRatio(1)=1;
        fParamTmp[0] = 1.0;
      } else if (QuadModelv_DW.fParam_iy[0] < 0.0) {
        // '<S58>:1:106' elseif remainVoltageRatio(1)<0
        // '<S58>:1:107' remainVoltageRatio(1)=0;
        fParamTmp[0] = 0.0;
      }

      // '<S58>:1:109' cell_voltage = remainVoltageRatio(1)*4.2;
      hover_current = fParamTmp[0] * 4.2;
    }

    // 如果电池模型未启用，续航为无穷久，同时各个全局变量归零
    // '<S58>:1:114' if ~isUseCustomHovTime&&~isPowOff&&~isLowVoltage&&~islowCapacity 
    if (tmp_4 && (!QuadModelv_DW.hFault_g5) && (!QuadModelv_DW.hFault_mv) &&
        (!QuadModelv_DW.hFault_d)) {
      // '<S58>:1:115' cell_voltage=4.2;
      hover_current = 4.2;

      // '<S58>:1:116' discharge = 0;
      QuadModelv_DW.discharge = 0.0;

      // '<S58>:1:117' startTime = 0;
      QuadModelv_DW.startTime = 0.0;

      // '<S58>:1:118' reset = 0;
      QuadModelv_DW.reset = 0.0;
    }

    // 效率矩阵
    // '<S58>:1:122' EffRatio=cell_voltage/4.2;
    // '<S58>:1:123' remCapacity=1-discharge_capacity;
    discharge_capacity = 1.0 - discharge_capacity;

    // Gain: '<S2>/MotorFault3' incorporates:
    //   MATLAB Function: '<S2>/MATLAB Function'

    // ========================================================================== 
    QuadModelv_B.MotorFault3 = QuadModelv_P.MotorFault3_Gain * hover_current;
  }

  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // MATLAB Function: '<S293>/FaultParamsExtract' incorporates:
    //   Constant: '<Root>/Constant1'
    //   Constant: '<S293>/FaultID'

    // MATLAB Function 'MotorFault/FaultParamsExtract': '<S295>:1'
    // '<S295>:1:5' if isempty(hFault)
    // '<S295>:1:8' if isempty(fParam)
    // '<S295>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S295>:1:13' fParamTmp=zeros(20,1);
    std::memset(&fParamTmp[0], 0, 20U * sizeof(real_T));

    // '<S295>:1:14' j=1;
    j = 1;

    // '<S295>:1:16' for i=1:8
    for (r = 0; r < 8; r++) {
      // '<S295>:1:17' if inInts(i) == FaultID
      if (QuadModelv_B.inSILInts[r] == QuadModelv_P.FaultID_Value_n) {
        // '<S295>:1:18' hFaultTmp=true;
        hFaultTmp = true;

        // '<S295>:1:19' fParamTmp(2*j-1)=inFloats(2*i-1);
        s458_iter = (r + 1) << 1;
        fParamTmp_tmp = j << 1;
        fParamTmp[fParamTmp_tmp - 2] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 2];

        // '<S295>:1:20' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[fParamTmp_tmp - 1] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 1];

        // '<S295>:1:21' j=j+1;
        j++;
      }
    }

    // '<S295>:1:24' if hFaultTmp
    if (hFaultTmp) {
      // '<S295>:1:25' hFault=hFaultTmp;
      QuadModelv_DW.hFault_o = true;

      // '<S295>:1:26' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = QuadModelv_P.Constant1_Value_a[16];
      fParamTmp[17] = QuadModelv_P.Constant1_Value_a[17];
      fParamTmp[18] = QuadModelv_P.Constant1_Value_a[18];
      fParamTmp[19] = QuadModelv_P.Constant1_Value_a[19];

      // '<S295>:1:27' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_a[0], &fParamTmp[0], 20U * sizeof(real_T));
    }

    // MATLAB Function: '<S293>/MotorFaultModel' incorporates:
    //   Constant: '<S293>/MotorNum'
    //   MATLAB Function: '<S293>/FaultParamsExtract'
    //   Product: '<S2>/Product'

    // '<S295>:1:30' hasFault=hFault;
    // '<S295>:1:31' FaultParam=fParam;
    for (i = 0; i < 8; i++) {
      QuadModelv_B.y_k[i] = QuadModelv_B.y_l[i] * QuadModelv_B.MotorFault3;
    }

    r = QuadModelv_P.MotorFault1.MotorNum - 1;

    // MATLAB Function 'MotorFault/MotorFaultModel': '<S296>:1'
    // '<S296>:1:2' y = PWMIn;
    // '<S296>:1:4' if motNum<1
    if (QuadModelv_P.MotorFault1.MotorNum < 1) {
      // '<S296>:1:5' motNum=int32(1);
      r = 0;
    }

    // '<S296>:1:8' if motNum>8
    if (r + 1 > 8) {
      // '<S296>:1:9' motNum=int32(8);
      r = 7;
    }

    // '<S296>:1:12' if isMotorFault
    if (QuadModelv_DW.hFault_o) {
      //  FaultParams(1)为标志位
      // '<S296>:1:14' flag = MotorFaultParams(1);
      //  覆盖模式，当为-1时，保持原有输出
      // '<S296>:1:17' if flag == 1
      if (QuadModelv_DW.fParam_a[0] == 1.0) {
        // '<S296>:1:18' for ind = 1:motNum
        for (j = 0; j <= r; j++) {
          // '<S296>:1:19' if MotorFaultParams(ind+1) ~= -1
          if (j + 1 > 2147483646) {
            i = MAX_int32_T;
          } else {
            i = j + 2;
          }

          if (QuadModelv_DW.fParam_a[i - 1] != -1.0) {
            // '<S296>:1:20' y(ind) = MotorFaultParams(ind+1);
            if (j + 1 > 2147483646) {
              i = MAX_int32_T;
            } else {
              i = j + 2;
            }

            QuadModelv_B.y_k[j] = QuadModelv_DW.fParam_a[i - 1];
          }
        }

        //  叠加模式，当为-1时，保持原有输出
      } else if (QuadModelv_DW.fParam_a[0] == 2.0) {
        // '<S296>:1:25' elseif flag == 2
        // '<S296>:1:26' for ind = 1:motNum
        for (j = 0; j <= r; j++) {
          // '<S296>:1:27' if MotorFaultParams(ind+1) ~= -1
          if (j + 1 > 2147483646) {
            i = MAX_int32_T;
          } else {
            i = j + 2;
          }

          if (QuadModelv_DW.fParam_a[i - 1] != -1.0) {
            // '<S296>:1:28' y(ind) = y(ind) + MotorFaultParams(ind+1);
            if (j + 1 > 2147483646) {
              i = MAX_int32_T;
            } else {
              i = j + 2;
            }

            QuadModelv_B.y_k[j] += QuadModelv_DW.fParam_a[i - 1];
          }
        }

        //  噪声模式，当为-1时，保持原有输出
      } else if (QuadModelv_DW.fParam_a[0] == 3.0) {
        // '<S296>:1:33' elseif flag == 3
        // '<S296>:1:34' mean = MotorFaultParams(2);
        // '<S296>:1:35' stddev = MotorFaultParams(3);
        // '<S296>:1:36' for ind = 1:motNum
        for (j = 0; j <= r; j++) {
          // '<S296>:1:37' if MotorFaultParams(ind+3) ~= -1
          if (j + 1 > 2147483644) {
            i = MAX_int32_T;
          } else {
            i = j + 4;
          }

          if (QuadModelv_DW.fParam_a[i - 1] != -1.0) {
            // '<S296>:1:38' y(ind) = y(ind) + generate_wgn(mean,stddev);
            // '<S296>:1:58' if isempty(phase)
            // '<S296>:1:65' if phase
            if (QuadModelv_DW.phase_o) {
              // '<S296>:1:66' while true
              do {
                // '<S296>:1:67' U1 = rand();
                U1 = QuadModelv_rand_f();

                // '<S296>:1:68' U2 = rand();
                X = QuadModelv_rand_f();

                // '<S296>:1:69' V1 = 2.0 * U1 - 1.0;
                U1 = 2.0 * U1 - 1.0;

                // '<S296>:1:70' V2 = 2.0 * U2 - 1.0;
                QuadModelv_DW.V2_jb = 2.0 * X - 1.0;

                // '<S296>:1:71' S = V1 * V1 + V2 * V2;
                QuadModelv_DW.S_kr = U1 * U1 + QuadModelv_DW.V2_jb *
                  QuadModelv_DW.V2_jb;

                // '<S296>:1:72' if S < 1.0 && abs(S) >= 1e-8
              } while ((!(QuadModelv_DW.S_kr < 1.0)) || (!(std::abs
                         (QuadModelv_DW.S_kr) >= 1.0E-8)));

              // '<S296>:1:76' X = V1 * sqrt(-2.0 * log(S) / S);
              X = std::sqrt(-2.0 * std::log(QuadModelv_DW.S_kr) /
                            QuadModelv_DW.S_kr) * U1;
            } else {
              // '<S296>:1:77' else
              // '<S296>:1:78' X = V2 * sqrt(-2.0 * log(S) / S);
              X = std::sqrt(-2.0 * std::log(QuadModelv_DW.S_kr) /
                            QuadModelv_DW.S_kr) * QuadModelv_DW.V2_jb;
            }

            // '<S296>:1:81' y = mean + stddev * X;
            // '<S296>:1:83' phase = ~phase;
            QuadModelv_DW.phase_o = !QuadModelv_DW.phase_o;
            QuadModelv_B.y_k[j] += QuadModelv_DW.fParam_a[2] * X +
              QuadModelv_DW.fParam_a[1];
          }
        }

        //  尺度因子模式，当为-1时，保持原有输出
      } else if (QuadModelv_DW.fParam_a[0] == 4.0) {
        // '<S296>:1:43' elseif flag == 4
        // '<S296>:1:44' for ind = 1:motNum
        for (j = 0; j <= r; j++) {
          // '<S296>:1:45' if MotorFaultParams(ind+1) ~= -1
          if (j + 1 > 2147483646) {
            i = MAX_int32_T;
          } else {
            i = j + 2;
          }

          if (QuadModelv_DW.fParam_a[i - 1] != -1.0) {
            // '<S296>:1:46' y(ind) = y(ind) * MotorFaultParams(ind+1);
            if (j + 1 > 2147483646) {
              i = MAX_int32_T;
            } else {
              i = j + 2;
            }

            QuadModelv_B.y_k[j] *= QuadModelv_DW.fParam_a[i - 1];
          }
        }
      }
    }

    // End of MATLAB Function: '<S293>/MotorFaultModel'
  }

  // Outputs for Atomic SubSystem: '<S294>/MotorNonlinearDynamic1'
  // Constant: '<S294>/ModelParam.motorWb' incorporates:
  //   Constant: '<S294>/ModelParam.motorCr'
  //   Constant: '<S294>/ModelParam.motorT'

  QuadMode_MotorNonlinearDynamic1(QuadModelv_P.ModelParam_motorWb,
    QuadModelv_P.ModelParam_motorCr, QuadModelv_P.ModelParam_motorT,
    QuadModelv_B.ModelInit_RPM, QuadModelv_B.y_k[0],
    &QuadModelv_B.MotorNonlinearDynamic1, &QuadModelv_DW.MotorNonlinearDynamic1,
    &QuadModelv_P.MotorNonlinearDynamic1, &QuadModelv_P,
    &QuadModelv_X.MotorNonlinearDynamic1);

  // End of Outputs for SubSystem: '<S294>/MotorNonlinearDynamic1'

  // Outputs for Atomic SubSystem: '<S294>/MotorNonlinearDynamic2'
  QuadMode_MotorNonlinearDynamic1(QuadModelv_P.ModelParam_motorWb,
    QuadModelv_P.ModelParam_motorCr, QuadModelv_P.ModelParam_motorT,
    QuadModelv_B.ModelInit_RPM, QuadModelv_B.y_k[1],
    &QuadModelv_B.MotorNonlinearDynamic2, &QuadModelv_DW.MotorNonlinearDynamic2,
    &QuadModelv_P.MotorNonlinearDynamic2, &QuadModelv_P,
    &QuadModelv_X.MotorNonlinearDynamic2);

  // End of Outputs for SubSystem: '<S294>/MotorNonlinearDynamic2'

  // Outputs for Atomic SubSystem: '<S294>/MotorNonlinearDynamic3'
  QuadMode_MotorNonlinearDynamic1(QuadModelv_P.ModelParam_motorWb,
    QuadModelv_P.ModelParam_motorCr, QuadModelv_P.ModelParam_motorT,
    QuadModelv_B.ModelInit_RPM, QuadModelv_B.y_k[2],
    &QuadModelv_B.MotorNonlinearDynamic3, &QuadModelv_DW.MotorNonlinearDynamic3,
    &QuadModelv_P.MotorNonlinearDynamic3, &QuadModelv_P,
    &QuadModelv_X.MotorNonlinearDynamic3);

  // End of Outputs for SubSystem: '<S294>/MotorNonlinearDynamic3'

  // Outputs for Atomic SubSystem: '<S294>/MotorNonlinearDynamic4'
  QuadMode_MotorNonlinearDynamic1(QuadModelv_P.ModelParam_motorWb,
    QuadModelv_P.ModelParam_motorCr, QuadModelv_P.ModelParam_motorT,
    QuadModelv_B.ModelInit_RPM, QuadModelv_B.y_k[3],
    &QuadModelv_B.MotorNonlinearDynamic4, &QuadModelv_DW.MotorNonlinearDynamic4,
    &QuadModelv_P.MotorNonlinearDynamic4, &QuadModelv_P,
    &QuadModelv_X.MotorNonlinearDynamic4);

  // End of Outputs for SubSystem: '<S294>/MotorNonlinearDynamic4'

  // Outputs for Atomic SubSystem: '<S294>/MotorNonlinearDynamic5'
  QuadMode_MotorNonlinearDynamic1(QuadModelv_P.ModelParam_motorWb,
    QuadModelv_P.ModelParam_motorCr, QuadModelv_P.ModelParam_motorT,
    QuadModelv_B.ModelInit_RPM, QuadModelv_B.y_k[4],
    &QuadModelv_B.MotorNonlinearDynamic5, &QuadModelv_DW.MotorNonlinearDynamic5,
    &QuadModelv_P.MotorNonlinearDynamic5, &QuadModelv_P,
    &QuadModelv_X.MotorNonlinearDynamic5);

  // End of Outputs for SubSystem: '<S294>/MotorNonlinearDynamic5'

  // Outputs for Atomic SubSystem: '<S294>/MotorNonlinearDynamic6'
  QuadMode_MotorNonlinearDynamic1(QuadModelv_P.ModelParam_motorWb,
    QuadModelv_P.ModelParam_motorCr, QuadModelv_P.ModelParam_motorT,
    QuadModelv_B.ModelInit_RPM, QuadModelv_B.y_k[5],
    &QuadModelv_B.MotorNonlinearDynamic6, &QuadModelv_DW.MotorNonlinearDynamic6,
    &QuadModelv_P.MotorNonlinearDynamic6, &QuadModelv_P,
    &QuadModelv_X.MotorNonlinearDynamic6);

  // End of Outputs for SubSystem: '<S294>/MotorNonlinearDynamic6'

  // Outputs for Atomic SubSystem: '<S294>/MotorNonlinearDynamic7'
  QuadMode_MotorNonlinearDynamic1(QuadModelv_P.ModelParam_motorWb,
    QuadModelv_P.ModelParam_motorCr, QuadModelv_P.ModelParam_motorT,
    QuadModelv_B.ModelInit_RPM, QuadModelv_B.y_k[6],
    &QuadModelv_B.MotorNonlinearDynamic7, &QuadModelv_DW.MotorNonlinearDynamic7,
    &QuadModelv_P.MotorNonlinearDynamic7, &QuadModelv_P,
    &QuadModelv_X.MotorNonlinearDynamic7);

  // End of Outputs for SubSystem: '<S294>/MotorNonlinearDynamic7'

  // Outputs for Atomic SubSystem: '<S294>/MotorNonlinearDynamic8'
  QuadMode_MotorNonlinearDynamic1(QuadModelv_P.ModelParam_motorWb,
    QuadModelv_P.ModelParam_motorCr, QuadModelv_P.ModelParam_motorT,
    QuadModelv_B.ModelInit_RPM, QuadModelv_B.y_k[7],
    &QuadModelv_B.MotorNonlinearDynamic8, &QuadModelv_DW.MotorNonlinearDynamic8,
    &QuadModelv_P.MotorNonlinearDynamic8, &QuadModelv_P,
    &QuadModelv_X.MotorNonlinearDynamic8);

  // End of Outputs for SubSystem: '<S294>/MotorNonlinearDynamic8'
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[4] == 0) {
    // Gain: '<S510>/Gain' incorporates:
    //   Gain: '<S511>/Output'
    //   RandomNumber: '<S511>/White Noise'

    QuadModelv_B.Gain = std::sqrt(QuadModelv_P.BandLimitedWhiteNoise_Cov) /
      0.31622776601683794 * QuadModelv_DW.NextOutput *
      QuadModelv_P.FaultParamAPI.FaultInParams[0];
  }

  // RelationalOperator: '<S512>/Gust Start' incorporates:
  //   Constant: '<S512>/Gust start time'

  rtb_Rn = (rtb_Clock1_tmp >= QuadModelv_P.FaultParamAPI.FaultInParams[2]);
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // Outputs for Enabled SubSystem: '<S512>/Distance into gust (x)' incorporates:
    //   EnablePort: '<S515>/Enable'

    if (rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
      // Logic: '<S512>/Logical Operator2' incorporates:
      //   Constant: '<S512>/Constant'

      if ((rtb_Rn != 0.0) && QuadModelv_P.DiscreteWindGustModel_Gx) {
        if (!QuadModelv_DW.Distanceintogustx_MODE) {
          // InitializeConditions for Integrator: '<S515>/Distance into Gust (x) (Limited to gust length d)' 
          QuadModelv_X.DistanceintoGustxLimitedtogus_e =
            QuadModelv_P.DistanceintoGustxLimitedtogustl;
          QuadModelv_DW.Distanceintogustx_MODE = true;
        }
      } else {
        QuadModelv_DW.Distanceintogustx_MODE = false;
      }

      // End of Logic: '<S512>/Logical Operator2'
    }

    // End of Outputs for SubSystem: '<S512>/Distance into gust (x)'

    // SignalConversion generated from: '<S516>/Enable' incorporates:
    //   Constant: '<S512>/Constant1'
    //   Logic: '<S512>/Logical Operator1'

    QuadModelv_B.HiddenBuf_InsertedFor_Distanc_c = ((rtb_Rn != 0.0) &&
      QuadModelv_P.DiscreteWindGustModel_Gy);
  }

  // Outputs for Enabled SubSystem: '<S512>/Distance into gust (x)' incorporates:
  //   EnablePort: '<S515>/Enable'

  if (QuadModelv_DW.Distanceintogustx_MODE) {
    // Outputs for Enabled SubSystem: '<S512>/Distance into gust (x)'
    // Integrator: '<S515>/Distance into Gust (x) (Limited to gust length d)'
    // Limited  Integrator
    if (QuadModelv_X.DistanceintoGustxLimitedtogus_e >=
        QuadModelv_P.Distanceintogustx_d_m) {
      QuadModelv_X.DistanceintoGustxLimitedtogus_e =
        QuadModelv_P.Distanceintogustx_d_m;
    } else if (QuadModelv_X.DistanceintoGustxLimitedtogus_e <=
               QuadModelv_P.DistanceintoGustxLimitedtogus_j) {
      QuadModelv_X.DistanceintoGustxLimitedtogus_e =
        QuadModelv_P.DistanceintoGustxLimitedtogus_j;
    }

    // End of Outputs for SubSystem: '<S512>/Distance into gust (x)'

    // Integrator: '<S515>/Distance into Gust (x) (Limited to gust length d)'
    QuadModelv_B.DistanceintoGustxLimitedtogustl =
      QuadModelv_X.DistanceintoGustxLimitedtogus_e;
  }

  // End of Outputs for SubSystem: '<S512>/Distance into gust (x)'

  // Outputs for Enabled SubSystem: '<S512>/Distance into gust (y)'
  // Outputs for Enabled SubSystem: '<S512>/Distance into gust (y)'
  QuadModelv_Distanceintogusty(QuadModelv_B.HiddenBuf_InsertedFor_Distanc_c,
    QuadModelv_P.Distanceintogusty_d_m, &QuadModelv_B.Distanceintogusty,
    &QuadModelv_DW.Distanceintogusty, &QuadModelv_P.Distanceintogusty,
    &QuadModelv_X.Distanceintogusty);

  // End of Outputs for SubSystem: '<S512>/Distance into gust (y)'
  // End of Outputs for SubSystem: '<S512>/Distance into gust (y)'
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // SignalConversion generated from: '<S517>/Enable' incorporates:
    //   Constant: '<S512>/Constant2'
    //   Logic: '<S512>/Logical Operator3'

    QuadModelv_B.HiddenBuf_InsertedFor_Distanc_m = ((rtb_Rn != 0.0) &&
      QuadModelv_P.DiscreteWindGustModel_Gz);
  }

  // Outputs for Enabled SubSystem: '<S512>/Distance into gust (z)'
  // Outputs for Enabled SubSystem: '<S512>/Distance into gust (z)'
  QuadModelv_Distanceintogusty(QuadModelv_B.HiddenBuf_InsertedFor_Distanc_m,
    QuadModelv_P.Distanceintogustz_d_m, &QuadModelv_B.Distanceintogustz,
    &QuadModelv_DW.Distanceintogustz, &QuadModelv_P.Distanceintogustz,
    &QuadModelv_X.Distanceintogustz);

  // End of Outputs for SubSystem: '<S512>/Distance into gust (z)'
  // End of Outputs for SubSystem: '<S512>/Distance into gust (z)'

  // DataTypeConversion: '<S8>/Data Type Conversion2' incorporates:
  //   Constant: '<S512>/2'
  //   Gain: '<S512>/Gust magnitude//2.0'
  //   Gain: '<S512>/pi//Gust length'
  //   Sum: '<S512>/Sum1'
  //   Trigonometry: '<S512>/cos(pi*x//dm)'

  rtb_DataTypeConversion2[0] = 3.1415926535897931 /
    QuadModelv_P.DiscreteWindGustModel_d_m[0] *
    QuadModelv_B.DistanceintoGustxLimitedtogustl;
  rtb_DataTypeConversion2[1] = 3.1415926535897931 /
    QuadModelv_P.DiscreteWindGustModel_d_m[1] *
    QuadModelv_B.Distanceintogusty.DistanceintoGustxLimitedtogustl;
  rtb_DataTypeConversion2[2] = 3.1415926535897931 /
    QuadModelv_P.DiscreteWindGustModel_d_m[2] *
    QuadModelv_B.Distanceintogustz.DistanceintoGustxLimitedtogustl;
  rtb_DataTypeConversion2[0] = QuadModelv_P.DiscreteWindGustModel_v_m[0] / 2.0 *
    (QuadModelv_P.u_Value - std::cos(rtb_DataTypeConversion2[0]));
  rtb_DataTypeConversion2[1] = QuadModelv_P.DiscreteWindGustModel_v_m[1] / 2.0 *
    (QuadModelv_P.u_Value - std::cos(rtb_DataTypeConversion2[1]));
  rtb_DataTypeConversion2[2] = QuadModelv_P.DiscreteWindGustModel_v_m[2] / 2.0 *
    (QuadModelv_P.u_Value - std::cos(rtb_DataTypeConversion2[2]));

  // Sum: '<S510>/Sum'
  rtb_Product3_o = QuadModelv_B.Gain + rtb_DataTypeConversion2[0];
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[2] == 0) {
    // UniformRandomNumber: '<S67>/Uniform Random Number'
    QuadModelv_B.UniformRandomNumber[0] =
      QuadModelv_DW.UniformRandomNumber_NextOutput[0];
    QuadModelv_B.UniformRandomNumber[1] =
      QuadModelv_DW.UniformRandomNumber_NextOutput[1];
    QuadModelv_B.UniformRandomNumber[2] =
      QuadModelv_DW.UniformRandomNumber_NextOutput[2];
  }

  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // SignalConversion generated from: '<S514>/ SFunction ' incorporates:
    //   MATLAB Function: '<S510>/PropFaultModel'

    rtb_Gain_ok[0] = QuadModelv_B.MotorNonlinearDynamic1.Motor_Dynamics.x;
    rtb_Gain_ok[1] = QuadModelv_B.MotorNonlinearDynamic2.Motor_Dynamics.x;
    rtb_Gain_ok[2] = QuadModelv_B.MotorNonlinearDynamic3.Motor_Dynamics.x;
    rtb_Gain_ok[3] = QuadModelv_B.MotorNonlinearDynamic4.Motor_Dynamics.x;
    rtb_Gain_ok[4] = QuadModelv_B.MotorNonlinearDynamic5.Motor_Dynamics.x;
    rtb_Gain_ok[5] = QuadModelv_B.MotorNonlinearDynamic6.Motor_Dynamics.x;
    rtb_Gain_ok[6] = QuadModelv_B.MotorNonlinearDynamic7.Motor_Dynamics.x;
    rtb_Gain_ok[7] = QuadModelv_B.MotorNonlinearDynamic8.Motor_Dynamics.x;

    // MATLAB Function: '<S510>/PropFaultModel' incorporates:
    //   Constant: '<S510>/FaultID1'

    j = QuadModelv_P.PropFault.PropNum;

    //  本模块需要用到8维FaultParam的输入
    // 因此故障ID的格式应该是：
    // inSILInts=[FaultID FaultID FaultID FaultID ...]
    // inSILInts=[param1 param2 param3 param4 param5 param6 param7 param8 ...] -> FaultParam 
    //  param的每一位表示本电机的健康系数（0到1），实际上就是pwm_out = pwm_in * param 
    //  如果param1=0，则电机1直接坏掉，始终输出0
    // MATLAB Function 'PropFault/PropFaultModel': '<S514>:1'
    // '<S514>:1:9' y=inPWMs;
    std::memcpy(&QuadModelv_B.y[0], &rtb_Gain_ok[0], sizeof(real_T) << 3U);

    // 默认情况下不注入故障，直接将数据输出
    // '<S514>:1:11' if PropNum<1
    if (QuadModelv_P.PropFault.PropNum < 1) {
      // '<S514>:1:12' PropNum=int32(1);
      j = 1;
    }

    // '<S514>:1:15' if PropNum>8
    if (j > 8) {
      // '<S514>:1:16' PropNum=int32(8);
      j = 8;
    }

    // '<S514>:1:19' if hasFault
    if (rtb_hasFault) {
      // 如果注入了故障
      // '<S514>:1:20' for i=1:PropNum
      for (r = 0; r < j; r++) {
        // '<S514>:1:21' y(i) = inPWMs(i) * FaultParam(i)+ noise;
        QuadModelv_B.y[r] = rtb_Gain_ok[r] * rtb_FaultParam[r] + rtb_Product3_o;
      }
    }

    // MATLAB Function: '<S67>/FaultParamsExtract4' incorporates:
    //   Constant: '<Root>/Constant1'
    //   Constant: '<S67>/FaultID4'

    // MATLAB Function 'WindFault/FaultParamsExtract4': '<S100>:1'
    // '<S100>:1:5' if isempty(hFault)
    // '<S100>:1:8' if isempty(fParam)
    // '<S100>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S100>:1:13' fParamTmp=zeros(20,1);
    std::memset(&fParamTmp[0], 0, 20U * sizeof(real_T));

    // '<S100>:1:14' j=1;
    j = 1;

    // '<S100>:1:15' for i=1:8
    for (r = 0; r < 8; r++) {
      // '<S100>:1:16' if inInts(i) == FaultID
      if (QuadModelv_B.inSILInts[r] == QuadModelv_P.FaultID4_Value) {
        // '<S100>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S100>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        s458_iter = (r + 1) << 1;
        fParamTmp_tmp = j << 1;
        fParamTmp[fParamTmp_tmp - 2] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 2];

        // '<S100>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[fParamTmp_tmp - 1] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 1];

        // '<S100>:1:20' j=j+1;
        j++;
      }
    }

    // '<S100>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S100>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_ak = true;

      // '<S100>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = QuadModelv_P.Constant1_Value_a[16];
      fParamTmp[17] = QuadModelv_P.Constant1_Value_a[17];
      fParamTmp[18] = QuadModelv_P.Constant1_Value_a[18];
      fParamTmp[19] = QuadModelv_P.Constant1_Value_a[19];

      // '<S100>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_de[0], &fParamTmp[0], 20U * sizeof
                  (real_T));
    }

    // '<S100>:1:29' hasFault_WindNoise=hFault;
    // '<S100>:1:30' FaultParam=fParam;
    std::memcpy(&fParamTmp[0], &QuadModelv_DW.fParam_de[0], 20U * sizeof(real_T));

    // MATLAB Function: '<S67>/noiseUpperWindBodySwitch' incorporates:
    //   MATLAB Function: '<S67>/FaultParamsExtract4'

    // MATLAB Function 'WindFault/noiseUpperWindBodySwitch': '<S106>:1'
    // '<S106>:1:3' y = [0;0;0];
    Product1[0] = 0.0;
    Product1[1] = 0.0;
    Product1[2] = 0.0;

    // '<S106>:1:4' if isNoiseFault
    if (QuadModelv_DW.hFault_ak) {
      // '<S106>:1:5' Param_GlobalNoiseGainSwitch=NoiseParams(1);
      // '<S106>:1:6' y=Noise* Param_GlobalNoiseGainSwitch;
      Product1[0] = QuadModelv_B.UniformRandomNumber[0] * fParamTmp[0];
      Product1[1] = fParamTmp[0] * QuadModelv_B.UniformRandomNumber[1];
      Product1[2] = fParamTmp[0] * QuadModelv_B.UniformRandomNumber[2];
    }

    // End of MATLAB Function: '<S67>/noiseUpperWindBodySwitch'

    // Sum: '<S67>/Sum1' incorporates:
    //   Constant: '<S67>/Constant_[1 1 1]'

    QuadModelv_B.Sum1[0] = QuadModelv_P.Constant_111_Value[0] + Product1[0];
    QuadModelv_B.Sum1[1] = QuadModelv_P.Constant_111_Value[1] + Product1[1];
    QuadModelv_B.Sum1[2] = QuadModelv_P.Constant_111_Value[2] + Product1[2];

    // MATLAB Function: '<S67>/FaultParamsExtract' incorporates:
    //   Constant: '<Root>/Constant1'
    //   Constant: '<S67>/FaultID'

    // MATLAB Function 'WindFault/FaultParamsExtract': '<S96>:1'
    // '<S96>:1:5' if isempty(hFault)
    // '<S96>:1:8' if isempty(fParam)
    // '<S96>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S96>:1:13' fParamTmp=zeros(20,1);
    std::memset(&fParamTmp[0], 0, 20U * sizeof(real_T));

    // '<S96>:1:14' j=1;
    j = 1;

    // '<S96>:1:15' for i=1:8
    for (r = 0; r < 8; r++) {
      // '<S96>:1:16' if inInts(i) == FaultID
      if (QuadModelv_B.inSILInts[r] == QuadModelv_P.FaultID_Value_a) {
        // '<S96>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S96>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        s458_iter = (r + 1) << 1;
        fParamTmp_tmp = j << 1;
        fParamTmp[fParamTmp_tmp - 2] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 2];

        // '<S96>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[fParamTmp_tmp - 1] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 1];

        // '<S96>:1:20' j=j+1;
        j++;
      }
    }

    // '<S96>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S96>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_ax = true;

      // '<S96>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = QuadModelv_P.Constant1_Value_a[16];
      fParamTmp[17] = QuadModelv_P.Constant1_Value_a[17];
      fParamTmp[18] = QuadModelv_P.Constant1_Value_a[18];
      fParamTmp[19] = QuadModelv_P.Constant1_Value_a[19];

      // '<S96>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_la[0], &fParamTmp[0], 20U * sizeof
                  (real_T));
    }

    // '<S96>:1:29' hasFault_ConstWind=hFault;
    QuadModelv_B.hasFault_ConstWind = QuadModelv_DW.hFault_ax;

    // MATLAB Function: '<S67>/FaultParamsExtract1'
    // '<S96>:1:30' FaultParam=fParam;
    // MATLAB Function 'WindFault/FaultParamsExtract1': '<S97>:1'
    // '<S97>:1:5' if isempty(hFault)
    // '<S97>:1:8' if isempty(fParam)
    // '<S97>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S97>:1:13' fParamTmp=zeros(20,1);
    for (i = 0; i < 20; i++) {
      // MATLAB Function: '<S67>/FaultParamsExtract'
      QuadModelv_B.FaultParam_p[i] = QuadModelv_DW.fParam_la[i];

      // MATLAB Function: '<S67>/FaultParamsExtract1'
      fParamTmp[i] = 0.0;
    }

    // MATLAB Function: '<S67>/FaultParamsExtract1' incorporates:
    //   Constant: '<Root>/Constant1'
    //   Constant: '<S67>/FaultID1'

    // '<S97>:1:14' j=1;
    j = 1;

    // '<S97>:1:15' for i=1:8
    for (r = 0; r < 8; r++) {
      // '<S97>:1:16' if inInts(i) == FaultID
      if (QuadModelv_B.inSILInts[r] == QuadModelv_P.FaultID1_Value_p) {
        // '<S97>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S97>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        s458_iter = (r + 1) << 1;
        fParamTmp_tmp = j << 1;
        fParamTmp[fParamTmp_tmp - 2] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 2];

        // '<S97>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[fParamTmp_tmp - 1] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 1];

        // '<S97>:1:20' j=j+1;
        j++;
      }
    }

    // '<S97>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S97>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_i = true;

      // '<S97>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = QuadModelv_P.Constant1_Value_a[16];
      fParamTmp[17] = QuadModelv_P.Constant1_Value_a[17];
      fParamTmp[18] = QuadModelv_P.Constant1_Value_a[18];
      fParamTmp[19] = QuadModelv_P.Constant1_Value_a[19];

      // '<S97>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_ae[0], &fParamTmp[0], 20U * sizeof
                  (real_T));
    }

    // '<S97>:1:29' hasFault_GustWind=hFault;
    QuadModelv_B.hasFault_GustWind = QuadModelv_DW.hFault_i;

    // MATLAB Function: '<S67>/FaultParamsExtract2'
    // '<S97>:1:30' FaultParam=fParam;
    // MATLAB Function 'WindFault/FaultParamsExtract2': '<S98>:1'
    // '<S98>:1:5' if isempty(hFault)
    // '<S98>:1:8' if isempty(fParam)
    // '<S98>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S98>:1:13' fParamTmp=zeros(20,1);
    for (i = 0; i < 20; i++) {
      // MATLAB Function: '<S67>/FaultParamsExtract1'
      QuadModelv_B.FaultParam_g[i] = QuadModelv_DW.fParam_ae[i];

      // MATLAB Function: '<S67>/FaultParamsExtract2'
      fParamTmp[i] = 0.0;
    }

    // MATLAB Function: '<S67>/FaultParamsExtract2' incorporates:
    //   Constant: '<Root>/Constant1'
    //   Constant: '<S67>/FaultID2'

    // '<S98>:1:14' j=1;
    j = 1;

    // '<S98>:1:15' for i=1:8
    for (r = 0; r < 8; r++) {
      // '<S98>:1:16' if inInts(i) == FaultID
      if (QuadModelv_B.inSILInts[r] == QuadModelv_P.FaultID2_Value_j) {
        // '<S98>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S98>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        s458_iter = (r + 1) << 1;
        fParamTmp_tmp = j << 1;
        fParamTmp[fParamTmp_tmp - 2] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 2];

        // '<S98>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[fParamTmp_tmp - 1] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 1];

        // '<S98>:1:20' j=j+1;
        j++;
      }
    }

    // '<S98>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S98>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_m = true;

      // '<S98>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = QuadModelv_P.Constant1_Value_a[16];
      fParamTmp[17] = QuadModelv_P.Constant1_Value_a[17];
      fParamTmp[18] = QuadModelv_P.Constant1_Value_a[18];
      fParamTmp[19] = QuadModelv_P.Constant1_Value_a[19];

      // '<S98>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_l[0], &fParamTmp[0], 20U * sizeof(real_T));
    }

    // '<S98>:1:29' hasFault_TurbWind=hFault;
    QuadModelv_B.hasFault_TurbWind = QuadModelv_DW.hFault_m;

    // MATLAB Function: '<S67>/FaultParamsExtract3'
    // '<S98>:1:30' FaultParam=fParam;
    // MATLAB Function 'WindFault/FaultParamsExtract3': '<S99>:1'
    // '<S99>:1:5' if isempty(hFault)
    // '<S99>:1:8' if isempty(fParam)
    // '<S99>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S99>:1:13' fParamTmp=zeros(20,1);
    for (i = 0; i < 20; i++) {
      // MATLAB Function: '<S67>/FaultParamsExtract2'
      QuadModelv_B.FaultParam_i[i] = QuadModelv_DW.fParam_l[i];

      // MATLAB Function: '<S67>/FaultParamsExtract3'
      fParamTmp[i] = 0.0;
    }

    // MATLAB Function: '<S67>/FaultParamsExtract3' incorporates:
    //   Constant: '<Root>/Constant1'
    //   Constant: '<S67>/FaultID3'

    // '<S99>:1:14' j=1;
    j = 1;

    // '<S99>:1:15' for i=1:8
    for (r = 0; r < 8; r++) {
      // '<S99>:1:16' if inInts(i) == FaultID
      if (QuadModelv_B.inSILInts[r] == QuadModelv_P.FaultID3_Value_d) {
        // '<S99>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S99>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        s458_iter = (r + 1) << 1;
        fParamTmp_tmp = j << 1;
        fParamTmp[fParamTmp_tmp - 2] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 2];

        // '<S99>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[fParamTmp_tmp - 1] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 1];

        // '<S99>:1:20' j=j+1;
        j++;
      }
    }

    // '<S99>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S99>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_b = true;

      // '<S99>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = QuadModelv_P.Constant1_Value_a[16];
      fParamTmp[17] = QuadModelv_P.Constant1_Value_a[17];
      fParamTmp[18] = QuadModelv_P.Constant1_Value_a[18];
      fParamTmp[19] = QuadModelv_P.Constant1_Value_a[19];

      // '<S99>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_h[0], &fParamTmp[0], 20U * sizeof(real_T));
    }

    // '<S99>:1:29' hasFault_SheerWind=hFault;
    QuadModelv_B.hasFault_SheerWind = QuadModelv_DW.hFault_b;

    // '<S99>:1:30' FaultParam=fParam;
    std::memcpy(&QuadModelv_B.FaultParam_a[0], &QuadModelv_DW.fParam_h[0], 20U *
                sizeof(real_T));
  }

  // Gain: '<S67>/Gain_-1' incorporates:
  //   Integrator: '<S14>/xe,ye,ze'

  rtb_Rn = QuadModelv_P.Gain_1_Gain * QuadModelv_X.xeyeze_CSTATE[2];

  // Saturate: '<S67>/Saturation_2'
  if (rtb_Rn > QuadModelv_P.Saturation_2_UpperSat) {
    rtb_Rn = QuadModelv_P.Saturation_2_UpperSat;
  } else if (rtb_Rn < QuadModelv_P.Saturation_2_LowerSat) {
    rtb_Rn = QuadModelv_P.Saturation_2_LowerSat;
  }

  // End of Saturate: '<S67>/Saturation_2'

  // UnitConversion: '<S149>/Unit Conversion' incorporates:
  //   UnitConversion: '<S110>/Unit Conversion'
  //   UnitConversion: '<S186>/Unit Conversion'

  // Unit Conversion - from: m to: ft
  // Expression: output = (3.28084*input) + (0)
  VectorConcatenate_tmp = 3.280839895013123 * rtb_Rn;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // UnitConversion: '<S155>/Unit Conversion' incorporates:
    //   Constant: '<S67>/Constant_V'

    // Unit Conversion - from: m/s to: ft/s
    // Expression: output = (3.28084*input) + (0)
    QuadModelv_B.UnitConversion = 3.280839895013123 *
      QuadModelv_P.Constant_V_Value;
  }

  // Saturate: '<S182>/Limit Function 10ft to 1000ft' incorporates:
  //   UnitConversion: '<S149>/Unit Conversion'

  if (VectorConcatenate_tmp > QuadModelv_P.LimitFunction10ftto1000ft_Upper) {
    rtb_TmpSignalConversionAtSFun_1 =
      QuadModelv_P.LimitFunction10ftto1000ft_Upper;
  } else if (VectorConcatenate_tmp <
             QuadModelv_P.LimitFunction10ftto1000ft_Lower) {
    rtb_TmpSignalConversionAtSFun_1 =
      QuadModelv_P.LimitFunction10ftto1000ft_Lower;
  } else {
    rtb_TmpSignalConversionAtSFun_1 = VectorConcatenate_tmp;
  }

  // End of Saturate: '<S182>/Limit Function 10ft to 1000ft'

  // Fcn: '<S182>/Low Altitude Scale Length'
  rtb_q3dot = 0.000823 * rtb_TmpSignalConversionAtSFun_1 + 0.177;
  if (rtb_q3dot < 0.0) {
    rtb_sincos_o1_idx_1 = -rt_powd_snf(-rtb_q3dot, 1.2);
  } else {
    rtb_sincos_o1_idx_1 = rt_powd_snf(rtb_q3dot, 1.2);
  }

  // Fcn: '<S182>/Low Altitude Scale Length'
  rtb_Product2_kl = rtb_TmpSignalConversionAtSFun_1 / rtb_sincos_o1_idx_1;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // UnitConversion: '<S184>/Unit Conversion' incorporates:
    //   Constant: '<S183>/Medium//High Altitude'

    // Unit Conversion - from: m to: ft
    // Expression: output = (3.28084*input) + (0)
    QuadModelv_B.UnitConversion_g = 3.280839895013123 *
      QuadModelv_P.DrydenWindTurbulenceModelDiscre;
  }

  // Saturate: '<S165>/Limit Height h<1000ft' incorporates:
  //   UnitConversion: '<S149>/Unit Conversion'

  if (VectorConcatenate_tmp > QuadModelv_P.LimitHeighth1000ft_UpperSat) {
    rtb_sincos_o1_idx_1 = QuadModelv_P.LimitHeighth1000ft_UpperSat;
  } else if (VectorConcatenate_tmp < QuadModelv_P.LimitHeighth1000ft_LowerSat) {
    rtb_sincos_o1_idx_1 = QuadModelv_P.LimitHeighth1000ft_LowerSat;
  } else {
    rtb_sincos_o1_idx_1 = VectorConcatenate_tmp;
  }

  // Fcn: '<S165>/Low Altitude Intensity' incorporates:
  //   Saturate: '<S165>/Limit Height h<1000ft'

  rtb_q3dot = 0.000823 * rtb_sincos_o1_idx_1 + 0.177;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // Gain: '<S165>/sigma_wg ' incorporates:
    //   Constant: '<S95>/Windspeed at 20ft (6m)'
    //   UnitConversion: '<S156>/Unit Conversion'

    // Unit Conversion - from: m/s to: ft/s
    // Expression: output = (3.28084*input) + (0)
    QuadModelv_B.sigma_wg = 3.280839895013123 *
      QuadModelv_P.FaultParamAPI.FaultInParams[7] * QuadModelv_P.sigma_wg_Gain;
  }

  // Fcn: '<S165>/Low Altitude Intensity'
  if (rtb_q3dot < 0.0) {
    rtb_sincos_o1_idx_1 = -rt_powd_snf(-rtb_q3dot, 0.4);
  } else {
    rtb_sincos_o1_idx_1 = rt_powd_snf(rtb_q3dot, 0.4);
  }

  // Product: '<S165>/sigma_ug, sigma_vg' incorporates:
  //   Fcn: '<S165>/Low Altitude Intensity'

  rtb_Product3_o = 1.0 / rtb_sincos_o1_idx_1 * QuadModelv_B.sigma_wg;

  // Interpolation_n-D: '<S164>/Medium//High Altitude Intensity' incorporates:
  //   PreLookup: '<S164>/PreLook-Up Index Search  (altitude)'
  //   UnitConversion: '<S149>/Unit Conversion'

  bpIndex[0] = plook_bincpa(VectorConcatenate_tmp,
    QuadModelv_P.PreLookUpIndexSearchaltitude_Br, 11U, &rtb_q3dot,
    &QuadModelv_DW.PreLookUpIndexSearchaltitude_DW);
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // PreLookup: '<S164>/PreLook-Up Index Search  (prob of exceed)' incorporates:
    //   Constant: '<S164>/Probability of  Exceedance'

    QuadModelv_B.PreLookUpIndexSearchprobofexc_p = plook_bincpa
      (QuadModelv_P.DrydenWindTurbulenceModelDis_n3,
       QuadModelv_P.PreLookUpIndexSearchprobofexcee, 6U,
       &QuadModelv_B.PreLookUpIndexSearchprobofexcee,
       &QuadModelv_DW.PreLookUpIndexSearchprobofexcee);
  }

  // Interpolation_n-D: '<S164>/Medium//High Altitude Intensity'
  frac[0] = rtb_q3dot;
  frac[1] = QuadModelv_B.PreLookUpIndexSearchprobofexcee;
  bpIndex[1] = QuadModelv_B.PreLookUpIndexSearchprobofexc_p;

  // Interpolation_n-D: '<S164>/Medium//High Altitude Intensity'
  rtb_Add = intrp2d_la_pw(bpIndex, frac,
    QuadModelv_P.MediumHighAltitudeIntensity_Tab, 12U,
    QuadModelv_P.MediumHighAltitudeIntensity_max);
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0) {
    // Sqrt: '<S157>/Sqrt1' incorporates:
    //   Constant: '<S157>/Constant1'

    rtb_q3dot = std::sqrt(QuadModelv_P.WhiteNoise_Ts);

    // Product: '<S157>/Product' incorporates:
    //   Constant: '<S157>/Constant'
    //   Product: '<S157>/Divide'
    //   RandomNumber: '<S157>/White Noise'
    //   Sqrt: '<S157>/Sqrt'

    QuadModelv_B.Product_d[0] = std::sqrt(QuadModelv_P.WhiteNoise_pwr[0]) /
      rtb_q3dot * QuadModelv_DW.NextOutput_c[0];
    QuadModelv_B.Product_d[1] = std::sqrt(QuadModelv_P.WhiteNoise_pwr[1]) /
      rtb_q3dot * QuadModelv_DW.NextOutput_c[1];
    QuadModelv_B.Product_d[2] = std::sqrt(QuadModelv_P.WhiteNoise_pwr[2]) /
      rtb_q3dot * QuadModelv_DW.NextOutput_c[2];
    QuadModelv_B.Product_d[3] = std::sqrt(QuadModelv_P.WhiteNoise_pwr[3]) /
      rtb_q3dot * QuadModelv_DW.NextOutput_c[3];
  }

  // Outputs for Enabled SubSystem: '<S148>/Hugw(z)'
  // Constant: '<S148>/Constant3'
  QuadModelv_Hugwz(QuadModelv_P.DrydenWindTurbulenceModelDisc_g,
                   QuadModelv_B.UnitConversion, rtb_Product2_kl,
                   QuadModelv_B.UnitConversion_g, rtb_Product3_o, rtb_Add,
                   QuadModelv_B.Product_d[0], &QuadModelv_B.Hugwz_g,
                   &QuadModelv_DW.Hugwz_g, &QuadModelv_P.Hugwz_g);

  // End of Outputs for SubSystem: '<S148>/Hugw(z)'

  // Gain: '<S154>/Lv'
  rtb_Sum_d[0] = QuadModelv_P.Lv_Gain * rtb_Product2_kl;
  rtb_Sum_d[1] = QuadModelv_P.Lv_Gain * QuadModelv_B.UnitConversion_g;

  // Outputs for Enabled SubSystem: '<S148>/Hvgw(z)'
  // Constant: '<S148>/Constant3'
  QuadModelv_Hvgwz(QuadModelv_P.DrydenWindTurbulenceModelDisc_g, rtb_Product3_o,
                   rtb_Add, rtb_Sum_d, QuadModelv_B.UnitConversion,
                   QuadModelv_B.Product_d[1], &QuadModelv_B.Hvgwz_f,
                   &QuadModelv_DW.Hvgwz_f, &QuadModelv_P.Hvgwz_f);

  // End of Outputs for SubSystem: '<S148>/Hvgw(z)'

  // Gain: '<S154>/Lw'
  rtb_Sum_d[0] = QuadModelv_P.Lw_Gain * rtb_TmpSignalConversionAtSFun_1;
  rtb_Sum_d[1] = QuadModelv_P.Lw_Gain * QuadModelv_B.UnitConversion_g;

  // Outputs for Enabled SubSystem: '<S148>/Hwgw(z)'
  // Constant: '<S148>/Constant3'
  QuadModelv_Hwgwz(QuadModelv_P.DrydenWindTurbulenceModelDisc_g,
                   QuadModelv_B.UnitConversion, rtb_Sum_d, QuadModelv_B.sigma_wg,
                   rtb_Add, QuadModelv_B.Product_d[2], &QuadModelv_B.Hwgwz_a,
                   &QuadModelv_DW.Hwgwz_a, &QuadModelv_P.Hwgwz_a);

  // End of Outputs for SubSystem: '<S148>/Hwgw(z)'
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // UnitConversion: '<S146>/Unit Conversion' incorporates:
    //   Constant: '<S95>/Wind direction'

    // Unit Conversion - from: deg to: rad
    // Expression: output = (0.0174533*input) + (0)
    QuadModelv_B.UnitConversion_p = 0.017453292519943295 *
      QuadModelv_P.FaultParamAPI.FaultInParams[8];
  }

  // If: '<S153>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' incorporates:
  //   Constant: '<S67>/Constant_DCM'
  //   UnitConversion: '<S149>/Unit Conversion'

  if (rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    if (VectorConcatenate_tmp <= 1000.0) {
      rtAction = 0;
    } else if (VectorConcatenate_tmp >= 2000.0) {
      rtAction = 1;
    } else {
      rtAction = 2;
    }

    QuadModelv_DW.ifHeightMaxlowaltitudeelseifHei = rtAction;
  } else {
    rtAction = QuadModelv_DW.ifHeightMaxlowaltitudeelseifHei;
  }

  switch (rtAction) {
   case 0:
    // Outputs for IfAction SubSystem: '<S153>/Low altitude  velocities' incorporates:
    //   ActionPort: '<S175>/Action Port'

    QuadModelv_Lowaltituderates(QuadModelv_P.Constant_DCM_Value,
      QuadModelv_B.Hugwz_g.Sum, QuadModelv_B.Hvgwz_f.Sum,
      QuadModelv_B.Hwgwz_a.Sum, QuadModelv_B.UnitConversion_p,
      rtb_DataTypeConversion2);

    // End of Outputs for SubSystem: '<S153>/Low altitude  velocities'
    break;

   case 1:
    // Outputs for IfAction SubSystem: '<S153>/Medium//High  altitude velocities' incorporates:
    //   ActionPort: '<S176>/Action Port'

    // DataTypeConversion: '<S8>/Data Type Conversion2' incorporates:
    //   Gain: '<S176>/Gain'

    rtb_DataTypeConversion2[0] = QuadModelv_P.Gain_Gain_h *
      QuadModelv_B.Hugwz_g.Sum[1];
    rtb_DataTypeConversion2[1] = QuadModelv_P.Gain_Gain_h *
      QuadModelv_B.Hvgwz_f.Sum[1];
    rtb_DataTypeConversion2[2] = QuadModelv_P.Gain_Gain_h *
      QuadModelv_B.Hwgwz_a.Sum[1];

    // End of Outputs for SubSystem: '<S153>/Medium//High  altitude velocities'
    break;

   default:
    // Outputs for IfAction SubSystem: '<S153>/Interpolate  velocities' incorporates:
    //   ActionPort: '<S174>/Action Port'

    QuadModel_Interpolatevelocities(QuadModelv_B.Hugwz_g.Sum,
      QuadModelv_B.Hvgwz_f.Sum, QuadModelv_B.Hwgwz_a.Sum,
      QuadModelv_P.Constant_DCM_Value, QuadModelv_B.UnitConversion_p,
      VectorConcatenate_tmp, rtb_DataTypeConversion2,
      &QuadModelv_P.Interpolatevelocities_i);

    // End of Outputs for SubSystem: '<S153>/Interpolate  velocities'
    break;
  }

  // End of If: '<S153>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' 
  // Unit Conversion - from: ft/s to: m/s
  // Expression: output = (0.3048*input) + (0)
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // MATLAB Function: '<S67>/MATLAB Function' incorporates:
    //   UnitConversion: '<S95>/Unit Conversion'

    std::memcpy(&rtb_FaultParam[0], &QuadModelv_B.FaultParam_g[0], 20U * sizeof
                (real_T));

    // MATLAB Function 'WindFault/MATLAB Function': '<S101>:1'
    // '<S101>:1:3' if(GustWindParams(2)<1)
    if (QuadModelv_B.FaultParam_g[1] < 1.0) {
      // '<S101>:1:4' GustWindParams(2)=1;
      rtb_FaultParam[1] = 1.0;
    }

    // '<S101>:1:6' timeVel=60/GustWindParams(2);
    rtb_Product2_kl = 60.0 / rtb_FaultParam[1];

    // '<S101>:1:8' vmax=GustWindParams(1);
    rtb_Product3_o = QuadModelv_B.FaultParam_g[0];

    // '<S101>:1:11' if isempty(t0)
    // '<S101>:1:16' if isempty(isInGust)
    // '<S101>:1:21' if isempty(t1)
    // '<S101>:1:26' if isempty(a)
    // '<S101>:1:31' if isempty(ang)
    // '<S101>:1:36' if isempty(wlast)
    if (!QuadModelv_DW.wlast_not_empty) {
      // '<S101>:1:37' wlast=vmax*0.5;
      QuadModelv_DW.wlast = QuadModelv_B.FaultParam_g[0] * 0.5;
      QuadModelv_DW.wlast_not_empty = true;
    }

    // '<S101>:1:41' if isempty(wNow)
    if (!QuadModelv_DW.wNow_not_empty) {
      // '<S101>:1:42' wNow=1.4*0.8*vmax;
      QuadModelv_DW.wNow = 1.1199999999999999 * QuadModelv_B.FaultParam_g[0];
      QuadModelv_DW.wNow_not_empty = true;
    }

    // '<S101>:1:46' if isGustWind&&isInGust<0.5
    if (QuadModelv_B.hasFault_GustWind && (QuadModelv_DW.isInGust < 0.5)) {
      // '<S101>:1:47' t0=t;
      QuadModelv_DW.t0 = rtb_Clock1_tmp;

      // '<S101>:1:48' t1=timeVel*(3/4+1/2*rand(1));
      QuadModelv_DW.t1 = (0.5 * QuadModelv_rand_g3() + 0.75) * rtb_Product2_kl;

      // '<S101>:1:49' a=1/4+1/2*rand(1);
      QuadModelv_DW.a = 0.5 * QuadModelv_rand_g3() + 0.25;

      // '<S101>:1:50' ang=pi*2*rand(1);
      QuadModelv_DW.ang = 6.2831853071795862 * QuadModelv_rand_g3();

      // '<S101>:1:51' wlast=wNow;
      QuadModelv_DW.wlast = QuadModelv_DW.wNow;

      // '<S101>:1:52' wNow=1.2*(rand(1)*0.5+0.5)*vmax;
      QuadModelv_DW.wNow = (QuadModelv_rand_g3() * 0.5 + 0.5) * 1.2 *
        rtb_Product3_o;

      // '<S101>:1:54' isInGust=1;
      QuadModelv_DW.isInGust = 1.0;
    }

    // '<S101>:1:57' if ~isGustWind
    if (!QuadModelv_B.hasFault_GustWind) {
      // '<S101>:1:58' isInGust=0;
      QuadModelv_DW.isInGust = 0.0;
    }

    // '<S101>:1:61' gWind=[0;0;0];
    // '<S101>:1:63' if(t<t0+t1*a)
    rtb_sincos_o1_idx_1 = QuadModelv_DW.t1 * QuadModelv_DW.a;
    if (rtb_Clock1_tmp < rtb_sincos_o1_idx_1 + QuadModelv_DW.t0) {
      // '<S101>:1:64' gWindMag=(wNow-wlast)/2*(1-cos((t-t0)/(t1*a)*pi))+wlast;
      rtb_q3dot = (1.0 - std::cos((rtb_Clock1_tmp - QuadModelv_DW.t0) /
        rtb_sincos_o1_idx_1 * 3.1415926535897931)) * ((QuadModelv_DW.wNow -
        QuadModelv_DW.wlast) / 2.0) + QuadModelv_DW.wlast;
    } else {
      // '<S101>:1:65' else
      // '<S101>:1:66' gWindMag=wNow;
      rtb_q3dot = QuadModelv_DW.wNow;
    }

    // '<S101>:1:69' if abs(t-(t0+t1))<0.05
    if (std::abs(rtb_Clock1_tmp - (QuadModelv_DW.t0 + QuadModelv_DW.t1)) < 0.05)
    {
      // '<S101>:1:70' rng(t*1000);
      rtb_sincos_o1_idx_1 = rtb_Clock1_tmp * 1000.0;
      if (rtb_sincos_o1_idx_1 < 4.294967296E+9) {
        if (rtb_sincos_o1_idx_1 >= 0.0) {
          seed = static_cast<uint32_T>(rtb_sincos_o1_idx_1);
        } else {
          seed = 0U;
        }
      } else {
        seed = MAX_uint32_T;
      }

      switch (QuadModelv_DW.method_e) {
       case 7U:
        if (seed == 0U) {
          seed = 5489U;
        }

        QuadModelv_DW.state_iu[0] = seed;
        for (r = 0; r < 623; r++) {
          seed = ((seed >> 30U ^ seed) * 1812433253U + static_cast<uint32_T>(r))
            + 1U;
          QuadModelv_DW.state_iu[r + 1] = seed;
        }

        QuadModelv_DW.state_iu[624] = 624U;
        break;

       case 5U:
        QuadModelv_DW.state_km[0] = 362436069U;
        QuadModelv_DW.state_km[1] = seed;
        if (QuadModelv_DW.state_km[1] == 0U) {
          QuadModelv_DW.state_km[1] = 521288629U;
        }
        break;

       default:
        r = static_cast<int32_T>(seed >> 16U);
        j = static_cast<int32_T>(seed & 32768U);
        seed = ((((seed - (static_cast<uint32_T>(r) << 16U)) -
                  static_cast<uint32_T>(j)) << 16U) + static_cast<uint32_T>(j))
          + static_cast<uint32_T>(r);
        if (seed < 1U) {
          seed = 1144108930U;
        } else if (seed > 2147483646U) {
          seed = 2147483646U;
        }

        QuadModelv_DW.state_ny = seed;
        break;
      }

      // '<S101>:1:71' t0=t;
      QuadModelv_DW.t0 = rtb_Clock1_tmp;

      // '<S101>:1:72' t1=timeVel*(3/4+1/2*rand(1));
      QuadModelv_DW.t1 = (0.5 * QuadModelv_rand_g3() + 0.75) * rtb_Product2_kl;

      // '<S101>:1:73' a=1/4+1/2*rand(1);
      QuadModelv_DW.a = 0.5 * QuadModelv_rand_g3() + 0.25;

      // '<S101>:1:74' ang=ang+0.1*pi*rand(1);
      QuadModelv_DW.ang += 0.31415926535897931 * QuadModelv_rand_g3();

      // '<S101>:1:75' wlast=wNow;
      QuadModelv_DW.wlast = QuadModelv_DW.wNow;

      // '<S101>:1:76' wNow=1.2*(rand(1)*0.5+0.5)*vmax;
      QuadModelv_DW.wNow = (QuadModelv_rand_g3() * 0.5 + 0.5) * 1.2 *
        rtb_Product3_o;
    }

    // '<S101>:1:79' gWind(1)=gWindMag*cos(ang)*0.8+turb(1)*0.4;
    QuadModelv_B.gWind[0] = rtb_q3dot * std::cos(QuadModelv_DW.ang) * 0.8 +
      0.3048 * rtb_DataTypeConversion2[0] * 0.4;

    // '<S101>:1:80' gWind(2)=gWindMag*sin(ang)*0.8+turb(2)*0.4;
    QuadModelv_B.gWind[1] = rtb_q3dot * std::sin(QuadModelv_DW.ang) * 0.8 +
      0.3048 * rtb_DataTypeConversion2[1] * 0.4;

    // '<S101>:1:81' gWind(3)=turb(3)*0.4;
    QuadModelv_B.gWind[2] = 0.3048 * rtb_DataTypeConversion2[2] * 0.4;

    // End of MATLAB Function: '<S67>/MATLAB Function'

    // UnitConversion: '<S116>/Unit Conversion' incorporates:
    //   Constant: '<S67>/Constant_V'

    // Unit Conversion - from: m/s to: ft/s
    // Expression: output = (3.28084*input) + (0)
    QuadModelv_B.UnitConversion_a = 3.280839895013123 *
      QuadModelv_P.Constant_V_Value;
  }

  // UnitConversion: '<S110>/Unit Conversion'
  // Unit Conversion - from: m to: ft
  // Expression: output = (3.28084*input) + (0)
  rtb_Product2_kl = VectorConcatenate_tmp;

  // Saturate: '<S143>/Limit Function 10ft to 1000ft'
  if (rtb_Product2_kl > QuadModelv_P.LimitFunction10ftto1000ft_Upp_d) {
    rtb_TmpSignalConversionAtSFun_1 =
      QuadModelv_P.LimitFunction10ftto1000ft_Upp_d;
  } else if (rtb_Product2_kl < QuadModelv_P.LimitFunction10ftto1000ft_Low_j) {
    rtb_TmpSignalConversionAtSFun_1 =
      QuadModelv_P.LimitFunction10ftto1000ft_Low_j;
  } else {
    rtb_TmpSignalConversionAtSFun_1 = rtb_Product2_kl;
  }

  // End of Saturate: '<S143>/Limit Function 10ft to 1000ft'

  // Fcn: '<S143>/Low Altitude Scale Length'
  rtb_q3dot = 0.000823 * rtb_TmpSignalConversionAtSFun_1 + 0.177;
  if (rtb_q3dot < 0.0) {
    rtb_sincos_o1_idx_1 = -rt_powd_snf(-rtb_q3dot, 1.2);
  } else {
    rtb_sincos_o1_idx_1 = rt_powd_snf(rtb_q3dot, 1.2);
  }

  // Fcn: '<S143>/Low Altitude Scale Length'
  rtb_q2dot = rtb_TmpSignalConversionAtSFun_1 / rtb_sincos_o1_idx_1;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // UnitConversion: '<S145>/Unit Conversion' incorporates:
    //   Constant: '<S144>/Medium//High Altitude'

    // Unit Conversion - from: m to: ft
    // Expression: output = (3.28084*input) + (0)
    QuadModelv_B.UnitConversion_n = 3.280839895013123 *
      QuadModelv_P.DrydenWindTurbulenceModelDisc_m;
  }

  // Saturate: '<S126>/Limit Height h<1000ft'
  if (rtb_Product2_kl > QuadModelv_P.LimitHeighth1000ft_UpperSat_e) {
    rtb_sincos_o1_idx_1 = QuadModelv_P.LimitHeighth1000ft_UpperSat_e;
  } else if (rtb_Product2_kl < QuadModelv_P.LimitHeighth1000ft_LowerSat_o) {
    rtb_sincos_o1_idx_1 = QuadModelv_P.LimitHeighth1000ft_LowerSat_o;
  } else {
    rtb_sincos_o1_idx_1 = rtb_Product2_kl;
  }

  // Fcn: '<S126>/Low Altitude Intensity' incorporates:
  //   Saturate: '<S126>/Limit Height h<1000ft'

  rtb_q3dot = 0.000823 * rtb_sincos_o1_idx_1 + 0.177;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // Gain: '<S126>/sigma_wg ' incorporates:
    //   Constant: '<S94>/Windspeed at 20ft (6m)'
    //   UnitConversion: '<S117>/Unit Conversion'

    // Unit Conversion - from: m/s to: ft/s
    // Expression: output = (3.28084*input) + (0)
    QuadModelv_B.sigma_wg_p = 3.280839895013123 *
      QuadModelv_P.FaultParamAPI.FaultInParams[3] * QuadModelv_P.sigma_wg_Gain_o;
  }

  // Fcn: '<S126>/Low Altitude Intensity'
  if (rtb_q3dot < 0.0) {
    rtb_sincos_o1_idx_1 = -rt_powd_snf(-rtb_q3dot, 0.4);
  } else {
    rtb_sincos_o1_idx_1 = rt_powd_snf(rtb_q3dot, 0.4);
  }

  // Product: '<S126>/sigma_ug, sigma_vg' incorporates:
  //   Fcn: '<S126>/Low Altitude Intensity'

  X = 1.0 / rtb_sincos_o1_idx_1 * QuadModelv_B.sigma_wg_p;

  // Interpolation_n-D: '<S125>/Medium//High Altitude Intensity' incorporates:
  //   PreLookup: '<S125>/PreLook-Up Index Search  (altitude)'

  bpIndex_0[0] = plook_bincpa(rtb_Product2_kl,
    QuadModelv_P.PreLookUpIndexSearchaltitude__g, 11U, &rtb_q3dot,
    &QuadModelv_DW.PreLookUpIndexSearchaltitude__b);
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // PreLookup: '<S125>/PreLook-Up Index Search  (prob of exceed)' incorporates:
    //   Constant: '<S125>/Probability of  Exceedance'

    QuadModelv_B.PreLookUpIndexSearchprobofexc_n = plook_bincpa
      (QuadModelv_P.DrydenWindTurbulenceModelDis_gr,
       QuadModelv_P.PreLookUpIndexSearchprobofexc_n, 6U,
       &QuadModelv_B.PreLookUpIndexSearchprobofexc_i,
       &QuadModelv_DW.PreLookUpIndexSearchprobofexc_l);
  }

  // Interpolation_n-D: '<S125>/Medium//High Altitude Intensity'
  frac_0[0] = rtb_q3dot;
  frac_0[1] = QuadModelv_B.PreLookUpIndexSearchprobofexc_i;
  bpIndex_0[1] = QuadModelv_B.PreLookUpIndexSearchprobofexc_n;

  // Interpolation_n-D: '<S125>/Medium//High Altitude Intensity'
  rtb_Product3_o = intrp2d_la_pw(bpIndex_0, frac_0,
    QuadModelv_P.MediumHighAltitudeIntensity_T_b, 12U,
    QuadModelv_P.MediumHighAltitudeIntensity_m_i);
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0) {
    // Sqrt: '<S118>/Sqrt1' incorporates:
    //   Constant: '<S118>/Constant1'

    rtb_q3dot = std::sqrt(QuadModelv_P.WhiteNoise_Ts_a);

    // Product: '<S118>/Product' incorporates:
    //   Constant: '<S118>/Constant'
    //   Product: '<S118>/Divide'
    //   RandomNumber: '<S118>/White Noise'
    //   Sqrt: '<S118>/Sqrt'

    QuadModelv_B.Product_p[0] = std::sqrt(QuadModelv_P.WhiteNoise_pwr_d[0]) /
      rtb_q3dot * QuadModelv_DW.NextOutput_p[0];
    QuadModelv_B.Product_p[1] = std::sqrt(QuadModelv_P.WhiteNoise_pwr_d[1]) /
      rtb_q3dot * QuadModelv_DW.NextOutput_p[1];
    QuadModelv_B.Product_p[2] = std::sqrt(QuadModelv_P.WhiteNoise_pwr_d[2]) /
      rtb_q3dot * QuadModelv_DW.NextOutput_p[2];
    QuadModelv_B.Product_p[3] = std::sqrt(QuadModelv_P.WhiteNoise_pwr_d[3]) /
      rtb_q3dot * QuadModelv_DW.NextOutput_p[3];
  }

  // Outputs for Enabled SubSystem: '<S109>/Hugw(z)'
  // Constant: '<S109>/Constant3'
  QuadModelv_Hugwz(QuadModelv_P.DrydenWindTurbulenceModelDisc_k,
                   QuadModelv_B.UnitConversion_a, rtb_q2dot,
                   QuadModelv_B.UnitConversion_n, X, rtb_Product3_o,
                   QuadModelv_B.Product_p[0], &QuadModelv_B.Hugwz,
                   &QuadModelv_DW.Hugwz, &QuadModelv_P.Hugwz);

  // End of Outputs for SubSystem: '<S109>/Hugw(z)'

  // Gain: '<S115>/Lv'
  frac_0[0] = QuadModelv_P.Lv_Gain_o * rtb_q2dot;
  frac_0[1] = QuadModelv_P.Lv_Gain_o * QuadModelv_B.UnitConversion_n;

  // Outputs for Enabled SubSystem: '<S109>/Hvgw(z)'
  // Constant: '<S109>/Constant3'
  QuadModelv_Hvgwz(QuadModelv_P.DrydenWindTurbulenceModelDisc_k, X,
                   rtb_Product3_o, frac_0, QuadModelv_B.UnitConversion_a,
                   QuadModelv_B.Product_p[1], &QuadModelv_B.Hvgwz,
                   &QuadModelv_DW.Hvgwz, &QuadModelv_P.Hvgwz);

  // End of Outputs for SubSystem: '<S109>/Hvgw(z)'

  // Gain: '<S115>/Lw'
  frac_0[0] = QuadModelv_P.Lw_Gain_c * rtb_TmpSignalConversionAtSFun_1;
  frac_0[1] = QuadModelv_P.Lw_Gain_c * QuadModelv_B.UnitConversion_n;

  // Outputs for Enabled SubSystem: '<S109>/Hwgw(z)'
  // Constant: '<S109>/Constant3'
  QuadModelv_Hwgwz(QuadModelv_P.DrydenWindTurbulenceModelDisc_k,
                   QuadModelv_B.UnitConversion_a, frac_0,
                   QuadModelv_B.sigma_wg_p, rtb_Product3_o,
                   QuadModelv_B.Product_p[2], &QuadModelv_B.Hwgwz,
                   &QuadModelv_DW.Hwgwz, &QuadModelv_P.Hwgwz);

  // End of Outputs for SubSystem: '<S109>/Hwgw(z)'
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // UnitConversion: '<S107>/Unit Conversion' incorporates:
    //   Constant: '<S94>/Wind direction'

    // Unit Conversion - from: deg to: rad
    // Expression: output = (0.0174533*input) + (0)
    QuadModelv_B.UnitConversion_ab = 0.017453292519943295 *
      QuadModelv_P.FaultParamAPI.FaultInParams[4];
  }

  // If: '<S114>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' incorporates:
  //   Constant: '<S67>/Constant_DCM'

  if (rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    if (rtb_Product2_kl <= 1000.0) {
      rtAction = 0;
    } else if (rtb_Product2_kl >= 2000.0) {
      rtAction = 1;
    } else {
      rtAction = 2;
    }

    QuadModelv_DW.ifHeightMaxlowaltitudeelseifH_p = rtAction;
  } else {
    rtAction = QuadModelv_DW.ifHeightMaxlowaltitudeelseifH_p;
  }

  switch (rtAction) {
   case 0:
    // Outputs for IfAction SubSystem: '<S114>/Low altitude  velocities' incorporates:
    //   ActionPort: '<S136>/Action Port'

    QuadModelv_Lowaltituderates(QuadModelv_P.Constant_DCM_Value,
      QuadModelv_B.Hugwz.Sum, QuadModelv_B.Hvgwz.Sum, QuadModelv_B.Hwgwz.Sum,
      QuadModelv_B.UnitConversion_ab, rtb_DataTypeConversion2);

    // End of Outputs for SubSystem: '<S114>/Low altitude  velocities'
    break;

   case 1:
    // Outputs for IfAction SubSystem: '<S114>/Medium//High  altitude velocities' incorporates:
    //   ActionPort: '<S137>/Action Port'

    // DataTypeConversion: '<S8>/Data Type Conversion2' incorporates:
    //   Gain: '<S137>/Gain'

    rtb_DataTypeConversion2[0] = QuadModelv_P.Gain_Gain_pq *
      QuadModelv_B.Hugwz.Sum[1];
    rtb_DataTypeConversion2[1] = QuadModelv_P.Gain_Gain_pq *
      QuadModelv_B.Hvgwz.Sum[1];
    rtb_DataTypeConversion2[2] = QuadModelv_P.Gain_Gain_pq *
      QuadModelv_B.Hwgwz.Sum[1];

    // End of Outputs for SubSystem: '<S114>/Medium//High  altitude velocities'
    break;

   default:
    // Outputs for IfAction SubSystem: '<S114>/Interpolate  velocities' incorporates:
    //   ActionPort: '<S135>/Action Port'

    QuadModel_Interpolatevelocities(QuadModelv_B.Hugwz.Sum,
      QuadModelv_B.Hvgwz.Sum, QuadModelv_B.Hwgwz.Sum,
      QuadModelv_P.Constant_DCM_Value, QuadModelv_B.UnitConversion_ab,
      rtb_Product2_kl, rtb_DataTypeConversion2,
      &QuadModelv_P.Interpolatevelocities);

    // End of Outputs for SubSystem: '<S114>/Interpolate  velocities'
    break;
  }

  // End of If: '<S114>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' 

  // UnitConversion: '<S94>/Unit Conversion'
  // Unit Conversion - from: ft/s to: m/s
  // Expression: output = (0.3048*input) + (0)
  Product1[0] = 0.3048 * rtb_DataTypeConversion2[0];
  Product1[1] = 0.3048 * rtb_DataTypeConversion2[1];
  Product1[2] = 0.3048 * rtb_DataTypeConversion2[2];

  // MATLAB Function 'WindFault/TurbWindStrength_Dec_Switch': '<S104>:1'
  // '<S104>:1:3' TurbWind=vwind*TurbWindParams(1);
  // Unit Conversion - from: m to: ft
  // Expression: output = (3.28084*input) + (0)
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // Math: '<S105>/ln(ref_height//z0)' incorporates:
    //   Constant: '<S105>/ref_height//z0'
    //
    //  About '<S105>/ln(ref_height//z0)':
    //   Operator: log

    QuadModelv_B.lnref_heightz0 = std::log(QuadModelv_P.ref_heightz0_Value);
  }

  // Saturate: '<S105>/3ft-->inf'
  if (VectorConcatenate_tmp > QuadModelv_P.uftinf_UpperSat) {
    rtb_sincos_o1_idx_1 = QuadModelv_P.uftinf_UpperSat;
  } else if (VectorConcatenate_tmp < QuadModelv_P.uftinf_LowerSat) {
    rtb_sincos_o1_idx_1 = QuadModelv_P.uftinf_LowerSat;
  } else {
    rtb_sincos_o1_idx_1 = VectorConcatenate_tmp;
  }

  // Product: '<S105>/Product' incorporates:
  //   Gain: '<S105>/h//z0'
  //   Math: '<S105>/ln(h//z0)'
  //   Saturate: '<S105>/3ft-->inf'
  //
  //  About '<S105>/ln(h//z0)':
  //   Operator: log

  rtb_q3dot = std::log(QuadModelv_P.hz0_Gain * rtb_sincos_o1_idx_1) /
    QuadModelv_B.lnref_heightz0;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // UnitConversion: '<S185>/Unit Conversion' incorporates:
    //   Constant: '<S105>/Wind Direction'

    // Unit Conversion - from: deg to: rad
    // Expression: output = (0.0174533*input) + (0)
    uavlostJyz = 0.017453292519943295 *
      QuadModelv_P.FaultParamAPI.FaultInParams[6];

    // Gain: '<S105>/Wind speed at reference height' incorporates:
    //   Constant: '<S105>/Wdeg1'
    //   Trigonometry: '<S105>/SinCos'

    QuadModelv_B.Windspeedatreferenceheight[0] =
      -QuadModelv_P.FaultParamAPI.FaultInParams[5] * std::cos(uavlostJyz);
    QuadModelv_B.Windspeedatreferenceheight[1] =
      -QuadModelv_P.FaultParamAPI.FaultInParams[5] * std::sin(uavlostJyz);
    QuadModelv_B.Windspeedatreferenceheight[2] =
      -QuadModelv_P.FaultParamAPI.FaultInParams[5] * QuadModelv_P.Wdeg1_Value;
  }

  // DataTypeConversion: '<S8>/Data Type Conversion2' incorporates:
  //   Product: '<S105>/Product1'

  // MATLAB Function 'WindFault/SheerWindStrength_Dec_Switch': '<S103>:1'
  // '<S103>:1:2' SheerWind=vwind*SheerWindParams(1);
  // MATLAB Function 'WindFault/MATLAB Function1': '<S102>:1'
  // '<S102>:1:2' wind=[0;0;0];
  rtb_DataTypeConversion2[0] = rtb_q3dot *
    QuadModelv_B.Windspeedatreferenceheight[0];

  // MATLAB Function: '<S67>/MATLAB Function1'
  rtb_wind[0] = 0.0;

  // DataTypeConversion: '<S8>/Data Type Conversion2' incorporates:
  //   Product: '<S105>/Product1'

  rtb_DataTypeConversion2[1] = rtb_q3dot *
    QuadModelv_B.Windspeedatreferenceheight[1];

  // MATLAB Function: '<S67>/MATLAB Function1'
  rtb_wind[1] = 0.0;

  // DataTypeConversion: '<S8>/Data Type Conversion2' incorporates:
  //   Product: '<S105>/Product1'

  rtb_DataTypeConversion2[2] = rtb_q3dot *
    QuadModelv_B.Windspeedatreferenceheight[2];

  // MATLAB Function: '<S67>/MATLAB Function1' incorporates:
  //   MATLAB Function: '<S67>/SheerWindStrength_Dec_Switch'
  //   MATLAB Function: '<S67>/TurbWindStrength_Dec_Switch'
  //   Product: '<S105>/Transform from Inertial to Body axes'

  rtb_wind[2] = 0.0;

  // '<S102>:1:3' if isConstWind
  if (QuadModelv_B.hasFault_ConstWind) {
    // '<S102>:1:4' wind=[ConstWind(1);ConstWind(2);ConstWind(3)];
    rtb_wind[0] = QuadModelv_B.FaultParam_p[0];
    rtb_wind[1] = QuadModelv_B.FaultParam_p[1];
    rtb_wind[2] = QuadModelv_B.FaultParam_p[2];
  } else if (QuadModelv_B.hasFault_GustWind) {
    // '<S102>:1:5' elseif isGustWind
    // '<S102>:1:6' wind=GustWind;
    rtb_wind[0] = QuadModelv_B.gWind[0];
    rtb_wind[1] = QuadModelv_B.gWind[1];
    rtb_wind[2] = QuadModelv_B.gWind[2];
  } else if (QuadModelv_B.hasFault_TurbWind) {
    // '<S102>:1:7' elseif isTurbWind
    // '<S102>:1:8' wind=TurbWind;
    rtb_wind[0] = Product1[0] * QuadModelv_B.FaultParam_i[0];
    rtb_wind[1] = QuadModelv_B.FaultParam_i[0] * Product1[1];
    rtb_wind[2] = QuadModelv_B.FaultParam_i[0] * Product1[2];
  } else if (QuadModelv_B.hasFault_SheerWind) {
    // Product: '<S105>/Transform from Inertial to Body axes' incorporates:
    //   Constant: '<S67>/Constant_DCM'
    //   DataTypeConversion: '<S8>/Data Type Conversion2'

    // '<S102>:1:9' elseif isSheerWind
    // '<S102>:1:10' wind=SheerWind;
    VectorConcatenate_tmp_0 = rtb_DataTypeConversion2[1];
    rtb_TmpSignalConversionAtSFun_1 = rtb_DataTypeConversion2[0];
    rtb_Rn = rtb_DataTypeConversion2[2];
    for (i = 0; i < 3; i++) {
      VectorConcatenate_0[i] = (QuadModelv_P.Constant_DCM_Value[i + 3] *
        VectorConcatenate_tmp_0 + QuadModelv_P.Constant_DCM_Value[i] *
        rtb_TmpSignalConversionAtSFun_1) + QuadModelv_P.Constant_DCM_Value[i + 6]
        * rtb_Rn;
    }

    rtb_wind[0] = VectorConcatenate_0[0] * QuadModelv_B.FaultParam_a[0];
    rtb_wind[1] = QuadModelv_B.FaultParam_a[0] * VectorConcatenate_0[1];
    rtb_wind[2] = QuadModelv_B.FaultParam_a[0] * VectorConcatenate_0[2];
  }

  // Product: '<S67>/Product'
  rtb_TmpSignalConversionAtSFun_1 = rtb_wind[1];
  rtb_Rn = rtb_wind[0];
  rtb_q3dot = rtb_wind[2];
  for (i = 0; i < 3; i++) {
    // DataTypeConversion: '<S8>/Data Type Conversion2' incorporates:
    //   Concatenate: '<S36>/Vector Concatenate'
    //   Product: '<S67>/Product'

    VectorConcatenate_tmp_0 = (VectorConcatenate[i + 3] *
      rtb_TmpSignalConversionAtSFun_1 + VectorConcatenate[i] * rtb_Rn) +
      VectorConcatenate[i + 6] * rtb_q3dot;
    rtb_DataTypeConversion2[i] = VectorConcatenate_tmp_0;

    // Product: '<S67>/Product1' incorporates:
    //   DataTypeConversion: '<S8>/Data Type Conversion2'
    //   Integrator: '<S14>/ub,vb,wb'
    //   Sum: '<S67>/Sum'
    //   Sum: '<S67>/Sum1'

    rtb_sincos_o1_f[i] = (QuadModelv_X.ubvbwb_CSTATE[i] -
                          VectorConcatenate_tmp_0) * QuadModelv_B.Sum1[i];
  }

  // MATLAB Function: '<S7>/Propeller Model old Version' incorporates:
  //   Constant: '<S6>/ModelParam.uavR'
  //   Constant: '<S7>/ModelParam.motorJm'
  //   Constant: '<S7>/ModelParam.rotorCm'
  //   Constant: '<S7>/ModelParam.rotorCt'
  //   Constant: '<S7>/ModelParam.uavType'
  //   Integrator: '<S14>/p,q,r '

  rtAction = QuadModelv_P.ModelParam_uavType;

  //  This is control effectiveness model, where Fb is the force produced by motors, Mb is 
  //  the moment produced by motors, and Fd is the resistance produced by the air. 
  //  The gyroscopic torque is not considered here.
  // MATLAB Function 'Force and Moment Model/Propeller Model old Version': '<S198>:1' 
  //  -1: anticlockwise, 1: clockwise
  // '<S198>:1:7' TypeMotorDirMatrix=[...
  // '<S198>:1:8'     [-1 -1 -1 0 0 0 0 0];... % Tri 3*1 X
  // '<S198>:1:9'     [1 1 1 0 0 0 0 0];...    % Tri 3*1 +
  // '<S198>:1:10'     [-1 -1 1 1 0 0 0 0];... % Quad 4*1 X
  // '<S198>:1:11'     [-1 -1 1 1 0 0 0 0];... % Quad 4*1 +
  // '<S198>:1:12'     [1 -1 1 -1 -1 1 0 0];... % Hex 6*1 X
  // '<S198>:1:13'     [1 -1 1 -1 -1 1 0 0];... % Hex 6*1 +
  // '<S198>:1:14'     [1 -1 1 -1 1 -1 0 0];... % HexCoa 3*2
  // '<S198>:1:15'     [1 1 -1 -1 -1 -1 1 1];... % Oct 8*1 X
  // '<S198>:1:16'     [1 1 -1 -1 -1 -1 1 1];... % Oct 8*1 +
  // '<S198>:1:17'     [1 -1 1 -1 1 -1 1 -1];... % OctCoa 4*2 X
  // '<S198>:1:18' ];
  //  % Tri 3*1 X
  //     % Tri 3*1 +
  //  % Quad 4*1 X
  //  % Quad 4*1 +
  //  % Hex 6*1 X
  //  % Hex 6*1 +
  //  % HexCoa 3*2
  //  % Oct 8*1 X
  //  % Oct 8*1 +
  //  % OctCoa 4*2 X
  // '<S198>:1:20' TypeMotorAngMatrix=[...
  // '<S198>:1:21'     [pi/3 pi -pi/3 0 0 0 0 0];...
  // '<S198>:1:22'     [pi/3 pi -pi/3 0 0 0 0 0];...
  // '<S198>:1:23'     [pi/4 pi/4+pi pi/4+3*pi/2 pi/4+pi/2 0 0 0 0];...
  // '<S198>:1:24'     [pi/2 3*pi/2 0 pi 0 0 0 0];...
  // '<S198>:1:25'     [pi/2 3/2*pi 3/2*pi+pi/3 pi/2+pi/3 pi/2-pi/3 3/2*pi-pi/3 0 0];... 
  // '<S198>:1:26'     [0 pi pi+pi/3 pi/3 -pi/3 pi-pi/3 0 0];...
  // '<S198>:1:27'     [pi/3 pi/3 pi pi -pi/3 -pi/3 0 0];...
  // '<S198>:1:28'     [pi/8 pi+pi/8 pi/8+pi/4 pi-pi/8 -pi/8 3/2*pi-pi/8 3/2*pi+pi/8 pi/2+pi/8];... 
  // '<S198>:1:29'     [0 pi pi/4 3/4*pi -pi/4 pi+pi/4 3/2*pi pi/2];...
  // '<S198>:1:30'     [pi/4 -pi/4 pi+pi/4 pi-pi/4 -pi/4 pi/4 pi-pi/4 pi+pi/4];... 
  // '<S198>:1:31' ];
  // '<S198>:1:33' TypeMotorNonVector=int8([3;3;4;4;6;6;6;8;8;8]);
  // '<S198>:1:35' Mb = [0;0;0];
  // '<S198>:1:36' Fb = [0;0;0];
  rtb_Mb[0] = 0.0;
  Product1[0] = 0.0;
  rtb_Mb[1] = 0.0;
  rtb_Mb[2] = 0.0;
  Product1[2] = 0.0;

  // '<S198>:1:37' P=wb(1);
  // '<S198>:1:37' Q=wb(2);
  // '<S198>:1:39' if uavType==50
  if (QuadModelv_P.ModelParam_uavType == 50) {
    // '<S198>:1:41' Fb(1)=MotorRads(1)+MotorRads(3);
    Product1[0] = QuadModelv_B.y[0] + QuadModelv_B.y[2];

    // '<S198>:1:42' Mb(3)=MotorRads(1)*0.1-MotorRads(3)*0.1;
    rtb_Mb[2] = QuadModelv_B.y[0] * 0.1 - QuadModelv_B.y[2] * 0.1;
  } else {
    // '<S198>:1:48' if uavType<0||uavType>length(TypeMotorNonVector)
    if ((QuadModelv_P.ModelParam_uavType < 0) ||
        (QuadModelv_P.ModelParam_uavType > 10)) {
      // '<S198>:1:49' uavType=int8(1);
      rtAction = 1;
    }

    // '<S198>:1:52' TypeMotorDirVector=TypeMotorDirMatrix(uavType,:);
    // '<S198>:1:53' TypeMotorAngVector=TypeMotorAngMatrix(uavType,:);
    // '<S198>:1:55' for i=1:TypeMotorNonVector(uavType)
    j = d[rtAction - 1];
    for (r = 0; r < j; r++) {
      // '<S198>:1:56' propAng = TypeMotorAngVector(i);
      s458_iter = (10 * r + rtAction) - 1;
      rtb_q3dot = e[s458_iter];

      // '<S198>:1:57' propdir = TypeMotorDirVector(i);
      // '<S198>:1:58' PropT = Ct*MotorRads(i)*MotorRads(i);
      rtb_TmpSignalConversionAtSFun_1 = QuadModelv_B.y[r];
      rtb_Rn = QuadModelv_P.ModelParam_rotorCt * rtb_TmpSignalConversionAtSFun_1
        * rtb_TmpSignalConversionAtSFun_1;

      // '<S198>:1:59' PropM = Cm*MotorRads(i)*MotorRads(i);
      // Thrust -> Moment
      // '<S198>:1:62' Mb=Mb+[-R*sin(propAng)*PropT;R*cos(propAng)*PropT;-PropM*propdir]; 
      c = c_0[s458_iter];
      rtb_Mb[2] += -(QuadModelv_P.ModelParam_rotorCm *
                     rtb_TmpSignalConversionAtSFun_1 *
                     rtb_TmpSignalConversionAtSFun_1) * static_cast<real_T>(c);

      // gyroscopic moment
      // '<S198>:1:65' Mb=Mb+[Q*Jm*MotorRads(i)*propdir;-P*Jm*MotorRads(i)*propdir;0]; 
      rtb_Mb[0] = QuadModelv_X.pqr_CSTATE[1] * QuadModelv_P.ModelParam_motorJm *
        rtb_TmpSignalConversionAtSFun_1 * static_cast<real_T>(c) +
        (-QuadModelv_P.ModelParam_uavR * std::sin(rtb_q3dot) * rtb_Rn + rtb_Mb[0]);
      rtb_Mb[1] = -QuadModelv_X.pqr_CSTATE[0] * QuadModelv_P.ModelParam_motorJm *
        rtb_TmpSignalConversionAtSFun_1 * static_cast<real_T>(c) +
        (QuadModelv_P.ModelParam_uavR * std::cos(rtb_q3dot) * rtb_Rn + rtb_Mb[1]);

      // '<S198>:1:67' Fb=Fb+[0;0;-PropT];
      rtb_TmpSignalConversionAtSFun_1 = Product1[2];
      Product1[0] = 0.0;
      Product1[2] = rtb_TmpSignalConversionAtSFun_1 - rtb_Rn;
    }
  }

  // End of MATLAB Function: '<S7>/Propeller Model old Version'

  // MATLAB Function: '<S7>/Body AeroCenter' incorporates:
  //   Constant: '<S7>/ModelParam.uavCd'
  //   Product: '<S67>/Product1'

  //  This is control effectiveness model, where Fb is the force produced by motors, Mb is 
  //  the moment produced by motors, and Fd is the resistance produced by the air. 
  //  The gyroscopic torque is not considered here.
  //  uavDearo 机身重心位置，向下为正
  // MATLAB Function 'Force and Moment Model/Body AeroCenter': '<S192>:1'
  // '<S192>:1:7' Fd = -Cd*Vb.*abs(Vb);
  //
  // '<S192>:1:9' Mfd=[Fd(2)*uavDearo;-Fd(1)*uavDearo;0];
  // '<S192>:1:11' Md=-CCm.*wb.*abs(wb);
  // '<S192>:1:12' Md=Mfd+Md;
  //   CCm=4.72e-3;
  //   Md=-CCm*wb.*abs(wb)*0.5;%0.3
  //   Md= Mfd+Md;
  U1 = -QuadModelv_P.ModelParam_uavCd * rtb_sincos_o1_f[0] * std::abs
    (rtb_sincos_o1_f[0]);
  rtb_Rn = U1;

  // Sum: '<S7>/Add2' incorporates:
  //   Sum: '<S1>/Sum1'

  VectorConcatenate_tmp_0 = (rtb_Sum1_ki[0] + Product1[0]) + U1;
  rtb_Sum1_ki[0] = VectorConcatenate_tmp_0;

  // Product: '<S12>/Divide' incorporates:
  //   DataTypeConversion: '<S8>/Data Type Conversion1'
  //   Sum: '<S1>/Sum1'

  rtb_DataTypeConversion1[0] = VectorConcatenate_tmp_0 / QuadModelv_B.m;

  // MATLAB Function: '<S7>/Body AeroCenter' incorporates:
  //   Constant: '<S7>/ModelParam.uavCd'
  //   Product: '<S67>/Product1'

  U1 = -QuadModelv_P.ModelParam_uavCd * rtb_sincos_o1_f[1] * std::abs
    (rtb_sincos_o1_f[1]);

  // Sum: '<S7>/Add2' incorporates:
  //   Sum: '<S1>/Sum1'

  VectorConcatenate_tmp_0 = rtb_Sum1_ki[1] + U1;
  rtb_Sum1_ki[1] = VectorConcatenate_tmp_0;

  // Product: '<S12>/Divide' incorporates:
  //   DataTypeConversion: '<S8>/Data Type Conversion1'
  //   Sum: '<S1>/Sum1'

  rtb_DataTypeConversion1[1] = VectorConcatenate_tmp_0 / QuadModelv_B.m;

  // Sum: '<S7>/Add2' incorporates:
  //   Constant: '<S7>/ModelParam.uavCd'
  //   MATLAB Function: '<S7>/Body AeroCenter'
  //   Product: '<S67>/Product1'
  //   Sum: '<S1>/Sum1'

  VectorConcatenate_tmp_0 = -QuadModelv_P.ModelParam_uavCd * rtb_sincos_o1_f[2] *
    std::abs(rtb_sincos_o1_f[2]) + (rtb_Sum1_ki[2] + Product1[2]);

  // Product: '<S12>/Divide' incorporates:
  //   DataTypeConversion: '<S8>/Data Type Conversion1'
  //   Sum: '<S1>/Sum1'

  rtb_DataTypeConversion1[2] = VectorConcatenate_tmp_0 / QuadModelv_B.m;

  // Gain: '<S415>/Zero-Order Hold' incorporates:
  //   Integrator: '<S14>/p,q,r '

  rtb_wind[0] = QuadModelv_P.ZeroOrderHold_Gain * QuadModelv_X.pqr_CSTATE[0];
  rtb_wind[1] = QuadModelv_P.ZeroOrderHold_Gain * QuadModelv_X.pqr_CSTATE[1];
  rtb_wind[2] = QuadModelv_P.ZeroOrderHold_Gain * QuadModelv_X.pqr_CSTATE[2];
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // Gain: '<S415>/Gain' incorporates:
    //   Constant: '<S398>/center of gravity'
    //   Constant: '<S415>/wl_ins'
    //   Gain: '<S415>/Zero-Order Hold4'
    //   Sum: '<S415>/Sum7'

    QuadModelv_B.Gain_f[0] = (QuadModelv_P.ZeroOrderHold4_Gain *
      QuadModelv_P.centerofgravity_Value[0] -
      QuadModelv_P.ThreeaxisInertialMeasurementU_e[0]) *
      QuadModelv_P.Gain_Gain_hr[0];
    QuadModelv_B.Gain_f[1] = (QuadModelv_P.ZeroOrderHold4_Gain *
      QuadModelv_P.centerofgravity_Value[1] -
      QuadModelv_P.ThreeaxisInertialMeasurementU_e[1]) *
      QuadModelv_P.Gain_Gain_hr[1];
    QuadModelv_B.Gain_f[2] = (QuadModelv_P.ZeroOrderHold4_Gain *
      QuadModelv_P.centerofgravity_Value[2] -
      QuadModelv_P.ThreeaxisInertialMeasurementU_e[2]) *
      QuadModelv_P.Gain_Gain_hr[2];

    // Concatenate: '<S17>/Matrix Concatenation' incorporates:
    //   Constant: '<S1>/Constant_dI//dt'
    //   Constant: '<S6>/uavJ'
    //   Sum: '<S6>/Add6'

    for (i = 0; i < 3; i++) {
      rtb_MatrixConcatenation[6 * i] = QuadModelv_P.ModelParam_uavJ[3 * i] -
        rtb_uavlostJ[3 * i];

      // Sum: '<S6>/Add6' incorporates:
      //   Concatenate: '<S17>/Matrix Concatenation'
      //   Constant: '<S6>/uavJ'

      j = 3 * i + 1;
      rtb_MatrixConcatenation[6 * i + 1] = QuadModelv_P.ModelParam_uavJ[j] -
        rtb_uavlostJ[j];

      // Sum: '<S6>/Add6' incorporates:
      //   Concatenate: '<S17>/Matrix Concatenation'
      //   Constant: '<S6>/uavJ'

      j = 3 * i + 2;
      rtb_MatrixConcatenation[6 * i + 2] = QuadModelv_P.ModelParam_uavJ[j] -
        rtb_uavlostJ[j];
    }

    for (i = 0; i < 3; i++) {
      rtb_MatrixConcatenation[6 * i + 3] = QuadModelv_P.Constant_dIdt_Value[3 *
        i];
      rtb_MatrixConcatenation[6 * i + 4] = QuadModelv_P.Constant_dIdt_Value[3 *
        i + 1];
      rtb_MatrixConcatenation[6 * i + 5] = QuadModelv_P.Constant_dIdt_Value[3 *
        i + 2];
    }

    // End of Concatenate: '<S17>/Matrix Concatenation'
    for (i = 0; i < 3; i++) {
      // Selector: '<S16>/Selector1' incorporates:
      //   Concatenate: '<S17>/Matrix Concatenation'

      QuadModelv_B.Selector1[3 * i] = rtb_MatrixConcatenation[6 * i + 3];

      // Selector: '<S16>/Selector' incorporates:
      //   Concatenate: '<S17>/Matrix Concatenation'

      QuadModelv_B.Selector[3 * i] = rtb_MatrixConcatenation[6 * i];

      // Selector: '<S16>/Selector1' incorporates:
      //   Selector: '<S16>/Selector'

      j = 3 * i + 1;

      // Selector: '<S16>/Selector1' incorporates:
      //   Concatenate: '<S17>/Matrix Concatenation'

      QuadModelv_B.Selector1[j] = rtb_MatrixConcatenation[6 * i + 4];

      // Selector: '<S16>/Selector' incorporates:
      //   Concatenate: '<S17>/Matrix Concatenation'

      QuadModelv_B.Selector[j] = rtb_MatrixConcatenation[6 * i + 1];

      // Selector: '<S16>/Selector1' incorporates:
      //   Selector: '<S16>/Selector'

      j = 3 * i + 2;

      // Selector: '<S16>/Selector1' incorporates:
      //   Concatenate: '<S17>/Matrix Concatenation'

      QuadModelv_B.Selector1[j] = rtb_MatrixConcatenation[6 * i + 5];

      // Selector: '<S16>/Selector' incorporates:
      //   Concatenate: '<S17>/Matrix Concatenation'

      QuadModelv_B.Selector[j] = rtb_MatrixConcatenation[6 * i + 2];
    }
  }

  // Product: '<S427>/j x i'
  rtb_TmpSignalConversionAtSFun_1 = QuadModelv_B.Gain_f[0] * rtb_wind[1];

  // Sum: '<S423>/Sum' incorporates:
  //   Product: '<S426>/i x j'
  //   Product: '<S426>/j x k'
  //   Product: '<S426>/k x i'
  //   Product: '<S427>/i x k'
  //   Product: '<S427>/k x j'

  Product1[0] = rtb_wind[1] * QuadModelv_B.Gain_f[2] - QuadModelv_B.Gain_f[1] *
    rtb_wind[2];
  Product1[1] = QuadModelv_B.Gain_f[0] * rtb_wind[2] - rtb_wind[0] *
    QuadModelv_B.Gain_f[2];
  Product1[2] = rtb_wind[0] * QuadModelv_B.Gain_f[1] -
    rtb_TmpSignalConversionAtSFun_1;

  // Sum: '<S422>/Sum' incorporates:
  //   Product: '<S424>/i x j'
  //   Product: '<S424>/j x k'
  //   Product: '<S424>/k x i'
  //   Product: '<S425>/i x k'
  //   Product: '<S425>/j x i'
  //   Product: '<S425>/k x j'

  rtb_q3dot = Product1[0] * rtb_wind[1];
  rtb_q2dot = rtb_wind[1] * Product1[2];
  X = Product1[0] * rtb_wind[2];
  uavlostJyz = rtb_wind[0] * Product1[1];
  rtb_Product1_d = Product1[1] * rtb_wind[2];
  rtb_sincos_o1_idx_1 = rtb_wind[0] * Product1[2];

  // Integrator: '<S14>/p,q,r ' incorporates:
  //   Product: '<S49>/Product'

  tz_idx_0 = QuadModelv_X.pqr_CSTATE[1];
  tz_idx_3 = QuadModelv_X.pqr_CSTATE[0];
  tz_idx_1 = QuadModelv_X.pqr_CSTATE[2];

  // Product: '<S49>/Product'
  for (i = 0; i < 3; i++) {
    // Sum: '<S415>/Sum4' incorporates:
    //   Integrator: '<S14>/p,q,r '
    //   Selector: '<S16>/Selector'

    rtb_wind[i] = (QuadModelv_B.Selector[i + 3] * tz_idx_0 +
                   QuadModelv_B.Selector[i] * tz_idx_3) +
      QuadModelv_B.Selector[i + 6] * tz_idx_1;
  }

  // Product: '<S52>/j x i' incorporates:
  //   Integrator: '<S14>/p,q,r '

  rtb_TmpSignalConversionAtSFun_1 = rtb_wind[0] * QuadModelv_X.pqr_CSTATE[1];
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    for (i = 0; i < 3; i++) {
      // Selector: '<S16>/Selector2' incorporates:
      //   Concatenate: '<S17>/Matrix Concatenation'

      QuadModelv_B.Selector2[3 * i] = rtb_MatrixConcatenation[6 * i];
      QuadModelv_B.Selector2[3 * i + 1] = rtb_MatrixConcatenation[6 * i + 1];
      QuadModelv_B.Selector2[3 * i + 2] = rtb_MatrixConcatenation[6 * i + 2];
    }
  }

  // Integrator: '<S14>/p,q,r ' incorporates:
  //   Product: '<S50>/Product'

  tz_idx_0 = QuadModelv_X.pqr_CSTATE[1];
  tz_idx_3 = QuadModelv_X.pqr_CSTATE[0];
  tz_idx_1 = QuadModelv_X.pqr_CSTATE[2];

  // Product: '<S50>/Product' incorporates:
  //   Integrator: '<S14>/p,q,r '
  //   Selector: '<S16>/Selector1'

  for (i = 0; i < 3; i++) {
    VectorConcatenate_0[i] = (QuadModelv_B.Selector1[i + 3] * tz_idx_0 +
      QuadModelv_B.Selector1[i] * tz_idx_3) + QuadModelv_B.Selector1[i + 6] *
      tz_idx_1;
  }

  // Sum: '<S16>/Sum2' incorporates:
  //   Constant: '<S7>/ModelParam.uavCCm'
  //   Constant: '<S7>/ModelParam.uavDearo'
  //   Integrator: '<S14>/p,q,r '
  //   MATLAB Function: '<S7>/Body AeroCenter'
  //   Product: '<S50>/Product'
  //   Product: '<S51>/i x j'
  //   Product: '<S51>/j x k'
  //   Product: '<S51>/k x i'
  //   Product: '<S52>/i x k'
  //   Product: '<S52>/k x j'
  //   Sum: '<S1>/Sum'
  //   Sum: '<S48>/Sum'
  //   Sum: '<S7>/Add3'

  tmp[0] = (((((-QuadModelv_P.ModelParam_uavCCm[0] * QuadModelv_X.pqr_CSTATE[0] *
                std::abs(QuadModelv_X.pqr_CSTATE[0]) + U1 *
                QuadModelv_P.ModelParam_uavDearo) + (QuadModelv_B.M1[0] +
    rtb_Mb[0])) + QuadModelv_B.deltaM[0]) + QuadModelv_B.fm[3]) -
            VectorConcatenate_0[0]) - (QuadModelv_X.pqr_CSTATE[1] * rtb_wind[2]
    - rtb_wind[1] * QuadModelv_X.pqr_CSTATE[2]);
  tmp[1] = (((((-QuadModelv_P.ModelParam_uavCCm[1] * QuadModelv_X.pqr_CSTATE[1] *
                std::abs(QuadModelv_X.pqr_CSTATE[1]) + -rtb_Rn *
                QuadModelv_P.ModelParam_uavDearo) + (QuadModelv_B.M1[1] +
    rtb_Mb[1])) + QuadModelv_B.deltaM[1]) + QuadModelv_B.fm[4]) -
            VectorConcatenate_0[1]) - (rtb_wind[0] * QuadModelv_X.pqr_CSTATE[2]
    - QuadModelv_X.pqr_CSTATE[0] * rtb_wind[2]);
  tmp[2] = ((((-QuadModelv_P.ModelParam_uavCCm[2] * QuadModelv_X.pqr_CSTATE[2] *
               std::abs(QuadModelv_X.pqr_CSTATE[2]) + (QuadModelv_B.M1[2] +
    rtb_Mb[2])) + QuadModelv_B.deltaM[2]) + QuadModelv_B.fm[5]) -
            VectorConcatenate_0[2]) - (QuadModelv_X.pqr_CSTATE[0] * rtb_wind[1]
    - rtb_TmpSignalConversionAtSFun_1);

  // Product: '<S16>/Product2' incorporates:
  //   Selector: '<S16>/Selector2'

  rt_mrdivide_U1d1x3_U2d_9vOrDY9Z(tmp, QuadModelv_B.Selector2,
    QuadModelv_B.Product2);

  // Gain: '<S415>/Zero-Order Hold3'
  rtb_wind[0] = QuadModelv_P.ZeroOrderHold3_Gain * QuadModelv_B.Product2[0];
  rtb_wind[1] = QuadModelv_P.ZeroOrderHold3_Gain * QuadModelv_B.Product2[1];
  rtb_wind[2] = QuadModelv_P.ZeroOrderHold3_Gain * QuadModelv_B.Product2[2];

  // Product: '<S429>/j x i'
  rtb_TmpSignalConversionAtSFun_1 = QuadModelv_B.Gain_f[0] * rtb_wind[1];

  // Product: '<S398>/Matrix Multiply1' incorporates:
  //   Concatenate: '<S36>/Vector Concatenate'

  rtb_Rn = QuadModelv_B.MatrixConcatenate[1];
  U1 = QuadModelv_B.MatrixConcatenate[0];
  tz_idx_0 = QuadModelv_B.MatrixConcatenate[2];
  for (i = 0; i < 3; i++) {
    VectorConcatenate_0[i] = (VectorConcatenate[i + 3] * rtb_Rn +
      VectorConcatenate[i] * U1) + VectorConcatenate[i + 6] * tz_idx_0;
  }

  // End of Product: '<S398>/Matrix Multiply1'

  // Sum: '<S415>/Sum' incorporates:
  //   Gain: '<S415>/Zero-Order Hold1'
  //   Gain: '<S415>/Zero-Order Hold2'
  //   Product: '<S428>/i x j'
  //   Product: '<S428>/j x k'
  //   Product: '<S428>/k x i'
  //   Product: '<S429>/i x k'
  //   Product: '<S429>/k x j'
  //   Sum: '<S419>/Sum'
  //   Sum: '<S422>/Sum'

  Product1[0] = ((QuadModelv_P.ZeroOrderHold1_Gain * rtb_DataTypeConversion1[0]
                  - QuadModelv_P.ZeroOrderHold2_Gain * VectorConcatenate_0[0]) +
                 (rtb_q2dot - rtb_Product1_d)) + (rtb_wind[1] *
    QuadModelv_B.Gain_f[2] - QuadModelv_B.Gain_f[1] * rtb_wind[2]);
  Product1[1] = ((QuadModelv_P.ZeroOrderHold1_Gain * rtb_DataTypeConversion1[1]
                  - QuadModelv_P.ZeroOrderHold2_Gain * VectorConcatenate_0[1]) +
                 (X - rtb_sincos_o1_idx_1)) + (QuadModelv_B.Gain_f[0] *
    rtb_wind[2] - rtb_wind[0] * QuadModelv_B.Gain_f[2]);
  Product1[2] = ((QuadModelv_P.ZeroOrderHold1_Gain * rtb_DataTypeConversion1[2]
                  - QuadModelv_P.ZeroOrderHold2_Gain * VectorConcatenate_0[2]) +
                 (uavlostJyz - rtb_q3dot)) + (rtb_wind[0] * QuadModelv_B.Gain_f
    [1] - rtb_TmpSignalConversionAtSFun_1);

  // Product: '<S415>/Product' incorporates:
  //   Constant: '<S415>/Scale factors & Cross-coupling  errors'

  rtb_TmpSignalConversionAtSFun_1 = Product1[1];
  rtb_Rn = Product1[0];
  rtb_q3dot = Product1[2];

  // Sum: '<S415>/Sum4' incorporates:
  //   Constant: '<S415>/Measurement bias'
  //   Constant: '<S415>/Scale factors & Cross-coupling  errors'
  //   Product: '<S415>/Product'

  for (i = 0; i < 3; i++) {
    rtb_wind[i] = ((QuadModelv_P.ThreeaxisInertialMeasurementU_h[i + 3] *
                    rtb_TmpSignalConversionAtSFun_1 +
                    QuadModelv_P.ThreeaxisInertialMeasurementU_h[i] * rtb_Rn) +
                   QuadModelv_P.ThreeaxisInertialMeasurementU_h[i + 6] *
                   rtb_q3dot) + QuadModelv_P.ThreeaxisInertialMeasurementUni[i];
  }

  // End of Sum: '<S415>/Sum4'

  // Switch: '<S417>/Switch' incorporates:
  //   Constant: '<S417>/Constant'

  if (QuadModelv_P.Constant_Value_iy >= QuadModelv_P.Switch_Threshold) {
    // Sum: '<S415>/Sum1' incorporates:
    //   SecondOrderIntegrator: '<S421>/Integrator, Second-Order Limited'

    rtb_Rn = QuadModelv_X.IntegratorSecondOrderLimited_CS[0];
    rtb_TmpSignalConversionAtSFun_1 =
      QuadModelv_X.IntegratorSecondOrderLimited_CS[1];
    rtb_q3dot = QuadModelv_X.IntegratorSecondOrderLimited_CS[2];
  } else {
    // Sum: '<S415>/Sum1'
    rtb_Rn = rtb_wind[0];
    rtb_TmpSignalConversionAtSFun_1 = rtb_wind[1];
    rtb_q3dot = rtb_wind[2];
  }

  // End of Switch: '<S417>/Switch'
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // MATLAB Function: '<S398>/FaultParamsExtract' incorporates:
    //   Constant: '<Root>/Constant1'
    //   Constant: '<S398>/FaultID'

    // MATLAB Function 'SensorFault/FaultParamsExtract': '<S401>:1'
    // '<S401>:1:5' if isempty(hFault)
    // '<S401>:1:8' if isempty(fParam)
    // '<S401>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S401>:1:13' fParamTmp=zeros(20,1);
    std::memset(&fParamTmp[0], 0, 20U * sizeof(real_T));

    // '<S401>:1:14' j=1;
    j = 1;

    // '<S401>:1:15' for i=1:8
    for (r = 0; r < 8; r++) {
      // '<S401>:1:16' if inInts(i) == FaultID
      if (QuadModelv_B.inSILInts[r] == QuadModelv_P.FaultID_Value_f) {
        // '<S401>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S401>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        s458_iter = (r + 1) << 1;
        fParamTmp_tmp = j << 1;
        fParamTmp[fParamTmp_tmp - 2] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 2];

        // '<S401>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[fParamTmp_tmp - 1] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 1];

        // '<S401>:1:20' j=j+1;
        j++;
      }
    }

    // '<S401>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S401>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_gx = true;

      // '<S401>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = QuadModelv_P.Constant1_Value_a[16];
      fParamTmp[17] = QuadModelv_P.Constant1_Value_a[17];
      fParamTmp[18] = QuadModelv_P.Constant1_Value_a[18];
      fParamTmp[19] = QuadModelv_P.Constant1_Value_a[19];

      // '<S401>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_k[0], &fParamTmp[0], 20U * sizeof(real_T));
    }

    // Saturate: '<S415>/Saturation'
    // '<S401>:1:29' hasFault_acc=hFault;
    // '<S401>:1:30' FaultParam=fParam;
    if (rtb_Rn > QuadModelv_P.Saturation_UpperSat[0]) {
      rtb_Rn = QuadModelv_P.Saturation_UpperSat[0];

      // MATLAB Function: '<S398>/Acc NoiseFun'
      rtb_Mb[0] = QuadModelv_P.Saturation_UpperSat[0];
    } else if (rtb_Rn < QuadModelv_P.Saturation_LowerSat[0]) {
      rtb_Rn = QuadModelv_P.Saturation_LowerSat[0];

      // MATLAB Function: '<S398>/Acc NoiseFun'
      rtb_Mb[0] = QuadModelv_P.Saturation_LowerSat[0];
    } else {
      // MATLAB Function: '<S398>/Acc NoiseFun'
      rtb_Mb[0] = rtb_Rn;
    }

    if (rtb_TmpSignalConversionAtSFun_1 > QuadModelv_P.Saturation_UpperSat[1]) {
      rtb_TmpSignalConversionAtSFun_1 = QuadModelv_P.Saturation_UpperSat[1];

      // MATLAB Function: '<S398>/Acc NoiseFun'
      rtb_Mb[1] = QuadModelv_P.Saturation_UpperSat[1];
    } else if (rtb_TmpSignalConversionAtSFun_1 <
               QuadModelv_P.Saturation_LowerSat[1]) {
      rtb_TmpSignalConversionAtSFun_1 = QuadModelv_P.Saturation_LowerSat[1];

      // MATLAB Function: '<S398>/Acc NoiseFun'
      rtb_Mb[1] = QuadModelv_P.Saturation_LowerSat[1];
    } else {
      // MATLAB Function: '<S398>/Acc NoiseFun'
      rtb_Mb[1] = rtb_TmpSignalConversionAtSFun_1;
    }

    if (rtb_q3dot > QuadModelv_P.Saturation_UpperSat[2]) {
      rtb_q3dot = QuadModelv_P.Saturation_UpperSat[2];

      // MATLAB Function: '<S398>/Acc NoiseFun'
      rtb_Mb[2] = QuadModelv_P.Saturation_UpperSat[2];
    } else if (rtb_q3dot < QuadModelv_P.Saturation_LowerSat[2]) {
      rtb_q3dot = QuadModelv_P.Saturation_LowerSat[2];

      // MATLAB Function: '<S398>/Acc NoiseFun'
      rtb_Mb[2] = QuadModelv_P.Saturation_LowerSat[2];
    } else {
      // MATLAB Function: '<S398>/Acc NoiseFun'
      rtb_Mb[2] = rtb_q3dot;
    }

    // End of Saturate: '<S415>/Saturation'

    // MATLAB Function: '<S398>/Acc NoiseFun' incorporates:
    //   MATLAB Function: '<S398>/FaultParamsExtract'

    // MATLAB Function 'SensorFault/Acc NoiseFun': '<S399>:1'
    // '<S399>:1:2' y = AccIn;
    // '<S399>:1:4' if isAccFault
    if (QuadModelv_DW.hFault_gx) {
      //  FaultParams(1)为标志位
      // '<S399>:1:6' flag = AccFaultParams(1);
      //  覆盖模式，当为-1时，保持原有输出
      // '<S399>:1:9' if flag == 1
      if (QuadModelv_DW.fParam_k[0] == 1.0) {
        // '<S399>:1:10' for ind = 1:3
        // '<S399>:1:11' if AccFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_k[1] != -1.0) {
          // '<S399>:1:12' y(ind) = AccFaultParams(ind+1);
          rtb_Mb[0] = QuadModelv_DW.fParam_k[1];
        }

        // '<S399>:1:11' if AccFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_k[2] != -1.0) {
          // '<S399>:1:12' y(ind) = AccFaultParams(ind+1);
          rtb_Mb[1] = QuadModelv_DW.fParam_k[2];
        }

        // '<S399>:1:11' if AccFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_k[3] != -1.0) {
          // '<S399>:1:12' y(ind) = AccFaultParams(ind+1);
          rtb_Mb[2] = QuadModelv_DW.fParam_k[3];
        }

        //  叠加模式，当为-1时，保持原有输出
      } else if (QuadModelv_DW.fParam_k[0] == 2.0) {
        // '<S399>:1:17' elseif flag == 2
        // '<S399>:1:18' for ind = 1:3
        // '<S399>:1:19' if AccFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_k[1] != -1.0) {
          // '<S399>:1:20' y(ind) = y(ind) + AccFaultParams(ind+1);
          rtb_Mb[0] = rtb_Rn + QuadModelv_DW.fParam_k[1];
        }

        // '<S399>:1:19' if AccFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_k[2] != -1.0) {
          // '<S399>:1:20' y(ind) = y(ind) + AccFaultParams(ind+1);
          rtb_Mb[1] = rtb_TmpSignalConversionAtSFun_1 + QuadModelv_DW.fParam_k[2];
        }

        // '<S399>:1:19' if AccFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_k[3] != -1.0) {
          // '<S399>:1:20' y(ind) = y(ind) + AccFaultParams(ind+1);
          rtb_Mb[2] = rtb_q3dot + QuadModelv_DW.fParam_k[3];
        }

        //  噪声模式，当为-1时，保持原有输出
      } else if (QuadModelv_DW.fParam_k[0] == 3.0) {
        // '<S399>:1:25' elseif flag == 3
        // '<S399>:1:26' mean = AccFaultParams(2);
        // '<S399>:1:27' stddev = AccFaultParams(3);
        // '<S399>:1:28' for ind = 1:3
        // '<S399>:1:29' if AccFaultParams(ind+3) ~= -1
        if (QuadModelv_DW.fParam_k[3] != -1.0) {
          // '<S399>:1:30' y(ind) = y(ind) + generate_wgn(mean,stddev);
          // '<S399>:1:50' if isempty(phase)
          // '<S399>:1:57' if phase
          if (QuadModelv_DW.phase_br) {
            // '<S399>:1:58' while true
            do {
              // '<S399>:1:59' U1 = rand();
              U1 = QuadModelv_rand_e();

              // '<S399>:1:60' U2 = rand();
              X = QuadModelv_rand_e();

              // '<S399>:1:61' V1 = 2.0 * U1 - 1.0;
              U1 = 2.0 * U1 - 1.0;

              // '<S399>:1:62' V2 = 2.0 * U2 - 1.0;
              QuadModelv_DW.V2_j = 2.0 * X - 1.0;

              // '<S399>:1:63' S = V1 * V1 + V2 * V2;
              QuadModelv_DW.S_m = U1 * U1 + QuadModelv_DW.V2_j *
                QuadModelv_DW.V2_j;

              // '<S399>:1:64' if S < 1.0 && abs(S) >= 1e-8
            } while ((!(QuadModelv_DW.S_m < 1.0)) || (!(std::abs
                       (QuadModelv_DW.S_m) >= 1.0E-8)));

            // '<S399>:1:68' X = V1 * sqrt(-2.0 * log(S) / S);
            X = std::sqrt(-2.0 * std::log(QuadModelv_DW.S_m) / QuadModelv_DW.S_m)
              * U1;
          } else {
            // '<S399>:1:69' else
            // '<S399>:1:70' X = V2 * sqrt(-2.0 * log(S) / S);
            X = std::sqrt(-2.0 * std::log(QuadModelv_DW.S_m) / QuadModelv_DW.S_m)
              * QuadModelv_DW.V2_j;
          }

          // '<S399>:1:73' y = mean + stddev * X;
          // '<S399>:1:75' phase = ~phase;
          QuadModelv_DW.phase_br = !QuadModelv_DW.phase_br;
          rtb_Mb[0] = (QuadModelv_DW.fParam_k[2] * X + QuadModelv_DW.fParam_k[1])
            + rtb_Rn;
        }

        // '<S399>:1:29' if AccFaultParams(ind+3) ~= -1
        if (QuadModelv_DW.fParam_k[4] != -1.0) {
          // '<S399>:1:30' y(ind) = y(ind) + generate_wgn(mean,stddev);
          // '<S399>:1:50' if isempty(phase)
          // '<S399>:1:57' if phase
          if (QuadModelv_DW.phase_br) {
            // '<S399>:1:58' while true
            do {
              // '<S399>:1:59' U1 = rand();
              U1 = QuadModelv_rand_e();

              // '<S399>:1:60' U2 = rand();
              X = QuadModelv_rand_e();

              // '<S399>:1:61' V1 = 2.0 * U1 - 1.0;
              U1 = 2.0 * U1 - 1.0;

              // '<S399>:1:62' V2 = 2.0 * U2 - 1.0;
              QuadModelv_DW.V2_j = 2.0 * X - 1.0;

              // '<S399>:1:63' S = V1 * V1 + V2 * V2;
              QuadModelv_DW.S_m = U1 * U1 + QuadModelv_DW.V2_j *
                QuadModelv_DW.V2_j;

              // '<S399>:1:64' if S < 1.0 && abs(S) >= 1e-8
            } while ((!(QuadModelv_DW.S_m < 1.0)) || (!(std::abs
                       (QuadModelv_DW.S_m) >= 1.0E-8)));

            // '<S399>:1:68' X = V1 * sqrt(-2.0 * log(S) / S);
            X = std::sqrt(-2.0 * std::log(QuadModelv_DW.S_m) / QuadModelv_DW.S_m)
              * U1;
          } else {
            // '<S399>:1:69' else
            // '<S399>:1:70' X = V2 * sqrt(-2.0 * log(S) / S);
            X = std::sqrt(-2.0 * std::log(QuadModelv_DW.S_m) / QuadModelv_DW.S_m)
              * QuadModelv_DW.V2_j;
          }

          // '<S399>:1:73' y = mean + stddev * X;
          // '<S399>:1:75' phase = ~phase;
          QuadModelv_DW.phase_br = !QuadModelv_DW.phase_br;
          rtb_Mb[1] = (QuadModelv_DW.fParam_k[2] * X + QuadModelv_DW.fParam_k[1])
            + rtb_TmpSignalConversionAtSFun_1;
        }

        // '<S399>:1:29' if AccFaultParams(ind+3) ~= -1
        if (QuadModelv_DW.fParam_k[5] != -1.0) {
          // '<S399>:1:30' y(ind) = y(ind) + generate_wgn(mean,stddev);
          // '<S399>:1:50' if isempty(phase)
          // '<S399>:1:57' if phase
          if (QuadModelv_DW.phase_br) {
            // '<S399>:1:58' while true
            do {
              // '<S399>:1:59' U1 = rand();
              U1 = QuadModelv_rand_e();

              // '<S399>:1:60' U2 = rand();
              X = QuadModelv_rand_e();

              // '<S399>:1:61' V1 = 2.0 * U1 - 1.0;
              U1 = 2.0 * U1 - 1.0;

              // '<S399>:1:62' V2 = 2.0 * U2 - 1.0;
              QuadModelv_DW.V2_j = 2.0 * X - 1.0;

              // '<S399>:1:63' S = V1 * V1 + V2 * V2;
              QuadModelv_DW.S_m = U1 * U1 + QuadModelv_DW.V2_j *
                QuadModelv_DW.V2_j;

              // '<S399>:1:64' if S < 1.0 && abs(S) >= 1e-8
            } while ((!(QuadModelv_DW.S_m < 1.0)) || (!(std::abs
                       (QuadModelv_DW.S_m) >= 1.0E-8)));

            // '<S399>:1:68' X = V1 * sqrt(-2.0 * log(S) / S);
            X = std::sqrt(-2.0 * std::log(QuadModelv_DW.S_m) / QuadModelv_DW.S_m)
              * U1;
          } else {
            // '<S399>:1:69' else
            // '<S399>:1:70' X = V2 * sqrt(-2.0 * log(S) / S);
            X = std::sqrt(-2.0 * std::log(QuadModelv_DW.S_m) / QuadModelv_DW.S_m)
              * QuadModelv_DW.V2_j;
          }

          // '<S399>:1:73' y = mean + stddev * X;
          // '<S399>:1:75' phase = ~phase;
          QuadModelv_DW.phase_br = !QuadModelv_DW.phase_br;
          rtb_Mb[2] = (QuadModelv_DW.fParam_k[2] * X + QuadModelv_DW.fParam_k[1])
            + rtb_q3dot;
        }

        //  尺度因子模式，当为-1时，保持原有输出
      } else if (QuadModelv_DW.fParam_k[0] == 4.0) {
        // '<S399>:1:35' elseif flag == 4
        // '<S399>:1:36' for ind = 1:3
        // '<S399>:1:37' if AccFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_k[1] != -1.0) {
          // '<S399>:1:38' y(ind) = y(ind) * AccFaultParams(ind+1);
          rtb_Mb[0] = rtb_Rn * QuadModelv_DW.fParam_k[1];
        }

        // '<S399>:1:37' if AccFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_k[2] != -1.0) {
          // '<S399>:1:38' y(ind) = y(ind) * AccFaultParams(ind+1);
          rtb_Mb[1] = rtb_TmpSignalConversionAtSFun_1 * QuadModelv_DW.fParam_k[2];
        }

        // '<S399>:1:37' if AccFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_k[3] != -1.0) {
          // '<S399>:1:38' y(ind) = y(ind) * AccFaultParams(ind+1);
          rtb_Mb[2] = rtb_q3dot * QuadModelv_DW.fParam_k[3];
        }
      }
    }

    // DataTypeConversion: '<S398>/Data Type Conversion2'
    QuadModelv_B.DataTypeConversion2_i[0] = static_cast<real32_T>(rtb_Mb[0]);
    QuadModelv_B.DataTypeConversion2_i[1] = static_cast<real32_T>(rtb_Mb[1]);
    QuadModelv_B.DataTypeConversion2_i[2] = static_cast<real32_T>(rtb_Mb[2]);
  }

  // Integrator: '<S14>/p,q,r ' incorporates:
  //   Constant: '<S416>/Scale factors & Cross-coupling  errors '
  //   Product: '<S416>/Product'

  // Unit Conversion - from: m/s^2 to: gn
  // Expression: output = (0.101972*input) + (0)
  tz_idx_0 = QuadModelv_X.pqr_CSTATE[0];
  tz_idx_3 = QuadModelv_X.pqr_CSTATE[1];
  tz_idx_1 = QuadModelv_X.pqr_CSTATE[2];

  // Sum: '<S416>/Sum4' incorporates:
  //   Constant: '<S416>/Measurement bias'
  //   Constant: '<S416>/Scale factors & Cross-coupling  errors '
  //   Constant: '<S416>/g-sensitive bias'
  //   Gain: '<S416>/Zero-Order Hold'
  //   Gain: '<S416>/Zero-Order Hold1'
  //   Integrator: '<S14>/p,q,r '
  //   Product: '<S416>/Product'
  //   Product: '<S416>/Product1'
  //   UnitConversion: '<S414>/Unit Conversion'

  for (i = 0; i < 3; i++) {
    rtb_Mb[i] = (((QuadModelv_P.ZeroOrderHold_Gain_b * tz_idx_0 *
                   QuadModelv_P.ThreeaxisInertialMeasurementU_m[i] +
                   QuadModelv_P.ThreeaxisInertialMeasurementU_m[i + 3] *
                   (QuadModelv_P.ZeroOrderHold_Gain_b * tz_idx_3)) +
                  QuadModelv_P.ThreeaxisInertialMeasurementU_m[i + 6] *
                  (QuadModelv_P.ZeroOrderHold_Gain_b * tz_idx_1)) +
                 QuadModelv_P.ThreeaxisInertialMeasurementU_b[i]) +
      0.10197162129779282 * rtb_DataTypeConversion1[i] *
      QuadModelv_P.ZeroOrderHold1_Gain_i *
      QuadModelv_P.ThreeaxisInertialMeasurementU_c[i];
  }

  // End of Sum: '<S416>/Sum4'
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // MATLAB Function: '<S398>/FaultParamsExtract1' incorporates:
    //   Constant: '<Root>/Constant1'
    //   Constant: '<S398>/FaultID1'

    // MATLAB Function 'SensorFault/FaultParamsExtract1': '<S402>:1'
    // '<S402>:1:5' if isempty(hFault)
    // '<S402>:1:8' if isempty(fParam)
    // '<S402>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S402>:1:13' fParamTmp=zeros(20,1);
    std::memset(&fParamTmp[0], 0, 20U * sizeof(real_T));

    // '<S402>:1:14' j=1;
    j = 1;

    // '<S402>:1:15' for i=1:8
    for (r = 0; r < 8; r++) {
      // '<S402>:1:16' if inInts(i) == FaultID
      if (QuadModelv_B.inSILInts[r] == QuadModelv_P.FaultID1_Value_i) {
        // '<S402>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S402>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        s458_iter = (r + 1) << 1;
        fParamTmp_tmp = j << 1;
        fParamTmp[fParamTmp_tmp - 2] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 2];

        // '<S402>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[fParamTmp_tmp - 1] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 1];

        // '<S402>:1:20' j=j+1;
        j++;
      }
    }

    // '<S402>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S402>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_l = true;

      // '<S402>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = QuadModelv_P.Constant1_Value_a[16];
      fParamTmp[17] = QuadModelv_P.Constant1_Value_a[17];
      fParamTmp[18] = QuadModelv_P.Constant1_Value_a[18];
      fParamTmp[19] = QuadModelv_P.Constant1_Value_a[19];

      // '<S402>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_i[0], &fParamTmp[0], 20U * sizeof(real_T));
    }

    // Switch: '<S430>/Switch' incorporates:
    //   Constant: '<S430>/Constant'
    //   SecondOrderIntegrator: '<S432>/Integrator, Second-Order Limited'

    // '<S402>:1:29' hasFault_gyro=hFault;
    // '<S402>:1:30' FaultParam=fParam;
    if (QuadModelv_P.Constant_Value_e >= QuadModelv_P.Switch_Threshold_p) {
      rtb_Rn = QuadModelv_X.IntegratorSecondOrderLimited__i[0];
    } else {
      rtb_Rn = rtb_Mb[0];
    }

    // Saturate: '<S416>/Saturation'
    if (rtb_Rn > QuadModelv_P.Saturation_UpperSat_j[0]) {
      rtb_TmpSignalConversionAtSFun_1 = QuadModelv_P.Saturation_UpperSat_j[0];

      // MATLAB Function: '<S398>/Gyro NoiseFun'
      rtb_DataTypeConversion1[0] = QuadModelv_P.Saturation_UpperSat_j[0];
    } else if (rtb_Rn < QuadModelv_P.Saturation_LowerSat_a[0]) {
      rtb_TmpSignalConversionAtSFun_1 = QuadModelv_P.Saturation_LowerSat_a[0];

      // MATLAB Function: '<S398>/Gyro NoiseFun'
      rtb_DataTypeConversion1[0] = QuadModelv_P.Saturation_LowerSat_a[0];
    } else {
      rtb_TmpSignalConversionAtSFun_1 = rtb_Rn;

      // MATLAB Function: '<S398>/Gyro NoiseFun'
      rtb_DataTypeConversion1[0] = rtb_Rn;
    }

    // Switch: '<S430>/Switch' incorporates:
    //   Constant: '<S430>/Constant'
    //   SecondOrderIntegrator: '<S432>/Integrator, Second-Order Limited'

    if (QuadModelv_P.Constant_Value_e >= QuadModelv_P.Switch_Threshold_p) {
      rtb_Rn = QuadModelv_X.IntegratorSecondOrderLimited__i[1];
    } else {
      rtb_Rn = rtb_Mb[1];
    }

    // Saturate: '<S416>/Saturation'
    if (rtb_Rn > QuadModelv_P.Saturation_UpperSat_j[1]) {
      rtb_Sum_o1 = QuadModelv_P.Saturation_UpperSat_j[1];

      // MATLAB Function: '<S398>/Gyro NoiseFun'
      rtb_DataTypeConversion1[1] = QuadModelv_P.Saturation_UpperSat_j[1];
    } else if (rtb_Rn < QuadModelv_P.Saturation_LowerSat_a[1]) {
      rtb_Sum_o1 = QuadModelv_P.Saturation_LowerSat_a[1];

      // MATLAB Function: '<S398>/Gyro NoiseFun'
      rtb_DataTypeConversion1[1] = QuadModelv_P.Saturation_LowerSat_a[1];
    } else {
      rtb_Sum_o1 = rtb_Rn;

      // MATLAB Function: '<S398>/Gyro NoiseFun'
      rtb_DataTypeConversion1[1] = rtb_Rn;
    }

    // Switch: '<S430>/Switch' incorporates:
    //   Constant: '<S430>/Constant'
    //   SecondOrderIntegrator: '<S432>/Integrator, Second-Order Limited'

    if (QuadModelv_P.Constant_Value_e >= QuadModelv_P.Switch_Threshold_p) {
      rtb_Rn = QuadModelv_X.IntegratorSecondOrderLimited__i[2];
    } else {
      rtb_Rn = rtb_Mb[2];
    }

    // Saturate: '<S416>/Saturation'
    if (rtb_Rn > QuadModelv_P.Saturation_UpperSat_j[2]) {
      rtb_Rn = QuadModelv_P.Saturation_UpperSat_j[2];

      // MATLAB Function: '<S398>/Gyro NoiseFun'
      rtb_DataTypeConversion1[2] = QuadModelv_P.Saturation_UpperSat_j[2];
    } else if (rtb_Rn < QuadModelv_P.Saturation_LowerSat_a[2]) {
      rtb_Rn = QuadModelv_P.Saturation_LowerSat_a[2];

      // MATLAB Function: '<S398>/Gyro NoiseFun'
      rtb_DataTypeConversion1[2] = QuadModelv_P.Saturation_LowerSat_a[2];
    } else {
      // MATLAB Function: '<S398>/Gyro NoiseFun'
      rtb_DataTypeConversion1[2] = rtb_Rn;
    }

    // MATLAB Function: '<S398>/Gyro NoiseFun' incorporates:
    //   MATLAB Function: '<S398>/FaultParamsExtract1'

    // MATLAB Function 'SensorFault/Gyro NoiseFun': '<S405>:1'
    // '<S405>:1:2' y = GyroIn;
    // '<S405>:1:4' if isGyroFault
    if (QuadModelv_DW.hFault_l) {
      //  FaultParams(1)为标志位
      // '<S405>:1:6' flag = GyroFaultParams(1);
      //  覆盖模式，当为-1时，保持原有输出
      // '<S405>:1:9' if flag == 1
      if (QuadModelv_DW.fParam_i[0] == 1.0) {
        // '<S405>:1:10' for ind = 1:3
        // '<S405>:1:11' if GyroFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_i[1] != -1.0) {
          // '<S405>:1:12' y(ind) = GyroFaultParams(ind+1);
          rtb_DataTypeConversion1[0] = QuadModelv_DW.fParam_i[1];
        }

        // '<S405>:1:11' if GyroFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_i[2] != -1.0) {
          // '<S405>:1:12' y(ind) = GyroFaultParams(ind+1);
          rtb_DataTypeConversion1[1] = QuadModelv_DW.fParam_i[2];
        }

        // '<S405>:1:11' if GyroFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_i[3] != -1.0) {
          // '<S405>:1:12' y(ind) = GyroFaultParams(ind+1);
          rtb_DataTypeConversion1[2] = QuadModelv_DW.fParam_i[3];
        }

        //  叠加模式，当为-1时，保持原有输出
      } else if (QuadModelv_DW.fParam_i[0] == 2.0) {
        // '<S405>:1:17' elseif flag == 2
        // '<S405>:1:18' for ind = 1:3
        // '<S405>:1:19' if GyroFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_i[1] != -1.0) {
          // '<S405>:1:20' y(ind) = y(ind) + GyroFaultParams(ind+1);
          rtb_DataTypeConversion1[0] = rtb_TmpSignalConversionAtSFun_1 +
            QuadModelv_DW.fParam_i[1];
        }

        // '<S405>:1:19' if GyroFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_i[2] != -1.0) {
          // '<S405>:1:20' y(ind) = y(ind) + GyroFaultParams(ind+1);
          rtb_DataTypeConversion1[1] = rtb_Sum_o1 + QuadModelv_DW.fParam_i[2];
        }

        // '<S405>:1:19' if GyroFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_i[3] != -1.0) {
          // '<S405>:1:20' y(ind) = y(ind) + GyroFaultParams(ind+1);
          rtb_DataTypeConversion1[2] = rtb_Rn + QuadModelv_DW.fParam_i[3];
        }

        //  噪声模式，当为-1时，保持原有输出
      } else if (QuadModelv_DW.fParam_i[0] == 3.0) {
        // '<S405>:1:25' elseif flag == 3
        // '<S405>:1:26' mean = GyroFaultParams(2);
        // '<S405>:1:27' stddev = GyroFaultParams(3);
        // '<S405>:1:28' for ind = 1:3
        // '<S405>:1:29' if GyroFaultParams(ind+3) ~= -1
        if (QuadModelv_DW.fParam_i[3] != -1.0) {
          // '<S405>:1:30' y(ind) = y(ind) + generate_wgn(mean,stddev);
          // '<S405>:1:50' if isempty(phase)
          // '<S405>:1:57' if phase
          if (QuadModelv_DW.phase_a) {
            // '<S405>:1:58' while true
            do {
              // '<S405>:1:59' U1 = rand();
              U1 = QuadModelv_rand_i();

              // '<S405>:1:60' U2 = rand();
              X = QuadModelv_rand_i();

              // '<S405>:1:61' V1 = 2.0 * U1 - 1.0;
              U1 = 2.0 * U1 - 1.0;

              // '<S405>:1:62' V2 = 2.0 * U2 - 1.0;
              QuadModelv_DW.V2_e = 2.0 * X - 1.0;

              // '<S405>:1:63' S = V1 * V1 + V2 * V2;
              QuadModelv_DW.S_k = U1 * U1 + QuadModelv_DW.V2_e *
                QuadModelv_DW.V2_e;

              // '<S405>:1:64' if S < 1.0 && abs(S) >= 1e-8
            } while ((!(QuadModelv_DW.S_k < 1.0)) || (!(std::abs
                       (QuadModelv_DW.S_k) >= 1.0E-8)));

            // '<S405>:1:68' X = V1 * sqrt(-2.0 * log(S) / S);
            X = std::sqrt(-2.0 * std::log(QuadModelv_DW.S_k) / QuadModelv_DW.S_k)
              * U1;
          } else {
            // '<S405>:1:69' else
            // '<S405>:1:70' X = V2 * sqrt(-2.0 * log(S) / S);
            X = std::sqrt(-2.0 * std::log(QuadModelv_DW.S_k) / QuadModelv_DW.S_k)
              * QuadModelv_DW.V2_e;
          }

          // '<S405>:1:73' y = mean + stddev * X;
          // '<S405>:1:75' phase = ~phase;
          QuadModelv_DW.phase_a = !QuadModelv_DW.phase_a;
          rtb_DataTypeConversion1[0] = (QuadModelv_DW.fParam_i[2] * X +
            QuadModelv_DW.fParam_i[1]) + rtb_TmpSignalConversionAtSFun_1;
        }

        // '<S405>:1:29' if GyroFaultParams(ind+3) ~= -1
        if (QuadModelv_DW.fParam_i[4] != -1.0) {
          // '<S405>:1:30' y(ind) = y(ind) + generate_wgn(mean,stddev);
          // '<S405>:1:50' if isempty(phase)
          // '<S405>:1:57' if phase
          if (QuadModelv_DW.phase_a) {
            // '<S405>:1:58' while true
            do {
              // '<S405>:1:59' U1 = rand();
              U1 = QuadModelv_rand_i();

              // '<S405>:1:60' U2 = rand();
              X = QuadModelv_rand_i();

              // '<S405>:1:61' V1 = 2.0 * U1 - 1.0;
              U1 = 2.0 * U1 - 1.0;

              // '<S405>:1:62' V2 = 2.0 * U2 - 1.0;
              QuadModelv_DW.V2_e = 2.0 * X - 1.0;

              // '<S405>:1:63' S = V1 * V1 + V2 * V2;
              QuadModelv_DW.S_k = U1 * U1 + QuadModelv_DW.V2_e *
                QuadModelv_DW.V2_e;

              // '<S405>:1:64' if S < 1.0 && abs(S) >= 1e-8
            } while ((!(QuadModelv_DW.S_k < 1.0)) || (!(std::abs
                       (QuadModelv_DW.S_k) >= 1.0E-8)));

            // '<S405>:1:68' X = V1 * sqrt(-2.0 * log(S) / S);
            X = std::sqrt(-2.0 * std::log(QuadModelv_DW.S_k) / QuadModelv_DW.S_k)
              * U1;
          } else {
            // '<S405>:1:69' else
            // '<S405>:1:70' X = V2 * sqrt(-2.0 * log(S) / S);
            X = std::sqrt(-2.0 * std::log(QuadModelv_DW.S_k) / QuadModelv_DW.S_k)
              * QuadModelv_DW.V2_e;
          }

          // '<S405>:1:73' y = mean + stddev * X;
          // '<S405>:1:75' phase = ~phase;
          QuadModelv_DW.phase_a = !QuadModelv_DW.phase_a;
          rtb_DataTypeConversion1[1] = (QuadModelv_DW.fParam_i[2] * X +
            QuadModelv_DW.fParam_i[1]) + rtb_Sum_o1;
        }

        // '<S405>:1:29' if GyroFaultParams(ind+3) ~= -1
        if (QuadModelv_DW.fParam_i[5] != -1.0) {
          // '<S405>:1:30' y(ind) = y(ind) + generate_wgn(mean,stddev);
          // '<S405>:1:50' if isempty(phase)
          // '<S405>:1:57' if phase
          if (QuadModelv_DW.phase_a) {
            // '<S405>:1:58' while true
            do {
              // '<S405>:1:59' U1 = rand();
              U1 = QuadModelv_rand_i();

              // '<S405>:1:60' U2 = rand();
              X = QuadModelv_rand_i();

              // '<S405>:1:61' V1 = 2.0 * U1 - 1.0;
              U1 = 2.0 * U1 - 1.0;

              // '<S405>:1:62' V2 = 2.0 * U2 - 1.0;
              QuadModelv_DW.V2_e = 2.0 * X - 1.0;

              // '<S405>:1:63' S = V1 * V1 + V2 * V2;
              QuadModelv_DW.S_k = U1 * U1 + QuadModelv_DW.V2_e *
                QuadModelv_DW.V2_e;

              // '<S405>:1:64' if S < 1.0 && abs(S) >= 1e-8
            } while ((!(QuadModelv_DW.S_k < 1.0)) || (!(std::abs
                       (QuadModelv_DW.S_k) >= 1.0E-8)));

            // '<S405>:1:68' X = V1 * sqrt(-2.0 * log(S) / S);
            X = std::sqrt(-2.0 * std::log(QuadModelv_DW.S_k) / QuadModelv_DW.S_k)
              * U1;
          } else {
            // '<S405>:1:69' else
            // '<S405>:1:70' X = V2 * sqrt(-2.0 * log(S) / S);
            X = std::sqrt(-2.0 * std::log(QuadModelv_DW.S_k) / QuadModelv_DW.S_k)
              * QuadModelv_DW.V2_e;
          }

          // '<S405>:1:73' y = mean + stddev * X;
          // '<S405>:1:75' phase = ~phase;
          QuadModelv_DW.phase_a = !QuadModelv_DW.phase_a;
          rtb_DataTypeConversion1[2] = (QuadModelv_DW.fParam_i[2] * X +
            QuadModelv_DW.fParam_i[1]) + rtb_Rn;
        }

        //  尺度因子模式，当为-1时，保持原有输出
      } else if (QuadModelv_DW.fParam_i[0] == 4.0) {
        // '<S405>:1:35' elseif flag == 4
        // '<S405>:1:36' for ind = 1:3
        // '<S405>:1:37' if GyroFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_i[1] != -1.0) {
          // '<S405>:1:38' y(ind) = y(ind) * GyroFaultParams(ind+1);
          rtb_DataTypeConversion1[0] = rtb_TmpSignalConversionAtSFun_1 *
            QuadModelv_DW.fParam_i[1];
        }

        // '<S405>:1:37' if GyroFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_i[2] != -1.0) {
          // '<S405>:1:38' y(ind) = y(ind) * GyroFaultParams(ind+1);
          rtb_DataTypeConversion1[1] = rtb_Sum_o1 * QuadModelv_DW.fParam_i[2];
        }

        // '<S405>:1:37' if GyroFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_i[3] != -1.0) {
          // '<S405>:1:38' y(ind) = y(ind) * GyroFaultParams(ind+1);
          rtb_DataTypeConversion1[2] = rtb_Rn * QuadModelv_DW.fParam_i[3];
        }
      }
    }

    // DataTypeConversion: '<S398>/Data Type Conversion3'
    QuadModelv_B.DataTypeConversion3[0] = static_cast<real32_T>
      (rtb_DataTypeConversion1[0]);
    QuadModelv_B.DataTypeConversion3[1] = static_cast<real32_T>
      (rtb_DataTypeConversion1[1]);
    QuadModelv_B.DataTypeConversion3[2] = static_cast<real32_T>
      (rtb_DataTypeConversion1[2]);

    // Sum: '<S442>/Sum' incorporates:
    //   Constant: '<S411>/Decimal Year'
    //   Constant: '<S442>/epoch'

    rtb_Sum_o1 = QuadModelv_P.DecimalYear_Value - QuadModelv_P.epoch_Value;

    // RelationalOperator: '<S456>/Relational Operator' incorporates:
    //   Constant: '<S411>/Decimal Year'
    //   Memory: '<S456>/otime'

    rtb_RelationalOperator = (QuadModelv_P.DecimalYear_Value !=
      QuadModelv_DW.otime_PreviousInput);
  }

  // Switch: '<S447>/Switch' incorporates:
  //   Abs: '<S447>/Abs'
  //   Bias: '<S447>/Bias'
  //   Bias: '<S447>/Bias1'
  //   Constant: '<S447>/Constant2'
  //   Constant: '<S448>/Constant'
  //   Math: '<S447>/Math Function1'
  //   RelationalOperator: '<S448>/Compare'

  if (std::abs(LoadLeakRatio) > QuadModelv_P.CompareToConstant_const_n) {
    rtb_q3dot = rt_modd_snf(LoadLeakRatio + QuadModelv_P.Bias_Bias_el,
      QuadModelv_P.Constant2_Value_e) + QuadModelv_P.Bias1_Bias_jm;
  } else {
    rtb_q3dot = LoadLeakRatio;
  }

  // End of Switch: '<S447>/Switch'

  // Abs: '<S444>/Abs1'
  rtb_Rn = std::abs(rtb_q3dot);

  // RelationalOperator: '<S446>/Compare' incorporates:
  //   Constant: '<S446>/Constant'

  hFaultTmp = (rtb_Rn > QuadModelv_P.CompareToConstant_const_a);

  // Switch: '<S438>/Switch1' incorporates:
  //   Constant: '<S438>/Constant'
  //   Constant: '<S438>/Constant1'

  if (hFaultTmp) {
    rtb_sincos_o1_idx_1 = QuadModelv_P.Constant_Value_go;
  } else {
    rtb_sincos_o1_idx_1 = QuadModelv_P.Constant1_Value_ak;
  }

  // Sum: '<S438>/Sum' incorporates:
  //   Switch: '<S438>/Switch1'

  rtb_TmpSignalConversionAtSFun_1 = rtb_sincos_o1_idx_1 + Z;

  // Switch: '<S445>/Switch' incorporates:
  //   Abs: '<S445>/Abs'
  //   Constant: '<S449>/Constant'
  //   RelationalOperator: '<S449>/Compare'

  if (std::abs(rtb_TmpSignalConversionAtSFun_1) >
      QuadModelv_P.CompareToConstant_const_e) {
    // Switch: '<S445>/Switch' incorporates:
    //   Bias: '<S445>/Bias'
    //   Bias: '<S445>/Bias1'
    //   Constant: '<S445>/Constant2'
    //   Math: '<S445>/Math Function1'

    QuadModelv_B.Switch_p = rt_modd_snf(rtb_TmpSignalConversionAtSFun_1 +
      QuadModelv_P.Bias_Bias_b, QuadModelv_P.Constant2_Value_c) +
      QuadModelv_P.Bias1_Bias_p;
  } else {
    // Switch: '<S445>/Switch'
    QuadModelv_B.Switch_p = rtb_TmpSignalConversionAtSFun_1;
  }

  // End of Switch: '<S445>/Switch'

  // Switch: '<S444>/Switch'
  if (hFaultTmp) {
    // Signum: '<S444>/Sign1'
    if (rtIsNaN(rtb_q3dot)) {
      rtb_sincos_o1_idx_1 = (rtNaN);
    } else if (rtb_q3dot < 0.0) {
      rtb_sincos_o1_idx_1 = -1.0;
    } else {
      rtb_sincos_o1_idx_1 = (rtb_q3dot > 0.0);
    }

    // Switch: '<S444>/Switch' incorporates:
    //   Bias: '<S444>/Bias'
    //   Bias: '<S444>/Bias1'
    //   Gain: '<S444>/Gain'
    //   Product: '<S444>/Divide1'
    //   Signum: '<S444>/Sign1'

    QuadModelv_B.Switch_l = ((rtb_Rn + QuadModelv_P.Bias_Bias_c0) *
      QuadModelv_P.Gain_Gain_kn + QuadModelv_P.Bias1_Bias_j) *
      rtb_sincos_o1_idx_1;
  } else {
    // Switch: '<S444>/Switch'
    QuadModelv_B.Switch_l = rtb_q3dot;
  }

  // End of Switch: '<S444>/Switch'

  // UnitConversion: '<S500>/Unit Conversion'
  // Unit Conversion - from: deg to: rad
  // Expression: output = (0.0174533*input) + (0)
  uavlostJyz = 0.017453292519943295 * QuadModelv_B.Switch_p;
  X = 0.017453292519943295 * QuadModelv_B.Switch_l;

  // Trigonometry: '<S453>/sincos'
  tz_idx_0 = std::sin(uavlostJyz);
  rtb_Rn = std::cos(uavlostJyz);
  rtb_sincos_o1_idx_1 = std::sin(X);
  U1 = std::cos(X);
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // Outputs for Enabled SubSystem: '<S442>/Convert from geodetic to  spherical coordinates ' incorporates:
    //   EnablePort: '<S452>/Enable'

    if (rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
      // RelationalOperator: '<S455>/Relational Operator' incorporates:
      //   Memory: '<S455>/olon'

      QuadModelv_DW.Convertfromgeodetictosphericalc = (QuadModelv_B.Switch_p !=
        QuadModelv_DW.olon_PreviousInput);
    }

    // End of Outputs for SubSystem: '<S442>/Convert from geodetic to  spherical coordinates ' 

    // Memory: '<S454>/olat'
    QuadModelv_B.olat = QuadModelv_DW.olat_PreviousInput;
  }

  // Outputs for Enabled SubSystem: '<S442>/Convert from geodetic to  spherical coordinates ' incorporates:
  //   EnablePort: '<S452>/Enable'

  if (QuadModelv_DW.Convertfromgeodetictosphericalc) {
    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
      // Outputs for Iterator SubSystem: '<S452>/For Iterator Subsystem' incorporates:
      //   ForIterator: '<S499>/For Iterator'

      for (j = 1; j <= QuadModelv_P.ForIterator_IterationLimit; j++) {
        // Switch: '<S499>/cp[m-1] sp[m-1]' incorporates:
        //   SignalConversion generated from: '<S452>/cp[2]'
        //   SignalConversion generated from: '<S452>/sp[2]'
        //   UnitDelay: '<S499>/Unit Delay1'

        if (j > QuadModelv_P.cpm1spm1_Threshold) {
          uavlostJyz = QuadModelv_DW.UnitDelay1_DSTATE[0];
          X = QuadModelv_DW.UnitDelay1_DSTATE[1];
        } else {
          uavlostJyz = rtb_Rn;
          X = tz_idx_0;
        }

        // End of Switch: '<S499>/cp[m-1] sp[m-1]'

        // Sum: '<S499>/Sum2' incorporates:
        //   Product: '<S499>/Product1'
        //   Product: '<S499>/Product2'
        //   SignalConversion generated from: '<S452>/cp[2]'
        //   SignalConversion generated from: '<S452>/sp[2]'

        rtb_TmpSignalConversionAtSFun_1 = uavlostJyz * tz_idx_0 + X * rtb_Rn;

        // Assignment: '<S499>/Assignment' incorporates:
        //   Constant: '<S499>/Constant'

        if (j == 1) {
          std::memcpy(&rtb_Assignment[0], &QuadModelv_P.Constant_Value_lw[0],
                      11U * sizeof(real_T));
        }

        rtb_Assignment[j - 1] = rtb_TmpSignalConversionAtSFun_1;

        // End of Assignment: '<S499>/Assignment'

        // Sum: '<S499>/Sum1' incorporates:
        //   Product: '<S499>/Product3'
        //   Product: '<S499>/Product8'
        //   SignalConversion generated from: '<S452>/cp[2]'
        //   SignalConversion generated from: '<S452>/sp[2]'

        rtb_q2dot = uavlostJyz * rtb_Rn - X * tz_idx_0;

        // Assignment: '<S499>/Assignment1' incorporates:
        //   Constant: '<S499>/Constant1'

        if (j == 1) {
          std::memcpy(&rtb_Assignment1[0], &QuadModelv_P.Constant1_Value_n1[0],
                      11U * sizeof(real_T));
        }

        rtb_Assignment1[j - 1] = rtb_q2dot;

        // End of Assignment: '<S499>/Assignment1'

        // Update for UnitDelay: '<S499>/Unit Delay1'
        QuadModelv_DW.UnitDelay1_DSTATE[0] = rtb_q2dot;
        QuadModelv_DW.UnitDelay1_DSTATE[1] = rtb_TmpSignalConversionAtSFun_1;
      }

      // End of Outputs for SubSystem: '<S452>/For Iterator Subsystem'
      for (i = 0; i < 11; i++) {
        // Gain: '<S452>/Gain'
        QuadModelv_B.Gain_i[i] = QuadModelv_P.Gain_Gain_a * rtb_Assignment1[i];
      }

      for (i = 0; i < 11; i++) {
        // Gain: '<S452>/Gain1'
        QuadModelv_B.Gain1[i] = QuadModelv_P.Gain1_Gain_n * rtb_Assignment[i];
      }
    }

    // SignalConversion generated from: '<S452>/cp[13]' incorporates:
    //   Constant: '<S452>/cp[1]'
    //   SignalConversion generated from: '<S452>/cp[2]'

    QuadModelv_B.OutportBufferForcp13[0] = QuadModelv_P.cp1_Value;
    QuadModelv_B.OutportBufferForcp13[1] = rtb_Rn;
    std::memcpy(&QuadModelv_B.OutportBufferForcp13[2], &QuadModelv_B.Gain_i[0],
                11U * sizeof(real_T));

    // SignalConversion generated from: '<S452>/sp[13]' incorporates:
    //   Constant: '<S452>/sp[1]'
    //   SignalConversion generated from: '<S452>/sp[2]'

    QuadModelv_B.OutportBufferForsp13[0] = QuadModelv_P.sp1_Value;
    QuadModelv_B.OutportBufferForsp13[1] = tz_idx_0;
    std::memcpy(&QuadModelv_B.OutportBufferForsp13[2], &QuadModelv_B.Gain1[0],
                11U * sizeof(real_T));
  }

  // End of Outputs for SubSystem: '<S442>/Convert from geodetic to  spherical coordinates ' 

  // Gain: '<S411>/Gain'
  QuadModelv_B.Gain_f0 = QuadModelv_P.Gain_Gain_d0 * rtb_Saturation_1;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // Memory: '<S454>/oalt'
    QuadModelv_B.oalt = QuadModelv_DW.oalt_PreviousInput;
  }

  // Logic: '<S454>/Logical Operator' incorporates:
  //   RelationalOperator: '<S454>/Relational Operator'
  //   RelationalOperator: '<S454>/Relational Operator1'

  rtb_hasFault = ((QuadModelv_B.Switch_l != QuadModelv_B.olat) ||
                  (QuadModelv_B.Gain_f0 != QuadModelv_B.oalt));

  // Product: '<S453>/Product'
  uavlostJyz = rtb_sincos_o1_idx_1 * rtb_sincos_o1_idx_1;

  // Product: '<S453>/Product1'
  rtb_Product1_d = U1 * U1;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // Outputs for Enabled SubSystem: '<S442>/Convert from geodetic to  spherical coordinates' incorporates:
    //   EnablePort: '<S451>/Enable'

    if (rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
      // SignalConversion generated from: '<S451>/Enable'
      QuadModelv_DW.Convertfromgeodetictospherica_g = rtb_hasFault;
    }

    // End of Outputs for SubSystem: '<S442>/Convert from geodetic to  spherical coordinates' 
  }

  // Outputs for Enabled SubSystem: '<S442>/Convert from geodetic to  spherical coordinates' incorporates:
  //   EnablePort: '<S451>/Enable'

  if (QuadModelv_DW.Convertfromgeodetictospherica_g) {
    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
      // Product: '<S451>/b2' incorporates:
      //   Constant: '<S451>/b'

      QuadModelv_B.b2 = QuadModelv_P.b_Value * QuadModelv_P.b_Value;

      // Product: '<S451>/a2' incorporates:
      //   Constant: '<S451>/a'

      QuadModelv_B.a2 = QuadModelv_P.a_Value * QuadModelv_P.a_Value;

      // Sum: '<S494>/Sum1'
      QuadModelv_B.c2 = QuadModelv_B.a2 - QuadModelv_B.b2;
    }

    // Sqrt: '<S494>/sqrt' incorporates:
    //   Product: '<S494>/Product'
    //   Sum: '<S494>/Sum'

    rtb_Rn = std::sqrt(QuadModelv_B.a2 - uavlostJyz * QuadModelv_B.c2);

    // Product: '<S451>/Product1'
    rtb_TmpSignalConversionAtSFun_1 = rtb_Rn * QuadModelv_B.Gain_f0;

    // Sqrt: '<S493>/sqrt' incorporates:
    //   Product: '<S493>/Product10'
    //   Product: '<S493>/Product9'
    //   Sum: '<S493>/Sum7'

    rtb_q3dot = std::sqrt(rtb_Product1_d * QuadModelv_B.a2 + QuadModelv_B.b2 *
                          uavlostJyz);
    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
      // Product: '<S496>/a4'
      QuadModelv_B.a4 = QuadModelv_B.a2 * QuadModelv_B.a2;

      // Sum: '<S496>/Sum9' incorporates:
      //   Product: '<S496>/b4'

      QuadModelv_B.c4 = QuadModelv_B.a4 - QuadModelv_B.b2 * QuadModelv_B.b2;
    }

    // Sqrt: '<S451>/sqrt' incorporates:
    //   Gain: '<S496>/Gain'
    //   Product: '<S496>/Product1'
    //   Product: '<S496>/Product6'
    //   Product: '<S496>/Product7'
    //   Product: '<S496>/Product8'
    //   Sum: '<S496>/Sum5'
    //   Sum: '<S496>/Sum6'

    QuadModelv_B.sqrt_o = std::sqrt((QuadModelv_B.a4 - QuadModelv_B.c4 *
      uavlostJyz) / (rtb_Rn * rtb_Rn) + (QuadModelv_P.Gain_Gain_j *
      rtb_TmpSignalConversionAtSFun_1 + QuadModelv_B.Gain_f0 *
      QuadModelv_B.Gain_f0));

    // Product: '<S491>/Product11' incorporates:
    //   Sum: '<S491>/Sum8'

    QuadModelv_B.Product11 = (QuadModelv_B.Gain_f0 + rtb_q3dot) /
      QuadModelv_B.sqrt_o;

    // Sum: '<S495>/Sum2'
    rtb_Rn = QuadModelv_B.a2 + rtb_TmpSignalConversionAtSFun_1;

    // Sum: '<S495>/Sum1'
    rtb_TmpSignalConversionAtSFun_1 += QuadModelv_B.b2;

    // Product: '<S492>/Product4' incorporates:
    //   Product: '<S492>/Product3'
    //   Product: '<S495>/Product1'
    //   Product: '<S495>/Product2'
    //   Sqrt: '<S492>/sqrt'
    //   Sum: '<S492>/Sum3'

    QuadModelv_B.Product4_p = rtb_sincos_o1_idx_1 / std::sqrt(rtb_Rn * rtb_Rn /
      (rtb_TmpSignalConversionAtSFun_1 * rtb_TmpSignalConversionAtSFun_1) *
      rtb_Product1_d + uavlostJyz);

    // Product: '<S497>/Product1'
    rtb_q3dot *= QuadModelv_B.sqrt_o;
    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
      // Sum: '<S497>/Sum1'
      QuadModelv_B.c2_h = QuadModelv_B.a2 - QuadModelv_B.b2;
    }

    // Product: '<S497>/Product12'
    QuadModelv_B.Product12 = QuadModelv_B.c2_h / rtb_q3dot * U1 *
      rtb_sincos_o1_idx_1;

    // Sqrt: '<S498>/sqrt' incorporates:
    //   Constant: '<S498>/Constant'
    //   Product: '<S498>/Product5'
    //   Sum: '<S498>/Sum4'

    QuadModelv_B.sqrt_d = std::sqrt(QuadModelv_P.Constant_Value_nh -
      QuadModelv_B.Product4_p * QuadModelv_B.Product4_p);
  }

  // End of Outputs for SubSystem: '<S442>/Convert from geodetic to  spherical coordinates' 

  // Product: '<S442>/aor' incorporates:
  //   Constant: '<S442>/re'

  rtb_Product1_d = QuadModelv_P.re_Value / QuadModelv_B.sqrt_o;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // Outputs for Iterator SubSystem: '<S442>/Compute magnetic vector in spherical coordinates' incorporates:
    //   ForIterator: '<S450>/For Iterator'

    // InitializeConditions for UnitDelay: '<S450>/Unit Delay'
    uavlostJyz = QuadModelv_P.UnitDelay_InitialCondition_pm;

    // InitializeConditions for UnitDelay: '<S450>/Unit Delay2'
    tz_idx_0 = QuadModelv_P.UnitDelay2_InitialCondition_j[0];
    tz_idx_1 = QuadModelv_P.UnitDelay2_InitialCondition_j[1];
    tz_idx_2 = QuadModelv_P.UnitDelay2_InitialCondition_j[2];
    tz_idx_3 = QuadModelv_P.UnitDelay2_InitialCondition_j[3];
    for (r = 1; r <= QuadModelv_P.ForIterator_IterationLimit_h; r++) {
      // Switch: '<S450>/ar(n)' incorporates:
      //   Product: '<S442>/ar'

      if (r <= QuadModelv_P.arn_Threshold) {
        uavlostJyz = rtb_Product1_d * rtb_Product1_d;
      }

      // End of Switch: '<S450>/ar(n)'

      // Product: '<S450>/Product8'
      rtb_q3dot = uavlostJyz * rtb_Product1_d;

      // Sum: '<S450>/Sum' incorporates:
      //   Constant: '<S450>/Constant'

      if ((r < 0) && (QuadModelv_P.Constant_Value_lq < MIN_int32_T - r)) {
        j = MIN_int32_T;
      } else if ((r > 0) && (QuadModelv_P.Constant_Value_lq > MAX_int32_T - r))
      {
        j = MAX_int32_T;
      } else {
        j = r + QuadModelv_P.Constant_Value_lq;
      }

      // Outputs for Iterator SubSystem: '<S450>/For Iterator Subsystem' incorporates:
      //   ForIterator: '<S458>/For Iterator'

      // InitializeConditions for UnitDelay: '<S459>/Unit Delay1'
      rtb_Rn = QuadModelv_P.UnitDelay1_InitialCondition_g;

      // InitializeConditions for UnitDelay: '<S459>/Unit Delay3'
      rtb_TmpSignalConversionAtSFun_1 = QuadModelv_P.UnitDelay3_InitialCondition;

      // InitializeConditions for UnitDelay: '<S459>/Unit Delay2'
      rtb_q2dot = QuadModelv_P.UnitDelay2_InitialCondition;

      // InitializeConditions for UnitDelay: '<S459>/Unit Delay4'
      X = QuadModelv_P.UnitDelay4_InitialCondition;
      for (i = 0; i < 6; i++) {
        ForIterator_IterationMarker[i] = 1U;
      }

      // Sum: '<S450>/Sum'
      if (j > 2147483646) {
        j = 2147483646;
      } else if (j < 0) {
        j = 0;
      }

      if (j >= 1) {
        // Selector: '<S459>/snorm[n+m*13]' incorporates:
        //   Constant: '<S463>/Constant'
        //   Sum: '<S463>/Sum1'

        if ((QuadModelv_P.Constant_Value_pe < 0) && (r < MIN_int32_T
             - QuadModelv_P.Constant_Value_pe)) {
          qY = MIN_int32_T;
        } else if ((QuadModelv_P.Constant_Value_pe > 0) && (r > MAX_int32_T
                    - QuadModelv_P.Constant_Value_pe)) {
          qY = MAX_int32_T;
        } else {
          qY = QuadModelv_P.Constant_Value_pe + r;
        }

        // Sum: '<S459>/Sum' incorporates:
        //   Constant: '<S459>/Constant'

        if ((r < 0) && (QuadModelv_P.Constant_Value_j < MIN_int32_T - r)) {
          tmp_1 = MIN_int32_T;
        } else if ((r > 0) && (QuadModelv_P.Constant_Value_j > MAX_int32_T - r))
        {
          tmp_1 = MAX_int32_T;
        } else {
          tmp_1 = r + QuadModelv_P.Constant_Value_j;
        }

        // Sum: '<S459>/Sum4' incorporates:
        //   Constant: '<S459>/Constant1'

        if ((r < 0) && (QuadModelv_P.Constant1_Value_ja < MIN_int32_T - r)) {
          tmp_3 = MIN_int32_T;
        } else if ((r > 0) && (QuadModelv_P.Constant1_Value_ja > MAX_int32_T - r))
        {
          tmp_3 = MAX_int32_T;
        } else {
          tmp_3 = r + QuadModelv_P.Constant1_Value_ja;
        }
      }

      for (s458_iter = 1; s458_iter <= j; s458_iter++) {
        // Sum: '<S458>/Sum1' incorporates:
        //   Constant: '<S458>/Constant'

        if (QuadModelv_P.Constant_Value_cy < s458_iter - MAX_int32_T) {
          fParamTmp_tmp = MAX_int32_T;
        } else {
          fParamTmp_tmp = s458_iter - QuadModelv_P.Constant_Value_cy;
        }

        // Outputs for Enabled SubSystem: '<S458>/Time adjust the gauss coefficients' incorporates:
        //   EnablePort: '<S461>/Enable'

        if (rtb_RelationalOperator) {
          // Outputs for Atomic SubSystem: '<S461>/If Action Subsystem'
          // Sum: '<S487>/Sum1' incorporates:
          //   Constant: '<S487>/Constant1'
          //   Sum: '<S458>/Sum1'

          if ((fParamTmp_tmp < 0) && (QuadModelv_P.Constant1_Value_g <
               MIN_int32_T - fParamTmp_tmp)) {
            qY_1 = MIN_int32_T;
          } else if ((fParamTmp_tmp > 0) && (QuadModelv_P.Constant1_Value_g >
                      MAX_int32_T - fParamTmp_tmp)) {
            qY_1 = MAX_int32_T;
          } else {
            qY_1 = fParamTmp_tmp + QuadModelv_P.Constant1_Value_g;
          }

          // Sum: '<S487>/Sum2' incorporates:
          //   Constant: '<S487>/Constant'

          if ((r < 0) && (QuadModelv_P.Constant_Value_g0 < MIN_int32_T - r)) {
            qY_0 = MIN_int32_T;
          } else if ((r > 0) && (QuadModelv_P.Constant_Value_g0 > MAX_int32_T
                                 - r)) {
            qY_0 = MAX_int32_T;
          } else {
            qY_0 = r + QuadModelv_P.Constant_Value_g0;
          }

          // End of Outputs for SubSystem: '<S461>/If Action Subsystem'

          // Assignment: '<S461>/Assignment'
          if (ForIterator_IterationMarker[4] < 2) {
            ForIterator_IterationMarker[4] = 2U;

            // Assignment: '<S461>/Assignment' incorporates:
            //   UnitDelay: '<S461>/Unit Delay'

            std::memcpy(&QuadModelv_B.Assignment[0],
                        &QuadModelv_DW.UnitDelay_DSTATE_k[0], 169U * sizeof
                        (real_T));
          }

          // Outputs for Atomic SubSystem: '<S461>/If Action Subsystem'
          // Selector: '<S487>/cd[m][n]' incorporates:
          //   Assignment: '<S461>/Assignment'
          //   Constant: '<S461>/cd[maxdef][maxdef]'
          //   Selector: '<S487>/c[m][n]'
          //   Sum: '<S487>/Sum1'
          //   Sum: '<S487>/Sum2'

          i = ((qY_0 - 1) * 13 + qY_1) - 1;

          // Assignment: '<S461>/Assignment' incorporates:
          //   Constant: '<S461>/c[maxdef][maxdef]'
          //   Constant: '<S461>/cd[maxdef][maxdef]'
          //   Product: '<S487>/Product'
          //   Selector: '<S487>/c[m][n]'
          //   Selector: '<S487>/cd[m][n]'
          //   Sum: '<S487>/Sum'

          QuadModelv_B.Assignment[i] = QuadModelv_P.cdmaxdefmaxdef_Value[i] *
            rtb_Sum_o1 + QuadModelv_P.cmaxdefmaxdef_Value[i];

          // End of Outputs for SubSystem: '<S461>/If Action Subsystem'

          // Switch: '<S488>/tc_old' incorporates:
          //   Constant: '<S488>/zeros(maxdef+1,maxdef+1)'
          //   UnitDelay: '<S488>/Unit Delay'

          if (r > QuadModelv_P.tc_old_Threshold) {
            std::memcpy(&rtb_tc_old[0], &QuadModelv_DW.UnitDelay_DSTATE_b[0],
                        169U * sizeof(real_T));
          } else {
            std::memcpy(&rtb_tc_old[0], &QuadModelv_P.zerosmaxdef1maxdef1_Value
                        [0], 169U * sizeof(real_T));
          }

          // End of Switch: '<S488>/tc_old'

          // If: '<S488>/If' incorporates:
          //   Sum: '<S458>/Sum1'

          if (fParamTmp_tmp != 0) {
            // Outputs for IfAction SubSystem: '<S488>/If Action Subsystem1' incorporates:
            //   ActionPort: '<S489>/Action Port'

            // Sum: '<S489>/Sum2' incorporates:
            //   Constant: '<S489>/Constant'

            if ((r < 0) && (QuadModelv_P.Constant_Value_nf < MIN_int32_T - r)) {
              qY_1 = MIN_int32_T;
            } else if ((r > 0) && (QuadModelv_P.Constant_Value_nf > MAX_int32_T
                                   - r)) {
              qY_1 = MAX_int32_T;
            } else {
              qY_1 = r + QuadModelv_P.Constant_Value_nf;
            }

            // Assignment: '<S489>/Assignment2'
            if (ForIterator_IterationMarker[5] < 2) {
              ForIterator_IterationMarker[5] = 2U;

              // Assignment: '<S489>/Assignment2' incorporates:
              //   Switch: '<S488>/tc_old'

              std::memcpy(&QuadModelv_B.Assignment2[0], &rtb_tc_old[0], 169U *
                          sizeof(real_T));
            }

            // Selector: '<S489>/c[m][n]' incorporates:
            //   Assignment: '<S489>/Assignment2'
            //   Constant: '<S461>/cd[maxdef][maxdef]'
            //   Selector: '<S489>/cd[m][n]'
            //   Sum: '<S489>/Sum2'

            i = ((fParamTmp_tmp - 1) * 13 + qY_1) - 1;

            // Assignment: '<S489>/Assignment2' incorporates:
            //   Constant: '<S461>/c[maxdef][maxdef]'
            //   Constant: '<S461>/cd[maxdef][maxdef]'
            //   Product: '<S489>/Product'
            //   Selector: '<S489>/c[m][n]'
            //   Selector: '<S489>/cd[m][n]'
            //   Sum: '<S489>/Sum'

            QuadModelv_B.Assignment2[i] = QuadModelv_P.cdmaxdefmaxdef_Value[i] *
              rtb_Sum_o1 + QuadModelv_P.cmaxdefmaxdef_Value[i];

            // Gain: '<S489>/Gain' incorporates:
            //   Assignment: '<S489>/Assignment2'
            //   Merge: '<S488>/Merge'

            for (i = 0; i < 169; i++) {
              rtb_tc_old[i] = QuadModelv_P.Gain_Gain_ps *
                QuadModelv_B.Assignment2[i];
            }

            // End of Gain: '<S489>/Gain'
            // End of Outputs for SubSystem: '<S488>/If Action Subsystem1'
          }

          // End of If: '<S488>/If'
          for (i = 0; i < 169; i++) {
            // Sum: '<S461>/Sum2' incorporates:
            //   Assignment: '<S461>/Assignment'
            //   Merge: '<S488>/Merge'

            uavlostJyz = QuadModelv_B.Assignment[i];
            rtb_sincos_o1_idx_1 = rtb_tc_old[i];

            // Sum: '<S461>/Sum2' incorporates:
            //   Assignment: '<S461>/Assignment'
            //   Merge: '<S488>/Merge'

            QuadModelv_B.Sum2_h[i] = uavlostJyz + rtb_sincos_o1_idx_1;

            // Update for UnitDelay: '<S461>/Unit Delay' incorporates:
            //   Assignment: '<S461>/Assignment'

            QuadModelv_DW.UnitDelay_DSTATE_k[i] = uavlostJyz;

            // Update for UnitDelay: '<S488>/Unit Delay' incorporates:
            //   Merge: '<S488>/Merge'

            QuadModelv_DW.UnitDelay_DSTATE_b[i] = rtb_sincos_o1_idx_1;
          }
        }

        // End of Outputs for SubSystem: '<S458>/Time adjust the gauss coefficients' 

        // Sum: '<S464>/Sum4' incorporates:
        //   Constant: '<S464>/Constant1'
        //   Sum: '<S458>/Sum1'

        uavlostJyz = static_cast<real_T>(fParamTmp_tmp) +
          QuadModelv_P.Constant1_Value_n;

        // If: '<S464>/If' incorporates:
        //   Sum: '<S458>/Sum1'

        if (fParamTmp_tmp == 0) {
          // Outputs for IfAction SubSystem: '<S464>/If Action Subsystem' incorporates:
          //   ActionPort: '<S470>/Action Port'

          // Product: '<S470>/Product' incorporates:
          //   Constant: '<S470>/Constant'
          //   Selector: '<S470>/Selector'
          //   Sum: '<S461>/Sum2'
          //   Sum: '<S470>/Sum'

          rtb_sincos_o1_idx_1 = QuadModelv_B.Sum2_h[((static_cast<int32_T>(
            static_cast<real_T>(r) + QuadModelv_P.Constant_Value_g) - 1) * 13 +
            static_cast<int32_T>(QuadModelv_P.Constant_Value_g)) - 1];

          // Merge: '<S464>/Merge' incorporates:
          //   Gain: '<S470>/Gain1'
          //   Product: '<S470>/Product'
          //   Selector: '<S464>/cp[m+1]'
          //   Selector: '<S470>/Selector'

          U1 = QuadModelv_B.OutportBufferForcp13[static_cast<int32_T>(uavlostJyz)
            - 1] * rtb_sincos_o1_idx_1 * QuadModelv_P.Gain1_Gain;

          // Merge: '<S464>/Merge1' incorporates:
          //   Gain: '<S470>/Gain2'
          //   Product: '<S470>/Product'
          //   Selector: '<S464>/cp[m+1]'
          //   Selector: '<S464>/sp[m+1]'

          Merge1 = QuadModelv_B.OutportBufferForsp13[static_cast<int32_T>
            (uavlostJyz) - 1] * rtb_sincos_o1_idx_1 * QuadModelv_P.Gain2_Gain;

          // End of Outputs for SubSystem: '<S464>/If Action Subsystem'
        } else {
          // Outputs for IfAction SubSystem: '<S464>/If Action Subsystem1' incorporates:
          //   ActionPort: '<S471>/Action Port'

          // Product: '<S471>/Product1' incorporates:
          //   Constant: '<S473>/Constant'
          //   Selector: '<S471>/Selector1'
          //   Sum: '<S473>/Sum'

          i = ((fParamTmp_tmp - 1) * 13 + static_cast<int32_T>
               (static_cast<real_T>(r) + QuadModelv_P.Constant_Value_k4)) - 1;

          // Product: '<S471>/Product' incorporates:
          //   Constant: '<S472>/Constant'
          //   Selector: '<S471>/Selector'
          //   Sum: '<S472>/Sum'

          qY_1 = ((static_cast<int32_T>(static_cast<real_T>(r) +
                    QuadModelv_P.Constant_Value_i) - 1) * 13 +
                  static_cast<int32_T>(static_cast<real_T>(fParamTmp_tmp) +
                   QuadModelv_P.Constant_Value_i)) - 1;

          // Product: '<S471>/Product1' incorporates:
          //   Product: '<S471>/Product'
          //   Selector: '<S464>/cp[m+1]'
          //   Selector: '<S464>/sp[m+1]'

          rtb_sincos_o1_idx_1 = QuadModelv_B.OutportBufferForsp13
            [static_cast<int32_T>(uavlostJyz) - 1];

          // Merge: '<S464>/Merge' incorporates:
          //   Product: '<S471>/Product'
          //   Product: '<S471>/Product1'
          //   Selector: '<S464>/cp[m+1]'
          //   Selector: '<S464>/sp[m+1]'
          //   Selector: '<S471>/Selector'
          //   Selector: '<S471>/Selector1'
          //   Sum: '<S461>/Sum2'
          //   Sum: '<S471>/Sum'

          U1 = QuadModelv_B.OutportBufferForcp13[static_cast<int32_T>(uavlostJyz)
            - 1] * QuadModelv_B.Sum2_h[qY_1] + QuadModelv_B.Sum2_h[i] *
            rtb_sincos_o1_idx_1;

          // Merge: '<S464>/Merge1' incorporates:
          //   Constant: '<S464>/Constant1'
          //   Product: '<S471>/Product'
          //   Product: '<S471>/Product1'
          //   Selector: '<S464>/cp[m+1]'
          //   Selector: '<S471>/Selector'
          //   Selector: '<S471>/Selector1'
          //   Sum: '<S461>/Sum2'
          //   Sum: '<S464>/Sum4'
          //   Sum: '<S471>/Sum1'

          Merge1 = QuadModelv_B.Sum2_h[qY_1] * rtb_sincos_o1_idx_1 -
            QuadModelv_B.OutportBufferForcp13[static_cast<int32_T>
            (static_cast<real_T>(fParamTmp_tmp) + QuadModelv_P.Constant1_Value_n)
            - 1] * QuadModelv_B.Sum2_h[i];

          // End of Outputs for SubSystem: '<S464>/If Action Subsystem1'
        }

        // End of If: '<S464>/If'

        // Outputs for Enabled SubSystem: '<S458>/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations' incorporates:
        //   EnablePort: '<S460>/Enable'

        // SignalConversion generated from: '<S460>/Enable'
        if (rtb_hasFault) {
          // If: '<S460>/if n == m elseif (n==1&m==0) elseif (n>1&m~=n)' incorporates:
          //   Sum: '<S458>/Sum1'

          if (r == fParamTmp_tmp) {
            // Outputs for IfAction SubSystem: '<S460>/If Action Subsystem' incorporates:
            //   ActionPort: '<S474>/Action Port'

            // Selector: '<S474>/Selector' incorporates:
            //   Constant: '<S478>/Constant'
            //   Gain: '<S478>/Gain'
            //   Sum: '<S478>/Sum1'
            //   Sum: '<S478>/Sum2'

            if ((fParamTmp_tmp >= 0) && (QuadModelv_P.Constant_Value_os <
                 fParamTmp_tmp - MAX_int32_T)) {
              i = MAX_int32_T;
            } else if ((fParamTmp_tmp < 0) && (QuadModelv_P.Constant_Value_os >
                        fParamTmp_tmp - MIN_int32_T)) {
              i = MIN_int32_T;
            } else {
              i = fParamTmp_tmp - QuadModelv_P.Constant_Value_os;
            }

            i = mul_s32_sat(QuadModelv_P.Gain_Gain_bd, i);
            if ((r < 0) && (i < MIN_int32_T - r)) {
              i = MIN_int32_T;
            } else if ((r > 0) && (i > MAX_int32_T - r)) {
              i = MAX_int32_T;
            } else {
              i += r;
            }

            // Merge: '<S460>/Merge' incorporates:
            //   Product: '<S474>/Product1'
            //   Selector: '<S474>/Selector'
            //   Sum: '<S478>/Sum1'
            //   UnitDelay: '<S460>/Unit Delay1'

            QuadModelv_B.Merge_i = QuadModelv_DW.UnitDelay1_DSTATE_j[i - 1] *
              QuadModelv_B.sqrt_d;

            // Selector: '<S474>/Selector' incorporates:
            //   Constant: '<S478>/Constant'
            //   Gain: '<S478>/Gain'
            //   Sum: '<S478>/Sum1'
            //   Sum: '<S478>/Sum2'

            if ((fParamTmp_tmp >= 0) && (QuadModelv_P.Constant_Value_os <
                 fParamTmp_tmp - MAX_int32_T)) {
              i = MAX_int32_T;
            } else if ((fParamTmp_tmp < 0) && (QuadModelv_P.Constant_Value_os >
                        fParamTmp_tmp - MIN_int32_T)) {
              i = MIN_int32_T;
            } else {
              i = fParamTmp_tmp - QuadModelv_P.Constant_Value_os;
            }

            i = mul_s32_sat(QuadModelv_P.Gain_Gain_bd, i);
            if ((r < 0) && (i < MIN_int32_T - r)) {
              i = MIN_int32_T;
            } else if ((r > 0) && (i > MAX_int32_T - r)) {
              i = MAX_int32_T;
            } else {
              i += r;
            }

            // Merge: '<S460>/Merge1' incorporates:
            //   Product: '<S474>/Product'
            //   Product: '<S474>/Product2'
            //   Selector: '<S474>/Selector'
            //   Selector: '<S474>/Selector1'
            //   Sum: '<S474>/Sum'
            //   Sum: '<S478>/Sum1'
            //   UnitDelay: '<S460>/Unit Delay'
            //   UnitDelay: '<S460>/Unit Delay1'

            QuadModelv_B.Merge1_a = QuadModelv_DW.UnitDelay_DSTATE_i[((r - 1) *
              13 + fParamTmp_tmp) - 1] * QuadModelv_B.sqrt_d +
              QuadModelv_DW.UnitDelay1_DSTATE_j[i - 1] * QuadModelv_B.Product4_p;

            // End of Outputs for SubSystem: '<S460>/If Action Subsystem'
          } else if ((r == 1) && (fParamTmp_tmp == 0)) {
            // Outputs for IfAction SubSystem: '<S460>/If Action Subsystem1' incorporates:
            //   ActionPort: '<S475>/Action Port'

            // Selector: '<S475>/Selector' incorporates:
            //   Gain: '<S480>/Gain'
            //   Sum: '<S480>/Sum1'

            qY_1 = mul_s32_sat(QuadModelv_P.Gain_Gain_hz, 0);
            if (qY_1 > 2147483646) {
              i = MAX_int32_T;
            } else {
              i = qY_1 + 1;
            }

            // Merge: '<S460>/Merge' incorporates:
            //   Product: '<S475>/Product3'
            //   Selector: '<S475>/Selector'
            //   Sum: '<S480>/Sum1'
            //   UnitDelay: '<S460>/Unit Delay1'

            QuadModelv_B.Merge_i = QuadModelv_DW.UnitDelay1_DSTATE_j[i - 1] *
              QuadModelv_B.Product4_p;

            // Selector: '<S475>/Selector' incorporates:
            //   Sum: '<S480>/Sum1'

            if (qY_1 > 2147483646) {
              i = MAX_int32_T;
            } else {
              i = qY_1 + 1;
            }

            // Merge: '<S460>/Merge1' incorporates:
            //   Constant: '<S481>/Constant'
            //   Product: '<S475>/Product'
            //   Product: '<S475>/Product2'
            //   Selector: '<S475>/Selector'
            //   Selector: '<S475>/Selector1'
            //   Sum: '<S475>/Sum'
            //   Sum: '<S480>/Sum1'
            //   UnitDelay: '<S460>/Unit Delay'
            //   UnitDelay: '<S460>/Unit Delay1'

            QuadModelv_B.Merge1_a =
              QuadModelv_DW.UnitDelay_DSTATE_i[QuadModelv_P.Constant_Value_es -
              1] * QuadModelv_B.Product4_p - QuadModelv_DW.UnitDelay1_DSTATE_j[i
              - 1] * QuadModelv_B.sqrt_d;

            // End of Outputs for SubSystem: '<S460>/If Action Subsystem1'
          } else if ((r > 1) && (r != fParamTmp_tmp)) {
            // Outputs for IfAction SubSystem: '<S460>/If Action Subsystem2' incorporates:
            //   ActionPort: '<S476>/Action Port'

            // Sum: '<S483>/Sum' incorporates:
            //   Constant: '<S483>/Constant'

            if (QuadModelv_P.Constant_Value_g2 > MAX_int32_T - r) {
              qY_1 = MAX_int32_T;
            } else {
              qY_1 = r + QuadModelv_P.Constant_Value_g2;
            }

            if ((fParamTmp_tmp < 0) && (QuadModelv_P.Constant_Value_g2 <
                 MIN_int32_T - fParamTmp_tmp)) {
              qY_0 = MIN_int32_T;
            } else if ((fParamTmp_tmp > 0) && (QuadModelv_P.Constant_Value_g2 >
                        MAX_int32_T - fParamTmp_tmp)) {
              qY_0 = MAX_int32_T;
            } else {
              qY_0 = fParamTmp_tmp + QuadModelv_P.Constant_Value_g2;
            }

            // Gain: '<S482>/Gain'
            rtb_Sum1_i = mul_s32_sat(QuadModelv_P.Gain_Gain_aa, fParamTmp_tmp);

            // Sum: '<S485>/Sum2' incorporates:
            //   Constant: '<S485>/Constant1'

            if (QuadModelv_P.Constant1_Value_fy < r - MAX_int32_T) {
              i = MAX_int32_T;
            } else {
              i = r - QuadModelv_P.Constant1_Value_fy;
            }

            // Switch: '<S476>/Switch' incorporates:
            //   Constant: '<S476>/Constant'
            //   RelationalOperator: '<S485>/Relational Operator'
            //   Selector: '<S476>/Selector1'
            //   Sum: '<S484>/Sum'
            //   Sum: '<S484>/Sum2'
            //   Sum: '<S485>/Sum2'
            //   UnitDelay: '<S460>/Unit Delay'

            if (i >= fParamTmp_tmp) {
              // Sum: '<S484>/Sum' incorporates:
              //   Constant: '<S484>/Constant'

              if ((fParamTmp_tmp < 0) && (QuadModelv_P.Constant_Value_py <
                   MIN_int32_T - fParamTmp_tmp)) {
                i = MIN_int32_T;
              } else if ((fParamTmp_tmp > 0) && (QuadModelv_P.Constant_Value_py >
                          MAX_int32_T - fParamTmp_tmp)) {
                i = MAX_int32_T;
              } else {
                i = fParamTmp_tmp + QuadModelv_P.Constant_Value_py;
              }

              // Selector: '<S476>/Selector1' incorporates:
              //   Constant: '<S484>/Constant1'
              //   Sum: '<S484>/Sum2'

              if (QuadModelv_P.Constant1_Value_c < r - MAX_int32_T) {
                tmp_2 = MAX_int32_T;
              } else {
                tmp_2 = r - QuadModelv_P.Constant1_Value_c;
              }

              rtb_sincos_o1_idx_1 = QuadModelv_DW.UnitDelay_DSTATE_i[((tmp_2 - 1)
                * 13 + i) - 1];
            } else {
              rtb_sincos_o1_idx_1 = QuadModelv_P.Constant_Value_m;
            }

            // Sum: '<S484>/Sum' incorporates:
            //   Constant: '<S484>/Constant'

            if ((fParamTmp_tmp < 0) && (QuadModelv_P.Constant_Value_py <
                 MIN_int32_T - fParamTmp_tmp)) {
              i = MIN_int32_T;
            } else if ((fParamTmp_tmp > 0) && (QuadModelv_P.Constant_Value_py >
                        MAX_int32_T - fParamTmp_tmp)) {
              i = MAX_int32_T;
            } else {
              i = fParamTmp_tmp + QuadModelv_P.Constant_Value_py;
            }

            // Selector: '<S476>/Selector' incorporates:
            //   Sum: '<S482>/Sum1'

            if (rtb_Sum1_i > MAX_int32_T - r) {
              tmp_2 = MAX_int32_T;
            } else {
              tmp_2 = r + rtb_Sum1_i;
            }

            // Selector: '<S476>/Selector2' incorporates:
            //   Constant: '<S476>/k[13][13]'
            //   Sum: '<S483>/Sum'

            qY_0 = ((qY_1 - 1) * 13 + qY_0) - 1;

            // Merge: '<S460>/Merge1' incorporates:
            //   Constant: '<S476>/k[13][13]'
            //   Product: '<S476>/Product'
            //   Product: '<S476>/Product1'
            //   Product: '<S476>/Product4'
            //   Selector: '<S476>/Selector'
            //   Selector: '<S476>/Selector1'
            //   Selector: '<S476>/Selector2'
            //   Sum: '<S476>/Sum'
            //   Sum: '<S482>/Sum1'
            //   Sum: '<S484>/Sum'
            //   Switch: '<S476>/Switch'
            //   UnitDelay: '<S460>/Unit Delay'
            //   UnitDelay: '<S460>/Unit Delay1'

            QuadModelv_B.Merge1_a = (QuadModelv_DW.UnitDelay_DSTATE_i[((r - 1) *
              13 + i) - 1] * QuadModelv_B.Product4_p -
              QuadModelv_DW.UnitDelay1_DSTATE_j[tmp_2 - 1] * QuadModelv_B.sqrt_d)
              - QuadModelv_P.k1313_Value_c[qY_0] * rtb_sincos_o1_idx_1;

            // Sum: '<S486>/Sum2' incorporates:
            //   Constant: '<S486>/Constant1'

            if (QuadModelv_P.Constant1_Value_hs < r - MAX_int32_T) {
              i = MAX_int32_T;
            } else {
              i = r - QuadModelv_P.Constant1_Value_hs;
            }

            // Switch: '<S476>/Switch1' incorporates:
            //   Constant: '<S476>/Constant1'
            //   RelationalOperator: '<S486>/Relational Operator'
            //   Selector: '<S476>/Selector'
            //   Sum: '<S482>/Sum1'
            //   Sum: '<S486>/Sum2'
            //   UnitDelay: '<S460>/Unit Delay1'

            if (i >= fParamTmp_tmp) {
              // Selector: '<S476>/Selector' incorporates:
              //   Constant: '<S482>/Constant1'
              //   Sum: '<S482>/Sum1'
              //   Sum: '<S482>/Sum2'

              if (QuadModelv_P.Constant1_Value_m < r - MAX_int32_T) {
                qY_1 = MAX_int32_T;
              } else {
                qY_1 = r - QuadModelv_P.Constant1_Value_m;
              }

              if ((qY_1 < 0) && (rtb_Sum1_i < MIN_int32_T - qY_1)) {
                i = MIN_int32_T;
              } else if ((qY_1 > 0) && (rtb_Sum1_i > MAX_int32_T - qY_1)) {
                i = MAX_int32_T;
              } else {
                i = qY_1 + rtb_Sum1_i;
              }

              rtb_sincos_o1_idx_1 = QuadModelv_DW.UnitDelay1_DSTATE_j[i - 1];
            } else {
              rtb_sincos_o1_idx_1 = QuadModelv_P.Constant1_Value_o;
            }

            // Selector: '<S476>/Selector' incorporates:
            //   Sum: '<S482>/Sum1'

            if (rtb_Sum1_i > MAX_int32_T - r) {
              i = MAX_int32_T;
            } else {
              i = r + rtb_Sum1_i;
            }

            // Merge: '<S460>/Merge' incorporates:
            //   Constant: '<S476>/k[13][13]'
            //   Product: '<S476>/Product2'
            //   Product: '<S476>/Product3'
            //   Selector: '<S476>/Selector'
            //   Selector: '<S476>/Selector2'
            //   Sum: '<S476>/Sum1'
            //   Sum: '<S482>/Sum1'
            //   Switch: '<S476>/Switch1'
            //   UnitDelay: '<S460>/Unit Delay1'

            QuadModelv_B.Merge_i = QuadModelv_DW.UnitDelay1_DSTATE_j[i - 1] *
              QuadModelv_B.Product4_p - QuadModelv_P.k1313_Value_c[qY_0] *
              rtb_sincos_o1_idx_1;

            // End of Outputs for SubSystem: '<S460>/If Action Subsystem2'
          }

          // End of If: '<S460>/if n == m elseif (n==1&m==0) elseif (n>1&m~=n)'

          // Sum: '<S460>/Sum' incorporates:
          //   Constant: '<S460>/Constant'
          //   Sum: '<S458>/Sum1'

          if ((r < 0) && (QuadModelv_P.Constant_Value_fi < MIN_int32_T - r)) {
            qY_1 = MIN_int32_T;
          } else if ((r > 0) && (QuadModelv_P.Constant_Value_fi > MAX_int32_T
                                 - r)) {
            qY_1 = MAX_int32_T;
          } else {
            qY_1 = r + QuadModelv_P.Constant_Value_fi;
          }

          if ((fParamTmp_tmp < 0) && (QuadModelv_P.Constant_Value_fi <
               MIN_int32_T - fParamTmp_tmp)) {
            qY_0 = MIN_int32_T;
          } else if ((fParamTmp_tmp > 0) && (QuadModelv_P.Constant_Value_fi >
                      MAX_int32_T - fParamTmp_tmp)) {
            qY_0 = MAX_int32_T;
          } else {
            qY_0 = fParamTmp_tmp + QuadModelv_P.Constant_Value_fi;
          }

          // Assignment: '<S460>/Assignment' incorporates:
          //   Sum: '<S460>/Sum'
          //   UnitDelay: '<S460>/Unit Delay'

          if (ForIterator_IterationMarker[2] < 2) {
            ForIterator_IterationMarker[2] = 2U;
            std::memcpy(&QuadModelv_B.Assignment_g[0],
                        &QuadModelv_DW.UnitDelay_DSTATE_i[0], 169U * sizeof
                        (real_T));
          }

          QuadModelv_B.Assignment_g[(qY_0 + 13 * (qY_1 - 1)) - 1] =
            QuadModelv_B.Merge1_a;

          // End of Assignment: '<S460>/Assignment'

          // Assignment: '<S460>/Assignment_snorm'
          if (ForIterator_IterationMarker[3] < 2) {
            ForIterator_IterationMarker[3] = 2U;

            // Assignment: '<S460>/Assignment_snorm' incorporates:
            //   UnitDelay: '<S460>/Unit Delay1'

            std::memcpy(&QuadModelv_B.Assignment_snorm[0],
                        &QuadModelv_DW.UnitDelay1_DSTATE_j[0], 169U * sizeof
                        (real_T));
          }

          // Sum: '<S477>/Sum2' incorporates:
          //   Constant: '<S477>/Constant'
          //   Sum: '<S460>/Sum'

          if ((qY_0 >= 0) && (QuadModelv_P.Constant_Value_lp < qY_0 -
                              MAX_int32_T)) {
            i = MAX_int32_T;
          } else if ((qY_0 < 0) && (QuadModelv_P.Constant_Value_lp > qY_0 -
                      MIN_int32_T)) {
            i = MIN_int32_T;
          } else {
            i = qY_0 - QuadModelv_P.Constant_Value_lp;
          }

          // Sum: '<S477>/Sum1' incorporates:
          //   Gain: '<S477>/Gain'
          //   Sum: '<S460>/Sum'
          //   Sum: '<S477>/Sum2'

          i = mul_s32_sat(QuadModelv_P.Gain_Gain_ob, i);
          if ((qY_1 < 0) && (i < MIN_int32_T - qY_1)) {
            i = MIN_int32_T;
          } else if ((qY_1 > 0) && (i > MAX_int32_T - qY_1)) {
            i = MAX_int32_T;
          } else {
            i += qY_1;
          }

          // Assignment: '<S460>/Assignment_snorm' incorporates:
          //   Sum: '<S477>/Sum1'

          QuadModelv_B.Assignment_snorm[i - 1] = QuadModelv_B.Merge_i;

          // Update for UnitDelay: '<S460>/Unit Delay' incorporates:
          //   Assignment: '<S460>/Assignment'

          std::memcpy(&QuadModelv_DW.UnitDelay_DSTATE_i[0],
                      &QuadModelv_B.Assignment_g[0], 169U * sizeof(real_T));

          // Update for UnitDelay: '<S460>/Unit Delay1'
          std::memcpy(&QuadModelv_DW.UnitDelay1_DSTATE_j[0],
                      &QuadModelv_B.Assignment_snorm[0], 169U * sizeof(real_T));
        }

        // End of SignalConversion generated from: '<S460>/Enable'
        // End of Outputs for SubSystem: '<S458>/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations' 

        // Selector: '<S459>/snorm[n+m*13]' incorporates:
        //   Gain: '<S463>/Gain'
        //   Sum: '<S458>/Sum1'
        //   Sum: '<S463>/Sum1'

        i = mul_s32_sat(QuadModelv_P.Gain_Gain_ij, fParamTmp_tmp);
        if ((qY < 0) && (i < MIN_int32_T - qY)) {
          i = MIN_int32_T;
        } else if ((qY > 0) && (i > MAX_int32_T - qY)) {
          i = MAX_int32_T;
        } else {
          i += qY;
        }

        // Product: '<S459>/par' incorporates:
        //   Selector: '<S459>/snorm[n+m*13]'
        //   Sum: '<S463>/Sum1'

        uavlostJyz = QuadModelv_B.Assignment_snorm[i - 1] * rtb_q3dot;

        // Outputs for Enabled SubSystem: '<S459>/Special case - North//South Geographic Pole' incorporates:
        //   EnablePort: '<S462>/Enable'

        // Logic: '<S465>/Logical Operator' incorporates:
        //   Constant: '<S465>/Constant'
        //   Constant: '<S465>/Constant1'
        //   RelationalOperator: '<S465>/Relational Operator'
        //   RelationalOperator: '<S465>/Relational Operator1'
        //   Sum: '<S458>/Sum1'

        if ((QuadModelv_B.sqrt_d == QuadModelv_P.Constant1_Value_h) &&
            (QuadModelv_P.Constant_Value_a == fParamTmp_tmp)) {
          QuadModelv_DW.SpecialcaseNorthSouthGeographic = true;

          // If: '<S462>/n ==1' incorporates:
          //   Assignment: '<S467>/Assignment2'

          if (r == 1) {
            // Outputs for IfAction SubSystem: '<S462>/If Action Subsystem1' incorporates:
            //   ActionPort: '<S466>/Action Port'

            // Assignment: '<S466>/Assignment2' incorporates:
            //   Constant: '<S466>/Constant'
            //   Selector: '<S466>/pp[n-1]'
            //   Sum: '<S466>/Sum2'
            //   UnitDelay: '<S462>/Unit Delay1'

            if (ForIterator_IterationMarker[0] < 2) {
              ForIterator_IterationMarker[0] = 2U;
              std::memcpy(&QuadModelv_B.Assignment2_a[0],
                          &QuadModelv_DW.UnitDelay1_DSTATE_a[0], 13U * sizeof
                          (real_T));
            }

            QuadModelv_B.Assignment2_a[static_cast<int32_T>
              (QuadModelv_P.Constant_Value_k + 1.0) - 1] =
              QuadModelv_DW.UnitDelay1_DSTATE_a[0];

            // End of Assignment: '<S466>/Assignment2'
            // End of Outputs for SubSystem: '<S462>/If Action Subsystem1'
          } else {
            // Outputs for IfAction SubSystem: '<S462>/If Action Subsystem2' incorporates:
            //   ActionPort: '<S467>/Action Port'

            if (ForIterator_IterationMarker[1] < 2) {
              // Assignment: '<S467>/Assignment2'
              ForIterator_IterationMarker[1] = 2U;

              // Assignment: '<S467>/Assignment2' incorporates:
              //   UnitDelay: '<S462>/Unit Delay1'

              std::memcpy(&QuadModelv_B.Assignment2_m[0],
                          &QuadModelv_DW.UnitDelay1_DSTATE_a[0], 13U * sizeof
                          (real_T));
            }

            // Sum: '<S467>/Sum2' incorporates:
            //   Constant: '<S467>/Constant'

            if ((r < 0) && (QuadModelv_P.Constant_Value_mh < MIN_int32_T - r)) {
              i = MIN_int32_T;
            } else if ((r > 0) && (QuadModelv_P.Constant_Value_mh > MAX_int32_T
                                   - r)) {
              i = MAX_int32_T;
            } else {
              i = r + QuadModelv_P.Constant_Value_mh;
            }

            // Sum: '<S469>/Sum' incorporates:
            //   Constant: '<S469>/Constant'

            if ((fParamTmp_tmp < 0) && (QuadModelv_P.Constant_Value_op <
                 MIN_int32_T - fParamTmp_tmp)) {
              tmp_2 = MIN_int32_T;
            } else if ((fParamTmp_tmp > 0) && (QuadModelv_P.Constant_Value_op >
                        MAX_int32_T - fParamTmp_tmp)) {
              tmp_2 = MAX_int32_T;
            } else {
              tmp_2 = fParamTmp_tmp + QuadModelv_P.Constant_Value_op;
            }

            if ((r < 0) && (QuadModelv_P.Constant_Value_op < MIN_int32_T - r)) {
              qY_1 = MIN_int32_T;
            } else if ((r > 0) && (QuadModelv_P.Constant_Value_op > MAX_int32_T
                                   - r)) {
              qY_1 = MAX_int32_T;
            } else {
              qY_1 = r + QuadModelv_P.Constant_Value_op;
            }

            // Selector: '<S467>/pp[n-2] pp[n-1]' incorporates:
            //   Constant: '<S468>/Constant1'
            //   Sum: '<S468>/Sum2'

            if ((r >= 0) && (QuadModelv_P.Constant1_Value_l < r - MAX_int32_T))
            {
              qY_0 = MAX_int32_T;
            } else if ((r < 0) && (QuadModelv_P.Constant1_Value_l > r -
                                   MIN_int32_T)) {
              qY_0 = MIN_int32_T;
            } else {
              qY_0 = r - QuadModelv_P.Constant1_Value_l;
            }

            // Assignment: '<S467>/Assignment2' incorporates:
            //   Constant: '<S467>/k[13][13]'
            //   Product: '<S467>/Product1'
            //   Product: '<S467>/Product2'
            //   Selector: '<S467>/Selector2'
            //   Selector: '<S467>/pp[n-2] pp[n-1]'
            //   Sum: '<S467>/Sum1'
            //   Sum: '<S467>/Sum2'
            //   Sum: '<S468>/Sum2'
            //   Sum: '<S469>/Sum'
            //   UnitDelay: '<S462>/Unit Delay1'

            QuadModelv_B.Assignment2_m[i - 1] =
              QuadModelv_DW.UnitDelay1_DSTATE_a[r - 1] * QuadModelv_B.Product4_p
              - QuadModelv_P.k1313_Value[((qY_1 - 1) * 13 + tmp_2) - 1] *
              QuadModelv_DW.UnitDelay1_DSTATE_a[qY_0 - 1];

            // End of Outputs for SubSystem: '<S462>/If Action Subsystem2'
          }

          // End of If: '<S462>/n ==1'

          // SignalConversion generated from: '<S462>/pp[n]' incorporates:
          //   UnitDelay: '<S462>/Unit Delay1'

          rtb_TmpSignalConversionAtppnInp[0] =
            QuadModelv_DW.UnitDelay1_DSTATE_a[0];
          rtb_TmpSignalConversionAtppnInp[1] = QuadModelv_B.Assignment2_a[1];
          std::memcpy(&rtb_TmpSignalConversionAtppnInp[2],
                      &QuadModelv_B.Assignment2_m[2], 11U * sizeof(real_T));

          // Product: '<S462>/Product2' incorporates:
          //   Constant: '<S462>/Constant'
          //   Constant: '<S462>/Constant1'
          //   Selector: '<S462>/pp[n]'
          //   Sum: '<S462>/Sum2'

          QuadModelv_B.Product2_b = rtb_TmpSignalConversionAtppnInp
            [static_cast<int32_T>(static_cast<real_T>(r) +
            QuadModelv_P.Constant1_Value_i) - 1] * rtb_q3dot *
            QuadModelv_P.Constant_Value_p * Merge1;

          // Update for UnitDelay: '<S462>/Unit Delay1'
          std::memcpy(&QuadModelv_DW.UnitDelay1_DSTATE_a[0],
                      &rtb_TmpSignalConversionAtppnInp[0], 13U * sizeof(real_T));
        } else if (QuadModelv_DW.SpecialcaseNorthSouthGeographic) {
          // Disable for Product: '<S462>/Product2' incorporates:
          //   Outport: '<S462>/bpp'

          QuadModelv_B.Product2_b = QuadModelv_P.bpp_Y0;
          QuadModelv_DW.SpecialcaseNorthSouthGeographic = false;
        }

        // End of Logic: '<S465>/Logical Operator'
        // End of Outputs for SubSystem: '<S459>/Special case - North//South Geographic Pole' 

        // Sum: '<S459>/Sum' incorporates:
        //   Constant: '<S459>/Constant'
        //   Sum: '<S458>/Sum1'

        if ((fParamTmp_tmp < 0) && (QuadModelv_P.Constant_Value_j < MIN_int32_T
             - fParamTmp_tmp)) {
          i = MIN_int32_T;
        } else if ((fParamTmp_tmp > 0) && (QuadModelv_P.Constant_Value_j >
                    MAX_int32_T - fParamTmp_tmp)) {
          i = MAX_int32_T;
        } else {
          i = fParamTmp_tmp + QuadModelv_P.Constant_Value_j;
        }

        // Sum: '<S459>/Sum1' incorporates:
        //   Assignment: '<S460>/Assignment'
        //   Product: '<S459>/Product'
        //   Selector: '<S459>/dp[n][m]'
        //   Sum: '<S459>/Sum'
        //   UnitDelay: '<S459>/Unit Delay1'

        QuadModelv_B.Sum1_f = rtb_Rn - QuadModelv_B.Assignment_g[((tmp_1 - 1) *
          13 + i) - 1] * U1 * rtb_q3dot;

        // Sum: '<S459>/Sum4' incorporates:
        //   Constant: '<S459>/Constant1'
        //   Sum: '<S458>/Sum1'

        if ((fParamTmp_tmp < 0) && (QuadModelv_P.Constant1_Value_ja <
             MIN_int32_T - fParamTmp_tmp)) {
          i = MIN_int32_T;
        } else if ((fParamTmp_tmp > 0) && (QuadModelv_P.Constant1_Value_ja >
                    MAX_int32_T - fParamTmp_tmp)) {
          i = MAX_int32_T;
        } else {
          i = fParamTmp_tmp + QuadModelv_P.Constant1_Value_ja;
        }

        // Sum: '<S459>/Sum2' incorporates:
        //   Constant: '<S459>/fm'
        //   Product: '<S459>/Product1'
        //   Selector: '<S459>/fm[m]'
        //   Sum: '<S459>/Sum4'
        //   UnitDelay: '<S459>/Unit Delay3'

        QuadModelv_B.Sum2_k = QuadModelv_P.fm_Value[i - 1] * uavlostJyz * Merge1
          + rtb_TmpSignalConversionAtSFun_1;

        // Sum: '<S459>/Sum3' incorporates:
        //   Constant: '<S459>/fn'
        //   Product: '<S459>/Product2'
        //   Selector: '<S459>/fn[m]'
        //   Sum: '<S459>/Sum4'
        //   UnitDelay: '<S459>/Unit Delay2'

        QuadModelv_B.Sum3 = QuadModelv_P.fn_Value[tmp_3 - 1] * uavlostJyz * U1 +
          rtb_q2dot;

        // Sum: '<S459>/Sum5' incorporates:
        //   UnitDelay: '<S459>/Unit Delay4'

        QuadModelv_B.Sum5 = X + QuadModelv_B.Product2_b;

        // Update for UnitDelay: '<S459>/Unit Delay1'
        rtb_Rn = QuadModelv_B.Sum1_f;

        // Update for UnitDelay: '<S459>/Unit Delay3'
        rtb_TmpSignalConversionAtSFun_1 = QuadModelv_B.Sum2_k;

        // Update for UnitDelay: '<S459>/Unit Delay2'
        rtb_q2dot = QuadModelv_B.Sum3;

        // Update for UnitDelay: '<S459>/Unit Delay4'
        X = QuadModelv_B.Sum5;
      }

      // End of Outputs for SubSystem: '<S450>/For Iterator Subsystem'

      // Sum: '<S450>/Sum1' incorporates:
      //   UnitDelay: '<S450>/Unit Delay2'

      QuadModelv_B.Sum1_l[0] = tz_idx_0 + QuadModelv_B.Sum1_f;
      QuadModelv_B.Sum1_l[1] = tz_idx_1 + QuadModelv_B.Sum2_k;
      QuadModelv_B.Sum1_l[2] = tz_idx_2 + QuadModelv_B.Sum3;
      QuadModelv_B.Sum1_l[3] = tz_idx_3 + QuadModelv_B.Sum5;

      // Update for UnitDelay: '<S450>/Unit Delay'
      uavlostJyz = rtb_q3dot;

      // Update for UnitDelay: '<S450>/Unit Delay2'
      tz_idx_0 = QuadModelv_B.Sum1_l[0];
      tz_idx_1 = QuadModelv_B.Sum1_l[1];
      tz_idx_2 = QuadModelv_B.Sum1_l[2];
      tz_idx_3 = QuadModelv_B.Sum1_l[3];
    }

    // End of Outputs for SubSystem: '<S442>/Compute magnetic vector in spherical coordinates' 
  }

  // Switch: '<S502>/Switch' incorporates:
  //   Product: '<S502>/Product'

  if (QuadModelv_B.sqrt_d != 0.0) {
    rtb_Rn = QuadModelv_B.Sum1_l[1] / QuadModelv_B.sqrt_d;
  } else {
    rtb_Rn = QuadModelv_B.Sum1_l[3];
  }

  // End of Switch: '<S502>/Switch'

  // Sum: '<S501>/Sum1' incorporates:
  //   Product: '<S501>/Product1'
  //   Product: '<S501>/Product4'

  rtb_TmpSignalConversionAtSFun_1 = (0.0 - QuadModelv_B.Product11 *
    QuadModelv_B.Sum1_l[0]) - QuadModelv_B.Sum1_l[2] * QuadModelv_B.Product12;

  // UnitConversion: '<S506>/Unit Conversion' incorporates:
  //   Trigonometry: '<S504>/Trigonometric Function1'

  // Unit Conversion - from: rad to: deg
  // Expression: output = (57.2958*input) + (0)
  rtb_q3dot = 57.295779513082323 * rt_atan2d_snf(rtb_Rn,
    rtb_TmpSignalConversionAtSFun_1);

  // Sum: '<S503>/Sum1' incorporates:
  //   Product: '<S503>/Product1'
  //   Product: '<S503>/Product4'

  rtb_q2dot = QuadModelv_B.Product12 * QuadModelv_B.Sum1_l[0] -
    QuadModelv_B.Sum1_l[2] * QuadModelv_B.Product11;

  // Sum: '<S504>/Sum' incorporates:
  //   Product: '<S504>/Product'
  //   Product: '<S504>/Product1'

  rtb_Rn = rtb_Rn * rtb_Rn + rtb_TmpSignalConversionAtSFun_1 *
    rtb_TmpSignalConversionAtSFun_1;

  // UnitConversion: '<S443>/Unit Conversion' incorporates:
  //   Sqrt: '<S504>/sqrt1'
  //   Trigonometry: '<S504>/Trigonometric Function'
  //   UnitConversion: '<S505>/Unit Conversion'

  // Unit Conversion - from: rad to: deg
  // Expression: output = (57.2958*input) + (0)
  // Unit Conversion - from: deg to: rad
  // Expression: output = (0.0174533*input) + (0)
  uavlostJyz = 0.017453292519943295 * rtb_q3dot;
  X = 57.295779513082323 * rt_atan2d_snf(rtb_q2dot, std::sqrt(rtb_Rn)) *
    0.017453292519943295;

  // Sqrt: '<S504>/sqrt' incorporates:
  //   Product: '<S504>/Product2'
  //   Sum: '<S504>/Sum1'

  rtb_Rn = std::sqrt(rtb_q2dot * rtb_q2dot + rtb_Rn);

  // Product: '<S436>/h1' incorporates:
  //   Trigonometry: '<S436>/sincos'

  rtb_TmpSignalConversionAtSFun_1 = std::cos(X) * rtb_Rn;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // MATLAB Function: '<S398>/FaultParamsExtract2' incorporates:
    //   Constant: '<Root>/Constant1'
    //   Constant: '<S398>/FaultID2'

    // MATLAB Function 'SensorFault/FaultParamsExtract2': '<S403>:1'
    // '<S403>:1:5' if isempty(hFault)
    // '<S403>:1:8' if isempty(fParam)
    // '<S403>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S403>:1:13' fParamTmp=zeros(20,1);
    std::memset(&fParamTmp[0], 0, 20U * sizeof(real_T));

    // '<S403>:1:14' j=1;
    j = 1;

    // '<S403>:1:15' for i=1:8
    for (r = 0; r < 8; r++) {
      // '<S403>:1:16' if inInts(i) == FaultID
      if (QuadModelv_B.inSILInts[r] == QuadModelv_P.FaultID2_Value_d) {
        // '<S403>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S403>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        s458_iter = (r + 1) << 1;
        fParamTmp_tmp = j << 1;
        fParamTmp[fParamTmp_tmp - 2] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 2];

        // '<S403>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[fParamTmp_tmp - 1] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 1];

        // '<S403>:1:20' j=j+1;
        j++;
      }
    }

    // '<S403>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S403>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_g = true;

      // '<S403>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = QuadModelv_P.Constant1_Value_a[16];
      fParamTmp[17] = QuadModelv_P.Constant1_Value_a[17];
      fParamTmp[18] = QuadModelv_P.Constant1_Value_a[18];
      fParamTmp[19] = QuadModelv_P.Constant1_Value_a[19];

      // '<S403>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_p[0], &fParamTmp[0], 20U * sizeof(real_T));
    }

    // SignalConversion generated from: '<S398>/Matrix Multiply2' incorporates:
    //   Product: '<S436>/x1'
    //   Product: '<S436>/y1'
    //   Product: '<S436>/z1'
    //   Trigonometry: '<S436>/sincos'

    // '<S403>:1:29' hasFault_mag=hFault;
    // '<S403>:1:30' FaultParam=fParam;
    rtb_q2dot = std::cos(uavlostJyz) * rtb_TmpSignalConversionAtSFun_1;
    rtb_TmpSignalConversionAtSFun_1 *= std::sin(uavlostJyz);
    rtb_Rn *= std::sin(X);

    // Product: '<S398>/Matrix Multiply2' incorporates:
    //   Concatenate: '<S36>/Vector Concatenate'

    for (i = 0; i < 3; i++) {
      VectorConcatenate_0[i] = (VectorConcatenate[i + 3] *
        rtb_TmpSignalConversionAtSFun_1 + VectorConcatenate[i] * rtb_q2dot) +
        VectorConcatenate[i + 6] * rtb_Rn;
    }

    // End of Product: '<S398>/Matrix Multiply2'

    // MATLAB Function: '<S398>/Mag NoiseFun' incorporates:
    //   Gain: '<S398>/Gain_Mag'
    //   Gain: '<S398>/nT2Gauss'
    //   MATLAB Function: '<S398>/FaultParamsExtract2'

    rtb_DataTypeConversion1[0] = QuadModelv_P.Gain_Mag_Gain *
      VectorConcatenate_0[0] * QuadModelv_P.nT2Gauss_Gain;
    rtb_DataTypeConversion1[1] = QuadModelv_P.Gain_Mag_Gain *
      VectorConcatenate_0[1] * QuadModelv_P.nT2Gauss_Gain;
    rtb_DataTypeConversion1[2] = QuadModelv_P.Gain_Mag_Gain *
      VectorConcatenate_0[2] * QuadModelv_P.nT2Gauss_Gain;

    // MATLAB Function 'SensorFault/Mag NoiseFun': '<S409>:1'
    // '<S409>:1:2' y = MagIn;
    // '<S409>:1:4' if isMagFault
    if (QuadModelv_DW.hFault_g) {
      //  FaultParams(1)为标志位
      // '<S409>:1:6' flag = MagFaultParams(1);
      //  覆盖模式，当为-1时，保持原有输出
      // '<S409>:1:9' if flag == 1
      if (QuadModelv_DW.fParam_p[0] == 1.0) {
        // '<S409>:1:10' for ind = 1:3
        // '<S409>:1:11' if MagFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_p[1] != -1.0) {
          // '<S409>:1:12' y(ind) = MagFaultParams(ind+1);
          rtb_DataTypeConversion1[0] = QuadModelv_DW.fParam_p[1];
        }

        // '<S409>:1:11' if MagFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_p[2] != -1.0) {
          // '<S409>:1:12' y(ind) = MagFaultParams(ind+1);
          rtb_DataTypeConversion1[1] = QuadModelv_DW.fParam_p[2];
        }

        // '<S409>:1:11' if MagFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_p[3] != -1.0) {
          // '<S409>:1:12' y(ind) = MagFaultParams(ind+1);
          rtb_DataTypeConversion1[2] = QuadModelv_DW.fParam_p[3];
        }

        //  叠加模式，当为-1时，保持原有输出
      } else if (QuadModelv_DW.fParam_p[0] == 2.0) {
        // '<S409>:1:17' elseif flag == 2
        // '<S409>:1:18' for ind = 1:3
        // '<S409>:1:19' if MagFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_p[1] != -1.0) {
          // '<S409>:1:20' y(ind) = y(ind) + MagFaultParams(ind+1);
          rtb_DataTypeConversion1[0] += QuadModelv_DW.fParam_p[1];
        }

        // '<S409>:1:19' if MagFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_p[2] != -1.0) {
          // '<S409>:1:20' y(ind) = y(ind) + MagFaultParams(ind+1);
          rtb_DataTypeConversion1[1] += QuadModelv_DW.fParam_p[2];
        }

        // '<S409>:1:19' if MagFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_p[3] != -1.0) {
          // '<S409>:1:20' y(ind) = y(ind) + MagFaultParams(ind+1);
          rtb_DataTypeConversion1[2] += QuadModelv_DW.fParam_p[3];
        }

        //  噪声模式，当为-1时，保持原有输出
      } else if (QuadModelv_DW.fParam_p[0] == 3.0) {
        // '<S409>:1:25' elseif flag == 3
        // '<S409>:1:26' mean = MagFaultParams(2);
        // '<S409>:1:27' stddev = MagFaultParams(3);
        // '<S409>:1:28' for ind = 1:3
        // '<S409>:1:29' if MagFaultParams(ind+3) ~= -1
        if (QuadModelv_DW.fParam_p[3] != -1.0) {
          // '<S409>:1:30' y(ind) = y(ind) + generate_wgn(mean,stddev);
          // '<S409>:1:50' if isempty(phase)
          // '<S409>:1:57' if phase
          if (QuadModelv_DW.phase_b) {
            // '<S409>:1:58' while true
            do {
              // '<S409>:1:59' U1 = rand();
              U1 = QuadModelv_rand_ah();

              // '<S409>:1:60' U2 = rand();
              X = QuadModelv_rand_ah();

              // '<S409>:1:61' V1 = 2.0 * U1 - 1.0;
              U1 = 2.0 * U1 - 1.0;

              // '<S409>:1:62' V2 = 2.0 * U2 - 1.0;
              QuadModelv_DW.V2_i = 2.0 * X - 1.0;

              // '<S409>:1:63' S = V1 * V1 + V2 * V2;
              QuadModelv_DW.S_g = U1 * U1 + QuadModelv_DW.V2_i *
                QuadModelv_DW.V2_i;

              // '<S409>:1:64' if S < 1.0 && abs(S) >= 1e-8
            } while ((!(QuadModelv_DW.S_g < 1.0)) || (!(std::abs
                       (QuadModelv_DW.S_g) >= 1.0E-8)));

            // '<S409>:1:68' X = V1 * sqrt(-2.0 * log(S) / S);
            X = std::sqrt(-2.0 * std::log(QuadModelv_DW.S_g) / QuadModelv_DW.S_g)
              * U1;
          } else {
            // '<S409>:1:69' else
            // '<S409>:1:70' X = V2 * sqrt(-2.0 * log(S) / S);
            X = std::sqrt(-2.0 * std::log(QuadModelv_DW.S_g) / QuadModelv_DW.S_g)
              * QuadModelv_DW.V2_i;
          }

          // '<S409>:1:73' y = mean + stddev * X;
          // '<S409>:1:75' phase = ~phase;
          QuadModelv_DW.phase_b = !QuadModelv_DW.phase_b;
          rtb_DataTypeConversion1[0] += QuadModelv_DW.fParam_p[2] * X +
            QuadModelv_DW.fParam_p[1];
        }

        // '<S409>:1:29' if MagFaultParams(ind+3) ~= -1
        if (QuadModelv_DW.fParam_p[4] != -1.0) {
          // '<S409>:1:30' y(ind) = y(ind) + generate_wgn(mean,stddev);
          // '<S409>:1:50' if isempty(phase)
          // '<S409>:1:57' if phase
          if (QuadModelv_DW.phase_b) {
            // '<S409>:1:58' while true
            do {
              // '<S409>:1:59' U1 = rand();
              U1 = QuadModelv_rand_ah();

              // '<S409>:1:60' U2 = rand();
              X = QuadModelv_rand_ah();

              // '<S409>:1:61' V1 = 2.0 * U1 - 1.0;
              U1 = 2.0 * U1 - 1.0;

              // '<S409>:1:62' V2 = 2.0 * U2 - 1.0;
              QuadModelv_DW.V2_i = 2.0 * X - 1.0;

              // '<S409>:1:63' S = V1 * V1 + V2 * V2;
              QuadModelv_DW.S_g = U1 * U1 + QuadModelv_DW.V2_i *
                QuadModelv_DW.V2_i;

              // '<S409>:1:64' if S < 1.0 && abs(S) >= 1e-8
            } while ((!(QuadModelv_DW.S_g < 1.0)) || (!(std::abs
                       (QuadModelv_DW.S_g) >= 1.0E-8)));

            // '<S409>:1:68' X = V1 * sqrt(-2.0 * log(S) / S);
            X = std::sqrt(-2.0 * std::log(QuadModelv_DW.S_g) / QuadModelv_DW.S_g)
              * U1;
          } else {
            // '<S409>:1:69' else
            // '<S409>:1:70' X = V2 * sqrt(-2.0 * log(S) / S);
            X = std::sqrt(-2.0 * std::log(QuadModelv_DW.S_g) / QuadModelv_DW.S_g)
              * QuadModelv_DW.V2_i;
          }

          // '<S409>:1:73' y = mean + stddev * X;
          // '<S409>:1:75' phase = ~phase;
          QuadModelv_DW.phase_b = !QuadModelv_DW.phase_b;
          rtb_DataTypeConversion1[1] += QuadModelv_DW.fParam_p[2] * X +
            QuadModelv_DW.fParam_p[1];
        }

        // '<S409>:1:29' if MagFaultParams(ind+3) ~= -1
        if (QuadModelv_DW.fParam_p[5] != -1.0) {
          // '<S409>:1:30' y(ind) = y(ind) + generate_wgn(mean,stddev);
          // '<S409>:1:50' if isempty(phase)
          // '<S409>:1:57' if phase
          if (QuadModelv_DW.phase_b) {
            // '<S409>:1:58' while true
            do {
              // '<S409>:1:59' U1 = rand();
              U1 = QuadModelv_rand_ah();

              // '<S409>:1:60' U2 = rand();
              X = QuadModelv_rand_ah();

              // '<S409>:1:61' V1 = 2.0 * U1 - 1.0;
              U1 = 2.0 * U1 - 1.0;

              // '<S409>:1:62' V2 = 2.0 * U2 - 1.0;
              QuadModelv_DW.V2_i = 2.0 * X - 1.0;

              // '<S409>:1:63' S = V1 * V1 + V2 * V2;
              QuadModelv_DW.S_g = U1 * U1 + QuadModelv_DW.V2_i *
                QuadModelv_DW.V2_i;

              // '<S409>:1:64' if S < 1.0 && abs(S) >= 1e-8
            } while ((!(QuadModelv_DW.S_g < 1.0)) || (!(std::abs
                       (QuadModelv_DW.S_g) >= 1.0E-8)));

            // '<S409>:1:68' X = V1 * sqrt(-2.0 * log(S) / S);
            X = std::sqrt(-2.0 * std::log(QuadModelv_DW.S_g) / QuadModelv_DW.S_g)
              * U1;
          } else {
            // '<S409>:1:69' else
            // '<S409>:1:70' X = V2 * sqrt(-2.0 * log(S) / S);
            X = std::sqrt(-2.0 * std::log(QuadModelv_DW.S_g) / QuadModelv_DW.S_g)
              * QuadModelv_DW.V2_i;
          }

          // '<S409>:1:73' y = mean + stddev * X;
          // '<S409>:1:75' phase = ~phase;
          QuadModelv_DW.phase_b = !QuadModelv_DW.phase_b;
          rtb_DataTypeConversion1[2] += QuadModelv_DW.fParam_p[2] * X +
            QuadModelv_DW.fParam_p[1];
        }

        //  尺度因子模式，当为-1时，保持原有输出
      } else if (QuadModelv_DW.fParam_p[0] == 4.0) {
        // '<S409>:1:35' elseif flag == 4
        // '<S409>:1:36' for ind = 1:3
        // '<S409>:1:37' if MagFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_p[1] != -1.0) {
          // '<S409>:1:38' y(ind) = y(ind) * MagFaultParams(ind+1);
          rtb_DataTypeConversion1[0] *= QuadModelv_DW.fParam_p[1];
        }

        // '<S409>:1:37' if MagFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_p[2] != -1.0) {
          // '<S409>:1:38' y(ind) = y(ind) * MagFaultParams(ind+1);
          rtb_DataTypeConversion1[1] *= QuadModelv_DW.fParam_p[2];
        }

        // '<S409>:1:37' if MagFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_p[3] != -1.0) {
          // '<S409>:1:38' y(ind) = y(ind) * MagFaultParams(ind+1);
          rtb_DataTypeConversion1[2] *= QuadModelv_DW.fParam_p[3];
        }
      }
    }

    // End of MATLAB Function: '<S398>/Mag NoiseFun'

    // DataTypeConversion: '<S398>/Data Type Conversion4'
    QuadModelv_B.DataTypeConversion4[0] = static_cast<real32_T>
      (rtb_DataTypeConversion1[0]);
    QuadModelv_B.DataTypeConversion4[1] = static_cast<real32_T>
      (rtb_DataTypeConversion1[1]);
    QuadModelv_B.DataTypeConversion4[2] = static_cast<real32_T>
      (rtb_DataTypeConversion1[2]);
  }

  // Saturate: '<S64>/Limit  altitude  to troposhere'
  if (rtb_Saturation_1 > QuadModelv_P.Limitaltitudetotroposhere_Upper) {
    rtb_sincos_o1_idx_1 = QuadModelv_P.Limitaltitudetotroposhere_Upper;
  } else if (rtb_Saturation_1 < QuadModelv_P.Limitaltitudetotroposhere_Lower) {
    rtb_sincos_o1_idx_1 = QuadModelv_P.Limitaltitudetotroposhere_Lower;
  } else {
    rtb_sincos_o1_idx_1 = rtb_Saturation_1;
  }

  // Sum: '<S64>/Sum1' incorporates:
  //   Constant: '<S64>/Sea Level  Temperature'
  //   Gain: '<S64>/Lapse Rate'
  //   Saturate: '<S64>/Limit  altitude  to troposhere'

  rtb_Rn = QuadModelv_P.SeaLevelTemperature_Value_i -
    QuadModelv_P.LapseRate_Gain * rtb_sincos_o1_idx_1;

  // Gain: '<S64>/1//T0'
  rtb_TmpSignalConversionAtSFun_1 = QuadModelv_P.uT0_Gain * rtb_Rn;

  // Sum: '<S64>/Sum' incorporates:
  //   Constant: '<S64>/Altitude of Troposphere'

  X = QuadModelv_P.AltitudeofTroposphere_Value - rtb_Saturation_1;

  // Math: '<S64>/(T//T0)^(g//LR) ' incorporates:
  //   Constant: '<S64>/Constant'

  if ((rtb_TmpSignalConversionAtSFun_1 < 0.0) && (QuadModelv_P.Constant_Value_mq
       > std::floor(QuadModelv_P.Constant_Value_mq))) {
    rtb_sincos_o1_idx_1 = -rt_powd_snf(-rtb_TmpSignalConversionAtSFun_1,
      QuadModelv_P.Constant_Value_mq);
  } else {
    rtb_sincos_o1_idx_1 = rt_powd_snf(rtb_TmpSignalConversionAtSFun_1,
      QuadModelv_P.Constant_Value_mq);
  }

  // Saturate: '<S64>/Limit  altitude  to Stratosphere'
  if (X > QuadModelv_P.LimitaltitudetoStratosphere_Upp) {
    X = QuadModelv_P.LimitaltitudetoStratosphere_Upp;
  } else if (X < QuadModelv_P.LimitaltitudetoStratosphere_Low) {
    X = QuadModelv_P.LimitaltitudetoStratosphere_Low;
  }

  // Sum: '<S413>/Sum' incorporates:
  //   Product: '<S413>/Product'
  //   Product: '<S413>/Product1'
  //   Product: '<S413>/Product2'
  //   Sum: '<S508>/Sum'

  tz_idx_0 = (rtb_sincos_o1_f[0] * rtb_sincos_o1_f[0] + rtb_sincos_o1_f[1] *
              rtb_sincos_o1_f[1]) + rtb_sincos_o1_f[2] * rtb_sincos_o1_f[2];

  // Product: '<S400>/Product2' incorporates:
  //   Gain: '<S64>/g//R'
  //   Gain: '<S64>/rho0'
  //   Math: '<S64>/(T//T0)^(g//LR) '
  //   Math: '<S64>/Stratosphere Model'
  //   Product: '<S507>/Product2'
  //   Product: '<S64>/Product'
  //   Product: '<S64>/Product1'
  //   Product: '<S64>/Product3'
  //   Saturate: '<S64>/Limit  altitude  to Stratosphere'
  //   Sum: '<S413>/Sum'
  //
  //  About '<S64>/Stratosphere Model':
  //   Operator: exp

  rtb_TmpSignalConversionAtSFun_1 = rtb_sincos_o1_idx_1 /
    rtb_TmpSignalConversionAtSFun_1 * QuadModelv_P.rho0_Gain * std::exp(1.0 /
    rtb_Rn * (QuadModelv_P.gR_Gain * X)) * tz_idx_0;

  // Gain: '<S400>/1//2rhoV^2' incorporates:
  //   Product: '<S400>/Product2'

  rtb_q2dot = rtb_TmpSignalConversionAtSFun_1 * QuadModelv_P.u2rhoV2_Gain;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0) {
    // Product: '<S406>/Product4' incorporates:
    //   Constant: '<S406>/Constant2'
    //   Gain: '<S398>/Gain5'
    //   UniformRandomNumber: '<S398>/Uniform Random Number'

    QuadModelv_B.Product4 = QuadModelv_P.Gain5_Gain *
      QuadModelv_DW.UniformRandomNumber_NextOutpu_l *
      QuadModelv_P.Constant2_Value_oq;
  }

  // Sum: '<S398>/Add2'
  uavlostJyz = (rtb_q2dot + rtb_Saturation_1) + QuadModelv_B.Product4;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // MATLAB Function: '<S398>/FaultParamsExtract3' incorporates:
    //   Constant: '<Root>/Constant1'
    //   Constant: '<S398>/FaultID3'

    // MATLAB Function 'SensorFault/FaultParamsExtract3': '<S404>:1'
    // '<S404>:1:5' if isempty(hFault)
    // '<S404>:1:8' if isempty(fParam)
    // '<S404>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S404>:1:13' fParamTmp=zeros(20,1);
    std::memset(&fParamTmp[0], 0, 20U * sizeof(real_T));

    // '<S404>:1:14' j=1;
    j = 1;

    // '<S404>:1:15' for i=1:8
    for (r = 0; r < 8; r++) {
      // '<S404>:1:16' if inInts(i) == FaultID
      if (QuadModelv_B.inSILInts[r] == QuadModelv_P.FaultID3_Value_l) {
        // '<S404>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S404>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        s458_iter = (r + 1) << 1;
        fParamTmp_tmp = j << 1;
        fParamTmp[fParamTmp_tmp - 2] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 2];

        // '<S404>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[fParamTmp_tmp - 1] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 1];

        // '<S404>:1:20' j=j+1;
        j++;
      }
    }

    // '<S404>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S404>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_a = true;

      // '<S404>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = QuadModelv_P.Constant1_Value_a[16];
      fParamTmp[17] = QuadModelv_P.Constant1_Value_a[17];
      fParamTmp[18] = QuadModelv_P.Constant1_Value_a[18];
      fParamTmp[19] = QuadModelv_P.Constant1_Value_a[19];

      // '<S404>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_n[0], &fParamTmp[0], 20U * sizeof(real_T));
    }

    // MATLAB Function: '<S398>/baro NoiseFun' incorporates:
    //   MATLAB Function: '<S398>/FaultParamsExtract3'

    // '<S404>:1:29' hasFault_baro=hFault;
    // '<S404>:1:30' FaultParam=fParam;
    // MATLAB Function 'SensorFault/baro NoiseFun': '<S412>:1'
    // '<S412>:1:2' y = BaroIn;
    QuadModelv_B.y_p = uavlostJyz;

    // '<S412>:1:4' if isBaroFault
    if (QuadModelv_DW.hFault_a) {
      // '<S412>:1:5' flag = BaroFaultParams(1);
      // '<S412>:1:8' if flag == 1
      if (QuadModelv_DW.fParam_n[0] == 1.0) {
        // '<S412>:1:9' y = BaroFaultParams(2);
        QuadModelv_B.y_p = QuadModelv_DW.fParam_n[1];
      } else if (QuadModelv_DW.fParam_n[0] == 2.0) {
        // '<S412>:1:10' elseif flag == 2
        // '<S412>:1:11' y = y + BaroFaultParams(2);
        QuadModelv_B.y_p = uavlostJyz + QuadModelv_DW.fParam_n[1];
      } else if (QuadModelv_DW.fParam_n[0] == 3.0) {
        // '<S412>:1:12' elseif flag == 3
        // '<S412>:1:13' mean = BaroFaultParams(2);
        // '<S412>:1:14' stddev = BaroFaultParams(3);
        // '<S412>:1:15' y = y + generate_wgn(mean,stddev);
        // '<S412>:1:27' if isempty(phase)
        // '<S412>:1:34' if phase
        if (QuadModelv_DW.phase) {
          // '<S412>:1:35' while true
          do {
            // '<S412>:1:36' U1 = rand();
            U1 = QuadModelv_rand_l();

            // '<S412>:1:37' U2 = rand();
            X = QuadModelv_rand_l();

            // '<S412>:1:38' V1 = 2.0 * U1 - 1.0;
            U1 = 2.0 * U1 - 1.0;

            // '<S412>:1:39' V2 = 2.0 * U2 - 1.0;
            QuadModelv_DW.V2 = 2.0 * X - 1.0;

            // '<S412>:1:40' S = V1 * V1 + V2 * V2;
            QuadModelv_DW.S = U1 * U1 + QuadModelv_DW.V2 * QuadModelv_DW.V2;

            // '<S412>:1:41' if S < 1.0 && abs(S) >= 1e-8
          } while ((!(QuadModelv_DW.S < 1.0)) || (!(std::abs(QuadModelv_DW.S) >=
                     1.0E-8)));

          // '<S412>:1:45' X = V1 * sqrt(-2.0 * log(S) / S);
          X = std::sqrt(-2.0 * std::log(QuadModelv_DW.S) / QuadModelv_DW.S) * U1;
        } else {
          // '<S412>:1:46' else
          // '<S412>:1:47' X = V2 * sqrt(-2.0 * log(S) / S);
          X = std::sqrt(-2.0 * std::log(QuadModelv_DW.S) / QuadModelv_DW.S) *
            QuadModelv_DW.V2;
        }

        // '<S412>:1:50' y = mean + stddev * X;
        // '<S412>:1:52' phase = ~phase;
        QuadModelv_DW.phase = !QuadModelv_DW.phase;
        QuadModelv_B.y_p = (QuadModelv_DW.fParam_n[2] * X +
                            QuadModelv_DW.fParam_n[1]) + uavlostJyz;
      } else if (QuadModelv_DW.fParam_n[0] == 4.0) {
        // '<S412>:1:16' elseif flag == 4
        // '<S412>:1:17' y = y * BaroFaultParams(2);
        QuadModelv_B.y_p = uavlostJyz * QuadModelv_DW.fParam_n[1];
      }
    }

    // End of MATLAB Function: '<S398>/baro NoiseFun'

    // Saturate: '<S408>/Limit  altitude  to troposhere'
    if (QuadModelv_B.y_p > QuadModelv_P.Limitaltitudetotroposhere_Upp_h) {
      rtb_sincos_o1_idx_1 = QuadModelv_P.Limitaltitudetotroposhere_Upp_h;
    } else if (QuadModelv_B.y_p < QuadModelv_P.Limitaltitudetotroposhere_Low_g)
    {
      rtb_sincos_o1_idx_1 = QuadModelv_P.Limitaltitudetotroposhere_Low_g;
    } else {
      rtb_sincos_o1_idx_1 = QuadModelv_B.y_p;
    }

    // Sum: '<S408>/Sum1' incorporates:
    //   Constant: '<S408>/Sea Level  Temperature'
    //   Gain: '<S408>/Lapse Rate'
    //   Saturate: '<S408>/Limit  altitude  to troposhere'

    uavlostJyz = QuadModelv_P.SeaLevelTemperature_Value -
      QuadModelv_P.LapseRate_Gain_g * rtb_sincos_o1_idx_1;

    // Gain: '<S408>/1//T0'
    rtb_Product1_d = QuadModelv_P.uT0_Gain_d * uavlostJyz;

    // Sum: '<S408>/Sum' incorporates:
    //   Constant: '<S408>/Altitude of Troposphere'

    X = QuadModelv_P.AltitudeofTroposphere_Value_f - QuadModelv_B.y_p;

    // Math: '<S408>/(T//T0)^(g//LR) ' incorporates:
    //   Constant: '<S408>/Constant'

    if ((rtb_Product1_d < 0.0) && (QuadModelv_P.Constant_Value_o > std::floor
         (QuadModelv_P.Constant_Value_o))) {
      rtb_sincos_o1_idx_1 = -rt_powd_snf(-rtb_Product1_d,
        QuadModelv_P.Constant_Value_o);
    } else {
      rtb_sincos_o1_idx_1 = rt_powd_snf(rtb_Product1_d,
        QuadModelv_P.Constant_Value_o);
    }

    // Saturate: '<S408>/Limit  altitude  to Stratosphere'
    if (X > QuadModelv_P.LimitaltitudetoStratosphere_U_p) {
      X = QuadModelv_P.LimitaltitudetoStratosphere_U_p;
    } else if (X < QuadModelv_P.LimitaltitudetoStratosphere_L_f) {
      X = QuadModelv_P.LimitaltitudetoStratosphere_L_f;
    }

    // Gain: '<S398>/Gain' incorporates:
    //   Gain: '<S408>/P0'
    //   Gain: '<S408>/g//R'
    //   Math: '<S408>/(T//T0)^(g//LR) '
    //   Math: '<S408>/Stratosphere Model'
    //   Product: '<S408>/Product1'
    //   Product: '<S408>/Product2'
    //   Saturate: '<S408>/Limit  altitude  to Stratosphere'
    //
    //  About '<S408>/Stratosphere Model':
    //   Operator: exp

    QuadModelv_B.Gain_k = std::exp(1.0 / uavlostJyz * (QuadModelv_P.gR_Gain_d *
      X)) * (QuadModelv_P.P0_Gain * rtb_sincos_o1_idx_1) *
      QuadModelv_P.Gain_Gain_io;
  }

  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0) {
    // Product: '<S407>/Product4' incorporates:
    //   Constant: '<S407>/Constant'
    //   Constant: '<S407>/Constant2'
    //   Gain: '<S398>/Gain9'
    //   Gain: '<S407>/Gain2'
    //   Sum: '<S407>/Sum'
    //   UniformRandomNumber: '<S398>/Uniform Random Number4'

    QuadModelv_B.Product4_h = (QuadModelv_P.Gain2_Gain_a *
      QuadModelv_P.Constant_Value_ly + QuadModelv_P.Constant2_Value_i) *
      (QuadModelv_P.Gain9_Gain * QuadModelv_DW.UniformRandomNumber4_NextOutput);
  }

  // Unit Conversion - from: K to: degC
  // Expression: output = (1*input) + (-273.15)
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // DataTypeConversion: '<S394>/Data Type Conversion6' incorporates:
    //   Constant: '<S394>/Constant'

    rtb_sincos_o1_idx_1 = std::floor(QuadModelv_P.Constant_Value_gb);
    if (rtIsNaN(rtb_sincos_o1_idx_1) || rtIsInf(rtb_sincos_o1_idx_1)) {
      rtb_sincos_o1_idx_1 = 0.0;
    } else {
      rtb_sincos_o1_idx_1 = std::fmod(rtb_sincos_o1_idx_1, 4.294967296E+9);
    }

    // DataTypeConversion: '<S394>/Data Type Conversion6'
    QuadModelv_B.fields_updated = rtb_sincos_o1_idx_1 < 0.0 ?
      static_cast<uint32_T>(-static_cast<int32_T>(static_cast<uint32_T>
      (-rtb_sincos_o1_idx_1))) : static_cast<uint32_T>(rtb_sincos_o1_idx_1);
  }

  // Outport: '<Root>/MavHILSensor' incorporates:
  //   BusCreator: '<S394>/Bus Creator'
  //   Constant: '<S5>/Constant1'
  //   DataTypeConversion: '<S398>/Data Type Conversion5'
  //   Gain: '<S398>/Gain1'
  //   Sum: '<S398>/Sum'
  //   Sum: '<S5>/Sum2'
  //   UnitConversion: '<S65>/Unit Conversion'

  QuadModelv_Y.MavHILSensor.time_usec = rtb_time_usec;
  QuadModelv_Y.MavHILSensor.xacc = QuadModelv_B.DataTypeConversion2_i[0];
  QuadModelv_Y.MavHILSensor.yacc = QuadModelv_B.DataTypeConversion2_i[1];
  QuadModelv_Y.MavHILSensor.zacc = QuadModelv_B.DataTypeConversion2_i[2];
  QuadModelv_Y.MavHILSensor.xgyro = QuadModelv_B.DataTypeConversion3[0];
  QuadModelv_Y.MavHILSensor.ygyro = QuadModelv_B.DataTypeConversion3[1];
  QuadModelv_Y.MavHILSensor.zgyro = QuadModelv_B.DataTypeConversion3[2];
  QuadModelv_Y.MavHILSensor.xmag = QuadModelv_B.DataTypeConversion4[0];
  QuadModelv_Y.MavHILSensor.ymag = QuadModelv_B.DataTypeConversion4[1];
  QuadModelv_Y.MavHILSensor.zmag = QuadModelv_B.DataTypeConversion4[2];
  QuadModelv_Y.MavHILSensor.abs_pressure = static_cast<real32_T>
    (QuadModelv_B.Gain_k);
  QuadModelv_Y.MavHILSensor.diff_pressure = static_cast<real32_T>
    (QuadModelv_P.Gain1_Gain_g * rtb_q2dot + QuadModelv_B.Product4_h);
  QuadModelv_Y.MavHILSensor.pressure_alt = static_cast<real32_T>
    (QuadModelv_B.y_p);
  QuadModelv_Y.MavHILSensor.temperature = static_cast<real32_T>((rtb_Rn -
    273.15000000000003) + QuadModelv_P.Constant1_Value_j);
  QuadModelv_Y.MavHILSensor.fields_updated = QuadModelv_B.fields_updated;

  // If: '<S321>/If' incorporates:
  //   Merge: '<S321>/Merge'

  if (rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    rtAction = static_cast<int8_T>(!(rtb_TmpSignalConversionAtSFun_0 > 0.0));
    QuadModelv_DW.If_ActiveSubsystem_m = rtAction;
  } else {
    rtAction = QuadModelv_DW.If_ActiveSubsystem_m;
  }

  if (rtAction == 0) {
    // Outputs for IfAction SubSystem: '<S321>/Positive Trace' incorporates:
    //   ActionPort: '<S328>/Action Port'

    QuadModelv_PositiveTrace(rtb_TmpSignalConversionAtSFun_0, VectorConcatenate,
      &QuadModelv_B.Merge_f[0], &QuadModelv_B.Merge_f[1],
      &QuadModelv_P.PositiveTrace_b);

    // End of Outputs for SubSystem: '<S321>/Positive Trace'
  } else {
    // Outputs for IfAction SubSystem: '<S321>/Negative Trace' incorporates:
    //   ActionPort: '<S327>/Action Port'

    QuadModelv_NegativeTrace(VectorConcatenate, QuadModelv_B.Merge_f,
      &QuadModelv_DW.NegativeTrace_l, &QuadModelv_P.NegativeTrace_l);

    // End of Outputs for SubSystem: '<S321>/Negative Trace'
  }

  // End of If: '<S321>/If'

  // DataTypeConversion: '<S324>/Data Type Conversion7' incorporates:
  //   Gain: '<S324>/Gain4'

  rtb_sincos_o1_idx_1 = rt_roundd_snf(QuadModelv_P.Gain4_Gain *
    QuadModelv_B.Product[0]);
  if (rtIsNaN(rtb_sincos_o1_idx_1) || rtIsInf(rtb_sincos_o1_idx_1)) {
    rtb_sincos_o1_idx_1 = 0.0;
  } else {
    rtb_sincos_o1_idx_1 = std::fmod(rtb_sincos_o1_idx_1, 65536.0);
  }

  rtb_TmpSignalConversionAtSFun_0 = rt_roundd_snf(QuadModelv_P.Gain4_Gain *
    QuadModelv_B.Product[1]);
  if (rtIsNaN(rtb_TmpSignalConversionAtSFun_0) || rtIsInf
      (rtb_TmpSignalConversionAtSFun_0)) {
    rtb_TmpSignalConversionAtSFun_0 = 0.0;
  } else {
    rtb_TmpSignalConversionAtSFun_0 = std::fmod(rtb_TmpSignalConversionAtSFun_0,
      65536.0);
  }

  rtb_Rn = rt_roundd_snf(QuadModelv_P.Gain4_Gain * QuadModelv_B.Product[2]);
  if (rtIsNaN(rtb_Rn) || rtIsInf(rtb_Rn)) {
    rtb_Rn = 0.0;
  } else {
    rtb_Rn = std::fmod(rtb_Rn, 65536.0);
  }

  // DataTypeConversion: '<S324>/Data Type Conversion9' incorporates:
  //   Gain: '<S324>/Gain2'
  //   Gain: '<S324>/Gain7'
  //   Gain: '<S507>/1//2rhoV^2'
  //   Sqrt: '<S324>/Airspeed1'
  //   Sqrt: '<S324>/Sqrt1'

  rtb_TmpSignalConversionAtSFun_1 = rt_roundd_snf(std::sqrt
    (rtb_TmpSignalConversionAtSFun_1 * QuadModelv_P.u2rhoV2_Gain_a *
     QuadModelv_P.Gain2_Gain_j) * QuadModelv_P.Gain7_Gain);
  if (rtIsNaN(rtb_TmpSignalConversionAtSFun_1) || rtIsInf
      (rtb_TmpSignalConversionAtSFun_1)) {
    rtb_TmpSignalConversionAtSFun_1 = 0.0;
  } else {
    rtb_TmpSignalConversionAtSFun_1 = std::fmod(rtb_TmpSignalConversionAtSFun_1,
      65536.0);
  }

  rtb_q2dot = rt_roundd_snf(std::sqrt(tz_idx_0) * QuadModelv_P.Gain7_Gain);
  if (rtIsNaN(rtb_q2dot) || rtIsInf(rtb_q2dot)) {
    rtb_q2dot = 0.0;
  } else {
    rtb_q2dot = std::fmod(rtb_q2dot, 65536.0);
  }

  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // DataTypeConversion: '<S324>/Data Type Conversion8' incorporates:
    //   Gain: '<S324>/Gain5'

    tmp_0 = rt_roundf_snf(QuadModelv_P.Gain5_Gain_p *
                          QuadModelv_B.DataTypeConversion2_i[0]);
    if (rtIsNaNF(tmp_0) || rtIsInfF(tmp_0)) {
      tmp_0 = 0.0F;
    } else {
      tmp_0 = std::fmod(tmp_0, 65536.0F);
    }

    // DataTypeConversion: '<S324>/Data Type Conversion8'
    QuadModelv_B.DataTypeConversion8[0] = static_cast<int16_T>(tmp_0 < 0.0F ?
      static_cast<int32_T>(static_cast<int16_T>(-static_cast<int16_T>(
      static_cast<uint16_T>(-tmp_0)))) : static_cast<int32_T>
      (static_cast<int16_T>(static_cast<uint16_T>(tmp_0))));

    // DataTypeConversion: '<S324>/Data Type Conversion8' incorporates:
    //   Gain: '<S324>/Gain5'

    tmp_0 = rt_roundf_snf(QuadModelv_P.Gain5_Gain_p *
                          QuadModelv_B.DataTypeConversion2_i[1]);
    if (rtIsNaNF(tmp_0) || rtIsInfF(tmp_0)) {
      tmp_0 = 0.0F;
    } else {
      tmp_0 = std::fmod(tmp_0, 65536.0F);
    }

    // DataTypeConversion: '<S324>/Data Type Conversion8'
    QuadModelv_B.DataTypeConversion8[1] = static_cast<int16_T>(tmp_0 < 0.0F ?
      static_cast<int32_T>(static_cast<int16_T>(-static_cast<int16_T>(
      static_cast<uint16_T>(-tmp_0)))) : static_cast<int32_T>
      (static_cast<int16_T>(static_cast<uint16_T>(tmp_0))));

    // DataTypeConversion: '<S324>/Data Type Conversion8' incorporates:
    //   Gain: '<S324>/Gain5'

    tmp_0 = rt_roundf_snf(QuadModelv_P.Gain5_Gain_p *
                          QuadModelv_B.DataTypeConversion2_i[2]);
    if (rtIsNaNF(tmp_0) || rtIsInfF(tmp_0)) {
      tmp_0 = 0.0F;
    } else {
      tmp_0 = std::fmod(tmp_0, 65536.0F);
    }

    // DataTypeConversion: '<S324>/Data Type Conversion8'
    QuadModelv_B.DataTypeConversion8[2] = static_cast<int16_T>(tmp_0 < 0.0F ?
      static_cast<int32_T>(static_cast<int16_T>(-static_cast<int16_T>(
      static_cast<uint16_T>(-tmp_0)))) : static_cast<int32_T>
      (static_cast<int16_T>(static_cast<uint16_T>(tmp_0))));
  }

  // Outport: '<Root>/MavHILStateQuat' incorporates:
  //   BusCreator: '<S324>/Bus Creator'
  //   DataTypeConversion: '<S324>/Data Type Conversion2'
  //   DataTypeConversion: '<S324>/Data Type Conversion3'
  //   Integrator: '<S14>/p,q,r '

  QuadModelv_Y.MavHILStateQuat.time_usec = rtb_time_usec;
  QuadModelv_Y.MavHILStateQuat.attitude_quaternion[0] = static_cast<real32_T>
    (QuadModelv_B.Merge_f[0]);
  QuadModelv_Y.MavHILStateQuat.attitude_quaternion[1] = static_cast<real32_T>
    (QuadModelv_B.Merge_f[1]);
  QuadModelv_Y.MavHILStateQuat.attitude_quaternion[2] = static_cast<real32_T>
    (QuadModelv_B.Merge_f[2]);
  QuadModelv_Y.MavHILStateQuat.attitude_quaternion[3] = static_cast<real32_T>
    (QuadModelv_B.Merge_f[3]);
  QuadModelv_Y.MavHILStateQuat.rollspeed = static_cast<real32_T>
    (QuadModelv_X.pqr_CSTATE[0]);
  QuadModelv_Y.MavHILStateQuat.pitchspeed = static_cast<real32_T>
    (QuadModelv_X.pqr_CSTATE[1]);
  QuadModelv_Y.MavHILStateQuat.yawspeed = static_cast<real32_T>
    (QuadModelv_X.pqr_CSTATE[2]);

  // DataTypeConversion: '<S324>/Data Type Conversion4' incorporates:
  //   Gain: '<S324>/Gain'

  X = rt_roundd_snf(QuadModelv_P.Gain_Gain_b * LoadLeakRatio);
  if (rtIsNaN(X) || rtIsInf(X)) {
    X = 0.0;
  } else {
    X = std::fmod(X, 4.294967296E+9);
  }

  // Outport: '<Root>/MavHILStateQuat' incorporates:
  //   DataTypeConversion: '<S324>/Data Type Conversion4'

  QuadModelv_Y.MavHILStateQuat.lat = X < 0.0 ? -static_cast<int32_T>(
    static_cast<uint32_T>(-X)) : static_cast<int32_T>(static_cast<uint32_T>(X));

  // DataTypeConversion: '<S324>/Data Type Conversion1' incorporates:
  //   Gain: '<S324>/Gain1'

  X = rt_roundd_snf(QuadModelv_P.Gain1_Gain_k * Z);
  if (rtIsNaN(X) || rtIsInf(X)) {
    X = 0.0;
  } else {
    X = std::fmod(X, 4.294967296E+9);
  }

  // Outport: '<Root>/MavHILStateQuat' incorporates:
  //   DataTypeConversion: '<S324>/Data Type Conversion1'

  QuadModelv_Y.MavHILStateQuat.lon = X < 0.0 ? -static_cast<int32_T>(
    static_cast<uint32_T>(-X)) : static_cast<int32_T>(static_cast<uint32_T>(X));

  // DataTypeConversion: '<S324>/Data Type Conversion5' incorporates:
  //   Gain: '<S324>/Gain3'

  X = rt_roundd_snf(QuadModelv_P.Gain3_Gain * rtb_Saturation_1);
  if (rtIsNaN(X) || rtIsInf(X)) {
    X = 0.0;
  } else {
    X = std::fmod(X, 4.294967296E+9);
  }

  // Outport: '<Root>/MavHILStateQuat' incorporates:
  //   BusCreator: '<S324>/Bus Creator'
  //   DataTypeConversion: '<S324>/Data Type Conversion5'
  //   DataTypeConversion: '<S324>/Data Type Conversion7'
  //   DataTypeConversion: '<S324>/Data Type Conversion9'

  QuadModelv_Y.MavHILStateQuat.alt = X < 0.0 ? -static_cast<int32_T>(
    static_cast<uint32_T>(-X)) : static_cast<int32_T>(static_cast<uint32_T>(X));
  QuadModelv_Y.MavHILStateQuat.vx = static_cast<int16_T>(rtb_sincos_o1_idx_1 <
    0.0 ? static_cast<int32_T>(static_cast<int16_T>(-static_cast<int16_T>(
    static_cast<uint16_T>(-rtb_sincos_o1_idx_1)))) : static_cast<int32_T>(
    static_cast<int16_T>(static_cast<uint16_T>(rtb_sincos_o1_idx_1))));
  QuadModelv_Y.MavHILStateQuat.vy = static_cast<int16_T>
    (rtb_TmpSignalConversionAtSFun_0 < 0.0 ? static_cast<int32_T>
     (static_cast<int16_T>(-static_cast<int16_T>(static_cast<uint16_T>
        (-rtb_TmpSignalConversionAtSFun_0)))) : static_cast<int32_T>(
      static_cast<int16_T>(static_cast<uint16_T>(rtb_TmpSignalConversionAtSFun_0))));
  QuadModelv_Y.MavHILStateQuat.vz = static_cast<int16_T>(rtb_Rn < 0.0 ?
    static_cast<int32_T>(static_cast<int16_T>(-static_cast<int16_T>(static_cast<
    uint16_T>(-rtb_Rn)))) : static_cast<int32_T>(static_cast<int16_T>(
    static_cast<uint16_T>(rtb_Rn))));
  QuadModelv_Y.MavHILStateQuat.ind_airspeed = static_cast<uint16_T>
    (rtb_TmpSignalConversionAtSFun_1 < 0.0 ? static_cast<int32_T>
     (static_cast<uint16_T>(-static_cast<int16_T>(static_cast<uint16_T>
        (-rtb_TmpSignalConversionAtSFun_1)))) : static_cast<int32_T>(
      static_cast<uint16_T>(rtb_TmpSignalConversionAtSFun_1)));
  QuadModelv_Y.MavHILStateQuat.true_airspeed = static_cast<uint16_T>(rtb_q2dot <
    0.0 ? static_cast<int32_T>(static_cast<uint16_T>(-static_cast<int16_T>(
    static_cast<uint16_T>(-rtb_q2dot)))) : static_cast<int32_T>(static_cast<
    uint16_T>(rtb_q2dot)));
  QuadModelv_Y.MavHILStateQuat.xacc = QuadModelv_B.DataTypeConversion8[0];
  QuadModelv_Y.MavHILStateQuat.yacc = QuadModelv_B.DataTypeConversion8[1];
  QuadModelv_Y.MavHILStateQuat.zacc = QuadModelv_B.DataTypeConversion8[2];

  // MATLAB Function: '<S12>/ZLimit' incorporates:
  //   Integrator: '<S14>/xe,ye,ze'

  // MATLAB Function 'OutputPort1/ZLimit': '<S326>:1'
  // '<S326>:1:2' Xee=xe;
  rtb_sincos_o1_f[2] = QuadModelv_X.xeyeze_CSTATE[2];

  // '<S326>:1:3' if Xee(3)>zt
  if (QuadModelv_X.xeyeze_CSTATE[2] > QuadModelv_B.y_m) {
    // '<S326>:1:4' Xee(3)=zt;
    rtb_sincos_o1_f[2] = QuadModelv_B.y_m;
  }

  // End of MATLAB Function: '<S12>/ZLimit'
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // UnitConversion: '<S389>/Unit Conversion' incorporates:
    //   Constant: '<S371>/ref_rotation'

    // Unit Conversion - from: deg to: rad
    // Expression: output = (0.0174533*input) + (0)
    uavlostJyz = 0.017453292519943295 * QuadModelv_P.FlatEarthtoLLA_psi_e;

    // Trigonometry: '<S374>/SinCos'
    QuadModelv_B.SinCos_o1_m = std::sin(uavlostJyz);

    // Trigonometry: '<S374>/SinCos'
    QuadModelv_B.SinCos_o2_i = std::cos(uavlostJyz);

    // Sum: '<S392>/Sum' incorporates:
    //   Constant: '<S392>/Constant'
    //   Constant: '<S392>/f'

    uavlostJyz = QuadModelv_P.f_Value_l - QuadModelv_P.Constant_Value_na;

    // Sqrt: '<S393>/sqrt' incorporates:
    //   Constant: '<S393>/Constant'
    //   Product: '<S393>/Product1'
    //   Sum: '<S393>/Sum1'

    uavlostJyz = std::sqrt(QuadModelv_P.Constant_Value_ol - uavlostJyz *
      uavlostJyz);

    // Switch: '<S385>/Switch' incorporates:
    //   Abs: '<S385>/Abs'
    //   Bias: '<S385>/Bias'
    //   Bias: '<S385>/Bias1'
    //   Constant: '<S371>/ref_position'
    //   Constant: '<S385>/Constant2'
    //   Constant: '<S386>/Constant'
    //   Math: '<S385>/Math Function1'
    //   RelationalOperator: '<S386>/Compare'

    if (std::abs(QuadModelv_P.ModelParam_GPSLatLong[0]) >
        QuadModelv_P.CompareToConstant_const_b) {
      tz_idx_0 = rt_modd_snf(QuadModelv_P.ModelParam_GPSLatLong[0] +
        QuadModelv_P.Bias_Bias_j, QuadModelv_P.Constant2_Value_m) +
        QuadModelv_P.Bias1_Bias_o4;
    } else {
      tz_idx_0 = QuadModelv_P.ModelParam_GPSLatLong[0];
    }

    // End of Switch: '<S385>/Switch'

    // Abs: '<S382>/Abs1'
    rtb_Product1_d = std::abs(tz_idx_0);

    // RelationalOperator: '<S384>/Compare' incorporates:
    //   Constant: '<S384>/Constant'

    hFaultTmp = (rtb_Product1_d > QuadModelv_P.CompareToConstant_const_g);

    // Switch: '<S382>/Switch'
    if (hFaultTmp) {
      // Signum: '<S382>/Sign1'
      if (rtIsNaN(tz_idx_0)) {
        rtb_sincos_o1_idx_1 = (rtNaN);
      } else if (tz_idx_0 < 0.0) {
        rtb_sincos_o1_idx_1 = -1.0;
      } else {
        rtb_sincos_o1_idx_1 = (tz_idx_0 > 0.0);
      }

      // Switch: '<S382>/Switch' incorporates:
      //   Bias: '<S382>/Bias'
      //   Bias: '<S382>/Bias1'
      //   Gain: '<S382>/Gain'
      //   Product: '<S382>/Divide1'
      //   Signum: '<S382>/Sign1'

      QuadModelv_B.Switch_h = ((rtb_Product1_d + QuadModelv_P.Bias_Bias_d) *
        QuadModelv_P.Gain_Gain_i + QuadModelv_P.Bias1_Bias_at) *
        rtb_sincos_o1_idx_1;
    } else {
      // Switch: '<S382>/Switch'
      QuadModelv_B.Switch_h = tz_idx_0;
    }

    // End of Switch: '<S382>/Switch'

    // UnitConversion: '<S390>/Unit Conversion'
    // Unit Conversion - from: deg to: rad
    // Expression: output = (0.0174533*input) + (0)
    rtb_Product1_d = 0.017453292519943295 * QuadModelv_B.Switch_h;

    // Trigonometry: '<S391>/Trigonometric Function1'
    rtb_TmpSignalConversionAtSFun_1 = std::sin(rtb_Product1_d);

    // Product: '<S391>/Product1' incorporates:
    //   Product: '<S388>/Product2'

    rtb_TmpSignalConversionAtSFun_0 = uavlostJyz * uavlostJyz;

    // Sum: '<S391>/Sum1' incorporates:
    //   Constant: '<S391>/Constant'
    //   Product: '<S391>/Product1'

    rtb_TmpSignalConversionAtSFun_1 = QuadModelv_P.Constant_Value_cc -
      rtb_TmpSignalConversionAtSFun_0 * rtb_TmpSignalConversionAtSFun_1 *
      rtb_TmpSignalConversionAtSFun_1;

    // Product: '<S388>/Product1' incorporates:
    //   Constant: '<S388>/Constant1'
    //   Sqrt: '<S388>/sqrt'

    rtb_Rn = QuadModelv_P.Constant1_Value_d0 / std::sqrt
      (rtb_TmpSignalConversionAtSFun_1);

    // Trigonometry: '<S388>/Trigonometric Function1' incorporates:
    //   Constant: '<S388>/Constant'
    //   Constant: '<S388>/Constant2'
    //   Product: '<S388>/Product3'
    //   Sum: '<S388>/Sum1'

    QuadModelv_B.TrigonometricFunction1_d = rt_atan2d_snf
      (QuadModelv_P.Constant2_Value_k, (QuadModelv_P.Constant_Value_br -
        rtb_TmpSignalConversionAtSFun_0) * rtb_Rn /
       rtb_TmpSignalConversionAtSFun_1);

    // Trigonometry: '<S388>/Trigonometric Function2' incorporates:
    //   Constant: '<S388>/Constant3'
    //   Product: '<S388>/Product4'
    //   Trigonometry: '<S388>/Trigonometric Function'

    QuadModelv_B.TrigonometricFunction2_i = rt_atan2d_snf
      (QuadModelv_P.Constant3_Value_l, rtb_Rn * std::cos(rtb_Product1_d));

    // Switch: '<S373>/Switch1' incorporates:
    //   Constant: '<S373>/Constant'
    //   Constant: '<S373>/Constant1'

    if (hFaultTmp) {
      rtb_sincos_o1_idx_1 = QuadModelv_P.Constant_Value_d;
    } else {
      rtb_sincos_o1_idx_1 = QuadModelv_P.Constant1_Value_b;
    }

    // Sum: '<S373>/Sum' incorporates:
    //   Constant: '<S371>/ref_position'
    //   Switch: '<S373>/Switch1'

    tz_idx_0 = rtb_sincos_o1_idx_1 + QuadModelv_P.ModelParam_GPSLatLong[1];

    // Switch: '<S383>/Switch' incorporates:
    //   Abs: '<S383>/Abs'
    //   Constant: '<S387>/Constant'
    //   RelationalOperator: '<S387>/Compare'

    if (std::abs(tz_idx_0) > QuadModelv_P.CompareToConstant_const_iz) {
      // Switch: '<S383>/Switch' incorporates:
      //   Bias: '<S383>/Bias'
      //   Bias: '<S383>/Bias1'
      //   Constant: '<S383>/Constant2'
      //   Math: '<S383>/Math Function1'

      QuadModelv_B.Switch_o = rt_modd_snf(tz_idx_0 + QuadModelv_P.Bias_Bias_f,
        QuadModelv_P.Constant2_Value_h) + QuadModelv_P.Bias1_Bias_gl;
    } else {
      // Switch: '<S383>/Switch'
      QuadModelv_B.Switch_o = tz_idx_0;
    }

    // End of Switch: '<S383>/Switch'
  }

  // Product: '<S374>/rad long ' incorporates:
  //   Integrator: '<S14>/xe,ye,ze'
  //   Product: '<S374>/x*sin'
  //   Product: '<S374>/y*cos'
  //   Sum: '<S374>/Sum1'

  rtb_TmpSignalConversionAtSFun_1 = (QuadModelv_X.xeyeze_CSTATE[0] *
    QuadModelv_B.SinCos_o1_m + QuadModelv_X.xeyeze_CSTATE[1] *
    QuadModelv_B.SinCos_o2_i) * QuadModelv_B.TrigonometricFunction2_i;

  // Sum: '<S371>/Sum' incorporates:
  //   Integrator: '<S14>/xe,ye,ze'
  //   Product: '<S374>/rad lat'
  //   Product: '<S374>/x*cos'
  //   Product: '<S374>/y*sin'
  //   Sum: '<S374>/Sum'
  //   UnitConversion: '<S375>/Unit Conversion'

  // Unit Conversion - from: rad to: deg
  // Expression: output = (57.2958*input) + (0)
  tz_idx_0 = (QuadModelv_X.xeyeze_CSTATE[0] * QuadModelv_B.SinCos_o2_i -
              QuadModelv_X.xeyeze_CSTATE[1] * QuadModelv_B.SinCos_o1_m) *
    QuadModelv_B.TrigonometricFunction1_d * 57.295779513082323 +
    QuadModelv_B.Switch_h;

  // Switch: '<S379>/Switch' incorporates:
  //   Abs: '<S379>/Abs'
  //   Bias: '<S379>/Bias'
  //   Bias: '<S379>/Bias1'
  //   Constant: '<S379>/Constant2'
  //   Constant: '<S380>/Constant'
  //   Math: '<S379>/Math Function1'
  //   RelationalOperator: '<S380>/Compare'

  if (std::abs(tz_idx_0) > QuadModelv_P.CompareToConstant_const_bz) {
    tz_idx_0 = rt_modd_snf(tz_idx_0 + QuadModelv_P.Bias_Bias_ee,
      QuadModelv_P.Constant2_Value_nr) + QuadModelv_P.Bias1_Bias_k;
  }

  // End of Switch: '<S379>/Switch'

  // Abs: '<S376>/Abs1'
  uavlostJyz = std::abs(tz_idx_0);

  // Switch: '<S376>/Switch' incorporates:
  //   Constant: '<S372>/Constant'
  //   Constant: '<S372>/Constant1'
  //   Constant: '<S378>/Constant'
  //   RelationalOperator: '<S378>/Compare'
  //   Switch: '<S372>/Switch1'

  if (uavlostJyz > QuadModelv_P.CompareToConstant_const_d) {
    // Signum: '<S376>/Sign1'
    if (rtIsNaN(tz_idx_0)) {
      rtb_sincos_o1_idx_1 = (rtNaN);
    } else if (tz_idx_0 < 0.0) {
      rtb_sincos_o1_idx_1 = -1.0;
    } else {
      rtb_sincos_o1_idx_1 = (tz_idx_0 > 0.0);
    }

    // Switch: '<S376>/Switch' incorporates:
    //   Bias: '<S376>/Bias'
    //   Bias: '<S376>/Bias1'
    //   Gain: '<S376>/Gain'
    //   Product: '<S376>/Divide1'
    //   Signum: '<S376>/Sign1'

    tz_idx_0 = ((uavlostJyz + QuadModelv_P.Bias_Bias_e) *
                QuadModelv_P.Gain_Gain_d + QuadModelv_P.Bias1_Bias_g2) *
      rtb_sincos_o1_idx_1;
    rtb_sincos_o1_idx_1 = QuadModelv_P.Constant_Value_bd;
  } else {
    rtb_sincos_o1_idx_1 = QuadModelv_P.Constant1_Value_f;
  }

  // End of Switch: '<S376>/Switch'

  // Sum: '<S372>/Sum' incorporates:
  //   Sum: '<S371>/Sum'
  //   Switch: '<S372>/Switch1'
  //   UnitConversion: '<S375>/Unit Conversion'

  uavlostJyz = (57.295779513082323 * rtb_TmpSignalConversionAtSFun_1 +
                QuadModelv_B.Switch_o) + rtb_sincos_o1_idx_1;

  // Switch: '<S377>/Switch' incorporates:
  //   Abs: '<S377>/Abs'
  //   Constant: '<S381>/Constant'
  //   RelationalOperator: '<S381>/Compare'

  if (std::abs(uavlostJyz) > QuadModelv_P.CompareToConstant_const_ex) {
    // Switch: '<S377>/Switch' incorporates:
    //   Bias: '<S377>/Bias'
    //   Bias: '<S377>/Bias1'
    //   Constant: '<S377>/Constant2'
    //   Math: '<S377>/Math Function1'

    uavlostJyz = rt_modd_snf(uavlostJyz + QuadModelv_P.Bias_Bias_gf,
      QuadModelv_P.Constant2_Value_g) + QuadModelv_P.Bias1_Bias_a;
  }

  // End of Switch: '<S377>/Switch'

  // Sum: '<S371>/Sum1' incorporates:
  //   Constant: '<S366>/ModelParam.envAltitude'
  //   UnaryMinus: '<S371>/Ze2height'

  rtb_Rn = -rtb_sincos_o1_f[2] - QuadModelv_P.ModelParam_envAltitude;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // MATLAB Function: '<S366>/FaultParamsExtract' incorporates:
    //   Constant: '<Root>/Constant1'
    //   Constant: '<S366>/FaultID'

    // MATLAB Function 'GPSFault/FaultParamsExtract': '<S370>:1'
    // '<S370>:1:5' if isempty(hFault)
    // '<S370>:1:8' if isempty(fParam)
    // '<S370>:1:12' hFaultTmp=false;
    hFaultTmp = false;

    // '<S370>:1:13' fParamTmp=zeros(20,1);
    std::memset(&fParamTmp[0], 0, 20U * sizeof(real_T));

    // '<S370>:1:14' j=1;
    j = 1;

    // '<S370>:1:15' for i=1:8
    for (r = 0; r < 8; r++) {
      // '<S370>:1:16' if inInts(i) == FaultID
      if (QuadModelv_B.inSILInts[r] == QuadModelv_P.FaultID_Value_p) {
        // '<S370>:1:17' hFaultTmp=true;
        hFaultTmp = true;

        // '<S370>:1:18' fParamTmp(2*j-1)=inFloats(2*i-1);
        s458_iter = (r + 1) << 1;
        fParamTmp_tmp = j << 1;
        fParamTmp[fParamTmp_tmp - 2] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 2];

        // '<S370>:1:19' fParamTmp(2*j)=inFloats(2*i);
        fParamTmp[fParamTmp_tmp - 1] = QuadModelv_P.Constant1_Value_a[s458_iter
          - 1];

        // '<S370>:1:20' j=j+1;
        j++;
      }
    }

    // '<S370>:1:23' if hFaultTmp
    if (hFaultTmp) {
      // '<S370>:1:24' hFault=hFaultTmp;
      QuadModelv_DW.hFault_lp = true;

      // '<S370>:1:25' fParamTmp(17:20) = inFloats(17:20);
      fParamTmp[16] = QuadModelv_P.Constant1_Value_a[16];
      fParamTmp[17] = QuadModelv_P.Constant1_Value_a[17];
      fParamTmp[18] = QuadModelv_P.Constant1_Value_a[18];
      fParamTmp[19] = QuadModelv_P.Constant1_Value_a[19];

      // '<S370>:1:26' fParam=fParamTmp;
      std::memcpy(&QuadModelv_DW.fParam_d[0], &fParamTmp[0], 20U * sizeof(real_T));
    }

    // MATLAB Function: '<S366>/Acc NoiseFun' incorporates:
    //   SignalConversion generated from: '<S369>/ SFunction '

    // '<S370>:1:29' hasFault_GPS=hFault;
    // '<S370>:1:30' FaultParam=fParam;
    rtb_TmpSignalConversionAtSFun_0 = tz_idx_0;
    rtb_TmpSignalConversionAtSFun_1 = uavlostJyz;

    // Saturate: '<S366>/Saturation'
    if (rtb_Rn > QuadModelv_P.Saturation_UpperSat_h) {
      // MATLAB Function: '<S366>/Acc NoiseFun'
      rtb_Rn = QuadModelv_P.Saturation_UpperSat_h;
    } else if (rtb_Rn < QuadModelv_P.Saturation_LowerSat_au) {
      // MATLAB Function: '<S366>/Acc NoiseFun'
      rtb_Rn = QuadModelv_P.Saturation_LowerSat_au;
    }

    // End of Saturate: '<S366>/Saturation'

    // MATLAB Function: '<S366>/Acc NoiseFun' incorporates:
    //   MATLAB Function: '<S366>/FaultParamsExtract'
    //   SignalConversion generated from: '<S369>/ SFunction '

    // MATLAB Function 'GPSFault/Acc NoiseFun': '<S369>:1'
    // '<S369>:1:2' y = GPSIn;
    // '<S369>:1:3' satellites = 10;
    QuadModelv_B.satellites = 10.0;

    // '<S369>:1:4' GPS3Dfixed = 3;
    QuadModelv_B.GPS3Dfixed = 3.0;

    // '<S369>:1:6' if isGPSFault
    if (QuadModelv_DW.hFault_lp) {
      //  FaultParams(1)为标志位
      // '<S369>:1:8' flag = GPSFaultParams(1);
      //  覆盖模式，当为-1时，保持原有输出
      // '<S369>:1:11' if flag == 1
      if (QuadModelv_DW.fParam_d[0] == 1.0) {
        // '<S369>:1:12' for ind = 1:3
        // '<S369>:1:13' if GPSFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_d[1] != -1.0) {
          // '<S369>:1:14' y(ind) = GPSFaultParams(ind+1);
          rtb_TmpSignalConversionAtSFun_0 = QuadModelv_DW.fParam_d[1];
        }

        // '<S369>:1:13' if GPSFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_d[2] != -1.0) {
          // '<S369>:1:14' y(ind) = GPSFaultParams(ind+1);
          rtb_TmpSignalConversionAtSFun_1 = QuadModelv_DW.fParam_d[2];
        }

        // '<S369>:1:13' if GPSFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_d[3] != -1.0) {
          // '<S369>:1:14' y(ind) = GPSFaultParams(ind+1);
          rtb_Rn = QuadModelv_DW.fParam_d[3];
        }

        //  叠加模式，当为-1时，保持原有输出
      } else if (QuadModelv_DW.fParam_d[0] == 2.0) {
        // '<S369>:1:19' elseif flag == 2
        // '<S369>:1:20' for ind = 1:3
        // '<S369>:1:21' if GPSFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_d[1] != -1.0) {
          // '<S369>:1:22' y(ind) = y(ind) + GPSFaultParams(ind+1);
          rtb_TmpSignalConversionAtSFun_0 = tz_idx_0 + QuadModelv_DW.fParam_d[1];
        }

        // '<S369>:1:21' if GPSFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_d[2] != -1.0) {
          // '<S369>:1:22' y(ind) = y(ind) + GPSFaultParams(ind+1);
          rtb_TmpSignalConversionAtSFun_1 = uavlostJyz + QuadModelv_DW.fParam_d
            [2];
        }

        // '<S369>:1:21' if GPSFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_d[3] != -1.0) {
          // '<S369>:1:22' y(ind) = y(ind) + GPSFaultParams(ind+1);
          rtb_Rn += QuadModelv_DW.fParam_d[3];
        }

        //  噪声模式，当为-1时，保持原有输出
      } else if (QuadModelv_DW.fParam_d[0] == 3.0) {
        // '<S369>:1:27' elseif flag == 3
        // '<S369>:1:28' mean = GPSFaultParams(2);
        // '<S369>:1:29' stddev = GPSFaultParams(3);
        // '<S369>:1:30' for ind = 1:3
        // '<S369>:1:31' if GPSFaultParams(ind+3) ~= -1
        if (QuadModelv_DW.fParam_d[3] != -1.0) {
          // '<S369>:1:32' y(ind) = y(ind) + generate_wgn(mean,stddev);
          // '<S369>:1:62' if isempty(phase)
          // '<S369>:1:69' if phase
          if (QuadModelv_DW.phase_l) {
            // '<S369>:1:70' while true
            do {
              // '<S369>:1:71' U1 = rand();
              U1 = QuadModelv_rand_d();

              // '<S369>:1:72' U2 = rand();
              X = QuadModelv_rand_d();

              // '<S369>:1:73' V1 = 2.0 * U1 - 1.0;
              U1 = 2.0 * U1 - 1.0;

              // '<S369>:1:74' V2 = 2.0 * U2 - 1.0;
              QuadModelv_DW.V2_o = 2.0 * X - 1.0;

              // '<S369>:1:75' S = V1 * V1 + V2 * V2;
              QuadModelv_DW.S_k1 = U1 * U1 + QuadModelv_DW.V2_o *
                QuadModelv_DW.V2_o;

              // '<S369>:1:76' if S < 1.0 && abs(S) >= 1e-8
            } while ((!(QuadModelv_DW.S_k1 < 1.0)) || (!(std::abs
                       (QuadModelv_DW.S_k1) >= 1.0E-8)));

            // '<S369>:1:80' X = V1 * sqrt(-2.0 * log(S) / S);
            X = std::sqrt(-2.0 * std::log(QuadModelv_DW.S_k1) /
                          QuadModelv_DW.S_k1) * U1;
          } else {
            // '<S369>:1:81' else
            // '<S369>:1:82' X = V2 * sqrt(-2.0 * log(S) / S);
            X = std::sqrt(-2.0 * std::log(QuadModelv_DW.S_k1) /
                          QuadModelv_DW.S_k1) * QuadModelv_DW.V2_o;
          }

          // '<S369>:1:85' y = mean + stddev * X;
          // '<S369>:1:87' phase = ~phase;
          QuadModelv_DW.phase_l = !QuadModelv_DW.phase_l;
          rtb_TmpSignalConversionAtSFun_0 = (QuadModelv_DW.fParam_d[2] * X +
            QuadModelv_DW.fParam_d[1]) + tz_idx_0;
        }

        // '<S369>:1:31' if GPSFaultParams(ind+3) ~= -1
        if (QuadModelv_DW.fParam_d[4] != -1.0) {
          // '<S369>:1:32' y(ind) = y(ind) + generate_wgn(mean,stddev);
          // '<S369>:1:62' if isempty(phase)
          // '<S369>:1:69' if phase
          if (QuadModelv_DW.phase_l) {
            // '<S369>:1:70' while true
            do {
              // '<S369>:1:71' U1 = rand();
              U1 = QuadModelv_rand_d();

              // '<S369>:1:72' U2 = rand();
              X = QuadModelv_rand_d();

              // '<S369>:1:73' V1 = 2.0 * U1 - 1.0;
              U1 = 2.0 * U1 - 1.0;

              // '<S369>:1:74' V2 = 2.0 * U2 - 1.0;
              QuadModelv_DW.V2_o = 2.0 * X - 1.0;

              // '<S369>:1:75' S = V1 * V1 + V2 * V2;
              QuadModelv_DW.S_k1 = U1 * U1 + QuadModelv_DW.V2_o *
                QuadModelv_DW.V2_o;

              // '<S369>:1:76' if S < 1.0 && abs(S) >= 1e-8
            } while ((!(QuadModelv_DW.S_k1 < 1.0)) || (!(std::abs
                       (QuadModelv_DW.S_k1) >= 1.0E-8)));

            // '<S369>:1:80' X = V1 * sqrt(-2.0 * log(S) / S);
            X = std::sqrt(-2.0 * std::log(QuadModelv_DW.S_k1) /
                          QuadModelv_DW.S_k1) * U1;
          } else {
            // '<S369>:1:81' else
            // '<S369>:1:82' X = V2 * sqrt(-2.0 * log(S) / S);
            X = std::sqrt(-2.0 * std::log(QuadModelv_DW.S_k1) /
                          QuadModelv_DW.S_k1) * QuadModelv_DW.V2_o;
          }

          // '<S369>:1:85' y = mean + stddev * X;
          // '<S369>:1:87' phase = ~phase;
          QuadModelv_DW.phase_l = !QuadModelv_DW.phase_l;
          rtb_TmpSignalConversionAtSFun_1 = (QuadModelv_DW.fParam_d[2] * X +
            QuadModelv_DW.fParam_d[1]) + uavlostJyz;
        }

        // '<S369>:1:31' if GPSFaultParams(ind+3) ~= -1
        if (QuadModelv_DW.fParam_d[5] != -1.0) {
          // '<S369>:1:32' y(ind) = y(ind) + generate_wgn(mean,stddev);
          // '<S369>:1:62' if isempty(phase)
          // '<S369>:1:69' if phase
          if (QuadModelv_DW.phase_l) {
            // '<S369>:1:70' while true
            do {
              // '<S369>:1:71' U1 = rand();
              U1 = QuadModelv_rand_d();

              // '<S369>:1:72' U2 = rand();
              X = QuadModelv_rand_d();

              // '<S369>:1:73' V1 = 2.0 * U1 - 1.0;
              U1 = 2.0 * U1 - 1.0;

              // '<S369>:1:74' V2 = 2.0 * U2 - 1.0;
              QuadModelv_DW.V2_o = 2.0 * X - 1.0;

              // '<S369>:1:75' S = V1 * V1 + V2 * V2;
              QuadModelv_DW.S_k1 = U1 * U1 + QuadModelv_DW.V2_o *
                QuadModelv_DW.V2_o;

              // '<S369>:1:76' if S < 1.0 && abs(S) >= 1e-8
            } while ((!(QuadModelv_DW.S_k1 < 1.0)) || (!(std::abs
                       (QuadModelv_DW.S_k1) >= 1.0E-8)));

            // '<S369>:1:80' X = V1 * sqrt(-2.0 * log(S) / S);
            X = std::sqrt(-2.0 * std::log(QuadModelv_DW.S_k1) /
                          QuadModelv_DW.S_k1) * U1;
          } else {
            // '<S369>:1:81' else
            // '<S369>:1:82' X = V2 * sqrt(-2.0 * log(S) / S);
            X = std::sqrt(-2.0 * std::log(QuadModelv_DW.S_k1) /
                          QuadModelv_DW.S_k1) * QuadModelv_DW.V2_o;
          }

          // '<S369>:1:85' y = mean + stddev * X;
          // '<S369>:1:87' phase = ~phase;
          QuadModelv_DW.phase_l = !QuadModelv_DW.phase_l;
          rtb_Rn += QuadModelv_DW.fParam_d[2] * X + QuadModelv_DW.fParam_d[1];
        }

        //  尺度因子模式，当为-1时，保持原有输出
      } else if (QuadModelv_DW.fParam_d[0] == 4.0) {
        // '<S369>:1:37' elseif flag == 4
        // '<S369>:1:38' for ind = 1:3
        // '<S369>:1:39' if GPSFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_d[1] != -1.0) {
          // '<S369>:1:40' y(ind) = y(ind) * GPSFaultParams(ind+1);
          rtb_TmpSignalConversionAtSFun_0 = tz_idx_0 * QuadModelv_DW.fParam_d[1];
        }

        // '<S369>:1:39' if GPSFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_d[2] != -1.0) {
          // '<S369>:1:40' y(ind) = y(ind) * GPSFaultParams(ind+1);
          rtb_TmpSignalConversionAtSFun_1 = uavlostJyz * QuadModelv_DW.fParam_d
            [2];
        }

        // '<S369>:1:39' if GPSFaultParams(ind+1) ~= -1
        if (QuadModelv_DW.fParam_d[3] != -1.0) {
          // '<S369>:1:40' y(ind) = y(ind) * GPSFaultParams(ind+1);
          rtb_Rn *= QuadModelv_DW.fParam_d[3];
        }

        //  改变搜星数
      } else if (QuadModelv_DW.fParam_d[0] == 5.0) {
        // '<S369>:1:45' elseif flag == 5
        // '<S369>:1:46' satellites = GPSFaultParams(2);
        QuadModelv_B.satellites = QuadModelv_DW.fParam_d[1];

        //  改变Fixed
      } else if (QuadModelv_DW.fParam_d[0] == 6.0) {
        // '<S369>:1:49' elseif flag == 6
        // '<S369>:1:50' GPS3Dfixed = GPSFaultParams(2);
        QuadModelv_B.GPS3Dfixed = QuadModelv_DW.fParam_d[1];
      }
    }

    // DataTypeConversion: '<S322>/Data Type Conversion4' incorporates:
    //   Gain: '<S322>/latScale'

    rtb_sincos_o1_idx_1 = rt_roundd_snf(QuadModelv_P.latScale_Gain *
      rtb_TmpSignalConversionAtSFun_0);
    if (rtIsNaN(rtb_sincos_o1_idx_1) || rtIsInf(rtb_sincos_o1_idx_1)) {
      rtb_sincos_o1_idx_1 = 0.0;
    } else {
      rtb_sincos_o1_idx_1 = std::fmod(rtb_sincos_o1_idx_1, 4.294967296E+9);
    }

    // DataTypeConversion: '<S322>/Data Type Conversion4'
    QuadModelv_B.lat = rtb_sincos_o1_idx_1 < 0.0 ? -static_cast<int32_T>(
      static_cast<uint32_T>(-rtb_sincos_o1_idx_1)) : static_cast<int32_T>(
      static_cast<uint32_T>(rtb_sincos_o1_idx_1));

    // DataTypeConversion: '<S322>/Data Type Conversion5' incorporates:
    //   Gain: '<S322>/lonScale'

    rtb_sincos_o1_idx_1 = rt_roundd_snf(QuadModelv_P.lonScale_Gain *
      rtb_TmpSignalConversionAtSFun_1);
    if (rtIsNaN(rtb_sincos_o1_idx_1) || rtIsInf(rtb_sincos_o1_idx_1)) {
      rtb_sincos_o1_idx_1 = 0.0;
    } else {
      rtb_sincos_o1_idx_1 = std::fmod(rtb_sincos_o1_idx_1, 4.294967296E+9);
    }

    // DataTypeConversion: '<S322>/Data Type Conversion5'
    QuadModelv_B.lon = rtb_sincos_o1_idx_1 < 0.0 ? -static_cast<int32_T>(
      static_cast<uint32_T>(-rtb_sincos_o1_idx_1)) : static_cast<int32_T>(
      static_cast<uint32_T>(rtb_sincos_o1_idx_1));

    // DataTypeConversion: '<S322>/Data Type Conversion6' incorporates:
    //   Gain: '<S322>/altScale'

    rtb_sincos_o1_idx_1 = rt_roundd_snf(QuadModelv_P.altScale_Gain * rtb_Rn);
    if (rtIsNaN(rtb_sincos_o1_idx_1) || rtIsInf(rtb_sincos_o1_idx_1)) {
      rtb_sincos_o1_idx_1 = 0.0;
    } else {
      rtb_sincos_o1_idx_1 = std::fmod(rtb_sincos_o1_idx_1, 4.294967296E+9);
    }

    // DataTypeConversion: '<S322>/Data Type Conversion6'
    QuadModelv_B.alt = rtb_sincos_o1_idx_1 < 0.0 ? -static_cast<int32_T>(
      static_cast<uint32_T>(-rtb_sincos_o1_idx_1)) : static_cast<int32_T>(
      static_cast<uint32_T>(rtb_sincos_o1_idx_1));

    // DataTypeConversion: '<S322>/Data Type Conversion8' incorporates:
    //   Constant: '<S322>/ModelParam.GPSEphFinal'
    //   Gain: '<S322>/Gain6'

    rtb_sincos_o1_idx_1 = rt_roundd_snf(QuadModelv_P.Gain6_Gain *
      QuadModelv_P.ModelParam_GPSEphFinal);
    if (rtIsNaN(rtb_sincos_o1_idx_1) || rtIsInf(rtb_sincos_o1_idx_1)) {
      rtb_sincos_o1_idx_1 = 0.0;
    } else {
      rtb_sincos_o1_idx_1 = std::fmod(rtb_sincos_o1_idx_1, 65536.0);
    }

    // DataTypeConversion: '<S322>/Data Type Conversion8'
    QuadModelv_B.eph = static_cast<uint16_T>(rtb_sincos_o1_idx_1 < 0.0 ?
      static_cast<int32_T>(static_cast<uint16_T>(-static_cast<int16_T>(
      static_cast<uint16_T>(-rtb_sincos_o1_idx_1)))) : static_cast<int32_T>(
      static_cast<uint16_T>(rtb_sincos_o1_idx_1)));

    // DataTypeConversion: '<S322>/Data Type Conversion9' incorporates:
    //   Constant: '<S322>/ModelParam.GPSEpvFinal'
    //   Gain: '<S322>/Gain8'

    rtb_sincos_o1_idx_1 = rt_roundd_snf(QuadModelv_P.Gain8_Gain *
      QuadModelv_P.ModelParam_GPSEpvFinal);
    if (rtIsNaN(rtb_sincos_o1_idx_1) || rtIsInf(rtb_sincos_o1_idx_1)) {
      rtb_sincos_o1_idx_1 = 0.0;
    } else {
      rtb_sincos_o1_idx_1 = std::fmod(rtb_sincos_o1_idx_1, 65536.0);
    }

    // DataTypeConversion: '<S322>/Data Type Conversion9'
    QuadModelv_B.epv = static_cast<uint16_T>(rtb_sincos_o1_idx_1 < 0.0 ?
      static_cast<int32_T>(static_cast<uint16_T>(-static_cast<int16_T>(
      static_cast<uint16_T>(-rtb_sincos_o1_idx_1)))) : static_cast<int32_T>(
      static_cast<uint16_T>(rtb_sincos_o1_idx_1)));
  }

  // TransferFcn: '<S368>/Transfer Fcn1'
  rtb_TmpSignalConversionAtSFun_1 = QuadModelv_P.TransferFcn1_C *
    QuadModelv_X.TransferFcn1_CSTATE;

  // Sum: '<S322>/Sum3' incorporates:
  //   TransferFcn: '<S368>/Transfer Fcn4'

  rtb_TmpSignalConversionAtSFun_0 = QuadModelv_P.TransferFcn4_C *
    QuadModelv_X.TransferFcn4_CSTATE + QuadModelv_B.Product[0];
  rtb_TmpSignalConversionAtSFun_1 += QuadModelv_B.Product[1];

  // MATLAB Function: '<S322>/GenCogVel'
  // MATLAB Function 'OutputPort1/HILGPSModel/GenCogVel': '<S367>:1'
  // '<S367>:1:2' v=norm(u(1:2));
  uavlostJyz = 3.3121686421112381E-170;
  rtb_Product1_d = std::abs(rtb_TmpSignalConversionAtSFun_0);
  if (rtb_Product1_d > 3.3121686421112381E-170) {
    tz_idx_0 = 1.0;
    uavlostJyz = rtb_Product1_d;
  } else {
    rtb_Rn = rtb_Product1_d / 3.3121686421112381E-170;
    tz_idx_0 = rtb_Rn * rtb_Rn;
  }

  rtb_Product1_d = std::abs(rtb_TmpSignalConversionAtSFun_1);
  if (rtb_Product1_d > uavlostJyz) {
    rtb_Rn = uavlostJyz / rtb_Product1_d;
    tz_idx_0 = tz_idx_0 * rtb_Rn * rtb_Rn + 1.0;
    uavlostJyz = rtb_Product1_d;
  } else {
    rtb_Rn = rtb_Product1_d / uavlostJyz;
    tz_idx_0 += rtb_Rn * rtb_Rn;
  }

  tz_idx_0 = uavlostJyz * std::sqrt(tz_idx_0);

  // '<S367>:1:4' if v < 1
  if (tz_idx_0 < 1.0) {
    // '<S367>:1:5' cot = 0;
    uavlostJyz = 0.0;
  } else {
    // '<S367>:1:6' else
    // '<S367>:1:7' cot=atan2d(u(2),u(1));
    uavlostJyz = 57.295779513082323 * rt_atan2d_snf
      (rtb_TmpSignalConversionAtSFun_1, rtb_TmpSignalConversionAtSFun_0);
  }

  // '<S367>:1:10' if cot<0
  if (uavlostJyz < 0.0) {
    // '<S367>:1:11' cot=cot+360;
    uavlostJyz += 360.0;
  }

  // DataTypeConversion: '<S322>/Data Type Conversion3' incorporates:
  //   Gain: '<S322>/VeScale'
  //   Sum: '<S322>/Sum3'
  //   TransferFcn: '<S368>/Transfer Fcn2'

  rtb_sincos_o1_idx_1 = rt_roundd_snf(QuadModelv_P.VeScale_Gain *
    rtb_TmpSignalConversionAtSFun_0);
  if (rtIsNaN(rtb_sincos_o1_idx_1) || rtIsInf(rtb_sincos_o1_idx_1)) {
    rtb_sincos_o1_idx_1 = 0.0;
  } else {
    rtb_sincos_o1_idx_1 = std::fmod(rtb_sincos_o1_idx_1, 65536.0);
  }

  rtb_TmpSignalConversionAtSFun_0 = rt_roundd_snf(QuadModelv_P.VeScale_Gain *
    rtb_TmpSignalConversionAtSFun_1);
  if (rtIsNaN(rtb_TmpSignalConversionAtSFun_0) || rtIsInf
      (rtb_TmpSignalConversionAtSFun_0)) {
    rtb_TmpSignalConversionAtSFun_0 = 0.0;
  } else {
    rtb_TmpSignalConversionAtSFun_0 = std::fmod(rtb_TmpSignalConversionAtSFun_0,
      65536.0);
  }

  rtb_Rn = rt_roundd_snf((QuadModelv_P.TransferFcn2_C *
    QuadModelv_X.TransferFcn2_CSTATE + QuadModelv_B.Product[2]) *
    QuadModelv_P.VeScale_Gain);
  if (rtIsNaN(rtb_Rn) || rtIsInf(rtb_Rn)) {
    rtb_Rn = 0.0;
  } else {
    rtb_Rn = std::fmod(rtb_Rn, 65536.0);
  }

  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // DataTypeConversion: '<S322>/Data Type Conversion10'
    rtb_TmpSignalConversionAtSFun_1 = rt_roundd_snf(QuadModelv_B.GPS3Dfixed);
    if (rtIsNaN(rtb_TmpSignalConversionAtSFun_1) || rtIsInf
        (rtb_TmpSignalConversionAtSFun_1)) {
      rtb_TmpSignalConversionAtSFun_1 = 0.0;
    } else {
      rtb_TmpSignalConversionAtSFun_1 = std::fmod
        (rtb_TmpSignalConversionAtSFun_1, 256.0);
    }

    // DataTypeConversion: '<S322>/Data Type Conversion10'
    QuadModelv_B.fix_type = static_cast<uint8_T>(rtb_TmpSignalConversionAtSFun_1
      < 0.0 ? static_cast<int32_T>(static_cast<uint8_T>(-static_cast<int8_T>(
      static_cast<uint8_T>(-rtb_TmpSignalConversionAtSFun_1)))) : static_cast<
      int32_T>(static_cast<uint8_T>(rtb_TmpSignalConversionAtSFun_1)));

    // DataTypeConversion: '<S322>/Data Type Conversion11'
    rtb_TmpSignalConversionAtSFun_1 = rt_roundd_snf(QuadModelv_B.satellites);
    if (rtIsNaN(rtb_TmpSignalConversionAtSFun_1) || rtIsInf
        (rtb_TmpSignalConversionAtSFun_1)) {
      rtb_TmpSignalConversionAtSFun_1 = 0.0;
    } else {
      rtb_TmpSignalConversionAtSFun_1 = std::fmod
        (rtb_TmpSignalConversionAtSFun_1, 256.0);
    }

    // DataTypeConversion: '<S322>/Data Type Conversion11'
    QuadModelv_B.satellites_visible = static_cast<uint8_T>
      (rtb_TmpSignalConversionAtSFun_1 < 0.0 ? static_cast<int32_T>
       (static_cast<uint8_T>(-static_cast<int8_T>(static_cast<uint8_T>
          (-rtb_TmpSignalConversionAtSFun_1)))) : static_cast<int32_T>(
        static_cast<uint8_T>(rtb_TmpSignalConversionAtSFun_1)));
  }

  // Outport: '<Root>/MavHILGPS' incorporates:
  //   BusCreator: '<S322>/Bus Creator'

  QuadModelv_Y.MavHILGPS.time_usec = rtb_time_usec;
  QuadModelv_Y.MavHILGPS.lat = QuadModelv_B.lat;
  QuadModelv_Y.MavHILGPS.lon = QuadModelv_B.lon;
  QuadModelv_Y.MavHILGPS.alt = QuadModelv_B.alt;
  QuadModelv_Y.MavHILGPS.eph = QuadModelv_B.eph;
  QuadModelv_Y.MavHILGPS.epv = QuadModelv_B.epv;

  // DataTypeConversion: '<S322>/Data Type Conversion2' incorporates:
  //   Gain: '<S322>/VelScale'
  //   MATLAB Function: '<S322>/GenCogVel'

  rtb_TmpSignalConversionAtSFun_1 = rt_roundd_snf(QuadModelv_P.VelScale_Gain *
    tz_idx_0);
  if (rtIsNaN(rtb_TmpSignalConversionAtSFun_1) || rtIsInf
      (rtb_TmpSignalConversionAtSFun_1)) {
    rtb_TmpSignalConversionAtSFun_1 = 0.0;
  } else {
    rtb_TmpSignalConversionAtSFun_1 = std::fmod(rtb_TmpSignalConversionAtSFun_1,
      65536.0);
  }

  // Outport: '<Root>/MavHILGPS' incorporates:
  //   DataTypeConversion: '<S322>/Data Type Conversion2'
  //   DataTypeConversion: '<S322>/Data Type Conversion3'

  QuadModelv_Y.MavHILGPS.vel = static_cast<uint16_T>
    (rtb_TmpSignalConversionAtSFun_1 < 0.0 ? static_cast<int32_T>
     (static_cast<uint16_T>(-static_cast<int16_T>(static_cast<uint16_T>
        (-rtb_TmpSignalConversionAtSFun_1)))) : static_cast<int32_T>(
      static_cast<uint16_T>(rtb_TmpSignalConversionAtSFun_1)));
  QuadModelv_Y.MavHILGPS.vn = static_cast<int16_T>(rtb_sincos_o1_idx_1 < 0.0 ?
    static_cast<int32_T>(static_cast<int16_T>(-static_cast<int16_T>
    (static_cast<uint16_T>(-rtb_sincos_o1_idx_1)))) : static_cast<int32_T>(
    static_cast<int16_T>(static_cast<uint16_T>(rtb_sincos_o1_idx_1))));
  QuadModelv_Y.MavHILGPS.ve = static_cast<int16_T>
    (rtb_TmpSignalConversionAtSFun_0 < 0.0 ? static_cast<int32_T>
     (static_cast<int16_T>(-static_cast<int16_T>(static_cast<uint16_T>
        (-rtb_TmpSignalConversionAtSFun_0)))) : static_cast<int32_T>(
      static_cast<int16_T>(static_cast<uint16_T>(rtb_TmpSignalConversionAtSFun_0))));
  QuadModelv_Y.MavHILGPS.vd = static_cast<int16_T>(rtb_Rn < 0.0 ?
    static_cast<int32_T>(static_cast<int16_T>(-static_cast<int16_T>(static_cast<
    uint16_T>(-rtb_Rn)))) : static_cast<int32_T>(static_cast<int16_T>(
    static_cast<uint16_T>(rtb_Rn))));

  // DataTypeConversion: '<S322>/Data Type Conversion7' incorporates:
  //   Gain: '<S322>/AngleScale'
  //   MATLAB Function: '<S322>/GenCogVel'

  rtb_sincos_o1_idx_1 = rt_roundd_snf(QuadModelv_P.AngleScale_Gain * uavlostJyz);
  if (rtIsNaN(rtb_sincos_o1_idx_1) || rtIsInf(rtb_sincos_o1_idx_1)) {
    rtb_sincos_o1_idx_1 = 0.0;
  } else {
    rtb_sincos_o1_idx_1 = std::fmod(rtb_sincos_o1_idx_1, 65536.0);
  }

  // Outport: '<Root>/MavHILGPS' incorporates:
  //   BusCreator: '<S322>/Bus Creator'
  //   DataTypeConversion: '<S322>/Data Type Conversion7'

  QuadModelv_Y.MavHILGPS.cog = static_cast<uint16_T>(rtb_sincos_o1_idx_1 < 0.0 ?
    static_cast<int32_T>(static_cast<uint16_T>(-static_cast<int16_T>(
    static_cast<uint16_T>(-rtb_sincos_o1_idx_1)))) : static_cast<int32_T>(
    static_cast<uint16_T>(rtb_sincos_o1_idx_1)));
  QuadModelv_Y.MavHILGPS.fix_type = QuadModelv_B.fix_type;
  QuadModelv_Y.MavHILGPS.satellites_visible = QuadModelv_B.satellites_visible;
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // DataTypeConversion: '<S322>/Data Type Conversion14' incorporates:
    //   Constant: '<S322>/ModelParam.GPSEphFinal'
    //   Gain: '<S322>/Gain1'

    rtb_sincos_o1_idx_1 = rt_roundd_snf(QuadModelv_P.Gain1_Gain_c *
      QuadModelv_P.ModelParam_GPSEphFinal);
    if (rtIsNaN(rtb_sincos_o1_idx_1) || rtIsInf(rtb_sincos_o1_idx_1)) {
      rtb_sincos_o1_idx_1 = 0.0;
    } else {
      rtb_sincos_o1_idx_1 = std::fmod(rtb_sincos_o1_idx_1, 4.294967296E+9);
    }

    // DataTypeConversion: '<S322>/Data Type Conversion14'
    QuadModelv_B.hAcc = rtb_sincos_o1_idx_1 < 0.0 ? static_cast<uint32_T>(-
      static_cast<int32_T>(static_cast<uint32_T>(-rtb_sincos_o1_idx_1))) :
      static_cast<uint32_T>(rtb_sincos_o1_idx_1);

    // DataTypeConversion: '<S322>/Data Type Conversion15' incorporates:
    //   Constant: '<S322>/ModelParam.GPSEpvFinal'
    //   Gain: '<S322>/Gain3'

    rtb_sincos_o1_idx_1 = rt_roundd_snf(QuadModelv_P.Gain3_Gain_l *
      QuadModelv_P.ModelParam_GPSEpvFinal);
    if (rtIsNaN(rtb_sincos_o1_idx_1) || rtIsInf(rtb_sincos_o1_idx_1)) {
      rtb_sincos_o1_idx_1 = 0.0;
    } else {
      rtb_sincos_o1_idx_1 = std::fmod(rtb_sincos_o1_idx_1, 4.294967296E+9);
    }

    // DataTypeConversion: '<S322>/Data Type Conversion15'
    QuadModelv_B.vAcc = rtb_sincos_o1_idx_1 < 0.0 ? static_cast<uint32_T>(-
      static_cast<int32_T>(static_cast<uint32_T>(-rtb_sincos_o1_idx_1))) :
      static_cast<uint32_T>(rtb_sincos_o1_idx_1);
  }

  // DataTypeConversion: '<S322>/Data Type Conversion1' incorporates:
  //   Gain: '<S322>/VeScale1'

  rtb_sincos_o1_idx_1 = rt_roundd_snf(QuadModelv_P.VeScale1_Gain *
    QuadModelv_B.Product[0]);
  if (rtIsNaN(rtb_sincos_o1_idx_1) || rtIsInf(rtb_sincos_o1_idx_1)) {
    rtb_sincos_o1_idx_1 = 0.0;
  } else {
    rtb_sincos_o1_idx_1 = std::fmod(rtb_sincos_o1_idx_1, 4.294967296E+9);
  }

  rtb_TmpSignalConversionAtSFun_0 = rt_roundd_snf(QuadModelv_P.VeScale1_Gain *
    QuadModelv_B.Product[1]);
  if (rtIsNaN(rtb_TmpSignalConversionAtSFun_0) || rtIsInf
      (rtb_TmpSignalConversionAtSFun_0)) {
    rtb_TmpSignalConversionAtSFun_0 = 0.0;
  } else {
    rtb_TmpSignalConversionAtSFun_0 = std::fmod(rtb_TmpSignalConversionAtSFun_0,
      4.294967296E+9);
  }

  rtb_Rn = rt_roundd_snf(QuadModelv_P.VeScale1_Gain * QuadModelv_B.Product[2]);
  if (rtIsNaN(rtb_Rn) || rtIsInf(rtb_Rn)) {
    rtb_Rn = 0.0;
  } else {
    rtb_Rn = std::fmod(rtb_Rn, 4.294967296E+9);
  }

  // MATLAB Function: '<S322>/-pi-pi---->0-2pi'
  // //rad
  // MATLAB Function 'OutputPort1/HILGPSModel/-pi-pi---->0-2pi': '<S365>:1'
  // '<S365>:1:3' if yaw180>0
  if (VectorConcatenate_n_idx_0 > 0.0) {
    // '<S365>:1:4' yaw360 = yaw180;
    rtb_Product1_d = VectorConcatenate_n_idx_0;
  } else {
    // '<S365>:1:5' else
    // '<S365>:1:6' yaw360 = yaw180 +2*pi;
    rtb_Product1_d = VectorConcatenate_n_idx_0 + 6.2831853071795862;
  }

  // End of MATLAB Function: '<S322>/-pi-pi---->0-2pi'

  // Outport: '<Root>/PXIHILGPSData' incorporates:
  //   BusCreator: '<S322>/Bus Creator1'
  //   DataTypeConversion: '<S322>/Data Type Conversion1'

  QuadModelv_Y.PXIHILGPSData.fix_type = QuadModelv_B.fix_type;
  QuadModelv_Y.PXIHILGPSData.satellites_visible =
    QuadModelv_B.satellites_visible;
  QuadModelv_Y.PXIHILGPSData.lat = QuadModelv_B.lat;
  QuadModelv_Y.PXIHILGPSData.lon = QuadModelv_B.lon;
  QuadModelv_Y.PXIHILGPSData.alt = QuadModelv_B.alt;
  QuadModelv_Y.PXIHILGPSData.hAcc = QuadModelv_B.hAcc;
  QuadModelv_Y.PXIHILGPSData.vAcc = QuadModelv_B.vAcc;
  QuadModelv_Y.PXIHILGPSData.velN = rtb_sincos_o1_idx_1 < 0.0 ?
    -static_cast<int32_T>(static_cast<uint32_T>(-rtb_sincos_o1_idx_1)) :
    static_cast<int32_T>(static_cast<uint32_T>(rtb_sincos_o1_idx_1));
  QuadModelv_Y.PXIHILGPSData.velE = rtb_TmpSignalConversionAtSFun_0 < 0.0 ? -
    static_cast<int32_T>(static_cast<uint32_T>(-rtb_TmpSignalConversionAtSFun_0))
    : static_cast<int32_T>(static_cast<uint32_T>(rtb_TmpSignalConversionAtSFun_0));
  QuadModelv_Y.PXIHILGPSData.velD = rtb_Rn < 0.0 ? -static_cast<int32_T>(
    static_cast<uint32_T>(-rtb_Rn)) : static_cast<int32_T>(static_cast<uint32_T>
    (rtb_Rn));

  // DataTypeConversion: '<S322>/Data Type Conversion12' incorporates:
  //   Gain: '<S322>/VelScale1'
  //   MATLAB Function: '<S322>/GenCogVel'

  rtb_sincos_o1_idx_1 = rt_roundd_snf(QuadModelv_P.VelScale1_Gain * tz_idx_0);
  if (rtIsNaN(rtb_sincos_o1_idx_1) || rtIsInf(rtb_sincos_o1_idx_1)) {
    rtb_sincos_o1_idx_1 = 0.0;
  } else {
    rtb_sincos_o1_idx_1 = std::fmod(rtb_sincos_o1_idx_1, 4.294967296E+9);
  }

  // Outport: '<Root>/PXIHILGPSData' incorporates:
  //   DataTypeConversion: '<S322>/Data Type Conversion12'

  QuadModelv_Y.PXIHILGPSData.gSpeed = rtb_sincos_o1_idx_1 < 0.0 ? -static_cast<
    int32_T>(static_cast<uint32_T>(-rtb_sincos_o1_idx_1)) : static_cast<int32_T>
    (static_cast<uint32_T>(rtb_sincos_o1_idx_1));

  // DataTypeConversion: '<S322>/Data Type Conversion13' incorporates:
  //   Gain: '<S322>/headMotAngle'
  //   MATLAB Function: '<S322>/GenCogVel'

  rtb_sincos_o1_idx_1 = rt_roundd_snf(QuadModelv_P.headMotAngle_Gain *
    uavlostJyz);
  if (rtIsNaN(rtb_sincos_o1_idx_1) || rtIsInf(rtb_sincos_o1_idx_1)) {
    rtb_sincos_o1_idx_1 = 0.0;
  } else {
    rtb_sincos_o1_idx_1 = std::fmod(rtb_sincos_o1_idx_1, 4.294967296E+9);
  }

  // Outport: '<Root>/PXIHILGPSData' incorporates:
  //   DataTypeConversion: '<S322>/Data Type Conversion13'

  QuadModelv_Y.PXIHILGPSData.headMot = rtb_sincos_o1_idx_1 < 0.0 ?
    -static_cast<int32_T>(static_cast<uint32_T>(-rtb_sincos_o1_idx_1)) :
    static_cast<int32_T>(static_cast<uint32_T>(rtb_sincos_o1_idx_1));

  // DataTypeConversion: '<S322>/Data Type Conversion16' incorporates:
  //   Gain: '<S322>/headVehAngleScale'

  rtb_sincos_o1_idx_1 = rt_roundd_snf(QuadModelv_P.headVehAngleScale_Gain *
    rtb_Product1_d);
  if (rtIsNaN(rtb_sincos_o1_idx_1) || rtIsInf(rtb_sincos_o1_idx_1)) {
    rtb_sincos_o1_idx_1 = 0.0;
  } else {
    rtb_sincos_o1_idx_1 = std::fmod(rtb_sincos_o1_idx_1, 4.294967296E+9);
  }

  // Outport: '<Root>/PXIHILGPSData' incorporates:
  //   DataTypeConversion: '<S322>/Data Type Conversion16'

  QuadModelv_Y.PXIHILGPSData.headVeh = rtb_sincos_o1_idx_1 < 0.0 ?
    -static_cast<int32_T>(static_cast<uint32_T>(-rtb_sincos_o1_idx_1)) :
    static_cast<int32_T>(static_cast<uint32_T>(rtb_sincos_o1_idx_1));

  // DataTypeConversion: '<S398>/Data Type Conversion1' incorporates:
  //   Gain: '<S398>/magDecGain'

  rtb_sincos_o1_idx_1 = std::floor(QuadModelv_P.magDecGain_Gain * rtb_q3dot);
  if (rtIsNaN(rtb_sincos_o1_idx_1) || rtIsInf(rtb_sincos_o1_idx_1)) {
    rtb_sincos_o1_idx_1 = 0.0;
  } else {
    rtb_sincos_o1_idx_1 = std::fmod(rtb_sincos_o1_idx_1, 65536.0);
  }

  // Outport: '<Root>/magDec' incorporates:
  //   DataTypeConversion: '<S398>/Data Type Conversion1'

  QuadModelv_Y.magDec = static_cast<int16_T>(rtb_sincos_o1_idx_1 < 0.0 ?
    static_cast<int32_T>(static_cast<int16_T>(-static_cast<int16_T>
    (static_cast<uint16_T>(-rtb_sincos_o1_idx_1)))) : static_cast<int32_T>(
    static_cast<int16_T>(static_cast<uint16_T>(rtb_sincos_o1_idx_1))));

  // Outport: '<Root>/SysTime'
  QuadModelv_Y.SysTime = rtb_time_usec;

  // Outport: '<Root>/VelE'
  QuadModelv_Y.VelE[0] = QuadModelv_B.Product[0];

  // Outport: '<Root>/PosE' incorporates:
  //   Integrator: '<S14>/xe,ye,ze'

  QuadModelv_Y.PosE[0] = QuadModelv_X.xeyeze_CSTATE[0];

  // Outport: '<Root>/VelE'
  QuadModelv_Y.VelE[1] = QuadModelv_B.Product[1];

  // Outport: '<Root>/PosE' incorporates:
  //   Integrator: '<S14>/xe,ye,ze'

  QuadModelv_Y.PosE[1] = QuadModelv_X.xeyeze_CSTATE[1];

  // Outport: '<Root>/VelE'
  QuadModelv_Y.VelE[2] = QuadModelv_B.Product[2];

  // Outport: '<Root>/PosE'
  QuadModelv_Y.PosE[2] = rtb_sincos_o1_f[2];

  // Outport: '<Root>/AngEuler'
  QuadModelv_Y.AngEuler[0] = VectorConcatenate_n_idx_2;
  QuadModelv_Y.AngEuler[1] = VectorConcatenate_n_idx_1;
  QuadModelv_Y.AngEuler[2] = VectorConcatenate_n_idx_0;

  // Outport: '<Root>/AngQuatern'
  QuadModelv_Y.AngQuatern[0] = QuadModelv_B.Merge_f[0];
  QuadModelv_Y.AngQuatern[1] = QuadModelv_B.Merge_f[1];
  QuadModelv_Y.AngQuatern[2] = QuadModelv_B.Merge_f[2];
  QuadModelv_Y.AngQuatern[3] = QuadModelv_B.Merge_f[3];

  // Outport: '<Root>/RateB' incorporates:
  //   Integrator: '<S14>/p,q,r '

  QuadModelv_Y.RateB[0] = QuadModelv_X.pqr_CSTATE[0];

  // Sum: '<S1>/Sum1'
  rtb_Sum1_ki[0] += QuadModelv_B.fm[0];

  // Outport: '<Root>/RateB' incorporates:
  //   Integrator: '<S14>/p,q,r '

  QuadModelv_Y.RateB[1] = QuadModelv_X.pqr_CSTATE[1];

  // Sum: '<S1>/Sum1'
  rtb_Sum1_ki[1] += QuadModelv_B.fm[1];

  // Outport: '<Root>/RateB' incorporates:
  //   Integrator: '<S14>/p,q,r '

  QuadModelv_Y.RateB[2] = QuadModelv_X.pqr_CSTATE[2];
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // Outputs for Iterator SubSystem: '<S53>/For Each Subsystem' incorporates:
    //   ForEach: '<S54>/For Each'

    for (ForEach_itr = 0; ForEach_itr < 1; ForEach_itr++) {
      // ForEachSliceAssignment generated from: '<S54>/F' incorporates:
      //   ForEachSliceSelector generated from: '<S54>/mdot'
      //   Product: '<S54>/Product'
      //   SignalConversion generated from: '<S53>/For Each Subsystem'

      rtb_DataTypeConversion1[0] = 0.0;
      rtb_DataTypeConversion1[1] = 0.0;
      rtb_DataTypeConversion1[2] = 0.0;
    }

    // End of Outputs for SubSystem: '<S53>/For Each Subsystem'

    // Sum: '<S53>/Sum of Elements'
    QuadModelv_B.SumofElements[0] = 0.0;
    QuadModelv_B.SumofElements[1] = 0.0;
    QuadModelv_B.SumofElements[2] = 0.0;
  }

  // Product: '<S56>/j x i' incorporates:
  //   Integrator: '<S14>/p,q,r '
  //   Integrator: '<S14>/ub,vb,wb'

  rtb_TmpSignalConversionAtSFun_1 = QuadModelv_X.pqr_CSTATE[0] *
    QuadModelv_X.ubvbwb_CSTATE[1];

  // Sum: '<S14>/Sum' incorporates:
  //   Integrator: '<S14>/p,q,r '
  //   Integrator: '<S14>/ub,vb,wb'
  //   Product: '<S14>/Product'
  //   Product: '<S55>/i x j'
  //   Product: '<S55>/j x k'
  //   Product: '<S55>/k x i'
  //   Product: '<S56>/i x k'
  //   Product: '<S56>/k x j'
  //   Sum: '<S17>/Sum'
  //   Sum: '<S18>/Sum'
  //   Sum: '<S1>/Sum1'
  //   Sum: '<S7>/Add2'

  QuadModelv_B.Sum_p[0] = (rtb_Sum1_ki[0] + QuadModelv_B.SumofElements[0]) /
    QuadModelv_B.m + (QuadModelv_X.ubvbwb_CSTATE[1] * QuadModelv_X.pqr_CSTATE[2]
                      - QuadModelv_X.pqr_CSTATE[1] * QuadModelv_X.ubvbwb_CSTATE
                      [2]);
  QuadModelv_B.Sum_p[1] = (rtb_Sum1_ki[1] + QuadModelv_B.SumofElements[1]) /
    QuadModelv_B.m + (QuadModelv_X.pqr_CSTATE[0] * QuadModelv_X.ubvbwb_CSTATE[2]
                      - QuadModelv_X.ubvbwb_CSTATE[0] * QuadModelv_X.pqr_CSTATE
                      [2]);
  QuadModelv_B.Sum_p[2] = ((QuadModelv_B.fm[2] + VectorConcatenate_tmp_0) +
    QuadModelv_B.SumofElements[2]) / QuadModelv_B.m +
    (QuadModelv_X.ubvbwb_CSTATE[0] * QuadModelv_X.pqr_CSTATE[1] -
     rtb_TmpSignalConversionAtSFun_1);

  // Outport: '<Root>/AccB'
  QuadModelv_Y.AccB[0] = QuadModelv_B.Sum_p[0];
  QuadModelv_Y.AccB[1] = QuadModelv_B.Sum_p[1];
  QuadModelv_Y.AccB[2] = QuadModelv_B.Sum_p[2];
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // MATLAB Function: '<S12>/Model Fail Assessment' incorporates:
    //   Constant: '<S12>/ModelFailEnable'
    //   Integrator: '<S14>/p,q,r '
    //   Integrator: '<S14>/xe,ye,ze'
    //   SignalConversion generated from: '<S325>/ SFunction '

    // MATLAB Function 'OutputPort1/Model Fail Assessment': '<S325>:1'
    // '<S325>:1:4' if isempty(xTemp)
    // '<S325>:1:9' if isempty(kk)
    // '<S325>:1:14' if isempty(ss)
    // '<S325>:1:18' if ~ModelFailEnable&&(kk<0.5)
    if ((!(QuadModelv_P.ModelFailEnable != 0.0)) && (QuadModelv_DW.kk < 0.5)) {
      // '<S325>:1:19' xTemp=zeros(8,1);
      std::memset(&QuadModelv_DW.xTemp[0], 0, sizeof(real_T) << 3U);

      // '<S325>:1:20' kk=0;
      QuadModelv_DW.kk = 0.0;

      // '<S325>:1:21' ss=[xe;ang];
      QuadModelv_DW.ss[0] = QuadModelv_X.xeyeze_CSTATE[0];
      QuadModelv_DW.ss[3] = VectorConcatenate_n_idx_2;
      QuadModelv_DW.ss[1] = QuadModelv_X.xeyeze_CSTATE[1];
      QuadModelv_DW.ss[4] = VectorConcatenate_n_idx_1;
      QuadModelv_DW.ss[2] = rtb_sincos_o1_f[2];
      QuadModelv_DW.ss[5] = VectorConcatenate_n_idx_0;
    }

    // '<S325>:1:24' if ModelFailEnable&&~landed
    if ((QuadModelv_P.ModelFailEnable != 0.0) && (rtb_landed == 0)) {
      // '<S325>:1:25' xTemp(1:4,1)=[norm(xe-ss(1:3,1));norm(ve);norm(ang-ss(4:6,1));norm(wb)]; 
      rtb_Sum1_ki[0] = QuadModelv_X.xeyeze_CSTATE[0] - QuadModelv_DW.ss[0];
      rtb_DataTypeConversion1[0] = VectorConcatenate_n_idx_2 - QuadModelv_DW.ss
        [3];
      rtb_Sum1_ki[1] = QuadModelv_X.xeyeze_CSTATE[1] - QuadModelv_DW.ss[1];
      rtb_DataTypeConversion1[1] = VectorConcatenate_n_idx_1 - QuadModelv_DW.ss
        [4];
      rtb_Sum1_ki[2] = rtb_sincos_o1_f[2] - QuadModelv_DW.ss[2];
      rtb_DataTypeConversion1[2] = VectorConcatenate_n_idx_0 - QuadModelv_DW.ss
        [5];
      QuadModelv_DW.xTemp[0] = QuadModelv_norm(rtb_Sum1_ki);
      QuadModelv_DW.xTemp[1] = QuadModelv_norm(QuadModelv_B.Product);
      QuadModelv_DW.xTemp[2] = QuadModelv_norm(rtb_DataTypeConversion1);
      QuadModelv_DW.xTemp[3] = QuadModelv_norm(QuadModelv_X.pqr_CSTATE);

      // '<S325>:1:26' xTemp(5:8,1)= (xTemp(5:8,1)*kk+xTemp(1:4,1))/(kk+1);
      QuadModelv_DW.xTemp[4] = (QuadModelv_DW.xTemp[4] * QuadModelv_DW.kk +
        QuadModelv_DW.xTemp[0]) / (QuadModelv_DW.kk + 1.0);
      QuadModelv_DW.xTemp[5] = (QuadModelv_DW.xTemp[5] * QuadModelv_DW.kk +
        QuadModelv_DW.xTemp[1]) / (QuadModelv_DW.kk + 1.0);
      QuadModelv_DW.xTemp[6] = (QuadModelv_DW.xTemp[6] * QuadModelv_DW.kk +
        QuadModelv_DW.xTemp[2]) / (QuadModelv_DW.kk + 1.0);
      QuadModelv_DW.xTemp[7] = (QuadModelv_DW.xTemp[7] * QuadModelv_DW.kk +
        QuadModelv_DW.xTemp[3]) / (QuadModelv_DW.kk + 1.0);

      // '<S325>:1:28' kk=kk+1;
      QuadModelv_DW.kk++;
    }

    // Outport: '<Root>/FailInfo' incorporates:
    //   MATLAB Function: '<S12>/Model Fail Assessment'

    // landed = int8(0);
    // '<S325>:1:32' FailInfo=xTemp;
    std::memcpy(&QuadModelv_Y.FailInfo[0], &QuadModelv_DW.xTemp[0], sizeof
                (real_T) << 3U);

    // Outport: '<Root>/landed'
    QuadModelv_Y.landed = rtb_landed;

    // Gain: '<S12>/Gain'
    for (i = 0; i < 8; i++) {
      rtb_Gain_ok[i] = QuadModelv_P.Gain_Gain_ja * QuadModelv_B.y[i];
    }

    // End of Gain: '<S12>/Gain'

    // Outport: '<Root>/MotorRPMS'
    std::memcpy(&QuadModelv_Y.MotorRPMS[0], &rtb_Gain_ok[0], sizeof(real_T) <<
                3U);
  }

  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0) {
    // Outport: '<Root>/BattCellVoltage'
    QuadModelv_Y.BattCellVoltage = hover_current;

    // Outport: '<Root>/BattRemCapacity'
    QuadModelv_Y.BattRemCapacity = discharge_capacity;
  }

  // Outport: '<Root>/wind'
  QuadModelv_Y.wind[0] = rtb_DataTypeConversion2[0];
  QuadModelv_Y.wind[1] = rtb_DataTypeConversion2[1];
  QuadModelv_Y.wind[2] = rtb_DataTypeConversion2[2];
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // DataTypeConversion: '<S12>/Data Type Conversion' incorporates:
    //   Constant: '<S12>/CopterID'

    rtb_sincos_o1_idx_1 = std::floor(QuadModelv_P.CopterID_Value);
    if (rtIsNaN(rtb_sincos_o1_idx_1) || rtIsInf(rtb_sincos_o1_idx_1)) {
      rtb_sincos_o1_idx_1 = 0.0;
    } else {
      rtb_sincos_o1_idx_1 = std::fmod(rtb_sincos_o1_idx_1, 4.294967296E+9);
    }

    // DataTypeConversion: '<S12>/Data Type Conversion'
    QuadModelv_B.copterID = rtb_sincos_o1_idx_1 < 0.0 ? -static_cast<int32_T>(
      static_cast<uint32_T>(-rtb_sincos_o1_idx_1)) : static_cast<int32_T>(
      static_cast<uint32_T>(rtb_sincos_o1_idx_1));

    // DataTypeConversion: '<S12>/Data Type Conversion2' incorporates:
    //   Constant: '<S12>/UAVType'

    QuadModelv_B.vehicleType = QuadModelv_P.ModelParam_uavType;
    for (i = 0; i < 8; i++) {
      // DataTypeConversion: '<S12>/Data Type Conversion8'
      QuadModelv_B.MotorRPMS[i] = static_cast<real32_T>(rtb_Gain_ok[i]);
    }
  }

  // Outport: '<Root>/MavVehile3DInfo' incorporates:
  //   BusCreator generated from: '<Root>/MavVehile3DInfo'
  //   Clock: '<S12>/Clock1'
  //   DataTypeConversion: '<S12>/Data Type Conversion4'
  //   DataTypeConversion: '<S12>/Data Type Conversion5'
  //   DataTypeConversion: '<S12>/Data Type Conversion6'
  //   DataTypeConversion: '<S12>/Data Type Conversion7'
  //   DataTypeConversion: '<S12>/Data Type Conversion9'
  //   DataTypeConversion: '<S324>/Data Type Conversion3'
  //   Integrator: '<S14>/p,q,r '
  //   Integrator: '<S14>/xe,ye,ze'

  QuadModelv_Y.MavVehile3DInfo.copterID = QuadModelv_B.copterID;
  QuadModelv_Y.MavVehile3DInfo.vehicleType = QuadModelv_B.vehicleType;
  QuadModelv_Y.MavVehile3DInfo.runnedTime = rtb_Clock1_tmp;
  QuadModelv_Y.MavVehile3DInfo.VelE[0] = static_cast<real32_T>
    (QuadModelv_B.Product[0]);
  QuadModelv_Y.MavVehile3DInfo.PosE[0] = static_cast<real32_T>
    (QuadModelv_X.xeyeze_CSTATE[0]);
  QuadModelv_Y.MavVehile3DInfo.VelE[1] = static_cast<real32_T>
    (QuadModelv_B.Product[1]);
  QuadModelv_Y.MavVehile3DInfo.PosE[1] = static_cast<real32_T>
    (QuadModelv_X.xeyeze_CSTATE[1]);
  QuadModelv_Y.MavVehile3DInfo.VelE[2] = static_cast<real32_T>
    (QuadModelv_B.Product[2]);
  QuadModelv_Y.MavVehile3DInfo.PosE[2] = static_cast<real32_T>(rtb_sincos_o1_f[2]);
  QuadModelv_Y.MavVehile3DInfo.AngEuler[0] = static_cast<real32_T>
    (VectorConcatenate_n_idx_2);
  QuadModelv_Y.MavVehile3DInfo.AngEuler[1] = static_cast<real32_T>
    (VectorConcatenate_n_idx_1);
  QuadModelv_Y.MavVehile3DInfo.AngEuler[2] = static_cast<real32_T>
    (VectorConcatenate_n_idx_0);
  QuadModelv_Y.MavVehile3DInfo.AngQuatern[0] = static_cast<real32_T>
    (QuadModelv_B.Merge_f[0]);
  QuadModelv_Y.MavVehile3DInfo.AngQuatern[1] = static_cast<real32_T>
    (QuadModelv_B.Merge_f[1]);
  QuadModelv_Y.MavVehile3DInfo.AngQuatern[2] = static_cast<real32_T>
    (QuadModelv_B.Merge_f[2]);
  QuadModelv_Y.MavVehile3DInfo.AngQuatern[3] = static_cast<real32_T>
    (QuadModelv_B.Merge_f[3]);
  for (i = 0; i < 8; i++) {
    QuadModelv_Y.MavVehile3DInfo.MotorRPMS[i] = QuadModelv_B.MotorRPMS[i];
  }

  QuadModelv_Y.MavVehile3DInfo.AccB[0] = static_cast<real32_T>
    (QuadModelv_B.Sum_p[0]);
  QuadModelv_Y.MavVehile3DInfo.RateB[0] = static_cast<real32_T>
    (QuadModelv_X.pqr_CSTATE[0]);
  QuadModelv_Y.MavVehile3DInfo.PosGPS[0] = LoadLeakRatio;

  // DataTypeConversion: '<S8>/Data Type Conversion2' incorporates:
  //   DataTypeConversion: '<S12>/Data Type Conversion4'

  rtb_DataTypeConversion2[0] = static_cast<real32_T>(QuadModelv_B.Product[0]);

  // Outport: '<Root>/outCopterData' incorporates:
  //   DataTypeConversion: '<S12>/Data Type Conversion4'
  //   DataTypeConversion: '<S12>/Data Type Conversion9'
  //   DataTypeConversion: '<S324>/Data Type Conversion3'
  //   Integrator: '<S14>/p,q,r '

  QuadModelv_Y.outCopterData[0] = static_cast<real32_T>(QuadModelv_B.Sum_p[0]);
  QuadModelv_Y.outCopterData[3] = static_cast<real32_T>(QuadModelv_X.pqr_CSTATE
    [0]);
  QuadModelv_Y.outCopterData[6] = static_cast<real32_T>(QuadModelv_B.Product[0]);

  // Outport: '<Root>/MavVehile3DInfo' incorporates:
  //   DataTypeConversion: '<S12>/Data Type Conversion9'
  //   DataTypeConversion: '<S324>/Data Type Conversion3'
  //   Integrator: '<S14>/p,q,r '
  //   SignalConversion generated from: '<Root>/MavVehile3DInfo'

  QuadModelv_Y.MavVehile3DInfo.AccB[1] = static_cast<real32_T>
    (QuadModelv_B.Sum_p[1]);
  QuadModelv_Y.MavVehile3DInfo.RateB[1] = static_cast<real32_T>
    (QuadModelv_X.pqr_CSTATE[1]);
  QuadModelv_Y.MavVehile3DInfo.PosGPS[1] = Z;

  // DataTypeConversion: '<S8>/Data Type Conversion2' incorporates:
  //   DataTypeConversion: '<S12>/Data Type Conversion4'

  rtb_DataTypeConversion2[1] = static_cast<real32_T>(QuadModelv_B.Product[1]);

  // Outport: '<Root>/outCopterData' incorporates:
  //   DataTypeConversion: '<S12>/Data Type Conversion4'
  //   DataTypeConversion: '<S12>/Data Type Conversion9'
  //   DataTypeConversion: '<S324>/Data Type Conversion3'
  //   Integrator: '<S14>/p,q,r '

  QuadModelv_Y.outCopterData[1] = static_cast<real32_T>(QuadModelv_B.Sum_p[1]);
  QuadModelv_Y.outCopterData[4] = static_cast<real32_T>(QuadModelv_X.pqr_CSTATE
    [1]);
  QuadModelv_Y.outCopterData[7] = static_cast<real32_T>(QuadModelv_B.Product[1]);

  // Outport: '<Root>/MavVehile3DInfo' incorporates:
  //   DataTypeConversion: '<S12>/Data Type Conversion9'
  //   DataTypeConversion: '<S324>/Data Type Conversion3'
  //   Integrator: '<S14>/p,q,r '
  //   SignalConversion generated from: '<Root>/MavVehile3DInfo'

  QuadModelv_Y.MavVehile3DInfo.AccB[2] = static_cast<real32_T>
    (QuadModelv_B.Sum_p[2]);
  QuadModelv_Y.MavVehile3DInfo.RateB[2] = static_cast<real32_T>
    (QuadModelv_X.pqr_CSTATE[2]);
  QuadModelv_Y.MavVehile3DInfo.PosGPS[2] = rtb_Saturation_1;

  // DataTypeConversion: '<S8>/Data Type Conversion2' incorporates:
  //   DataTypeConversion: '<S12>/Data Type Conversion4'

  rtb_DataTypeConversion2[2] = static_cast<real32_T>(QuadModelv_B.Product[2]);

  // Outport: '<Root>/outCopterData' incorporates:
  //   Constant: '<S8>/Constant'
  //   DataTypeConversion: '<S12>/Data Type Conversion4'
  //   DataTypeConversion: '<S12>/Data Type Conversion9'
  //   DataTypeConversion: '<S324>/Data Type Conversion3'
  //   Integrator: '<S14>/p,q,r '

  QuadModelv_Y.outCopterData[2] = static_cast<real32_T>(QuadModelv_B.Sum_p[2]);
  QuadModelv_Y.outCopterData[5] = static_cast<real32_T>(QuadModelv_X.pqr_CSTATE
    [2]);
  QuadModelv_Y.outCopterData[8] = static_cast<real32_T>(QuadModelv_B.Product[2]);
  std::memcpy(&QuadModelv_Y.outCopterData[9], &QuadModelv_P.Constant_Value_f[0],
              23U * sizeof(real_T));

  // Gain: '<S26>/High Gain Quaternion Normalization' incorporates:
  //   Constant: '<S26>/Constant'
  //   DotProduct: '<S26>/Dot Product'
  //   Integrator: '<S15>/q0 q1 q2 q3'
  //   Sum: '<S26>/Sum'

  rtb_Rn = (QuadModelv_P.Constant_Value_h - (((QuadModelv_X.q0q1q2q3_CSTATE[0] *
    QuadModelv_X.q0q1q2q3_CSTATE[0] + QuadModelv_X.q0q1q2q3_CSTATE[1] *
    QuadModelv_X.q0q1q2q3_CSTATE[1]) + QuadModelv_X.q0q1q2q3_CSTATE[2] *
              QuadModelv_X.q0q1q2q3_CSTATE[2]) + QuadModelv_X.q0q1q2q3_CSTATE[3]
             * QuadModelv_X.q0q1q2q3_CSTATE[3])) *
    QuadModelv_P.CustomVariableMass6DOFQuaternio;

  // SignalConversion generated from: '<S15>/q0 q1 q2 q3' incorporates:
  //   Fcn: '<S26>/q0dot'
  //   Fcn: '<S26>/q1dot'
  //   Fcn: '<S26>/q2dot'
  //   Fcn: '<S26>/q3dot'
  //   Integrator: '<S14>/p,q,r '
  //   Integrator: '<S15>/q0 q1 q2 q3'

  QuadModelv_B.TmpSignalConversionAtq0q1q2q3_c[0] = ((QuadModelv_X.pqr_CSTATE[0]
    * QuadModelv_X.q0q1q2q3_CSTATE[1] + QuadModelv_X.pqr_CSTATE[1] *
    QuadModelv_X.q0q1q2q3_CSTATE[2]) + QuadModelv_X.pqr_CSTATE[2] *
    QuadModelv_X.q0q1q2q3_CSTATE[3]) * -0.5 + rtb_Rn *
    QuadModelv_X.q0q1q2q3_CSTATE[0];
  QuadModelv_B.TmpSignalConversionAtq0q1q2q3_c[1] =
    ((QuadModelv_X.q0q1q2q3_CSTATE[0] * QuadModelv_X.pqr_CSTATE[0] +
      QuadModelv_X.q0q1q2q3_CSTATE[2] * QuadModelv_X.pqr_CSTATE[2]) -
     QuadModelv_X.pqr_CSTATE[1] * QuadModelv_X.q0q1q2q3_CSTATE[3]) * 0.5 +
    rtb_Rn * QuadModelv_X.q0q1q2q3_CSTATE[1];
  QuadModelv_B.TmpSignalConversionAtq0q1q2q3_c[2] =
    ((QuadModelv_X.q0q1q2q3_CSTATE[0] * QuadModelv_X.pqr_CSTATE[1] +
      QuadModelv_X.pqr_CSTATE[0] * QuadModelv_X.q0q1q2q3_CSTATE[3]) -
     QuadModelv_X.q0q1q2q3_CSTATE[1] * QuadModelv_X.pqr_CSTATE[2]) * 0.5 +
    rtb_Rn * QuadModelv_X.q0q1q2q3_CSTATE[2];
  QuadModelv_B.TmpSignalConversionAtq0q1q2q3_c[3] =
    ((QuadModelv_X.q0q1q2q3_CSTATE[0] * QuadModelv_X.pqr_CSTATE[2] +
      QuadModelv_X.q0q1q2q3_CSTATE[1] * QuadModelv_X.pqr_CSTATE[1]) -
     QuadModelv_X.pqr_CSTATE[0] * QuadModelv_X.q0q1q2q3_CSTATE[2]) * 0.5 +
    rtb_Rn * QuadModelv_X.q0q1q2q3_CSTATE[3];
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // UnitConversion: '<S111>/Unit Conversion' incorporates:
    //   Constant: '<S94>/Wingspan'

    // Unit Conversion - from: m to: ft
    // Expression: output = (3.28084*input) + (0)
    QuadModelv_B.UnitConversion_e = 3.280839895013123 *
      QuadModelv_P.DrydenWindTurbulenceModelDis_np;
  }

  // Outputs for Enabled SubSystem: '<S108>/Hpgw'
  // Constant: '<S108>/Constant1'
  QuadModelv_Hpgw(QuadModelv_P.DrydenWindTurbulenceModelDisc_k, frac_0,
                  QuadModelv_B.sigma_wg_p, rtb_Product3_o,
                  QuadModelv_B.Product_p[3], QuadModelv_B.UnitConversion_e,
                  &QuadModelv_B.Hpgw, &QuadModelv_DW.Hpgw, &QuadModelv_P.Hpgw);

  // End of Outputs for SubSystem: '<S108>/Hpgw'

  // Outputs for Enabled SubSystem: '<S108>/Hqgw'
  // Constant: '<S108>/Constant2'
  QuadModelv_Hqgw(QuadModelv_P.DrydenWindTurbulenceModelDisc_k,
                  QuadModelv_B.UnitConversion_a, QuadModelv_B.Hwgwz.Sum,
                  QuadModelv_B.UnitConversion_e, &QuadModelv_B.Hqgw,
                  &QuadModelv_DW.Hqgw, &QuadModelv_P.Hqgw);

  // End of Outputs for SubSystem: '<S108>/Hqgw'

  // Outputs for Enabled SubSystem: '<S108>/Hrgw'
  // Constant: '<S108>/Constant3'
  QuadModelv_Hrgw(QuadModelv_P.DrydenWindTurbulenceModelDisc_k,
                  QuadModelv_B.UnitConversion_a, QuadModelv_B.Hvgwz.Sum,
                  QuadModelv_B.UnitConversion_e, &QuadModelv_B.Hrgw,
                  &QuadModelv_DW.Hrgw, &QuadModelv_P.Hrgw);

  // End of Outputs for SubSystem: '<S108>/Hrgw'

  // If: '<S113>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' incorporates:
  //   Constant: '<S67>/Constant_DCM'

  if (rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    if (rtb_Product2_kl <= 1000.0) {
      rtAction = 0;
    } else if (rtb_Product2_kl >= 2000.0) {
      rtAction = 1;
    } else {
      rtAction = 2;
    }

    QuadModelv_DW.ifHeightMaxlowaltitudeelseifH_j = rtAction;
  } else {
    rtAction = QuadModelv_DW.ifHeightMaxlowaltitudeelseifH_j;
  }

  switch (rtAction) {
   case 0:
    // Outputs for IfAction SubSystem: '<S113>/Low altitude  rates' incorporates:
    //   ActionPort: '<S128>/Action Port'

    QuadModelv_Lowaltituderates(QuadModelv_P.Constant_DCM_Value,
      QuadModelv_B.Hpgw.Sum, QuadModelv_B.Hqgw.Sum1, QuadModelv_B.Hrgw.Sum1,
      QuadModelv_B.UnitConversion_ab, rtb_DataTypeConversion1);

    // End of Outputs for SubSystem: '<S113>/Low altitude  rates'
    break;

   case 1:
    break;

   default:
    // Outputs for IfAction SubSystem: '<S113>/Interpolate  rates' incorporates:
    //   ActionPort: '<S127>/Action Port'

    QuadModelv_Interpolaterates(QuadModelv_B.Hpgw.Sum, QuadModelv_B.Hqgw.Sum1,
      QuadModelv_B.Hrgw.Sum1, QuadModelv_P.Constant_DCM_Value,
      QuadModelv_B.UnitConversion_ab, rtb_Product2_kl, rtb_DataTypeConversion1,
      &QuadModelv_P.Interpolaterates);

    // End of Outputs for SubSystem: '<S113>/Interpolate  rates'
    break;
  }

  // End of If: '<S113>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' 
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // UnitConversion: '<S150>/Unit Conversion' incorporates:
    //   Constant: '<S95>/Wingspan'

    // Unit Conversion - from: m to: ft
    // Expression: output = (3.28084*input) + (0)
    QuadModelv_B.UnitConversion_f = 3.280839895013123 *
      QuadModelv_P.DrydenWindTurbulenceModelDisc_d;
  }

  // Outputs for Enabled SubSystem: '<S147>/Hpgw'
  // Constant: '<S147>/Constant1'
  QuadModelv_Hpgw(QuadModelv_P.DrydenWindTurbulenceModelDisc_g, rtb_Sum_d,
                  QuadModelv_B.sigma_wg, rtb_Add, QuadModelv_B.Product_d[3],
                  QuadModelv_B.UnitConversion_f, &QuadModelv_B.Hpgw_h,
                  &QuadModelv_DW.Hpgw_h, &QuadModelv_P.Hpgw_h);

  // End of Outputs for SubSystem: '<S147>/Hpgw'

  // Outputs for Enabled SubSystem: '<S147>/Hqgw'
  // Constant: '<S147>/Constant2'
  QuadModelv_Hqgw(QuadModelv_P.DrydenWindTurbulenceModelDisc_g,
                  QuadModelv_B.UnitConversion, QuadModelv_B.Hwgwz_a.Sum,
                  QuadModelv_B.UnitConversion_f, &QuadModelv_B.Hqgw_a,
                  &QuadModelv_DW.Hqgw_a, &QuadModelv_P.Hqgw_a);

  // End of Outputs for SubSystem: '<S147>/Hqgw'

  // Outputs for Enabled SubSystem: '<S147>/Hrgw'
  // Constant: '<S147>/Constant3'
  QuadModelv_Hrgw(QuadModelv_P.DrydenWindTurbulenceModelDisc_g,
                  QuadModelv_B.UnitConversion, QuadModelv_B.Hvgwz_f.Sum,
                  QuadModelv_B.UnitConversion_f, &QuadModelv_B.Hrgw_a,
                  &QuadModelv_DW.Hrgw_a, &QuadModelv_P.Hrgw_a);

  // End of Outputs for SubSystem: '<S147>/Hrgw'

  // If: '<S152>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' incorporates:
  //   Constant: '<S67>/Constant_DCM'
  //   UnitConversion: '<S149>/Unit Conversion'

  if (rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
    if (VectorConcatenate_tmp <= 1000.0) {
      rtAction = 0;
    } else if (VectorConcatenate_tmp >= 2000.0) {
      rtAction = 1;
    } else {
      rtAction = 2;
    }

    QuadModelv_DW.ifHeightMaxlowaltitudeelseifH_e = rtAction;
  } else {
    rtAction = QuadModelv_DW.ifHeightMaxlowaltitudeelseifH_e;
  }

  switch (rtAction) {
   case 0:
    // Outputs for IfAction SubSystem: '<S152>/Low altitude  rates' incorporates:
    //   ActionPort: '<S167>/Action Port'

    QuadModelv_Lowaltituderates(QuadModelv_P.Constant_DCM_Value,
      QuadModelv_B.Hpgw_h.Sum, QuadModelv_B.Hqgw_a.Sum1,
      QuadModelv_B.Hrgw_a.Sum1, QuadModelv_B.UnitConversion_p,
      rtb_DataTypeConversion1);

    // End of Outputs for SubSystem: '<S152>/Low altitude  rates'
    break;

   case 1:
    break;

   default:
    // Outputs for IfAction SubSystem: '<S152>/Interpolate  rates' incorporates:
    //   ActionPort: '<S166>/Action Port'

    QuadModelv_Interpolaterates(QuadModelv_B.Hpgw_h.Sum,
      QuadModelv_B.Hqgw_a.Sum1, QuadModelv_B.Hrgw_a.Sum1,
      QuadModelv_P.Constant_DCM_Value, QuadModelv_B.UnitConversion_p,
      VectorConcatenate_tmp, rtb_DataTypeConversion1,
      &QuadModelv_P.Interpolaterates_g);

    // End of Outputs for SubSystem: '<S152>/Interpolate  rates'
    break;
  }

  // End of If: '<S152>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' 
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // If: '<S203>/If1' incorporates:
    //   Constant: '<S203>/Constant'

    rtAction = -1;
    if (rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
      if (QuadModelv_P.DirectionCosineMatrixtoQuaterni != 1.0) {
        rtAction = 0;
      }

      QuadModelv_DW.If1_ActiveSubsystem = rtAction;
    } else {
      rtAction = QuadModelv_DW.If1_ActiveSubsystem;
    }

    if (rtAction == 0) {
      // Outputs for IfAction SubSystem: '<S203>/If Warning//Error' incorporates:
      //   ActionPort: '<S227>/if'

      QuadModelv_IfWarningError(VectorConcatenate,
        QuadModelv_P.DirectionCosineMatrixtoQuaterni,
        QuadModelv_P.DirectionCosineMatrixtoQuater_i,
        &QuadModelv_P.IfWarningError);

      // End of Outputs for SubSystem: '<S203>/If Warning//Error'
    }

    // End of If: '<S203>/If1'

    // If: '<S241>/If1' incorporates:
    //   Constant: '<S241>/Constant'

    rtAction = -1;
    if (rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
      if (QuadModelv_P.DirectionCosineMatrixtoQuater_p != 1.0) {
        rtAction = 0;
      }

      QuadModelv_DW.If1_ActiveSubsystem_o = rtAction;
    } else {
      rtAction = QuadModelv_DW.If1_ActiveSubsystem_o;
    }

    if (rtAction == 0) {
      // Outputs for IfAction SubSystem: '<S241>/If Warning//Error' incorporates:
      //   ActionPort: '<S265>/if'

      QuadModelv_IfWarningError(VectorConcatenate_p,
        QuadModelv_P.DirectionCosineMatrixtoQuater_p,
        QuadModelv_P.DirectionCosineMatrixtoQuate_i5,
        &QuadModelv_P.IfWarningError_f);

      // End of Outputs for SubSystem: '<S241>/If Warning//Error'
    }

    // End of If: '<S241>/If1'

    // If: '<S329>/If1' incorporates:
    //   Constant: '<S329>/Constant'

    rtAction = -1;
    if (rtsiIsModeUpdateTimeStep(&(&QuadModelv_M)->solverInfo)) {
      if (QuadModelv_P.DirectionCosineMatrixtoQuater_c != 1.0) {
        rtAction = 0;
      }

      QuadModelv_DW.If1_ActiveSubsystem_d = rtAction;
    } else {
      rtAction = QuadModelv_DW.If1_ActiveSubsystem_d;
    }

    if (rtAction == 0) {
      // Outputs for IfAction SubSystem: '<S329>/If Warning//Error' incorporates:
      //   ActionPort: '<S353>/if'

      QuadModelv_IfWarningError(VectorConcatenate,
        QuadModelv_P.DirectionCosineMatrixtoQuater_c,
        QuadModelv_P.DirectionCosineMatrixtoQuater_b,
        &QuadModelv_P.IfWarningError_a);

      // End of Outputs for SubSystem: '<S329>/If Warning//Error'
    }

    // End of If: '<S329>/If1'
  }

  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[5] == 0) {
    // Gain: '<S322>/BiasGain1' incorporates:
    //   UniformRandomNumber: '<S322>/Uniform Random Number4'

    QuadModelv_B.BiasGain1[0] = QuadModelv_P.BiasGain1_Gain *
      QuadModelv_DW.UniformRandomNumber4_NextOutp_p[0];
    QuadModelv_B.BiasGain1[1] = QuadModelv_P.BiasGain1_Gain *
      QuadModelv_DW.UniformRandomNumber4_NextOutp_p[1];
    QuadModelv_B.BiasGain1[2] = QuadModelv_P.BiasGain1_Gain *
      QuadModelv_DW.UniformRandomNumber4_NextOutp_p[2];
  }

  // Gain: '<S421>/2*zeta * wn'
  rtb_q3dot = 2.0 * QuadModelv_P.FaultParamAPI.FaultInParams[9] *
    QuadModelv_P.ThreeaxisInertialMeasurement_c1;

  // Gain: '<S421>/wn^2'
  rtb_Clock1_tmp = QuadModelv_P.ThreeaxisInertialMeasurement_c1 *
    QuadModelv_P.ThreeaxisInertialMeasurement_c1;

  // Sum: '<S421>/Sum2' incorporates:
  //   Gain: '<S421>/2*zeta * wn'
  //   Gain: '<S421>/wn^2'
  //   SecondOrderIntegrator: '<S421>/Integrator, Second-Order Limited'
  //   Sum: '<S421>/Sum3'

  QuadModelv_B.Sum2[0] = (rtb_wind[0] -
    QuadModelv_X.IntegratorSecondOrderLimited_CS[0]) * rtb_Clock1_tmp -
    rtb_q3dot * QuadModelv_X.IntegratorSecondOrderLimited_CS[3];
  QuadModelv_B.Sum2[1] = (rtb_wind[1] -
    QuadModelv_X.IntegratorSecondOrderLimited_CS[1]) * rtb_Clock1_tmp -
    rtb_q3dot * QuadModelv_X.IntegratorSecondOrderLimited_CS[4];
  QuadModelv_B.Sum2[2] = (rtb_wind[2] -
    QuadModelv_X.IntegratorSecondOrderLimited_CS[2]) * rtb_Clock1_tmp -
    rtb_q3dot * QuadModelv_X.IntegratorSecondOrderLimited_CS[5];

  // Gain: '<S432>/2*zeta * wn'
  rtb_q3dot = 2.0 * QuadModelv_P.FaultParamAPI.FaultInParams[13] *
    QuadModelv_P.ThreeaxisInertialMeasurementU_i;

  // Gain: '<S432>/wn^2'
  rtb_Clock1_tmp = QuadModelv_P.ThreeaxisInertialMeasurementU_i *
    QuadModelv_P.ThreeaxisInertialMeasurementU_i;

  // Sum: '<S432>/Sum2' incorporates:
  //   Gain: '<S432>/2*zeta * wn'
  //   Gain: '<S432>/wn^2'
  //   SecondOrderIntegrator: '<S432>/Integrator, Second-Order Limited'
  //   Sum: '<S432>/Sum3'

  QuadModelv_B.Sum2_f[0] = (rtb_Mb[0] -
    QuadModelv_X.IntegratorSecondOrderLimited__i[0]) * rtb_Clock1_tmp -
    rtb_q3dot * QuadModelv_X.IntegratorSecondOrderLimited__i[3];
  QuadModelv_B.Sum2_f[1] = (rtb_Mb[1] -
    QuadModelv_X.IntegratorSecondOrderLimited__i[1]) * rtb_Clock1_tmp -
    rtb_q3dot * QuadModelv_X.IntegratorSecondOrderLimited__i[4];
  QuadModelv_B.Sum2_f[2] = (rtb_Mb[2] -
    QuadModelv_X.IntegratorSecondOrderLimited__i[2]) * rtb_Clock1_tmp -
    rtb_q3dot * QuadModelv_X.IntegratorSecondOrderLimited__i[5];
  if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
      (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
    // Assertion: '<S433>/Assertion' incorporates:
    //   Constant: '<S433>/max_val'
    //   Constant: '<S433>/min_val'
    //   Logic: '<S433>/conjunction'
    //   RelationalOperator: '<S433>/max_relop'
    //   RelationalOperator: '<S433>/min_relop'

    utAssert((QuadModelv_P.CheckAltitude_min <= rtb_Saturation_1) &&
             (rtb_Saturation_1 <= QuadModelv_P.CheckAltitude_max));

    // Assertion: '<S434>/Assertion' incorporates:
    //   Constant: '<S434>/max_val'
    //   Constant: '<S434>/min_val'
    //   Logic: '<S434>/conjunction'
    //   RelationalOperator: '<S434>/max_relop'
    //   RelationalOperator: '<S434>/min_relop'

    utAssert((QuadModelv_P.CheckLatitude_min <= LoadLeakRatio) && (LoadLeakRatio
              <= QuadModelv_P.CheckLatitude_max));

    // Assertion: '<S435>/Assertion' incorporates:
    //   Constant: '<S435>/max_val'
    //   Constant: '<S435>/min_val'
    //   Logic: '<S435>/conjunction'
    //   RelationalOperator: '<S435>/max_relop'
    //   RelationalOperator: '<S435>/min_relop'

    utAssert((QuadModelv_P.CheckLongitude_min <= Z) && (Z <=
              QuadModelv_P.CheckLongitude_max));

    // Assertion: '<S437>/Assertion' incorporates:
    //   Constant: '<S437>/max_val'
    //   Constant: '<S437>/min_val'
    //   Logic: '<S437>/conjunction'
    //   RelationalOperator: '<S437>/max_relop'
    //   RelationalOperator: '<S437>/min_relop'

    utAssert((QuadModelv_P.Istimewithinmodellimits_min <= rtb_Sum_o1) &&
             (rtb_Sum_o1 <= QuadModelv_P.Istimewithinmodellimits_max));

    // MATLAB Function 'OutputPort1/HILSensorMavModel1/Subsystem/AccelNoiseGainFunction': '<S395>:1' 
    // '<S395>:1:3' if theta>0.1
    // MATLAB Function 'OutputPort1/HILSensorMavModel1/Subsystem/GyroNoiseGainFunction': '<S396>:1' 
    // '<S396>:1:3' if theta>0.1
    // MATLAB Function 'OutputPort1/HILSensorMavModel1/Subsystem/MagNoiseGainFunction': '<S397>:1' 
    // '<S397>:1:3' if theta>0.1
  }

  // Outport: '<Root>/PosGPS' incorporates:
  //   SignalConversion generated from: '<Root>/MavVehile3DInfo'

  QuadModelv_Y.PosGPS[0] = LoadLeakRatio;
  QuadModelv_Y.PosGPS[1] = Z;
  QuadModelv_Y.PosGPS[2] = rtb_Saturation_1;
  if (rtmIsMajorTimeStep((&QuadModelv_M))) {
    // Update for Integrator: '<S15>/q0 q1 q2 q3'
    QuadModelv_DW.q0q1q2q3_IWORK = 0;
    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
      // Update for Memory: '<Root>/Memory' incorporates:
      //   Inport: '<Root>/inPWMs'

      std::memcpy(&QuadModelv_DW.Memory_PreviousInput[0], &QuadModelv_U.inPWMs[0],
                  sizeof(real_T) << 3U);
    }

    // Update for Atomic SubSystem: '<S294>/MotorNonlinearDynamic1'
    Q_MotorNonlinearDynamic1_Update(&QuadModelv_DW.MotorNonlinearDynamic1);

    // End of Update for SubSystem: '<S294>/MotorNonlinearDynamic1'

    // Update for Atomic SubSystem: '<S294>/MotorNonlinearDynamic2'
    Q_MotorNonlinearDynamic1_Update(&QuadModelv_DW.MotorNonlinearDynamic2);

    // End of Update for SubSystem: '<S294>/MotorNonlinearDynamic2'

    // Update for Atomic SubSystem: '<S294>/MotorNonlinearDynamic3'
    Q_MotorNonlinearDynamic1_Update(&QuadModelv_DW.MotorNonlinearDynamic3);

    // End of Update for SubSystem: '<S294>/MotorNonlinearDynamic3'

    // Update for Atomic SubSystem: '<S294>/MotorNonlinearDynamic4'
    Q_MotorNonlinearDynamic1_Update(&QuadModelv_DW.MotorNonlinearDynamic4);

    // End of Update for SubSystem: '<S294>/MotorNonlinearDynamic4'

    // Update for Atomic SubSystem: '<S294>/MotorNonlinearDynamic5'
    Q_MotorNonlinearDynamic1_Update(&QuadModelv_DW.MotorNonlinearDynamic5);

    // End of Update for SubSystem: '<S294>/MotorNonlinearDynamic5'

    // Update for Atomic SubSystem: '<S294>/MotorNonlinearDynamic6'
    Q_MotorNonlinearDynamic1_Update(&QuadModelv_DW.MotorNonlinearDynamic6);

    // End of Update for SubSystem: '<S294>/MotorNonlinearDynamic6'

    // Update for Atomic SubSystem: '<S294>/MotorNonlinearDynamic7'
    Q_MotorNonlinearDynamic1_Update(&QuadModelv_DW.MotorNonlinearDynamic7);

    // End of Update for SubSystem: '<S294>/MotorNonlinearDynamic7'

    // Update for Atomic SubSystem: '<S294>/MotorNonlinearDynamic8'
    Q_MotorNonlinearDynamic1_Update(&QuadModelv_DW.MotorNonlinearDynamic8);

    // End of Update for SubSystem: '<S294>/MotorNonlinearDynamic8'
    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[4] == 0) {
      // Update for RandomNumber: '<S511>/White Noise'
      QuadModelv_DW.NextOutput = rt_nrand_Upu32_Yd_f_pw_snf
        (&QuadModelv_DW.RandSeed) * QuadModelv_P.WhiteNoise_StdDev +
        QuadModelv_P.WhiteNoise_Mean;
    }

    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[2] == 0) {
      // Update for UniformRandomNumber: '<S67>/Uniform Random Number'
      QuadModelv_DW.UniformRandomNumber_NextOutput[0] =
        (QuadModelv_P.UniformRandomNumber_Maximum[0] -
         QuadModelv_P.UniformRandomNumber_Minimum[0]) *
        rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_o[0]) +
        QuadModelv_P.UniformRandomNumber_Minimum[0];
      QuadModelv_DW.UniformRandomNumber_NextOutput[1] =
        (QuadModelv_P.UniformRandomNumber_Maximum[1] -
         QuadModelv_P.UniformRandomNumber_Minimum[1]) *
        rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_o[1]) +
        QuadModelv_P.UniformRandomNumber_Minimum[1];
      QuadModelv_DW.UniformRandomNumber_NextOutput[2] =
        (QuadModelv_P.UniformRandomNumber_Maximum[2] -
         QuadModelv_P.UniformRandomNumber_Minimum[2]) *
        rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_o[2]) +
        QuadModelv_P.UniformRandomNumber_Minimum[2];
    }

    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0) {
      // Update for RandomNumber: '<S157>/White Noise'
      QuadModelv_DW.NextOutput_c[0] = rt_nrand_Upu32_Yd_f_pw_snf
        (&QuadModelv_DW.RandSeed_p[0]) * QuadModelv_P.WhiteNoise_StdDev_j +
        QuadModelv_P.WhiteNoise_Mean_j;
      QuadModelv_DW.NextOutput_c[1] = rt_nrand_Upu32_Yd_f_pw_snf
        (&QuadModelv_DW.RandSeed_p[1]) * QuadModelv_P.WhiteNoise_StdDev_j +
        QuadModelv_P.WhiteNoise_Mean_j;
      QuadModelv_DW.NextOutput_c[2] = rt_nrand_Upu32_Yd_f_pw_snf
        (&QuadModelv_DW.RandSeed_p[2]) * QuadModelv_P.WhiteNoise_StdDev_j +
        QuadModelv_P.WhiteNoise_Mean_j;
      QuadModelv_DW.NextOutput_c[3] = rt_nrand_Upu32_Yd_f_pw_snf
        (&QuadModelv_DW.RandSeed_p[3]) * QuadModelv_P.WhiteNoise_StdDev_j +
        QuadModelv_P.WhiteNoise_Mean_j;
    }

    // Update for Enabled SubSystem: '<S148>/Hugw(z)'
    QuadModelv_Hugwz_Update(&QuadModelv_B.Hugwz_g, &QuadModelv_DW.Hugwz_g);

    // End of Update for SubSystem: '<S148>/Hugw(z)'

    // Update for Enabled SubSystem: '<S148>/Hvgw(z)'
    QuadModelv_Hvgwz_Update(&QuadModelv_B.Hvgwz_f, &QuadModelv_DW.Hvgwz_f);

    // End of Update for SubSystem: '<S148>/Hvgw(z)'

    // Update for Enabled SubSystem: '<S148>/Hwgw(z)'
    QuadModelv_Hwgwz_Update(&QuadModelv_B.Hwgwz_a, &QuadModelv_DW.Hwgwz_a);

    // End of Update for SubSystem: '<S148>/Hwgw(z)'
    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0) {
      // Update for RandomNumber: '<S118>/White Noise'
      QuadModelv_DW.NextOutput_p[0] = rt_nrand_Upu32_Yd_f_pw_snf
        (&QuadModelv_DW.RandSeed_or[0]) * QuadModelv_P.WhiteNoise_StdDev_k +
        QuadModelv_P.WhiteNoise_Mean_m;
      QuadModelv_DW.NextOutput_p[1] = rt_nrand_Upu32_Yd_f_pw_snf
        (&QuadModelv_DW.RandSeed_or[1]) * QuadModelv_P.WhiteNoise_StdDev_k +
        QuadModelv_P.WhiteNoise_Mean_m;
      QuadModelv_DW.NextOutput_p[2] = rt_nrand_Upu32_Yd_f_pw_snf
        (&QuadModelv_DW.RandSeed_or[2]) * QuadModelv_P.WhiteNoise_StdDev_k +
        QuadModelv_P.WhiteNoise_Mean_m;
      QuadModelv_DW.NextOutput_p[3] = rt_nrand_Upu32_Yd_f_pw_snf
        (&QuadModelv_DW.RandSeed_or[3]) * QuadModelv_P.WhiteNoise_StdDev_k +
        QuadModelv_P.WhiteNoise_Mean_m;
    }

    // Update for Enabled SubSystem: '<S109>/Hugw(z)'
    QuadModelv_Hugwz_Update(&QuadModelv_B.Hugwz, &QuadModelv_DW.Hugwz);

    // End of Update for SubSystem: '<S109>/Hugw(z)'

    // Update for Enabled SubSystem: '<S109>/Hvgw(z)'
    QuadModelv_Hvgwz_Update(&QuadModelv_B.Hvgwz, &QuadModelv_DW.Hvgwz);

    // End of Update for SubSystem: '<S109>/Hvgw(z)'

    // Update for Enabled SubSystem: '<S109>/Hwgw(z)'
    QuadModelv_Hwgwz_Update(&QuadModelv_B.Hwgwz, &QuadModelv_DW.Hwgwz);

    // End of Update for SubSystem: '<S109>/Hwgw(z)'
    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[1] == 0) {
      // Update for Memory: '<S456>/otime' incorporates:
      //   Constant: '<S411>/Decimal Year'

      QuadModelv_DW.otime_PreviousInput = QuadModelv_P.DecimalYear_Value;

      // Update for Memory: '<S455>/olon'
      QuadModelv_DW.olon_PreviousInput = QuadModelv_B.Switch_p;

      // Update for Memory: '<S454>/olat'
      QuadModelv_DW.olat_PreviousInput = QuadModelv_B.Switch_l;

      // Update for Memory: '<S454>/oalt'
      QuadModelv_DW.oalt_PreviousInput = QuadModelv_B.Gain_f0;
    }

    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[3] == 0) {
      // Update for UniformRandomNumber: '<S398>/Uniform Random Number'
      QuadModelv_DW.UniformRandomNumber_NextOutpu_l =
        (QuadModelv_P.UniformRandomNumber_Maximum_m -
         QuadModelv_P.UniformRandomNumber_Minimum_d) *
        rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_e) +
        QuadModelv_P.UniformRandomNumber_Minimum_d;

      // Update for UniformRandomNumber: '<S398>/Uniform Random Number4'
      QuadModelv_DW.UniformRandomNumber4_NextOutput =
        (QuadModelv_P.UniformRandomNumber4_Maximum -
         QuadModelv_P.UniformRandomNumber4_Minimum) * rt_urand_Upu32_Yd_f_pw_snf
        (&QuadModelv_DW.RandSeed_b) + QuadModelv_P.UniformRandomNumber4_Minimum;
    }

    // Update for Enabled SubSystem: '<S108>/Hpgw'
    QuadModelv_Hpgw_Update(&QuadModelv_B.Hpgw, &QuadModelv_DW.Hpgw);

    // End of Update for SubSystem: '<S108>/Hpgw'

    // Update for Enabled SubSystem: '<S108>/Hqgw'
    QuadModelv_Hqgw_Update(QuadModelv_B.Hwgwz.Sum, &QuadModelv_B.Hqgw,
      &QuadModelv_DW.Hqgw);

    // End of Update for SubSystem: '<S108>/Hqgw'

    // Update for Enabled SubSystem: '<S108>/Hrgw'
    QuadModelv_Hrgw_Update(QuadModelv_B.Hvgwz.Sum, &QuadModelv_B.Hrgw,
      &QuadModelv_DW.Hrgw);

    // End of Update for SubSystem: '<S108>/Hrgw'

    // Update for Enabled SubSystem: '<S147>/Hpgw'
    QuadModelv_Hpgw_Update(&QuadModelv_B.Hpgw_h, &QuadModelv_DW.Hpgw_h);

    // End of Update for SubSystem: '<S147>/Hpgw'

    // Update for Enabled SubSystem: '<S147>/Hqgw'
    QuadModelv_Hqgw_Update(QuadModelv_B.Hwgwz_a.Sum, &QuadModelv_B.Hqgw_a,
      &QuadModelv_DW.Hqgw_a);

    // End of Update for SubSystem: '<S147>/Hqgw'

    // Update for Enabled SubSystem: '<S147>/Hrgw'
    QuadModelv_Hrgw_Update(QuadModelv_B.Hvgwz_f.Sum, &QuadModelv_B.Hrgw_a,
      &QuadModelv_DW.Hrgw_a);

    // End of Update for SubSystem: '<S147>/Hrgw'
    if (rtmIsMajorTimeStep((&QuadModelv_M)) &&
        (&QuadModelv_M)->Timing.TaskCounters.TID[5] == 0) {
      // Update for UniformRandomNumber: '<S322>/Uniform Random Number4'
      QuadModelv_DW.UniformRandomNumber4_NextOutp_p[0] =
        (QuadModelv_P.UniformRandomNumber4_Maximum_c[0] -
         QuadModelv_P.UniformRandomNumber4_Minimum_o[0]) *
        rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_n[0]) +
        QuadModelv_P.UniformRandomNumber4_Minimum_o[0];
      QuadModelv_DW.UniformRandomNumber4_NextOutp_p[1] =
        (QuadModelv_P.UniformRandomNumber4_Maximum_c[1] -
         QuadModelv_P.UniformRandomNumber4_Minimum_o[1]) *
        rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_n[1]) +
        QuadModelv_P.UniformRandomNumber4_Minimum_o[1];
      QuadModelv_DW.UniformRandomNumber4_NextOutp_p[2] =
        (QuadModelv_P.UniformRandomNumber4_Maximum_c[2] -
         QuadModelv_P.UniformRandomNumber4_Minimum_o[2]) *
        rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_n[2]) +
        QuadModelv_P.UniformRandomNumber4_Minimum_o[2];
    }
  }                                    // end MajorTimeStep

  if (rtmIsMajorTimeStep((&QuadModelv_M))) {
    rt_ertODEUpdateContinuousStates(&(&QuadModelv_M)->solverInfo);

    // Update absolute time for base rate
    // The "clockTick0" counts the number of times the code of this task has
    //  been executed. The absolute time is the multiplication of "clockTick0"
    //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
    //  overflow during the application lifespan selected.

    ++(&QuadModelv_M)->Timing.clockTick0;
    (&QuadModelv_M)->Timing.t[0] = rtsiGetSolverStopTime(&(&QuadModelv_M)
      ->solverInfo);

    {
      // Update absolute timer for sample time: [0.001s, 0.0s]
      // The "clockTick1" counts the number of times the code of this task has
      //  been executed. The resolution of this integer timer is 0.001, which is the step size
      //  of the task. Size of "clockTick1" ensures timer will not overflow during the
      //  application lifespan selected.

      (&QuadModelv_M)->Timing.clockTick1++;
    }

    rate_scheduler((&QuadModelv_M));
  }                                    // end MajorTimeStep
}

// Derivatives for root system: '<Root>'
void MulticopterModelClass::QuadModelv_derivatives()
{
  XDot_QuadModelv_T *_rtXdot;
  boolean_T lsat;
  boolean_T usat;
  _rtXdot = ((XDot_QuadModelv_T *) (&QuadModelv_M)->derivs);

  // Derivatives for SecondOrderIntegrator: '<S421>/Integrator, Second-Order Limited' 
  if (QuadModelv_DW.IntegratorSecondOrderLimited_MO[0] == 0) {
    _rtXdot->IntegratorSecondOrderLimited_CS[0] =
      QuadModelv_X.IntegratorSecondOrderLimited_CS[3];
    _rtXdot->IntegratorSecondOrderLimited_CS[3] = QuadModelv_B.Sum2[0];
  }

  if (QuadModelv_DW.IntegratorSecondOrderLimited_MO[1] == 0) {
    _rtXdot->IntegratorSecondOrderLimited_CS[1] =
      QuadModelv_X.IntegratorSecondOrderLimited_CS[4];
    _rtXdot->IntegratorSecondOrderLimited_CS[4] = QuadModelv_B.Sum2[1];
  }

  if (QuadModelv_DW.IntegratorSecondOrderLimited_MO[2] == 0) {
    _rtXdot->IntegratorSecondOrderLimited_CS[2] =
      QuadModelv_X.IntegratorSecondOrderLimited_CS[5];
    _rtXdot->IntegratorSecondOrderLimited_CS[5] = QuadModelv_B.Sum2[2];
  }

  // End of Derivatives for SecondOrderIntegrator: '<S421>/Integrator, Second-Order Limited' 

  // Derivatives for Integrator: '<S15>/q0 q1 q2 q3'
  _rtXdot->q0q1q2q3_CSTATE[0] = QuadModelv_B.TmpSignalConversionAtq0q1q2q3_c[0];
  _rtXdot->q0q1q2q3_CSTATE[1] = QuadModelv_B.TmpSignalConversionAtq0q1q2q3_c[1];
  _rtXdot->q0q1q2q3_CSTATE[2] = QuadModelv_B.TmpSignalConversionAtq0q1q2q3_c[2];
  _rtXdot->q0q1q2q3_CSTATE[3] = QuadModelv_B.TmpSignalConversionAtq0q1q2q3_c[3];

  // Derivatives for Integrator: '<S14>/xe,ye,ze'
  _rtXdot->xeyeze_CSTATE[0] = QuadModelv_B.Product[0];

  // Derivatives for Integrator: '<S14>/p,q,r '
  _rtXdot->pqr_CSTATE[0] = QuadModelv_B.Product2[0];

  // Derivatives for Integrator: '<S14>/ub,vb,wb'
  _rtXdot->ubvbwb_CSTATE[0] = QuadModelv_B.Sum_p[0];

  // Derivatives for Integrator: '<S14>/xe,ye,ze'
  _rtXdot->xeyeze_CSTATE[1] = QuadModelv_B.Product[1];

  // Derivatives for Integrator: '<S14>/p,q,r '
  _rtXdot->pqr_CSTATE[1] = QuadModelv_B.Product2[1];

  // Derivatives for Integrator: '<S14>/ub,vb,wb'
  _rtXdot->ubvbwb_CSTATE[1] = QuadModelv_B.Sum_p[1];

  // Derivatives for Integrator: '<S14>/xe,ye,ze'
  _rtXdot->xeyeze_CSTATE[2] = QuadModelv_B.Product[2];

  // Derivatives for Integrator: '<S14>/p,q,r '
  _rtXdot->pqr_CSTATE[2] = QuadModelv_B.Product2[2];

  // Derivatives for Integrator: '<S14>/ub,vb,wb'
  _rtXdot->ubvbwb_CSTATE[2] = QuadModelv_B.Sum_p[2];

  // Derivatives for Atomic SubSystem: '<S294>/MotorNonlinearDynamic1'
  Qu_MotorNonlinearDynamic1_Deriv(&QuadModelv_B.MotorNonlinearDynamic1,
    &_rtXdot->MotorNonlinearDynamic1);

  // End of Derivatives for SubSystem: '<S294>/MotorNonlinearDynamic1'

  // Derivatives for Atomic SubSystem: '<S294>/MotorNonlinearDynamic2'
  Qu_MotorNonlinearDynamic1_Deriv(&QuadModelv_B.MotorNonlinearDynamic2,
    &_rtXdot->MotorNonlinearDynamic2);

  // End of Derivatives for SubSystem: '<S294>/MotorNonlinearDynamic2'

  // Derivatives for Atomic SubSystem: '<S294>/MotorNonlinearDynamic3'
  Qu_MotorNonlinearDynamic1_Deriv(&QuadModelv_B.MotorNonlinearDynamic3,
    &_rtXdot->MotorNonlinearDynamic3);

  // End of Derivatives for SubSystem: '<S294>/MotorNonlinearDynamic3'

  // Derivatives for Atomic SubSystem: '<S294>/MotorNonlinearDynamic4'
  Qu_MotorNonlinearDynamic1_Deriv(&QuadModelv_B.MotorNonlinearDynamic4,
    &_rtXdot->MotorNonlinearDynamic4);

  // End of Derivatives for SubSystem: '<S294>/MotorNonlinearDynamic4'

  // Derivatives for Atomic SubSystem: '<S294>/MotorNonlinearDynamic5'
  Qu_MotorNonlinearDynamic1_Deriv(&QuadModelv_B.MotorNonlinearDynamic5,
    &_rtXdot->MotorNonlinearDynamic5);

  // End of Derivatives for SubSystem: '<S294>/MotorNonlinearDynamic5'

  // Derivatives for Atomic SubSystem: '<S294>/MotorNonlinearDynamic6'
  Qu_MotorNonlinearDynamic1_Deriv(&QuadModelv_B.MotorNonlinearDynamic6,
    &_rtXdot->MotorNonlinearDynamic6);

  // End of Derivatives for SubSystem: '<S294>/MotorNonlinearDynamic6'

  // Derivatives for Atomic SubSystem: '<S294>/MotorNonlinearDynamic7'
  Qu_MotorNonlinearDynamic1_Deriv(&QuadModelv_B.MotorNonlinearDynamic7,
    &_rtXdot->MotorNonlinearDynamic7);

  // End of Derivatives for SubSystem: '<S294>/MotorNonlinearDynamic7'

  // Derivatives for Atomic SubSystem: '<S294>/MotorNonlinearDynamic8'
  Qu_MotorNonlinearDynamic1_Deriv(&QuadModelv_B.MotorNonlinearDynamic8,
    &_rtXdot->MotorNonlinearDynamic8);

  // End of Derivatives for SubSystem: '<S294>/MotorNonlinearDynamic8'

  // Derivatives for Enabled SubSystem: '<S512>/Distance into gust (x)'
  if (QuadModelv_DW.Distanceintogustx_MODE) {
    // Derivatives for Integrator: '<S515>/Distance into Gust (x) (Limited to gust length d)' incorporates:
    //   Constant: '<S510>/Constant'

    lsat = (QuadModelv_X.DistanceintoGustxLimitedtogus_e <=
            QuadModelv_P.DistanceintoGustxLimitedtogus_j);

    // Derivatives for Enabled SubSystem: '<S512>/Distance into gust (x)'
    usat = (QuadModelv_X.DistanceintoGustxLimitedtogus_e >=
            QuadModelv_P.Distanceintogustx_d_m);

    // End of Derivatives for SubSystem: '<S512>/Distance into gust (x)'
    if (((!lsat) && (!usat)) || (lsat &&
         (QuadModelv_P.FaultParamAPI.FaultInParams[1] > 0.0)) || (usat &&
         (QuadModelv_P.FaultParamAPI.FaultInParams[1] < 0.0))) {
      _rtXdot->DistanceintoGustxLimitedtogus_e =
        QuadModelv_P.FaultParamAPI.FaultInParams[1];
    } else {
      // in saturation
      _rtXdot->DistanceintoGustxLimitedtogus_e = 0.0;
    }

    // End of Derivatives for Integrator: '<S515>/Distance into Gust (x) (Limited to gust length d)' 
  } else {
    ((XDot_QuadModelv_T *) (&QuadModelv_M)->derivs)
      ->DistanceintoGustxLimitedtogus_e = 0.0;
  }

  // End of Derivatives for SubSystem: '<S512>/Distance into gust (x)'

  // Derivatives for Enabled SubSystem: '<S512>/Distance into gust (y)'
  // Derivatives for Enabled SubSystem: '<S512>/Distance into gust (y)'
  // Constant: '<S510>/Constant'
  QuadMod_Distanceintogusty_Deriv(QuadModelv_P.FaultParamAPI.FaultInParams[1],
    QuadModelv_P.Distanceintogusty_d_m, &QuadModelv_DW.Distanceintogusty,
    &QuadModelv_P.Distanceintogusty, &QuadModelv_X.Distanceintogusty,
    &_rtXdot->Distanceintogusty);

  // End of Derivatives for SubSystem: '<S512>/Distance into gust (y)'
  // End of Derivatives for SubSystem: '<S512>/Distance into gust (y)'

  // Derivatives for Enabled SubSystem: '<S512>/Distance into gust (z)'
  // Derivatives for Enabled SubSystem: '<S512>/Distance into gust (z)'
  QuadMod_Distanceintogusty_Deriv(QuadModelv_P.FaultParamAPI.FaultInParams[1],
    QuadModelv_P.Distanceintogustz_d_m, &QuadModelv_DW.Distanceintogustz,
    &QuadModelv_P.Distanceintogustz, &QuadModelv_X.Distanceintogustz,
    &_rtXdot->Distanceintogustz);

  // End of Derivatives for SubSystem: '<S512>/Distance into gust (z)'
  // End of Derivatives for SubSystem: '<S512>/Distance into gust (z)'

  // Derivatives for SecondOrderIntegrator: '<S432>/Integrator, Second-Order Limited' 
  if (QuadModelv_DW.IntegratorSecondOrderLimited__n[0] == 0) {
    _rtXdot->IntegratorSecondOrderLimited__i[0] =
      QuadModelv_X.IntegratorSecondOrderLimited__i[3];
    _rtXdot->IntegratorSecondOrderLimited__i[3] = QuadModelv_B.Sum2_f[0];
  }

  if (QuadModelv_DW.IntegratorSecondOrderLimited__n[1] == 0) {
    _rtXdot->IntegratorSecondOrderLimited__i[1] =
      QuadModelv_X.IntegratorSecondOrderLimited__i[4];
    _rtXdot->IntegratorSecondOrderLimited__i[4] = QuadModelv_B.Sum2_f[1];
  }

  if (QuadModelv_DW.IntegratorSecondOrderLimited__n[2] == 0) {
    _rtXdot->IntegratorSecondOrderLimited__i[2] =
      QuadModelv_X.IntegratorSecondOrderLimited__i[5];
    _rtXdot->IntegratorSecondOrderLimited__i[5] = QuadModelv_B.Sum2_f[2];
  }

  // End of Derivatives for SecondOrderIntegrator: '<S432>/Integrator, Second-Order Limited' 

  // Derivatives for TransferFcn: '<S368>/Transfer Fcn4'
  _rtXdot->TransferFcn4_CSTATE = 0.0;
  _rtXdot->TransferFcn4_CSTATE += QuadModelv_P.TransferFcn4_A *
    QuadModelv_X.TransferFcn4_CSTATE;
  _rtXdot->TransferFcn4_CSTATE += QuadModelv_B.BiasGain1[0];

  // Derivatives for TransferFcn: '<S368>/Transfer Fcn1'
  _rtXdot->TransferFcn1_CSTATE = 0.0;
  _rtXdot->TransferFcn1_CSTATE += QuadModelv_P.TransferFcn1_A *
    QuadModelv_X.TransferFcn1_CSTATE;
  _rtXdot->TransferFcn1_CSTATE += QuadModelv_B.BiasGain1[1];

  // Derivatives for TransferFcn: '<S368>/Transfer Fcn2'
  _rtXdot->TransferFcn2_CSTATE = 0.0;
  _rtXdot->TransferFcn2_CSTATE += QuadModelv_P.TransferFcn2_A *
    QuadModelv_X.TransferFcn2_CSTATE;
  _rtXdot->TransferFcn2_CSTATE += QuadModelv_B.BiasGain1[2];
}

// Model initialize function
void MulticopterModelClass::initialize()
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  // non-finite (run-time) assignments
  QuadModelv_P.uftinf_UpperSat = rtInf;

  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&(&QuadModelv_M)->solverInfo, &(&QuadModelv_M)
                          ->Timing.simTimeStep);
    rtsiSetTPtr(&(&QuadModelv_M)->solverInfo, &rtmGetTPtr((&QuadModelv_M)));
    rtsiSetStepSizePtr(&(&QuadModelv_M)->solverInfo, &(&QuadModelv_M)
                       ->Timing.stepSize0);
    rtsiSetdXPtr(&(&QuadModelv_M)->solverInfo, &(&QuadModelv_M)->derivs);
    rtsiSetContStatesPtr(&(&QuadModelv_M)->solverInfo, (real_T **)
                         &(&QuadModelv_M)->contStates);
    rtsiSetNumContStatesPtr(&(&QuadModelv_M)->solverInfo, &(&QuadModelv_M)
      ->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&(&QuadModelv_M)->solverInfo,
      &(&QuadModelv_M)->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&(&QuadModelv_M)->solverInfo,
      &(&QuadModelv_M)->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&(&QuadModelv_M)->solverInfo,
      &(&QuadModelv_M)->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&(&QuadModelv_M)->solverInfo, (&rtmGetErrorStatus
      ((&QuadModelv_M))));
    rtsiSetRTModelPtr(&(&QuadModelv_M)->solverInfo, (&QuadModelv_M));
  }

  rtsiSetSimTimeStep(&(&QuadModelv_M)->solverInfo, MAJOR_TIME_STEP);
  (&QuadModelv_M)->intgData.y = (&QuadModelv_M)->odeY;
  (&QuadModelv_M)->intgData.f[0] = (&QuadModelv_M)->odeF[0];
  (&QuadModelv_M)->intgData.f[1] = (&QuadModelv_M)->odeF[1];
  (&QuadModelv_M)->intgData.f[2] = (&QuadModelv_M)->odeF[2];
  (&QuadModelv_M)->intgData.f[3] = (&QuadModelv_M)->odeF[3];
  (&QuadModelv_M)->contStates = ((X_QuadModelv_T *) &QuadModelv_X);
  rtsiSetSolverData(&(&QuadModelv_M)->solverInfo, static_cast<void *>
                    (&(&QuadModelv_M)->intgData));
  rtsiSetIsMinorTimeStepWithModeChange(&(&QuadModelv_M)->solverInfo, false);
  rtsiSetSolverName(&(&QuadModelv_M)->solverInfo,"ode4");
  rtmSetTPtr((&QuadModelv_M), &(&QuadModelv_M)->Timing.tArray[0]);
  (&QuadModelv_M)->Timing.stepSize0 = 0.001;
  rtmSetFirstInitCond((&QuadModelv_M), 1);

  {
    real_T tmp;
    int32_T i;
    int32_T t;
    uint32_T tseed;
    static const uint32_T tmp_0[625] = { 5489U, 1301868182U, 2938499221U,
      2950281878U, 1875628136U, 751856242U, 944701696U, 2243192071U, 694061057U,
      219885934U, 2066767472U, 3182869408U, 485472502U, 2336857883U, 1071588843U,
      3418470598U, 951210697U, 3693558366U, 2923482051U, 1793174584U,
      2982310801U, 1586906132U, 1951078751U, 1808158765U, 1733897588U,
      431328322U, 4202539044U, 530658942U, 1714810322U, 3025256284U, 3342585396U,
      1937033938U, 2640572511U, 1654299090U, 3692403553U, 4233871309U,
      3497650794U, 862629010U, 2943236032U, 2426458545U, 1603307207U,
      1133453895U, 3099196360U, 2208657629U, 2747653927U, 931059398U, 761573964U,
      3157853227U, 785880413U, 730313442U, 124945756U, 2937117055U, 3295982469U,
      1724353043U, 3021675344U, 3884886417U, 4010150098U, 4056961966U,
      699635835U, 2681338818U, 1339167484U, 720757518U, 2800161476U, 2376097373U,
      1532957371U, 3902664099U, 1238982754U, 3725394514U, 3449176889U,
      3570962471U, 4287636090U, 4087307012U, 3603343627U, 202242161U,
      2995682783U, 1620962684U, 3704723357U, 371613603U, 2814834333U,
      2111005706U, 624778151U, 2094172212U, 4284947003U, 1211977835U, 991917094U,
      1570449747U, 2962370480U, 1259410321U, 170182696U, 146300961U, 2836829791U,
      619452428U, 2723670296U, 1881399711U, 1161269684U, 1675188680U,
      4132175277U, 780088327U, 3409462821U, 1036518241U, 1834958505U,
      3048448173U, 161811569U, 618488316U, 44795092U, 3918322701U, 1924681712U,
      3239478144U, 383254043U, 4042306580U, 2146983041U, 3992780527U,
      3518029708U, 3545545436U, 3901231469U, 1896136409U, 2028528556U,
      2339662006U, 501326714U, 2060962201U, 2502746480U, 561575027U, 581893337U,
      3393774360U, 1778912547U, 3626131687U, 2175155826U, 319853231U, 986875531U,
      819755096U, 2915734330U, 2688355739U, 3482074849U, 2736559U, 2296975761U,
      1029741190U, 2876812646U, 690154749U, 579200347U, 4027461746U, 1285330465U,
      2701024045U, 4117700889U, 759495121U, 3332270341U, 2313004527U,
      2277067795U, 4131855432U, 2722057515U, 1264804546U, 3848622725U,
      2211267957U, 4100593547U, 959123777U, 2130745407U, 3194437393U, 486673947U,
      1377371204U, 17472727U, 352317554U, 3955548058U, 159652094U, 1232063192U,
      3835177280U, 49423123U, 3083993636U, 733092U, 2120519771U, 2573409834U,
      1112952433U, 3239502554U, 761045320U, 1087580692U, 2540165110U, 641058802U,
      1792435497U, 2261799288U, 1579184083U, 627146892U, 2165744623U,
      2200142389U, 2167590760U, 2381418376U, 1793358889U, 3081659520U,
      1663384067U, 2009658756U, 2689600308U, 739136266U, 2304581039U,
      3529067263U, 591360555U, 525209271U, 3131882996U, 294230224U, 2076220115U,
      3113580446U, 1245621585U, 1386885462U, 3203270426U, 123512128U, 12350217U,
      354956375U, 4282398238U, 3356876605U, 3888857667U, 157639694U, 2616064085U,
      1563068963U, 2762125883U, 4045394511U, 4180452559U, 3294769488U,
      1684529556U, 1002945951U, 3181438866U, 22506664U, 691783457U, 2685221343U,
      171579916U, 3878728600U, 2475806724U, 2030324028U, 3331164912U,
      1708711359U, 1970023127U, 2859691344U, 2588476477U, 2748146879U,
      136111222U, 2967685492U, 909517429U, 2835297809U, 3206906216U, 3186870716U,
      341264097U, 2542035121U, 3353277068U, 548223577U, 3170936588U, 1678403446U,
      297435620U, 2337555430U, 466603495U, 1132321815U, 1208589219U, 696392160U,
      894244439U, 2562678859U, 470224582U, 3306867480U, 201364898U, 2075966438U,
      1767227936U, 2929737987U, 3674877796U, 2654196643U, 3692734598U,
      3528895099U, 2796780123U, 3048728353U, 842329300U, 191554730U, 2922459673U,
      3489020079U, 3979110629U, 1022523848U, 2202932467U, 3583655201U,
      3565113719U, 587085778U, 4176046313U, 3013713762U, 950944241U, 396426791U,
      3784844662U, 3477431613U, 3594592395U, 2782043838U, 3392093507U,
      3106564952U, 2829419931U, 1358665591U, 2206918825U, 3170783123U, 31522386U,
      2988194168U, 1782249537U, 1105080928U, 843500134U, 1225290080U,
      1521001832U, 3605886097U, 2802786495U, 2728923319U, 3996284304U,
      903417639U, 1171249804U, 1020374987U, 2824535874U, 423621996U, 1988534473U,
      2493544470U, 1008604435U, 1756003503U, 1488867287U, 1386808992U,
      732088248U, 1780630732U, 2482101014U, 976561178U, 1543448953U, 2602866064U,
      2021139923U, 1952599828U, 2360242564U, 2117959962U, 2753061860U,
      2388623612U, 4138193781U, 2962920654U, 2284970429U, 766920861U,
      3457264692U, 2879611383U, 815055854U, 2332929068U, 1254853997U,
      3740375268U, 3799380844U, 4091048725U, 2006331129U, 1982546212U,
      686850534U, 1907447564U, 2682801776U, 2780821066U, 998290361U, 1342433871U,
      4195430425U, 607905174U, 3902331779U, 2454067926U, 1708133115U,
      1170874362U, 2008609376U, 3260320415U, 2211196135U, 433538229U,
      2728786374U, 2189520818U, 262554063U, 1182318347U, 3710237267U,
      1221022450U, 715966018U, 2417068910U, 2591870721U, 2870691989U,
      3418190842U, 4238214053U, 1540704231U, 1575580968U, 2095917976U,
      4078310857U, 2313532447U, 2110690783U, 4056346629U, 4061784526U,
      1123218514U, 551538993U, 597148360U, 4120175196U, 3581618160U, 3181170517U,
      422862282U, 3227524138U, 1713114790U, 662317149U, 1230418732U, 928171837U,
      1324564878U, 1928816105U, 1786535431U, 2878099422U, 3290185549U,
      539474248U, 1657512683U, 552370646U, 1671741683U, 3655312128U, 1552739510U,
      2605208763U, 1441755014U, 181878989U, 3124053868U, 1447103986U,
      3183906156U, 1728556020U, 3502241336U, 3055466967U, 1013272474U,
      818402132U, 1715099063U, 2900113506U, 397254517U, 4194863039U, 1009068739U,
      232864647U, 2540223708U, 2608288560U, 2415367765U, 478404847U, 3455100648U,
      3182600021U, 2115988978U, 434269567U, 4117179324U, 3461774077U, 887256537U,
      3545801025U, 286388911U, 3451742129U, 1981164769U, 786667016U, 3310123729U,
      3097811076U, 2224235657U, 2959658883U, 3370969234U, 2514770915U,
      3345656436U, 2677010851U, 2206236470U, 271648054U, 2342188545U,
      4292848611U, 3646533909U, 3754009956U, 3803931226U, 4160647125U,
      1477814055U, 4043852216U, 1876372354U, 3133294443U, 3871104810U,
      3177020907U, 2074304428U, 3479393793U, 759562891U, 164128153U, 1839069216U,
      2114162633U, 3989947309U, 3611054956U, 1333547922U, 835429831U, 494987340U,
      171987910U, 1252001001U, 370809172U, 3508925425U, 2535703112U, 1276855041U,
      1922855120U, 835673414U, 3030664304U, 613287117U, 171219893U, 3423096126U,
      3376881639U, 2287770315U, 1658692645U, 1262815245U, 3957234326U,
      1168096164U, 2968737525U, 2655813712U, 2132313144U, 3976047964U,
      326516571U, 353088456U, 3679188938U, 3205649712U, 2654036126U, 1249024881U,
      880166166U, 691800469U, 2229503665U, 1673458056U, 4032208375U, 1851778863U,
      2563757330U, 376742205U, 1794655231U, 340247333U, 1505873033U, 396524441U,
      879666767U, 3335579166U, 3260764261U, 3335999539U, 506221798U, 4214658741U,
      975887814U, 2080536343U, 3360539560U, 571586418U, 138896374U, 4234352651U,
      2737620262U, 3928362291U, 1516365296U, 38056726U, 3599462320U, 3585007266U,
      3850961033U, 471667319U, 1536883193U, 2310166751U, 1861637689U,
      2530999841U, 4139843801U, 2710569485U, 827578615U, 2012334720U,
      2907369459U, 3029312804U, 2820112398U, 1965028045U, 35518606U, 2478379033U,
      643747771U, 1924139484U, 4123405127U, 3811735531U, 3429660832U,
      3285177704U, 1948416081U, 1311525291U, 1183517742U, 1739192232U,
      3979815115U, 2567840007U, 4116821529U, 213304419U, 4125718577U,
      1473064925U, 2442436592U, 1893310111U, 4195361916U, 3747569474U,
      828465101U, 2991227658U, 750582866U, 1205170309U, 1409813056U, 678418130U,
      1171531016U, 3821236156U, 354504587U, 4202874632U, 3882511497U,
      1893248677U, 1903078632U, 26340130U, 2069166240U, 3657122492U, 3725758099U,
      831344905U, 811453383U, 3447711422U, 2434543565U, 4166886888U, 3358210805U,
      4142984013U, 2988152326U, 3527824853U, 982082992U, 2809155763U, 190157081U,
      3340214818U, 2365432395U, 2548636180U, 2894533366U, 3474657421U,
      2372634704U, 2845748389U, 43024175U, 2774226648U, 1987702864U, 3186502468U,
      453610222U, 4204736567U, 1392892630U, 2471323686U, 2470534280U,
      3541393095U, 4269885866U, 3909911300U, 759132955U, 1482612480U, 667715263U,
      1795580598U, 2337923983U, 3390586366U, 581426223U, 1515718634U, 476374295U,
      705213300U, 363062054U, 2084697697U, 2407503428U, 2292957699U, 2426213835U,
      2199989172U, 1987356470U, 4026755612U, 2147252133U, 270400031U,
      1367820199U, 2369854699U, 2844269403U, 79981964U, 624U };

    // Start for If: '<S42>/If'
    QuadModelv_DW.If_ActiveSubsystem = -1;

    // Start for If: '<S194>/If'
    QuadModelv_DW.If_ActiveSubsystem_h = -1;

    // Start for If: '<S193>/If'
    QuadModelv_DW.If_ActiveSubsystem_l = -1;

    // Start for Constant: '<S294>/ModelInit_RPM'
    QuadModelv_B.ModelInit_RPM = QuadModelv_P.ModelInit_RPM;

    // Start for If: '<S153>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' 
    QuadModelv_DW.ifHeightMaxlowaltitudeelseifHei = -1;

    // Start for If: '<S114>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' 
    QuadModelv_DW.ifHeightMaxlowaltitudeelseifH_p = -1;

    // Start for If: '<S321>/If'
    QuadModelv_DW.If_ActiveSubsystem_m = -1;

    // Start for If: '<S113>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' 
    QuadModelv_DW.ifHeightMaxlowaltitudeelseifH_j = -1;

    // Start for If: '<S152>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' 
    QuadModelv_DW.ifHeightMaxlowaltitudeelseifH_e = -1;

    // Start for If: '<S203>/If1'
    QuadModelv_DW.If1_ActiveSubsystem = -1;

    // Start for If: '<S241>/If1'
    QuadModelv_DW.If1_ActiveSubsystem_o = -1;

    // Start for If: '<S329>/If1'
    QuadModelv_DW.If1_ActiveSubsystem_d = -1;

    // InitializeConditions for SecondOrderIntegrator: '<S421>/Integrator, Second-Order Limited' 
    QuadModelv_X.IntegratorSecondOrderLimited_CS[0] =
      QuadModelv_P.IntegratorSecondOrderLimited_IC;
    QuadModelv_X.IntegratorSecondOrderLimited_CS[1] =
      QuadModelv_P.IntegratorSecondOrderLimited_IC;
    QuadModelv_X.IntegratorSecondOrderLimited_CS[2] =
      QuadModelv_P.IntegratorSecondOrderLimited_IC;
    QuadModelv_X.IntegratorSecondOrderLimited_CS[3] =
      QuadModelv_P.IntegratorSecondOrderLimited__n;
    QuadModelv_X.IntegratorSecondOrderLimited_CS[4] =
      QuadModelv_P.IntegratorSecondOrderLimited__n;
    QuadModelv_X.IntegratorSecondOrderLimited_CS[5] =
      QuadModelv_P.IntegratorSecondOrderLimited__n;

    // InitializeConditions for Integrator: '<S15>/q0 q1 q2 q3'
    if (rtmIsFirstInitCond((&QuadModelv_M))) {
      QuadModelv_X.q0q1q2q3_CSTATE[0] = 0.0;
      QuadModelv_X.q0q1q2q3_CSTATE[1] = 0.0;
      QuadModelv_X.q0q1q2q3_CSTATE[2] = 0.0;
      QuadModelv_X.q0q1q2q3_CSTATE[3] = 0.0;
    }

    QuadModelv_DW.q0q1q2q3_IWORK = 1;

    // End of InitializeConditions for Integrator: '<S15>/q0 q1 q2 q3'

    // InitializeConditions for Integrator: '<S14>/xe,ye,ze'
    QuadModelv_X.xeyeze_CSTATE[0] = QuadModelv_P.ModelInit_PosE[0];

    // InitializeConditions for Integrator: '<S14>/p,q,r '
    QuadModelv_X.pqr_CSTATE[0] = QuadModelv_P.ModelInit_RateB[0];

    // InitializeConditions for Integrator: '<S14>/ub,vb,wb'
    QuadModelv_X.ubvbwb_CSTATE[0] = QuadModelv_P.ModelInit_VelB[0];

    // InitializeConditions for Integrator: '<S14>/xe,ye,ze'
    QuadModelv_X.xeyeze_CSTATE[1] = QuadModelv_P.ModelInit_PosE[1];

    // InitializeConditions for Integrator: '<S14>/p,q,r '
    QuadModelv_X.pqr_CSTATE[1] = QuadModelv_P.ModelInit_RateB[1];

    // InitializeConditions for Integrator: '<S14>/ub,vb,wb'
    QuadModelv_X.ubvbwb_CSTATE[1] = QuadModelv_P.ModelInit_VelB[1];

    // InitializeConditions for Integrator: '<S14>/xe,ye,ze'
    QuadModelv_X.xeyeze_CSTATE[2] = QuadModelv_P.ModelInit_PosE[2];

    // InitializeConditions for Integrator: '<S14>/p,q,r '
    QuadModelv_X.pqr_CSTATE[2] = QuadModelv_P.ModelInit_RateB[2];

    // InitializeConditions for Integrator: '<S14>/ub,vb,wb'
    QuadModelv_X.ubvbwb_CSTATE[2] = QuadModelv_P.ModelInit_VelB[2];

    // InitializeConditions for Memory: '<Root>/Memory'
    for (i = 0; i < 8; i++) {
      QuadModelv_DW.Memory_PreviousInput[i] =
        QuadModelv_P.Memory_InitialCondition;
    }

    // End of InitializeConditions for Memory: '<Root>/Memory'

    // InitializeConditions for RandomNumber: '<S511>/White Noise'
    tmp = std::floor(QuadModelv_P.BandLimitedWhiteNoise_seed);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) +
               static_cast<uint32_T>(t)) << 16U) + static_cast<uint32_T>(t)) +
      static_cast<uint32_T>(i);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed = tseed;
    QuadModelv_DW.NextOutput = rt_nrand_Upu32_Yd_f_pw_snf
      (&QuadModelv_DW.RandSeed) * QuadModelv_P.WhiteNoise_StdDev +
      QuadModelv_P.WhiteNoise_Mean;

    // End of InitializeConditions for RandomNumber: '<S511>/White Noise'

    // InitializeConditions for UniformRandomNumber: '<S67>/Uniform Random Number' 
    tmp = std::floor(QuadModelv_P.UniformRandomNumber_Seed[0]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>
      (static_cast<uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) +
               static_cast<uint32_T>(t)) << 16U) + static_cast<uint32_T>(t)) +
      static_cast<uint32_T>(i);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_o[0] = tseed;
    QuadModelv_DW.UniformRandomNumber_NextOutput[0] =
      (QuadModelv_P.UniformRandomNumber_Maximum[0] -
       QuadModelv_P.UniformRandomNumber_Minimum[0]) * rt_urand_Upu32_Yd_f_pw_snf
      (&QuadModelv_DW.RandSeed_o[0]) + QuadModelv_P.UniformRandomNumber_Minimum
      [0];
    tmp = std::floor(QuadModelv_P.UniformRandomNumber_Seed[1]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) +
               static_cast<uint32_T>(t)) << 16U) + static_cast<uint32_T>(t)) +
      static_cast<uint32_T>(i);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_o[1] = tseed;
    QuadModelv_DW.UniformRandomNumber_NextOutput[1] =
      (QuadModelv_P.UniformRandomNumber_Maximum[1] -
       QuadModelv_P.UniformRandomNumber_Minimum[1]) * rt_urand_Upu32_Yd_f_pw_snf
      (&QuadModelv_DW.RandSeed_o[1]) + QuadModelv_P.UniformRandomNumber_Minimum
      [1];
    tmp = std::floor(QuadModelv_P.UniformRandomNumber_Seed[2]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) +
               static_cast<uint32_T>(t)) << 16U) + static_cast<uint32_T>(t)) +
      static_cast<uint32_T>(i);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_o[2] = tseed;
    QuadModelv_DW.UniformRandomNumber_NextOutput[2] =
      (QuadModelv_P.UniformRandomNumber_Maximum[2] -
       QuadModelv_P.UniformRandomNumber_Minimum[2]) * rt_urand_Upu32_Yd_f_pw_snf
      (&QuadModelv_DW.RandSeed_o[2]) + QuadModelv_P.UniformRandomNumber_Minimum
      [2];

    // End of InitializeConditions for UniformRandomNumber: '<S67>/Uniform Random Number' 

    // InitializeConditions for RandomNumber: '<S157>/White Noise'
    tmp = std::floor(QuadModelv_P.DrydenWindTurbulenceModelDisc_n[0]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>(static_cast<
      uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) +
               static_cast<uint32_T>(t)) << 16U) + static_cast<uint32_T>(t)) +
      static_cast<uint32_T>(i);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_p[0] = tseed;
    QuadModelv_DW.NextOutput_c[0] = rt_nrand_Upu32_Yd_f_pw_snf
      (&QuadModelv_DW.RandSeed_p[0]) * QuadModelv_P.WhiteNoise_StdDev_j +
      QuadModelv_P.WhiteNoise_Mean_j;

    // InitializeConditions for RandomNumber: '<S118>/White Noise'
    tmp = std::floor(QuadModelv_P.DrydenWindTurbulenceModelDisc_i[0]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>
      (static_cast<uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) +
               static_cast<uint32_T>(t)) << 16U) + static_cast<uint32_T>(t)) +
      static_cast<uint32_T>(i);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_or[0] = tseed;
    QuadModelv_DW.NextOutput_p[0] = rt_nrand_Upu32_Yd_f_pw_snf
      (&QuadModelv_DW.RandSeed_or[0]) * QuadModelv_P.WhiteNoise_StdDev_k +
      QuadModelv_P.WhiteNoise_Mean_m;

    // InitializeConditions for RandomNumber: '<S157>/White Noise'
    tmp = std::floor(QuadModelv_P.DrydenWindTurbulenceModelDisc_n[1]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>
      (static_cast<uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) +
               static_cast<uint32_T>(t)) << 16U) + static_cast<uint32_T>(t)) +
      static_cast<uint32_T>(i);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_p[1] = tseed;
    QuadModelv_DW.NextOutput_c[1] = rt_nrand_Upu32_Yd_f_pw_snf
      (&QuadModelv_DW.RandSeed_p[1]) * QuadModelv_P.WhiteNoise_StdDev_j +
      QuadModelv_P.WhiteNoise_Mean_j;

    // InitializeConditions for RandomNumber: '<S118>/White Noise'
    tmp = std::floor(QuadModelv_P.DrydenWindTurbulenceModelDisc_i[1]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>
      (static_cast<uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) +
               static_cast<uint32_T>(t)) << 16U) + static_cast<uint32_T>(t)) +
      static_cast<uint32_T>(i);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_or[1] = tseed;
    QuadModelv_DW.NextOutput_p[1] = rt_nrand_Upu32_Yd_f_pw_snf
      (&QuadModelv_DW.RandSeed_or[1]) * QuadModelv_P.WhiteNoise_StdDev_k +
      QuadModelv_P.WhiteNoise_Mean_m;

    // InitializeConditions for RandomNumber: '<S157>/White Noise'
    tmp = std::floor(QuadModelv_P.DrydenWindTurbulenceModelDisc_n[2]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>
      (static_cast<uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) +
               static_cast<uint32_T>(t)) << 16U) + static_cast<uint32_T>(t)) +
      static_cast<uint32_T>(i);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_p[2] = tseed;
    QuadModelv_DW.NextOutput_c[2] = rt_nrand_Upu32_Yd_f_pw_snf
      (&QuadModelv_DW.RandSeed_p[2]) * QuadModelv_P.WhiteNoise_StdDev_j +
      QuadModelv_P.WhiteNoise_Mean_j;

    // InitializeConditions for RandomNumber: '<S118>/White Noise'
    tmp = std::floor(QuadModelv_P.DrydenWindTurbulenceModelDisc_i[2]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>
      (static_cast<uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) +
               static_cast<uint32_T>(t)) << 16U) + static_cast<uint32_T>(t)) +
      static_cast<uint32_T>(i);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_or[2] = tseed;
    QuadModelv_DW.NextOutput_p[2] = rt_nrand_Upu32_Yd_f_pw_snf
      (&QuadModelv_DW.RandSeed_or[2]) * QuadModelv_P.WhiteNoise_StdDev_k +
      QuadModelv_P.WhiteNoise_Mean_m;

    // InitializeConditions for RandomNumber: '<S157>/White Noise'
    tmp = std::floor(QuadModelv_P.DrydenWindTurbulenceModelDisc_n[3]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>
      (static_cast<uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) +
               static_cast<uint32_T>(t)) << 16U) + static_cast<uint32_T>(t)) +
      static_cast<uint32_T>(i);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_p[3] = tseed;
    QuadModelv_DW.NextOutput_c[3] = rt_nrand_Upu32_Yd_f_pw_snf
      (&QuadModelv_DW.RandSeed_p[3]) * QuadModelv_P.WhiteNoise_StdDev_j +
      QuadModelv_P.WhiteNoise_Mean_j;

    // InitializeConditions for RandomNumber: '<S118>/White Noise'
    tmp = std::floor(QuadModelv_P.DrydenWindTurbulenceModelDisc_i[3]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>
      (static_cast<uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) +
               static_cast<uint32_T>(t)) << 16U) + static_cast<uint32_T>(t)) +
      static_cast<uint32_T>(i);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_or[3] = tseed;
    QuadModelv_DW.NextOutput_p[3] = rt_nrand_Upu32_Yd_f_pw_snf
      (&QuadModelv_DW.RandSeed_or[3]) * QuadModelv_P.WhiteNoise_StdDev_k +
      QuadModelv_P.WhiteNoise_Mean_m;

    // InitializeConditions for SecondOrderIntegrator: '<S432>/Integrator, Second-Order Limited' 
    QuadModelv_X.IntegratorSecondOrderLimited__i[0] =
      QuadModelv_P.IntegratorSecondOrderLimited__f;
    QuadModelv_X.IntegratorSecondOrderLimited__i[1] =
      QuadModelv_P.IntegratorSecondOrderLimited__f;
    QuadModelv_X.IntegratorSecondOrderLimited__i[2] =
      QuadModelv_P.IntegratorSecondOrderLimited__f;
    QuadModelv_X.IntegratorSecondOrderLimited__i[3] =
      QuadModelv_P.IntegratorSecondOrderLimited__l;
    QuadModelv_X.IntegratorSecondOrderLimited__i[4] =
      QuadModelv_P.IntegratorSecondOrderLimited__l;
    QuadModelv_X.IntegratorSecondOrderLimited__i[5] =
      QuadModelv_P.IntegratorSecondOrderLimited__l;

    // InitializeConditions for Memory: '<S456>/otime'
    QuadModelv_DW.otime_PreviousInput = QuadModelv_P.otime_InitialCondition;

    // InitializeConditions for Memory: '<S455>/olon'
    QuadModelv_DW.olon_PreviousInput = QuadModelv_P.olon_InitialCondition;

    // InitializeConditions for Memory: '<S454>/olat'
    QuadModelv_DW.olat_PreviousInput = QuadModelv_P.olat_InitialCondition;

    // InitializeConditions for Memory: '<S454>/oalt'
    QuadModelv_DW.oalt_PreviousInput = QuadModelv_P.oalt_InitialCondition;

    // InitializeConditions for UniformRandomNumber: '<S398>/Uniform Random Number' 
    tmp = std::floor(QuadModelv_P.UniformRandomNumber_Seed_g);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>
      (static_cast<uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) +
               static_cast<uint32_T>(t)) << 16U) + static_cast<uint32_T>(t)) +
      static_cast<uint32_T>(i);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_e = tseed;
    QuadModelv_DW.UniformRandomNumber_NextOutpu_l =
      (QuadModelv_P.UniformRandomNumber_Maximum_m -
       QuadModelv_P.UniformRandomNumber_Minimum_d) * rt_urand_Upu32_Yd_f_pw_snf(
      &QuadModelv_DW.RandSeed_e) + QuadModelv_P.UniformRandomNumber_Minimum_d;

    // End of InitializeConditions for UniformRandomNumber: '<S398>/Uniform Random Number' 

    // InitializeConditions for UniformRandomNumber: '<S398>/Uniform Random Number4' 
    tmp = std::floor(QuadModelv_P.UniformRandomNumber4_Seed);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>
      (static_cast<uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) +
               static_cast<uint32_T>(t)) << 16U) + static_cast<uint32_T>(t)) +
      static_cast<uint32_T>(i);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_b = tseed;
    QuadModelv_DW.UniformRandomNumber4_NextOutput =
      (QuadModelv_P.UniformRandomNumber4_Maximum -
       QuadModelv_P.UniformRandomNumber4_Minimum) * rt_urand_Upu32_Yd_f_pw_snf
      (&QuadModelv_DW.RandSeed_b) + QuadModelv_P.UniformRandomNumber4_Minimum;

    // End of InitializeConditions for UniformRandomNumber: '<S398>/Uniform Random Number4' 

    // InitializeConditions for TransferFcn: '<S368>/Transfer Fcn4'
    QuadModelv_X.TransferFcn4_CSTATE = 0.0;

    // InitializeConditions for TransferFcn: '<S368>/Transfer Fcn1'
    QuadModelv_X.TransferFcn1_CSTATE = 0.0;

    // InitializeConditions for TransferFcn: '<S368>/Transfer Fcn2'
    QuadModelv_X.TransferFcn2_CSTATE = 0.0;

    // InitializeConditions for UniformRandomNumber: '<S322>/Uniform Random Number4' 
    tmp = std::floor(QuadModelv_P.UniformRandomNumber4_Seed_e[0]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>
      (static_cast<uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) +
               static_cast<uint32_T>(t)) << 16U) + static_cast<uint32_T>(t)) +
      static_cast<uint32_T>(i);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_n[0] = tseed;
    QuadModelv_DW.UniformRandomNumber4_NextOutp_p[0] =
      (QuadModelv_P.UniformRandomNumber4_Maximum_c[0] -
       QuadModelv_P.UniformRandomNumber4_Minimum_o[0]) *
      rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_n[0]) +
      QuadModelv_P.UniformRandomNumber4_Minimum_o[0];
    tmp = std::floor(QuadModelv_P.UniformRandomNumber4_Seed_e[1]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>
      (static_cast<uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) +
               static_cast<uint32_T>(t)) << 16U) + static_cast<uint32_T>(t)) +
      static_cast<uint32_T>(i);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_n[1] = tseed;
    QuadModelv_DW.UniformRandomNumber4_NextOutp_p[1] =
      (QuadModelv_P.UniformRandomNumber4_Maximum_c[1] -
       QuadModelv_P.UniformRandomNumber4_Minimum_o[1]) *
      rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_n[1]) +
      QuadModelv_P.UniformRandomNumber4_Minimum_o[1];
    tmp = std::floor(QuadModelv_P.UniformRandomNumber4_Seed_e[2]);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = std::fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? static_cast<uint32_T>(-static_cast<int32_T>
      (static_cast<uint32_T>(-tmp))) : static_cast<uint32_T>(tmp);
    i = static_cast<int32_T>(tseed >> 16U);
    t = static_cast<int32_T>(tseed & 32768U);
    tseed = ((((tseed - (static_cast<uint32_T>(i) << 16U)) +
               static_cast<uint32_T>(t)) << 16U) + static_cast<uint32_T>(t)) +
      static_cast<uint32_T>(i);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    QuadModelv_DW.RandSeed_n[2] = tseed;
    QuadModelv_DW.UniformRandomNumber4_NextOutp_p[2] =
      (QuadModelv_P.UniformRandomNumber4_Maximum_c[2] -
       QuadModelv_P.UniformRandomNumber4_Minimum_o[2]) *
      rt_urand_Upu32_Yd_f_pw_snf(&QuadModelv_DW.RandSeed_n[2]) +
      QuadModelv_P.UniformRandomNumber4_Minimum_o[2];

    // End of InitializeConditions for UniformRandomNumber: '<S322>/Uniform Random Number4' 

    // SystemInitialize for IfAction SubSystem: '<S194>/Negative Trace'
    // '<S191>:1:6' hFault=false;
    // '<S191>:1:9' fParam=zeros(20,1);
    // '<S189>:1:6' hFault=false;
    // '<S189>:1:9' fParam=zeros(20,1);
    // '<S190>:1:6' hFault=false;
    // '<S190>:1:9' fParam=zeros(20,1);
    // '<S188>:1:8' tOld=0;
    QuadModelv_NegativeTrace_Init(&QuadModelv_DW.NegativeTrace_d);

    // End of SystemInitialize for SubSystem: '<S194>/Negative Trace'

    // SystemInitialize for Merge: '<S194>/Merge'
    QuadModelv_B.Merge[0] = QuadModelv_P.Merge_InitialOutput_f[0];
    QuadModelv_B.Merge[1] = QuadModelv_P.Merge_InitialOutput_f[1];
    QuadModelv_B.Merge[2] = QuadModelv_P.Merge_InitialOutput_f[2];
    QuadModelv_B.Merge[3] = QuadModelv_P.Merge_InitialOutput_f[3];

    // SystemInitialize for IfAction SubSystem: '<S193>/Negative Trace'
    QuadModelv_NegativeTrace_Init(&QuadModelv_DW.NegativeTrace);

    // End of SystemInitialize for SubSystem: '<S193>/Negative Trace'

    // SystemInitialize for Merge: '<S193>/Merge'
    QuadModelv_B.Merge_c[0] = QuadModelv_P.Merge_InitialOutput_h4[0];
    QuadModelv_B.Merge_c[1] = QuadModelv_P.Merge_InitialOutput_h4[1];
    QuadModelv_B.Merge_c[2] = QuadModelv_P.Merge_InitialOutput_h4[2];
    QuadModelv_B.Merge_c[3] = QuadModelv_P.Merge_InitialOutput_h4[3];

    // SystemInitialize for MATLAB Function: '<Root>/CollisionDetection'
    QuadModelv_DW.method_do = 7U;
    QuadModelv_DW.state_g3 = 1144108930U;
    QuadModelv_DW.state_pv[0] = 362436069U;
    QuadModelv_DW.state_pv[1] = 521288629U;

    // SystemInitialize for MATLAB Function: '<S7>/Ground Model'
    // '<S3>:1:7' isCol=int8(0);
    // '<S3>:1:13' fOut=[0;0;0;0;0;0];
    // '<S3>:1:19' mv0=[0;0;0];
    // '<S3>:1:25' tColi=0;
    // '<S196>:1:11' takeoffFlag=int8(0);
    // '<S196>:1:16' landFlag=int8(1);
    QuadModelv_DW.landFlag = 1;

    // SystemInitialize for MATLAB Function: '<Root>/Crash_Motor_Fault'
    // '<S513>:1:6' hFault=false;
    // '<S513>:1:9' fParam=zeros(20,1);
    QuadModelv_DW.method_b = 7U;
    QuadModelv_DW.state_h = 1144108930U;
    QuadModelv_DW.state_ef[0] = 362436069U;
    QuadModelv_DW.state_ef[1] = 521288629U;

    // SystemInitialize for MATLAB Function: '<Root>/CollisionDetection'
    // '<S59>:1:6' hFault=false;
    // '<S59>:1:9' fParam=zeros(20,1);
    // '<S60>:1:6' hFault=false;
    // '<S60>:1:9' fParam=zeros(20,1);
    // '<S61>:1:6' hFault=false;
    // '<S61>:1:9' fParam=zeros(20,1);
    // '<S62>:1:6' hFault=false;
    // '<S62>:1:9' fParam=zeros(20,1);
    // '<S58>:1:15' discharge = 0;
    // '<S58>:1:20' startTime = 0;
    // '<S58>:1:25' reset = 0;
    // '<S295>:1:6' hFault=false;
    // '<S295>:1:9' fParam=zeros(20,1);
    std::memcpy(&QuadModelv_DW.state_jn[0], &tmp_0[0], 625U * sizeof(uint32_T));

    // SystemInitialize for MATLAB Function: '<Root>/Crash_Motor_Fault'
    std::memcpy(&QuadModelv_DW.state_m[0], &tmp_0[0], 625U * sizeof(uint32_T));

    // SystemInitialize for MATLAB Function: '<S293>/MotorFaultModel'
    std::memcpy(&QuadModelv_DW.state_jf[0], &tmp_0[0], 625U * sizeof(uint32_T));

    // '<S296>:1:59' phase = true;
    QuadModelv_DW.phase_o = true;

    // '<S296>:1:60' V1 = 0;
    // '<S296>:1:61' V2 = 0;
    // '<S296>:1:62' S = 0;
    QuadModelv_DW.method_l1 = 7U;
    QuadModelv_DW.state_ib = 1144108930U;
    QuadModelv_DW.state_po[0] = 362436069U;
    QuadModelv_DW.state_po[1] = 521288629U;

    // SystemInitialize for Atomic SubSystem: '<S294>/MotorNonlinearDynamic1'
    Qua_MotorNonlinearDynamic1_Init(&QuadModelv_DW.MotorNonlinearDynamic1,
      &QuadModelv_X.MotorNonlinearDynamic1);

    // End of SystemInitialize for SubSystem: '<S294>/MotorNonlinearDynamic1'

    // SystemInitialize for Atomic SubSystem: '<S294>/MotorNonlinearDynamic2'
    Qua_MotorNonlinearDynamic1_Init(&QuadModelv_DW.MotorNonlinearDynamic2,
      &QuadModelv_X.MotorNonlinearDynamic2);

    // End of SystemInitialize for SubSystem: '<S294>/MotorNonlinearDynamic2'

    // SystemInitialize for Atomic SubSystem: '<S294>/MotorNonlinearDynamic3'
    Qua_MotorNonlinearDynamic1_Init(&QuadModelv_DW.MotorNonlinearDynamic3,
      &QuadModelv_X.MotorNonlinearDynamic3);

    // End of SystemInitialize for SubSystem: '<S294>/MotorNonlinearDynamic3'

    // SystemInitialize for Atomic SubSystem: '<S294>/MotorNonlinearDynamic4'
    Qua_MotorNonlinearDynamic1_Init(&QuadModelv_DW.MotorNonlinearDynamic4,
      &QuadModelv_X.MotorNonlinearDynamic4);

    // End of SystemInitialize for SubSystem: '<S294>/MotorNonlinearDynamic4'

    // SystemInitialize for Atomic SubSystem: '<S294>/MotorNonlinearDynamic5'
    Qua_MotorNonlinearDynamic1_Init(&QuadModelv_DW.MotorNonlinearDynamic5,
      &QuadModelv_X.MotorNonlinearDynamic5);

    // End of SystemInitialize for SubSystem: '<S294>/MotorNonlinearDynamic5'

    // SystemInitialize for Atomic SubSystem: '<S294>/MotorNonlinearDynamic6'
    Qua_MotorNonlinearDynamic1_Init(&QuadModelv_DW.MotorNonlinearDynamic6,
      &QuadModelv_X.MotorNonlinearDynamic6);

    // End of SystemInitialize for SubSystem: '<S294>/MotorNonlinearDynamic6'

    // SystemInitialize for Atomic SubSystem: '<S294>/MotorNonlinearDynamic7'
    Qua_MotorNonlinearDynamic1_Init(&QuadModelv_DW.MotorNonlinearDynamic7,
      &QuadModelv_X.MotorNonlinearDynamic7);

    // End of SystemInitialize for SubSystem: '<S294>/MotorNonlinearDynamic7'

    // SystemInitialize for Atomic SubSystem: '<S294>/MotorNonlinearDynamic8'
    Qua_MotorNonlinearDynamic1_Init(&QuadModelv_DW.MotorNonlinearDynamic8,
      &QuadModelv_X.MotorNonlinearDynamic8);

    // End of SystemInitialize for SubSystem: '<S294>/MotorNonlinearDynamic8'

    // SystemInitialize for Enabled SubSystem: '<S512>/Distance into gust (x)'
    // InitializeConditions for Integrator: '<S515>/Distance into Gust (x) (Limited to gust length d)' 
    QuadModelv_X.DistanceintoGustxLimitedtogus_e =
      QuadModelv_P.DistanceintoGustxLimitedtogustl;

    // SystemInitialize for Integrator: '<S515>/Distance into Gust (x) (Limited to gust length d)' incorporates:
    //   Outport: '<S515>/x'

    QuadModelv_B.DistanceintoGustxLimitedtogustl = QuadModelv_P.x_Y0;

    // End of SystemInitialize for SubSystem: '<S512>/Distance into gust (x)'

    // SystemInitialize for Enabled SubSystem: '<S512>/Distance into gust (y)'
    QuadMode_Distanceintogusty_Init(&QuadModelv_B.Distanceintogusty,
      &QuadModelv_P.Distanceintogusty, &QuadModelv_X.Distanceintogusty);

    // End of SystemInitialize for SubSystem: '<S512>/Distance into gust (y)'

    // SystemInitialize for Enabled SubSystem: '<S512>/Distance into gust (z)'
    QuadMode_Distanceintogusty_Init(&QuadModelv_B.Distanceintogustz,
      &QuadModelv_P.Distanceintogustz, &QuadModelv_X.Distanceintogustz);

    // End of SystemInitialize for SubSystem: '<S512>/Distance into gust (z)'

    // SystemInitialize for Enabled SubSystem: '<S148>/Hugw(z)'
    // '<S100>:1:6' hFault=false;
    // '<S100>:1:9' fParam=zeros(20,1);
    // '<S96>:1:6' hFault=false;
    // '<S96>:1:9' fParam=zeros(20,1);
    // '<S97>:1:6' hFault=false;
    // '<S97>:1:9' fParam=zeros(20,1);
    // '<S98>:1:6' hFault=false;
    // '<S98>:1:9' fParam=zeros(20,1);
    // '<S99>:1:6' hFault=false;
    // '<S99>:1:9' fParam=zeros(20,1);
    QuadModelv_Hugwz_Init(&QuadModelv_B.Hugwz_g, &QuadModelv_DW.Hugwz_g,
                          &QuadModelv_P.Hugwz_g);

    // End of SystemInitialize for SubSystem: '<S148>/Hugw(z)'

    // SystemInitialize for Enabled SubSystem: '<S148>/Hvgw(z)'
    QuadModelv_Hvgwz_Init(&QuadModelv_B.Hvgwz_f, &QuadModelv_DW.Hvgwz_f,
                          &QuadModelv_P.Hvgwz_f);

    // End of SystemInitialize for SubSystem: '<S148>/Hvgw(z)'

    // SystemInitialize for Enabled SubSystem: '<S148>/Hwgw(z)'
    QuadModelv_Hwgwz_Init(&QuadModelv_B.Hwgwz_a, &QuadModelv_DW.Hwgwz_a,
                          &QuadModelv_P.Hwgwz_a);

    // End of SystemInitialize for SubSystem: '<S148>/Hwgw(z)'

    // SystemInitialize for MATLAB Function: '<S67>/MATLAB Function'
    std::memcpy(&QuadModelv_DW.state_iu[0], &tmp_0[0], 625U * sizeof(uint32_T));
    QuadModelv_DW.method_e = 7U;
    QuadModelv_DW.state_ny = 1144108930U;
    QuadModelv_DW.state_km[0] = 362436069U;
    QuadModelv_DW.state_km[1] = 521288629U;

    // '<S101>:1:12' t0=0;
    // '<S101>:1:17' isInGust=0;
    // '<S101>:1:22' t1=2;
    QuadModelv_DW.t1 = 2.0;

    // '<S101>:1:27' a=0.5;
    QuadModelv_DW.a = 0.5;

    // '<S101>:1:32' ang=pi/2;
    QuadModelv_DW.ang = 1.5707963267948966;

    // SystemInitialize for Enabled SubSystem: '<S109>/Hugw(z)'
    QuadModelv_Hugwz_Init(&QuadModelv_B.Hugwz, &QuadModelv_DW.Hugwz,
                          &QuadModelv_P.Hugwz);

    // End of SystemInitialize for SubSystem: '<S109>/Hugw(z)'

    // SystemInitialize for Enabled SubSystem: '<S109>/Hvgw(z)'
    QuadModelv_Hvgwz_Init(&QuadModelv_B.Hvgwz, &QuadModelv_DW.Hvgwz,
                          &QuadModelv_P.Hvgwz);

    // End of SystemInitialize for SubSystem: '<S109>/Hvgw(z)'

    // SystemInitialize for Enabled SubSystem: '<S109>/Hwgw(z)'
    QuadModelv_Hwgwz_Init(&QuadModelv_B.Hwgwz, &QuadModelv_DW.Hwgwz,
                          &QuadModelv_P.Hwgwz);

    // End of SystemInitialize for SubSystem: '<S109>/Hwgw(z)'

    // SystemInitialize for MATLAB Function: '<S398>/Acc NoiseFun'
    // '<S401>:1:6' hFault=false;
    // '<S401>:1:9' fParam=zeros(20,1);
    // '<S399>:1:51' phase = true;
    QuadModelv_DW.phase_br = true;

    // '<S399>:1:52' V1 = 0;
    // '<S399>:1:53' V2 = 0;
    // '<S399>:1:54' S = 0;
    QuadModelv_DW.method_j = 7U;
    QuadModelv_DW.state_d = 1144108930U;
    QuadModelv_DW.state_k[0] = 362436069U;
    QuadModelv_DW.state_k[1] = 521288629U;

    // '<S402>:1:6' hFault=false;
    // '<S402>:1:9' fParam=zeros(20,1);
    std::memcpy(&QuadModelv_DW.state_f[0], &tmp_0[0], 625U * sizeof(uint32_T));

    // SystemInitialize for MATLAB Function: '<S398>/Gyro NoiseFun'
    std::memcpy(&QuadModelv_DW.state_n[0], &tmp_0[0], 625U * sizeof(uint32_T));

    // '<S405>:1:51' phase = true;
    QuadModelv_DW.phase_a = true;

    // '<S405>:1:52' V1 = 0;
    // '<S405>:1:53' V2 = 0;
    // '<S405>:1:54' S = 0;
    QuadModelv_DW.method_l = 7U;
    QuadModelv_DW.state_e = 1144108930U;
    QuadModelv_DW.state_c[0] = 362436069U;

    // SystemInitialize for Enabled SubSystem: '<S442>/Convert from geodetic to  spherical coordinates ' 
    // SystemInitialize for Iterator SubSystem: '<S452>/For Iterator Subsystem'
    // InitializeConditions for UnitDelay: '<S499>/Unit Delay1'
    QuadModelv_DW.UnitDelay1_DSTATE[0] =
      QuadModelv_P.UnitDelay1_InitialCondition_d;

    // End of SystemInitialize for SubSystem: '<S452>/For Iterator Subsystem'
    // End of SystemInitialize for SubSystem: '<S442>/Convert from geodetic to  spherical coordinates ' 

    // SystemInitialize for MATLAB Function: '<S398>/Gyro NoiseFun'
    QuadModelv_DW.state_c[1] = 521288629U;

    // SystemInitialize for Enabled SubSystem: '<S442>/Convert from geodetic to  spherical coordinates ' 
    // SystemInitialize for Iterator SubSystem: '<S452>/For Iterator Subsystem'
    // InitializeConditions for UnitDelay: '<S499>/Unit Delay1'
    QuadModelv_DW.UnitDelay1_DSTATE[1] =
      QuadModelv_P.UnitDelay1_InitialCondition_d;

    // End of SystemInitialize for SubSystem: '<S452>/For Iterator Subsystem'

    // SystemInitialize for SignalConversion generated from: '<S452>/sp[13]' incorporates:
    //   Outport: '<S452>/sp[13]'

    std::memcpy(&QuadModelv_B.OutportBufferForsp13[0], &QuadModelv_P.sp13_Y0[0],
                13U * sizeof(real_T));

    // SystemInitialize for SignalConversion generated from: '<S452>/cp[13]' incorporates:
    //   Outport: '<S452>/cp[13]'

    std::memcpy(&QuadModelv_B.OutportBufferForcp13[0], &QuadModelv_P.cp13_Y0[0],
                13U * sizeof(real_T));

    // End of SystemInitialize for SubSystem: '<S442>/Convert from geodetic to  spherical coordinates ' 

    // SystemInitialize for Enabled SubSystem: '<S442>/Convert from geodetic to  spherical coordinates' 
    // SystemInitialize for Sqrt: '<S451>/sqrt' incorporates:
    //   Outport: '<S451>/r'

    QuadModelv_B.sqrt_o = QuadModelv_P.r_Y0;

    // SystemInitialize for Product: '<S492>/Product4' incorporates:
    //   Outport: '<S451>/ct'

    QuadModelv_B.Product4_p = QuadModelv_P.ct_Y0;

    // SystemInitialize for Sqrt: '<S498>/sqrt' incorporates:
    //   Outport: '<S451>/st'

    QuadModelv_B.sqrt_d = QuadModelv_P.st_Y0;

    // SystemInitialize for Product: '<S497>/Product12' incorporates:
    //   Outport: '<S451>/sa'

    QuadModelv_B.Product12 = QuadModelv_P.sa_Y0;

    // SystemInitialize for Product: '<S491>/Product11' incorporates:
    //   Outport: '<S451>/ca'

    QuadModelv_B.Product11 = QuadModelv_P.ca_Y0;

    // End of SystemInitialize for SubSystem: '<S442>/Convert from geodetic to  spherical coordinates' 

    // SystemInitialize for Iterator SubSystem: '<S442>/Compute magnetic vector in spherical coordinates' 
    // SystemInitialize for Iterator SubSystem: '<S450>/For Iterator Subsystem'
    // SystemInitialize for Enabled SubSystem: '<S458>/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations' 
    // SystemInitialize for Merge: '<S460>/Merge1'
    QuadModelv_B.Merge1_a = QuadModelv_P.Merge1_InitialOutput;

    // SystemInitialize for Merge: '<S460>/Merge'
    QuadModelv_B.Merge_i = QuadModelv_P.Merge_InitialOutput;

    // End of SystemInitialize for SubSystem: '<S458>/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations' 

    // SystemInitialize for Enabled SubSystem: '<S458>/Time adjust the gauss coefficients' 
    // InitializeConditions for UnitDelay: '<S461>/Unit Delay'
    std::memcpy(&QuadModelv_DW.UnitDelay_DSTATE_k[0],
                &QuadModelv_P.UnitDelay_InitialCondition_b[0], 169U * sizeof
                (real_T));

    // InitializeConditions for UnitDelay: '<S488>/Unit Delay'
    std::memcpy(&QuadModelv_DW.UnitDelay_DSTATE_b[0],
                &QuadModelv_P.UnitDelay_InitialCondition_p[0], 169U * sizeof
                (real_T));

    // SystemInitialize for Sum: '<S461>/Sum2' incorporates:
    //   Outport: '<S461>/tc[13][13]'

    std::memcpy(&QuadModelv_B.Sum2_h[0], &QuadModelv_P.tc1313_Y0[0], 169U *
                sizeof(real_T));

    // End of SystemInitialize for SubSystem: '<S458>/Time adjust the gauss coefficients' 

    // SystemInitialize for Enabled SubSystem: '<S458>/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations' 
    // InitializeConditions for UnitDelay: '<S460>/Unit Delay'
    std::memcpy(&QuadModelv_DW.UnitDelay_DSTATE_i[0],
                &QuadModelv_P.UnitDelay_InitialCondition[0], 169U * sizeof
                (real_T));

    // InitializeConditions for UnitDelay: '<S460>/Unit Delay1'
    std::memcpy(&QuadModelv_DW.UnitDelay1_DSTATE_j[0],
                &QuadModelv_P.UnitDelay1_InitialCondition_j[0], 169U * sizeof
                (real_T));

    // SystemInitialize for Assignment: '<S460>/Assignment' incorporates:
    //   Outport: '<S460>/dp[13][13]'

    std::memcpy(&QuadModelv_B.Assignment_g[0], &QuadModelv_P.dp1313_Y0[0], 169U *
                sizeof(real_T));

    // SystemInitialize for Assignment: '<S460>/Assignment_snorm' incorporates:
    //   Outport: '<S460>/snorm[169]'

    std::memcpy(&QuadModelv_B.Assignment_snorm[0], &QuadModelv_P.snorm169_Y0[0],
                169U * sizeof(real_T));

    // End of SystemInitialize for SubSystem: '<S458>/Compute unnormalized associated  legendre polynomials and  derivatives via recursion relations' 

    // SystemInitialize for Enabled SubSystem: '<S459>/Special case - North//South Geographic Pole' 
    // InitializeConditions for UnitDelay: '<S462>/Unit Delay1'
    std::memcpy(&QuadModelv_DW.UnitDelay1_DSTATE_a[0],
                &QuadModelv_P.UnitDelay1_InitialCondition[0], 13U * sizeof
                (real_T));

    // SystemInitialize for IfAction SubSystem: '<S462>/If Action Subsystem1'
    // SystemInitialize for Assignment: '<S466>/Assignment2' incorporates:
    //   Outport: '<S466>/pp[13]'

    std::memcpy(&QuadModelv_B.Assignment2_a[0], &QuadModelv_P.pp13_Y0[0], 13U *
                sizeof(real_T));

    // End of SystemInitialize for SubSystem: '<S462>/If Action Subsystem1'

    // SystemInitialize for IfAction SubSystem: '<S462>/If Action Subsystem2'
    // SystemInitialize for Assignment: '<S467>/Assignment2' incorporates:
    //   Outport: '<S467>/pp[13]'

    std::memcpy(&QuadModelv_B.Assignment2_m[0], &QuadModelv_P.pp13_Y0_n[0], 13U *
                sizeof(real_T));

    // End of SystemInitialize for SubSystem: '<S462>/If Action Subsystem2'

    // SystemInitialize for Product: '<S462>/Product2' incorporates:
    //   Outport: '<S462>/bpp'

    QuadModelv_B.Product2_b = QuadModelv_P.bpp_Y0;

    // End of SystemInitialize for SubSystem: '<S459>/Special case - North//South Geographic Pole' 

    // SystemInitialize for Sum: '<S459>/Sum1' incorporates:
    //   Outport: '<S458>/bt'

    QuadModelv_B.Sum1_f = QuadModelv_P.bt_Y0;

    // SystemInitialize for Sum: '<S459>/Sum2' incorporates:
    //   Outport: '<S458>/bp'

    QuadModelv_B.Sum2_k = QuadModelv_P.bp_Y0;

    // SystemInitialize for Sum: '<S459>/Sum3' incorporates:
    //   Outport: '<S458>/br'

    QuadModelv_B.Sum3 = QuadModelv_P.br_Y0;

    // SystemInitialize for Sum: '<S459>/Sum5' incorporates:
    //   Outport: '<S458>/bpp'

    QuadModelv_B.Sum5 = QuadModelv_P.bpp_Y0_k;

    // End of SystemInitialize for SubSystem: '<S450>/For Iterator Subsystem'

    // SystemInitialize for Sum: '<S450>/Sum1' incorporates:
    //   Outport: '<S450>/bt,bp,br,bpp'

    QuadModelv_B.Sum1_l[0] = QuadModelv_P.btbpbrbpp_Y0[0];
    QuadModelv_B.Sum1_l[1] = QuadModelv_P.btbpbrbpp_Y0[1];
    QuadModelv_B.Sum1_l[2] = QuadModelv_P.btbpbrbpp_Y0[2];
    QuadModelv_B.Sum1_l[3] = QuadModelv_P.btbpbrbpp_Y0[3];

    // End of SystemInitialize for SubSystem: '<S442>/Compute magnetic vector in spherical coordinates' 

    // SystemInitialize for MATLAB Function: '<S398>/Mag NoiseFun'
    // '<S403>:1:6' hFault=false;
    // '<S403>:1:9' fParam=zeros(20,1);
    // '<S409>:1:51' phase = true;
    QuadModelv_DW.phase_b = true;

    // '<S409>:1:52' V1 = 0;
    // '<S409>:1:53' V2 = 0;
    // '<S409>:1:54' S = 0;
    QuadModelv_DW.method_d = 7U;
    QuadModelv_DW.state_g = 1144108930U;
    QuadModelv_DW.state_j[0] = 362436069U;
    QuadModelv_DW.state_j[1] = 521288629U;

    // '<S404>:1:6' hFault=false;
    // '<S404>:1:9' fParam=zeros(20,1);
    std::memcpy(&QuadModelv_DW.state_b[0], &tmp_0[0], 625U * sizeof(uint32_T));

    // SystemInitialize for MATLAB Function: '<S398>/baro NoiseFun'
    std::memcpy(&QuadModelv_DW.state_i[0], &tmp_0[0], 625U * sizeof(uint32_T));

    // '<S412>:1:28' phase = true;
    QuadModelv_DW.phase = true;

    // '<S412>:1:29' V1 = 0;
    // '<S412>:1:30' V2 = 0;
    // '<S412>:1:31' S = 0;
    QuadModelv_DW.method = 7U;
    QuadModelv_DW.state = 1144108930U;
    QuadModelv_DW.state_a[0] = 362436069U;
    QuadModelv_DW.state_a[1] = 521288629U;

    // SystemInitialize for IfAction SubSystem: '<S321>/Negative Trace'
    QuadModelv_NegativeTrace_Init(&QuadModelv_DW.NegativeTrace_l);

    // End of SystemInitialize for SubSystem: '<S321>/Negative Trace'

    // SystemInitialize for Merge: '<S321>/Merge'
    QuadModelv_B.Merge_f[0] = QuadModelv_P.Merge_InitialOutput_fc[0];
    QuadModelv_B.Merge_f[1] = QuadModelv_P.Merge_InitialOutput_fc[1];
    QuadModelv_B.Merge_f[2] = QuadModelv_P.Merge_InitialOutput_fc[2];
    QuadModelv_B.Merge_f[3] = QuadModelv_P.Merge_InitialOutput_fc[3];

    // SystemInitialize for MATLAB Function: '<S366>/Acc NoiseFun'
    // '<S370>:1:6' hFault=false;
    // '<S370>:1:9' fParam=zeros(20,1);
    std::memcpy(&QuadModelv_DW.state_p[0], &tmp_0[0], 625U * sizeof(uint32_T));

    // '<S369>:1:63' phase = true;
    QuadModelv_DW.phase_l = true;

    // '<S369>:1:64' V1 = 0;
    // '<S369>:1:65' V2 = 0;
    // '<S369>:1:66' S = 0;
    QuadModelv_DW.method_dt = 7U;
    QuadModelv_DW.state_ee = 1144108930U;
    QuadModelv_DW.state_cw[0] = 362436069U;
    QuadModelv_DW.state_cw[1] = 521288629U;

    // SystemInitialize for Enabled SubSystem: '<S108>/Hpgw'
    // '<S325>:1:5' xTemp=zeros(8,1);
    // '<S325>:1:10' kk=0;
    // '<S325>:1:15' ss=zeros(6,1);
    QuadModelv_Hpgw_Init(&QuadModelv_B.Hpgw, &QuadModelv_DW.Hpgw,
                         &QuadModelv_P.Hpgw);

    // End of SystemInitialize for SubSystem: '<S108>/Hpgw'

    // SystemInitialize for Enabled SubSystem: '<S108>/Hqgw'
    QuadModelv_Hqgw_Init(&QuadModelv_B.Hqgw, &QuadModelv_DW.Hqgw,
                         &QuadModelv_P.Hqgw);

    // End of SystemInitialize for SubSystem: '<S108>/Hqgw'

    // SystemInitialize for Enabled SubSystem: '<S108>/Hrgw'
    QuadModelv_Hrgw_Init(&QuadModelv_B.Hrgw, &QuadModelv_DW.Hrgw,
                         &QuadModelv_P.Hrgw);

    // End of SystemInitialize for SubSystem: '<S108>/Hrgw'

    // SystemInitialize for Enabled SubSystem: '<S147>/Hpgw'
    QuadModelv_Hpgw_Init(&QuadModelv_B.Hpgw_h, &QuadModelv_DW.Hpgw_h,
                         &QuadModelv_P.Hpgw_h);

    // End of SystemInitialize for SubSystem: '<S147>/Hpgw'

    // SystemInitialize for Enabled SubSystem: '<S147>/Hqgw'
    QuadModelv_Hqgw_Init(&QuadModelv_B.Hqgw_a, &QuadModelv_DW.Hqgw_a,
                         &QuadModelv_P.Hqgw_a);

    // End of SystemInitialize for SubSystem: '<S147>/Hqgw'

    // SystemInitialize for Enabled SubSystem: '<S147>/Hrgw'
    QuadModelv_Hrgw_Init(&QuadModelv_B.Hrgw_a, &QuadModelv_DW.Hrgw_a,
                         &QuadModelv_P.Hrgw_a);

    // End of SystemInitialize for SubSystem: '<S147>/Hrgw'
  }

  // set "at time zero" to false
  if (rtmIsFirstInitCond((&QuadModelv_M))) {
    rtmSetFirstInitCond((&QuadModelv_M), 0);
  }
}

// Model terminate function
void MulticopterModelClass::terminate()
{
  // (no terminate code required)
}

// Constructor
MulticopterModelClass::MulticopterModelClass() :
  QuadModelv_U(),
  QuadModelv_Y(),
  QuadModelv_B(),
  QuadModelv_DW(),
  QuadModelv_X(),
  QuadModelv_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
MulticopterModelClass::~MulticopterModelClass()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
RT_MODEL_QuadModelv_T * MulticopterModelClass::getRTM()
{
  return (&QuadModelv_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
