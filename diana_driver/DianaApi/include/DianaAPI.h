#pragma once

#include "DianaAPIDef.h"

#pragma pack(1)

// 下列 ifdef 块是创建使从 DLL 导出更简单的宏的标准方法。
// 此 DLL 中的所有文件都是用命令行上定义的 DIANA_API_EXPORTS 符号编译的。
// 在使用此 DLL 的任何其他项目上不应定义此符号。
// 这样，源文件中包含此文件的任何其他项目都会将 DIANA_API_EXPORTS 函数视为是从 DLL 导入的，
// 而此 DLL 则将用此宏定义的符号视为是被导出的。

#if defined _WIN32
#ifdef DIANA_API_EXPORTS
#define DIANA_API extern "C" __declspec(dllexport)
#else
#define DIANA_API extern "C" __declspec(dllimport)
#endif
#else
#if __GNUC__ >= 4
#define DIANA_API extern "C" __attribute__((visibility("default")))
#else
#define DIANA_API extern "C"
#endif
#endif

/*************************************** version 0.1 ***************************************/

extern "C" typedef struct _SrvNetSt {
    char SrvIp[32];
    unsigned short LocHeartbeatPort;
    unsigned short LocRobotStatePort;
    unsigned short LocSrvPort;
} srv_net_st;

DIANA_API int initSrv(FNCERRORCALLBACK fnError, FNCSTATECALLBACK fnState, srv_net_st *pinfo);

DIANA_API int destroySrv();

DIANA_API int setPushPeriod(int intPeriod);  //设置状态推送频率

extern "C" typedef enum _MoveTCPDirection {
    T_MOVE_X_UP,
    T_MOVE_X_DOWN,
    T_MOVE_Y_UP,
    T_MOVE_Y_DOWN,
    T_MOVE_Z_UP,
    T_MOVE_Z_DOWN
} tcp_direction_e;

DIANA_API int moveTCP(/*IN*/ tcp_direction_e d, /*IN*/ double v, /*IN*/ double a, /*IN*/ double *active_tcp = nullptr);

DIANA_API int rotationTCP(/*IN*/ tcp_direction_e d, /*IN*/ double v, /*IN*/ double a,
                          /*IN*/ double *active_tcp = nullptr);

extern "C" typedef enum _MoveJointDirection {
    T_MOVE_UP = 0,
    T_MOVE_DOWN,
} joint_direction_e;

DIANA_API int moveJoint(/*IN*/ joint_direction_e d, /*IN*/ int i, /*IN*/ double v, /*IN*/ double a);

DIANA_API int moveJToTarget(/*IN[7]*/ double *joints, /*IN*/ double v, /*IN*/ double a);

DIANA_API int moveJToPose(/*IN[6]*/ double *pose, /*IN*/ double v, /*IN*/ double a,
                          /*IN*/ double *active_tcp = nullptr);

#define moveJ moveJToTarget

#define moveL moveLToPose

DIANA_API int moveLToTarget(/*IN[7]*/ double *joints, /*IN*/ double v, /*IN*/ double a);

DIANA_API int moveLToPose(/*IN[6]*/ double *pose, /*IN*/ double v, /*IN*/ double a,
                          /*IN*/ double *active_tcp = nullptr);

DIANA_API int speedJ(/*IN[7]*/ double *speed, /*IN*/ double a, /*option*/ double t);

DIANA_API int speedL(/*IN[6]*/ double *speed, /*IN[2]*/ double *a, /*option*/ double t,
                     /*IN*/ double *active_tcp = nullptr);

DIANA_API int freeDriving(bool enable);

DIANA_API int stop();

DIANA_API int forward(/*IN[7]*/ double *joints, /*OUT[6]*/ double *pose, /*IN*/ double *active_tcp = nullptr);

DIANA_API int inverse(/*IN[6]*/ double *pose, /*OUT[7]*/ double *joints, /*IN*/ double *active_tcp = nullptr);

DIANA_API int getJointPos(/*OUT[7]*/ double *joints);

DIANA_API int getJointAngularVel(/*OUT[7]*/ double *vels);

DIANA_API int getJointCurrent(/*OUT[7]*/ double *joints);

DIANA_API int getJointTorque(/*OUT[7]*/ double *torques);

DIANA_API int getTcpPos(/*OUT[6]*/ double *pose);

DIANA_API double getTcpExternalForce();

DIANA_API int releaseBrake();  // powerOn

DIANA_API int holdBrake();  // powerOff

typedef enum _Mode {
    T_MODE_INVALID = -1,
    T_MODE_POSITION = 0,
    T_MODE_JOINT_IMPEDANCE,
    T_MODE_CART_IMPEDANCE,
} mode_e;

DIANA_API int changeControlMode(mode_e m);  // ChangeMode:Position, JointImpedance, CartImpedance

/*************************************** version 0.2 ***************************************/

DIANA_API unsigned short getLibraryVersion();

DIANA_API const char *formatError(int e);

DIANA_API int getLastError();

DIANA_API int setLastError(int e);

// setDefaultActiveTcp:该函数将会改变moveTCP，rotationTCP，moveJToPose，moveLToPose，speedJ，speedL，forward，inverse，getTcpPos，getTcpExternalForce的默认行为。
DIANA_API int setDefaultActiveTcp(/*IN[16]*/ double *default_tcp);

DIANA_API int getLinkState();

/*************************************** version 0.3 ***************************************/

DIANA_API int getTcpForce(/*OUT[6]*/ double *forces);

DIANA_API int getJointForce(/*OUT[7]*/ double *forces);

DIANA_API bool isCollision();

/*************************************** DH参数标定 ***************************************/

DIANA_API int initDHCali(/*IN[3*nrSets]*/ double *tcpMeas, /*IN[7*nrSets]*/ double *jntPosMeas,
                         /*[IN]*/ unsigned int nrSets);

DIANA_API int getDHCaliResult(/*OUT[28]*/ double *rDH, /*OUT[6]*/ double *wRT, /*OUT[3]*/ double *tRT,
                              /*OUT[2]*/ double *confid);

DIANA_API int setDH(/*IN[7]*/ double *a, /*IN[7]*/ double *alpha, /*IN[7]*/ double *d, /*IN[7]*/ double *theta);

DIANA_API int setWrd2BasRT(/*IN[6]*/ double *RTw2b);

DIANA_API int setFla2TcpRT(/*IN[3]*/ double *RTf2t);

/*************************************** version 0.4 ***************************************/

DIANA_API char getRobotState();

DIANA_API int resume();

DIANA_API int setJointCollision(/*IN[7]*/ double *collision);

DIANA_API int setCartCollision(/*IN[6]*/ double *collision);

/*************************************** version 0.5 ***************************************/

DIANA_API int enterForceMode(/*[IN]*/ int intFrameType,
                             /*IN[16]*/ double *dblFrameMatrix,
                             /*IN[3]*/ double *dblForceDirection,
                             /*[IN]*/ double dblForceValue,
                             /*[IN]*/ double dblMaxApproachVelocity,
                             /*[IN]*/ double dblMaxAllowTcpOffset);

DIANA_API int leaveForceMode(/*IN*/ int intExitMode);

DIANA_API int setDefaultActiveTcpPose(/*IN[6]*/ double *arrPose);

DIANA_API int setResultantCollision(/*IN*/ double force);  //合力

DIANA_API int setJointImpedance(/*IN[7]*/ double *arrStiff, /*IN[7]*/ double *arrDamp);

DIANA_API int getJointImpedance(/*OUT[7]*/ double *arrStiff, /*OUT[7]*/ double *arrDamp);

DIANA_API int setCartImpedance(/*IN[6]*/ double *arrStiff, /*IN[6]*/ double *arrDamp);

DIANA_API int getCartImpedance(/*OUT[6]*/ double *arrStiff, /*OUT[6]*/ double *arrDamp);

/*************************************** version 2.2 ( version 0.6 ) ***************************************/

DIANA_API int zeroSpaceFreeDriving(/*IN*/ bool enable);

DIANA_API int createPath(/*IN*/ int type, /*OUT*/ unsigned int &id_path); /*1 for moveJ, 2 for moveL*/

DIANA_API int addMoveL(/*IN*/ unsigned int id_path, /*IN[7]*/ double *joints, /*IN*/ double vel, /*IN*/ double acc,
                       /*IN*/ double blendradius);

DIANA_API int addMoveJ(/*IN*/ unsigned int id_path, /*IN[7]*/ double *joints, /*IN*/ double vel_percent,
                       /*IN*/ double acc_percent, /*IN*/ double blendradius_percent);

DIANA_API int runPath(/*IN*/ unsigned int id_path);

DIANA_API int destroyPath(/*IN*/ unsigned int id_path);

/*************************************** version 0.7 ***************************************/

DIANA_API int rpy2Axis(/*IN[3]*/ double *arr);

DIANA_API int axis2RPY(/*IN[3]*/ double *arr);

DIANA_API int homogeneous2Pose(/*IN[16]*/ double *matrix, /*OUT[6]*/ double *pose);

DIANA_API int pose2Homogeneous(/*IN[6]*/ double *pose, /*OUT[16]*/ double *matrix);

DIANA_API int enableTorqueReceiver(/*bool*/ bool bEnable);

#ifdef _API_SUPPORT_V2_
DIANA_API int sendTorque_rt(/*IN[7]*/ double *torque, /*IN*/ double t);
#endif

/*************************************** version 0.8 ***************************************/

DIANA_API int enableCollisionDetection(bool bEnable);

DIANA_API int setActiveTcpPayload(/*IN[10]*/ double *payload);

DIANA_API int servoJ(/*IN[7]*/ double *joints, /*IN*/ double time, /*IN*/ double look_ahead_time, /*IN*/ double gain);

DIANA_API int servoL(/*IN[6]*/ double *pose, /*IN*/ double time, /*IN*/ double look_ahead_time, /*IN*/ double gain,
                     /*IN*/ double scale, /*IN*/ double *active_tcp = nullptr);

DIANA_API int servoJ_ex(/*IN[7]*/ double *joints, /*IN*/ double time, /*IN*/ double look_ahead_time, /*IN*/ double gain,
                        /*IN*/ bool realiable);

DIANA_API int servoL_ex(/*IN[6]*/ double *pose, /*IN*/ double time, /*IN*/ double look_ahead_time, /*IN*/ double gain,
                        /*IN*/ double scale, /*IN*/ bool realiable, /*IN*/ double *active_tcp = nullptr);

DIANA_API int speedJ_ex(/*IN[7]*/ double *speed, /*IN*/ double a, /*option*/ double t, /*IN*/ bool realiable);

DIANA_API int speedL_ex(/*IN[6]*/ double *speed, /*IN[2]*/ double *a, /*option*/ double t, /*IN*/ bool realiable,
                        /*IN*/ double *active_tcp = nullptr);

/*************************************** version 0.9 ***************************************/

DIANA_API int dumpToUDisk();

DIANA_API int inverse_ext(/*IN[7]*/ double *ref_joints, /*IN[6]*/ double *pose, /*OUT[7]*/ double *joints,
                          /*IN*/ double *active_tcp = nullptr);

DIANA_API int getJointLinkPos(/*OUT[7]*/ double *joints);

enum COMPLEX_PATH_TYPE {
    NORMAL_JOINT_PATH = 0,
    MOVEP_JOINT_PATH = 1,
    NORMAL_POSE_PATH = 2,
    MOVEP_POSE_PATH = 3,
};

DIANA_API int createComplexPath(/*IN*/ int complex_path_type, /*OUT*/ unsigned int &complex_path_id);

DIANA_API int addMoveLByTarget(/*IN*/ unsigned int complex_path_id, /*IN[7]*/ double *target_joints, /*IN*/ double vel,
                               /*IN*/ double acc, /*IN*/ double blendradius);

DIANA_API int addMoveLByPose(/*IN*/ unsigned int complex_path_id, /*IN[6]*/ double *target_pose, /*IN*/ double vel,
                             /*IN*/ double acc, /*IN*/ double blendradius);

DIANA_API int addMoveJByTarget(/*IN*/ unsigned int complex_path_id, /*IN[7]*/ double *target_joints,
                               /*IN*/ double vel_percent, /*IN*/ double acc_percent, /*IN*/ double blendradius_percent);

DIANA_API int addMoveJByPose(/*IN*/ unsigned int complex_path_id, /*IN[6]*/ double *target_pose,
                             /*IN*/ double vel_percent, /*IN*/ double acc_percent, /*IN*/ double blendradius_percent);

DIANA_API int addMoveCByTarget(/*IN*/ unsigned int complex_path_id, /*IN[7]*/ double *pass_joints,
                               /*IN[7]*/ double *target_joints, /*IN*/ double vel, /*IN*/ double acc,
                               /*IN*/ double blendradius, /*IN*/ bool ignore_rotation);

DIANA_API int addMoveCByPose(/*IN*/ unsigned int complex_path_id, /*IN[6]*/ double *pass_pose,
                             /*IN[6]*/ double *target_pose, /*IN*/ double vel, /*IN*/ double acc,
                             /*IN*/ double blendradius, /*IN*/ bool ignore_rotation);

DIANA_API int runComplexPath(/*IN*/ unsigned int complex_path_id);

DIANA_API int destroyComplexPath(/*IN*/ unsigned int complex_path_id);

DIANA_API int saveEnvironment();

DIANA_API int moveTcpBaseLinearRelative(double *pose, double v, double a);

/*************************************** version 0.10 ***************************************/

DIANA_API int dumpToUDiskEx(/*IN*/ double timeout_second);

DIANA_API int enterForceMode_ex(/*IN[3]*/ double *dblForceDirection, /*[IN]*/ double dblForceValue,
                                /*[IN]*/ double dblMaxApproachVelocity, /*[IN]*/ double dblMaxAllowTcpOffset,
                                /*IN[6]*/ double *dblActiveTcp = nullptr);

DIANA_API int readDI(/*IN*/ const char *groupName, /*IN*/ const char *IOName, /*OUT*/ int &value);
DIANA_API int readAI(/*IN*/ const char *groupName, /*IN*/ const char *IOName, /*OUT*/ int &mode, /*OUT*/ double &value);
DIANA_API int setAIMode(/*IN*/ const char *groupName, /*IN*/ const char *IOName, /*IN*/ int mode);
DIANA_API int writeDO(/*IN*/ const char *groupName, /*IN*/ const char *IOName, /*IN*/ int value);
DIANA_API int writeAO(/*IN*/ const char *groupName, /*IN*/ const char *IOName, /*IN*/ int mode, /*IN*/ double value);
DIANA_API int readBusCurrent(/*OUT*/ double &current);
DIANA_API int readBusVoltage(/*OUT*/ double &voltage);
/*************************************** version 1.1***************************************/

DIANA_API int getDH(/*OUT[7]*/ double *aDH, /*OUT[7]*/ double *alphaDH, /*OUT[7]*/ double *dDH,
                    /*OUT[7]*/ double *thetaDH);

DIANA_API int getOriginalJointTorque(/*OUT[7]*/ double *torques);

DIANA_API int getJacobiMatrix(/*OUT[42]*/ double *matrix_jacobi);

/*************************************** version 1.2***************************************/

DIANA_API int resetDH();

/*************************************** version 1.3***************************************/

DIANA_API int runProgram(const char *programName);
DIANA_API int stopProgram(const char *programName);
DIANA_API int getVariableValue(const char *variableName, double &value);
DIANA_API int setVariableValue(const char *variableName, const double &value);
DIANA_API int isTaskRunning(const char *variableName);
DIANA_API int pauseProgram();
DIANA_API int resumeProgram();
DIANA_API int stopAllProgram();
DIANA_API int isAnyTaskRunning();

/*************************************** version 1.4***************************************/

DIANA_API int cleanErrorInfo();
DIANA_API int setCollisionLevel(int level);
//以下mapping相关函数暂不支持python版本
DIANA_API int mappingInt8Variant(const char *variantName, int index);
DIANA_API int mappingDoubleVariant(const char *variantName, int index);
DIANA_API int mappingInt8IO(const char *groupName, const char *name, int index);
DIANA_API int mappingDoubleIO(const char *groupName, const char *name, int index);
DIANA_API int setMappingAddress(double *dblAddress, int doubleItemCount, int8_t *int8Address, int int8ItemCount);
DIANA_API int lockMappingAddress();
DIANA_API int unlockMappingAddress();

/*************************************** version 1.5***************************************/

DIANA_API int getJointCount();

DIANA_API int getWayPoint(const char *waypointName, double *dblTcppos, double *dblJoints);
DIANA_API int setWayPoint(const char *waypointName, double *dblTcppos, double *dblJoints);
DIANA_API int addWayPoint(const char *waypointName, double *dblTcppos, double *dblJoints);
DIANA_API int deleteWayPoint(const char *waypointName);

DIANA_API int setExternalAppendTorCutoffFreq(/*IN*/ double dblFreq);

/*************************************** 以下是测试用函数 ***************************************/
#ifdef PRESS_TEST

DIANA_API void getTestData(int &fail, int &timeout, int &success);
#endif

#pragma pack()
