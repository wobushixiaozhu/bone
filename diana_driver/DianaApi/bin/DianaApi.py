#!usr/bin/python3


from ctypes import *
import platform
import os
import time
import sys
from enum import Enum
import math

errorCodeMessage = {
    0: 'NO_ERROR_CODE',
    -1000: 'ERROR_CODE_SOCKET_OTHER_ERROR',
    -1001: 'ERROR_CODE_WSASTART_FAIL',
    -1002: 'ERROR_CODE_CREATE_SOCKET_FAIL',
    -1003: 'ERROR_CODE_BIND_PORT_FAIL',
    -1004: 'ERROR_CODE_SOCKET_READ_FAIL',
    -1005: 'ERROR_CODE_SOCKET_TIMEOUT',
    -1006: 'ERROR_CODE_RECVFROM_FAIL',
    -1007: 'ERROR_CODE_SENDTO_FAIL',
    -1008: 'ERROR_CODE_LOST_HEARTBEAT',
    -1009: 'ERROR_CODE_LOST_ROBOTSTATE',
    -1010: 'ERROR_CODE_GET_DH_FAILED',
    -1011: 'ERROR_CODE_RELEASE_BRAKE_FAILED',
    -1012: 'ERROR_CODE_HOLD_BRAKE_FAILED',
    -2001: 'ERROR_CODE_JOINT_REGIST_ERROR',
    -2101: 'ERROR_CODE_COMMUNICATE_ERROR',
    -2201: 'ERROR_CODE_CALLING_CONFLICT_ERR',
    -2202: 'ERROR_CODE_COLLISION_ERROR',
    -2203: 'ERROR_CODE_NOT_FOLLOW_POSITION_CMD',
    -2204: 'ERROR_CODE_NOT_FOLLOW_TCP_CMD',
    -2205: 'ERROR_CODE_NOT_ALL_AT_OP_STATE',
    -2206: 'ERROR_CODE_OUT_RANGE_FEEDBACK',
    -2207: 'ERROR_CODE_EMERGENCY_STOP',
    -2208: 'ERROR_CODE_NO_INIT_PARAMETER',
    -2209: 'ECODE_NOT_MATCH_LOAD',
    -2301: 'ERROR_CODE_PLAN_ERROR',
    -2302: 'ERROR_CODE_INTERPOLATE_ERROR',
    -2303: 'ERROR_CODE_INTERPOLATE_TORQUE_ERROR',
    -2304: 'ERROR_CODE_SINGULAR_VALUE_ERROR',
    -3001: 'ERROR_CODE_RESOURCE_UNAVAILABLE',
    -3002: 'ERROR_CODE_DUMP_LOG_TIMEOUT',
    -3003: 'ERROR_CODE_DUMP_LOG_FAILED',
    -3004: 'RESET_DH_FAILED',
}

scriptLibraryNames = {
	'Linux': 'libDianaApi.so',
	'Windows': 'DianaApi.dll'
}

scriptLibraryDir = os.path.split(os.path.realpath(__file__))[0]
scriptLibraryPath = os.path.join(
    scriptLibraryDir, scriptLibraryNames[platform.system()])
api_mod =CDLL(scriptLibraryPath)
#Enumration starts
#add Enumration here


class tcp_direction_e(Enum):
    T_MOVE_X_UP = 0
    T_MOVE_X_DOWN = 1
    T_MOVE_Y_UP = 2
    T_MOVE_Y_DOWN = 3
    T_MOVE_Z_UP = 4
    T_MOVE_Z_DOWN = 5


class mode_e(Enum):
    T_MODE_INVALID = -1
    T_MODE_POSITION = 0
    T_MODE_JOINT_IMPEDANCE = 2
    T_MODE_CART_IMPEDANCE = 3


class joint_direction_e(Enum):
    T_MOVE_UP = 0
    T_MOVE_DOWN = 1


class complex_path_type(Enum):
    NORMAL_JOINT_PATH = 0
    MOVEP_JOINT_PATH = 1
    NORMAL_POSE_PATH = 2
    MOVEP_POSE_PATH = 3

#Enumeration ends

#Structure starts
#add Structure here


class DIANA_JOINTS_POSITION(Structure):
    pass


DIANA_JOINTS_POSITION._fields_ = [
    ('values', c_double*7)
]


class DIANA_JOINTS_FORCE(Structure):
    pass


DIANA_JOINTS_FORCE._fields_ = [
    ('values', c_double*7)
]


class DIANA_TCP_POSE(Structure):
    pass


DIANA_TCP_POSE._fields_ = [
    ('values', c_double * 6)
]


class DIANA_JOINTS_SPEED(Structure):
    pass


DIANA_JOINTS_SPEED._fields_ = [
    ('values', c_double*7)
]


class DIANA_JOINTS_SPEED_L(Structure):
    pass


DIANA_JOINTS_SPEED_L._fields_ = [
    ('values', c_double*6)
]


class DIANA_JOINTS_ACC(Structure):
    pass


DIANA_JOINTS_ACC._fields_ = [
    ('values', c_double*2)
]


class DIANA_FRAME_MATRIX(Structure):
    pass


DIANA_FRAME_MATRIX._fields_ = [
    ('values', c_double * 16)
]


class DIANA_FORCE_DIRECTION(Structure):
    pass


DIANA_FORCE_DIRECTION._fields_ = [
    ('values', c_double * 3)
]


class SRV_NET_ST(Structure):
    pass


SRV_NET_ST._fields_ = [
    ('ipAddress', c_char * 32),
    ('SLocHeartbeatPortrvIp', c_ushort),
    ('LocRobotStatePort', c_ushort),
    ('LocSrvPort', c_ushort)
]


class DIANA_JOINT_ANGULAR_VEL(Structure):
    pass


DIANA_JOINT_ANGULAR_VEL._fields_ = [
    ('values', c_double * 7)
]


class DIANA_JOINT_CURRENT(Structure):
    pass


DIANA_JOINT_CURRENT._fields_ = [
    ('values', c_double * 7)
]


class DIANA_JOINT_TORQUE(Structure):
    pass


DIANA_JOINT_TORQUE._fields_ = [
    ('values', c_double * 7)
]


class DIANA_TCP_FORCE(Structure):
    pass


DIANA_TCP_FORCE._fields_ = [
    ('values', c_double * 6)
]


class DIANA_DEFAULT_TCP(Structure):
    pass


DIANA_DEFAULT_TCP._fields_ = [
    ('values', c_double * 16)
]


class DIANA_TCP_VECTOR(Structure):
    pass


DIANA_TCP_VECTOR._fields_ = [
    ('values', c_double * 6)
]


class DIANA_RPY_VECTOR(Structure):
    pass


DIANA_RPY_VECTOR._fields_ = [
    ('values', c_double * 3)
]


class DIANA_AXIS_VECTOR(Structure):
    pass


DIANA_AXIS_VECTOR._fields_ = [
    ('values', c_double * 3)
]


class DIANA_JOINT_COLLISION(Structure):
    pass


DIANA_JOINT_COLLISION._fields_ = [
    ('values', c_double * 7)
]


class DIANA_CART_COLLISION(Structure):
    pass


DIANA_CART_COLLISION._fields_ = [
    ('values', c_double * 6)
]


class DIANA_JOINT_STIFF(Structure):
    pass


DIANA_JOINT_STIFF._fields_ = [
    ('values', c_double * 7)
]


class DIANA_JOINT_DAMP(Structure):
    pass


DIANA_JOINT_DAMP._fields_ = [
    ('values', c_double * 7)
]


class DIANA_CART_STIFF(Structure):
    pass


DIANA_CART_STIFF._fields_ = [
    ('values', c_double * 6)
]


class DIANA_CART_DAMP(Structure):
    pass


DIANA_CART_DAMP._fields_ = [
    ('values', c_double * 6)
]


class DIANA_TCP_PAYLOAD(Structure):
    pass


DIANA_TCP_PAYLOAD._fields_ = [
    ('values', c_double * 10)
]


class StrTrajectoryState(Structure):
    _pack_ = 1
    _fields_ = [
        ('taskId', c_int),
        ('segCount', c_int),
        ('segIndex', c_int),
        ('errorCode', c_int),
        ('isPaused', c_int),
        ('isFreeDriving', c_int),
        ('isZeroSpaceFreeDriving', c_int)
    ]


class StrErrorInfo(Structure):
    _pack_ = 1
    _fields_ = [
        ('errorId', c_int),
        ('errorType', c_int),
        ('errorCode', c_int),
        ('errorMsg', c_char * 64)
    ]


class StrRobotStateInfo(Structure):
    _pack_ = 1
    _fields_ = [
        ('jointPos', c_double * 7),
        ('jointAngularVel', c_double * 7),
        ('jointCurrent', c_double * 7),
        ('jointTorque', c_double * 7),
        ('tcpPos', c_double * 6),
        ('tcpExternalForce', c_double),
        ('bCollision', c_bool),
        ('bTcpForceValid', c_bool),
        ('tcpForce', c_double * 6),
        ('jointForce', c_double * 7),
        ('trajState', StrTrajectoryState),
        ('StrErrorInfo', StrErrorInfo)
    ]

class DIANA_DH_A(Structure):
        pass


DIANA_DH_A._fields_ = [
    ('values', c_double * 7)
]

class DIANA_DH_Alpha(Structure):
        pass


DIANA_DH_Alpha._fields_ = [
    ('values', c_double * 7)
]

class DIANA_DH_D(Structure):
        pass


DIANA_DH_D._fields_ = [
    ('values', c_double * 7)
]

class DIANA_DH_Theta(Structure):
        pass


DIANA_DH_Theta._fields_ = [
    ('values', c_double * 7)
]


class DIANA_JACOBI_MATRIX(Structure):
    pass


DIANA_JACOBI_MATRIX._fields_ = [
    ('values', c_double * 42)
]

class DIANA_DIGTAL_VALUE_ALL(Structure):
    pass

DIANA_DIGTAL_VALUE_ALL._fields_ = [
    ('value', c_int * 8)
]

class DIANA_ANALOG_VALUE_ALL(Structure):
    pass

DIANA_ANALOG_VALUE_ALL._fields_ = [
    ('value', c_double * 2)
]

class DIANA_ANALOG_MODE(Structure):
    pass

DIANA_ANALOG_MODE._fields_ = [
    ('value', c_int * 2)
]

class DIANA_DH_TCP_MEAS(Structure):
    pass

DIANA_DH_TCP_MEAS._fields_ = [
    ('value', c_double * 3)
]

class DIANA_DH_JOINT_POS__MEAS(Structure):
    pass

DIANA_DH_JOINT_POS__MEAS._fields_ = [
    ('value', c_double * 7)
]

class DIANA_DH_R(Structure):
    pass

DIANA_DH_R._fields_ = [
    ('value', c_double * 28)
]

class DIANA_DH_WRT(Structure):
    pass

DIANA_DH_WRT._fields_ = [
    ('value', c_double * 6)
]

class DIANA_DH_TRT(Structure):
    pass

DIANA_DH_TRT._fields_ = [
    ('value', c_double * 3)
]

class DIANA_DH_CONFID(Structure):
    pass

DIANA_DH_CONFID._fields_ = [
    ('value', c_double * 2)
]

class DIANA_DH_RTw2b(Structure):
    pass

DIANA_DH_RTw2b._fields_ = [
    ('value', c_double * 6)
]

class DIANA_DH_RTf2t(Structure):
    pass

DIANA_DH_RTf2t._fields_ = [
    ('value', c_double * 3)
]

class DIANA_KP(Structure):
    pass

DIANA_KP._fields_ = [
    ('value', c_double * 7)
]

class DIANA_KD(Structure):
    pass

DIANA_KD._fields_ = [
    ('value', c_double * 7)
]

class DIANA_ACTIVE_TCP(Structure):
    pass

DIANA_ACTIVE_TCP._fields_ = [
    ('value', c_double * 6)
]


# Structure ends


#Callback function starts
#add callback function here
FNCERRORCALLBACK = CFUNCTYPE(None, c_int)

FNCSTATECALLBACK = CFUNCTYPE(None, POINTER(StrRobotStateInfo))
#Callback function ends


def message(ret):
    if(ret >= 0):
        return True
    else:
        errorcode = getLastError()
        errorMessage = errorCodeMessage.get(errorcode)
        callerName = 'function ' + sys._getframe().f_back.f_code.co_name + ' fails,'
        if errorMessage == None:
            print(callerName + 'Error Code ' + str(errorcode))
        else:
            print(callerName + errorMessage)
        return False

def wait_move():
    time.sleep(0.02)
    while True:
        state = getRobotState()
        if state != 0:
            break
        else:
            time.sleep(0.001)
    stop()

def forward(joints, pose):
    dianaJointsPosition = DIANA_JOINTS_POSITION()
    dianaJointsPosition.values = (c_double * 7)()
    for index in range(0, len(joints)):
        dianaJointsPosition.values[index] = joints[index]
    dianaTcpPose = DIANA_TCP_POSE()
    dianaTcpPose.values = (c_double * 6)()
    active_tcp = c_void_p(0)
    api_mod.forward.argtypes = [
        POINTER(DIANA_JOINTS_POSITION), POINTER(DIANA_TCP_POSE), c_void_p]
    api_mod.forward.restype = c_int
    ret = api_mod.forward(dianaJointsPosition, byref(dianaTcpPose), active_tcp)
    for index in range(0, len(dianaTcpPose.values)):
        pose[index] = dianaTcpPose.values[index]
    return message(ret)


def inverse(pose, joints):
    dianaTcpPose = DIANA_TCP_POSE()
    dianaTcpPose.values = (c_double * 6)()
    for index in range(0, len(dianaTcpPose.values)):
        dianaTcpPose.values[index] = pose[index]
    dianaJointsPosition = DIANA_JOINTS_POSITION()
    dianaJointsPosition.values = (c_double * 7)()
    active_tcp = c_void_p(0)
    api_mod.inverse.argtypes = [POINTER(DIANA_TCP_POSE), POINTER(
        DIANA_JOINTS_POSITION),  c_void_p]
    api_mod.inverse.restype = c_int
    ret = api_mod.inverse(dianaTcpPose, byref(dianaJointsPosition), active_tcp)
    for index in range(0, len(joints)):
        joints[index] = dianaJointsPosition.values[index]
    return message(ret)

def speedJ(speed, acc, t=0.0):
    dianaJointsSpeed = DIANA_JOINTS_SPEED()
    dianaJointsSpeed.values = (c_double * 7)()
    for index in range(0, len(speed)):
        dianaJointsSpeed.values[index] = speed[index]
    api_mod.speedJ.argtypes = [POINTER(DIANA_JOINTS_SPEED), c_double, c_double]
    api_mod.speedJ.restype = c_int
    ret = api_mod.speedJ(dianaJointsSpeed, acc, t)
    return message(ret)


def speedL(speed, acc, t=0.0):
    dianaJointsSpeed = DIANA_JOINTS_SPEED_L()
    dianaJointsSpeed.values = (c_double * 6)()
    for index in range(0, len(dianaJointsSpeed.values)):
        dianaJointsSpeed.values[index] = speed[index]
    dianaJointsAcc = DIANA_JOINTS_ACC()
    dianaJointsAcc.values = (c_double * 2)()
    for index in range(0, len(dianaJointsAcc.values)):
        dianaJointsAcc.values[index] = acc[index]
    active_tcp = c_void_p(0)
    api_mod.speedL.argtypes = [POINTER(DIANA_JOINTS_SPEED_L), POINTER(
        DIANA_JOINTS_ACC), c_double, c_void_p]
    api_mod.speedL.restype = c_int
    ret = api_mod.speedL(dianaJointsSpeed, dianaJointsAcc, t, active_tcp)
    return message(ret)

def freeDriving(enable):
    api_mod.freeDriving.argtypes = [c_bool]
    api_mod.freeDriving.restype = c_int
    ret = api_mod.freeDriving(enable)
    return message(ret)


def holdBrake():
    api_mod.holdBrake.restype = c_int
    ret = api_mod.holdBrake()
    return message(ret)


def releaseBrake():
    api_mod.releaseBrake.restype = c_int
    ret = api_mod.releaseBrake()
    return message(ret)


def stop():
    api_mod.stop.restype = c_int
    ret = api_mod.stop()
    return message(ret)


def getRobotState():
    api_mod.getRobotState.restype = c_int8
    ret = api_mod.getRobotState()
    return ret


def initSrv(srv_net_st, fnError=None, fnState=None):
    pinfo = SRV_NET_ST()
    strIpAddress = srv_net_st[0]
    pinfo.ipAddress = bytes(strIpAddress.encode('utf-8'))
    pinfo.SLocHeartbeatPortrvIp = srv_net_st[1]
    pinfo.LocRobotStatePort = srv_net_st[2]
    pinfo.LocSrvPort = srv_net_st[3]
    api_mod.initSrv.argtypes = [FNCERRORCALLBACK, FNCSTATECALLBACK, POINTER(SRV_NET_ST)]
    api_mod.initSrv.restype = c_int
    if fnError == None:
        fnError = FNCERRORCALLBACK()
    if fnState == None:
        fnState = FNCSTATECALLBACK()
    ret = api_mod.initSrv(fnError, fnState, byref(pinfo))
    return message(ret)


def destroySrv():
    api_mod.destroySrv.restype = c_int
    ret = api_mod.destroySrv()
    return message(ret)

def setPushPeriod(period):
    api_mod.setPushPeriod.argtypes = [c_int]
    api_mod.setPushPeriod.restype = c_int
    ret = api_mod.setPushPeriod(period)
    return message(ret)

def getJointPos(jointsPosition):
    dianaJointsPosition = DIANA_JOINTS_POSITION()
    dianaJointsPosition.values = (c_double * 7)()
    api_mod.getJointPos.argtypes = [POINTER(DIANA_JOINTS_POSITION)]
    api_mod.getJointPos.restype = c_int
    ret = api_mod.getJointPos(byref(dianaJointsPosition))
    for index in range(0, len(jointsPosition)):
      jointsPosition[index] = dianaJointsPosition.values[index]
    return message(ret)


def getTcpPos(tcpPose):
    dianaTcpPose = DIANA_TCP_POSE()
    dianaTcpPose.values = (c_double * 6)()
    api_mod.getTcpPos.argtypes = [POINTER(DIANA_TCP_POSE)]
    api_mod.getTcpPos.restype = c_int
    ret = api_mod.getTcpPos(byref(dianaTcpPose))
    for index in range(0, len(dianaTcpPose.values)):
      tcpPose[index] = dianaTcpPose.values[index]
    return message(ret)


def getJointAngularVel(jointAngularVel):
    dianaJointAngularVel = DIANA_JOINT_ANGULAR_VEL()
    dianaJointAngularVel.values = (c_double * 7)()
    api_mod.getJointAngularVel.argtypes = [POINTER(DIANA_JOINT_ANGULAR_VEL)]
    api_mod.getJointAngularVel.restype = c_int
    ret = api_mod.getJointAngularVel(byref(dianaJointAngularVel))
    for index in range(0, len(jointAngularVel)):
      jointAngularVel[index] = dianaJointAngularVel.values[index]
    return message(ret)


def getTcpForce(tcpForce):
    dianaTcpForce = DIANA_TCP_FORCE()
    dianaTcpForce.values = (c_double * 6)()
    api_mod.getTcpForce.argtypes = [POINTER(DIANA_TCP_FORCE)]
    api_mod.getTcpForce.restype = c_int
    ret = api_mod.getTcpForce(byref(dianaTcpForce))
    for index in range(0, len(dianaTcpForce.values)):
      tcpForce[index] = dianaTcpForce.values[index]
    return message(ret)


def getJointForce(jointForce):
    dianaJointForce = DIANA_JOINTS_FORCE()
    dianaJointForce.values = (c_double * 7)()
    api_mod.getJointForce.argtypes = [POINTER(DIANA_JOINTS_FORCE)]
    api_mod.getJointForce.restype = c_int
    ret = api_mod.getJointForce(byref(dianaJointForce))
    for index in range(0, len(jointForce)):
      jointForce[index] = dianaJointForce.values[index]
    return message(ret)

def moveJToTarget(joints, v, a):
    dianaJointPos = DIANA_JOINTS_POSITION()
    dianaJointPos.values = (c_double * 7)()
    for index in range(0, len(joints)):
        dianaJointPos.values[index] = joints[index]
    api_mod.moveJToTarget.argtypes = [
        POINTER(DIANA_JOINTS_POSITION), c_double, c_double]
    api_mod.moveJToTarget.restype = c_int
    ret = api_mod.moveJToTarget(dianaJointPos, v, a)
    return message(ret)


def moveJToPose(pose, v, a):
    dianaTcpPose = DIANA_TCP_POSE()
    dianaTcpPose.values = (c_double * 6)()
    active_tcp = c_void_p(0)
    for index in range(0, len(dianaTcpPose.values)):
        dianaTcpPose.values[index] = pose[index]
    api_mod.moveJToPose.argtypes = [
        POINTER(DIANA_TCP_POSE), c_double, c_double, c_void_p]
    api_mod.moveJToPose.restype = c_int
    ret = api_mod.moveJToPose(dianaTcpPose, v, a, active_tcp)
    return message(ret)

def moveLToTarget(joints, v, a):
    dianaJointPos = DIANA_JOINTS_POSITION()
    dianaJointPos.values = (c_double * 7)()
    for index in range(0, len(joints)):
        dianaJointPos.values[index] = joints[index]
    api_mod.moveLToTarget.argtypes = [
        POINTER(DIANA_JOINTS_POSITION), c_double, c_double]
    api_mod.moveLToTarget.restype = c_int
    ret = api_mod.moveLToTarget(dianaJointPos, v, a)
    return message(ret)


def moveLToPose(pose, v, a):
    dianaTcpPose = DIANA_TCP_POSE()
    dianaTcpPose.values = (c_double * 6)()
    active_tcp = c_void_p(0)
    for index in range(0, len(dianaTcpPose.values)):
        dianaTcpPose.values[index] = pose[index]
    api_mod.moveLToPose.argtypes = [
        POINTER(DIANA_TCP_POSE), c_double, c_double, c_void_p]
    api_mod.moveLToPose.restype = c_int
    ret = api_mod.moveLToPose(dianaTcpPose, v, a, active_tcp)
    return message(ret)


def getJointCurrent(jointCurrent):
    dianaJointCurrent = DIANA_JOINT_CURRENT()
    dianaJointCurrent.values = (c_double*7)()
    for index in range(0, len(jointCurrent)):
        dianaJointCurrent.values[index] = jointCurrent[index]
    api_mod.getJointCurrent.argtypes = [POINTER(DIANA_JOINT_CURRENT)]
    api_mod.getJointCurrent.restype = c_int
    ret = api_mod.getJointCurrent(byref(dianaJointCurrent))
    for index in range(0, len(jointCurrent)):
        jointCurrent[index] = dianaJointCurrent.values[index]
    return message(ret)


def getJointTorque(jointTorque):
    dianaJointTorque = DIANA_JOINT_TORQUE()
    dianaJointTorque.values = (c_double*7)()
    for index in range(0, len(jointTorque)):
        dianaJointTorque.values[index] = jointTorque[index]
    api_mod.getJointTorque.argtypes = [POINTER(DIANA_JOINT_TORQUE)]
    api_mod.getJointTorque.restype = c_int
    ret = api_mod.getJointTorque(byref(dianaJointTorque))
    for index in range(0, len(jointTorque)):
        jointTorque[index] = dianaJointTorque.values[index]
    return message(ret)


def setDefaultActiveTcp(default_tcp):
    defaultTcp = DIANA_DEFAULT_TCP()
    defaultTcp.values = (c_double*16)()
    for index in range(0, len(defaultTcp.values)):
        defaultTcp.values[index] = default_tcp[index]
    api_mod.setDefaultActiveTcp.argtypes = [POINTER(DIANA_DEFAULT_TCP)]
    api_mod.setDefaultActiveTcp.restype = c_int
    ret = api_mod.setDefaultActiveTcp(defaultTcp)
    if(ret == 0):
        print("setDefaultActiveTcp succeeds")
        return True
    else:
        print("setDefaultActiveTcp fails")
        return False


##########v2.0##########################


def moveTCP(d, v, a):
    active_tcp = c_void_p(0)
    api_mod.moveTCP.argtypes = [c_int, c_double, c_double, c_void_p]
    api_mod.moveTCP.restype = c_int
    ret = api_mod.moveTCP(d.value, v, a, active_tcp)
    return message(ret)


def rotationTCP(d, v, a):
    active_tcp = c_void_p(0)
    api_mod.rotationTCP.argtypes = [c_int, c_double, c_double, c_void_p]
    api_mod.rotationTCP.restype = c_int
    ret = api_mod.rotationTCP(d.value, v, a, active_tcp)
    return message(ret)


def moveJoint(d, i, v, a):
    api_mod.moveJoint.argtypes = [c_int, c_int, c_double, c_double]
    api_mod.moveJoint.restype = c_int
    ret = api_mod.moveJoint(d.value, i, v, a)
    return message(ret)


def getTcpExternalForce():
    api_mod.getTcpExternalForce.restype = c_double
    ret = api_mod.getTcpExternalForce()
    return ret


def changeControlMode(m):
    api_mod.changeControlMode.argtypes = [c_int]
    api_mod.changeControlMode.restype = c_int
    ret = api_mod.changeControlMode(m.value)
    return message(ret)


def getLibraryVersion():
    api_mod.getLibraryVersion.restype = c_uint
    ret = api_mod.getLibraryVersion()
    return ret


def formatError(e):
    api_mod.formatError.argtypes = [c_int]
    api_mod.formatError.restype = c_char_p
    ret = api_mod.formatError(e)
    return ret


def getLastError():
    api_mod.getLastError.restype = c_int
    ret = api_mod.getLastError()
    return ret


def setLastError(e):
    api_mod.setLastError.argtypes = [c_int]
    api_mod.setLastError.restype = c_int
    ret = api_mod.setLastError(e)
    return ret


def getLinkState():
    api_mod.getLinkState.restype = c_int
    ret = api_mod.getLinkState()
    return message(ret)


def isCollision():
    api_mod.isCollision.restype = c_bool
    ret = api_mod.isCollision()
    return ret

######################DH参数标定#############################
def initDHCali(tcpMeas,jntPosMeas,nrSets):
    DHtcpMeas = DIANA_DH_TCP_MEAS()
    DHtcpMeas.values = (c_double*3)()
    DHjntPosMeas = DIANA_DH_JOINT_POS__MEAS()
    DHjntPosMeas.values = (c_double*7)()
    for index in range(0,len(DHtcpMeas.values)):
        DHtcpMeas.values[index] = tcpMeas[index]
    for index in range(0,len(DHjntPosMeas.values)):
        DHjntPosMeas.values[index] = jntPosMeas[index]
    api_mod.initDHCali.restype = c_int
    api_mod.initDHCali.argtypes = [ POINTER(DIANA_DH_TCP_MEAS),POINTER(DIANA_DH_JOINT_POS__MEAS),c_uint]
    ret = api_mod.initDHCali(byref(DHtcpMeas), byref(DHjntPosMeas), nrSets)
    return message(ret)

def getDHCaliResult(rDH,wRT,tRT,confid):
    dhR = DIANA_DH_R()
    dhR.values = (c_double*28)()
    rtW = DIANA_DH_WRT()
    rtW.values = (c_double*6)()
    rtT = DIANA_DH_TRT()
    rtT.values = (c_double*3)()
    dhConfid = DIANA_DH_CONFID()
    dhConfid.values = (c_double*2)()
    api_mod.getDHCaliResult.restype = c_int
    api_mod.getDHCaliResult.argtypes = [ POINTER(DIANA_DH_R),POINTER(DIANA_DH_WRT),POINTER(DIANA_DH_TRT),POINTER(DIANA_DH_CONFID)]
    ret = api_mod.getDHCaliResult(byref(dhR), byref(rtW), byref(rtT), byref(dhConfid))
    for index in range(0,len(rDH)):
        rDH[index] = dhR.values[index]
    for index in range(0,len(wRT)):
        wRT[index] = rtW.values[index]
    for index in range(0,len(tRT)):
        tRT[index] = rtT.values[index]
    for index in range(0,len(confid)):
        confid[index] = dhConfid.values[index]
    return message(ret)
#_________________________________________________________________________________________#
def setDH(a,alpha,d,theta):
    arr_aDH = DIANA_DH_A()
    arr_aDH.values = (c_double*7)()
    for index in range(0, len(a)):
        arr_aDH.values[index] = a[index]
    arr_alphaDH = DIANA_DH_Alpha()
    arr_alphaDH.values = (c_double*7)()
    for index in range(0, len(alpha)):
        arr_alphaDH.values[index] = alpha[index]
    arr_dDH = DIANA_DH_D()
    arr_dDH.values = (c_double*7)()
    for index in range(0, len(d)):
        arr_dDH.values[index] = d[index]
    arr_thetaDH = DIANA_DH_Theta()
    arr_thetaDH.values = (c_double*7)()
    for index in range(0, len(theta)):
        arr_thetaDH.values[index] = theta[index]
    api_mod.setDH.restype = c_int
    api_mod.setDH.argtypes = [ POINTER(DIANA_DH_A), 
        POINTER(DIANA_DH_Alpha),POINTER(DIANA_DH_D), POINTER(DIANA_DH_Theta)]
    ret = api_mod.setDH(byref(arr_aDH),byref(arr_alphaDH),byref(arr_dDH),byref(arr_thetaDH))
    return message(ret)

def setWrd2BasRT(RTw2b):
    arr_RTw2b = DIANA_DH_RTw2b()
    arr_RTw2b.values = (c_double*6)()
    for index in range(0, len(RTw2b)):
        arr_RTw2b.values[index] = RTw2b[index]
    api_mod.setWrd2BasRT.restype = c_int
    api_mod.setWrd2BasRT.argtypes = [ POINTER(DIANA_DH_RTw2b)]
    ret = api_mod.setWrd2BasRT(byref(arr_RTw2b))
    return message(ret)

def setFla2TcpRT(RTf2t):
    arr_RTf2t = DIANA_DH_RTf2t()
    arr_RTf2t.values = (c_double*3)()
    for index in range(0, len(RTf2t)):
        arr_RTf2t.values[index] = RTf2t[index]
    api_mod.setFla2TcpRT.restype = c_int
    api_mod.setFla2TcpRT.argtypes = [ POINTER(DIANA_DH_RTf2t)]
    ret = api_mod.setFla2TcpRT(byref(arr_RTf2t))
    return message(ret)
#########################################################

def resume():
    api_mod.resume.restype = c_int
    ret = api_mod.resume()
    return message(ret)


def setJointCollision(collision):
    jointCollision = DIANA_JOINT_COLLISION()
    jointCollision.values = (c_double*7)()
    for index in range(0, len(collision)):
        jointCollision.values[index] = collision[index]
    api_mod.setJointCollision.argtypes = [POINTER(DIANA_JOINT_COLLISION)]
    api_mod.setJointCollision.restype = c_int
    ret = api_mod.setJointCollision(jointCollision)
    return message(ret)


def setCartCollision(collision):
    cartCollision = DIANA_CART_COLLISION()
    cartCollision.values = (c_double*6)()
    for index in range(0, len(cartCollision.values)):
        cartCollision.values[index] = collision[index]
    api_mod.setCartCollision.argtypes = [POINTER(DIANA_CART_COLLISION)]
    api_mod.setCartCollision.restype = c_int
    ret = api_mod.setCartCollision(cartCollision)
    return message(ret)

def enterForceMode(frame_type, frame_matrix, force_direction, force_value, max_approach_velocity, max_allow_tcp_offset):
    dianaFrameMatrix = DIANA_FRAME_MATRIX()
    dianaFrameMatrix.values = (c_double * 16)()
    for index in range(0, len(dianaFrameMatrix.values)):
        dianaFrameMatrix.values[index] = frame_matrix[index]
    dianaForceDirection = DIANA_FORCE_DIRECTION()
    dianaForceDirection.values = (c_double * 3)()
    for index in range(0, len(dianaForceDirection.values)):
        dianaForceDirection.values[index] = force_direction[index]
    api_mod.enterForceMode.argtypes = [c_int, POINTER(DIANA_FRAME_MATRIX), POINTER(
        DIANA_FORCE_DIRECTION), c_double, c_double, c_double]
    api_mod.enterForceMode.restype = c_int
    ret = api_mod.enterForceMode(frame_type, byref(dianaFrameMatrix), byref(
        dianaForceDirection), force_value, max_approach_velocity, max_allow_tcp_offset)
    return message(ret)

def leaveForceMode(mode):
    api_mod.leaveForceMode.argtypes = [c_int]
    api_mod.leaveForceMode.restype = c_int
    ret = api_mod.leaveForceMode(mode)
    return message(ret)

def setDefaultActiveTcpPose(arrPose):
    tcpPose = DIANA_TCP_VECTOR()
    tcpPose.values = (c_double*6)()
    for index in range(0, len(tcpPose.values)):
        tcpPose.values[index] = arrPose[index]
    api_mod.setDefaultActiveTcpPose.argtypes = [POINTER(DIANA_TCP_VECTOR)]
    api_mod.setDefaultActiveTcpPose.restype = c_int
    ret = api_mod.setDefaultActiveTcpPose(tcpPose)
    return message(ret)

def setResultantCollision(force):
    api_mod.setResultantCollision.argtypes = [c_double]
    api_mod.setResultantCollision.restype = c_int
    ret = api_mod.setResultantCollision(force)
    return message(ret)

def setJointImpedance(arrStiff, arrDamp):
    jointStiff = DIANA_JOINT_STIFF()
    jointStiff.values = (c_double*7)()
    for index in range(0, len(arrStiff)):
        jointStiff.values[index] = arrStiff[index]
    jointDamp = DIANA_JOINT_DAMP()
    jointDamp.values = (c_double*7)()
    for index in range(0, len(arrDamp)):
        jointDamp.values[index] = arrDamp[index]
    api_mod.setJointImpedance.argtypes = [
        POINTER(DIANA_JOINT_STIFF), POINTER(DIANA_JOINT_DAMP)]
    api_mod.setJointImpedance.restype = c_int
    ret = api_mod.setJointImpedance(jointStiff, jointDamp)
    return message(ret)


def getJointImpedance(arrStiff, arrDamp):
    jointStiff = DIANA_JOINT_STIFF()
    jointStiff.values = (c_double*7)()
    jointDamp = DIANA_JOINT_DAMP()
    jointDamp.values = (c_double*7)()
    api_mod.getJointImpedance.argtypes = [
        POINTER(DIANA_JOINT_STIFF), POINTER(DIANA_JOINT_DAMP)]
    api_mod.getJointImpedance.restype = c_int
    ret = api_mod.getJointImpedance(byref(jointStiff), byref(jointDamp))
    for index in range(0, len(arrStiff)):
        arrStiff[index] = jointStiff.values[index]
    for index in range(0, len(arrDamp)):
        arrDamp[index] = jointDamp.values[index]
    return message(ret)


def setCartImpedance(arrStiff, arrDamp):
    cartStiff = DIANA_CART_STIFF()
    cartStiff.values = (c_double*6)()
    for index in range(0, len(cartStiff.values)):
        cartStiff.values[index] = arrStiff[index]
    cartDamp = DIANA_CART_DAMP()
    cartDamp.values = (c_double*6)()
    for index in range(0, len(cartDamp.values)):
        cartDamp.values[index] = arrDamp[index]
    api_mod.setCartImpedance.argtypes = [
        POINTER(DIANA_CART_STIFF), POINTER(DIANA_CART_DAMP)]
    api_mod.setCartImpedance.restype = c_int
    ret = api_mod.setCartImpedance(cartStiff, cartDamp)
    return message(ret)


def getCartImpedance(arrStiff, arrDamp):
    cartStiff = DIANA_CART_STIFF()
    cartStiff.values = (c_double*6)()
    cartDamp = DIANA_CART_DAMP()
    cartDamp.values = (c_double*6)()
    api_mod.getCartImpedance.argtypes = [
        POINTER(DIANA_CART_STIFF), POINTER(DIANA_CART_DAMP)]
    api_mod.getCartImpedance.restype = c_int
    ret = api_mod.getCartImpedance(byref(cartStiff), byref(cartDamp))
    for index in range(0, len(cartStiff.values)):
       arrStiff[index] = cartStiff.values[index]
    for index in range(0, len(cartDamp.values)):
        arrDamp[index] = cartDamp.values[index]
    return message(ret)


def zeroSpaceFreeDriving(enable):
    api_mod.zeroSpaceFreeDriving.argtypes = [c_bool]
    api_mod.zeroSpaceFreeDriving.restype = c_int
    ret = api_mod.zeroSpaceFreeDriving(enable)
    return message(ret)

def createPath(id_type):
    id_path = c_uint()
    api_mod.createPath.argtypes = [c_int, POINTER(c_uint)]
    api_mod.createPath.restype = c_int
    ret = api_mod.createPath(id_type, byref(id_path))
    return ret, id_path


def addMoveL(id_path, joints, vel, acc, blendradius):
    dianaJointPos = DIANA_JOINTS_POSITION()
    dianaJointPos.values = (c_double * 7)()
    for index in range(0, len(joints)):
        dianaJointPos.values[index] = joints[index]
    api_mod.addMoveL.argtypes = [c_uint, POINTER(
        DIANA_JOINTS_POSITION), c_double, c_double, c_double]
    api_mod.addMoveL.restype = c_int
    ret = api_mod.addMoveL(id_path, dianaJointPos, vel, acc, blendradius)
    return message(ret)


def addMoveJ(id_path, joints, vel, acc, blendradius):
    dianaJointPos = DIANA_JOINTS_POSITION()
    dianaJointPos.values = (c_double * 7)()
    for index in range(0, len(joints)):
        dianaJointPos.values[index] = joints[index]
    api_mod.addMoveJ.argtypes = [c_uint, POINTER(
        DIANA_JOINTS_POSITION), c_double, c_double, c_double]
    api_mod.addMoveJ.restype = c_int
    ret = api_mod.addMoveJ(id_path, dianaJointPos, vel, acc, blendradius)
    return message(ret)


def runPath(id_path):
    api_mod.runPath.argtypes = [c_uint]
    api_mod.runPath.restype = c_int
    ret = api_mod.runPath(id_path)
    return message(ret)


def destroyPath(id_path):
    api_mod.destroyPath.argtypes = [c_uint]
    api_mod.destroyPath.restype = c_int
    ret = api_mod.destroyPath(id_path)
    return message(ret)

###################### version 0.7#########################

def rpy2Axis(arr):
    _rpy = DIANA_RPY_VECTOR()
    _rpy.values = (c_double*3)()
    for index in range(0, len(_rpy.values)):
        _rpy.values[index] = arr[index]
    api_mod.rpy2Axis.argtypes = [POINTER(DIANA_RPY_VECTOR)]
    api_mod.rpy2Axis.restype = c_int
    ret = api_mod.rpy2Axis(byref(_rpy))
    for index in range(0, len(_rpy.values)):
        arr[index] = _rpy.values[index]
    return message(ret)


def axis2RPY(arr):
    _axis = DIANA_AXIS_VECTOR()
    _axis.values = (c_double*3)()
    for index in range(0, len(_axis.values)):
        _axis.values[index] = arr[index]
    api_mod.axis2RPY.argtypes = [POINTER(DIANA_AXIS_VECTOR)]
    api_mod.axis2RPY.restype = c_int
    ret = api_mod.axis2RPY(byref(_axis))
    for index in range(0, len(_axis.values)):
        arr[index] = _axis.values[index]
    return message(ret)


def homogeneous2Pose(matrix, pose):
    _matrix = DIANA_FRAME_MATRIX()
    _matrix.values = (c_double*16)()
    for index in range(0, len(_matrix.values)):
        _matrix.values[index] = matrix[index]
    _pose = DIANA_TCP_POSE()
    _pose.values = (c_double*6)()
    api_mod.homogeneous2Pose.argtypes = [
        POINTER(DIANA_FRAME_MATRIX), POINTER(DIANA_TCP_POSE)]
    api_mod.homogeneous2Pose.restype = c_int
    ret = api_mod.homogeneous2Pose(_matrix, byref(_pose))
    for index in range(0, len(_pose.values)):
        pose[index] = _pose.values[index]
    return message(ret)


def pose2Homogeneous(pose, matrix):
    _pose = DIANA_TCP_POSE()
    _pose.values = (c_double*6)()
    for index in range(0, len(_pose.values)):
        _pose.values[index] = pose[index]
    _matrix = DIANA_FRAME_MATRIX()
    _matrix.values = (c_double*16)()
    api_mod.pose2Homogeneous.argtypes = [
        POINTER(DIANA_TCP_POSE), POINTER(DIANA_FRAME_MATRIX)]
    api_mod.pose2Homogeneous.restype = c_int
    ret = api_mod.pose2Homogeneous(_pose, byref(_matrix))
    for index in range(0, len(_matrix.values)):
        matrix[index] = _matrix.values[index]
    return message(ret)

def enableTorqueReceiver(bEnable):
    api_mod.enableTorqueReceiver.restype = c_int
    api_mod.enableTorqueReceiver.argtypes = [ c_bool]
    ret = api_mod.enableTorqueReceiver(bEnable)
    return message(ret)

def sendTorque_rt(torque,t):
    jnt_torque = DIANA_JOINT_TORQUE()
    jnt_torque.values = (c_double*7)()
    for index in range(0, len(torque)):
        jnt_torque.values[index] = torque[index]
    api_mod.sendTorque_rt.restype = c_int
    api_mod.sendTorque_rt.argtypes = [ POINTER(DIANA_JOINT_TORQUE),c_double]
    ret = api_mod.sendTorque_rt(byref(jnt_torque),t)
    return message(ret)

###################### version 0.8#########################

def enableCollisionDetection(enable):
    api_mod.enableCollisionDetection.argtypes = [c_bool]
    api_mod.enableCollisionDetection.restype = c_int
    ret = api_mod.enableCollisionDetection(enable)
    return message(ret)

def setActiveTcpPayload(payload):
    tcpPayload = DIANA_TCP_PAYLOAD()
    tcpPayload.values = (c_double*10)()
    for index in range(0, len(tcpPayload.values)):
        tcpPayload.values[index] = payload[index]
    api_mod.setActiveTcpPayload.argtypes = [POINTER(DIANA_TCP_PAYLOAD)]
    api_mod.setActiveTcpPayload.restype = c_int
    ret = api_mod.setActiveTcpPayload(tcpPayload)
    return message(ret)

def servoJ(joints_pos, t=0.01, ah_t=0.01, gain=300):
    dianaJointsPosition = DIANA_JOINTS_POSITION()
    dianaJointsPosition.values = (c_double * 7)()
    for index in range(0, len(joints_pos)):
        dianaJointsPosition.values[index] = joints_pos[index]
    api_mod.servoJ.argtypes = [
        POINTER(DIANA_JOINTS_POSITION), c_double, c_double, c_double]
    api_mod.servoJ.restype = c_int
    ret = api_mod.servoJ(dianaJointsPosition, t, ah_t, gain)
    return message(ret)


def servoL(tcp_pose, t=0.01, ah_t=0.01, gain=300, scale=1000):
    dianaTcpPose = DIANA_TCP_POSE()
    dianaTcpPose.values = (c_double * 6)()
    for index in range(0, len(dianaTcpPose.values)):
        dianaTcpPose.values[index] = tcp_pose[index]
    active_tcp = c_void_p(0)
    api_mod.servoL.argtypes = [
        POINTER(DIANA_TCP_POSE), c_double, c_double, c_double, c_double, c_void_p]
    api_mod.servoL.restype = c_int
    ret = api_mod.servoL(dianaTcpPose, t, ah_t, gain, scale, active_tcp)
    return message(ret)

def servoJ_ex(joints_pos, t=0.01, ah_t=0.01, gain=300, realiable=False):
    dianaJointsPosition = DIANA_JOINTS_POSITION()
    dianaJointsPosition.values = (c_double * 7)()
    for index in range(0, len(joints_pos)):
        dianaJointsPosition.values[index] = joints_pos[index]
    api_mod.servoJ_ex.argtypes = [
        POINTER(DIANA_JOINTS_POSITION), c_double, c_double, c_double, c_bool]
    api_mod.servoJ_ex.restype = c_int
    ret = api_mod.servoJ_ex(dianaJointsPosition, t, ah_t, gain, realiable)
    return message(ret)


def servoL_ex(tcp_pose, t=0.01, ah_t=0.01, gain=300, scale=1000, realiable=False):
    dianaTcpPose = DIANA_TCP_POSE()
    dianaTcpPose.values = (c_double * 6)()
    for index in range(0, len(dianaTcpPose.values)):
        dianaTcpPose.values[index] = tcp_pose[index]
    active_tcp = c_void_p(0)
    api_mod.servoL_ex.argtypes = [POINTER(
        DIANA_TCP_POSE), c_double, c_double, c_double, c_double, c_void_p, c_bool]
    api_mod.servoL_ex.restype = c_int
    ret = api_mod.servoL_ex(dianaTcpPose, t, ah_t, gain,
                         scale, active_tcp, realiable)
    return message(ret)


def speedJ_ex(speed, acc, t=0.0, realiable=False):
    dianaJointsSpeed = DIANA_JOINTS_SPEED()
    dianaJointsSpeed.values = (c_double * 7)()
    for index in range(0, len(speed)):
        dianaJointsSpeed.values[index] = speed[index]
    api_mod.speedJ_ex.argtypes = [
        POINTER(DIANA_JOINTS_SPEED), c_double, c_double, c_bool]
    api_mod.speedJ_ex.restype = c_int
    ret = api_mod.speedJ_ex(dianaJointsSpeed, acc, t, realiable)
    return message(ret)


def speedL_ex(speed, acc, t=0.0, realiable=False):
    dianaJointsSpeed = DIANA_JOINTS_SPEED_L()
    dianaJointsSpeed.values = (c_double * 6)()
    for index in range(0, len(dianaJointsSpeed.values)):
        dianaJointsSpeed.values[index] = speed[index]
    dianaJointsAcc = DIANA_JOINTS_ACC()
    dianaJointsAcc.values = (c_double * 2)()
    for index in range(0, len(dianaJointsAcc.values)):
        dianaJointsAcc.values[index] = acc[index]
    active_tcp = c_void_p(0)
    api_mod.speedL_ex.argtypes = [POINTER(DIANA_JOINTS_SPEED_L), POINTER(
        DIANA_JOINTS_ACC), c_double, c_void_p, c_bool]
    api_mod.speedL_ex.restype = c_int
    ret = api_mod.speedL_ex(dianaJointsSpeed, dianaJointsAcc,
                         t, active_tcp, realiable)
    return message(ret)

###################### version 0.9#########################

def dumpToUDisk():
    api_mod.dumpToUDisk.restype = c_int
    ret = api_mod.dumpToUDisk()
    return message(ret)

def inverse_ext(ref_joints, pose, joints):
    refJointsPosition = DIANA_JOINTS_POSITION()
    refJointsPosition.values = (c_double * 7)()
    for index in range(0, len(ref_joints)):
        refJointsPosition.values[index] = ref_joints[index]
    dianaTcpPose = DIANA_TCP_POSE()
    dianaTcpPose.values = (c_double * 6)()
    for index in range(0, len(dianaTcpPose.values)):
        dianaTcpPose.values[index] = pose[index]
    dianaJointsPosition = DIANA_JOINTS_POSITION()
    dianaJointsPosition.values = (c_double * 7)()
    active_tcp = c_void_p(0)
    api_mod.inverse_ext.argtypes = [POINTER(DIANA_JOINTS_POSITION), POINTER(DIANA_TCP_POSE), POINTER(
        DIANA_JOINTS_POSITION),  c_void_p]
    api_mod.inverse_ext.restype = c_int
    ret = api_mod.inverse_ext(refJointsPosition, dianaTcpPose,
                           byref(dianaJointsPosition), active_tcp)
    for index in range(0, len(joints)):
        joints[index] = dianaJointsPosition.values[index]
    return message(ret)


def getJointLinkPos(joints):
    dianaJointPos = DIANA_JOINTS_POSITION()
    dianaJointPos.values = (c_double*7)()
    for index in range(0, len(joints)):
        dianaJointPos.values[index] = joints[index]
    api_mod.getJointLinkPos.argtypes = [POINTER(DIANA_JOINTS_POSITION)]
    api_mod.getJointLinkPos.restype = c_int
    ret = api_mod.getJointLinkPos(byref(dianaJointPos))
    for index in range(0, len(joints)):
        joints[index] = dianaJointPos.values[index]
    return message(ret)


def createComplexPath(path_type):
    complex_path_id = c_uint()
    api_mod.createComplexPath.argtypes = [c_int, POINTER(c_uint)]
    api_mod.createComplexPath.restype = c_int
    ret = api_mod.createComplexPath(path_type.value, byref(complex_path_id))
    return ret, complex_path_id


def addMoveLSegmentByTarget(complex_path_id, joints, vel, acc, blendradius):
    dianaJointPos = DIANA_JOINTS_POSITION()
    dianaJointPos.values = (c_double * 7)()
    for index in range(0, len(joints)):
        dianaJointPos.values[index] = joints[index]
    api_mod.addMoveLByTarget.argtypes = [c_uint, POINTER(
        DIANA_JOINTS_POSITION), c_double, c_double, c_double]
    api_mod.addMoveLByTarget.restype = c_int
    ret = api_mod.addMoveLByTarget(
        complex_path_id, dianaJointPos, vel, acc, blendradius)
    return message(ret)


def addMoveLSegmentByPose(complex_path_id, pose, vel, acc, blendradius):
    dianaTcpPose = DIANA_TCP_POSE()
    dianaTcpPose.values = (c_double * 6)()
    for index in range(0, len(dianaTcpPose.values)):
        dianaTcpPose.values[index] = pose[index]
    api_mod.addMoveLByPose.argtypes = [c_uint, POINTER(
        DIANA_TCP_POSE), c_double, c_double, c_double]
    api_mod.addMoveLByPose.restype = c_int
    ret = api_mod.addMoveLByPose(
        complex_path_id, dianaTcpPose, vel, acc, blendradius)
    return message(ret)


def addMoveJSegmentByTarget(complex_path_id, joints, vel_percent, acc_percent, blendradius_percent):
    dianaJointPos = DIANA_JOINTS_POSITION()
    dianaJointPos.values = (c_double * 7)()
    for index in range(0, len(joints)):
        dianaJointPos.values[index] = joints[index]
    api_mod.addMoveJByTarget.argtypes = [c_uint, POINTER(
        DIANA_JOINTS_POSITION), c_double, c_double, c_double]
    api_mod.addMoveJByTarget.restype = c_int
    ret = api_mod.addMoveJByTarget(
        complex_path_id, dianaJointPos, vel_percent, acc_percent, blendradius_percent)
    return message(ret)


def addMoveJSegmentByPose(complex_path_id, pose, vel_percent, acc_percent, blendradius_percent):
    dianaTcpPose = DIANA_TCP_POSE()
    dianaTcpPose.values = (c_double * 6)()
    for index in range(0, len(dianaTcpPose.values)):
        dianaTcpPose.values[index] = pose[index]
    api_mod.addMoveJByPose.argtypes = [c_uint, POINTER(
        DIANA_TCP_POSE), c_double, c_double, c_double]
    api_mod.addMoveJByPose.restype = c_int
    ret = api_mod.addMoveJByPose(
        complex_path_id, dianaTcpPose, vel_percent, acc_percent, blendradius_percent)
    return message(ret)


def addMoveCSegmentByTarget(complex_path_id, pass_joints, target_joints, vel, acc, blendradius, ignore_rotation):
    dianaPassJointPos = DIANA_JOINTS_POSITION()
    dianaPassJointPos.values = (c_double * 7)()
    dianaTargetJointPos = DIANA_JOINTS_POSITION()
    dianaTargetJointPos.values = (c_double * 7)()
    for index in range(0, len(pass_joints)):
        dianaPassJointPos.values[index] = pass_joints[index]
    for index in range(0, len(target_joints)):
        dianaTargetJointPos.values[index] = target_joints[index]
    api_mod.addMoveCByTarget.argtypes = [c_uint, POINTER(DIANA_JOINTS_POSITION), POINTER(
        DIANA_JOINTS_POSITION), c_double, c_double, c_double, c_bool]
    api_mod.addMoveCByTarget.restype = c_int
    ret = api_mod.addMoveCByTarget(complex_path_id, dianaPassJointPos,
                                dianaTargetJointPos, vel, acc, blendradius, ignore_rotation)
    return message(ret)


def addMoveCSegmentByPose(complex_path_id, pass_pose, target_pose, vel, acc, blendradius, ignore_rotation):
    dianaPassTcpPose = DIANA_TCP_POSE()
    dianaPassTcpPose.values = (c_double * 6)()
    dianaTargetTcpPose = DIANA_TCP_POSE()
    dianaTargetTcpPose.values = (c_double * 6)()
    for index in range(0, len(dianaPassTcpPose.values)):
        dianaPassTcpPose.values[index] = pass_pose[index]
    for index in range(0, len(dianaTargetTcpPose.values)):
        dianaTargetTcpPose.values[index] = target_pose[index]
    api_mod.addMoveCByPose.argtypes = [c_uint, POINTER(DIANA_TCP_POSE), POINTER(
        DIANA_TCP_POSE), c_double, c_double, c_double, c_bool]
    api_mod.addMoveCByPose.restype = c_int
    ret = api_mod.addMoveCByPose(complex_path_id, dianaPassTcpPose,
                              dianaTargetTcpPose, vel, acc, blendradius, ignore_rotation)
    return message(ret)


def runComplexPath(complex_path_id):
    api_mod.runComplexPath.argtypes = [c_uint]
    api_mod.runComplexPath.restype = c_int
    ret = api_mod.runComplexPath(complex_path_id)
    return message(ret)


def destroyComplexPath(complex_path_id):
    api_mod.destroyComplexPath.argtypes = [c_uint]
    api_mod.destroyComplexPath.restype = c_int
    ret = api_mod.destroyComplexPath(complex_path_id)
    return message(ret)


def saveEnvironment():
    api_mod.saveEnvironment.restype = c_int
    ret = api_mod.saveEnvironment()
    return message(ret)


def moveTcpBaseLinearRelative(pose, v, a):
    dianaTcpPose = DIANA_TCP_POSE()
    dianaTcpPose.values = (c_double * 6)()
    for index in range(0, len(dianaTcpPose.values)):
        dianaTcpPose.values[index] = pose[index]
    api_mod.moveTcpBaseLinearRelative.argtypes = [POINTER(
        DIANA_TCP_POSE), c_double, c_double]
    api_mod.moveTcpBaseLinearRelative.restype = c_int
    ret = api_mod.moveTcpBaseLinearRelative(dianaTcpPose, v, a)
    return message(ret)

###################### version 0.10#########################

def dumpToUDiskEx(timeout_second):
    api_mod.dumpToUDiskEx.restype = c_int
    api_mod.dumpToUDiskEx.argtypes = [c_double]
    ret = api_mod.dumpToUDiskEx(timeout_second)
    return message(ret)

def enterForceMode_ex(forceDirection,forceValue,maxApproachVelocity,maxAllowTcpOffset,active_tcp):
    arr_ForceDirection = DIANA_FORCE_DIRECTION()
    arr_ForceDirection.values = (c_double*3)()
    for index in range(0, len(arr_ForceDirection.values)):
        arr_ForceDirection.values[index] = forceDirection[index]
    arr_Active_Tcp = DIANA_ACTIVE_TCP()
    arr_Active_Tcp.values = (c_double*6)()
    for index in range(0, len(arr_Active_Tcp.values)):
        arr_Active_Tcp.values[index] = active_tcp[index]
    api_mod.enterForceMode_ex.restype = c_int
    api_mod.enterForceMode_ex.argtypes = [POINTER(DIANA_FORCE_DIRECTION),c_double,c_double,c_double,POINTER(DIANA_ACTIVE_TCP)]
    ret = api_mod.enterForceMode_ex(byref(arr_ForceDirection),forceValue,maxApproachVelocity,maxAllowTcpOffset,byref(arr_Active_Tcp))
    return message(ret)

def readDI(group_name, di_name):
    DIValue = c_int()
    api_mod.readDI.argtypes = [c_char_p, c_char_p, POINTER(c_int)]
    api_mod.readDI.restype = c_int
    ret = api_mod.readDI(bytes(group_name, encoding = "utf-8"), bytes(di_name, encoding = "utf-8"), byref(DIValue))
    return ret, DIValue

def readAI(group_name, di_name):
    AIValue = c_double()
    AIMode = c_int()
    api_mod.readAI.argtypes = [c_char_p, c_char_p, POINTER(c_int),POINTER(c_double)]
    api_mod.readAI.restype = c_int
    ret = api_mod.readAI(bytes(group_name, encoding = "utf-8"), bytes(di_name, encoding = "utf-8"), byref(AIMode),byref(AIValue))
    return ret, AIMode, AIValue

def setAIMode(group_name, di_name,mode):
    api_mod.setAIMode.argtypes = [c_char_p, c_char_p, c_int]
    api_mod.setAIMode.restype = c_int
    ret = api_mod.setAIMode(bytes(group_name, encoding = "utf-8"), bytes(di_name, encoding = "utf-8"),mode)
    return message(ret)

def writeDO(group_name, di_name,mode,value):
    api_mod.writeDO.argtypes = [c_char_p, c_char_p, c_int]
    api_mod.writeDO.restype = c_int
    ret = api_mod.writeDO(bytes(group_name, encoding = "utf-8"), bytes(di_name, encoding = "utf-8"),value)
    return message(ret)

def writeAO(group_name, di_name,mode,value):
    api_mod.writeAO.argtypes = [c_char_p, c_char_p, c_int, c_double]
    api_mod.writeAO.restype = c_int
    ret = api_mod.writeAO(bytes(group_name, encoding = "utf-8"), bytes(di_name, encoding = "utf-8"),mode,value)
    return message(ret)

def readBusCurrent():
    Current = c_double()
    api_mod.readBusCurrent.argtypes = [POINTER(c_double)]
    api_mod.readBusCurrent.restype = c_int
    ret = api_mod.readBusCurrent(byref(Current))
    return ret, Current

def readBusVoltage():
    Voltage = c_double()
    api_mod.readBusVoltage.argtypes = [POINTER(c_double)]
    api_mod.readBusVoltage.restype = c_int
    ret = api_mod.readBusVoltage(byref(Voltage))
    return ret, Voltage

###################### version 1.10#########################

def getDH(aDH,alphaDH,dDH,thetaDH):
    arr_aDH = DIANA_DH_A()
    arr_aDH.values = (c_double*7)()
    arr_alphaDH = DIANA_DH_Alpha()
    arr_alphaDH.values = (c_double*7)()
    arr_dDH = DIANA_DH_D()
    arr_dDH.values = (c_double*7)()
    arr_thetaDH = DIANA_DH_Theta()
    arr_thetaDH.values = (c_double*7)()
    api_mod.getDH.restype = c_int
    api_mod.getDH.argtypes = [ POINTER(DIANA_DH_A), 
        POINTER(DIANA_DH_Alpha),POINTER(DIANA_DH_D), POINTER(DIANA_DH_Theta)]
    ret = api_mod.getDH(byref(arr_aDH),byref(arr_alphaDH),byref(arr_dDH),byref(arr_thetaDH))
    for index in range(0, len(aDH)):
       aDH[index] = arr_aDH.values[index]
    for index in range(0, len(alphaDH)):
       alphaDH[index] = arr_alphaDH.values[index]
    for index in range(0, len(dDH)):
       dDH[index] = arr_dDH.values[index]
    for index in range(0, len(thetaDH)):
       thetaDH[index] = arr_thetaDH.values[index]
    return message(ret)

def getOriginalJointTorque(torques):
    jointTorque = DIANA_JOINT_TORQUE()
    jointTorque.values = (c_double*7)()
    api_mod.getOriginalJointTorque.restype = c_int
    api_mod.getOriginalJointTorque.argtypes = [ POINTER(DIANA_JOINT_TORQUE)]
    ret = api_mod.getOriginalJointTorque(byref(jointTorque))
    for index in range(0, len(torques)):
       torques[index] = jointTorque.values[index]
    return message(ret)

def getJacobiMatrix(matrix_jacobi):
    jacobi_matrix = DIANA_JACOBI_MATRIX()
    jacobi_matrix.values = (c_double*42)()
    api_mod.getJacobiMatrix.restype = c_int
    api_mod.getJacobiMatrix.argtypes = [ POINTER(DIANA_JACOBI_MATRIX)]
    ret = api_mod.getJacobiMatrix(byref(jacobi_matrix))
    for index in range(0, len(matrix_jacobi)):
       matrix_jacobi[index] = jacobi_matrix.values[index]
    return message(ret)

###################### version 1.20#########################

def resetDH():
    api_mod.resetDH.restype = c_int
    ret = api_mod.resetDH()
    return message(ret)

###################### version 1.30#########################
def runProgram(name):
    api_mod.runProgram.argtypes = [c_char_p]
    api_mod.runProgram.restype = c_int
    ret = api_mod.runProgram(bytes(name, encoding = "utf-8"))
    return message(ret)

def stopProgram(name):
    api_mod.stopProgram.argtypes = [c_char_p]
    api_mod.stopProgram.restype = c_int
    ret = api_mod.stopProgram(bytes(name, encoding = "utf-8"))
    return message(ret)

def getVariableValue(name,value):
    api_mod.getVariableValue.argtypes = [c_char_p, POINTER(c_double)]
    api_mod.getVariableValue.restype = c_int
    ret = api_mod.getVariableValue(bytes(name, encoding = "utf-8"),byref(value))
    return message(ret)

def setVariableValue(name,value):
    api_mod.setVariableValue.argtypes = [c_char_p, c_double]
    api_mod.setVariableValue.restype = c_int
    ret = api_mod.setVariableValue(bytes(name, encoding = "utf-8"),value)
    return message(ret)

def isTaskRunning(name):
    api_mod.isTaskRunning.argtypes = [c_char_p]
    api_mod.isTaskRunning.restype = c_int
    ret = api_mod.isTaskRunning(bytes(name, encoding = "utf-8"))
    return message(ret)

def pauseProgram():
    api_mod.pauseProgram.restype = c_int
    ret = api_mod.pauseProgram()
    return message(ret)

def resumeProgram():
    api_mod.resumeProgram.restype = c_int
    ret = api_mod.resumeProgram()
    return message(ret)

def stopAllProgram():
    api_mod.stopAllProgram.restype = c_int
    ret = api_mod.stopAllProgram()
    return message(ret)

def isAnyTaskRunning():
    api_mod.isAnyTaskRunning.restype = c_int
    ret = api_mod.isAnyTaskRunning()
    return message(ret)

###################### version 1.40#########################

def cleanErrorInfo():
    api_mod.cleanErrorInfo.restype = c_int
    ret = api_mod.cleanErrorInfo()
    return message(ret)
    
def setCollisionLevel(level):
    api_mod.setCollisionLevel.argtypes = [c_int]
    api_mod.setCollisionLevel.restype = c_int
    ret = api_mod.setCollisionLevel(level)
    return message(ret)
