from __future__ import division
import numpy as np
import math
import sim as vrep
import time

from sys import path
path.append('./3rdparty/casadi-v3.5.1')
from casadi import *

path.append('./src')
from hopping_controller_mpc import HoppingControllerMPC

# import socket       #引入套接字
# import threading    #引入并行
# import struct       #引入结构体

RAD2DEG = 180 / math.pi   # 常数，弧度转度数
tstep = 0.01             # 定义仿真步长


baseName = 'Body'
jointName = 'Tran'

print('Program started')
# 关闭潜在的连接
vrep.simxFinish(-1)
# 每隔0.2s检测一次，直到连接上V-rep
while True:
    clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    if clientID > -1:
        break
    else:
        time.sleep(0.2)
        print("Failed connecting to remote API server!")
print("Connection success!")


# 设置仿真步长，为了保持API端与V-rep端相同步长
vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, tstep, vrep.simx_opmode_oneshot)
# 然后打开同步模式
vrep.simxSynchronous(clientID, True) 
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

_, baseHandle = vrep.simxGetObjectHandle(clientID, baseName, vrep.simx_opmode_blocking)
_, jointHandle = vrep.simxGetObjectHandle(clientID, jointName, vrep.simx_opmode_blocking)

print('Handles available!')

_, base_pos = vrep.simxGetObjectPosition(clientID, baseHandle, -1, vrep.simx_opmode_streaming)
_, jointConfig = vrep.simxGetJointPosition(clientID, jointHandle, vrep.simx_opmode_streaming)

lastCmdTime=vrep.simxGetLastCmdTime(clientID)  # 记录当前时间
vrep.simxSynchronousTrigger(clientID)  # 让仿真走一步


N = 20
dT = 0.1
controller = HoppingControllerMPC(N, dT)

p_target = [1,0]
x_init = [0,0]
u_init = [0]

#用于测试的计数
index = 0

# 开始仿真
while vrep.simxGetConnectionId(clientID) != -1:
    currCmdTime=vrep.simxGetLastCmdTime(clientID)  # 记录当前时间
    dt = currCmdTime - lastCmdTime # 记录时间间隔，用于控制
   
    # ***
    _, base_pos = vrep.simxGetObjectPosition(clientID, baseHandle, -1, vrep.simx_opmode_streaming)
    _, jointConfig = vrep.simxGetJointPosition(clientID, jointHandle, vrep.simx_opmode_streaming)
    
    _, linear_velocity, angular_velocity = vrep.simxGetObjectVelocity(clientID, baseHandle, vrep.simx_opmode_streaming)
    #print(jointConfig)
    #print(base_pos)

    x_init[0] = base_pos[2]
    x_init[1] = linear_velocity[2]

    print(x_init)

    controller.prob_describe()
    controller.update(p_target,x_init,u_init)

    sol = controller.sol
    X = controller.X
    U = controller.U

    control_force = sol.value(U[0,0])
    #
    #control_force = 400
    if np.sign(control_force) >= 0:
        set_force = control_force
        set_velocity = 9999
    else:
        set_force = -control_force
        set_velocity = -9999

    # 控制命令需要同时方式，故暂停通信，用于存储所有控制命令一起发送
    vrep.simxPauseCommunication(clientID, True)
    
    vrep.simxSetJointTargetVelocity(clientID, jointHandle, set_velocity, vrep.simx_opmode_oneshot)
    vrep.simxSetJointForce(clientID, jointHandle, set_force, vrep.simx_opmode_oneshot)

    #vrep.simxSetJointForce(clientID,jointHandle,set_force,vrep.simx_opmode_oneshot);

    # vrep.simxSetJointTargetPosition(clientID, jointHandle, 100, vrep.simx_opmode_oneshot)
    # vrep.simxSetJointTargetPosition(clientID, jointHandle[1], 5/RAD2DEG, vrep.simx_opmode_oneshot)

    # vrep.simxSetJointTargetVelocity(clientID, jointHandle[2], 500/RAD2DEG, vrep.simx_opmode_oneshot)
    # vrep.simxSetJointTargetVelocity(clientID, jointHandle[3], 500/RAD2DEG, vrep.simx_opmode_oneshot)
    
    vrep.simxPauseCommunication(clientID, False)

    # ***
    lastCmdTime=currCmdTime    # 记录当前时间
    vrep.simxSynchronousTrigger(clientID)  # 进行下一步
    vrep.simxGetPingTime(clientID)    # 使得该仿真步走完