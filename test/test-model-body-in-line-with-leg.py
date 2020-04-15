from __future__ import division
import numpy as np
import math
import sim as vrep
import time

from sys import path
path.append('./3rdparty/casadi-v3.5.1')
from casadi import *

path.append('./src')
from height_controller_mpc import HeightControllerMPC


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



RAD2DEG = 180 / math.pi   # 常数，弧度转度数
tstep = 0.005             # 定义仿真步长

# 设置仿真步长，为了保持API端与V-rep端相同步长
vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, tstep, vrep.simx_opmode_oneshot)
# 然后打开同步模式
vrep.simxSynchronous(clientID, True) 
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)



baseName = 'Body'
jointName = ['Tran', 'RobotHip', 'RobotKnee']

print(len(jointName))

_, baseHandle = vrep.simxGetObjectHandle(clientID, baseName, vrep.simx_opmode_blocking)
jointHandle = [0,0,0]
for k in range(len(jointName)):
    _, jointHandle[k] = vrep.simxGetObjectHandle(clientID, jointName[k], vrep.simx_opmode_blocking)

print('Handles available!')

jointConfig = [0,0,0]
jointVelocity = [0,0,0]
_, base_pos = vrep.simxGetObjectPosition(clientID, baseHandle, -1, vrep.simx_opmode_streaming)
for k in range(len(jointHandle)):
    _, jointConfig[k] = vrep.simxGetJointPosition(clientID, jointHandle[k], vrep.simx_opmode_streaming)

lastCmdTime=vrep.simxGetLastCmdTime(clientID)  # 记录当前时间
# vrep.simxSynchronousTrigger(clientID)  # 让仿真走一步


p_target = np.zeros(2)
x_init = np.zeros(2)
control_force = np.zeros(2)

set_velocity = np.zeros(2)
set_force = np.zeros(2)

p_target[0] = 0.5
p_target[1] = 0

j_target = -90/RAD2DEG

#用于测试的计数
index = 0

# 开始仿真
while vrep.simxGetConnectionId(clientID) != -1:
    currCmdTime=vrep.simxGetLastCmdTime(clientID)  # 记录当前时间
    dt = currCmdTime - lastCmdTime # 记录时间间隔，用于控制
   
    # ***
    _, base_pos = vrep.simxGetObjectPosition(clientID, baseHandle, -1, vrep.simx_opmode_streaming)
    for k in range(len(jointHandle)):
        _, jointConfig[k] = vrep.simxGetJointPosition(clientID, jointHandle[k], vrep.simx_opmode_streaming)
        #print(jointConfig[k])
    
    _, linear_velocity, angular_velocity = vrep.simxGetObjectVelocity(clientID, baseHandle, vrep.simx_opmode_streaming)
    for k in range(len(jointHandle)):
        _, jointVelocity[k] = vrep.simxGetObjectFloatParameter(clientID, jointHandle[k], 2012, vrep.simx_opmode_streaming)
    #print(jointConfig)
    #print(base_pos)

    x_init[0] = base_pos[2]
    x_init[1] = linear_velocity[2]
    
    
    p_error = j_target-jointConfig[2]
    d_error = 0-jointVelocity[2]
    # 摆动期 kp=10, kd=1
    # 支撑期 kp=10, kd=1
    control_force[1] = (40*p_error + 3*d_error)
    control_force[0] = 0.01

    print(control_force[1])
    #
    #control_force = 400
    if np.sign(control_force[0]) >= 0:
        set_force[0] = control_force[0]
        set_velocity[0] = 9999
    else:
        set_force[0] = -control_force[0]
        set_velocity[0] = -9999

    if np.sign(control_force[1]) >= 0:
        set_force[1] = control_force[1]
        set_velocity[1] = 9999
    else:
        set_force[1] = -control_force[1]
        set_velocity[1] = -9999

    # 控制命令需要同时方式，故暂停通信，用于存储所有控制命令一起发送
    vrep.simxPauseCommunication(clientID, True)
    
    vrep.simxSetJointTargetVelocity(clientID, jointHandle[1], set_velocity[0], vrep.simx_opmode_oneshot)
    vrep.simxSetJointForce(clientID, jointHandle[1], set_force[0], vrep.simx_opmode_oneshot)
    
    vrep.simxSetJointTargetVelocity(clientID, jointHandle[2], set_velocity[1], vrep.simx_opmode_oneshot)
    vrep.simxSetJointForce(clientID, jointHandle[2], set_force[1], vrep.simx_opmode_oneshot)
    
    vrep.simxPauseCommunication(clientID, False)

    # ***
    print(currCmdTime-lastCmdTime)
    lastCmdTime=currCmdTime    # 记录当前时间
    vrep.simxSynchronousTrigger(clientID)  # 进行下一步
    vrep.simxGetPingTime(clientID)    # 使得该仿真步走完