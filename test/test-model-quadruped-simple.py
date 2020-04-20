from __future__ import division
import numpy as np
import math
import sim as vrep
import time

# from sys import path
# path.append('./3rdparty/casadi-v3.5.1')
# from casadi import *

# path.append('./src')
# from height_controller_mpc import HeightControllerMPC




'''
首先连接vrep
'''
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


'''
设置仿真步长，并打开同步模式
这里需要说明的是仿真步长的设置似乎不能成功；同步模式可打开。
'''
RAD2DEG = 180 / math.pi   # 常数，弧度转度数
tstep = 0.005             # 定义仿真步长

# 设置仿真步长，为了保持API端与V-rep端相同步长
vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, tstep, vrep.simx_opmode_oneshot)
# 然后打开同步模式
vrep.simxSynchronous(clientID, True) 
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)


'''
获取模型中的handle，后续所有的读写操作都要用到。
'''
baseName = 'MainBody'
jointName = ['HipFL', 
            'ThighFL', 
            'ShankFL',
            'HipFR', 
            'ThighFR', 
            'ShankFR',
            'HipRL', 
            'ThighRL', 
            'ShankRL',
            'HipRR', 
            'ThighRR', 
            'ShankRR']

n_joint = len(jointName)
print(n_joint)

time.sleep(1)

_, baseHandle = vrep.simxGetObjectHandle(clientID, baseName, vrep.simx_opmode_blocking)
jointHandle = []
for i in range(n_joint):
    jointHandle.append(0)

for k in range(n_joint):
    _, jointHandle[k] = vrep.simxGetObjectHandle(clientID, jointName[k], vrep.simx_opmode_blocking)

print('Handles available!')



'''
首次读取本体和各关节的状态
'''
jointConfig = np.zeros(n_joint)
jointVelocity = np.zeros(n_joint)
_, base_pos = vrep.simxGetObjectPosition(clientID, baseHandle, -1, vrep.simx_opmode_streaming)
for k in range(len(jointHandle)):
    _, jointConfig[k] = vrep.simxGetJointPosition(clientID, jointHandle[k], vrep.simx_opmode_streaming)



'''
进入循环前，先让仿真走一步。
这个操作是参考模板进行的，具体原因未知。
'''
lastCmdTime=vrep.simxGetLastCmdTime(clientID)  # 记录当前时间
vrep.simxSynchronousTrigger(clientID)  # 让仿真走一步


'''
程序的主循环实现12个关节的位置控制。
之所以不用vrep内置的位置控制器，是因为前面仿真过程中发现vrep的关节速度控制器性能比较好，但位置控制器参数非常难以调节。
虽然也可能是我还没有完全弄清楚vrep仿真的细节，但这个可以先放一放。

为了编制循环，关节期望、误差和状态都使用向量。对应的关节与前面jointName中存储的顺序一致。
'''

j_target = np.zeros(12)

# 在vrep中输出关节力矩的方法比较独特，需要设置一个难以到达的速度和最大输出力矩。
control_force = np.zeros(12)
set_velocity = np.zeros(12)
set_force = np.zeros(12)

# p_target = np.zeros(2)
# x_init = np.zeros(2)
# control_force = np.zeros(2)

# set_velocity = np.zeros(2)
# set_force = np.zeros(2)

# p_target[0] = 0.5
# p_target[1] = 0

# j_target = -90/RAD2DEG

#用于测试的计数
index = 0

# 开始仿真
while vrep.simxGetConnectionId(clientID) != -1:
    currCmdTime=vrep.simxGetLastCmdTime(clientID)  # 记录当前时间
    dt = currCmdTime - lastCmdTime # 记录时间间隔，用于控制
    print(dt)
   
    # ***
    _, base_pos = vrep.simxGetObjectPosition(clientID, baseHandle, -1, vrep.simx_opmode_streaming)
    for k in range(n_joint):
        _, jointConfig[k] = vrep.simxGetJointPosition(clientID, jointHandle[k], vrep.simx_opmode_streaming)
        # print(jointConfig[k])
    
    _, linear_velocity, angular_velocity = vrep.simxGetObjectVelocity(clientID, baseHandle, vrep.simx_opmode_streaming)
    for k in range(n_joint):
        _, jointVelocity[k] = vrep.simxGetObjectFloatParameter(clientID, jointHandle[k], 2012, vrep.simx_opmode_streaming)
    
    print(jointConfig)
    #print(base_pos)

    # 髋部前向关节的期望角度是-45度
    for i in range(4):
        j_target[1+3*i] = -45/RAD2DEG

    # 膝部关节的期望角度是90度
    for i in range(4):
        j_target[2+3*i] = 90/RAD2DEG

    # 计算控制量
    for k in range(n_joint):
        p_error = j_target[k] - jointConfig[k]
        d_error = 0 - jointVelocity[k]
        control_force[k] = 300*p_error + 5*d_error

    print(control_force)
    # 
    for k in range(n_joint):
        if np.sign(control_force[k]) >= 0:
            set_force[k] = control_force[k]
            set_velocity[k] = 9999
        else:
            set_force[k] = -control_force[k]
            set_velocity[k] = -9999

    # x_init[0] = base_pos[2]
    # x_init[1] = linear_velocity[2]
    
    
    # p_error = j_target-jointConfig[2]
    # d_error = 0-jointVelocity[2]
    # # 摆动期 kp=10, kd=1
    # # 支撑期 kp=10, kd=1
    # control_force[1] = (40*p_error + 3*d_error)
    # control_force[0] = 0.01

    # print(control_force[1])
    # #
    # #control_force = 400
    # if np.sign(control_force[0]) >= 0:
    #     set_force[0] = control_force[0]
    #     set_velocity[0] = 9999
    # else:
    #     set_force[0] = -control_force[0]
    #     set_velocity[0] = -9999

    # if np.sign(control_force[1]) >= 0:
    #     set_force[1] = control_force[1]
    #     set_velocity[1] = 9999
    # else:
    #     set_force[1] = -control_force[1]
    #     set_velocity[1] = -9999

    # 控制命令需要同时方式，故暂停通信，用于存储所有控制命令一起发送
    vrep.simxPauseCommunication(clientID, True)
    
    for k in range(n_joint):
        vrep.simxSetJointTargetVelocity(clientID, jointHandle[k], set_velocity[k], vrep.simx_opmode_oneshot)
        vrep.simxSetJointForce(clientID, jointHandle[k], set_force[k], vrep.simx_opmode_oneshot)
    
    vrep.simxPauseCommunication(clientID, False)

    # ***
    # print(currCmdTime-lastCmdTime)
    lastCmdTime=currCmdTime    # 记录当前时间
    vrep.simxSynchronousTrigger(clientID)  # 进行下一步
    vrep.simxGetPingTime(clientID)    # 使得该仿真步走完