# Hopping_Control_Simulated_in_Python_and_Coppelia



## 1、包含的实验



### 1.1 刚体的高度控制

src/height_control_process

将刚体约束在竖直方向，控制需要考虑刚体的重力。

【对应的模型】

Coppelia_Model/single_body_up_and_down

### 1.2 刚体的浮动控制

src/hopping_control_process

将刚体约束在竖直方向，控制考虑刚体的重力以及驱动力施加的时机（对应步态）。

【对应的模型】

Coppelia_Model/single_body_up_and_down


### 1.3 四足机器人的运动控制

#### 1.3.1 目标
复现和改进cheetah software。
主要思路是：（1）用coppelia模型代替cheetah software的内置模型。（2）将控制器分解为两个模块，把控制算法与平台之间的bridge（hardwarebridge和simaltionbridge）分离出来，让算法模块更纯粹。同时设计算法模块时使用一个process程序调用所有的子模块，消除子模块之间的相互调用。

在仿真模型一层，为了对接bridge，实现一个interface接口程序。初步可以按cheetah software目前的结构，将关节伺服放在接口程序中。未来还可以对照真实平台的结构，将关节伺服控制从interface中分离出来。

#### 1.3.2 对应的coppelia模型

Coppelia_Model/quadruped_simple

该模型是直接用coppelia的gui设计的，所有的杆件都是内置的shape，没有使用对应真实机器人的mesh。


#### 1.3.3 实现过程

由于四足机器人运动控制实现较为复杂，模块众多，直接阅读源码有些困难。这里将实现过程记录如下：

【1】 quadruped_simple模型的测试

test/test-mode-quadruped-simple.py

上面的程序参考coppelia的接口程序模板，实现机器人状态（包括本体状态和关节状态）的读取和关节力矩指令的下发。程序中实现了简单的关节PD控制器。设置的关节期望角度为初始状态下的关节角度。

运行程序时，四足机器人保持站立位形，并持续输出关节状态和输出力矩。






































