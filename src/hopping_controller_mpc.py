
# 使用casadi实现车辆跟踪控制的优化求解器

from sys import path
path.append('./3rdparty/casadi-v3.5.1')
from casadi import *

from pylab import plot, step, figure, legend, show, spy

#import matplotlib.pyplot as plt

class HoppingControllerMPC:
    def __init__(self, N, dT):
        self.N = N
        self.dT = dT

    def prob_describe(self):
        N = self.N # number of control intervals
        dT = self.dT

        # ---- dynamic constraints --------
        M = 30              #kg
        g = 9.8             #
    
        x = SX.sym('x', 2)
        u = SX.sym('u', 1)
        xdot = vertcat(x[1], (u[0]/M)-g)
        f = Function('f', [x, u], [xdot])

        opti = Opti() # Optimization problem
        
        # ---- decision variables ---------
        X = opti.variable(2,N+1) # state trajectory
        height = X[0,:]
        velocity = X[1,:]
        U = opti.variable(1,N)   # control trajectory (throttle)

        # 参数
        P = opti.parameter(2,N)   # 目标位置参数


        for k in range(N):
            x_next = X[:,k] + dT*f(X[:,k], U[:,k])
            opti.subject_to(X[:,k+1]==x_next) # close the gaps

        Q = DM.zeros(2, 2)
        Q[0, 0] = 100
        Q[1, 1] = 1

        R = DM.zeros(1, 1)
        R[0, 0] = 0

        Qt = DM.zeros(2, 2)
        Qt[0, 0] = 1
        Qt[1, 1] = 0.2

        obj = 0
        for k in range(N):
            st_error = X[:,k+1] - P[:,k]
            obj = obj + mtimes([st_error.T, Q, st_error]) + mtimes([U[:,k].T, R, U[:,k]])
            # if k+1 == N:
            #     obj = obj + mtimes([st_error.T, Qt, st_error])


        #print(obj.shape)

        # ---- objective          ---------
        opti.minimize(obj) 

        #
        opti.subject_to(opti.bounded([0],U,[1000])) # control is limited

        self.X = X
        self.U = U
        self.P = P
        self.opti = opti


    def update(self,p_target,x_init,u_init):
        N = self.N # number of control intervals
        dT = self.dT
        X = self.X
        U = self.U
        P = self.P
        opti = self.opti
        
        # ---- boundary conditions --------
        #opti.subject_to(X[:,0]==[0,0,0])   # start at position 0 ...
        opti.subject_to(X[:,0]==x_init)

        # 初始化目标参数
        for k in range (N):
            opti.set_value(P[:,k],p_target)

        # ---- initial values for solver ---
        for k in range(N):
            # opti.set_initial(X[:,k+1], [0,0,0])
            # opti.set_initial(U[:,k],[0,0])
            opti.set_initial(X[:,k+1], x_init)
            opti.set_initial(U[:,k],u_init)

        # NLP solver options
        # opts = {}
        # opts["expand"] = True
        # opts["max_iter"] = 100
        # opts["verbose"] = True
        # opts["linear_solver"] = "ma57"
        # opts["hessian_approximation"] = "limited-memory"


        # ---- solve NLP              ------
        #opti.solver("ipopt") # set numerical backend
        #p_opts = {"expand": True,"print_stats": False}
        p_opts = {"expand": True}
        s_opts = {"max_iter": 30}

        opti.solver("ipopt",p_opts,s_opts)
        self.sol = opti.solve()   # actual solve


    def showfig(self):
        # ---- post-processing        ------
        figure()
        sol = self.sol
        X = self.X
        U = self.U
        plot(sol.value(X[0,:]))
        figure()
        plot(sol.value(X[1,:]))
        figure()
        plot(sol.value(U[0,:]))
        #show()
        

if __name__ == '__main__':
    
    tracker = HoppingControllerMPC(20, 0.1)
    tracker.prob_describe()

    p_target = [1,0]
    x_init = [0,0]
    u_init = [0]
    tracker.update(p_target,x_init,u_init)
    tracker.showfig()
    show()



