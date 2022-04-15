from cmath import cos, sin
import numpy as np
from math import *
import random
import copy

class DWA:
    def plan(self, data, dwaconfig, midpos, planning_obs):
        
        VMin = dwaconfig.min_speed            #最小速度
        VMax = dwaconfig.max_speed             #最大速度
        WMin = -dwaconfig.max_yawrate    #最小角速度
        WMax = dwaconfig.max_yawrate   #最大角速度
        Va = dwaconfig.max_accel               #加速度
        Wa = dwaconfig.max_dyawrate      #角加速度

        Vreso = dwaconfig.v_reso            #速度分辨率
        Wreso = dwaconfig.yawrate_reso   #角速度分辨率
        Dt = 0.1                #时间间隔
        PredictTime = dwaconfig.predict_time      #模拟轨迹的持续时间
        
        alpha = 1.0             #距离目标点的评价函数的权重系数
        Belta = 1.0             #速度评价函数的权重系数
        Gamma = 1.0             #距离障碍物距离的评价函数的权重系数
        x=np.array(data)        #设定初始位置，角速度，线速度
        u=np.array([0,0])                                        #设定初始速度
        goal=np.array(midpos)                                     #设定目标位置
        global_tarj=np.array(x)
        vx, vw = self.DwaCore(x, u, midpos, planning_obs, Vreso, Wreso, Va, Wa, VMin, VMax, WMin, WMax, PredictTime, Dt)

        #the information above can be get from main
        return vx, vw

    def GoalCost(self, Goal, Pos):#goal is the midpos
        return sqrt((Pos[-1,0]-Goal[0])**2+(Pos[-1,1]-Goal[1])**2)

    def VelocityCost(self, Pos, VMax):#input max V
        return VMax-Pos[-1,3]

    def ObstacleCost(self, Pos, Obstacle):
        MinDistance = float('Inf')
        for i in range(len(Pos)):
            for j in range(len(Obstacle)):
                Current_Distance = sqrt((Pos[i,0]-Obstacle[j,0])**2+(Pos[i,1]-Obstacle[j,1])**2)
                if Current_Distance < 0.25:
                    return float('Inf')
                if Current_Distance < MinDistance:
                    MinDistance = Current_Distance

        return 1/MinDistance

    def VRange(self, X, Va, Wa, VMin, VMax, WMin, WMax, Dt):##input Va, Wa, VMin, VMax
        Vmin_Actual = X[3]-Va*Dt          #实际在dt时间间隔内的最小速度
        Vmax_actual = X[3]+Va*Dt          #实际载dt时间间隔内的最大速度
        Wmin_Actual = X[4]-Wa*Dt          #实际在dt时间间隔内的最小角速度
        Wmax_Actual = X[4]+Wa*Dt          #实际在dt时间间隔内的最大角速度
        VW = [max(VMin,Vmin_Actual),min(VMax,Vmax_actual),max(WMin,Wmin_Actual),min(WMax,Wmax_Actual)]  #因为前面本身定义了机器人最小最大速度所以这里取交集
        return VW

    def Motion(self,X,u,dt):
        X[0] = X[0] + u[0] * dt * cos(X[2]) #the location on x direction
        X[1] = X[1] + u[0] * dt * sin(X[2]) #the location on y direciton
        X[2] = X[2] + u[1] * dt #the change of angle
        X[3] = u[0] #v
        X[4] = u[1] #w
        return X

    def CalculateTraj(self, X, u, PredictTime, Dt):#一条模拟轨迹的完整计算
        Traj = np.array(X)
        Xnew = np.array(X)
        time = 0
        while time <= PredictTime:
            Xnew=self.Motion(Xnew,u,Dt)
            Traj=np.vstack((Traj,Xnew))   #一条完整模拟轨迹中所有信息集合成一个矩阵
            time=time+Dt
        return Traj

    def DwaCore(self, X, u, Goal, Obstacle, Vreso, Wreso, Va, Wa, VMin, VMax, WMin, WMax, PredictTime, Dt):
        vw = self.VRange(X, Va, Wa, VMin, VMax, WMin, WMax, Dt)
        BestTraj = np.array(X)
        MinScore = 10000.0
        for v in np.arange(vw[0],vw[1],Vreso): ##
            for w in np.arange(vw[2],vw[3],Wreso):##
                Traj = self.CalculateTraj(X,[v,w], PredictTime, Dt)
                GoalScore = self.GoalCost(Goal,Traj)
                VelScore = self.VelocityCost(Traj, VMax)
                ObsScore = self.ObstacleCost(Traj, Obstacle)
                Score = GoalScore + VelScore + ObsScore
                if MinScore >= Score:
                    MinScore = Score
                    u = np.array([v,w])
                    BestTraj = Traj
        return u[0], u[1] #, BestTraj