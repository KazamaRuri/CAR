from __future__ import division
import Adafruit_PCA9685
import time
import math
import numpy as np

p0=Adafruit_PCA9685.PCA9685()
p1=Adafruit_PCA9685.PCA9685()
p2=Adafruit_PCA9685.PCA9685()
p3=Adafruit_PCA9685.PCA9685()
p4=Adafruit_PCA9685.PCA9685()
p5=Adafruit_PCA9685.PCA9685()

p0.set_pwm_freq(50)
p1.set_pwm_freq(50)
p2.set_pwm_freq(50)
p3.set_pwm_freq(50)
p4.set_pwm_freq(50)
p5.set_pwm_freq(50)

l1=104#第一节
l2=99.5#第二节机械臂长度
l3=8#水平机械臂长既摄像头位置离第4个舵机的距离
dian=list(range(3))#机械臂对应起始位置
Q=list(range(4))
q_last=[90,60,90,0]#舵机起始角度
xs=1#相机照片大小与实际大小比例系数（实际大小相片大小）

def moveArmto(x,y,z):#xyz为摄像头位置
    if x>0 or x<0 :
        j0_1=math.atan(y/x)
    elif x==0:
        if y>0:
            j0_1=90
        else:
            j0_1=270
    (x,y)=(x-(l3*math.cos(j0_1)),y-(l3*math.sin(j0_1)))
    #记录位置，为下一个动作提供基点
    global dian
    dian=[x,y,z]
    R=math.sqrt(x*x+y*y+z*z)
    j0_0=math.acos(((x*x+y*y+z*z)+l1*l1-l2*l2)/(2*l1*R))
    A=math.sqrt(x*x+y*y)
    j0_2=math.atan(z/A)
    j1=j0_0+j0_2
    j0_3=math.acos((R*R+l2*l2-l1*l1)/(2*l2*R))
    j2=j0_0+j0_3
    j3=j0_3-j0_2 #end-effter-在水平面
    j0=j0_1#使后续使用规律化

    angle=[j0+math.pi/4,j1,j2,j3]
    #对各个角进行规划
    v_array=[0,0]#指定起止速度
    a_array=[0,0]#指定起止加速度
    start=[0]
    for i in range(len(angle)):
        start[0]=sum(math.fabs(angle[i-1]-q_last[i-1]),start[0])
        t_array=[0,math.sqrt(start/len(angle)/(math.pi/3))]#指定起止时间
    for n in range(len(angle)):
        q_array=[q_last[n-1], angle[n-1]]#指定起止位置
        T = t_array[1] - t_array[0]  # 时间差
        a0 = q_array[0]
        a1 = v_array[0]
        a2 = a_array[0] / 2
        a3 = (20 * q_array[1] - 20 * q_array[0] - (8 * v_array[1] + 12 * v_array[0]) * T - (3 * a_array[0] - a_array[ 1]) * math.pow(T, 2)) / (2 * math.pow(T, 3))
        a4 = (30 * q_array[0] - 30 * q_array[1] + (14 * v_array[ 1] + 16 * v_array[0]) * T + (3 * a_array[0] - 2 * a_array[1]) * math.pow(T, 2)) / (2 * math.pow(T, 4))
        a5 = (12 * q_array[1] - 12 * q_array[0] - (6 * v_array[ 1] + 6 * v_array[0]) * T - ( a_array[0] - a_array[ 1]) * math.pow(T, 2)) / (2 * math.pow(T, 5))
        ti = np.arange(t_array[0], t_array[1], .01)
        '''
        求解出角度，角速度，角加速度随某个时间区间随时间变换的曲线
        '''
        qi = a0 + a1 * (ti - t_array[0]) + a2 * np.power((ti - t_array[0]), 2) + a3 * np.power((ti - t_array[0]), 3) + a4 * np.power( (ti - t_array[0]), 4) + a5 * np.power((ti - t_array[0]), 5)
        qi = qi.tolist()  # 将矩阵转换为list，否则进行数据整合会报错
        Q[n]= [q_array[0]] + qi[1:]
    for i in range(len(Q[1])-1):
        duty0=4096*((Q[0][i-1]*7.4)+500)/20000
        duty1=4096*((Q[1][i-1]*11)+500)/20000
        duty2=4096*((Q[2][i-1]*11)+500)/20000
        duty3=4096*((Q[3][i-1]*11)+500)/20000
        #执行pwm信号
        p0.set_pwm(12,0,int(duty0))
        p1.set_pwm(13,0,int(duty1))
        p2.set_pwm(14,0,int(duty2))
        p3.set_pwm(15,0,int(duty3))
        #给予执行时间
        time.sleep(0.01)
    global q_last
    q_last=[j0+math.pi/4,j1,j2,j3]    
    p0.set_pwm(12,0,0)
    p1.set_pwm(13,0,0)
    p2.set_pwm(14,0,0)
    p3.set_pwm(15,0,0)
    time.sleep(0.01)

def moveArmby(a,b,c):#xyz为摄像头位置
    global dian
    if dian[0]<0 or dian[1]>0 :
        j0_1=math.atan(dian[1]/dian[0])
    elif dian[0]==0:
        if dian[1]>0:
            j0_1=90
        else:
            j0_1=270
    (x,y,z)=(dian[0]+a,dian[1]+b,dian[2]+c)#此点为第四个舵机的位置
    (x,y)=(x-(l3*math.cos(j0_1)),y-(l2*math.sin(j0_1)))
    dian=[x,y,z]#记录位置，为下一个动作提供基点
    R=math.sqrt(x*x+y*y+z*z)
    j0_0=math.acos((R*R+l1*l1-l2*l2)/(2*l1*R))
    a=math.sqrt(x*x+y*y)
    j0_2=math.atan(z/a)
    j1=j0_0+j0_2
    j0_3=math.acos((R*R+l2*l2-l1*l1)/(2*l2*R))
    j2=j0_0+j0_3
    j3=j0_3-j0_2 #end-effter-在水平面
    j0=j0_1#使后续使用规律化

    angle=[j0+math.pi/4,j1,j2,j3]
    #对各个角进行规划
    v_array=[0,0]#指定起止速度
    a_array=[0,0]#指定起止加速度
    start=[0]
    for i in range(len(angle)):
        start[0]=sum([math.fabs(angle[i-1]-q_last[i-1])],start[0])
        t_array=[0,math.sqrt(start[0]/len(angle)/(math.pi/3))]#指定起止时间
    for n in range(len(angle)):
        q_array=[q_last[n-1], n]#指定起止位置
        T = t_array[1] - t_array[0]  # 时间差
        a0 = q_array[0]
        a1 = v_array[0]
        a2 = a_array[0] / 2
        a3 = (20 * q_array[1] - 20 * q_array[0] - (8 * v_array[1] + 12 * v_array[0]) * T - (3 * a_array[0] - a_array[ 1]) * math.pow(T, 2)) / (2 * math.pow(T, 3))
        a4 = (30 * q_array[0] - 30 * q_array[1] + (14 * v_array[1] + 16 * v_array[0]) * T + (3 * a_array[0] - 2 * a_array[1]) * math.pow(T, 2)) / (2 * math.pow(T, 4))
        a5 = (12 * q_array[1] - 12 * q_array[0] - (6 * v_array[1] + 6 * v_array[0]) * T - ( a_array[0] - a_array[ 1]) * math.pow(T, 2)) / (2 * math.pow(T, 5))
        ti = np.arange(t_array[0], t_array[1], .01)
        '''
        求解出角度，角速度，角加速度随某个时间区间随时间变换的曲线
        '''
        qi = a0 + a1 * (ti - t_array[0]) + a2 * np.power((ti - t_array[0]), 2) + a3 * np.power((ti - t_array[0]), 3) + a4 * np.power( (ti - t_array[0]), 4) + a5 * np.power((ti - t_array[0]), 5)
        qi = qi.tolist()  # 将矩阵转换为list，否则进行数据整合会报错
        Q[n]= [q_array[0]] + qi[1:] #进行数据整合
    for i in range(len(q_last)-1):
        duty0=4096*((Q[0][i-1]*7.4)+500)/20000
        duty1=4096*((Q[1][i-1]*11)+500)/20000
        duty2=4096*((Q[2][i-1]*11)+500)/20000
        duty3=4096*((Q[3][i-1]*11)+500)/20000
        #执行pwm信号
        p0.set_pwm(12,0,int(duty0))
        p1.set_pwm(13,0,int(duty1))
        p2.set_pwm(14,0,int(duty2))
        p3.set_pwm(15,0,int(duty3))
        #给予执行时间
        time.sleep(0.01)
    global q_last
    q_last=[j0+math.pi/4,j1,j2,j3]
    #机械臂消抖
    p0.set_pwm(12,0,0)
    p1.set_pwm(13,0,0)
    p2.set_pwm(14,0,0)
    p3.set_pwm(15,0,0)
    time.sleep(0.01)

def shuipng(j4):
    duty4=4096*((j4*11)+500)/20000
    p4.set_pwm(14,0,duty4)
    time.sleep(0.5)
    p4.set_pwm(14,0,0)

def endEffector(j5):
    duty5=4096*((j5*11)+500)/20000
    p5.set_pwm(15,0,duty5)
    time.sleep(0.4)
    p5.set_pwm(15,0,0)

moveArmto(0,50,50)