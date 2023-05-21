#!/usr/bin/env python

import control
import rospy
import math
from pr_5.msg import Robot_systems
#from geometry_msgs.msg import Twist

encr = 0
encl = 0
w_lg = 0
w_rg = 0
teta_g = 0
x_g = 0
y_g = 0
t_g = 0

def wspeed(w_l, w_r):
    global t_g
    k = 1
    T = 1.5
    N = 4096
    W = control.tf(k, [T, 1])
    system = control.LinearIOSystem(W)
    print("first")
    #магия сау
    t = float(rospy.get_time())
    #a, w, demo_x = control.input_output_response(system, [t_g, t], [w_g, w_wheel], return_x=True)
    dt = t - t_g
    print(t,t_g,dt)
    #вычисление показаний энкодера
    dencl = int(w_l * dt * N / (2 * 3.1415))
    dencr = int(w_r * dt * N / (2 * 3.1415))
    #вычисление угловой скорости колеса
    wl = dencl * 2 * 3.1415 / dt / N
    wr = dencr * 2 * 3.1415 / dt / N
    t_g = t
    return dt, wl,wr,  dencl, dencr
def funny(V, W):
    global t_g
    global teta_g
    global x_g
    global y_g
    global t_g
    global encr
    global encl
    global w_rg
    global w_lg
    L = 0.287
    r = 0.033
    #угловые скорости колёс по полученным данным
    wl = (V - 0.5 * L * W) / r
    wr = (V + 0.5 * L * W) / r
    #угловые скорости колёс после сау
    dt, wl,wr, dencl, dencr = wspeed(wl, wr)
    encl += dencl
    encr += dencr
    #посчитанные угловая и линейная скорости
    V = 0.5 * r * (wl + wr)
    W = r * (wr - wl) / L
    teta = W * dt
    x = V * math.cos(teta) * dt
    y = V * math.sin(teta) * dt
    x_g += x
    y_g += y
    teta_g += teta
    return t_g

def callback(data):
    funny(data.linear_vel, data.angular_vel)
    rospy.loginfo("Encoder_left %0.2f", encl)
    rospy.loginfo("Encoder_right %0.2f", encr)
    rospy.loginfo("Y: %0.2f", y_g)
    rospy.loginfo("X: %0.2f", x_g)

def listener():
    rospy.init_node('custom_listener', anonymous=True)
    rospy.Subscriber("custom_chatter", Robot_systems, callback)
    t_g = float(rospy.get_time())
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
