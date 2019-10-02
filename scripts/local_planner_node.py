#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import tf
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import sys
reload(sys)
sys.setdefaultencoding('utf-8')

class model:
    
    def __init__(self):
        self.vmax = 1.0
        self.vmin = 0.0
        self.omega_max = 0.4
        self.lin_acc = 0.2
        self.ang_acc = 0.2
        self.dt = 0.1
        self.time = 3.0
        self.v_res = 0.01
        self.acc_res = 0.0175
        self.alpha = 0.1
        self.beta = 0.2
        self.gamma = 0.1
        

def motion(x, u, dt):
    # motion model

    x[2] += u[1] * dt
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]

    return x
    
def calc_dynamic_window(x, model):

    # Dynamic window from robot specification
    Vs = [model.vmin, model.vmax,
          -model.omega_max, model.omega_max]

    # Dynamic window from motion model
    Vd = [x[3] - model.lin_acc * model.dt,
          x[3] + model.lin_acc * model.dt,
          x[4] - model.ang_acc * model.dt,
          x[4] + model.ang_acc * model.dt]

    #  [vmin,vmax, yawrate min, yawrate max]  Fa intersezione tra Vs e Vd
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw
    
def calc_heading(x, goal):
    
    theta = np.degrees(x[2])
    goalTheta = np.degrees(math.atan2(goal[1]-x[1], goal[0]-x[0]))
    
    if goalTheta > theta:
        targetTheta = goalTheta - theta
    else:
        targetTheta = theta - goalTheta
    
    return 180 - targetTheta
    
def calc_dist(x, obstacle, obstacleRadius):
    dist = 2
    
    for ob in obstacle:
        distance = math.hypot(ob[0]-x[0], ob[1]-x[1]) - obstacleRadius
        if dist > distance:
            dist = distance
    return dist
    
def calc_trajectory(x, u, model):
    dt = model.dt
    time = 0
    traj = x
    while time <= model.time:
        time += dt
        x = motion(x, u, dt)
        traj = np.vstack((traj, x)) 
        
    return x, traj
    
def normalize(evalOF):
    print(evalOF)
    if sum(evalOF[:,2]) != 0.0:
        evalOF[:,2] = evalOF[:,2]/sum(evalOF[:,2])
    if sum(evalOF[:,3]) != 0.0:
        evalOF[:,3] = evalOF[:,3]/sum(evalOF[:,3])     
    if sum(evalOF[:,4]) != 0.0:
        evalOF[:,4] = evalOF[:,4]/sum(evalOF[:,4])    
    
    return evalOF
    

def compute_obj_func(x, dw, goal, obstacle, obstacleRadius, model):
    evalOF = np.array(x)
    trajOF = np.array(x)
    
    for vt in np.arange(dw[0], dw[1], model.v_res):
        for ot in np.arange(dw[2], dw[3], model.acc_res):
            x, traj = calc_trajectory(x, [vt, ot], model)
            heading = calc_heading(x, goal)
            dist = calc_dist(x, obstacle, obstacleRadius)
            vel = abs(vt)
            
            evalOF = np.vstack((evalOF, [vt, ot, heading, dist, vel]))  
            trajOF = np.vstack((trajOF, traj))
            
    return evalOF, trajOF
    
    
def DWA(x, goal, obstacle, obstacleRadius, model):
    
    dw = calc_dynamic_window(x, model)
    
    evalOF, trajOF = compute_obj_func(x, dw, goal, obstacle, obstacleRadius, model)
    
    
    if evalOF.size == 0:
        print("no path")
        u = [0.0, 0.0]
        
    
    evalOF = normalize(evalOF) # è una lista
    #final_value = []
    print(evalOF)
    final_value = np.array([[i] for i in range(len(evalOF))]) # array colonna

    #for i in range(int(evalOF.shape[0])):
    for i in range(len(evalOF)):
        param = np.array([model.alpha, model.beta, model.gamma])
        
        #final_value.append(np.dot(param, evalOF[i,2:5]))
        #final_value[i] = np.dot(param, evalOF[i,2:5])
        final_value[i] = param[0]*evalOF[i,2] + param[1]*evalOF[i,3] + param[2]*evalOF[i,4]
        
        
    #final_value = np.asarray(final_value)

    
    evalOF = np.hstack((evalOF, final_value))
    
    index = np.argmax(final_value)
    u = evalOF[index,0:2]
    
    return u, trajOF
    
def get_rpy(rb_odom):
    orientation = rb_odom.pose.pose.orientation
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    euler = tf.transformations.euler_from_quaternion(quaternion)
    return euler[2]    
    
class subOdom(object):
  def __init__(self):
    self.sub = rospy.Subscriber('/odom', Odometry, self.sub_callback)
    self.data = Odometry()
    
  def sub_callback(self, msg):
    self.data = msg
    
class pubCmd(object):
  def __init__(self, pub, sub_msg):
    self._pub = pub
    self.data = sub_msg

  def callback(self,sub_msg):
    self.data = sub_msg
    
    
def main():

    rospy.init_node('local_planner_node', anonymous=True)
    rate = rospy.Rate(10)
    odom = subOdom()
    rate.sleep()
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
  
    x = odom.data.pose.pose.position.x
    y = odom.data.pose.pose.position.y
    yaw = get_rpy(odom.data)
    
    X = [x, y, yaw, 0.0, 0.0] 
    
    goal = np.array([10, 10])
    obstacle = np.array([[-1, -1],
                   [0, 2],
                   [4.0, 2.0],
                   [5.0, 4.0],
                   [5.0, 5.0],
                   [12.0, 12.0]
                   ])
    obstacleRadius = 0.4
    
    robot = model()
    dt = robot.dt

    while not rospy.is_shutdown():
        u, traj = DWA(X, goal, obstacle, obstacleRadius, robot)
        X = motion(X, u, dt)
        print(X)
        msg = Twist()
        
        cmd_msg = pubCmd(pub, msg)
        cmd_msg.data.linear.x = u[0]
        cmd_msg.data.angular.z = u[1]
        cmd_msg._pub.publish(cmd_msg.data)
    
    rate.sleep()
    
if __name__=='__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass    


    
