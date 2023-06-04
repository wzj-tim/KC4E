#!/usr/bin/env python3

import rospy
import sys
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from gazebo_msgs.msg import ModelState

position_x_esti = 0
velocity_x_esti = 0
position_y_esti = 0
velocity_y_esti = 0

class PID_Controller:
    def __init__(self,kp,ki,kd,output_min,output_max):
        # 初始化PID的三个参数，以及误差项
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error = 0
        self.last_error = 0
        self.error_sum = 0
        self.error_diff = 0
        # 初始化最大输出与最小输出
        self.output_min = output_min
        self.output_max = output_max
        self.output = 0

    def constrain(self, output):
        # 控制器输出阈值限制
        if output > self.output_max:
            output = self.output_max
        elif output < self.output_min:
            output = self.output_min
        else:
            output = output
        return output

    def get_output(self, error):
        # 使用位置式PID获取输出
        self.error = error
        self.error_sum += self.error
        self.error_diff = self.error - self.last_error
        self.last_error = self.error

        output = self.kp * self.error + self.ki * self.error_sum + self.kd * self.error_diff
        self.output = self.constrain(output)

        return self.output



def controller():
    global position_x_esti, velocity_x_esti, position_y_esti, velocity_y_esti
    pub = rospy.Publisher('/robot/control', Twist, queue_size = 10)
    dbg_PosErr = rospy.Publisher('debug/posErr', Point, queue_size = 10) # pose&speed
    dbg_Spd = rospy.Publisher('debug/speed', Point, queue_size = 10) # pose&speed
    dbg_Force = rospy.Publisher('debug/force', Point, queue_size = 10) # pose&speed
    dbg_ExpPath = rospy.Publisher('debug/path', Point, queue_size = 10) # pose&speed
    dbg_Dominator = rospy.Publisher('debug/dominator', Point, queue_size = 10)
    dbg_XPID = rospy.Publisher('debug/xpid', Twist, queue_size = 10)

    rospy.init_node('talker',anonymous=True)

    rospy.Subscriber("/robot/esti_model_state", ModelState, callback_getting_esti)
    #============design your trace below=============
    rate = rospy.Rate(100)

    # speedControllerX = PID_Controller(20,0,0,-200,200)
    # positionControllerX = PID_Controller(1,0,1,-50,50)
    speedControllerX = PID_Controller(8,0,0,-300,300)
    positionControllerX = PID_Controller(1.2,0,2,-50,50)
    speedControllerY = PID_Controller(8,0,0,-300,300)
    positionControllerY = PID_Controller(1.2,0,2,-50,50)

    state = 0

    direction = 0
    lastTarget = [0,0]
    target = [3, 3]
    targets = [[3,3],[-4,0],[3,-3],[0,0]]
    target_idx = 0
    target_cnt = 0
    waitCounter = 300
    while 1:

        if(waitCounter>0): # initialize, wait for the observed value to become stable
            waitCounter = waitCounter-1
            print(waitCounter)
            twist = Twist()
            pub.publish(twist)

            posError = Point()
            speed = Point()
            force = Point()
            path = Point()
            dominator = Point()
            xpid = Twist()

            dbg_PosErr.publish(posError)
            dbg_Spd.publish(speed)
            dbg_Force.publish(force)
            dbg_ExpPath.publish(path)
            dbg_Dominator.publish(dominator)
            dbg_XPID.publish(xpid)

            rate.sleep()
            continue

        twist = Twist()

        curPose = np.array([position_x_esti, position_y_esti])
        expPath = np.array([target[0]-lastTarget[0], target[1]-lastTarget[1]])
        pathNorm_ = np.linalg.norm(expPath)
        verticalVect = np.array([-expPath[1],expPath[0]])/pathNorm_
        totalError = np.array([target[0]-curPose[0], target[1]-curPose[1]])
        paraError = (np.dot(expPath,totalError))/pathNorm_/pathNorm_ * expPath
        vertError = totalError - paraError

        totalSpeed = [velocity_x_esti, velocity_y_esti]
        paraSpeed = (np.dot(expPath,totalSpeed))/pathNorm_/pathNorm_ * expPath
        vertSpeed = totalSpeed - paraSpeed

        paraExpSpd_ = positionControllerX.get_output(np.dot(paraError,expPath))
        paraForce_ = speedControllerX.get_output(paraExpSpd_ - np.dot(paraSpeed,expPath))

        print(paraError,paraSpeed)
        vertExpSpd_ = positionControllerY.get_output(np.dot(vertError,verticalVect))
        vertForce_ = speedControllerY.get_output(vertExpSpd_ - np.dot(vertSpeed,verticalVect))

        totalForce = paraForce_ /pathNorm_ * expPath + vertForce_ * verticalVect

        twist.linear.x = totalForce[0]
        twist.linear.y = totalForce[1]

        xposError = target[0] - position_x_esti
        # xexpSpd = positionControllerX.get_output(xposError)
        # xspdErr = xexpSpd - velocity_x_esti
        # twist.linear.x = speedControllerX.get_output(xspdErr)

        yposError = target[1] - position_y_esti
        # yexpSpd = positionControllerY.get_output(yposError)
        # yspdErr = yexpSpd - velocity_y_esti
        # twist.linear.y = speedControllerY.get_output(yspdErr)
        # print("dir:{},expPos:{:.2f},realPos:{:2.2f},posErr:{:2.2f},expSpd:{:2.2f},realSpd:{:2.2f},spdErr:{:2.2f},force:{:2.2f}".format(direction,target,position_x_esti,posError,expSpd,velocity_x_esti,spdErr,twist.linear.x))

        if abs(xposError)<0.02 and abs(yposError)<0.02:
            target_cnt = target_cnt+1
        else:
            target_cnt = 0

        if target_cnt > 3:
            lastTarget = targets[target_idx]
            target_idx = (target_idx+1)%4
            target = targets[target_idx]

        pub.publish(twist)

        posError.x = np.linalg.norm(paraError)
        posError.y = np.linalg.norm(vertError)
        speed.x = paraSpeed[0]
        speed.y = paraSpeed[1]
        force.x = totalForce[0]
        force.y = totalForce[1]
        path.x = expPath[0]
        path.y = expPath[1]
        dominator.x = pathNorm_/1000
        dominator.y = np.linalg.norm(vertError)
        dominator.z = target_idx/100


        dbg_PosErr.publish(posError)
        dbg_Spd.publish(speed)
        dbg_Force.publish(force)
        dbg_ExpPath.publish(path)
        dbg_Dominator.publish(dominator)
        dbg_XPID.publish(xpid)

        rate.sleep()

    sys.exit(0)


'''
        posError = target - position_x_esti

        if(direction == 0):
            if(abs(posError) < 0.1 or position_x_esti > 2.5):
                direction = 1
                target = -2.5
        elif(direction == 1):
            if(abs(posError) < 0.1 or position_x_esti < -2.5):
                direction = 0
                target = 2.5

        twist = Twist()
        expSpd = positionControllerX.get_output(posError)
        spdErr = expSpd - velocity_x_esti
        twist.linear.x = speedControllerX.get_output(spdErr)
        # speedControllerX.get_output(positionControllerX.get_output(error)-velocity_x_esti)
        pub.publish(twist)
        
        print("dir:{},expPos:{:.2f},realPos:{:2.2f},posErr:{:2.2f},expSpd:{:2.2f},realSpd:{:2.2f},spdErr:{:2.2f},force:{:2.2f}".format(direction,target,position_x_esti,posError,expSpd,velocity_x_esti,spdErr,twist.linear.x))
'''
    


def callback_getting_esti(state):
    global position_x_esti, velocity_x_esti, position_y_esti, velocity_y_esti
    # print(state.pose.position.x)
    position_x_esti = state.pose.position.x
    velocity_x_esti = state.twist.linear.x
    position_y_esti = state.pose.position.y
    velocity_y_esti = state.twist.linear.y

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass


























