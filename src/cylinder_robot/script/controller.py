#!/usr/bin/env python3

import rospy
import sys
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
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
    dbg_pubR = rospy.Publisher('debug/control_real', Twist, queue_size = 10) # pose&speed
    dbg_pubE = rospy.Publisher('debug/control_expect', Twist, queue_size = 10) # pose&speed

    rospy.init_node('talker',anonymous=True)

    rospy.Subscriber("/robot/esti_model_state", ModelState, callback_getting_esti)
    #============design your trace below=============
    rate = rospy.Rate(10)

    # speedControllerX = PID_Controller(20,0,0,-200,200)
    # positionControllerX = PID_Controller(1,0,1,-50,50)
    speedControllerX = PID_Controller(42,0,0,-200,200)
    positionControllerX = PID_Controller(2,0,1.2,-50,50)
    speedControllerY = PID_Controller(42,0,0,-200,200)
    positionControllerY = PID_Controller(2,0,1.2,-50,50)

    state = 0

    direction = 0
    lastTarget = [0,0]
    target = [3, 3]
    targets = [[3,3],[-4,0],[3,-3],[0,0],[3,3]]
    target_idx = 0
    target_cnt = 0
    waitCounter = 30
    while 1:

        if(waitCounter>0): # initialize, wait for the observed value to become stable
            waitCounter = waitCounter-1
            print(waitCounter)
            twist = Twist()
            pub.publish(twist)
            rate.sleep()
            continue

        twist = Twist()
        twist_E = Twist()
        twist_R = Twist()

        xposError = target[0] - position_x_esti
        xexpSpd = positionControllerX.get_output(xposError)
        xspdErr = xexpSpd - velocity_x_esti
        twist.linear.x = speedControllerX.get_output(xspdErr)

        yposError = target[1] - position_y_esti
        yexpSpd = positionControllerY.get_output(yposError)
        yspdErr = yexpSpd - velocity_y_esti
        twist.linear.y = speedControllerY.get_output(yspdErr)
        # print("dir:{},expPos:{:.2f},realPos:{:2.2f},posErr:{:2.2f},expSpd:{:2.2f},realSpd:{:2.2f},spdErr:{:2.2f},force:{:2.2f}".format(direction,target,position_x_esti,posError,expSpd,velocity_x_esti,spdErr,twist.linear.x))

        if abs(xposError)<0.01 and abs(yposError)<0.01:
            target_cnt = target_cnt+1
        else:
            target_cnt = 0

        if target_cnt > 3:
            lastTarget = targets[target_idx]
            target_idx = (target_idx+1)%5
            target = targets[target_idx]

        pub.publish(twist)

        twist_E.linear.x = target[0]
        twist_E.linear.y = target[1]
        twist_E.angular.x = xexpSpd
        twist_E.angular.y = yexpSpd

        twist_R.linear.x = position_x_esti
        twist_R.linear.y = position_y_esti
        twist_R.angular.x = velocity_x_esti
        twist_R.angular.y = velocity_y_esti

        dbg_pubE.publish(twist_E)
        dbg_pubR.publish(twist_R)

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


























