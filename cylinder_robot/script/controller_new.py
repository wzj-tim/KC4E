#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState

position_x_esti = 0

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
    global position_x_esti
    pub = rospy.Publisher('/robot/control', Twist, queue_size = 10)

    rospy.init_node('talker',anonymous=True)

    rospy.Subscriber("/robot/esti_model_state", ModelState, callback_getting_esti)
    #============design your trace below=============
    rate = rospy.Rate(10)

    Controller = PID_Controller(5,0,1,-50,50)

    direction = 0
    turn = 0
    target = 2.5
    while turn < 1:
        if(direction is 0):
            error = -(target - position_x_esti)
            twist = Twist()
            twist.linear.x=Controller.get_output(error)
            pub.publish(twist)
            if(abs(error) < 0.001):
                direction = 1
                target = 7.5

        if(direction is 1):
            error = -(target - position_x_esti)
            twist = Twist()
            twist.linear.x=Controller.get_output(error)
            pub.publish(twist)
            if(abs(error) < 0.001):
                direction = 0
                target = 2.5

        rate.sleep() 

        # if direction is 0:
        #     for i in range(0,100):
        #         twist = Twist()
        #         twist.linear.x=1.5*abs(i-49.5)/(i-49.5)
        #         # twist.angular.z=0.5*abs(i-49.5)/(i-49.5)
        #         pub.publish(twist)
        #         rate.sleep() 
        #     direction = 1
        # if direction is 1:
        #     for i in range(0,100):
        #         twist = Twist()
        #         twist.linear.x=-1.5*abs(i-49.5)/(i-49.5)
        #         # twist.angular.z=0.5*abs(i-49.5)/(i-49.5)
        #         pub.publish(twist)
        #         rate.sleep() 
        #     direction = 0
        #     turn = turn + 1
        
    
    sys.exit(0)


def callback_getting_esti(state):
    global position_x_esti
    print(state.pose.position.x)
    position_x_esti = state.pose.position.x

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass


























