#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from mapping import get_path

class TurtlebotMover:
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.bridge = CvBridge()
        self.found_cart = False
        self.current_pose = (0, 0, 0)
        self.distance_tolerance = 0.1 

        self.MAX_LINEAR_ACCELERATION = 0.02
        self.MAX_LINEAR_VELOCITY = 1.0
        self.prev_linear_velocity = 0.0

        rospy.wait_for_service('gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)

        self.last_error_theta = 0
        self.integral_theta = 0
        self.car_positions = []

    def move_to_goal(self, goal, speed=10, Kp=1, Ki=0.5, Kd=0.8):
        delta_x = goal[0] - self.current_pose[0]
        delta_y = goal[1] - self.current_pose[1]
        distance_to_goal = math.sqrt(delta_x**2 + delta_y**2)

        if self.found_cart and distance_to_goal > self.distance_tolerance:
            print("Found Utility Cart")
            self.found_cart = False
            return

        move_cmd = Twist()
        move_cmd.linear.x = 0
        move_cmd.linear.y = 0
        move_cmd.linear.z = 0
        move_cmd.angular.x = 0
        move_cmd.angular.y = 0
        move_cmd.angular.z = 0

        goal_theta = math.atan2(delta_y, delta_x)

        if distance_to_goal < self.distance_tolerance:
            return move_cmd

        delta_theta = goal_theta - self.current_pose[2]
        delta_theta = (delta_theta + math.pi) % (2 * math.pi) - math.pi

        self.integral_theta += delta_theta
        derivative_theta = delta_theta - self.last_error_theta
        move_cmd.angular.z = Kp * delta_theta + Ki * self.integral_theta + Kd * derivative_theta
        self.last_error_theta = delta_theta

        if abs(delta_theta) > self.distance_tolerance:
            pass
        else:
            target_linear_velocity = min(speed * distance_to_goal, speed)
            delta_v = target_linear_velocity - self.prev_linear_velocity
            if abs(delta_v) > self.MAX_LINEAR_ACCELERATION:
                target_linear_velocity = self.prev_linear_velocity + math.copysign(self.MAX_LINEAR_ACCELERATION, delta_v)
            self.prev_linear_velocity = target_linear_velocity
            move_cmd.linear.x = target_linear_velocity

        return move_cmd

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_green = np.array([35, 100, 100])
        upper_green = np.array([85, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        THRESHOLD = 40000

        if cv2.countNonZero(mask) > THRESHOLD and not self.found_cart:
            print("Found Utility Cart")
            self.found_cart = True

    def odom_callback(self, msg):
        resp = self.get_model_state('mobile_base', '')
        position = resp.pose.position
        orientation = resp.pose.orientation
        qx, qy, qz, qw = orientation.x, orientation.y, orientation.z, orientation.w
        euler = euler_from_quaternion([qx, qy, qz, qw])
        self.current_pose = (position.x, position.y, euler[2])

    def get_gazebo_model_state(self):
        rospy.wait_for_service('gazebo/get_model_state')
        try:
            get_model_state = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
            resp = get_model_state('mobile_base', '')
            return resp.pose
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def get_car_position(self):
        resp_car = self.get_model_state('car', '')
        position_car = resp_car.pose.position
        orientation_car = resp_car.pose.orientation
        qx, qy, qz, qw = orientation_car.x, orientation_car.y, orientation_car.z, orientation_car.w
        euler_car = euler_from_quaternion([qx, qy, qz, qw])
        self.car_positions.append((position_car.x, position_car.y))

    def print_car_positions(self):
        print(self.car_positions)


if __name__ == "__main__":
    rospy.init_node('turtlebot_mover')

    mover = TurtlebotMover()
    x = input("x: ")
    y = input("y: ")
    actual_model_state = mover.get_gazebo_model_state()
    print("Actual model state: {}".format(actual_model_state))

    goal = [int(x), int(y)]

    origin = mover.current_pose

    shortest_path = get_path(origin, goal)

    rate = rospy.Rate(10)

    try:
        for step in shortest_path:
            goal_x, goal_y = step
            goal = [goal_x, goal_y]

            while not rospy.is_shutdown():
                move_cmd = mover.move_to_goal(goal)
                
                if move_cmd is not None:
                    mover.pub.publish(move_cmd)

                if math.sqrt((goal_x - mover.current_pose[0]) ** 2 + (goal_y - mover.current_pose[1]) ** 2) < mover.distance_tolerance:
                    break

                rate.sleep()

    except rospy.ROSInterruptException:
        pass

    mover.get_car_position()
    print("Car positions:")
    mover.print_car_positions()
