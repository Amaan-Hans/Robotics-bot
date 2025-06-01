#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from gazebo_msgs.srv import GetModelState
import math
from motion_planning import get_path

class SurveillanceBotMover:
    def __init__(self):
        # Publishers and Subscribers
        self.pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.handle_odometry)


        # State variables
        self.pose = (0.0, 0.0, 0.0)  # (x, y, orientation in radians)
        self.goal_threshold = 0.1

        # Velocity constraints
        self.max_accel = 0.02 # Adjust as needed
        self.max_speed = 1.0 # Adjust as needed
        self.last_velocity = 0.0

        # Angular control state
        self.prev_angular_error = 0.0
        self.angular_integral = 0.0

        self.model_state_srv  = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)


    def handle_odometry(self, msg):
        try:
            result = self.model_state_srv('mobile_base', '')  # Robot's model name in Gazebo World
            pos = result.pose.position
            ori = result.pose.orientation
            euler = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
            self.pose = (pos.x, pos.y, euler[2])
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def get_robot_pose(self):
        try:
            return self.model_state_srv('mobile_base', '').pose # Robot's model name in Gazebo World
        except rospy.ServiceException as e:
            rospy.logwarn("Could not retrieve model state: %s", e)
            return None

    def navigate_to(self, target, linear_gain=10.0, Kp=1.0, Ki=0.5, Kd=0.8):
        command = Twist()
        
        dx = target[0] - self.pose[0]
        dy = target[1] - self.pose[1]
        angle_to_goal = math.atan2(dy, dx)
        distance = math.hypot(dx, dy)

        if distance < self.goal_threshold:
            return command  # Goal reached, return zero command
        
        # Calculate orientation error and normalize
        angle_error = (angle_to_goal - self.pose[2] + math.pi) % (2 * math.pi) - math.pi

        # PID control for turning
        self.angular_integral += angle_error
        derivative = angle_error - self.prev_angular_error
        command.angular.z = Kp * angle_error + Ki * self.angular_integral + Kd * derivative
        self.prev_angular_error = angle_error

        if abs(angle_error) < self.goal_threshold:
            desired_speed = min(linear_gain * distance, self.max_speed)
            velocity_change = desired_speed - self.last_velocity

            if abs(velocity_change) > self.max_accel:
                desired_speed = self.last_velocity + math.copysign(self.max_accel, velocity_change)

            command.linear.x = desired_speed
            self.last_velocity = desired_speed

        return command

def main():
    rospy.init_node('turtlebot_mover')
    controller = SurveillanceBotMover()

    x = input("x: ")
    y = input("y: ")

    actual_position = controller.get_robot_pose()
    print("Robot current position:", actual_position)
    
    target = [int(x), int(y)] 
    start = controller.pose
    shortest_path = get_path(start, target)
    
    rate = rospy.Rate(10)  # 10 Hz

    try:
        # Now traverse the shortest path
        for step in shortest_path:
            target = [step[0], step[1]]
            print("Moving to target:", target)
            while not rospy.is_shutdown():
                motion = controller.navigate_to(target)
                controller.pub.publish(motion)

                dx = target[0] - controller.pose[0]
                dy = target[1] - controller.pose[1]
                if math.hypot(dx, dy) < controller.goal_threshold:
                    break

                rate.sleep()
                
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")


if __name__ == "__main__":
    main()
