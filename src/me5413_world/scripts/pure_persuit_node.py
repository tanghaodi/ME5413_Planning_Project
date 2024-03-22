#!/usr/bin/env python3
# coding:utf-8

import rospy
import math
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from tf.transformations import euler_from_quaternion
import dynamic_reconfigure.server
from me5413_world.cfg import pure_persuitConfig

class PurePursuitNode:
    def __init__(self):
        rospy.init_node('pure_pursuit_node')
        
        self.path_sub = rospy.Subscriber("/me5413_world/planning/local_path", Path, self.path_callback)
        self.odom_sub = rospy.Subscriber("/gazebo/ground_truth/state", Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher("/jackal_velocity_controller/cmd_vel", Twist, queue_size=1)

        self.current_path = Path()
        self.robot_odom = Odometry()

        self.server = dynamic_reconfigure.server.Server(pure_persuitConfig, self.config_callback)
        #self.speed_target = 0.5  # Default speed target
        #self.lookahead_distance = 2.0  # Default look ahead distance
        #self.steering_gain = 2.0  # Default steering gain

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    
    def config_callback(self, config, level):
        rospy.loginfo("Reconfiguring: speed_target={0}, lookahead_distance={1}, steering_gain={2}".format(config["speed_target"], config["lookahead_distance"], config["steering_gain"]))
        self.speed_target = config.speed_target
        self.lookahead_distance = config.lookahead_distance
        self.steering_gain = config.steering_gain

        return config


    def path_callback(self, msg):
        self.current_path = msg

    def odom_callback(self, msg):
        self.robot_odom = msg

    def find_lookahead_point(self):
        for pose_stamped in self.current_path.poses:
            pose = pose_stamped.pose
            dx = pose.position.x - self.robot_odom.pose.pose.position.x
            dy = pose.position.y - self.robot_odom.pose.pose.position.y
            distance = math.sqrt(dx**2 + dy**2)
            if distance >= self.lookahead_distance:
                return pose
        return None

    def compute_steering_angle(self, lookahead_point):
        if lookahead_point is None:
            return 0
        # Calculate current pose
        orientation_q = self.robot_odom.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        # Calculate look ahead point
        dx = lookahead_point.position.x - self.robot_odom.pose.pose.position.x
        dy = lookahead_point.position.y - self.robot_odom.pose.pose.position.y
        angle_to_goal = math.atan2(dy, dx)
        
        angle_diff = self.normalize_angle(angle_to_goal - yaw)
        return angle_diff

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            lookahead_point = self.find_lookahead_point()
            if lookahead_point is not None:
                steering_angle = self.compute_steering_angle(lookahead_point)
                cmd_msg = Twist()
                cmd_msg.linear.x = self.speed_target
                cmd_msg.angular.z = self.steering_gain * steering_angle  # Steering gain
                self.cmd_pub.publish(cmd_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        pp_node = PurePursuitNode()
        pp_node.run()
    except rospy.ROSInterruptException:
        pass

