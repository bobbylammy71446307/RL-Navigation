#!/usr/bin/env python3
import math
import os
import random
import subprocess
import time
from os import path

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist, Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from squaternion import Quaternion
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from gazebo_msgs.srv import SetModelState
import tf

GOAL_REACHED_DIST = 0.3
COLLISION_DIST = 0.4
TIME_DELTA = 0.1

odom_topic="/odometry/filtered"
imu_topic="/imu/data"



class GazeboEnv:
    """Superclass for all Gazebo environments."""

    def __init__(self, environment_dim,ros_package=None,launch_file=None):
        self.environment_dim = environment_dim
        self.odom_x = 0
        self.odom_y = 0

        self.goal_x = -99
        self.goal_y = -99

        self.start_pos_x=0
        self.start_pos_y=0

        self.laser_data = np.ones(self.environment_dim) * 10
        self.last_odom = None

        self.gaps = [[-np.pi / 2 - 0.03, -np.pi / 2 + np.pi / self.environment_dim]]
        for m in range(self.environment_dim - 1):
            self.gaps.append(
                [self.gaps[m][1], self.gaps[m][1] + np.pi / self.environment_dim]
            )
        self.gaps[-1][-1] += 0.03

        rospy.init_node("gym", anonymous=True)


        # Set up the ROS publishers and subscribers
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.set_state = rospy.Publisher(
            "gazebo/set_model_state", ModelState, queue_size=10
        )
        self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.reset_proxy = rospy.ServiceProxy("/gazebo/reset_world", Empty)
        self.publisher = rospy.Publisher("goal_point", MarkerArray, queue_size=3)
        self.publisher2 = rospy.Publisher("linear_velocity", MarkerArray, queue_size=1)
        self.publisher3 = rospy.Publisher("angular_velocity", MarkerArray, queue_size=1)
        self.laser_sub = rospy.Subscriber(
            "front/scan", LaserScan, self.laser_callback, queue_size=1
        )
        self.odom = rospy.Subscriber(
            odom_topic, Odometry, self.odom_callback, queue_size=1
        )
        self.reset_odom_pub=rospy.Publisher('/set_pose', PoseWithCovarianceStamped, queue_size=1)


    # range as state representation
    def laser_callback(self, scan_msg):
        self.laser_data = np.ones(self.environment_dim) * 10  # Initialize with large values

        angle = scan_msg.angle_min
        for i, dist in enumerate(scan_msg.ranges):
            if scan_msg.range_min < dist < scan_msg.range_max:
                x = dist * math.cos(angle)
                y = dist * math.sin(angle)

                # Ignore points behind or very close to the sensor
                if x > 0.1:
                    dot = x * 1 + y * 0
                    mag1 = math.sqrt(x**2 + y**2)
                    mag2 = 1  # magnitude of (1, 0)
                    beta = math.acos(dot / (mag1 * mag2)) * np.sign(y)

                    for j in range(len(self.gaps)):
                        if self.gaps[j][0] <= beta < self.gaps[j][1]:
                            self.laser_data[j] = min(self.laser_data[j], dist)
                            break
            angle += scan_msg.angle_increment

    def odom_callback(self, od_data):
        self.last_odom = od_data

    # Perform an action and read a new state
    def step(self, action):
        target = False
        # Publish the robot action
        vel_cmd = Twist()
        vel_cmd.linear.x = action[0]
        vel_cmd.angular.z = action[1]
        self.vel_pub.publish(vel_cmd)
        #self.publish_markers(action)

        #wait for react
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")

        # propagate state for TIME_DELTA seconds
        time.sleep(TIME_DELTA)

        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            pass
            self.pause()
        except (rospy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")



        # read velodyne laser state
        done, collision, min_laser = self.observe_collision(self.laser_data)
        v_state = []
        v_state[:] = self.laser_data[:]
        laser_state = [v_state]

        # Calculate robot heading from odometry data

        self.odom_x = self.last_odom.pose.pose.position.x
        self.odom_y = self.last_odom.pose.pose.position.y

        quaternion = Quaternion(
            self.last_odom.pose.pose.orientation.w,
            self.last_odom.pose.pose.orientation.x,
            self.last_odom.pose.pose.orientation.y,
            self.last_odom.pose.pose.orientation.z,
        )
        euler = quaternion.to_euler(degrees=False)
        angle = round(euler[2], 4)

        # Calculate distance to the goal from the robot
        distance = np.linalg.norm(
            [self.odom_x - self.goal_x, self.odom_y - self.goal_y]
        )
        print("odom",self.odom_x,self.odom_y)

        # Calculate the relative angle between the robots heading and heading toward the goal
        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y
        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))
        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle
        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta

        # Detect if the goal has been reached and give a large positive reward
        if distance < GOAL_REACHED_DIST:
            target = True
            done = True
        # print(self.odom_x,self.odom_y)
        # print("distance:", distance)
        #print("angle of robot", math.degrees(theta))


        robot_state = [distance, theta, action[0], action[1]]
        state = np.append(laser_state, robot_state)
        reward = self.get_reward(target, collision, action, min_laser)

        return state, reward, done, target

    def reset(self):
        x=np.random.uniform(-0.5,22)
        if 21<=x<=22:
            y=np.random.uniform(-22,0.5)
        else:
            y=np.random.uniform(-0.8,0.8)
        angle = np.random.uniform(-np.pi, np.pi)
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, angle)
        quaternion_world=tf.transformations.quaternion_from_euler(0.0, 0.0, angle-np.pi/2)
        # self.respawn(x,y,quaternion_world)
        self.respawn(x,y,quaternion)
        # print("jackal spawned at", x,y)
        self.reset_odom(x,y,quaternion)
        # set a random goal in empty space in environment
        self.change_goal(x,y)
        print("goal set at", self.goal_x,self.goal_y)
        self.start_pos_x=self.last_odom.pose.pose.position.x if self.last_odom is not None else 0
        self.start_pos_y=self.last_odom.pose.pose.position.y if self.last_odom is not None else 0
        # randomly scatter boxes in the environment
        #self.publish_markers([0.0, 0.0])
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")

        time.sleep(TIME_DELTA)

        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")
        v_state = []
        v_state[:] = self.laser_data[:]
        laser_state = [v_state]

        distance = np.linalg.norm(
            [self.odom_x - self.goal_x, self.odom_y - self.goal_y]
        )

        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y

        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))

        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle

        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta

        robot_state = [distance, theta, 0.0, 0.0]
        state = np.append(laser_state, robot_state)
        return state

    def change_goal(self,x,y):
        self.goal_x=-99
        self.goal_y=-99
        if 21<=y<=22:
            self.goal_y=y
            while not 0<=self.goal_x<=22:
                if np.random.rand() < 0.5:
                    rand = np.random.uniform(-4, -2)
                else:
                    rand = np.random.uniform(2, 4)
                self.goal_x=x+rand
                # self.goal_x=x+np.random.uniform(2, 4)
        else:
            while not 0<=self.goal_y<=22:
                if np.random.rand() < 0.5:
                    rand = np.random.uniform(-4, -2)
                else:
                    rand = np.random.uniform(2, 4)
                self.goal_y=y+rand
                #self.goal_y=y+np.random.uniform(2, 4)
            self.goal_x=x


    def reset_odom(self,x,y,quaternion):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"  
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 2.6
        qx,qy,qz,qw=quaternion
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        self.reset_odom_pub.publish(msg)


    def respawn(self,jackal_x,jackal_y,jackal_quaternion):
        rospy.wait_for_service('/gazebo/set_model_state')
        # self.respawn_object("jackal",-jackal_y,jackal_x,self.rotate_z(jackal_quaternion))
        self.respawn_object("jackal",-jackal_y,jackal_x,jackal_quaternion)
        # rospy.sleep(0.1)
        self.respawn_object("person_standing",18,22,[4.999999583255033e-07, -5.000000416744966e-07, -0.7071063120935576, 0.7071072502792263])
        # rospy.sleep(0.1)
        self.respawn_object("Construction Cone",5,22,[-5.0009999850000004e-07,4.9989999850000006e-07,1.1999999960000002e-05,0.999999999928])
        # rospy.sleep(0.1)
        self.respawn_object("husky",13.4539,21.0879, [-1.99898265e-06, -4.49970630e-06, -9.81604052e-02, 9.95176028e-01])
        # rospy.sleep(0.1)
        self.respawn_object("polaris_ranger_ev",9.09305,22.6416,[-5.04786183e-05, 3.29748259e-05, -8.13950386e-04, 9.99999668e-01])


    def respawn_object(self,model,x,y,quaternion):
        model_state = ModelState()
        model_state.model_name = model
        model_state.pose.position.x = x
        model_state.pose.position.y = y
        model_state.pose.position.z = 2.7
        qx,qy,qz,qw=quaternion
        model_state.pose.orientation.x = qx
        model_state.pose.orientation.y = qy
        model_state.pose.orientation.z = qz
        model_state.pose.orientation.w = qw
        self.set_state.publish(model_state)

    def rotate_z(self,quaternion,degree=-80):

        radians = math.radians(degree)
        half_angle = radians / 2.0
        q_z = [0, 0, math.sin(half_angle), math.cos(half_angle)]

        transformed_quaternion = tf.transformations.quaternion_multiply(q_z, quaternion)

        return transformed_quaternion

    # def publish_markers(self, action):
    #     # Publish visual data in Rviz
    #     markerArray = MarkerArray()
    #     marker = Marker()
    #     marker.header.frame_id = "odom"
    #     marker.type = marker.CYLINDER
    #     marker.action = marker.ADD
    #     marker.scale.x = 0.1
    #     marker.scale.y = 0.1
    #     marker.scale.z = 0.01
    #     marker.color.a = 1.0
    #     marker.color.r = 0.0
    #     marker.color.g = 1.0
    #     marker.color.b = 0.0
    #     marker.pose.orientation.w = 1.0
    #     marker.pose.position.x = self.goal_x
    #     marker.pose.position.y = self.goal_y
    #     marker.pose.position.z = 0

    #     markerArray.markers.append(marker)

    #     self.publisher.publish(markerArray)

    #     markerArray2 = MarkerArray()
    #     marker2 = Marker()
    #     marker2.header.frame_id = "odom"
    #     marker2.type = marker.CUBE
    #     marker2.action = marker.ADD
    #     marker2.scale.x = abs(action[0])
    #     marker2.scale.y = 0.1
    #     marker2.scale.z = 0.01
    #     marker2.color.a = 1.0
    #     marker2.color.r = 1.0
    #     marker2.color.g = 0.0
    #     marker2.color.b = 0.0
    #     marker2.pose.orientation.w = 1.0
    #     marker2.pose.position.x = 5
    #     marker2.pose.position.y = 0
    #     marker2.pose.position.z = 0

    #     markerArray2.markers.append(marker2)
    #     self.publisher2.publish(markerArray2)

    #     markerArray3 = MarkerArray()
    #     marker3 = Marker()
    #     marker3.header.frame_id = "odom"
    #     marker3.type = marker.CUBE
    #     marker3.action = marker.ADD
    #     marker3.scale.x = abs(action[1])
    #     marker3.scale.y = 0.1
    #     marker3.scale.z = 0.01
    #     marker3.color.a = 1.0
    #     marker3.color.r = 1.0
    #     marker3.color.g = 0.0
    #     marker3.color.b = 0.0
    #     marker3.pose.orientation.w = 1.0
    #     marker3.pose.position.x = 5
    #     marker3.pose.position.y = 0.2
    #     marker3.pose.position.z = 0

    #     markerArray3.markers.append(marker3)
    #     self.publisher3.publish(markerArray3)

    @staticmethod
    def observe_collision(laser_data):
        # Detect a collision from laser data
        min_laser = min(laser_data)
        if min_laser < COLLISION_DIST:
            return True, True, min_laser
        return False, False, min_laser

    @staticmethod
    def get_reward(target, collision, action, min_laser):
        if target:
            return 100.0
        elif collision:
            return -100
        else:
            r3 = lambda x: 1 - x if x < 1 else 0.0
            return action[0] / 2 - abs(action[1]) / 2 - r3(min_laser) / 2 
        
if __name__ == "__main__":
    env = GazeboEnv(20)
    time.sleep(1)
    for i in range(-0,-180,-5):
        print("the angle is",i)
        radians = math.radians(i)
        quart=tf.transformations.quaternion_from_euler(0.0, 0.0, radians)
        env.respawn(20, 0.5, quart)
        env.reset_odom(20,0.5, quart)
        time.sleep(1)



