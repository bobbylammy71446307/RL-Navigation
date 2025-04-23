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



class GazeboEnv:
    """Superclass for all Gazebo environments."""

    def __init__(self, rospackage,launchfile, environment_dim):
        self.environment_dim = environment_dim
        self.odom_x = 0
        self.odom_y = 0

        self.goal_x = -99
        self.goal_y = -99

        self.start_pos_x=0
        self.start_pos_y=0

        self.upper = 5.0
        self.lower = -5.0
        self.laser_data = np.ones(self.environment_dim) * 10
        self.last_odom = None

        self.set_self_state = ModelState()
        self.set_self_state.model_name = "jackal"
        self.set_self_state.pose.position.x = 0.0
        self.set_self_state.pose.position.y = 0.0
        self.set_self_state.pose.position.z = 0.0
        self.set_self_state.pose.orientation.x = 0.0
        self.set_self_state.pose.orientation.y = 0.0
        self.set_self_state.pose.orientation.z = 0.0
        self.set_self_state.pose.orientation.w = 1.0

        self.gaps = [[-np.pi / 2 - 0.03, -np.pi / 2 + np.pi / self.environment_dim]]
        for m in range(self.environment_dim - 1):
            self.gaps.append(
                [self.gaps[m][1], self.gaps[m][1] + np.pi / self.environment_dim]
            )
        self.gaps[-1][-1] += 0.03

        # port = "11311"
        # subprocess.Popen(["roscore", "-p", port])

        # print("Roscore launched!")

        # # Launch the simulation with the given launchfile name
        rospy.init_node("gym", anonymous=True)

        # subprocess.Popen(["roslaunch", rospackage, launchfile])
        # print("Gazebo launched!")

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
        self.listener = tf.TransformListener()

        self.odom = rospy.Subscriber(
            "/odometry/filtered", Odometry, self.odom_callback, queue_size=1
        )
        self.reset_odom_pub=rospy.Publisher('/set_pose', PoseWithCovarianceStamped, queue_size=1)


    # Read velodyne pointcloud and turn it into distance data, then select the minimum value for each angle
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
        self.publish_markers(action)

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
        try:
            # Lookup transform from 'odom' to 'base_link'
            (trans, rot) = self.listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        self.odom_x = self.start_pos_x+self.last_odom.pose.pose.position.y 
        self.odom_y = self.start_pos_y+self.last_odom.pose.pose.position.x 
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
        #print("odom",self.odom_x,self.odom_y)

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
        print(self.odom_x,self.odom_y)
        print("distance:", distance)


        robot_state = [distance, theta, action[0], action[1]]
        state = np.append(laser_state, robot_state)
        reward = self.get_reward(target, collision, action, min_laser,distance)

        return state, reward, done, target

    def reset(self):
        valid=False
        while valid==False:
            y=np.random.uniform(-0.5,22)
            if 21<=y<=22:
                x=np.random.uniform(-0.5,22)
            else:
                x=np.random.uniform(-0.8,0.8)
            if [y,x] not in [[22,5],[22,8],[21,8],[22,9],[21,9],[22,10],[21,10],[22,15],[22,18]]:
                valid=True
        self.respawn_jackal(x,y)
        self.start_pos_x=x
        self.start_pos_y=y
        print("jackal spawned at", x,y)
        angle = np.random.uniform(-np.pi, np.pi)
        self.reset_odom()
        # set a random goal in empty space in environment
        self.change_goal(x,y)
        print("goal set at", self.goal_x,self.goal_y)

        # randomly scatter boxes in the environment
        self.publish_markers([0.0, 0.0])

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


    def reset_odom(self,x=0,y=0):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"  # or "map", depending on your config

        msg.pose.pose.position.x = 0
        msg.pose.pose.position.y = 0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        self.reset_odom_pub.publish(msg)


    def respawn_jackal(self,x,y):
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            yaw = 1.5708  # radians
            quat = Quaternion.from_euler(0.0, 0.0, yaw)
            initial_pose = Pose()
            initial_pose.position.x = x
            initial_pose.position.y = y
            initial_pose.position.z = 3
            initial_pose.orientation = Quaternion(*quat)

            initial_twist = Twist()
            initial_twist.linear.x = 0.0
            initial_twist.angular.z = 0.0

            jackal_state = ModelState()
            jackal_state.model_name = 'jackal' 
            jackal_state.pose = initial_pose
            jackal_state.twist = initial_twist
            jackal_state.reference_frame = 'world'
            rospy.wait_for_service('/gazebo/pause_physics')
            rospy.ServiceProxy('/gazebo/pause_physics', Empty)()

            # Set model state
            result=set_state(jackal_state)

            # Then unpause physics
            rospy.wait_for_service('/gazebo/unpause_physics')
            rospy.ServiceProxy('/gazebo/unpause_physics', Empty)()
            if not result.success:
                rospy.logwarn("Failed to reset Jackal position.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def publish_markers(self, action):
        # Publish visual data in Rviz
        markerArray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.goal_x
        marker.pose.position.y = self.goal_y
        marker.pose.position.z = 0

        markerArray.markers.append(marker)

        self.publisher.publish(markerArray)

        markerArray2 = MarkerArray()
        marker2 = Marker()
        marker2.header.frame_id = "odom"
        marker2.type = marker.CUBE
        marker2.action = marker.ADD
        marker2.scale.x = abs(action[0])
        marker2.scale.y = 0.1
        marker2.scale.z = 0.01
        marker2.color.a = 1.0
        marker2.color.r = 1.0
        marker2.color.g = 0.0
        marker2.color.b = 0.0
        marker2.pose.orientation.w = 1.0
        marker2.pose.position.x = 5
        marker2.pose.position.y = 0
        marker2.pose.position.z = 0

        markerArray2.markers.append(marker2)
        self.publisher2.publish(markerArray2)

        markerArray3 = MarkerArray()
        marker3 = Marker()
        marker3.header.frame_id = "odom"
        marker3.type = marker.CUBE
        marker3.action = marker.ADD
        marker3.scale.x = abs(action[1])
        marker3.scale.y = 0.1
        marker3.scale.z = 0.01
        marker3.color.a = 1.0
        marker3.color.r = 1.0
        marker3.color.g = 0.0
        marker3.color.b = 0.0
        marker3.pose.orientation.w = 1.0
        marker3.pose.position.x = 5
        marker3.pose.position.y = 0.2
        marker3.pose.position.z = 0

        markerArray3.markers.append(marker3)
        self.publisher3.publish(markerArray3)

    @staticmethod
    def observe_collision(laser_data):
        # Detect a collision from laser data
        min_laser = min(laser_data)
        if min_laser < COLLISION_DIST:
            return True, True, min_laser
        return False, False, min_laser

    @staticmethod
    def get_reward(target, collision, action, min_laser,distance):
        if target:
            return 100.0
        elif collision:
            return -100
        else:
            r3 = lambda x: 1 - x if x < 1 else 0.0
            return action[0] / 2 - abs(action[1]) / 2 - r3(min_laser) / 2 