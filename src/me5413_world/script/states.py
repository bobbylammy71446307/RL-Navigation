#!/usr/bin/env python3

from dynamic_reconfigure.client import Client as DynamicReconfigureClient
import rospy
import smach
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String,Bool
from perception_depth import Image_segmentation
from threading import Thread

way_point_2=[22.0,-21.0]

class IdleState(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['task_received'])
    
    def execute(self, userdata):
        rospy.loginfo("Idling")
        rospy.sleep(0.1)
        return 'task_received'

class Task1_obs_avoid_nav(smach.State):
    def __init__(self,goal=[22, -21, 0.0, 0.0, 0.0, -1.0, 0.1]):
        smach.State.__init__(self,outcomes=['goal_reached','failed','stopped'])
        self.client=actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.timeout = rospy.Duration(240)
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "map"
        self.target_pose.pose.position.x = goal[0]
        self.target_pose.pose.position.y = goal[1]
        self.target_pose.pose.position.z = goal[2]
        self.target_pose.pose.orientation.x = goal[3]
        self.target_pose.pose.orientation.y = goal[4]
        self.target_pose.pose.orientation.z = goal[5]
        self.target_pose.pose.orientation.w = goal[6]

    def execute(self,userdata):
        rospy.loginfo("Executing Task1_obs_avoid_nav")

        if not self.client.wait_for_server(timeout=rospy.Duration(5)):
            rospy.logerr("Failed to connect to move_base server")
            return 'failed'
        
        goal = MoveBaseGoal()
        goal.target_pose = self.target_pose
        self.client.send_goal(goal)

        start_time = rospy.Time.now()
        last_state=None
        
        while not rospy.is_shutdown():
            # Check for preemption
            if self.preempt_requested():
                self.client.cancel_goal()
                return 'stopped'
                
            # Check current state
            current_state = self.client.get_state()
            
            if current_state != last_state:
                rospy.loginfo("Navigation state: %s", actionlib.GoalStatus.to_string(current_state))
                last_state = current_state

            # Handle final states
            if current_state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal reached!")
                return 'goal_reached'
            elif current_state in [actionlib.GoalStatus.PREEMPTED, actionlib.GoalStatus.ABORTED,
                                    actionlib.GoalStatus.REJECTED, actionlib.GoalStatus.RECALLED]:
                rospy.loginfo("Navigation failed with state: %s", actionlib.GoalStatus.to_string(current_state))
                return 'failed'
                
            # Check timeout
            if (rospy.Time.now() - start_time) > self.timeout:
                rospy.logwarn("Navigation timed out")
                self.client.cancel_goal()
                return 'failed'
                
            rospy.sleep(0.01)

        # If we exit while loop due to shutdown
        self.client.cancel_all_goals()
        return 'stopped'
    
class Task2_exploration(smach.State):
    def __init__(self,waypoint_list):
        smach.State.__init__(self,outcomes=['goal_reached','failed','stopped'])
        self.client=actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.waypoint_list=waypoint_list
        self.goal = MoveBaseGoal()
        self.goal.target_pose = PoseStamped()
        self.goal.target_pose.header.frame_id = "map"
        self.timeout = rospy.Duration(240)

    
    def execute(self,userdata):
        rospy.loginfo("Executing Task2_exploration")

        if not self.client.wait_for_server(timeout=rospy.Duration(5)):
            rospy.logerr("Failed to connect to move_base server")
            return 'failed'
        for waypoint in self.waypoint_list:

            self.goal.target_pose.pose.position.x = waypoint[0]
            self.goal.target_pose.pose.position.y = waypoint[1]
            self.goal.target_pose.pose.position.z = waypoint[2]
            self.goal.target_pose.pose.orientation.x = waypoint[3]
            self.goal.target_pose.pose.orientation.y = waypoint[4]
            self.goal.target_pose.pose.orientation.z = waypoint[5]
            self.goal.target_pose.pose.orientation.w = waypoint[6]
            self.client.send_goal(self.goal)
            rospy.loginfo("Sent waypoint number %s", self.waypoint_list.index(waypoint)+1)
            start_time = rospy.Time.now()
            last_state=None

            while not rospy.is_shutdown():

                # Check for preemption
                if self.preempt_requested():
                    self.client.cancel_goal()
                    return 'stopped'

                # Check current state
                current_state = self.client.get_state()

                if current_state != last_state:
                    rospy.loginfo("Navigation state: %s", actionlib.GoalStatus.to_string(current_state))
                    last_state = current_state

                # Handle final states
                if current_state == actionlib.GoalStatus.SUCCEEDED:
                    if waypoint == self.waypoint_list[-1]:
                        rospy.loginfo("Final goal reached!")
                        return 'goal_reached'
                    else:
                        rospy.loginfo(f"Waypoint number {self.waypoint_list.index(waypoint)+1} reached!" )
                        break
                elif current_state in [actionlib.GoalStatus.PREEMPTED, actionlib.GoalStatus.ABORTED,
                                        actionlib.GoalStatus.REJECTED, actionlib.GoalStatus.RECALLED]:
                    rospy.loginfo("Navigation failed with state: %s", actionlib.GoalStatus.to_string(current_state))
                    return 'failed'

                # Check timeout
                if (rospy.Time.now() - start_time) > self.timeout:
                    rospy.logwarn("Navigation timed out")
                    self.client.cancel_goal()
                    return 'failed'

                rospy.sleep(0.01)

            # If we exit while loop due to shutdown
        self.client.cancel_all_goals()

        return 'stopped'


class Task3_unlock_bridge(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['done', 'failed'])
        self.topic_name='/cmd_open_bridge'
        self.publisher=rospy.Publisher(self.topic_name,Bool,queue_size=10)
        self.ros_msg=Bool(True)
        self.timeout = 10  # seconds
        
    def wait_for_connection(self):
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < self.timeout:
            if self.publisher.get_num_connections() > 0:
                self.connected = True
                return True
            rospy.sleep(0.1)
        return False
    
    def execute(self,userdata):
        if not self.wait_for_connection():
            rospy.logerr("Failed to connect to subscribers for topic %s", self.topic_name)
            return 'failed'
        try:
            rospy.loginfo("Executing Task2_unlock_bridge")
            self.publisher.publish(self.ros_msg)
            rospy.loginfo("Published message to topic: %s", self.topic_name)
            rospy.sleep(0.05)  # Wait for a moment to ensure the message is received
            return 'done'
        except Exception as e:
            rospy.logerr("Failed to publish message: %s", str(e))
            return 'failed'



    


if __name__=="__main__":
    pass