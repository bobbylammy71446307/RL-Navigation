#!/usr/bin/env python3

from dynamic_reconfigure.client import Client as DynamicReconfigureClient
import rospy
import smach
import smach_ros
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String


class IdleState(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['task_received','stopped'])
        self.task_sub = rospy.Subscriber('/current_task', String, self.task_cb)
        self.received_task = False
    
    def task_cb(self, msg):
        self.received_task = True
    def execute(self, userdata):
        rospy.loginfo("Idling")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.received_task:
                return 'task_received'
            rate.sleep()
        return 'stopped'

class Task1_obs_avoid_nav(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['goal_reached','failed','stopped'],input_keys=['target_pose'])
        self.client=actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.timeout = rospy.Duration(240)
    def execute(self,userdata):
        rospy.loginfo("Executing Task1_obs_avoid_nav")
        try:
            self.client.wait_for_server(timeout=rospy.Duration(5))  # Timeout after 5 seconds
        except rospy.ROSException as e:
            rospy.logerr("Failed to connect to move_base server: %s", e)
            return 'failed'
        goal = MoveBaseGoal()
        goal.target_pose = userdata.target_pose
        self.client.send_goal(goal)

        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            # Check for preemption
            if self.preempt_requested():
                self.client.cancel_goal()
                return 'stopped'
                
            # Check current state
            state = self.client.get_state()
            
            # Handle final states
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal reached!")
                return 'goal_reached'
            elif state in [actionlib.GoalStatus.PREEMPTED, actionlib.GoalStatus.ABORTED,
                          actionlib.GoalStatus.REJECTED, actionlib.GoalStatus.RECALLED]:
                rospy.loginfo("Navigation failed with state: %s", 
                            actionlib.GoalStatus.to_string(state))
                return 'failed'
                
            # Check timeout
            if (rospy.Time.now() - start_time) > self.timeout:
                rospy.logwarn("Navigation timed out")
                self.client.cancel_goal()
                return 'failed'
                
            rate.sleep()
            
        # If we exit while loop due to shutdown
        self.client.cancel_all_goals()
        return 'stopped'
        


def main():
    rospy.init_node('fsm_navigation')

    sm=smach.StateMachine(outcomes=['done'])
    sm.userdata.target_pose = PoseStamped()
    sm.userdata.target_pose.header.frame_id = "map"

    sm.userdata.target_pose.pose.position.x = 5.0
    sm.userdata.target_pose.pose.position.y = 0.0
    sm.userdata.target_pose.pose.orientation.w = 1.0
    with sm:
        smach.StateMachine.add('IDLE',IdleState(),transitions={'task_received':'TASK1_OBS_AVOID_NAV','stopped':'done'})
        smach.StateMachine.add('TASK1_OBS_AVOID_NAV',Task1_obs_avoid_nav(),transitions={'goal_reached':'IDLE','failed':'done','stopped':'done'})

    
    # Execute SMACH plan
    outcome = sm.execute()
    


if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("FSM shutdown by user")