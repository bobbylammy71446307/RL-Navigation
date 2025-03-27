#!/usr/bin/env python3

from dynamic_reconfigure.client import Client as DynamicReconfigureClient
import rospy
import smach
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String

way_point_1=[22.0,0.0]
way_point_2=[22.0,-21.0]

class IdleState(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['task_received'])
    
    def execute(self, userdata):
        rospy.loginfo("Idling")
        rospy.sleep(0.1)
        return 'task_received'

class Task1_obs_avoid_nav(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['goal_1_reached','final_goal_reached','failed','stopped'],input_keys=['target_pose'])
        self.client=actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.timeout = rospy.Duration(240)

    def execute(self,userdata):
        rospy.loginfo("Executing Task1_obs_avoid_nav")

        if not self.client.wait_for_server(timeout=rospy.Duration(5)):
            rospy.logerr("Failed to connect to move_base server")
            return 'failed'
        
        goal = MoveBaseGoal()
        goal.target_pose = userdata.target_pose
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
                if [userdata.target_pose.pose.position.x, userdata.target_pose.pose.position.y]== way_point_1:
                    userdata.target_pose.pose.position.x = way_point_2[0]
                    userdata.target_pose.pose.position.y = way_point_2[1]
                    rospy.loginfo("Goal 1 reached!")

                    return 'goal_1_reached'
                else:
                    rospy.loginfo("Goal reached!")
                    return 'final_goal_reached'
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
        


def main():
    rospy.init_node('fsm_navigation')

    sm=smach.StateMachine(outcomes=['done'])
    sm.userdata.target_pose = PoseStamped()
    sm.userdata.target_pose.header.frame_id = "map"

    sm.userdata.target_pose.pose.position.x = way_point_1[0]
    sm.userdata.target_pose.pose.position.y = way_point_1[1]
    sm.userdata.target_pose.pose.orientation.w = 1.0
    with sm:
        smach.StateMachine.add('IDLE',IdleState(),transitions={'task_received':'TASK1_OBS_AVOID_NAV'})
        smach.StateMachine.add('TASK1_OBS_AVOID_NAV',Task1_obs_avoid_nav(),transitions={'final_goal_reached':'done','goal_1_reached':'TASK1_OBS_AVOID_NAV','failed':'done','stopped':'done'})

    
    # Execute SMACH plan
    outcome = sm.execute()
    


if __name__=="__main__":
    try:
        main()
        #hello
    except rospy.ROSInterruptException:
        rospy.loginfo("FSM shutdown by user")
