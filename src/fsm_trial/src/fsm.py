#!/usr/bin/env python3
import smach
import rospy
i
import subprocess
from geometry_msgs.msg import PoseStamped
from states import Task1_obs_avoid_nav, IdleState


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
    
    # Wait for ctrl-c to stop the application
    rospy.spin()


if __name__=="__main__":
    main()
