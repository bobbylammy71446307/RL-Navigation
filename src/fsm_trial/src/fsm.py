#!/usr/bin/env python3
import smach
import rospy
import subprocess
from geometry_msgs.msg import PoseStamped
from states import Task1_obs_avoid_nav, IdleState, Task2_exploration, Task3_unlock_bridge
waypoint_list = [
    [22, -21, 0.0, 0.0, 0.0, -1.0, 0.1],
    [10, -19, 0.0, 0.0, 0.0, -1.0, 0.0],
    [10, -15, 0.0, 0.0, 0.0, 0.707, 0.707],
    [18, -15, 0.0, 0.0, 0.0, 0.0, 1.0],
    [18, -11, 0.0, 0.0, 0.0, 0.7, 0.7],
    [10, -11, 0.0, 0.0, 0.0, -1.0, 0.0],
    [10, -7,  0.0, 0.0, 0.0, 0.7, 0.7],
    [18, -7,  0.0, 0.0, 0.0, 0.0, 1.0],
    [18, -3,  0.0, 0.0, 0.0, 0.7, 0.7],
    [10, -3,  0.0, 0.0, 0.0, -1.0, 0.0]]

def main():
    rospy.init_node('fsm_navigation')

    sm=smach.StateMachine(outcomes=['done'])
    with sm:
        smach.StateMachine.add('IDLE',IdleState(),transitions={'task_received':'TASK1_OBS_AVOID_NAV'})
        smach.StateMachine.add('TASK1_OBS_AVOID_NAV',Task1_obs_avoid_nav(),transitions={'goal_reached':'TASK2_EXPLORATION','failed':'done','stopped':'done'})
        smach.StateMachine.add('TASK2_EXPLORATION',Task2_exploration(waypoint_list),transitions={'goal_reached':'TASK3_UNLOCK_BRIDGE','failed':'done','stopped':'done'})
        smach.StateMachine.add('TASK3_UNLOCK_BRIDGE',Task3_unlock_bridge(),transitions={'done':'done','failed':'done'})
    # Execute SMACH plan
    outcome = sm.execute()
    
    # Wait for ctrl-c to stop the application
    rospy.spin()


if __name__=="__main__":
    main()
