#!/usr/bin/env python3
import smach
import rospy
from states import Task1_obs_avoid_nav, Task2_exploration, Task3_move_to_bridge, Task4_unlock_bridge, Task5_choose_box


exploration_waypoint_list = [
    [19, -19, 0.0, 0.0, 0.0, -1.0, 0.1],
    [10, -19, 0.0, 0.0, 0.0, -1.0, 0.0],
    [10, -16, 0.0, 0.0, 0.0, 0.707, 0.707],
    [19, -16, 0.0, 0.0, 0.0, 0.0, 1.0],
    [19, -13, 0.0, 0.0, 0.0, 0.7, 0.7],
    [10, -13, 0.0, 0.0, 0.0, -1.0, 0.0],
    [10, -10,  0.0, 0.0, 0.0, 0.7, 0.7],
    [19, -10,  0.0, 0.0, 0.0, 0.0, 1.0],
    [19, -7,  0.0, 0.0, 0.0, 0.7, 0.7],
    [10, -7,  0.0, 0.0, 0.0, -1.0, 0.0],
    [10, -4,  0.0, 0.0, 0.0, 0.7, 0.7],
    [19, -4,  0.0, 0.0, 0.0, -1.0, 0.0]]

final_box_waypoint_list = []



def main():
    rospy.init_node('fsm_navigation')
    sm=smach.StateMachine(outcomes=['done'])
    with sm:
        # smach.StateMachine.add('TASK1_OBS_AVOID_NAV',Task1_obs_avoid_nav(),transitions={'goal_reached':'TASK2_EXPLORATION','failed':'done','stopped':'done'})
        # smach.StateMachine.add('TASK2_EXPLORATION',Task2_exploration(exploration_waypoint_list),transitions={'goal_reached':'TASK3_MOVE_TO_BRIDGE','failed':'done','stopped':'done'})
        # smach.StateMachine.add('TASK3_MOVE_TO_BRIDGE',Task3_move_to_bridge(),transitions={'goal_reached':'TASK4_UNLOCK_BRIDGE','failed':'done','stopped':'done'})
        smach.StateMachine.add('TASK4_UNLOCK_BRIDGE',Task4_unlock_bridge(),transitions={'done':'TASK5_MOVE_TO_FINAL','failed':'done'})
        smach.StateMachine.add('TASK5_MOVE_TO_FINAL',Task5_choose_box(final_box_waypoint_list),transitions={'done':'done','failed':'done'})
    # Execute SMACH plan
    sm.execute()
    
    # Wait for ctrl-c to stop the application
    rospy.spin()


if __name__=="__main__":
    main()
