#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import copy
import rospy
import moveit_commander
from std_msgs.msg import Bool, Int32
import math
import time
# import moveit_msgs.msg
# import geometry_msgs.msg
# from std_msgs.msg import String
# from moveit_commander.conversions import pose_to_list

TABLE_HEIGH = 0.35
PEN_RISE = 0.04


class Writing_Control():
    def __init__(self):
        self.eef_step = 0.00015
        argv = ['/home/yujun/catkin_ws/src/gpt_demo/src/writing.py', 'joint_states:=/qt_robot/joints/state']
        moveit_commander.roscpp_initialize(argv)
        try:
            rospy.init_node('writing', anonymous=True)
        except:
            pass
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("right_arm")
        self.plan = None
        self.waypoints = []
        self.wpose = 0

        self.signal_publisher = rospy.Publisher("/qt_executing_signal", Int32, queue_size=1)
        rospy.sleep(3)

            # We can get the name of the reference frame for this robot:
        planning_frame = self.group.get_planning_frame()
        print ("============ Reference frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = self.group.get_end_effector_link()
        print ("============ End effector: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print ("============ Robot Groups:", self.robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        print("============ Printing robot state")
        print("current pose: \n {}".format(self.group.get_current_pose().pose))
        print("current pose reference frame: {}".format(self.group.get_pose_reference_frame()))

        self.group.allow_replanning(True)
        self.group.set_pose_reference_frame("base_link")
        self.group.set_planning_time(5.0)
        self.group.clear_path_constraints()
        self.group.clear_pose_targets()

    def publish_signal(self, signal):
        if signal == "start":
            print("publishing start")
            self.signal_publisher.publish(1)  # Publish "start" signal
        elif signal == "finish":
            print("publishing finish")
            self.signal_publisher.publish(0)  # Publish "finish" signal
        elif signal == "pen_up":
            print("publishing pen_up")
            self.signal_publisher.publish(2)
        elif signal == "pen_down":
            print("publishing pen_down")
            self.signal_publisher.publish(3)
        elif signal == "clear_trajectory":
            print("publishing clear_trajectory")
            self.signal_publisher.publish(4)

    def writing_prepare_arm(self):
        self.group.set_start_state_to_current_state()
        self.group.set_position_target([0.18, -0.25, TABLE_HEIGH])
        self.plan = self.group.go(wait=True)
        self.group.set_position_target([0.20, -0.22, TABLE_HEIGH])
        self.plan = self.group.go(wait=True)
        print("Reached starting point")

    def pen_down(self):
        # get current pose
        # move to pen down position
        self.wpose.position.z = TABLE_HEIGH
        self.waypoints.append(copy.deepcopy(self.wpose))

    def pen_up(self):
        # get current pose
        # move to pen up position
        self.wpose.position.z = TABLE_HEIGH + PEN_RISE
        self.waypoints.append(copy.deepcopy(self.wpose))

    def writing_test(self):
        # generate waypoints
        self.waypoints.clear()
        self.wpose = self.group.get_current_pose().pose
        self.waypoints.append(copy.deepcopy(self.wpose))

        self.wpose.position.y += 0.1
        self.waypoints.append(copy.deepcopy(self.wpose))
        
        self.execute()


    def write_letter_F(self):
        vertical_line_length = 0.05
        horizontal_line_length = 0.028
        middle_line_offset = vertical_line_length / 4

        # Move to initial position
        self.waypoints.clear()
        self.wpose = self.group.get_current_pose().pose
        self.waypoints.append(copy.deepcopy(self.wpose))

        self.wpose.position.x += 0.055
        self.wpose.position.y += 0.03

        self.waypoints.append(copy.deepcopy(self.wpose))

        self.publish_signal("pen_up")
        self.execute()
        self.publish_signal("pen_down")      

        # generate waypoints
        self.waypoints.clear()
        self.wpose = self.group.get_current_pose().pose
        self.waypoints.append(copy.deepcopy(self.wpose))

        # Draw the vertical line down
        self.wpose.position.x -= vertical_line_length
        self.waypoints.append(copy.deepcopy(self.wpose))
        
        self.execute()

        # Move back to top
        self.waypoints.clear()
        self.wpose = self.group.get_current_pose().pose
        self.waypoints.append(copy.deepcopy(self.wpose))

        # Move back to top
        self.pen_up()
        self.wpose.position.x += vertical_line_length
        self.waypoints.append(copy.deepcopy(self.wpose))
        self.pen_down()

        self.publish_signal("pen_up")
        self.execute()
        self.publish_signal("pen_down")

        # Draw top horizontal line
        self.waypoints.clear()
        self.wpose = self.group.get_current_pose().pose
        self.waypoints.append(copy.deepcopy(self.wpose))

        self.wpose.position.y -= horizontal_line_length
        self.wpose.position.z += 0.003
        self.waypoints.append(copy.deepcopy(self.wpose))

        self.execute()

        # Move to middle horizontal line start

        self.waypoints.clear()
        self.wpose = self.group.get_current_pose().pose
        self.waypoints.append(copy.deepcopy(self.wpose))
        
        self.pen_up()
        self.wpose.position.x -= middle_line_offset
        self.wpose.position.y += horizontal_line_length - 0.005
        self.waypoints.append(copy.deepcopy(self.wpose))
        self.pen_down()
        self.publish_signal("pen_up")
        self.execute()
        self.publish_signal("pen_down")
        
        # Draw middle horizontal line
        self.waypoints.clear()
        self.wpose = self.group.get_current_pose().pose
        self.waypoints.append(copy.deepcopy(self.wpose))

        self.wpose.position.y -= horizontal_line_length  # Draw line
        self.waypoints.append(copy.deepcopy(self.wpose))

        self.execute()

        print("done")

    def write_letter_X(self):
        horizontal_length = 0.03  # Horizontal length for the X
        vertical_length = 0.05  # Vertical length for the X

        # Starting position for the first diagonal (top left corner)
        start_x = self.group.get_current_pose().pose.position.x
        start_y = self.group.get_current_pose().pose.position.y

        self.waypoints.clear()
        self.wpose = self.group.get_current_pose().pose
        self.waypoints.append(copy.deepcopy(self.wpose))

        self.pen_up()
        self.wpose.position.x += vertical_length
        self.wpose.position.y += horizontal_length
        self.waypoints.append(copy.deepcopy(self.wpose))
        self.pen_down()

        self.publish_signal("pen_up")
        self.execute()
        self.publish_signal("pen_down")

        self.waypoints.clear()
        self.wpose = self.group.get_current_pose().pose
        self.waypoints.append(copy.deepcopy(self.wpose))

        self.wpose.position.x -= vertical_length
        self.wpose.position.y -= horizontal_length
        self.waypoints.append(copy.deepcopy(self.wpose))
        self.execute()

        self.waypoints.clear()
        self.wpose = self.group.get_current_pose().pose
        self.waypoints.append(copy.deepcopy(self.wpose))

        self.pen_up()
        self.wpose.position.x = start_x + vertical_length + 0.015
        self.wpose.position.y = start_y
        self.waypoints.append(copy.deepcopy(self.wpose))
        self.pen_down()

        self.publish_signal("pen_up")
        self.execute()
        self.publish_signal("pen_down")

        self.waypoints.clear()
        self.wpose = self.group.get_current_pose().pose
        self.waypoints.append(copy.deepcopy(self.wpose))

        self.wpose.position.x -= vertical_length - 0.005
        self.wpose.position.y += horizontal_length + 0.005
        self.waypoints.append(copy.deepcopy(self.wpose))

        self.execute()

        print("done")

    def write_letter_R(self):
        vertical_length = 0.05  # Vertical length for the R
        segment_length = 0.025  # Horizontal length for the R
        tail_length = 0.025  # Length of the tail for the R

        # Starting position for the first vertical line (top left corner)
        start_x = self.group.get_current_pose().pose.position.x
        start_y = self.group.get_current_pose().pose.position.y

        self.waypoints.clear()
        self.wpose = self.group.get_current_pose().pose
        self.waypoints.append(copy.deepcopy(self.wpose))

        self.pen_up()
        self.wpose.position.x += vertical_length + 0.005
        self.wpose.position.y += segment_length + 0.05
        self.wpose.position.z += 0.03
        self.waypoints.append(copy.deepcopy(self.wpose))
        self.pen_down()

        self.publish_signal("pen_up")
        self.execute()
        self.publish_signal("pen_down")

        # First vertical line (top to bottom)
        self.waypoints.clear()
        self.wpose = self.group.get_current_pose().pose
        self.waypoints.append(copy.deepcopy(self.wpose))

        self.wpose.position.x -= vertical_length
        self.waypoints.append(copy.deepcopy(self.wpose))

        self.execute()

        # # Move to the start of curve
        self.waypoints.clear()
        self.wpose = self.group.get_current_pose().pose
        self.waypoints.append(copy.deepcopy(self.wpose))

        self.pen_up()
        self.wpose.position.x = start_x + vertical_length + 0.005
        self.waypoints.append(copy.deepcopy(self.wpose))
        self.pen_down()

        self.publish_signal("pen_up")
        self.execute()
        self.publish_signal("pen_down")

        # Draw the curve
        self.waypoints.clear()
        self.wpose = self.group.get_current_pose().pose
        self.waypoints.append(copy.deepcopy(self.wpose))


        # Middle horizontal segment (moving right)
        self.wpose.position.y -= segment_length * 2 / 3
        self.waypoints.append(copy.deepcopy(self.wpose))

        # Bottom left diagonal down segment
        self.wpose.position.y -= segment_length / 3
        self.wpose.position.x -= segment_length / 3
        self.waypoints.append(copy.deepcopy(self.wpose))
        self.wpose.position.x -= segment_length / 3
        self.waypoints.append(copy.deepcopy(self.wpose))
        self.wpose.position.y += segment_length / 3
        self.wpose.position.x -= segment_length / 3
        self.waypoints.append(copy.deepcopy(self.wpose))

        # Bottom horizontal segment (moving left)
        self.wpose.position.y += segment_length * 2 / 3 + 0.005
        self.waypoints.append(copy.deepcopy(self.wpose))

        self.wpose.position.x += 0.005
        self.waypoints.append(copy.deepcopy(self.wpose))

        # draw tail
        self.wpose.position.x -= tail_length + 0.003
        self.wpose.position.y -= tail_length + 0.003
        self.waypoints.append(copy.deepcopy(self.wpose))

        self.wpose.position.x -= tail_length / 3
        self.waypoints.append(copy.deepcopy(self.wpose))

        # Plan and execute the trajectory for 'R'
        self.execute()



    def write_letter_H(self):
        vertical_length = 0.05  # Vertical length for the H
        horizontal_length = 0.03  # Horizontal length for the H

        # Starting position for the first vertical line (top left corner)
        start_x = self.group.get_current_pose().pose.position.x
        start_y = self.group.get_current_pose().pose.position.y

        # Move to initial position
        self.waypoints.clear()
        self.wpose = self.group.get_current_pose().pose
        self.waypoints.append(copy.deepcopy(self.wpose))

        self.wpose.position.x += 0.055
        self.wpose.position.y += 0.03

        self.waypoints.append(copy.deepcopy(self.wpose))

        self.publish_signal("pen_up")
        self.execute()
        self.publish_signal("pen_down")      

        # First vertical line (top to bottom)
        self.waypoints.clear()
        self.wpose = self.group.get_current_pose().pose
        self.waypoints.append(copy.deepcopy(self.wpose))

        self.wpose.position.x -= vertical_length
        self.waypoints.append(copy.deepcopy(self.wpose))

        self.execute()

        # Move to the start of the horizontal line (middle of the first vertical line)
        self.waypoints.clear()
        self.wpose = self.group.get_current_pose().pose
        self.waypoints.append(copy.deepcopy(self.wpose))

        self.pen_up()
        self.wpose.position.x += vertical_length / 5 * 3
        self.waypoints.append(copy.deepcopy(self.wpose))
        self.pen_down()

        self.publish_signal("pen_up")
        self.execute()
        self.publish_signal("pen_down")

        # Draw the horizontal line
        self.waypoints.clear()
        self.wpose = self.group.get_current_pose().pose
        self.waypoints.append(copy.deepcopy(self.wpose))

        self.wpose.position.y -= horizontal_length
        self.waypoints.append(copy.deepcopy(self.wpose))

        self.execute()

        # Move to the start of the second vertical line (top right corner of the H)
        self.waypoints.clear()
        self.wpose = self.group.get_current_pose().pose
        self.waypoints.append(copy.deepcopy(self.wpose))

        self.pen_up()
        self.wpose.position.x = start_x + vertical_length - 0.005
        self.wpose.position.y -= 0.005
        self.wpose.position.z += 0.004
        self.waypoints.append(copy.deepcopy(self.wpose))
        self.pen_down()

        self.publish_signal("pen_up")
        self.execute()
        self.publish_signal("pen_down")

        # # Second vertical line (top to bottom)
        self.waypoints.clear()
        self.wpose = self.group.get_current_pose().pose
        self.waypoints.append(copy.deepcopy(self.wpose))

        self.wpose.position.x -= vertical_length * 6 / 7
        self.waypoints.append(copy.deepcopy(self.wpose))

        self.execute()

        print("done")

    def write_letter_Q(self):
        # Octagon dimensions
        vertical_edge = 0.025
        horizontal_edge = 0.01
        diagonal_edge = 0.02

        # Move to initial position
        self.waypoints.clear()
        self.wpose = self.group.get_current_pose().pose
        self.waypoints.append(copy.deepcopy(self.wpose))

        self.wpose.position.x += 0.038
        self.wpose.position.y += 0.040
        self.wpose.position.z += 0.05

        self.waypoints.append(copy.deepcopy(self.wpose))

        self.publish_signal("pen_up")
        self.execute()
        self.publish_signal("pen_down")     

        # Starting position for the octagon (top left corner)
        start_x = self.group.get_current_pose().pose.position.x
        start_y = self.group.get_current_pose().pose.position.y

        # Generate waypoints for the octagon
        self.waypoints.clear()
        self.wpose = self.group.get_current_pose().pose

        # Start with the left vertical edge
        self.wpose.position.x -= vertical_edge
        self.waypoints.append(copy.deepcopy(self.wpose))

        # bottom left diagonal
        self.wpose.position.x -= diagonal_edge / 2
        self.wpose.position.y -= diagonal_edge / 2
        self.waypoints.append(copy.deepcopy(self.wpose))

        # bottom horizontal edge
        self.wpose.position.y -= horizontal_edge
        self.waypoints.append(copy.deepcopy(self.wpose))

        # bottom right diagonal
        self.wpose.position.x += diagonal_edge / 2
        self.wpose.position.y -= diagonal_edge / 2
        self.waypoints.append(copy.deepcopy(self.wpose))

        # Right vertical edge
        self.wpose.position.x += vertical_edge
        self.waypoints.append(copy.deepcopy(self.wpose))

        # top right diagonal
        self.wpose.position.x += diagonal_edge / 2
        self.wpose.position.y += diagonal_edge / 2
        self.waypoints.append(copy.deepcopy(self.wpose))

        # top horizontal edge
        self.wpose.position.y += horizontal_edge
        self.waypoints.append(copy.deepcopy(self.wpose))

        # top left diagonal
        self.wpose.position.x -= diagonal_edge / 2
        self.wpose.position.y += diagonal_edge / 2
        self.waypoints.append(copy.deepcopy(self.wpose))

        # Plan and execute the octagon trajectory
        self.execute()

        # Drawing the tail of the letter Q
        self.waypoints.clear()
        self.wpose = self.group.get_current_pose().pose

        # Tail starting point (bottom right of the octagon)
        self.wpose.position.y = start_y - horizontal_edge - diagonal_edge / 2
        self.wpose.position.x = start_x - vertical_edge

        self.pen_up()
        self.waypoints.append(copy.deepcopy(self.wpose))
        self.pen_down()

        self.publish_signal("pen_up")
        self.execute()
        self.publish_signal("pen_down")

        # Tail end point
        tail_length = 0.03
        self.wpose.position.x -= tail_length / 2
        self.wpose.position.y -= tail_length / 2

        self.waypoints.append(copy.deepcopy(self.wpose))

        self.wpose.position.y -= tail_length / 2
        self.waypoints.append(copy.deepcopy(self.wpose))
        # Plan and execute the tail trajectory
        self.execute()

        print("done")

    def write_letter_S(self):
        # Abandoned

        # Define the dimensions for the segments
        segment_length = 0.025  # Length of each segment
        horizontal_segment_length = 0.015

        # Drawing the tail of the letter Q
        self.waypoints.clear()
        self.wpose = self.group.get_current_pose().pose

        # Tail starting point (bottom right of the octagon)
        self.wpose.position.x += segment_length * 1.25

        self.pen_up()
        self.waypoints.append(copy.deepcopy(self.wpose))
        self.pen_down()

        self.publish_signal("pen_up")
        self.execute()
        self.publish_signal("pen_down")

        # Starting position for the letter 'S' (top right)
        start_x = self.group.get_current_pose().pose.position.x
        start_y = self.group.get_current_pose().pose.position.y

        # Generate waypoints for the letter 'S'
        self.waypoints.clear()
        self.wpose = self.group.get_current_pose().pose

        # Top horizontal segment (moving left)
        self.wpose.position.x -= horizontal_segment_length
        self.waypoints.append(copy.deepcopy(self.wpose))

        # Top right diagonal down segment
        self.wpose.position.x -= segment_length / 3
        self.wpose.position.y += segment_length / 3
        self.waypoints.append(copy.deepcopy(self.wpose))
        self.wpose.position.y += segment_length / 3
        self.waypoints.append(copy.deepcopy(self.wpose))
        self.wpose.position.x += segment_length / 3
        self.wpose.position.y += segment_length / 3
        self.waypoints.append(copy.deepcopy(self.wpose))

        # Middle horizontal segment (moving right)
        self.wpose.position.x += horizontal_segment_length
        self.waypoints.append(copy.deepcopy(self.wpose))

        # Bottom left diagonal down segment
        self.wpose.position.x += segment_length / 3
        self.wpose.position.y += segment_length / 3
        self.waypoints.append(copy.deepcopy(self.wpose))
        self.wpose.position.y += segment_length / 3
        self.waypoints.append(copy.deepcopy(self.wpose))
        self.wpose.position.x -= segment_length / 3
        self.wpose.position.y += segment_length / 3
        self.waypoints.append(copy.deepcopy(self.wpose))

        # Bottom horizontal segment (moving left)
        self.wpose.position.x -= horizontal_segment_length
        self.waypoints.append(copy.deepcopy(self.wpose))

        # Plan and execute the trajectory for 'S'
        self.execute()

        print("done")

    def writing_end_arm(self):
        """
        x: 0.008657861640165741
        y: -0.19843352310414675
        z: 0.14804548207158294

        """
        # move to initial position
        self.waypoints.clear()
        self.wpose = self.group.get_current_pose().pose
        self.waypoints.append(copy.deepcopy(self.wpose))
        self.wpose.position.x = 0.17
        self.wpose.position.y = -0.17
        self.wpose.position.z = 0.28
        self.waypoints.append(copy.deepcopy(self.wpose))
        self.wpose.position.x = 0.0
        self.wpose.position.y = -0.18
        self.wpose.position.z = 0.15
        self.waypoints.append(copy.deepcopy(self.wpose))
        self.execute()


    def execute(self):
        # plan trajectory
        self.plan, fraction = self.group.compute_cartesian_path(
                                    self.waypoints,   # waypoints to follow
                                    self.eef_step,        # eef_step
                                    0.0,         # jump_threshold
                                    False)       # avoid_collisions
        
        # execute the plan
        self.group.execute(self.plan, True)
    def writing_execution(self, letter='H'):

        # Publish a signal on the topic
        self.publish_signal("start")
        # generate waypoints
        waypoints = []
        wpose = self.group.get_current_pose().pose
        waypoints.append(copy.deepcopy(wpose))
        
        print('letter: ', letter)
        if letter == 'F':
            self.write_letter_F()
        elif letter == 'X':
            self.write_letter_X()
        elif letter == 'H':
            self.write_letter_H()
        elif letter == 'Q':
            self.write_letter_Q()
        # elif letter == 'S':
        #     self.write_letter_S()
        elif letter == 'R':
            self.write_letter_R()
        else:
            print("Invalid letter")
        # self.write_letter_F()
        # self.write_letter_X()
        # self.write_letter_H()
        # self.write_letter_Q()
        # self.write_letter_S()

        self.publish_signal("finish")

if __name__ == "__main__":
    print("writing control test")
    control = Writing_Control()
    control.publish_signal("clear_trajectory")
    control.writing_prepare_arm()
    control.writing_execution('R')
    control.writing_prepare_arm()
    control.writing_execution('F')
    control.writing_prepare_arm()
    control.writing_execution('H')
    control.writing_prepare_arm()
    control.writing_execution('Q')
    control.writing_prepare_arm()
    control.writing_execution('X')
    # control.writing_test()
    