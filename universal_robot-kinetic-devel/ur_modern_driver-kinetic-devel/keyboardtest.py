#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL

def all_close(goal, actual, tolerance):
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    robot = moveit_commander.RobotCommander()

    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher(
    '/move_group/display_planned_path',
    moveit_msgs.msg.DisplayTrajectory,queue_size=20)

    planning_frame = group.get_planning_frame()
    group_names = robot.get_group_names()
    print "***********Robot's current position is***********"
    pose = group.get_current_pose()
    print type(pose)
    print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.group_names = group_names
    
  def test_move(self):

    group = self.group
    group.set_goal_position_tolerance(0.1)
    group.set_goal_orientation_tolerance(0.1)
    group.set_named_target('home')
    group.go()
    current_pose = self.group.get_current_pose().pose
    print "1"
    print current_pose
    group.set_named_target('up')
    group.go()
    print "2"
    current_pose = self.group.get_current_pose().pose
    print current_pose
    group.set_named_target('home')
    group.go()
    print "3"
    current_pose = self.group.get_current_pose().pose
    print current_pose
   
  def key_board(self):
    group = self.group
    group.set_named_target('up')
    group.go()
    current_pose = self.group.get_current_pose().pose
    pose_goal = geometry_msgs.msg.Pose()
    print current_pose
    pose_goal = current_pose
    #print pose_goal
    print "****start control arm by keyboard****"
    
    while 1:
      order = raw_input()
      if order == 'w':
        pose_goal.position.z += 0.1    
      if order == 's':
        pose_goal.position.z -= 0.1
      if order == 'a':
	pose_goal.position.x += 0.1
        pose_goal.position.y += 0.1
        pose_goal.position.z += 0.1
      if order == 'd':
	pose_goal.position.x -= 0.1
        pose_goal.position.y -= 0.1
        pose_goal.position.z -= 0.1
      if order == 'e':
        break
      #pose_goal.position.z = 0.4
      print pose_goal
      group.set_goal_position_tolerance(0.1)
      group.set_goal_orientation_tolerance(0.1)
      group.set_pose_target(pose_goal)
      plan = group.go(wait=True)
      group.stop()
      group.clear_pose_targets()
      current_pose = self.group.get_current_pose().pose
      print current_pose
      print "****finis****"

def main():
  try:
    tutorial = MoveGroupPythonIntefaceTutorial()
    loop = 1
    while loop:
      print "please input your commander"
      commander = raw_input()
      if commander == 't':
        print "****test move****"
        tutorial.test_move()
      if commander == 'k':
        print "****key_board control****"
        tutorial.key_board()
        break
      if commander == 'e':
        print "****test end****"
        break

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

