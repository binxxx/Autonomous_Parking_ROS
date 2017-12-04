#! /usr/bin/python

# this is some unit tests for my gazebo_interface

from __future__ import print_function
import rospy

import actionlib
import gazebo_interface.msg

def robot_updater():
  client = actionlib.SimpleActionClient('/gazebo_interface/model_updater', gazebo_interface.msg.executeAction)

  # Waits until the action server has started up and started
  # listening for goals.
  print ('Waiting for server...')
  client.wait_for_server()

  # Creates a goal to send to the action server.
  goal = gazebo_interface.msg.executeGoal()
  goal.dimension = 3
  goal.trajectory = [0, 1, 0, 1, 1, 0.785398, 1, 2, 1.570796, 0, 2, 0.785398, 0, 1, 0]

  # Sends the goal to the action server.
  print ('Sending the goal...')
  client.send_goal(goal)

  # Waits for the server to finish performing the action.
  print ('Waiting for result...')
  client.wait_for_result()

  # Prints out the result of executing the action
  print ('Done.')
  return client.get_result()

if __name__ == '__main__':
  try:
    rospy.init_node('gazebo_interface_UT')
    result = robot_updater()
    print (result.reason)
  except rospy.ROSInterruptException:
    print ("program interrupted before completion", file = sys.stderr)