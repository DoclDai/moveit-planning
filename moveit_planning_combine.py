#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##
import datetime
import csv
import math
import time
import numpy as np
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
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
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

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=80)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def plan_ee_angle(self, scale=1):
    move_group = self.move_group

    waypoints = []
    wpose = move_group.get_current_pose().pose

    # xini = wpose.position.x
    # yini = wpose.position.y
    # zini = wpose.position.z

    wpose.orientation.w = scale * 0.0
    wpose.orientation.x = scale * -1.0
    wpose.orientation.y = scale * 0.0
    wpose.orientation.z = scale*0.0

    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction


  def plan_cartesian_path_ini_state(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through. If executing  interactively in a
    ## Python shell, set scale = 1.0.
    ##
    x_safe_reach = [0.0, -0.45, 0.805]
    scale = 1
    waypoints = []
    wpose = move_group.get_current_pose().pose
    # wpose.position.x = 0.55
    # wpose.position.y = 0.0
    # wpose.position.z = 0.805
    # wpose.orientation.w = 0.0
    # wpose.orientation.x = 1/(2)**0.5
    # wpose.orientation.y = 0.0
    # wpose.orientation.z = 1/(2)**0.5
    wpose.orientation.w = scale * 0.0
    wpose.orientation.x = scale * -1.0
    wpose.orientation.y = scale * 0.0
    wpose.orientation.z = scale*0.0
    # waypoints.append(copy.deepcopy(wpose))

    wpose.position.x = x_safe_reach[0]
    wpose.position.y = x_safe_reach[1] 
    wpose.position.z = x_safe_reach[2]
    waypoints.append(copy.deepcopy(wpose)) 

    
    xini = wpose.position.x
    yini = wpose.position.y
    zini = wpose.position.z

    # print(xini, yini, zini)

    # We want the Cartesian path to be interpolated at a resolution of 1 cm ???
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction
  
  def plan_cartesian_path_washing_state(self,  scale=1):
    xini = 0.55
    yini = 0.0
    zini = 0.805
    p = 100
    move_group = self.move_group
    wpose = move_group.get_current_pose().pose

    # Wash_bucket, sponge, palette location
    # x_safe_reach = [-0.3, -0.6, 0.5]
    x_safe_reach = [0.0, -0.45, 0.805]
    x_wash = [-0.552, -0.285, 0.0550 + 0.33]
    x_sponge = [-0.362, -0.302, 0.1275 + 0.33]
    x_palette = [-0.255, -0.304, 0.1050 + 0.33]
    x_safe_return =  [0.0, -0.45, 0.805]

    # print(x_safe_reach)
    waypoints = []

    # wpose.orientation.w = scale * 0.0
    # wpose.orientation.x = scale * -1.0
    # wpose.orientation.y = scale * 0.0
    # wpose.orientation.z = scale*0.0
    # # waypoints.append(copy.deepcopy(wpose))    

    # wpose.position.x = x_safe_reach[0]
    # wpose.position.y = x_safe_reach[1] 
    # wpose.position.z = x_safe_reach[2]
    # waypoints.append(copy.deepcopy(wpose))

    wpose.position.x = x_wash[0]
    wpose.position.y = x_wash[1]
    wpose.position.z = x_wash[2] + 0.1
    waypoints.append(copy.deepcopy(wpose))

    # for i in range(p):
    #   wpose.position.x = i/p *(x_safe_reach[0] - xini) + xini
    #   wpose.position.y = i/p *(x_safe_reach[1] - yini) + yini
    #   wpose.position.z = i/p *(x_safe_reach[2] - zini) + zini
    #   waypoints.append(copy.deepcopy(wpose))

    # Wash the brush
    for i in range(3):
      wpose.position.x = x_wash[0]
      wpose.position.y = x_wash[1] + 0.03
      wpose.position.z = x_wash[2]
      waypoints.append(copy.deepcopy(wpose))

      wpose.position.x = x_wash[0]
      wpose.position.y = x_wash[1]
      wpose.position.z = x_wash[2]
      waypoints.append(copy.deepcopy(wpose))

   # Lift the brush to avoid collision
    wpose.position.x = x_wash[0]
    wpose.position.y = x_wash[1]
    wpose.position.z = x_wash[2] + 0.1
    waypoints.append(copy.deepcopy(wpose))

    # Touch the sponge
    for i in range(2):
      wpose.position.x = x_sponge[0]
      wpose.position.y = x_sponge[1]
      wpose.position.z = x_sponge[2]
      waypoints.append(copy.deepcopy(wpose))

      wpose.position.x = x_sponge[0]
      wpose.position.y = x_sponge[1]
      wpose.position.z = x_sponge[2] - 0.05
      waypoints.append(copy.deepcopy(wpose))
    
    wpose.position.x = x_sponge[0]
    wpose.position.y = x_sponge[1]
    wpose.position.z = x_sponge[2]
    waypoints.append(copy.deepcopy(wpose))

    # Dip paint from the palette
    for i in range(2):
      wpose.position.x = x_palette[0]
      wpose.position.y = x_palette[1]
      wpose.position.z = x_palette[2]
      waypoints.append(copy.deepcopy(wpose))

      wpose.position.x = x_palette[0]
      wpose.position.y = x_palette[1]
      wpose.position.z = x_palette[2] - 0.05
      waypoints.append(copy.deepcopy(wpose))

    wpose.position.x = x_palette[0]
    wpose.position.y = x_palette[1]
    wpose.position.z = x_palette[2]
    waypoints.append(copy.deepcopy(wpose))

    # Go to safe return point first
    wpose.position.x = x_safe_return[0]
    wpose.position.y = x_safe_return[1]
    wpose.position.z = x_safe_return[2]
    waypoints.append(copy.deepcopy(wpose))

    # for i in range(p):
    #   wpose.position.x = -i/p *(x_palette[0] - xini)+x_palette[0]
    #   wpose.position.y = -i/p *(x_palette[1] - yini)+x_palette[1]
    #   wpose.position.z = -i/p *(x_palette[2] - zini)+x_palette[2]
    #   waypoints.append(copy.deepcopy(wpose))
  
    # wpose.position.x = xini
    # wpose.position.y = yini
    # wpose.position.z = zini
    # waypoints.append(copy.deepcopy(wpose))
    # wpose.orientation.x = 1/(2)**0.5
    # wpose.orientation.y = 0.0
    # wpose.orientation.z = 1/(2)**0.5
    # waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)
    # Note: We are just planning, not asking move_group to actually move the robot yet:
    
    jnt_with_no = []
    for i in range(len(plan.joint_trajectory.points)):
      jnt_with_no.append(plan.joint_trajectory.points[i].positions)

    current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M")
    jointfile = "/home/simuser/joint_file/Washing_trajectory_noHE" + current_time + ".csv"

    with open(jointfile, 'w') as csvfile:
      # creating a csv writer object
      csvwriter = csv.writer(csvfile)
      for i in range(len(jnt_with_no)):
        csvwriter.writerow(jnt_with_no[i])
    
    return plan, fraction
  
  def plan_cartesian_path_dipping_state(self,  scale=1):
    xini = 0.55
    yini = 0.0
    zini = 0.805
    p = 100
    move_group = self.move_group
    wpose = move_group.get_current_pose().pose

    # Wash_bucket, sponge, palette location
    x_safe_reach = [0.0, -0.45, 0.805]
    # x_wash = [-0.552, -0.285, 0.0550 + 0.33]
    # x_sponge = [-0.362, -0.302, 0.1275 + 0.33]
    x_palette = [-0.255, -0.304, 0.1250 + 0.33]
    x_safe_return =  [-0.2, -0.6, 0.5]

    # print(x_safe_reach)
    waypoints = []

    # wpose.orientation.w = scale * 0.0
    # wpose.orientation.x = scale * -1.0
    # wpose.orientation.y = scale * 0.0
    # wpose.orientation.z = scale*0.0
    # # waypoints.append(copy.deepcopy(wpose))

    # wpose.position.x = x_safe_reach[0]
    # wpose.position.y = x_safe_reach[1] 
    # wpose.position.z = x_safe_reach[2]
    # waypoints.append(copy.deepcopy(wpose)) 

    # Dip paint from the palette
    for i in range(2):
      wpose.position.x = x_palette[0]
      wpose.position.y = x_palette[1]
      wpose.position.z = x_palette[2]
      waypoints.append(copy.deepcopy(wpose))

      wpose.position.x = x_palette[0]
      wpose.position.y = x_palette[1]
      wpose.position.z = x_palette[2] - 0.05
      waypoints.append(copy.deepcopy(wpose))

    wpose.position.x = x_palette[0]
    wpose.position.y = x_palette[1]
    wpose.position.z = x_palette[2]
    waypoints.append(copy.deepcopy(wpose))

    # # Go to safe return point first
    # wpose.position.x = x_safe_return[0]
    # wpose.position.y = x_safe_return[1]
    # wpose.position.z = x_safe_return[2]
    # waypoints.append(copy.deepcopy(wpose))

    # for i in range(p):
    #   wpose.position.x = -i/p *(x_safe_return[0] - xini) + x_safe_return[0]
    #   wpose.position.y = -i/p *(x_safe_return[1] - yini) + x_safe_return[1]
    #   wpose.position.z = -i/p *(x_safe_return[2] - zini) + x_safe_return[2]
    #   waypoints.append(copy.deepcopy(wpose))
    wpose.position.x = x_safe_reach[0]
    wpose.position.y = x_safe_reach[1] 
    wpose.position.z = x_safe_reach[2]
    waypoints.append(copy.deepcopy(wpose)) 
  
    # wpose.position.x = xini
    # wpose.position.y = yini
    # wpose.position.z = zini
    # waypoints.append(copy.deepcopy(wpose))
    # wpose.orientation.x = 1/(2)**0.5
    # wpose.orientation.y = 0.0
    # wpose.orientation.z = 1/(2)**0.5
    # waypoints.append(copy.deepcopy(wpose))
    
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)
    # Note: We are just planning, not asking move_group to actually move the robot yet:
    
    jnt_with_no = []
    for i in range(len(plan.joint_trajectory.points)):
      jnt_with_no.append(plan.joint_trajectory.points[i].positions)

    current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M")
    jointfile = "/home/simuser/joint_file/Dipping_trajectory_noHE" + current_time + ".csv"

    with open(jointfile, 'w') as csvfile:
      # creating a csv writer object
      csvwriter = csv.writer(csvfile)
      for i in range(len(jnt_with_no)):
        csvwriter.writerow(jnt_with_no[i])
    
    return plan, fraction

  def csv_reading(self):
    # Read in the csv input here
    rows = []
    filename = "/home/simuser/ws_moveit/src/moveit_tutorials/doc/move_group_python_interface/scripts/action_color/actions_c4.csv"
    # reading csv file
    with open(filename, 'r') as csvfile:
      # creating a csv reader object
      csvreader = csv.reader(csvfile)
 
      # extracting each data row one by one
      for row in csvreader:
        rows.append(row)
 
      # get total number of rows
      print("Total no. of rows: %d"%(csvreader.line_num))
    return rows
  
  def brush_dynamics(self, width):
    self.width = width
    self.height = width/1.1667

  def plan_cartesian_path(self, rows, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through. If executing  interactively in a
    ## Python shell, set scale = 1.0.

    wpose = move_group.get_current_pose().pose

    xini = 0.55
    yini = 0.0
    zini = 0.805
    distance = 0.05 # Distance from initial position to the canvas vertically
    waypoints = []

    numPts=10  # number of points in Bezier Curve
    numPts_ends = 4 # number of extra points for improving stroke ends drawing
    t1 = np.array([i*1.0/numPts/(numPts_ends+1) for i in range(0,numPts_ends+2)])
    t2 = np.array([i*1.0/numPts for i in range(2,numPts-1)])
    t3 = np.array([i*1.0/numPts/(numPts_ends+1)+(numPts-1)*1.0/numPts for i in range(0,numPts_ends+2)])

    t = np.concatenate((t1, t2, t3), axis=0)
    jnt_with_no = []
    for strokenum in range(len(rows)):
      waypoints = []
      x0 = float(rows[strokenum][0])*0.3-0.15
      y0 = float(rows[strokenum][1])*0.2-0.1
      x1 = float(rows[strokenum][2])*0.3-0.15
      y1 = float(rows[strokenum][3])*0.2-0.1
      x2 = float(rows[strokenum][4])*0.3-0.15
      y2 = float(rows[strokenum][5])*0.2-0.1
      controlPts=[[x0,y0],[x1,y1],[x2,y2]] # control points
      #print(controlPts)
      B_x=(1-t)*((1-t)*controlPts[0][0]+t*controlPts[1][0])+t*((1-t)*controlPts[1][0]+t*controlPts[2][0])
      B_y=(1-t)*((1-t)*controlPts[0][1]+t*controlPts[1][1])+t*((1-t)*controlPts[1][1]+t*controlPts[2][1])
      # B_z=(1-t)*((1-t)*controlPts[0][2]+t*controlPts[1][2])+t*((1-t)*controlPts[1][2]+t*controlPts[2][2])
    #print("Waypose position = ",wpose.position)
      for k in range(len(t)):
        if k < numPts_ends+2:
          wpose.position.x = xini + distance + k*self.height/(numPts_ends+1)
          wpose.position.z = zini - B_y[k]
          wpose.position.y = yini + B_x[k]
          waypoints.append(copy.deepcopy(wpose))

        elif k > numPts + 2*numPts_ends - (numPts_ends+1):
          wpose.position.x = xini + distance + self.height - (k - (numPts + 2*numPts_ends - (numPts_ends+1)))*self.height/(numPts_ends+1)
          wpose.position.z = zini - B_y[k]
          wpose.position.y = yini + B_x[k]
          waypoints.append(copy.deepcopy(wpose))

        else:
          wpose.position.x = xini + distance + self.height
          wpose.position.z = zini - B_y[k]
          wpose.position.y = yini + B_x[k]
          waypoints.append(copy.deepcopy(wpose))

      wpose.position.x = xini
      wpose.position.y = yini
      wpose.position.z = zini
      waypoints.append(copy.deepcopy(wpose))

      #wpose.position.z += scale * 0.1  # Third move sideways (y)
      #waypoints.append(copy.deepcopy(wpose))

      # We want the Cartesian path to be interpolated at a resolution of 1 cm
      # which is why we will specify 0.01 as the eef_step in Cartesian
      # translation.  We will disable the jump threshold by setting it to 0.0,
      # ignoring the check for infeasible jumps in joint space, which is sufficient
      # for this tutorial.
      (plan, fraction) = move_group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold
      for i in range(len(plan.joint_trajectory.points)):
        trajline = [strokenum]
        trajline.extend(plan.joint_trajectory.points[i].positions)
        jnt_with_no.append(trajline)

    
    current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M")
    jointfile = "/home/simuser/joint_file/traj_color/ALL_Dynamic_c4_joint_trajectory" + current_time + ".csv"

    with open(jointfile, 'w') as csvfile:
      # creating a csv writer object
      csvwriter = csv.writer(csvfile)
      for i in range(len(jnt_with_no)):
        csvwriter.writerow(jnt_with_no[i])

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

    ## END_SUB_TUTORIAL


  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL


  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL



def main():
  try:
    print ""
    print "----------------------------------------------------------"
    print "Welcome to the MoveIt MoveGroup Python Interface Tutorial"
    print "----------------------------------------------------------"
    print "Press Ctrl-D to exit at any time"
    print ""
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
    raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()



    print "============ Press `Enter` to read the csv input..."
    raw_input()
    rows = tutorial.csv_reading()
    rows_unplanned = rows
    number_of_rows = len(rows)

    print "============ Press `Enter` to specify width of the stroke..."
    # raw_input()
    width = float(input('Width (meter):'))
    tutorial.brush_dynamics(width)

    print "============ Press `Enter` to move to the initial state ..."
    raw_input()
    cartesian_plan, fraction = tutorial.plan_cartesian_path_ini_state()
    # tutorial.display_trajectory(cartesian_plan)
    tutorial.execute_plan(cartesian_plan)

    # print "============ Press `Enter` to plan the wash & dip trajectory ..."
    # raw_input()
    # cartesian_plan, fraction = tutorial.plan_cartesian_path_washing_state()
    # rospy.loginfo('The washing state is executed')

    print "============ Press `Enter` to plan the dip trajectory ..."
    raw_input()
    cartesian_plan, fraction = tutorial.plan_cartesian_path_dipping_state()
    rospy.loginfo('The dipping state is executed')

    print "============ Press `Enter` to plan all trajectory ..."
    raw_input()
    cartesian_plan, fraction = tutorial.plan_cartesian_path(rows)


    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

## BEGIN_TUTORIAL
## .. _moveit_commander:
##    http://docs.ros.org/melodic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
## .. _DisplayTrajectory:
##    http://docs.ros.org/melodic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
## .. _RobotTrajectory:
##    http://docs.ros.org/melodic/api/moveit_msgs/html/msg/RobotTrajectory.html
##
## .. _rospy:
##    http://docs.ros.org/melodic/api/rospy/html/
## CALL_SUB_TUTORIAL imports
## CALL_SUB_TUTORIAL setup
## CALL_SUB_TUTORIAL basic_info
## CALL_SUB_TUTORIAL plan_to_joint_state
## CALL_SUB_TUTORIAL plan_to_pose
## CALL_SUB_TUTORIAL plan_cartesian_path
## CALL_SUB_TUTORIAL display_trajectory
## CALL_SUB_TUTORIAL execute_plan
## CALL_SUB_TUTORIAL add_box
## CALL_SUB_TUTORIAL wait_for_scene_update
## CALL_SUB_TUTORIAL attach_object
## CALL_SUB_TUTORIAL detach_object
## CALL_SUB_TUTORIAL remove_object
## END_TUTORIAL
