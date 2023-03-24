#! /usr/bin/python

import math
import numpy as np

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from onrobot_rg_control.msg import OnRobotRGOutput

from geometry import *
from customFunctions import *


def jointsAndEndEffectorInformation():
    # Where the UR3E raw end effector is:
    print("\n============ End Effector Location of Wrist: ")
    pose = group.get_current_pose(end_effector_link="wrist_3_link").pose
    print(pose)
    print("\nWith Euler Angles:")
    print(euler_from_quaternion(pose.orientation.x,
          pose.orientation.y, pose.orientation.z, pose.orientation.w))

    # Where the gripper end effector is (roughly cos it assumes a fixed poition, not the actual position):
    print("\n============ End Effector Location of Left Finger Link: ")
    pose = group.get_current_pose(end_effector_link="rg2_l_finger_link").pose
    print(pose)
    print("\nWith Euler Angles:")
    print(euler_from_quaternion(pose.orientation.x,
          pose.orientation.y, pose.orientation.z, pose.orientation.w))

    joint_states_vector = group.get_joint_value_target()
    for i in joint_states_vector:
        # print("Joint "+ str(i) +" is at: "+ str(int(joint_states_vector[i])) + " radians")
        print("Joint " + str(i) + " is at: " + str(int(i)) + " radians")


def genCommand(char, command):
    max_force = 400
    max_width = 1100
    try:
        command.rGFR = 400
        command.rGWD = min(max_width, int(char))
        command.rCTR = 16
    except ValueError:
        pass
    return command


def moveGripper(position):  # enter position when callling function
    position = int(position)

    command = OnRobotRGOutput()
    command = genCommand(position, command)
    pubGripper.publish(command)
    rospy.sleep(0.01)
    print("\n\n\nMoving Gripper To Position:" + str(position) + "\n\n")


# desiredSetPosition = either home OR zeros
def movingRobotToSetPoses(desiredSetPosition):
    print("============ Going to '" + desiredSetPosition + "' pose\n")
    group.set_named_target(desiredSetPosition)

    # Now, we call the planner to compute the plan and visualize it if successful
    plan1 = group.plan()
    rospy.sleep(0.1)
    # To move execute in moveit, and therefore move in Gazebo
    group.go(wait=True)


# Joint being changed is from 0-5, and desiredJointPosition is in radians
def changeSpecificJoint(joint_being_changed, desiredJointPosition):
    joint_states_vector = group.get_joint_value_target()

    print("changing joint " + str(joint_being_changed) + " which is currently at: " +
          str(joint_states_vector[joint_being_changed]) + " to: " + str(desiredJointPosition))
    joint_states_vector[joint_being_changed] = desiredJointPosition
    group.set_joint_value_target(joint_states_vector)
    group.go(wait=True)


# all 6 entered variables are the desired joint position values in radians
def changeAllJoints(dj0, dj1, dj2, dj3, dj4, dj5):
    joint_states_vector = group.get_joint_value_target()
    print("Original Joints:")
    for i in joint_states_vector:
        # print(str(joint_states_vector[i]))
        print(str(i))

    joint_states_vector = [dj0, dj1, dj2, dj3, dj4, dj5]
    print("Change Joints to:")
    for i in joint_states_vector:
        # print(str(joint_states_vector[i]))
        print(str(i))

    group.set_joint_value_target(joint_states_vector)
    group.go(wait=True)


def cartesianPathPlanning():
    waypoints = []
    scale = 1
    pose = group.get_current_pose().pose
    pose.position.x -= scale * -0.2
    waypoints.append(copy.deepcopy(pose))
    pose.position.z += scale * -0.3
    waypoints.append(copy.deepcopy(pose))
    pose.position.z -= scale * -0.1
    pose.position.y -= scale * -1.5
    waypoints.append(copy.deepcopy(pose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm which is why we will specify 0.01 as
    # the eef_step in Cartesian translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient for most cases
    (plan, fraction) = group.compute_cartesian_path(
        waypoints,   # waypoints to follow
        0.01,        # eef_step
        0.0)         # jump_threshold
    print("\n\nWaypoints:")
    print(waypoints)

    print("Executing:")
    group.execute(plan, wait=True)


def jogEndEffector(x, y, z):

    # Where the UR3E raw end effector is:
    print("\n\n============ End Effector Location of Wrist: ")
    pose = group.get_current_pose(end_effector_link="wrist_3_link").pose

    print("\nPose Before:")
    print(pose)

    print("\nPose After:")
    newPose = translate_pose_msg(pose, x, y, z)
    print(newPose)

    group.set_pose_target(newPose)
    group.go(wait=True)


# RPY: to convert: 90deg, 0, -90deg, enter: 1.5707, -1.5707,1.5707
def setEndEffectorWithOrientation(x, y, z, roll, pitch, yaw):
    pose_target = geometry_msgs.msg.Pose()

    q = get_quaternion_from_euler(roll, pitch, yaw)
    pose_target.orientation.x = q[0]
    pose_target.orientation.y = q[1]
    pose_target.orientation.z = q[2]
    pose_target.orientation.w = q[3]

    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z
    group.set_pose_target(pose_target)
    group.go(wait=True)


# RPY: to convert: 90deg, 0, -90deg, enter: 1.5707, -1.5707,1.5707
def setEndEffectorOfFingerWithOrientation(x, y, z, roll, pitch, yaw):
    pose_target = geometry_msgs.msg.Pose()

    q = get_quaternion_from_euler(roll, pitch, yaw)
    pose_target.orientation.x = q[0]
    pose_target.orientation.y = q[1]
    pose_target.orientation.z = q[2]
    pose_target.orientation.w = q[3]

    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z
    pose_target.translate_pose_msg(0, 0, 0)

    pose_target = translate_pose_msg(pose_target, 0, 0, -0.2) # WHERE 0.2 IS THE OFFSET DISTANCE FROM FINGER TIP TO THE WRIST OF UR3e

    group.set_pose_target(pose_target)
    group.go(wait=True)



if __name__ == '__main__':
    try:

        # INITIALIZE GRIPPER:
        pubGripper = rospy.Publisher('OnRobotRGOutput', OnRobotRGOutput, queue_size=1)

        # INITIALIZE ROBOT AND MOVEIT:
        # First initialize moveit_commander and rospy.
        print("============ Starting tutorial setup")
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('gripperAndRobotPublisher', anonymous=True)
        # Instantiate a RobotCommander object.  This object is an interface to the robot as a whole.
        robot = moveit_commander.RobotCommander()
        # Instantiate a PlanningSceneInterface object.  This object is an interface to the world surrounding the robot.
        scene = moveit_commander.PlanningSceneInterface()
        # Instantiate a MoveGroupCommander object. This object is an interface to one group of joints. This interface can be used to plan and execute motions on the arm.
        group = moveit_commander.MoveGroupCommander("ur3e_group")
        group2 = moveit_commander.MoveGroupCommander("rg2_group")

        # We create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        # Optional:
        group.set_max_velocity_scaling_factor(1)
        group.set_max_acceleration_scaling_factor(1)
        group.set_planning_time(10)
        group.set_num_planning_attempts(3)
        group.allow_replanning(1)

        # GETTINGCURRENT POSITION/END EFFECTOR INFORMATION
        # jointsAndEndEffectorInformation()

        # MOVING ROBOT TO INPUTTED SET POSITION
        # movingRobotToSetPoses("home")
        # movingRobotToSetPoses("zeros")
        movingRobotToSetPoses("home_facing_down")
        # movingRobotToSetPoses("home_facing_forward")

        # MOVING GRIPPER TO INPUTTED POSITION
        # moveGripper(0)

        # MOVING SINGLE JOINT ON UR3E
        # changeSpecificJoint(1,-1)

        # MOVING ROBOT ARM TO INPUTTED VECTOR
        # changeAllJoints(0.1,0,0,0.5,1,1.1)
        # changeAllJoints(0,0,0,0,0,1)

        # SET END EFFECTOR (WRIST OF UR3e):
        # setEndEffectorWithOrientation(0.4,0.2,0.3,1.5707, -1.5707,1.5707)
        # SET END EFFECTOR (FINGER OF GRIPPER):
        # setEndEffectorOfFingerWithOrientation(0.4,0.25,0.3,1.5707, -1.5707,1.5707)

        # PERFORM CARTESIAN PATH PLANNING:
        # cartesianPathPlanning()
        
        # JOG END EFFECTOR
        # jogEndEffector(0.1, 0, 0) #X,Y,Z


    except rospy.ROSInterruptException:
        pass
