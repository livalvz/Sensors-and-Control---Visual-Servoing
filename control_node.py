#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose

def move_robot():
    # Initialize moveit_commander and rospy
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_ur3e_robot', anonymous=True)

    # Instantiate a RobotCommander object
    robot = moveit_commander.RobotCommander()

    # Instantiate a PlanningSceneInterface object
    scene = moveit_commander.PlanningSceneInterface()

    # Instantiate a MoveGroupCommander object
    group_name = "manipulator"  # Adjust if your group name is different
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Set maximum velocity and acceleration scaling factors
    move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_max_acceleration_scaling_factor(0.1)

    # Get the name of the reference frame
    planning_frame = move_group.get_planning_frame()
    rospy.loginfo(f"Planning frame: {planning_frame}")

    # Define the target pose
    pose_goal = Pose()
    pose_goal.position.x = 0.4  # Adjust these values as needed
    pose_goal.position.y = 0.0
    pose_goal.position.z = 0.3
    pose_goal.orientation.x = 0.0
    pose_goal.orientation.y = 0.0
    pose_goal.orientation.z = 0.0
    pose_goal.orientation.w = 1.0  # Neutral orientation

    # Set the target pose
    move_group.set_pose_target(pose_goal)

    # Plan and execute the motion
    success = move_group.go(wait=True)

    # Stop the robot after planning
    move_group.stop()

    # Clear the pose target
    move_group.clear_pose_targets()

    if success:
        rospy.loginfo("Robot moved to the target pose successfully.")
    else:
        rospy.logerr("Failed to move the robot to the target pose.")

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass

