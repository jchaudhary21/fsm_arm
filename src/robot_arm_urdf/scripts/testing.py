#! /usr/bin/env python3
import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import actionlib
import math
import json
import enquiries
import termios
import tty


# Stroring the node name, arm planning group and gripper planning group in variables
nodeName = "PicknPlace_Node"
armGroup = "arm_gp"
gripperGroup = "gripper_gp"

joint_I   = [-3.14, 3.14]
joint_II  = [-1.57, 1.57]
joint_III = [-3.14, 0]
joint_IV  = [-3.14, 0]
stepSize = 0.5 


class Ur5Moveit:
    def __init__(self):
        #  Initialize the node name : PicknPlace_Node
        rospy.init_node(nodeName, anonymous=True)
        # Initialize the planning groups arm_gp and gripper_gp
        self.arm_planning_group = armGroup
        self.gripper_planning_group = gripperGroup
        # Initialize the moveit_commander
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        # Initialize the  RobotCommander
        self._robot = moveit_commander.RobotCommander()
        # Initialize the PlanningSceneInterface
        self._scene = moveit_commander.PlanningSceneInterface()
        # Initialize the MoveGroupCommander for arm_planning_group and gripper_planning_group
        self.arm_group = moveit_commander.MoveGroupCommander(self.arm_planning_group)
        self.gripper_group = moveit_commander.MoveGroupCommander(
            self.gripper_planning_group
        )

        # Initialize the display_trajectory_publisher
        self._display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=1,
        )

        # Initialize the exectute_trajectory_client
        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            "execute_trajectory", moveit_msgs.msg.ExecuteTrajectoryAction
        )

        # Wait for the exectute_trajectory_client
        self._exectute_trajectory_client.wait_for_server()
        # Initialize the planning_frame, end effector link and group_names
        self._planning_frame = self.arm_group.get_planning_frame()
        self._eef_link = self.arm_group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        # Print the basic information of the Robotic ARM
        rospy.loginfo(
            "\033[94m" + "Planning Group:{}".format(self._planning_frame) + "\033[0m"
        )
        rospy.loginfo(
            "\033[94m" + "End Effector Link: {}".format(self._eef_link) + "\033[0m"
        )
        rospy.loginfo(
            "\033[94m" + "Group Names: {}".format(self._group_names) + "\033[0m"
        )
        rospy.loginfo("\033[94m" + " >>> Ur5Moveit init done." + "\033[0m")

    # Function to set the joint angles of the arm : "set_joint_angles"
    def set_joint_angles(self, arg_list_joint_angles):
        """
        Details : This function is used to set the joint angles of the arm.
        Arguments : arg_list_joint_angles : list of joint angles
        Return : None
        """

        
        # Get the current joint values of the arm
        list_joint_values = self.arm_group.get_current_joint_values()
        rospy.loginfo("\033[94m" + ">>> Current Joint Values:" + "\033[0m")
        rospy.loginfo(list_joint_values)
        # Set the joint values of the arm
        self.arm_group.set_joint_value_target("joint_1", 3.14)
        self.arm_group.plan()
        flag_plan = self.arm_group.go(wait=True)
        # Get the final joint values of the arm
        list_joint_values = self.arm_group.get_current_joint_values()
        rospy.loginfo("\033[94m" + ">>> Final Joint Values:" + "\033[0m")
        rospy.loginfo(list_joint_values)
            # Get the final pose of the arm
        pose_values = self.arm_group.get_current_pose().pose
        rospy.loginfo("\033[94m" + ">>> Final Pose:" + "\033[0m")
        rospy.loginfo(pose_values)

        if flag_plan == True:
                rospy.loginfo("\033[94m" + ">>> set_joint_angles() Success" + "\033[0m")

        else:
                rospy.logerr("\033[94m" + ">>> set_joint_angles() Failed." + "\033[0m")

        # except :
        #      pass 
        

    # Function to set the pose of the arm : "set_pose"




# Main Function
def main():


        # Creating Object
        ur5 = Ur5Moveit()

        angleList = [0, 0, 0, 0, 0]
        angleNeededList = []
        # Read JSON file from given location

        while not rospy.is_shutdown():

                for i in range (0,5):
                
                    angleNeededList.append(math.radians(angleList[i]))
                    ur5.set_joint_angles(angleNeededList)


if __name__ == "__main__":
    main()
