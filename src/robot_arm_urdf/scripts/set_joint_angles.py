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
stepSize  = 0.4 
dicAngle = {"joint_1": [-3.14, 3.14], "joint_2": [-1.57 , 1.57], "joint_3": [-3.14, 0], "joint_4": [-3.14, 0], "joint_5": [0, 3.14]}



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
        rospy.loginfo( "Planning Group: "    + "\033[94m" + "{}".format(self._planning_frame) + "\033[0m"
        )
        rospy.loginfo( "End Effector Link: " + "\033[94m" + "{}".format(self._eef_link) + "\033[0m"
        )
        rospy.loginfo( "Group Names:"        + "\033[94m" + "{}".format(self._group_names) + "\033[0m"
        )
        rospy.loginfo("\033[94m" + "Ur5Moveit init done." + "\033[0m")

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
        self.arm_group.set_joint_value_target(arg_list_joint_angles)
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

        # return


        

    # Function to set the pose of the arm : "set_pose"
    def gripper_joint_angles(self, arg_list_joint_angles):
        """
        Details : This function is used to set the joint angles of the gripper.
        Arguments : arg_list_joint_angles : list of joint angles
        Return : None
        """

        # Get the current joint values of the gripper
        list_joint_values = self.gripper_group.get_current_joint_values()
        rospy.loginfo("\033[94m" + ">>> Current Joint Values:" + "\033[0m")
        rospy.loginfo(list_joint_values)

        # Set the joint values of the gripper
        self.gripper_group.set_joint_value_target(arg_list_joint_angles)
        self.gripper_group.plan()
        flag_plan = self.gripper_group.go(wait=True)

        if flag_plan == True:
            rospy.loginfo("\033[94m" + ">>> set_joint_angles() Success" + "\033[0m")

        else:
            rospy.logerr("\033[94m" + ">>> set_joint_angles() Failed." + "\033[0m")

        return flag_plan
    
    def keypress_jointAngle(self, joint_name, index, sign) :
            
    
        
        list_joint_values = self.arm_group.get_current_joint_values()
        
        joint_angle = round(list_joint_values[index], 2) 

        if sign == 1 :
            joint_angle = joint_angle + stepSize 
        elif sign == 0 :
            joint_angle = joint_angle - stepSize 


        if (dicAngle.get(joint_name)[0] < joint_angle and dicAngle.get(joint_name)[1] > joint_angle ) :
            
            self.arm_group.set_joint_value_target(joint_name, joint_angle)
            self.arm_group.plan()
            flag_plan = self.arm_group.go()
            list_joint_values = self.arm_group.get_current_joint_values()
            pose_values = self.arm_group.get_current_pose().pose
            



    # Function to set the pose of the arm : "set_pose"
    def get_continuous_keypress(self):
        original_terminal_settings = termios.tcgetattr(sys.stdin)
        
        padding = " " * 27 
        print (f'{padding}\033[35;1mROBOTIC ARM CONTROLLER\033[0m')
        print("\n")
        padding = " " * 5 
        print (f'{padding}\033[33;1mTo move\033[0m')
        print ("\033[90;1mBase Link :\033[0m" , "\033[1;37mpress x / c\033[0m")
        print ("\033[90;1mJoint I   :\033[0m" , "\033[1;37mpress w / s\033[0m")
        print ("\033[90;1mJoint II  :\033[0m",  "\033[1;37mpress e / d\033[0m")
        print ("\033[90;1mJoint III :\033[0m",  "\033[1;37mpress r / f\033[0m")
        print ("\033[90;1mJoint IV  :\033[0m",  "\033[1;37mpress g / h\033[0m")
        print("\n\n")
        print("\033[91mPress keys (Ctrl + C + Enter to exit):\033[0m")

        try:
            tty.setcbreak(sys.stdin.fileno())

            while not rospy.is_shutdown():
                char = sys.stdin.read(1)


               
                # Check if the character is not empty and not a new line (Enter)
                if char and char != '\n':
                    
                    
                    if char == 'x' or char == 'c' :
                        if char == 'x' :
                            sign = 1
                        else :
                            sign = 0 
                        self.keypress_jointAngle("joint_1", 0, sign)


                    if char == 'g' or char == 'h' :
                        if char == 'g' :
                            sign = 1
                        else :
                            sign = 0 
                        self.keypress_jointAngle("joint_5", 4, sign)


                    if char == 'w' or char == 's' :
                        if char == 'w' :
                            sign = 1
                        else :
                            sign = 0 
                        self.keypress_jointAngle("joint_2", 1, sign)


                    if char == 'e' or char == 'd' :
                        if char == 'e' :
                            sign = 1
                        else :
                            sign = 0 
                        self.keypress_jointAngle("joint_3", 2, sign)


                    if char == 'r' or char == 'f' :
                        if char == 'r' :
                            sign = 1
                        else :
                            sign = 0 
                        self.keypress_jointAngle("joint_4", 3, sign)



        except KeyboardInterrupt:
            # Handle KeyboardInterrupt (Ctrl+C) to exit the loop gracefully
            pass
        finally:
            # Restore the original terminal settings
            print("Exiting...")
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_terminal_settings)



    # Function to set the pose of the arm : "set_pose"
    def __del__(self):
        """
        Details : This function is used to delete the object of class Ur5Moveit.
        Arguments : None
        Return : None
        """
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("\033[94m" + "Object of class Ur5Moveit Deleted." + "\033[0m")


def print_colored(text, color_code):
    print(f"\033[{color_code}m{text}\033[0m")


# Main Function
def main():
    text = "FSM - IITD-AIA Foundation For Smart Manufacturing"
    width = 79
    middle = math.floor(width / 2)
    padding = middle - len(text) // 2
    print(
        f"{' ' * padding}\033[1m\033[37m\033[40m\033[18m{text}\033[0m{' ' * padding}\n"
    )
    options = [
        "1. Automatic Pick and Place",
        "2. Manual COntroller using KeyBoard",
        "3. About Project and Dependencies",
    ]
    choice = enquiries.choose("Choose one of these options: \n", options)

    if choice == "1. Automatic Pick and Place":
        print(choice[3:])

        # Creating Object
        ur5 = Ur5Moveit()

        # Read JSON file from given location
        with open(
            "/home/jc/catkin_ws/src/robot_arm_urdf/scripts/angle_data.json", "r"
        ) as f:
            data = json.load(f)

        # Initialize the lists
        """
        GroupTypeList     : List of group types
        GripperStateList  : List of gripper states           
        PacketTypeList    : List of packet types      
        ActionPerformList : List of actions to be performed
        AngleNeededList   : List of angles needed to be set
                         
        """
        groupTypeList = ["JOINTS_Positions", "Gripper"]
        gripperStateList = ["OPEN", "CLOSE"]
        packetTypeList = ["Green_Yellow_Packets", "Blue_Red_Packets"]
        actionPerformList = ["PICK", "INBW", "PLACE"]
        angleNeededList = []

        while not rospy.is_shutdown():
            for packetType in packetTypeList:
                for actionPerform in actionPerformList:
                    for angleIndex in range(5):
                        angleRadian = math.radians(
                            data[groupTypeList[0]][packetType][actionPerform][
                                "Angle_Needed"
                            ][angleIndex]
                        )
                        angleNeededList.append(angleRadian)
                    ur5.set_joint_angles(angleNeededList)
                    angleNeededList = []

                    if actionPerform == "PICK":
                        for angleIndex in range(8):
                            angleRadian = math.radians(
                                data[groupTypeList[1]][gripperStateList[1]][
                                    "Angle_Needed"
                                ][angleIndex]
                            )
                            angleNeededList.append(angleRadian)
                        ur5.gripper_joint_angles(angleNeededList)
                        angleNeededList = []
                        rospy.sleep(2)

                    elif actionPerform == "PLACE":
                        for angleIndex in range(8):
                            angleRadian = math.radians(
                                data[groupTypeList[1]][gripperStateList[0]][
                                    "Angle_Needed"
                                ][angleIndex]
                            )
                            angleNeededList.append(angleRadian)
                        ur5.gripper_joint_angles(angleNeededList)
                        angleNeededList = []

            print("Task Completed")
            break

    elif choice == "3. About Project and Dependencies":
        """
        This is the about section of the project.
        """

        print("About: ")
        print_colored("Pick and Place using UR5 Robotic ARM", "94")  # Blue
        print("Created By: ")
        print_colored("JAYESH CHAUDHARY / jayeshchaudhary21jan@gmail.com", "94")  # Blue
        print("")
        print_colored(
            "This project is an internship project created by Jayesh Chaudhary during an internship at FSM - IITD-AIA Foundation For Smart Manufacturing under the guidance of KRRISH JINDAL.",
            "97",
        )  # Bright Black
        print("")
        print_colored("Description:", "92")  # Green
        print("    To run this project, the following tech stack is needed:")
        print_colored("        -> ROS Noetic", "92")  # Green
        print_colored("        -> Moveit", "92")  # Green
        print_colored("        -> Python 3.8", "92")  # Green
        print_colored("        -> Ubuntu 20.04", "92")  # Green
        print("")
        print_colored(
            "This project is created using UR5 Robotic Arm URDF and Moveit. It uses Moveit to plan the path and then execute the path and python script to call the Moveit API and to read the JSON file which contains the angles needed to be set for the given task.\n",
            "97",
        )  # Bright Black



    elif choice == "2. Manual COntroller using KeyBoard" :

 



        ur5 = Ur5Moveit()
        ur5.get_continuous_keypress()


if __name__ == "__main__":
    main()
