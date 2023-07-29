#! /usr/bin/env python3
import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import actionlib


nodeName="PicknPlace_Node"
armGroup="arm_gp"
gripperGroup="gripper_gp"


class Ur5Moveit:
    # Constructor
    def __init__(self):
        rospy.init_node(nodeName, anonymous=True)
        self._arm_planning_group = armGroup
        self._gripper_planning_group = gripperGroup
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._arm_group = moveit_commander.MoveGroupCommander(self._arm_planning_group)
        self._gripper_group = moveit_commander.MoveGroupCommander(self._gripper_planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=1,
        )
        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            "execute_trajectory", moveit_msgs.msg.ExecuteTrajectoryAction
        )
        self._exectute_trajectory_client.wait_for_server()
        self._planning_frame = self._arm_group.get_planning_frame()
        self._eef_link = self._arm_group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

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

    def set_joint_angles(self, arg_list_joint_angles):
        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo("\033[94m" + ">>> Current Joint Values:" + "\033[0m")
        rospy.loginfo(list_joint_values)
        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)
        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo("\033[94m" + ">>> Final Joint Values:" + "\033[0m")
        rospy.loginfo(list_joint_values)
        pose_values = self._group.get_current_pose().pose
        rospy.loginfo("\033[94m" + ">>> Final Pose:" + "\033[0m")
        rospy.loginfo(pose_values)

        if flag_plan == True:
            rospy.loginfo("\033[94m" + ">>> set_joint_angles() Success" + "\033[0m")

        else:
            rospy.logerr("\033[94m" + ">>> set_joint_angles() Failed." + "\033[0m")

        return flag_plan