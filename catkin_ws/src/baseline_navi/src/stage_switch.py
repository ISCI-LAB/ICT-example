#!/usr/bin/python

import rospy
import actionlib
from baseline_navi.srv import Stage_Grasp, Stage_GraspResponse
from baseline_navi.srv import Stage_Totag, Stage_TotagResponse
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from tf import transformations
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import Int32


class StageSwitch(object):
    def __init__(self):
        self.stage = 0
        self.start_sub = rospy.Subscriber('locobot_motion/grasp_start', Int32, self.start_cb, queue_size=1)
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.goal = MoveBaseGoal()
        rospy.wait_for_service('baseline_navi/stage_grasp')
        self.grasp_service = rospy.ServiceProxy('baseline_navi/stage_grasp', Stage_Grasp)
        rospy.wait_for_service('baseline_navi/stage_totag')
        self.totag_service = rospy.ServiceProxy('baseline_navi/stage_totag', Stage_Totag)
        self.client = actionlib.SimpleActionClient(
            "navigation_controller/send_goal",
            MoveBaseAction)
        self.client.wait_for_server()

    def start_cb(self, start_msg):
        if start_msg.data == 1:
            self.grasp_service(0)  # reset for arm and camera
            self.grasp_service(1)  # grasp object
            self.set_goal_euler([0, 0, 0],[0, 0, 0])  # turn back
            coordinate = self.totag_service(True)  # get apriltag's coordinate
            self.set_goal_quaternion(coordinate, 0.5)
            coordinate = self.totag_service(True)  # get apriltag's coordinate
            self.set_goal_quaternion(coordinate, 0.75)
            coordinate = self.totag_service(True)  # get apriltag's coordinate
            self.set_goal_quaternion(coordinate)
            self.grasp_service(3)  # get back to original place
            self.set_goal_euler([0, 0, 0], [0, 0, 0])

    def set_goal_euler(self, translation, rotation, weight=1):
        self.goal.target_pose.pose.position.x = translation[0] * weight
        self.goal.target_pose.pose.position.y = translation[1] * weight
        self.goal.target_pose.pose.position.z = translation[2] * weight
        (x, y, z, w) = transformations.quaternion_from_euler(rotation[0], rotation[1], rotation[2])
        self.goal.target_pose.pose.orientation.x = x
        self.goal.target_pose.pose.orientation.y = y
        self.goal.target_pose.pose.orientation.z = z
        self.goal.target_pose.pose.orientation.w = w
        self.client.send_goal(self.goal)
        self.client.wait_for_result()

    def set_goal_quaternion(self, coordinate, weight=1):
        self.goal.target_pose.pose.position.x = coordinate.x * weight
        self.goal.target_pose.pose.position.y = coordinate.y * weight
        self.goal.target_pose.pose.position.z = coordinate.z * weight
        self.goal.target_pose.pose.orientation.x = coordinate.rx
        self.goal.target_pose.pose.orientation.y = coordinate.ry
        self.goal.target_pose.pose.orientation.z = coordinate.rz
        self.goal.target_pose.pose.orientation.w = coordinate.rw
        self.client.send_goal(self.goal)
        self.client.wait_for_result()


if __name__ == "__main__":
    rospy.init_node('stage_switch')
    stage_switch = StageSwitch()
    rospy.spin()
