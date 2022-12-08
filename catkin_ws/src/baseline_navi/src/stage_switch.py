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
import numpy as np


class StageSwitch(object):
    def __init__(self):
        self.stage = 0
        self.color_map = {"red": 1,
                          "green": 2,
                          "blue": 3}
        self.map_frame_id = rospy.get_param("/calculat_navigation_cmd/map_frame_id")
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
        self.grasp_service(0)  # reset for arm and camera
        color = self.grasp_service(1).color  # grasp object
        tag_id = self.color_map[color]
        for i in range(2):
            goal_weight = 0.5 + 0.5 * i
            tag_found = False
            while not tag_found:
                coordinate = self.totag_service(tag_id)  # get apriltag's coordinate
                if coordinate.z >= 999:  # tag_id not found
                    current_pose, current_orn = self.get_robot_position()
                    desired_orn = transformations.euler_from_quaternion(current_orn)[2] + np.pi / 4
                    self.navigation_set_goal(current_pose, [0, 0, desired_orn])  # turn back
                    print("tag not found, continue spin")
                    continue
                else:
                    tag_found = True
                    self.navigation_set_goal([coordinate.x, coordinate.y, coordinate.z],
                                             [coordinate.rx, coordinate.ry, coordinate.rz, coordinate.rw],
                                             goal_weight)
        self.grasp_service(3)  # get back to original place
        self.navigation_set_goal([0, 0, 0], [0, 0, 0])

    def navigation_set_goal(self, translation, rotation, weight=1):
        self.goal.target_pose.pose.position.x = translation[0] * weight
        self.goal.target_pose.pose.position.y = translation[1] * weight
        self.goal.target_pose.pose.position.z = translation[2] * weight
        (x, y, z, w) = transformations.quaternion_from_euler(rotation[0], rotation[1], rotation[2]) if len(rotation) == 3 else rotation
        self.goal.target_pose.pose.orientation.x = x
        self.goal.target_pose.pose.orientation.y = y
        self.goal.target_pose.pose.orientation.z = z
        self.goal.target_pose.pose.orientation.w = w
        self.client.send_goal(self.goal)
        self.client.wait_for_result()

    def get_robot_position(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.map_frame_id, "base_link", rospy.Time())
            pose = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            orn = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
            return pose, orn
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(e)


if __name__ == "__main__":
    rospy.init_node('stage_switch')
    stage_switch = StageSwitch()
    rospy.spin()
