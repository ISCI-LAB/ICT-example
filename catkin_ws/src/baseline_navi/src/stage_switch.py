#!/usr/bin/python

import rospy
from baseline_navi.srv import StageChange, StageChangeResponse
from baseline_navi.srv import Stage_Grasp, Stage_GraspResponse
from baseline_navi.srv import Stage_Totag, Stage_TotagResponse
from navigation_controller.srv import command
from baseline_navi.msg import TaskStage
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import Int32

class StageSwitch(object):
    def __init__(self):
        self.stage = 0
        self.start_sub = rospy.Subscriber('locobot_motion/grasp_start', Int32, self.start_cb, queue_size=1)
        self.stage_pub = rospy.Publisher("baseline_navi/current_stage", TaskStage, queue_size = 1)
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.wait_for_service('pos_cmd')
        self.goal_service = rospy.ServiceProxy('pos_cmd', command)
        rospy.wait_for_service('baseline_navi/stage_grasp')
        self.grasp_service = rospy.ServiceProxy('baseline_navi/stage_grasp', Stage_Grasp)
        rospy.wait_for_service('baseline_navi/stage_totag')
        self.totag_service = rospy.ServiceProxy('baseline_navi/stage_totag', Stage_Totag)
    
    def start_cb(self, start_msg):
        if start_msg.data == 1:
            self.grasp_service(1)
            self.totag_service(True)
            self.grasp_service(3)

    def loop_service_confirm(self):
        goal_achived = False
        while not goal_achived:
            try:
                resp = self.goal_service(0, 0, 0, 0)
                goal_achived = resp.run_completed
            except rospy.ServiceException, e:
                print("Service call failed: %s", e)

        return goal_achived

    def pos_cmd_request(self, request_type, request_x, request_y, request_theta):
        try:
            resp = self.goal_service(request_type, request_x, request_y, request_theta)
        except rospy.ServiceException, e:
            print("Service call failed: %s", e)

    def send_goal(self, goal_x, goal_y, goal_theta):
        self.pos_cmd_request(2, goal_x, goal_y, 0)
        goal_achived = self.loop_service_confirm()

        if not goal_achived:
            print("Fail at type 2 VSLAM Navigation")
            return False

        self.pos_cmd_request(1, 0, 0, goal_theta)
        goal_achive = self.loop_service_confirm()

        return goal_achived

    

if __name__ == "__main__":
    rospy.init_node('stage_switch')
    stage_switch = StageSwitch()
    rospy.spin()
