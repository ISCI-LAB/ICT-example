#!/usr/bin/python

import rospy
from baseline_navi.srv import StageChange, StageChangeResponse
from navigation_controller.srv import command
from baseline_navi.msg import TaskStage
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs

class StageSwitch(object):
    def __init__(self):
        self.stage = 0
        self.stage_srv = rospy.Service('baseline_navi/stage_request', StageChange, self.stage_service_callback)
        self.stage_pub = rospy.Publisher("baseline_navi/current_stage", TaskStage, queue_size = 1)
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.wait_for_service('pos_cmd')
        self.goal_service = rospy.ServiceProxy('pos_cmd', command)

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

    def stage_service_callback(self, request):
        print("Current stage: {}, {} request for switching task stage to {}.".format(self.stage, request.request_node, request.desired_stage))

        self.stage = request.desired_stage

        if self.stage == 2:
            self.pos_cmd_request(1, 0, 0, 2.0)
            self.loop_service_confirm()

        self.stage_publish()

        if self.stage == 4:
            self.pos_cmd_request(1, 0, 0, -2.0)
            self.loop_service_confirm()
            self.send_goal(0, 0, 0)
            self.stage = 1
            self.stage_publish()
            print("Current stage: 4, switch task stage to 1 for initialization.")

        return StageChangeResponse(True)

    def stage_publish(self):
        stage_msg = TaskStage()
        stage_msg.current_stage = self.stage

        if self.stage == 0:
            stage_msg.task_state = "stop"
        elif self.stage == 1:
            stage_msg.task_state = "grasping"
        elif self.stage == 2:
            stage_msg.task_state = "apriltag_navigating"
        elif self.stage == 3:
            stage_msg.task_state = "placing"
        elif self.stage == 4:
            stage_msg.task_state = "init_position"

        self.stage_pub.publish(stage_msg)

if __name__ == "__main__":
    rospy.init_node('stage_switch')
    stage_switch = StageSwitch()
    rospy.spin()
