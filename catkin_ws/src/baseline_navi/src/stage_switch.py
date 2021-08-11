#!/usr/bin/python3

import rospy
from baseline_navi.srv import StageChange, StageChangeResponse
from baseline_navi.msg import TaskStage

class StageSwitch(object):
    def __init__(self):
        self.stage = 0
        self.stage_srv = rospy.Service('baseline_navi/stage_request', StageChange, self.stage_service_callback)
        self.stage_pub = rospy.Publisher("baseline_navi/current_stage", TaskStage, queue_size = 1)

    def stage_service_callback(self, request):
        print(f"Current stage: {self.stage}, {request.request_node} request for switching task stage to {request.desired_stage}.")

        self.stage = request.desired_stage
        stage_msg = TaskStage()
        stage_msg.current_stage = self.stage

        if self.stage == 1:
            stage_msg.task_state = "grasping"
        elif self.stage == 2:
            stage_msg.task_state = "navigating"
        elif self.stage == 3:
            stage_msg.task_state = "placing"

        self.stage_pub.publish(stage_msg)

        return StageChangeResponse(True)

if __name__ == "__main__":
    rospy.init_node('stage_switch')
    stage_switch = StageSwitch()
    rospy.spin()
