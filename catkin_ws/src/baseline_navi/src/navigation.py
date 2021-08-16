#!/usr/bin/env python

import rospy
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from pyrobot import Robot
import tf
from std_msgs.msg import UInt8
import sys
from baseline_navi.srv import StageChange, StageChangeResponse
from baseline_navi.msg import TaskStage
sys.path.append('~/low_cost_ws/src/pyrobot/examples/grasping')
# from locobot import Grasper

last_msg = ''
class LocobotController(object): 
    def __init__(self):
        self.stage = 0
        self.bot = Robot(
            "locobot",
            base_config={
                "base_controller": "ilqr",
                "base_planner": "none",
            }
        )
        self.goal = rospy.Subscriber('goal', PoseStamped, self.goal_cb, queue_size=1)
        self.stagecb = rospy.Subscriber('stage',UInt8,self.stage_cb,queue_size=1)
        rospy.wait_for_service('baseline_navi/stage_request')
        self.stage_service=rospy.ServiceProxy('baseline_navi/stage_request',StageChange)
    def stage_cb(self, stage_msg):
        # print(stage_msg)
        tmp = stage_msg.data

        if tmp == 2:
            self.stage = 2
            rospy.loginfo("stage => 2. goto apriltag")

    def goal_cb(self, goal_msg):
        global last_msg

        x = str(goal_msg.pose.position.x)
        y = str(goal_msg.pose.position.y)
        (r, p, yy) = tf.transformations.euler_from_quaternion([goal_msg.pose.orientation.x, goal_msg.pose.orientation.y, goal_msg.pose.orientation.z, goal_msg.pose.orientation.w])
        theta = str(yy)
        posn = np.asarray([x,y,theta], dtype=np.float64, order="C")

        if self.stage == 2:
            print(x,y,yy)
            if goal_msg.pose.position.y != last_msg:
                last_msg = goal_msg.pose.position.y

                rospy.loginfo("Go to the goal: [%s , %s].", x, y)
                self.go_to_relative(posn)
                self.bot.base.stop()
                rospy.loginfo("Achieve the goal.")

                try:
                    resp = self.stage_service(3, "navigation")

                    if resp.success :
                        print("success change stage to 3")
                        self.stage = 3
                        
                    return resp
                except rospy.ServiceException, e:
                    print("Service call failed: %s", e)

    def go_to_relative(self, posn):
        self.bot.base.go_to_relative(
            posn, use_map=False, close_loop=True, smooth=True
        )

if __name__ == "__main__":
    rospy.init_node('navigation')
    teleop = rospy.get_param("teleop", False) 
    locobot_controller = LocobotController()
    rospy.loginfo("Wait for inputting goal points.")
    rospy.spin()
