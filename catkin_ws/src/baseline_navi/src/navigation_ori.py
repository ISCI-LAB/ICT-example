#!/usr/bin/env python

import rospy
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from pyrobot import Robot

last_msg = ''

def navigate():
    rospy.init_node('navigation')
    rospy.loginfo("Wait for inputting goal points.")
    goal = rospy.Subscriber('goal', PoseStamped, goal_cb, queue_size=1)
    rospy.spin()
 
def goal_cb(goal_msg):
    global last_msg

    x = str(goal_msg.pose.position.x)
    y = str(goal_msg.pose.position.y)
    theta = str(goal_msg.pose.orientation.w)
    posn = np.asarray([x,y,theta], dtype=np.float64, order="C")

    if goal_msg.pose.position.y != last_msg:
        last_msg = goal_msg.pose.position.y

        rospy.loginfo("Go to the goal: [%s , %s].", x, y)
        go_to_relative(posn)
        rospy.loginfo("Achieve the goal.")

def go_to_relative(posn):
    bot = Robot(
        "locobot",
        base_config={
            "base_controller": "ilqr",
            "base_planner": "none",
        },
    )
    bot.base.go_to_relative(
        posn, use_map=False, close_loop=True, smooth=True
    )

if __name__ == "__main__":
    teleop = rospy.get_param("teleop", False) 
    navigate()
