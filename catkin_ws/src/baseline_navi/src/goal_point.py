#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped

def goal_point():
    teleop = rospy.get_param("teleop", False)

    stage = rospy.get_param('stage', 1)
    stage = int(stage)

    goals = rospy.get_param('goals', [[3,4,1.57],[3,3,0]])
    goal_cmd = rospy.Publisher("goal", PoseStamped, queue_size = 1)
    goal_pub = PoseStamped()

    while not rospy.is_shutdown():
        stage = rospy.get_param('/goal_point/stage')
        stage = int(stage)

        if float(goals[stage-1][0]) != '':
            goal_pub.pose.position.x= float(goals[stage-1][0])
            goal_pub.pose.position.y= float(goals[stage-1][1])
            goal_pub.pose.orientation.z = float(1)
            goal_pub.pose.orientation.w = float(goals[stage-1][2])

            goal_pub.header.frame_id = 'base_link'
            goal_cmd.publish(goal_pub)
            
        else:
            pass

if __name__ == "__main__":    
    rospy.init_node('goal_point')
    goal_point()
    rospy.spin()
