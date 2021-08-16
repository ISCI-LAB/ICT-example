#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray
import tf2_geometry_msgs
import tf2_ros

class ApriltagsToGoalPoint(object):
    def __init__(self):
        # Setup the node
        self.msg_received = False
        self.msg_tags = AprilTagDetectionArray()
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Setup the publisher and subscriber
        self.sub_tag = rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.tagCallback)
        self.goal_pub = rospy.Publisher("goal", PoseStamped, queue_size = 1)

    def tagCallback(self, msg_tags):
        goal_pose = PoseStamped()
        self.msg_tags = msg_tags
        self.msg_received = True

        # added goal point pub code
        for tag in self.msg_tags.detections:
            
            if tag.id[0] == 1:
                transform = self.tf_buffer.lookup_transform("base_link",
                                   tag.pose.header.frame_id, #source frame
                                   rospy.Time(0), #get the tf at first available time
                                   rospy.Duration(1.0))
                pose_transformed = tf2_geometry_msgs.do_transform_pose(tag.pose.pose, transform)
                goal_pose.pose.position.x = pose_transformed.pose.position.x
                goal_pose.pose.position.y = pose_transformed.pose.position.y
                
                goal_pose.pose.orientation.x = pose_transformed.pose.orientation.x
                goal_pose.pose.orientation.y = pose_transformed.pose.orientation.y
                goal_pose.pose.orientation.z = pose_transformed.pose.orientation.z
                goal_pose.pose.orientation.w = pose_transformed.pose.orientation.w
                goal_pose.header.frame_id = 'base_link'

                # For debug
                # print("pose_transformed:\n", pose_transformed.pose)
                print("goal_pose:\n", goal_pose)

                self.goal_pub.publish(goal_pose)

if __name__ == '__main__':
    rospy.init_node('tagDetections_to_goalpoint_node', anonymous=False)
    apriltags_to_goal_point = ApriltagsToGoalPoint()
    rospy.spin()
