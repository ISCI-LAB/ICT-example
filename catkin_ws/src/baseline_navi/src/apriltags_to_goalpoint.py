#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray
import tf2_geometry_msgs
import tf2_ros
import tf
from baseline_navi.srv import Stage_Totag, Stage_TotagResponse
import math


class ApriltagsToGoalPoint(object):
    def __init__(self):
        # Setup the node
        self.msg_received = False
        self.msg_tags = AprilTagDetectionArray()
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcast = tf.TransformBroadcaster()
        self.stage = False
        self.adjust_counter = 0
        self.last_msg = 0
        self.rate = rospy.Rate(5)
        self.goal = PoseStamped()
        # Setup the publisher and subscriber
        self.sub_tag = rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.tagCallback)
        self.motion_stage_srv = rospy.Service('baseline_navi/stage_totag', Stage_Totag, self.totag_stage_service_cb)

    def totag_stage_service_cb(self, totag_request):
        self.stage = True
        while self.stage:
            self.rate.sleep()
        return Stage_TotagResponse(
            self.goal.pose.position.x, self.goal.pose.position.y, self.goal.pose.position.z,
            self.goal.pose.orientation.x, self.goal.pose.orientation.y, self.goal.pose.orientation.z,
            self.goal.pose.orientation.w)

    def set_goal(self, translation, rotation, weight):
        self.goal.pose.position.x = translation.x * weight
        self.goal.pose.position.y = translation.y * weight
        self.goal.pose.position.z = translation.z * weight
        self.goal.pose.orientation.x = rotation.x
        self.goal.pose.orientation.y = rotation.y
        self.goal.pose.orientation.z = rotation.z
        self.goal.pose.orientation.w = rotation.w

    def tagCallback(self, msg_tags):
        if self.stage:
            self.msg_tags = msg_tags
            self.msg_received = True
        # added goal point pub code
            for tag in self.msg_tags.detections:
                if tag.id[0] == 1:
                    self.tf_broadcast.sendTransform((0.0, -0.0, 0.5),
                                      np.array([-0.5, 0.5, 0.5, 0.5]),
                                      rospy.Time.now(),
                                      'goal',
                                      'tag_1')

                    transform_goal = self.tf_buffer.lookup_transform(
                        'odom',
                        'goal',  # source frame
                        rospy.Time(0),  # get the tf at first available time
                        rospy.Duration(1.0))

                    if self.last_msg == 0:
                        self.last_msg = transform_goal.transform.translation.x

                    # For debug
                    # print("adjust_counter: {}, goal: [{}, {}, {}]".format(self.adjust_counter, transform_goal.transform.translation.x, transform_goal.transform.translation.y, y))

                    if math.sqrt((self.last_msg - transform_goal.transform.translation.x)**2) > 0.15:
                        print("Tag frame bias too much, skip detection.")
                        continue

                    self.last_msg = transform_goal.transform.translation.x
                    self.set_goal(transform_goal.transform.translation, transform_goal.transform.rotation, 1)
                    self.stage = False


if __name__ == '__main__':
    rospy.init_node('tagDetections_to_goalpoint_node', anonymous=False)
    apriltags_to_goal_point = ApriltagsToGoalPoint()
    rospy.spin()
