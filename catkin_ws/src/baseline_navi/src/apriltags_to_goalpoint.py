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
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class ApriltagsToGoalPoint(object):
    def __init__(self):
        # Setup the node
        self.msg_received = False
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcast = tf.TransformBroadcaster()
        self.tag_id = None
        self.map_frame_id = rospy.get_param("/calculat_navigation_cmd/map_frame_id")
        # Setup the publisher and subscriber
        self.sub_tag = rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.tagCallback)
        self.motion_stage_srv = rospy.Service('baseline_navi/stage_totag', Stage_Totag, self.totag_stage_service_cb)
        self.client = actionlib.SimpleActionClient(
            "navigation_controller/send_goal",
            MoveBaseAction)
        self.client.wait_for_server()

    def totag_stage_service_cb(self, totag_request):
        self.tag_id = totag_request.tag_id
        if not self.msg_received:
            return Stage_TotagResponse(0, 0, 999, 0, 0, 0, 0)
        else:
            goal = self.coordinate_projection()
            if goal is not None:
                return Stage_TotagResponse(
                    goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
                    goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z,
                    goal.pose.orientation.w)
            else:
                return Stage_TotagResponse(0, 0, 999, 0, 0, 0, 0)

    def set_goal(self, translation, rotation, weight):
        goal = PoseStamped()
        goal.pose.position.x = translation.x * weight
        goal.pose.position.y = translation.y * weight
        goal.pose.position.z = 0.
        goal.pose.orientation.x = rotation.x
        goal.pose.orientation.y = rotation.y
        goal.pose.orientation.z = rotation.z
        goal.pose.orientation.w = rotation.w
        return goal

    def tagCallback(self, msg_tags):
        tag_id_check = False

        for tag in msg_tags.detections:
            if tag.id[0] == self.tag_id:
                tag_id_check = True
                break
        self.msg_received = tag_id_check

    def coordinate_projection(self):
        for timeout_counter in range(5):
            try:
                self.tf_broadcast.sendTransform(
                    (0.0, -0.0, 0.5),
                    np.array([-0.5, 0.5, 0.5, 0.5]),
                    rospy.Time.now(),
                    'goal',
                    'tag_{}'.format(self.tag_id))

                transform_goal = self.tf_buffer.lookup_transform(
                    self.map_frame_id,
                    'goal',  # source frame
                    rospy.Time(0),  # get the tf at first available time
                    rospy.Duration(1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr(e)
                continue

            goal = self.set_goal(transform_goal.transform.translation, transform_goal.transform.rotation, 1)
            return goal
        return None


if __name__ == '__main__':
    rospy.init_node('tagDetections_to_goalpoint_node', anonymous=False)
    apriltags_to_goal_point = ApriltagsToGoalPoint()
    rospy.spin()
