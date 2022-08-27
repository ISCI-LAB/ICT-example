#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray
import tf2_geometry_msgs
import tf2_ros
import tf
from baseline_navi.msg import TaskStage
from navigation_controller.srv import command
from baseline_navi.srv import StageChange
from baseline_navi.srv import Stage_Totag, Stage_TotagResponse
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import time
import math

class ApriltagsToGoalPoint(object):
    def __init__(self):
        # Setup the node
        self.msg_received = False
        self.msg_tags = AprilTagDetectionArray()
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcast = tf.TransformBroadcaster()
        self.stage = 0
        self.adjust_counter = 0
        self.last_msg = 0
        self.goal = MoveBaseGoal()
        # Setup the publisher and subscriber
        self.sub_tag = rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.tagCallback)
        self.stage_sub = rospy.Subscriber('baseline_navi/current_stage', TaskStage, self.stage_cb, queue_size = 1)
        self.motion_stage_srv = rospy.Service('baseline_navi/stage_totag', Stage_Totag, self.totag_stage_service_cb)
        rospy.wait_for_service('pos_cmd')
        self.goal_service = rospy.ServiceProxy('pos_cmd', command)
        self.client = actionlib.SimpleActionClient(
            "navigation_controller/send_goal",
            MoveBaseAction)
        self.client.wait_for_server()

    def totag_stage_service_cb(self, totag_request):
        self.stage = 1
        while self.stage:
            time.sleep(0.5)
        return Stage_TotagResponse(True)

    def stage_cb(self, stage_msg):
        tmp = stage_msg.current_stage
        if tmp == 2:
            self.adjust_counter = 0
        self.stage = tmp 

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
        resp = self.pos_cmd_request(2, goal_x, goal_y, 0)
        goal_achived = self.loop_service_confirm()
        
        if not goal_achived:
            print("Fail at type 2 VSLAM Navigation")
            return False

        self.pos_cmd_request(1, 0, 0, goal_theta)
        goal_achive = self.loop_service_confirm()

        return goal_achived

    def set_goal(self, translation, rotation, weight):
        self.goal.target_pose.pose.position.x = translation.x * weight
        self.goal.target_pose.pose.position.y = translation.y * weight
        self.goal.target_pose.pose.position.z = translation.z * weight
        self.goal.target_pose.pose.orientation.x = rotation.x
        self.goal.target_pose.pose.orientation.y = rotation.y
        self.goal.target_pose.pose.orientation.z = rotation.z
        self.goal.target_pose.pose.orientation.w = rotation.w




    def tagCallback(self, msg_tags):
        if self.stage == 1 :
            goal_pose = PoseStamped()
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

                    transform_goal = self.tf_buffer.lookup_transform('odom',
                                       'goal', #source frame
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(1.0))

                    (r, p, y) = tf.transformations.euler_from_quaternion([
                                    transform_goal.transform.rotation.x, transform_goal.transform.rotation.y,
                                    transform_goal.transform.rotation.z, transform_goal.transform.rotation.w])
                    #list for goal's x y z and quaternion
                    goal_list = []
                    print("type of transform_goal.transform.rotation")
                    print(type(transform_goal.transform.rotation))
                    if self.last_msg == 0:
                        self.last_msg = transform_goal.transform.translation.x

                    # For debug
                    # print("adjust_counter: {}, goal: [{}, {}, {}]".format(self.adjust_counter, transform_goal.transform.translation.x, transform_goal.transform.translation.y, y))

                    if math.sqrt((self.last_msg - transform_goal.transform.translation.x)**2) > 0.15:
                        print("Tag frame bias too much, skip detection.")
                        continue

                    self.last_msg = transform_goal.transform.translation.x

                    if self.adjust_counter < 2:
                        print(transform_goal.transform.translation.x)
                        print(transform_goal.transform.translation.y)
                        goal_weight = (self.adjust_counter + 2) * 0.25
                        print(goal_weight)
                        #new way to send goal
                        self.set_goal(transform_goal.transform.translation, transform_goal.transform.rotation, goal_weight)
                        self.client.send_goal(self.goal)
                        print("wait for result")
                        self.client.wait_for_result()
                        print("goal reach success")
                        # goal_achived = self.send_goal(transform_goal.transform.translation.x * goal_weight,
                        #     transform_goal.transform.translation.y * goal_weight, y)
                    else:
                        #new way to send goal
                        self.set_goal(transform_goal.transform.translation, transform_goal.transform.rotation, 1)
                        self.client.send_goal(self.goal)
                        print("wait for result")
                        self.client.wait_for_result()
                        print("goal reach success")
                        # goal_achived = self.send_goal(transform_goal.transform.translation.x, transform_goal.transform.translation.y, y)
                        self.stage = 0

                    # if goal_achived:
                    #     if self.adjust_counter < 2:
                    #         self.adjust_counter += 1
                    if self.adjust_counter < 2:
                        self.adjust_counter += 1
                    time.sleep(1)

if __name__ == '__main__':
    rospy.init_node('tagDetections_to_goalpoint_node', anonymous=False)
    apriltags_to_goal_point = ApriltagsToGoalPoint()
    rospy.spin()
