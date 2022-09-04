#!/usr/bin/env python

# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Grasping example with the PyRobot API.
Follow the associated README for installation instructions.
"""

import argparse
import copy
import signal
import sys
import time
import numpy as np
import rospy
from pyrobot import Robot
from tf import TransformListener
from motion_pkg.srv import Grasp_Point, Grasp_PointResponse
from baseline_navi.srv import Stage_Grasp, Stage_GraspResponse

BB_SIZE = 5
MAX_DEPTH = 3.0
BASE_FRAME = "base_link"
KINECT_FRAME = "camera_color_optical_frame"
DEFAULT_PITCH = 1.57
MIN_DEPTH = 0.1
N_SAMPLES = 100
PATCH_SIZE = 100


class Motion_(object):
    """
    This class contains functionality to make the LoCoBot grasp objects placed in front of it.
    """

    def __init__(self):
                # TODO - use planning_mode=no_plan, its better
        self.robot = Robot(
            "locobot",
            arm_config={"use_moveit": True, "moveit_planner": "ESTkConfigDefault"},
        )
        self.pregrasp_height = 0.2
        self.grasp_height = 0.13
        self.retract_position = list([-1.5, 0.5, 0.3, -0.7, 0.0])
        self.reset_pan = 0.0
        self.reset_tilt = 0.8
        self.n_tries = 5
        self._sleep_time = 2
        self._transform_listener = TransformListener()
        rospy.wait_for_service('locobot_grasppoint')
        self.grasppoint_service = rospy.ServiceProxy('locobot_grasppoint', Grasp_Point)
        self.grasp_stage_srv = rospy.Service('baseline_navi/stage_grasp', Stage_Grasp, self.grasp_stage_service_cb)
        self.color = ""

    def reset(self):
        """
        Resets the arm to it's retract position.
        :returns: Success of the reset procedure
        :rtype: bool
        """
        success = False
        for _ in range(self.n_tries):
            success = self.robot.arm.set_joint_positions(self.retract_position)
            if success:
                break
        self.robot.gripper.open()
        self.robot.camera.set_pan(self.reset_pan)
        self.robot.camera.set_tilt(self.reset_tilt)
        return success

    def grasp(self, grasp_pose):
        """
        Performs manipulation operations to grasp at the desired pose.

        :param grasp_pose: Desired grasp pose for grasping.
        :type grasp_pose: list
        :returns: Success of grasping procedure.
        :rtype: bool
        """

        pregrasp_position = [grasp_pose[0], grasp_pose[1], self.pregrasp_height]
        grasp_angle = self.get_grasp_angle(grasp_pose)
        grasp_position = [grasp_pose[0], grasp_pose[1], self.grasp_height]

        rospy.loginfo("Going to pre-grasp pose:\n\n {} \n".format(pregrasp_position))
        result = self.set_pose(pregrasp_position, roll=grasp_angle)
        time.sleep(self._sleep_time)

        rospy.loginfo("Going to grasp pose:\n\n {} \n".format(grasp_position))
        result = self.set_pose(grasp_position, roll=grasp_angle)
        time.sleep(self._sleep_time)

        rospy.loginfo("Closing gripper")
        self.robot.gripper.close()
        time.sleep(1)

        rospy.loginfo("Going to pre-grasp pose")
        result = self.set_pose(pregrasp_position, roll=grasp_angle)
        time.sleep(1)
        return result

    def set_pose(self, position, pitch=DEFAULT_PITCH, roll=0.0):
        """
        Sets desired end-effector pose.

        :param position: End-effector position to reach.
        :param pitch: Pitch angle of the end-effector.
        :param roll: Roll angle of the end-effector
        :type position: list
        :type pitch: float
        :type roll: float
        :returns: Success of pose setting process.
        :rtype: bool
        """

        success = 0
        for _ in range(self.n_tries):
            position = np.array(position)
            success = self.robot.arm.set_ee_pose_pitch_roll(
                position=position, pitch=pitch, roll=roll, plan=False, numerical=False
            )
            if success == 1:
                print("set_pose success")
                break
        return success

    def get_grasp_angle(self, grasp_pose):
        """
        Obtain normalized grasp angle from the grasp pose.
        This is needs since the grasp angle is relative to the end effector.

        :param grasp_pose: Desired grasp pose for grasping.
        :type grasp_pose: list
        :returns: Relative grasp angle
        :rtype: float
        """

        cur_angle = np.arctan2(grasp_pose[1], grasp_pose[0])
        delta_angle = grasp_pose[2] + cur_angle
        if delta_angle > np.pi / 2:
            delta_angle = delta_angle - np.pi
        elif delta_angle < -np.pi / 2:
            delta_angle = 2 * np.pi + delta_angle
        return delta_angle

    def exit(self):
        """
        Graceful exit.
        """

        rospy.loginfo("Exiting...")
        self.reset()
        sys.exit(0)

    def signal_handler(self, sig, frame):
        """
        Signal handling function.
        """
        self.exit()

    def handle_grasp(self, req):
        rospy.loginfo("Grasp attempt x={:.4f},y={:.4f},theta={:.4f}".format(req.x,req.y,req.theta))
        self.color = req.color
        try:
            success = self.reset()
            assert  success
        except:
            rospy.logerr("Arm reset failed")
        grasp_pose = [req.x, req.y, req.theta]
        print("\n Grasp Pose: \n\n {} \n\n".format(grasp_pose))
        self.robot.camera.set_tilt(0.0)
        self.grasp(grasp_pose)

        # robotics arm placing
        rospy.loginfo('Going to placing pose')
        print(self.color)
        if self.color == 'red':
            result = self.set_pose([0.234, -0.235, 0.3], roll=0.0)
            print("red")
        elif self.color == 'green':
            result = self.set_pose([0.174, -0.235, 0.3], roll=0.0)
            print("green")
        elif self.color == 'blue':
            result = self.set_pose([0.114, -0.235, 0.3], roll=0.0)
            print("blue")
        time.sleep(self._sleep_time)
        return result

    def grasp_stage_service_cb(self, grasp_request):
        if grasp_request.request == 0:
            self.reset()
            time.sleep(1)
        elif grasp_request.request == 1:
            print("call grasp.py")
            response = self.grasppoint_service(True)
            self.handle_grasp(response)
        elif grasp_request.request == 3:
            rospy.loginfo('Opening gripper')
            self.robot.gripper.open()
            rospy.loginfo('Going to placing above pose')
            self.set_pose([0.15, 0.0, 0.3], roll=0.0)
            rospy.loginfo('back to original point')
        return Stage_GraspResponse(True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process args for grasper")
    parser.add_argument('--n_grasps', help='Number of grasps for inference', type=int, default=5)
    parser.add_argument('--n_samples', help='Number of samples for a single grasp inference', type=int, default=N_SAMPLES)
    parser.add_argument('--patch_size', help='Size of a sampled grasp patch', type=int, default=PATCH_SIZE)
    parser.add_argument('--no_visualize', help='False to visualize grasp at each iteration, True otherwise',
                        dest='display_grasp', action='store_false')
    parser.set_defaults(no_visualize=True)

    args, unknown = parser.parse_known_args()
    rospy.init_node('locobot_motion_server')
    motion = Motion_()
    rospy.spin()
