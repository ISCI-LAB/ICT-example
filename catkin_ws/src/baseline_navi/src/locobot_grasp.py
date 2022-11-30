#!/usr/bin/env python

# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Grasping example with the PyRobot API.

Follow the associated README for installation instructions.
"""

import os
import copy
import sys
import time
import numpy as np
import rospy
from geometry_msgs.msg import Quaternion, PointStamped
from tf import TransformListener
from std_msgs.msg import Int32
from motion_pkg.srv import Grasp_Point, Grasp_PointResponse
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
sys.path.append(os.path.expanduser("~") + "/ICT-example/third_party")
from grasp_samplers.grasp_model import GraspModel

MODEL_URL = 'https://www.dropbox.com/s/fta8zebyzfrt3fw/checkpoint.pth.20?dl=0'
BB_SIZE = 5
MAX_DEPTH = 3.0
BASE_FRAME = 'base_link'
KINECT_FRAME = 'camera_color_optical_frame'
DEFAULT_PITCH = 1.57
MIN_DEPTH = 0.1
N_SAMPLES = 78
PATCH_SIZE = 100


class Grasp_pose(object):
    """
    This class contains functionality to make the LoCoBot grasp objects placed in front of it.
    """

    def __init__(self,
                 url=MODEL_URL,
                 model_name='model.pth',
                 n_samples=N_SAMPLES,
                 patch_size=PATCH_SIZE,
                 *kargs, **kwargs):
        """
        The constructor for :class:`Grasper` class. 

        :param url: Link to the grasp model file
        :param model_name: Name of the path where the grasp model will be saved
        :param n_samples: Number of samples for the grasp sampler
        :param patch_size: Size of the patch for the grasp sampler
        :type url: string
        :type model_name: string
        :type n_samples: int
        :type patch_size: int
        """

        # TODO - use planning_mode=no_plan, its better
        self.grasp_model = GraspModel(model_name=model_name,
                                      url=url,
                                      nsamples=n_samples,
                                      patchsize=patch_size)
        self.color = ''
        self._transform_listener = TransformListener()
        self.bridge = CvBridge()
        self.image_rgb = None
        self.image_depth = None
        self.camera_info = None
        self.image_sub = rospy.Subscriber('camera/color/image_raw', Image, self.image_cb, queue_size=1)
        self.camera_info_sub = rospy.Subscriber('camera/color/camera_info', CameraInfo, self.camera_info_cb, queue_size=1)
        self.depth_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_cb, queue_size=1)
        self.grasppoint_srv = rospy.Service('locobot_grasppoint', Grasp_Point, self.handle_grasp)

    def image_cb(self, image_msg):
        try:
            self.image_rgb = self.bridge.imgmsg_to_cv2(image_msg, "rgb8")
        except CvBridgeError, e:
            print(e)

    def camera_info_cb(self, info_msg):
        self.camera_info = np.array(info_msg.P).reshape(3, 4)

    def depth_cb(self, depth_msg):
        try:
            self.image_depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough").copy()
            self.image_depth[np.isnan(self.image_depth)] = 0.
        except CvBridgeError, e:
            print(e)

    def _process_depth(self, cur_depth=None):
        if cur_depth is None:
            cur_depth = self.image_depth.astype(np.float) / 1000 if np.max(self.image_depth) > 20. else self.image_depth
        cur_depth[cur_depth > MAX_DEPTH] = 0.
        return cur_depth

    def _get_z_mean(self, depth, pt, bb=BB_SIZE):
        sum_z = 0.
        nps = 0
        for i in range(bb * 2):
            for j in range(bb * 2):
                new_pt = [pt[0] - bb + i, pt[1] - bb + j]
                try:
                    new_z = depth[int(new_pt[0]), int(new_pt[1])]
                    if new_z > MIN_DEPTH:
                        sum_z += new_z
                        nps += 1
                except:
                    pass
        if nps == 0.:
            return 0.
        else:
            return sum_z / nps

    def _get_3D_camera(self, pt, norm_z=None):
        assert len(pt) == 2
        cur_depth = self._process_depth()
        z = self._get_z_mean(cur_depth, [pt[0], pt[1]])
        rospy.loginfo('depth of point is : {}'.format(z))
        if z == 0.:
            raise RuntimeError
        if norm_z is not None:
            z = z / norm_z
        u = pt[1]
        v = pt[0]
        P = copy.deepcopy(self.camera_info)
        rospy.loginfo('P is: {}'.format(P))
        P_n = np.zeros((3, 3))
        P_n[:, :2] = P[:, :2]
        P_n[:, 2] = P[:, 3] + P[:, 2] * z
        P_n_inv = np.linalg.inv(P_n)
        temp_p = np.dot(P_n_inv, np.array([u, v, 1]))
        temp_p = temp_p / temp_p[-1]
        temp_p[-1] = z
        return temp_p

    def _convert_frames(self, pt):
        assert len(pt) == 3
        rospy.loginfo('Point to convert: {}'.format(pt))
        ps = PointStamped()
        ps.header.frame_id = KINECT_FRAME
        ps.point.x, ps.point.y, ps.point.z = pt
        base_ps = self._transform_listener.transformPoint(BASE_FRAME, ps)
        rospy.loginfo(
            'transform : {}'.format(self._transform_listener.lookupTransform(BASE_FRAME, KINECT_FRAME, rospy.Time(0))))
        base_pt = np.array([base_ps.point.x, base_ps.point.y, base_ps.point.z])
        rospy.loginfo('Base point to convert: {}'.format(base_pt))
        return base_pt

    def get_3D(self, pt, z_norm=None):
        temp_p = self._get_3D_camera(pt, z_norm)
        rospy.loginfo('temp_p: {}'.format(temp_p))
        base_pt = self._convert_frames(temp_p)
        return base_pt

    def compute_grasp(self, dims=[(240, 480), (100, 540)], display_grasp=True):
        """
        Runs the grasp model to generate the best predicted grasp.

        :param dims: List of tuples of min and max indices of the image axis.
        :param display_grasp: Displays image of the grasp.
        :type dims: list
        :type display_grasp: bool

        :returns: Grasp configuration
        :rtype: list
        """
        print("Compute grasp pose")
        img = self.image_rgb
        img = img[dims[0][0]:dims[0][1], dims[1][0]:dims[1][1]]
        selected_grasp = list(self.grasp_model.predict(img))
        rospy.loginfo('Pixel grasp: {}'.format(selected_grasp))
        img_grasp = copy.deepcopy(selected_grasp)

        x = selected_grasp[0]
        y = selected_grasp[1]
        r = img[x][y][0]
        g = img[x][y][1]
        b = img[x][y][2]
        index = [r, g, b].index(max([r, g, b]))
        if index == 0:
            self.color = 'red'
        elif index == 1:
            self.color = 'green'
        else:
            self.color = 'blue'

        selected_grasp[0] += dims[0][0]
        selected_grasp[1] += dims[1][0]
        selected_grasp[:2] = self.get_3D(selected_grasp[:2])[:2]
        if display_grasp:
            self.grasp_model.display_predicted_image()

        return selected_grasp

    def handle_grasp(self, request):
        pred_grasp = self.compute_grasp()
        print("Pred grasp: {}".format(pred_grasp))
        return Grasp_PointResponse(pred_grasp[0], pred_grasp[1], pred_grasp[2], self.color)


def main():
    """
    This is the main function for running the grasping demo.
    """

    rospy.init_node('grasp_pose', anonymous=True)
    print("Init pose estimation node")
    grasp_pose = Grasp_pose(n_samples=N_SAMPLES, patch_size=PATCH_SIZE)
    rospy.spin()


if __name__ == "__main__":
    main()
