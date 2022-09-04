import sys
import os
import numpy as np
import torch
from torch.autograd import Variable
import cv2
import time
import argparse

sys.path.append(os.path.dirname(os.path.realpath(__file__)).rstrip("grasp_samplers/test"))
from grasp_samplers.deeper_models import *
from grasp_samplers.grasp_model import GraspModel

MODEL_URL = 'https://www.dropbox.com/s/fta8zebyzfrt3fw/checkpoint.pth.20?dl=0'


def create_parser():
    parser = argparse.ArgumentParser(description='Patch Grasp Args')
    parser.add_argument('--visual', action="store_true")

    return parser


def run_prediction(data, visual=False):
    result = []
    grasp_model = GraspModel(model_name='model.pth',
                             url=MODEL_URL,
                             nsamples=78,
                             patchsize=100)
    inference_time = 0.

    for i in data:
        start = time.time()
        grasp_model.predict(i)
        end = time.time()
        inference_time += (end - start)
        if visual:
            grasp_model.display_predicted_image()

    return inference_time


if __name__ == "__main__":
    parser = create_parser()
    args = parser.parse_args()
    current_dir = os.path.dirname(os.path.realpath(__file__)).rstrip("grasp_samplers/test")
    data = []

    for name in range(1, 5):
        img = cv2.imread('{}/grasp_samplers/demo_images/{}.jpg'.format(current_dir, name))[:, :, [2, 1, 0]]
        img = cv2.resize(img, (224, 224))
        data.append(img)

    with torch.no_grad():
        result = run_prediction(data, args.visual)

    rm_cmd = "rm {}*.patch".format(current_dir.rstrip("third_party"))

    os.system(rm_cmd)
    print(rm_cmd)
    print("Prediction time: {}".format(result))
    assert result < 10, "Check if CUDA available"
