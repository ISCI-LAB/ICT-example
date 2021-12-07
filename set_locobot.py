# -*- coding: utf-8 -*- 
from pyrobot import Robot
import gflags
import sys

if __name__ == '__main__':
    Flags = gflags.FLAGS 
    gflags.DEFINE_float("pan",87,"set pan")     
    gflags.DEFINE_float("tilt",87,"set tilt")
    Flags(sys.argv)
    # base_config_dict={'base_controller': 'ilqr'}
    bot = Robot('locobot')

    # target_position = [0.0, 0.0, -2.0] 
    # bot.base.go_to_relative(target_position, smooth=False, close_loop=True)
    # bot.base.go_to_relative(target_position)
    bot.camera.reset()
    if Flags.pan != 87 :
        bot.camera.set_pan(Flags.pan)
    if Flags.tilt != 87:
	bot.camera.set_tilt(Flags.tilt)
    sys.exit()
