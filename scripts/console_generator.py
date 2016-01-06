#!/usr/bin/env python

"""
This is a test script to generate console messages for testing /
demonstration.
"""

import rospy
import random

import rosgraph_msgs as rm
from rosgraph_msgs.msg import Log

class LogT(object):
    def __init__(self, filename, function, line, name, level, msg):
        self.msg = msg
        
        self.log = Log()
        self.log.level = level
        self.log.name = name
        self.log.file = '/home/tallest/workspaces/gir/src/' + filename
        self.log.line = line
        self.log.function = function

    def generate(self):
        self.log.header.stamp = rospy.Time.now()

        if isinstance(self.msg, str):
            self.log.msg = self.msg
        else:
            self.log.msg = self.msg()
        
        return self.log


log_templates = [
    LogT('typewriter/src/linebreak.cpp', 'break_lines', 50,
         '/typewriter/output', Log.DEBUG, "Breaking\nLines."),
    LogT('taco_system/src/taste_evaluator.cpp', 'calc_spicy', 225,
         '/taste/taco_manager', Log.WARN, lambda : ('sample exceeds max capsaicin levels (%.2f)' % random.uniform(12, 19))),
    LogT('squirrel_observer/src/distraction.cpp', 'handle_distraction', 891,
         '/vision/squirrel', Log.INFO, 'squirrel detected! sending voice notification'),
    LogT('squirrel_observer/src/classification.cpp', 'handle_distraction', 891,
         '/vision/squirrel', Log.WARN, 'covariance too high, classifying as squirrel to be on safe side'),
    LogT('order_processing/src/obedience.cpp', 'process_orders', 12,
         '/behavior/mission', Log.ERROR, 'implement me!'),
    LogT('joint_control/src/joint_trajectory.cpp', 'iteration', 348,
         '/mobility/knee_controller', Log.WARN, 'low stability margin detected'),
    LogT('laser/src/safety_lock.cpp', 'activate_lock', 921,
         '/weapons/laser_eyes', Log.FATAL, 'segmentation fault')
    ]

rospy.init_node('console_pub')
rosout = rospy.Publisher('/rosout', Log, queue_size=0)

while not rospy.is_shutdown():
    rospy.sleep(random.uniform(0.001, 0.1))
    msg = random.choice(log_templates).generate()
    rosout.publish(msg)
