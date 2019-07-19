#!/usr/bin/env python

import rospy
from riptide_msgs.msg import AlignmentCommand, DepthCommand, LinearCommand
from darknet_ros_msgs.msg import BoundingBoxes
"""from darknet_ros_msgs import BoundingBox"""
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from dynamic_reconfigure.server import Server
from riptide_controllers.cfg import AlignmentControllerConfig

import math

class AlignmentController():

    MAX_FORCE = 40.0
    X_FORCE_P = 2.0
    Y_FORCE_P = 2.0
    Z_FORCE_P = 2.0

    cam_width = 644
    cam_height = 482
    currentCam = 0

    """
    x_error = 0
    y_error = 0
    z_error = 0
    """
    error = Vector3(0, 0, 0)

    """
    x_forceCmd = 0
    y_forceCmd = 0
    z_forceCmd = 0
    """
    force = Vector3(0, 0, 0)

    width_ratio = 0
    
    # Do the shutdown in action
    # time0 = time.time()

    def cmdCb(self, msg):
        self.object = msg.object
        self.width = msg.width
    
    def bboxCb(self, msg):
        self.width_ratio = msg.width_ratio
        for bbox in msg.bounding_boxes:
            if self.object == bbox.Class:
                self.error.x = ((bbox.xmin + bbox.xmax) / 2 - self.cam_width / 2)
                self.error.y = ((bbox.ymin + bbox.ymax) / 2 - self.cam_height / 2)
                self.error.z = ((bbox.xmax - bbox.xmin) - self.cam_width)
        
        self.force.x = self.X_FORCE_P * self.error.x
        self.force.y = self.Y_FORCE_P * self.error.y
        self.force.z = self.Z_FORCE_P * self.error.z
        if self.currentCam == 0:
            YPub.publish(self.force.x, LinearCommand.FORCE)
            XPub.publish(-self.force.z, LinearCommand.FORCE)
            depthPub.publish(-self.force.y, DepthCommand.depth)
        else:
            YPub.publish(self.force.x, LinearCommand.FORCE)
            XPub.publish(self.force.y, LinearCommand.FORCE)
            depthPub.publish(-self.force.z, DepthCommand.depth)

    def reconfigure(self, config, name):
        self.X_FORCE_P = config[name + "_x_force_p"]
        self.Y_FORCE_P = config[name + "_y_force_p"]
        self.Z_FORCE_P = config[name + "_z_force_p"]
        self.MAX_FORCE = config[name + "_max_force"]
        
alignmentController = AlignmentController()

XPub = rospy.Publisher("/command/x", Float64, queue_size=5)
YPub = rospy.Publisher("/command/y", Float64, queue_size=5)
depthPub = rospy.Publisher("/command/depth", DepthCommand, queue_size=5)

def dynamicReconfigureCb(config, level):
    # On dynamic reconfiguration
    alignmentController.reconfigure(config, "align")
    return config

if __name__ == '__main__':

    rospy.init_node("alignment_controller")

    # Set subscribers
    # rospy.Subscriber("/command/camera", Int8, cameraSelectionCb)
    rospy.Subscriber("/command/alignment", AlignmentCommand, alignmentController.cmdCb)
    rospy.Subscriber("/state/bboxes", BoundingBoxes, alignmentController.bboxCb)
    
    Server(AlignmentControllerConfig, dynamicReconfigureCb)

    rospy.spin()
