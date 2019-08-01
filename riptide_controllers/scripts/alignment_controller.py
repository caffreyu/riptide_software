#!/usr/bin/env python

import rospy
from riptide_msgs.msg import AlignmentCommand, DepthCommand, LinearCommand, Depth
from darknet_ros_msgs.msg import BoundingBoxes
<<<<<<< HEAD
"""from darknet_ros_msgs import BoundingBox"""
from std_msgs.msg import Float64
=======
from std_msgs.msg import Float64, Int32, Int8
>>>>>>> 344b14c7a4d8eaa6e32f0cc0ef6bcca6031e2f1d
from geometry_msgs.msg import Vector3
from dynamic_reconfigure.server import Server
from riptide_controllers.cfg import AlignmentControllerConfig

import math

class AlignmentController():
    MAX_FORCE = 30.0
    X_FORCE_P = 2.0
    Y_FORCE_P = 2.0
    DEPTH_FORCE_P = 0.1

    cam_width = 644
    cam_height = 482
    currentCam = 0

<<<<<<< HEAD
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

=======
    obj = ""
>>>>>>> 344b14c7a4d8eaa6e32f0cc0ef6bcca6031e2f1d
    width_ratio = 0
    watchdog_timer = None
    
    # Do the shutdown in action
    # time0 = time.time()

    def cmdCb(self, msg):
        self.obj = msg.object
        self.width_ratio  = msg.width_ratio
        if self.obj == "":
            if self.watchdog_timer is not None:
                    self.watchdog_timer.shutdown()
            self.shutdown()

    def depthCb(self, msg):
        self.currentDepth = msg.depth

    def cameraSelectionCb(self, msg):
        if msg.data != self.currentCam:
            self.currentCam = msg.data
    
    def bboxCb(self, msg):
        for bbox in msg.bounding_boxes:
<<<<<<< HEAD
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
=======
            if self.obj == bbox.Class:
                if self.watchdog_timer is not None:
                    self.watchdog_timer.shutdown()
                self.watchdog_timer = rospy.Timer(rospy.Duration(0.5), self.shutdown, True)
                self.x_error = (bbox.xmin + bbox.xmax) / 2 - self.cam_width / 2
                self.y_error = (bbox.ymin + bbox.ymax) / 2 - self.cam_height / 2
                self.z_error = (bbox.xmax - bbox.xmin) - self.cam_width * self.width_ratio

                if self.currentCam == 0:
                    YPub.publish(min(self.MAX_FORCE, max(-self.MAX_FORCE, self.x_error * self.Y_FORCE_P)), LinearCommand.FORCE)
                    XPub.publish(min(self.MAX_FORCE, max(-self.MAX_FORCE, -self.z_error * self.X_FORCE_P)), LinearCommand.FORCE)
                    depthPub.publish(True, self.currentDepth + self.y_error * self.DEPTH_FORCE_P)
                else:
                    YPub.publish(min(self.MAX_FORCE, max(-self.MAX_FORCE, self.x_error * self.Y_FORCE_P)), LinearCommand.FORCE)
                    XPub.publish(min(self.MAX_FORCE, max(-self.MAX_FORCE, -self.y_error * self.X_FORCE_P)), LinearCommand.FORCE)
                    depthPub.publish(True, self.currentDepth - self.z_error * self.DEPTH_FORCE_P)

    def shutdown(self, timer = None):
        YPub.publish(0, LinearCommand.FORCE)
        XPub.publish(0, LinearCommand.FORCE)
        depthPub.publish(True, self.currentDepth)
>>>>>>> 344b14c7a4d8eaa6e32f0cc0ef6bcca6031e2f1d

    def reconfigure(self, config, name):
        self.X_FORCE_P = config[name + "_x_force_p"]
        self.Y_FORCE_P = config[name + "_y_force_p"]
<<<<<<< HEAD
        self.Z_FORCE_P = config[name + "_z_force_p"]
=======
        self.DEPTH_FORCE_P = config[name + "_depth_force_p"]
>>>>>>> 344b14c7a4d8eaa6e32f0cc0ef6bcca6031e2f1d
        self.MAX_FORCE = config[name + "_max_force"]
        
alignmentController = AlignmentController()

XPub = rospy.Publisher("/command/x", LinearCommand, queue_size=5)
YPub = rospy.Publisher("/command/y", LinearCommand, queue_size=5)
depthPub = rospy.Publisher("/command/depth", DepthCommand, queue_size=5)

def dynamicReconfigureCb(config, level):
    # On dynamic reconfiguration
    alignmentController.reconfigure(config, "align")
    return config

if __name__ == '__main__':

    rospy.init_node("alignment_controller")

    # Set subscribers
    rospy.Subscriber("/command/camera", Int8, alignmentController.cameraSelectionCb)
    rospy.Subscriber("/command/alignment", AlignmentCommand, alignmentController.cmdCb)
    rospy.Subscriber("/state/bboxes", BoundingBoxes, alignmentController.bboxCb)
    rospy.Subscriber("/state/depth", Depth, alignmentController.depthCb)
    
    Server(AlignmentControllerConfig, dynamicReconfigureCb)

    rospy.spin()
