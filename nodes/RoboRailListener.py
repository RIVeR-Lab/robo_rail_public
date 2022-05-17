#!/usr/bin/env python3
import sys
import os
# sys.path.append("..")
# sys.path.append("../src")
# sys.path.append(".")
# sys.path.append("./src")


import rospy
import rospkg
# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path('robo_rail') + '/nodes')
rospy.logwarn(sys.path)
from hardwareConnect import *
from RoboRailCommands import *
from nanolib_helper import *
from robo_rail.srv import roboRail
import threading
from std_msgs.msg import Int32

def callback(data):
    rospy.loginfo(rospy.getname() + " actionToDo: %s" + data.actionToDo)
    rospy.loginfo(rospy.getname() + " params: %f" + data.params)
    rospy.loginfo(rospy.getname() + " otherParams: %f" + data.otherParams)


class RoboRailListener():
    def __init__(self):
        self.nanolib_helper = NanolibHelper()
        self.nanolib_helper.setup()
        # its possible to set the logging level to a different level
        self.nanolib_helper.set_logging_level(Nanolib.LogLevel_Off)

        self.bus_hardware_ids = hardwareConnect(self.nanolib_helper)
        print(self.bus_hardware_ids.getDeviceId())
        self.deviceHandle = self.bus_hardware_ids.device_handle

        self.nanolib_helper.connect_device(self.deviceHandle)
        self.roboRailCommands = RoboRailCommands(self.nanolib_helper, self.deviceHandle)

        # TODO init the motor, just call the functions to set up
        self.roboRailCommands.positionInit()

        self.service_collect = rospy.Service('/robo_rail_listener', roboRail, self.__roborail_control_server)
        # TODO define a publisher
        self.pos_pub = rospy.Publisher('/rail_position', Int32, queue_size=3)
        rospy.loginfo("RoboRail server started")
        mythread = threading.Thread(target=self.publish_pose)
        mythread.start()

    # nathnaiel please help im on like 3 hours of sleep

    def __roborail_control_server(self, req):
        print(req)
        return self.roboRailCommands.functionRunner(req)

    def publish_pose(self):
        while not rospy.is_shutdown():
            self.pos_pub.publish(self.roboRailCommands.getCurrentPosition())
            rospy.sleep(0.001)


if __name__ == "__main__":
    rospy.init_node("robo_rail")
    RRListener = RoboRailListener()
    rospy.spin()
