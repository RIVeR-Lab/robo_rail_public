#!/usr/bin/env python3
from src.nanolib_helper import NanolibHelper
from nanotec_nanolib import *
import queue
from std_msgs.msg import String
import time


# status_word = nanolib_helper.write_number(device_handle, 0x7, Nanolib.OdIndex(0x6040, 0x00), 16)
# TODO edit all functions to take a variable amount of arguments (self, *args)

# container for position and velocity to pass into FIFO
class positionWrapper:
    def __init__(self, position: int, velocity: int):
        self.position = position
        self.velocity = velocity


# Wrapper and main class to control different commands of the roboRail
class RoboRailCommands:
    def __init__(self, nanoLibModule: NanolibHelper, deviceHandle: Nanolib.DeviceHandle):
        self.nanoLibModule = nanoLibModule
        self.deviceHandle = deviceHandle
        self.PosQueue = queue.Queue()
        self.__min = -1
        self.__max = 30000

    # position : int
    # This sets the target position
    def setTargetPosition(self, *args):
        if (args[0] > self.__max):
            print("OVER MAX VALUE, DEFAULTING TO MAX VALUE")
            self.nanoLibModule.write_number(self.deviceHandle, self.__max, Nanolib.OdIndex(0x607A, 0x00), 32)
        elif (args[0] < self.__min):
            print("UNDER MIN VALUE, DEFAULTING TO MIN VALUE")
            self.nanoLibModule.write_number(self.deviceHandle, self.__min, Nanolib.OdIndex(0x607A, 0x00), 32)
        else:
            self.nanoLibModule.write_number(self.deviceHandle, args[0], Nanolib.OdIndex(0x607A, 0x00), 32)

        # velocity : int
        # This sets the target velocity
    def setTargetVelocity(self, *args):
        self.nanoLibModule.write_number(self.deviceHandle, args[0], Nanolib.OdIndex(0x60FF, 0x00), 32)

        # velocity : int
        # This sets the target velocity

    #Controls acceleration IN POSITION MODE
    def setProfileAcceleration(self, *args):
        self.nanoLibModule.write_number(self.deviceHandle, args[0], Nanolib.OdIndex(0x6083, 0x00), 32)

    # Min, Max : int, int
    def setOffset(self, *args):
        # Position range limits
        self.nanoLibModule.write_number(self.deviceHandle, self.__min, Nanolib.OdIndex(0x607B, 0x01), 32)
        self.nanoLibModule.write_number(self.deviceHandle, self.__max, Nanolib.OdIndex(0x607B, 0x02), 32)
        # software range limits
        self.nanoLibModule.write_number(self.deviceHandle, args[0], Nanolib.OdIndex(0x607D, 0x01), 32)
        self.nanoLibModule.write_number(self.deviceHandle, args[1], Nanolib.OdIndex(0x607D, 0x02), 32)

    # maxSpeed : int
    # This sets the velocity FOR WHEN IN POSITION MODE
    def setProfileVelocity(self, *args):
        self.nanoLibModule.write_number(self.deviceHandle, args[0], Nanolib.OdIndex(0x6081, 0x00), 32)

    # endVelocity : int
    # Velocity to be at the end
    def endVelocity(self, *args):
        self.nanoLibModule.write_number(self.deviceHandle, args[0], Nanolib.OdIndex(0x6082, 0x00), 32)

    # homeOffest : int
    # This sets the offset for home osition value
    def homeOffset(self, *args):
        self.nanoLibModule.write_number(self.deviceHandle, args[0], Nanolib.OdIndex(0x607C, 0x00), 32)

    # mode : int
    # manual setting opperation mode
    def opperationMode(self, *args):
        self.nanoLibModule.write_number(self.deviceHandle, args[0], Nanolib.OdIndex(0x6060, 0x00), 8)

    # This sets the controller into profile position mode
    def setPositionMode(self, *args):
        self.nanoLibModule.write_number(self.deviceHandle, 0x1, Nanolib.OdIndex(0x6060, 0x00), 8)

    # This sets the controller into profile Velocity mode
    def setProfileVelocityMode(self, *args):
        self.nanoLibModule.write_number(self.deviceHandle, 0x3, Nanolib.OdIndex(0x6060, 0x00), 8)

    # this sets the position of each new position to a present value
    def setPositionReltoPreset(self, *args):
        self.nanoLibModule.write_number(self.deviceHandle, 0x1, Nanolib.OdIndex(0x60F2, 0x00), 16)

    # this sets the position of each new position to the previous value
    def setPositionReltoLast(self, *args):
        self.nanoLibModule.write_number(self.deviceHandle, 0x0, Nanolib.OdIndex(0x60F2, 0x00), 16)

    def setHome(self, *args):
        self.nanoLibModule.write_number(self.deviceHandle, 0x0, Nanolib.OdIndex(0x607C, 0x00), 32)

    def enableVoltage(self, *args):
        self.nanoLibModule.write_number(self.deviceHandle, 0x6, Nanolib.OdIndex(0x6040, 0x00), 16)

    # This will enable voltage as it will throw an error if voltage is not enabled
    def enableSwitchOn(self, *args):
        self.nanoLibModule.write_number(self.deviceHandle, 0x7, Nanolib.OdIndex(0x6040, 0x00), 16)

    # This turns on the motor, it will enable voltage and switch on, this will run the command given to it
    def enableOpperation(self, *args):
        self.nanoLibModule.write_number(self.deviceHandle, 0xF, Nanolib.OdIndex(0x6040, 0x00), 16)

    # This turns on the motor, it will enable voltage and switch on, this will run the command given to it
    # travel command triggered by bit 4 is immediately executed.
    def enablePosOpperation(self, *args):
        self.nanoLibModule.write_number(self.deviceHandle, 0x5F, Nanolib.OdIndex(0x6040, 0x00), 16)

    # This will turn off the position opperation to allow for the new value to be set
    def midPosOpperationStop(self, *args):
        self.nanoLibModule.write_number(self.deviceHandle, 0x57, Nanolib.OdIndex(0x6040, 0x00), 16)

    # This keeps voltage on but sets quickstop value to 0 which will stop the motor
    def quickStop(self, *args):
        self.nanoLibModule.write_number(self.deviceHandle, 0x4, Nanolib.OdIndex(0x6040, 0x00), 16)

    # Turns all of control word off
    def switchOff(self, *args):
        self.nanoLibModule.write_number(self.deviceHandle, 0x0, Nanolib.OdIndex(0x6040, 0x00), 16)

    # Motor halt
    def enableHalt(self, *args):
        self.nanoLibModule.write_number(self.deviceHandle, 0x10F, Nanolib.OdIndex(0x6040, 0x00), 16)

    # returns the control word value
    def getControlWord(self, *args):
        return self.nanoLibModule.read_number(self.deviceHandle, Nanolib.OdIndex(0x6040, 0x00))

    # return the status word value
    def getStutusWord(self, *args):
        return self.nanoLibModule.read_number(self.deviceHandle, Nanolib.OdIndex(0x6041, 0x00))

    # returns a value of a passes address
    def getAddress(self, *args):
        return self.nanoLibModule.read_number(self.deviceHandle, Nanolib.OdIndex(args[0], 0x00))

    # turns 1 if it has reached target position, 0 if it hasn't
    def getReachedtarget(self, *args):
        return ((self.getStutusWord() >> 10) & 1)

    def getPosMinValue(self, *args):
        return self.__min

    def getPosMaxValue(self, *args):
        return self.__max

    def getCurrentPosition(self, *args):
        return self.nanoLibModule.read_number(self.deviceHandle, Nanolib.OdIndex(0x6064, 0x00))
        # prints error stack

    def printErrorStack(self, *args):
        print('\nRead device error stack')
        error_stack = self.nanoLibModule.read_array(self.deviceHandle, Nanolib.OdIndex(0x1003, 0x00))
        print('The error stack has {} elements\n'.format(error_stack[0]))



    #####################################
    #
    #
    # HIGH LEVEL ABSTRACTION TO RUN AND ADD VALUES
    #
    #
    ####################################

    # inits motor controller for position mode
    def positionInit(self):
        self.enableVoltage()
        self.setPositionMode()
        self.setHome()
        self.setOffset(self.getPosMinValue(), self.getPosMaxValue())
        self.setPositionReltoLast()

    # add a position and velocity to the position FIFO
    def addToPosQueue(self, Position: int, Velocity: int):
        self.PosQueue.put(positionWrapper(int(Position), int(Velocity)))

    # this will drain the position FIFO
    def runQueue(self, ignore, ignore1):
        self.enableSwitchOn()
        for i in range(self.PosQueue.qsize()):
            temp = self.PosQueue.get()
            self.setTargetPosition(temp.position)
            self.setProfileVelocity(temp.velocity)
            self.setProfileAcceleration(1000)
            self.enablePosOpperation()
            while self.getReachedtarget() == 0:
                time.sleep(0.001)
            self.midPosOpperationStop()
            time.sleep(.1)
        self.enableVoltage()

        # function runner for ROS to allow for srv messages to run
    def functionRunner(self, req):
        print("Recieved", req.command)
        eval("self." + req.command)(req.params, req.otherParams)
        return "Task Complete"