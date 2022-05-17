#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath("RoboRailCommands.py"))
sys.path.append(os.path.dirname(SCRIPT_DIR))


from nanolib_helper import *
from RoboRailCommands import RoboRailCommands


def object_dictionary_access_examples(nanolib_helper, device_handle):

    roboRail = RoboRailCommands(nanolib_helper, device_handle)

    # 1: 0 Switched on
    # 2: 1  Enable Voltage
    # 4: 1  Quick stop (0 means quick stop on)
    # 8: 0 Enable Operation

    roboRail.positionInit()
    print(roboRail.getCurrentPosition())



    #roboRail.addToPosQueue(roboRail.getPosMaxValue(), 250)
    #roboRail.addToPosQueue(roboRail.getPosMinValue(), 250)

    roboRail.addToPosQueue(29000, 10)
    roboRail.addToPosQueue(2, 10)

    roboRail.runQueue(0,0)
    print(roboRail.getCurrentPosition())

    roboRail.printErrorStack()


if __name__ == '__main__':
    nanolib_helper = NanolibHelper()

    # create access to the nanolib
    nanolib_helper.setup()

    print('Nanolib Example')

    # its possible to set the logging level to a different level
    nanolib_helper.set_logging_level(Nanolib.LogLevel_Off)


    # list all hardware available, decide for the first one
    bus_hardware_ids = nanolib_helper.get_bus_hardware()

    if bus_hardware_ids.empty():
        raise Exception('No bus hardware found.')

    print('\nAvailable bus hardware:\n')

    line_num = 0
    # just for better overview: print out available hardware
    for bus_hardware_id in bus_hardware_ids:
        print('{}. {} with protocol: {}'.format(line_num, bus_hardware_id.getName(), bus_hardware_id.getProtocol()))
        line_num += 1

    print('\nPlease select (type) bus hardware number and press [ENTER]:', end='');

    line_num = int(input())

    print('');

    if ((line_num < 0) or (line_num >= bus_hardware_ids.size())):
        raise Exception('Invalid selection!')

    # Use the selected bus hardware
    bus_hw_id = bus_hardware_ids[line_num]

    print(f"\n\n{bus_hw_id}\n\n")

    # create bus hardware options for opening the hardware
    bus_hw_options = nanolib_helper.create_bus_hardware_options(bus_hw_id)

    # now able to open the hardware itself
    nanolib_helper.open_bus_hardware(bus_hw_id, bus_hw_options)

    nanolib_helper.set_logging_level(Nanolib.LogLevel_Off)

    # either scan the whole bus for devices (in case the bus supports scanning)
    device_ids = nanolib_helper.scan_bus(bus_hw_id)

    nanolib_helper.set_logging_level(Nanolib.LogLevel_Off)

    print("")
    for device_id in device_ids:
        print("Found Device: {}".format(device_id.toString()))

    if (device_ids.size() == 0):
        raise Exception('No devices found.')

    print('\nAvailable devices:\n')

    line_num =  int(input())
    # just for better overview: print out available hardware
    for id in device_ids:
        print('{}. {} [device id: {}, hardware: {}]'.format(line_num, id.getDescription(), id.getDeviceId(),
                                                            id.getBusHardwareId().getName()))
        line_num += 1

    print('\nPlease select (enter) device number(0-{}) and press [ENTER]:'.format(line_num - 1), end='');

    line_num = 0

    print('');

    if ((line_num < 0) or (line_num >= device_ids.size())):
        raise Exception('Invalid selection!')

    # We can create the device id manually
    # device_id = Nanolib.DeviceId(bus_hw_id, 1, "")
    # or select first found device on the bus
    device_id = device_ids[line_num]

    print(f"\n\n{device_id}\n\n")

    device_handle = nanolib_helper.create_device(device_id)

    # now connect to the device
    print(device_handle)
    nanolib_helper.connect_device(device_handle)

    # now ready to work with the controller, here are some examples on how to access the
    # object dictionary:
    object_dictionary_access_examples(nanolib_helper, device_handle)

    # cleanup and close everything
    nanolib_helper.disconnect_device(device_handle)
    nanolib_helper.close_bus_hardware(bus_hw_id)

    print("Closing everything successfully")
