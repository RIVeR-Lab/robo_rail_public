#!/usr/bin/env python3
from RoboRailCommands import *
from nanotec_nanolib import *
import nanolib_helper
from nanolib_helper import *
from nanolib_helper import NanolibHelper


from nanotec_nanolib import *


class hardwareConnect:
    def __init__(self, nanolib):
        self.nanolib_helper = nanolib
        # list all hardware available, decide for the first one
        self.bus_hardware_ids = self.nanolib_helper.get_bus_hardware()


        if self.bus_hardware_ids.empty():
            raise Exception('No bus hardware found.')

        print('\nAvailable bus hardware:\n')

        line_num = 0
        # just for better overview: print out available hardware
        for bus_hardware_id in self.bus_hardware_ids:
            print('{}. {} with protocol: {}'.format(line_num, bus_hardware_id.getName(), bus_hardware_id.getProtocol()))
            line_num += 1

        print('\nPlease select (type) bus hardware number and press [ENTER]:', end='');

        line_num = 1

        print('');

        if (line_num < 0) or (line_num >= self.bus_hardware_ids.size()):
            raise Exception('Invalid selection!')

        # Use the selected bus hardware
        self.bus_hw_id = self.bus_hardware_ids[line_num]

        # create bus hardware options for opening the hardware
        bus_hw_options = self.nanolib_helper.create_bus_hardware_options(self.bus_hw_id)

        # now able to open the hardware itself
        self.nanolib_helper.open_bus_hardware(self.bus_hw_id, bus_hw_options)

        self.nanolib_helper.set_logging_level(Nanolib.LogLevel_Off)

        # either scan the whole bus for devices (in case the bus supports scanning)
        device_ids = self.nanolib_helper.scan_bus(self.bus_hw_id)

        self.nanolib_helper.set_logging_level(Nanolib.LogLevel_Off)

        print("")
        for device_id in device_ids:
            print("Found Device: {}".format(device_id.toString()))

        if device_ids.size() == 0:
            raise Exception('No devices found.')

        print('\nAvailable devices:\n')

        line_num = 0
        # just for better overview: print out available hardware
        for id in device_ids:
            print('{}. {} [device id: {}, hardware: {}]'.format(line_num, id.getDescription(), id.getDeviceId(),
                                                                id.getBusHardwareId().getName()))
            line_num += 1

        print('\nPlease select (enter) device number(0-{}) and press [ENTER]:'.format(line_num - 1), end='');

        line_num = 0 #int(input()) HARD CODED WITH THE ASSUMPTION OF A SINGLE DEVICE

        print('');

        if (line_num < 0) or (line_num >= device_ids.size()):
            raise Exception('Invalid selection!')


        # We can create the device id manually
        # device_id = Nanolib.DeviceId(bus_hw_id, 1, "")
        # or select first found device on the bus
        self.device_id = device_ids[line_num]
        print(f"\n\n{device_id}\n\n")
        self.device_handle = self.nanolib_helper.create_device(self.getDeviceId())


    def getDeviceId(self):
        return self.device_id

    def getDeviceHandle(self):
        return self.device_handle