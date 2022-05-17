# robo_rail
ROS package and documentation for control of linear rail

## Has to run in the project directory (catkin_ws/src/robo_rail)
rosrun robo_rail RoboRailListener.py


## PRE-REQS:
Install nanolib from nanotec to run software. Select python for windows/linux. Theres a README on how to install 
https://en.nanotec.com/products/9985-nanolib

Rospy needs to be installed.

## Basic commands:

To init position call function positionInit and pass (int minimumRange, int MaximumRange)
To add position to queue call function addToPosQueue(int Position, int velocity)
To run Queue call runQueue


    #0x6040 is Controlword 16 bits
    #0x6041 is status
    #0x6060 modes of opperation is 8 bis
    
    #0x60FF is target velocity 32 bits
    #607Ah Target Position 32 bits
    #6072h positioning option code
    #607Bh is posotion range limit, 32 bits, offsets, 0x00 highest sub index supported 8 bits, 0x01 min position range 32bits, 0x02 max position range limit 32 bit 
    #607Ch Home Offset 32 bits, difference between zero psoition and reference point of macine
    #607D software position limit, 32 bits
    #6081 profile velocity (max travel speed allowed) 32 bits
    #6082j end velocity (velocity at the end of travel) 32 bits

Position information
C5-E CANopen/USB technical Manual | page 50

one tick is roughtly 0.00131 inches

## How to run

MIN position : -1

MAX position : 29000

to add to queue
rosservice call /robo_rail_listener "addToPosQueue" 20000 200

to run queue

rosservice call /robo_rail_listener "runQueue" 0 0

