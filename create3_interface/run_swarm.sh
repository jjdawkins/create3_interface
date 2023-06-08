#!/bin/bash

/bin/sh -c "/bin/python3 /home/arty/create3_ws/src/create3_interface/create3_interface/create3_swarm_demo_node.py create_1 &
sleep 1 &
/bin/python3 /home/arty/create3_ws/src/create3_interface/create3_interface/create3_swarm_demo_node.py create_2 &
sleep 1 &
/bin/python3 /home/arty/create3_ws/src/create3_interface/create3_interface/create3_swarm_demo_node.py create_3 &
sleep 1 &
/bin/python3 /home/arty/create3_ws/src/create3_interface/create3_interface/create3_swarm_demo_node.py create_4 "