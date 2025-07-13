"""
TinyLEO Container Access Script

This script demonstrates how to retrieve commands for accessing specific 
containers in the TinyLEO simulation environment.

Usage:
------
1. Run this script to generate `nsenter` commands for accessing the specified containers.
2. Use the generated commands to enter the containers and perform desired tests manually.

Example:
--------
Generated command for accessing a container:
`nsenter -m -u -i -n -p -t <PID> bash`

Replace `<PID>` with the actual process ID of the container.
"""

import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from southbound.sn_controller import *

GS_lat_long = [[32,-12], [-5.11,-56.39],[64, 94]]

# Define ground station (GS) cell positions in the grid
GS_cell = [[4,6],[7,4],[2,9]]

# Path to the configuration file
configuration_file_path = "config/tinyleo_config.json"

# Initialize the southbound controller
sn = RemoteController(configuration_file_path,GS_lat_long,GS_cell)

container_pid = sn.get_pid_map()

container_list = ['GS1','GS2',"SH1SAT270","SH1SAT1","SH1SAT1596",'SH1SAT1483','SH1SAT912','SH1SAT1604','SH1SAT1636','SH1SAT828']
for container in container_list:
    print(container,'   ','nsenter','-m', '-u', '-i', '-n', '-p', '-t', container_pid[container],'bash')