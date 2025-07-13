"""
This script is responsible for managing and starting the SRv6 routing agent (`srv6_agent.py`) 
inside multiple container namespaces. It reads a file containing container names and their 
corresponding PIDs, then uses the `nsenter` command to enter the namespace of each container 
and execute the routing agent script.

Key functionalities:
1. Reads the container PID file to map container names to their PIDs.
2. Iterates through the PID mappings and uses `nsenter` to enter the container's namespace.
3. Starts the `srv6_agent.py` script inside each container using the specified Python interpreter.
4. Ensures the process runs independently by using `setsid`.

This script is useful for automating the deployment and execution of the SRv6 routing agent 
across multiple containers in a network simulation or production environment.
"""

import subprocess
from tqdm import tqdm

dir = '/root/tinyleo-Arbitrary-LeastDelay'
pid_path = f"{dir}/container_pid.txt"
pid_maps = {}
with open(pid_path, 'r') as f:
    for line in f:
        if len(line) == 0 or line.isspace():
            continue
        for name_pid in line.strip().split():
            # print(name_pid)
            name_pid = name_pid.split(':')
            pid_maps[name_pid[0]] = name_pid[1]
func = "srv6_agent.py"

with tqdm(total=len(pid_maps), desc="Deploying SRv6 agents", unit="container") as pbar:
    for name, pid in pid_maps.items():
        try:
            subprocess.Popen(
                ["setsid", 'nsenter', '-m', '-u', '-i', '-n', '-p', '-t', pid,
                 'python3', f'/resources/controller/geographic_srv6_anycast/{func}'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            pbar.update(1)
        except subprocess.CalledProcessError as e:
            print(f"Failed to start {func} for {name} with pid {pid}: {e}")
            pbar.update(1)