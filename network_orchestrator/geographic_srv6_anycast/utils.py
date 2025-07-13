"""
Utility Functions and Classes for SRv6 Routing Management
This module provides utility functions and classes to support SRv6 routing management.
It includes functionalities for:
- Event handling for route updates and link damage.
- File system monitoring for configuration changes.
- Shared memory operations for topology data.
- IPv6 iptables management.
- Routing table and rule management.
"""

import json
import subprocess
import iptc
import time
import socket
import os
import pickle
from multiprocessing import shared_memory,resource_tracker
from watchdog.events import FileSystemEventHandler
from watchdog.observers import Observer

class RouteUpdateEvent:
    """
    A class to manage event notifications for route updates using an event file descriptor (eventfd).
    This is used to signal and handle route update events in a non-blocking manner.
    """
    def __init__(self):
        """
        Initializes the RouteUpdateEvent instance by creating a non-blocking event file descriptor.
        """
        self.event_fd = os.eventfd(0, os.EFD_NONBLOCK)

    def get_fd(self):
        """
        Returns the file descriptor associated with the event.

        Returns:
            int: The event file descriptor.
        """
        return self.event_fd

    def set(self):
        """
        Signals the event by writing to the event file descriptor.
        This is used to notify that a route update event has occurred.
        """
        os.write(self.event_fd, b"\x01\x00\x00\x00\x00\x00\x00\x00")

    def clear(self):
        """
        Clears the event by reading from the event file descriptor.
        This is used to reset the event state.
        """
        try:
            os.read(self.event_fd, 8)
        except BlockingIOError:
            pass 

    def close(self):
        """
        Closes the event file descriptor to release resources.
        """
        os.close(self.event_fd)

class RouteUpdateHandler(FileSystemEventHandler):
    """
    A file system event handler to monitor changes in specific files and trigger route updates.
    This class listens for modifications to files like `ts.txt` and `flush.txt` and triggers
    corresponding events in the route manager.
    """

    def __init__(self, route_manager):
        """
        Initializes the RouteUpdateHandler with a reference to the route manager.

        Args:
            route_manager: The route manager instance to trigger events on.
        """
        self.route_manager = route_manager
        self.ts_count = 0 
        self.flush_count = 0

    def on_modified(self, event):
        """
        Handles file modification events. Triggers route update events based on the modified file.

        Args:
            event: The file system event containing details about the modification.
        """
        if event.src_path.endswith("ts.txt"):
            self.ts_count += 1
            if self.ts_count == 2:
                self.route_manager.ts_changed_trigger()
                self.ts_count = 0
        elif event.src_path.endswith("flush.txt"):
            self.flush_count += 1
            if self.flush_count == 2:
                self.route_manager.flush_route_trigger()
                self.flush_count = 0

class LinkDamageHandler(FileSystemEventHandler):
    """
    A file system event handler to monitor changes in the `link_change.txt` file and trigger
    link damage events. This is used to handle scenarios where a link damage event needs to
    be processed by the route manager.
    """
    def __init__(self, route_manager):
        """
        Initializes the LinkDamageHandler with a reference to the route manager.

        Args:
            route_manager: The route manager instance to trigger events on.
        """
        self.route_manager = route_manager
        self.modify_count = 0 

    def on_modified(self, event):
        """
        Handles file modification events. Triggers a link damage event based on the modified file.

        Args:
            event: The file system event containing details about the modification.
        """
        if event.src_path.endswith("link_change.txt"):
            self.modify_count += 1
            if self.modify_count == 2:
                self.route_manager.link_changed_trigger()
                self.modify_count = 0

def get_isls(ts):
    with open(f"/resources/isl_state/{ts}.json") as f:
        isls = json.load(f)
    return isls

def get_gsls(ts):
    with open(f"/resources/gsl_state/{ts}.json") as f:
        gsls = json.load(f)
    return gsls

def get_inter_cell_isls(ts):
    with open(f"/resources/inter_cell_isl/{ts}.json") as f:
        inter_cell_isls = json.load(f)
    return inter_cell_isls

def get_sat_cell(ts):
    with open(f"/resources/sat_cell/{ts}.json") as f:
        sat_cell = json.load(f)
    return sat_cell

def get_cell_ring(ts):
    with open(f"/resources/cell_ring/{ts}.json") as f:
        cell_ring = json.load(f)
    return cell_ring

def get_seg_list():
    with open(f"/resources/controller/seg_list.json") as f:
        seg_list = json.load(f)
    return seg_list

def get_ts():
    with open(f"/resources/controller/ts.txt") as f:
        ts = int(f.read().strip())
    return ts

def load_topo_from_shm(name):
    """
    Loads topology data from shared memory.

    Args:
        name (str): The name of the shared memory segment.

    Returns:
        dict: The deserialized topology data.
    """
    shm = shared_memory.SharedMemory(name=name)
    data = bytes(shm.buf[:])
    shm.close()
    resource_tracker.unregister(shm._name, "shared_memory")
    data = pickle.loads(data)
    return data

def init_sat_state_from_file():
    """
    Initializes the satellite state by reading data from files.

    Returns:
        tuple: A tuple containing the timestamp, ISLs, GSLs, inter-cell ISLs, satellite cell, and cell ring.
    """
    ts = get_ts()
    isls = get_isls(ts)
    gsls = get_gsls(ts)
    inter_cell_isls = get_inter_cell_isls(ts)
    sat_cell = get_sat_cell(ts)
    cell_ring = get_cell_ring(ts)
    return ts,isls,gsls,inter_cell_isls,sat_cell,cell_ring

def init_sat_state_from_memory(name):
    """
    Initializes the satellite state by reading data from shared memory.

    Args:
        name (str): The name of the shared memory segment.

    Returns:
        tuple: A tuple containing the timestamp, ISLs, GSLs, inter-cell ISLs, satellite cell, and cell ring.
    """
    data = load_topo_from_shm(name)
    ts = int(data['ts'])
    isls = data['isls']
    gsls = data['gsls']
    inter_cell_isls = data['inter_cell_isls']
    sat_cell = data['sat_cell']
    cell_ring = data['cell_ring']
    return ts,isls,gsls,inter_cell_isls,sat_cell,cell_ring

def init_gs_state_from_file():
    """
    Initializes the ground station state by reading data from files.

    Returns:
        tuple: A tuple containing the timestamp, satellite name, satellite IP, satellite MAC, and segment list.
    """
    ts = get_ts()
    gsls  = get_gsls(ts)
    for key in gsls:
        sat = key
        sat_ip,sat_mac = gsls[key][0],gsls[key][1]
        break
    seg_list = get_seg_list()
    return ts,sat,sat_ip,sat_mac,seg_list

def init_gs_state_from_memory(name):
    """
    Initializes the ground station state by reading data from shared memory.

    Args:
        name (str): The name of the shared memory segment.

    Returns:
        tuple: A tuple containing the timestamp, satellite name, satellite IP, satellite MAC, and segment list.
    """
    data = load_topo_from_shm(name)
    ts = int(data['ts'])
    gsls  = data['gsls']
    for key in gsls:
        sat = key
        sat_ip,sat_mac = gsls[key][0],gsls[key][1]
        break
    seg_list = data['seg_list']
    return ts,sat,sat_ip,sat_mac,seg_list

def seg_key(src,dst):

    return f"{src}->{dst}"

def cell_str2list(cell):
    return [int(x) for x in cell[1:-1].split(",")]

def seg_key_reverse(key):
    src, dst = key.split("->")
    src = cell_str2list(src)
    dst = cell_str2list(dst)
    return src, dst

def ip2cell(ip):
    cell_addr = ip.split('::')[0].split(':')
    return [int(cell_addr[1]),int(cell_addr[3])]

def get_cell_dis(cell1,cell2,cell_width=11):
    """
    Calculates the distance between two cells in a grid.

    Args:
        cell1 (list): The first cell as [row, column].
        cell2 (list): The second cell as [row, column].
        cell_width (int): The width of the grid (default is 11).

    Returns:
        int: The distance between the two cells.
    """
    y_dis = abs(cell1[1]-cell2[1])
    return abs(cell1[0]-cell2[0])+min(y_dis,cell_width-y_dis)

def get_cell_route(cell1, cell2, cell_width=11):
    """
    Calculates the route between two cells in a grid.

    Args:
        cell1 (list): The starting cell as [row, column].
        cell2 (list): The destination cell as [row, column].
        cell_width (int): The width of the grid (default is 11).

    Returns:
        list: A list of tuples representing the path between the two cells.
    """
    path = []
    r1, c1 = cell1
    r2, c2 = cell2
    if r1 != r2:
        if r1 < r2:
            step = 1
        else:
            step = -1
        x_dis = abs(r1 - r2)
        for i in range(1, x_dis):
            path.append((r1 + i * step, c1))
        if c1 != c2:
            path.append((r1 + x_dis * step, c1))
    final_row = r2
    if c1 != c2:
        diff = (c2 - c1) % cell_width
        wrap_diff = (c1 - c2) % cell_width

        if diff <= wrap_diff:
            for i in range(1, diff): 
                path.append((final_row, (c1 + i - 1) % cell_width + 1))
        else:
            for i in range(1, wrap_diff):
                path.append((final_row, (c1 - i - 1) % cell_width + 1))
    return path

def clear_iptables():
    """
    Clears all IPv6 iptables rules, including those in the "mangle" table.
    """
    subprocess.run(["ip6tables", "-F"])
    subprocess.run(["ip6tables", "-t", "mangle", "-F"])

def create_route_table(iproute, table_id, table_name):
    """
    Creates a new routing table if it does not already exist.

    Args:
        iproute (IPRoute): An instance of pyroute2.IPRoute for managing routes.
        table_id (int): The ID of the routing table.
        table_name (str): The name of the routing table.
    """
    rt_tables_path = "/etc/iproute2/rt_tables"
    with open(rt_tables_path, 'r') as f:
        lines = f.readlines()

    for line in lines:
        if line.strip().endswith(table_name) or line.strip().startswith(str(table_id)):
            return

    with open(rt_tables_path, 'a') as f:
        f.write(f"{table_id} {table_name}\n")

def add_route_rule(iproute, table_id, priority=100):
    """
    Adds a routing rule to direct traffic to a specific routing table.

    Args:
        iproute (IPRoute): An instance of pyroute2.IPRoute for managing routes.
        table_id (int): The ID of the routing table.
        priority (int): The priority of the rule (default is 100).
    """
    try:
        iproute.rule('add', table=table_id, priority=priority,family=socket.AF_INET6)
    except Exception as e:
        pass

def delete_route_rule(iproute, table_id, priority=100):
    """
    Deletes a routing rule that directs traffic to a specific routing table.

    Args:
        iproute (IPRoute): An instance of pyroute2.IPRoute for managing routes.
        table_id (int): The ID of the routing table.
        priority (int): The priority of the rule (default is 100).
    """
    try:
        iproute.rule('delete', table=table_id, priority=priority,family=socket.AF_INET6)
    except Exception as e:
        pass

def flush_table(table_id):
    """
    Flushes all routes in a specific routing table.

    Args:
        table_id (int): The ID of the routing table to flush.

    This function uses the `ip` command to flush all routes in the specified IPv6 routing table.
    If an error occurs during the process, the exception is caught and ignored.
    """
    try:
        subprocess.run(["ip", "-6","route", "flush", "table", str(table_id)])
    except Exception as e:
        pass

def add_ip6tables_rule(chain_name, table_name="filter", **kwargs):
    """
    Adds an IPv6 iptables rule to a specified chain and table.

    Args:
        chain_name (str): The name of the iptables chain (e.g., "INPUT", "OUTPUT").
        table_name (str): The name of the iptables table (default is "filter").
        **kwargs: Additional parameters for the rule, such as:
            - in_interface (str): The input network interface.
            - out_interface (str): The output network interface.
            - protocol (str): The protocol to match (e.g., "icmpv6").
            - src (str): The source IP address or subnet.
            - dst (str): The destination IP address or subnet.
            - addrtype (dict): Address type matching (e.g., src_type, dst_type).
            - icmpv6_type (str): The ICMPv6 type to match.
            - mark (str): A mark to set on the packet.
            - queue_num (int): The NFQUEUE number to send packets to.

    This function creates and appends an IPv6 iptables rule to the specified chain and table.
    """
    table = iptc.Table6(table_name)
    chain = iptc.Chain(table, chain_name)
    rule = iptc.Rule6()

    if "in_interface" in kwargs:
        rule.in_interface = kwargs["in_interface"]
    if "out_interface" in kwargs:
        rule.out_interface = kwargs["out_interface"]

    if "protocol" in kwargs:
        rule.protocol = kwargs["protocol"]

    if "src" in kwargs:
        rule.src = kwargs["src"]

    if "dst" in kwargs:
        rule.dst = kwargs["dst"]

    if "addrtype" in kwargs:
        match = rule.create_match("addrtype")
        if "src_type" in kwargs:
            match.src_type = kwargs["src_type"]
        if "dst_type" in kwargs:
            match.dst_type = kwargs["dst_type"]

    if "icmpv6_type" in kwargs:
        match = rule.create_match("icmp6")
        match.icmpv6_type = kwargs["icmpv6_type"]

    if "mark" in kwargs:
        target = rule.create_target("MARK")
        target.set_mark = kwargs["mark"]

    if "queue_num" in kwargs:
        target = rule.create_target("NFQUEUE")
        target.queue_num = kwargs["queue_num"]

    chain.append_rule(rule) 

