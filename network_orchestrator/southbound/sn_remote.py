"""
TinyLEO Remote Container Management Script
==========================================
This script is designed to run on remote container servers in the TinyLEO simulation environment. 
It is invoked by the controller to manage containerized satellite and ground station nodes, 
configure network links, and simulate network behaviors.

Key Features:
- Initialize and manage containers for satellites and ground stations.
- Configure inter-satellite (ISL) and ground station links (GSL).
- Simulate network faults and recoveries.
- Perform network tests such as ping, traceroute, and iperf.
- Manage shared memory for topology and link state updates.
- Clean up containers and reset the simulation environment.

Usage:
------
This script is executed on remote servers and controlled by the main controller. 
It supports the following commands:

- nodes: Initialize containers for satellites and ground stations.
- networks: Configure or update network links for a specific timestamp.
- clean: Clean up containers and reset the environment.
- ping: Perform a ping test between nodes.
- iperf: Perform an iperf test between nodes.
- traceroute: Perform a traceroute test between nodes.
- fault_test: Simulate network faults by introducing link failures.

Example Command:
    python3 sn_remote.py nodes <machine_id> <workdir>
"""

import os
import subprocess
import sys
import json
import glob
import ctypes
import shutil
from concurrent.futures import ProcessPoolExecutor,ThreadPoolExecutor
import pickle
from multiprocessing import shared_memory,resource_tracker
import grpc
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__))))
from link_failure_grpc import link_failure_pb2,link_failure_pb2_grpc



ASSIGN_FILENAME = 'assign.json'
PID_FILENAME = 'container_pid.txt'
SDN_IP = '101.6.21.12'

NOT_ASSIGNED = 'NA'
VXLAN_PORT = 4789
# FIXME
CLONE_NEWNET = 0x40000000
libc = ctypes.CDLL(None)
main_net_fd = os.open('/proc/self/ns/net', os.O_RDONLY)

def _pid_map(pid_path, pop = False):
    """
    Loads or retrieves the mapping of container names to their process IDs.

    Args:
        pid_path (str): Path to the file containing the PID mapping.
        pop (bool): If True, clears the cached PID mapping after retrieval.

    Returns:
        dict: A dictionary mapping container names to their process IDs.

    Raises:
        SystemExit: If the PID file is not found.
    """
    global _pid_map_cache
    if _pid_map_cache is None:
        _pid_map_cache = {}
        if not os.path.exists(pid_path):
            print('Error: container index file not found, please create nodes')
            exit(1)
        with open(pid_path, 'r') as f:
            for line in f:
                if len(line) == 0 or line.isspace():
                    continue
                for name_pid in line.strip().split():
                    if name_pid == NOT_ASSIGNED:
                        continue
                    name_pid = name_pid.split(':')
                    _pid_map_cache[name_pid[0]] = name_pid[1]
    if pop:
        ret = _pid_map_cache
        _pid_map_cache = None
        return ret
    return _pid_map_cache

def _failure_report(sat1,sat2):
    """
    Reports a link failure between two satellites to the SDN controller and retrieves the updated satellite states.

    Args:
        sat1 (str): Name of the first satellite involved in the link failure.
        sat2 (str): Name of the second satellite involved in the link failure.

    Steps:
        1. Establish a gRPC channel with the SDN controller at the specified IP and port.
        2. Create a LinkFailureRequest containing the IDs of the two satellites.
        3. Send the request to the SDN controller using the LinkFailureServiceStub.
        4. If the response indicates success:
            - Parse the updated satellite states from the response message.
            - Return the list of updated satellites.
        5. If the response indicates failure:
            - Log the failure message.
            - Return None.

    Returns:
        list or None: A list of updated satellites if the request is successful, otherwise None.
    """
    with grpc.insecure_channel(f'{SDN_IP}:50051') as channel:
        stub = link_failure_pb2_grpc.LinkFailureServiceStub(channel)
        response = stub.HandleLinkFailure(link_failure_pb2.LinkFailureRequest(
            satellite_ids=[sat1,sat2],
        ))
        # print(f"Response: success={response.success}, message='{response.message}'")
        if response.success:
            update_sats = response.message.split(",")
            return update_sats
        else:
            print(f"Failed to update satellites: {response.message}")
            return None

def _get_params(path):
    """
    Reads and parses the satellite and ground station assignment parameters.

    Args:
        path (str): Path to the assignment JSON file.

    Returns:
        tuple: A tuple containing:
            - sat_mid_dict_shell (list): Satellite-to-machine mapping by shell.
            - gs_mid_dict (dict): Ground station-to-machine mapping.
            - ip_lst (list): List of machine IP addresses.
    """
    with open(path, 'r') as f:
        obj = json.load(f)
        sat_mid_dict_shell = obj['sat_mid_shell']
        gs_mid_dict = obj['gs_mid']
        ip_lst = obj['ip']
    return sat_mid_dict_shell, gs_mid_dict, ip_lst

def _parse_isls(path):
    """
    Parses inter-satellite link (ISL) configuration from a file.

    Args:
        path (str): Path to the ISL configuration file.

    Returns:
        tuple: A tuple containing:
            - del_lst (list): List of ISLs to delete.
            - update_lst (list): List of ISLs to update with new parameters.
            - add_lst (list): List of ISLs to add with new parameters.
    """
    del_lst, update_lst, add_lst = [], [], []
    f = open(path, 'r')
    for line in f:
        toks = line.strip().split('|')
        sat_name = toks[0]
        if len(toks[1]) > 0:
            for isl_sat in toks[1].split(' '):
                del_lst.append((sat_name, isl_sat))
        if len(toks[2]) > 0:
            for isl_sat in toks[2].split(' '):
                sat_delay = isl_sat.split(',')
                update_lst.append((sat_name, sat_delay[0], sat_delay[1]))
        if len(toks[3]) > 0:
            for isl_sat in toks[3].split(' '):
                idx_sat_delay = isl_sat.split(',')
                add_lst.append((sat_name, idx_sat_delay[0], idx_sat_delay[1], int(idx_sat_delay[2])))
    f.close()
    return del_lst, update_lst, add_lst

def _parse_gsls(path):
    """
    Parses ground station link (GSL) configuration from a file.

    Args:
        path (str): Path to the GSL configuration file.

    Returns:
        tuple: A tuple containing:
            - del_lst (list): List of GSLs to delete.
            - update_lst (list): List of GSLs to update with new parameters.
            - add_lst (list): List of GSLs to add with new parameters.
    """
    del_lst, update_lst, add_lst = [], [], []
    f = open(path, 'r')
    for line in f:
        toks = line.strip().split('|')
        gs_name = toks[0]
        if len(toks[1]) > 0:
            for gsl_sat in toks[1].split(' '):
                del_lst.append((gs_name, gsl_sat))
        if len(toks[2]) > 0:
            for gsl_sat in toks[2].split(' '):
                sat_delay = gsl_sat.split(',')
                update_lst.append((gs_name, sat_delay[0], sat_delay[1]))
        if len(toks[3]) > 0:
            for gsl_sat in toks[3].split(' '):
                idx_sat_delay = gsl_sat.split(',')
                add_lst.append((gs_name, idx_sat_delay[0], idx_sat_delay[1], int(idx_sat_delay[2])))
    f.close()
    return del_lst, update_lst, add_lst

# name1 in local machine
def _del_link(name1, name2):
    """
    Deletes a network link between two nodes.

    Args:
        name1 (str): Name of the first node.
        name2 (str): Name of the second node.
    """
    n1_n2 = f"{name2}"
    fd = os.open('/run/netns/' + name1, os.O_RDONLY)
    libc.setns(fd, CLONE_NEWNET)
    os.close(fd)
    subprocess.check_call(('ip', 'link', 'del', n1_n2))


def _init_if(name, if_name, addr,addr6, delay, bw, loss,mac_address=None):
    """
    Initializes a network interface with specified parameters.

    Args:
        name (str): Name of the node.
        if_name (str): Name of the interface.
        addr (str): IPv4 address for the interface.
        addr6 (str): IPv6 address for the interface.
        delay (str): Network delay in milliseconds.
        bw (str): Bandwidth in Gbps.
        loss (str): Packet loss percentage.
        mac_address (str, optional): MAC address for the interface.
    """
    fd = os.open('/run/netns/' + name, os.O_RDONLY)
    libc.setns(fd, CLONE_NEWNET)
    os.close(fd)
    # subprocess.check_call(('ip', 'addr', 'add', addr, 'dev', if_name))
    if mac_address is not None:
        subprocess.check_call(('ip', 'link', 'set', if_name, 'address', mac_address))
    subprocess.check_call(('ip', 'addr', 'add', addr6, 'dev', if_name))
    subprocess.check_call(
        ('tc', 'qdisc', 'add', 'dev', if_name, 'root',
         'netem', 'delay', delay+'ms', 'loss', loss+'%', 'rate', bw+'Gbit')
    )
    # srv6 enabled
    subprocess.check_call(('sysctl', '-w', f'net.ipv6.conf.{if_name}.seg6_enabled=1'), stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    subprocess.check_call(('sysctl', '-w', f'net.ipv6.conf.{if_name}.hop_limit=255'), stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    subprocess.check_call(('ip', 'link', 'set', if_name, 'up'))


def _update_if(name, if_name, delay, bw, loss):
    """
    Updates the parameters of an existing network interface.

    Args:
        name (str): Name of the node.
        if_name (str): Name of the interface.
        delay (str): Updated network delay in milliseconds.
        bw (str): Updated bandwidth in Gbps.
        loss (str): Updated packet loss percentage.
    """
    fd = os.open('/run/netns/' + name, os.O_RDONLY)
    libc.setns(fd, CLONE_NEWNET)
    os.close(fd)
    subprocess.check_call(
        ('tc', 'qdisc', 'change', 'dev', if_name, 'root',
        'netem', 'delay', delay + 'ms', 'rate', bw + 'Gbit', 'loss', loss + '%')
    )

def _update_link_intra_machine(name1, name2, delay, bw, loss):
    """
    Updates a link between two nodes on the same machine.

    Args:
        name1 (str): Name of the first node.
        name2 (str): Name of the second node.
        delay (str): Updated network delay in milliseconds.
        bw (str): Updated bandwidth in Gbps.
        loss (str): Updated packet loss percentage.
    """
    n1_n2 = f"{name2}"
    n2_n1 = f"{name1}"
    _update_if(name1, n1_n2, delay, bw, loss)
    _update_if(name2, n2_n1, delay, bw, loss)

# name1 in local machine
def _update_link_local(name1, name2, delay, bw, loss):
    n1_n2 = f"{name2}"
    _update_if(name1, n1_n2, delay, bw, loss)

def load_links_dict(path):
    """
    Loads a dictionary of links from a file.

    Args:
        path (str): Path to the file containing link information.

    Returns:
        dict: A dictionary mapping nodes to their respective links.
    """
    f = open(path, 'r')
    links_dict = {}
    for line in f:
        toks = line.strip().split(':')
        link_lst = []
        for isl in toks[1].split():
            link_lst.append(isl.split(','))
        links_dict[toks[0]] = link_lst
    f.close()
    return links_dict

def _isl_addr6_ips(name1, name2):
    """
    Generates IPv6 addresses for an inter-satellite link (ISL).

    Args:
        name1 (str): Name of the first satellite.
        name2 (str): Name of the second satellite.

    Returns:
        tuple: A tuple containing the IPv6 addresses for both ends of the link.
    """
    sat1_id = int(name1.split('SAT')[-1])
    sat2_id = int(name2.split('SAT')[-1])
    if sat1_id < sat2_id:
        addr6_prefix = f"aaaa:{sat1_id}:aaaa:{sat2_id}::"
        return addr6_prefix+"10/64", addr6_prefix+"40/64"
    else:
        addr6_prefix = f"aaaa:{sat2_id}:aaaa:{sat1_id}::"
        return addr6_prefix+"40/64", addr6_prefix+"10/64"
    
def _isl_mac(name1, name2):
    """
    Generates MAC addresses for an inter-satellite link (ISL).

    Args:
        name1 (str): Name of the first satellite.
        name2 (str): Name of the second satellite.

    Returns:
        tuple: A tuple containing the MAC addresses for both ends of the link.
    """
    sat1_id = int(name1.split('SAT')[-1])
    sat2_id = int(name2.split('SAT')[-1])
    
    if sat1_id < sat2_id:
        mac_prefix = "aa:%02x:%02x:%02x:%02x" % (
            (sat1_id >> 8) & 0xFF,  
            sat1_id & 0xFF,        
            (sat2_id >> 8) & 0xFF,
            sat2_id & 0xFF
        )
        return f"{mac_prefix}:10", f"{mac_prefix}:40"
    else:
        mac_prefix = "aa:%02x:%02x:%02x:%02x" % (
            (sat2_id >> 8) & 0xFF,
            sat2_id & 0xFF,
            (sat1_id >> 8) & 0xFF,
            sat1_id & 0xFF
        )
        return f"{mac_prefix}:40", f"{mac_prefix}:10"

def _gsl_addr6_ips(name1, name2):
    """
    Generates IPv6 addresses for a ground station link (GSL).

    Args:
        name1 (str): Name of the ground station.
        name2 (str): Name of the satellite.

    Returns:
        tuple: A tuple containing the IPv6 addresses for both ends of the link.
    """
    gs_id = name1.split('GS')[-1]
    sat_id = name2.split('SAT')[-1]
    sat_state = load_topo_from_shm(name2)
    cell = sat_state['sat_cell']
    addr6_prefix = f"ce:{cell[0]}:ce:{cell[1]}:{gs_id}::"
    return addr6_prefix+f"{gs_id}/80", addr6_prefix+f"aaaa:{sat_id}/80"

def _gsl_mac(name1, name2):
    """
    Generates MAC addresses for a ground station link (GSL).

    Args:
        name1 (str): Name of the ground station.
        name2 (str): Name of the satellite.

    Returns:
        tuple: A tuple containing the MAC addresses for both ends of the link.
    """
    gs_id = int(name1.split('GS')[-1])
    sat_id = int(name2.split('SAT')[-1])
    mac_prefix = "ce:%02x:%02x:%02x:%02x" % (
            (gs_id >> 8) & 0xFF,  
            gs_id & 0xFF,        
            (sat_id >> 8) & 0xFF,
            sat_id & 0xFF
        )
    return f"{mac_prefix}:10", f"{mac_prefix}:40"


def _add_link_intra_machine(idx, name1, name2, prefix, delay, bw, loss,link_type):
    """
    Adds a network link between two nodes on the same machine.

    Args:
        idx (int): Unique identifier for the link.
        name1 (str): Name of the first node.
        name2 (str): Name of the second node.
        prefix (str): IP address prefix for the link.
        delay (str): Network delay in milliseconds.
        bw (str): Bandwidth in Gbps.
        loss (str): Packet loss percentage.
        link_type (str): Type of the link ("ISL" for inter-satellite, "GSL" for ground station).
    """
    n1_n2 = f"{name2}"
    n2_n1 = f"{name1}"
    libc.setns(main_net_fd, CLONE_NEWNET)
    subprocess.check_call(
        ('ip', 'link', 'add', n1_n2, 'netns', name1,
         'type', 'veth', 'peer', n2_n1, 'netns', name2)
    )
    if link_type == "ISL":
        name1_addr6, name2_addr6 = _isl_addr6_ips(name1, name2)
        name1_mac, name2_mac = _isl_mac(name1, name2)
        _init_if(name1, n1_n2, prefix+'.10/24',name1_addr6, delay, bw, loss, name1_mac)
        _init_if(name2, n2_n1, prefix+'.40/24',name2_addr6, delay, bw, loss, name2_mac)
    if link_type == "GSL":
        name1_addr6, name2_addr6 = _gsl_addr6_ips(name1, name2)
        name1_mac, name2_mac = _gsl_mac(name1, name2)
        _init_if(name1, n1_n2, prefix+'.10/24',name1_addr6, delay, bw, loss, name1_mac)
        _init_if(name2, n2_n1, prefix+'.40/24',name2_addr6, delay, bw, loss, name2_mac)
    
def _add_link_inter_machine(idx, name1, name2, remote_ip, prefix, delay, bw, loss):
    n1_n2 = f"{name2}"
    n2_n1 = f"{name1}"
    subprocess.check_call(
        ('ip', 'link', 'add', n1_n2, 'type', 'vxlan',
         'id', str(idx), 'remote', remote_ip, 'dstport', VXLAN_PORT)
    )
    _init_if(name1, n1_n2, prefix+'.10/24', delay, bw, loss)
    _init_if(name2, n2_n1, prefix+'.40/24', delay, bw, loss)

def sn_init_nodes(dir, sat_mid_dict_shell, gs_mid_dict):
    """
    Initializes containers for satellites and ground stations.

    Args:
        dir (str): Working directory for the simulation.
        sat_mid_dict_shell (list): Mapping of satellites to machines by shell.
        gs_mid_dict (dict): Mapping of ground stations to machines.

    This function creates containers for satellites and ground stations, assigns
    network namespaces, and configures system parameters for forwarding and SRv6.
    """
    def _load_netns(pid, name):
        netns_link = f'/run/netns/{name}'
        if not os.path.exists(netns_link):
            subprocess.check_call(('ln', '-s', f'/proc/{pid}/ns/net', netns_link))
        sn_container_check_call(
            pid,
            ('sysctl', 'net.ipv6.conf.all.forwarding=1'),
            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT
        )
        sn_container_check_call(
            pid, 
            ('sysctl', 'net.ipv4.conf.all.forwarding=1'),
            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT
        )
        sn_container_check_call(
            pid, 
            ('sysctl', 'net.ipv6.conf.all.seg6_enabled=1'),
            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT
        )
        sn_container_check_call(
            pid, 
            ('sysctl', 'net.ipv6.conf.all.hop_limit=255'),
            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT
        )
    subprocess.check_call(('sysctl', 'net.ipv4.neigh.default.gc_thresh1=4096'),stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    subprocess.check_call(('sysctl', 'net.ipv4.neigh.default.gc_thresh2=8192'),stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    subprocess.check_call(('sysctl', 'net.ipv4.neigh.default.gc_thresh3=16384'),stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    subprocess.check_call(('sysctl', 'net.ipv6.neigh.default.gc_thresh1=4096'),stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    subprocess.check_call(('sysctl', 'net.ipv6.neigh.default.gc_thresh2=8192'),stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    subprocess.check_call(('sysctl', 'net.ipv6.neigh.default.gc_thresh3=16384'),stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    subprocess.check_call(('sysctl', 'fs.inotify.max_user_instances=65535'),stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    subprocess.check_call(('sysctl', 'fs.inotify.max_user_watches=65535'),stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)



    pid_file = open(dir + '/' + PID_FILENAME, 'w', encoding='utf-8')
    sat_cnt = 0
    for shell_id, mid_dict in enumerate(sat_mid_dict_shell):
        for node, mid in mid_dict.items():  
            if mid != machine_id:
                pid_file.write(NOT_ASSIGNED + ' ')
                continue
            node_dir = f"{dir}/shell{shell_id}/overlay/{node}"
            sat_cnt += 1
            os.makedirs(node_dir, exist_ok=True)
            pid_file.write(node+':'+str(pyctr.container_run(node_dir, node))+' ')
        pid_file.write('\n')
        print(f'[{machine_id}] shell {shell_id}: {sat_cnt} satellites initialized')
    
    gs_lst = []
    overlay_dir = f"{dir}/GS-{len(gs_mid_dict)}/overlay"
    for node, mid in gs_mid_dict.items():
        if mid != machine_id:
            pid_file.write(NOT_ASSIGNED + ' ')
            continue
        gs_lst.append(node)
        node_dir = f'{overlay_dir}/{node}'
        os.makedirs(node_dir, exist_ok=True)
        pid_file.write(node+':'+str(pyctr.container_run(node_dir, node))+' ')
    pid_file.write('\n')
    print(f'[{machine_id}] GS:', ','.join(gs_lst))

    pid_file.close()
    sn_operate_every_node(dir, _load_netns)


def sn_init_network_muti(
        dir, ts, sat_mid_dict_shell, gs_mid_dict, ip_lst,
        isl_bw, isl_loss, gsl_bw, gsl_loss
    ):
    """
    Initializes the network configuration for satellites and ground stations.

    Args:
        dir (str): Working directory for the simulation.
        ts (int): Timestamp for the network initialization.
        sat_mid_dict_shell (list): Mapping of satellites to machines by shell.
        gs_mid_dict (dict): Mapping of ground stations to machines.
        ip_lst (list): List of machine IP addresses.
        isl_bw (str): Bandwidth for inter-satellite links.
        isl_loss (str): Packet loss for inter-satellite links.
        gsl_bw (str): Bandwidth for ground station links.
        gsl_loss (str): Packet loss for ground station links.

    This function creates and configures network links for satellites and ground stations.
    """
    with ProcessPoolExecutor(max_workers=256) as executor:
        for shell_id, mid_dict in enumerate(sat_mid_dict_shell):
            shell_dir = f"{dir}/shell{shell_id}"
            if not os.path.exists(shell_dir):
                continue
            del_cnt, update_cnt, add_cnt = 0, 0, 0
            del_lst, update_lst, add_lst = _parse_isls(f'{shell_dir}/isl/init.txt')
            for sat_name, isl_sat, delay, idx in add_lst:
                if mid_dict[sat_name] == machine_id:
                    add_cnt += 1
                    if mid_dict[isl_sat] == machine_id:
                        executor.submit(_add_link_intra_machine, idx, sat_name, isl_sat,f'10.{idx >> 8}.{idx & 0xFF}', delay, isl_bw, isl_loss,"ISL")
                    else:
                        executor.submit(_add_link_inter_machine, idx, sat_name, isl_sat, ip_lst[mid_dict[isl_sat]],f'10.{idx >> 8}.{idx & 0xFF}', delay, isl_bw, isl_loss)
                elif mid_dict[isl_sat] == machine_id:
                    add_cnt += 1
                    executor.submit(_add_link_inter_machine, idx, isl_sat, sat_name, ip_lst[mid_dict[sat_name]],f'10.{idx >> 8}.{idx & 0xFF}', delay, isl_bw, isl_loss)
            print(f"[{machine_id}] Shell {shell_id}:",
                f"{add_cnt} added.")
    for shell_id, mid_dict in enumerate(sat_mid_dict_shell):
        for node, mid in mid_dict.items():
            resources_dir = f"{dir}/shell{shell_id}/overlay/{node}/resources"
            os.makedirs(resources_dir, exist_ok=True)
            with open(f"{resources_dir}/link_change.txt",'w') as f:
                f.write("0")


def sn_update_network_muti(
        dir, ts, sat_mid_dict_shell, gs_mid_dict, ip_lst,
        isl_bw, isl_loss, gsl_bw, gsl_loss,failure=False
    ):
    """
    Updates the network configuration for satellites and ground stations.

    Args:
        dir (str): Working directory for the simulation.
        ts (int): Timestamp for the network update.
        sat_mid_dict_shell (list): Mapping of satellites to machines by shell.
        gs_mid_dict (dict): Mapping of ground stations to machines.
        ip_lst (list): List of machine IP addresses.
        isl_bw (str): Bandwidth for inter-satellite links.
        isl_loss (str): Packet loss for inter-satellite links.
        gsl_bw (str): Bandwidth for ground station links.
        gsl_loss (str): Packet loss for ground station links.
        failure (bool): Indicates whether the update is triggered by a failure recovery.

    This function updates the network links for satellites and ground stations based on
    the current timestamp and configuration files.
    """
    with ProcessPoolExecutor(max_workers=256) as executor:
        for shell_id, mid_dict in enumerate(sat_mid_dict_shell):
            shell_dir = f"{dir}/shell{shell_id}"
            if not os.path.exists(shell_dir):
                continue
            del_cnt, update_cnt, add_cnt = 0, 0, 0
            del_lst, update_lst, add_lst = _parse_isls(f'{shell_dir}/isl/{ts}.txt')
            for sat_name, isl_sat in del_lst:
                if mid_dict[sat_name] == machine_id:
                    del_cnt += 1
                    executor.submit(_del_link, sat_name, isl_sat)
                elif mid_dict[isl_sat] == machine_id:
                    del_cnt += 1
                    executor.submit(_del_link, isl_sat, sat_name)
            for sat_name, isl_sat, delay in update_lst:
                if mid_dict[sat_name] == machine_id:
                    update_cnt += 1
                    if mid_dict[isl_sat] == machine_id:
                        executor.submit(_update_link_intra_machine, sat_name, isl_sat, delay, isl_bw, isl_loss)
                    else:
                        executor.submit(_update_link_local,sat_name, isl_sat, delay, isl_bw, isl_loss)
                elif mid_dict[isl_sat] == machine_id:
                    update_cnt += 1
                    executor.submit(_update_link_local, isl_sat, sat_name, delay, isl_bw, isl_loss)
            for sat_name, isl_sat, delay, idx in add_lst:
                if mid_dict[sat_name] == machine_id:
                    add_cnt += 1
                    if mid_dict[isl_sat] == machine_id:
                        executor.submit(_add_link_intra_machine, idx, sat_name, isl_sat,f'10.{idx >> 8}.{idx & 0xFF}', delay, isl_bw, isl_loss,"ISL")
                    else:
                        executor.submit(_add_link_inter_machine, idx, sat_name, isl_sat, ip_lst[mid_dict[isl_sat]],f'10.{idx >> 8}.{idx & 0xFF}', delay, isl_bw, isl_loss)
                elif mid_dict[isl_sat] == machine_id:
                    add_cnt += 1
                    executor.submit(_add_link_inter_machine, idx, isl_sat, sat_name, ip_lst[mid_dict[sat_name]],f'10.{idx >> 8}.{idx & 0xFF}', delay, isl_bw, isl_loss)
            print(f"[{machine_id}] Shell {shell_id}:",
                f"{del_cnt} deleted, {update_cnt} updated, {add_cnt} added.")
        if failure:
            return
        gs_dir = f"{dir}/GS-{len(gs_mid_dict)}"
        if not os.path.exists(gs_dir):
            return
        del_cnt, update_cnt, add_cnt = 0, 0, 0
        del_lst, update_lst, add_lst = _parse_gsls(f'{gs_dir}/gsl/{ts}.txt')

        sat_mid_dict = sat_mid_dict_shell[0]

        for gs_name, gsl_sat in del_lst:
            if gs_mid_dict[gs_name] == machine_id:
                del_cnt += 1
                executor.submit(_del_link, gs_name, gsl_sat)
            elif sat_mid_dict[gsl_sat] == machine_id:
                del_cnt += 1
                executor.submit(_del_link, gsl_sat, gs_name)
        for gs_name, gsl_sat, delay in update_lst:
            if gs_mid_dict[gs_name] == machine_id:
                update_cnt += 1
                if sat_mid_dict[gsl_sat] == machine_id:
                    executor.submit(_update_link_intra_machine,gs_name, gsl_sat, delay, gsl_bw, gsl_loss)
                else:
                    executor.submit(_update_link_local,gs_name, gsl_sat, delay, gsl_bw, gsl_loss)
            elif sat_mid_dict[gsl_sat] == machine_id:
                update_cnt += 1
                executor.submit(_update_link_local, gsl_sat, gs_name, delay, gsl_bw, gsl_loss)
        for gs_name, gsl_sat, delay, idx in add_lst:
            if gs_mid_dict[gs_name] == machine_id:
                add_cnt += 1
                if sat_mid_dict[gsl_sat] == machine_id:
                    executor.submit(_add_link_intra_machine, idx, gs_name, gsl_sat,f'9.{idx >> 8}.{idx & 0xFF}', delay, gsl_bw, gsl_loss,"GSL")
                else:
                    executor.submit(_add_link_inter_machine, idx, gs_name, gsl_sat, ip_lst[gs_mid_dict[gsl_sat]],f'9.{idx >> 8}.{idx & 0xFF}', delay, gsl_bw, gsl_loss)
            elif sat_mid_dict[gsl_sat] == machine_id:
                add_cnt += 1
                executor.submit(_add_link_inter_machine, idx, gsl_sat, gs_name, ip_lst[gs_mid_dict[gs_name]],f'9.{idx >> 8}.{idx & 0xFF}', delay, gsl_bw, gsl_loss)

    print(f"[{machine_id}] GSL:",
          f"{del_cnt} deleted, {update_cnt} updated, {add_cnt} added.")
    return


def sn_container_check_call(pid, cmd, *args, **kwargs):
    subprocess.check_call(
        ('nsenter', '-m', '-u', '-i', '-n', '-p', '-t', pid, *cmd),
        *args, **kwargs
    )

def sn_container_run(pid, cmd, *args, **kwargs):
    subprocess.run(
        ('nsenter', '-m', '-u', '-i', '-n', '-p', '-t', pid, *cmd),
        *args, **kwargs
    )

def sn_container_check_output(pid, cmd, *args, **kwargs):
    return subprocess.check_output(
        ('nsenter', '-m', '-u', '-i', '-n', '-p', '-t', pid, *cmd),
        *args, **kwargs
    )

def sn_operate_every_node(dir, func, *args):
    pid_map = _pid_map(dir + '/' + PID_FILENAME)
    with ThreadPoolExecutor(max_workers=50) as executor:
        for name, pid in pid_map.items():
            executor.submit(func, pid, name, *args)

def sn_ping(dir, src_gs, dst_gs):
    """
    Executes a ping test from a source ground station to a destination ground station.

    Args:
        dir (str): Working directory for the simulation.
        src_gs (str): Name of the source ground station.
        dst_gs (str): Name of the destination ground station.
    """
    pid_map = _pid_map(f"{dir}/{PID_FILENAME}")
    # suppose src in this machine
    src_pid = pid_map[src_gs]
    # dst_pid = pid_map[dst_gs]
    dst_data = load_topo_from_shm(dst_gs)
    dst_addr = dst_data['gs_ip']
    subprocess.run(
        ('nsenter', '-m', '-u', '-i', '-n', '-p', '-t', src_pid,
         'ping', '-6', dst_addr,'-i','0.005','-c','500', '-D'),
         stdout=sys.stdout, stderr=subprocess.STDOUT
    )

def sn_traceroute(dir, src_gs, dst_gs):
    """
    Executes a traceroute test from a source ground station to a destination ground station.

    Args:
        dir (str): Working directory for the simulation.
        src_gs (str): Name of the source ground station.
        dst_gs (str): Name of the destination ground station.
    """
    pid_map = _pid_map(f"{dir}/{PID_FILENAME}")
    # suppose src in this machine
    src_pid = pid_map[src_gs]
    # dst_pid = pid_map[dst_gs]
    dst_data = load_topo_from_shm(dst_gs)
    dst_addr = dst_data['gs_ip']
    subprocess.run(
        ('nsenter', '-m', '-u', '-i', '-n', '-p', '-t', src_pid,
         'traceroute', '-6', '-q','1',"-m","128", dst_addr),
         stdout=sys.stdout, stderr=subprocess.STDOUT
    )

def sn_iperf(dir, src_gs, dst_gs):
    """
    Executes an iperf test between a source ground station and a destination ground station.

    Args:
        dir (str): Working directory for the simulation.
        src_gs (str): Name of the source ground station.
        dst_gs (str): Name of the destination ground station.
    """
    pid_map = _pid_map(f"{dir}/{PID_FILENAME}")
    # suppose src in this machine
    src_pid = pid_map[src_gs]
    dst_pid = pid_map[dst_gs]
    dst_data = load_topo_from_shm(dst_gs)
    dst_addr = dst_data['gs_ip']
    server = subprocess.Popen(
        ('nsenter', '-m', '-u', '-i', '-n', '-p', '-t', dst_pid,
         'iperf3', '-s'),
         stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT
    )

    subprocess.run(
        ('nsenter', '-m', '-u', '-i', '-n', '-p', '-t', src_pid,
         'iperf3', '-c', dst_addr, '-u', '-b', '10M', '-i', '1', '-t', '10'),
         stdout=sys.stdout, stderr=subprocess.STDOUT
    )
    server.terminate()

def sn_clean(dir, sat_mid_dict_shell, gs_mid_dict):
    """
    Cleans up the simulation environment by removing containers, namespaces, and overlay directories.

    Args:
        dir (str): Working directory for the simulation.
        sat_mid_dict_shell (list): Mapping of satellites to machines by shell.
        gs_mid_dict (dict): Mapping of ground stations to machines.

    This function performs the following cleanup tasks:
    - Removes network namespace links for satellites and ground stations.
    - Terminates all container processes.
    - Deletes the PID file and overlay directories for satellites and ground stations.
    """
    for ns_link in glob.glob(f"/run/netns/SH*S*"):
        if os.path.islink(ns_link):
            os.remove(ns_link)
    for ns_link in glob.glob(f"/run/netns/G*"):
        if os.path.islink(ns_link):
            os.remove(ns_link)
    pid_file = f"{dir}/{PID_FILENAME}"
    if not os.path.exists(pid_file):
        return
    pid_map = _pid_map(pid_file, True)
    for pid in pid_map.values():
        if pid == NOT_ASSIGNED:
            continue
        try:
            os.kill(int(pid), 9)
        except ProcessLookupError:
            pass
    os.remove(pid_file)

    for shell_id, mid_dict in enumerate(sat_mid_dict_shell):
        overlay_dir = f"{dir}/shell{shell_id}"
        try:
            shutil.rmtree(overlay_dir)
        except FileNotFoundError:
            pass
    
    overlay_dir = f"{dir}/GS-{len(gs_mid_dict)}"
    try:
        shutil.rmtree(overlay_dir)
    except FileNotFoundError:
        pass
    print(f"[{machine_id}] Cleaned up!")


def sat_link_change(workdir,sat):
    """
    Marks a satellite's link state as changed.

    Args:
        workdir (str): Working directory for the simulation.
        sat (str): Name of the satellite.

    This function writes a "1" to the satellite's `link_change.txt` file to indicate
    that its link state has been updated.
    """
    with open(f"{workdir}/shell0/overlay/{sat}/resources/link_change.txt",'w') as f:
        f.write("1")

def fault_test(workdir, ts, sat_mid_dict_shell, gs_mid_dict, ip_lst,
                isl_bw, isl_loss, gsl_bw, gsl_loss):
    """
    Simulates a fault test by introducing a link failure between two satellites and updating the network state.

    Args:
        workdir (str): Working directory for the simulation.
        ts (int): Timestamp for the fault test.
        sat_mid_dict_shell (list): Mapping of satellites to machines by shell.
        gs_mid_dict (dict): Mapping of ground stations to machines.
        ip_lst (list): List of machine IP addresses.
        isl_bw (str): Bandwidth for inter-satellite links.
        isl_loss (str): Packet loss for inter-satellite links.
        gsl_bw (str): Bandwidth for ground station links.
        gsl_loss (str): Packet loss for ground station links.

    Steps:
        1. Define the two satellites (`sat1` and `sat2`) involved in the link failure.
        2. Remove the inter-satellite link (ISL) between `sat1` and `sat2`:
            - Update the shared memory for both satellites to reflect the link removal.
        3. Mark the link state of the affected satellites as changed using `sat_link_change`.
        4. Report the link failure to the SDN controller using `_failure_report`.
        5. If the SDN controller provides updated satellite states:
            - Update the link state for the affected satellites using `update_link_state`.
            - Reconfigure the network using `sn_update_network_muti`.
            - Mark the updated satellites' link states as changed again.

    Returns:
        None
    """
    # local link change
    sat1 = "SH1SAT1596"
    sat2 = 'SH1SAT1483'
    link_change_sats = [sat1,sat2]

    sat1_state = load_topo_from_shm(sat1)
    sat1_state['isls'].pop(sat2, None)
    replace_shared_memory(sat1, pickle.dumps(sat1_state))
    # with open(f"{workdir}/shell0/overlay/{sat1}/resources/isl_state/{ts}.json",'r') as f:
    #     isls = json.load(f)
    #     isls.pop(sat2, None)
    # with open(f"{workdir}/shell0/overlay/{sat1}/resources/isl_state/{ts}.json", 'w') as f:
    #     json.dump(isls, f)
    
    sat2_state = load_topo_from_shm(sat2)
    sat2_state['isls'].pop(sat1, None)
    replace_shared_memory(sat2, pickle.dumps(sat2_state))
    # with open(f"{workdir}/shell0/overlay/{sat2}/resources/isl_state/{ts}.json",'r') as f:
    #     isls = json.load(f)
    #     isls.pop(sat1, None)
    # with open(f"{workdir}/shell0/overlay/{sat2}/resources/isl_state/{ts}.json", 'w') as f:
    #     json.dump(isls, f)

    for sat in link_change_sats:
        # print(f"sat {sat} link change",time.time())
        sat_link_change(workdir,sat)
    
    # report to sdn
    update_sats =_failure_report(sat1, sat2)
    # failure recovery
    if update_sats:
        update_link_state(
            workdir, ts, sat_mid_dict_shell, gs_mid_dict, update_sats)
        sn_update_network_muti(
            workdir, ts, sat_mid_dict_shell, gs_mid_dict, ip_lst,
            isl_bw, isl_loss, gsl_bw, gsl_loss,failure=True
        )
        for sat in update_sats:
            sat_link_change(workdir,sat)
    
def replace_shared_memory(name, data):
    """
    Replaces the shared memory for a given satellite or ground station.

    Args:
        name (str): Name of the shared memory segment.
        data (bytes): Data to write to the shared memory.

    This function creates or replaces a shared memory segment with the given data.
    """
    try:
        shm = shared_memory.SharedMemory(name=name)
        shm.close()
        shm.unlink()
        shm = shared_memory.SharedMemory(name=name, create=True, size=len(data))
        shm.buf[:len(data)] = data
    except FileNotFoundError:
        shm = shared_memory.SharedMemory(name=name, create=True, size=len(data))
        shm.buf[:len(data)] = data
    finally:
        shm.close()
        resource_tracker.unregister(shm._name, "shared_memory")

def load_topo_from_shm(name):
    """
    Loads topology data from shared memory.

    Args:
        name (str): Name of the shared memory segment.

    Returns:
        dict: The topology data loaded from shared memory.
    """
    # start_time = time.time()
    shm = shared_memory.SharedMemory(name=name)
    data = bytes(shm.buf[:])
    shm.close()
    resource_tracker.unregister(shm._name, "shared_memory")
    data = pickle.loads(data)
    # print(f"Time taken to load from shared memory: {time.time() - start_time}")
    return data

def update_link_state(dir,ts,sat_mid_dict_shell, gs_mid_dict, update_sats = None):
    """
    Updates the link state of satellites and ground stations based on the latest topology.

    Args:
        dir (str): Working directory for the simulation.
        ts (int): Timestamp for the update.
        sat_mid_dict_shell (list): Mapping of satellites to machines by shell.
        gs_mid_dict (dict): Mapping of ground stations to machines.
        update_sats (list, optional): List of satellites to update. If None, all satellites are updated.

    Steps:
        1. Load the latest node states from the JSON file for the given timestamp.
        2. For each satellite:
            - Check if the satellite belongs to the current machine.
            - If `update_sats` is provided, only update the specified satellites.
            - Replace the shared memory for the satellite with the updated state.
        3. If `update_sats` is None, update the ground station states:
            - Check if the ground station belongs to the current machine.
            - Replace the shared memory for the ground station with the updated state.

    Returns:
        None
    """
    sat_cnt = 0
    with open(f"{dir}/all_node_states/{ts}.json", 'r') as f:
        all_node_states = json.load(f)

    for shell_id, mid_dict in enumerate(sat_mid_dict_shell):
        for node, mid in mid_dict.items():
            if mid != machine_id:
                continue
            if update_sats is not None and node not in update_sats:
                continue
            sat_cnt += 1
            sat_state = all_node_states[node]

            # sat_isl_dir = f"{dir}/shell{shell_id}/overlay/{node}/resources/isl_state"
            # os.makedirs(sat_isl_dir,exist_ok=True)
            # json.dump(sat_state['isls'], open(f"{sat_isl_dir}/{ts}.json", 'w'))

            # cell_ring_dir = f"{dir}/shell{shell_id}/overlay/{node}/resources/cell_ring"
            # os.makedirs(cell_ring_dir,exist_ok=True)
            # json.dump(sat_state['cell_ring'], open(f"{cell_ring_dir}/{ts}.json", 'w'))

            # sat_cell_dir = f"{dir}/shell{shell_id}/overlay/{node}/resources/sat_cell"
            # os.makedirs(sat_cell_dir, exist_ok=True)
            # json.dump(sat_state['sat_cell'], open(f"{sat_cell_dir}/{ts}.json", 'w'))

            # inter_cell_isl_dir = f"{dir}/shell{shell_id}/overlay/{node}/resources/inter_cell_isl"
            # os.makedirs(inter_cell_isl_dir, exist_ok=True)
            # json.dump(sat_state['inter_cell_isls'], open(f"{inter_cell_isl_dir}/{ts}.json", 'w'))

            # sat_gsl_dir = f"{dir}/shell{shell_id}/overlay/{node}/resources/gsl_state"
            # os.makedirs(sat_gsl_dir, exist_ok=True)
            # json.dump(sat_state['gsls'], open(f"{sat_gsl_dir}/{ts}.json", 'w'))

            replace_shared_memory(node,pickle.dumps(sat_state))

        print(f'[{machine_id}] shell {shell_id}: {sat_cnt} Satellites state update!')
    
    if update_sats is not None:
        return
    
    gs_cnt = 0
    for node, mid in gs_mid_dict.items():
        if mid != machine_id:
            continue
        gs_cnt += 1
        gs_state = all_node_states[node]
        replace_shared_memory(node,pickle.dumps(gs_state))

        # gs_link_dir = f"{dir}/GS-{len(gs_mid_dict)}/overlay/{node}/resources/gsl_state"
        # os.makedirs(gs_link_dir,exist_ok=True)
        # json.dump(gs_state['gsls'], open(f"{gs_link_dir}/{ts}.json", 'w'))

    print(f'[{machine_id}] GS:', gs_cnt, 'GSs state update!')

def update_ts(workdir,ts):
    """
    Updates the current timestamp for the simulation.

    Args:
        workdir (str): Working directory for the simulation.
        ts (int): New timestamp to set.
    """
    with open(f"{workdir}/controller/ts.txt", 'w') as f:
        f.write(ts)

def flush_route(workdir):
    """
    Triggers a route flush in the simulation.

    Args:
        workdir (str): Working directory for the simulation.
    """
    with open(f"{workdir}/controller/flush.txt", 'w') as f:
        f.write("1")

def get_ts(workdir):
    """
    Retrieves the current timestamp for the simulation.
    """
    with open(f"{workdir}/controller/ts.txt", 'r') as f:
        return int(f.read().strip())
    

if __name__ == '__main__':
    _pid_map_cache = None
    if len(sys.argv) < 2:
        print('Usage: sn_remote.py <command> ...')
        exit(1)
    cmd = sys.argv[1]
    if cmd == 'exec':
        pid_map = _pid_map(os.path.dirname(__file__) + '/' + PID_FILENAME)
        if len(sys.argv) < 4:
            print('Usage: sn_remote.py exec <node> <command> ...')
            exit(1)
        if sys.argv[2] not in pid_map:
            print('Error:', sys.argv[3], 'not found')
            exit(1)
        exit(subprocess.run(
            ('nsenter', '-a', '-t', pid_map[sys.argv[2]],
            *sys.argv[3:])
        ).returncode)

    if len(sys.argv) < 3:
        machine_id = None
    else:
        try:
            machine_id = int(sys.argv[2])
        except:
            machine_id = None
    if len(sys.argv) < 4:
        workdir = os.path.dirname(__file__)
    else:
        workdir = sys.argv[3]

    # C module
    try:
        import pyctr
    except ModuleNotFoundError:
        subprocess.check_call(
            "cd " + workdir + " && "
            "gcc $(python3-config --cflags --ldflags)"
            "-shared -fPIC -O2 pyctr.c -o pyctr.so",
            shell=True
        )
        import pyctr
    
    sat_mid_dict_shell, gs_mid_dict, ip_lst = _get_params(workdir + '/' + ASSIGN_FILENAME)
    if cmd == 'nodes':
        sn_clean(workdir, sat_mid_dict_shell, gs_mid_dict)
        sn_init_nodes(workdir, sat_mid_dict_shell, gs_mid_dict)
    elif cmd == 'list':
        print(f"{'NODE':<20} STATE")
        for name in _pid_map(workdir + '/' + PID_FILENAME):
            print(f"{name:<20} {'OK'}")
    elif cmd == 'networks':
        ts, isl_bw, isl_loss, gsl_bw, gsl_loss = sys.argv[4], sys.argv[5], sys.argv[6], sys.argv[7], sys.argv[8]
        if ts == '-1':
            sn_init_network_muti(
                workdir, ts, sat_mid_dict_shell, gs_mid_dict, ip_lst,
                isl_bw, isl_loss, gsl_bw, gsl_loss
            )
        else:
            update_link_state(workdir,ts, sat_mid_dict_shell, gs_mid_dict)
            sn_update_network_muti(
                workdir, ts, sat_mid_dict_shell, gs_mid_dict, ip_lst,
                isl_bw, isl_loss, gsl_bw, gsl_loss
            )
            update_ts(workdir,ts)
    elif cmd == 'update_ts':
        update_ts(workdir,sys.argv[4])
    elif cmd == 'flush_route':
        flush_route(workdir)
    elif cmd == 'clean':
        sn_clean(workdir, sat_mid_dict_shell, gs_mid_dict)
    elif cmd == 'ping':
        sn_ping(workdir, sys.argv[4], sys.argv[5])
    elif cmd == 'iperf':
        sn_iperf(workdir, sys.argv[4], sys.argv[5])
    elif cmd == 'traceroute':
        sn_traceroute(workdir, sys.argv[4], sys.argv[5])
    elif cmd == 'fault_test':
        ts, isl_bw, isl_loss, gsl_bw, gsl_loss = sys.argv[4], sys.argv[5], sys.argv[6], sys.argv[7], sys.argv[8]
        fault_test(
            workdir, ts, sat_mid_dict_shell, gs_mid_dict, ip_lst,
            isl_bw, isl_loss, gsl_bw, gsl_loss
        )
    else:
        print('Unknown command')
    os.close(main_net_fd)
