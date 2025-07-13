"""
TinyLEO Southbound Utilities

This module provides utility functions for managing the southbound interface 
of the TinyLEO simulation environment.
"""
import os
import json
import argparse
import paramiko
import sys
import glob
import numpy as np
import json
import copy
from concurrent import futures
import grpc
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__))))
from link_failure_grpc import link_failure_pb2,link_failure_pb2_grpc

class LinkFailureService(link_failure_pb2_grpc.LinkFailureServiceServicer):
    """
    gRPC service for handling link failures between satellites.

    This service communicates with the SDN controller to process link failure events
    and retrieve updated satellite states.

    Args:
        remote_controller (RemoteController): An instance of the RemoteController class
        responsible for managing satellite states and handling link failures.
    Methods:
        HandleLinkFailure(request, context):
            Handles a gRPC request to process a link failure between two satellites.
    """
    def __init__(self, remote_controller):
        """
        Initializes the LinkFailureService with a remote controller.

        Args:
            remote_controller (RemoteController): The controller responsible for managing
            satellite states and handling link failures.
        """
        self.remote_controller = remote_controller

    def HandleLinkFailure(self, request, context):
        """
        Handles a gRPC request to process a link failure between two satellites.

        Args:
            request (LinkFailureRequest): The gRPC request containing the IDs of the two satellites.
            context (grpc.ServicerContext): The gRPC context for the request.

        Returns:
            LinkFailureResponse: A response indicating success or failure, along with a message.
        """
        if len(request.satellite_ids) != 2:
            return link_failure_pb2.LinkFailureResponse(
                success=False,
                message="Exactly two satellite IDs must be provided."
            )
        
        sat1, sat2 = request.satellite_ids
        print(f"Handling link failure between {sat1} and {sat2}")
        try:
            update_sats = self.remote_controller.handle_link_failure(sat1, sat2)
            return link_failure_pb2.LinkFailureResponse(
                success=True,
                message=",".join(update_sats)
            )
        except Exception as e:
            return link_failure_pb2.LinkFailureResponse(
                success=False,
                message=f"Failed to handle link failure: {str(e)}"
            )
    
def link_faliure_server(remote_controller):
    """
    Starts a gRPC server for the LinkFailureService.

    Args:
        remote_controller (RemoteController): The controller responsible for managing
        satellite states and handling link failures.

    Returns:
        grpc.Server: The running gRPC server instance.
    """
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=1))
    link_failure_pb2_grpc.add_LinkFailureServiceServicer_to_server(LinkFailureService(remote_controller), server)
    server.add_insecure_port('[::]:50051')
    print("Link faliure server is running on port 50051...\n")
    server.start()
    return server

def sn_load_file(path):
    """
    Loads a configuration file and parses command-line arguments.

    Args:
        path (str): Path to the JSON configuration file.

    Returns:
        argparse.Namespace: Parsed arguments containing configuration values.

    This function reads a JSON configuration file, extracts relevant fields, 
    and sets up command-line arguments with default values from the file.
    """
    f = open(path, 'r', encoding='utf8')
    table = json.load(f)
    parser = argparse.ArgumentParser(description='manual to this script')
    parser.add_argument('--cons_name', type=str, default=table['Name'])
    parser.add_argument('--link_style', type=str, default=table['Satellite link'])
    parser.add_argument('--link_policy', type=str, default=table['Link policy'])
    # link delay updating granularity
    parser.add_argument('--duration', type=int, default=(table['Duration (s)'] if 'Duration (s)' in table else 0))
    parser.add_argument('--sat_bandwidth',
                        type=int,
                        default=table['satellite link bandwidth ("X" Gbps)'])
    parser.add_argument('--sat_ground_bandwidth',
                        type=int,
                        default=table['sat-ground bandwidth ("X" Gbps)'])
    parser.add_argument('--sat_loss',
                        type=int,
                        default=table['satellite link loss ("X"% )'])
    parser.add_argument('--sat_ground_loss',
                        type=int,
                        default=table['sat-ground loss ("X"% )'])
    parser.add_argument('--antenna_number',
                        type=int,
                        default=table['antenna number'])
    parser.add_argument('--antenna_elevation',
                        type=int,
                        default=table['antenna elevation angle'])
    parser.add_argument('--topo_dir',
                        type=str,
                        default=table['topo_dir'])
    sn_args = parser.parse_args()
    sn_args.__setattr__('machine_lst', table['Machines'])
    return sn_args

def sn_connect_remote(host, port, username, password):
    """
    Establishes an SSH connection to a remote server.

    Args:
        host (str): Hostname or IP address of the remote server.
        port (int): Port number for the SSH connection.
        username (str): Username for authentication.
        password (str): Password for authentication.

    Returns:
        tuple: A tuple containing the SSH client and SFTP client objects.

    This function sets up an SSH connection and an SFTP session for file transfers.
    """
    remote_ssh = paramiko.SSHClient()
    remote_ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    remote_ssh.connect(hostname=host, port=port, username=username, password=password)
    return remote_ssh, remote_ssh.open_sftp()

def sn_remote_cmd(remote_ssh, cmd):
    """
    Executes a command on a remote server via SSH.

    Args:
        remote_ssh (paramiko.SSHClient): SSH client object.
        cmd (str): Command to execute on the remote server.

    Returns:
        str: Output of the executed command.

    This function runs a command on the remote server and returns its output.
    """
    return remote_ssh.exec_command(f"bash -i -c '{cmd}'")[1].read().decode().strip()

def sn_remote_wait_output(remote_ssh, cmd):
    """
    Executes a command on a remote server and streams its output.

    Args:
        remote_ssh (paramiko.SSHClient): SSH client object.
        cmd (str): Command to execute on the remote server.

    This function runs a command on the remote server and prints its output line by line.
    """
    for line in remote_ssh.exec_command(f"bash -i -c '{cmd}'", get_pty=True)[1]:
        print(line, end='')

def sn_check_utility(time_index, remote_ssh, local_dir):
    """
    Checks system utility statistics on a remote server.

    Args:
        time_index (int): Timestamp or index for the utility check.
        remote_ssh (paramiko.SSHClient): SSH client object.
        local_dir (str): Local directory to save the utility statistics.

    This function retrieves system utility statistics (e.g., CPU, memory) using `vmstat` 
    and saves the output to a local file.
    """
    result = sn_remote_cmd(remote_ssh, "vmstat")
    f = open(os.path.join(local_dir, f"utility-info_{time_index}.txt"), "w")
    f.write(result)
    f.close()

def upload_folder(sftp, local_folder, remote_folder):
    """
    Recursively uploads a local folder to a remote server.

    Args:
        sftp (paramiko.SFTPClient): SFTP client object.
        local_folder (str): Path to the local folder.
        remote_folder (str): Path to the remote folder.

    This function uploads all files and subdirectories from a local folder 
    to a specified remote folder.
    """
    try:
        sftp.mkdir(remote_folder) 
    except IOError:
        pass  
    for item in os.listdir(local_folder):
        local_path = os.path.join(local_folder, item)
        remote_path = os.path.join(remote_folder, item)

        if os.path.isfile(local_path):
            sftp.put(local_path, remote_path)
        elif os.path.isdir(local_path):
            upload_folder(sftp, local_path, remote_path)

def to_cbf(lat_long):# the xyz coordinate system.
    """
    Converts geographic coordinates (latitude, longitude, altitude) to Cartesian coordinates.

    Args:
        lat_long (array-like): Array of geographic coordinates in the format 
                               [latitude, longitude, altitude].

    Returns:
        numpy.ndarray: Cartesian coordinates in the format [x, y, z].

    This function assumes a spherical Earth model with a radius of 6371 km.
    """
    lat_long = np.array(lat_long)
    radius = 6371
    if lat_long.shape[-1] > 2:
        radius += lat_long[..., 2]
    theta_mat = np.radians(lat_long[..., 0])
    phi_mat = np.radians(lat_long[..., 1])
    z_mat = radius * np.sin(theta_mat)
    rho_mat = radius * np.cos(theta_mat)
    x_mat = rho_mat * np.cos(phi_mat)
    y_mat = rho_mat * np.sin(phi_mat)
    return np.stack((x_mat, y_mat, z_mat), -1)

# def _bound_gsl(antenna_elevation, altitude):
#     a = 6371 * np.cos(np.radians(90 + antenna_elevation))
#     return a + np.sqrt(np.square(a) + np.square(altitude) + 2 * altitude * 6371)

def get_gs_name(gid):
    return f'GS{gid+1}'

def get_satellite_name(satellite_id):
    return f'SH1SAT{satellite_id+1}'

def get_satellite_id(satellite_name):
    return int(satellite_name.split('SAT')[-1]) - 1

def _update_tinyleo_isl(dir,ts,shell_lst,all_node_states):
    """
    Updates the inter-satellite link (ISL) states for each shell.

    Args:
        dir (str): Working directory for the simulation.
        ts (int): Current timestamp.
        shell_lst (list): List of shell configurations.
        all_node_states (dict): Dictionary to store the state of all nodes.

    Returns:
        list: A list containing ISL information for each shell.
    """
    isl_per_shell = []
    for i, shell in enumerate(shell_lst):
        name_lst = []
        sat_lla = []
        sat_names = []

        for sid, node in enumerate(shell['position']):
            # TODO : del
            lla = (node['latitude'], node['longitude'], node['altitude'])
            sat_lla.append(lla)
            name = f'SH{i+1}SAT{sid+1}'
            sat_names.append(name)

        sat_cbf = to_cbf(sat_lla)   # np array, sat_num * 3
        for sid, node in enumerate(shell['position']):
            name = sat_names[sid]
            all_node_states[name] = {'position': {},'ts':ts,'isls': {},'gsls': {},'cell_ring':[],'sat_cell':[],'inter_cell_isls':{}}
            all_node_states[name]['position']['lla'] = sat_lla[sid]
            all_node_states[name]['position']['cbf'] = list(sat_cbf[sid])
            
        name_lst = sat_names
        isls = [list() for _ in range(len(name_lst))]
        for link in shell['links']:
            sid1, sid2 = link['sat1'], link['sat2']
            if sid1 > sid2:
                sid1, sid2 = sid2, sid1
            delay = np.sqrt(np.sum(np.square(sat_cbf[sid1] - sat_cbf[sid2])))/(299792.458) * 1000 #17.31 / 29.5 * 
            isls[sid1].append((name_lst[sid2], delay))

        isl_per_shell.append([shell['name'], name_lst, sat_cbf, isls])        
    return isl_per_shell


def init_tinyleo_links(dir,shell_lst,gs_dirname):
    """
    Initializes the inter-satellite links (ISLs) for the simulation.

    Args:
        dir (str): Working directory for the simulation.
        shell_lst (list): List of shell configurations.
        gs_dirname (str): Directory name for ground station links.

    Returns:
        tuple: A tuple containing satellite names, link states, and link count.
    """
    topo_init_shell = []
    for i, shell in enumerate(shell_lst):
        init_isls = [{} for _ in range(len(shell['timeslots'][0]['position']))]
        name_lst = []
        for t, slot in enumerate(shell['timeslots']):
            sat_lla = []
            sat_names = []
            for sid, node in enumerate(slot['position']):
                # print(node)
                lla = (node['latitude'], node['longitude'], node['altitude'])
                sat_lla.append(lla)
                name = f'SH{i+1}SAT{sid+1}'
                sat_names.append(name)

            sat_cbf = to_cbf(sat_lla)   # np array, sat_num * 3

            if len(name_lst) == 0:
                name_lst = sat_names
            elif len(name_lst) != len(sat_names):
                raise RuntimeError("satellites change between slots!")

            for link in slot['links']:
                sid1, sid2 = link['sat1'], link['sat2']
                if sid1 > sid2:
                    sid1, sid2 = sid2, sid1
                delay = np.sqrt(np.sum(np.square(sat_cbf[sid1] - sat_cbf[sid2])))/(299792.458) * 1000 #17.31 / 29.5 * 
                if name_lst[sid2] not in init_isls[sid1]:
                    init_isls[sid1][name_lst[sid2]] = delay

        topo_init_shell.append([shell['name'], name_lst, init_isls])

    sat_names_shell = [shell[1] for shell in topo_init_shell]
    all_link_states,link_count = _init_tinyleo_link_files(dir, topo_init_shell, gs_dirname)
    return sat_names_shell, all_link_states, link_count

def _init_tinyleo_link_files(dir, topo_init_shell, gs_dirname):
    """
    Initializes the link files for ISLs and GSLs.

    Args:
        dir (str): Working directory for the simulation.
        topo_init_shell (list): Initial topology information for each shell.
        gs_dirname (str): Directory name for ground station links.

    Returns:
        tuple: A tuple containing all link states and the link count.
    """
    gsl_dir = os.path.join(dir, gs_dirname, 'gsl')
    for file in glob.glob(os.path.join(gsl_dir, '*.txt')):
        os.remove(file)
    os.makedirs(gsl_dir, exist_ok=True)
    link_count = 1
    all_link_states = {}
    for shell_name, sat_name_lst, init_isls in topo_init_shell:
        isl_dir = os.path.join(dir, shell_name, 'isl')
        os.makedirs(isl_dir, exist_ok=True)
        for file in glob.glob(os.path.join(isl_dir, '*.txt')):
            os.remove(file)
        f_init = open(f"{isl_dir}/init.txt", 'w')
        for sid, isl_lst in enumerate(init_isls):
            f_init.write(f"{sat_name_lst[sid]}|")
            # del some isls
            f_init.write('|')
            # update some isls
            f_init.write('|')
            # add some isls
            add_lst = []
            for link_sat in isl_lst:
                delay = isl_lst[link_sat]
                key = f'{sat_name_lst[sid]}-{link_sat}'
                all_link_states[key] = delay
                add_lst.append(f"{link_sat},{delay:.2f},{link_count}")
                link_count += 1
            f_init.write(' '.join(add_lst))
            f_init.write('\n')
        f_init.close()
    return all_link_states,link_count

def update_tinyleo_link(dir,ts,all_link_states,shell_lst,GS_lat_long,antenna_number,isl_sats,GS_cell,link_count,geopraphic_routing_policy,inter_cell_isls,all_node_states):
    """
    Updates the ISLs and GSLs for the simulation.

    Args:
        dir (str): Working directory for the simulation.
        ts (int): Current timestamp.
        all_link_states (dict): Dictionary of all link states.
        shell_lst (list): List of shell configurations.
        GS_lat_long (list): List of ground station coordinates.
        antenna_number (int): Number of antennas per ground station.
        isl_sats (dict): Mapping of sats with isls to cells.
        GS_cell (dict): Mapping of ground stations to cells.
        link_count (int): Current link count.
        geopraphic_routing_policy (list): List of segment information.
        inter_cell_isls (dict): Inter-cell ISL information.
        all_node_states (dict): Dictionary to store the state of all nodes.

    Returns:
        int: Updated link count.

    This function updates the ISLs and GSLs, generates link files, and updates node states.
    """
    isl_per_shell = _update_tinyleo_isl(dir,ts,shell_lst,all_node_states)
    gsls = _update_tinyleo_gsl(isl_per_shell,to_cbf(GS_lat_long),antenna_number,isl_sats, GS_cell)
    link_count = _update_tinyleo_link_files(dir,ts,isl_per_shell,gsls,GS_lat_long,all_link_states,link_count,all_node_states,isl_sats,geopraphic_routing_policy)
    _update_nodes_states_files(dir,ts,isl_sats,inter_cell_isls,all_node_states)
    return link_count

def _update_tinyleo_link_files(dir,ts,isl_per_shell,gsls,GS_lat_long,all_link_states,link_count,all_node_states,isl_sats,geopraphic_routing_policy):
    """
    Updates the inter-satellite link (ISL) and ground station link (GSL) files.

    Args:
        dir (str): Working directory for the simulation.
        ts (int): Current timestamp.
        isl_per_shell (list): ISL information for each shell.
        gsls (list): GSL information for ground stations.
        GS_lat_long (list): List of ground station coordinates.
        all_link_states (dict): Dictionary of all link states.
        link_count (int): Current link count.
        all_node_states (dict): Dictionary to store the state of all nodes.
        isl_sats (dict): Mapping of satellites with ISLs to cells.
        geopraphic_routing_policy (list): List of segment information.

    Returns:
        int: Updated link count.

    This function updates the ISL and GSL files for the current timestamp, 
    modifies node states, and tracks changes in link states.
    """
    for shell_name, sat_name_lst, sat_cbf, isls in isl_per_shell:
        isl_dir = os.path.join(dir, shell_name, 'isl')
        f_update = open(f"{isl_dir}/{ts}.txt", 'w')
        for sid, isl_lst in enumerate(isls):
            sat1 = sat_name_lst[sid]
            for isl in isl_lst:
                sat2 = isl[0]
                delay = isl[1]
                sat1_ip,sat2_ip = isl_addr6_ips(sat1,sat2)
                sat1_mac,sat2_mac = isl_mac(sat1,sat2)
                sat1_ip = sat1_ip.split('/')[0]
                sat2_ip = sat2_ip.split('/')[0]
                all_node_states[sat1]['isls'][sat2] = [sat2_ip, sat2_mac, delay]
                all_node_states[sat2]['isls'][sat1] = [sat1_ip, sat1_mac, delay]


            f_update.write(f"{sat_name_lst[sid]}|")
            # del some isls
            f_update.write('|')
            update_lst = []
            add_lst = []
            for isl in isl_lst:
                key = f'{sat_name_lst[sid]}-{isl[0]}'
                if key in all_link_states:
                    if isl[1] - all_link_states[key] > 1:
                        update_lst.append(f"{isl[0]},{isl[1]:.2f}")
                        all_link_states[key] = isl[1]
                else:
                    all_link_states[key] = isl[1]
                    add_lst.append(f"{isl[0]},{isl[1]:.2f},{link_count}")
                    link_count += 1
            # update some isls
            f_update.write(' '.join(update_lst) + '|')
            # add some isls
            f_update.write(' '.join(add_lst))
            f_update.write('\n')
        f_update.close()
    
    # GSL
    gsl_dir = os.path.join(dir, 'GS-' + str(len(GS_lat_long)), 'gsl')
    f_update = open(f"{gsl_dir}/{ts}.txt", 'w')
    for gid, gsl_lst in enumerate(gsls):
        gs = get_gs_name(gid)
        all_node_states[gs] = {'ts':ts,'gsls':{},'seg_list':geopraphic_routing_policy}
        for gsl in gsl_lst:
            sat = gsl[0]
            delay = gsl[1]
            gs_ip, sat_ip = _gsl_addr6_ips(gs, sat, isl_sats)
            gs_mac, sat_mac = _gsl_mac(gs, sat)
            gs_ip = gs_ip.split('/')[0]
            sat_ip = sat_ip.split('/')[0]
            all_node_states[gs]['gs_ip'] = gs_ip
            all_node_states[gs]['gsls'][sat] = [sat_ip, sat_mac, delay]
            all_node_states[sat]['gsls'][gs] = [gs_ip, gs_mac, delay]

        f_update.write(f"{get_gs_name(gid)}|")
        # TODO del some gsls
        f_update.write('|')
        update_lst = []
        add_lst = []
        for gsl in gsl_lst:
            key = f'{get_gs_name(gid)}-{gsl[0]}'
            if key in all_link_states:
                if gsl[1] - all_link_states[key] > 1:
                    update_lst.append(f"{gsl[0]},{gsl[1]:.2f}")
                    all_link_states[key] = gsl[1]
            else:
                all_link_states[key] = gsl[1]
                add_lst.append(f"{gsl[0]},{gsl[1]:.2f},{link_count}")
                link_count += 1
        # update some gsls
        f_update.write(' '.join(update_lst) + '|')
        # add some gsls
        f_update.write(' '.join(add_lst))
        f_update.write('\n')
    f_update.close()
    return link_count

def _update_tinyleo_gsl(isl_per_shell, gs_cbf, antenna_num,isl_sats = None,GS_cell=None):
    """
    Updates the ground station links (GSLs) for the simulation.

    Args:
        isl_per_shell (list): ISL information for each shell.
        gs_cbf (numpy.ndarray): Cartesian coordinates of ground stations.
        antenna_num (int): Number of antennas per ground station.
        isl_sats (dict, optional): Mapping of satellites with ISLs to cells. Defaults to None.
        GS_cell (dict, optional): Mapping of ground stations to cells. Defaults to None.

    Returns:
        list: Updated GSL information for all ground stations.

    This function calculates the distance between ground stations and satellites, 
    applies constraints (e.g., antenna limits, cell matching), and generates GSLs.
    """
    gsls_per_shell = [] # [[[ [gsl] for every gs] for every ts] for every shell]
    for shellname, name_lst, sat_cbf, isls in isl_per_shell:
        # (gs_num) op (sat_num) -> (gs_num, sat_num)
        dx = np.subtract.outer(gs_cbf[..., 0], sat_cbf[..., 0])
        dy = np.subtract.outer(gs_cbf[..., 1], sat_cbf[..., 1])
        dz = np.subtract.outer(gs_cbf[..., 2], sat_cbf[..., 2])
        dist = np.sqrt(np.square(dx) + np.square(dy) + np.square(dz))
        gsls = []
        for gs_id,gs_dist in enumerate(dist):
            gs_dist = gs_dist.flatten()
            #TODO: elevation angle bound
            # bound_mask = gs_dist < bound_dis
            bound_mask = gs_dist == gs_dist
            sat_indices = np.arange(len(gs_dist))[bound_mask]
            gs_dist = gs_dist[bound_mask]
            sorted_sat = gs_dist.argsort()
            add_num = 0
            gsl_tmp = []
            for sat in sorted_sat:
                if add_num >= antenna_num:
                    break
                sat_name = name_lst[sat_indices[sat]]
                if isl_sats and sat_name not in isl_sats:
                    # print(f"satellite {sat_name} not in isl_sats")
                    continue
                if GS_cell and isl_sats[sat_name]!=GS_cell[gs_id]:
                    continue
                gsl_tmp.append((sat_name, gs_dist[sat] / (299792.458) * 1000))#17.31 / 29.5 * 
                add_num += 1
            gsls.append(gsl_tmp)
        gsls_per_shell.append(gsls)
    gsls = [
        list() for gid in range(len(gs_cbf))
    ]
    for gid, gsl_lst in enumerate(gsls):
        for shell_id in range(len(gsls_per_shell)):
            for sat_name, delay in gsls_per_shell[shell_id][gid]:
                if len(gsl_lst) >= antenna_num:
                    break
                gsl_lst.append((sat_name, delay))
    return gsls

def isl_addr6_ips(name1, name2):
    """
    Generates IPv6 addresses for an inter-satellite link (ISL).

    Args:
        name1 (str): Name of the first satellite (e.g., "SH1SAT1").
        name2 (str): Name of the second satellite (e.g., "SH1SAT2").

    Returns:
        tuple: A tuple containing the IPv6 addresses for the two satellites.
    """
    sat1_id = int(name1.split('SAT')[-1])
    sat2_id = int(name2.split('SAT')[-1])
    if sat1_id < sat2_id:
        addr6_prefix = f"aaaa:{sat1_id}:aaaa:{sat2_id}::"
        return addr6_prefix+"10/64", addr6_prefix+"40/64"
    else:
        addr6_prefix = f"aaaa:{sat2_id}:aaaa:{sat1_id}::"
        return addr6_prefix+"40/64", addr6_prefix+"10/64"
    
def isl_mac(name1, name2):
    """
    Generates MAC addresses for an inter-satellite link (ISL).

    Args:
        name1 (str): Name of the first satellite (e.g., "SH1SAT1").
        name2 (str): Name of the second satellite (e.g., "SH1SAT2").

    Returns:
        tuple: A tuple containing the MAC addresses for the two satellites.
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

def _gsl_addr6_ips(name1, name2,sat_cell):
    """
    Generates IPv6 addresses for a ground station link (GSL).

    Args:
        name1 (str): Name of the ground station (e.g., "GS1").
        name2 (str): Name of the satellite (e.g., "SH1SAT1").
        sat_cell (dict): Mapping of satellites to their cell information.

    Returns:
        tuple: A tuple containing the IPv6 addresses for the ground station and satellite.
    """
    gs_id = name1.split('GS')[-1]
    sat_id = name2.split('SAT')[-1]
    cell = sat_cell[name2]
    addr6_prefix = f"ce:{cell[0]}:ce:{cell[1]}:{gs_id}::"
    return addr6_prefix+f"{gs_id}/80", addr6_prefix+f"aaaa:{sat_id}/80"

def _gsl_mac(name1, name2):
    """
    Generates MAC addresses for a ground station link (GSL).

    Args:
        name1 (str): Name of the ground station (e.g., "GS1").
        name2 (str): Name of the satellite (e.g., "SH1SAT1").

    Returns:
        tuple: A tuple containing the MAC addresses for the ground station and satellite.
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

def _update_nodes_states_files(dir,ts,sat_cells,inter_cell_isls,all_node_states):
    """
    Updates the node states and writes them to a JSON file.

    Args:
        dir (str): Working directory for the simulation.
        ts (int): Current timestamp.
        sat_cells (dict): Mapping of satellites to their cell information.
        inter_cell_isls (dict): Inter-cell ISL information.
        all_node_states (dict): Dictionary to store the state of all nodes.
    """
    cell_orders = {}
    for sat in sat_cells:
        all_node_states[sat]['sat_cell'] = sat_cells[sat]
    for sat in sat_cells:
        cell = str(sat_cells[sat])
        if cell in cell_orders:
            continue
        cell_orders[cell] = [sat]
        sat1 = sat
        while True:
            flag = False
            for sat2 in all_node_states[sat1]['isls']:
                if str(sat_cells[sat2]) != cell:
                    continue
                if sat2 in cell_orders[cell]:
                    continue
                cell_orders[cell].append(sat2)
                sat1 = sat2
                flag = True
                break
            if not flag:
                break

    for cell in cell_orders:
        for sat in cell_orders[cell]:
            all_node_states[sat]['cell_ring'] = cell_orders[cell]

    for sat in inter_cell_isls:
        all_node_states[sat]['inter_cell_isls'] = inter_cell_isls[sat]

    with open(os.path.join(dir,'all_node_states', f'{ts}.json'), 'w') as f:
        json.dump(all_node_states, f, indent=4)

def compute_delay_from_cbf(cbf1, cbf2):
    """
    Compute delay from CBF.
    
    Args:
        cbf1: CBF of satellite 1
        cbf2: CBF of satellite 2
        
    Returns:
        Delay in ms
    """
    return ((cbf1[0] - cbf2[0])**2 + (cbf1[1] - cbf2[1])**2 + (cbf1[2] - cbf2[2])**2)**0.5 / (299792.458) * 1000 

def get_cell_ring(links_input):
    """
    Constructs a cell ring (a circular sequence of satellites) from a list of links.

    Args:
        links_input (list): A list of links, where each link is a tuple of two satellite IDs.

    Returns:
        list: A list of satellite names representing the cell ring in order.
    """
    links = copy.deepcopy(links_input)
    cell_ring = [links[0][0],links[0][1]]
    links.remove(links[0])
    while len(links) > 1:
        for link in links:
            if link[0] == cell_ring[-1]:
                cell_ring.append(link[1])
                links.remove(link)
                break
            elif link[1] == cell_ring[-1]:
                cell_ring.append(link[0])
                links.remove(link)
                break
    sat_name_cell_ring = []
    for sat in cell_ring:
        sat_name_cell_ring.append(get_satellite_name(sat))
    return sat_name_cell_ring

def same_ring_direction(ring1,ring2):
    """
    Compares the direction of two cell rings to determine if they are the same.

    Args:
        ring1 (list): The first cell ring, represented as a list of satellite names.
        ring2 (list): The second cell ring, represented as a list of satellite names.
        
    Returns:
        bool: True if the two rings have the same direction, False otherwise.
    """
    for i in range(len(ring1)):
        cell1_sat1 = ring1[i]
        cell1_sat2 = ring1[(i+1)%len(ring1)]
        flag = False
        for j in range(len(ring2)):
            cell2_sat1 = ring2[j]
            if cell1_sat1 == cell2_sat1:
                flag = True
                break
        if not flag:
            continue
        pre_cell2_sat2 = ring2[(j-1)%len(ring2)]
        if cell1_sat2 == pre_cell2_sat2:
            return False
        next_cell2_sat2 = ring2[(j+1)%len(ring2)]
        if cell1_sat2 == next_cell2_sat2:
            return True
    return True