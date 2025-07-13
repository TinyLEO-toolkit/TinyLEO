"""
Southbound Network Controller for TinyLEO Toolkit
This module implements the southbound network controller for managing satellite and ground station networks.
It includes functionalities for:
- Managing containers and their lifecycle (creation, initialization, and cleanup).
- Initializing and updating network topology.
- Managing remote machines and their configurations.
- Handling link failures.
- Deploying SRv6 agents for data plane operations.
- Supporting network testing tools like ping, iperf, and traceroute.
"""

import time
import json
import threading
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from southbound.sn_utils import *
from failure_recovery_mpc import MPCFaultHandler
from sn_orchestrator_mpc import *


ASSIGN_FILENAME = 'assign.json'
PID_FILENAME = 'container_pid.txt'
NOT_ASSIGNED = 'NA'

class RemoteController():
    """
    A class to manage containers on remote machines for the TinyLEO Toolkit.
    This class handles the initialization, configuration, and management of
    satellite and ground station networks, including container lifecycle,
    topology updates, fault testing, and recovery.

    Attributes:
        gs_lat_long (list): List of ground station latitude and longitude coordinates.
        link_style (str): The style of inter-satellite links (e.g., dynamic or static).
        link_policy (str): The policy for link management.
        duration (int): The total duration of the simulation.
        sat_bandwidth (int): Bandwidth of inter-satellite links.
        sat_ground_bandwidth (int): Bandwidth of satellite-to-ground links.
        sat_loss (float): Packet loss rate for inter-satellite links.
        sat_ground_loss (float): Packet loss rate for satellite-to-ground links.
        antenna_number (int): Number of antennas per ground station.
        elevation (float): Elevation angle for antennas.
        configuration_dir (str): Directory path for configuration files.
        experiment_name (str): Name of the experiment.
        gs_dirname (str): Directory name for ground station data.
        GS_cell (dict): Mapping of ground station cells.
        local_dir (str): Local directory for experiment data.
        machine_lst (list): List of remote machines for the simulation.
        data_plane_dir (str): Directory for data plane geographic srv6 anycast.
    """

    def __init__(self,configuration_file_path,GS_lat_long,GS_cell):
        """
        Initializes the RemoteController instance with the given arguments.

        Args:
            configuration_file_path (str): Path to the configuration file.
            GS_lat_long (list): List of ground station latitude and longitude coordinates.
            GS_cell (dict): Mapping of ground station cells.
        """
        sn_args = sn_load_file(configuration_file_path)
        self.gs_lat_long = GS_lat_long
        self.link_style = sn_args.link_style
        self.link_policy = sn_args.link_policy
        self.duration = sn_args.duration
        self.sat_bandwidth = sn_args.sat_bandwidth
        self.sat_ground_bandwidth = sn_args.sat_ground_bandwidth
        self.sat_loss = sn_args.sat_loss
        self.sat_ground_loss = sn_args.sat_ground_loss
        self.antenna_number = sn_args.antenna_number
        self.elevation = sn_args.antenna_elevation
        self.configuration_dir = os.path.abspath(os.path.dirname(configuration_file_path))
        self.experiment_name = sn_args.cons_name+'-'+ sn_args.link_style +'-'+ sn_args.link_policy
        self.gs_dirname = 'GS-'+ str(len(self.gs_lat_long))
        self.GS_cell = GS_cell
        self.local_dir = os.path.abspath(os.path.join(self.configuration_dir,'..', self.experiment_name))
        self.machine_lst = sn_args.machine_lst
        self.topo_dir = sn_args.topo_dir
        self.data_plane_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..','geographic_srv6_anycast'))
    
    def init_remote_machine(self):
        """
        Initializes the simulation environment in remote machines.
        """
        # Predict and generate topology data for the simulation
        print('Predict topo data')
        predict_all_topologies(
            duration=self.duration,
            satellite_file=f"{self.topo_dir}/eval1_573_jinyao_24k_half.npy",
            traffic_matrix_file=f"{self.topo_dir}/traffic_matrix_max_24k_new.npy",
            grid_satellites_file=f"{self.topo_dir}/new_grid_satellites.npy",
            result_output_dir=self.local_dir,
            num_processes=8
        )
        # Initialize files for containers and topology
        self._init_tinyleo_topology()
        for shell_id, shell in enumerate(self.shell_lst):
            shell['name'] = f"shell{shell_id}"
        self._init_local()
        sat_names_shell,self.all_link_states,self.link_count = init_tinyleo_links(self.local_dir,self.shell_lst,self.gs_dirname)
        (self.remote_lst,self.sat_mid_dict,self.gs_mid_dict) = self._assign_remote(sat_names_shell, self.machine_lst)

    def start_link_faliure_server(self):
        """
        Starts a gRPC server to handle link failure events.
        """
        self.link_failure_server = link_faliure_server(self)
        def server_wait():
            self.link_failure_server.wait_for_termination()
        t = threading.Thread(target=server_wait, daemon=True)
        t.start()

    def handle_link_failure(self, sat1, sat2):
        """
        Handles a link failure between two satellites and updates the network topology accordingly.

        Args:
            sat1 (str): Name of the first satellite involved in the link failure.
            sat2 (str): Name of the second satellite involved in the link failure.

        Returns:
            list: A list of satellites whose states were updated.
        """
        print(f"Link failure between {sat1} and {sat2}")
        mpc = MPCFaultHandler(self.topo_dir,self.local_dir,self.ts)
        result = mpc.handle_link_failure(get_satellite_id(sat1), get_satellite_id(sat2))
        update_sats = set()
        if result:
            if 'replacement_info' in result:
                remove_sat = get_satellite_name(result['replacement_info']['removed_satellite'])
                replacement_sat = get_satellite_name(result['replacement_info']['replacement_satellite'])
                print('remove sat',remove_sat,'replace sat',replacement_sat)
                update_sats.add(remove_sat)
                update_sats.add(replacement_sat)
        sat1 = replacement_sat
        # isls
        for sat2 in self.all_node_states[remove_sat]['isls']:
            sat1_ip,sat2_ip = isl_addr6_ips(sat1,sat2)
            sat1_ip,sat2_ip = sat1_ip.split('/')[0],sat2_ip.split('/')[0]
            sat1_mac,sat2_mac = isl_mac(sat1,sat2)
            delay = compute_delay_from_cbf(self.all_node_states[sat1]['position']['cbf'],self.all_node_states[sat2]['position']['cbf'])
            self.all_node_states[sat1]['isls'][sat2] = [sat2_ip, sat2_mac, delay]
            self.all_node_states[sat2]['isls'].pop(remove_sat)
            self.all_node_states[sat2]['isls'][sat1] = [sat1_ip, sat1_mac, delay]
        self.all_node_states[remove_sat]['isls'] = {}

        # cell_ring
        new_cell_ring = self.all_node_states[remove_sat]['cell_ring']
        for i in range(len(new_cell_ring)):
            if new_cell_ring[i] == remove_sat:
                new_cell_ring[i] = replacement_sat
        for sat in new_cell_ring:
            update_sats.add(sat)
            self.all_node_states[sat]['cell_ring'] = new_cell_ring
        self.all_node_states[remove_sat]['cell_ring'] = []

        # sat_cell
        self.all_node_states[replacement_sat]['sat_cell'] = self.all_node_states[remove_sat]['sat_cell']
        self.all_node_states[remove_sat]['sat_cell'] = []

        # intra_cell_isls
        if self.all_node_states[remove_sat]['inter_cell_isls']!={}:
            self.all_node_states[replacement_sat]['inter_cell_isls'] = self.all_node_states[remove_sat]['inter_cell_isls']
            self.all_node_states[remove_sat]['inter_cell_isls'] = {}
            near_sat = self.all_node_states[replacement_sat]['inter_cell_isls']['near_sat']
            sat_cell = self.all_node_states[replacement_sat]['sat_cell']
            self.all_node_states[near_sat]['inter_cell_isls']['near_sat'] = replacement_sat
            self.all_node_states[near_sat]['inter_cell_isls']['near_cell'] = sat_cell
        with open(os.path.join(self.local_dir,'all_node_states', f'{self.ts}.json'), 'w') as f:
            json.dump(self.all_node_states, f, indent=4)
        f_update = open(os.path.join(self.local_dir,'shell0','isl',f'{self.ts}.txt'), 'w')
        
        for sat1 in update_sats:
            f_update.write(f"{sat1}|")
            f_update.write("|")
            f_update.write("|")
            add_lst = []
            for sat2 in self.all_node_states[sat1]['isls']:
                delay = self.all_node_states[sat1]['isls'][sat2][2]
                # print(sat1,sat2,delay)
                if int(sat1.split('SAT')[-1])<int(sat2.split('SAT')[-1]):
                    key = f"{sat1}-{sat2}"
                else:
                    key = f"{sat2}-{sat1}"
                if key not in self.all_link_states:
                    self.all_link_states[key] = delay
                    add_lst.append(f"{sat2},{delay:.2f},{self.link_count}")
                    self.link_count += 1
            f_update.write(' '.join(add_lst))
            f_update.write('\n')
        f_update.close()
        conn_threads = []
        for remote in self.remote_lst:
            thread = threading.Thread(
                target=remote.upload_failure_recovery_file,
                args=(self.ts, )
                )
            thread.start()
            conn_threads.append(thread)
        for thread in conn_threads:
            thread.join()
        print('update sats:', len(update_sats), update_sats)
        return list(update_sats)

    def _init_tinyleo_topology(self):
        """
        Initializes the TinyLEO topology by loading the predicted inter-satellite
        link positions from a JSON file.
        """
        self.shell_lst = []
        topo_path = os.path.join(self.local_dir,'predict_isl_position_all.json')
        with open(topo_path, 'r') as f:
            tinyleo_shell = json.load(f)
        self.shell_lst.append(tinyleo_shell)

    def update_tinyleo_topology(self,ts):
        """
        Updates the TinyLEO topology for a specific timestamp.

        Args:
            ts (int): The timestamp for which the topology should be updated.
        """
        self.ts = ts
        generate_topology_for_timestamp(
            timestamp=self.ts,
            satellite_file=f"{self.topo_dir}/eval1_573_jinyao_24k_half.npy",
            block_positions_file=f"{self.topo_dir}/block_positions.json",
            traffic_matrix_file=f"{self.topo_dir}/traffic_matrix_max_24k_new.npy",
            grid_satellites_file=f"{self.topo_dir}/new_grid_satellites.npy",
            output_dir=self.local_dir,
            num_processes=8
        )

        self.shell_lst = []
        topo_path = os.path.join(self.local_dir,'all_isl_positions',f"{ts}.json")
        with open(topo_path, 'r') as f:
            tinyleo_topo = json.load(f)
        tinyleo_topo['name'] = "shell0"
        self.shell_lst.append(tinyleo_topo)
        isl_sats_path = os.path.join(self.local_dir, 'sat_cells', f"{ts}.json")
        with open(isl_sats_path, 'r') as f:
            self.isl_sats = json.load(f)
        inter_cell_isls = os.path.join(self.local_dir, 'inter_cell_isls', f"{ts}.json")
        with open(inter_cell_isls, 'r') as f:
            self.inter_cell_isls = json.load(f)
        self.all_node_states = {}
        self.link_count = update_tinyleo_link(self.local_dir,ts,self.all_link_states,self.shell_lst,self.gs_lat_long,
                                              self.antenna_number,self.isl_sats,self.GS_cell,self.link_count,self.geopraphic_routing_policy,
                                              self.inter_cell_isls,self.all_node_states)
        self.update_remote_topology()
        
    def update_remote_topology(self):
        """
        Updates the network topology on all remote machines for a specific timestamp.

        Args:
            ts (int): The timestamp for which the topology should be updated.
        """
        print(f"Update networks at t {self.ts}...")
        update_start = time.time()
        conn_threads = []
        for remote in self.remote_lst:
            thread = threading.Thread(
                target=remote.update_network,
                args=(self.ts,
                    self.sat_bandwidth,
                    self.sat_loss,
                    self.sat_ground_bandwidth,
                    self.sat_ground_loss
            ))
            thread.start()
            conn_threads.append(thread)
        for thread in conn_threads:
            thread.join()
        end = time.time()
        print(end-update_start, "s for network update\n")

    def tinyleo_fault_test(self):
        """
        Performs a fault test on the network by simulating link failures.
        """
        print(f"fault test ...")
        update_start = time.time()
        conn_threads = []
        for remote in self.remote_lst:
            thread = threading.Thread(
                target=remote.fault_test,
                args=(self.ts,
                    self.sat_bandwidth,
                    self.sat_loss,
                    self.sat_ground_bandwidth,
                    self.sat_ground_loss
            ))
            thread.start()
            conn_threads.append(thread)
        for thread in conn_threads:
            thread.join()
        end = time.time()
        print(end-update_start, "s for fault test\n")

    def deploy_tinyleo_srv6_agent(self):
        """
        Deploy the SRv6 agent on all remote machines for data plane operations.
        """
        print(f"Deploy srv6 agent...")
        update_start = time.time()
        conn_threads = []
        for remote in self.remote_lst:
            thread = threading.Thread(
                target=remote.deploy_tinyleo_srv6_agent,
                args=()
                )
            thread.start()
            conn_threads.append(thread)
        for thread in conn_threads:
            thread.join()
        end = time.time()
        print(end-update_start, "s for srv6 agent deploy\n")

    def _init_local(self):
        """
        Prepares the local directory structure and configuration files for the simulation.
        """
        for txt_file in glob.glob(os.path.join(self.local_dir, '*.txt')):
            os.remove(txt_file)
        for shell in self.shell_lst:
            os.makedirs(os.path.join(self.local_dir, shell['name']), exist_ok=True)
        os.makedirs(os.path.join(self.local_dir, self.gs_dirname), exist_ok=True)
        os.makedirs(os.path.join(self.local_dir,'result'), exist_ok=True)
        os.makedirs(os.path.join(self.local_dir,'all_node_states'), exist_ok=True)
        with open(os.path.join(self.configuration_dir, 'geopraphic_routing_policy.json')) as f:
            self.geopraphic_routing_policy = json.load(f)
        with open(os.path.join(self.topo_dir, 'block_positions.json')) as f:
            self.block_positions = json.load(f)

    def _assign_remote(self, sat_names_shell, machine_lst):
        """
        Assigns satellites and ground stations to remote machines for simulation.

        Args:
            sat_names_shell (list): List of satellite names grouped by shell.
            machine_lst (list): List of remote machine configurations.

        Returns:
            tuple: A tuple containing:
                - remote_lst (list): List of RemoteMachine instances.
                - sat_mid_dict (dict): Mapping of satellite names to machine IDs.
                - gs_mid_dict (dict): Mapping of ground station names to machine IDs.
        """
        assert len(sat_names_shell) == len(self.shell_lst)

        # TODO: better partition
        if len(sat_names_shell) * 2 <= len(machine_lst):
            # need intra-shell partition
            machine_per_shell = len(machine_lst) // len(sat_names_shell)
            raise NotImplementedError
        else:
            # only divide shell
            shell_per_machine = len(self.shell_lst) // len(machine_lst)
            remainder = len(sat_names_shell) % len(machine_lst)

            shell_id = 0
            sat_mid_dict_shell = []
            assigned_shell_lst = []
            for i, remote in enumerate(machine_lst):
                shell_num = shell_per_machine
                if i < remainder:
                    shell_num += 1
                assigned_shells = [
                    (self.shell_lst[j]['name'], sat_names_shell[j])
                      for j in range(shell_id, shell_id + shell_num)
                ]
                # all satellites of a shell assigned to a single machine
                for shell_name, sat_names in assigned_shells:
                    sat_mid_dict = {}
                    for sat_name in sat_names:
                        sat_mid_dict[sat_name] = i
                    sat_mid_dict_shell.append(sat_mid_dict)
                assigned_shell_lst.append(assigned_shells)
                shell_id += shell_num
            gs_mid_dict = {}

            for gs_id,_ in enumerate(self.gs_lat_long):
                gs_name = get_gs_name(gs_id)
                gs_mid_dict[gs_name] = 0

            ip_lst = [remote['IP'] for remote in machine_lst]
            assign_obj = {
                'sat_mid_shell': sat_mid_dict_shell,
                'gs_mid': gs_mid_dict,
                'ip': ip_lst,
            }
            with open(os.path.join(self.local_dir, ASSIGN_FILENAME), 'w') as f:
                json.dump(assign_obj, f)

        remote_lst = []
        for i, remote in enumerate(machine_lst):
            remote_lst.append(RemoteMachine(
                i,
                remote['IP'],
                remote['port'],
                remote['username'],
                remote['password'],
                assigned_shell_lst[i],
                self.experiment_name,
                self.local_dir,
                self.gs_dirname if i in gs_mid_dict.values() else None
                )
            )
        return remote_lst, sat_mid_dict, gs_mid_dict

    def create_nodes(self):
        """
        Initializes all simulation nodes (e.g., satellites and ground stations).
        """
        print('Initializing nodes ...')
        begin = time.time()
        for remote in self.remote_lst:
            remote.init_nodes()
        print("Node initialization:", time.time() - begin, "s consumed.\n")
        self._load_node_map()

    def node_map(self):
        """
        Retrieves the mapping of nodes to their respective remote machines.

        Returns:
            dict: A dictionary mapping node names to RemoteMachine instances.
        """
        if hasattr(self, 'nodes'):
            return self.nodes
        self._load_node_map()
        return self.nodes

    def _load_node_map(self):
        """
        Loads the mapping of nodes to their respective remote machines.
        """
        self.nodes = {}
        self.undamaged_lst = list()
        self.total_sat_lst = list()
        for remote in self.remote_lst:
            for node in remote.get_nodes():
                if node.startswith('Error'):
                    print(node)
                    exit(1)
                if not node.startswith('GS'):
                    self.undamaged_lst.append(node)
                    self.total_sat_lst.append(node)
                self.nodes[node.strip()] = remote

    def create_links(self):
        """
        Initializes network links between nodes in the simulation.
        """
        print('Initializing links ...')
        thread_lst = []
        begin = time.time()
        for remote in self.remote_lst:
            thread = threading.Thread(
                target=remote.init_network,
                args=(self.sat_bandwidth,
                      self.sat_loss,
                      self.sat_ground_bandwidth,
                      self.sat_ground_loss),
            )
            thread.start()
            thread_lst.append(thread)
        for thread in thread_lst:
            thread.join()
        print("Link initialization:", time.time() - begin, 's consumed.\n')

    def set_ping(self, src, dst, filename):
        """
        Sets up a ping test between two nodes.

        Args:
            src (str): Source node name.
            dst (str): Destination node name.
            filename (str): Name of the file to save the ping results.
        """
        node_map = self.node_map()
        machine = node_map[src]
        machine.ping_async(
            os.path.join(self.local_dir,'result',f'ping-{filename}.txt'),
            src, dst
        )

    def set_iperf(self, src, dst, filename):
        """
        Sets up an iperf test between two nodes.

        Args:
            src (str): Source node name.
            dst (str): Destination node name.
            filename (str): Name of the file to save the iperf results.
        """
        node_map = self.node_map()
        machine = node_map[src]
        machine.iperf_async(
            os.path.join(self.local_dir,'result', f'iperf-{filename}.txt'),
            src, dst
        )

    def set_traceroute(self, src, dst, filename):
        """
        Sets up a traceroute test between two nodes.

        Args:
            src (str): Source node name.
            dst (str): Destination node name.
            filename (str): Name of the file to save the traceroute results.
        """
        node_map = self.node_map()
        machine = node_map[src]
        machine.traceroute_async(
            os.path.join(self.local_dir,'result', f'traceroute-{filename}.txt'),
            src, dst
        )

    def get_pid_map(self):
        """
        Retrieves the mapping of container names to their process IDs.

        Returns:
            dict: A dictionary mapping container names to process IDs.
        """
        pid_map_file = os.path.join(self.local_dir, PID_FILENAME)
        _pid_map = {}
        if not os.path.exists(pid_map_file):
            print('Error: container index file not found, please create nodes')
            exit(1)
        with open(pid_map_file, 'r') as f:
            for line in f:
                if len(line) == 0 or line.isspace():
                    continue
                for name_pid in line.strip().split():
                    if name_pid == NOT_ASSIGNED:
                        continue
                    name_pid = name_pid.split(':')
                    _pid_map[name_pid[0]] = name_pid[1]
        return _pid_map
    
    def clean(self):
        """
        Cleans up the simulation environment by removing containers and links.
        """
        print("Removing containers and links...")
        for remote in self.remote_lst:
            remote.clean()
        print("All containers and links remoted.")

class RemoteMachine:
    """
    A class to manage remote machines in the TinyLEO simulation environment.
    This class handles the initialization, configuration, and management of
    remote machines, including uploading necessary files, initializing nodes,
    and managing network configurations.

    Attributes:
        id (int): Unique identifier for the remote machine.
        shell_lst (list): List of satellite shells assigned to this machine.
        local_dir (str): Local directory for storing simulation data.
        gs_dirname (str): Directory name for ground station data.
        ssh (paramiko.SSHClient): SSH client for remote command execution.
        sftp (paramiko.SFTPClient): SFTP client for file transfers.
        dir (str): Remote directory for storing simulation files.
    """

    def __init__(self, id, host, port, username, password, 
                 shell_lst, experiment_name, local_dir, gs_dirname):
        """
        Initializes the RemoteMachine instance and sets up the remote environment.

        Args:
            id (int): Unique identifier for the remote machine.
            host (str): Hostname or IP address of the remote machine.
            port (int): SSH port for the remote machine.
            username (str): SSH username.
            password (str): SSH password.
            shell_lst (list): List of satellite shells assigned to this machine.
            experiment_name (str): Name of the experiment.
            local_dir (str): Local directory for storing simulation data.
            gs_dirname (str): Directory name for ground station data.
        """
        self.id = id
        self.shell_lst = shell_lst
        self.local_dir = local_dir
        self.gs_dirname = gs_dirname
        self.ssh, self.sftp = sn_connect_remote(
            host = host,
            port = port,
            username = username,
            password = password,
        )
        sn_remote_cmd(self.ssh, 'mkdir ~/' + experiment_name)
        self.dir = sn_remote_cmd(self.ssh, 'echo ~/' + experiment_name)
        self.sftp.put(
            os.path.join(os.path.dirname(__file__), 'sn_remote.py'),
            self.dir + '/sn_remote.py'
        )
        self.sftp.put(
            os.path.join(os.path.dirname(__file__), 'pyctr.c'),
            self.dir + '/pyctr.c'
        )
        self.sftp.put(
            os.path.join(self.local_dir, ASSIGN_FILENAME),
            self.dir + '/' + ASSIGN_FILENAME
        )
        upload_folder(
            self.sftp,
            os.path.join(os.path.dirname(__file__), 'link_failure_grpc'),
            os.path.join(self.dir, 'link_failure_grpc')
        )
        controller_dir = os.path.join(self.dir,'controller')
        sn_remote_cmd(self.ssh, 'mkdir ' + controller_dir)
        all_node_states_dir = os.path.join(self.dir,'all_node_states')
        sn_remote_cmd(self.ssh, 'mkdir ' + all_node_states_dir)
        upload_folder(
            self.sftp,
            os.path.abspath(os.path.join(os.path.dirname(__file__), '..','geographic_srv6_anycast')),
            os.path.join(controller_dir, 'geographic_srv6_anycast')
        )

    
    def init_nodes(self):
        """
        Initializes nodes (e.g., satellites and ground stations) on the remote machine.
        """
        sn_remote_wait_output(
            self.ssh,
            f"python3 {self.dir}/sn_remote.py nodes {self.id} {self.dir}"
        )
        self.sftp.get(
            os.path.join(self.dir, PID_FILENAME),
            os.path.join(self.local_dir, PID_FILENAME)
        )
    
    def get_nodes(self):
        """
        Retrieves the list of nodes initialized on the remote machine.

        Returns:
            list: A list of node names.
        """
        lines = sn_remote_cmd(
            self.ssh,
            f"python3 {self.dir}/sn_remote.py list {self.id} {self.dir}"
        ).splitlines()[1:]
        nodes = [
            line.split()[0] for line in lines
        ]
        return nodes
    
    def init_network(self, isl_bw, isl_loss, gsl_bw, gsl_loss):
        """
        Initializes the network configuration on the remote machine.

        Args:
            isl_bw (int): Bandwidth for inter-satellite links.
            isl_loss (float): Packet loss rate for inter-satellite links.
            gsl_bw (int): Bandwidth for ground station links.
            gsl_loss (float): Packet loss rate for ground station links.
        """
        for shell_name, sat_names in self.shell_lst:
            isl_dir = os.path.join(self.dir, shell_name,'isl')
            sn_remote_cmd(self.ssh, 'mkdir ' + isl_dir)
            self.sftp.put(
                os.path.join(self.local_dir, shell_name, 'isl', 'init.txt'),
                os.path.join(isl_dir, 'init.txt')
            )
        if self.gs_dirname is not None:
            gsl_dir = os.path.join(self.dir, self.gs_dirname, 'gsl')
            sn_remote_cmd(self.ssh, 'mkdir ' + gsl_dir)
        self.update_network(-1, isl_bw, isl_loss, gsl_bw, gsl_loss)

    def upload_failure_recovery_file(self, t):
        """
        Uploads failure recovery files to the remote machine.

        Args:
            t (int): The timestamp of the failure recovery files to be uploaded.

        Steps:
            1. Upload the JSON file containing the updated node states for the given timestamp
            from the local directory to the remote machine.
            2. For each shell in the simulation:
                - Upload the ISL (Inter-Satellite Link) configuration file for the given timestamp
                from the local directory to the corresponding directory on the remote machine.
        """
        self.sftp.put(
            os.path.join(self.local_dir, 'all_node_states', f'{t}.json'),
            os.path.join(self.dir, 'all_node_states', f'{t}.json')
        )
        for shell_name, sat_names in self.shell_lst:
            self.sftp.put(
                os.path.join(self.local_dir, shell_name, 'isl', f'{t}.txt'),
                os.path.join(self.dir, shell_name,'isl', f'{t}.txt')
            )

    def update_network(self, t, isl_bw, isl_loss, gsl_bw, gsl_loss):
        """
        Updates the network configuration on the remote machine for a specific timestamp.

        Args:
            t (int): Timestamp for the network update (-1 for initialization).
            isl_bw (int): Bandwidth for inter-satellite links.
            isl_loss (float): Packet loss rate for inter-satellite links.
            gsl_bw (int): Bandwidth for ground station links.
            gsl_loss (float): Packet loss rate for ground station links.
        """
        if t != -1:
            self.sftp.put(
                os.path.join(self.local_dir, 'all_node_states', f'{t}.json'),
                os.path.join(self.dir, 'all_node_states', f'{t}.json')
            )
            for shell_name, sat_names in self.shell_lst:
                self.sftp.put(
                    os.path.join(self.local_dir, shell_name, 'isl', f'{t}.txt'),
                    os.path.join(self.dir, shell_name,'isl', f'{t}.txt')
                )
            if self.gs_dirname is not None:
                self.sftp.put(
                    os.path.join(self.local_dir, self.gs_dirname, 'gsl', f'{t}.txt'),
                    os.path.join(self.dir, self.gs_dirname, 'gsl', f'{t}.txt')
                )
        sn_remote_wait_output(
            self.ssh,
            f"python3 {self.dir}/sn_remote.py networks {self.id} {self.dir} "
            f"{t} {isl_bw} {isl_loss} {gsl_bw} {gsl_loss}"
        )

    def fault_test(self, t, isl_bw, isl_loss, gsl_bw, gsl_loss):
        """
        Simulates a fault test on the remote machine by introducing link failures.
        """
        sn_remote_wait_output(
            self.ssh,
            f"python3 {self.dir}/sn_remote.py fault_test {self.id} {self.dir} "
            f"{t} {isl_bw} {isl_loss} {gsl_bw} {gsl_loss}"
        )

    def deploy_tinyleo_srv6_agent(self):
        """
        Deploy the SRv6 agent on the remote machine for data plane operations.
        """
        sn_remote_wait_output(
            self.ssh,
            f"python3 {self.dir}/controller/geographic_srv6_anycast/deploy_srv6_agent.py"
        )

    def ping_async(self, res_path, src, dst):
        """
        Executes a ping test asynchronously between two nodes.

        Args:
            res_path (str): Path to save the ping results.
            src (str): Source node name.
            dst (str): Destination node name.

        Returns:
            threading.Thread: The thread executing the ping test.
        """
        def _ping_inner(ssh, dir, res_path, src, dst):
            output = sn_remote_cmd(
                ssh,
                f"python3 {dir}/sn_remote.py ping {self.id} {dir} "
                f"{src} {dst} 2>&1"
            )
            with open(res_path, 'w') as f:
                f.write(output)
        thread = threading.Thread(
            target=_ping_inner,
            args=(self.ssh, self.dir, res_path, src, dst)
        )
        thread.start()
        return thread
    
    def iperf_async(self, res_path, src, dst):
        """
        Executes an iperf test asynchronously between two nodes.

        Args:
            res_path (str): Path to save the iperf results.
            src (str): Source node name.
            dst (str): Destination node name.

        Returns:
            threading.Thread: The thread executing the iperf test.
        """
        def _iperf_inner(ssh, dir, res_path, src, dst):
            output = sn_remote_cmd(
                ssh,
                f"python3 {self.dir}/sn_remote.py iperf {self.id} {self.dir} "
                f"{src} {dst} 2>&1"
            )
            with open(res_path, 'w') as f:
                f.write(output)

        thread = threading.Thread(
            target=_iperf_inner,
            args=(self.ssh, self.dir, res_path, src, dst)
        )
        thread.start()
        return thread
    
    def traceroute_async(self, res_path, src, dst):
        """
        Executes a traceroute test asynchronously between two nodes.

        Args:
            res_path (str): Path to save the traceroute results.
            src (str): Source node name.
            dst (str): Destination node name.

        Returns:
            threading.Thread: The thread executing the traceroute test.
        """
        def _traceroute_async(ssh, dir, res_path, src, dst):
            output = sn_remote_cmd(
                ssh,
                f"python3 {self.dir}/sn_remote.py traceroute {self.id} {self.dir} "
                f"{src} {dst} 2>&1"
            )
            with open(res_path, 'w') as f:
                f.write(output)

        thread = threading.Thread(
            target=_traceroute_async,
            args=(self.ssh, self.dir, res_path, src, dst)
        )
        thread.start()
        return thread
    
    def check_utility(self, res_path):
        """
        Checks the system utility (e.g., CPU, memory usage) on the remote machine.

        Args:
            res_path (str): Path to save the utility check results.
        """
        output = sn_remote_cmd(self.ssh, "vmstat 2>&1")
        with open(res_path, 'w') as f:
            f.write(output)
        
    def clean(self):
        """
        Cleans up the remote machine by removing containers and resetting the environment.
        """
        sn_remote_cmd(
            self.ssh,
            f"python3 {self.dir}/sn_remote.py clean {self.id} {self.dir}"
        )