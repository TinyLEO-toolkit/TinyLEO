"""
Fault Handling Module for TinyLEO MPC (Model Predictive Control)

This module provides fault detection, diagnosis, and recovery capabilities
for TinyLEO satellite constellations using Model Predictive Control.
It implements various strategies to detect link failures, identify their root causes,
and execute appropriate recovery actions to maintain network connectivity.

The implementation aligns with the fault handling mechanisms described in the TinyLEO
paper, which aims to maintain network stability despite link failures in
both inter-domain and intra-domain satellite connections.
"""

import numpy as np
import os
import math
import time
from typing import Dict, List, Tuple, Set
from collections import defaultdict

# Global constants
R_EARTH = 6371  # Earth radius (km)
ATMOSPHERE_HEIGHT = 80  # Atmosphere height (km)
R_ATMOSPHERE = R_EARTH + ATMOSPHERE_HEIGHT  # Atmosphere outer radius (km)

class MPCFaultHandler:
    """
    MPC Fault Handler for satellite constellation link failures.
    
    This class provides methods to detect, diagnose, and recover from link failures
    in satellite constellations by utilizing Model Predictive Control techniques.
    """
    
    def __init__(self, base_dir, topo_dir, timestamp=0):
        """
        Initialize the MPC fault handler.
        
        Parameters:
            base_dir (str): Base directory containing satellite data
            topo_dir (str): Directory containing topology data
            timestamp (int): Current timestamp for data retrieval
        """
        self.base_dir = base_dir
        self.topo_dir = topo_dir
        self.timestamp = timestamp
        self.load_data()
        
    def load_data(self):
        """
        Load all required data for fault handling.
        
        This method loads:
        - Inter-domain topology
        - Intra-domain topology
        - Satellite parameters
        - Satellite locations
        - Grid-to-satellite mapping
        """
        # Load inter-domain topology
        self.inter_topology = np.load(
            os.path.join(self.topo_dir, 'inter_topology', f'{self.timestamp}.npy'), 
            allow_pickle=True
        )
        
        # Load intra-domain topology
        self.intra_topology = np.load(
            os.path.join(self.topo_dir, 'intra_topology', f'{self.timestamp}.npy'),
            allow_pickle=True
        ).item()
        
        # Load satellite parameters
        supply_data = np.load(
            os.path.join(self.base_dir, "eval1_573_jinyao_24k_half.npy"), 
            allow_pickle=True
        )
        
        # Extract satellite parameters
        self.num_satellites = len(supply_data)
        self.satellite_params = {}
        self.satellite_locations = {self.timestamp: {}}
        
        for idx, data in enumerate(supply_data):
            param, random_numbers, _, sat_location, _ = data
            
            self.satellite_params[idx] = {
                'height': param[0],
                'inclination': param[1],
                'alpha0': param[2],
                'initial_slot': random_numbers
            }
            
            # Only load location for the current timestamp
            self.satellite_locations[self.timestamp][idx] = sat_location[self.timestamp]
        
        # Load grid-to-satellite coverage data
        grid_satellites = np.load(
            os.path.join(self.base_dir, "new_grid_satellites.npy"), 
            allow_pickle=True
        ).item()
        self.grid_satellites = grid_satellites[self.timestamp]
        
    def get_original_satellite_id(self, virtual_id):
        """
        Convert virtual node ID to original satellite ID.
        
        Parameters:
            virtual_id (int): Virtual node identifier
            
        Returns:
            int: Original satellite identifier
        """
        return virtual_id % self.num_satellites
        
    def geodetic_to_cartesian(self, lat, lon, height):
        """
        Convert geodetic coordinates to Cartesian coordinates.
        
        Parameters:
            lat (float): Latitude in radians
            lon (float): Longitude in radians
            height (float): Height above Earth's surface in km
            
        Returns:
            tuple: (x, y, z) Cartesian coordinates in km
        """
        r = R_EARTH + height
        x = r * math.cos(lat) * math.cos(lon)
        y = r * math.cos(lat) * math.sin(lon)
        z = r * math.sin(lat)
        return x, y, z
        
    def calculate_distance(self, pos1, pos2):
        """
        Calculate Euclidean distance between two points in 3D space.
        
        Parameters:
            pos1 (tuple): First point coordinates (x, y, z)
            pos2 (tuple): Second point coordinates (x, y, z)
            
        Returns:
            float: Euclidean distance in km
        """
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(pos1, pos2)))
        
    def calculate_satellite_distance(self, sat1_id, sat2_id):
        """
        Calculate distance between two satellites.
        
        Parameters:
            sat1_id (int): First satellite ID
            sat2_id (int): Second satellite ID
            
        Returns:
            float: Distance between satellites in km
        """
        # Get satellite positions and heights
        lon1, lat1 = self.satellite_locations[self.timestamp][sat1_id]
        lon2, lat2 = self.satellite_locations[self.timestamp][sat2_id]
        height1 = self.satellite_params[sat1_id]['height']
        height2 = self.satellite_params[sat2_id]['height']
        
        # Convert to Cartesian coordinates
        pos1 = self.geodetic_to_cartesian(lat1, lon1, height1)
        pos2 = self.geodetic_to_cartesian(lat2, lon2, height2)
        
        # Calculate Euclidean distance
        return self.calculate_distance(pos1, pos2)
        
    def check_satellite_visibility(self, sat1_id, sat2_id):
        """
        Check if two satellites can establish a line-of-sight connection.
        
        This method determines if the line connecting two satellites intersects
        with Earth's atmosphere, which would block communication.
        
        Parameters:
            sat1_id (int): First satellite ID
            sat2_id (int): Second satellite ID
            
        Returns:
            bool: True if satellites can see each other, False otherwise
        """
        # Get satellite heights
        height1 = self.satellite_params[sat1_id]['height']
        height2 = self.satellite_params[sat2_id]['height']
        
        # Calculate maximum visible distance
        r1 = R_EARTH + height1
        r2 = R_EARTH + height2
        max_distance = math.sqrt(r1 ** 2 - R_ATMOSPHERE ** 2) + math.sqrt(r2 ** 2 - R_ATMOSPHERE ** 2)
        
        # Calculate actual distance
        distance = self.calculate_satellite_distance(sat1_id, sat2_id)
        
        return distance <= max_distance
        
    def get_intra_domain_neighbors(self, satellite_id, grid_id):
        """
        Get neighboring satellites within the same domain (grid).
        
        Parameters:
            satellite_id (int): Target satellite ID
            grid_id (int): Grid identifier
            
        Returns:
            list: List of tuples (satellite_id, virtual_node_id) of neighbors
        """
        neighbors = []
        
        if grid_id in self.intra_topology:
            for src, dst in self.intra_topology[grid_id]:
                src_sat = self.get_original_satellite_id(src)
                dst_sat = self.get_original_satellite_id(dst)
                
                if src_sat == satellite_id:
                    neighbors.append((dst_sat, dst))
                elif dst_sat == satellite_id:
                    neighbors.append((src_sat, src))
        
        return neighbors
        
    def find_inter_domain_connections(self, satellite_id):
        """
        Find all inter-domain connections involving the specified satellite.
        
        Parameters:
            satellite_id (int): Target satellite ID
            
        Returns:
            list: List of tuples (peer_satellite_id, peer_virtual_id, grid1, grid2)
                  representing inter-domain connections
        """
        connections = []
        
        for (node1, node2), (grid1, grid2) in self.inter_topology:
            sat1 = self.get_original_satellite_id(node1)
            sat2 = self.get_original_satellite_id(node2)
            
            if sat1 == satellite_id:
                connections.append((sat2, node2, grid1, grid2))
            elif sat2 == satellite_id:
                connections.append((sat1, node1, grid1, grid2))
        
        return connections
        
    def identify_failed_link_type(self, failed_sat1, failed_sat2):
        """
        Determine the type of failed link (inter-domain or intra-domain).
        
        Parameters:
            failed_sat1 (int): First failed satellite ID
            failed_sat2 (int): Second failed satellite ID
            
        Returns:
            tuple: (link_type, grid_ids, virtual_node_ids)
                   where link_type is "inter", "intra", or "unknown"
                   grid_ids is a tuple of affected grid IDs
                   virtual_node_ids is a tuple of virtual nodes involved
        """
        # Check if this is an inter-domain link
        for connection in self.inter_topology:
            (node1, node2), (grid1, grid2) = connection
            sat1 = self.get_original_satellite_id(node1)
            sat2 = self.get_original_satellite_id(node2)
            
            if (sat1 == failed_sat1 and sat2 == failed_sat2):
                return "inter", (grid1, grid2), (node1, node2)
            if (sat1 == failed_sat2 and sat2 == failed_sat1):
                return "inter", (grid2, grid1), (node2, node1)
        
        # Check if this is an intra-domain link
        for grid_id, connections in self.intra_topology.items():
            for src, dst in connections:
                src_sat = self.get_original_satellite_id(src)
                dst_sat = self.get_original_satellite_id(dst)
                
                if (src_sat == failed_sat1 and dst_sat == failed_sat2):
                    return "intra", (grid_id,), (src, dst)
                if (src_sat == failed_sat2 and dst_sat == failed_sat1):
                    return "intra", (grid_id,), (dst, src)
        
        # No matching link found
        return "unknown", (), ()
        
    def repair_inter_domain_link(self, failed_sat1, failed_sat2, virtual_nodes, grids):
        """
        Repair an inter-domain link failure.
        
        This method attempts to find a replacement satellite to restore connectivity
        between two domains when an inter-domain link fails.
        
        Parameters:
            failed_sat1 (int): First failed satellite ID
            failed_sat2 (int): Second failed satellite ID
            virtual_nodes (tuple): Virtual node IDs of failed satellites
            grids (tuple): Grid IDs where the failure occurred
            
        Returns:
            tuple: (inter_topology, intra_topology, replacement_info)
                   where inter_topology is updated inter-domain topology
                   intra_topology is updated intra-domain topology
                   replacement_info contains details about the replacement satellite
        """
        grid1, grid2 = grids
        node1, node2 = virtual_nodes
        replacement_info = None  # Initialize replacement info as None
        
        # Determine which satellite to remove and which to keep
        if self.get_original_satellite_id(node1) == failed_sat1:
            # Decide to remove satellite from the first grid
            sat_to_remove = failed_sat1
            virtual_to_remove = node1
            sat_to_keep = failed_sat2
            virtual_to_keep = node2
            grid_to_search = grid1
        else:
            # Decide to remove satellite from the second grid
            sat_to_remove = failed_sat2
            virtual_to_remove = node2
            sat_to_keep = failed_sat1
            virtual_to_keep = node1
            grid_to_search = grid2
        
        # Get intra-domain neighbors of the satellite to be removed
        neighbors_to_remove = self.get_intra_domain_neighbors(sat_to_remove, grid_to_search)
        
        # Search for a replacement satellite in the grid
        best_candidate = None
        best_virtual = None
        best_score = float('inf')
        
        for candidate_id in self.grid_satellites[grid_to_search]:
            candidate_real_id = self.get_original_satellite_id(candidate_id)
            
            # Skip the failed satellite
            if candidate_real_id == sat_to_remove:
                continue
            
            # Check if the candidate is already in use in topology
            is_used = False
            
            # Check intra-domain topology
            for src, dst in self.intra_topology.get(grid_to_search, []):
                src_id = self.get_original_satellite_id(src)
                dst_id = self.get_original_satellite_id(dst)
                if candidate_real_id == src_id or candidate_real_id == dst_id:
                    is_used = True
                    break
            
            # Check inter-domain topology
            if not is_used:
                for (n1, n2), (g1, g2) in self.inter_topology:
                    s1 = self.get_original_satellite_id(n1)
                    s2 = self.get_original_satellite_id(n2)
                    if candidate_real_id == s1 or candidate_real_id == s2:
                        is_used = True
                        break
            
            if is_used:
                continue
            
            # Check if candidate satellite is visible to the kept satellite
            if not self.check_satellite_visibility(candidate_real_id, sat_to_keep):
                continue
            
            # Check if candidate satellite is visible to all neighbors of removed satellite
            can_connect = True
            for neighbor_id, _ in neighbors_to_remove:
                if not self.check_satellite_visibility(candidate_real_id, neighbor_id):
                    can_connect = False
                    break
            
            if not can_connect:
                continue
            
            # Calculate distance as score
            distance_to_keep = self.calculate_satellite_distance(candidate_real_id, sat_to_keep)
            
            if distance_to_keep < best_score:
                best_score = distance_to_keep
                best_candidate = candidate_real_id
                best_virtual = candidate_id
        
        # If a suitable replacement satellite is found, update topology
        if best_candidate is not None:
            print(f"Inter-domain link repair: Using satellite {best_candidate} to replace satellite {sat_to_remove}")
            # Create replacement info dictionary
            replacement_info = {
                'removed_satellite': sat_to_remove,
                'removed_virtual_id': virtual_to_remove,
                'replacement_satellite': best_candidate,
                'replacement_virtual_id': best_virtual,
                'grid': grid_to_search
            }
            
            # Create new topology
            new_inter_topology = []
            for connection in self.inter_topology:
                (n1, n2), (g1, g2) = connection
                s1 = self.get_original_satellite_id(n1)
                s2 = self.get_original_satellite_id(n2)
                
                # Skip the failed link
                if (s1 == sat_to_remove and s2 == sat_to_keep) or (s1 == sat_to_keep and s2 == sat_to_remove):
                    continue
                
                # Add other inter-domain connections
                new_inter_topology.append(connection)
            
            # Add new inter-domain connection
            if grid_to_search == grid1:
                new_inter_topology.append(((best_virtual, virtual_to_keep), (grid1, grid2)))
            else:
                new_inter_topology.append(((virtual_to_keep, best_virtual), (grid1, grid2)))
            
            # Update intra-domain topology
            new_intra_topology = {}
            for g_id, connections in self.intra_topology.items():
                if g_id != grid_to_search:
                    new_intra_topology[g_id] = connections.copy()
                else:
                    # Create new connections list
                    new_connections = []
                    for src, dst in connections:
                        src_id = self.get_original_satellite_id(src)
                        dst_id = self.get_original_satellite_id(dst)
                        
                        # Skip connections involving the removed satellite
                        if src_id == sat_to_remove or dst_id == sat_to_remove:
                            continue
                        
                        # Preserve other connections
                        new_connections.append((src, dst))
                    
                    # Add connections between replacement satellite and neighbors of removed satellite
                    for _, neighbor_virtual in neighbors_to_remove:
                        new_connections.append((best_virtual, neighbor_virtual))
                    
                    new_intra_topology[g_id] = new_connections
            
            # Return only topology for the two involved grids
            result_intra_topology = {
                grid1: new_intra_topology.get(grid1, []),
                grid2: new_intra_topology.get(grid2, [])
            }
            
            # Filter inter-domain connections involving only these two grids
            result_inter_topology = []
            for (n1, n2), (g1, g2) in new_inter_topology:
                if (g1 == grid1 and g2 == grid2) or (g1 == grid2 and g2 == grid1):
                    result_inter_topology.append(((n1, n2), (g1, g2)))
            
            return result_inter_topology, result_intra_topology, replacement_info
            
        # Try replacing the second satellite if first attempt failed
        print(f"Failed to replace satellite {sat_to_remove}, now trying to replace satellite {sat_to_keep}...")
        
        # Reset replacement targets
        if sat_to_remove == failed_sat1:
            sat_to_remove = failed_sat2
            virtual_to_remove = node2
            sat_to_keep = failed_sat1
            virtual_to_keep = node1
            grid_to_search = grid2
        else:
            sat_to_remove = failed_sat1
            virtual_to_remove = node1
            sat_to_keep = failed_sat2
            virtual_to_keep = node2
            grid_to_search = grid1
        
        # Get new target satellite's intra-domain neighbors
        neighbors_to_remove = self.get_intra_domain_neighbors(sat_to_remove, grid_to_search)
        
        # Search again for a replacement satellite
        best_candidate = None
        best_virtual = None
        best_score = float('inf')
        for candidate_id in self.grid_satellites[grid_to_search]:
            candidate_real_id = self.get_original_satellite_id(candidate_id)
            
            # Skip the failed satellite
            if candidate_real_id == sat_to_remove:
                continue
            
            # Check if the candidate is already in use in topology
            is_used = False
            
            # Check intra-domain topology
            for src, dst in self.intra_topology.get(grid_to_search, []):
                src_id = self.get_original_satellite_id(src)
                dst_id = self.get_original_satellite_id(dst)
                if candidate_real_id == src_id or candidate_real_id == dst_id:
                    is_used = True
                    break
            
            # Check inter-domain topology
            if not is_used:
                for (n1, n2), (g1, g2) in self.inter_topology:
                    s1 = self.get_original_satellite_id(n1)
                    s2 = self.get_original_satellite_id(n2)
                    if candidate_real_id == s1 or candidate_real_id == s2:
                        is_used = True
                        break
            
            if is_used:
                continue
            
            # Check if candidate satellite is visible to the kept satellite
            if not self.check_satellite_visibility(candidate_real_id, sat_to_keep):
                continue
            
            # Check if candidate satellite is visible to all neighbors of removed satellite
            can_connect = True
            for neighbor_id, _ in neighbors_to_remove:
                if not self.check_satellite_visibility(candidate_real_id, neighbor_id):
                    can_connect = False
                    break
            
            if not can_connect:
                continue
            
            # Calculate distance as score
            distance_to_keep = self.calculate_satellite_distance(candidate_real_id, sat_to_keep)
            
            if distance_to_keep < best_score:
                best_score = distance_to_keep
                best_candidate = candidate_real_id
                best_virtual = candidate_id
        
        # If a suitable replacement satellite is found, update topology
        if best_candidate is not None:
            print(f"Inter-domain link repair: Using satellite {best_candidate} to replace satellite {sat_to_remove}")
            # Create replacement info dictionary
            replacement_info = {
                'removed_satellite': sat_to_remove,
                'removed_virtual_id': virtual_to_remove,
                'replacement_satellite': best_candidate,
                'replacement_virtual_id': best_virtual,
                'grid': grid_to_search
            }
            
            # Create new topology
            new_inter_topology = []
            for connection in self.inter_topology:
                (n1, n2), (g1, g2) = connection
                s1 = self.get_original_satellite_id(n1)
                s2 = self.get_original_satellite_id(n2)
                
                # Skip the failed link
                if (s1 == sat_to_remove and s2 == sat_to_keep) or (s1 == sat_to_keep and s2 == sat_to_remove):
                    continue
                
                # Add other inter-domain connections
                new_inter_topology.append(connection)
            
            # Add new inter-domain connection
            if grid_to_search == grid1:
                new_inter_topology.append(((best_virtual, virtual_to_keep), (grid1, grid2)))
            else:
                new_inter_topology.append(((virtual_to_keep, best_virtual), (grid1, grid2)))
            
            # Update intra-domain topology
            new_intra_topology = {}
            for g_id, connections in self.intra_topology.items():
                if g_id != grid_to_search:
                    new_intra_topology[g_id] = connections.copy()
                else:
                    # Create new connections list
                    new_connections = []
                    for src, dst in connections:
                        src_id = self.get_original_satellite_id(src)
                        dst_id = self.get_original_satellite_id(dst)
                        
                        # Skip connections involving the removed satellite
                        if src_id == sat_to_remove or dst_id == sat_to_remove:
                            continue
                        
                        # Preserve other connections
                        new_connections.append((src, dst))
                    
                    # Add connections between replacement satellite and neighbors of removed satellite
                    for _, neighbor_virtual in neighbors_to_remove:
                        new_connections.append((best_virtual, neighbor_virtual))
                    
                    new_intra_topology[g_id] = new_connections
            
            # Return only topology for the two involved grids
            result_intra_topology = {
                grid1: new_intra_topology.get(grid1, []),
                grid2: new_intra_topology.get(grid2, [])
            }
            
            # Filter inter-domain connections involving only these two grids
            result_inter_topology = []
            for (n1, n2), (g1, g2) in new_inter_topology:
                if (g1 == grid1 and g2 == grid2) or (g1 == grid2 and g2 == grid1):
                    result_inter_topology.append(((n1, n2), (g1, g2)))
            
            return result_inter_topology, result_intra_topology, replacement_info
            
        # Both attempts failed, output warning
        print(f"Warning: Cannot find suitable replacement satellite to repair inter-domain link {failed_sat1}-{failed_sat2}")
        # Return None to indicate no replacement found
        result_intra_topology = {
            grid1: self.intra_topology.get(grid1, []),
            grid2: self.intra_topology.get(grid2, [])
        }
        
        # Filter inter-domain connections involving only these two grids (excluding failed link)
        result_inter_topology = []
        for (n1, n2), (g1, g2) in self.inter_topology:
            s1 = self.get_original_satellite_id(n1)
            s2 = self.get_original_satellite_id(n2)
            
            # Exclude failed link
            if (s1 == failed_sat1 and s2 == failed_sat2) or (s1 == failed_sat2 and s2 == failed_sat1):
                continue
            
            if (g1 == grid1 and g2 == grid2) or (g1 == grid2 and g2 == grid1):
                result_inter_topology.append(((n1, n2), (g1, g2)))
                
        return result_inter_topology, result_intra_topology, None
    
    def repair_intra_domain_link(self, failed_sat1, failed_sat2, virtual_nodes, grid_id):
        """
        Repair an intra-domain link failure.
        
        This method attempts to find a replacement satellite to restore connectivity
        within a domain when an intra-domain link fails.
        
        Parameters:
            failed_sat1 (int): First failed satellite ID
            failed_sat2 (int): Second failed satellite ID
            virtual_nodes (tuple): Virtual node IDs of failed satellites
            grid_id (tuple): Single-element tuple containing grid ID
            
        Returns:
            tuple: (inter_topology, intra_topology, replacement_info)
                   where inter_topology is updated inter-domain topology
                   intra_topology is updated intra-domain topology
                   replacement_info contains details about the replacement satellite
        """
        failed_virtual1, failed_virtual2 = virtual_nodes
        grid_id = grid_id[0]  # Extract grid ID from tuple
        replacement_info = None  # Initialize replacement info as None
        
        # Decide which satellite to remove (choosing the first one)
        sat_to_remove = failed_sat1
        virtual_to_remove = failed_virtual1
        sat_to_keep = failed_sat2
        virtual_to_keep = failed_virtual2
        
        # Get all neighbors of the satellite to be removed (excluding the other failed satellite)
        neighbors_to_remove = []
        for n, v in self.get_intra_domain_neighbors(sat_to_remove, grid_id):
            if n != sat_to_keep:
                neighbors_to_remove.append((n, v))
        
        # Get inter-domain connections involving the satellite to be removed
        inter_connections = self.find_inter_domain_connections(sat_to_remove)
        print(inter_connections)
        
        # Search for a replacement satellite in the grid
        best_candidate = None
        best_virtual = None
        best_score = float('inf')
        
        for candidate_id in self.grid_satellites[grid_id]:
            candidate_real_id = self.get_original_satellite_id(candidate_id)
            
            # Skip failed satellites
            if candidate_real_id == sat_to_remove or candidate_real_id == sat_to_keep:
                continue
            
            # Check if the candidate is already in use in topology
            is_used = False
            
            # Check intra-domain topology
            for src, dst in self.intra_topology.get(grid_id, []):
                src_id = self.get_original_satellite_id(src)
                dst_id = self.get_original_satellite_id(dst)
                if candidate_real_id == src_id or candidate_real_id == dst_id:
                    is_used = True
                    break
            
            # Check inter-domain topology
            if not is_used:
                for (n1, n2), (g1, g2) in self.inter_topology:
                    s1 = self.get_original_satellite_id(n1)
                    s2 = self.get_original_satellite_id(n2)
                    if candidate_real_id == s1 or candidate_real_id == s2:
                        is_used = True
                        break
            
            if is_used:
                continue
            
            # Check if candidate can connect to all required satellites
            can_connect = True
            
            # Check visibility with kept failed satellite
            if not self.check_satellite_visibility(candidate_real_id, sat_to_keep):
                can_connect = False
            
            # Check visibility with neighbors of removed satellite
            for neighbor_id, _ in neighbors_to_remove:
                if not self.check_satellite_visibility(candidate_real_id, neighbor_id):
                    can_connect = False
                    break
            
            # Check visibility with inter-domain connections
            for connected_sat, _, _, _ in inter_connections:
                if not self.check_satellite_visibility(candidate_real_id, connected_sat):
                    can_connect = False
                    break
            
            if not can_connect:
                continue
            
            # Calculate total distance to all connected satellites as score
            total_distance = self.calculate_satellite_distance(candidate_real_id, sat_to_keep)
            
            for neighbor_id, _ in neighbors_to_remove:
                total_distance += self.calculate_satellite_distance(candidate_real_id, neighbor_id)
            
            for connected_sat, _, _, _ in inter_connections:
                total_distance += self.calculate_satellite_distance(candidate_real_id, connected_sat)
            
            if total_distance < best_score:
                best_score = total_distance
                best_candidate = candidate_real_id
                best_virtual = candidate_id
        
        # If a suitable replacement satellite is found, update topology
        if best_candidate is not None:
            print(f"Intra-domain link repair: Using satellite {best_candidate} to replace satellite {sat_to_remove}")
            # Create replacement info dictionary
            replacement_info = {
                'removed_satellite': sat_to_remove,
                'removed_virtual_id': virtual_to_remove,
                'replacement_satellite': best_candidate,
                'replacement_virtual_id': best_virtual,
                'grid': grid_id
            }
            
            # Create new topology
            new_inter_topology = []
            for connection in self.inter_topology:
                (n1, n2), (g1, g2) = connection
                s1 = self.get_original_satellite_id(n1)
                s2 = self.get_original_satellite_id(n2)
                
                # Replace inter-domain connections involving the removed satellite
                if s1 == sat_to_remove:
                    new_inter_topology.append(((best_virtual, n2), (g1, g2)))
                elif s2 == sat_to_remove:
                    new_inter_topology.append(((n1, best_virtual), (g1, g2)))
                else:
                    new_inter_topology.append(connection)
            
            # Update intra-domain topology
            new_intra_topology = {}
            for g_id, connections in self.intra_topology.items():
                if g_id != grid_id:
                    new_intra_topology[g_id] = connections.copy()
                else:
                    # Create new connections list
                    new_connections = []
                    for src, dst in connections:
                        src_id = self.get_original_satellite_id(src)
                        dst_id = self.get_original_satellite_id(dst)
                        
                        # Skip connections involving the removed satellite
                        if src_id == sat_to_remove or dst_id == sat_to_remove:
                            continue
                        
                        # Preserve other connections
                        new_connections.append((src, dst))
                    
                    # Add connection between replacement satellite and kept failed satellite
                    new_connections.append((best_virtual, virtual_to_keep))
                    
                    # Add connections between replacement satellite and neighbors of removed satellite
                    for _, neighbor_virtual in neighbors_to_remove:
                        new_connections.append((best_virtual, neighbor_virtual))
                    
                    new_intra_topology[grid_id] = new_connections
            
            # Determine all involved grids
            involved_grids = {grid_id}
            for _, _, g1, g2 in inter_connections:
                involved_grids.add(g1)
                involved_grids.add(g2)
            
            # Return topology for involved grids only
            result_intra_topology = {g: new_intra_topology.get(g, []) for g in involved_grids}
            
            # Filter inter-domain connections involving these grids
            result_inter_topology = []
            for (n1, n2), (g1, g2) in new_inter_topology:
                if g1 in involved_grids and g2 in involved_grids:
                    result_inter_topology.append(((n1, n2), (g1, g2)))
            
            return result_inter_topology, result_intra_topology, replacement_info
            
        # Try replacing the second satellite
        print(f"Failed to replace satellite {sat_to_remove}, now trying to replace satellite {sat_to_keep}...")
        
        # Reset replacement targets
        sat_to_remove = failed_sat2
        virtual_to_remove = failed_virtual2
        sat_to_keep = failed_sat1
        virtual_to_keep = failed_virtual1
        
        # Get all neighbors of the satellite to be removed (excluding the other failed satellite)
        neighbors_to_remove = []
        for n, v in self.get_intra_domain_neighbors(sat_to_remove, grid_id):
            if n != sat_to_keep:
                neighbors_to_remove.append((n, v))
        
        # Get inter-domain connections involving the satellite to be removed
        inter_connections = self.find_inter_domain_connections(sat_to_remove)
        print(inter_connections)
        
        # Search for a replacement satellite in the grid
        best_candidate = None
        best_virtual = None
        best_score = float('inf')
        
        for candidate_id in self.grid_satellites[grid_id]:
            candidate_real_id = self.get_original_satellite_id(candidate_id)
            
            # Skip failed satellites
            if candidate_real_id == sat_to_remove or candidate_real_id == sat_to_keep:
                continue
            
            # Check if the candidate is already in use in topology
            is_used = False
            
            # Check intra-domain topology
            for src, dst in self.intra_topology.get(grid_id, []):
                src_id = self.get_original_satellite_id(src)
                dst_id = self.get_original_satellite_id(dst)
                if candidate_real_id == src_id or candidate_real_id == dst_id:
                    is_used = True
                    break
            
            # Check inter-domain topology
            if not is_used:
                for (n1, n2), (g1, g2) in self.inter_topology:
                    s1 = self.get_original_satellite_id(n1)
                    s2 = self.get_original_satellite_id(n2)
                    if candidate_real_id == s1 or candidate_real_id == s2:
                        is_used = True
                        break
            
            if is_used:
                continue
            
            # Check if candidate can connect to all required satellites
            can_connect = True
            
            # Check visibility with kept failed satellite
            if not self.check_satellite_visibility(candidate_real_id, sat_to_keep):
                can_connect = False
            
            # Check visibility with neighbors of removed satellite
            for neighbor_id, _ in neighbors_to_remove:
                if not self.check_satellite_visibility(candidate_real_id, neighbor_id):
                    can_connect = False
                    break
            
            # Check visibility with inter-domain connections
            for connected_sat, _, _, _ in inter_connections:
                if not self.check_satellite_visibility(candidate_real_id, connected_sat):
                    can_connect = False
                    break
            
            if not can_connect:
                continue
            
            # Calculate total distance to all connected satellites as score
            total_distance = self.calculate_satellite_distance(candidate_real_id, sat_to_keep)
            
            for neighbor_id, _ in neighbors_to_remove:
                total_distance += self.calculate_satellite_distance(candidate_real_id, neighbor_id)
            
            for connected_sat, _, _, _ in inter_connections:
                total_distance += self.calculate_satellite_distance(candidate_real_id, connected_sat)
            
            if total_distance < best_score:
                best_score = total_distance
                best_candidate = candidate_real_id
                best_virtual = candidate_id
        
        # If a suitable replacement satellite is found, update topology
        if best_candidate is not None:
            print(f"Intra-domain link repair: Using satellite {best_candidate} to replace satellite {sat_to_remove}")
            # Create replacement info dictionary
            replacement_info = {
                'removed_satellite': sat_to_remove,
                'removed_virtual_id': virtual_to_remove,
                'replacement_satellite': best_candidate,
                'replacement_virtual_id': best_virtual,
                'grid': grid_id
            }
            
            # Create new topology
            new_inter_topology = []
            for connection in self.inter_topology:
                (n1, n2), (g1, g2) = connection
                s1 = self.get_original_satellite_id(n1)
                s2 = self.get_original_satellite_id(n2)
                
                # Replace inter-domain connections involving the removed satellite
                if s1 == sat_to_remove:
                    new_inter_topology.append(((best_virtual, n2), (g1, g2)))
                elif s2 == sat_to_remove:
                    new_inter_topology.append(((n1, best_virtual), (g1, g2)))
                else:
                    new_inter_topology.append(connection)
            
            # Update intra-domain topology
            new_intra_topology = {}
            for g_id, connections in self.intra_topology.items():
                if g_id != grid_id:
                    new_intra_topology[g_id] = connections.copy()
                else:
                    # Create new connections list
                    new_connections = []
                    for src, dst in connections:
                        src_id = self.get_original_satellite_id(src)
                        dst_id = self.get_original_satellite_id(dst)
                        
                        # Skip connections involving the removed satellite
                        if src_id == sat_to_remove or dst_id == sat_to_remove:
                            continue
                        
                        # Preserve other connections
                        new_connections.append((src, dst))
                    
                    # Add connection between replacement satellite and kept failed satellite
                    new_connections.append((best_virtual, virtual_to_keep))
                    print(virtual_to_keep)
                    
                    # Add connections between replacement satellite and neighbors of removed satellite
                    for _, neighbor_virtual in neighbors_to_remove:
                        new_connections.append((best_virtual, neighbor_virtual))
                    
                    new_intra_topology[grid_id] = new_connections
            
            # Determine all involved grids
            involved_grids = {grid_id}
            for _, _, g1, g2 in inter_connections:
                involved_grids.add(g1)
                involved_grids.add(g2)
            
            # Return topology for involved grids only
            result_intra_topology = {g: new_intra_topology.get(g, []) for g in involved_grids}
            
            # Filter inter-domain connections involving these grids
            result_inter_topology = []
            for (n1, n2), (g1, g2) in new_inter_topology:
                if g1 in involved_grids and g2 in involved_grids:
                    result_inter_topology.append(((n1, n2), (g1, g2)))
            
            return result_inter_topology, result_intra_topology, replacement_info
            
        # Both attempts failed, output warning
        print(f"Warning: Cannot find suitable replacement satellite to repair intra-domain link {failed_sat1}-{failed_sat2}")
        
        # Return None to indicate no replacement found
        involved_grids = {grid_id}
        for sat in [failed_sat1, failed_sat2]:
            for _, _, g1, g2 in self.find_inter_domain_connections(sat):
                involved_grids.add(g1)
                involved_grids.add(g2)
                
        # Create new intra-domain topology, excluding failed link
        result_intra_topology = {}
        for g_id in involved_grids:
            if g_id not in self.intra_topology:
                result_intra_topology[g_id] = []
                continue
                
            new_connections = []
            for src, dst in self.intra_topology[g_id]:
                src_id = self.get_original_satellite_id(src)
                dst_id = self.get_original_satellite_id(dst)
                
                # Skip failed link
                if (src_id == failed_sat1 and dst_id == failed_sat2) or (src_id == failed_sat2 and dst_id == failed_sat1):
                    continue
                
                new_connections.append((src, dst))
                
            result_intra_topology[g_id] = new_connections
            
        # Filter inter-domain connections involving these grids
        result_inter_topology = []
        for (n1, n2), (g1, g2) in self.inter_topology:
            if g1 in involved_grids and g2 in involved_grids:
                result_inter_topology.append(((n1, n2), (g1, g2)))
                
        return result_inter_topology, result_intra_topology, None

    def handle_link_failure(self, failed_sat1, failed_sat2):
        """
        Handle a link failure between two satellites.
        
        This is the main entry point for fault handling, which:
        1. Identifies the type of link failure
        2. Calls the appropriate repair method
        3. Returns the repaired topology
        
        Parameters:
            failed_sat1 (int): First failed satellite ID
            failed_sat2 (int): Second failed satellite ID
            
        Returns:
            dict: Result containing:
                  - link_type: "inter-domain" or "intra-domain"
                  - failed_satellites: Tuple of failed satellite IDs
                  - grids: Affected grid IDs
                  - inter_topology: Updated inter-domain topology
                  - intra_topology: Updated intra-domain topology
                  - replacement_info: Details about replacement (if any)
        """
        # Determine the type of failed link
        link_type, grids, virtual_nodes = self.identify_failed_link_type(failed_sat1, failed_sat2)
        
        if link_type == "inter":
            print(f"Detected inter-domain link failure: Satellite {failed_sat1}-Satellite {failed_sat2}, affecting grids {grids}")
            result_inter, result_intra, replacement_info = self.repair_inter_domain_link(
                failed_sat1, failed_sat2, virtual_nodes, grids
            )
            result = {
                'link_type': 'inter-domain',
                'failed_satellites': (failed_sat1, failed_sat2),
                'grids': grids,
                'inter_topology': result_inter,
                'intra_topology': result_intra
            }
            # Add replacement info if available
            if replacement_info:
                result['replacement_info'] = replacement_info
            return result
            
        elif link_type == "intra":
            print(f"Detected intra-domain link failure: Satellite {failed_sat1}-Satellite {failed_sat2}, affecting grid {grids[0]}")
            result_inter, result_intra, replacement_info = self.repair_intra_domain_link(
                failed_sat1, failed_sat2, virtual_nodes, grids
            )
            result = {
                'link_type': 'intra-domain',
                'failed_satellites': (failed_sat1, failed_sat2),
                'grid': grids[0],
                'inter_topology': result_inter,
                'intra_topology': result_intra
            }
            # Add replacement info if available
            if replacement_info:
                result['replacement_info'] = replacement_info
            return result
            
        else:
            print(f"Error: No link found between Satellite {failed_sat1} and Satellite {failed_sat2}")
            return None

# Main program
if __name__ == "__main__":
    # Example usage
    base_dir = "test/data/topo_data"  # Data directory
    topo_dir = "test/tinyleo-Arbitrary-LeastDelay"  # Topology data directory
    
    # Initialize MPC fault handler
    mpc = MPCFaultHandler(base_dir=base_dir, topo_dir=topo_dir, timestamp=0)
    
    # Specify the two satellite IDs of the failed link
    # failed_satellite1 = 1044  # First failed satellite ID
    # failed_satellite2 = 750  # Second failed satellite ID

    failed_satellite1 = 1482  # First failed satellite ID
    failed_satellite2 = 1595  # Second failed satellite ID
    
    # Process the failure and get the repaired topology
    result = mpc.handle_link_failure(failed_satellite1, failed_satellite2)
    print(result)
    if result:
        # Print replacement information (if available)
        if 'replacement_info' in result:
            rep_info = result['replacement_info']
            print(f"\nReplacement Information Summary:")
            print(f"  Removed satellite: {rep_info['removed_satellite']} (Virtual ID: {rep_info['removed_virtual_id']})")
            print(f"  Replacement satellite: {rep_info['replacement_satellite']} (Virtual ID: {rep_info['replacement_virtual_id']})")
            print(f"  Grid ID: {rep_info['grid']}")
        else:
            print("\nNo suitable replacement satellite found, only removed the failed link")
        