"""
Orbital Model Predictive Controller (MPC) for TinyLEO.

This module implements the MPC approach described in section 4.2 of the TinyLEO paper:
"Stable Model Predictive Control Plane" and corresponds to the control-plane 
functionality in the TinyLEO toolkit (section 5).

The MPC decouples high-level networking intents (stable demands) from low-level 
enforcements (dynamic supplies) for high network usability. It accomplishes this
through a three-stage matching process:
1. Satellite-to-grid matching: Assign satellites as gateways to neighboring grids
2. Inter-grid satellite matching: Establish ISLs between grid pairs based on preferences
3. Intra-grid satellite connections: Create rings within each grid for local connectivity
"""

import numpy as np
import os
import time
import math
import random
from multiprocessing import Pool
from collections import defaultdict
from typing import Dict, List, Tuple, Set, Optional
from tqdm import tqdm

from utility_functions import *

# Global caches for topology results
GLOBAL_INTER_DOMAIN_TOPOLOGY_CACHE = {}  # Inter-domain topology cache {timestamp: topology}
GLOBAL_INTRA_DOMAIN_TOPOLOGY_CACHE = {}  # Intra-domain topology cache {timestamp: topology}
used_virtual_nodes = set()  # Global variable to track used virtual nodes

# ISL sustainability assessment (caching for performance)
sustainability_cache = {}

def calculate_isl_sustainability(sat1_data: dict, sat2_data: dict, 
                               sat_locations: Dict[int, Dict], 
                               current_timestamp: int,
                               sat1_id: int,
                               sat2_id: int,
                               max_timestamps: int = 21) -> int:
    """
    Calculate how long an inter-satellite link (ISL) can be maintained.
    Corresponds to the ISL stability metric used in the paper's MPC approach.
    
    Args:
        sat1_data: Parameters of first satellite
        sat2_data: Parameters of second satellite
        sat_locations: Dictionary of satellite locations by timestamp
        current_timestamp: Current timestamp
        sat1_id: ID of first satellite
        sat2_id: ID of second satellite
        max_timestamps: Maximum number of timestamps to look ahead
        
    Returns:
        Number of timestamps the ISL can be maintained
    """
    # Create cache key (ensure sat1_id < sat2_id for consistency)
    cache_key = (min(sat1_id, sat2_id), max(sat1_id, sat2_id), current_timestamp)
    
    # Check cache
    if cache_key in sustainability_cache:
        return sustainability_cache[cache_key]
    
    # Get satellite heights
    height1 = sat1_data['height']
    height2 = sat2_data['height']
    
    # Calculate maximum visible distance
    r1 = R_EARTH + height1
    r2 = R_EARTH + height2
    max_distance = math.sqrt(r1 ** 2 - R_ATMOSPHERE ** 2) + math.sqrt(r2 ** 2 - R_ATMOSPHERE ** 2)
    
    # Calculate sustainability time
    sustainability_time = 0
    for t in range(current_timestamp, min(current_timestamp + max_timestamps, len(sat_locations))):
        lon1, lat1 = sat_locations[t][sat1_id]
        lon2, lat2 = sat_locations[t][sat2_id]
        
        pos1 = geodetic_to_cartesian(lat1, lon1, height1)
        pos2 = geodetic_to_cartesian(lat2, lon2, height2)
        
        distance = calculate_distance(pos1, pos2)
        if distance > max_distance:
            break
            
        sustainability_time += 1
    
    # Store in cache
    sustainability_cache[cache_key] = sustainability_time
    return sustainability_time

#--------------------------------------------------
# Phase 1: Satellite-to-Grid Matching
#--------------------------------------------------

def calculate_min_distance_to_grid(node_id: int, grid_id: int, neighbor_grid_id: int,
                                 grid_satellites: Dict, satellite_params: Dict,
                                 satellite_locations: Dict, current_timestamp: int,
                                 num_satellites: int) -> Tuple[float, int]:
    """
    Calculate minimum distance from a node to any satellite in target grid.
    This is part of the first stage of MPC's three-stage matching process.
    
    Args:
        node_id: Virtual node ID
        grid_id: Source grid ID
        neighbor_grid_id: Target neighbor grid ID
        grid_satellites: Grid to satellites mapping
        satellite_params: Satellite parameters
        satellite_locations: Satellite locations
        current_timestamp: Current timestamp
        num_satellites: Total number of satellites
        
    Returns:
        Tuple of (minimum_distance, closest_node_id) or (infinity, None) if no visible satellites
    """
    original_node_id = get_original_satellite_id(node_id, num_satellites)
    
    min_distance = float('inf')
    min_node = None
    has_visible = False
    
    # Calculate visibility parameters
    height1 = satellite_params[original_node_id]['height']
    r1 = R_EARTH + height1
    
    # Calculate distance to all satellites in target grid, find minimum
    for other_node in grid_satellites[current_timestamp][neighbor_grid_id]:
        other_sat_id = get_original_satellite_id(other_node, num_satellites)
        if other_sat_id != original_node_id:  # Exclude same satellite
            height2 = satellite_params[other_sat_id]['height']
            r2 = R_EARTH + height2
            max_distance = math.sqrt(r1 ** 2 - R_ATMOSPHERE ** 2) + math.sqrt(r2 ** 2 - R_ATMOSPHERE ** 2)
            
            distance = calculate_satellite_distance(
                original_node_id,
                other_sat_id,
                satellite_locations,
                current_timestamp,
                satellite_params
            )
            
            if distance <= max_distance:  # Only consider visible satellites
                has_visible = True
                if distance < min_distance:
                    min_distance = distance
                    min_node = other_node
    
    return (min_distance, min_node) if has_visible else (float('inf'), None)

def match_grid_nodes(grid_id: int, grid_satellites: Dict, traffic_matrix: np.ndarray,
                   satellite_params: Dict, satellite_locations: Dict,
                   current_timestamp: int, num_satellites: int,
                   previous_assignments: Dict[Tuple[int, int], List[int]] = None,
                   excluded_nodes: Set[int] = None) -> Tuple[int, Dict[int, List[int]]]:
    """
    Satellite-to-Grid Gateway Matching (Stage 1 of MPC's three-stage matching process).
    
    This function corresponds directly to the first part of MPC in paper section 4.2:
    "For each connected neighbor v with n_u,v > 0, u should allocate n_u,v out of its n_u 
    satellites as 'gateways' toward v via ISLs to meet the demand."
    
    Args:
        grid_id: Grid ID to process
        grid_satellites: Grid to satellites mapping
        traffic_matrix: Traffic demand matrix
        satellite_params: Satellite parameters
        satellite_locations: Satellite locations
        current_timestamp: Current timestamp
        num_satellites: Total number of satellites
        previous_assignments: Previously assigned nodes (optional)
        excluded_nodes: Nodes to exclude from matching (optional)
        
    Returns:
        Tuple of (grid_id, assignments) where assignments is a dict mapping
        neighbor grid IDs to lists of assigned nodes
    """
    if previous_assignments is None:
        previous_assignments = {}
    if excluded_nodes is None:
        excluded_nodes = set()
        
    current_grid_satellites = grid_satellites[current_timestamp]
    
    if not current_grid_satellites[grid_id]:
        return grid_id, {}
        
    # Get available nodes
    available_nodes = [node for node in current_grid_satellites[grid_id] 
                      if node not in excluded_nodes]
    
    if not available_nodes:
        return grid_id, {}
    
    # Get all neighboring grids and their demands
    neighbors = get_neighboring_grids(grid_id)
    grid_demands = {
        n: traffic_matrix[grid_id][n] 
        for n in neighbors 
        if traffic_matrix[grid_id][n] > 0
    }
    
    # Calculate minimum distance from each node to each target grid
    node_grid_distances = {}
    for node in available_nodes:
        node_grid_distances[node] = {}
        for neighbor_id in grid_demands.keys():
            min_dist, min_node = calculate_min_distance_to_grid(
                node, grid_id, neighbor_id,
                grid_satellites, satellite_params,
                satellite_locations, current_timestamp,
                num_satellites
            )
            node_grid_distances[node][neighbor_id] = (min_dist, min_node) 
    
    # Matching algorithm - Implementation of many-to-one stable matching described in paper
    assignments = defaultdict(list)
    remaining_nodes = set(available_nodes)
    
    # Process grids by demand in descending order (prioritize higher demand grids)
    sorted_neighbors = sorted(grid_demands.items(), key=lambda x: x[1], reverse=True)
    
    for neighbor_id, required in sorted_neighbors:
        # Sort satellites by distance (preference based on distance)
        all_distances = [(node, *node_grid_distances[node][neighbor_id])
                        for node in remaining_nodes]
        all_distances.sort(key=lambda x: x[1])  # Sort by distance (ascending)
        
        # Select optimal nodes - TinyLEO's gateway selection process
        selected = 0
        for node, min_dist, min_target_node in all_distances:
            if min_dist == float('inf'):  # Skip non-visible nodes
                continue
            if selected >= required or node not in remaining_nodes:
                break
                
            assignments[neighbor_id].append(node)
            remaining_nodes.remove(node)
            selected += 1
    
    # Return grid_id and matching results
    return grid_id, assignments

#--------------------------------------------------
# Phase 2: Inter-Grid Satellite Matching (ISL Establishment)
#--------------------------------------------------

def stable_match_satellites(nodes_grid1: List[int], nodes_grid2: List[int],
                          grid1_id: int, grid2_id: int,
                          satellite_params: Dict, satellite_locations: Dict,
                          current_timestamp: int, num_satellites: int) -> List[Tuple[Tuple[int, int], Tuple[int, int]]]:
    """
    Establish ISLs between grid pairs using stable matching (Stage 2 of MPC's three-stage matching).
    
    This function corresponds to the second part of MPC in paper section 4.2:
    "TinyLEO runs another one-to-one stable matching between these satellites in u and v
    using the ISL lifetime as satellite s ∈ u's preference to s' ∈ v."
    
    Args:
        nodes_grid1: List of node IDs in first grid
        nodes_grid2: List of node IDs in second grid
        grid1_id: ID of first grid
        grid2_id: ID of second grid
        satellite_params: Satellite parameters
        satellite_locations: Satellite locations
        current_timestamp: Current timestamp
        num_satellites: Total number of satellites
        
    Returns:
        List of connections, each as ((node1, node2), (grid1, grid2))
    """
    if not nodes_grid1 or not nodes_grid2:
        return []
    
    def get_node_preference(node1: int, node2: int) -> Tuple[float, bool]:
        """Calculate preference between nodes based on distance and orbit"""
        sat1 = get_original_satellite_id(node1, num_satellites)
        sat2 = get_original_satellite_id(node2, num_satellites)
        
        if sat1 == sat2:  # Exclude same satellite
            return (float('inf'), False)
        
        # Calculate distance between satellites
        distance = calculate_satellite_distance(
            sat1,
            sat2,
            satellite_locations,
            current_timestamp,
            satellite_params
        )
        
        # Check if within visible range
        height1 = satellite_params[sat1]['height']
        height2 = satellite_params[sat2]['height']
        r1 = R_EARTH + height1
        r2 = R_EARTH + height2
        max_distance = math.sqrt(r1 ** 2 - R_ATMOSPHERE ** 2) + math.sqrt(r2 ** 2 - R_ATMOSPHERE ** 2)
        
        if distance > max_distance:
            return (float('inf'), False)
        
        # Consider same-orbit property for preferences, as mentioned in the paper
        # for topological stability
        same_orbit = is_same_orbit(satellite_params[sat1], satellite_params[sat2])
        return (distance, same_orbit)
    
    # Build preference lists
    preferences_grid1 = {}
    preferences_grid2 = {}
    
    # Build preference lists for nodes in grid1
    for n1 in nodes_grid1:
        prefs = []
        for n2 in nodes_grid2:
            distance, same_orbit = get_node_preference(n1, n2)
            if distance != float('inf'):  # Only consider visible connections
                prefs.append((n2, distance, same_orbit))
        # Sort by distance (ascending) and same-orbit priority
        prefs.sort(key=lambda x: (x[1], not x[2]))
        preferences_grid1[n1] = [n for n, _, _ in prefs]
    
    # Build preference lists for nodes in grid2
    for n2 in nodes_grid2:
        prefs = []
        for n1 in nodes_grid1:
            distance, same_orbit = get_node_preference(n2, n1)
            if distance != float('inf'):
                prefs.append((n1, distance, same_orbit))
        # Sort by distance (ascending) and same-orbit priority
        prefs.sort(key=lambda x: (x[1], not x[2]))
        preferences_grid2[n2] = [n for n, _, _ in prefs]
    
    # Gale-Shapley algorithm for stable matching (as mentioned in the paper)
    matches = {}
    free_nodes = set(nodes_grid1)
    proposals = {node: 0 for node in nodes_grid1}
    
    while free_nodes:
        n1 = free_nodes.pop()
        if not preferences_grid1.get(n1, []):
            continue
            
        while proposals[n1] < len(preferences_grid1[n1]):
            n2 = preferences_grid1[n1][proposals[n1]]
            proposals[n1] += 1
            
            if n2 not in matches:
                matches[n2] = n1
                break
            else:
                current_match = matches[n2]
                if (preferences_grid2[n2].index(n1) < 
                    preferences_grid2[n2].index(current_match)):
                    matches[n2] = n1
                    free_nodes.add(current_match)
                    break
            
            if proposals[n1] >= len(preferences_grid1[n1]):
                free_nodes.add(n1)
    
    # Convert result format
    final_connections = []
    for n2, n1 in matches.items():
        node_connection = (n1, n2)
        grid_connection = (grid1_id, grid2_id)
        final_connections.append((node_connection, grid_connection))
    
    return final_connections

def perform_second_phase_matching(first_phase_result, satellite_params, satellite_locations, 
                        current_timestamp, num_satellites):
    """
    Orchestrate the second phase of the three-stage matching: process all grid pairs and 
    match satellites to form ISLs.
    
    Args:
        first_phase_result: Result from first phase matching
        satellite_params: Satellite parameters
        satellite_locations: Satellite locations
        current_timestamp: Current timestamp
        num_satellites: Total number of satellites
        
    Returns:
        List of topology connections
    """
    start_time = time.time()
    num_grids = len(first_phase_result)
    topology_connections = []  # Store final connections
    
    # Direct iteration over all grid pairs
    found_pairs = 0
    
    # Use nested loops to process grid pairs
    for i in range(num_grids):
        for j in range(i + 1, num_grids):
            if first_phase_result[i][j] != [[], [], []]:
                nodes_grid1 = first_phase_result[i][j][0]
                nodes_grid2 = first_phase_result[j][i][0]
                if nodes_grid1 and nodes_grid2:
                    found_pairs += 1
                    try:
                        # Call stable matching algorithm to process this grid pair
                        matches = stable_match_satellites(
                            nodes_grid1, 
                            nodes_grid2,
                            i, j,
                            satellite_params,
                            satellite_locations,
                            current_timestamp,
                            num_satellites
                        )
                        # Add matching results
                        topology_connections.extend(matches)
                        
                    except Exception as e:
                        print(f"\nError processing grid pair ({i},{j}): {str(e)}")
    
    elapsed = time.time() - start_time
    print(f"Second phase matching completed: Processed {found_pairs} grid pairs, generated {len(topology_connections)} ISL connections, time: {elapsed:.2f}s")
    
    return topology_connections

#--------------------------------------------------
# Phase 3: Intra-Grid Satellite Ring Formation
#--------------------------------------------------

def classify_satellites_to_grids(topology_connections: List[Tuple[Tuple[int, int], Tuple[int, int]]],
                               grid_satellites: Dict,
                               current_timestamp: int) -> Dict[int, Set[int]]:
    """
    Classify satellite nodes to their respective grids for intra-grid topology formation.
    
    Args:
        topology_connections: Inter-domain topology connections
        grid_satellites: Grid to satellites mapping
        current_timestamp: Current timestamp
        
    Returns:
        Dictionary mapping grid IDs to sets of node IDs
    """
    # Initialize result dictionary
    grid_nodes = defaultdict(set)  # Store nodes for each grid
    
    # Process nodes in inter-domain topology connections
    for node_conn, grid_conn in topology_connections:
        node1, node2 = node_conn
        grid1, grid2 = grid_conn
        
        # Assign nodes to corresponding grids
        grid_nodes[grid1].add(node1)
        grid_nodes[grid2].add(node2)
        
    return dict(grid_nodes)

def create_intra_grid_topology(grid_nodes: Set[int],
                             satellite_params: Dict,
                             satellite_locations: Dict,
                             current_timestamp: int,
                             num_satellites: int) -> List[Tuple[int, int]]:
    """
    Create ring topology for a single grid based on distance.
    
    This function corresponds to the third part of MPC in paper section 4.2:
    "Last, inside each cell u, TinyLEO connects all above matched satellites s ∈ u as a ring 
    to ensure their connectivity."
    
    Args:
        grid_nodes: Set of node IDs in the grid
        satellite_params: Satellite parameters
        satellite_locations: Satellite locations
        current_timestamp: Current timestamp
        num_satellites: Total number of satellites
        
    Returns:
        List of connections as (node1, node2) pairs
    """
    nodes = list(grid_nodes)
    n = len(nodes)
    
    # Handle special cases
    if n <= 1:
        return []
    if n == 2:
        return [(nodes[0], nodes[1])]
    
    # Initialize data structures
    remaining_nodes = set(nodes)  # Nodes to be connected
    nodes_with_outgoing = set()   # Nodes with outgoing edges
    connections = []              # Store connection relationships
    outgoing_edges = {}          # Store outgoing edges for nodes
    incoming_edges = {}          # Store incoming edges for nodes
    
    # Calculate distances between all node pairs
    distances = {}
    for i in range(n):
        distances[nodes[i]] = {}
        for j in range(n):
            if i != j:
                sat1 = get_original_satellite_id(nodes[i], num_satellites)
                sat2 = get_original_satellite_id(nodes[j], num_satellites)
                
                if sat1 == sat2:
                    distances[nodes[i]][nodes[j]] = float('inf')
                    continue
                
                # Calculate distance
                distance = calculate_satellite_distance(
                    sat1,
                    sat2,
                    satellite_locations,
                    current_timestamp,
                    satellite_params
                )
                
                # Check visibility
                height1 = satellite_params[sat1]['height']
                height2 = satellite_params[sat2]['height']
                r1 = R_EARTH + height1
                r2 = R_EARTH + height2
                max_distance = math.sqrt(r1 ** 2 - R_ATMOSPHERE ** 2) + math.sqrt(r2 ** 2 - R_ATMOSPHERE ** 2)
                
                if distance > max_distance:
                    distances[nodes[i]][nodes[j]] = float('inf')
                else:
                    distances[nodes[i]][nodes[j]] = distance
    
    def get_nearest_node(current_node: int, excluded_nodes: Set[int]) -> Tuple[int, float]:
        """Find the nearest available node"""
        nearest_node = None
        min_distance = float('inf')
        for node in remaining_nodes:
            if node != current_node and node not in excluded_nodes:
                distance = distances[current_node][node]
                if distance < min_distance:
                    min_distance = distance
                    nearest_node = node
        return nearest_node, min_distance
    
    # Start with the first node
    current_node = nodes[0]
    
    while len(remaining_nodes) > 0:
        if current_node not in remaining_nodes:
            # If current node has been processed, choose a new starting node
            if remaining_nodes:
                current_node = next(iter(remaining_nodes))
            continue
            
        excluded_nodes = {current_node}
        if current_node in incoming_edges:
            excluded_nodes.add(incoming_edges[current_node])
            
        nearest_node, min_distance = get_nearest_node(current_node, excluded_nodes)
        
        if nearest_node is None or min_distance == float('inf'):
            # No available connection
            break
            
        if nearest_node in nodes_with_outgoing:
            # Target node already has outgoing edge, compare distances
            existing_target = outgoing_edges[nearest_node]
            existing_distance = distances[nearest_node][existing_target]
            
            if distances[nearest_node][current_node] < existing_distance:
                # Break existing connection (current connection is closer)
                connections.remove((nearest_node, existing_target))
                nodes_with_outgoing.remove(nearest_node)
                del outgoing_edges[nearest_node]
                del incoming_edges[existing_target]
                
                # Establish new connection
                connections.append((current_node, nearest_node))
                nodes_with_outgoing.add(current_node)
                outgoing_edges[current_node] = nearest_node
                incoming_edges[nearest_node] = current_node
                
                # Set next node to process
                current_node = existing_target
            else:
                # Find next nearest node
                excluded_nodes.add(nearest_node)
                continue
        else:
            # Establish new connection
            connections.append((current_node, nearest_node))
            nodes_with_outgoing.add(current_node)
            outgoing_edges[current_node] = nearest_node
            incoming_edges[nearest_node] = current_node
            
            # Remove processed node
            remaining_nodes.remove(current_node)
            
            # Set next node to process
            current_node = nearest_node
    
    # Close the ring
    if connections and len(connections) == n - 1:
        last_node = connections[-1][1]
        first_node = connections[0][0]
        if last_node != first_node:
            connections.append((last_node, first_node))
    
    return connections

def process_single_timestamp_topologies(grid_classified_nodes: Dict[int, Set[int]],
                                      satellite_params: Dict,
                                      satellite_locations: Dict,
                                      timestamp: int,
                                      num_satellites: int) -> Dict[int, List[Tuple[int, int]]]:
    """
    Process intra-domain topologies for all grids in a single timestamp.
    
    Args:
        grid_classified_nodes: Dictionary mapping grid IDs to sets of node IDs
        satellite_params: Satellite parameters
        satellite_locations: Satellite locations
        timestamp: Current timestamp
        num_satellites: Total number of satellites
        
    Returns:
        Dictionary mapping grid IDs to lists of connections
    """
    intra_grid_topologies = {}
    total_intra_connections = 0
    
    # Process each grid sequentially
    for grid_id, nodes in grid_classified_nodes.items():
        if len(nodes) > 1:  # Only process grids with multiple nodes
            connections = create_intra_grid_topology(
                nodes,
                satellite_params,
                satellite_locations,
                timestamp,
                num_satellites
            )
            
            if connections:  # Only save results with connections
                intra_grid_topologies[grid_id] = connections
                total_intra_connections += len(connections)   
    
    print(f"Intra-domain topology generation: Processed {len(grid_classified_nodes)} grids, created {total_intra_connections} ring connections")
    return intra_grid_topologies

#--------------------------------------------------
# Core MPC Functions: Initial and Incremental Topology Generation
#--------------------------------------------------

def process_first_timestamp(
    traffic_matrix: np.ndarray,
    grid_satellites: Dict,
    satellite_params: Dict,
    satellite_locations: Dict,
    current_timestamp: int,
    num_satellites: int,
    global_pool: Pool
) -> List[Tuple[Tuple[int, int], Tuple[int, int]]]:
    """
    Process the first timestamp using two-phase matching approach.
    
    This is the initial MPC execution that creates baseline topology.
    
    Args:
        traffic_matrix: Traffic demand matrix
        grid_satellites: Grid to satellites mapping
        satellite_params: Satellite parameters
        satellite_locations: Satellite locations
        current_timestamp: Current timestamp
        num_satellites: Total number of satellites
        global_pool: Process pool for parallel execution
        
    Returns:
        List of topology connections
    """
    global used_virtual_nodes
    used_virtual_nodes = set()  # Reset
    
    # Record total start time
    total_start_time = time.time()
    
    # First phase: Grid matching (parallel)
    num_grids = traffic_matrix.shape[0]
    first_phase_result = [[[[], [], []] for _ in range(num_grids)] for _ in range(num_grids)]
    previous_assignments = {}
    
    # Prepare parallel task parameters
    tasks = []
    for grid_id in range(num_grids):
        if grid_id in grid_satellites[current_timestamp]:  # Ensure grid has covering satellites
            tasks.append((grid_id, grid_satellites, traffic_matrix, satellite_params, 
                         satellite_locations, current_timestamp, num_satellites, 
                         previous_assignments.copy(), used_virtual_nodes.copy()))
    
    # Execute grid matching in parallel - Satellite-to-Grid matching
    results = global_pool.starmap(match_grid_nodes, tasks)
    
    # Integrate results
    for grid_id, assignments in results:
        for neighbor_id, matched_nodes in assignments.items():
            first_phase_result[grid_id][neighbor_id][0].extend(matched_nodes)
            first_phase_result[neighbor_id][grid_id][1].extend(matched_nodes)
            first_phase_result[neighbor_id][grid_id][2].append(traffic_matrix[grid_id][neighbor_id])
            # Update global set of used virtual nodes
            for node in matched_nodes:
                used_virtual_nodes.add(node)
    
    # Second phase: Satellite stable matching
    topology_connections = perform_second_phase_matching(
        first_phase_result,
        satellite_params,
        satellite_locations,
        current_timestamp,
        num_satellites
    )
    
    return topology_connections

def process_timestamp_incremental(
    traffic_matrix: np.ndarray,
    grid_satellites: Dict,
    satellite_params: Dict,
    satellite_locations: Dict,
    current_timestamp: int,
    previous_topology: List[Tuple[Tuple[int, int], Tuple[int, int]]],
    num_satellites: int,
    global_pool: Pool
) -> List[Tuple[Tuple[int, int], Tuple[int, int]]]:
    """
    Process a timestamp incrementally, maintaining valid connections from previous timestamp.
    
    This function implements the incremental topology evolution and repair mentioned in the paper,
    which is key to the MPC approach for maintaining stable topologies despite satellite mobility.
    
    Args:
        traffic_matrix: Traffic demand matrix
        grid_satellites: Grid to satellites mapping
        satellite_params: Satellite parameters
        satellite_locations: Satellite locations
        current_timestamp: Current timestamp
        previous_topology: Topology from previous timestamp
        num_satellites: Total number of satellites
        global_pool: Process pool for parallel execution
        
    Returns:
        List of topology connections for current timestamp
    """
    global used_virtual_nodes
    used_virtual_nodes = set()  # Reset virtual node usage record
    
    if not previous_topology:  # First timestamp
        return process_first_timestamp(
            traffic_matrix,
            grid_satellites,
            satellite_params,
            satellite_locations,
            current_timestamp,
            num_satellites,
            global_pool
        )
    
    # Create a copy of demand matrix and previous_assignments dictionary
    remaining_demands = traffic_matrix.copy()
    maintained_connections = []
    previous_assignments = {}  # Record already assigned connections
    
    # Iterate through all connections from previous timestamp
    for (node1, node2), (prev_grid1, prev_grid2) in previous_topology:
        # Check if satellites are still visible - key to maintaining stable ISLs
        if not check_satellite_visibility(
            node1, node2, satellite_params, satellite_locations, 
            current_timestamp, num_satellites
        ):
            continue
        if node1 in used_virtual_nodes or node2 in used_virtual_nodes:
            continue
        
        # Get grids covered by both nodes in current timestamp
        grids1 = get_grid_coverage(node1, grid_satellites, current_timestamp)
        grids2 = get_grid_coverage(node2, grid_satellites, current_timestamp)
        
        # Check if there are adjacent grids with demand
        maintained = False
        for grid1 in grids1:
            if maintained:
                break
            neighbors1 = get_neighboring_grids(grid1)
            for grid2 in grids2:
                if grid2 in neighbors1 and remaining_demands[grid1][grid2] > 0:
                    # Maintain connection, update demand
                    remaining_demands[grid1][grid2] -= 1
                    remaining_demands[grid2][grid1] -= 1
                    
                    # Add connection with its corresponding grid information
                    connection = ((node1, node2), (grid1, grid2))
                    maintained_connections.append(connection)
                    
                    used_virtual_nodes.add(node1)
                    used_virtual_nodes.add(node2)
                    
                    # Record assignment relationship
                    previous_assignments[(grid1, grid2)] = [node1]
                    previous_assignments[(grid2, grid1)] = [node2]
                    
                    maintained = True
                    break
    
    # First phase: Parallel grid matching for remaining demands
    num_grids = traffic_matrix.shape[0]
    first_phase_result = [[[[], [], []] for _ in range(num_grids)] for _ in range(num_grids)]
    
    # Prepare parallel task parameters
    tasks = []
    for grid_id in range(num_grids):
        if grid_id in grid_satellites[current_timestamp]:  # Ensure grid has covering satellites
            tasks.append((grid_id, grid_satellites, remaining_demands, satellite_params, 
                         satellite_locations, current_timestamp, num_satellites, 
                         previous_assignments.copy(), used_virtual_nodes.copy()))
    
    # Execute grid matching in parallel
    results = global_pool.starmap(match_grid_nodes, tasks)
    
    # Integrate results
    for grid_id, assignments in results:
        for neighbor_id, matched_nodes in assignments.items():
            first_phase_result[grid_id][neighbor_id][0].extend(matched_nodes)
            first_phase_result[neighbor_id][grid_id][1].extend(matched_nodes)
            first_phase_result[neighbor_id][grid_id][2].append(remaining_demands[grid_id][neighbor_id])
            # Update global set of used virtual nodes
            for node in matched_nodes:
                used_virtual_nodes.add(node)
    
    # Second phase: Satellite stable matching
    new_connections = perform_second_phase_matching(
        first_phase_result,
        satellite_params,
        satellite_locations,
        current_timestamp,
        num_satellites
    )
    
    # Merge maintained connections and newly established connections
    final_topology = maintained_connections + new_connections
    
    print(f"Timestamp {current_timestamp}: Maintained {len(maintained_connections)} connections, created {len(new_connections)} new connections")
    
    return final_topology

#--------------------------------------------------
# Model Predictive Controller Main Functions 
#--------------------------------------------------

def generate_topology_for_timestamp(timestamp, satellite_file, block_positions_file, 
                                   traffic_matrix_file, grid_satellites_file, 
                                   output_dir, num_processes):
    """
    Main MPC function: Generate topology for a specified timestamp.
    
    This implements the complete MPC workflow from paper section 4.2,
    combining all three stages of stable matching to generate the 
    satellite network topology that enforces the geographic intent.
    
    Args:
        timestamp: Timestamp index to process
        satellite_file: Path to satellite orbit data (.npy)
        block_positions_file: Path to grid position information (.json)
        traffic_matrix_file: Path to traffic matrix (.npy)
        grid_satellites_file: Path to grid-satellite mapping (.npy)
        output_dir: Output result directory
        num_processes: Number of multi-processes
        
    Returns:
        Generated inter-domain topology (for incremental processing of next timestamp)
    """
    global GLOBAL_INTER_DOMAIN_TOPOLOGY_CACHE, GLOBAL_INTRA_DOMAIN_TOPOLOGY_CACHE
    
    # Record start time
    timestamp_start = time.time()
    
    # create temporary pool
    temp_pool = None
    temp_pool = Pool(processes=num_processes)
    global_pool = temp_pool
    # print(f"Created temporary process pool using {num_processes} processes")
    
    try:
        # Check if timestamp already processed
        if timestamp in GLOBAL_INTER_DOMAIN_TOPOLOGY_CACHE and timestamp in GLOBAL_INTRA_DOMAIN_TOPOLOGY_CACHE:
            print(f"Using cached topology data for timestamp {timestamp}")
            inter_topology = GLOBAL_INTER_DOMAIN_TOPOLOGY_CACHE[timestamp]
            intra_topology = GLOBAL_INTRA_DOMAIN_TOPOLOGY_CACHE[timestamp]
            
            # Data exists, just save and return
            # Load necessary data
            supply_data = np.load(satellite_file, allow_pickle=True)
            num_satellites = len(supply_data)
            
            satellite_params = {}
            satellite_locations = {timestamp: {}}
            for idx, data in enumerate(supply_data):
                param, random_numbers, _, sat_location, _ = data
                satellite_params[idx] = {
                    'height': param[0],
                    'inclination': param[1],
                    'alpha0': param[2],
                    'initial_slot': random_numbers
                }
                satellite_locations[timestamp][idx] = sat_location[timestamp]
            
            with open(block_positions_file, 'r') as f:
                block_positions = json.load(f)
            
            # Save topology
            save_timestamp_topology(
                inter_topology,
                intra_topology,
                block_positions,
                satellite_locations,
                satellite_params,
                num_satellites,
                timestamp,
                output_dir
            )
            
            print(f"Loaded from cache and saved topology for timestamp {timestamp}, total time: {(time.time() - timestamp_start)*1000:.2f}ms")
            return inter_topology
        
        # Check if previous timestamp topology needed
        previous_topology = None
        if timestamp > 0:
            # First check global cache
            if timestamp-1 in GLOBAL_INTER_DOMAIN_TOPOLOGY_CACHE:
                print(f"Getting topology for timestamp {timestamp-1} from cache")
                previous_topology = GLOBAL_INTER_DOMAIN_TOPOLOGY_CACHE[timestamp-1]
            else:
                # Recursively process previous timestamp
                print(f"Topology for timestamp {timestamp-1} not found, will process recursively")
                previous_topology = generate_topology_for_timestamp(
                    timestamp=timestamp-1,
                    satellite_file=satellite_file,
                    block_positions_file=block_positions_file,
                    traffic_matrix_file=traffic_matrix_file,
                    grid_satellites_file=grid_satellites_file,
                    output_dir=output_dir,
                    num_processes=num_processes
                )
        
        # Load necessary data
        data_access_start = time.time()
        
        # Load satellite parameters and positions
        supply_data = np.load(satellite_file, allow_pickle=True)
        num_satellites = len(supply_data)
        
        satellite_params = {}
        satellite_locations = {timestamp: {}}
        for idx, data in enumerate(supply_data):
            param, random_numbers, _, sat_location, _ = data
            satellite_params[idx] = {
                'height': param[0],
                'inclination': param[1],
                'alpha0': param[2],
                'initial_slot': random_numbers
            }
            satellite_locations[timestamp][idx] = sat_location[timestamp]
        
        # Load grid position information
        with open(block_positions_file, 'r') as f:
            block_positions = json.load(f)
        
        # Load traffic matrix
        traffic_matrix = np.load(traffic_matrix_file)
            
        # Load grid coverage data
        grid_data = np.load(grid_satellites_file, allow_pickle=True).item()
        grid_satellites = {timestamp: {}}
        for grid_id, satellites in grid_data[timestamp].items():
            if isinstance(satellites, (list, np.ndarray)):
                grid_satellites[timestamp][grid_id] = list(satellites)
            else:
                grid_satellites[timestamp][grid_id] = [satellites]
        
        data_access_time = (time.time() - data_access_start) * 1000  # milliseconds
        
        # Process inter-domain topology
        inter_domain_start = time.time()
        if timestamp == 0:
            # First timestamp uses non-incremental processing
            inter_topology = process_first_timestamp(
                traffic_matrix,
                grid_satellites,
                satellite_params,
                satellite_locations,
                timestamp,
                num_satellites,
                global_pool
            )
        else:
            # Non-first timestamp uses incremental processing
            inter_topology = process_timestamp_incremental(
                traffic_matrix,
                grid_satellites,
                satellite_params,
                satellite_locations,
                timestamp,
                previous_topology,
                num_satellites,
                global_pool
            )
            
        inter_domain_time = (time.time() - inter_domain_start) * 1000  # milliseconds
        
        # Process intra-domain topology
        intra_domain_start = time.time()
        
        # Satellite classification
        grid_classified_nodes = classify_satellites_to_grids(
            inter_topology,
            grid_satellites,
            timestamp
        )
        
        # Ring topology generation
        intra_topology = process_single_timestamp_topologies(
            grid_classified_nodes,
            satellite_params,
            satellite_locations,
            timestamp,
            num_satellites
        )
        intra_domain_time = (time.time() - intra_domain_start) * 1000  # milliseconds
        
        # Total topology generation time
        total_topology_time = inter_domain_time + intra_domain_time  # milliseconds
        
        # Save topology results to global cache
        GLOBAL_INTER_DOMAIN_TOPOLOGY_CACHE[timestamp] = inter_topology
        GLOBAL_INTRA_DOMAIN_TOPOLOGY_CACHE[timestamp] = intra_topology
        
        # Save topology results to file
        os.makedirs(output_dir, exist_ok=True)
        
        # Save timestamp topology to specified path
        save_timestamp_topology(
            inter_topology,
            intra_topology,
            block_positions,
            satellite_locations,
            satellite_params,
            num_satellites,
            timestamp,
            output_dir
        )
        
        # Print detailed time statistics
        print(f"\nTimestamp {timestamp} topology generation time statistics (ms):")
        print(f"  Data access time: {data_access_time:.2f}ms")
        print(f"  Inter-domain topology generation: {inter_domain_time:.2f}ms ({(inter_domain_time/total_topology_time*100):.1f}%)")
        print(f"  Intra-domain topology generation: {intra_domain_time:.2f}ms ({(intra_domain_time/total_topology_time*100):.1f}%)")
        print(f"  Total topology generation time: {total_topology_time:.2f}ms")
        print(f"  Total timestamp processing time: {((time.time() - timestamp_start) * 1000):.2f}ms")
        
        # Return inter-domain topology for incremental processing of next timestamp
        return inter_topology
        
    finally:
        # If temporary process pool was used, ensure it's closed
        if temp_pool is not None:
            temp_pool.close()
            temp_pool.join()
            print()

def predict_all_topologies(duration, satellite_file, traffic_matrix_file, grid_satellites_file,
                          result_output_dir=None, num_processes=8):
    """
    Predict topologies for all timestamps for the entire simulation time horizon.
    
    This is the main entry point for the TinyLEO toolkit's MPC feature described in section 5.
    It processes all timestamps, generating and optionally saving the full LEO network topology
    evolution over time.
    
    Args:
        duration: Number of timestamps to process
        satellite_file: Path to satellite orbit data (.npy)
        traffic_matrix_file: Path to traffic matrix (.npy)
        grid_satellites_file: Path to grid-satellite mapping (.npy)
        result_output_dir: Output directory for results
        num_processes: Number of multi-processes to use
    """
    # Set default result output directory if not specified
    if result_output_dir is None:
        result_output_dir = "data_plane"
    
    # 1. Configure parameters
    time_stats = {
        'timestamp': [],
        'data_access_time': [],
        'inter_domain_topology_time': [], 
        'intra_domain_topology_time': [],
        'total_topology_time': []
    }
    
    # Create temp process pool
    temp_pool = Pool(processes=num_processes)
    # print(f"Created temp process pool using {num_processes} processes")
    
    try:
        # 2. Get total timestamps and number of satellites
        supply_data = np.load(satellite_file, allow_pickle=True)
        _, _, _, sat_location0, _ = supply_data[0]
        total_timestamps = len(sat_location0)
        num_satellites = len(supply_data)
        
        print(f"Total timestamps: {total_timestamps}")
        print(f"Number of satellites: {num_satellites}")
        
        # Load all timestamp data at once outside the loop
        print("\nLoading all timestamp data at once...")
        all_traffic_matrices, all_grid_satellites, all_satellite_locations, satellite_params = \
            load_all_timestamps_data(
                traffic_matrix_file,
                satellite_file,
                grid_satellites_file,
                total_timestamps
            )
        
        # Store all results
        all_inter_topology = {}
        all_intra_topology = {}
        previous_topology = None
        
        # Process each timestamp
        print("\nProcessing all timestamps...")
        overall_start_time = time.time()
        
        for timestamp in tqdm(range(total_timestamps)[:duration], desc="Processing timestamps"):
            # Record timestamp start time
            timestamp_start = time.time()
            
            # Get data for current timestamp from preloaded data
            data_access_start = time.time()
            traffic_matrix = all_traffic_matrices[timestamp]
            
            # Construct temporary nested dictionaries to be compatible with existing function interfaces
            grid_satellites_temp = {timestamp: all_grid_satellites[timestamp]}
            satellite_locations_temp = {timestamp: all_satellite_locations[timestamp]}
            data_access_time = (time.time() - data_access_start) * 1000  # milliseconds
                
            # Process inter-domain topology
            inter_domain_start = time.time()
            if timestamp == 0:
                topology_result = process_first_timestamp(
                    traffic_matrix,
                    grid_satellites_temp,
                    satellite_params,
                    satellite_locations_temp,
                    timestamp,
                    num_satellites,
                    temp_pool
                )
            else:
                topology_result = process_timestamp_incremental(
                    traffic_matrix,
                    grid_satellites_temp,
                    satellite_params,
                    satellite_locations_temp,
                    timestamp,
                    previous_topology,
                    num_satellites,
                    temp_pool
                )
            inter_domain_time = (time.time() - inter_domain_start) * 1000  # milliseconds
                
            # Process intra-domain topology
            intra_domain_start = time.time()
            
            # Satellite classification
            grid_classified_nodes = classify_satellites_to_grids(
                topology_result,
                grid_satellites_temp,
                timestamp
            )
            
            # Ring topology generation
            intra_topology = process_single_timestamp_topologies(
                grid_classified_nodes,
                satellite_params,
                satellite_locations_temp,
                timestamp,
                num_satellites
            )
            intra_domain_time = (time.time() - intra_domain_start) * 1000  # milliseconds
            
            # Total topology generation time
            total_topology_time = inter_domain_time + intra_domain_time  # milliseconds
            
            # Store results
            all_inter_topology[timestamp] = topology_result
            all_intra_topology[timestamp] = intra_topology
            previous_topology = topology_result
            
            # Record time statistics
            time_stats['timestamp'].append(timestamp)
            time_stats['data_access_time'].append(data_access_time)
            time_stats['inter_domain_topology_time'].append(inter_domain_time)
            time_stats['intra_domain_topology_time'].append(intra_domain_time)
            time_stats['total_topology_time'].append(total_topology_time)
            
            # Clean up memory
            del traffic_matrix, grid_satellites_temp, satellite_locations_temp
                
        # 4. Calculate total processing time
        total_processing_time = time.time() - overall_start_time
        print(f"\nTotal processing time: {total_processing_time:.2f}s\n")
        
        # 5. Save results and create consolidated output files
        # Create output directory if it doesn't exist
        os.makedirs(result_output_dir, exist_ok=True)
        
        create_all_isl_position_json(
            supply_data,
            all_inter_topology,
            all_intra_topology,
            result_output_dir
        )
        
    finally:
        # Ensure process pool is properly closed
        temp_pool.close()
        temp_pool.join()
        # print("Temp process pool closed")
