"""
Utility functions for TinyLEO satellite network.
These are helper functions for orbital mechanics, visibility calculations,
data processing, and other general utilities.
"""

import numpy as np
import math
import os
import json
import time
from collections import defaultdict
from typing import Dict, List, Tuple, Set, Optional

# Global constants
R_EARTH = 6371  # Earth radius (km)
ATMOSPHERE_HEIGHT = 80  # Atmosphere height (km)
R_ATMOSPHERE = R_EARTH + ATMOSPHERE_HEIGHT

# Basic utility functions
def get_original_satellite_id(virtual_id: int, num_satellites: int) -> int:
    """
    Get the original satellite ID from a virtual node ID.
    
    Args:
        virtual_id: Virtual node ID
        num_satellites: Total number of satellites
        
    Returns:
        Original satellite ID
    """
    return virtual_id % num_satellites

def get_neighboring_grids(grid_id: int, grid_rows: int = 11, grid_cols: int = 11) -> list[int]:
    """
    Get neighboring grid IDs for a given grid, considering spherical connectivity.
    
    Args:
        grid_id: The grid ID to find neighbors for
        grid_rows: Number of grid rows (default: 11)
        grid_cols: Number of grid columns (default: 11)
        
    Returns:
        List of neighboring grid IDs
    """
    if grid_id >= grid_rows * grid_cols:
        print(f"Warning: Invalid grid_id {grid_id} for {grid_rows}x{grid_cols} grid")
        return []
        
    neighbors = []
    row = grid_id // grid_cols
    col = grid_id % grid_cols
    
    # Upper grid
    up_row = (row - 1) if row > 0 else (grid_rows - 1)
    up_grid = up_row * grid_cols + col
    if up_grid < grid_rows * grid_cols:
        neighbors.append(up_grid)
    
    # Lower grid
    down_row = (row + 1) if row < grid_rows - 1 else 0
    down_grid = down_row * grid_cols + col
    if down_grid < grid_rows * grid_cols:
        neighbors.append(down_grid)
    
    # Left grid
    left_col = (col - 1) if col > 0 else (grid_cols - 1)
    left_grid = row * grid_cols + left_col
    if left_grid < grid_rows * grid_cols:
        neighbors.append(left_grid)
    
    # Right grid
    right_col = (col + 1) if col < grid_cols - 1 else 0
    right_grid = row * grid_cols + right_col
    if right_grid < grid_rows * grid_cols:
        neighbors.append(right_grid)
    
    return list(set(neighbors))  # Remove duplicates

# Data loading functions
def load_all_timestamps_data(traffic_matrix_file: str, 
                           satellite_data_file: str,
                           grid_satellite_file: str,
                           total_timestamps: int) -> tuple:
    """
    Load all timestamps data at once for the entire simulation.
    
    Args:
        traffic_matrix_file: Path to traffic matrix file
        satellite_data_file: Path to satellite data file
        grid_satellite_file: Path to grid-satellite mapping file
        total_timestamps: Total number of timestamps to load
        
    Returns:
        Tuple of (traffic_matrices, grid_satellites, satellite_locations, satellite_params)
    """
    # Record loading start time
    load_start_time = time.time()
    
    # Load satellite parameters data (static across timestamps)
    supply_data = np.load(satellite_data_file, allow_pickle=True)
    num_satellites = len(supply_data)
    
    # Initialize data structures
    satellite_locations = {t: {} for t in range(total_timestamps)}
    satellite_params = {}  # Satellite parameters don't change over time
    
    print(f"Loading satellite parameters and locations for {num_satellites} satellites...")
    # Process satellite parameters and positions
    for idx, data in enumerate(supply_data):
        param, random_numbers, _, sat_location, _ = data
        
        # Satellite parameters don't change over time
        satellite_params[idx] = {
            'height': param[0],
            'inclination': param[1],
            'alpha0': param[2],
            'initial_slot': random_numbers
        }
        
        # Store positions for all timestamps
        for t in range(total_timestamps):
            satellite_locations[t][idx] = sat_location[t]
    
    # Load grid coverage data
    print("Loading grid satellite coverage data...")
    grid_satellites = np.load(grid_satellite_file, allow_pickle=True).item()
    
    # Load traffic matrix (assumed same for all timestamps)
    print("Loading traffic matrix...")
    traffic_matrix = np.load(traffic_matrix_file)
    
    # Create processed traffic matrices dictionary
    processed_traffic_matrices = {}
    for t in range(total_timestamps):
        processed_traffic_matrices[t] = traffic_matrix.copy()
    
    # Calculate total loading time
    total_load_time = time.time() - load_start_time
    print(f"All data loaded in {total_load_time:.2f} seconds")
    
    return processed_traffic_matrices, grid_satellites, satellite_locations, satellite_params

def load_single_timestamp_data(traffic_matrix_file: str, 
                             satellite_data_file: str,
                             grid_satellite_file: str,
                             timestamp: int) -> tuple:
    """
    Load data for a single timestamp to reduce memory usage.
    
    Args:
        traffic_matrix_file: Path to traffic matrix file
        satellite_data_file: Path to satellite data file
        grid_satellite_file: Path to grid-satellite mapping file
        timestamp: Timestamp to load data for
        
    Returns:
        Tuple of (traffic_matrix, grid_satellites, satellite_locations, satellite_params)
    """
    # Load fixed parameter data
    supply_data = np.load(satellite_data_file, allow_pickle=True)
    num_satellites = len(supply_data)
    
    # Only load satellite positions for the specified timestamp
    satellite_locations = {timestamp: {}}
    satellite_params = {}
    
    for idx, data in enumerate(supply_data):
        param, random_numbers, _, sat_location, _ = data
        
        satellite_params[idx] = {
            'height': param[0],
            'inclination': param[1],
            'alpha0': param[2],
            'initial_slot': random_numbers
        }
        
        # Only load positions for current timestamp
        satellite_locations[timestamp][idx] = sat_location[timestamp]
    
    # Load grid coverage data, ensure correct data structure
    grid_data = np.load(grid_satellite_file, allow_pickle=True).item()
    grid_satellites = {timestamp: {}}
    for grid_id, satellites in grid_data[timestamp].items():
        if isinstance(satellites, (list, np.ndarray)):  # Ensure list format
            grid_satellites[timestamp][grid_id] = list(satellites)
        else:
            grid_satellites[timestamp][grid_id] = [satellites]
    
    # Load traffic matrix
    traffic_matrix = np.load(traffic_matrix_file)
    processed_traffic_matrices = {timestamp: traffic_matrix}
    
    return processed_traffic_matrices[timestamp], grid_satellites, satellite_locations, satellite_params

# Geometric and visibility calculation functions
def geodetic_to_cartesian(lat: float, lon: float, height: float) -> Tuple[float, float, float]:
    """
    Convert geodetic coordinates to Cartesian coordinates.
    
    Args:
        lat: Latitude in radians
        lon: Longitude in radians
        height: Height above Earth's surface in km
        
    Returns:
        Tuple of (x, y, z) coordinates in km
    """
    r = R_EARTH + height
    x = r * math.cos(lat) * math.cos(lon)
    y = r * math.cos(lat) * math.sin(lon)
    z = r * math.sin(lat)
    return x, y, z

def calculate_distance(pos1: Tuple[float, float, float], 
                      pos2: Tuple[float, float, float]) -> float:
    """
    Calculate Euclidean distance between two 3D points.
    
    Args:
        pos1: First position (x, y, z)
        pos2: Second position (x, y, z)
        
    Returns:
        Distance between points in km
    """
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(pos1, pos2)))

def calculate_satellite_distance(sat1_id: int, sat2_id: int,
                               satellite_locations: Dict,
                               current_timestamp: int,
                               satellite_params: Dict) -> float:
    """
    Calculate distance between two satellites.
    
    Args:
        sat1_id: First satellite ID
        sat2_id: Second satellite ID
        satellite_locations: Dictionary of satellite locations
        current_timestamp: Current timestamp
        satellite_params: Dictionary of satellite parameters
        
    Returns:
        Distance between satellites in km
    """
    # Get satellite positions and heights
    lon1, lat1 = satellite_locations[current_timestamp][sat1_id]
    lon2, lat2 = satellite_locations[current_timestamp][sat2_id]
    height1 = satellite_params[sat1_id]['height']
    height2 = satellite_params[sat2_id]['height']
    
    # Convert to Cartesian coordinates
    pos1 = geodetic_to_cartesian(lat1, lon1, height1)
    pos2 = geodetic_to_cartesian(lat2, lon2, height2)
    
    # Calculate Euclidean distance
    return calculate_distance(pos1, pos2)

def check_satellite_visibility(node1: int, 
                             node2: int,
                             satellite_params: Dict,
                             satellite_locations: Dict,
                             current_timestamp: int,
                             num_satellites: int) -> bool:
    """
    Check if two virtual nodes' corresponding satellites have line-of-sight visibility.
    
    Args:
        node1: First virtual node ID
        node2: Second virtual node ID
        satellite_params: Dictionary of satellite parameters
        satellite_locations: Dictionary of satellite locations
        current_timestamp: Current timestamp
        num_satellites: Total number of satellites
        
    Returns:
        True if satellites are visible to each other, False otherwise
    """
    # Get actual satellite IDs
    sat1_id = get_original_satellite_id(node1, num_satellites)
    sat2_id = get_original_satellite_id(node2, num_satellites)
    
    # Get satellite parameters
    sat1_data = satellite_params[sat1_id]
    sat2_data = satellite_params[sat2_id]
    height1 = sat1_data['height']
    height2 = sat2_data['height']
    
    # Calculate maximum visible distance
    r1 = R_EARTH + height1
    r2 = R_EARTH + height2
    max_distance = math.sqrt(r1 ** 2 - R_ATMOSPHERE ** 2) + math.sqrt(r2 ** 2 - R_ATMOSPHERE ** 2)
    
    # Check current timestamp visibility
    lon1, lat1 = satellite_locations[current_timestamp][sat1_id]
    lon2, lat2 = satellite_locations[current_timestamp][sat2_id]
    
    pos1 = geodetic_to_cartesian(lat1, lon1, height1)
    pos2 = geodetic_to_cartesian(lat2, lon2, height2)
    
    # Calculate distance and check visibility
    distance = calculate_distance(pos1, pos2)
    return distance <= max_distance

# Satellite orbit related functions
def get_orbit_key(sat_params: Dict) -> Tuple[float, float, float]:
    """
    Generate unique key for satellite orbit based on parameters.
    
    Args:
        sat_params: Dictionary of satellite parameters
        
    Returns:
        Tuple of (height, inclination, alpha0) that uniquely identifies the orbit
    """
    return (sat_params['height'], sat_params['inclination'], sat_params['alpha0'])

def is_same_orbit(sat1_params: Dict, sat2_params: Dict) -> bool:
    """
    Determine if two satellites are in the same orbit.
    
    Args:
        sat1_params: Parameters of first satellite
        sat2_params: Parameters of second satellite
        
    Returns:
        True if satellites are in the same orbit, False otherwise
    """
    return get_orbit_key(sat1_params) == get_orbit_key(sat2_params)

def get_grid_coverage(node_id: int, 
                    grid_satellites: Dict, 
                    current_timestamp: int) -> Set[int]:
    """
    Get the set of grids covered by a node at current timestamp.
    
    Args:
        node_id: Virtual node ID
        grid_satellites: Grid to satellites mapping
        current_timestamp: Current timestamp
        
    Returns:
        Set of grid IDs covered by the node
    """
    covered_grids = set()
    for grid_id, nodes in grid_satellites[current_timestamp].items():
        if node_id in nodes:
            covered_grids.add(grid_id)
    return covered_grids

# Results saving functions
def get_satellite_name(satellite_id):
    """
    Generate satellite name from satellite ID.
    
    Args:
        satellite_id: Satellite ID
        
    Returns:
        Formatted satellite name
    """
    return f'SH1SAT{satellite_id+1}'

def save_satellite_cell_assignments(inter_topology, block_positions, timestamp, output_dir):
    """
    Save satellite to cell assignments.
    
    Args:
        inter_topology: Inter-domain topology
        block_positions: Grid positions
        timestamp: Current timestamp
        output_dir: Output directory
    """
    sat_cell_map = {}
    for (node1, node2), (grid1, grid2) in inter_topology:
        # Get actual satellite IDs
        sat1 = get_satellite_name(node1)
        sat2 = get_satellite_name(node2)
        sat_cell_map[sat1] = block_positions[str(grid1)]['row_col']
        sat_cell_map[sat2] = block_positions[str(grid2)]['row_col']
    
    os.makedirs(os.path.join(output_dir, 'sat_cells'), exist_ok=True)
    with open(os.path.join(output_dir, 'sat_cells', f"{timestamp}.json"), 'w') as f:
        json.dump(sat_cell_map, f, indent=2)

def save_inter_cell_connections(inter_topology, block_positions, timestamp, output_dir):
    """
    Save inter-cell ISL information.
    
    Args:
        inter_topology: Inter-domain topology
        block_positions: Grid positions
        timestamp: Current timestamp
        output_dir: Output directory
    """
    inter_isl_map = {}
    for (node1, node2), (grid1, grid2) in inter_topology:
        sat1 = get_satellite_name(node1)
        sat2 = get_satellite_name(node2)
        if sat1 not in inter_isl_map:
            inter_isl_map[sat1] = {
                "near_sat": sat2,
                "near_cell": block_positions[str(grid2)]['row_col']
            }
        if sat2 not in inter_isl_map:
            inter_isl_map[sat2] = {
                "near_sat": sat1,
                "near_cell": block_positions[str(grid1)]['row_col']
            }
    
    os.makedirs(os.path.join(output_dir, 'inter_cell_isls'), exist_ok=True)
    with open(os.path.join(output_dir, 'inter_cell_isls', f"{timestamp}.json"), 'w') as f:
        json.dump(inter_isl_map, f, indent=2)

def save_isl_positions(inter_topology, intra_topology, sat_locations, sat_params, timestamp, num_satellites, output_dir):
    """
    Save ISL position information.
    
    Args:
        inter_topology: Inter-domain topology
        intra_topology: Intra-domain topology
        sat_locations: Satellite locations
        sat_params: Satellite parameters
        timestamp: Current timestamp
        num_satellites: Total number of satellites
        output_dir: Output directory
    """
    timeslot = {
        "position": [],
        "links": []
    }

    # Add satellite positions
    for sat_id in range(num_satellites):
        lon, lat = sat_locations[timestamp][sat_id]
        height = sat_params[sat_id]['height']
        
        # Convert radians to degrees
        lat_deg = math.degrees(lat)
        lon_deg = math.degrees(lon)
        
        timeslot["position"].append({
            "latitude": float(lat_deg),
            "longitude": float(lon_deg),
            "altitude": float(height)
        })
    
    # Add links (inter-domain)
    used_links = set()
    for (node1, node2), _ in inter_topology:
        sat1 = get_original_satellite_id(node1, num_satellites)
        sat2 = get_original_satellite_id(node2, num_satellites)
        link_key = tuple(sorted([sat1, sat2]))
        
        if link_key not in used_links:
            timeslot["links"].append({
                "sat1": link_key[0],
                "sat2": link_key[1]
            })
            used_links.add(link_key)
    
    # Add links (intra-domain)
    for grid_connections in intra_topology.values():
        for node1, node2 in grid_connections:
            sat1 = get_original_satellite_id(node1, num_satellites)
            sat2 = get_original_satellite_id(node2, num_satellites)
            link_key = tuple(sorted([sat1, sat2]))
            
            if link_key not in used_links:
                timeslot["links"].append({
                    "sat1": link_key[0],
                    "sat2": link_key[1]
                })
                used_links.add(link_key)
    
    os.makedirs(os.path.join(output_dir, 'all_isl_positions'), exist_ok=True)
    with open(os.path.join(output_dir, 'all_isl_positions', f"{timestamp}.json"), 'w') as f:
        json.dump(timeslot, f, indent=2)

def save_timestamp_topology(inter_topology, intra_topology, block_positions, sat_locations, sat_params, num_satellites, timestamp, output_dir):
    """
    Save timestamp topology to specified path.
    
    Args:
        inter_topology: Inter-domain topology
        intra_topology: Intra-domain topology
        block_positions: Grid positions
        sat_locations: Satellite locations
        sat_params: Satellite parameters
        num_satellites: Total number of satellites
        timestamp: Current timestamp
        output_dir: Output directory
    """
    os.makedirs(os.path.join(output_dir, 'inter_topology'), exist_ok=True)
    np.save(os.path.join(output_dir, 'inter_topology',f"{timestamp}.npy"), inter_topology)
    os.makedirs(os.path.join(output_dir, 'intra_topology'), exist_ok=True)
    np.save(os.path.join(output_dir, 'intra_topology',f"{timestamp}.npy"), intra_topology)
    # Save satellite cell assignments
    save_satellite_cell_assignments(inter_topology, block_positions, timestamp, output_dir)
    # Save inter-cell ISLs
    save_inter_cell_connections(inter_topology, block_positions, timestamp, output_dir)
    # Save satellite topology
    save_isl_positions(inter_topology, intra_topology, sat_locations, sat_params, timestamp, num_satellites, output_dir)

def create_all_isl_position_json(supply_data, inter_topology, intra_topology, output_dir):
    """
    Create consolidated ISL position JSON file for all timestamps.
    
    Args:
        supply_data: Satellite orbit data
        inter_topology: Inter-domain topology for all timestamps
        intra_topology: Intra-domain topology for all timestamps
        output_dir: Output directory
    """
    # Initialize result
    result = {"timeslots": []}
    
    # Get number of timestamps and satellites
    _, _, _, sat_location0, _ = supply_data[0]
    num_timestamps = len(sat_location0)
    num_satellites = len(supply_data)
    
    # Build satellite position and parameter information
    satellite_locations = {t: {} for t in range(num_timestamps)}
    satellite_params = {}
    
    # Process data for each satellite
    for idx, data in enumerate(supply_data):
        param, _, _, sat_location, _ = data
        
        # Store satellite parameters
        satellite_params[idx] = {
            'height': param[0],
            'inclination': param[1],
            'alpha0': param[2]
        }
        
        # Store position information
        for t in range(num_timestamps):
            satellite_locations[t][idx] = sat_location[t]
    
    # Process each timestamp
    for t in range(len(inter_topology)):
        timeslot = {
            "position": [],
            "links": []
        }
        
        # Add satellite positions (in satellite ID order)
        for sat_id in range(num_satellites):
            lon, lat = satellite_locations[t][sat_id]
            height = satellite_params[sat_id]['height']
            
            # Convert radians to degrees
            lat_deg = math.degrees(lat)
            lon_deg = math.degrees(lon)
            
            timeslot["position"].append({
                "latitude": float(lat_deg),
                "longitude": float(lon_deg),
                "altitude": float(height)
            })
        
        # Add links (inter-domain)
        used_links = set()
        for (node1, node2), _ in inter_topology[t]:
            sat1 = get_original_satellite_id(node1, num_satellites)
            sat2 = get_original_satellite_id(node2, num_satellites)
            link_key = tuple(sorted([sat1, sat2]))
            
            if link_key not in used_links:
                timeslot["links"].append({
                    "sat1": link_key[0],
                    "sat2": link_key[1]
                })
                used_links.add(link_key)
        
        # Add links (intra-domain)
        if t in intra_topology:
            for grid_connections in intra_topology[t].values():
                for node1, node2 in grid_connections:
                    sat1 = get_original_satellite_id(node1, num_satellites)
                    sat2 = get_original_satellite_id(node2, num_satellites)
                    link_key = tuple(sorted([sat1, sat2]))
                    
                    if link_key not in used_links:
                        timeslot["links"].append({
                            "sat1": link_key[0],
                            "sat2": link_key[1]
                        })
                        used_links.add(link_key)
        
        result["timeslots"].append(timeslot)
    
    # Save as JSON file
    with open(f'{output_dir}/predict_isl_position_all.json', 'w') as f:
        json.dump(result, f, indent=2)
