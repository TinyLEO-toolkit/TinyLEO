"""
TinyLEO Northbound API Module

This module provides a high-level interface for geographic traffic engineering
in low Earth orbit satellite networks. It processes routing configurations and
generates traffic matrices based on demand specifications.
"""

import json
import numpy as np
import networkx as nx
import heapq
import math
import os
import copy
from collections import defaultdict
from typing import Dict, List, Tuple, Set, Optional, Any


class TinyLEONorthboundAPI:
    """
    TinyLEO Northbound API: Process geographic topology and routing configuration,
    and generate traffic matrices.
    
    This class provides methods to configure, analyze, and optimize traffic flow
    in a satellite network based on geographic grid model.
    """
    
    def __init__(self, config_file: str):
        """
        Initialize the API and load configuration file.
        
        Args:
            config_file: Path to the MPC configuration file in JSON format
        """
        # Validate config file exists
        if not os.path.exists(config_file):
            raise FileNotFoundError(f"Configuration file not found: {config_file}")
            
        self.config_file = config_file
        self.config = None
        self.grid_density = defaultdict(lambda: 1)  # Default density of 1
        self.grid_rows = 11
        self.grid_cols = 11
        self.traffic_matrix = None
        self.isl_matrix = None
        self.scaled_traffic_matrix = None
        self.scaled_isl_matrix = None
        self.scaling_factor = None
        self.paths = {}
        self._load_config()
    
    def _load_config(self) -> None:
        """
        Load and validate the MPC configuration file.
        
        Raises:
            Exception: If the configuration file cannot be loaded or is invalid
        """
        try:
            with open(self.config_file, 'r', encoding='utf-8') as f:
                self.config = json.load(f)
            
            # Get grid configuration
            self.grid_rows = self.config['grid_config'].get('rows', 11)
            self.grid_cols = self.config['grid_config'].get('cols', 11)
            
            print(f"Successfully loaded configuration: {self.config_file}")
        except Exception as e:
            print(f"Error loading configuration file: {e}")
            raise
    
    def _load_grid_satellite_density(self, grid_satellites_file: str) -> dict:
        """
        Internal method to load satellite density data if needed.
        
        Args:
            grid_satellites_file: Path to the grid satellite data file (.npy format)
            
        Returns:
            dict: A dictionary mapping grid ID to the number of satellites covering it
            
        Raises:
            FileNotFoundError: If the satellite density file does not exist
        """
        # Validate file exists
        if not os.path.exists(grid_satellites_file):
            raise FileNotFoundError(f"Grid satellite file not found: {grid_satellites_file}")
            
        try:
            coverage_matrices = np.load(grid_satellites_file, allow_pickle=True).item()
            # Use the first timestamp's data
            first_timestamp = list(coverage_matrices.keys())[0]
            coverage = coverage_matrices[first_timestamp]
            
            grid_density = {}
            for grid_id in range(len(coverage)):
                grid_density[grid_id] = len(coverage[grid_id])
            
            # Update class attribute
            self.grid_density = grid_density
            
            return grid_density
        except Exception as e:
            print(f"Error loading satellite density data: {e}")
            # If file loading fails, return default density
            self.grid_density = defaultdict(lambda: 1)
            return self.grid_density
    
    def get_neighbors(self, grid_id: int) -> list:
        """
        Get the adjacent grid IDs for the given grid, including connections across the 180° meridian.
        
        Args:
            grid_id: The grid ID to find neighbors for
            
        Returns:
            list: A list of neighboring grid IDs
        """
        neighbors = []
        row = grid_id // self.grid_cols
        col = grid_id % self.grid_cols
        
        # Upper grid
        if row > 0:
            neighbors.append((row - 1) * self.grid_cols + col)
        # Lower grid
        if row < self.grid_rows - 1:
            neighbors.append((row + 1) * self.grid_cols + col)
        # Left grid
        if col > 0:
            neighbors.append(row * self.grid_cols + (col - 1))
        else:
            # If leftmost, connect to rightmost (wrap around)
            neighbors.append(row * self.grid_cols + (self.grid_cols - 1))
        # Right grid
        if col < self.grid_cols - 1:
            neighbors.append(row * self.grid_cols + (col + 1))
        else:
            # If rightmost, connect to leftmost (wrap around)
            neighbors.append(row * self.grid_cols)
        
        return neighbors
    
    def calculate_edge_weight(self, grid1: int, grid2: int, policy_params: Dict = None) -> float:
        """
        Calculate the edge weight between two adjacent grids, adjusted based on routing policy.
        
        Args:
            grid1: First grid ID
            grid2: Second grid ID
            policy_params: Optional routing policy parameters
            
        Returns:
            float: The calculated edge weight
        """
        row1, col1 = grid1 // self.grid_cols, grid1 % self.grid_cols
        row2, col2 = grid2 // self.grid_cols, grid2 % self.grid_cols
        
        # Base settings
        base_weight = 1.0
        # Use a fixed density factor 
        density_factor = 0.5
        
        # Get average satellite density between the two grids
        avg_density = (self.grid_density.get(grid1, 0) + self.grid_density.get(grid2, 0)) / 2
        max_density = max(self.grid_density.values()) if self.grid_density else 1
        
        # Adjust weight based on density
        density_weight = 1 - (avg_density / max_density) * density_factor
        
        # Calculate base weight (considering wrapping across 180° meridian)
        if (col1 == 0 and col2 == self.grid_cols-1) or (col1 == self.grid_cols-1 and col2 == 0):
            # Connection across 180° meridian, weight same as regular adjacent grid
            base_distance = 1.0
        else:
            base_distance = base_weight
            
        # Apply routing policy adjustments
        if policy_params:
            if 'preferred_cells' in policy_params:
                # If it's a preferred area, reduce weight
                if grid1 in policy_params['preferred_cells'] or grid2 in policy_params['preferred_cells']:
                    return base_distance * policy_params.get('preference_weight', 0.5)
                    
            if 'avoid_cells' in policy_params:
                # If it's an area to avoid, increase weight significantly
                if grid1 in policy_params['avoid_cells'] or grid2 in policy_params['avoid_cells']:
                    return base_distance * 10.0  # Significantly increase weight to avoid this area
        
        return base_distance * (1 + density_weight)
    
    def find_path(self, start: int, end: int, demand: Dict) -> list:
        """
        Find path from start to end based on the demand's routing policy.
        
        Args:
            start: Source grid ID
            end: Destination grid ID
            demand: Demand dictionary containing routing policy
            
        Returns:
            list: Ordered list of grid IDs representing the path
        """
        routing_policy = demand.get('routing_policy', 'shortest_path')
        
        if routing_policy not in ['shortest_path', 'oceanic_offload', 'geo_avoid', 'multipath']:
            routing_policy = 'shortest_path'  # Default to shortest path
        
        # Call appropriate path finding method based on policy
        if routing_policy == 'shortest_path':
            return self.find_shortest_path(start, end)
        elif routing_policy == 'oceanic_offload':
            return self.find_oceanic_path(start, end, demand)
        elif routing_policy == 'geo_avoid':
            return self.find_avoiding_path(start, end, demand)
        elif routing_policy == 'multipath':
            # For multipath, just return the shortest path for now
            # In a future implementation, this would generate multiple paths
            return self.find_shortest_path(start, end)
    
    def find_shortest_path(self, start: int, end: int) -> list:
        """
        Find the shortest path using a modified Dijkstra's algorithm.
        
        Args:
            start: Source grid ID
            end: Destination grid ID
            
        Returns:
            list: Ordered list of grid IDs representing the shortest path
        """
        num_grids = self.grid_rows * self.grid_cols
        distances = {i: float('inf') for i in range(num_grids)}
        distances[start] = 0
        pq = [(0, start)]
        previous = {start: None}
        
        while pq:
            current_distance, current = heapq.heappop(pq)
            
            if current == end:
                break
                
            if current_distance > distances[current]:
                continue
                
            for neighbor in self.get_neighbors(current):
                edge_weight = self.calculate_edge_weight(current, neighbor)
                distance = current_distance + edge_weight
                
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous[neighbor] = current
                    heapq.heappush(pq, (distance, neighbor))
        
        # Build path
        path = []
        current = end
        while current is not None:
            path.append(current)
            current = previous.get(current)
        
        if not path or path[-1] != start:
            return []  # No path found
            
        return path[::-1]
    
    def find_oceanic_path(self, start: int, end: int, demand: Dict) -> list:
        """
        Find a path that preferentially routes through oceanic areas.
        
        Args:
            start: Source grid ID
            end: Destination grid ID
            demand: Demand dictionary containing oceanic routing parameters
            
        Returns:
            list: Ordered list of grid IDs representing the path
        """
        num_grids = self.grid_rows * self.grid_cols
        distances = {i: float('inf') for i in range(num_grids)}
        distances[start] = 0
        pq = [(0, start)]
        previous = {start: None}
        
        while pq:
            current_distance, current = heapq.heappop(pq)
            
            if current == end:
                break
                
            if current_distance > distances[current]:
                continue
                
            for neighbor in self.get_neighbors(current):
                # Use weight calculation specific to oceanic_offload policy
                edge_weight = self.calculate_edge_weight(current, neighbor, demand)
                distance = current_distance + edge_weight
                
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous[neighbor] = current
                    heapq.heappush(pq, (distance, neighbor))
        
        # Build path
        path = []
        current = end
        while current is not None:
            path.append(current)
            current = previous.get(current)
        
        if not path or path[-1] != start:
            # If oceanic path not found, fall back to shortest path
            return self.find_shortest_path(start, end)
            
        return path[::-1]
    
    def find_avoiding_path(self, start: int, end: int, demand: Dict) -> list:
        """
        Find a path that avoids specific areas.
        
        Args:
            start: Source grid ID
            end: Destination grid ID
            demand: Demand dictionary containing areas to avoid
            
        Returns:
            list: Ordered list of grid IDs representing the path
        """
        avoid_cells = demand.get('avoid_cells', [])
        
        # If start or end is in the avoid list, we can't avoid
        if start in avoid_cells or end in avoid_cells:
            return self.find_shortest_path(start, end)
            
        num_grids = self.grid_rows * self.grid_cols
        distances = {i: float('inf') for i in range(num_grids)}
        distances[start] = 0
        pq = [(0, start)]
        previous = {start: None}
        
        while pq:
            current_distance, current = heapq.heappop(pq)
            
            if current == end:
                break
                
            if current_distance > distances[current]:
                continue
                
            for neighbor in self.get_neighbors(current):
                # Skip areas to avoid
                if neighbor in avoid_cells:
                    continue
                    
                edge_weight = self.calculate_edge_weight(current, neighbor)
                distance = current_distance + edge_weight
                
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous[neighbor] = current
                    heapq.heappush(pq, (distance, neighbor))
        
        # Build path
        path = []
        current = end
        while current is not None:
            path.append(current)
            current = previous.get(current)
        
        if not path or path[-1] != start:
            # If avoiding path not found, fall back to shortest path
            return self.find_shortest_path(start, end)
            
        return path[::-1]
    
    def generate_traffic_matrix(self, grid_satellites_file: str = None) -> np.ndarray:
        """
        Generate traffic matrix based on all traffic demands in the configuration.
        
        Args:
            grid_satellites_file: Optional path to grid satellite coverage file for density calculation
            
        Returns:
            np.ndarray: The generated traffic matrix
        """
        # If satellite density file is provided, load it
        if grid_satellites_file:
            try:
                self._load_grid_satellite_density(grid_satellites_file)
            except FileNotFoundError as e:
                print(f"Warning: {e}. Using default density values.")
            
        num_grids = self.grid_rows * self.grid_cols
        traffic_matrix = np.zeros((num_grids, num_grids), dtype=float)
        
        # Process all demands
        for demand in self.config['traffic_demands']:
            source = demand['source']
            destination = demand['destination']
            flow_gbps = demand['demand_gbps']
            
            # Find path
            path = self.find_path(source, destination, demand)
            if not path:
                print(f"Warning: No path found from {source} to {destination}, skipping this demand")
                continue
                
            # Save path for future reference
            key = (source, destination)
            self.paths[key] = path
            
            # Allocate traffic to the path
            for i in range(len(path) - 1):
                grid1, grid2 = path[i], path[i+1]
                traffic_matrix[grid1][grid2] += flow_gbps
                traffic_matrix[grid2][grid1] += flow_gbps  # Bidirectional traffic
        
        self.traffic_matrix = traffic_matrix
        return traffic_matrix
    
    def calculate_isl_requirements(self) -> np.ndarray:
        """
        Calculate the number of ISLs required between each pair of grids.
        
        Returns:
            np.ndarray: Matrix of ISL requirements
        """
        if self.traffic_matrix is None:
            self.generate_traffic_matrix()
        
        # Get ISL capacity
        isl_capacity = self.config['global_settings']['isl_capacity_gbps']
        
        # Calculate required ISLs
        isl_matrix = np.zeros_like(self.traffic_matrix, dtype=int)
        for i in range(self.traffic_matrix.shape[0]):
            for j in range(i + 1, self.traffic_matrix.shape[1]):
                if self.traffic_matrix[i][j] > 0:
                    required_isls = math.ceil(self.traffic_matrix[i][j] / isl_capacity)
                    isl_matrix[i][j] = required_isls
                    isl_matrix[j][i] = required_isls
        
        self.isl_matrix = isl_matrix
        return isl_matrix
    
    def is_network_connected(self) -> bool:
        """
        Check if the network built from the traffic matrix is fully connected.
        
        Returns:
            bool: True if network is connected, False otherwise
        """
        if self.traffic_matrix is None:
            self.generate_traffic_matrix()
            
        # Create network graph
        G = nx.Graph()
        num_grids = self.grid_rows * self.grid_cols
        
        # Add all grids as nodes
        for i in range(num_grids):
            G.add_node(i)
            
        # Add all edges with traffic
        for i in range(num_grids):
            for j in range(i + 1, num_grids):
                if self.traffic_matrix[i][j] > 0:
                    G.add_edge(i, j)
        
        # Check if graph is connected
        # Note: we only check if nodes with traffic are connected, not all grids
        active_nodes = [node for node in G.nodes() if G.degree(node) > 0]
        if not active_nodes:
            return True  # No active nodes, consider as connected
            
        active_graph = G.subgraph(active_nodes)
        return nx.is_connected(active_graph)
    
    def is_path_reachable(self, source: int, target: int) -> bool:
        """
        Check if there is a path between two grids.
        
        Args:
            source: Source grid ID
            target: Target grid ID
            
        Returns:
            bool: True if a path exists, False otherwise
        """
        if self.traffic_matrix is None:
            self.generate_traffic_matrix()
            
        # Create network graph
        G = nx.Graph()
        num_grids = self.grid_rows * self.grid_cols
        
        # Add all grids as nodes
        for i in range(num_grids):
            G.add_node(i)
            
        # Add all edges with traffic
        for i in range(num_grids):
            for j in range(i + 1, num_grids):
                if self.traffic_matrix[i][j] > 0:
                    G.add_edge(i, j)
        
        # Check if source and target are in the graph
        if source not in G or target not in G:
            return False
            
        # Check if there's a path between source and target
        return nx.has_path(G, source, target)
    
    def _load_satellite_data(self, satellite_data_file: str, grid_satellite_file: str) -> tuple:
        """
        Internal method to load satellite data and grid satellite coverage information.
        
        Args:
            satellite_data_file: Path to satellite orbit data file
            grid_satellite_file: Path to grid satellite coverage data file
            
        Returns:
            tuple: (coverage_matrices, satellite_locations, satellite_params)
            
        Raises:
            FileNotFoundError: If either file does not exist
        """
        # Validate files exist
        if not os.path.exists(satellite_data_file):
            raise FileNotFoundError(f"Satellite data file not found: {satellite_data_file}")
            
        if not os.path.exists(grid_satellite_file):
            raise FileNotFoundError(f"Grid satellite coverage file not found: {grid_satellite_file}")
            
        # Load satellite data
        supply_data = np.load(satellite_data_file, allow_pickle=True)
        _, _, _, sat_location0, _ = supply_data[0]
        num_timestamps = len(sat_location0)
        
        # Initialize data structures
        coverage_matrices = np.load(grid_satellite_file, allow_pickle=True).item()
        num_time = len(coverage_matrices)
        satellite_locations = {t: {} for t in range(num_timestamps)}
        satellite_params = {}
        
        # Process each satellite's data
        for idx, data in enumerate(supply_data):
            param, random_numbers, cover, sat_location, _ = data
            
            satellite_params[idx] = {
                'height': param[0],
                'inclination': param[1],
                'alpha0': param[2],
                'initial_slot': random_numbers
            }
            
            # Store location information
            for t in range(num_timestamps):
                satellite_locations[t][idx] = sat_location[t]
        
        return coverage_matrices, satellite_locations, satellite_params
    
    def scale_traffic_matrix(self, satellite_data_file: str, grid_satellite_file: str, 
                           num_satellites: int, num_timestamps: int) -> tuple:
        """
        Scale the traffic matrix based on satellite constraints and calculate ISL requirements.
        
        Args:
            satellite_data_file: Path to satellite orbit data file
            grid_satellite_file: Path to grid satellite coverage data file
            num_satellites: Number of satellites
            num_timestamps: Number of time slots
            
        Returns:
            tuple: (scaled_traffic_matrix, scaled_isl_matrix, scaling_factor)
            
        Raises:
            FileNotFoundError: If required data files are not found
        """
        try:
            # If we don't have a traffic matrix yet, generate one using the satellite density
            if self.traffic_matrix is None:
                self._load_grid_satellite_density(grid_satellite_file)
                self.generate_traffic_matrix()
            
            # Load satellite data
            grid_satellites, _, _ = self._load_satellite_data(satellite_data_file, grid_satellite_file)
            
            # Get ISL capacity
            isl_capacity = self.config['global_settings']['isl_capacity_gbps']
            
            # Find appropriate scaling factor
            scaling_factor = self._find_scaling_factor(
                self.traffic_matrix,
                grid_satellites,
                num_satellites,
                num_timestamps,
                isl_capacity
            )
            
            # Apply scaling factor
            scaled_matrix = self.traffic_matrix * scaling_factor
            
            # Calculate ISL requirements for each grid pair
            scaled_isl_matrix = np.zeros_like(scaled_matrix, dtype=int)
            for i in range(scaled_matrix.shape[0]):
                for j in range(i + 1, scaled_matrix.shape[1]):
                    if scaled_matrix[i][j] > 0:
                        isls = math.ceil(scaled_matrix[i][j] / isl_capacity)
                        scaled_isl_matrix[i][j] = isls
                        scaled_isl_matrix[j][i] = isls
            
            # Save results
            self.scaled_traffic_matrix = scaled_matrix
            self.scaled_isl_matrix = scaled_isl_matrix
            self.scaling_factor = scaling_factor
            
            return scaled_matrix, scaled_isl_matrix, scaling_factor
            
        except FileNotFoundError as e:
            print(f"Error: {e}")
            raise
        except Exception as e:
            print(f"Error scaling traffic matrix: {e}")
            raise
    
    def _find_scaling_factor(self, traffic_matrix: np.ndarray,
                           grid_satellites: Dict,
                           num_satellites: int,
                           num_timestamps: int,
                           isl_capacity: float = 200.0) -> float:
        """
        Internal method to find the appropriate scaling factor using binary search.
        
        Args:
            traffic_matrix: Original traffic matrix
            grid_satellites: Grid satellite coverage data
            num_satellites: Number of satellites
            num_timestamps: Number of time slots
            isl_capacity: Capacity of each ISL in Gbps
            
        Returns:
            float: The scaling factor
        """
        # Binary search for appropriate scaling factor
        left = 0.0
        right = 1.0  # Start with 1, larger factors will certainly violate constraints
        best_factor = 0.0
        epsilon = 1e-6  # Precision
        
        while right - left > epsilon:
            mid = (left + right) / 2
            scaled_matrix = traffic_matrix * mid
            
            # Check if all time slots satisfy constraints
            valid = True
            for t in range(num_timestamps):
                # Check satellite connection constraints
                if not self._check_satellite_constraints(scaled_matrix, grid_satellites, num_satellites, t, isl_capacity):
                    valid = False
                    break
            
            if valid:
                best_factor = mid
                left = mid
            else:
                right = mid
        
        return best_factor
    
    def _check_satellite_constraints(self, traffic_matrix: np.ndarray,
                                   grid_satellites: Dict,
                                   num_satellites: int,
                                   timestamp: int,
                                   isl_capacity: float = 200.0) -> bool:
        """
        Internal method to check if satellite connection constraints are satisfied.
        
        Args:
            traffic_matrix: Traffic matrix to check
            grid_satellites: Grid satellite coverage data
            num_satellites: Number of satellites
            timestamp: Time slot to check
            isl_capacity: Capacity of each ISL in Gbps
            
        Returns:
            bool: True if constraints are satisfied, False otherwise
        """
        satellite_connections = defaultdict(int)
        num_grids = traffic_matrix.shape[0]
        
        # Check all grid pairs with traffic
        for i in range(num_grids):
            for j in range(i + 1, num_grids):
                if traffic_matrix[i][j] > 0:
                    # Calculate required ISLs
                    required_isls = math.ceil(traffic_matrix[i][j] / isl_capacity)
                    
                    # Get satellites over each grid
                    sats_i = set(self._get_original_satellite_id(vid, num_satellites) 
                               for vid in grid_satellites[timestamp].get(i, []))
                    sats_j = set(self._get_original_satellite_id(vid, num_satellites) 
                               for vid in grid_satellites[timestamp].get(j, []))
                    
                    # Check if enough satellites are available for connections
                    available_connections = 0
                    for sat in sats_i:
                        if satellite_connections[sat] < 1:  # Each satellite can have at most one connection
                            for sat_j in sats_j:
                                if satellite_connections[sat_j] < 1:
                                    available_connections += 1
                                    if available_connections >= required_isls:
                                        break
                            if available_connections >= required_isls:
                                break
                    
                    if available_connections < required_isls:
                        return False
                    
                    # Update satellite connections
                    connections_made = 0
                    for sat in sats_i:
                        if connections_made >= required_isls:
                            break
                        if satellite_connections[sat] < 1:
                            for sat_j in sats_j:
                                if satellite_connections[sat_j] < 1:
                                    satellite_connections[sat] += 1
                                    satellite_connections[sat_j] += 1
                                    connections_made += 1
                                    if connections_made >= required_isls:
                                        break
        
        return True
    
    def _get_original_satellite_id(self, virtual_id: int, num_satellites: int) -> int:
        """
        Internal method to get the original satellite ID from virtual node ID.
        
        Args:
            virtual_id: Virtual node ID
            num_satellites: Number of satellites
            
        Returns:
            int: Original satellite ID
        """
        if virtual_id < num_satellites:
            return virtual_id
        elif virtual_id < 2 * num_satellites:
            return virtual_id - num_satellites
        else:
            return virtual_id - 2 * num_satellites
    
    def generate_and_save_traffic_matrices(self, grid_satellites_file: str = None, output_dir: str = 'output', save: bool = True) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generate traffic matrix and ISL requirements matrix, with option to save to files.
        
        Args:
            grid_satellites_file: Optional path to grid satellite coverage file for density calculation
            output_dir: Output directory path
            save: Whether to save matrices to files
            
        Returns:
            Tuple[np.ndarray, np.ndarray]: (traffic_matrix, isl_matrix)
        """
        # Generate traffic matrix with optional density information
        if self.traffic_matrix is None:
            self.generate_traffic_matrix(grid_satellites_file)
            
        # Calculate ISL requirements
        if self.isl_matrix is None:
            self.calculate_isl_requirements()
        
        # Save to files if requested
        if save:
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)
                
            # Save original traffic matrix
            np.save(os.path.join(output_dir, 'traffic_matrix.npy'), self.traffic_matrix)
            # Save ISL requirements matrix
            np.save(os.path.join(output_dir, 'isl_matrix.npy'), self.isl_matrix)
            print(f"Traffic matrix and ISL requirements matrix saved to {output_dir} directory")
        
        return self.traffic_matrix, self.isl_matrix
    
    def scale_and_save_traffic_matrices(self, satellite_data_file: str, grid_satellite_file: str,
                                       num_satellites: int, num_timestamps: int,
                                       output_dir: str = 'output', save: bool = True) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        Scale traffic matrix based on satellite constraints and calculate ISL requirements, with option to save to files.
        
        Args:
            satellite_data_file: Path to satellite orbit data file
            grid_satellite_file: Path to grid satellite coverage data file
            num_satellites: Number of satellites
            num_timestamps: Number of time slots
            output_dir: Output directory path
            save: Whether to save matrices to files
            
        Returns:
            Tuple[np.ndarray, np.ndarray, float]: (scaled_traffic_matrix, scaled_isl_matrix, scaling_factor)
            
        Raises:
            FileNotFoundError: If required data files are not found
        """
        try:
            # Scale traffic and calculate ISL matrices - will also handle file existence checks
            scaled_matrix, scaled_isl_matrix, scaling_factor = self.scale_traffic_matrix(
                satellite_data_file, 
                grid_satellite_file,
                num_satellites,
                num_timestamps
            )
            
            # Save to files if requested
            if save:
                if not os.path.exists(output_dir):
                    os.makedirs(output_dir)
                    
                np.save(os.path.join(output_dir, 'scaled_traffic_matrix.npy'), scaled_matrix)
                np.save(os.path.join(output_dir, 'scaled_isl_matrix.npy'), scaled_isl_matrix)
                
                # Save scaling factor
                with open(os.path.join(output_dir, 'scaling_factor.txt'), 'w') as f:
                    f.write(f"Scaling factor: {scaling_factor}")
                    
                print(f"Scaled traffic matrix and ISL matrix saved to {output_dir} directory")
            
            return scaled_matrix, scaled_isl_matrix, scaling_factor
            
        except FileNotFoundError as e:
            print(f"Error: {e}")
            raise
        except Exception as e:
            print(f"Error scaling and saving traffic matrices: {e}")
            raise
    
    def analyze_and_save_connectivity(self, output_dir: str = 'output', save: bool = True) -> Dict:
        """
        Analyze network connectivity and path reachability, with option to save results to a file.
        
        Args:
            output_dir: Output directory path
            save: Whether to save results to a file
            
        Returns:
            Dict: Connectivity analysis results
        """
        # First ensure we have a traffic matrix
        if self.traffic_matrix is None:
            self.generate_traffic_matrix()
        
        # Prepare connectivity analysis results
        connectivity = {
            'is_connected': self.is_network_connected(),
            'path_reachability': {}
        }
        
        # Check reachability for each demand path
        for demand in self.config['traffic_demands']:
            source = demand['source']
            destination = demand['destination']
            connectivity['path_reachability'][f"{source}-{destination}"] = self.is_path_reachable(source, destination)
        
        # Save to file if requested
        if save:
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)
                
            with open(os.path.join(output_dir, 'connectivity.json'), 'w', encoding='utf-8') as f:
                json.dump(connectivity, f, indent=2)
                
            print(f"Connectivity analysis results saved to {output_dir} directory")
        
        return connectivity
    
    def get_and_save_paths(self, output_dir: str = 'output', save: bool = True) -> Dict:
        """
        Get all path information, with option to save to a file.
        
        Args:
            output_dir: Output directory path
            save: Whether to save path information to a file
            
        Returns:
            Dict: Path information
        """
        # First ensure we have calculated paths
        if not self.paths and self.traffic_matrix is None:
            self.generate_traffic_matrix()
        
        # Prepare path information
        paths_info = {}
        for (source, destination), path in self.paths.items():
            paths_info[f"{source}-{destination}"] = path
        
        # Save to file if requested
        if save:
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)
                
            with open(os.path.join(output_dir, 'paths.json'), 'w', encoding='utf-8') as f:
                json.dump(paths_info, f, indent=2)
                
            print(f"Path information saved to {output_dir} directory")
        
        return paths_info
    
    def save_all_results(self, output_dir: str = 'output', 
                        grid_satellites_file: str = None,
                        satellite_data_file: str = None, 
                        num_satellites: int = None, 
                        num_timestamps: int = None) -> None:
        """
        Save all processing results to files.
        
        Args:
            output_dir: Output directory path
            grid_satellites_file: Path to grid satellite coverage data file (optional)
            satellite_data_file: Path to satellite orbit data file (optional)
            num_satellites: Number of satellites (optional)
            num_timestamps: Number of time slots (optional)
        """
        # Generate and save basic traffic matrix and ISL matrix
        self.generate_and_save_traffic_matrices(
            grid_satellites_file=grid_satellites_file,
            output_dir=output_dir
        )
        
        # If satellite data is provided, generate and save scaled matrices
        scale_success = False
        if all([satellite_data_file, grid_satellites_file, num_satellites, num_timestamps]):
            try:
                self.scale_and_save_traffic_matrices(
                    satellite_data_file,
                    grid_satellites_file,
                    num_satellites,
                    num_timestamps,
                    output_dir=output_dir
                )
                scale_success = True
            except FileNotFoundError as e:
                print(f"Warning: {e}")
                print("Skipping traffic matrix scaling step.")
            except Exception as e:
                print(f"Warning: Error in scaling traffic matrices: {e}")
                print("Skipping traffic matrix scaling step.")
        
        # Analyze and save connectivity results
        self.analyze_and_save_connectivity(output_dir=output_dir)
        
        # Get and save path information
        self.get_and_save_paths(output_dir=output_dir)
        
        print(f"All results saved to {output_dir} directory")
        if not scale_success and all([satellite_data_file, grid_satellites_file, num_satellites, num_timestamps]):
            print("Note: Traffic matrix scaling was not completed due to errors.")
