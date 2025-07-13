from tinyleo.northbound import TinyLEONorthboundAPI
import os
import matplotlib.pyplot as plt
import numpy as np

"""
Example script demonstrating the usage of TinyLEO Northbound API.
All file existence checks are handled within the API.
"""
# Create output directories
os.makedirs("result/output_nbi", exist_ok=True)
os.makedirs("result/output_scaled_nbi", exist_ok=True)
os.makedirs("result/output_all_nbi", exist_ok=True)

# Example configuration files and data
config_file = "config/mpc_config.json"
grid_satellites_file = "data/grid_satellites_map_for_backbone.npy"
satellite_data_file = "data/satellite_constellation_for_backbone.npy"
num_satellites = 3343
num_timestamps = 21

# 1. Initialize the API
print("\n1. Initializing TinyLEO Northbound API...")
api = TinyLEONorthboundAPI(config_file)

# 2. Generate traffic matrices and ISL requirements
print("\n2. Generating and saving traffic matrices...")
traffic_matrix, isl_matrix = api.generate_and_save_traffic_matrices(
    grid_satellites_file=grid_satellites_file, 
    output_dir="output"
)
print(f"Generated traffic matrix of shape: {traffic_matrix.shape}")
print(f"Total traffic: {traffic_matrix.sum() / 2:.2f} Gbps") # Divide by 2 because traffic is bidirectional

# 3. Get path information
print("\n3. Getting and saving path information...")
paths = api.get_and_save_paths(output_dir="output")
print(f"Found {len(paths)} paths")

# 4. Analyze network connectivity
print("\n4. Analyzing and saving network connectivity...")
connectivity = api.analyze_and_save_connectivity(output_dir="output")
print(f"Network is connected: {connectivity['is_connected']}")
print(f"Path reachability analysis completed for {len(connectivity['path_reachability'])} demand pairs")

# 5. Scale traffic matrices based on satellite constraints
print("\n5. Scaling and saving traffic matrices based on satellite constraints...")
try:
    scaled_matrix, scaled_isl_matrix, scaling_factor = api.scale_and_save_traffic_matrices(
        satellite_data_file=satellite_data_file,
        grid_satellite_file=grid_satellites_file,
        num_satellites=num_satellites,
        num_timestamps=num_timestamps,
        output_dir="output_scaled"
    )
    print(f"Scaling factor: {scaling_factor:.4f}")
    print(f"Scaled traffic total: {scaled_matrix.sum() / 2:.2f} Gbps")
    print(f"Scaled ISLs total: {scaled_isl_matrix.sum() / 2}")
except Exception as e:
    print(f"Error in scaling traffic matrices: {e}")

# 6. Save all results with one call
print("\n6. Saving all results with one method call...")
api.save_all_results(
    output_dir="output_all",
    grid_satellites_file=grid_satellites_file,
    satellite_data_file=satellite_data_file,
    num_satellites=num_satellites,
    num_timestamps=num_timestamps
)
print("All results saved to 'output_all' directory")
