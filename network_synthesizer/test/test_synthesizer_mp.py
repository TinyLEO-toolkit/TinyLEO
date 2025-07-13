"""
TinyLEO Network Synthesizer Test Script - MP Algorithm Implementation

This script tests the multi-process (MP) based network synthesizer for:
1. Access Network 
2. Backbone Network

Features:
- Supports both pre-configured (JSON) and programmatic configuration modes
- Utilizes parallel processing for efficient constellation synthesis
- Generates operational satellite location data for online network operation
"""

import sys
import os
import numpy as np
import math
import json

# Add parent directory to Python path to enable module imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import synthesizer_mp  # Import the multi-process synthesizer implementation

if __name__ == "__main__":
    # ==================================================================
    # CONFIGURATION SETUP
    # ==================================================================
    # Test mode selection:
    # "default" - uses pre-configured JSON files (recommended for production)
    # "custom" - generates configurations programmatically (for development/testing)
    test_mode = "default"  
    
    # ==================================================================
    # ACCESS NETWORK PROCESSING
    # ==================================================================
    print("\nInitializing Access Network Synthesis...")
    
    if test_mode == "default":
        # PRODUCTION CONFIG: Load from JSON configuration file
        config_file_access = "config/mp_config_access.json"
        with open(config_file_access, "r") as f:
            config_access = json.load(f)  # Load access network configuration
    else:
        # DEVELOPMENT CONFIG: Programmatic configuration
        config_access = {
            "demand_file": "data/satellite_network_access_demand.npy",  # User traffic demand matrix
            "results_save_path": "data/eval1_573_access.npy",  # Optimization results output
            "memory_threshold": 32,  # Maximum memory allocation in GB
            "num_processes": 100,  # Number of parallel worker processes
            "texture_save_path": "data/candidate_texture_library_access/",  # Coverage pattern storage
            "satellite_height": 573,  # Operational altitude in km (typical LEO)
            "epsilon": 1e-6  # Service availability threshold (1e-6 = 99.9999%)
        }
    
    # Initialize and execute MP synthesizer for access network
    print("Running MP Synthesizer for Access Network...")
    synthesizer_mp.Synthesizer(config_access)
    
    # Post-processing of optimization results
    print("Processing Synthesis Results...")
    results = np.load(config_access["results_save_path"], allow_pickle=True)
    
    # Generate operational satellite location data for mission control
    print("Generating Operational Satellite Data...")
    synthesizer_mp.generate_operation_sat_location(
        results, 
        "data/operation_sat_info_access.npy"  # Output file for access network
    )
    
    # ==================================================================
    # BACKBONE NETWORK PROCESSING  
    # ==================================================================
    print("\nInitializing Backbone Network Synthesis...")
    
    if test_mode == "default":
        # PRODUCTION CONFIG: Load from JSON configuration file
        config_file_backbone = "config/mp_config_backbone.json"
        with open(config_file_backbone, "r") as f:
            config_backbone = json.load(f)  # Load backbone network configuration
    else:
        # DEVELOPMENT CONFIG: Programmatic configuration
        config_backbone = {
            "demand_file": "data/satellite_network_backbone_demand.npy",  # Inter-satellite demand
            "results_save_path": "data/eval1_573_backbone.npy",  # Separate results file
            "memory_threshold": 32,  # Consistent resource allocation
            "num_processes": 100,  # Same parallel processing scale
            "texture_save_path": "data/candidate_texture_library_backbone/",  # Dedicated pattern storage
            "satellite_height": 573,  # Matching operational altitude
            "epsilon": 1e-6  # Identical availability requirement
        }
        
    # Initialize and execute MP synthesizer for backbone network
    print("Running MP Synthesizer for Backbone Network...")
    synthesizer_mp.Synthesizer(config_backbone)
    
    # Post-processing of planning results from MP synthesizer
    print("Processing Synthesis Results...")
    results = np.load(config_backbone["results_save_path"], allow_pickle=True)
    
    # Generate operational satellite location data for mission control
    print("Generating Operational Satellite Data...")
    synthesizer_mp.generate_operation_sat_location(
        results,
        "data/operation_sat_info_backbone.npy"  # Output file for backbone network
    )

    print("\nNetwork Synthesis Completed Successfully!")