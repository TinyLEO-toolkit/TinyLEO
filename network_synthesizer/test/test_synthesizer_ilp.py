"""
TinyLEO Network Synthesizer Test Script- ILP Solver Implementation

This script tests the integer linear programming (ILP) based network synthesizer for both:
1. Access Network 
2. Backbone Network 

The script supports two modes:
- Default mode: Loads configurations from JSON files
- Custom mode: Generates configurations programmatically
"""

import sys
import os
import numpy as np
import math
import json

# Add parent directory to Python path to enable module imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import synthesizer_ilp  # Import the ILP-based network synthesizer

if __name__ == "__main__":
    # ==================================================================
    # CONFIGURATION SETUP
    # ==================================================================
    # Test mode selection:
    # "default" - uses pre-configured JSON files
    # "custom" - generates parameters programmatically
    test_mode = "default"  
    
    # ==================================================================
    # ACCESS NETWORK PROCESSING
    # ==================================================================
    print("\nProcessing Access Network...")
    
    if test_mode == "default":
        # DEFAULT MODE: Load configurations from JSON files
        config_file_access = "config/ilp_config_access.json"
        with open(config_file_access, "r") as f:
            config_access = json.load(f)  # Load access network configuration
    else:
        # CUSTOM MODE: Generate configurations programmatically
        config_access = {
            "demand_file": "data/satellite_network_access_demand.npy",  # User demand matrix
            "matrix_path": "data/constraint_matrices_access/",  # Storage for ILP constraint_matrices
            "num_processes": 100,  # Parallel processing threads
            "texture_save_path": "data/candidate_texture_library_access/",  # Orbital texture data
            "satellite_height": 573,  # Orbit altitude in km 
            "results_save_path": "data/eval1_573_access_ilp.npy",  # Solution output
        }
    
    # Initialize and run the ILP synthesizer for access network
    ILP_synthesizer_access = synthesizer_ilp.Synthesizer(config_access)
    ILP_synthesizer_access.ILP_generate_constraint_matrices()  # Generate LP constraint matrices
    ILP_synthesizer_access.ILP_solver()  # Solve the linear programming problem
    
    # ==================================================================
    # BACKBONE NETWORK PROCESSING
    # ==================================================================
    print("\nProcessing Backbone Network...")   

    if test_mode == "default":
        # DEFAULT MODE: Load configurations from JSON files
        config_file_backbone = "config/ilp_config_backbone.json"
        with open(config_file_backbone, "r") as f:
            config_backbone = json.load(f)  # Load backbone network configuration
    else:
        # CUSTOM MODE: Generate configurations programmatically
        config_backbone = {
            "demand_file": "data/satellite_network_backbone_demand.npy",  # Inter-satellite demand
            "matrix_path": "data/constraint_matrices_backbone/",  # Separate matrix storage
            "num_processes": 100,  # Consistent parallel processing
            "texture_save_path": "data/candidate_texture_library_backbone/",  # Separate coverage data
            "satellite_height": 573,  # Same altitude as access network
            "results_save_path": "data/eval1_573_backbone_ilp.npy",  # Separate solution output
        }
    
    # Initialize and run the ILP synthesizer for backbone network
    ILP_synthesizer_backbone = synthesizer_ilp.Synthesizer(config_backbone)
    ILP_synthesizer_backbone.ILP_generate_constraint_matrices()  # Generate backbone-specific constraints
    ILP_synthesizer_backbone.ILP_solver()  # Solve the backbone network problem
    
    print("\nNetwork Synthesis Completed Successfully!")