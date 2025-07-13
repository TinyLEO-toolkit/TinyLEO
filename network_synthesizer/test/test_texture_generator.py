"""
Orbital Texture Generator Test Script

Tests the texture generation pipeline for satellite network synthesis.
Generates ground track coverage patterns for both access and backbone networks.
Supports two modes:
1. Default: Loads pre-configured JSON files
2. Custom: Programmatically generates orbital parameters
"""

import sys
import os
import numpy as np
import math
import json

# Add parent directory to Python path to enable module imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import *
from orbital_texture_generator import TextureGenerator

def candidate_space(num_beta, num_alpha, sat_height, single_shell=True, q_thresh=50):
    """
    Generates discretized orbital parameter space for constellation design.
    
    Args:
        num_beta (int): Number of inclination partitions (0° to 180°)
        num_alpha (int): Number of RAAN partitions (0° to 360°)
        sat_height (float): Nominal altitude in km (used when single_shell=True)
        single_shell (bool): If False, scans altitudes 340-10000km
        q_thresh (int): Maximum rational denominator for repeat ground tracks
        
    Returns:
        list: Orbital parameter combinations in format:
              [ [h,T,q,p,η], β, α ] where:
              - h: altitude (km)
              - T: orbital period (s)
              - q,p: resonance ratio (T/TE = p/q)
              - η: coverage efficiency
              - β: inclination (rad)
              - α: RAAN (rad)
    """
    T_range = []
    
    # Orbital period generation
    if single_shell:
        # Single altitude configuration
        h = sat_height
        T = satellite_period(h * 1e3)  # Convert to meters for calculation
        p, q = approximate_ratio(int(T), precision=1e-3)  # Rational approximation
        eta = coverage_eta(T)          # Coverage efficiency metric
        T_range.append([h, T, q, p, eta])
    else:
        # Multi-altitude configuration scan
        for h in np.arange(340, 10000, 1):  # 1km resolution
            T = satellite_period(h * 1e3)
            p, q = approximate_ratio(int(T), precision=1e-3)
            if q <= q_thresh:  # Resonance filtering
                eta = coverage_eta(T)
                T_range.append([h, T, q, p, eta])

    # Angular discretization
    beta_range = [math.radians(b) for b in np.linspace(0, 180, num_beta)]
    alpha_range = [math.radians(a) for a in np.linspace(0, 360, num_alpha)]

    # Parameter space generation
    return [
        [T, beta, alpha]
        for T in T_range
        for beta in beta_range
        for alpha in alpha_range
    ]

if __name__ == "__main__":
    # Operational mode selection
    test_mode = "default"  # Alternatives: "custom"
    
    # ==================================================================
    # ACCESS NETWORK PROCESSING
    # ==================================================================
    print("\nInitializing Access Network Texture Generation...")
    
    if test_mode == "default":
        # Load pre-configured access network parameters
        with open("config/texture_generator_config_access.json") as f:
            config_access = json.load(f)
    else:
        # Programmatic configuration
        params = candidate_space(
            num_beta=45, 
            num_alpha=90,
            sat_height=573  # Standard LEO altitude
        )
        
        config_access = {
            "mode": "access",
            "candidate_orbits": params,
            "memory_threshold": 32,  # GB
            "num_processes": 100,    # Parallel workers
            "satellite_height": 573,
            "demand_file": "data/satellite_network_access_demand.npy",
            "cell_size": 4,          # Degrees
            "texture_save_path": "data/candidate_texture_library_access/",
            "user_per_sat": 960      # Capacity
        }
    
    # Execute texture generation
    TextureGenerator(config_access).run()

    # ==================================================================
    # BACKBONE NETWORK PROCESSING  
    # ==================================================================
    print("\nInitializing Backbone Network Texture Generation...")
    
    if test_mode == "default":
        # Load pre-configured backbone parameters
        with open("config/texture_generator_config_backbone.json") as f:
            config_backbone = json.load(f)
    else:
        # Programmatic configuration (shared parameters)
        config_backbone = {
            "mode": "backbone",              # Different network type
            "candidate_orbits": params,      
            "memory_threshold": 32,          
            "num_processes": 100,
            "satellite_height": 573,
            "demand_file": "data/satellite_network_backbone_demand.npy",  # Different demand
            "cell_size": 4,
            "texture_save_path": "data/candidate_texture_library_backbone/",
            "user_per_sat": 960
        }
    
    # Execute texture generation
    TextureGenerator(config_backbone).run()