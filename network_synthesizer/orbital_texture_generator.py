'''
This module generates coverage texture libraries for offline network planning, supporting both access and backbone network.
'''
import os
import sys
import math
import json
import multiprocessing as mp
import numpy as np
from haversine import haversine, Unit
from scipy.sparse import lil_matrix
from tqdm import tqdm

from utils import *

# Constant
TE=24*3600 #[s] Earth's rotation period
RE=6371e3 #[m] Earth's equatorial radius (6,371 km converted to meters)
u= 3.986e14 # Standard gravitational parameter (μ = G*M)
K=RE/pow(u,1/3)*pow(2*np.pi,2/3) 
eps=25*np.pi/180 #Minimum elevation angle for UE visibility,25° elevation angle converted to radians

class TextureGenerator():
    '''
    This class generates and manages satellite coverage texture library for offline network planning.
    
    The texture library stores pre-computed coverage texture for candidate satellite orbits,
    enabling dynamic spatiotemporal supply-demand matching.
    
    Attributes:
        mode (str): Operation mode - 'access' or 'backbone'
        candidate_orbits (list): List of candidate orbital parameters
        texture_save_path (str): Path to save generated texture files
        memory_threshold (int): Memory usage limit
        num_processes (int): Number of parallel processes
        satellite_height (float): Satellite altitude in km
        demand_list (list): List of demand points/cells
        cell_size (int): Size of grid cells in degrees
        
    Example:
        >>> lib = texture_library(mode="access", ...)
        >>> lib.prepare_candidate_texture()
    '''
    def __init__(self,config):
        self.mode=config["mode"]
        # Validate operation mode
        if self.mode not in ["access","backbone"]:
            raise ValueError("Mode must be either 'access' or 'backbone'")
            
        self.candidate_orbits=config["candidate_orbits"]
        
        # Setup texture library directories
        self.texture_save_path=config["texture_save_path"]
        mkdir(self.texture_save_path)

        # System constraints
        self.memory_threshold=config["memory_threshold"]
        self.num_processes=config["num_processes"]
        
        # Orbital mechanics calculations
        self.satellite_height=config["satellite_height"]
        self.T=satellite_period(self.satellite_height*1e3)
        self.p,self.q=approximate_ratio(int(self.T),precision=1e-3)
        self.eta=coverage_eta(self.T)
        self.step=2*self.eta
        self.time_split=int(2*np.pi*self.q/self.step)
        
        # Demand processing
        self.demand_list=np.load(config["demand_file"],allow_pickle=True)
        self.cell_size=config["cell_size"]
        # Access mode specific configurations
        if self.mode=="access":
            for idx in range(len(self.demand_list)):
                self.demand_list[idx]['lat_lon'][0]=np.radians(self.demand_list[idx]['lat_lon'][0])
                self.demand_list[idx]['lat_lon'][1]=np.radians(self.demand_list[idx]['lat_lon'][1])
            self.S_set=self._calculate_cell_area()
            self.user_per_sat=config["user_per_sat"]
        
    
    def _alpha_gamma_to_lon_lat(self,alpha, gamma, inc):
        """
        Convert orbital parameters (alpha, gamma) into geographic coordinates (longitude, latitude).
    
        Args:
            alpha (float): RAAN (in radians), typically ranging from 0 to 2π.
            gamma (float): Orbital angle from the ascending node (in radians).
            inc (float): Orbital inclination (in radians).
    
        Returns:
            Tuple[float, float]: 
                - lon (float): Geographic longitude in radians, range [-π, π].
                - lat (float): Geographic latitude in radians, range [-π/2, π/2].
        """
    
        # Step 1: Calculate latitude lat
        lat = math.asin(math.sin(inc) * math.sin(gamma))
        
        # Step 2: Calculate temp
        temp = math.atan2(math.cos(inc) * math.sin(gamma), math.cos(gamma))
        
        # Step 3: Calculate longitude lon
        lon = (temp + alpha) % (2 * math.pi)
        
        # Adjust lon to the range [-π, π]
        if lon > math.pi:
            lon -= 2 * math.pi
        
        return lon, lat


    # backbone network texture 
    def _get_grid_id(self,lat_deg, lon_deg):
        """
        Determine the grid ID based on latitude and longitude coordinates.
        Grid numbering starts from the top-left corner (0-120), left-to-right and top-to-bottom.
        
        Args:
            lat_deg: Latitude in degrees
            lon_deg: Longitude in degrees
            
        Returns:
            int: Grid ID (0-120) or -1 if invalid
        """
        # Normalize longitude to [-180, 180] range
        lon_deg = ((lon_deg + 180) % 360) - 180
        
        # Latitude boundaries (90°N to 90°S, 11 rows)
        lat = 90
        row = -1
        for i in range(11):
            if i < 10:
                # First 10 rows: 16 degrees each
                if lat_deg <= lat and lat_deg > (lat - 16):
                    row = i
                    break
                lat -= 16
            else:
                # Last row: 20 degrees
                if lat_deg <= lat and lat_deg >= -90:
                    row = i
                    break
        
        # Longitude boundaries (-180° to 180°, 11 columns)
        lon = -180
        col = -1
        for j in range(11):
            if j < 10:
                # First 10 columns: 32 degrees each
                if lon_deg >= lon and lon_deg < (lon + 32):
                    col = j
                    break
                lon += 32
            else:
                # Last column: 40 degrees
                if lon_deg >= lon and lon_deg <= 180:
                    col = j
                    break
        
        # Handle edge cases
        if row == -1:
            if lat_deg == -90:  # South Pole
                row = 10
        if col == -1:
            if lon_deg == 180:  # 180th meridian
                col = 10
        
        # Calculate grid ID: top-left to bottom-right numbering
        if row != -1 and col != -1:
            grid_id = row * 11 + col
            return grid_id
        else:
            # Handle invalid coordinates
            print(f"Warning: Could not determine grid for lat={lat_deg}, lon={lon_deg}")
            return -1
    
    def _create_new_grid_satellites(self,satellite_location):
        """
        Assigns a satellite to its corresponding grid based on geographic coordinates.
        
        Args:
            satellite_location: Tuple containing (longitude, latitude) in radians
            
        Returns:
            int: Grid ID (0-120) where the satellite belongs
            
        Raises:
            ValueError: If coordinates cannot be mapped to a valid grid
        """
        lon, lat = satellite_location
        num_grids = 11 * 11  # Total 11x11 grid cells
        
        # Convert from radians to degrees
        lon_deg = np.degrees(lon)
        lat_deg = np.degrees(lat)
        
        # Determine which grid the satellite belongs to
        grid_id = self._get_grid_id(lat_deg, lon_deg)
        
        # Validate grid ID before returning
        if 0 <= grid_id < num_grids:
            return grid_id
        else:
            raise ValueError(
                f"Invalid grid mapping for coordinates: "
                f"lon={lon_deg:.4f}°, lat={lat_deg:.4f}° "
                f"(computed grid_id={grid_id})"
            )
    
    def _cal_supply_all_cell_backbone(self,param):
        """
        Calculate coverage for all grid cells along a stable ground track.
        
        Args:
            param: Satellite orbit parameters [T_set, beta, alpha0]
                   where T_set = [h, T, q, p, eta]
            
        Returns:
            None: Saves coverage results to file instead of returning
            
        Output File:
            Saves numpy array containing:
            - Coverage sets for each time slot
            - Satellite longitude/latitude positions
        """
        # Orbit parameters
        T_set = param[0]
        q, eta, T = T_set[2], T_set[4], T_set[1]  
        time_step = eta * 2
        inclination = param[1]
        initial_alpha = param[2]
        
        total_supply = []
        num_time_slots = int(2 * np.pi * q / time_step)  # Total number of time slots
        
        for i in range(num_time_slots):
            # Calculate satellite position at current time slot
            current_time = time_step * i / (2 * np.pi / T)
            alpha_sat = (initial_alpha - (2 * np.pi / T) * current_time) % (2 * np.pi)
            gamma_sat = (time_step * i) % (2 * np.pi)
            
            # Convert to geographic coordinates
            lon_sat, lat_sat = self._alpha_gamma_to_lon_lat(alpha_sat, gamma_sat, inclination)
            
            # Determine coverage for current position
            grid_id = self._create_new_grid_satellites([lon_sat, lat_sat])
            coverage_set = [[grid_id, 1.0]]  # Current coverage set
            
            total_supply.append([coverage_set, lon_sat, lat_sat])
        
        return [param[0][0],param[1],param[2],total_supply]
    
    # access network texture
    def _is_cover(self,l_sat,p_sat,l_cell,p_cell,eta):
        """
        Determine whether a satellite at (l_sat, p_sat) covers a ground cell at (l_cell, p_cell),
        given the coverage angle eta.
    
        Args:
            l_sat (float): Longitude of the satellite in radians.
            p_sat (float): Latitude of the satellite in radians.
            l_cell (float): Longitude of the ground cell in radians.
            p_cell (float): Latitude of the ground cell in radians.
            eta (float): Half of the satellite's coverage angle in radians.
    
        Returns:
            bool: True if the satellite covers the cell, False otherwise.
        """
        double_eta=2*eta
        if (p_sat-double_eta)<-np.pi/2 or (p_sat+double_eta)>np.pi/2 or (l_sat-double_eta)<-np.pi or (l_sat+double_eta)>np.pi: # If the latitude range is out of bounds, it is directly judged by the spherical distance
            d=haversine((np.degrees(p_sat),np.degrees(l_sat)),(np.degrees(p_cell),np.degrees(l_cell)),unit=Unit.METERS)
            if d <= eta*RE:
                return True
        else:
            if (p_sat-double_eta)<=p_cell<=(p_sat+double_eta) and (l_sat-double_eta)<=l_cell<=(l_sat+double_eta):  # prescreening
                d=haversine((np.degrees(p_sat),np.degrees(l_sat)),(np.degrees(p_cell),np.degrees(l_cell)),unit=Unit.METERS)
                if d <= eta*RE:
                    return True
        return False
        
    def _calculate_cell_area(self):
        '''
        Calculate the approximate area of each grid cell on Earth's surface.
    
        Returns:
            list[float]: A list of areas (in square meters) for each cell.
        '''
        S_set = []
        for idx, demand in enumerate(self.demand_list):
            #Extract latitude (in radians) and calculate the height of the cell
            p_cell = demand['lat_lon'][0]  # latitude
            h = RE * np.radians(self.cell_size)  # hight of the cell (m)
        
            # Calculate the length of the sides of the cell
            L1 = RE * np.sin(np.pi / 2 - abs(p_cell)) * np.radians(self.cell_size)
            tmp=abs(p_cell - np.radians(self.cell_size))
            L2 = RE * np.sin(np.pi / 2 - min(tmp,np.pi/2)) * np.radians(self.cell_size)
            # The trapezoidal formula is used to calculate the area
            if tmp>np.pi/2: 
                h=RE*abs(p_cell + np.pi/2)
            S = 0.5 * h * (L1 + L2)
            S_set.append(S)
        return S_set
    
    def _cal_supply_all_cell_access(self,param):
        """
        Calculate the coverage (supply) for each time slot along a satellite's stable subsatellite point trajectory.
    
        Args:
            param (List): Orbital parameters in the form [ [h, T, q, p, eta], inclination (rad), alpha0 (rad) ].
    
        Saves:
            A NumPy `.npy` file at `base_cover_path` with the name formatted as:
            "<altitude>_<inclination>_<alpha0>.npy", where each entry contains:
                - A list of covered ground cell indices with their adjusted coverage weight.
                - Satellite longitude and latitude at that time slot.
        """
        
        T_set=param[0] 
        q,eta,T=T_set[2], T_set[4],T_set[1]
        step=eta*2
        inc=param[1]
        a0=param[2]
        total_supply=[]
        for i in range(int(2*np.pi*q/step)):# Number of time slices == Number of slots int(2*np.pi*q/step)
            #for each satellite
            t=step*i/(2*np.pi/T)
            a_sat=(a0-(2*np.pi/TE)*t)%(2*np.pi)
            b_sat=(step*i)%(2*np.pi)
            l_sat,p_sat=self._alpha_gamma_to_lon_lat(a_sat,b_sat,inc)
            r_set=[] # All the ground cells covered by the satellite
            for idx,demand in enumerate(self.demand_list):
                l_cell=demand['lat_lon'][1]
                p_cell=demand['lat_lon'][0]
                r=self._is_cover(l_sat,p_sat,l_cell,p_cell,eta) 
                if r:
                    S=self.S_set[idx]
                    r_set.append([idx,S,demand['density']])
            if len(r_set)!=0:
                S_sum=np.sum([item[1] for item in r_set])
                factor=self.user_per_sat/S_sum
                r_set=[[item[0],item[1]*factor]for item in r_set]
                total_supply.append([r_set,l_sat,p_sat])
        return [param[0][0],param[1],param[2],total_supply]
    
    def _recover_fulltime(self,data):
        '''
        Generate the full-time texture of the target satellites
        Args:
            data (list): A list of [slots_num, cover, n_length], where:
                - slots_num (list[int]): Initial indices.
                - cover (list): Satellite coverage data.
                - n_length (int): Length of the full-time demand vector.
         Returns:
            list: [selected_cover (csr_matrix), sat_location (list of coordinates)]
        '''
        num,cover,n_length,cell_number=data
        selected_cover = lil_matrix((1, n_length), dtype=np.float64)
        # print(np.shape(selected_cover))
        sat_location=[]
        for t in range(int(self.time_split)):
            num=(num+t)%int(self.time_split) # The location of the satellite at time t
            sat_location=[cover[num][1],cover[num][2]]
            for idx,users in cover[num][0]: # cover
                selected_cover[0,t*cell_number+idx]+=users 
        selected_cover = selected_cover.tocsr()
        return [selected_cover,sat_location]
    
    def _generate_texture_single_orbit(self,candidate_data):
        '''
        Generate texture data for orbital slots located in a given candidate orbit and save them as  `.npy` files.
        Args:
            candidate_data (list): Contains orbit parameters and precomputed coverage data in the format:
                [height (float), inclination (float), right ascension (float), coverage_data (list)]
                where coverage_data is the output from _cal_supply_all_cell_access/backbone methods.
        Returns:
            bool: Always returns True to indicate successful completion.
        '''
        h,beta,alpha0,cover=candidate_data
        param=[h,beta,alpha0]
        # Initialize coverage matrix dimensions
        cell_number=len(self.demand_list) # Number of demand cells
        n_length=len(self.demand_list)*self.time_split # Total coverage matrix size
        
        for slot in range(self.time_split):# Process each orbital slot (the number of orbital slots== that of time slots)
            # Reconstruct full temporal coverage for this orbital slot
            fulltime_cover = self._recover_fulltime([slot,cover,n_length,cell_number])
            tmp = np.asarray([param,slot,fulltime_cover], dtype="object")
            np.save(self.texture_save_path+str(h)+"_"+str(beta)+"_"+str(alpha0)+"_"+str(slot)+".npy",tmp)
        return True

    def run(self):
        '''
        Prepare satellite coverage textures for candidate orbits in parallel processing.
        '''
        if self.mode=="backbone":
            print("Prepare candidate texture for backbone network")
            with mp.Pool(processes=self.num_processes,) as pool:
                orbit_coverage = list(tqdm(pool.imap(self._cal_supply_all_cell_backbone, self.candidate_orbits), total=len(self.candidate_orbits),mininterval=20,maxinterval=40))
                pool.close()  # Prevents any more tasks from being submitted to the pool
                pool.join()   # Wait for the worker processes to exit
            
        if self.mode=="access":
            print("Prepare candidate texture for access network")
            with mp.Pool(processes=self.num_processes,) as pool:
                orbit_coverage = list(tqdm(pool.imap(self._cal_supply_all_cell_access, self.candidate_orbits), total=len(self.candidate_orbits),mininterval=20,maxinterval=40))
                pool.close()  # Prevents any more tasks from being submitted to the pool
                pool.join()   # Wait for the worker processes to exit

        print("Perform temporal unfolding of the texture corresponding to the satellite on candidate orbital slot")
        
        with mp.Pool(processes=self.num_processes) as pool:
            fulltime_texture =list(tqdm(pool.imap(self._generate_texture_single_orbit, orbit_coverage), total=len(orbit_coverage), mininterval=10,maxinterval=20))
            pool.close()  # Prevents any more tasks from being submitted to the pool
            pool.join()  # Wait for the worker processes to exit
        print("The texture library is ready in "+self.texture_save_path+".")
    
