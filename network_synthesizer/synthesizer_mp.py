'''
A class for performing spatiotemporal matching pursuit algorithm for satellite constellation design.
'''
import os
import json
import multiprocessing as mp
import numpy as np
import psutil
from tqdm import tqdm

from utils import *

class Synthesizer():
    '''
    This class implements a greedy algorithm to select optimal satellite orbits that maximize coverage
    of ground demand points while respecting memory and computational constraints.

    Attributes:
        demand_list (list): List of demand points
        results_save_path (str): Path to save constellation selection results
        memory_threshold (float): Maximum allowed memory usage in GB
        num_processes (int): Number of parallel processes to use
        texture_save_path (str): Path to precomputed coverage texture files
        satellite_height (float): Satellite altitude in km
        texture_save_path (str): Complete path to coverage texture files
    '''
    def __init__(self,config):
        self.demand_list=np.load(config['demand_file'],allow_pickle=True)
        self.results_save_path=config['results_save_path']
        self.memory_threshold=config['memory_threshold']
        self.num_processes=config['num_processes']
        self.texture_save_path=config['texture_save_path']
        self.satellite_height=config['satellite_height']
        self.T=satellite_period(self.satellite_height*1e3)
        self.p,self.q=approximate_ratio(int(self.T),precision=1e-3)
        self.eta=coverage_eta(self.T)
        self.step=2*self.eta
        self.time_split=int(2*np.pi*self.q/self.step)
        self.MP_algorithm(config['epsilon'])
        
    def _compute_dot(self,data):
        """
        Compute the dot product of the full-time coverage vector and the residual vector R.
    
        Args:
            args (tuple): A tuple containing:
                - R (np.ndarray): The residual vector.
                - cid (int): The index of the coverage file.
                - file (string):The path of the texture file of the candidate orbital slot
    
        Returns:
            list: A list containing the computed dot product value and the coverage file index `cid`.
        """
        R,cid,file=data
        data=np.load(file,allow_pickle=True)
        fulltime_cover=data[2][0]
        dot=fulltime_cover @ R
        return [dot[0],cid]
        
    def _update_residual(self,R, file,sat_num):
        """
        Update the residual demand R based on coverage data for a given satellite.
    
        Args:
            R (np.ndarray): The residual vector to be updated.
            file (string):  The path of coverage file.
            sat_num (int): The number of satellites involved in the update.
    
        Returns:
            np.ndarray: The updated residual vector with non-negative values.
        """
        data=np.load(file,allow_pickle=True)
        R[data[2][0].indices] -= (data[2][0].data*sat_num)
        R = np.maximum(R, 0)  # 保证 R 中的所有元素非负
        return R
        
    def _find_max_in_chunk(self,chunk):
        """
        Find the maximum value and its corresponding index in a given chunk.
    
        Args:
            chunk (list): A list of tuples or lists, where each element contains a value and an index.
    
        Returns:
            list: A list containing the maximum value and its corresponding index.
        """
        chunk=np.array(chunk)
        max_value = np.max(chunk[:, 0])  
        local_index = np.argmax(chunk[:, 0])
        max_index = chunk[local_index][1]
        return [max_value,int(max_index)]
    
    def MP_algorithm(self,epsilon):
        '''
        Run spatiotemporal matching pursuit algorithm for satellite constellation design.

        Args:
            epsilon (float): network availability ratio (relative to total demand). Default 1e-6.
    
        Returns:
            bool: True when completed successfully.
    
        Side Effects:
            - Saves selected orbits to self.results_save_path
            - Prints progress if log_details=True
            - Monitors and manages memory usage
        '''
        process = psutil.Process(os.getpid())
        # Network demand
        if isinstance(self.demand_list[0], dict):
            tmp_R=[demand['density'] for demand in self.demand_list]
        else:
            tmp_R=[demand for demand in self.demand_list] 
        ttmpR=tmp_R*int(self.time_split)
        R=np.array(ttmpR).reshape(-1)
        standard_R=sum(R)
        
        # MP_algorithm
        selected = []
        all_cover_files=get_allfile(self.texture_save_path)
        print("The number of all_cover_files:",len(all_cover_files))
        print("Start MP")
        
        while np.sum(R) >= standard_R * epsilon:
            count = 0
            with mp.Pool(processes=self.num_processes) as pool:
                while np.sum(R) >= standard_R * epsilon:
                    # Prepare arguments for parallel processing (optimized to avoid nested list overhead)
                    args_set = [(R, cid, all_cover_files[cid]) for cid in range(len(all_cover_files))]
                    
                    # Parallel computation using imap_unordered for better scheduling efficiency
                    dot_set = list(tqdm(pool.imap_unordered(self._compute_dot, args_set, chunksize=max(1, len(args_set) // self.num_processes)),
                                        total=len(args_set),mininterval=10))
                    # Split results into chunks for parallel max finding
                    chunk_size = max(len(dot_set) // self.num_processes, 1)
                    chunks = [dot_set[i:i + chunk_size] for i in range(0, len(dot_set), chunk_size)]
                    
                    # Find maximum values in parallel chunks
                    results = list(pool.imap_unordered(self._find_max_in_chunk, chunks))
                        
                    # Get global maximum from all chunks
                    max_dot, max_dot_index = max(results, key=lambda x: x[0])
                    
                    # Process the best candidate orbit
                    candidate_orbit = np.load(all_cover_files[max_dot_index], allow_pickle=True)
                    sat_num = 1
                    R = self._update_residual(R, all_cover_files[max_dot_index], sat_num)
                    
                    # Store selected orbit parameters: [param, slot, sat_location, sat_num]
                    selected.append([
                        candidate_orbit[0], 
                        candidate_orbit[1], 
                        candidate_orbit[2][1], 
                        sat_num
                    ])
                
                    print("Selected:", candidate_orbit[0], candidate_orbit[1])
                    print(f"Residual demand: {np.sum(R)}")
                    # Memory management: clean up temporary variables
                    del dot_set, args_set
                    # Memory usage monitoring and early termination
                    current_mem = process.memory_info().rss/(1024*1024*1024)
                    if current_mem > self.memory_threshold * 0.7:
                        print(f"Memory usage: {process.memory_info().rss / 1024 / 1024:.2f} MB")
                        print(f"Residual demand: {np.sum(R)}")
                        print(f"Residual stats - Non-zero: {np.count_nonzero(R)}, Max: {np.max(R)}, Mean: {np.mean(R)}")
                        break
                    count += 1
            pool.close()  # Prevents any more tasks from being submitted to the pool
            pool.join()   # Wait 
        tmp = np.asarray(selected, dtype="object")
        np.save(self.results_save_path,tmp)
        return True
def generate_constellation_params(selected_orbits,save_file):
        '''
        Generate orbital elements for synthesized constellation from selected orbits.
            
        Args:
            selected_orbits (list): List of selected orbit configurations containing:
                - orbit_param: Orbital parameters [height, beta, alpha0]
                - slot: Orbital slot identifier
                - texture: orbital texture data
                - sat_num: Number of satellites in this orbit
            save_file (str): Path to save the generated parameters
        
        Returns:
            bool: True if operation completes successfully
        '''
        sat_params=[]
        sat_nums=0
        for item in selected_orbits:
            orbit_param,slot,texture,sat_num=item
            h,beta,a0=orbit_param
            t=self.step*slot/(2*np.pi/T)
            a_sat=(a0-(2*np.pi/TE)*t)%(2*np.pi)
            b_sat=(self.step*slot)%(2*np.pi)
            sat_params.append({"height":h,"inclination":np.degrees(beta),"RAAN":np.degrees(a_sat),"True anomaly":np.degrees(b_sat)})
            sat_nums+=sat_num
        print("The number of satellite:",sat_nums)
        np.save(save_file,sat_params)
        print("Data is ready in ",save_file,".")
        return True
        
def generate_operation_sat_location(selected_orbits,save_file):
    '''
    Generate operational satellite location data for MPC (Mission Planning and Control).
    
    Args:
        selected_orbits (list): List of selected orbit configurations containing:
            - orbit_param: [height, beta, alpha0]
            - slot: Orbital slot identifier  
            - texture: Coverage texture reference
        save_file (str): Path to save operational data
    
    Returns:
        bool: True if operation completes successfully

    '''
    operation_data=[]
    for selected in selected_orbits:
        tmp_texture=np.load(self.texture_save_path+str(int(selected[0][0]))+"_"+str(selected[0][1])+"_"+str(selected[0][2])+"_"+str(selected[1])+".npy",allow_pickle=True)[2] # Load texture data
        _,covered_cell_id=tmp_texture[0].nonzero() # Get indices of covered ground cells (non-zero elements)
        operation_data.append([selected[0],selected[1],covered_cell_id,selected[2],1]) #param slot cover location
    tmp=np.asarray(operation_data, dtype="object")
    np.save(save_file,tmp)
    print("Data is ready in ",save_file,".")
    return True