'''
A class for solving spatiotemporal supply-demand matching using integer linear programming .
    
'''
import os
import json
import multiprocessing as mp
import numpy as np
from tqdm import tqdm
import gurobipy as gp
from gurobipy import GRB

from utils import *

class Synthesizer():
    '''
    This class formulates and solves the spatiotemporal supply-demand matching as a linear programming problem, aiming to minimize the number of satellites while meeting network demands across all time slots.
    '''
    def __init__(self,config):
        """
        Initialize the integer linear programming solver with problem parameters.
        
        Args:
            demand_list: List of demand points (either dicts with 'density' or raw values)
            matrix_path: Directory to save constraint matrices
            num_processes: Number of parallel processes to use
            satellite_height: Satellite altitude in kilometers
            texture_save_path: Path to precomputed coverage texture files
        """
        self.demand_list=np.load(config["demand_file"],allow_pickle=True)
        self.matrix_path=config['matrix_path']
        self.num_processes=config['num_processes']
        self.texture_save_path=config['texture_save_path']
        self.satellite_height=config['satellite_height']
        self.T=satellite_period(self.satellite_height*1e3)
        self.p,self.q=approximate_ratio(int(self.T),precision=1e-3)
        self.eta=coverage_eta(self.T)
        self.step=2*self.eta
        self.time_split=int(2*np.pi*self.q/self.step)
        self.lp_results_filename=config['results_save_path']
        
    def _generate_matrix_single_slot(self,time_slot):
        '''
        Generate and save a texture matrix for a single time slot.
        Args:
            time_slot (int): The time slot index to process
            
        Returns:
            bool: True if operation completed successfully
            
        '''
        matrix=[]
        time_offset=time_slot*len(self.demand_list) # Calculate time offset in the coverage array
        for constrain_id, idx in enumerate(self.nonzero_cid): # Populate the matrix with texture data for this time slot
            matrix.append([candidate[0, time_offset + idx] for candidate in self.all_cover])
        np.save(self.matrix_path + str(time_slot) + '.npy', matrix) # Save final matrix as numpy binary file
        return True
        
    def ILP_generate_constraint_matrices(self):
        '''
        Generate constraint matrices for integer linear programming using parallel processing.
        
        Returns:
            bool: True if operation completed successfully
        '''
        # Set output directory and create if needed
        mkdir(self.matrix_path)
        
        # Identify locations with non-zero demand
        self.nonzero_cid=[]
        for idx,demand in enumerate(self.demand_list):
            if isinstance(demand, dict):
                if demand['density']>0:
                    self.nonzero_cid.append(idx)
            else:
                if demand>0:
                    self.nonzero_cid.append(idx)
        # Load all candidate coverage texture
        self.all_cover=[]
        full_cover_files=get_allfile(self.texture_save_path)
        for file in tqdm(full_cover_files,desc="Load texture library"):
            tmp_data=np.load(file,allow_pickle=True)
            self.all_cover.append(tmp_data[2][0])
        
        # Process all time slots in parallel
        time_list=list(range(self.time_split))
        with mp.Pool(processes=self.num_processes) as pool:
            r = list(tqdm(pool.imap(self._generate_matrix_single_slot, time_list), total=len(time_list), mininterval=10,maxinterval=20,desc="Generate constraint_matrices"))
            pool.close()  # Prevents any more tasks from being submitted to the pool
            pool.join()  # Wait for the worker processes to exit
        return True

    def ILP_solver(self,MIPGap=0.15,warm_start_path=None):
        '''
        Solve the satellite constellation optimization problem using Gurobi's MIP solver.
        Args:
            MIPGap (float): Relative optimality gap tolerance (default: 0.15=15%)
            warm_start_path (str, optional): Path to warm start solution file
        Note:
            Saves results as a dictionary mapping variable names to values
        '''

        m = gp.Model("mip1")# Initialize Gurobi model
        
        # Create binary variables for each candidate satellite
        full_cover_files=get_allfile(self.texture_save_path)
        for file in tqdm(full_cover_files):
            params=file.split("/")[-1].strip(".npy")
            m.addVar(vtype=GRB.BINARY, name=f'x_{params}')
        m.update()
        check_var=m.getVars()
        print("The number of variables:",len(check_var))
        
        # Set objective: minimize total number of satellites
        exp=gp.quicksum(m.getVars())
        m.setObjective(exp, GRB.MINIMIZE)
        m.update()
        
        # Prepare demand constraints
        if isinstance(demand, dict):
            constrain_demand=[item['density']for item in self.demand_list if item['density']>0]
        else:
            constrain_demand=[item for item in self.demand_list if item >0]
            
        
        # Add coverage constraints for each time slot
        for time_slot in tqdm(range(self.time_split)):
            slot_matrix=np.load(self.matrix_path+str(time_slot)+".npy",allow_pickle=True)
            m.addMConstr(slot_matrix,m.getVars(),'>',constrain_demand)
            del slot_matrix
            
        # Configure solver parameters
        m.Params.MIPGap = MIPGap
        m.Params.MIPFocus=1 # Focus on finding feasible solutions
        m.Params.Heuristics=0.2
        m.Params.Cuts=3
        m.Params.Presolve=2
        m.setParam('OutputFlag', 1) # Enable solver output
        m.setParam('Threads', self.num_processes) # Parallel threads
        m.setParam("ConcurrentMIP", 1)
        m.update()  
        constrs = m.getConstrs()
        print("Length of constraints:", len(constrs))
        del constrs
        
        # Optional warm start from previous solution
        if warm_start_path!=None:
            model_start=np.load(warm_start_path,allow_pickle=True).item()
            for var in tqdm(m.getVars()):
                var.Start = model_start[var.VarName]  # 一个 dict
            m.update()
        # Solve and save results
        try:
            m.optimize()
            print(f"Obj: {m.ObjVal:g}") # Print objective value
            for v in m.getVars():
                if v.X!=0:
                    print(f"{v.VarName} {v.X:g}")
            # Print and save non-zero variables
            variables_dict = {v.VarName: v.X for v in m.getVars() if v.X!=0}
            np.save(self.lp_results_filename,variables_dict)
        
        except gp.GurobiError as e:
            print(f"Error code {e.errno}: {e}")
        
        except AttributeError:
            print("Encountered an attribute error")
        return True
        