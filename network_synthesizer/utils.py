import os
import numpy as np
import math
from fractions import Fraction

# Constant
TE=24*3600 #[s] Earth's rotation period
RE=6371e3 #[m] Earth's equatorial radius (6,371 km converted to meters)
u= 3.986e14 # Standard gravitational parameter (μ = G*M)
K=RE/pow(u,1/3)*pow(2*np.pi,2/3) 
eps=25*np.pi/180 #Minimum elevation angle for UE visibility,25° elevation angle converted to radians

def mkdir(path):
    '''
    Create a directory if it does not exist.

    Args:
        path (str): The path of the directory to create.

    Returns:
        None
    '''
    folder = os.path.exists(path)
    if not folder:                   # Determine whether a folder exists. If no folder exists, create it as a folder
        os.makedirs(path)      

def get_allfile(path):  
    '''
    Get all file paths in the specified directory, excluding Jupyter checkpoint files.

    Args:
        path (str): Path to the directory to scan.

    Returns:
        list[str]: A list of file paths in the directory.
    '''
    all_file = []
    for f in os.listdir(path):  
        f_name = os.path.join(path, f)
        if "ipynb_checkpoints" not in f_name:
            all_file.append(f_name)
    return all_file

def satellite_period(h):
        """
        Calculate the orbital period of a satellite at a given altitude.
    
        Args:
            h (float): Altitude above Earth's surface in meters.
    
        Returns:
            float: Orbital period in seconds.
        """
        a=RE+h
        T=float(2*np.pi*pow(a**3/u,0.5))
        return T
    
def coverage_eta(T):
        """
        Compute the coverage angle eta (in radians) as a function of satellite period.
    
        Args:
            T (float): Orbital period of the satellite in seconds.
    
        Returns:
            float: Coverage angle eta in radians.
        """
        eta=math.acos(K*math.cos(eps)/pow(T,2/3))-eps 
        return eta #radian
def approximate_ratio(T, precision=1e-3):
    """
    Calculate the approximate minimum integer ratio of two numbers within a given precision.

    Args:
        T (int): Numerator value.
        precision (float, optional): Allowed relative error in the ratio approximation. 
                                     Default is 1e-3.

    Returns:
        Tuple[int, int]: The simplified integer ratio (numerator, denominator).

    """
    # Calculate fractions and simplify
    ratio = Fraction(T, TE).limit_denominator(int(1/precision))
    return ratio.numerator, ratio.denominator