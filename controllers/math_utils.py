import numpy as np 

def calculate_correction_axis(current, target):
    current     = np.array(current)
    target      = np.array(target)
    axis        = np.cross(current, target)
    norm        = np.linalg.norm(axis)      # calculates the magnitude (length) of the axis vector
    
    return tuple(axis/norm) if norm!=0 else (0.0, 0.0, 0.0)