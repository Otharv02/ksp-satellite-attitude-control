import numpy as np

def calculate_correction_axis(current, target):
    try:
        current = np.array(current, dtype=float)
        target = np.array(target, dtype=float)

        # Normalize both vectors
        current_norm = current / np.linalg.norm(current)
        target_norm = target / np.linalg.norm(target)

        # Calculate rotation axis (cross product)
        axis = np.cross(current_norm, target_norm)
        axis_norm = np.linalg.norm(axis)

        # Calculate angle between vectors using dot product
        dot = np.clip(np.dot(current_norm, target_norm), -1.0, 1.0)
        angle_rad = np.arccos(dot)

        if axis_norm < 1e-6 or angle_rad < 1e-3:
            return (0.0, 0.0, 0.0), 0.0  # Almost aligned

        axis_unit = axis / axis_norm
        return tuple(axis_unit), angle_rad

    except Exception as e:
        print(f"[Correction Axis Error] {e}")
        return (0.0, 0.0, 0.0), 0.0
