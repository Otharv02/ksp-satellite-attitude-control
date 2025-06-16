"""
Using an equatorial orbit instead of a polar orbit.
In a polar orbit, the camera's direction changes relative to the surface as the satellite moves,
making it hard to keep the camera pointed down. In an equatorial orbit, the orientation stays
more stable, so it's easier to control and keep the camera facing the ground.
"""

import krpc
import math 


def connect_to_ksp():
    return krpc.connect(name = "LEO-Sat")


def get_data_from_vessel(conn,vessel):


    # helper fun (for angle between the camera_dir_world and target_direction)
    def get_angle(v1, v2):
        dot  = sum(a * b for a, b in zip(v1,v2))
        mag1 = math.sqrt(sum(a * a for a in v1))
        mag2 = math.sqrt(sum(b * b for b in v2))
        return math.acos(dot/(mag1 * mag2)) # radians
    
    ref_frame           = vessel.surface_reference_frame  # orientation relative to the planet’s surface.
    current_direction   = vessel.direction(ref_frame)

    # Target Direction 
    camera_dir          = (0, 0, 1) # orientation of camera 
    
    # To transform camera direction from vessel frame to surface direction (Where the camera is pointing in global world frame)
    camera_dir_world    = conn.space_center.transform_direction (
        camera_dir,                       # (0,0,-1) pointing up
        vessel.reference_frame,           # local vector from here
        ref_frame                         # Transform to this 
    ) 

    target_dir          = (0, 0, -1) # straight down

    angular_velocity    = vessel.angular_velocity(ref_frame)

    angle_error         = get_angle(camera_dir_world, target_dir)

    return {
        "current_direction"   : current_direction,
        "camera_dir_world"    : camera_dir_world,
        "target_dir"          : target_dir,
        "angular_velocity"    : angular_velocity,
        "angle_to_target_deg" : math.degrees(angle_error)
    }