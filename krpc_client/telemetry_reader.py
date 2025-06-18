"""
Using an equatorial orbit instead of a polar orbit.
In a polar orbit, the camera's direction changes relative to the surface as the satellite moves,
making it hard to keep the camera pointed down. In an equatorial orbit, the orientation stays
more stable, so it's easier to control and keep the camera facing the ground.
"""

import krpc
import math 

# Camera orientation in the vessel's local reference frame (+Y is "up" on most probe cores)
camera_local_dir = (0, 1, 0)

def connect_to_ksp():
    return krpc.connect(name="LEO-Sat")

def normalize(vec):
    mag = math.sqrt(sum(x**2 for x in vec))
    return tuple(x / mag for x in vec)

def angle_between(v1, v2):
    dot  = sum(a * b for a, b in zip(v1, v2))
    mag1 = math.sqrt(sum(a * a for a in v1))
    mag2 = math.sqrt(sum(b * b for b in v2))
    return math.acos(dot / (mag1 * mag2))  # radians

def get_data_from_vessel(conn, vessel):
    # Reference frame where directions make physical sense (planet-centered inertial frame)
    ref_frame = vessel.orbit.body.reference_frame

    # Compute satellite's position in world coordinates
    position_world = vessel.position(ref_frame)

    # Compute normalized direction toward planet center (i.e., nadir)
    target_dir = tuple(-x for x in normalize(position_world))

    # Transform camera direction from vessel-local to world frame
    camera_dir_world = conn.space_center.transform_direction(
        camera_local_dir,
        vessel.reference_frame,     # from vessel-local frame
        ref_frame                   # to world/planet frame
    )

    # Compute angle between camera and target direction
    angle_error = angle_between(camera_dir_world, target_dir)

    # Optional: Get vessel's angular velocity for stabilization control
    angular_velocity = vessel.angular_velocity(ref_frame)

    # Optional (if needed): vessel.direction in this frame
    # current_direction = vessel.direction(ref_frame)

    return {
        # "current_direction": current_direction,  # Uncomment if needed
        "camera_dir_world": camera_dir_world,
        "target_dir": target_dir,
        "angular_velocity": angular_velocity,
        "angle_to_target_deg": math.degrees(angle_error)
    }
