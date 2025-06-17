import time 
import yaml
import math
from krpc_client.telemetry_reader import connect_to_ksp, get_data_from_vessel
from controllers.pid_controller import PID
from controllers.math_utils import calculate_correction_axis
from actuators.control_output import apply_controls

# load config 
with open("config.yaml", "r") as f:
    config = yaml.safe_load(f)

# connect to ksp
conn                = connect_to_ksp()
vessel              = conn.space_center.active_vessel

# PID config
pid_config  = config["pid"]
pitch_pid   = PID(**pid_config["pitch"])
yaw_pid     = PID(**pid_config["yaw"])
roll_pid    = PID(**pid_config["roll"])

deadzone = config.get("deadzone_angle_deg", 2.0)
max_output = config.get("max_control_output", 0.5)
previous_time = time.time()




# #PID controller for pitch, roll, yaw 
# pitch_pid = PID(kp=1.5, ki=1, kd=2)
# yaw_pid   = PID(kp=1.5, ki=1, kd=2)
# roll_pid  = PID(kp=1.5, ki=1, kd=2)


# testing 
# Clamp function
def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))


try:
    while True:
        current_time    = time.time()
        dt              = current_time - previous_time 
        previous_time   = current_time

        if dt < 0.001 or dt > 0.1:
            continue

        # calculating the magnitude of the vector 
        # (calculating currection axis)
    
        telemetry = get_data_from_vessel(conn,vessel)
        angle_error = telemetry["angle_to_target_deg"]
        

        # Dead zone 
        if angle_error < deadzone:
            apply_controls(vessel, 0, 0, 0)
            break

        

        correction_axis, angle_error_rad = calculate_correction_axis(
            telemetry["camera_dir_world"],
            telemetry["target_dir"]
        )
        

        # correction_axis =calculate_correction_axis(
        #     telemetry["camera_dir_world"],
        #     telemetry["target_dir"]
        # )

        # transform to vessel local frame 
        correction_axis_local = conn.space_center.transform_direction(
            correction_axis,                      # world/surface-frame vector
            vessel.surface_reference_frame,       # FROM surface frame
            vessel.reference_frame                # TO vessel local control frame
        )

        # Angular velocity damping 
        ang_vel = telemetry["angular_velocity"]

        # angle_error_rad = math.radians(angle_error)


        # Correct axis mapping for KSP:
        # Pitch = rotation around X-axis (rightward)
        # Yaw   = rotation around Y-axis (upward)  
        # Roll  = rotation around Z-axis (forward)

        pitch_error = correction_axis_local[0] * angle_error_rad # x-axis
        yaw_error = correction_axis_local[1] * angle_error_rad # Y-axis
        roll_error = correction_axis_local[2] * angle_error_rad # Z-axis

        # Add angular velocity damping (subtract current angular velocity)
        pitch_error_damped = pitch_error - ang_vel[0] * 0.2  # X angular velocity
        yaw_error_damped = yaw_error - ang_vel[1] * 0.2      # Y angular velocity  
        roll_error_damped = roll_error - ang_vel[2] * 0.2    # Z angular velocity

        # Map axes and apply signs
        pitch_control = -pitch_pid.update(pitch_error_damped, dt)
        yaw_control   = -yaw_pid.update(yaw_error_damped, dt)
        roll_control  = -roll_pid.update(roll_error_damped, dt)

        # Clamp output
        pitch_control = clamp(pitch_control, -max_output, max_output)
        yaw_control   = clamp(yaw_control, -max_output, max_output)
        roll_control  = clamp(roll_control, -max_output, max_output)


        # send inputs to vessel 
        apply_controls(vessel, pitch_control, yaw_control, roll_control)

        # Debug output (optional)
        if int(current_time) % 2 == 0:  # Print every ~2 seconds
            print(f"Angle Error: {angle_error:.2f}°, " f"Controls: P={pitch_control:.3f}, Y={yaw_control:.3f}, R={roll_control:.3f}")

        


except Exception as e:
    print("Control Stopped due to error:")
    print(e)
    apply_controls(vessel, 0, 0, 0)