

import time 
import math
from krpc_client.telemetry_reader import connect_to_ksp, get_data_from_vessel
from controllers.pid_controller import PID
from controllers.math_utils import calculate_correction_axis
from actuators.control_output import apply_controls

# Connect to KSP
conn = connect_to_ksp()
vessel = conn.space_center.active_vessel

# PID Configs (inline now)
pitch_pid = PID(kp=0.6, ki=0.0, kd=0.5) # 
yaw_pid   = PID(kp=0.6, ki=0.0, kd=0.5) # 
roll_pid  = PID(kp=0.6, ki=0.0, kd=0.5)

# Tunable parameters
deadzone = 1.5           # degrees
max_output = 0.4         # max control surface authority
previous_time = time.time()

# Clamp function
def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))

try:
    while True:
        current_time = time.time()
        dt = current_time - previous_time 
        previous_time = current_time

        if dt < 0.001 or dt > 0.1:
            time.sleep(0.01)
            continue

        # Get telemetry from vessel
        telemetry = get_data_from_vessel(conn, vessel)
        angle_error = telemetry["angle_to_target_deg"]

        # Stop correcting if within deadzone
        if angle_error < deadzone:
            apply_controls(vessel, 0, 0, 0)
            continue

        # Get rotation axis and magnitude
        correction_axis, angle_error_rad = calculate_correction_axis(
            telemetry["camera_dir_world"],
            telemetry["target_dir"]
        )

        # Convert correction direction to vessel local space
        correction_axis_local = conn.space_center.transform_direction(
            correction_axis,                      # global correction axis
            vessel.surface_reference_frame,       # FROM world frame
            vessel.reference_frame                # TO vessel-local frame
        )

        # Get vessel's angular velocity
        ang_vel = telemetry["angular_velocity"]

        # Map correction to pitch/yaw/roll axes
        pitch_error = correction_axis_local[0] * angle_error_rad
        yaw_error   = correction_axis_local[1] * angle_error_rad
        roll_error  = correction_axis_local[2] * angle_error_rad

        # Add Damping (basic derivative control)
        pitch_error_damped = pitch_error - ang_vel[0] * 0.2
        yaw_error_damped   = yaw_error   - ang_vel[1] * 0.2
        roll_error_damped  = roll_error  - ang_vel[2] * 0.2

        # PID Update
        pitch_control = -pitch_pid.update(pitch_error_damped, dt)
        yaw_control   = -yaw_pid.update(yaw_error_damped, dt)
        roll_control  = -roll_pid.update(roll_error_damped, dt)

        # Clamp final outputs
        pitch_control = clamp(pitch_control, -max_output, max_output)
        yaw_control   = clamp(yaw_control, -max_output, max_output)
        roll_control  = clamp(roll_control, -max_output, max_output)

        # Apply control input
        apply_controls(vessel, pitch_control, yaw_control, roll_control)

        # Debug print (every 2 seconds)
        if int(current_time) % 2 == 0:
            print(f"Angle Error: {angle_error:.2f}°, "
                  f"Controls: P={pitch_control:.3f}, Y={yaw_control:.3f}, R={roll_control:.3f}")

except Exception as e:
    print("Control Stopped due to error:")
    print(e)
    apply_controls(vessel, 0, 0, 0)
