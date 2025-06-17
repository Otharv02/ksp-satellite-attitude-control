import time 
import yaml
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

pid_config = config["pid"]
pitch_pid = PID(**pid_config["pitch"])
yaw_pid   = PID(**pid_config["yaw"])
roll_pid  = PID(**pid_config["roll"])

deadzone = config.get("deadzone_angle_deg", 0.5)
previous_time = time.time()




# #PID controller for pitch, roll, yaw 
# pitch_pid = PID(kp=1.5, ki=1, kd=2)
# yaw_pid   = PID(kp=1.5, ki=1, kd=2)
# roll_pid  = PID(kp=1.5, ki=1, kd=2)



# previous_time       = time.time()

try:
    while True:
        current_time    = time.time()
        dt              = current_time - previous_time 
        previous_time   = current_time

        # calculating the magnitude of the vector 
        # (calculating currection axis)
    
        telemetry = get_data_from_vessel(conn,vessel)
        angle_error = telemetry["angle_to_target_deg"]
        
        correction_axis =calculate_correction_axis(
            telemetry["camera_dir_world"],
            telemetry["target_dir"]
        )

        # Dead zone 
        if angle_error < 0.5:
            apply_controls(vessel, 0, 0, 0)
            continue
        
        correction_axis_local = conn.space_center.transform_direction(
            correction_axis,                      # world/surface-frame vector
            vessel.surface_reference_frame,       # FROM surface frame
            vessel.reference_frame                # TO vessel local control frame
        )

        # Angular velocity damping 
        ang_vel = telemetry["angular_velocity"]


        # Map axes and apply signs
        pitch_control = -pitch_pid.update(correction_axis_local[1] - ang_vel[1], dt)
        yaw_control   = -yaw_pid.update(correction_axis_local[2] - ang_vel[2], dt)
        roll_control  = -roll_pid.update(correction_axis_local[0] - ang_vel[0], dt)




        # pitch_control = -pitch_pid.update(correction_axis[0],dt)
        # yaw_control = yaw_pid.update(correction_axis[1],dt)
        # roll_control = roll_pid.update(correction_axis[2],dt)

        # send inputs to vessel 
        apply_controls(vessel, pitch_control, yaw_control, roll_control)

except Exception as e:
    print("Control Stopped due to error:")
    print(e)
    apply_controls(vessel, 0, 0, 0)