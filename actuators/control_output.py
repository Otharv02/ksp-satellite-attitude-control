
def apply_controls(vessel, pitch, yaw, roll):
    vessel.control.pitch   = max(min(pitch, 1), -1)
    vessel.control.yaw     = max(min(yaw, 1), -1)
    vessel.control.roll     = max(min(roll, 1), -1)