"""
Example to move ROV in straight line, with depth hold
"""

# Import mavutil
from pymavlink import mavutil
import time
import sys

# Create the connection
master = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Choose a mode
#Try: ['STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'CIRCLE', 'SURFACE', 'POSHOLD', 'MANUAL']
mode = 'ALT_HOLD'

# Check if mode is available
if mode not in master.mode_mapping():
    print('Unknown mode : {}'.format(mode))
    print('Try:', list(master.mode_mapping().keys()))
    sys.exit(1)

# Get mode ID
mode_id = master.mode_mapping()[mode]
# Set new mode
# master.mav.command_long_send(
#    master.target_system, master.target_component,
#    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
#    0, mode_id, 0, 0, 0, 0, 0) or:
# master.set_mode(mode_id) or:
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)

while True:
    # Wait for ACK command
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    ack_msg = ack_msg.to_dict()

    # Check if command in the same in `set_mode`
    if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
        continue

    # Print the ACK result !
    print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
    break

#arming ROV
def is_armed():
    try:
        return bool(master.wait_heartbeat().base_mode & 0b10000000)
    except:
        return False


while not is_armed():
    master.arducopter_arm()

# Create a function to send RC values
# More information about Joystick channels
# here: https://www.ardusub.com/operators-manual/rc-input-and-output.html#rc-inputs
#channel_ID:
#5....forward & reverse---> 1900 "forward" till 1500 "stop" till 1100 "revsrese"
#6.... right or left---> 1900 "right" till 1500 "stop" till 1100 "left"
def set_rc_channel_pwm(channel_id, pwm=1500):
    """ Set RC channel pwm value
    Args:
        channel_id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if channel_id < 1 or channel_id > 18:
        print("Channel does not exist.")
        return

    rc_channel_values = [65535 for _ in range(8)]
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_channel_values)


while True:
    try:
        set_rc_channel_pwm(5, 1500) #x-axis control
        set_rc_channel_pwm(6, 1500) #y-axis control
        set_rc_channel_pwm(3, 1500) #z-axis control
        time.sleep(0.2)
        print('command sent')
    except Exception as error:
        print(error)
        sys.exit(0)
