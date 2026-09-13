import time
from orca_core.hardware.feetech_client import FeetechClient

port = '/dev/cu.usbmodem5B790775661'  # <-- Replace with your actual port
motor_id = 1
baudrate = 1_000_000

print(f'Connecting to servo ID {motor_id} on {port}...')
client = FeetechClient(motor_ids=[motor_id], port=port, baudrate=baudrate)

try:
    client.connect()
    print('Connected successfully!')

    # 1. Read initial telemetry
    read = client.read_position_velocity_current()
    temps = client.read_temperature()
    initial_pos = read.position[0]
    print(f'Initial Position: {initial_pos:.3f} rad | Temperature: {temps[0]} C')

    # 2. Enable torque
    print('Enabling torque...')
    client.set_torque_enabled([motor_id], True)
    time.sleep(0.5)

    # 3. Small gentle motion test (+0.2 rad / ~11 degrees)
    target_pos = initial_pos + 0.2
    print(f'Moving to target: {target_pos:.3f} rad...')
    client.write_desired_pos([motor_id], target_pos)
    time.sleep(1.0)

    # 4. Read new position
    read_after = client.read_position_velocity_current()
    print(f'New Position: {read_after.position[0]:.3f} rad')

    # 5. Return back to starting position
    print(f'Returning to initial position: {initial_pos:.3f} rad...')
    client.write_desired_pos([motor_id], initial_pos)
    time.sleep(1.0)

    print('Test passed successfully!')
finally:
    # 6. Always disable torque and disconnect safely
    client.set_torque_enabled([motor_id], False)
    client.disconnect()
    print('Disconnected.')
