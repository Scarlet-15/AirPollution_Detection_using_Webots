"""
Mavic 2 PRO - Autonomous Pollution Detection Mission
Fixed crash issue and optimized for aerial pollution detection
"""

from controller import Robot, Camera, GPS, InertialUnit, Gyro, LED
import math

def clamp(value, low, high):
    return max(low, min(value, high))

# Initialize robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Enable devices
camera = robot.getDevice("camera")
camera.enable(timestep)

front_left_led = robot.getDevice("front left led")
front_right_led = robot.getDevice("front right led")

imu = robot.getDevice("inertial unit")
imu.enable(timestep)

gps = robot.getDevice("gps")
gps.enable(timestep)

gyro = robot.getDevice("gyro")
gyro.enable(timestep)

camera_roll = robot.getDevice("camera roll")
camera_pitch = robot.getDevice("camera pitch")

# Motors
front_left_motor = robot.getDevice("front left propeller")
front_right_motor = robot.getDevice("front right propeller")
rear_left_motor = robot.getDevice("rear left propeller")
rear_right_motor = robot.getDevice("rear right propeller")

motors = [front_left_motor, front_right_motor, rear_left_motor, rear_right_motor]

for m in motors:
    m.setPosition(float('inf'))
    m.setVelocity(1.0)

# ==================== CONFIGURATION ====================
WAYPOINTS = [
    (0, 0, 8),
    (-5, -10, 8),
    (0, -12, 8),
    (8, -5, 8),
    (8, 2, 8),
    (0, 8, 8),
    (-12, 8, 8),
    (-5, 5, 8),
    (0, 0, 8),
]

TARGET_ALTITUDE = 8.0
WAYPOINT_THRESHOLD = 1.5  # Reduced for tighter waypoint following
RED_THRESHOLD = 0.08  # Slightly reduced for better detection

# PID CONSTANTS (from working manual code)
k_vertical_thrust = 68.5
k_vertical_offset = 0.6
k_vertical_p = 3.0
k_roll_p = 50.0
k_pitch_p = 30.0

# Position control gains - REDUCED to prevent aggressive movements
k_position_p = 0.7  # Reduced from 0.5
max_tilt = 1  # Reduced maximum tilt angle

# State variables
state = "INIT"
waypoint_idx = 0
pollution_list = []
target_x, target_y, target_z = 0.0, 0.0, TARGET_ALTITUDE
past_time = 0.0
past_x_global = 0.0
past_y_global = 0.0
past_altitude = 0.0
initial_altitude = 0.0
stabilize_counter = 0
pollution_check_counter = 0

print("="*70)
print("MAVIC 2 PRO - POLLUTION DETECTION MISSION")
print("="*70)
print(f"Total waypoints: {len(WAYPOINTS)}")
print(f"Target altitude: {TARGET_ALTITUDE}m")
print("="*70 + "\n")

def detect_pollution():
    """Detect red pollution sources in camera"""
    img = camera.getImage()
    if img is None:
        return False
    
    width = camera.getWidth()
    height = camera.getHeight()
    red_count = 0
    total_count = 0
    
    # Sample center region more thoroughly
    for y in range(height // 3, 2 * height // 3, 2):
        for x in range(width // 3, 2 * width // 3, 2):
            r = camera.imageGetRed(img, width, x, y)
            g = camera.imageGetGreen(img, width, x, y)
            b = camera.imageGetBlue(img, width, x, y)
            
            # Detect bright red (more lenient thresholds)
            if r > 180 and g < 120 and b < 120:
                red_count += 1
            total_count += 1
    
    if total_count == 0:
        return False
    
    red_ratio = red_count / total_count
    return red_ratio > RED_THRESHOLD

def is_new_location(x, y, existing_list, min_distance=5.0):
    """Check if location is new"""
    for prev_x, prev_y, _ in existing_list:
        dist = math.sqrt((x - prev_x)**2 + (y - prev_y)**2)
        if dist < min_distance:
            return False
    return True

# Wait for sensors to initialize
print("Initializing sensors...")
for _ in range(10):
    robot.step(timestep)

position = gps.getValues()
initial_altitude = position[2]
print(f"Initial position: X={position[0]:.2f}, Y={position[1]:.2f}, Z={position[2]:.2f}")
print(f"Ground level: {initial_altitude:.2f}m")
print("Starting mission...\n")

# ==================== MAIN CONTROL LOOP ====================
while robot.step(timestep) != -1:
    
    time = robot.getTime()
    dt = time - past_time
    
    # Read sensor values
    position = gps.getValues()
    roll, pitch, yaw = imu.getRollPitchYaw()
    roll_velocity, pitch_velocity, yaw_velocity = gyro.getValues()
    
    x_global = position[0]
    y_global = position[1]
    altitude = position[2] - initial_altitude  # Relative altitude
    
    # Calculate velocities
    if dt > 0:
        x_velocity = (x_global - past_x_global) / dt
        y_velocity = (y_global - past_y_global) / dt
        altitude_velocity = (altitude - past_altitude) / dt
    else:
        x_velocity = 0
        y_velocity = 0
        altitude_velocity = 0
    
    # LED blinking
    led_state = int(time) % 2
    front_left_led.set(led_state)
    front_right_led.set(1 - led_state)
    
    # Camera stabilization
    cam_roll_target = clamp(-0.115 * roll_velocity, -0.5, 0.5)
    cam_pitch_target = clamp(-0.1 * pitch_velocity, -0.5, 0.5)
    camera_roll.setPosition(cam_roll_target)
    camera_pitch.setPosition(cam_pitch_target)
    
    # Calculate horizontal velocity (used in multiple states)
    horizontal_velocity = math.sqrt(x_velocity**2 + y_velocity**2)
    
    # ==================== STATE MACHINE ====================
    if state == "INIT":
        target_x, target_y, target_z = 0, 0, TARGET_ALTITUDE
        if time > 1.0:
            state = "TAKEOFF"
            print("State: TAKEOFF - Initiating climb\n")
    
    elif state == "TAKEOFF":
        target_x, target_y, target_z = 0, 0, TARGET_ALTITUDE
        
        if altitude > TARGET_ALTITUDE - 0.5:
            state = "STABILIZE"
            stabilize_counter = 0
            print(f"State: STABILIZE - Reached {altitude:.2f}m\n")
    
    elif state == "STABILIZE":
        target_x, target_y, target_z = 0, 0, TARGET_ALTITUDE
        stabilize_counter += 1
        
        if stabilize_counter > int(3000 / timestep):
            state = "NAVIGATE"
            waypoint_idx = 0
            target_x, target_y, _ = WAYPOINTS[waypoint_idx]
            target_z = TARGET_ALTITUDE
            print("="*70)
            print("State: NAVIGATE - Starting waypoint mission")
            print("="*70)
            for i, wp in enumerate(WAYPOINTS):
                print(f"  WP{i+1}: ({wp[0]:.1f}, {wp[1]:.1f}, {wp[2]:.1f})")
            print("="*70 + "\n")
            print(f"→ Heading to Waypoint 1\n")
    
    
    elif state == "NAVIGATE":
        target_z = TARGET_ALTITUDE
        
        dist_to_target = math.sqrt((target_x - x_global)**2 + (target_y - y_global)**2)
        
        if dist_to_target < WAYPOINT_THRESHOLD and horizontal_velocity < 1.0:
            print(f"✓ Waypoint {waypoint_idx + 1}/{len(WAYPOINTS)} reached")
            print(f"  Position: ({x_global:.2f}, {y_global:.2f}, {altitude:.2f})")
            print(f"  Distance error: {dist_to_target:.2f}m\n")
            
            waypoint_idx += 1
            
            if waypoint_idx >= len(WAYPOINTS):
                state = "RETURN_HOME"
                target_x, target_y, target_z = 0, 0, TARGET_ALTITUDE
                print("="*70)
                print(f"All waypoints complete! Found {len(pollution_list)} pollution sources")
                print("="*70)
                print("State: RETURN_HOME - Heading to start position\n")
            else:
                target_x, target_y, _ = WAYPOINTS[waypoint_idx]
                target_z = TARGET_ALTITUDE
                print(f"→ Heading to Waypoint {waypoint_idx + 1}\n")
        
        # Check for pollution (every 10 timesteps to reduce false positives)
        pollution_check_counter += 1
        if pollution_check_counter >= 10:
            pollution_check_counter = 0
            if detect_pollution():
                if is_new_location(x_global, y_global, pollution_list):
                    pollution_list.append((x_global, y_global, altitude))
                    print("\n" + "!"*70)
                    print(f"!!! POLLUTION SOURCE #{len(pollution_list)} DETECTED !!!")
                    print(f"!!! Location: ({x_global:.2f}, {y_global:.2f}, {altitude:.2f}) !!!")
                    print("!"*70 + "\n")
    
    elif state == "RETURN_HOME":
        target_x, target_y, target_z = 0, 0, TARGET_ALTITUDE
        
        dist_to_home = math.sqrt(x_global**2 + y_global**2)
        
        if dist_to_home < 1.5:
            state = "LAND"
            target_x, target_y, target_z = 0, 0, 0.3
            print("State: LAND - Descending to ground\n")
    
    elif state == "LAND":
        target_x, target_y, target_z = 0, 0, 0.3
        
        if altitude < 0.5:
            print("\n" + "="*70)
            print("LANDING COMPLETE")
            print("="*70)
            print(f"Mission time: {time:.1f}s")
            print(f"Pollution sources detected: {len(pollution_list)}")
            if pollution_list:
                print("\nDetection locations:")
                for i, (px, py, pz) in enumerate(pollution_list):
                    print(f"  {i+1}. X={px:.2f}, Y={py:.2f}, Z={pz:.2f}")
            print("="*70 + "\n")
            
            # Stop all motors
            for m in motors:
                m.setVelocity(0)
            break
    
    # ==================== CONTROL CALCULATIONS ====================
    
    # Position errors
    x_error = target_x - x_global
    y_error = target_y - y_global
    
    # Transform to body frame for roll/pitch control
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    x_error_body = x_error * cos_yaw + y_error * sin_yaw
    y_error_body = -x_error * sin_yaw + y_error * cos_yaw
    
    # Velocity damping in body frame
    x_velocity_body = x_velocity * cos_yaw + y_velocity * sin_yaw
    y_velocity_body = -x_velocity * sin_yaw + y_velocity * cos_yaw
    
    # Desired roll and pitch based on position error WITH velocity damping
    if state == "TAKEOFF" or state == "STABILIZE":
        roll_disturbance = 0
        pitch_disturbance = 0
    else:
        # PD control for position (P term + D term for damping)
        roll_disturbance = k_position_p * clamp(y_error_body, -3.0, 3.0) - 0.3 * y_velocity_body
        pitch_disturbance = k_position_p * clamp(-x_error_body, -3.0, 3.0) - 0.3 * x_velocity_body
        
        # Clamp to safe tilt angles
        roll_disturbance = clamp(roll_disturbance, -max_tilt, max_tilt)
        pitch_disturbance = clamp(pitch_disturbance, -max_tilt, max_tilt)
    
    # Stabilization (from working code)
    roll_input = k_roll_p * clamp(roll, -1, 1) + roll_velocity + roll_disturbance
    pitch_input = k_pitch_p * clamp(pitch, -1, 1) + pitch_velocity + pitch_disturbance
    yaw_input = 0
    
    # Altitude control (from working code)
    diff_alt = clamp(target_z - altitude + k_vertical_offset, -1, 1)
    vertical_input = k_vertical_p * (diff_alt ** 3)
    
    # Motor mixing (EXACT from working code)
    fl = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
    fr = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input
    rl = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
    rr = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input
    
    # Apply velocities (EXACT motor directions from working code)
    front_left_motor.setVelocity(fl)
    front_right_motor.setVelocity(-fr)
    rear_left_motor.setVelocity(-rl)
    rear_right_motor.setVelocity(rr)
    
    # Debug output
    if int(time * 10) % 30 == 0:
        print(f"[{time:.1f}s] Alt: {altitude:.2f}m | Pos: ({x_global:.2f}, {y_global:.2f}) | "
              f"Vel: {horizontal_velocity:.2f}m/s | State: {state}")
    
    # Update past values
    past_time = time
    past_x_global = x_global
    past_y_global = y_global
    past_altitude = altitude

print("\nMission terminated.")