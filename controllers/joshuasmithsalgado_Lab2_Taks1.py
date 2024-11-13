# Import MyRobot Class
from fairis_tools.my_robot import MyRobot
import math
import matplotlib.pyplot as plt
import time

# Create the robot instance.
robot = MyRobot()
robot.axel_length = 0.265
# Loads the environment from the maze file
maze_file = '../../worlds/Fall24/maze2.xml'
robot.load_environment(maze_file)

# PID Controller Function
# PID control constants
kp = 10  # can be used for high velocity at a distance 
ki = 0.01  # risky to put at zero but it gets there with minimal error
kd = 7  # gain for smooth deceleration 
stopping_distance = 1.0  # 1 meter away from the wall
dt = 0.032  # Timestep
previous_error = 0
integral = 0

def pid_control(robot, kp, ki, kd, desired_distance, previous_error, integral, dt=0.032):
    # Get the front distance from the LIDAR sensor
    lidar_data = robot.get_lidar_range_image()
    front_distance = lidar_data[400]  # Index 400 is the front

    # Calculate the error (difference between current and desired distance)
    error = front_distance - desired_distance

    # Calculate the integral and derivative terms
    integral += error * dt
    derivative = (error - previous_error) / dt

    # Compute the PID output (motor velocity)
    motor_velocity = kp * abs(error) + ki * integral + kd * derivative

    # Determine direction: move forward if too far, backward if too close
    if error < 0:
        motor_velocity = -motor_velocity  # Reverse direction when too close

    # Saturate motor velocity within a certain range
    max_velocity = 10.0  # Maximum allowable velocity
    min_velocity = -10.0  # Minimum allowable velocity (for backing up)
    motor_velocity = max(min(motor_velocity, max_velocity), min_velocity)

    # Apply the velocity to both motors (for forward or backward movement)
    robot.set_right_motors_velocity(motor_velocity)
    robot.set_left_motors_velocity(motor_velocity)
    print("Motor velocity is", motor_velocity)

    # Update the previous error for the next iteration
    previous_error = error

    # Return updated integral and previous error
    return previous_error, integral, front_distance

"""
def wall_following_control(robot, kp, ki, kd, desired_distance, previous_error, integral, dt, mode):
    # Get LIDAR data
    lidar_data = robot.get_lidar_range_image()
    
    # Average distances from LIDAR readings
    if mode == 'left':
        side_distance_reading = sum(lidar_data[200:251]) / 51  # Average left side readings
        print("Following left wall")
    else:
        side_distance_reading = sum(lidar_data[550:601]) / 51  # Average right side readings
        print("Following right wall")

    front_distance_reading = lidar_data[400]  # Front distance reading

    # Calculate the error for wall distance
    error = desired_distance - side_distance_reading  # Positive means too far, negative means too close

    # Implement a deadband to reduce occilasion 
    deadband = 0.1  # +/- 0.1 meters
    if abs(error) < deadband:
        error = 0  # No correction needed if within deadband

    # PID control logic
    integral += error * dt
    derivative = (error - previous_error) / dt

    # Adjust PID output
    correction = kp * error + ki * integral + kd * derivative

    # Set motor velocities based on the correction
    max_velocity = 10.0  # High maximum velocity for quicker responses
    turning_radius = 0.5  # Adjust turning radius for smoother turns

    # Check if there's an obstacle in front
    if front_distance_reading < 0.4:  # If obstacle detected, rotate
        if mode == 'right':
            # Rotate left if following the right wall
            robot.set_right_motors_velocity(max_velocity / 4)  # Rotate left slowly
            robot.set_left_motors_velocity(-max_velocity / 4)
        else:
            # Rotate right if following the left wall
            robot.set_right_motors_velocity(-max_velocity / 4)  # Rotate right slowly
            robot.set_left_motors_velocity(max_velocity / 4)

       
        return previous_error, integral  # Return to allow main loop to proceed

    # Adjust velocities for wall following
    if mode == 'left':
        right_motor_speed = max_velocity - correction * turning_radius
        left_motor_speed = max_velocity
    else:  # mode == 'right'
        right_motor_speed = max_velocity
        left_motor_speed = max_velocity - correction * turning_radius

    # Ensure motor speeds are non-negative
    right_motor_speed = max(0, right_motor_speed)
    left_motor_speed = max(0, left_motor_speed)

    # Wall distance-based speed adjustments
    if mode == 'right':
        if side_distance_reading < 0.27:  # Too close to the right wall
            left_motor_speed = max_velocity * 0.3  # Slow down left motor to steer away
            right_motor_speed = max_velocity
        elif side_distance_reading > 0.6:  # Too far from the right wall
            left_motor_speed = max_velocity  # Speed up left motor to approach the wall
            right_motor_speed = max_velocity * 0.5  # Slow down right motor
    else:  # mode == 'left'
        if side_distance_reading < 0.27:  # Too close to the left wall
            right_motor_speed = max_velocity * 0.3  # Slow down right motor to steer away
            left_motor_speed = max_velocity
        elif side_distance_reading > 0.6:  # Too far from the left wall
            right_motor_speed = max_velocity  # Speed up right motor to approach the wall
            left_motor_speed = max_velocity * 0.5  # Slow down left motor

    # Set motor speeds
    robot.set_right_motors_velocity(right_motor_speed)
    print(f"Right motor speed: {right_motor_speed}")
    robot.set_left_motors_velocity(left_motor_speed)
    print(f"Left motor speed: {left_motor_speed}")

    # Update previous error
    previous_error = error

    return previous_error, integral
    """



# RobotPositionTracker class (already provided)
class RobotPositionTracker:
    def __init__(self, initial_x, initial_y, initial_heading_rad):
        self.x = initial_x
        self.y = initial_y
        self.heading_rad = initial_heading_rad  # in radians
        self.last_left_distance = 0
        self.last_right_distance = 0

    def update_position(self, left_distance, right_distance, theta_rad):
        delta_left = left_distance - self.last_left_distance
        delta_right = right_distance - self.last_right_distance

        self.last_left_distance = left_distance
        self.last_right_distance = right_distance

        average_distance = (delta_left + delta_right) / 2.0

        self.heading_rad = theta_rad
        self.x += average_distance * math.cos(self.heading_rad)
        self.y += average_distance * math.sin(self.heading_rad)

    def get_position(self):
        return (self.x, self.y, self.heading_rad)

# Initialize tracker with starting position
tracker = RobotPositionTracker(2.0, -2.0, math.pi)  # Start at (2.0, -2.0) with heading π radians

# Update function for position
def control_loop_pose(left_distance, right_distance, theta_rad):
    tracker.update_position(left_distance, right_distance, theta_rad)
    current_position = tracker.get_position()
    print(f"Current Position: {current_position}")
    return current_position

def get_heading_from_compass(compass_values):
    # Extract the x and y components from the compass values
    x = compass_values[0]
    z = compass_values[1]  # Compass is on the x-y plane
    
    # Calculate the heading in radians (swap x and z for proper orientation)
    heading_radians = math.atan2(x, z)  # Swap the order to fix the reversed directions
    
    # Convert radians to degrees
    heading_degrees = math.degrees(heading_radians)
    
    # Normalize the heading to be between 0 and 360 degrees
    if heading_degrees < 0:
        heading_degrees += 360

    return heading_degrees


# Move robot to a random starting position listed in maze file
robot.move_to_start()

desired_wall_distance = 0.45  # Maintain 0.5 meters from the wall
previous_error_wall_follow = 0
integral_wall_follow = 0
start_time = time.time()  # Start the timer outside the loop for later use
# Main Control Loop for Robot
while robot.experiment_supervisor.step(robot.timestep) != -1:
    currentCompass = robot.compass.getValues()
    heading = get_heading_from_compass(currentCompass)
    theta = math.radians(heading)
    lidar_data = robot.get_lidar_range_image()
    front_distance = lidar_data[400]  # Index 400 is the front
    left_distance = lidar_data[200]  
    right_distance = lidar_data[600]  

    print("Distance to front wall is", front_distance)
    #print("Distance to left wall is", left_distance)
   #print("Distance to right wall is", right_distance)

    # Reads and Prints Robot's Encoder Readings
    print("Motor Encoder Readings:", robot.get_encoder_readings())

    # Reads and Prints Robot's Lidar Readings Relative to Robot's Position
    print("Current Time", robot.experiment_supervisor.getTime())

    # Calculates distance the wheel has turned since the beginning of simulation
    distance_front_left_wheel_traveled = robot.wheel_radius * robot.get_front_left_motor_encoder_reading()
    distance_front_right_wheel_traveled = robot.wheel_radius * robot.get_front_right_motor_encoder_reading()
    
    # Update real-time position based on encoder and compass readings
    calculated_pose = control_loop_pose(distance_front_left_wheel_traveled, distance_front_right_wheel_traveled, theta)
    
    print("Wheel distance traveled is", distance_front_left_wheel_traveled)
     

    previous_error, integral, front_distance = pid_control(robot, kp, ki, kd, stopping_distance, previous_error, integral, dt)
    # Stop the robot if it's within the 1-meter range (with some tolerance)
    if abs(front_distance - stopping_distance) < 0.01:  # 5 cm tolerance
        robot.set_right_motors_velocity(0)
        robot.set_left_motors_velocity(0)
        print("Robot has stopped 1 meter from the wall.")
        break
    """
    mode = 'left' #change this to right or left depending on what wall you want to follow
    
    previous_error_wall_follow, integral_wall_follow = wall_following_control(
        robot, kp, ki, kd, desired_wall_distance, previous_error_wall_follow, integral_wall_follow, dt, mode
    )
    print(f"Robots current pos: {calculated_pose}")
    # Check if the robot reached the goal (within 0.5 meters tolerance)
    goal_x, goal_y = -1.0, 1.0  # Goal position
    if abs(calculated_pose[0] - goal_x) < 0.15 and abs(calculated_pose[1] - goal_y) < 0.15:
        robot.set_right_motors_velocity(0)
        robot.set_left_motors_velocity(0)
        print("Goal reached!")
        elapsed_time = time.time() - start_time #get the totoal time for the program 
        # Print the completion time based on the mode (left or right wall)
        if mode == 'left':
            print(f"Left wall completion time: {elapsed_time:.2f} seconds")
        else:
            print(f"Right wall completion time: {elapsed_time:.2f} seconds")
        break
   """


    
   
  
   
   
   
   