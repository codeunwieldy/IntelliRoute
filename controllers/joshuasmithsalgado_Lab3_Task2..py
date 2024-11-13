# Import necessary libraries
from fairis_tools.my_robot import MyRobot
import time
import math

# Initialize robot
robot = MyRobot()
robot.axel_length = 0.265
robot.load_environment('../../worlds/Fall24/maze6.xml')  
robot.move_to_start()
# PID controller constants
kp_distance = 8.0  # theses are really just for the forawrd movment 
ki_distance = 0.01
kd_distance = 7.0

# Initialize variables for PID control
integral_rotation = 0
previous_error_rotation = 0
integral_distance = 0
previous_error_distance = 0
dt = 0.032  # Control loop timestep
desired_distance = .5  # Desired distance from the wall
previous_error = 0
integral = 0

# Target distance from the goal
target_goal_distance = .5  # Target stop distance of 0.5 meters from goal

# Robot states
SEARCHING_GOAL = "searching_goal"
MOVING_TO_GOAL = "moving_to_goal"
AVOIDING_OBSTACLE = "avoiding_obstacle"
FOLLOWING_WALL = "following_wall"
current_state = SEARCHING_GOAL
goal_reached = False
goal_lost_counter = 0
goal_lost_threshold = 100  # Number of timesteps to move straight before following the wall
moving_straight = False


# Detect goal function
def detect_goal():
    # Enable recognition if not already enabled
    
    robot.rgb_camera.recognitionEnable(robot.timestep)

    # Get recognized objects
    rec_objects = robot.rgb_camera.getRecognitionObjects()

    # Debug print to check if we get recognized objects
    print(f"Number of recognized objects: {len(rec_objects)}")
    if(rec_objects ==0):
        return 0
        

    # Iterate through detected objects
    for obj in rec_objects:
        # Convert the ctypes color array to a Python list
        object_color = [obj.getColors()[i] for i in range(3)]
        
        # Debug print for checking detected colors
        print(f"Detected Object Color: {object_color}")

        # Check if the detected object color is close to yellow (normalized values)
        if 0.8 <= object_color[0] <= 1.0 and 0.8 <= object_color[1] <= 1.0 and 0.0 <= object_color[2] <= 0.2:
            
            print("Landmark detected")

            # Extracting object properties and converting to lists
            """
            object_id = obj.getId()
            object_position = [obj.getPosition()[i] for i in range(3)]
            object_size = [obj.getSize()[i] for i in range(2)]
            object_position_on_image = [obj.getPositionOnImage()[i] for i in range(2)]
            object_size_on_image = [obj.getSizeOnImage()[i] for i in range(2)]

            # Debug print statements
            print("Object ID:", object_id)
            print("Object Position (X, Y, Z):", object_position)
            print("Object Size (Y, Z):", object_size)
            print("Object Position on Image (X, Y):", object_position_on_image)
            print("Object Size on Image (X, Y):", object_size_on_image)
            """
            return obj  # Return the detected goal object
            

    # Return None if no valid object is detected
    return None


# Motion to goal
# Motion to goal with forward PID control
def motion_to_goal(goal):
    # Ensure goal is not None and has a valid position on image
    
    if goal is None or goal.getPositionOnImage() is None:
        return False  # Exit if goal is not valid

    # Extract the x position of the goal on the image
    object_position_on_image = goal.getPositionOnImage()
    if len(object_position_on_image) < 2:
        return False  # Exit if the position is not as expected

    goal_x = object_position_on_image[0]

    # Get the width of the camera's image in pixels
    image_width = robot.rgb_camera.getWidth()
    width = image_width[0] if isinstance(image_width, list) else image_width
    image_center_x = width / 2

    # Rotating to align with the goal
    if abs(goal_x - image_center_x) < 10:  # If the goal is centered within a margin
        
        
        goal_position = goal.getPosition()
        goal_distance = math.sqrt(goal_position[0] ** 2 + goal_position[2] ** 2)

        print("goal distance is ",{goal_distance-.337})
        
        
        # Move towards the goal once the rotation is complete
        if move_towards_goal(goal_distance-.337): #for some reason its consistatnly .337 off so i just out that there to counter it 
            print("Goal reached at 0.5 meters.")
            return True  # Goal reached
    else:
        # Rotate to align the robot with the goal
        error = goal_x - image_center_x
    
        # If the error is positive, rotate left; if negative, rotate right
        if error > 0:
            rotational_velocity = 15  # Rotate left at a fixed speed
        else:
            rotational_velocity = -15  # Rotate right at a fixed speed
        
        robot.set_right_motors_velocity(-rotational_velocity)
        robot.set_left_motors_velocity(rotational_velocity)
        print(f"Aligning with goal: rotational velocity = {rotational_velocity}")

    return False
def move_towards_goal(current_distance):
    print("going forward toward goal")
    global integral_distance, previous_error_distance

    error = current_distance - target_goal_distance
    integral_distance += error * dt
    derivative = (error - previous_error_distance) / dt
    forward_velocity = kp_distance * error + ki_distance * integral_distance + kd_distance * derivative

    # Limit the forward velocity to a maximum of 10
    forward_velocity = max(min(forward_velocity, 10), -10)

    robot.set_right_motors_velocity(forward_velocity)
    robot.set_left_motors_velocity(forward_velocity)
    previous_error_distance = error

    # Adjust stopping threshold based on target distance
    stop_threshold = 0.01 

    # Return True if the robot is within the target distance of the goal (indicating that the goal has been reached)
    return abs(error) < stop_threshold




# Wall following
def wall_following(robot, desired_distance, previous_error, integral, dt, mode='left'):
    print("Current state is wall following")
    lidar_data = robot.get_lidar_range_image()
    front_distance = lidar_data[400]
    side_distance = sum(lidar_data[550:601]) / 51 if mode == 'right' else sum(lidar_data[200:251]) / 51
    
    #if the robot encounters obsticals in front of it while wall following t will turn the appropriate way 
    if front_distance < 0.5 and mode == 'left':  # Avoid obstacle
        robot.set_left_motors_velocity(5)
        robot.set_right_motors_velocity(-5)
        return False
    elif front_distance < 0.5 and mode == 'right':
        robot.set_left_motors_velocity(-5)
        robot.set_right_motors_velocity(5)
        return False


    # PID Error Calculation
    error = desired_distance - side_distance
    integral += error * dt
    derivative = (error - previous_error) / dt

    # Limit the integral to prevent windup
    integral = max(min(integral, 10), -10)

    # PID Correction Calculation
    kp_distance = 2.0  
    ki_distance = 0.01
    kd_distance = 0.5 

    correction = kp_distance * error + ki_distance * integral + kd_distance * derivative

    # Limit the correction value to prevent excessive speeds
    correction = max(min(correction, 10), -10)

    # Calculate Motor Speeds
    max_speed = 12.0
    if mode == 'right':
        left_speed = max(0, min(max_speed, max_speed - correction))
        right_speed = max(0, min(max_speed, max_speed + correction))
    else:
        left_speed = max(0, min(max_speed, max_speed + correction))
        right_speed = max(0, min(max_speed, max_speed - correction))

    # Set Motor Velocities
    print(f'Left speed: {left_speed:.2f}, Right speed: {right_speed:.2f}')
    robot.set_left_motors_velocity(left_speed)
    robot.set_right_motors_velocity(right_speed)

    # Update previous error for next iteration
    previous_error = error
    
    return True



# Obstacle detection
def obstacle_avoidance():
    # Get LIDAR data
    lidar_data = robot.get_lidar_range_image()

    # Check a range of indices for front, left-front, and right-front
    #this prevents the robot from coliding while going forward towards an obect at a sharp angle
    
    front_distance = lidar_data[400]  # Directly in front
    left_front_distance = min(lidar_data[320:400])  # Slightly to the left
    right_front_distance = min(lidar_data[400:500])  # Slightly to the right

    # Threshold for detecting obstacles
    detection_threshold = 0.4

    # Return True if any of these sensors detect an obstacle within 0.4 meters
    return (
        front_distance < detection_threshold or
        left_front_distance < detection_threshold or
        right_front_distance < detection_threshold
    )


# Main control loop
start_time = time.time()  # Start the timer outside the loop for later use
while robot.experiment_supervisor.step(robot.timestep) != -1:
    elapsed_time = time.time() - start_time
    lidar_data = robot.get_lidar_range_image()
    front_distance = lidar_data[400]
    print("Front distance is ",{front_distance})
    goal = detect_goal()
    goal_distance = math.sqrt(goal.getPosition()[0] ** 2 + goal.getPosition()[2] ** 2) if goal else float('inf')
    

    if current_state == SEARCHING_GOAL:
        if goal:
            current_state = MOVING_TO_GOAL
        else:
            print("Searching for goal, rotating...")
            robot.set_left_motors_velocity(10)
            robot.set_right_motors_velocity(-10)

            # Switch to FOLLOWING_WALL if no goal is detected
            if not detect_goal():
                current_state = FOLLOWING_WALL

    elif current_state == MOVING_TO_GOAL:
        # Check if the robot needs to avoid an obstacle
        if obstacle_avoidance() and (goal_distance - 0.337) > 1.05:
            recognized_goal = detect_goal()
            if recognized_goal:
                current_state = MOVING_TO_GOAL
            else:
                current_state = AVOIDING_OBSTACLE
        elif motion_to_goal(goal):  # Move towards the goal
            print("Goal reached successfully at 0.5 meters.")
            robot.set_left_motors_velocity(0)
            robot.set_right_motors_velocity(0)
            break  # Exit loop when goal is reached
        elif not goal:  # If the goal is lost during movement
            print("Goal lost, moving straight to continue search...")
            goal_lost_counter += 1  # updates the lost counter so we can kinda create a timer when the goal is lost 
            print("Goal lost for ",{goal_lost_counter}," cycles")
            moving_straight = True
            robot.set_left_motors_velocity(10)
            robot.set_right_motors_velocity(10)
        
            # Continue moving straight for a limited duration, then switch to wall-following
            if goal_lost_counter > goal_lost_threshold:
                goal_lost_counter = 0
                moving_straight = False
                current_state = FOLLOWING_WALL
        else:
            
            moving_straight = False


    elif current_state == FOLLOWING_WALL:
        if goal:
            if not obstacle_avoidance(): #if no obsticals in front then we are moving to the goal if the goal is detected 
                current_state = MOVING_TO_GOAL
        elif not wall_following(robot, desired_distance, previous_error, integral, dt):
            current_state = AVOIDING_OBSTACLE

    elif current_state == AVOIDING_OBSTACLE:
        if wall_following(robot, desired_distance, previous_error, integral, dt):
            current_state = FOLLOWING_WALL

print(f'Total time is: {elapsed_time:.2f} seconds') # prints the total time




