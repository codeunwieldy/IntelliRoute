# Import necessary libraries
from fairis_tools.my_robot import MyRobot
import time
import math

# Initialize robot
robot = MyRobot()
robot.axel_length = 0.265
robot.load_environment('../../worlds/Fall24/maze5.xml')  

# PID controller constants
kp_distance = 10.0
ki_distance = 0.01
kd_distance = 7.0

# Initialize variables for PID control
integral_rotation = 0
previous_error_rotation = 0
integral_distance = 0
previous_error_distance = 0
dt = 0.032  # Control loop timestep
goal_reached = False

# Target distance from the goal
target_goal_distance = 0.5  # Target stop distance of 0.5 meters from goal

# Robot states
SEARCHING_GOAL = "searching_goal"
MOVING_TO_GOAL = "moving_to_goal"
AVOIDING_OBSTACLE = "avoiding_obstacle"
current_state = SEARCHING_GOAL



# PID functions
def rotate_to_goal(goal_x, image_center_x):
    print("Rotating toward goal")
    
    # Calculate the error (difference between goal position and image center)
    error = goal_x - image_center_x
    
    # If the error is positive, rotate left; if negative, rotate right
    if error > 0:
        rotational_velocity = 7  # Rotate left at a fixed speed
    else:
        rotational_velocity = -7  # Rotate right at a fixed speed
    
    # Set the motor velocities to rotate in place
    robot.set_right_motors_velocity(-rotational_velocity)
    robot.set_left_motors_velocity(rotational_velocity)
    
    # Print the current rotational velocity for debugging
    print("Rotational velocity:", rotational_velocity)
    
    # Return True if the error is within an acceptable range (indicating the goal is centered)
    return abs(error) < 20


#basic PID control for forward stoppind 
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

    # Return True if the robot is within the target distance of the goal (indicating that the goal has been reached)
    return abs(error) < 0.01


def motion_to_goal(goal):
    # Extract the necessary properties from the goal object
    object_position_on_image = [goal.getPositionOnImage()[i] for i in range(2)]
    goal_x = object_position_on_image[0]
    
    
    # Get the width of the camera's image in pixels
    image_width = robot.rgb_camera.getWidth()
    if isinstance(image_width, list):
        width = image_width[0]
    else:  # If it's a scalar
        width = image_width
        
    # Calculate the X center of the image (middle of the image width)
    image_center_x = width / 2
    print(f"Image center X: {image_center_x}")
    
    # Rotate towards the goal
    if rotate_to_goal(goal_x, image_center_x):
        # Extract 3D position of the object (goal)
        goal_position = goal.getPosition()
        goal_distance = math.sqrt(goal_position[0] ** 2 + goal_position[2] ** 2)
        print("goal distance is ",{goal_distance})
        
        # Move towards the goal once the rotation is complete
        if move_towards_goal(goal_distance):
            print("Goal reached at 0.5 meters.")
            return True  # Goal reached
    return False


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

            return obj  # Return the detected goal object
            

    # Return None if no valid object is detected
    return False




# Main Control Loop



while robot.experiment_supervisor.step(robot.timestep) != -1:
    
    if detect_goal() == 0:
        current_state = SEARCHING_GOAL

    if current_state == SEARCHING_GOAL:
        print("Searching for landmarks...")
        
        # Attempt to detect the goal
        goal = detect_goal()

        if goal:
            print("Goal detected, starting motion to goal...")
            current_state = MOVING_TO_GOAL
        else:
            print("No goal detected, rotating to search...")
            robot.set_right_motors_velocity(7.0)
            robot.set_left_motors_velocity(-7.0)

    elif current_state == MOVING_TO_GOAL:
        print("Moving towards goal...")
        # Pass the detected goal to motion_to_goal
        goal_reached = motion_to_goal(goal)

        if goal_reached:
            print("Goal reached successfully.")
            robot.set_right_motors_velocity(0.0)
            robot.set_left_motors_velocity(0.0)
            break

   
