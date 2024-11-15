# Import MyRobot Class
from fairis_tools.my_robot import MyRobot
import math
import time
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import norm
robot = MyRobot()
robot.load_environment('../../worlds/Fall24/maze7.xml')


# Robot parameters
WHEEL_RADIUS = 0.043  # meters
AXLE_LENGTH = 0.265   # meters
point_turn_velocity = 5
forward_velocity = 15

class map_cell:
    def __init__(self, index, wall_config):
        self.index = index
        self.visited = False
        self.wall_config = wall_config  # Format: [West, North, East, South]
        self.heading = 0

    def mark_visited(self):
        self.visited = True

    def get_visited(self):
        return self.visited

    def get_wall_config(self):
        return self.wall_config

    def set_heading(self, heading):
        self.heading = heading

    def get_heading(self):
        return self.heading

# Initialize map_layout list
map_layout = []

# Create wall configurations for cells 1 to 25
for index in range(1, 26):
    west_wall = 'W' if index % 5 == 1 else 'O'  # West walls for cells 1, 6, 11, 16, 21
    east_wall = 'W' if index % 5 == 0 else 'O'  # East walls for cells 5, 10, 15, 20, 25
    north_wall = 'W' if 1 <= index <= 5 else 'O'  # North walls for cells 1 to 5
    south_wall = 'W' if 21 <= index <= 25 else 'O'  # South walls for cells 21 to 25

    # Define the wall configuration in [West, North, East, South] order
    wall_config = [west_wall, north_wall, east_wall, south_wall]

    # Create a map_cell object and add it to map_layout
    cell = map_cell(index, wall_config)
    map_layout.append(cell)

# Print the wall configuration of each cell to verify
for cell in map_layout:
    print(f"Cell {cell.index}: Walls - {cell.get_wall_config()}")

    
   

    
# Landmarks position [(x, y), color]
landmarks = [                                           #im not sure if this lines up properly with the way we are creating the calculations for the precomputed_distances
    ((-2.5, 2.5), "yellow"), ((2.5, 2.5), "red"),
    ((2.5, -2.5), "blue"), ((-2.5, -2.5), "green")
]

# Precomputed distances from grid cell centers (1 to 25) to landmarks
precomputed_distances = np.zeros((25, 4))
for cell in range(25):
    cx, cy = (cell % 5) - 2, (cell // 5) - 2  # Convert cell index to (x, y) in the grid
    for idx, ((lx, ly), _) in enumerate(landmarks):
        precomputed_distances[cell][idx] = math.sqrt((cx - lx) ** 2 + (cy - ly) ** 2)

# Calculate normal probability density function
def calculate_normdist(s, mu, sigma):
    if s is None or mu is None:
        return 0.0  # Return 0 probability if either value is None
    coefficient = 1 / (math.sqrt(2 * math.pi * sigma ** 2))
    exponent = -((s - mu) ** 2) / (2 * sigma ** 2)
    return coefficient * math.exp(exponent)

# Pose estimation function
def estimate_pose(observed_distances):
    sigma = 1.0  # Standard deviation for measurement noise
    probabilities = []
    for cell in range(25):
        prob = 1.0
        valid_measurements = 0
        for idx in range(4):  # For each landmark
            s = precomputed_distances[cell][idx]
            mu = observed_distances[idx]
            if mu is not None:
                prob *= calculate_normdist(s, mu, sigma)
                valid_measurements += 1
        
        # Only consider this cell if we have at least one valid measurement
        if valid_measurements > 0:
            # Normalize the probability based on the number of valid measurements
            prob = prob ** (1 / valid_measurements)
            probabilities.append(prob)
        else:
            probabilities.append(0.0)
    
    # Return the index (cell number) with the highest probability
    if max(probabilities) > 0:
        max_prob_cell = np.argmax(probabilities)
        return max_prob_cell + 1
    else:
        return None  # Return None if no valid probabilities were calculated


def compare_gps_with_pose(positon,calculated_pose):
     gps_x = positon[0]
     gps_y = positon[1]
     cpose_x = calculated_pose[0]
     cpose_y = calculated_pose[1]
     cheading = calculated_pose[2]

     # Calculate distance from GPS position to calculated pose
     dx = gps_x - cpose_x
     dy = gps_y - cpose_y
     distance = math.sqrt(dx**2 + dy**2)

     # Print the comparison results
     print(f"GPS Position: ({gps_x:.2f}, {gps_y:.2f}), Calculated Pose: ({cpose_x:.2f}, {cpose_y:.2f})")
     print(f"Difference in the distance: {distance:.2f} m, Heading is the same, calculated with IMU: {cheading:.2f} rad")

# Function to perform a point turn
def pointTurn(target_degree):
    # Get the initial heading from the compass
    
    print("point turn in progress")
    if target_degree < 0:
        # Clockwise turn
        robot.set_right_motors_velocity(-point_turn_velocity)
        robot.set_left_motors_velocity(point_turn_velocity)
        print(f"Current Left wheel velocity is {point_turn_velocity} rad/s")
        print(f"Current Right wheel velocity is -{point_turn_velocity} rad/s")
    else:
        # Counterclockwise turn
        
        robot.set_right_motors_velocity(point_turn_velocity)
        robot.set_left_motors_velocity(-point_turn_velocity)
        print(f"Current Left wheel velocity is -{point_turn_velocity} rad/s")
        print(f"Current Right wheel velocity is {point_turn_velocity} rad/s")
        
def forward(): # sets robot at max speed forward
     robot.set_right_motors_velocity(forward_velocity)
     robot.set_left_motors_velocity(forward_velocity)
     print(f"Current Left wheel velocity is {forward_velocity} rad/s")
     print(f"Current Right wheel velocity is {forward_velocity} rad/s")

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

def center_on_goal(goal):
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
        return True
        
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
    
     
    



def detect_landmarks():
    # Enable recognition if not already enabled
    robot.rgb_camera.recognitionEnable(robot.timestep)

    # Get recognized objects
    rec_objects = robot.rgb_camera.getRecognitionObjects()
    
    # Debug print to check if we get recognized objects
    print(f"Number of recognized objects: {len(rec_objects)}")
    if len(rec_objects) == 0:
        return [None, None, None, None]  # Return a list of None if no objects are detected

    # Colors we're looking for (normalized RGB values)
    COLORS = {
        "red": [0.9, 0.0, 0.0],    # Approximate red color
        "green": [0.0, 0.9, 0.0],  # Approximate green color
        "blue": [0.0, 0.0, 0.9],   # Approximate blue color
        "yellow": [0.9, 0.9, 0.0]  # Approximate yellow color
    }

    # List to store the detected distances of each color
    observed_distances = [None, None, None, None]  # [red, green, blue, yellow]

    # Iterate through detected objects
    for obj in rec_objects:
        # Convert the ctypes color array to a Python list
        object_color = [obj.getColors()[i] for i in range(3)]
        
        # Debug print for checking detected colors
        print(f"Detected Object Color: {object_color}")

        # Check if the detected object color matches any of our target colors
        # Using simple thresholds to identify the color
        if 0.8 <= object_color[0] <= 1.0 and 0.0 <= object_color[1] <= 0.2 and 0.0 <= object_color[2] <= 0.2:
            # Detected red landmark
            print("Red Landmark detected")
            observed_distances[0] = (math.sqrt(obj.getPosition()[0] ** 2 + obj.getPosition()[2] ** 2))-0.337 if obj else float('inf')

        elif 0.0 <= object_color[0] <= 0.2 and 0.8 <= object_color[1] <= 1.0 and 0.0 <= object_color[2] <= 0.2:
            # Detected green landmark
            print("Green Landmark detected")
            observed_distances[1] = (math.sqrt(obj.getPosition()[0] ** 2 + obj.getPosition()[2] ** 2))-0.337 if obj else float('inf')

        elif 0.0 <= object_color[0] <= 0.2 and 0.0 <= object_color[1] <= 0.2 and 0.8 <= object_color[2] <= 1.0:
            # Detected blue landmark
            print("Blue Landmark detected")
            observed_distances[2] = (math.sqrt(obj.getPosition()[0] ** 2 + obj.getPosition()[2] ** 2))-0.337 if obj else float('inf')

        elif 0.8 <= object_color[0] <= 1.0 and 0.8 <= object_color[1] <= 1.0 and 0.0 <= object_color[2] <= 0.2:
            # Detected yellow landmark
            print("Yellow Landmark detected")
            observed_distances[3] = (math.sqrt(obj.getPosition()[0] ** 2 + obj.getPosition()[2] ** 2))-0.337 if obj else float('inf')

    # Return the distances list [red, green, blue, yellow]
    return observed_distances


def initial_localization():
    landmarks_detected = []
    for _ in range(4):  # Rotate 4 times, 90 degrees each
        pointTurn(90)
        time.sleep(1)  # Allow time for sensors to stabilize
        observed_distances = detect_landmarks()
        landmarks_detected.append(observed_distances)  # Append as a list
    
    # Combine the observed distances, taking the minimum non-None value for each landmark
    combined_distances = []
    for i in range(4):  # For each landmark
        distances = [ld[i] for ld in landmarks_detected if ld[i] is not None]
        if distances:
            combined_distances.append(min(distances))
        else:
            combined_distances.append(None)
    
    estimated_cell = estimate_pose(combined_distances)
    if estimated_cell is not None:
        return estimated_cell
    else:
        print("Unable to estimate initial position. Not enough landmarks detected.")
        return 1  # Return a default cell (e.g., cell 1) if estimation fails

# Initialize robot and perform initial localization
robot.move_to_start()
current_cell = initial_localization()                 #//////////////////////////the point turn function does not have a way to stop rotating rn so fix that 
if current_cell is not None:
    map_layout[current_cell - 1].mark_visited()
    print(f"We have started in cell {current_cell} based on the initial scan")
else:
    print("Initial localization failed. Starting from a default position.")
    current_cell = 1  # Or any other default starting cell


# Main control loop
start_time = time.time()  # Start the timer outside the loop for later use
while robot.experiment_supervisor.step(robot.timestep) != -1:
    elapsed_time = time.time() - start_time
    lidar_data = robot.get_lidar_range_image()
    front_distance = lidar_data[400]
    print("Front distance is ",{front_distance})
        # Get sensor readings
    currentCompass = robot.compass.getValues()
    compass_heading = get_heading_from_compass(currentCompass)
    lidar_data = robot.get_lidar_range_image()
    front_distance = lidar_data[400]
    observed_distances = detect_landmarks()  # Assuming this function returns distance for 4 landmarks
    print("Observed distances for landmarks (Red, Green, Blue, Yellow):", observed_distances)
    estimated_cell = estimate_pose(observed_distances) #estimate of the grid we are currently in 
    print(f"Estimated Cell: {estimated_cell}")


    if estimated_cell == 25:
        print("Reached target cell.")
        robot.set_right_motors_velocity(0)
        robot.set_left_motors_velocity(0)
        break



