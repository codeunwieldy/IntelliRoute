# Import MyRobot Class
from fairis_tools.my_robot import MyRobot
import math
import matplotlib.pyplot as plt

# Create the robot instance.
robot = MyRobot()
robot.axel_length = 0.265
# Loads the environment from the maze file
maze_file = '../../worlds/Fall24/maze1.xml'
robot.load_environment(maze_file)
Outer_wheel_velocity_ms = 0.3 #max meter a second for the outer wheel of a turn
forward_velocity = 15 # max forward velocity for accuracy when changing speeds suddenly 
point_turn_velocity = 1 # max point turn velocity for accuracy when monitoring what degree to stop at



estimated_x = []
estimated_y = []
gps_x = []
gps_y = []

# Function to update and plot the graph
def store_live_path(calculated_pose, xPos, yPos):
   
    estimated_x.append(calculated_pose[0])  # x value
    estimated_y.append(calculated_pose[1])  # y value
    
   
    gps_x.append(xPos)  # GPS x value
    gps_y.append(yPos)  # GPS y value


def TurningCalculations (radius, axel_length, max_mps, circle_p, wheel_radius):
    max_meters_per_s = max_mps  # Maximum speed of left wheel (in m/s) which is set at .8
    
    # Calculates the ICC given the know linear speed of the left wheel
    angular_speed_ICC = max_meters_per_s/(radius + (axel_length / 2))
    # Calculate the velocity of the right wheel
    right_W_Velocity = (radius - (axel_length / 2)) * angular_speed_ICC  # m/s for the right wheel during a right turn
   # robot_Linear_Velocity = (right_W_Velocity+max_meters_per_s)/2
    left_wheel_velocity = max_meters_per_s / wheel_radius
    right_W_Velocity = right_W_Velocity / wheel_radius  # Convert to rad/s for the motor
    distanceToTravel_leftW = ((radius + (axel_length / 2)) * circle_p * 2 * math.pi) 
    
    return left_wheel_velocity,right_W_Velocity,distanceToTravel_leftW




# Define waypoints
waypoints = [
    (2.0, -2.0, math.pi),
    (-1.5, -2.0, math.pi),
    (-2.0, -1.5, math.pi / 2),  # P2
    (-1.5, -1.0, 0),  # P3
    (1.5, -1.0, 0),
    (2.0, -0.5, math.pi / 4),
    (1.5, 1.0, 3 * math.pi / 4),
    (0.0, 0.0, math.pi),
    (-2.0, 0.0, math.pi / 2),
    (-2.0, 2.0, math.pi / 2),
    (1.5, 2.0, 0),   # P10
    (2.0, 1.5, math.pi),  # P11
    (1.5, 1.0, math.pi),  # P12
    (-1.0, 1.0, math.pi)   # P13 (Goal)
]

# Function to calculate distance, time, and angular velocities
def compute_trajectory(waypoints):
    wheel_velocities = []
    distances = []
    times = []
    total_time = 0  # adding to this to find the total time
    point_turning_times = []  # To store turning times
    total_distance = 0

    for i in range(len(waypoints) - 1):
        x0, y0, _ = waypoints[i]
        x1, y1, _ = waypoints[i + 1]
        D = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
        D-=.07 # this is because we are calculating from the center of the robot and this number works best at the current speed 
        # Determine turning times based on point index
        turning_times = 0
        if i in [4, 5, 6]:  # Points for left 45-degree turns
            turning_angle = math.pi / 4  # 45 degrees in radians
            turning_times = (robot.axel_length / 2) * turning_angle / (point_turn_velocity * robot.wheel_radius)
            Vli = -point_turn_velocity  # Right wheel positive for left turn
            Vri = point_turn_velocity  # Left wheel negative for left turn
        elif i in [8, 9]:  # Points for right 90-degree turns
            turning_angle = math.pi / 2  # 90 degrees in radians
            turning_times = (robot.axel_length / 2) * turning_angle / (point_turn_velocity * robot.wheel_radius)
            Vli = point_turn_velocity  # Right wheel positive for right turn
            Vri = -point_turn_velocity  # Left wheel negative for right turn

        if (i == 1 and i + 1 == 2) or (i == 2 and i + 1 == 3) or (i == 10 and i + 1 == 11) or (i == 11 and i + 1 == 12):
            # Circular arc calculation
            R = 0.5  # radius of circular path
            circle_p = 0.25  # proportion of the circle traveled (25%)
            Vli, Vri, left_wheel_distance = TurningCalculations(R, robot.axel_length, Outer_wheel_velocity_ms, circle_p, robot.wheel_radius)
            arc_length = R * 2 * math.pi * circle_p  # Arc length
            Va = (Vli+ Vri)/2
            T = arc_length / (Va* robot.wheel_radius)  # Using average of both wheels velocity 
            D = arc_length
        else:
            # Straight line segment
            Vli = forward_velocity  # Set left wheel velocity for straight segment
            Vri = forward_velocity  # Set right wheel velocity for straight segment
            T = D / (forward_velocity * robot.wheel_radius)  # Total time for the segment
        total_time_segment = T 
        total_time += (total_time_segment+turning_times)
        distances.append(D)
        total_distance += D
        times.append(total_time_segment)
        wheel_velocities.append((Vli, Vri))
        point_turning_times.append(turning_times)
        

    

    return wheel_velocities, distances, times, total_time, point_turning_times,total_distance
wheel_velocities, distances, times, total_time, point_turning_times,total_distance = compute_trajectory(waypoints)

# Print organized output
for i in range(len(wheel_velocities)):
    # Include turn information before the segment details
    if i in [4, 5, 6] or i in [8, 9]:  # Check if it's a turn
        
        print(f"Turn from Point {i} towards Point {i + 1}:")
        print(f"  Time for turn: { point_turning_times[i]:.2f} s")
        print("  Wheels turning at 1 rad/s")
        print()

    print(f"Segment from Point {i} to Point {i + 1}:")
    print(f"  Distance: {distances[i]:.2f} m")
    print(f"  Time: {times[i]:.2f} s")
    print(f"  Left Wheel Velocity: {wheel_velocities[i][0]:.2f} rad/s")
    print(f"  Right Wheel Velocity: {wheel_velocities[i][1]:.2f} rad/s")
    print()

print(f"Total time for the entire trajectory: {total_time:.2f} seconds")
print(f"Total distance for the entire trajectory: {total_distance:.2f} meters")





leftWheelDList = [3.43,5.39,8.37,8.93,9.44,12.77,14.92,18.545,20.518,23] # these are the actual values of the left wheel that the robot 
class Waypoint:                                                          # needs to follow because the correct computations arent acurate given  
    def __init__(self, distance, index):                                 # the iteration timing of the while loop 
        self.distance = distance
        self.pointTurn = False
        self.index = index
        self.visited = False  # Flag to check if waypoint has been visited
        self.endDegree = None  # To store the end degree of the point turn

    def mark_visited(self):
        self.visited = True

    def is_visited(self):
        return self.visited
    def mark_PTMade(self):
        self.pointTurn = True

    def didPointTurn(self):
        return self.pointTurn
    def set_end_degree(self, degree):
        self.endDegree = degree

    def get_end_degree(self):
        return self.endDegree
    
waypoints = [Waypoint(distance, index) for index, distance in enumerate(leftWheelDList)] # makes a list of the waypoint object with the calculated distance values, will keep track of where we are
def Turning (radius, axel_length, max_mps, circle_p, wheel_radius,direction):
    max_meters_per_s = max_mps  # Maximum speed of left wheel (in m/s) which is set at .8
    
    # Calculates the ICC given the know linear speed of the left wheel
    angular_speed_ICC = max_meters_per_s/(radius + (axel_length / 2))
    print("Angular Speed ICC:", angular_speed_ICC)
    
    # Calculate the velocity of the right wheel
    right_W_Velocity = (radius - (axel_length / 2)) * angular_speed_ICC  # m/s for the right wheel during a right turn
    robot_Linear_Velocity = (right_W_Velocity+max_meters_per_s)/2
    left_wheel_velocity = max_meters_per_s / wheel_radius
    right_W_Velocity = right_W_Velocity / wheel_radius  # Convert to rad/s for the motor
    print("Current Leftt Wheel Velocity (rad/s):", left_wheel_velocity)
    print("Current Right Wheel Velocity (rad/s):", right_W_Velocity)
    
    distanceToTravel = ((radius + (axel_length / 2)) * circle_p * 2 * math.pi) 
    print("Distance for left wheel to travel:", distanceToTravel)
    
    # Set velocities
    if direction==1: # for a right turn
        robot.set_right_motors_velocity(right_W_Velocity)  # Right wheel
        robot.set_left_motors_velocity(max_mps / wheel_radius)  # Left wheel
    else:
        robot.set_right_motors_velocity(max_mps / wheel_radius)  # Right wheel
        robot.set_left_motors_velocity(right_W_Velocity)  # Left wheel
    

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
     

     #calculates correct  heading given the webots compass values
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

class Points:   # class will have the waypoints, a visited attribute and the time is was visited so we can calculate the time while running the function in the main loop 
    def __init__(self, coordinates):
        self.coordinates = coordinates
        self.visited = False
        self.visited_time = None  # Time when the waypoint was visited

    def mark_visited(self, visit_time):
        self.visited = True
        self.visited_time = visit_time
points = [Points(coord) for coord in [ #creates the waypoints to keep track of
    (2.0, -2.0),      # P0
    (-1.5, -2.0),     # P1
    (-2.0, -1.5),     # P2
    (-1.5, -1.0),     # P3
    (1.5, -1.0),      # P4
    (2.0, -0.5),      # P5
    (1.5, 0.0),       # P6
    (0.0, 0.0),       # P7
    (-2.0, 0.0),      # P8
    (-2.0, 2.0),      # P9
    (1.5, 2.0),       # P10
    (2.0, 1.5),       # P11
    (1.5, 1.0),       # P12
    (-1.0, 1.0)       # P13
]]
def current_Point(position, current_simulation_time):
    # Extract x and y from position
    robot_x, robot_y = position[0], position[1]
    
    # Threshold for how close the robot must be to a point
    threshold = 0.1  

    for index, waypoint in enumerate(points):
        if not waypoint.visited:  # Only check unvisited waypoints
            # Calculate the distance from the robot's GPS to the waypoint
            distance = ((robot_x - waypoint.coordinates[0]) ** 2 + 
                        (robot_y - waypoint.coordinates[1]) ** 2) ** 0.5
            if distance < threshold:  # If the robot has reached the waypoint
                elapsed_time = 0
                if index > 0:  # Check if there is a previous waypoint
                    prev_waypoint = points[index - 1]
                    if prev_waypoint.visited_time is not None:
                        elapsed_time = current_simulation_time - prev_waypoint.visited_time
                    else:
                        print(f"Warning: Previous waypoint P{index - 1} has no visited time recorded.")
                
                waypoint.mark_visited(current_simulation_time)  # Mark as visited
                
                # Print information only if not transitioning from an unvisited initial waypoint
                if index > 0:
                    print(f"Robot has gone from point{index-1}: {points[index - 1].coordinates} to point{index}: {waypoint.coordinates} in {elapsed_time:.2f} seconds")
                
                return waypoint.coordinates  # Return the coordinates of the waypoint

    return None  # Return None if no points are reached


class RobotPositionTracker:
    def __init__(self, initial_x, initial_y, initial_heading_rad):
        self.x = initial_x
        self.y = initial_y
        self.heading_rad = initial_heading_rad  # in radians
        self.last_left_distance = 0
        self.last_right_distance = 0

    def update_position(self, left_distance, right_distance,theta_rad):
        # Calculate the differences in encoder readings
        delta_left = left_distance - self.last_left_distance
        delta_right = right_distance - self.last_right_distance
        
        # Update the last encoder readings
        self.last_left_distance = left_distance
        self.last_right_distance = right_distance
        
        # Calculate the average distance traveled
        average_distance = (delta_left + delta_right) / 2.0
        
        # Update the robot's heading
        self.heading_rad = theta_rad
        
        # Update the position based on the average distance and current heading
        self.x += average_distance * math.cos(self.heading_rad)
        self.y += average_distance * math.sin(self.heading_rad)
        
        # Optionally update heading to the new heading based on encoder differences
        # If you want to use theta_rad directly, make sure it represents the correct angle

    def get_position(self):
        return (self.x, self.y, self.heading_rad)

# Initialize tracker with starting position P0
tracker = RobotPositionTracker(2.0, -2.0, math.pi)  # π radians = 180 degrees

# usage in control loop
def control_loop_pose(left_distance, right_distance,theta_rad):
    tracker.update_position(left_distance, right_distance,theta_rad)
    current_position = tracker.get_position()
    print(f"Current Position based on encoder: {current_position}")
    return current_position
def compare_gps_with_pose(positon,calculated_pose,theta):
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
   
     

     
    





# Move robot to a random staring position listed in maze file
robot.move_to_start()

# Main Control Loop for Robot
while robot.experiment_supervisor.step(robot.timestep) != -1:




    # Reads and Prints Robot's Encoder Readings
    print("Motor Encoder Readings: ", robot.get_encoder_readings())

    # Reads and Prints Robot's Lidar Readings Relative to Robot's Position
    
    print("Current Time", robot.experiment_supervisor.getTime())

    
    

    # Calculates distance the wheel has turned since beginning of simulation
    distance_front_left_wheel_traveled = robot.wheel_radius * robot.get_front_left_motor_encoder_reading()
    distance_front_right_wheel_traveled = robot.wheel_radius * robot.get_front_right_motor_encoder_reading()
    print("distance left wheel travled", distance_front_left_wheel_traveled )
    robot.experiment_supervisor.getTime()
    position  =  robot.gps.getValues()
    currentCompass = robot.compass.getValues()
    heading = get_heading_from_compass(currentCompass)
    theta = math.radians(heading)
    xPos , yPos = position[0],position[1]
   
    #print("currrent global posistion is ",position)
    #print("currrent compass posistion is ",currentCompass)
    calculated_pose = control_loop_pose(distance_front_left_wheel_traveled, distance_front_right_wheel_traveled, theta) # updates real time position based on encoder and compass readings
    current_Point(position,robot.experiment_supervisor.getTime()) # prints when we get to a new point and the time it took to get to that point form the prvioseu point
    compare_gps_with_pose(position,calculated_pose,theta)
    store_live_path(calculated_pose, xPos, yPos)
     # Iterate through the lenghts specided by the left wheel distance list 
    if waypoints[0].is_visited() == False:        #if the point has not been visited then go forward until the left wheel has traveled the correct distance.
            forward()                             # when the distance is reached then set that object to vistited so the loop can continue
            if(distance_front_left_wheel_traveled>waypoints[0].distance):
                 waypoints[0].mark_visited()
                 
    elif  waypoints[1].is_visited() == False:
            Turning(.5, robot.axel_length, Outer_wheel_velocity_ms, .5, robot.wheel_radius, 1)   #turning function activated and the distance needed for left wheel to make turn is calculated                                                
            if(distance_front_left_wheel_traveled>waypoints[1].distance):
                 waypoints[1].mark_visited()                                            #distance is reached so is set to visited
                 
    elif  waypoints[2].is_visited() == False:
            forward()
            if(distance_front_left_wheel_traveled>=waypoints[2].distance):
                 waypoints[2].mark_visited()
                 
    elif  waypoints[2].is_visited() == True and waypoints[2].didPointTurn() == False:   #if the current point has been reached and the point has not made a point turn yet then acivate the point turn 
        pointTurn(45)
        if heading < 46 and heading > 44:                                               #once the desired compass heading has been reached the point will be set to made and will not activate this statement again 
            waypoints[2].mark_PTMade()                   
            
    elif waypoints[3].is_visited() == False and waypoints[2].didPointTurn() == True: #if the point has not been visited and the point turn has been made then we will go forward until specified left wheel distance
        forward()
        if(distance_front_left_wheel_traveled>waypoints[3].distance):
                 waypoints[3].mark_visited()
                 
    elif  waypoints[3].is_visited() == True and waypoints[3].didPointTurn() == False: 
        pointTurn(45)
        if heading < 136 and heading > 134: 
            waypoints[3].mark_PTMade()
            

    elif waypoints[4].is_visited() == False and waypoints[3].didPointTurn() == True:
        forward()
        if(distance_front_left_wheel_traveled>=waypoints[4].distance):
                 waypoints[4].mark_visited() 
                 
    elif  waypoints[4].is_visited() == True and waypoints[4].didPointTurn() == False:  
        pointTurn(45)

        if heading > 179 and heading <181: 
            waypoints[4].mark_PTMade()
             
    
    elif waypoints[5].is_visited() == False and waypoints[4].didPointTurn() == True:
        forward()
        if(distance_front_left_wheel_traveled>=waypoints[5].distance):
                 waypoints[5].mark_visited()
                 

    elif  waypoints[5].is_visited() == True and waypoints[5].didPointTurn() == False:  
        pointTurn(-90)
        if heading < 90:  
            waypoints[5].mark_PTMade()
            

    elif waypoints[6].is_visited() == False and waypoints[5].didPointTurn() == True:
        forward()
        if(distance_front_left_wheel_traveled>=waypoints[6].distance):
                 waypoints[6].mark_visited()
                 

    elif  waypoints[6].is_visited() == True and waypoints[6].didPointTurn() == False: 
        pointTurn(-90)
        if heading < 1.5 :  
            waypoints[6].mark_PTMade()
            

    elif waypoints[7].is_visited() == False and waypoints[6].didPointTurn() == True:
        forward()
        if(distance_front_left_wheel_traveled>=waypoints[7].distance):
                 waypoints[7].mark_visited()
                
    elif  waypoints[8].is_visited() == False and waypoints[7].is_visited() == True:
            travel = Turning(.5, robot.axel_length, Outer_wheel_velocity_ms, .5, robot.wheel_radius, 1)
            print(travel)
            if(distance_front_left_wheel_traveled>waypoints[8].distance):
                 waypoints[8].mark_visited()
                 
    elif waypoints[9].is_visited() == False:
            forward()
            if(distance_front_left_wheel_traveled>waypoints[9].distance):
                 waypoints[9].mark_visited()
                 robot.stop() 
                 break

    

plt.figure(figsize=(10, 6))
plt.title("Robot's Waypoint Comparison")

# Plot the final paths
plt.plot(estimated_x, estimated_y, "b-", label="Estimated Path")  # Encoder-based path
plt.plot(gps_x, gps_y, "r--", label="GPS Path")  # GPS-based path


plt.xlabel("X Position (meters)")
plt.ylabel("Y Position (meters)")
plt.legend()
plt.grid(True)
plt.axis("equal")


plt.savefig("gps_vs_encoder_path.png")


plt.show()

   
   
   
   