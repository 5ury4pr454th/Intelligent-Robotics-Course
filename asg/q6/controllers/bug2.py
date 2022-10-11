"""bug2 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from utils import two_points_line
import math


def angle_bw_slopes(m1, m2):
    return math.atan((m1 - m2)/(1 + m1*m2))

def point_near_line(m, c, x, y, threshold):
    if abs(y - m*x - c) < threshold:
        return 1
    else:
        return 0

# create the Robot instance.
robot = Robot()
max_speed = 6.24
goal_location = (0.625, 0.625)
threshold = 0.02

# get the time step of the current world.
timestep = 32

# get gps
gps_1 = robot.getDevice("gps_1")
gps_2 = robot.getDevice("gps_2")

# get motor
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

ps0 = robot.getDevice('ps0')
ps0.enable(timestep)

ps7 = robot.getDevice('ps7')
ps7.enable(timestep)
 
# set stuff
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

ps0 = robot.getDevice('ps0')
ps1 = robot.getDevice('ps1')
ps2 = robot.getDevice('ps2')

ps0.enable(timestep)
ps1.enable(timestep)
ps2.enable(timestep)

# enable gps
gps_1.enable(timestep)
gps_2.enable(timestep)
init_slope = two_points_line([-1.375, -0.125], goal_location)

while robot.step(timestep) != -1:

    # get current gps positions
    position_1 = gps_1.getValues()[0:3]
    position_2 = gps_2.getValues()[0:3]
    current_position_1 = (position_1[0], position_1[1])
    current_position_2 = (position_2[0], position_2[1])
    
    target_slope = two_points_line(current_position_1, goal_location)
    sensor_slope = two_points_line(current_position_1, current_position_2)

    angle_difference = angle_bw_slopes(target_slope[0], sensor_slope[0])
    distance_error = abs(current_position_1[0] - goal_location[0]) + abs(current_position_1[1] - goal_location[1])

    right_wall = ps2.getValue() > 80
    right_corner = ps1.getValue() > 80
    front_wall = ps0.getValue() > 80
    online = False
    
    # if reached the goal
    if distance_error < threshold:
        # stop if it is close
        left_motor.setVelocity(max_speed*0.0)
        right_motor.setVelocity(max_speed*0.0)
        timestep = -1  
    
    online = point_near_line(init_slope[0], init_slope[1], current_position_1[0], current_position_1[1], threshold)
    
    # checks if online
    if online:
    
        online = True
    
    else:
        online = False
    
    # if online and nothing in the front go towards goal
    
    if online:
    
        # rotate to align yourself
        if abs(angle_difference) > (threshold): 
        # get rotated  
        
            if (current_position_1[1] < goal_location[1]):          
            
                left_motor.setVelocity(-max_speed*0.025)
                right_motor.setVelocity(max_speed*0.025)
   
        # move front
        else:
            left_motor.setVelocity(max_speed*0.25)
            right_motor.setVelocity(max_speed*0.25)
            
    
    # follow line following
    else:
        
        left_motor.setVelocity(max_speed)
        right_motor.setVelocity(max_speed)
                
        if front_wall:
    
            # do the wall following
            left_motor.setVelocity(-max_speed/2)
            right_motor.setVelocity(max_speed)
    
        else:
        
            if right_wall:
                left_motor.setVelocity(max_speed)
                right_motor.setVelocity(max_speed)
            
            else:
                left_motor.setVelocity(max_speed)
                right_motor.setVelocity(max_speed/8)
                
            if right_corner:
               left_motor.setVelocity(max_speed/8)
               right_motor.setVelocity(max_speed)