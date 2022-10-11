"""simple_bug_0 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from utils import two_points_line
import math
    
def angle_bw_slopes(m1, m2):
    return math.atan((m1 - m2)/(1 + m1*m2))

def give_control_to_zero():
    return 

# create the Robot instance.
robot = Robot()
max_speed = 6.24
goal_location = (0.625, 0.625)
threshold = 0.01

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

while robot.step(timestep) != -1:

    # get current gps positions
    position_1 = gps_1.getValues()[0:3]
    position_2 = gps_2.getValues()[0:3]
    current_position_1 = (position_1[0], position_1[1])
    current_position_2 = (position_2[0], position_2[1])
    
    target_slope = two_points_line(current_position_1, goal_location)
    sensor_slope = two_points_line(current_position_1, current_position_2)

    angle_difference = angle_bw_slopes(target_slope, sensor_slope)
    distance_error = abs(current_position_1[0] - goal_location[0]) + abs(current_position_1[1] - goal_location[1])
    
    right_wall = ps2.getValue() > 80
    right_corner = ps1.getValue() > 80
    front_wall = ps0.getValue() > 80
    
    if front_wall:
        left_motor.setVelocity(-max_speed)
        right_motor.setVelocity(max_speed)
    else:
        if right_corner:
           left_motor.setVelocity(max_speed/8)
           right_motor.setVelocity(max_speed)
        
        elif abs(angle_difference) > threshold: 
            # get rotated
            left_motor.setVelocity(max_speed*0.025)
            right_motor.setVelocity(-max_speed*0.025)
        else:
            # go straight
            left_motor.setVelocity(max_speed*0.25)
            right_motor.setVelocity(max_speed*0.25)
            
        if distance_error < threshold:
            # stop if it is close
            left_motor.setVelocity(max_speed*0.0)
            right_motor.setVelocity(max_speed*0.0)
            timestep = -1