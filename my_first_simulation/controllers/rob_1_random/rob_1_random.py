from controller import Robot
from PD_Controller import PDController
import math
import time
import random
import socket

# Initialisierung
robot = Robot()
TIME_STEP = 100

# GPS-Gerät holen
gps = robot.getDevice("gps")
gps.enable(TIME_STEP)

def pos():
    return gps.getValues()

# Compass holen
compass = robot.getDevice("compass")
compass.enable(TIME_STEP)

def degree_rad():
    values = compass.getValues()  # [x, y, z] → auf Bodenebene: [x, y, z], wobei x/y sind wichtig
    angle = math.atan2(values[0], values[1])  # beachte: x = sin, y = cos
    return - angle + math.pi/2   # positiver Wert im Uhrzeigersinn
    
def degree_grad():
    return math.degrees(degree_rad()) % 360

# Motoren holen
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

# Position auf unendlich setzen → Geschwindigkeitsmodus
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

led0 = robot.getDevice("led0")
###################################
# Konfiguration
x_max = 2
y_max = 1

#Speeds
speed = 500

#Turn from wall
diff_angle = 5 #Grad° # 30
outside_diff = 0.1
speed_wall = 500
turn_speed_wall = 500

# Robot folgen
turn_follow_speed = 200
follow_speed = 0

# PD Konstanten
pd = PDController(500,300,10)

###################################

def set_motor_speeds(l, r):
    left_motor.setVelocity((7/1000)*l)
    right_motor.setVelocity((7/1000)*r)
    

def angle_calculate(x, y):
    return -math.atan2(y - pos()[1], x - pos()[0]) + math.pi/2

def is_inside_arena():
    position = pos()
    return (x_max - outside_diff > position[0] > outside_diff
            and y_max - outside_diff > position[1] > outside_diff)

def turn_from_wand_mith_motors():
    target_angle = None

    x = pos()[0]
    y = pos()[1]

    if x < outside_diff and y < outside_diff:
        target_angle = 45 
    elif x < outside_diff and y > y_max - outside_diff:
        target_angle = 45 + 90
    elif x > x_max - outside_diff and y < outside_diff:
        target_angle = 45 + 90*3
    elif x > x_max - outside_diff and y > y_max - outside_diff:
        target_angle = 45 + 90*2

    elif x < outside_diff:
        target_angle = 90
    elif y < outside_diff:
        target_angle = 0
    elif x > x_max - outside_diff:
        target_angle = 270
    elif y > y_max - outside_diff:
        target_angle = 180
    if target_angle is None:
        return
    
    target_angle = math.radians(target_angle)
    current_angle = degree_rad()

    error = (target_angle - current_angle + math.pi) % (2 * math.pi) - math.pi
    
    if not -math.radians(diff_angle) < error < math.radians(diff_angle):

        output = pd.update(error)
        output = max(-turn_speed_wall, min(turn_speed_wall, output))

        left = output
        right = -output
        
        set_motor_speeds(left, right)

    else:
        
        set_motor_speeds(speed_wall, speed_wall)


###################################
# Endlosschleife → dreht sich ständig
while robot.step(TIME_STEP) != -1:

    if is_inside_arena():
        led0.set(1)
        set_motor_speeds(random.randint(200, 500), random.randint(200, 500))
    else:
        led0.set(0)
        turn_from_wand_mith_motors()

   






