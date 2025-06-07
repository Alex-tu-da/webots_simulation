from controller import Robot
from PD_Controller import PDController
import math
import time
import random
import os

# Initialisierung
robot = Robot()
<<<<<<< HEAD
TIME_STEP = 100
=======
TIME_STEP = 100 # 1000 = 1s
>>>>>>> 9037687 (Initialer Commit für webots_sim_test0)

# GPS-Gerät holen
gps = robot.getDevice("gps")
gps.enable(TIME_STEP)
<<<<<<< HEAD

def pos():
    return gps.getValues()

# Compass holen
compass = robot.getDevice("compass")
compass.enable(TIME_STEP)

def degree_rad():
    values = compass.getValues()  # [x, y, z] → auf Bodenebene: [x, y, z], wobei x/y sind wichtig
    angle = math.atan2(values[0], values[1])  # beachte: x = sin, y = cos
    return - angle + math.pi/2   # positiver Wert im Uhrzeigersinn
    
=======
# Eigene Position, return [x,y,z]
def pos():
    return gps.getValues()

# Eigene Orientierung
compass = robot.getDevice("compass")
compass.enable(TIME_STEP)
# In Rad
def degree_rad():
    values = compass.getValues() 
    angle = math.atan2(values[0], values[1])  
    return - angle + math.pi/2
# In Grad
>>>>>>> 9037687 (Initialer Commit für webots_sim_test0)
def degree_grad():
    return math.degrees(degree_rad()) % 360

# Motoren holen
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

# Position auf unendlich setzen → Geschwindigkeitsmodus
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

led0 = robot.getDevice("led0")
<<<<<<< HEAD
=======
led1 = robot.getDevice("led1")
>>>>>>> 9037687 (Initialer Commit für webots_sim_test0)
###################################
# Konfiguration
x_max = 2
y_max = 1

#Speeds
speed = 500

#Turn from wall
<<<<<<< HEAD
diff_angle = 5 #Grad° # 30
=======
diff_angle = 5 #Grad° (30)
>>>>>>> 9037687 (Initialer Commit für webots_sim_test0)
outside_diff = 0.1
speed_wall = 500
turn_speed_wall = 500

# Robot folgen
turn_follow_speed = 200
follow_speed = 200

# PD Konstanten
pd = PDController(500,300,10)

<<<<<<< HEAD
###################################

=======
# Sonstige
a = 0

###################################
# Meine Methoden
>>>>>>> 9037687 (Initialer Commit für webots_sim_test0)
def set_motor_speeds(l, r):
    left_motor.setVelocity((7/1000)*l)
    right_motor.setVelocity((7/1000)*r)
    

def angle_calculate(x, y):
    return -math.atan2(y - pos()[1], x - pos()[0]) + math.pi/2

def is_inside_arena():
    return (x_max - outside_diff > pos()[0] > outside_diff
            and y_max - outside_diff > pos()[1] > outside_diff)

def turn_motors_to(x, y):

    target_angle = angle_calculate(x, y)
    current_angle = degree_rad()
    error = (target_angle - current_angle + math.pi) % (2 * math.pi) - math.pi
    
    output = pd.update(error)
    
    output = max(-turn_follow_speed, min(turn_follow_speed, output))

    
    if -math.pi / 2 < error < math.pi / 2:
        left = output
        right = -output
        set_motor_speeds(left + follow_speed, right + follow_speed)
    else:
        left = output
        right = -output
        set_motor_speeds(left, right)

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
        
        
def read_pos():
    file_path = "../rob_1_random/pos_robot1.txt"
    if not os.path.exists(file_path):
        print("📂 Datei nicht gefunden:", file_path)
        return None
    with open(file_path, "r") as f:
        content = f.read().strip()
        if not content:
            print("⚠️ Datei existiert, aber ist leer.")
            return None
        try:
            x, y = map(float, content.split(","))
            return [x, y]
        except ValueError:
            print(f"⚠️ Ungültiger Inhalt in Datei: {content}")
            return None
<<<<<<< HEAD


###################################
# Endlosschleife → dreht sich ständig
while robot.step(TIME_STEP) != -1:

    other_pos = read_pos()
    if is_inside_arena():
        led0.set(1)
        turn_motors_to(other_pos[0], other_pos[1])
    else:
        led0.set(0)
        turn_from_wand_mith_motors()
    
    #values = [s.getValue() for s in sensors]
    #print("IR-Werte:", values)








=======
##################################
# Voreinstellung
set_motor_speeds(0, 0)

###################################
# Schleife
while robot.step(TIME_STEP) != -1:

    

    other_pos = read_pos()
    if is_inside_arena():
        led1.set(1)
        turn_motors_to(other_pos[0], other_pos[1])
    else:
        led0.set(0)
        turn_from_wand_mith_motors()
>>>>>>> 9037687 (Initialer Commit für webots_sim_test0)
