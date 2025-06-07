import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from PD_Controller import PDController

from controller import Robot
import math
import time
import random
import socket
import json
import paho.mqtt.client as mqtt

# === MQTT-Konfiguration ===
BROKER = "localhost"
PORT = 1883
MY_ID = "1"

data = {}

<<<<<<< HEAD
=======

>>>>>>> 79011dd (Labyrinth)
# === MQTT-Callbackfunktionen ===
def on_connect(client, userdata, flags, rc):
    print(f"[{MY_ID}] Verbunden mit MQTT (Code {rc})")
    client.subscribe("robot_pos/all")


# on_message:
def on_message(client, userdata, msg):
    global data
    try:
        incoming = json.loads(msg.payload.decode())
        rid = incoming["id"]
<<<<<<< HEAD
        data[rid] = incoming
=======
        data[rid] = {
            "position": incoming["position"],
            "angle": incoming["angle"]
        }
>>>>>>> 79011dd (Labyrinth)
    except Exception as e:
        print(f"⚠️ Fehler beim Empfangen: {e}")


# === MQTT-Client initialisieren ===
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, PORT)
client.loop_start()

# === Roboter-Initialisierung ===
robot = Robot()
TIME_STEP = 100

gps = robot.getDevice("gps")
gps.enable(TIME_STEP)

compass = robot.getDevice("compass")
compass.enable(TIME_STEP)

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

led0 = robot.getDevice("led0")
led1 = robot.getDevice("led1")

# === Arena-Grenzen & Steuerungskonstanten ===
x_max = 2
y_max = 1
<<<<<<< HEAD
outside_diff = 0.1
speed_wall = 500
turn_speed_wall = 500
turn_follow_speed = 200
follow_speed = 200
diff_angle = 5
=======

outside_diff = 0.1
speed_wall = 500
turn_speed_wall = 500
diff_angle = 5

turn_follow_speed = 200
follow_speed = 200
diff_angle_follow = 90
>>>>>>> 79011dd (Labyrinth)

pd = PDController(500, 300, 10)


# === Hilfsfunktionen ===
def pos():
    return gps.getValues()


def degree_rad():
    values = compass.getValues()
    angle = math.atan2(values[0], values[1])
    return -angle + math.pi / 2


def degree_grad():
    return math.degrees(degree_rad()) % 360


def set_motor_speeds(l, r):
    left_motor.setVelocity((7 / 1000) * l)
    right_motor.setVelocity((7 / 1000) * r)


<<<<<<< HEAD
def angle_calculate(x, y):
    return -math.atan2(y - pos()[1], x - pos()[0]) + math.pi / 2


=======
>>>>>>> 79011dd (Labyrinth)
def send_position():
    try:
        x, y, _ = pos()
        angle = degree_grad()
        msg = {
            "id": MY_ID,
            "position": [x, y],
            "angle": angle
        }
        client.publish("robot_pos/all", json.dumps(msg))
    except Exception as e:
        print(f"❌ Fehler beim Senden der Position: {e}")


# === Komplette Funktionen ===
def is_inside_arena():
    return (x_max - outside_diff > data[MY_ID]["position"][0] > outside_diff
            and y_max - outside_diff > data[MY_ID]["position"][1] > outside_diff)

<<<<<<< HEAD
def turn_motors_to(x, y):
    target_angle = angle_calculate(x, y)
    current_angle = degree_rad()
=======
def angle_calculate(target_id):
    src = data[MY_ID]["position"]
    dst = data[target_id]["position"]
    return math.atan2(dst[0] - src[0], dst[1] - src[1])

def dist_calculate(target_id):
    src = data[MY_ID]["position"]
    dst = data[target_id]["position"]
    return math.hypot(dst[1] - src[1], dst[0] - src[0])

def turn_motors_to(obj_id):
    if obj_id is None:
        return
    target_angle = angle_calculate(obj_id)
    current_angle = math.radians(data[MY_ID]["angle"])
>>>>>>> 79011dd (Labyrinth)
    error = (target_angle - current_angle + math.pi) % (2 * math.pi) - math.pi

    output = pd.update(error)
    output = max(-turn_follow_speed, min(turn_follow_speed, output))

<<<<<<< HEAD
    if -math.pi / 2 < error < math.pi / 2:
        left = output
        right = -output
        set_motor_speeds(left + follow_speed, right + follow_speed)
    else:
        left = output
        right = -output
        set_motor_speeds(left, right)


def turn_from_wand_mith_motors():
    x = pos()[0]
    y = pos()[1]
    target_angle = None

    if x < outside_diff and y < outside_diff:
        target_angle = 45
    elif x < outside_diff and y > y_max - outside_diff:
        target_angle = 135
    elif x > x_max - outside_diff and y < outside_diff:
        target_angle = 315
    elif x > x_max - outside_diff and y > y_max - outside_diff:
        target_angle = 225
=======
    if not -math.radians(diff_angle_follow) < error < math.radians(diff_angle_follow):
        left = output
        right = -output
        set_motor_speeds(left, right)
    else:
        left = output
        right = -output
        set_motor_speeds(left + follow_speed, right + follow_speed)

def turn_from_wand_mith_motors():
    target_angle = None

    x = data[MY_ID]["position"][0]
    y = data[MY_ID]["position"][1]

    if x < outside_diff and y < outside_diff:
        target_angle = 45
    elif x < outside_diff and y > y_max - outside_diff:
        target_angle = 45 + 90
    elif x > x_max - outside_diff and y < outside_diff:
        target_angle = 45 + 90 * 3
    elif x > x_max - outside_diff and y > y_max - outside_diff:
        target_angle = 45 + 90 * 2

>>>>>>> 79011dd (Labyrinth)
    elif x < outside_diff:
        target_angle = 90
    elif y < outside_diff:
        target_angle = 0
    elif x > x_max - outside_diff:
        target_angle = 270
    elif y > y_max - outside_diff:
        target_angle = 180
<<<<<<< HEAD

    if target_angle is None:
        return

    target_angle = math.radians(target_angle)
    current_angle = degree_rad()
    error = (target_angle - current_angle + math.pi) % (2 * math.pi) - math.pi

    if not -math.radians(diff_angle) < error < math.radians(diff_angle):
        output = pd.update(error)
        output = max(-turn_speed_wall, min(turn_speed_wall, output))
        set_motor_speeds(output, -output)
=======
    if target_angle is None:
        return
    target_angle = math.radians(target_angle)
    current_angle = math.radians(data[MY_ID]["angle"])

    error = (target_angle - current_angle + math.pi) % (2 * math.pi) - math.pi
    if not (-math.radians(diff_angle) < error < math.radians(diff_angle)):

        output = pd.update(error)
        output = max(-turn_speed_wall, min(turn_speed_wall, output))

        left = output
        right = -output
        set_motor_speeds(left, right)

>>>>>>> 79011dd (Labyrinth)
    else:
        set_motor_speeds(speed_wall, speed_wall)


# === Startbedingung ===
set_motor_speeds(0, 0)
send_position()

# === Hauptschleife ===
while robot.step(TIME_STEP) != -1:
    send_position()
    
    if is_inside_arena():
        led0.set(1)
<<<<<<< HEAD
        set_motor_speeds(random.randint(200, 500), random.randint(200, 500))
=======
        #set_motor_speeds(random.randint(200, 500), random.randint(200, 500))
>>>>>>> 79011dd (Labyrinth)
    else:
        led0.set(0)
        turn_from_wand_mith_motors()
