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
MY_ID = "0"

data = {}

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
        data[rid] = {
            "position": incoming["position"],
            "angle": incoming["angle"]
        }
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

# Namen der IR-Sensoren
ir_names = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
# Sensoren abrufen und aktivieren
ir_sensors = []
for name in ir_names:
    sensor = robot.getDevice(name)
    sensor.enable(TIME_STEP)
    ir_sensors.append(sensor)

# === Arena-Grenzen & Steuerungskonstanten ===
x_max = 2
y_max = 1

outside_diff = 0.1
speed_wall = 500
turn_speed_wall = 500
diff_angle = 5

turn_follow_speed = 200
follow_speed = 200
diff_angle_follow = 90

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
    error = (target_angle - current_angle + math.pi) % (2 * math.pi) - math.pi

    output = pd.update(error)
    output = max(-turn_follow_speed, min(turn_follow_speed, output))

    if not -math.radians(diff_angle_follow) < error < math.radians(diff_angle_follow):
        left = output
        right = -output
        set_motor_speeds(left, right)
    else:
        left = output
        right = -output
        set_motor_speeds(left + follow_speed, right + follow_speed)
        
def collision():
    ir_values = [sensor.getValue() for sensor in ir_sensors]
    
    
    if ir_values[7] > 100 or  ir_values[0] > 100:
        
        if (ir_values[6] > 100 or ir_values[5] > 100) :
            set_motor_speeds(200, -100)
        elif (ir_values[1] > 100 or ir_values[2] > 100) :
            set_motor_speeds(-100, 200)
    else:
        if (ir_values[6] > 150 ) :
            set_motor_speeds(150, -50)
        elif (ir_values[1] > 150 ) :
            set_motor_speeds(-50, 150)
        else:
            set_motor_speeds(400, 400)


# === Startbedingung ===
set_motor_speeds(0, 0)
send_position()

# === Hauptschleife ===
while robot.step(TIME_STEP) != -1:
    send_position()
    
    collision()

    
