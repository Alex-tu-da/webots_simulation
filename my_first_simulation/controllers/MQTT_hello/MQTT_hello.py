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
TIME_STEP = 100
robot = Robot()
my_robot_id = "" +robot.getName()

data = {}
helloRob = []

# === MQTT-Callbackfunktionen ===
def on_connect(client, userdata, flags, rc):
    print(f"[{my_robot_id}] Verbunden mit MQTT (Code {rc})")
    client.subscribe("robot_pos/all")
    client.subscribe(f"robotHello/{my_robot_id}")

# on_message:
def on_message(client, userdata, msg):
    global data
    topic = msg.topic
    payload_str = msg.payload.decode()

    if topic == "robot_pos/all":
        try:
            incoming = json.loads(payload_str)
            for rid, rob_data in incoming.items():
                data[rid] = {
                    "position": rob_data['position'],
                    "angle": rob_data['angle']
                }
        except json.JSONDecodeError:
            print(f"Ungültiges JSON auf topic {topic}: {payload_str}")

    elif topic.startswith("robotHello/"):
        print(f"Nachricht empfangen: {payload_str}")

    else:
        print(f"Unbekanntes Topic: {topic}, payload: {payload_str}")

def send_position():
    try:
        x, y, z = pos()
        angle = degree_grad()
        msg = {
            my_robot_id: {    # das Robot-ID als Schlüssel
                "position": [x, y],
                "angle": angle
            }
        }
        client.publish("robot_pos/all", json.dumps(msg))

    except Exception as e:
        print(f"❌ Fehler beim Senden der Position: {e}")

# === MQTT-Client initialisieren ===
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, PORT)
client.loop_start()

# === Roboter-Initialisierung ===
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

# IR-Sensoren holen und aktivieren
ir_sensor = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
ir_sensors = [robot.getDevice(name) for name in ir_sensor]
for s in ir_sensors:
    s.enable(TIME_STEP)

# === Arena-Grenzen & Steuerungskonstanten ===
x_max = 1
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

# === Komplette Funktionen ===
def is_inside_arena():
    return (x_max - outside_diff > data[my_robot_id]["position"][0] > outside_diff
            and y_max - outside_diff > data[my_robot_id]["position"][1] > outside_diff)

def angle_calculate(target_id):
    src = data[my_robot_id]["position"]
    dst = data[target_id]["position"]
    return math.atan2(dst[0] - src[0], dst[1] - src[1])

def dist_calculate(target_id):
    src = data[my_robot_id]["position"]
    dst = data[target_id]["position"]
    return math.hypot(dst[1] - src[1], dst[0] - src[0])

def turn_motors_to(obj_id):
    if obj_id is None:
        return
    target_angle = angle_calculate(obj_id)
    current_angle = math.radians(data[my_robot_id]["angle"])
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

def turn_from_wand_mith_motors():
    target_angle = None

    x = data[my_robot_id]["position"][0]
    y = data[my_robot_id]["position"][1]

    if x < outside_diff and y < outside_diff:
        target_angle = 45
    elif x < outside_diff and y > y_max - outside_diff:
        target_angle = 45 + 90
    elif x > x_max - outside_diff and y < outside_diff:
        target_angle = 45 + 90 * 3
    elif x > x_max - outside_diff and y > y_max - outside_diff:
        target_angle = 45 + 90 * 2

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
    current_angle = math.radians(data[my_robot_id]["angle"])

    error = (target_angle - current_angle + math.pi) % (2 * math.pi) - math.pi
    if not (-math.radians(diff_angle) < error < math.radians(diff_angle)):

        output = pd.update(error)
        output = max(-turn_speed_wall, min(turn_speed_wall, output))

        left = output
        right = -output
        set_motor_speeds(left, right)
    else:
        set_motor_speeds(speed_wall, speed_wall)

def check_and_send_hello():
    for rid in data:
        if rid == my_robot_id:
            continue
        dist = dist_calculate(rid)
        if dist < 0.2:
            if rid not in helloRob:
                helloRob.append(rid)
                nachricht = f"Hallo von Roboter {my_robot_id}"
                client.publish(f"robotHello/{rid}", nachricht)
                print(f"Sende an Robot {rid}: {nachricht}")

def avoid_collision_ir():
    ir_values = [s.getValue() for s in ir_sensors]

    # Sensorgruppen
    front = [ir_values[0], ir_values[7]]  # ps0, ps1, ps2
    right = [ir_values[2], ir_values[1]]  # ps3, ps4
    left = [ir_values[5], ir_values[6]]   # ps6, ps7

    # Schwelle für "zu nah"
    threshold = 150

    # Auswertung
    if max(front) > threshold:
        if front[0] > front[1]:
            set_motor_speeds(-100,  100)
        else:
            set_motor_speeds(-100, 100)
        return True
    elif max(left) > threshold:
        set_motor_speeds(200, 150)
        return True
    elif max(right) > threshold:
        set_motor_speeds(150, 200)
        return True

    return False  # keine Kollision erkannt

# === Startbedingung ===
set_motor_speeds(0, 0)

# === Hauptschleife ===
while robot.step(TIME_STEP) != -1:
    send_position()
    if my_robot_id not in data:
        continue
    # === Hier scheiben ===
    
    check_and_send_hello()
    if is_inside_arena():
        if avoid_collision_ir():
            continue
        set_motor_speeds(random.randint(200, 500), random.randint(200, 500))
    else:
        turn_from_wand_mith_motors()
