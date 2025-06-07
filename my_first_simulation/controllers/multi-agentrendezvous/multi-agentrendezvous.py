from controller import Robot
import math
import time
import random
import socket
import json

# === MQTT-Konfiguration ===
BROKER = "localhost"
PORT = 1883

# === Roboter-Initialisierung ===
robot = Robot()
TIME_STEP = 100

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

led0 = robot.getDevice("led0")
led1 = robot.getDevice("led1")

tof = robot.getDevice("tof")
tof.enable(TIME_STEP)


# === Hilfsfunktionen ===


def set_motor_speeds(m):
    left_motor.setVelocity((7 / 1000) * m)
    right_motor.setVelocity((7 / 1000) * m)


def rotation(m):
    left_motor.setVelocity((7 / 1000) * m)
    right_motor.setVelocity((7 / 1000) * -m)


# === Startbedingung ===
set_motor_speeds(0)


def isRobot():
    distance = tof.getValue()
    if distance < 2000:
        return True
    else:
        return False
        
a = 0
speed = 200

# === Hauptschleife ===
while robot.step(TIME_STEP) != -1:
    if isRobot():
        distance = tof.getValue()
        a = 1
        if distance < 30:
            set_motor_speeds(0)
        else:
            set_motor_speeds(400)
    else:
        if a == 1:
            a = 0
            zahl = random.choice([1, 2])
            if zahl == 1:
                speed = 200
            else:
                speed = -200
        rotation(speed)
