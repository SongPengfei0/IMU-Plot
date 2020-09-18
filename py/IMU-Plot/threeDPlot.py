# coding: utf-8
from vpython import *
from math import *


def init():
    scene.range = 5
    scene.forward = vector(1, -1, -1)
    scene.width = 600
    scene.height = 600


toRad = pi / 180
toDeg = 180 / pi
Xarrow, Yarrow, Zarrow = None, None, None
bBoard = None
arduino, mpu9250 = None, None
myObj = None


def plot():
    global bBoard
    bBoard = box(length=6, width=2, height=.2, opacity=.4, color=color.white)
    global arduino, mpu9250, myObj
    arduino = box(pos=vector(0.5, .2, 0), length=2, width=1.8, height=.4, opacity=.2, color=color.blue)
    mpu9250 = box(pos=vector(-2.5, 0, 0), length=0.5, width=1, height=.2, opacity=.5, color=color.cyan)
    myObj = compound([bBoard, arduino, mpu9250])
    global Xarrow, Yarrow, Zarrow
    Xarrow = arrow(axis=vector(0, 0, -1), length=4, shaftwidth=.1, color=color.red)
    Yarrow = arrow(axis=vector(-1, 0, 0), length=4, shaftwidth=.1, color=color.green)
    Zarrow = arrow(axis=vector(0, -1, 0), length=4, shaftwidth=.1, color=color.blue)
    coordinate = vector(-5, -5, 0)
    coX = arrow(pos=coordinate, axis=vector(0, 0, -1), length=2, shaftwidth=.3, color=color.red)
    rate(50)


def update(yaw: float, pitch: float, roll: float):
    k = vector(-cos(yaw) * cos(pitch), sin(pitch), -sin(yaw) * cos(pitch))

    p = vector(-sin(yaw) * cos(roll), -sin(roll), cos(yaw) * cos(roll))

    m = vector(sin(roll), -cos(roll) * cos(pitch), -cos(roll) * sin(pitch))

    Xarrow.axis = k
    Yarrow.axis = p
    Zarrow.axis = m

    myObj.axis = vector(-p.x, -p.y, -p.z)
    myObj.up = m
