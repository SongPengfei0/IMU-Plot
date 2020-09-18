# coding: utf-8
import serial
import time
import threeDPlot
from vpython import *

toRad = pi / 180
toDeg = 180 / pi


def showSensorData():
    threeDPlot.init()
    arduinoData = serial.Serial('/dev/cu.usbmodem1411401', baudrate=115200)
    time.sleep(1)
    threeDPlot.plot()
    print("Start read data .")

    while True:
        while arduinoData.inWaiting() == 0:
            pass
        dataPacket = arduinoData.readline()
        dataPacket = str(dataPacket, 'utf-8')
        print(dataPacket)
        splitPacket = dataPacket.split(',')
        pitch = float(splitPacket[0])
        roll = float(splitPacket[1])
        yaw = float(splitPacket[2])
        print("pitch=", pitch, "roll=", roll, "yaw=", yaw)
        threeDPlot.update(yaw * toRad, pitch * toRad, roll * toRad)


if __name__ == '__main__':
    showSensorData()
