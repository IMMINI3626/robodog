from tools import *
import serial
from time import sleep

esp = serial.Serial("COM3", 115200, timeout=1)

dog = RoboDog()

if not dog.Open("COM4"):
    print("RoboDog 연결 실패")
    esp.close()
    exit()

state = None
dog.move(30)
print("DONE")
try:
    while True:

        data = esp.readline().decode("utf-8").strip()

        try:
            distance = float(data)
        except ValueError:
            continue

        print(f"{distance:.1f} cm")

        # 장애물 발견
        if distance < 20:
            if state != "STOP":
                dog.gesture(0)
                print(state)
                sleep(1)
                state = "STOP"
        else:
            if state != "MOVE":
                dog.move(30)
                sleep(1)
                state = "MOVE"

finally:
    dog.move(STOP)
    dog.Close()
    esp.close()