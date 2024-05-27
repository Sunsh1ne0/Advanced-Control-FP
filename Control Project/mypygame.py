import pgzrun
from Pendulum_class import Pendulum
from time import sleep

pend = Pendulum("COM6", 115200)

WIDTH = 1000
HEIGHT = 1000

def draw():
    # screen.fill((255, 255, 255))
    pass

speed = 200

def update():
    if keyboard.d:
        print(pend.set_speed(speed))
    elif keyboard.a:
        print(pend.set_speed(-speed))
    else:
        print(pend.set_speed(0))
    return

pgzrun.go()
