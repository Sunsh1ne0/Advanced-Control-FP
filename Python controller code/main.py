import numpy as np

from Pendulum_class import Pendulum
from Controllers import PID, EnergyBased, Swing
from time import sleep
import sys

pend = Pendulum(sys.argv[0], 115200)


def hold_state(data, speed):
    print('\r' + f"speed: {speed}, pole_pose: {data[0]}, cart_pose: {data[1]}, pole_speed: {data[2]:.3f}, cart_speed: {data[3]:.3f}",
          end='', flush=True)
    # print(f"speed: {speed}, pole_pose: {data[0]}, cart_pose: {data[1]}, pole_speed: {data[2]:.3f}, cart_speed: {data[3]:.3f}")


def safe(state, speed):
    window = 5
    coef = np.clip((25 - abs(state[1]))/window, 0, 1)
    if abs(state[1]) < 22:
        speed * coef
    else:
        move(0)
        speed = 0
    return speed


def move(position):
    speed_t = 110
    error, state = pend.get_state("float")
    while abs(state[1] - position) > 0.5:
        error, state = pend.get_state("float")
        if error:
            print("ERROR")
            continue
        speed = int(-speed_t * np.sign(state[1] - position))
        pend.set_speed(speed)
        hold_state(state, speed)

    pend.set_speed(0)

    while abs(abs(state[0]) - np.pi) > 0.1 or abs(state[2]) > 0.005:
        error, state = pend.get_state("float")
        if error:
            print("ERROR")
            continue
        hold_state(state, 0)
    pass


def main():
    print("Start")

    bound = 0.2
    target = np.array((0, 0, 0, 0))
    pid = PID((5000, 0, 1500, 0), bound)
    swing = Swing(0.5)
    # ener = EnergyBased(90)
    stable = False
    move(0)

    while True:
        error, state = pend.get_state("float")
        if error:
            print("ERROR")
            continue
        if abs(state[0]) < bound and abs(state[2]) < 0.003 and not stable:
            stable = True
            print()
            print("PID stated")
        elif abs(state[0]) > 3 and stable:
            stable = False
            swing = Swing(1.5)
            print()
            print("SWING stated")

        if stable:
            speed = pid.step(state, target)
        else:
            speed = swing.step(state, 132)

        speed = safe(state, speed)
        pend.set_speed(int(speed))
        hold_state(state, speed)
    return


if __name__ == "__main__":
    try:
        main()
    except:
        for i in range(20):
            print("STOP")
            pend.set_speed(0)
        raise Exception
