from Uart import UART
import time
import struct
import numpy as np

# -----COMMANDS-----


class Command:
    def __init__(self, number, command_length, response_length):
        self.cmd = number
        self.msg = command_length
        self.rsp = response_length


CALIBRATE = Command(0, 0, 0)
SET_SPEED = Command(1, 4, 0)
GET_STATE = Command(2, 0, 12)
PING      = Command(3, 0, 0)
GET_STATE_TIME = Command(4, 0, 14)

# -----COMMANDS-----

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


class Pendulum(UART):
    def __init__(self, port_name: str, baud_rate: int):
        super().__init__(port_name, baud_rate)
        time.sleep(1)
        self.open_com_port()
        print('Ready')

    def calibration(self):
        self.send_request(CALIBRATE.cmd, bytearray())

    def set_speed(self, speed: int):
        speed = int(speed)
        data = bytearray(struct.pack("i", speed))
        self.send_request(SET_SPEED.cmd, data)
        cmd_error, data = self.receive_response(SET_SPEED.rsp)
        return cmd_error

    def __get_state(self):
        self.send_request(GET_STATE.cmd, bytearray())
        cmd_error, data = self.receive_response(GET_STATE.rsp)
        if not cmd_error:
            pole_pose = struct.unpack('h', data[0:2])[0]
            cart_pose = struct.unpack('h', data[2:4])[0]
            pole_speed = struct.unpack('f', data[4:8])[0]
            cart_speed = struct.unpack('f', data[8:12])[0]
            return cmd_error, (pole_pose, cart_pose, pole_speed, cart_speed)
        else:
            return cmd_error, (0,0,0,0)

    def __get_state_time(self):
        self.send_request(GET_STATE_TIME.cmd, bytearray())
        cmd_error, data = self.receive_response(GET_STATE_TIME.rsp)
        if not cmd_error:
            pole_pose = struct.unpack('h', data[0:2])[0]
            cart_pose = struct.unpack('h', data[2:4])[0]
            pole_speed = struct.unpack('f', data[4:8])[0]
            cart_speed = struct.unpack('f', data[8:12])[0]
            timer = struct.unpack('h', data[12:14])[0]
            return cmd_error, (pole_pose, cart_pose, pole_speed, cart_speed, timer)
        else:
            return cmd_error, (0,0,0,0,0)

    def get_state(self, _format: str = "raw"):
        if _format == 'raw':
            return self.__get_state()
        elif _format == 'float':
            cmd, _data = self.__get_state()
            if not cmd:
                data = np.array(_data)

                if abs(data[0]) >= 4096:
                    if data[0] > 0:
                        data[0] -= 4096 * int(abs(data[0]) / 4096)
                    else:
                        data[0] += 4096 * int(abs(data[0]) / 4096)
                coeff = np.array((2*np.pi / 4096, 0.002059346625479734, 2*np.pi / 4096, 0.002059346625479734))
                shift = np.array((np.pi, 0, 0, 0))
                data = data * coeff + shift
                data[0] = data[0] - 2*np.pi if data[0] > np.pi else data[0]

                return cmd, data
            else:
                return cmd, (0,0,0,0)
        else:
            raise ValueError("Unsupported format")

    def get_state_time(self, _format: str = "raw"):
        if _format == 'raw':
            return self.__get_state()
        elif _format == 'float':
            cmd, _data = self.__get_state()
            if not cmd:
                data = np.array(_data)

                if abs(data[0]) >= 4096:
                    if data[0] > 0:
                        data[0] -= 4096 * int(abs(data[0]) / 4096)
                    else:
                        data[0] += 4096 * int(abs(data[0]) / 4096)
                coeff = np.array((2*np.pi / 4096, 0.002059346625479734, 2*np.pi / 4096, 0.002059346625479734, 1/1000))
                shift = np.array((np.pi, 0, 0, 0))
                data = data * coeff + shift
                data[0] = data[0] - 2*np.pi if data[0] > np.pi else data[0]
                return cmd, data
            else:
                return cmd, (0,0,0,0,0)
        else:
            raise ValueError("Unsupported format")

    def ping(self):
        self.send_request(PING.cmd, bytearray())
        cmd_error, data = self.receive_response(PING.rsp)
        return cmd_error



