import socket
import struct

class sim_scara_motion_controller:
    def __init__(self, host='127.0.0.1', port=9000):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect((host, port))
        print(f"[CLIENT] connected to {host}:{port}")

    def send_command(self, command_bytes):
        print("[CLIENT] sending command...")
        self.client.sendall(command_bytes)

        reply = self.client.recv(4)
        (success_flag,) = struct.unpack('<i', reply)
        print(f"[CLIENT] received success flag: {success_flag}")

    def SCARA_INITIALISE(self, link_1, link_2, z_min, z_max, settling_time, P_0, I_0, D_0, P_1, I_1, D_1):
        command = struct.pack('<ifffffffffff', 1, link_1, link_2, z_min, z_max, settling_time, P_0, I_0, D_0, P_1, I_1, D_1)
        self.send_command(command)

    def SCARA_MOVE_JOINT(self, axis, angle, time):
        command = struct.pack('<iiffiiiiiiii', 3, axis, angle, time, 0, 0, 0, 0, 0, 0, 0, 0)
        self.send_command(command)

    def SCARA_MOVE_JOINTS(self, angle_0, angle_1, angle_2, angle_3, time):
        command = struct.pack('<ifffffiiiiii', 4, angle_0, angle_1, angle_2, angle_3, time, 0, 0, 0, 0, 0, 0)
        self.send_command(command)

    def SCARA_MOVE_COORD(self, x, y, z_angle, z, time):
        command = struct.pack('<ifffffiiiiii', 5, x, y, z_angle, z, time, 0, 0, 0, 0, 0, 0)
        self.send_command(command)
