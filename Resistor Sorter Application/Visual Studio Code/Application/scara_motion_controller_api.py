import serial
import struct

class scara_motion_controller:
    def __init__(self, port, baudrate=115200, timeout=None):
        self.api = serial.Serial(port, baudrate, timeout=timeout)
        pass
    
    def send_command(self, command):
        print("sending command ...")
        self.api.write(command)

        success_flag = self.api.read(4)

        if len(success_flag) != 4:
            raise TimeoutError("system failure")
        
        flag, = struct.unpack('<i', success_flag)

        if flag != 1:
            raise ValueError(f"unexpected flag received: {flag}")
        
        print("... command successful")

    def SCARA_INITIALISE(self, link_1, link_2, z_min, z_max, settling_time, P_0, I_0, D_0, P_1, I_1, D_1):

        # |0001   |aaaa  |bbbb  |cccc |dddd |eeee         |ffff|gggg|hhhh|iiii|jjjj|kkkk|
        # |command|link_1|link_2|z_min|z_max|settling_time|P_0 |I_0 |D_0 |P_1 |I_1 |D_1 |

        # stm32f446re bytes are coded in memory in little endian format
        command = struct.pack('<ifffffffffff', 1, link_1, link_2, z_min, z_max, settling_time, P_0, I_0, D_0, P_1, I_1, D_1)
        self.send_command(command)
    
    def SCARA_AUTO_CALIBRATE(self):
        
        # |0002   |xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|
        # |command|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|

        command = struct.pack('<iiiiiiiiiiii', 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        self.send_command(command)
    
    def SCARA_MOVE_JOINT(self, axis, angle, time):

        # |0003   |aaaa|bbbb |cccc|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|
        # |command|axis|angle|time|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|

        command = struct.pack('<iiffiiiiiiii', 3, axis, angle, time, 0, 0, 0, 0, 0, 0, 0, 0)
        self.send_command(command)
    
    def SCARA_MOVE_JOINTS(self, angle_0, angle_1, angle_2, angle_3, time):

        # |0004   |aaaa   |bbbb   |cccc   |dddd   |eeee|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|
        # |command|angle_0|angle_1|angle_2|angle_3|time|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|

        command = struct.pack('<ifffffiiiiii', 4, angle_0, angle_1, angle_2, angle_3, time, 0, 0, 0, 0, 0, 0)
        self.send_command(command)

    def SCARA_MOVE_COORD(self, x, y, z_angle, z, time):

        # |0005   |aaaa|bbbb|cccc   |dddd|eeee|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|
        # |command|x   |y   |z_angle|z   |time|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|

        command = struct.pack('<ifffffiiiiii', 5, x, y, z_angle, z, time, 0, 0, 0, 0, 0, 0)
        self.send_command(command)
    
    def SCARA_READ_ANGLE(self, axis):
        # |0006   |aaaa|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|
        # |command|axis|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|

        command = struct.pack('<iiiiiiiiiiii', 6, axis, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        self.send_command(command)

        # |0006   |aaaa |xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|
		# |command|angle|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|
        # Read the float response (4 bytes)
        response = self.api.read(4)

        if len(response) != 4:
            raise TimeoutError("no response received")

        angle, = struct.unpack('<f', response)
        return angle

    def SCARA_READ_COORD(self):
        # |0007   |xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|
        # |command|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|

        command = struct.pack('<iiiiiiiiiiii', 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        self.send_command(command)

        # |0007   |aaaa|bbbb|cccc   |dddd|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|
        # |command|x   |y   |z_angle|z   |xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|xxxx|
        # Read 4 floats = 16 bytes
        response = self.api.read(16)

        if len(response) != 16:
            raise TimeoutError("No response received")

        x, y, z_angle, z = struct.unpack('<ffff', response)
        return x, y, z_angle, z