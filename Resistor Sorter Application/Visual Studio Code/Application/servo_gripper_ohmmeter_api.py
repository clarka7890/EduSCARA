import serial
import time

class servo_gripper_ohmmeter:
    def __init__(self, port, baudrate=9600, timeout=None):
        self.api = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)
        pass

    def send_command(self, cmd: str):
        self.api.write((cmd + '\n').encode())
        response = self.api.readline().decode().strip()
        print(f"GRIPPER: {response}")
        return response

    def OPEN(self):
        self.send_command("OPEN")

    def CLOSE(self):
        self.send_command("CLOSE")

    def IDLE(self):
        self.send_command("IDLE")

    def TEST(self):
        self.send_command("TEST")
        return self.send_command("TEST")
    
