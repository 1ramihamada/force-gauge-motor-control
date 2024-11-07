import serial
import time
from dynamixel_sdk import *  # Uses Dynamixel SDK library
import threading

# Settings for Mark-10 Force Gauge
FORCE_PORT = '/dev/ttyUSB2'  # Serial port for force gauge
FORCE_BAUD_RATE = 115200
FORCE_THRESHOLD = 1  # Force threshold in Newtons for movement

# Dynamixel Settings
DXL_ID = 1
DXL_PORT = '/dev/ttyUSB1'  # Port for Dynamixel motor
DXL_BAUDRATE = 57600
PROTOCOL_VERSION = 2.0
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
EXTENDED_POSITION_CONTROL_MODE = 4
MIN_POSITION = -1_000_000
MAX_POSITION = 1_000_000
POSITION_INCREMENT = 1000  # Amount to move with each increment
v_target = 1000
a_target = 100

# Control table addresses
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_OPERATING_MODE = 11
ADDR_PROFILE_VELOCITY = 112
ADDR_PROFILE_ACCELERATION = 108


class DynamixelController:
    def __init__(self):
        self.lock = threading.Lock()
        self.portHandler = None
        self.initialize_dynamixel()
        self.current_position = 0

    def initialize_dynamixel(self):
        try:
            # Initialize PortHandler and PacketHandler
            self.portHandler = PortHandler(DXL_PORT)
            self.packetHandler = PacketHandler(PROTOCOL_VERSION)

            if not self.portHandler.openPort():
                raise Exception("Failed to open Dynamixel port")

            if not self.portHandler.setBaudRate(DXL_BAUDRATE):
                raise Exception("Failed to set Dynamixel baudrate")

            self.disable_torque()
            self.set_operating_mode(EXTENDED_POSITION_CONTROL_MODE)
            self.enable_torque()
            self.set_profile_velocity(v_target)
            self.set_profile_acceleration(a_target)

        except Exception as e:
            print(f"Dynamixel Initialization Error: {e}")
            self.stop()  # Ensure resources are cleaned up

    def enable_torque(self):
        with self.lock:
            result, error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if result != COMM_SUCCESS or error != 0:
                raise Exception("Failed to enable torque")

    def disable_torque(self):
        with self.lock:
            result, error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            if result != COMM_SUCCESS or error != 0:
                raise Exception("Failed to disable torque")

    def set_operating_mode(self, mode):
        with self.lock:
            result, error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_OPERATING_MODE, mode)
            if result != COMM_SUCCESS or error != 0:
                raise Exception("Failed to set operating mode")

    def set_profile_velocity(self, velocity):
        with self.lock:
            result, error = self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID, ADDR_PROFILE_VELOCITY, velocity)
            if result != COMM_SUCCESS or error != 0:
                raise Exception("Failed to set profile velocity")

    def set_profile_acceleration(self, acceleration):
        with self.lock:
            result, error = self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID, ADDR_PROFILE_ACCELERATION, acceleration)
            if result != COMM_SUCCESS or error != 0:
                raise Exception("Failed to set profile acceleration")

    def set_goal_position(self, goal_position):
        goal_position = max(min(goal_position, MAX_POSITION), MIN_POSITION)
        with self.lock:
            result, error = self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID, ADDR_GOAL_POSITION, int(goal_position))
            if result != COMM_SUCCESS or error != 0:
                print("Failed to set goal position")

    def adjust_position(self, direction):
        if direction == 0:
            return  # Do not change position if direction is zero
        self.current_position += direction * POSITION_INCREMENT
        self.set_goal_position(self.current_position)


    def stop(self):
        # Disable torque and close port if opened
        try:
            self.disable_torque()
        except Exception:
            pass
        if self.portHandler and self.portHandler.is_open:
            self.portHandler.closePort()
            print("Dynamixel port closed.")

    def get_present_position(self):
        with self.lock:
            position, result, error = self.packetHandler.read4ByteTxRx(
                self.portHandler, DXL_ID, ADDR_PRESENT_POSITION)
            if result != COMM_SUCCESS:
                print(f"Failed to read present position: {self.packetHandler.getTxRxResult(result)}")
                return None
            elif error != 0:
                print(f"Dynamixel error while reading present position: {self.packetHandler.getRxPacketError(error)}")
                return None

            # Convert from unsigned to signed integer if necessary
            if position > 0x7FFFFFFF:
                position -= 0x100000000  # 2^32

            return position


class ForceMotorControl:
    def __init__(self, dynamixel_controller):
        self.dynamixel_controller = dynamixel_controller
        self.serial_port = serial.Serial(FORCE_PORT, baudrate=FORCE_BAUD_RATE, timeout=1)
        self.running = True

    def read_force(self):
        try:
            self.serial_port.write(b'?\r')
            time.sleep(0.02)  # Reduced from 0.1 to 0.02 seconds
            response = self.serial_port.readline().decode('utf-8').strip()
            return float(response) if response else None
        except Exception as e:
            print(f"Error reading force gauge: {e}")
            return None


        
    def control_motor(self):
        force_readings = []  # List to hold recent force readings
        smoothing_window = 2  # Number of readings to average

        while self.running:
            force = self.read_force()

            if force is not None:
                # Maintain a list of recent force readings for averaging
                force_readings.append(force)
                if len(force_readings) > smoothing_window:
                    force_readings.pop(0)  # Remove the oldest reading

                # Calculate the average force over the recent readings
                avg_force = sum(force_readings) / len(force_readings)

                # Implement dead zone with averaged force
                if avg_force >= FORCE_THRESHOLD:
                    print("Moving Up")
                    self.dynamixel_controller.adjust_position(-1)
                elif avg_force <= -FORCE_THRESHOLD:
                    print("Moving Down")
                    self.dynamixel_controller.adjust_position(1)
                else:
                    print("Force in dead zone, stopping motor")
                    # Get the motor's actual present position
                    present_position = self.dynamixel_controller.get_present_position()
                    if present_position is not None:
                        self.dynamixel_controller.set_goal_position(present_position)
                        # Update current_position to the actual position
                        self.dynamixel_controller.current_position = present_position

            time.sleep(0.01)  # Small delay for stability

    def stop(self):
        self.running = False
        self.serial_port.close()
        self.dynamixel_controller.stop()


def main():
    dynamixel_controller = DynamixelController()
    force_motor_control = ForceMotorControl(dynamixel_controller)

    try:
        motor_thread = threading.Thread(target=force_motor_control.control_motor)
        motor_thread.start()
        motor_thread.join()

    except KeyboardInterrupt:
        print("Stopping motor control...")
        force_motor_control.stop()


if __name__ == "__main__":
    main()
