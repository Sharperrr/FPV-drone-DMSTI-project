import rclpy
from rclpy.node import Node
import sys
import threading
import termios
import tty
import select
import time

from telemetry_interfaces.msg import Telemetry

GYRO_SENSITIVITY = 16.4
ACC_SENSITIVITY = 2048
GRAVITY = 9.81
COORDS_CONVERSION = 10000000

class Colors:
    RESET = "\033[0m"
    RED = "\033[31m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    BLUE = "\033[34m"
    MAGENTA = "\033[35m"
    CYAN = "\033[36m"
    WHITE = "\033[37m"
    SOFT_YELLOW = "\033[38;5;229m"
    SOFT_GREEN = "\033[38;5;151m"
    SOFT_ORANGE = "\033[38;5;216m"
    SOFT_PINK = "\033[38;5;218m"
    SOFT_BLUE = "\033[38;5;153m"


class TelemetryGUI(Node):
    def __init__(self):
        super().__init__('telemetry_gui')

        # Subscribe to betaflight_telemetry topic, that telemetry_node publishes to
        self.subscription = self.create_subscription(
            Telemetry,
            '/betaflight_telemetry',
            self.listener_callback,
            10
        )

        self.display_mode = 0
        self.data = None

        self.get_logger().info("Telemetry GUI started. Use LEFT/RIGHT arrow keys to switch modes.")

        # Start key listener in a separate thread
        self.key_listener_thread = threading.Thread(target=self.key_listener, daemon=True)
        self.key_listener_thread.start()

        sys.stdout.write("\033[2J\033[H")
        sys.stdout.flush()


    def change_mode(self, direction):
        """Change the display mode based on arrow key input"""
        self.display_mode = (self.display_mode + direction) % 7
        sys.stdout.write("\033[2J\033[H")
        sys.stdout.flush()
        self.update_display()

    def listener_callback(self, msg):
        """Store received telemetry data and update the display"""
        self.data = msg
        self.update_display()

    def update_display(self):
        """Update the terminal display based on the selected mode"""
        if self.data is None:
            return
        msg = self.data
        lines = []
        match self.display_mode:
            case 0:
                lines = [
                    f"{Colors.SOFT_YELLOW}---Raw inertial measurement unit readings---{Colors.RESET}",

                    f"| {Colors.RED}ACC X:{Colors.RESET} {msg.accx/ACC_SENSITIVITY*GRAVITY:.2f} m/s² | "
                    f"{Colors.GREEN}ACC Y:{Colors.RESET} {msg.accy/ACC_SENSITIVITY*GRAVITY:.2f} m/s² | "                    
                    f"{Colors.BLUE}ACC Z:{Colors.RESET} {msg.accz/ACC_SENSITIVITY*GRAVITY:.2f} m/s² |   "
                    f"| {Colors.RED}GYR X:{Colors.RESET} {msg.gyrx/GYRO_SENSITIVITY:.2f} deg/s | "
                    f"{Colors.GREEN}GYR Y:{Colors.RESET} {msg.gyry/GYRO_SENSITIVITY:.2f} deg/s | "                    
                    f"{Colors.BLUE}GYR Z:{Colors.RESET} {msg.gyrz/GYRO_SENSITIVITY:.2f} deg/s |   ",
                    
                    f"{Colors.WHITE}[Mode 1/6]{Colors.RESET}"
                ]
            case 1:
                lines = [
                    f"{Colors.SOFT_YELLOW}---Motor values---{Colors.RESET}",

                    f"{Colors.BLUE}Motor1:{Colors.RESET} {msg.motor1} | "
                    f"{Colors.BLUE}Motor2:{Colors.RESET} {msg.motor2} | "                    
                    f"{Colors.BLUE}Motor3:{Colors.RESET} {msg.motor3} | "
                    f"{Colors.BLUE}Motor4:{Colors.RESET} {msg.motor4}",
                    
                    f"{Colors.WHITE}[Mode 2/6]{Colors.RESET}"
                ]
            case 2:
                lines = [
                    f"{Colors.SOFT_YELLOW}---RC input values---{Colors.RESET}",

                    f"{Colors.YELLOW}YAW:{Colors.RESET} {msg.yaw} | "
                    f"{Colors.RED}THROTTLE:{Colors.RESET} {msg.throttle} | "
                    f"{Colors.CYAN}ROLL:{Colors.RESET} {msg.roll} | "
                    f"{Colors.MAGENTA}PITCH:{Colors.RESET} {msg.pitch} | "                    
                    f"{Colors.GREEN}AUX1:{Colors.RESET} {msg.aux1} | "
                    f"{Colors.GREEN}AUX2:{Colors.RESET} {msg.aux2} | "
                    f"{Colors.GREEN}AUX3:{Colors.RESET} {msg.aux3} | "
                    f"{Colors.GREEN}AUX4:{Colors.RESET} {msg.aux4} | "
                    f"{Colors.GREEN}AUX5:{Colors.RESET} {msg.aux5}",

                    f"{Colors.WHITE}[Mode 3/6]{Colors.RESET}"
                ]
            case 3:
                lines = [
                    f"{Colors.SOFT_YELLOW}---RAW GPS readings---{Colors.RESET}",

                    f"{Colors.SOFT_GREEN}GPS fix:{Colors.RESET} {msg.gpsfix} | "
                    f"{Colors.YELLOW}Number of satelites:{Colors.RESET} {msg.numsat}",

                    f"{Colors.SOFT_BLUE}Latitude:{Colors.RESET} {msg.latitude / COORDS_CONVERSION:.6f} | "
                    f"{Colors.SOFT_ORANGE}Longitude:{Colors.RESET} {msg.longitude / COORDS_CONVERSION:.6f}",

                    f"{Colors.RED}Altitude{Colors.RESET} {msg.altitude} m | "
                    f"{Colors.YELLOW}Speed:{Colors.RESET} {msg.speed} cm/s | "
                    f"{Colors.SOFT_PINK}Ground course:{Colors.RESET} {msg.groundcourse}",

                    f"{Colors.WHITE}[Mode 4/6]{Colors.RESET}"
                ]
            case 4:
                lines = [
                    f"{Colors.SOFT_YELLOW}---COMP GPS readings---{Colors.RESET}",

                    f"{Colors.RED}NOT USED{Colors.RESET}",

                    f"{Colors.WHITE}[Mode 5/6]{Colors.RESET}"
                ]
            case 5:
                lines = [
                    f"{Colors.SOFT_YELLOW}---Attitude and Altitude---{Colors.RESET}",

                    f"| {Colors.RED}ANG X:{Colors.RESET} {msg.angx/10:.1f}° | "
                    f"{Colors.GREEN}ANG Y:{Colors.RESET} {msg.angy/10:.1f}° | "
                    f"{Colors.YELLOW}Heading:{Colors.RESET} {msg.heading}° |   "
                    f"| {Colors.SOFT_BLUE}Estimated Altitude:{Colors.RESET} {msg.estalt} cm | "
                    f"{Colors.MAGENTA}Vario:{Colors.RESET} {msg.vario} cm/s |",

                    f"{Colors.WHITE}[Mode 6/6]{Colors.RESET}"
                ]
            

        total_lines = len(lines)

        # Move cursor up to overwrite previous block of lines
        sys.stdout.write(f"\033[{total_lines}A")

        for line in lines:
            sys.stdout.write(f"\r{line}\033[K\n")
        
        sys.stdout.flush()

    def key_listener(self):
        """Listen for arrow key presses to switch modes"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        try:
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(3)
                    if key == '\x1b[D':  # Left arrow
                        self.change_mode(-1)
                    elif key == '\x1b[C':  # Right arrow
                        self.change_mode(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryGUI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
