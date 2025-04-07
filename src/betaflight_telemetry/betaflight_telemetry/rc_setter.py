import sys
import rclpy
from rclpy.node import Node
import threading
import termios
import tty
import select

from telemetry_interfaces.srv import SetRc

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


class SetRcChansClient(Node):

    def __init__(self):
        super().__init__('rc_set_client')
        self.cli = self.create_client(SetRc, 'rc_control')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # Store RC values from command line or use defaults
        if len(sys.argv) != 9:
            self.get_logger().info('Using default RC values')
            self.roll = 1500
            self.pitch = 1500
            self.yaw = 1500
            self.throttle = 1000
            self.aux1 = 1000
            self.aux2 = 1000
            self.aux3 = 1000
            self.aux4 = 1000
        else:
            self.roll = int(sys.argv[1])
            self.pitch = int(sys.argv[2])
            self.yaw = int(sys.argv[3])
            self.throttle = int(sys.argv[4])
            self.aux1 = int(sys.argv[5])
            self.aux2 = int(sys.argv[6])
            self.aux3 = int(sys.argv[7])
            self.aux4 = int(sys.argv[8])

        self.key_listener_thread = threading.Thread(target=self.key_listener, daemon=True)
        self.key_listener_thread.start()

        self.timer = self.create_timer(0.05, self.send_request)

        sys.stdout.write("\033[2J\033[H")
        sys.stdout.flush()

    def send_request(self):
        """Send RC channel values to the rc_control service."""
        req = SetRc.Request()
        req.roll = self.roll
        req.pitch = self.pitch
        req.yaw = self.yaw
        req.throttle = self.throttle
        req.aux1 = self.aux1
        req.aux2 = self.aux2
        req.aux3 = self.aux3
        req.aux4 = self.aux4

        future = self.cli.call_async(req)
        future.add_done_callback(self.request_callback)

    def request_callback(self, future):
        """Handle the response from the service call."""
        try:
            response = future.result()
            lines = [
                    f"{Colors.SOFT_YELLOW}---RC input values---{Colors.RESET}",

                    f"{Colors.YELLOW}YAW:{Colors.RESET} {self.yaw} | "
                    f"{Colors.RED}THROTTLE:{Colors.RESET} {self.throttle} | "
                    f"{Colors.CYAN}ROLL:{Colors.RESET} {self.roll} | "
                    f"{Colors.MAGENTA}PITCH:{Colors.RESET} {self.pitch} | "                    
                    f"{Colors.GREEN}AUX1:{Colors.RESET} {self.aux1} | "
                    f"{Colors.GREEN}AUX2:{Colors.RESET} {self.aux2} | "
                    f"{Colors.GREEN}AUX3:{Colors.RESET} {self.aux3} | "
                    f"{Colors.GREEN}AUX4:{Colors.RESET} {self.aux4}",

                    f"{Colors.SOFT_ORANGE}W/S - throtthle, A/D - yaw, I/K - pitch, J/L - roll{Colors.RESET}"
                ]
            total_lines = len(lines)

            # Move cursor up to overwrite previous block of lines
            sys.stdout.write(f"\033[{total_lines}A")

            for line in lines:
                sys.stdout.write(f"\r{line}\033[K\n")
            
            sys.stdout.flush()

        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def key_listener(self):
        """Listen for arrow key presses to switch modes"""
        key_mappings = {
        'w': (self.control_throttle, 10),
        'W': (self.control_throttle, 10),
        's': (self.control_throttle, -10),
        'S': (self.control_throttle, -10),
        'a': (self.control_yaw, -10),
        'A': (self.control_yaw, -10),
        'd': (self.control_yaw, 10),
        'D': (self.control_yaw, 10),
        'i': (self.control_pitch, 10),
        'I': (self.control_pitch, 10),
        'k': (self.control_pitch, -10),
        'K': (self.control_pitch, -10),
        'j': (self.control_roll, -10),
        'J': (self.control_roll, -10),
        'l': (self.control_roll, 10),
        'L': (self.control_roll, 10),
        'q': (self.center_yaw, None),
        'Q': (self.center_yaw, None),
        'u': (self.center_right_stick, None),
        'U': (self.center_right_stick, None),
        'e': (self.toggle_aux1, None),
        }
        
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        try:
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key in key_mappings:
                        method, value = key_mappings[key]
                        if value is None:
                            method()
                        else:
                            method(value)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def control_throttle(self, direction):
        new_throttle = self.throttle + direction
        self.throttle = max(1000, min(2000, new_throttle))

    def control_yaw(self, direction):
        new_yaw = self.yaw + direction
        self.yaw = max(1000, min(2000, new_yaw))

    def center_yaw(self):
        self.yaw = 1500
    
    def control_roll(self, direction):
        new_roll = self.roll + direction
        self.roll = max(1000, min(2000, new_roll))
    
    def control_pitch(self, direction):
        new_pitch = self.pitch + direction
        self.pitch = max(1000, min(2000, new_pitch))

    def center_right_stick(self):
        self.roll = 1500
        self.pitch = 1500

    def toggle_aux1(self):
        if self.aux1 == 1000:
            self.aux1 = 2000
        else:
            self.aux1 = 1000


        

def main(args=None):
    rclpy.init(args=args)

    rc_client = SetRcChansClient()

    try:
        rclpy.spin(rc_client)
    except KeyboardInterrupt:
        rc_client.get_logger().info('Shutting down RC setter node...')
    finally:
        rc_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()