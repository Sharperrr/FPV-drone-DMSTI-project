import sys
import rclpy
from rclpy.node import Node

from telemetry_interfaces.srv import SetMotors

# Probably shouldn't use this to actually fly, but it technically works.

class SetMotorsClient(Node):

    def __init__(self):
        super().__init__('motor_set_client')
        self.cli = self.create_client(SetMotors, 'motor_control')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetMotors.Request()

    def send_request(self):
        if not len(sys.argv) == 5:
            self.req.motor1 = 1000
            self.req.motor2 = 1000
            self.req.motor3 = 1000
            self.req.motor4 = 1000
        else:
            self.req.motor1 = int(sys.argv[1])
            self.req.motor2 = int(sys.argv[2])
            self.req.motor3 = int(sys.argv[3])
            self.req.motor4 = int(sys.argv[4])

        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    motor_client = SetMotorsClient()
    motor_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(motor_client)
        if motor_client.future.done():
            try:
                response = motor_client.future.result()
            except Exception as e:
                motor_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                motor_client.get_logger().info(
                    f'''
Setting motors to:
motor 1 - {motor_client.req.motor1}
motor 2 - {motor_client.req.motor2}
motor 3 - {motor_client.req.motor3}
motor 4 - {motor_client.req.motor4}
---                    
Motors set successfully? - {response.success}''')
            break

    motor_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()