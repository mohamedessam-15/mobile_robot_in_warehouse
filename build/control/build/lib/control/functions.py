# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int32
# import time

# class ServoPublisher(Node):
#     def __init__(self):
#         super().__init__('servo_publisher')
#         self.publisher = self.create_publisher(Int32, 'servo_angle', 10)

#     def rotate_forward(self):
#         self.get_logger().info('Rotating from 0 to 90 degrees...')
#         angle = 0.0
#         while angle <= 90.0:
#             msg = Int32()
#             msg.data = int(angle)
#             self.publisher.publish(msg)
#             self.get_logger().info(f'Published angle: {int(angle)}')
#             angle += 1.5
#             time.sleep(0.01)
#         self.get_logger().info('Finished forward rotation.')

#     def rotate_reverse(self):
#         self.get_logger().info('Rotating from 90 to 0 degrees...')
#         angle = 90.0
#         while angle >= 0.0:
#             msg = Int32()
#             msg.data = int(angle)
#             self.publisher.publish(msg)
#             self.get_logger().info(f'Published angle: {int(angle)}')
#             angle -= 1.5
#             time.sleep(0.01)
#         self.get_logger().info('Finished reverse rotation.')

# def main(args=None):
#     rclpy.init(args=args)
#     node = ServoPublisher()

#     # Simulated conditions (replace with logic from your sensors)
#     simulate_condition_forward = True
#     simulate_condition_reverse = False

#     if simulate_condition_forward:
#         node.rotate_forward()
#     elif simulate_condition_reverse:
#         node.rotate_reverse()

#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Bool

# class IRSensor(Node):
#     def __init__(self):
#         super().__init__('ir_sensor_node')
#         self.subscription = self.create_subscription(
#             Bool,
#             'ir_detected',
#             self.ir_callback,
#             10
#         )

#     def ir_callback(self, msg):
#         if msg.data == 0:
#             self.get_logger().info('🔴 Object Detected')
#         else:
#             self.get_logger().info('⚪ No Object')

# def main(args=None):
#     rclpy.init(args=args)
#     node = IRSensor()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int32
# import time

# class MotorLogic(Node):
#     def __init__(self):
#         super().__init__('motor_logic_node')
#         self.publisher = self.create_publisher(Int32, 'motor_command', 10)

#     def move_forward(self):
#         self.get_logger().info('🚗 Moving Forward')
#         msg = Int32()
#         msg.data = 1
#         self.publisher.publish(msg)
#         time.sleep(3)  # run for 3 seconds
#         self.stop_motor()

#     def move_reverse(self):
#         self.get_logger().info('🔙 Moving Reverse')
#         msg = Int32()
#         msg.data = -1
#         self.publisher.publish(msg)
#         time.sleep(3)
#         self.stop_motor()

#     def stop_motor(self):
#         self.get_logger().info('⛔ Stopping Motor')
#         msg = Int32()
#         msg.data = 0
#         self.publisher.publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = MotorLogic()

#     # Replace these with your sensor-based conditions
#     condition_forward = True
#     condition_reverse = False

#     if condition_forward:
#         node.move_forward()
#     elif condition_reverse:
#         node.move_reverse()

#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Bool

# class LimitSwitchSubscriber(Node):
#     def __init__(self):
#         super().__init__('limit_switch_subscriber')
#         self.subscription = self.create_subscription(
#             Bool,
#             'limit_switch_state',
#             self.listener_callback,
#             10
#         )

#     def listener_callback(self, msg):
#         if msg.data:
#             self.get_logger().info('🔴 Limit switch is PRESSED')
#         else:
#             self.get_logger().info('⚪ Limit switch is NOT pressed')

# def main(args=None):
#     rclpy.init(args=args)
#     node = LimitSwitchSubscriber()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()









# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int32, Bool

# class IRProcessor(Node):
#     def __init__(self):
#         super().__init__('ir_processor_node')

#         self.ir_sub = self.create_subscription(Int32, 'ir_analog', self.ir_callback, 10)
#         self.led_pub = self.create_publisher(Bool, 'led_control', 10)

#         self.threshold = 3000  # Adjust based on lighting/sensor range

#     def ir_callback(self, msg):
#         self.get_logger().info(f'IR value: {msg.data}')
#         led_msg = Bool()
#         led_msg.data = msg.data < self.threshold  # LED ON if object detected
#         self.led_pub.publish(led_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = IRProcessor()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#      main()








import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
import time

class RobotInterfaces(Node):
    def __init__(self):
        super().__init__('robot_interfaces')

        # Publishers
        self.servo_publisher = self.create_publisher(Int32, 'servo_angle', 10)
            # Motor command publishers
        self.motor1_pub = self.create_publisher(Int32, 'motor1_command', 10)
        self.motor2_pub = self.create_publisher(Int32, 'motor2_command', 10)
        self.motor3_pub = self.create_publisher(Int32, 'motor3_command', 10)
        self.led_pub = self.create_publisher(Bool, 'led_control', 10)  # Second IR's LED control

        # Subscriptions
        self.create_subscription(Bool, 'ir_detected', self.ir_callback, 10)
        self.create_subscription(Int32, 'ir_analog', self.second_ir_callback, 10)  # From second IR sensor (analog)
        # Limit Switch Subscribers (Separate Callbacks)
        self.create_subscription(Bool, 'limit_switch1', self.limit_switch1_callback, 10)
        self.create_subscription(Bool, 'limit_switch2', self.limit_switch2_callback, 10)
        self.create_subscription(Bool, 'limit_switch3', self.limit_switch3_callback, 10)
        self.create_subscription(Bool, 'limit_switch4', self.limit_switch4_callback, 10)
        self.create_subscription(Bool, 'limit_switch5', self.limit_switch5_callback, 10)
        self.create_subscription(Bool, 'limit_switch6', self.limit_switch6_callback, 10)
        self.create_subscription(Bool, 'limit_switch7', self.limit_switch7_callback, 10)
        self.create_subscription(Bool, 'limit_switch8', self.limit_switch8_callback, 10)
        self.create_subscription(Bool, 'limit_switch9', self.limit_switch9_callback, 10)


        # State variables
        self.ir_detected = False
        self.limit_switch_pressed = False
        self.ir_threshold = 3000  # Threshold for second IR sensor

        # For each of the 9 switches
        self.switch_states = [False] * 9
        self.last_states = [False] * 9
        self.last_debounce_times = [0.0] * 9
        self.debounce_delay = 0.05  # 50 ms debounce

    # ==== Servo Functions ====
    def rotate_forward(self):
        self.get_logger().info('Rotating Servo: 0 to 90 degrees...')
        angle = 0.0
        while angle <= 90.0:
            msg = Int32()
            msg.data = int(angle)
            self.servo_publisher.publish(msg)
            angle += 1.5
            time.sleep(0.01)
        self.get_logger().info('Finished forward servo rotation.')

    def rotate_reverse(self):
        self.get_logger().info('Rotating Servo: 90 to 0 degrees...')
        angle = 90.0
        while angle >= 0.0:
            msg = Int32()
            msg.data = int(angle)
            self.servo_publisher.publish(msg)
            angle -= 1.5
            time.sleep(0.01)
        self.get_logger().info('Finished reverse servo rotation.')

    # ==== Motor Functions ====

    def move_motor1_forward(self):
        self.get_logger().info('🚗 Moving Motor 1 Forward')
        msg = Int32()
        msg.data = 1
        self.motor1_pub.publish(msg)

    def move_motor1_reverse(self):
        self.get_logger().info('🔙 Moving Motor 1 Reverse')
        msg = Int32()
        msg.data = -1
        self.motor1_pub.publish(msg)

    def stop_motor1(self):
        self.get_logger().info('⛔ Stopping Motor 1')
        msg = Int32()
        msg.data = 0
        self.motor1_pub.publish(msg)

    def move_motor2_forward(self):
        self.get_logger().info('🚗 Moving Motor 2 Forward')
        msg = Int32()
        msg.data = 1
        self.motor2_pub.publish(msg)

    def move_motor2_reverse(self):
        self.get_logger().info('🔙 Moving Motor 2 Reverse')
        msg = Int32()
        msg.data = -1
        self.motor2_pub.publish(msg)

    def stop_motor2(self):
        self.get_logger().info('⛔ Stopping Motor 2')
        msg = Int32()
        msg.data = 0
        self.motor2_pub.publish(msg)

    def move_motor3_forward(self):
        self.get_logger().info('🚗 Moving Motor 3 Forward')
        msg = Int32()
        msg.data = 1
        self.motor3_pub.publish(msg)

    def move_motor3_reverse(self):
        self.get_logger().info('🔙 Moving Motor 3 Reverse')
        msg = Int32()
        msg.data = -1
        self.motor3_pub.publish(msg)

    def stop_motor3(self):
        self.get_logger().info('⛔ Stopping Motor 3')
        msg = Int32()
        msg.data = 0
        self.motor3_pub.publish(msg)



    # ==== Sensor Callbacks ====
    def ir_callback(self, msg: Bool):
        self.ir_detected = bool(msg.data)
        if self.ir_detected:
            self.get_logger().info('⚪ No Object Detected')
        else:
            self.get_logger().info('🔴 Object Detected')




    # === Limit Switch Callbacks ===
    def limit_switch_callback(self, msg, index):
        current_time = time.time()
        state = msg.data
        if state != self.switch_states[index] and (current_time - self.last_debounce_times[index]) > self.debounce_delay:
            self.switch_states[index] = state
            self.last_debounce_times[index] = current_time
            if state:
                self.get_logger().info(f'🔴 Limit Switch {index + 1} PRESSED')
            else:
                self.get_logger().info(f'⚪ Limit Switch {index + 1} RELEASED')

    def limit_switch1_callback(self, msg): self.limit_switch_callback(msg, 0)
    def limit_switch2_callback(self, msg): self.limit_switch_callback(msg, 1)
    def limit_switch3_callback(self, msg): self.limit_switch_callback(msg, 2)
    def limit_switch4_callback(self, msg): self.limit_switch_callback(msg, 3)
    def limit_switch5_callback(self, msg): self.limit_switch_callback(msg, 4)
    def limit_switch6_callback(self, msg): self.limit_switch_callback(msg, 5)
    def limit_switch7_callback(self, msg): self.limit_switch_callback(msg, 6)
    def limit_switch8_callback(self, msg): self.limit_switch_callback(msg, 7)
    def limit_switch9_callback(self, msg): self.limit_switch_callback(msg, 8)

    def second_ir_callback(self, msg: Int32):
        ir_value = msg.data
        self.get_logger().info(f'Second IR Analog Value: {ir_value}')
        led_msg = Bool()
        led_msg.data = ir_value < self.ir_threshold
        self.led_pub.publish(led_msg)

        if led_msg.data:
            self.get_logger().info('🔴 Second IR Detected Object (LED ON)')
        else:
            self.get_logger().info('⚪ Second IR No Object (LED OFF)')


def main(args=None):
    rclpy.init(args=args)
    robot_interfaces = RobotInterfaces()
    rclpy.spin(robot_interfaces)
    robot_interfaces.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
