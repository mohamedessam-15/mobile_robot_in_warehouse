# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Bool, Int32, String
# import time

# from control.functions import RobotInterfaces  # Assuming this is in the same package

# class RobotLogic(Node):
#     def __init__(self):
#         super().__init__('robot_logic')

#         # Interfaces
#         self.interfaces = RobotInterfaces()

#         # State
#         self.floor_status = [False, False, False]  # Track if floors are busy

#         # Subscriptions
#         self.create_subscription(String, 'belt_command', self.belt_callback, 10)

#     def belt_callback(self, msg: String):
#         self.get_logger().info(f"📦 Belt sent command: {msg.data}")
#         if msg.data == 'pickup':
#             self.pick_and_store()

#     def pick_and_store(self):
#         # Step 1: Rotate servo to 90 degrees
#         self.interfaces.rotate_forward()

#         # Step 2: Move motor1 upward until IR1 cuts
#         self.interfaces.move_forward()
#         while self.interfaces.ir_detected:
#             rclpy.spin_once(self.interfaces, timeout_sec=0.1)
#         self.interfaces.stop_motor()

#         # Step 3: Move motor2 forward until IR2 is NOT cutting anymore
#         self.interfaces.motor_publisher.publish(Int32(data=1))
#         while True:
#             rclpy.spin_once(self.interfaces, timeout_sec=0.1)
#             if self.interfaces.ir_detected:  # IR2 initially cuts
#                 while self.interfaces.ir_detected:
#                     rclpy.spin_once(self.interfaces, timeout_sec=0.1)
#                 break
#         self.interfaces.motor_publisher.publish(Int32(data=0))

#         # Step 4: Close gripper (motor3) until S1 & S2 are pressed
#         self.interfaces.motor_publisher.publish(Int32(data=1))  # Motor3 forward
#         while not (self.interfaces.limit_switch_pressed):
#             rclpy.spin_once(self.interfaces, timeout_sec=0.1)
#         self.interfaces.motor_publisher.publish(Int32(data=0))

#         # Step 5: Reverse motor2 until S3 & S4 are pressed
#         self.interfaces.motor_publisher.publish(Int32(data=-1))
#         time.sleep(1.5)  # or poll limit switches properly
#         self.interfaces.motor_publisher.publish(Int32(data=0))

#         # Step 6: Rotate servo to 0 degrees
#         self.interfaces.rotate_reverse()

#         # Step 7: Move motor1 down until S5 & S6 are pressed
#         self.interfaces.move_reverse()
#         time.sleep(2.0)  # Replace with real polling logic
#         self.interfaces.stop_motor()

#         # Step 8: Raise to first available floor
#         for i, busy in enumerate(self.floor_status):
#             if not busy:
#                 floor = i + 1
#                 self.floor_status[i] = True
#                 self.get_logger().info(f"🚧 Sending to floor {floor}")
#                 break
#         else:
#             self.get_logger().info("📦 All floors are full. Going to shelves...")
#             self.deliver_to_shelves()
#             return

#         # Move up again
#         self.interfaces.move_forward()
#         time.sleep(floor * 1.5)  # crude timing based on floor
#         self.interfaces.stop_motor()

#         # Move forward (motor2) to place item
#         self.interfaces.motor_publisher.publish(Int32(data=1))
#         time.sleep(2.0)
#         self.interfaces.motor_publisher.publish(Int32(data=0))

#         # Open gripper (motor3) until S1 & S2 released
#         self.interfaces.motor_publisher.publish(Int32(data=-1))
#         time.sleep(1.5)
#         self.interfaces.motor_publisher.publish(Int32(data=0))

#         # Reverse motor2
#         self.interfaces.motor_publisher.publish(Int32(data=-1))
#         time.sleep(2.0)
#         self.interfaces.motor_publisher.publish(Int32(data=0))

#         # Reset gripper
#         self.interfaces.motor_publisher.publish(Int32(data=1))
#         time.sleep(1.5)
#         self.interfaces.motor_publisher.publish(Int32(data=0))

#         self.get_logger().info("✅ Product stored")

#     def deliver_to_shelves(self):
#         # TODO: Implement navigation and drop-off at shelves
#         self.get_logger().info("🚚 Navigating to shelves for unloading...")
#         # Reset floor statuses
#         self.floor_status = [False, False, False]


# def main(args=None):
#     rclpy.init(args=args)
#     node = RobotLogic()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

# logic_node.py
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Bool, Int32

# from control.functions import RobotInterfaces  # Make sure the import path is correct

# class LogicNode(Node):
#     def __init__(self):
#         super().__init__('logic_node')

#         # Instantiate RobotInterfaces as a component in this node
#         self.robot = RobotInterfaces()

#         # Set timer to run logic loop at 10Hz
#         self.timer = self.create_timer(0.1, self.control_logic)

#         # Internal state machine
#         self.state = 'IDLE'

#     def control_logic(self):
#         # Step 1: Wait for limit switch 9 (index 8) to be pressed
#         if self.state == 'IDLE' and self.robot.switch_states[0]:
#             self.get_logger().info('➡️ Limit Switch 9 pressed: Rotating servo forward.')
#             self.robot.rotate_forward()
#             self.state = 'SERVO_DONE'

#         # Step 2: After servo rotates, move motor 1 forward
#         elif self.state == 'SERVO_DONE':
#             self.get_logger().info('⚙️ Servo rotated: Moving Motor 1 forward.')
#             self.robot.move_motor1_forward()
#             self.state = 'MOVING'

#         # Step 3: Stop motor 1 when IR sensor detects object
#         elif self.state == 'MOVING' and not self.robot.ir_detected:
#             self.get_logger().info('🛑 IR sensor triggered: Stopping Motor 1.')
#             self.robot.stop_motor1()
#             self.state = 'IDLE'

# def main(args=None):
#     rclpy.init(args=args)
#     logic_node = LogicNode()
#     rclpy.spin(logic_node)
#     logic_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()






























import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
import time

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Publishers
        self.servo_publisher = self.create_publisher(Int32, 'servo_angle', 10)
        self.motor1_pub = self.create_publisher(Int32, 'motor1_command', 10)
        self.motor2_pub = self.create_publisher(Int32, 'motor2_command', 10)
        self.motor3_pub = self.create_publisher(Int32, 'motor3_command', 10)
        # self.led_pub = self.create_publisher(Bool, 'led_control', 10)

        # Subscriptions
        # self.create_subscription(Bool, 'ir_detected', self.ir_callback, 10)
        # self.create_subscription(Int32, 'ir_analog', self.second_ir_callback, 10)
        for i in range(9):
            self.create_subscription(Bool, f'limit_switch{i+1}', lambda msg, i=i: self.limit_switch_callback(msg, i), 10)

        # State
        self.ir_debounce_time = 0.2  # seconds
        self.ir_trigger_time = None
        self.ir_detected = False
        self.ir_threshold = 3000
        self.ir_analog_value = 0
        self.switch_states = [False] * 9
        self.last_debounce_times = [0.0] * 9
        self.debounce_delay = 0.05  # 50ms

        self.state = 'IDLE'
        self.ir_phase = 'WAIT_FOR_OBJECT'  # sub-phase tracking for IR behavior

        self.floor_status = [False, False, False]  # False = available, True = busy

        self.create_timer(0.1, self.control_logic)

    # ==== Logic ====
    def control_logic(self):
        # if self.state == 'IDLE' and self.switch_states[0]:  # limit switch 9
        #     self.get_logger().info('➡️ Limit Switch 9 pressed: Rotating servo forward.')
        #     self.pick_from_belt_or_shelf()

        # if self.state == 'MOTOR1_STOP' and self.switch_states[5] and self.switch_states[6]:
        #     self.place_in_mobile_robot()
        if self.state == 'IDLE': 
            self.get_logger().info(f'state: {self.state}')
            self.rotate_forward
            self.state = 'SERVO_DONE_FORWARD'
            self.get_logger().info(f'state: {self.state}')

        if self.state == 'SERVO_DONE_FORWARD':
            self.move_motor2_forward()
            self.state = 'MOTOR2_FORWARD_1'
            self.get_logger().info(f'state: {self.state}')
            time.sleep(3.0)

        if self.state == 'MOTOR2_FORWARD_1':
            self.stop_motor2()
            self.state = 'MOTOR2_STOP_1'
            self.get_logger().info(f'state: {self.state}')
            self.move_motor3_forward()
            self.state = 'MOTOR3_FORWARD_1'
            self.get_logger().info(f'state: {self.state}')
        
        if self.state == 'MOTOR3_FORWARD_1':
            if self.switch_states[1] and self.switch_states[2]:
                self.stop_motor3() 
                self.state = 'MOTOR3_STOP_1'
                self.get_logger().info(f'state: {self.state}')
                self.move_motor2_reverse()
                self.state = 'MOTOR2_REVERSE_1'
                self.get_logger().info(f'state: {self.state}')
        
        if self.state == 'MOTOR2_REVERSE_1':
            if self.switch_states[3] and self.switch_states[4]:
                self.stop_motor2()
                self.state = 'MOTOR2_STOP_2'
                self.get_logger().info(f'state: {self.state}')
                self.rotate_reverse()
                self.state = 'SERVO_DONE_REVERSE'
                self.get_logger().info(f'state: {self.state}')

        if self.state == 'SERVO_DONE_REVERSE':
            self.move_motor2_forward()
            self.state = 'MOTOR2_FORWARD_2'
            self.get_logger().info(f'state {self.state}')
            time.sleep(3.0)

        if self.state == 'MOTOR2_FORWARD_2':
            self.stop_motor2()
            self.state = 'MOTOR2_STOP_3'
            self.get_logger().info(f'state: {self.state}')
            self.move_motor3_reverse()
            self.state = 'MOTOR3_REVERSE_1'
            self.get_logger().info(f'state: {self.state}')

        if self.state == 'MOTOR3_REVERSE_1':
            if self.switch_states[5] and self.switch_states[6]:
                self.stop_motor3() 
                self.state = 'MOTOR3_STOP_2'
                self.get_logger().info(f'state: {self.state}')
                self.move_motor2_reverse()
                self.state = 'MOTOR2_REVERSE_2'
                self.get_logger().info(f'state: {self.state}')


        if self.state == 'MOTOR2_REVERSE_2':
            if self.switch_states[3] and self.switch_states[4]:
                self.stop_motor2()
                self.state = 'MOTOR2_STOP_4'
                self.get_logger().info(f'state: {self.state}')
                self.state = 'IDLE'
                self.get_logger().info(f'state: {self.state}')



        

                


        
        
        






    # ==== Actuator Functions ====
    def rotate_forward(self):
        #self.get_logger().info('Rotating Servo: 0 to 90 degrees...')
        angle = 0.0
        while angle <= 90.0:
            msg = Int32()
            msg.data = int(angle)
            self.servo_publisher.publish(msg)
            angle += 1.5
            time.sleep(0.1)
        #self.get_logger().info('Finished forward servo rotation.')

    def rotate_reverse(self):
        #self.get_logger().info('Rotating Servo: 90 to 0 degrees...')
        angle = 90.0
        while angle >= 0.0:
            msg = Int32()
            msg.data = int(angle)
            self.servo_publisher.publish(msg)
            angle -= 1.5
            time.sleep(0.01)
        #self.get_logger().info('Finished reverse servo rotation.')

    def move_motor1_forward(self):
        msg = Int32()
        msg.data = 1
        for _ in range(5):  # publish 5 times
            self.motor1_pub.publish(msg)
            time.sleep(0.02)  # small delay
        #self.get_logger().info('🚗 Moving Motor 1 Forward')

    def move_motor1_reverse(self):
        msg = Int32()
        msg.data = -1
        for _ in range(5):  # publish 5 times
            self.motor1_pub.publish(msg)
            time.sleep(0.02)  # small delay
        #self.get_logger().info('🔙 Moving Motor 1 Reverse')

    def stop_motor1(self):
        msg = Int32()
        msg.data = 0
        for _ in range(5):
            self.motor1_pub.publish(msg)
            time.sleep(0.02)
        #self.get_logger().info('⛔ Stopping Motor 1')

    def move_motor2_forward(self):
        msg = Int32()
        msg.data = 1
        for _ in range(5):  # publish 5 times
            self.motor2_pub.publish(msg)
            time.sleep(0.02)  # small delay)

    def move_motor2_reverse(self):
        msg = Int32()
        msg.data = -1
        for _ in range(5):  # publish 5 times
            self.motor2_pub.publish(msg)
            time.sleep(0.02)  # small delay

    def stop_motor2(self):
        msg = Int32()
        msg.data = 0
        for _ in range(5):  # publish 5 times
            self.motor2_pub.publish(msg)
            time.sleep(0.02)  # small delay

    def move_motor3_forward(self):
        msg = Int32()
        msg.data = 1
        for _ in range(5):  # publish 5 times
            self.motor3_pub.publish(msg)
            time.sleep(0.02)  # small delay

    def move_motor3_reverse(self):
        msg = Int32()
        msg.data = -1
        for _ in range(5):  # publish 5 times
            self.motor3_pub.publish(msg)
            time.sleep(0.02)  # small delay

    def stop_motor3(self):
        msg = Int32()
        msg.data = 0
        for _ in range(5):  # publish 5 times
            self.motor3_pub.publish(msg)
            time.sleep(0.02)  # small delay

    # ==== Callbacks ====
    def ir_callback(self, msg: Bool):
        self.ir_detected = bool(msg.data)
        # if self.ir_detected:
        #     self.get_logger().info('⚪ No Object Detected')
        # else:
        #     self.get_logger().info('🔴 Object Detected')

    def second_ir_callback(self, msg: Int32):
        self.ir_analog_value = msg.data
        #self.get_logger().info(f'Second IR Analog Value: {ir_value}')
        led_msg = Bool()
        led_msg.data = self.ir_analog_value < self.ir_threshold
        self.led_pub.publish(led_msg)

        # if led_msg.data:
        #     self.get_logger().info('🔴 Second IR Detected Object (LED ON)')
        # else:
        #     self.get_logger().info('⚪ Second IR No Object (LED OFF)')

    # === Limit Switch Callbacks ===
    def limit_switch_callback(self, msg, index):
        current_time = time.time()
        state = msg.data
        if state != self.switch_states[index] and (current_time - self.last_debounce_times[index]) > self.debounce_delay:
            self.switch_states[index] = state
            self.last_debounce_times[index] = current_time
            # if state:
            #     self.get_logger().info(f'🔴 Limit Switch {index + 1} PRESSED')
            # else:
            #     self.get_logger().info(f'⚪ Limit Switch {index + 1} RELEASED')

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

        # if led_msg.data:
        #     self.get_logger().info('🔴 Second IR Detected Object (LED ON)')
        # else:
        #     self.get_logger().info('⚪ Second IR No Object (LED OFF)')



    def pick_from_belt_or_shelf(self):
        if self.state == 'IDLE':
            self.rotate_forward()
            self.state = 'SERVO_DONE'

        if self.state == 'SERVO_DONE':
            self.get_logger().info('⚙️ Servo rotated: Moving Motor 1 forward.')
            self.move_motor1_forward()
            self.state = 'MOTOR1_FORWARD'

        if self.state == 'MOTOR1_FORWARD':
            if self.ir_detected:
                if self.ir_trigger_time is None:
                    self.ir_trigger_time = time.time()
                elif time.time() - self.ir_trigger_time >= self.ir_debounce_time:
                    self.get_logger().info('🛑 IR sensor triggered (debounced): Stopping Motor 1.')
                    self.stop_motor1()
                    self.move_motor2_forward()
                    self.state = 'MOTOR2_FORWARD'
                    self.ir_phase = 'WAIT_FOR_OBJECT'
                    self.ir_trigger_time = None
            else:
                self.ir_trigger_time = None

        if self.state == 'MOTOR2_FORWARD':
            if self.ir_phase == 'WAIT_FOR_OBJECT':
                if self.ir_analog_value < self.ir_threshold:
                    self.get_logger().info('👀 Object detected by analog IR.')
                    self.ir_phase = 'WAIT_FOR_OBJECT_GONE'

            elif self.ir_phase == 'WAIT_FOR_OBJECT_GONE':
                if self.ir_analog_value >= self.ir_threshold:
                    self.get_logger().info('🛑 Object no longer detected. Stopping Motor 2.')
                    self.stop_motor2()
                    self.ir_phase = 'WAIT_FOR_OBJECT'
                    self.move_motor3_forward()
                    self.state = 'MOTOR3_FORWARD'

        if self.state == 'MOTOR3_FORWARD':
            if self.switch_states[1] and self.switch_states[2]:  # limit switch 2 & 3
                self.get_logger().info('🛑 Limit switches 2 and 3 pressed: Stopping Motor 3.')
                self.stop_motor3()
                self.move_motor2_reverse()
                self.state = 'MOTOR2_REVERSE'

        if self.state == 'MOTOR2_REVERSE':
            if self.switch_states[3] and self.switch_states[4]:  # limit switch 4 & 5
                self.get_logger().info('🔙 Limit switches 4 and 5 pressed: Stopping Motor 2.')
                self.stop_motor2()
                self.rotate_reverse()
                self.state = 'SERVO_REVERSE'

        if self.state == 'SERVO_REVERSE':
            self.get_logger().info('🔁 Servo rotated reverse: Moving Motor 1 reverse.')
            self.move_motor1_reverse()
            self.state = 'MOTOR1_REVERSE'

        if self.state == 'MOTOR1_REVERSE':
            if self.switch_states[5] and self.switch_states[6]:  # limit switch 6 & 7
                self.get_logger().info('✅ Limit switches 6 and 7 pressed: Stopping Motor 1.')
                self.stop_motor1()
                self.state = 'MOTOR1_STOP'

        
    
    def pick_from_shelf(self):

    
        return
    
    def pick_from_mobile_robot(self):
        self.get_logger().info('🤖 Starting package pickup from mobile robot...')

        try:
            floor_index = self.floor_status.index(True)  # first occupied floor
        except ValueError:
            self.get_logger().warn('📭 All 3 floors are empty! Nothing to pick.')
            return

        self.get_logger().info(f'📤 Picking package from floor {floor_index + 1}')

        # Step 1: Move to the correct floor by counting IR detections
        self.ir_detection_count = 0
        self.move_motor1_forward()
        self.state = 'MOTOR1_FORWARD'
        time.sleep(0.2)

        while self.ir_detection_count < (floor_index + 1):
            if self.state == 'MOTOR1_FORWARD' and self.ir_detected:
                self.ir_detection_count += 1
                self.get_logger().info(f'🔢 IR detected {self.ir_detection_count} time(s)...')
                while self.ir_detected:
                    time.sleep(0.05)
            time.sleep(0.05)

        self.stop_motor1()
        self.get_logger().info(f'✅ Reached floor {floor_index + 1}, starting pickup...')

        # Step 2: Lower motor 2 with IR-based object detection (like in pick_from_belt_or_shelf)
        self.ir_phase = 'WAIT_FOR_OBJECT'
        self.move_motor2_forward()
        self.state = 'MOTOR2_FORWARD'

        if self.state == 'MOTOR2_FORWARD':
            if self.ir_phase == 'WAIT_FOR_OBJECT':
                if self.ir_analog_value < self.ir_threshold:
                    self.get_logger().info('👀 Object detected by analog IR.')
                    self.ir_phase = 'WAIT_FOR_OBJECT_GONE'

            elif self.ir_phase == 'WAIT_FOR_OBJECT_GONE':
                if self.ir_analog_value >= self.ir_threshold:
                    self.get_logger().info('🛑 Object no longer detected. Stopping Motor 2.')
                    self.stop_motor2()
                    self.move_motor3_forward()
                    self.state = 'MOTOR3_FORWARD'

        if self.state == 'MOTOR3_FORWARD':
            if self.switch_states[7] and self.switch_states[8]:  # Limit switches 8 & 9
                self.get_logger().info('📦 Package picked: Stopping Motor 3.')
                self.stop_motor3()
                self.move_motor2_reverse()
                self.state = 'MOTOR2_REVERSE'

        if self.state == 'MOTOR2_REVERSE':
            if self.switch_states[1] and self.switch_states[2]:  # Limit switches 2 & 3
                self.get_logger().info('⬆️ Mechanism lifted: Stopping Motor 2.')
                self.stop_motor2()
                self.floor_status[floor_index] = False
                self.get_logger().info(f'✅ Package picked from floor {floor_index + 1}. Updated floor status: {self.floor_status}')
                self.state = 'PICKUP_DONE'
    
    def place_in_belt(self):
        self.move_motor1_reverse()
        self.state = 'MOTOR1_REVERSE'
        
        if self.state == 'MOTOR1_REVERSE':
            if self.switch_states[5] and self.switch_states[6]:  # limit switch 6 & 7
                self.get_logger().info('✅ Limit switches 6 and 7 pressed: Stopping Motor 1.')
                self.stop_motor1()
                self.state = 'MOTOR1_STOP'

        self.rotate_forward()
        self.state = 'SERVO_DONE'

        if self.state == 'SERVO_DONE':
            self.get_logger().info('⚙️ Servo rotated: Moving Motor 1 forward.')
            self.move_motor1_forward()
            self.state = 'MOTOR1_FORWARD'

        if self.state == 'MOTOR1_FORWARD':
            if self.ir_detected:
                if self.ir_trigger_time is None:
                    self.ir_trigger_time = time.time()
                elif time.time() - self.ir_trigger_time >= self.ir_debounce_time:
                    self.get_logger().info('🛑 IR sensor triggered (debounced): Stopping Motor 1.')
                    self.stop_motor1()
                    self.ir_trigger_time = None
            else:
                self.ir_trigger_time = None

        self.move_motor2_forward()
        self.state = 'MOTOR2_FORWARD'
        time.sleep(1.0)  # optional: replace with limit switch later
        self.stop_motor2()

        # Step 3: Drop the package
        self.move_motor3_reverse()
        self.state = 'MOTOR3_REVERSE'

        if self.state == 'MOTOR3_REVERSE':
            if self.switch_states[7] and self.switch_states[8]:
                self.stop_motor3()
                self.move_motor2_reverse()
                self.state = 'MOTOR2_REVERSE'

        if self.state == 'MOTOR2_REVERSE':
            if self.switch_states[1] and self.switch_states[2]:
                self.stop_motor2()
                self.rotate_reverse()
                self.state = 'PLACEMENT_DONE'

    
    
    def place_in_shelf(self):

        return

    def place_in_mobile_robot(self):
        self.get_logger().info('🚚 Starting package placement into mobile robot...')

        try:
            floor_index = self.floor_status.index(False)
        except ValueError:
            self.get_logger().warn('❌ All 3 floors are full! Cannot place package.')
            return

        self.get_logger().info(f'📦 Placing package in floor {floor_index + 1}')

        # Step 1: Move to the correct floor by counting IR detections
        self.ir_detection_count = 0
        self.move_motor1_forward()
        self.state = 'MOTOR1_FORWARD'
        time.sleep(0.2)  # initial debounce

        while self.ir_detection_count < (floor_index + 1):
            if self.state == 'MOTOR1_FORWARD' and self.ir_detected:
                self.ir_detection_count += 1
                self.get_logger().info(f'🔢 IR detected {self.ir_detection_count} time(s)...')
                while self.ir_detected:
                    time.sleep(0.05)  # wait until IR goes LOW
            time.sleep(0.05)

        self.stop_motor1()
        self.get_logger().info(f'✅ Reached floor {floor_index + 1}, starting placement...')

        # Step 2: Lower the package
        self.move_motor2_forward()
        self.state = 'MOTOR2_FORWARD'
        time.sleep(1.0)  # optional: replace with limit switch later
        self.stop_motor2()

        # Step 3: Drop the package
        self.move_motor3_reverse()
        self.state = 'MOTOR3_REVERSE'

        if self.state == 'MOTOR3_REVERSE':
            if self.switch_states[7] and self.switch_states[8]:
                self.stop_motor3()
                self.move_motor2_reverse()
                self.state = 'MOTOR2_REVERSE'

        if self.state == 'MOTOR2_REVERSE':
            if self.switch_states[1] and self.switch_states[2]:
                self.stop_motor2()
                self.floor_status[floor_index] = True
                self.state = 'PLACEMENT_DONE'
                self.get_logger().info(f'✅ Package placed in floor {floor_index + 1}. Updated floor status: {self.floor_status}')

            


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()        

if __name__ == '__main__':
    main()


