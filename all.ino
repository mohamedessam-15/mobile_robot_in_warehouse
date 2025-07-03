#include <micro_ros_arduino.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>

// ==== Pins ====
#define SERVO_PIN 18

// Motor 1
#define IN1 5
#define IN2 19
#define ENA 21

// Motor 2
#define IN3 25
#define IN4 26
#define ENA2 27

// Motor 3
#define IN5 12
#define IN6 14
#define ENA3 13

// Sensors and LED
#define IR_SENSOR_PIN 22
#define SECOND_IR_SENSOR_PIN 34
#define LED_PIN 32

// Limit Switches
#define LIMIT_SWITCH_PIN_1 23
#define LIMIT_SWITCH_PIN_2 33
#define LIMIT_SWITCH_PIN_3 15
#define LIMIT_SWITCH_PIN_4 4
#define LIMIT_SWITCH_PIN_5 16
#define LIMIT_SWITCH_PIN_6 17
#define LIMIT_SWITCH_PIN_7 2
#define LIMIT_SWITCH_PIN_8 35
#define LIMIT_SWITCH_PIN_9 39

// ==== WiFi Info ====
char ssid[] = "WE_64EA98";
char password[] = "DRpharma3ESSAM";
char agent_ip[] = "192.168.100.3";
const int agent_port = 8888;

// ==== ROS Infrastructure ====
rclc_executor_t executor;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;

// ==== Servo ====
Servo myServo;
rcl_subscription_t servo_sub;
std_msgs__msg__Int32 servo_msg;
void servo_callback(const void *msgin) {
  int angle = ((std_msgs__msg__Int32 *)msgin)->data;
  angle = constrain(angle, 0, 180);
  myServo.write(angle);
}

// ==== Motor Commands ====
rcl_subscription_t motor1_sub, motor2_sub, motor3_sub;
std_msgs__msg__Int32 motor1_msg, motor2_msg, motor3_msg;

void motor1_callback(const void *msgin) {
  int cmd = ((std_msgs__msg__Int32 *)msgin)->data;
  digitalWrite(IN1, cmd > 0);
  digitalWrite(IN2, cmd < 0);
}

void motor2_callback(const void *msgin) {
  int cmd = ((std_msgs__msg__Int32 *)msgin)->data;
  digitalWrite(IN3, cmd > 0);
  digitalWrite(IN4, cmd < 0);
}

void motor3_callback(const void *msgin) {
  int cmd = ((std_msgs__msg__Int32 *)msgin)->data;
  digitalWrite(IN5, cmd > 0);
  digitalWrite(IN6, cmd < 0);
}

// ==== LED Control ====
rcl_subscription_t led_sub;
std_msgs__msg__Bool led_msg;
void led_callback(const void *msgin) {
  digitalWrite(LED_PIN, ((std_msgs__msg__Bool *)msgin)->data ? HIGH : LOW);
}

// ==== IR Sensors ====
rcl_publisher_t ir_pub;
std_msgs__msg__Bool ir_msg;

rcl_publisher_t ir_analog_pub;
std_msgs__msg__Int32 ir_analog_msg;

// ==== Limit Switches ====
rcl_publisher_t limit_switch_pubs[9];
std_msgs__msg__Bool limit_msgs[9];
const int limit_pins[9] = {
  LIMIT_SWITCH_PIN_1, LIMIT_SWITCH_PIN_2, LIMIT_SWITCH_PIN_3,
  LIMIT_SWITCH_PIN_4, LIMIT_SWITCH_PIN_5, LIMIT_SWITCH_PIN_6,
  LIMIT_SWITCH_PIN_7, LIMIT_SWITCH_PIN_8, LIMIT_SWITCH_PIN_9
};

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);

  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN);
  myServo.write(0);

  // Motors
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT); analogWrite(ENA, 255);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENA2, OUTPUT); analogWrite(ENA2, 255);
  pinMode(IN5, OUTPUT); pinMode(IN6, OUTPUT); pinMode(ENA3, OUTPUT); analogWrite(ENA3, 255);

  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(SECOND_IR_SENSOR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  for (int i = 0; i < 9; i++) pinMode(limit_pins[i], INPUT_PULLUP);

  delay(2000);
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_combined_node", "", &support);

  // Subscribers
  rclc_subscription_init_default(&servo_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "servo_angle");
  rclc_subscription_init_default(&motor1_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "motor1_command");
  rclc_subscription_init_default(&motor2_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "motor2_command");
  rclc_subscription_init_default(&motor3_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "motor3_command");
  rclc_subscription_init_default(&led_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "led_control");

  // Publishers
  rclc_publisher_init_default(&ir_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "ir_detected");
  rclc_publisher_init_default(&ir_analog_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "ir_analog");
  for (int i = 0; i < 9; i++) {
    char topic[20];
    sprintf(topic, "limit_switch%d", i + 1);
    rclc_publisher_init_default(&limit_switch_pubs[i], &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), topic);
  }

  // Executor
  rclc_executor_init(&executor, &support.context, 13, &allocator);
  rclc_executor_add_subscription(&executor, &servo_sub, &servo_msg, &servo_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &motor1_sub, &motor1_msg, &motor1_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &motor2_sub, &motor2_msg, &motor2_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &motor3_sub, &motor3_msg, &motor3_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &led_sub, &led_msg, &led_callback, ON_NEW_DATA);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // Publish digital IR
  ir_msg.data = digitalRead(IR_SENSOR_PIN);
  rcl_publish(&ir_pub, &ir_msg, NULL);

  // Publish analog IR
  ir_analog_msg.data = analogRead(SECOND_IR_SENSOR_PIN);
  rcl_publish(&ir_analog_pub, &ir_analog_msg, NULL);

  // Publish limit switches
  for (int i = 0; i < 9; i++) {
    limit_msgs[i].data = !digitalRead(limit_pins[i]);  // Active LOW
    rcl_publish(&limit_switch_pubs[i], &limit_msgs[i], NULL);
  }

  delay(50);
}


