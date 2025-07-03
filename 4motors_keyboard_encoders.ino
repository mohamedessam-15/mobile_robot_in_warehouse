#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32_multi_array.h>

// ==== WiFi Setup ====
char ssid[] = "WE_64EA98";
char password[] = "DRpharma3ESSAM";
char agent_ip[] = "192.168.100.3";
const int agent_port = 8888;

// ==== Motor Pins (PWM capable on ESP32) ====
#define ENA1 21
#define IN1_1 19
#define IN1_2 18

#define ENA2 5
#define IN2_1 16
#define IN2_2 17

#define ENA3 4
#define IN3_1 22  // UPDATED FROM 2
#define IN3_2 23  // UPDATED FROM 15

#define ENA4 27
#define IN4_1 14
#define IN4_2 12

#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

// PWM Channels
#define CH1 0
#define CH2 1
#define CH3 2
#define CH4 3

// ==== Encoder Pins ====
#define ENCODER_COUNT 4
int encoderA_pins[ENCODER_COUNT] = {34, 26, 15, 32};
int encoderB_pins[ENCODER_COUNT] = {35, 25, 13, 33};
volatile long encoderCounts[ENCODER_COUNT] = {0};

// ==== micro-ROS Entities ====
rcl_node_t node;
rcl_subscription_t cmd_vel_sub;
rcl_publisher_t encoder_pub;
rcl_timer_t encoder_timer;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
geometry_msgs__msg__Twist cmd_msg;
std_msgs__msg__Int32MultiArray encoder_msg;

// Motor timeout
unsigned long last_cmd_time = 0;
const unsigned long timeout_ms = 500;

// ==== Motor Control ====
void setMotor(int in1, int in2, int pwm_channel, float speed) {
  int pwm = (int)(constrain(fabs(speed), 0.0, 1.0) * 255);
  if (speed > 0.01) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < -0.01) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    pwm = 0;
  }
  ledcWrite(pwm_channel, pwm);
}

void stopAllMotors() {
  setMotor(IN1_1, IN1_2, CH1, 0);
  setMotor(IN2_1, IN2_2, CH2, 0);
  setMotor(IN3_1, IN3_2, CH3, 0);
  setMotor(IN4_1, IN4_2, CH4, 0);
}

void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  float linear = msg->linear.x;
  float angular = msg->angular.z;
  last_cmd_time = millis();

  // Tank-style calculation
  float left_speed = constrain(linear - angular, -1.0, 1.0);
  float right_speed = constrain(linear + angular, -1.0, 1.0);

  // Left motors: 1, 3 | Right motors: 2, 4
  setMotor(IN1_1, IN1_2, CH1, left_speed);
  setMotor(IN3_1, IN3_2, CH3, left_speed);
  setMotor(IN2_1, IN2_2, CH2, right_speed);
  setMotor(IN4_1, IN4_2, CH4, right_speed);
}

// ==== Encoder Interrupt Handlers ====
void IRAM_ATTR handleEncoder0() { encoderCounts[0] += (digitalRead(encoderB_pins[0]) == LOW) ? 1 : -1; }
void IRAM_ATTR handleEncoder1() { encoderCounts[1] += (digitalRead(encoderB_pins[1]) == LOW) ? 1 : -1; }
void IRAM_ATTR handleEncoder2() { encoderCounts[2] += (digitalRead(encoderB_pins[2]) == LOW) ? 1 : -1; }
void IRAM_ATTR handleEncoder3() { encoderCounts[3] += (digitalRead(encoderB_pins[3]) == LOW) ? 1 : -1; }

void encoder_timer_callback(rcl_timer_t *, int64_t) {
  for (int i = 0; i < ENCODER_COUNT; i++) {
    encoder_msg.data.data[i] = encoderCounts[i];
  }
  rcl_publish(&encoder_pub, &encoder_msg, NULL);

  // Optional: debug print
  Serial.printf("Encoders: %ld %ld %ld %ld\n",
                encoderCounts[0], encoderCounts[1],
                encoderCounts[2], encoderCounts[3]);
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Setup motor pins and PWM
  pinMode(IN1_1, OUTPUT); pinMode(IN1_2, OUTPUT);
  pinMode(IN2_1, OUTPUT); pinMode(IN2_2, OUTPUT);
  pinMode(IN3_1, OUTPUT); pinMode(IN3_2, OUTPUT);
  pinMode(IN4_1, OUTPUT); pinMode(IN4_2, OUTPUT);

  ledcSetup(CH1, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(ENA1, CH1);
  ledcSetup(CH2, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(ENA2, CH2);
  ledcSetup(CH3, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(ENA3, CH3);
  ledcSetup(CH4, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(ENA4, CH4);

  // Encoder setup with pull-ups
  for (int i = 0; i < ENCODER_COUNT; i++) {
    pinMode(encoderA_pins[i], INPUT);
    pinMode(encoderB_pins[i], INPUT);
  }

  attachInterrupt(digitalPinToInterrupt(encoderA_pins[0]), handleEncoder0, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderA_pins[1]), handleEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderA_pins[2]), handleEncoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderA_pins[3]), handleEncoder3, RISING);

  // Connect to micro-ROS agent
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
  delay(2000);

  // micro-ROS init
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_motor_node", "", &support);

  rclc_subscription_init_default(
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"
  );

  rclc_publisher_init_default(
    &encoder_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "encoder_counts"
  );

  // Encoder message memory setup
  encoder_msg.layout.dim.capacity = 1;
  encoder_msg.layout.dim.size = 1;
  encoder_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension *)malloc(sizeof(std_msgs__msg__MultiArrayDimension));
  encoder_msg.layout.dim.data[0].size = ENCODER_COUNT;
  encoder_msg.layout.dim.data[0].stride = ENCODER_COUNT;

  encoder_msg.data.capacity = ENCODER_COUNT;
  encoder_msg.data.size = ENCODER_COUNT;
  encoder_msg.data.data = (int32_t *)malloc(sizeof(int32_t) * ENCODER_COUNT);

  // Executor + timer
  rclc_timer_init_default(&encoder_timer, &support, RCL_MS_TO_NS(100), encoder_timer_callback);
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_msg, &cmd_vel_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &encoder_timer);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  if (millis() - last_cmd_time > timeout_ms) stopAllMotors();
  delay(10);
}

