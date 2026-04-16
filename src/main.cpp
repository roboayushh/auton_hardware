#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Encoder.h>

// micro-ROS headers
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>

// --- PIN DEFINITIONS ---
// I2C for MPU6050
#define I2C_SDA 8
#define I2C_SCL 9

// BTS7960 Driver 1 (Left Side Motors)
#define L_PWM_F 1 // RPWM
#define L_PWM_R 2 // LPWM

// BTS7960 Driver 2 (Right Side Motors)
#define R_PWM_F 41 // RPWM
#define R_PWM_R 42 // LPWM

// Encoders (4 wheels)
#define ENC_FL_A 4   
#define ENC_FL_B 5
#define ENC_FR_A 6   
#define ENC_FR_B 7
#define ENC_RL_A 15  
#define ENC_RL_B 16
#define ENC_RR_A 17  
#define ENC_RR_B 18

// --- PHYSICAL CONSTANTS ---
const float WHEEL_RADIUS = 0.033; // 3.3cm
const float WHEEL_BASE = 0.20;    // 20cm track width
const int TICKS_PER_REV = 233;
const float METERS_PER_TICK = (2.0 * PI * WHEEL_RADIUS) / TICKS_PER_REV;

// --- PID GAINS ---
// Start low. If the robot is weak, increase Kp.
float Kp = 1.0, Ki = 0.1, Kd = 0.05;

// --- ROBOT STATE VARIABLES ---
float target_v = 0.0; // Target linear velocity (m/s)
float target_w = 0.0; // Target angular velocity (rad/s)

float odom_x = 0.0;
float odom_y = 0.0;
float odom_theta = 0.0;

long prev_ticks_l = 0;
long prev_ticks_r = 0;
unsigned long prev_time = 0;

float last_error_l = 0, last_error_r = 0;
float iterm_l = 0, iterm_r = 0;

// --- WATCHDOG TIMER ---
unsigned long last_cmd_time = 0;
const unsigned long CMD_TIMEOUT = 500; // Stop motors if no command received for 500ms

// --- ROS OBJECTS ---
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_publisher_t pub_odom, pub_imu;
rcl_subscription_t sub_cmd;

nav_msgs__msg__Odometry msg_odom;
sensor_msgs__msg__Imu msg_imu;
geometry_msgs__msg__Twist msg_cmd;

// --- HARDWARE OBJECTS ---
Adafruit_MPU6050 mpu;
ESP32Encoder enc_fl, enc_fr, enc_rl, enc_rr;

// ==========================================
// UTILITY FUNCTIONS
// ==========================================

// Error loop to catch micro-ROS initialization failures
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ while(1){ delay(100); } } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


// Function to map PID output to the BTS7960 PWM pins
void set_motor_speed(int pwm_pin_f, int pwm_pin_r, float output) {
  // Use fabs for floats to be safe, then cast to int
  int pwm_val = (int)fabs(output);
  if (pwm_val > 255) pwm_val = 255;
  
  // Use a tiny deadband (0.1) to prevent micro-stuttering when stopped
  if (output > 0.1) {
    // Forward
    analogWrite(pwm_pin_f, pwm_val);
    analogWrite(pwm_pin_r, 0);
  } else if (output < -0.1) {
    // Reverse 
    analogWrite(pwm_pin_f, 0);
    analogWrite(pwm_pin_r, pwm_val);
  } else {
    // Stop
    analogWrite(pwm_pin_f, 0);
    analogWrite(pwm_pin_r, 0);
  }
}

// Computes the required PWM adjustment to hit the target velocity
float compute_pid(float target, float current, float &iterm, float &last_error, float dt) {
  float error = target - current;
  iterm += error * dt;
  
  // Anti-windup constraint for the integral term
  iterm = constrain(iterm, -255.0, 255.0); 

  float d_error = (error - last_error) / dt;
  last_error = error;
  
  // Return the PID sum
  return (Kp * error) + (Ki * iterm) + (Kd * d_error);
}FFFFFFFFFFFFFFFFFFFFFFF
// Triggers whenever Nav2 or Teleop publishes to /cmd_vel
void cmd_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  target_v = msg->linear.x;
  target_w = msg->angular.z;
  
  // Reset the watchdog timer because we just received a fresh command
  last_cmd_time = millis(); 
}

// Triggers at 50Hz (every 20ms) to update odometry and run the PID loop
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  if (timer == NULL) return;

  unsigned long now = millis();
  float dt = (now - prev_time) / 1000.0;

  if (dt <= 0.0f) return;
  
  prev_time = now;

  // --- 1. WATCHDOG SAFETY CHECK ---
  // If half a second has passed without a command, the Pi might have crashed. Stop the robot.
  if ((now - last_cmd_time) > CMD_TIMEOUT) {
    target_v = 0.0;
    target_w = 0.0;
  }

  // --- 2. READ ENCODERS & CALCULATE CURRENT VELOCITY ---
  // Average the front and rear wheels on each side for differential drive kinematics
  long curr_l = (enc_fl.getCount() + enc_rl.getCount()) / 2;
  long curr_r = (enc_fr.getCount() + enc_rr.getCount()) / 2;
  
  float vel_l = (curr_l - prev_ticks_l) * METERS_PER_TICK / dt;
  float vel_r = (curr_r - prev_ticks_r) * METERS_PER_TICK / dt;
  
  prev_ticks_l = curr_l; 
  prev_ticks_r = curr_r;

  // --- 3. INVERSE KINEMATICS ---
  // Convert the core target velocity (m/s) into target speeds for the left and right sides
  float target_l = target_v - (target_w * WHEEL_BASE / 2.0);
  float target_r = target_v + (target_w * WHEEL_BASE / 2.0);

  // --- 4. PID CONTROL ---
  // If target is 0, explicitly stop motors. Otherwise, calculate PID.
  if (target_v == 0 && target_w == 0) {
      set_motor_speed(L_PWM_F, L_PWM_R, 0);
      set_motor_speed(R_PWM_F, R_PWM_R, 0);
      iterm_l = 0; iterm_r = 0; // Reset integral windup when stopped
  } else {
      // Multiply PID output by a scaling factor to get it into the 0-255 PWM range
      // A Kp of 1.0 with an error of 0.5m/s gives 0.5. Multiplied by 200 = 100 PWM.
      float out_l = compute_pid(target_l, vel_l, iterm_l, last_error_l, dt) * 200.0; 
      float out_r = compute_pid(target_r, vel_r, iterm_r, last_error_r, dt) * 200.0;

      set_motor_speed(L_PWM_F, L_PWM_R, out_l);
      set_motor_speed(R_PWM_F, R_PWM_R, out_r);
  }

  // --- 5. ODOMETRY INTEGRATION ---
  float d_dist = (vel_r + vel_l) / 2.0 * dt;
  float d_theta = (vel_r - vel_l) / WHEEL_BASE * dt;
  
  odom_x += d_dist * cos(odom_theta);
  odom_y += d_dist * sin(odom_theta);
  odom_theta += d_theta;

  // --- 6. POPULATE & PUBLISH ODOMETRY MESSAGE ---
  msg_odom.pose.pose.position.x = odom_x;
  msg_odom.pose.pose.position.y = odom_y;
  
  // Convert Euler angle to Quaternion for ROS 2
  msg_odom.pose.pose.orientation.z = sin(odom_theta / 2.0);
  msg_odom.pose.pose.orientation.w = cos(odom_theta / 2.0);

  // Time Sync formatting
  msg_odom.header.stamp.nanosec = rmw_uros_epoch_nanos();
  
  RCSOFTCHECK(rcl_publish(&pub_odom, &msg_odom, NULL));
}

// ==========================================
// SETUP & MAIN LOOP
// ==========================================

void setup() {
  Serial.begin(115200);
  // Set motor pins as outputs
  pinMode(L_PWM_F, OUTPUT);
  pinMode(L_PWM_R, OUTPUT);
  pinMode(R_PWM_F, OUTPUT);
  pinMode(R_PWM_R, OUTPUT);
  set_microros_serial_transports(Serial);
  
  // --- HARDWARE INIT ---
  Wire.begin(I2C_SDA, I2C_SCL);
  mpu.begin(0x68, &Wire);

  // Configure internal pullups for the encoders
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  enc_fl.attachHalfQuad(ENC_FL_A, ENC_FL_B);
  enc_fr.attachHalfQuad(ENC_FR_A, ENC_FR_B);
  enc_rl.attachHalfQuad(ENC_RL_A, ENC_RL_B);
  enc_rr.attachHalfQuad(ENC_RR_A, ENC_RR_B);
  
  // --- micro-ROS AGENT CONNECTION ---
  // Halt here until the Raspberry Pi micro-ROS agent is reachable
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
      delay(100);
  }
  
  // Synchronize time with the Raspberry Pi to ensure TF transforms align perfectly
  rmw_uros_sync_session(100);

  // --- micro-ROS ENTITY INIT ---
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "base_hardware_node", "", &support));

  RCCHECK(rclc_publisher_init_default(&pub_odom, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom/unfiltered"));
  RCCHECK(rclc_subscription_init_default(&sub_cmd, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  
  // --- ASSIGNING FRAME IDs ---
  // In C, ROS 2 Strings must be allocated correctly with size and capacity.
  msg_odom.header.frame_id.data = (char*)"odom";
  msg_odom.header.frame_id.size = strlen(msg_odom.header.frame_id.data);
  msg_odom.header.frame_id.capacity = msg_odom.header.frame_id.size + 1;

  msg_odom.child_frame_id.data = (char*)"base_link";
  msg_odom.child_frame_id.size = strlen(msg_odom.child_frame_id.data);
  msg_odom.child_frame_id.capacity = msg_odom.child_frame_id.size + 1;

  // --- EXECUTOR & TIMER INIT ---
  // 20ms timeout = 50Hz control loop frequency
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(20), timer_callback));

  // The executor needs 2 handles: One for the Timer, One for the cmd_vel Subscriber
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd, &msg_cmd, &cmd_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  // Spin the executor to handle incoming messages and timer callbacks
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}