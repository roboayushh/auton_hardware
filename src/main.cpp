#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Encoder.h>
#include <time.h> // Required for struct timespec

// micro-ROS headers
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>

// --- PIN DEFINITIONS ---
#define I2C_SDA 8
#define I2C_SCL 9

// BTS7960 Driver 1 (Left Side) 
#define L_PWM_F 1 
#define L_PWM_R 2 

// BTS7960 Driver 2 (Right Side) 
#define R_PWM_F 12 
#define R_PWM_R 13 

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
const float WHEEL_RADIUS = 0.033; 
const float WHEEL_BASE = 0.20;    
const int TICKS_PER_REV = 233;
const float METERS_PER_TICK = (2.0 * PI * WHEEL_RADIUS) / TICKS_PER_REV;

// --- ROBOT STATE VARIABLES ---
float target_v = 0.0; 
float target_w = 0.0; 

float odom_x = 0.0;
float odom_y = 0.0;
float odom_theta = 0.0;

long prev_ticks_l = 0;
long prev_ticks_r = 0;
unsigned long prev_time = 0;

// --- TIME SYNC VARIABLES ---
unsigned long long time_offset = 0; // Holds the offset between ESP32 millis() and ROS 2 epoch

// --- WATCHDOG TIMER ---
unsigned long last_cmd_time = 0;
const unsigned long CMD_TIMEOUT = 500; 

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
// UTILITY & TIME SYNC FUNCTIONS
// ==========================================

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ while(1){ delay(100); } } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Block until a valid time sync is established with the ROS 2 agent
void syncTime() {
  unsigned long long ros_time_ms = 0;
  
  // Keep trying to sync until the agent provides a valid timestamp (> 0)
  while (ros_time_ms == 0) {
    if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
      rmw_uros_sync_session(100);
      ros_time_ms = rmw_uros_epoch_millis();
    }
    if (ros_time_ms == 0) {
      delay(100); // Back off to avoid spamming the network
    }
  }
  
  // Calculate the offset between the ESP32's hardware timer and the ROS 2 network time
  unsigned long now = millis();
  time_offset = ros_time_ms - now;
}

// Get the synchronized time based on the established offset
struct timespec getTime() {
  struct timespec tp = {0};
  // Current ROS time is the ESP32's running time plus the offset
  unsigned long long now = millis() + time_offset;
  tp.tv_sec = now / 1000;
  tp.tv_nsec = (now % 1000) * 1000000;
  return tp;
}

// Function to map an open-loop output to the motor pins
void set_motor_speed(int pwm_pin_f, int pwm_pin_r, float target_speed, const char* side) {
  float pwm_estimate = target_speed * 500.0; 
  
  int pwm_val = (int)fabs(pwm_estimate);
  if (pwm_val > 255) pwm_val = 255;
  
  if (pwm_estimate > 0.1) {
    analogWrite(pwm_pin_f, pwm_val);
    analogWrite(pwm_pin_r, 0);
  } else if (pwm_estimate < -0.1) {
    analogWrite(pwm_pin_f, 0);
    analogWrite(pwm_pin_r, pwm_val);
  } else {
    analogWrite(pwm_pin_f, 0);
    analogWrite(pwm_pin_r, 0);
  }
}

// ==========================================
// ROS 2 CALLBACKS
// ==========================================

void cmd_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  target_v = msg->linear.x;
  target_w = msg->angular.z;
  last_cmd_time = millis(); 
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  if (timer == NULL) return;

  unsigned long now = millis();
  float dt = (now - prev_time) / 1000.0;
  if (dt <= 0.0f) return;
  prev_time = now;

  // --- WATCHDOG ---
  if ((now - last_cmd_time) > CMD_TIMEOUT) {
    target_v = 0.0;
    target_w = 0.0;
  }

  // --- KINEMATICS ---
  long curr_l = (enc_fl.getCount() + enc_rl.getCount()) / 2;
  long curr_r = (enc_fr.getCount() + enc_rr.getCount()) / 2;
  
  float vel_l = (curr_l - prev_ticks_l) * METERS_PER_TICK / dt;
  float vel_r = (curr_r - prev_ticks_r) * METERS_PER_TICK / dt;

  prev_ticks_l = curr_l; 
  prev_ticks_r = curr_r;

  float target_l = target_v - (target_w * WHEEL_BASE / 2.0);
  float target_r = target_v + (target_w * WHEEL_BASE / 2.0);

  // --- OPEN-LOOP MOTOR CONTROL ---
  if (fabs(target_v) < 0.001 && fabs(target_w) < 0.001) {
      set_motor_speed(L_PWM_F, L_PWM_R, 0.0, "Left");
      set_motor_speed(R_PWM_F, R_PWM_R, 0.0, "Right");
  } else {
      set_motor_speed(L_PWM_F, L_PWM_R, target_l, "Left");
      set_motor_speed(R_PWM_F, R_PWM_R, target_r, "Right");
  }

  // --- ODOMETRY CALCULATIONS ---
  float d_dist = (vel_r + vel_l) / 2.0 * dt;
  float d_theta = (vel_r - vel_l) / WHEEL_BASE * dt;
  
  odom_x += d_dist * cos(odom_theta);
  odom_y += d_dist * sin(odom_theta);
  odom_theta += d_theta;

  msg_odom.pose.pose.position.x = odom_x;
  msg_odom.pose.pose.position.y = odom_y;
  msg_odom.pose.pose.orientation.z = sin(odom_theta / 2.0);
  msg_odom.pose.pose.orientation.w = cos(odom_theta / 2.0);

  // --- GET SYNCHRONIZED TIME ---
  struct timespec time_stamp = getTime();

  // Apply synchronized time to Odometry
  msg_odom.header.stamp.sec = time_stamp.tv_sec;
  msg_odom.header.stamp.nanosec = time_stamp.tv_nsec;
  
  RCSOFTCHECK(rcl_publish(&pub_odom, &msg_odom, NULL));

  // --- IMU PUBLISH ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  msg_imu.linear_acceleration.x = a.acceleration.x;
  msg_imu.linear_acceleration.y = a.acceleration.y;
  msg_imu.linear_acceleration.z = a.acceleration.z;
  
  msg_imu.angular_velocity.x = g.gyro.x;
  msg_imu.angular_velocity.y = g.gyro.y;
  msg_imu.angular_velocity.z = g.gyro.z;

  // Apply EXACT same synchronized time to IMU for flawless sensor fusion (EKF)
  msg_imu.header.stamp.sec = time_stamp.tv_sec;
  msg_imu.header.stamp.nanosec = time_stamp.tv_nsec;

  RCSOFTCHECK(rcl_publish(&pub_imu, &msg_imu, NULL));
}

// ==========================================
// SETUP & MAIN LOOP
// ==========================================

void setup() {
  Serial.begin(115200);
  
  pinMode(L_PWM_F, OUTPUT);
  pinMode(L_PWM_R, OUTPUT);
  pinMode(R_PWM_F, OUTPUT);
  pinMode(R_PWM_R, OUTPUT);

  analogWrite(L_PWM_F, 0); analogWrite(L_PWM_R, 0);
  analogWrite(R_PWM_F, 0); analogWrite(R_PWM_R, 0);

  set_microros_serial_transports(Serial);
  
  Wire.begin(I2C_SDA, I2C_SCL);
  mpu.begin(0x68, &Wire);
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  enc_fl.attachHalfQuad(ENC_FL_A, ENC_FL_B);
  enc_fr.attachHalfQuad(ENC_FR_A, ENC_FR_B);
  enc_rl.attachHalfQuad(ENC_RL_A, ENC_RL_B);
  enc_rr.attachHalfQuad(ENC_RR_A, ENC_RR_B);
  
  // 1. Wait for agent connection
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) { delay(100); }
  
  // 2. Initialize the micro-ROS support context FIRST
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "base_hardware_node", "", &support));

  // 3. NOW synchronize time, because the context exists!
  syncTime();

  // 4. Initialize Publishers and Subscribers
  RCCHECK(rclc_publisher_init_default(&pub_odom, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom/unfiltered"));
  RCCHECK(rclc_publisher_init_default(&pub_imu, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw"));
  RCCHECK(rclc_subscription_init_default(&sub_cmd, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  
  // Odometry frame IDs
  msg_odom.header.frame_id.data = (char*)"odom";
  msg_odom.header.frame_id.size = strlen(msg_odom.header.frame_id.data);
  msg_odom.header.frame_id.capacity = msg_odom.header.frame_id.size + 1;

  msg_odom.child_frame_id.data = (char*)"base_footprint";
  msg_odom.child_frame_id.size = strlen(msg_odom.child_frame_id.data);
  msg_odom.child_frame_id.capacity = msg_odom.child_frame_id.size + 1;

  // IMU frame IDs
  msg_imu.header.frame_id.data = (char*)"imu_link";
  msg_imu.header.frame_id.size = strlen(msg_imu.header.frame_id.data);
  msg_imu.header.frame_id.capacity = msg_imu.header.frame_id.size + 1;
  msg_imu.orientation_covariance[0] = -1.0; 

  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(20), timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd, &msg_cmd, &cmd_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}