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
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int32.h>
#include <nav_msgs/msg/odometry.h>


// --- PIN DEFINITIONS ---
#define I2C_SDA 8
#define I2C_SCL 9

#define ENC_FL_A 4
#define ENC_FL_B 5

#define ENC_FR_A 6
#define ENC_FR_B 7

#define ENC_RL_A 15
#define ENC_RL_B 16

#define ENC_RR_A 17
#define ENC_RR_B 18

const float METERS_PER_TICK = 0.0008899; 
const float WHEEL_BASE_WIDTH = 0.20;

// Robot State
float odom_x = 0.0;
float odom_y = 0.0;
float odom_theta = 0.0;
long prev_fl = 0, prev_fr = 0, prev_rl = 0, prev_rr = 0;

// ROS Objects
rcl_publisher_t pub_odom;
nav_msgs__msg__Odometry msg_odom;

// --- HARDWARE OBJECTS ---
Adafruit_MPU6050 mpu;
ESP32Encoder enc_fl;
ESP32Encoder enc_fr;
ESP32Encoder enc_rl;
ESP32Encoder enc_rr;

// --- MICRO-ROS OBJECTS ---
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;

// Publishers
rcl_publisher_t pub_imu;
rcl_publisher_t pub_enc_fl;
rcl_publisher_t pub_enc_fr;
rcl_publisher_t pub_enc_rl;
rcl_publisher_t pub_enc_rr;

// Messages
sensor_msgs__msg__Imu msg_imu;
std_msgs__msg__Int32 msg_enc_fl;
std_msgs__msg__Int32 msg_enc_fr;
std_msgs__msg__Int32 msg_enc_rl;
std_msgs__msg__Int32 msg_enc_rr;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while(1) {
    delay(100); // Silent error loop
  }
}

// --- MAIN LOOP CALLBACK ---
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  if (timer == NULL) return;

  // 1. Get current counts
  long curr_fl = enc_fl.getCount();
  long curr_fr = enc_fr.getCount();
  long curr_rl = enc_rl.getCount();
  long curr_rr = enc_rr.getCount();

  // 2. Calculate distance traveled by each side (averaging front/rear)
  float d_left = ((curr_fl - prev_fl) + (curr_rl - prev_rl)) / 2.0 * METERS_PER_TICK;
  float d_right = ((curr_fr - prev_fr) + (curr_rr - prev_rr)) / 2.0 * METERS_PER_TICK;

  // 3. Update previous counts for next loop
  prev_fl = curr_fl; prev_fr = curr_fr; prev_rl = curr_rl; prev_rr = curr_rr;

  // 4. Differential Kinematics
  float d_distance = (d_right + d_left) / 2.0;
  float d_theta = (d_right - d_left) / WHEEL_BASE_WIDTH;

  // 5. Update Pose
  odom_x += d_distance * cos(odom_theta);
  odom_y += d_distance * sin(odom_theta);
  odom_theta += d_theta;

  // 6. Fill Odometry Message
  msg_odom.header.stamp.nanosec = rmw_uros_epoch_nanos(); // If using sync'd clock
  msg_odom.pose.pose.position.x = odom_x;
  msg_odom.pose.pose.position.y = odom_y;
  
  // Convert Euler theta to Quaternion (standard for ROS2)
  msg_odom.pose.pose.orientation.z = sin(odom_theta / 2.0);
  msg_odom.pose.pose.orientation.w = cos(odom_theta / 2.0);

  RCSOFTCHECK(rcl_publish(&pub_odom, &msg_odom, NULL));
}

void setup() {
  // 1. Setup Serial
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  // 2. Setup IMU
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!mpu.begin(0x68, &Wire)) {
    error_loop();
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // 3. Setup Encoders
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  
  enc_fl.attachHalfQuad(ENC_FL_A, ENC_FL_B);
  enc_fr.attachHalfQuad(ENC_FR_A, ENC_FR_B);
  enc_rl.attachHalfQuad(ENC_RL_A, ENC_RL_B);
  enc_rr.attachHalfQuad(ENC_RR_A, ENC_RR_B);
  
  enc_fl.clearCount();
  enc_fr.clearCount();
  enc_rl.clearCount();
  enc_rr.clearCount();

  // 4. Wait for micro-ROS Agent
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
      delay(100);
  }

  // 5. Initialize micro-ROS Entities
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "base_hardware_node", "", &support));

  // Initialize all 5 publishers
// Initialize all 5 publishers (Middle argument MUST be 'msg')
  RCCHECK(rclc_publisher_init_default(&pub_imu, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw"));
  RCCHECK(rclc_publisher_init_default(&pub_enc_fl, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "encoder/front_left"));
  RCCHECK(rclc_publisher_init_default(&pub_enc_fr, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "encoder/front_right"));
  RCCHECK(rclc_publisher_init_default(&pub_enc_rl, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "encoder/rear_left"));
  RCCHECK(rclc_publisher_init_default(&pub_enc_rr, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "encoder/rear_right"));
  RCCHECK(rclc_publisher_init_default(&pub_odom, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"));
  msg_imu.orientation_covariance[0] = -1.0;

  // 6. Setup Timer & Executor (50Hz = 20ms timeout)
  const unsigned int timer_timeout = 20; 
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  // We only need 1 handle in the executor because we only have 1 timer triggering everything
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  delay(10);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}




//Encoder Only setup (test code, for single Encoder only)
// #include <Arduino.h>
// #include <ESP32Encoder.h>

// // micro-ROS headers
// #include <micro_ros_platformio.h>
// #include <rcl/rcl.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>
// #include <std_msgs/msg/int32.h>

// // ESP32-S3 Encoder Pins
// #define ENCODER_PIN_A 4
// #define ENCODER_PIN_B 5 // Leave disconnected if you only have a 1-pin sensor

// // Create the hardware encoder object
// ESP32Encoder encoder;

// // micro-ROS objects
// rcl_publisher_t publisher;
// std_msgs__msg__Int32 msg;
// rclc_executor_t executor;
// rclc_support_t support;
// rcl_allocator_t allocator;
// rcl_node_t node;
// rcl_timer_t timer;

// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// void error_loop() {
//   while(1) {
//     delay(100);
//   }
// }

// // Timer callback: Read the hardware counter and publish
// void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
//   RCLC_UNUSED(last_call_time);
//   if (timer != NULL) {
//     // Get the current tick count from the hardware
//     msg.data = (int32_t)encoder.getCount();
    
//     RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
//   }
// }

// void setup() {
//   Serial.begin(115200);
//   set_microros_serial_transports(Serial);
//   delay(2000);

//   // --- ENCODER INITIALIZATION ---
//   // Enable the ESP32's internal pull-up resistors for the encoder pins
//   ESP32Encoder::useInternalWeakPullResistors = puType::up;

//   // IF YOU HAVE A 2-PIN QUADRATURE ENCODER:
//   encoder.attachHalfQuad(ENCODER_PIN_A, ENCODER_PIN_B);

//   // IF YOU HAVE A 1-PIN OPTICAL SENSOR (e.g., LM393 D0 pin):
//   // Comment out the line above, and uncomment the line below:
//   // encoder.attachSingleEdge(ENCODER_PIN_A);
  
//   // Set the starting count to zero
//   encoder.clearCount();
//   // ------------------------------

//   allocator = rcl_get_default_allocator();

//   // Wait for the micro-ROS agent to be ready (The lifesaver loop!)
//   while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
//       delay(100);
//   }

//   RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
//   RCCHECK(rclc_node_init_default(&node, "wheel_encoder_node", "", &support));

//   // Create a publisher for a standard 32-bit integer
//   RCCHECK(rclc_publisher_init_default(
//     &publisher,
//     &node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
//     "encoder/ticks"));

//   // Publishing at 10Hz (100ms) - Fast enough for responsiveness, slow enough for easy terminal reading
//   const unsigned int timer_timeout = 100;
//   RCCHECK(rclc_timer_init_default(
//     &timer,
//     &support,
//     RCL_MS_TO_NS(timer_timeout),
//     timer_callback));

//   RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
//   RCCHECK(rclc_executor_add_timer(&executor, &timer));
// }

// void loop() {
//   delay(10);
//   RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
// }










//IMY Only setup (test code)
// #include <Arduino.h>
// #include <Wire.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>

// #include <micro_ros_platformio.h>
// #include <rcl/rcl.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>
// #include <sensor_msgs/msg/imu.h>

// #if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
// #error This example is only avaliable for Arduino framework with serial transport.
// #endif

// // ESP32-S3 I2C pins
// #define I2C_SDA 8
// #define I2C_SCL 9

// Adafruit_MPU6050 mpu;

// rcl_publisher_t publisher;
// sensor_msgs__msg__Imu msg;

// rclc_executor_t executor;
// rclc_support_t support;
// rcl_allocator_t allocator;
// rcl_node_t node;
// rcl_timer_t timer;

// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// // Error handle loop
// void error_loop() {
//   while(1) {
//     delay(100);
//   }
// }

// void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
//   RCLC_UNUSED(last_call_time);
//   if (timer != NULL) {
//     sensors_event_t a, g, temp;
//     mpu.getEvent(&a, &g, &temp);

//     msg.linear_acceleration.x = a.acceleration.x;
//     msg.linear_acceleration.y = a.acceleration.y;
//     msg.linear_acceleration.z = a.acceleration.z;

//     msg.angular_velocity.x = g.gyro.x;
//     msg.angular_velocity.y = g.gyro.y;
//     msg.angular_velocity.z = g.gyro.z;

//     RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
//   }
// }

// void setup() {
//   // Configure serial transport
//   Serial.begin(115200);
//   set_microros_serial_transports(Serial);
//   delay(2000);

//   // Initialize I2C and MPU6050
//   Wire.begin(I2C_SDA, I2C_SCL);
//   if (!mpu.begin(0x68, &Wire)) {
//     error_loop(); 
//   }
//   mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
//   mpu.setGyroRange(MPU6050_RANGE_500_DEG);
//   mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

//   allocator = rcl_get_default_allocator();

//   // create init_options
//   RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

//   // create node
//   RCCHECK(rclc_node_init_default(&node, "micro_ros_imu_node", "", &support));

//   // create publisher
//   RCCHECK(rclc_publisher_init_default(
//     &publisher,
//     &node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
//     "imu/data_raw"));

//   // Set up the IMU message static fields
//   msg.orientation_covariance[0] = -1.0;

//   // create timer (1000ms = 1Hz, easy for testing)
//   const unsigned int timer_timeout = 10;
//   RCCHECK(rclc_timer_init_default(
//     &timer,
//     &support,
//     RCL_MS_TO_NS(timer_timeout),
//     timer_callback));

//   // create executor
//   RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
//   RCCHECK(rclc_executor_add_timer(&executor, &timer));
// }

// void loop() {
//   delay(10);
//   RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
// }