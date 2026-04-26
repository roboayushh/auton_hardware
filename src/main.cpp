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
#define I2C_SDA 8
#define I2C_SCL 9
#define L_PWM_F 1 
#define L_PWM_R 2 
#define R_PWM_F 12 
#define R_PWM_R 13 

#define ENC_FL_A 4   
#define ENC_FL_B 5
#define ENC_FR_A 6   
#define ENC_FR_B 7
#define ENC_RL_A 15  
#define ENC_RL_B 16
#define ENC_RR_A 17  
#define ENC_RR_B 18

// --- PHYSICAL CONSTANTS ---
const float WHEEL_RADIUS = 0.0325; // FIXED: Changed to 0.0325 to perfectly match URDF
const float WHEEL_BASE = 0.20;    
const int TICKS_PER_REV = 233;
const float METERS_PER_TICK = (2.0 * PI * WHEEL_RADIUS) / TICKS_PER_REV;

// --- ROBOT STATE ---
float target_v = 0.0; 
float target_w = 0.0; 
float odom_x = 0.0, odom_y = 0.0, odom_theta = 0.0;
long prev_ticks_l = 0, prev_ticks_r = 0;
unsigned long prev_time = 0;
unsigned long long time_offset = 0;
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

Adafruit_MPU6050 mpu;
ESP32Encoder enc_fl, enc_fr, enc_rl, enc_rr;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ while(1){ delay(100); } } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void syncTime() {
    unsigned long long ros_time_ms = 0;
    while (ros_time_ms == 0) {
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
            rmw_uros_sync_session(100);
            ros_time_ms = rmw_uros_epoch_millis();
        }
        delay(100);
    }
    time_offset = ros_time_ms - millis();
}

struct timespec getTime() {
    struct timespec tp = {0};
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}

void set_motor_speed(int pwm_pin_f, int pwm_pin_r, float target_speed) {
    int pwm_val = min((int)fabs(target_speed * 500.0), 255);
    if (target_speed > 0.1) {
        analogWrite(pwm_pin_f, pwm_val); analogWrite(pwm_pin_r, 0);
    } else if (target_speed < -0.1) {
        analogWrite(pwm_pin_f, 0); analogWrite(pwm_pin_r, pwm_val);
    } else {
        analogWrite(pwm_pin_f, 0); analogWrite(pwm_pin_r, 0);
    }
}

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

    if ((now - last_cmd_time) > CMD_TIMEOUT) { target_v = 0.0; target_w = 0.0; }

    long curr_l = (enc_fl.getCount() + enc_rl.getCount()) / 2;
    long curr_r = (enc_fr.getCount() + enc_rr.getCount()) / 2;
    
    float vel_l = (curr_l - prev_ticks_l) * METERS_PER_TICK / dt;
    float vel_r = (curr_r - prev_ticks_r) * METERS_PER_TICK / dt;
    prev_ticks_l = curr_l; prev_ticks_r = curr_r;

    // --- MOTOR CONTROL ---
    float t_l = target_v - (target_w * WHEEL_BASE / 2.0);
    float t_r = target_v + (target_w * WHEEL_BASE / 2.0);
    set_motor_speed(L_PWM_F, L_PWM_R, t_l);
    set_motor_speed(R_PWM_F, R_PWM_R, t_r);

    // --- ODOMETRY KINEMATICS ---
    float linear_vel = (vel_r + vel_l) / 2.0;
    float angular_vel = (vel_r - vel_l) / WHEEL_BASE;
    float d_dist = linear_vel * dt;
    float d_theta = angular_vel * dt;
    
    odom_x += d_dist * cos(odom_theta);
    odom_y += d_dist * sin(odom_theta);
    odom_theta += d_theta;

    struct timespec ts = getTime();

    // Fill Pose
    msg_odom.header.stamp.sec = ts.tv_sec;
    msg_odom.header.stamp.nanosec = ts.tv_nsec;
    msg_odom.pose.pose.position.x = odom_x;
    msg_odom.pose.pose.position.y = odom_y;
    msg_odom.pose.pose.orientation.z = sin(odom_theta / 2.0);
    msg_odom.pose.pose.orientation.w = cos(odom_theta / 2.0);

    msg_odom.twist.twist.linear.x = linear_vel;
    msg_odom.twist.twist.angular.z = angular_vel;

    RCSOFTCHECK(rcl_publish(&pub_odom, &msg_odom, NULL));

    // --- IMU DATA ---
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    msg_imu.header.stamp.sec = ts.tv_sec;
    msg_imu.header.stamp.nanosec = ts.tv_nsec;
    msg_imu.linear_acceleration.x = a.acceleration.x;
    msg_imu.linear_acceleration.y = a.acceleration.y;
    msg_imu.linear_acceleration.z = a.acceleration.z;
    msg_imu.angular_velocity.z = g.gyro.z; 

    RCSOFTCHECK(rcl_publish(&pub_imu, &msg_imu, NULL));
}

void setup() {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    
    pinMode(L_PWM_F, OUTPUT); pinMode(L_PWM_R, OUTPUT);
    pinMode(R_PWM_F, OUTPUT); pinMode(R_PWM_R, OUTPUT);

    Wire.begin(I2C_SDA, I2C_SCL);
    mpu.begin(0x68, &Wire);

    enc_fl.attachHalfQuad(ENC_FL_A, ENC_FL_B);
    enc_fr.attachHalfQuad(ENC_FR_A, ENC_FR_B);
    enc_rl.attachHalfQuad(ENC_RL_A, ENC_RL_B);
    enc_rr.attachHalfQuad(ENC_RR_A, ENC_RR_B);
    
    while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) { delay(100); }
    
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "base_node", "", &support));

    syncTime();

    RCCHECK(rclc_publisher_init_default(&pub_odom, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom/unfiltered"));
    RCCHECK(rclc_publisher_init_default(&pub_imu, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw"));
    RCCHECK(rclc_subscription_init_default(&sub_cmd, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

    // --- FIXED: Micro-ROS String Initialization ---
    msg_odom.header.frame_id.data = (char*)"odom";
    msg_odom.header.frame_id.size = strlen(msg_odom.header.frame_id.data);
    msg_odom.header.frame_id.capacity = msg_odom.header.frame_id.size + 1;

    msg_odom.child_frame_id.data = (char*)"base_footprint";
    msg_odom.child_frame_id.size = strlen(msg_odom.child_frame_id.data);
    msg_odom.child_frame_id.capacity = msg_odom.child_frame_id.size + 1;

    msg_imu.header.frame_id.data = (char*)"imu_link";
    msg_imu.header.frame_id.size = strlen(msg_imu.header.frame_id.data);
    msg_imu.header.frame_id.capacity = msg_imu.header.frame_id.size + 1;

    // --- FIXED: Valid Base Quaternion for IMU ---
    msg_imu.orientation.x = 0.0;
    msg_imu.orientation.y = 0.0;
    msg_imu.orientation.z = 0.0;
    msg_imu.orientation.w = 1.0;

    // --- FIXED: Covariance Initialization (Prevents EKF NaN errors) ---
    // Initialize all to 0 first
    for(int i = 0; i < 36; i++) {
        msg_odom.pose.covariance[i] = 0.0;
        msg_odom.twist.covariance[i] = 0.0;
    }
    for(int i = 0; i < 9; i++) {
        msg_imu.angular_velocity_covariance[i] = 0.0;
        msg_imu.linear_acceleration_covariance[i] = 0.0;
        msg_imu.orientation_covariance[i] = -1.0; // Mark orientation as unknown
    }

    // Set diagonals (Small uncertainty)
    msg_odom.pose.covariance[0] = 0.01;  // x
    msg_odom.pose.covariance[7] = 0.01;  // y
    msg_odom.pose.covariance[35] = 0.05; // yaw
    msg_odom.twist.covariance[0] = 0.01; // linear x
    msg_odom.twist.covariance[35] = 0.05;// angular z

    msg_imu.angular_velocity_covariance[0] = 0.01;
    msg_imu.angular_velocity_covariance[4] = 0.01;
    msg_imu.angular_velocity_covariance[8] = 0.01;
    msg_imu.linear_acceleration_covariance[0] = 0.01;
    msg_imu.linear_acceleration_covariance[4] = 0.01;
    msg_imu.linear_acceleration_covariance[8] = 0.01;


    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(20), timer_callback));
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd, &msg_cmd, &cmd_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
    static unsigned long last_sync = 0;
    
    // Automatically re-sync time every 10 seconds to prevent TF drift
    if (millis() - last_sync > 10000) {
        if (rmw_uros_ping_agent(10, 1) == RMW_RET_OK) {
            rmw_uros_sync_session(10);
            time_offset = rmw_uros_epoch_millis() - millis();
        }
        last_sync = millis();
    }

    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}

