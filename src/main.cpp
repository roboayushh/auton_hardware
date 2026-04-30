#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Encoder.h>

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

// --- CONSTANTS & TIMING ---
const float WHEEL_BASE = 0.20;
const int TICKS_PER_REV = 233;
const unsigned long LOOP_PERIOD = 50; // 50ms = Exactly 20 Hz for the EKF

float target_v = 0.0;
float target_w = 0.0;

ESP32Encoder enc_fl, enc_fr, enc_rl, enc_rr;
Adafruit_MPU6050 mpu;

long prev_l = 0, prev_r = 0;
unsigned long prev_time = 0;

// Non-blocking serial buffer
String input_buffer = "";

void set_motor_speed(int pwm_f, int pwm_r, float speed) {
    int pwm = min((int)fabs(speed * 500.0), 255);
    if (speed > 0.1) {
        analogWrite(pwm_f, pwm); analogWrite(pwm_r, 0);
    } else if (speed < -0.1) {
        analogWrite(pwm_f, 0); analogWrite(pwm_r, pwm);
    } else {
        analogWrite(pwm_f, 0); analogWrite(pwm_r, 0);
    }
}

void parse_cmd(String line) {
    if (!line.startsWith("CMD")) return;

    float v, w;
    sscanf(line.c_str(), "CMD,%f,%f", &v, &w);
    target_v = v;
    target_w = w;
}

void setup() {
    Serial.begin(115200);

    pinMode(L_PWM_F, OUTPUT);
    pinMode(L_PWM_R, OUTPUT);
    pinMode(R_PWM_F, OUTPUT);
    pinMode(R_PWM_R, OUTPUT);

    Wire.begin(I2C_SDA, I2C_SCL);
    mpu.begin();

    enc_fl.attachHalfQuad(ENC_FL_A, ENC_FL_B);
    enc_fr.attachHalfQuad(ENC_FR_A, ENC_FR_B);
    enc_rl.attachHalfQuad(ENC_RL_A, ENC_RL_B);
    enc_rr.attachHalfQuad(ENC_RR_A, ENC_RR_B);

    prev_time = millis();
}

void loop() {
    // --- 1. NON-BLOCKING SERIAL READ ---
    // Read bytes as they arrive without stopping the microcontroller
    while (Serial.available() > 0) {
        char inChar = (char)Serial.read();
        if (inChar == '\n') {
            parse_cmd(input_buffer);
            input_buffer = ""; // Clear buffer after parsing
        } else {
            input_buffer += inChar;
        }
    }

    // --- 2. RIGID TIMING EXECUTION ---
    unsigned long now = millis();
    
    // Only execute control math if exactly 50ms have passed
    if (now - prev_time >= LOOP_PERIOD) {
        prev_time = now;

        // --- ENCODERS ---
        long curr_l = (enc_fl.getCount() + enc_rl.getCount()) / 2;
        long curr_r = -(enc_fr.getCount() + enc_rr.getCount()) / 2;

        long dl = curr_l - prev_l;
        long dr = curr_r - prev_r;

        prev_l = curr_l;
        prev_r = curr_r;

        // --- MOTOR CONTROL ---
        float t_l = target_v - (target_w * WHEEL_BASE / 2.0);
        float t_r = target_v + (target_w * WHEEL_BASE / 2.0);

        set_motor_speed(L_PWM_F, L_PWM_R, t_l);
        set_motor_speed(R_PWM_F, R_PWM_R, t_r);

        // --- IMU ---
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        // --- SEND ABSOLUTE TIMESTAMP PACKET ---
        // Notice we are sending 'now' (unsigned long) instead of 'dt'
        Serial.printf(
            "PKT,%lu,%ld,%ld,%.3f,%.3f,%.3f,%.3f\n",
            now, dl, dr,
            a.acceleration.x,
            a.acceleration.y,
            a.acceleration.z,
            g.gyro.z
        );
    }
    
    // Loop repeats as fast as possible, but math only triggers at 20Hz.
    // No delay() allowed here.
}