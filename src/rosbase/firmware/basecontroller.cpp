#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Arduino.h>

// Config
const int decay_mode = 0;  // 0 - fast, 255 - slow
const int max_linear_vel = 1;
const int max_angular_vel = 1;
const float linear_coeff = 255 / max_linear_vel;
const float angular_coeff = 255 / max_angular_vel;

// Main Control
int pwm_linear = 0;
int pwm_angular = 0;

// ROS
void set_velocity(const geometry_msgs::Twist& vel_msg);

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> vel_sub("cmd_vel", set_velocity);

geometry_msgs::Twist rf_msg;
ros::Publisher rf_publisher("rf_vel", &rf_msg);

geometry_msgs::Twist pwm_msg;
ros::Publisher pwm_publisher("current_pwm", &pwm_msg);

// Base
uint8_t left_forward = 3;
uint8_t left_backward = 9;
uint8_t right_forward = 5;
uint8_t right_backward = 6;

// RF
uint8_t rf_forward = 8;
uint8_t rf_backward = 7;
uint8_t rf_left = 4;
uint8_t rf_right = 2;

// Drivers
void set_motor_pwm(int pwm_linear, int pwm_angular) {
    int left_forward_pwm = decay_mode;
    int left_backward_pwm = decay_mode;
    int right_forward_pwm = decay_mode;
    int right_backward_pwm = decay_mode;

    if (pwm_linear == 0 && pwm_angular == 0) {
        left_forward_pwm = 0;
        left_backward_pwm = 0;
        right_forward_pwm = 0;
        right_backward_pwm = 0;
    } else if (pwm_angular < 0) {  // Right
        if (pwm_linear == 0) {
            left_forward_pwm = min(-pwm_angular, 255);
            right_backward_pwm = min(-pwm_angular, 255);
        } else if (pwm_linear > 0) {
            left_forward_pwm = min(pwm_linear - pwm_angular, 255);
            right_forward_pwm = max(pwm_linear + pwm_angular, 0);
        } else {
            left_backward_pwm = min(-pwm_linear - pwm_angular, 255);
            right_backward_pwm = max(-pwm_linear + pwm_angular, 0);
        }
    } else {  // Left
        if (pwm_linear == 0) {
            left_backward_pwm = min(pwm_angular, 255);
            right_forward_pwm = min(pwm_angular, 255);
        } else if (pwm_linear > 0) {
            left_forward_pwm = max(pwm_linear - pwm_angular, 0);
            right_forward_pwm = min(pwm_linear + pwm_angular, 255);
        } else {
            left_backward_pwm = max(-pwm_linear - pwm_angular, 0);
            right_backward_pwm = min(-pwm_linear + pwm_angular, 255);
        }
    }

    // Pass to motor controller
    analogWrite(left_forward, left_forward_pwm);
    analogWrite(left_backward, left_backward_pwm);
    analogWrite(right_forward, right_forward_pwm);
    analogWrite(right_backward, right_backward_pwm);

    // Update state
    pwm_msg.linear.x = left_forward_pwm;
    pwm_msg.linear.y = left_backward_pwm;
    pwm_msg.linear.z = right_forward_pwm;
    pwm_msg.angular.x = right_backward_pwm;
    pwm_msg.angular.y = pwm_linear;
    pwm_msg.angular.z = pwm_angular;
}

void set_velocity(const geometry_msgs::Twist& vel_msg) {
    ::pwm_linear = int(vel_msg.linear.x * linear_coeff);
    ::pwm_angular = int(vel_msg.angular.z * angular_coeff);

    if (pwm_linear == 0 && pwm_angular == 0) {
        set_motor_pwm(0, 0);  // Stop
        return;
    }

    float scaling_factor = max(float(abs(pwm_linear) + abs(pwm_angular)) / 255, 1.0);
    ::pwm_linear = round(float(pwm_linear) / scaling_factor);
    ::pwm_angular = round(float(pwm_angular) / scaling_factor);
}

void read_rf() {
    rf_msg.linear.x = digitalRead(rf_forward) - digitalRead(rf_backward);
    rf_msg.angular.z = digitalRead(rf_left) - digitalRead(rf_right);
}

void setup() {
    pinMode(left_forward, OUTPUT);
    pinMode(left_backward, OUTPUT);
    pinMode(right_forward, OUTPUT);
    pinMode(right_backward, OUTPUT);

    pinMode(rf_forward, INPUT);
    pinMode(rf_backward, INPUT);
    pinMode(rf_left, INPUT);
    pinMode(rf_right, INPUT);

    nh.initNode();
    nh.subscribe(vel_sub);

    nh.advertise(pwm_publisher);
    nh.advertise(rf_publisher);
}

void loop() {
    read_rf();
    rf_publisher.publish(&rf_msg);

    ::pwm_linear = 0;
    ::pwm_angular = 0;
    nh.spinOnce();

    set_motor_pwm(::pwm_linear, ::pwm_angular);
    pwm_publisher.publish(&pwm_msg);
}
