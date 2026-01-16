#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>

#include <ESP32Encoder.h>
#include <CytronMotorDriver.h>

// ================== PIN DEFINITIONS ==================
#define ENC_A 34
#define ENC_B 35
#define MOTOR_PWM 33
#define MOTOR_DIR 32
#define LED_PIN 2

// ================== CONSTANTS ==================
#define COUNTS_PER_REV 12000
#define MAX_PWM 200
#define MIN_PWM 40

// ================== DIRECTION INVERSION FLAGS ==================
#define INVERT_MOTOR false
#define INVERT_ENCODER false

// ================== PID PARAMETERS ==================
float KP = 0.0;      // Can be tuned via ROS parameters
float KI = 0.0;
float KD = 0.0;

// ================== POSITION TOLERANCE ==================
const float TOLERANCE_DEG = 0.5;        // Tolerance in degrees
const int SETTLED_THRESHOLD = 40;       // Number of cycles to be stable (40 * 5ms = 200ms at 200Hz)

// ================== MICRO-ROS OBJECTS ==================
rcl_publisher_t pub_current_angle;
rcl_publisher_t pub_target_angle;
rcl_publisher_t pub_error;
rcl_publisher_t pub_pwm;
rcl_publisher_t pub_p_term;
rcl_publisher_t pub_i_term;
rcl_publisher_t pub_d_term;

rcl_subscription_t sub_target_angle;
rcl_subscription_t sub_kp;
rcl_subscription_t sub_ki;
rcl_subscription_t sub_kd;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

std_msgs__msg__Float32 msg_current_angle;
std_msgs__msg__Float32 msg_target_angle;
std_msgs__msg__Float32 msg_error;
std_msgs__msg__Int32 msg_pwm;
std_msgs__msg__Float32 msg_p_term;
std_msgs__msg__Float32 msg_i_term;
std_msgs__msg__Float32 msg_d_term;

std_msgs__msg__Float32 received_target_angle;
std_msgs__msg__Float32 received_kp;
std_msgs__msg__Float32 received_ki;
std_msgs__msg__Float32 received_kd;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// ================== MOTOR OBJECTS ==================
ESP32Encoder encoder;
CytronMD motor(PWM_DIR, MOTOR_PWM, MOTOR_DIR);

// ================== PID VARIABLES ==================
float target_angle = 0.0;
float current_angle = 0.0;
float error_angle = 0;
float last_error_angle = 0;
float integral_angle = 0;
float derivative_angle = 0;
unsigned long last_time = 0;
int motor_pwm = 0;
int settled_count = 0;  // NEW: Counter for stable position

// ================== ERROR HANDLING ==================
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// ================== ANGLE CONVERSION ==================
float countsToAngle(long counts) {
  float angle = (counts * 360.0) / COUNTS_PER_REV;
  return INVERT_ENCODER ? -angle : angle;
}

// ================== SUBSCRIPTION CALLBACKS ==================
void subscription_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  target_angle = msg->data;
  
  // Reset integral and settled counter when new target is received
  integral_angle = 0;
  settled_count = 0;  // NEW: Reset counter on new target
  
  Serial.print("New target received: ");
  Serial.print(target_angle);
  Serial.println("°");
}

void kp_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  KP = msg->data;
  integral_angle = 0;  // Reset integral when gains change
  settled_count = 0;   // NEW: Reset counter when gains change
  Serial.print("KP updated to: ");
  Serial.println(KP, 3);
}

void ki_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  KI = msg->data;
  integral_angle = 0;  // Reset integral when gains change
  settled_count = 0;   // NEW: Reset counter when gains change
  Serial.print("KI updated to: ");
  Serial.println(KI, 3);
}

void kd_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  KD = msg->data;
  settled_count = 0;   // NEW: Reset counter when gains change
  Serial.print("KD updated to: ");
  Serial.println(KD, 3);
}

// ================== TIMER CALLBACK (PID LOOP) ==================
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    
    // Read current position
    long current_counts = encoder.getCount();
    current_angle = countsToAngle(current_counts);
    
    // Calculate error
    error_angle = target_angle - current_angle;
    
    // Calculate time delta
    unsigned long current_time = millis();
    float dt = (current_time - last_time) / 1000.0;
    last_time = current_time;
    
    if (dt > 0) {
      // PID calculations
      integral_angle += error_angle * dt;
      integral_angle = constrain(integral_angle, -50, 50);  // Anti-windup
      
      derivative_angle = (error_angle - last_error_angle) / dt;
      
      float p_term = KP * error_angle;
      float i_term = KI * integral_angle;
      float d_term = KD * derivative_angle;
      
      float pid_output = p_term + i_term + d_term;
      
      // Convert to PWM
      motor_pwm = constrain(pid_output, -MAX_PWM, MAX_PWM);
      
      if (INVERT_MOTOR) {
        motor_pwm = -motor_pwm;
      }
      
      // Deadband compensation
      if (abs(motor_pwm) > 0 && abs(motor_pwm) < MIN_PWM) {
        motor_pwm = (motor_pwm > 0) ? MIN_PWM : -MIN_PWM;
      }
      
      // NEW: Settled counter logic - only stop after being stable
      if (abs(error_angle) < TOLERANCE_DEG) {
        settled_count++;
        if (settled_count >= SETTLED_THRESHOLD) {
          motor_pwm = 0;  // Stop motor after being stable
        }
      } else {
        settled_count = 0;  // Reset counter if error exceeds tolerance
      }
      
      // Apply motor control
      motor.setSpeed(motor_pwm);
      
      // Publish data to ROS
      msg_current_angle.data = current_angle;
      msg_target_angle.data = target_angle;
      msg_error.data = error_angle;
      msg_pwm.data = motor_pwm;
      msg_p_term.data = p_term;
      msg_i_term.data = i_term;
      msg_d_term.data = d_term;

      RCSOFTCHECK(rcl_publish(&pub_current_angle, &msg_current_angle, NULL));
      RCSOFTCHECK(rcl_publish(&pub_target_angle, &msg_target_angle, NULL));
      RCSOFTCHECK(rcl_publish(&pub_error, &msg_error, NULL));
      RCSOFTCHECK(rcl_publish(&pub_pwm, &msg_pwm, NULL));
      RCSOFTCHECK(rcl_publish(&pub_p_term, &msg_p_term, NULL));
      RCSOFTCHECK(rcl_publish(&pub_i_term, &msg_i_term, NULL));
      RCSOFTCHECK(rcl_publish(&pub_d_term, &msg_d_term, NULL));
      
      last_error_angle = error_angle;
    }
  }
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  delay(1000);

  // ========== SETUP ENCODER & MOTOR ==========
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachFullQuad(ENC_A, ENC_B);
  encoder.clearCount();

  Serial.println("=== Setting up Micro-ROS ===");
  
  // ========== SETUP MICRO-ROS ==========
  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "pid_motor_controller", "", &support));

  // Create publishers
  RCCHECK(rclc_publisher_init_default(
    &pub_current_angle,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor/current_angle"));

  RCCHECK(rclc_publisher_init_default(
    &pub_target_angle,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor/target_angle"));

  RCCHECK(rclc_publisher_init_default(
    &pub_error,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor/error"));

  RCCHECK(rclc_publisher_init_default(
    &pub_pwm,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "motor/pwm"));

  RCCHECK(rclc_publisher_init_default(
    &pub_p_term,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor/pid/p_term"));

  RCCHECK(rclc_publisher_init_default(
    &pub_i_term,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor/pid/i_term"));

  RCCHECK(rclc_publisher_init_default(
    &pub_d_term,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor/pid/d_term"));

  // Create subscriber for target angle
  RCCHECK(rclc_subscription_init_default(
    &sub_target_angle,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor/target_angle_cmd"));

  // Create subscribers for PID parameters
  RCCHECK(rclc_subscription_init_default(
    &sub_kp,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor/pid/kp"));

  RCCHECK(rclc_subscription_init_default(
    &sub_ki,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor/pid/ki"));

  RCCHECK(rclc_subscription_init_default(
    &sub_kd,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor/pid/kd"));

  // Create timer (5ms = 200Hz control loop - matching first code)
  const unsigned int timer_timeout = 0.2
  
  ;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Create executor (1 timer + 4 subscriptions = 5 handles)
  // Note: Sometimes need to add extra handles for stability
  RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
  
  // Add timer to executor
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  
  // Add target angle subscription
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_target_angle, &received_target_angle, 
          &subscription_callback, ON_NEW_DATA));

  // Add PID parameter subscriptions
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_kp, &received_kp, 
          &kp_callback, ON_NEW_DATA));
  
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_ki, &received_ki, 
          &ki_callback, ON_NEW_DATA));
  
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_kd, &received_kd, 
          &kd_callback, ON_NEW_DATA));

  last_time = millis();

  Serial.println("=== PID Motor Control with Micro-ROS Ready ===");
  Serial.println("Publishing to topics:");
  Serial.println("  - motor/current_angle");
  Serial.println("  - motor/target_angle");
  Serial.println("  - motor/error");
  Serial.println("  - motor/pwm");
  Serial.println("  - motor/pid/p_term");
  Serial.println("  - motor/pid/i_term");
  Serial.println("  - motor/pid/d_term");
  Serial.println("Subscribing to:");
  Serial.println("  - motor/target_angle_cmd");
  Serial.println("  - motor/pid/kp");
  Serial.println("  - motor/pid/ki");
  Serial.println("  - motor/pid/kd");
  Serial.println("\nYou can now tune PID parameters in real-time!");
  Serial.print("Current gains - KP: ");
  Serial.print(KP);
  Serial.print(", KI: ");
  Serial.print(KI);
  Serial.print(", KD: ");
  Serial.println(KD);
}

// ================== LOOP ==================
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
