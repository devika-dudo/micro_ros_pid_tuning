#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float64_multi_array.h>

#include <ESP32Encoder.h>
#include <CytronMotorDriver.h>

// ================== MOTOR CONFIGURATION ==================
#define NUM_MOTORS 6

// Motor 1 pins (the one you're testing)
#define ENC_A_1 34
#define ENC_B_1 35
#define MOTOR_PWM_1 33
#define MOTOR_DIR_1 32

#define LED_PIN 2

// ================== MOTOR PARAMETERS ==================
#define COUNTS_PER_REV 240000
#define MAX_PWM 200
#define MIN_PWM 40

// ================== PID PARAMETERS (per motor) ==================
struct MotorPID {
  float kp;
  float ki;
  float kd;
  float target_angle;
  float current_angle;
  float error;
  float last_error;
  float integral;
  int pwm;
  int settled_count;
};

MotorPID motors[NUM_MOTORS];

const float TOLERANCE_DEG = 0.5;
const int SETTLED_THRESHOLD = 40;

// ================== MOTOR OBJECTS ==================
ESP32Encoder encoders[NUM_MOTORS];
CytronMD* motor_drivers[NUM_MOTORS];

// ================== MICRO-ROS OBJECTS ==================
rcl_publisher_t pub_joint_states;
rcl_subscription_t sub_joint_commands;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

sensor_msgs__msg__JointState joint_state_msg;
std_msgs__msg__Float64MultiArray joint_command_msg;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

unsigned long last_time = 0;

// ================== ERROR HANDLING ==================
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// ================== ANGLE CONVERSION ==================
float countsToAngle(long counts) {
  return (counts * 360.0) / COUNTS_PER_REV;
}

// ================== JOINT COMMAND CALLBACK ==================
void joint_command_callback(const void * msgin) {
  const std_msgs__msg__Float64MultiArray * msg = (const std_msgs__msg__Float64MultiArray *)msgin;
  
  for (size_t i = 0; i < NUM_MOTORS && i < msg->data.size; i++) {
    float new_target = msg->data.data[i] * (180.0 / M_PI);
    
    if (abs(new_target - motors[i].target_angle) > 0.1) {
      motors[i].target_angle = new_target;
      motors[i].integral = 0;
      motors[i].settled_count = 0;
    }
  }
}

// ================== PID CONTROL FOR ONE MOTOR ==================
void update_motor_pid(int motor_idx, float dt) {
  MotorPID &m = motors[motor_idx];
  
  m.error = m.target_angle - m.current_angle;
  
  m.integral += m.error * dt;
  m.integral = constrain(m.integral, -50, 50);
  
  float derivative = (m.error - m.last_error) / dt;
  
  float p_term = m.kp * m.error;
  float i_term = m.ki * m.integral;
  float d_term = m.kd * derivative;
  
  float pid_output = p_term + i_term + d_term;
  
  m.pwm = constrain(pid_output, -MAX_PWM, MAX_PWM);
  
  if (abs(m.pwm) > 0 && abs(m.pwm) < MIN_PWM) {
    m.pwm = (m.pwm > 0) ? MIN_PWM : -MIN_PWM;
  }
  
  if (abs(m.error) < TOLERANCE_DEG) {
    m.settled_count++;
    if (m.settled_count >= SETTLED_THRESHOLD) {
      m.pwm = 0;
    }
  } else {
    m.settled_count = 0;
  }
  
  m.last_error = m.error;
}

// ================== TIMER CALLBACK (200Hz CONTROL LOOP) ==================
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    
    unsigned long current_time = millis();
    float dt = (current_time - last_time) / 1000.0;
    last_time = current_time;
    
    if (dt > 0) {
      for (int i = 0; i < NUM_MOTORS; i++) {
        if (i == 0) {
          long counts = encoders[i].getCount();
          motors[i].current_angle = countsToAngle(counts);
          update_motor_pid(i, dt);
          motor_drivers[i]->setSpeed(motors[i].pwm);
        } else {
          motors[i].current_angle = 0.0;
          motors[i].pwm = 0;
        }
      }
      
      joint_state_msg.header.stamp.sec = current_time / 1000;
      joint_state_msg.header.stamp.nanosec = (current_time % 1000) * 1000000;
      
      for (int i = 0; i < NUM_MOTORS; i++) {
        joint_state_msg.position.data[i] = motors[i].current_angle * (M_PI / 180.0);
        joint_state_msg.velocity.data[i] = 0.0;
      }
      
      RCSOFTCHECK(rcl_publish(&pub_joint_states, &joint_state_msg, NULL));
    }
  }
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(1000);

  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].kp = 1.5;
    motors[i].ki = 0.1;
    motors[i].kd = 0.05;
    motors[i].target_angle = 0.0;
    motors[i].current_angle = 0.0;
    motors[i].error = 0.0;
    motors[i].last_error = 0.0;
    motors[i].integral = 0.0;
    motors[i].pwm = 0;
    motors[i].settled_count = 0;
  }

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoders[0].attachFullQuad(ENC_A_1, ENC_B_1);
  encoders[0].clearCount();
  motor_drivers[0] = new CytronMD(PWM_DIR, MOTOR_PWM_1, MOTOR_DIR_1);

  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_motor_controller", "", &support));

  joint_state_msg.name.capacity = NUM_MOTORS;
  joint_state_msg.name.size = NUM_MOTORS;
  joint_state_msg.name.data = (rosidl_runtime_c__String*) malloc(NUM_MOTORS * sizeof(rosidl_runtime_c__String));
  
  joint_state_msg.position.capacity = NUM_MOTORS;
  joint_state_msg.position.size = NUM_MOTORS;
  joint_state_msg.position.data = (double*) malloc(NUM_MOTORS * sizeof(double));
  
  joint_state_msg.velocity.capacity = NUM_MOTORS;
  joint_state_msg.velocity.size = NUM_MOTORS;
  joint_state_msg.velocity.data = (double*) malloc(NUM_MOTORS * sizeof(double));
  
  joint_state_msg.effort.capacity = 0;
  joint_state_msg.effort.size = 0;

  static char frame_id[] = "base_link";
  joint_state_msg.header.frame_id.data = frame_id;
  joint_state_msg.header.frame_id.size = strlen(frame_id);
  joint_state_msg.header.frame_id.capacity = sizeof(frame_id);

  const char* joint_names[NUM_MOTORS] = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  for (int i = 0; i < NUM_MOTORS; i++) {
    joint_state_msg.name.data[i].data = (char*) joint_names[i];
    joint_state_msg.name.data[i].size = strlen(joint_names[i]);
    joint_state_msg.name.data[i].capacity = strlen(joint_names[i]) + 1;
    joint_state_msg.position.data[i] = 0.0;
    joint_state_msg.velocity.data[i] = 0.0;
  }

  RCCHECK(rclc_publisher_init_default(
    &pub_joint_states,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/joint_states_from_esp32"));

  joint_command_msg.data.capacity = NUM_MOTORS;
  joint_command_msg.data.size = 0;
  joint_command_msg.data.data = (double*) malloc(NUM_MOTORS * sizeof(double));

  RCCHECK(rclc_subscription_init_default(
    &sub_joint_commands,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "/joint_commands_to_esp32"));

  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(5),
    timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_joint_commands, &joint_command_msg, 
          &joint_command_callback, ON_NEW_DATA));

  last_time = millis();
}

// ================== LOOP ==================
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
