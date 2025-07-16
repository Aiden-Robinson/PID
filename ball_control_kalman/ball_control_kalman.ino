// PID control with 2D Kalman Filtering
#include <Arduino.h>
#include <NewPing.h>
#include <Servo.h>

// Pin definitions
const int trigPin = 11;  // HC-SR04 trigger pin
const int echoPin = 10;  // HC-SR04 echo pin
const int servoPin = 9;  // Servo motor pin

// Beam length (maximum valid distance in cm for ultrasonic sensor)
const double beam_length_cm = 16.4;  // Maximum distance for ultrasonic sensor

// NewPing object for ultrasonic sensor
NewPing sonar(trigPin, echoPin, beam_length_cm);

// Servo limits (adjust based on your setup)
double servo_lower_lim_deg = 120;
double servo_upper_lim_deg = 160;
double old_tilt_deg = 140;  // Start at center position

// PID variables
double setpoint_cm = 8;  // Target distance (ball in middle of beam)
double cumulative_error = 0;
double previous_error = 0;

// PID gains (tune these values for your system)
double Kp = 1.2;
double Ki = 0.05;
double Kd = 1;

// Kalman Filter variables
// State vector: [position, velocity]
double state[2] = {8.2,
                   0.0};  // Initial state: position at setpoint, zero velocity
double P[2][2] = {{1.0, 0.0}, {0.0, 1.0}};  // Error covariance matrix
double Q[2][2] = {{0.1, 0.0}, {0.0, 0.1}};  // Process noise covariance
double R = 0.5;                             // Measurement noise variance
double dt = 0.02;                           // Time step (20ms loop)

unsigned long last_time = 0;

Servo myservo;

void setup() {
  Serial.begin(9600);

  // Initialize ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initialize servo
  myservo.attach(servoPin);
  myservo.write(140);  // Start at center position

  delay(1000);  // Give time for servo to reach position
  Serial.println("PID Ball Control with Kalman Filter Started");
  Serial.println("Setpoint: 8.2 cm");
  // Add labels for Serial Plotter
  Serial.println("Setpoint,Position,Error");

  last_time = millis();
}

void loop() {
  // Calculate actual time step
  unsigned long current_time = millis();
  dt = (current_time - last_time) / 1000.0;  // Convert to seconds
  last_time = current_time;

  // Ensure minimum time step to avoid division issues
  if (dt < 0.01) dt = 0.01;
  if (dt > 0.1) dt = 0.1;  // Cap maximum dt

  // Read distance from ultrasonic sensor
  double measurement = readUltrasonicDistance();

  if (measurement > 0 && measurement < beam_length_cm) {  // Valid reading
    // Apply Kalman filter
    kalmanFilter(measurement);

    // Use filtered position for PID control
    double filtered_position = state[0];
    double filtered_velocity = state[1];

    // Calculate PID output using filtered position
    double new_tilt_deg = pid(filtered_position);

    // Update servo position gradually
    update_servo(new_tilt_deg, old_tilt_deg, 5);  // 5ms delay between steps
    old_tilt_deg = new_tilt_deg;

    // Print setpoint, filtered position, and error for Serial Plotter
    double error = setpoint_cm - filtered_position;
    Serial.print(setpoint_cm);  // Setpoint
    Serial.print(",");
    Serial.print(filtered_position);  // Ball position
    Serial.print(",");
    Serial.println(error);  // Error
  }

  delay(20);  // 50 Hz update rate
}

void kalmanFilter(double measurement) {
  // Prediction step
  // State transition matrix F
  double F[2][2] = {{1.0, dt}, {0.0, 1.0}};

  // Predict state: x = F * x
  double predicted_state[2];
  predicted_state[0] = state[0] + dt * state[1];  // position + velocity * dt
  predicted_state[1] = state[1];                  // velocity remains same

  // Predict error covariance: P = F * P * F^T + Q
  double temp_P[2][2];
  // temp_P = F * P
  temp_P[0][0] = P[0][0] + dt * P[1][0];
  temp_P[0][1] = P[0][1] + dt * P[1][1];
  temp_P[1][0] = P[1][0];
  temp_P[1][1] = P[1][1];

  // P = temp_P * F^T + Q
  P[0][0] = temp_P[0][0] + dt * temp_P[0][1] + Q[0][0];
  P[0][1] = temp_P[0][1] + Q[0][1];
  P[1][0] = temp_P[1][0] + dt * temp_P[1][1] + Q[1][0];
  P[1][1] = temp_P[1][1] + Q[1][1];

  // Update step
  // Innovation (measurement residual)
  double innovation =
      measurement - predicted_state[0];  // We only measure position

  // Innovation covariance
  double S = P[0][0] + R;

  // Kalman gain
  double K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  // Update state estimate
  state[0] = predicted_state[0] + K[0] * innovation;
  state[1] = predicted_state[1] + K[1] * innovation;

  // Update error covariance
  double temp_P_update[2][2];
  temp_P_update[0][0] = P[0][0] - K[0] * P[0][0];
  temp_P_update[0][1] = P[0][1] - K[0] * P[0][1];
  temp_P_update[1][0] = P[1][0] - K[1] * P[0][0];
  temp_P_update[1][1] = P[1][1] - K[1] * P[0][1];

  // Copy back to P
  P[0][0] = temp_P_update[0][0];
  P[0][1] = temp_P_update[0][1];
  P[1][0] = temp_P_update[1][0];
  P[1][1] = temp_P_update[1][1];
}

double readUltrasonicDistance() {
  // Use NewPing library to get distance in cm
  unsigned int distance = sonar.ping_cm();
  return (double)distance;
}

void update_servo(double new_tilt_deg, double old_tilt_deg,
                  double delay_time_ms) {
  // Gradually move servo to new position to avoid jerky movements
  if (new_tilt_deg > old_tilt_deg) {
    for (double pos = old_tilt_deg; pos <= new_tilt_deg; pos += 1) {
      myservo.write(pos);
      delay(delay_time_ms);
    }
  }

  if (new_tilt_deg < old_tilt_deg) {
    for (double pos = old_tilt_deg; pos >= new_tilt_deg; pos -= 1) {
      myservo.write(pos);
      delay(delay_time_ms);
    }
  }
}

double pid(double distance_cm) {
  // Calculate error
  double error = setpoint_cm - distance_cm;

  // Proportional term
  double p_value = error * Kp;

  // Integral term
  cumulative_error += error;
  double i_value = cumulative_error * Ki;

  // Derivative term
  double d_value = (error - previous_error) * Kd;

  // Total PID output
  double pid_value = p_value + i_value + d_value;

  // Update for next iteration
  previous_error = error;

  // Map PID output to servo angle
  double servo_range = servo_upper_lim_deg - servo_lower_lim_deg;
  double new_servo_angle = map(pid_value, -1 * servo_range, servo_range,
                               servo_lower_lim_deg, servo_upper_lim_deg);

  // Constrain servo angle within limits
  if (new_servo_angle > servo_upper_lim_deg) {
    new_servo_angle = servo_upper_lim_deg;
  }

  if (new_servo_angle < servo_lower_lim_deg) {
    new_servo_angle = servo_lower_lim_deg;
  }

  return new_servo_angle;
}
