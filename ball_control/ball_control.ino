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
double setpoint_cm =
    8.2;  // Target distance (ball in middle of beam, adjusted for true center)
double cumulative_error = 0;
double previous_error = 0;

// PID gains (tune these values for your system)
double Kp = 2.4;
double Ki = 0.03;
double Kd = 0.3;

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
  Serial.println("PID Ball Control System Started");
  Serial.println("Setpoint: 8.5 cm");
}

void loop() {
  // Read smoothed distance from ultrasonic sensor
  double distance = getSmoothedDistance(5);  // Average of 5 readings

  if (distance > 0 &&
      distance < beam_length_cm) {  // Valid reading range (beam length)
    // Calculate PID output
    double new_tilt_deg = pid(distance);

    // Update servo position gradually
    update_servo(new_tilt_deg, old_tilt_deg, 5);  // 5ms delay between steps
    old_tilt_deg = new_tilt_deg;

    // Print values for monitoring
    Serial.print("Distance:");
    Serial.print(distance);
    Serial.print(",Setpoint:");
    Serial.print(setpoint_cm);
    Serial.print(",ServoAngle:");
    Serial.println(new_tilt_deg);
  }

  delay(20);  // 50 Hz update rate
}
// Returns the average of multiple ultrasonic readings for smoothing
double getSmoothedDistance(int numReadings) {
  double sum = 0;
  int validCount = 0;
  for (int i = 0; i < numReadings; i++) {
    double d = readUltrasonicDistance();
    if (d > 0 && d < beam_length_cm) {  // Only count valid readings
      sum += d;
      validCount++;
    }
    delay(2);  // Short delay between readings
  }
  if (validCount == 0) return 0;  // No valid readings
  return sum / validCount;
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
