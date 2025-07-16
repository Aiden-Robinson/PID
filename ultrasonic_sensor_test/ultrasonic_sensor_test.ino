
#include <Arduino.h>
// HC-SR04 Ultrasonic Sensor Test
// Wiring:
// VCC  -> 5V
// GND  -> GND
// TRIG -> D11
// ECHO -> D10

const int trigPin = 11;
const int echoPin = 10;

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  long duration;
  float distance;

  // Clear the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Set the trigPin HIGH for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echoPin
  duration = pulseIn(echoPin, HIGH);

  // Calculate the distance (cm)
  distance = duration * 0.0343 / 2;

  // Print the distance to Serial (for Serial Plotter)
  Serial.println(distance);

  delay(20);  // 50 Hz update rate
}
