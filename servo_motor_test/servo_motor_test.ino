// Basic Servo Control with Serial Input (A = left, D = right)
#include <Servo.h>

Servo myServo;
int pos = 140;  // Start at center position
const int servoPin = 9;

void setup() {
  myServo.attach(servoPin);
  myServo.write(pos);
  Serial.begin(9600);
  Serial.println("Send 'A' to turn left, 'D' to turn right.");
}

void loop() {
  if (Serial.available() > 0) {
    char input = Serial.read();
    if (input == 'A' || input == 'a') {
      pos -= 10;
      if (pos < 0) pos = 0;
      myServo.write(pos);
      Serial.println(pos);  // For Serial Plotter
    } else if (input == 'D' || input == 'd') {
      pos += 10;
      if (pos > 180) pos = 180;
      myServo.write(pos);
      Serial.println(pos);  // For Serial Plotter
    }
  }
}
