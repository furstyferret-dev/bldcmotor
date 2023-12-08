// BLDCmotor_example.ino

#include <BLDCmotor.h>

#define MOTOR_PIN 14
#define FG_PIN 27

// Create a BLDCmotor object
BLDCmotor motor(MOTOR_PIN, FG_PIN, true);

void IRAM_ATTR handleInterrupt() {
  motor.handleInterrupt(digitalRead(FG_PIN) == HIGH ? HIGH : LOW);
}

void setup() {
  Serial.begin(9600);

  // Attach an interrupt to the motor speed pin
  attachInterrupt(digitalPinToInterrupt(FG_PIN), handleInterrupt, CHANGE);

  // Initialize the motor
  motor.begin();

  // Set the target speed to 333.30 RPM
  motor.setTargetSpeed(333.30);

  // Start the motor
  motor.run();
}

void loop() {
  if (Serial.available()) { 
    float speed = Serial.parseFloat();
    Serial.println(speed);
    motor.setTargetSpeed(speed); 
  }

  // Update the motor speed according to the PID output
  motor.update();
}