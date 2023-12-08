// BLDCmotor.cpp
/* 
This library provides a means to control the cheap three-phase 
brushless DC motors available from AliExpress and similar sources.
They are often described as suitable for massage guns owing to their
high torque. 

Speed is regulated by a PWM signal but they don't have intrinsic speed control,
instead containing an internal frequency generator which pulses at a rate proportional
to motor speed.

They typically run on 6 - 24V DC and require three digital pins, in addition
to power and ground. 

PWR: 6-24v DC (motor dependent). Higher voltage = higher speed.
GND: Ground (must be common with the ESP32 ground).
PWM: 3.3-5v PWM signal. 0% is maximum speed, 100% is stopped.
FG:  Motor dependent. Typically 5 pulses per rotation.
F/R: Set motor direction. Changes will only become active once the motor stops.

DEPENDENCIES:
- QuickPID library

*/

#include "BLDCmotor.h"

// Helper function to calculate the RPM from the pulse length
double BLDCmotor::calculateRPM(unsigned long duration) {
  if (duration < 1000)
    return 0;

  // Calculate the time for one complete rotation in seconds
  double rotationTime = (10.0 * duration) / 1000000.0;

  // Calculate RPM: 1 minute/rotationTime
  double rpm = 60.0 / rotationTime;

  return rpm;
}

// Constructor with motor pin and FG pin
BLDCmotor::BLDCmotor(int motorPin, int fgPin, bool debug) {
  DEBUG = debug;
  
  // Assign the motor pin and the FG pin
  _motorPin = motorPin;
  _fgPin = fgPin;

  // Use default PWM parameters
  _pwmChannel = 0;
  _pwmResolution = 10;
  _pwmFrequency = 18000;

  // Use default PID parameters
  _Kp = 0.05;
  _Ki = 0.025;
  _Kd = 0.05;
  _setpoint = 330.30;
  _sampleTime = 100000; // Set the initial sample time
}

// Constructor with motor pin, FG pin, PWM channel, resolution, and frequency
BLDCmotor::BLDCmotor(int motorPin, int fgPin, int pwmChannel, int pwmResolution, int pwmFrequency, bool debug) {
  DEBUG = debug;
  
  // Assign the motor pin and the FG pin
  _motorPin = motorPin;
  _fgPin = fgPin;

  // Assign the PWM parameters
  _pwmChannel = pwmChannel;
  _pwmResolution = pwmResolution;
  _pwmFrequency = pwmFrequency;

  // Use default PID parameters
  _Kp = 0.05;
  _Ki = 0.025;
  _Kd = 0.05;
  _setpoint = 330.30;
  _sampleTime = 100000; // Set the initial sample time
}

// Initialize the motor and the PID controller
void BLDCmotor::begin() {
  // Set the motor pin and the FG pin as output and input respectively
  pinMode(_motorPin, OUTPUT);
  pinMode(_fgPin, INPUT);

  // Set up the PWM signal for the motor pin
  ledcSetup(_pwmChannel, _pwmFrequency, _pwmResolution);
  ledcAttachPin(_motorPin, _pwmChannel);

  // Create a new PID controller object
  _motorPID = new QuickPID(&_input, &_output, &_setpoint);

  // Set the PID tunings
  _motorPID->SetTunings(_Kp, _Ki, _Kd);

  // Set the PID mode to automatic
  _motorPID->SetMode(_motorPID->Control::automatic);

  // Set the PID sample time in microseconds
  _motorPID->SetSampleTimeUs(_sampleTime);

  // Set the PID output limits to match the PWM range
  _motorPID->SetOutputLimits(0, 1000);
}

// Set the target speed in RPM
void BLDCmotor::setTargetSpeed(float targetSpeed) {
  _setpoint = targetSpeed;
}

// Get the target speed in RPM
float BLDCmotor::getTargetSpeed() {
  return _setpoint;
}

// Set the PID constants
void BLDCmotor::setPID(float Kp, float Ki, float Kd) {
  _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;
  _motorPID->SetTunings(_Kp, _Ki, _Kd);
}

// Get the PID constants
void BLDCmotor::getPID(float &Kp, float &Ki, float &Kd) {
  Kp = _Kp;
  Ki = _Ki;
  Kd = _Kd;
}

// Set the PID interval time in microseconds
void BLDCmotor::setPIDInterval(unsigned long interval) {
  _sampleTime = interval;
  _motorPID->SetSampleTimeUs(_sampleTime);
}

// Get the PID interval time in microseconds
unsigned long BLDCmotor::getPIDInterval() {
  return _sampleTime;
}

// Read the current motor speed in RPM
float BLDCmotor::readSpeed() {
  // Calculate the average pulse length
  unsigned long pulseLengthAvg = (_pulseLengths.empty() ? 0 : _pulseLengthSum / _pulseLengths.size());
  unsigned long lastPulseTimeLocal = _lastPulseTime;

  // Check if a pulse has been received recently
  if (micros() - lastPulseTimeLocal > _pulseTimeout) {
    // If not, the motor has stopped, so set the average pulse length to a value that corresponds to 0 RPM
    pulseLengthAvg = 0;
  }

  // Calculate the RPM from the average pulse length
  float rpm = calculateRPM(pulseLengthAvg);

  return rpm;
}

// Start the motor
void BLDCmotor::run() {
  _running = true;	
  // Write a non-zero PWM signal to the motor pin
  ledcWrite(_pwmChannel, 1000 - 100);
}

// Stop the motor
void BLDCmotor::stop() {
  // Write a zero PWM signal to the motor pin
  ledcWrite(_pwmChannel, 1000 - 0);
  _running = false;
}

// Update the motor speed according to the PID output
void BLDCmotor::update() {
  // Read the current motor speed
  _input = readSpeed();

  // Compute the PID output
  _motorPID->Compute();

  // Write the PID output to the motor pin
  if (_running) ledcWrite(_pwmChannel, 1000 - _output);
   
  if (DEBUG) { 
  // Print the input, output, and setpoint values every second
  unsigned long currentMillis = millis();
  if (currentMillis - _lastPrintTime >= 1000) {
    Serial.print("\tOutput:");
    Serial.print(_output);
    Serial.print("\tRPM:");
    Serial.print(readSpeed());
    Serial.print("\tSetpoint:");
    Serial.println(_setpoint);
    _lastPrintTime = currentMillis;
  }
  }
}

void BLDCmotor::handleInterrupt(int pinState) {
 unsigned long interruptTime = micros();

  // If the pin is HIGH, it's the rising edge of the pulse, so record the start time
  if (pinState == HIGH) {
    _pulseStartTime = interruptTime;
  }
  // If the pin is LOW, it's the falling edge of the pulse, so calculate the pulse length
  else {
    unsigned long pulseLength = interruptTime - _pulseStartTime;
    _lastPulseTime = interruptTime;

    // Add the new pulse length to the queue and the sum
    _pulseLengths.push(pulseLength);
    _pulseLengthSum += pulseLength;

    // If the queue is too big, remove the oldest value from the queue and the sum
    if (_pulseLengths.size() > _range) {
      _pulseLengthSum -= _pulseLengths.front();
      _pulseLengths.pop();
    }
  }
}
