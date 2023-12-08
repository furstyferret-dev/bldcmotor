// BLDCmotor.h

#ifndef BLDCmotor_h
#define BLDCmotor_h

#include <Arduino.h>
#include <QuickPID.h>
#include <queue>

class BLDCmotor {
  public: 
    // Constructor with motor pin and FG pin
    BLDCmotor(int motorPin, int fgPin, bool debug = false);

    // Constructor with motor pin, FG pin, PWM channel, resolution, and frequency
    BLDCmotor(int motorPin, int fgPin, int pwmChannel, int pwmResolution, int pwmFrequency, bool debug = false);

    // Initialize the motor and the PID controller
    void begin();

    // Set the target speed in RPM
    void setTargetSpeed(float targetSpeed);

    // Get the target speed in RPM
    float getTargetSpeed();

    // Set the PID constants
    void setPID(float Kp, float Ki, float Kd);

    // Get the PID constants
    void getPID(float &Kp, float &Ki, float &Kd);

    // Set the PID interval time in microseconds
    void setPIDInterval(unsigned long interval);

    // Get the PID interval time in microseconds
    unsigned long getPIDInterval();

    // Read the current motor speed in RPM
    float readSpeed();

    // Start the motor
    void run();

    // Stop the motor
    void stop();

    // Update the motor speed according to the PID output
    void update();
	
	// Calculate the target speed based off pulse timing
	void handleInterrupt(int pinState);
	
	// Is the motor running
	bool isRunning();

  private:
    // Motor pins
    int _motorPin;
    int _fgPin;

    // PWM parameters
    int _pwmChannel;
    int _pwmResolution;
    int _pwmFrequency;
	bool _running;

    // PID parameters
    float _input;
    float _output;
    float _setpoint;
    float _Kp;
    float _Ki;
    float _Kd;
    unsigned long _sampleTime; // Variable to store the sample time
    QuickPID *_motorPID;

    // Pulse measurement parameters
    const int _range = 10; // Number of values to average
    std::queue<unsigned long> _pulseLengths; // Queue to hold the last 'range' pulse lengths
    unsigned long _pulseLengthSum = 0; // Sum of the last 'range' pulse lengths
    const float _pulsesPerRev = 10.0; // Number of pulses per motor revolution
    const unsigned long _pulseTimeout = 200000; // Timeout for pulse measurement in microseconds
    volatile unsigned long _pulseStartTime = 0;
    volatile unsigned long _pulseLength = 0;
    volatile unsigned long _lastPulseTime = 0;
	
	// Debugging
	unsigned long _lastPrintTime = 0; // Variable to hold the last time the debugging info was printed

    // Helper function to calculate the RPM from the pulse length
    double calculateRPM(unsigned long duration);
	
	bool DEBUG;
};

#endif
