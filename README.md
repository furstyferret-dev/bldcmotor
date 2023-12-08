# Brushless DC motor control

This library provides a means to control the cheap three-phase brushless DC motors available from AliExpress and similar sources. They are often described as suitable for massage guns owing to their high torque. 

![image](https://github.com/furstyferret-dev/bldcmotor/assets/56730846/5bdcd9d1-693f-4d4e-abae-f2794de49ef5)

If you want to use one, search for BL3830O, or keywords from "BL4825 Outer Rotor Brushless Motor DC 12V 18V 24V 4000RPM High Speed Large Torque PWM Speed Regulation for Fascial Gun". What they don't mention is that the whole unit spins - the only stationary component is the controller board with the triangular mounting bracket, which are 21.7mm / 120 degrees apart.

Although described as high speed (1000 rpm or more) I have no problem running these at between 200-500 rpm. They use almost no power and don't get hot, which being easily regulated to within 0.1 rpm.

Speed is regulated by a PWM signal but they don't have intrinsic speed control,
instead containing an internal frequency generator which pulses at a rate proportional
to motor speed.

They typically run on 6 - 24V DC and require three digital pins, in addition
to power and ground. DO NOT REVERSE POLARITY ON THE SUPPLY.

PWR: 9-24v DC (motor dependent). Higher voltage = higher speed. 
GND: Ground (must be common with the ESP32 ground).
PWM: 3.3-5v PWM signal. 0% is maximum speed, 100% is stopped.
FG:  Motor dependent. Typically 5 pulses per rotation.
F/R: Set motor direction. Changes will only become active once the motor stops.

## DEPENDENCIES:
- QuickPID library

# Getting started
- The example program assumes the PWM pin is on 14, and the FG pin is on 17. Although the FG pulses are 5v and strictly speaking outside of the ESP32 specification, the internal pull-up resistors will prevent damage. However, I used a level shifter for long term projects.
- The interrupt for frequency sensing needs to be in your main program and defined in the setup() method. Just copy and paste if you're not sure.
- When running the example, you can send speed commands to the motor by typing them in to the serial monitor. Make sure you turn off carriage returns though or the motor will stop...

## FAQs
- The motor is too fast / slow / oscillates / rumbles.
Tweak the PID gains. I started very low, but if you don't mind an aggressive response then you can bump up the gains (particularly kP). Each situation is different. It's possible to set them in the constructor or via calling setPID(float Kp, float Ki, float Kd). You can also alter the interval that the loop runs at.

- Debugging info needed? Call the constructor with true as an extra parameter: BLDCmotor(int motorPin, int fgPin, bool debug = false);
