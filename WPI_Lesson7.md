Now that you know how to make the robot move, now it is time to make it move intelligently. However, this can't be done without some sort of indication of the robot's state. This is what sensors are for.

This lesson will discuss how to obtain values from the various sensors on the robot.



# Providing Information #

Sensors are useless if you can't get a reading from them. While this can be done in the code, you still need hard values from the sensors in order to do anything with them. So, you need to find a way to print this data to the screen.

There are two ways to do this: through the driver station and through SmartDashboard.

## Driver Station ##

You already know how to do this. Simply inject the data into a [Printf()](WPI_Lesson4#Printing_to_the_Driver_Station.md) statement.

```
m_dsLCD->Printf(DriverStationLCD::kUserLine1,1,"Sensor Value: %f",m_sensorValue);
```

## SmartDashboard ##

The other way is to use an application called SmartDashboard. SmartDashboard is a program designed to provide data from the robot. It is the C++/Java equivalent of the FRC Dashboard. It also provides a function called [live window mode](WPI_Lesson3#Live_Window.md), which you may remember from lesson 3.

However, how you print data to SmartDashboard is quite different than how you print data to the driver station. As a matter of fact, an object for SmartDashboard is not even required to do so.

What you do depends on the type of variable you want to display on SmartDashboard.

```
void SmartDashboard::PutNumber(const char* name, double value)
void SmartDashboard::PutBoolean(const char* name, bool value)
```

### SmartDashboard parameters ###

  * `const char* name`: Name/identifier of the variable in SmartDashboard.
  * `value`: The variable.

### SmartDashboard use ###

Because there is no object (outside of Live Window) that is required to manage SmartDashboard, so when you want to call these functions, you need to include the `SmartDashboard::` prefix in order for your code to compile.

```
//PutNumber example
SmartDashboard::PutNumber("Sensor Value: ",m_sensorValue);
```

# Reading from Sensors #

The following section contains a selection of functions from commonly used sensors. Look at the header files for each of the classes for the components for all of them.

## A Special Function ##

Most sensors should have this function:

```
int PIDGet()
```

This function is special, because it returns a bitwise value (1-1024) representing the range of the sensor. This is important for later, because this provides the kind of input the cRIO needs to calculate certain values.

## Analog Sensors ##

### AnalogChannel ###

[Analog Channel](WPI_Lesson3#AnalogChannel.md) devices use voltage as a means of measurement. What the voltage range is depends on the sensor being used. Potentiometers, the device most commonly used that falls under this class, have a range of about 0-6 volts.

```
float AnalogChannel::GetAverageVoltage()
```

## Digital Sensors ##

### DigitalInput ###

Devices that use the [DigitalInput](WPI_Lesson3#DigitalInput.md) class return true/false values, as they are all switches of some sort.

```
bool DigitalInput::Get()
```

### Encoder ###

When you are dealing with an [encoder](WPI_Lesson3#Encoder.md), you should think of it as the trip odometer in your car. You use its input to determine if something needs to be done, and how it needs to be done. Should you do something that will only work when a certain value is hit, then you should reset the encoder.

The following functions get the distance (Pulses/set distance) recorded by the encoder, and reset it, respectively.

```
float Encoder::GetDistance()
void Encoder::Reset()
```

### Accelerometer ###

Since [accelerometers](WPI_Lesson3#Accelerometer.md) can measure acceleration on multiple axes, you need to specify the axis that you want to measure off of.

```
Accelerometer::GetAcceleration(axis)
```

#### GetAcceleration() parameters ####

  * `axis`: the `axis` object from within the `Accelerometer` class.
    * `this->kAxis_X`
    * `this->kAxis_Y`
    * `this->kAxis_Z`

#### GetAcceleration() use ####

Although it may look like an `enum` due to the way it is written, it actually is not. Therefore, the `kAxis_` object passed to the `GetAcceleration()` function needs to be a member of the object you are trying to read from.

```
//GetAcceleration example
m_accel->GetAcceleration(m_accel->kAxis_X);
```

# Activity #

Find a wiring schematic for a robot that is available right now. Program it so that you can actuate all of the motors, and print all of the sensor inputs to SmartDashboard. You may optionally print to the driver station as well, but should there be more than 6 sensors available to you, you need to find a way to display the 6 most recently changed sensors.

What is given below is for the 2013 robot.

## Wiring Schematic ##

_Note that not all of the mappings are posted at this time. I will post them when I get access to the latest version of the code._

  * Motors with sensors on them
    * Drivetrain
      * Left (`Victor`): PWM 1
      * Right (`Victor`): PWM 2
    * Climber (`Victor`): PWM 3
      * Climb ratchet (`Relay`): Relay 1
    * Feeder (`Victor`): PWM 7
      * Run at full speed on specified button press
    * Plate (2, both `Victor`): PWMs 8,10
  * Sensors
    * Drive encoders
      * Left: channels 1, 2; reversed
      * Right: channels 3, 4
    * Plate potentiometer: channel 1
    * Climber
      * Potentiometer: channel ?
      * Limit switch: channel ?
    * Feeder limit switch: channel ?
  * Control
    * Drive game controller: 1
    * Operator game controller: 2
  * Optional
    * Climb indicator light (`Relay`): Relay ?

## Notes ##

In order to use the climber, you must use the ratchet properly.

  * Extend: Pulse the ratchet forward for .125 seconds, pulse the climber backwards at a value of (-)0.3 for the same period of time, then turn the relay off, and extend the climber.
  * All other states: Pulse the ratchet backwards for .125 seconds, then turn the relay off

If you wish to use the climb indicator light, turn it on whenever the climber switch is pressed.

| ← [Chapter 2: Driving and User Input - Lesson 6: Earning Your Driver's License: Driving](WPI_Lesson6.md) | **Lesson 7** | [Lesson 8: Concepts of the PID](WPI_Lesson8.md) → |
|:-----------------------------------------------------------------------------------------------------------|:-------------|:----------------------------------------------------|