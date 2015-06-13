By this point, you have gone over 1) what parts of the robot are available, and 2) the structure of the program. Now, we will look at how we the parts of the robot are declared in the code.

Note that, if you are using RobotBuilder, all of this is done automatically.



# Naming and Declarative Style #

One of the most important things that you will need to learn is how to write code -- not in the sense of what you need to do, but in the literal sense. How you write your code really shows your skill as a programmer. If your coding style is atrocious, then no one (in some cases, not even you) will be able to understand your code. In terms of improvement, this is **bad**. If no one can help you fix bugs, etc., then you are essentially stuck.

One of the most important things that a programmer has is his/her peers. Quite often, beating out a bug can take multiple people, as the original programmer might not be able to see the bug. This is why it is so important that your code be legible.

In terms of general coding style, stick with a general C++ style. That is, keeping functions that take multiple statements on multiple lines, denoting different sections of your program, and so on.

## Naming ##

This is likely the most important piece of style. Every variable you declare does something (if it doesn't, get rid of it), but others need to know what it does in order for them to be able to help you with your program (single-letter names for looping variables, such as `x`, `y`, `z`, or `i` are fine, [but you will \*never\* use loops in robotics programming](WPI_Lesson2#Periodic_Routines.md)). Thus, we follow a specific naming scheme:

```
TypeIdentifier m_DescriptiveLabel;
```

The `m_` prefix stands for "member". It is a coding practice that has been in place for a long time (likely before FIRST came into existence), and while it isn't quite necessary, it is usually still included for clarity purposes.

The `DescriptiveLabel` part should be a **descriptive label**. You should be able to tell what the variable does without having to look at the code that it is used in. Do not use any irrelevant or confusing labels. For example:

**GOOD**
```
int m_TeleopLoopCounter;
```

From this declaration, I can already tell that you plan to use this variable to count CPU loops in the TeleopPeriodic routine.

**BAD**
```
int m_turtles;
```

The only conclusion I can draw from this declaration is that you are counting turtles. Such a feature would be useless on a competitive robot.

_RobotBuilder Note: **ONLY** if you are using RobotBuilder, take off the `m_` prefix. RobotBuilder tends to run into issues when the names get too complex._

## Declaring Members ##

How you declare your class members depends on what type the member is.

### Generic Variables ###

The most commonly used variables in robotics programming are `int`s, `float`s, and `double`s. Declare them like you normally would.

```
int m_TeleopLoopCounter;
```

### Robot Components ###

When you are declaring actual robot components, use a pointer. This is generally to manage memory efficiently. Each component is held as [dynamic memory](http://www.cplusplus.com/doc/tutorial/dynamic/), so you need to create `new` instances of each component (this is generally what the class constructor is for). I'll discuss exactly how to use the `new` keyword as we look at the individual components.

```
class BuiltinDefaultCode: public IterativeRobot
{
  public:
    Victor *m_lDrive;

    BuiltinDefaultCode()
    {
      m_lDrive = new Victor(1);
    }
};
```

_RobotBuilder Note: If you are using RobotBuilder, this is done automatically. You should still know how to do this, however._

# Robot Parts #

Only parts listed in the [Programmed Components](WPI_Lesson1#Programmed_Components.md) section of Lesson 1 can be declared as components in the program. Even then, the [cRIO](WPI_Lesson1#cRIO.md), [Digital Sidecar](WPI_Lesson1#Digital_Sidecar.md), [Analog Breakout box](WPI_Lesson1#Analog_Breakout.md), and [motors](WPI_Lesson1#Motors.md) are not declared.

Note that you do not need to declare all of the parts on the robot for your code to work. Of course, you will only be able to control what is declared. Also note that WindRiver cannot possibly detect the correct address (e.g. PWM or relay channel) of various components, so make sure that your assignment is correct.

Most parts are fairly straightforward. The tricky parts come in with the address assignment. Your electrical team should be able to tell you what is mounted where, but you should keep a wiring table and know how to find the relevant port, if necessary (see [WPI\_Lesson1#Digital\_Sidecar](WPI_Lesson1#Digital_Sidecar.md) or [WPI\_Lesson1#Analog\_Breakout](WPI_Lesson1#Analog_Breakout.md)).

Note that the location of each statement is not shown in the declaration example. The actual declaration belongs in the class list, and the initialization belongs in the class constructor.

NOTE: If you declare standard variables, such as `int`s or `bool`s, **you need to set a default value in the constructor**, like so:

```
//..
  BuiltinDefaultCode()
  {
    //...
    
    m_myInt = 0;
    m_myBool = false;
  }
//...
```

## Motor Controllers ##

### RobotDrive ###
The class `RobotDrive` is not actually a part on the robot, but it is an object that handles the drivetrain motors. A drivetrain can either consist of 2 or 4 drive controllers. On this team, we use PWM splitter cables such that there are only 2 PWM channels being used, but there are actually 4 drive controllers running the robot. Treat it like a 2-controller drivetrain.

In the initialization, provide all of the drive controllers that are part of the drivetrain. Provide the left controller(s) first, then the right controller(s). Ensure that you initialize the `RobotDrive` **AFTER** you have initialized the individual drivetrain controllers.

Also, you should disable the safety (watchdog). The watchdog is run by the drivetrain in order to detect errors. Using the watchdog is very complex, and, if used incorrectly, can cause serious operation errors. It's much simpler to disable it.

```
//RobotDrive declaration
RobotDrive *m_robotDrive;
/** You will see how to declare drive controllers momentarily
  * Let's assume we've declared two Victors, m_lDrive and m_rDrive
  */

//RobotDrive initialization
m_robotDrive = new RobotDrive (m_lDrive, m_rDrive);

m_robotDrive->SetSafetyEnabled(false);
```

### Victor ###
A Victor (which you have seen declared before) is of type `Victor` (with a capital V). It encompasses both Victor motor controllers and VEX motor controllers.

In the initialization, provide the PWM channel on the Digital Sidecar that the controller is hooked up to.

```
//Victor declaration
Victor *m_motor1;

//Victor initialization on PWM channel 1
m_motor1 = new Victor (1);
```

### Jaguar ###
A Jaguar is of type `Jaguar` (with a capital J). In the initialization, provide the PWM channel that the controller is hooked up to.

```
///Jaguar declaration
Jaguar *m_motor2;

//Jaguar initialization on PWM channel 2
m_motor2 = new Jaguar (2);
```

### Talon ###
A Talon is of type `Talon` (with a capital T). In the initialization, provide the controller's PWM channel.

```
//Talon declaration
Talon *m_motor3;

//Talon initialization on PWM channel 3
m_motor3 = new Talon (3);
```

### Servo ###
This category includes generic servos but not VEX motors (see [Victors](WPI_Lesson3#Victor.md)), as VEX motors are managed differently.

A servo is of type `Servo` (with a capital S). In the initialization, provide the servo's PWM channel.

```
//Servo declaration
Servo *m_servo1;

//Servo initialization on PWM channel 4
m_servo1 = new Servo (4);
```

### Spike ###
Other than window motors, Spikes also drive generic lights on the robot (this does not include the notification light).

A Spike is of type `Relay` (with a capital R). In the initialization, provide the relay channel the Spike is connected to.

```
//Spike declaration
Relay *m_relay1;

//Spike initialization on relay 1
m_relay1 = new Relay (1);
```

### Digital Outputs ###
In some cases, you have a controller/motor that is digital yet does not fall into this category. As long as it uses the [digital output](WPI_Lesson1#Digital_Sidecar.md) on the Digital Sidecar, use this class.

A digital output is of type `DigitalOutput` (with a capital D and O, no space). In the initialization, provide the digital output channel on the Digital Sidecar.

```
//Digital Output declaration
DigitalOutput *m_do;

//Digital Output initialization on digital output 1
m_do = new DigitalOutput (1);
```

## Sensors ##
Some common sensors use a generic class for their type. Below, if a generic class is given, the sensors that use that class are provided in a list.

### AnalogChannel ###
The class `AnalogChannel` (capital A,C; no space) manages generic analog devices. These devices manipulate voltage as output.

In the initialization, provide the analog channel of the device.

```
//AnalogChannel declaration
AnalogChannel *m_analogDev;

//AnalogChannel initialization on analog channel 1
m_analogDev = new AnalogChannel (1);
```

#### AnalogChannel Devices ####
The most common device used in this category is a [potentiometer](WPI_Lesson1#Potentiometer.md). If you have a general analog device, use this class (unless otherwise specified).

### DigitalInput ###
The class `DigitalInput` (capital D, I; no space) manages generic digital sensors.

In the initialization, provide the digital input channel on the Digital Sidecar.

```
//DigitalInput declaration
DigitalInput *m_digitalDev;

//DigitalInput initialization on digital input 1
m_digitalDev = new DigitalInput (1);
```

#### DigitalInput Devices ####
The most common device used in this category is a [limit switch](WPI_Lesson1#Limit_Switch.md). If you have a general digital device, use this class (unless otherwise specified).

### Encoder ###
An encoder is of type `Encoder` (capital E). In the initialization, do the following:

  * provide the device's two digital channels
  * provide whether the device should measure in reverse (true/false)
  * configure
    * distance measured per pulse
    * max period
  * start it

```
//Encoder declaration
Encoder *m_lEncode;

//Encoder initialization on digital inputs 2 and 3, not reversed, using a distance of 1 per pulse and a max period of 1
m_lEncode = new Encoder (2,3,false);
m_lEncode->SetDistancePerPulse(1);
m_lEncode->SetMaxPeriod(1.0);
m_lEncode->Start();
```

### Gear Tooth Sensor ###
A gear tooth sensor is of type `GearTooth` (capital G, T). In the initialization, provide the digital input channel and whether you want to have the sensor keep track of the direction of rotation or not, using a true/false value.

```
//Gear Tooth sensor declaration
GearTooth *m_gt;

//Gear Tooth sensor initialization on digital input 4
m_gt = new GearTooth (4);
```

### Ultrasonic Sensor ###
An ultrasonic sensor is of type `Ultrasonic` (capital U). In the initialization, provide the digital output channel of the sound emitter and the digital input of the echo detector.

```
//Ultrasonic sensor declaration
Ultrasonic *m_proximityDetect;

//Ultrasonic sensor initialization on digital output 2 and digital input 5
m_proximityDetect = new Ultrasonic (2,5);
```

### Gyroscope ###
A gyroscope is of class `Gyro` (capital G). In the initialization, provide the analog channel of the gyroscope, and set its sensitivity.

```
//Gyroscope declaration
Gyro *m_gyro;

//Gyroscope initialization on analog channel 2 with sensitivity of 2.00
m_gyro = new Gyro (2);
m_gyro->SetSensitivity(2.00);
```

### Accelerometer ###
An accelerometer is of type `Accelerometer` (capital A). In the initialization, do the following:

  * Provide the analog channel of the device
  * Set the voltage change per 1G (Earth gravity) change
  * Set the voltage output at 0G (no gravity)

```
//Accelerometer declaration
Accelerometer *m_accel;

//Accelerometer initialization on analog channel 3, voltage change of 2, and zero-G voltage of 1
m_accel = new Accelerometer (3);
m_accel->SetSensitivity(2.0);
m_accel->SetZero(1.0);
```

## Driver Station Components ##
There are a few components on the driver's side that you need to declare.

### Game Controller ###
A standard XBOX 360 controller is of type `Joystick` (capital J). In the initialization, provide the Joystick ID. Use the following rule:

  * Joystick 1: Driver
  * Joystick 2: Operator
    * The operator performs actions that manipulate various components of the robot (like an arm)

The actual ID of the joystick depends on when it was plugged in (first plugged in - 1, second - 2, and so on). There are a total of 4 available slots, but you will only ever use 2.

```
//Game controller declaration
Joystick *m_driver;
Joystick *m_operator;

/** Note: Use more general names for the two joysticks (such as Joystick1 and Joystick2) 
  * to keep things simple, copy 'n'paste-friendly, and easy to remember. 'm_driver' and 
  * 'm_operator' are only used here for clarity purposes.
  */

//Game controller initialization
m_driver = new Joystick (1);
m_operator = new Joystick (2);
```

### Driver Station ###
Believe it or not, the driver station (computer) has a few components that need to be declared. These are outputs and inputs on the computer screen that are useful for runtime information and debugging.

#### Driver Station LCD ####
In the FRC Driver Station software, there is a 6-line box titled 'User Messages'. This object is available for printing in C++.

It is of type `DriverStationLCD`. Its initialization is special, because it does not use the `new` keyword, as the Driver Station is already running (hence it is not new). Instead, you pass it a pointer to the currently running `DriverStationLCD` object.

```
//Driver Station LCD declaration
DriverStationLCD *m_dsLCD;

//Driver Station LCD initialization
m_dsLCD = DriverStationLCD::GetInstance();
```

#### Live Window ####
Smart Dashboard provides an option to manipulate robot components in real time. This feature is known as Live Window, and is what you program in the `Test` routines in your program. You manage Live Window with the `LiveWindow` object.

It also is running (in theory) when the robot connects to the driver station, so the `new` keyword will not work. It uses the same method that `DriverStationLCD` uses in initialization.

```
//Live Window declaration
LiveWindow *m_lw;

//Live Window initialization
m_lw = LiveWindow::GetInstance();
```

## Miscellaneous ##
#### Timer ####
A timer, is, well, a timer. This is useful for creating timed actions.

It is of type `Timer`. It has no special initialization.

```
//Timer declaration
Timer *m_timer;

//Timer initialization
m_timer = new Timer;

/** Notice how no parentheses were added after the 'Timer'
  * statement in the initialization. It's not necessary, so
  * just leave it out.
  */
```

# Activity #

Working alone or with a partner, declare and initialize everything in the activity from [lesson 2](WPI_Lesson2#Activity.md). Assume the following:

  * The `RobotDrive` object is a 2-controller drivetrain
  * The climber is **NOT** on a `RobotDrive` object
  * Motors
    * Drivetrain: Victors (2)
      * PWM: 1 and 2
    * Climber: Victor (1)
      * PWM: 3
    * Plate: Victors (2)
      * PWM: 8 and 10
    * Launcher: Victors (3)
      * PWM: 4,5,6
      * The launchers need to spin up, so use a Timer object to progressively add speed
    * Feeder: Victor
      * PWM: 7
    * **NEW** Climber Ratchet: Spike
      * Relay 1
  * Sensors
    * Left/Right encoders
      * Left: DI 1,2; reversed
      * Right: DI 3,4
      * Distance per pulse: 1
      * Max period: 1
    * Potentiometers
      * Plate/shot height
        * Channel 1
      * Climber
        * Channel 2
  * Driver Station
    * Two controllers
    * Driver Station LCD output

**STOP SCROLLING UNTIL YOU ARE READY TO CHECK YOUR ANSWER**

| ← [Lesson 2: Program Structure](WPI_Lesson2.md) | **Lesson 3** | [Chapter 2: Driving and User Input - Lesson 4: Using User Input](WPI_Lesson4.md) → |
|:--------------------------------------------------|:-------------|:-------------------------------------------------------------------------------------|