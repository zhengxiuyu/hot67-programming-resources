Now that you know what a PID is, you will learn how to apply it to the robot.



# Safety Note #

The most important thing with robotics in general is safety, as you well know by now. One important thing to remember is that the more automated the machine becomes, the more dangerous it can be. As a PID automates the robot to a certain degree, you should always be **VERY CAREFUL** when you are testing one for the first time (although you should always be very careful anyway).

# `PIDController` #

WPILib comes with a builtin PID controller class, named, quite creatively, `PIDController`. Declare it like you would any other robot component (as a pointer).

## Initialization ##

The `PIDController` constructor has 5 basic parameters for initialization.

```
PIDController::PIDController(float p, float i, float d, PIDSource* source, PIDOutput* output)
```

  * `float p`: The value of P, or the proportional constant.
  * `float i`: The value of I, or the integral constant.
  * `float d`: The value of D, or the derivative constant.
  * `PIDSource* source`: The sensor input to be used with the PID controller. All sensors inherit the `PIDSource` class.
  * `PIDOutput* output`: The motor controller output to be used with the PID controller. All motor controllers inherit the `PIDOutput` class.

## Use ##

Firstly, the `PIDController` has an on/off switch, which is off to start. In order to use it, it must first be enabled.

```
void PIDController::Enable()
```

Similarly, it can be disabled:

```
void PIDController::Disable()
```

Once enabled, all you need to do is set the setpoint. As soon as the robot reaches that point in the code, it will automatically move the output such that the input matches the setpoint.

```
void PIDController::SetSetpoint(float setpoint)
```

You can also check what the current setpoint is, as well.

```
float PIDController::GetSetpoint()
```

# Tuning a PID #

Writing a PID is one thing. Making it work is another thing.

Tuning a PID can be dangerous on any number of levels, as you have no idea of how the robot will respond. When you are tuning a PID, you should **ALWAYS** have your finger on the Enter key so that, when things go wrong, you can stop the robot before it does any serious damage.

## PID Setpoints ##

You may remember <a href='WPI_Lesson7#A_Special_Function'>the <code>PIDGet()</code> function from lesson 7</a>. Funny how it begins with P-I-D, huh? That's because this is the function that the `PIDController` reads from when it runs the calculations. This means that all of your setpoints need to use the bitwise value found by `PIDGet()`

To get these values, simply write the output of the `PIDGet()` function for whatever sensor you are using to the driver station, and then use those values for the setpoints.

## Prerequisites ##

_Before_ you start tuning a PID, make sure your code does the following:

  * Outputs the `PIDGet()` value of the sensor you are using to the driver station.
  * Reads from the right input and writes to the right output, and that those objects are mapped correctly.
  * Sets the setpoint _only_ after a button press.

Although not required, it would not be a bad idea to also put a soft killswitch in for the PID (disables the PID on a button press), which may be useful if others are helping you tune the PID. If you do this, do _not_ make the button a toggleswitch.

You should start with the following PID constants:

  * P: 1
  * I: 0
  * D: 0

## Finding the right values ##

When you are tuning the PID, you need to ask yourself the following questions. Below each question is a potential solution for a problem that you might see.

  * Is it moving in the right direction?
    * No: Make the P constant the opposite of what it is now (negative if it is positive, vice versa).
  * Is it being too aggressive/shy in its movements?
    * Aggressive: Lower the P constant.
    * Shy: Raise the P constant.
  * Is it overshooting and not correcting itself?
    * Solution 1: Try lowering the P constant.
    * Solution 2
      * Overshooting (try first): Adjust the D constant. Be careful if you are going to do this, and adjust D by values of 0.1 or even 0.01.
      * Steady-state error: Adjust the I constant. Take similar cautions with I that you would with D.

Knowing how much to change a P, I, or D value is a learned skill and does not really follow a definite algorithm. As suchm it cannot be taught through a tutorial. If you need help, ask someone who has tuned a PID before.

# Activity #

The content in this lesson could be tested, but most of the PID-capable subsystems on available robots require that you know the content from lesson 10.

If you do have access to a robot with a system that is ready to run a PID that only uses one sensor and one motor, then write code to implement a PID on that system. Otherwise, continue to the next lesson.

| ← [Lesson 8: PID Part 1 - PID Theory](WPI_Lesson8.md)  | **Lesson 9** | [Lesson 10: PID Part 3 - Advanced PID Programming](WPI_Lesson10.md) → |
|:---------------------------------------------------------|:-------------|:------------------------------------------------------------------------|