So now you know how to use the `PIDController` to create a PID control system on the robot. However, you still have a problem: what if the system you want to deploy a PID controller to uses more than one output to function properly? The `PIDController` only functions with one output.

Luckily, there is a method to use multiple outputs with one `PIDController`. This lesson will explain how you would do that.



# Subsystem Wrappers #

Writing to multiple outputs from a single PIDController is accomplished by creating something I call a _subsystem wrapper._ These are external classes that you (yes, you!) write to interface with the `PIDController`.

The way they work is by tricking the `PIDController` into thinking that it is writing into one output. Instead, it writes to the subsystem wrapper, which in turn writes to multiple outputs.

## Creation ##

Create the subsystem wrapper by making a blank `.h` file and a blank `.cpp` file, like you would for making any class. Make the name of the file the same name as the class, to avoid confusion.

When it comes to naming, make sure you are descriptive as to what exactly your wrapper is for. It should be in a `NoSpacesWithFirstCapitalLetters` format.

_Do **NOT** put an `m_` prefix in the class name! That is for individual objects._

Make sure you add `#include "WPILib.h"` to your header file, so you have access to everything.

## Inheritance ##

One concept that was not covered in the C++ tutorial was [inheritance](http://www.cplusplus.com/doc/tutorial/inheritance/#inheritance). This is a method that allows you to create new classes that extend existing classes.

In this case, you need your subsystem wrapper to inherit the class `PIDOutput` in order for the `PIDController` to accept it as an output.

```
class MySubsystemWrapper: public PIDOutput
{
//...
};
```

## Constructor ##

Because your subsystem wrapper is a separate object from your root object (the `IterativeRobot`-inheriting class), you will not be able to access all of your robot's components like you normally would. Instead, you have your class require them as parameters in your constructor.

To do this, you need to declare all of the components that you need with the subsystem as `private` members of this class. Declare them like you would declare the components in the root class (as pointers -- this is important!!).

Your constructor's parameters will be all of the required components (and ONLY the required components) as pointers. Then, in the constructor, simply set the class' components equal to the components provided through the parameters. This way, you can actually skip the initialization of these components.

Let's say that you are trying to write a PID that writes to two `Victors`.

```
//class header file
class MySubsystemWrapper: public PIDOutput
{
public:
  MySubsystemWrapper(Victor* m_motor1, Victor* m_motor2);
private:
  Victor* m_lMotor;
  Victor* m_rMotor;
};

//class implementation (.cpp) file
MySubsystemWrapper::MySubsystemWrapper(Victor* m_motor1, Victor* m_motor2)
{
  m_lMotor = m_motor1;
  m_rMotor = m_motor2;
}
```

## Writing to the Outputs ##

The `PIDOutput` class contains a (virtual) function that the `PIDController` uses to write the output of the PID. However, the one provided by the `PIDOutput` class does not do anything useful for this purpose, so we must write our own.

This function, called `PIDWrite()`, takes the following form:

```
void PIDOutput::PIDWrite(float output)
```

Since we now have the motors available to us, we can simply write a function that redirects this output at both motors.

```
//class header file
class MySubsystemWrapper: public PIDOutput
{
public:
  MySubsystemWrapper(Victor* m_motor1, Victor* m_motor2);
  void PIDWrite(float output);
private:
  Victor* m_lMotor;
  Victor* m_rMotor;
};

//this would appear below your constructor in the class implementation file
void MySubsystemWrapper::PIDWrite(float output)
{
  m_lMotor->Set(output);
  m_rMotor->Set(output);
}
```

# Interfacing with the `PIDController` #

Now that you have written the subsystem wrapper, you need to come back to the root class to put it all together.

Firstly, you need to include the class header file (so you have access to it). Keep it in the same directory as your main file, to keep things simple. Make sure that, when you include it, you use quotes and not the "`<>`" brackets when you include the file.

Secondly, declare an object of that class. Declare it and initialize it like you would any other component, but make sure that you initialize the subsystem wrapper **AFTER** you have initialized all of the components that the subsystem wrapper requires.

```
//Declaration example
MySubsystemWrapper* m_subsysWrap;

//Initialization example -- after initialization of m_lMotor and m_rMotor
m_subsysWrap = new MySubsystemWrapper(m_lMotor, m_rMotor);
```

Then, when you initialize the `PIDController` (this must be **after** the initialization of the subsystem wrapper), simply pass the subsystem wrapper to the `PIDController` as the output.

```
m_PIDController = new PIDController (m_p, m_i, m_d, m_sensor, m_subsysWrap);
```

That is all you have to do. The robot will output to both motors when it writes output from that point forward.

# Activity #

Write PID-enabled code for a subsystem on a robot that uses more than one motor.

| ← [Lesson 9: PID Part 2 - Implementing a PID](WPI_Lesson9.md)  | **Lesson 10** | [Chapter 4: Disabled and Autonomous - Lesson 11: Disabled Routines](WPI_Lesson11.md) → |
|:-----------------------------------------------------------------|:--------------|:-----------------------------------------------------------------------------------------|