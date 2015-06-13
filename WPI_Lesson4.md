Now that we have laid the foundation, it is time to build upon that foundation. This chapter will teach you how to make the robot move.

One of the important things for this, however, is to understand how the operator interface works. So, we will not actually make the robot do anything until the next lesson.



# Function Presentation #

Now that we're diving into specific functions, it is important that you understand how they are given to you in this tutorial. Whenever you are first given a function, its format will be like this:

```
RETURN_TYPE ComponentClass::FunctionName(OPTION_TYPE options)
```

where:

  * `RETURN_TYPE` is the type of variable the function returns.
  * `ComponentClass` is the class the function originates from.
  * `FunctionName` is the actual name of the function.
  * `options` are the specific options that each function takes.
    * `OPTION_TYPE` is the type of that option.

What you see printed is NOT exactly what you type in. It is shown in this format for informative purposes.

# The Game Controller #

There are really two different kinds of devices available for controlling the robot -- a game controller and a joystick. They have different programming interfaces. However, since we use the game controller, this tutorial will cover how to program for the game controller.

## Functions ##

If you recall from [the last lesson](WPI_Lesson3#Game%20Controller.md), the game controller uses the `Joystick` class (the joystick does use this class, but it is programmed in a different way), and you need to provide the joystick ID in the constructor. For the purposes of this tutorial, let's say that you declared a pointer to a `Joystick` called `m_gamepad` at ID 1.

### Buttons ###

Buttons are the simplest to program. They use the following function:

```
bool Joystick::GetRawButton(int button_index)
```

where `button_index` is the numerical index of the desired button. For the xbox controller, this is the mapping:

  * A = 1
  * B = 2
  * X = 3
  * Y = 4
  * Left Bumper = 5
  * Right Bumper = 6
  * Back = 7
  * Start = 8
  * Left Stick = 9
  * Right Stick = 10

_TIP! Check out [Defines.h](https://code.google.com/p/hot67-programming-resources/source/browse/trunk/tutorial/Defines.h) under the 'Tutorial' folder in the source code!_

**WARNING**: The left and right stick buttons are pressed when the actual joystick on the controller is pressed down. Because they are tied in with the controller, a driver can hit either button inadvertantly. Use these with extreme caution, if at all.

Obviously, this function returns true if a button is pressed.

Note that the triggers are not here. They are treated as controller joysticks (you will see why in a moment).

### Controller Joysticks ###

The joysticks on the game controller are accessed differently than you might think at first. Each joystick device is accessed in terms of its axes, and not the joystick as a whole (doing that would cause too many headaches).

To access the value on any axis, use the following function:

```
float Joystick::GetRawAxis(int axis_index)
```

where `axis_index` is the numerical index of the desired axis. This is the mapping:

  * Left stick / X axis = 1
  * Left stick / Y axis = 2
  * Triggers = 3
  * Right stick / X axis = 4
  * Right stick / Y axis = 5

_These mappings are available as header definitions in [Defines.h](https://code.google.com/p/hot67-programming-resources/source/browse/trunk/tutorial/Defines.h) as well._

#### The Joysticks ####

The joysticks, or axes 1, 2, 4, and 5, act a bit differently than you might expect. Let's look at axis 2, for example.

If you were to push the left joystick forward, using `GetRawAxis()` on axis 2 actually returns a negative value. Similarly, if you pulled it back, `GetRawAxis` returns a positive value.

However, if you took a horizontal axis, `GetRawAxis` would return a **positive** value if you were to push the joystick right, and a **negative** value if you pushed it left.

Why this is, I honestly have no clue.

Another thing of note is that each axis will return an absolute value of 1 when the joystick is pushed to its maximum extent, so if you pushed axis 2 to its furthest extent forward, it would return a value of -1.

#### The Triggers ####

You may have noticed that the triggers of the game controller appear as axis 3. This is due to their capability of being able to tell exactly how far they are pressed down.

They are on one axis because one axis is capable of representing both triggers -- the left trigger is positive, and the right trigger is negative.

**WARNING**: Because both triggers share one axis, holding both simultaneously will cause the values to cancel and return a value of 0.

Generally, if you need to treat the triggers like a button, check if the axis is past a threshold of 0.4 (0.4 on the left, -0.4 on the right). However, if necessary, you can adjust this threshold to where the driver/operator is most comfortable.

Again, the triggers will return an absolute value of 1 if one trigger is pressed completely.

# The Driver Station #

One of the most important parts of programming in general is being able to do something that has a tangible result, even if it is a string of characters on a screen. To this point, you do not know how to make any visible result when you are programming the robot. For any programmer this is a problem.

For the time being, I will teach you how to use the driver station display. We will dive deeper into using screen output devices in lesson 7.

## Printing to the Driver Station ##

If you remember from the [last lesson](WPI_Lesson3#Driver_Station_LCD.md), the driver station LCD (not the computer as a whole) is a 6-line display. Unfortunately, it is not written to intercept the `cout` stream. Instead, the driver station comes equipped with a clone of the [printf() function](CPP_TipsAndTricks#printf.md) (this is why I said it would be useful).

However, this `printf()` is different in its own way.

```
void DriverStationLCD::Printf(enum line, int column, const char* msg)
```

For starters, note that the function has a capital 'P', not a lowercase one.

### Printf parameters ###

  * `enum LINE`
    * The line of the driver station you wish to change. This takes the form `DriverStationLCD::kUser_Line#`, where # is the number of the line (1-6).
  * `int column`
    * The starting column (horizontal position) of the message.
  * `const char* msg`
    * The string that you want to write to the driver station. This takes the same format as the `cstdio printf()`. It is a `const char*` because that is the type of a string defined in quotes.

Note that a single line can hold up to 21 characters. An overflow will **NOT** wrap to the next line.

## Updating the Driver Station ##

Unlike the `printf()` you're used to, however, `Printf()` does not update the screen immediately. You have to manually update it yourself. So, once you're done writing your data, call this function:

```
DriverStationLCD::UpdateLCD()
```

Calling this function will cause the driver station to update.

# Implementing #

**This section just clarifies use of the functions we just discussed. If you feel confident, then you may [skip this section](WPI_Lesson4#Programming_the_Robot.md).**

If you're still a little confused, this section will help you understand the content better (hopefully).

## Quick Note: Periodic Routines ##

If you remember from [lesson 2](WPI_Lesson2#Periodic_Routines.md), `TeleopPeriodic()` will loop until the robot shuts down or another mode is entered. So, you must remember that, for the cases of this program, what you write is constantly running in a loop. Because it is already looping, you should write the `TeleopPeriodic()` function as if the whole thing were a loop. So, if you write something to the driver station in `TeleopPeriodic()`, it will write to the driver station every time the CPU performs another loop. _Future note: that is actually good, because then you can write real-time values to the driver station._

## Button ##

For example, let's look at a program that prints "Hello World!" to the driver station when, and only when, the A button is pressed.

What we need:

  * A `Joystick` object
  * The driver station object

```
#include "WPILib.h"

class BuiltinDefaultCode: public IterativeRobot
{
public:
  Joystick* m_gamepad;
  DriverStationLCD* m_dsLCD;
  
  BuiltinDefaultCode()
  {
    m_gamepad = new Joystick(1);
    m_dsLCD = DriverStationLCD::GetInstance();
  }
  
  //...
}
```

Now, we're ready to start programming the logic. This program will only use the `TeleopPeriodic()` routine, so leave the other routines blank.

```
  //...
  
  void TeleopPeriodic()
  {
    /** The first thing we need to do
      * is check for a button press.
      * Because GetRawButton() returns
      * a bool, we can simply put it
      * in the if statement.
      *
      * Remember, button A is index
      * 1.
      */
      
      if (m_gamepad->GetRawButton(1))
      {
	/** The user has pressed the
	  * button. Write "Hello World"
	  * to the driver station.
	  */
	  
	  m_dsLCD->Printf(DriverStationLCD::kUser_Line1,1,"Hello World!");
	  
	  //update the driver station
	  m_dsLCD->UpdateLCD();
      }
      else
      {
	/** The user has not pressed
	  * the button. In this case,
	  * we should clear the "Hello
	  * World" message. You can do
	  * this by printing an empty
	  * string to the driver station.
	  */
	  
	  m_dsLCD->Printf(DriverStationLCD::kUser_Line1,1,"");
	  
	  //update the driver station
	  m_dsLCD->UpdateLCD();
      }
  }

  //...
```

## Joystick ##

Now, we will write a program that prints a message to the driver station and tells if either trigger is pressed past a threshold of 0.4.

What we need:

  * A `Joystick` object
  * The driver station object

```
#include "WPILib.h"
#include <cmath>
/** Because we want to know if
  * *either* trigger is pressed,
  * we need the axis' absolute
  * value. That function is in
  * cmath.
  */

class BuiltinDefaultCode: public IterativeRobot
{
public:
  Joystick* m_gamepad;
  DriverStationLCD* m_dsLCD;
  
  BuiltinDefaultCode()
  {
    m_gamepad = new Joystick(1);
    m_dsLCD = DriverStationLCD::GetInstance();
  }
  
  //...
}
```

Now, let's run through the program's logic.

```
//...

  void TeleopPeriodic ()
  {
    //Easy things first...print the value of the axis to the driver station
    m_dsLCD->Printf(DriverStationLCD::kUser_Line1,1,"Left X Axis: %f",m_gamepad->GetRawAxis(1));
    
    //To reduce redundancy, we will update at the very end
    
    //Check to see if a trigger is being pressed beyond the threshold
    if (fabs(m_gamepad->GetRawAxis(3)) > 0.4)
      m_dsLCD->Printf(DriverStationLCD::kUser_Line1,1,"Trigger pressed");
      
    //Now we update.
    m_dsLCD->UpdateLCD();
  }
  
//...
```

# Programming the Robot #

Congratulations! You are now capable of making (the electronics on) the robot do something! Activities beyond this point will involve creating real robot programs. Again, one of the parts of programming is actually running the code to test its functionality.

This means actually downloading the code to the robot.

You may do so, provided that:

  * You have your mentor's approval
  * (You have likely already discussed this) You shout "ENABLING" before you enable the robot -- loud enough for everyone to hear, but not too loud so that it is the last thing they hear. You should ALWAYS do this, regardless of the fact that your program might not do anything to actuate motors, but there is always a chance that something could go badly, so you need to take every precaution.

As for actually downloading the code, this is how you do it:

  1. Build the project - Ctrl+B x2
    * As the programs you write grow more complex, you may need to clean as well (Project → Clean)
  1. Check the downloader preferences if you haven't already (you don't need to do this every time, only when you switch projects).
    1. Go to Windows → Preferences
    1. Select "FIRST Downloader Preferences"
    1. Click "Browse..."
    1. Navigate to this path in your project:
      * PPC603gnu/(ProjectName)/Debug
    1. Select this file:
      * (ProjectName).out
    1. Click "OK"
  1. Make sure you are connected to the robot -- either by tether or wifi
  1. Download the code (FIRST → Deploy)
    * ENSURE A SECURE CONNECTION THROUGHOUT THE ENTIRETY OF THIS STEP!!!!
  1. Open the FRC Driver Station
  1. Navigate to the Diagnostics tab
  1. Select "Reboot cRIO.."
  1. Hit "OK" and wait for the robot to reboot.
  1. Once communications and robot code are back up, you may enable.

Make sure that you are always downloading the right project when you download to the robot.

# Activity #

Write a program that does the following:

  * Has a counter (use a `float` for this)
    * Counts **UP** on press of the A button
    * Counts **DOWN** on press of the B button
    * Note: you will need to use a [timeout](WPI_TipsAndTricks#Timeouts.md).
  * Controls a `float` with the left joystick and triggers as follows:
    * When the left joystick is pressed **FORWARD**, it counts **UP** at a speed determined by how far the stick is pressed forward.
    * When the left joystick is pulled **BACKWARD**, it counts **DOWN** at a speed determined by how far the stick is pulled back.
    * When the left trigger is pressed, the float becomes negative
    * When the right trigger is pressed, the float becomes positive
    * Note: many of the controllers do not zero perfectly. Use a deadband of 0.2.
  * Print these numbers to the driver station.

| ← [Chapter 1: The Basics - Lesson 3: Parts of the Robot -- In Code](WPI_Lesson3.md) | **Lesson 4** | [Lesson 5: Controlling Motors](WPI_Lesson5.md) → |
|:--------------------------------------------------------------------------------------|:-------------|:---------------------------------------------------|