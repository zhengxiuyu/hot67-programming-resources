Now that we have reviewed what makes up the robot, we can move on to what makes up the program.

If you still don't understand classes, you NEED to study up on that before you continue.
[Part 1](http://www.cplusplus.com/doc/tutorial/classes/)
[Part 2](http://www.cplusplus.com/doc/tutorial/classes2/)



# Introduction #
The robot's program is a little different from what you are used to. Instead of using an `int main` and a lot of functions, you program the robot as a class. And only a class. The `int main` portion of the program doesn't change, so it is already embedded into the cRIO for you.

The underlying program on the cRIO reads the class that you wrote, and applies it to the robot. So, essentially, you are treating the robot as if it were a class.

# Getting Started #
WindRiver already has the robot class template built into itself. In order to get it, go to File > New > Example and then select VxWorks 6.3 Downloadable Kernel Sample Project > BuiltinDefaultCode (current with imaging tool). This will load all of the required files into your workspace (there are some other underlying files that you don't touch). The file you want is "BuiltinDefaultCode.cpp". Let's examine what it's done for us:

```
#include "WPILib.h"
```

This includes the library which, in turn, includes all of the functions of the robot you could ever wish.

```
class BuiltinDefaultCode: public IterativeRobot
```

This creates a new class called `BuiltinDefaultCode` that you will program. It inherits the `IterativeRobot` class, the class the cRIO treats the robot as. Your class **MUST** inherit `IterativeRobot`, otherwise it will not work.

Now, we will look at the structure of the robot class.

# Class Members #
```
class BuiltinDefaultCode: public IterativeRobot
{
  /*Put the components of your robot here.
   *...
   */
```

Here is where you declare everything that your robot has: drive controllers, sensors, etc. You also declare variables that you will use for logic here, as well.

There is a specific way in which we declare robot components -- that is, using pointers. But we'll get into that later.

# Class Constructor #
```
BuiltinDefaultCode::BuiltinDefaultCode()
{
  //Initialize robot components here
}
```

The class constructor is used to initialize parts of the robot and to set variables for their initial run. This is called before the [RobotInit()](WPI_Lesson2#RobotInit.md) method in the program. An example usage of the constructor:

```
BultinDefaultCode::BuiltinDefaultCode()
{
  m_Victor1 = new Victor(1);
  m_Counter = 0;
}
```

Again, how to initialize specific robot components will be reviewed in [Lesson 3](WPI_Lesson3.md).

You may notice the `public:` keyword before the class constructor. Everything declared before is private, to avoid memory overflow.

# Initialization Routines #
_Not to be confused with the component initialization in the [constructor](WPI_Lesson2#Class%20Constructor.md)_

The following routines are called once before their corresponding [periodic routines](WPI_Lesson2#Periodic%20Routines.md) are called. Note that [RobotInit](WPI_Lesson2#RobotInit.md) does not have a periodic routine.

One important concept to note at this point is that the cRIO operates in loops. It runs an init routine in one loop, then it switches to the corresponding periodic routine (which runs once every loop until the robot is disabled).

## RobotInit ##
```
void BuiltinDefaultCode::RobotInit()
{
  // Actions which would be performed once (and only once) upon initialization of the
  // robot would be put here.
}
```

The comment says it all -- everything that is not component and variable initialization that you want to do before the robot does anything else (e.g. sending things to the dashboard, if necessary).

## DisabledInit ##
```
void BuiltinDefaultCode::DisabledInit()
{
  // Actions that are to be called as the robot enters disabled mode would be put here
}
```

Disabled routines, or when 'Disabled' is selected in the driver station, are routines in which you can change variables, but you cannot change the state of the robot. The most common use for this mode is to change between autonomous modes. In the `DisabledInit` portion of the program, you would put statements that you want run as the robot enters disabled mode.

## AutonomousInit ##
```
void BuiltinDefaultCode::AutonomousInit()
{
  // Put actions that should be run as the autonomous period of the match starts here.
}
```

Autonomous is the part of the match where the robot functions independently of its human counterparts (with the slight exception of the Kinect, which no one uses). If there are some actions that need to be run as you enter autonomous, put them here.

## TeleopInit ##
```
void BuiltinDefaultCode::TeleopInit()
{
  // Put actions that should be run once teleoperated operation starts here.
}
```

Teleop, short for teleoperated, is when the drivers control the robot through the controls. This initalization routine is important, as it is often used to stop any leftover autonomous commands, should they run past their allocated period. It can also be used to reset the robot to a certain state after autonomous ends.

## TestInit ##
```
void BuiltinDefaultCode::TestInit()
{
  // Put setup for diagnostics here
}
```

Test mode is an important mode: you can use it to test the funtionality of your robot. Essentially, it is a diagnostic mode that you can use to determine exactly what is wrong with your robot.

In TestInit, you would set your robot up for diagnostics, if any are needed.

# Periodic Routines #
These are all routines that are run once the cRIO has finished the initialization stage for that type of operation, e.g. teleop. These continue to run, once per loop until the robot is disabled (in matches, the robot is disabled for a short period of time after autonomous to switch to teleop). Because they loop automatically, you should **never** write a loop into these routines.

## DisabledPeriodic ##
```
void BuiltinDefaultCode::DisabledPeriodic()
{
  //Put methods to be run while your robot is disabled here
}
```

Here is where you would program your robot to change autonomous modes, tune in a shot height, etc. Again, you cannot interact with drive controllers, etc. here.

## AutonomousPeriodic ##
```
void BuiltinDefaultCode::AutonomousPeriodic()
{
  //Put methods that make up your autonomous routine here.
}
```

Autonomous Periodic is what runs during the autonomous portion of the match. Here, you hard-code the robot's autonomous routine.

Note that, depending on the complexity of your autonomous, this can be the part of your program that requires the most logic.

## TeleopPeriodic ##
```
void BuiltinDefaultCode::TeleopPeriodic()
{
  //Put methods that would be run during teleop here.
}
```

Here is where you put in all of your Teleop functionality, which includes but is not limited to:

  * Driving
  * Controlling devices (such as shooters and arms) and PIDs
  * Updating the driver station and dashboard

Because this is easily one of the largest parts of the program (although there are a few exceptions), it is better to organize this, such that all of your Teleop functionality is split up amongst organized functions. For example, if you wanted to drive your robot and update the driver station, your TeleopPeriodic would look like this:

```
void BuiltinDefaultCode::TeleopPeriodic()
{
  TeleopDrive();
  TeleopDriverStationUpdate();
}

void BuiltinDefaultCode::TeleopDrive()
{
  //Methods for driving the robot here
}

void BuiltinDefaultCode::TeleopDriverStationUpdate()
{
  //Update the driver station here
}
```

**Note:** This will become even more apparent as the tutorial progresses, but **PLEASE** name your functions and variables with relevant names. Not doing so will cause a lot of ripped-out hair and shouting voices.

## TestPeriodic ##
```
void BuiltinDefaultCode::TestPeriodic()
{
  // Put diagnostic routines here.
}
```

Here, you would put all of your diagnostics. In the case of this tutorial, you would use game controller buttons to test individual motors.

Diagnostics are important because they help you determine the exact problem with your robot, if there is any. You should always program a diagnostic mode.

# Continuous Routines #
Continuous routines are similar to periodic routines, in that they are called over and over again. However, they can run multiple times per loop, and they take higher priority over periodic routines. Because of this, using these too much can lead to a runaway process thread and memory issues. Use these with caution, if at all.

## DisabledContinuous ##
```
void BuiltinDefaultCode::DisabledContinuous()
{
  //...
}
```

## AutonomousContinuous ##
```
void BuiltinDefaultCode::AutonomousContinuous()
{
  //...
}
```

## TeleopContinuous ##
```
void BuiltinDefaultCode::TeleopContinuous()
{
  //...
}
```

## TestContinuous ##
```
void BuiltinDefaultCode::TestContinuous()
{
  //...
}
```

# End of the Program #
At the end of the program, you may notice the following line:

```
START_ROBOT_CLASS(BuiltinDefaultCode);
```

All this line does is export the class as an object to the cRIO for use.

# Activity #
_The following activity uses the 2013 game and bot as reference_

Alone or with a partner, organize these operations into their respective program routine. Using the template provided by WindRiver, use comments to put each operation into its proper place. You will also need to declare your components (use comments for this as well). You do not need to use a computer for this (although you can, if you want).

Your team is determining exactly what you want your robot to do during a match. They come up with the following:

  * While disabled:
    * Adjust shot height
    * Change between two autonomous modes
  * In autonomous
    * Go to selected autonomous mode
      * One drives to the center, one to the side
    * Set shot height to requested offset from the default
    * Spin up launcher
    * Fire 3 frisbees into the top goal
    * Back up to center line
  * In teleop
    * Drive
    * Turn on/off launcher, adjust shot speed if necessary
    * Fire frisbees using a feeder
    * Use the climber, with lock safeguard
    * Update the driver station and dashboard
  * It must also have a diagnostic mode

| ← [Lesson 1: Parts of the Robot](WPI_Lesson1.md) | **Lesson 2** | [Lesson 3: Parts of the Robot -- In Code](WPI_Lesson3.md) → |
|:---------------------------------------------------|:-------------|:--------------------------------------------------------------|