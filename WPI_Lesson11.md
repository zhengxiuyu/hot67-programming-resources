Your work on robot programming thus far has only taken place within the `TeleopPeriodic()` routine. If you remember from [lesson 2](WPI_Lesson2.md), there were more routines than that -- namely Disabled and Autonomous. This chapter will teach you what these routines are for, and how you would program them.

Syntax-wise, programming in all of the new functions that will be presented throughout this chapter is the same as it would be in `TeleopPeriodic()`. However, there are restrictions on what can be done in the various routines, so you should be aware of what you are programming the robot to do and you are not telling it to break any of the restrictions.

However, in addition to the routines mentioned above, we have not discussed in great detail what initialization routines are.

_Note: By_ initialization, _I mean the routines that have `Init` appended to their function names, not the class constructor._



# `RobotInit()` #

This routine is the second function that is run on robot power-up (barring boot sequences, after the class constructor). This routine is used to perform any other initialization that needs to be performed before the robot moves into a periodic routine that was not performed within the class constructor. What goes here is generally specific to the robot being programmed.

# Preparing for a Periodic Routine #

Sometimes it is necessary to put the robot in a certain state before launching a periodic routine. Instead of putting in a loop counter and checking if it is the first loop, we instead use the initialization functions provided in the `IterativeRobot()` class. The cRIO always runs these functions the CPU loop before entering the associated mode.

Each initialization function is named similarly to its periodic relative, giving the name of the associated mode before `Init`.

Note that, for each of these functions, (`DisabledInit`, `AutonomousInit`, and `TeleopInit`), all of the restrictions for the associated mode still apply (those for Disabled and Autonomous you will learn later). Additionally, it is usually a good idea not to actuate motors during initialization routines.

# Activity #

There is nothing that can be readily performed in an activity at this point. If you want, you can play around with initialization routines (do so safely and according to mode limitations!), otherwise, continue to the next lesson.


| ← [Chapter 3: Control - Lesson 10: PID Part 3 - Advanced PID Programming](WPI_Lesson10.md)  | **Lesson 11** | [Lesson 12: Disabled Routines](WPI_Lesson12.md) → |
|:----------------------------------------------------------------------------------------------|:--------------|:----------------------------------------------------|