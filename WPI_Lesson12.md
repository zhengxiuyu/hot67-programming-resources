This lesson is all about the Disabled routines. Disabled mode runs when the robot is not enabled (when the 'disabled' button is highlighted on the driver station).



# Restrictions #

Disabled mode is the most restricted mode. In Disabled, robots **cannot** actuate anything on the robot. This means that victors, jaguars, talons, and even spikes cannot be changed in this mode (even if you have a spike attached to a light and not a motor).

However, you can read from sensors, read from the controller inputs, and print information to the driver station or SmartDashboard.

# Programming in Disabled #

Remember that, when you are programming in Disabled, you are still programming in a periodic routine (outside of initialization). As such, you should take all considerations regarding CPU looping into account when you program in Disabled.

# Uses #

Disabled is largely used to perform setup on the robot (choose autonomous strategy and adjusting an offset for PID subsystems), but can be used for any other setup related task (self-checks, etc) before the start of a match.

If you are using Disabled to set up the robot for Autonomous, it should generally do the following:

  * Look for user input
  * Show the current autonomous strategy
  * Show the current PID subsystem offset(s), if any, compared to the default value
  * Emphasize anything that the drivers might see as a problem that your code has detected (dangerous/game-risking autonomous case selected, strange input values, etc.)

You should be aware that, if your code is deployed to run on the robot when the drivers are driving it, it is possible they may hit a button on the controller without knowing, causing the robot to perform an unwanted autonomous strategy.

# Activity #

Redo [the activity from lesson 4](WPI_Lesson4#Activity.md) in Disabled mode.

Additionally, have the robot show you all of its sensor values in Disabled mode.

| ← [Lesson 11: Initialization Routines](WPI_Lesson11.md)  | **Lesson 12** | [Lesson 13: Autonomous Routines](WPI_Lesson13.md) → |
|:-----------------------------------------------------------|:--------------|:------------------------------------------------------|