In review, you have studied the various components of the robot, how they appear in code, and how to use them. The next three lessons will teach you one of the most important aspects of WPILib and robot programming in general -- the PID.

The PID is split up into three parts here because of its complexity. It is EXTREMELY important (enough that it will distinguish you as an accomplished robot programmer) that you know how to create a PID in your program - so pay close attention.

Robot programming is not the only thing that uses the PID -- it is used heavily in the automotive industry (think cruise control).

_Fun fact:_ PID theory was developed by watching helmsmen steer a ship.



# What is a PID? #

To put it simply: a PID is a method to control various motor outputs using sensor inputs in an intelligent manner.

A PID is implemented with a process or object known as the PID controller. It handles all of the necessary calculations in order to use a PID.

The PID runs based on something called a _setpoint_. The setpoint is where the PID controller is told to keep the sensor input value as constant as possible using the motor outputs it is given. For example, the cruise control in a car is told to keep the speed at a specific value (the setpoint), so it adjusts the output of the car's engine to keep the speed constant.

The name 'PID' comes from constants that are given to the PID controller that dictate the manner in which it adjusts the output of its motors:

  * **P** roportional
  * **I** ntegral
  * **D** erivative

In the descriptions of the constants below, the graphs effectively show a reading from a sensor over time, with the red line being the setpoint. The other lines are examples showing the reading over time given different constant values.

Just so you know: the PID is much more complex than what is explained here. Both the I and the D terms use calculus functions -- something very few reading this would know. If you want a more thorough explanation of PID theory, go [here](http://en.wikipedia.org/wiki/PID_controller#PID_controller_theory).

## Proportional ##

The proportional constant tells the PID controller to send an output **in proportion** to the error from the setpoint -- in other words, it multiplies the error from the setpoint by the proportional constant. Essentially, proportional control will provide more power to the output when presented with greater error.

Pure proportional control has a few problems. Firstly, if the error is significant, it tends to overshoot the setpoint, then moving back towards it. Secondly, if the proportional term is large enough, the system will not be able to respond as quickly to the changes in the desired output, and as such will lose stability. Finally, proportional control usually does not settle at the setpoint, but rather near it -- this is known as _steady-state error_. This occurs because the signal from the proportional control becomes neglectable, and the output does not change.

<img src='http://newton.ex.ac.uk/teaching/cdhw/Feedback/Figs/ProportionalGraph.gif' title='Example of proportional control, showing increasing gain' />

## Derivative ##

Derivative control fixes two of the three problems with proportional control: overshoot and instability.

Derivative control studies how the error changes over time and lessens the rate of change in the output if the sensor readings are approaching the setpoint too quickly, allowing the output to reach the setpoint more gracefully.

A high derivative constant will dampen the output considerably, drastically increasing the settling time.

Note that derivative control is sensitive to sensor noise (slight changes in the value that are associated with a lack of precision and not movement), and may take unneccessary dampening action on the output.

<img src='http://newton.ex.ac.uk/teaching/cdhw/Feedback/Figs/PDGraph.gif' title='Example of proportional and derivative control, showing increased dampening' />

## Integral ##

While derivative control eliminates overshoot and instability in the proportional control, it still does not eliminate the steady-state error, as the error is not changing, which satisfies derivative control. To fix this, integral control is used.

The integral constant looks at the average error in the system. As such, small, steady error (i.e. steady-state error) gradually accumulates over time and becomes more prominent to the integral controller.

Integral control adjusts the output such that the average error eventually reaches 0.

<img src='http://newton.ex.ac.uk/teaching/cdhw/Feedback/Figs/PIDGraph.gif' title='Example of PID Control' />

# Review #

Effectively, what you should remember about each of these terms is:

  * **P**: Proportionally adjusts the output of the system based on the error to make the sensor reading closer to the setpoint.
    * Key term: **P** ower
  * **I**: Adjusts the output of the system based on the average error.
    * Key term: (Fixing) Offset
  * **D**: Adjusts the output of the system based on how the error is currently changing.
    * Key term: **D** ampening

# Activity #

Download the [PID Simulation](https://hot67-programming-resources.googlecode.com/svn/trunk/tutorial/PIDSimulation.xls) and open it. Play with the P, I and D constants and see how the output reacts to different values (the output is the red line, the setpoint is the blue line).

Try to accomplish the following:

  * Minimize settling time (the time it takes to reach the setpoint)
  * Minimize overshoot
  * Minimize offset/steady-state error

_Simulation source: Carnegie Mellon University ([original file](http://www.cs.cmu.edu/afs/cs/academic/class/15494-s11/lectures/manipulation_with_friction_files/PID%20Simulation.xls))_

_Image credit: University of Exeter_

| ← [Lesson 7: Sensors and Information](WPI_Lesson7.md) | **Lesson 8** | [Lesson 9: PID Part 2 - Implementing a PID ](WPI_Lesson9.md) → |
|:--------------------------------------------------------|:-------------|:-----------------------------------------------------------------|