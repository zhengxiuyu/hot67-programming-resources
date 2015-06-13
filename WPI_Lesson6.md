Up to this point, you have learned the various components of the robot, how they are represented by software, and how to use some of those components. Now, you will learn how to program one of the essential functions of any robot: driving.



# `RobotDrive` #

You may remember the [RobotDrive](WPI_Lesson6#RobotDrive.md) object from lesson 3. This object provides various functions to make managing the drive system simpler. There are two functions that you will see most often: `ArcadeDrive()` and `TankDrive()`.

For all of the following examples, use this set of declarations:
```
//Declarations
Victor* m_lDrive;
Victor* m_rDrive;

RobotDrive* m_robotDrive;

Timer* m_timer;

//Initialization
m_lDrive = new Victor (1);
m_rDrive = new Victor (2);
 
m_robotDrive = new RobotDrive (m_lDrive, m_rDrive);

m_timer = new Timer;
```

## Arcade Drive ##

This is the drive method that we most commonly use.

```
void RobotDrive::ArcadeDrive(float velocity, float rotation)
```

### `ArcadeDrive()` parameters ###

  * `float velocity`: The desired relative velocity of the robot (forwards/backwards)
  * `float rotation`: the relative velocity at which the robot rotates (left/right)

### `ArcadeDrive()` use ###

How you use `ArcadeDrive()` depends on which routine you are programming. If you are programming in Teleop, you give it the opposite of the left Y axis and the right X axis (so that the robot doesn't drift sideways if the left Y is being used), like so:

```
m_robotDrive->ArcadeDrive(-m_gamepad->GetRawAxis(2), -m_gamepad->GetRawAxis(4));
```

However, in autonomous, you would feed `ArcadeDrive()` either set or calculated values. If you are using set values, you would use it in conjunction with a `Timer` or an encoder.

```
if (!m_timer->HasPeriodPassed(2.0))
   m_robotDrive->ArcadeDrive(0.75,0.75);
else
   m_robotDrive->ArcadeDrive(0.0,0.0);
```

The above code will run the robot in a curve to the right for two seconds, and then stop.

## Tank Drive ##

The other driving function can achieve essentially the same effect as the `ArcadeDrive()`, but the mechanism it uses is different.

```
void RobotDrive::TankDrive(float left_power, float right_power)
```

### `TankDrive()` parameters ###

  * `float left_power`: The relative power (-1.0 to 1.0) going to the left drive.
  * `float right_power`: The relative power going to the right drive.

### `TankDrive()` use ###

How you use `TankDrive()` is determined in the same way as `ArcadeDrive()`. However, the game controllers do not lend themselves to this driving method. However, `TankDrive()` is actually the preferred method for driving in autonomous (since it makes the most sense).

```
if (m_timer->HasPeriodPassed(2.0))
   m_robotDrive->TankDrive(0.0,0.0);
else
   m_robotDrive(0.0,0.0);
```

The above code will run the robot in a left turn, moving forward.

# Activity #

Program the robot to do the following:

  * Drive forward and backward with the left stick.
  * Move left and right with the right stick.
  * When the start button pressed, toggle between the above "mode" and the following:
    * Drive forward when Y is held.
    * **Rotate** the robot counterclockwise (to the left) when the X button is held
    * **Rotate** the robot clockwise (to the right) when the B button is held.
    * Drive backwards when the A button is held.
    * Run everything at **half power** in this drive mode!

_The joystick drive and the button drive **cannot** be enabled at the same time!_

| ← [Lesson 5: Controlling Motors](WPI_Lesson5.md) | **Lesson 6** | [Chapter 3: Control - Lesson 7: Providing Information](WPI_Lesson7.md) → |
|:---------------------------------------------------|:-------------|:---------------------------------------------------------------------------|