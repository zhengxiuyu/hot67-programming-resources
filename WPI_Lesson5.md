This lesson is all about making the robot actually do something beyond manage variables -- making it move.



# The Motor #

For the purposes of this tutorial, assume that all of the motors are [Victors](WPI_Lesson1#Victors.md). The API for controlling motors is basically the same across the three more complex drive controllers (Victors, [Jaguars](WPI_Lesson1#Jaguars.md), and [Talons](WPI_Lesson1#Talons.md)), so you won't need to worry about using different drive controllers. However, you will still need to make sure that you are declaring the correct controller.

Another thing that you should take into consideration when programming the robot's motors is the PWM of the motor that you are programming. You should always be sure that you 1) initialize the motor, and 2) initialize it on the right PWM!

For simplicity, the declaration of all of the components used in the code snippets below has been left out. You can assume the following:

```
/** Declarations **/
Joystick* m_gamepad;
Victor* m_arm;
Victor* m_launcher;

/** Initialization **/
m_gamepad = new Joystick (2);
m_arm = new Victor (4);
m_launcher = new Victor (6);
```

## Moving ##

Controlling the drive controllers is fairly simple. For basic control, you would use the following functions:

```
Victor::Set(double speed)
Jaguar::Set(double speed)
Talon::Set(double speed)
```

### `Set` parameters ###

  * `double speed`: Relative speed of the motor, where 1.0 is full speed forward, and -1.0 is full speed reverse. Note that this depends on the orientation of the **motor**, and not necessarily that of the robot. Due to that, you will need to run the motor with your code to check that you have coded the proper value.

### `Set` use ###

You can use `Set` in a variety of ways. If you need fine control over the speed of the motor, you would use `Set` in conjunction with a `GetRawAxis` from one of your controllers (almost always controller 2 -- the joysticks on controller 1 are always reserved for driving), like so:

```
m_arm->Set(-m_gamepad->GetRawAxis(2)); //remember that the Y axis is backwards!!!
```

The above code sets up controller 2, a `Victor` for controlling an arm on PWM 4, and then uses the left Y axis to move the arm up and down.

The other way you would use `Set` would be to bring a motor to a specific (relative) speed. This would be useful for powering up a launcher motor, etc. There is another way to do this, but that will be described in thorough detail throughout the next chapter. For now, stick with this method.

In this case, you would use the controller buttons (usually A, B, X, and Y) to set the speed. B should generally be your 'cancel' button, or speed of 0.

You might use it like this:

```
if (m_gamepad->GetRawButton(1)) //if the A button is pressed, full forward
  m_launcher->Set(1.0);
else if (m_gamepad->GetRawButton(3)) //if the X button is pressed, full reverse
  m_launcher->Set(-1.0);
else if (m_gamepad->GetRawButton(2)) //if the B button is pressed, cancel
  m_launcher->Set(0.0);
```

In this case, a button timeout would not be required, because you are setting the motor to the same value while the button is pressed.

## How Fast? ##

If you set one motor to different speeds throughout your program, or the `Set()` function is controlled by a controller joystick, you may find yourself at some point needing to figure out what that motor is currently set to. There is a function for that.

```
float Victor::Get()
float Jaguar::Get()
float Talon::Get()
```

## Physical Capabilities of the Robot ##

Now that you are programming things to move, you should, first of all, be aware that **things are moving**, so you should be even more certain that you and everyone around you know that the equipment you are working with can move and potentially cause serious injury. Secondly, you should be aware of the robot's physical capabilities while you are programming. For example, the 2013 bot's launcher system uses motors driving wheels via belts. If the motors spin up the belts too quickly, the belt pulleys will spin inside the belt, which in turn shreds the belt. In this case, the robot was programmed to spin up more slowly. At any rate, you should be aware of potential problems such as this.

# The Relay #

One of the other controllers that was mentioned in lesson 1 was the [Spike](WPI_Lesson1#Spikes%2FRelays.md), which ran off of a relay. Since these devices are basically like light switches (with the added bonus of being able to go in reverse as well), they are programmed differently.

For this section, you can further assume that two relays have been set up like this:

```
Relay* m_roller;
Relay* m_light;

m_roller = new Relay (1);
m_roller = new Relay (2);
```

## Relay `Set` ##

Like the other controllers, relays use the `Set` function. However, it is used differently than the other functions.

```
Relay::Set(enum RELAY_STATE)
```

### Relay `Set` parameters ###

  * `enum relay_state`: The desired state of the relay, which is one of the following:
    * `Relay::kOff`: Off
    * `Relay::kForward`: Forward, on (for lights)
    * `Relay::kReverse`: Reverse

Again, you need to be careful, as the `kForward` and `kReverse` states are determined from the wiring and the orientation of the motor.

### Relay `Set` use ###

You would use `Set` like you might expect, calling it to enable (forward or reverse) or disable the relay.

```
if (m_gamepad->GetRawButton(1))
  m_roller->Set(Relay::kForward);
else if (m_gamepad->GetRawButton(2))
  m_roller->Set(Relay::kReverse);
else
  m_roller->Set(Relay::kOff);
```

The above code will make a roller move in one direction if the A button is held, and in the other direction if the B button is held.

One thing that should be noted is that lights almost always are left on relays (unless they are unmanaged). Because they are lights, you should only ever use the `Relay:kOff` and `Relay::kForward` states.

```
if (m_gamepad->GetRawButton(4))
  m_light->Set(Relay::kForward);
else
  m_light->Set(Relay::kOff);
```

The above code will turn a light on if the Y button is held.

## What State? ##

Similar to the previous three, Relays also have a method to get what the controller is currently set to.

```
enum Relay::Get()
```

This returns the enumeration (from [above](WPI_Lesson5#Relay_Set_Use.md)) that represents the current state of the relay.

# Activity #

**This activity is designed for the 2013 robot. Talk to someone who is familiar with the setup of other robots to write something for another robot.**

Program the robot to do the following:

  * Move the plate up and down using the left Y axis.
    * PWMs: 8 and 10
    * Do not invert the controller reading
  * Bring the launcher up to full speed with the A button, and turn it off with the B button.
    * PWMs: 4-6
    * Slow spin up! In order to do this, you can start a [timer](WPI_TipsAndTricks#Timers.md) and set the victors to half the timer's value until the timer reaches 2 seconds.
      * Only for spin up. Setting the motors to 0 from full is fine.
  * Turn the climb indicator light on and off with the X button.
    * Relay: 2
    * [Timeout](WPI_TipsAndTricks#Timeouts.md)!

Don't forget the controller deadband (0.2), as the controllers do not zero properly. To regain fine control, add the deadband (if the controller reading is negative) / subtract the deadband (if the controller reading is positive) and then multiply that value by 1.25.

| ← [Lesson 4: Using User Input](WPI_Lesson4.md) | **Lesson 5** | [Lesson 6: Earning Your Driver's License: Driving](WPI_Lesson6.md) → |
|:-------------------------------------------------|:-------------|:-----------------------------------------------------------------------|