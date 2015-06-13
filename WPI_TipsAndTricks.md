This page is intended to provide useful information that is either too long to explain in the main tutorial or would not otherwise be included in any lesson. All of these require a certain level of understanding in order to be used. Like the other page, that will be listed below the header.

Note that the levels listed here are the **absolute minimum** levels of knowledge you will need to implement them. It may be easier for you to learn them at a later time.

If you have not reached lesson 4 yet, you may want to read the note about [how functions are presented in this tutorial](WPI_Lesson4#Function_Presentation.md).



# Defined Constants #
**Required Level: Lesson 1**

You may have noticed in Lesson 4 that a file called "[Defines.h](https://code.google.com/p/hot67-programming-resources/source/browse/trunk/tutorial/Defines.h)" was mentioned. This header file creates alias terms for the button and axis indicies that are more easily understood by humans. This section will explain exactly how it works, and how you would implement them.

You can read more about constants [here](http://www.cplusplus.com/doc/tutorial/constants/).

## The Header Definition ##

Aside from providing functions and other utilities for use in a source file, header files can also define specific values that can be accessed using a name in ALL\_CAPS\_WITH\_NO\_SPACES. A sample header definition is given below.

```
#define MY_NUMBER 4
```

Essentially, we are telling the compiler's preprocessor to create a constant called `MY_NUMBER`, with a value of 4. The compiler will then replace any instance of `MY_NUMBER` in associated code with 4.

Remember that you need to include the header file in all other files with which you intend to use the constant.

## Use ##

The use of the header definition is quite simple. Simply write it instead of its value. So, if we had the function `void MyFunction(int)`, and we wanted to pass it `MY_NUMBER`, you would simply write:

```
MyFunction(MY_NUMBER);
```

This would be equivalent to calling `MyFunction(4)`.

## Why? ##

The use of a header definition depends on the application of the header file itself. In our case, we are using it to turn seemingly arbitrary numbers into a more easily understood format. If you were writing a library, you would use a header definition to avoid the difference of values that occurs between platforms.

# Timers #
**Required Level: Lesson 3**

If you remember from Lesson 3, there was a `Timer` object that could be used to time events. This section will teach you how to use it.

## Managing the timer ##

The `Timer` object comes with the following functions to manage the timer:

```
void Timer::Start()
void Timer::Stop()
void Timer::Reset()
```

Obviously, you would use these whenever you wanted to start, stop, or reset the timer, respectively. However, the operation of the timer isn't that foolproof.

For one, you should keep in mind that the `Reset()` function does **not** stop the timer if it is running. The timer will continue to run, should you only call the `Reset()` function.

Secondly, the `Timer` object does not always function as planned. It has some issues, and as such, it should become habitual that you call the `Stop()` and `Reset()` functions (back-to-back, in that order) if you need to reset the timer, anyway. Should that not work, then call `Stop()`, `Start()`, and `Reset()` (the first two can switch, depending if you need the timer to continue running or not). Should that not work, then another course of action might be advisable.

You should note that an initialized timer that has not been started yet will have a value of 0, so, depending on what exactly you are using the `Timer` for, you may need to call the `Start()` function in the class constructor after in initializing the timer.

## Checking the timer ##

Of course, a timer would be not useful if it did not have some way to access its time.

```
double Timer::Get()
bool Timer::HasPeriodPassed(double period)
```

The `Get()` function simply accesses the time (in seconds) that the timer has counted.

The `HasPeriodPassed()` function is like checking to see if the timer has gone off, so to speak. Its only parameter is the length, in seconds, of the timed period. Note that (it is my understanding that) `HasPeriodPassed` will continue to return `true` once the defined period has passed. In order to make it return false again, you need to reset the timer.

# Timeouts #
**Required Level: Lesson 4**

One of the downsides of the `GetRawButton()` function is that it returns true if there is a button pressed, regardless of how long that it has been pressed for. Because the cRIO is running the `TeleopPeriodic()` routine in a loop, one button press in the driver's point of view may register anywhere from 5 to 20 times in the cRIO. Due to that, manipulating variables by set amounts (incrementing/decrementing) is trickier than it sounds. This section discusses how to create a button timeout.

There are two methods to do this.

## Timer ##
**Requires knowledge of how to use a [timer](WPI_TipsAndTricks#Timers.md).**

This method is the preferred method for implementing a timeout, as how long the timeout lasts will remain constant with a timer. However, this method is also less robust than the other, due to the generally high maintenance level of the `Timer()` object.

### Required Objects ###

  * A `Timer`
  * A `Joystick`

### Creating a Timeout with a `Timer` ###

For simplicity's sake, let's assume that you have already declared and initialized the above objects.

The first thing that you should do when creating any timeout is check to see if the timeout has passed. This may seem odd at first, as when the `TeleopPeriodic()` routine is first entered, the timeout will not have occurred. However, you need to rembember that the same code will run over and over. Think 4<sup>th</sup>-dimensionally!

Here, we will use the `HasPeriodPassed()` function. A half a second should do for the timeout period. You should not forget that the timer will have a value of 0 upon entering the `TeleopPeriodic()` routine unless you start the timer at initialization.

Once you have confirmed that it is OK to check for a button press (either the timer has past the timeout or the timeout has not been started yet), check for one. If there is a button press, then run the action you need to run on the button press, then stop, start, and reset the timer (to ensure proper function).

```
//Assume that the Timer m_timer has been initialized and started in the class constructor.

if (m_timer->HasPeriodPassed(0.5))
{
  if (m_joystick->GetRawButton(1))
  { 
    //run the action...
    m_timer->Stop();
    m_timer->Start();
    m_timer->Reset();
  }
}
```

Because the timer is running on its own, that is all you need to do to implement a button timeout with a `Timer()`.

## Intelligent Loop Counter ##

This method is more robust than the previous one, as it relies purely on hard-coded logic and simple variables. However, the timeout will not be a constant value, it depends on how fast the cRIO loops -- which is loosely dependent on what you have the robot programmed to do.

### Required Variable ###

  * An `int` to count CPU loops

### Creating a Timeout with an Intelligent Loop Counter ###

The first thing that should be noted is that this particular method relies on the fact that the timeout is underway if the loop counter is a non-zero value. Due to this, you need to increase the loop counter on the loop the button press was first detected. This loop should **not** be counted as part of the timeout, so you need to add 1 to your number of loops when you run a check.

The most efficient way to achieve this is using a `switch`/`case` statement (click [here](CPP_Lesson4.md) for a review). You need to check for the following values:

  * 0
  * 16

However, this only covers a) if it is OK to check for a button press, and b) if the timeout has past and the loop counter needs to be reset. You still need to increment the loop counter during the timeout. Because this will happen when the counter has a value of anywhere between 1 and 15, use a `default` label to increment the counter.

```
  //assume that int m_loopCounter has been declared and initialized with a value of 0
  
  switch (m_loopCounter)
  {
    case 0:
      if (m_joystick->GetRawButton(1))
      {
	//run the action...
	
	//start the timeout
	m_loopCounter++;
      }
      break;
    case 16:
      //timeout is over, reset the counter
      m_loopCounter = 0;
      break;
    default:
      m_loopCounter++;
      break;
  }
```