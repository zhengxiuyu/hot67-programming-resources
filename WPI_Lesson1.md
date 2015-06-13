By now, you should be well versed in the ways of C++. While this is a lot to have learned, this still is not all. You can't just `cout<<"Robot: Do this\n";` and expect the robot to do whatever it is you want (as a matter of fact, trying to do that could cause compiler issues).

Instead, this tutorial will teach you to use a different **library**, or set of functions that allow you to various things. You are already acquainted with these, as what you `#include` at the top of your source code are libraries. In this case, the library that you will be using is WPILib, which you would include as `#include "WPILib.h"`. But that will be discussed later.

What is more important right now is that you know what you are programming. If you don't, then you don't know how to program it. For the most part, what you really need to worry about is the electrical components of the robot; however, you should acquaint yourself with some of its mechanics so that you know what you want the electrical components to do.

This lesson will go over the various electrical components of the robot. Chances are that you already have gone over this, but it is still good to review.



# Basic Components #
All of the following components are not programmed, because their operation is usually set in stone.

## Battery ##
<img src='http://www.instructables.com/files/deriv/FHW/LWVN/H5ED7SOP/FHWLWVNH5ED7SOP.LARGE.jpg' width='186px' height='177px' />

If you do not know what this is, please slap yourself on the face several times.

## Main Breaker ##
<img src='http://frc-labs.com/wp-content/uploads/2012/09/100_1129.jpg' height='150px' width='201px' />

This is also fairly self-explanatory. This turns the whole robot on and off. The red button is the off button; there is a little black lever you push into the switch to turn the robot on. If you accidentally e-stop the robot, you need to cycle power on the robot using this.

## Power distribution panel ##
<img src='http://team358.org/files/programming/ControlSystem2009-/DistPanel.jpg' width='174px' height='150px' />

The main breaker feeds directly into this panel. From here, power is divided (depending on voltage) to the various components of the robot. Note that you don't always put a 5V device on a 5V terminal -- if the voltage from the battery dropped, the power for that device would also drop, causing it to stop working.

## Notification light ##
<img src='http://team358.org/files/programming/ControlSystem2009-/RSL-wiring.jpg' width='265px' height='110px' />

This orange light is actually programmed, but you will never see the code for this light. It is embedded in the cRIO controller, which will be discussed next. This light changes its flash frequency to tell you whether the robot is connected to a driver station or not.

# Programmed Components #
The following components are either programmed, affect your program in some way, or are affected by your program.

## cRIO ##
<img src='http://www.ni.com/cms/images/devzone/tut/9076_Empty.jpg' width='200' height='140px' />

This is the brain of the robot. It communicates with the driver station either drirectly through its LAN port, or through a radio connected to the LAN port. Your code is stored here. When you replace it, you need to reboot this (either using the main breaker or the "Reboot cRIO..." button on the driver station) in order for your changes to take effect.

A cRIO usually consists of two ethernet ports, a power input, a serial port, and 4 module ports (on older models: 8), which hold modules to send and receive data to/from various components of the robot.

## Digital Sidecar ##
<img src='http://a248.e.akamai.net/origin-cdn.volusion.com/vyfsn.knvgw/v/vspfiles/photos/am-0866-2.jpg?1345723078' width='200px' height='126px' />

This is connected to the cRIO through one of the cRIO's modules. You actually don't program this, but where things are plugged into this will heavily affect your program. When you give a device a numerical assignment (e.g. PWM 3), you are giving the port in which the device is plugged into on the digital sidecar. The following is a brief list of the ports. The location is with the FIRST logo facing up.
  * PWM ( **P** ulse **W** idth **M** odulation) outputs (1-10), for drive controllers
    * These are located along the left side of the device, labeled 'PWM'
  * Digital I/O (input/output) (1-14), for sensors
    * These are along the upper right of the device, labeled 'DIGITAL I/O'
  * Relays (1-16), which provide commands for controllers (like a [Spike](WPI_Lesson1#Spikes%2FRelays.md)) for devices that have an on-or-off state (e.g. lights).
    * These are located along the the lower right of the device, labeled 'RELAY'

There are more ports and uses for a digital sidecar, which are listed [here](http://www.usfirst.org/sites/default/files/uploadedFiles/Robotics_Programs/FRC/Game_and_Season__Info/2012_Assets/Digital%20Sidecar.pdf).

## Analog Breakout ##
<img src='http://team358.org/files/programming/ControlSystem2009-/AnalogModule.JPG' width='200px' height='212px' />

This is a cRIO module that usually receives sensor data in non-digital format. For example, potentiometers (see [Potentiometers](WPI_Lesson1#Potentiometer.md) for more) provide their data in the format of voltage. This device measures the analog input and converts it into a digital signal for the cRIO to process.

There are 8 analog channels, located along the top of the module.

## Motor Controllers ##
The following components are devices that are used to control other devices on the robot (hence the name). The devices that are controlled by controllers (e.g. motors) operate based on analog signals (most commonly voltage), so controllers are needed to "smarten" these devices and provide more control for these devices.

In the code, as you will learn later, you program these as if they are the device they are operating. You do not actually program the devices the controllers control.

### Victors ###
<img src='http://www.vexrobotics.com/media/catalog/product/cache/11/image/296x/5e06319eda06f020e43594a9c230972d/2/1/217-2769-950x950_1.jpg' width='148px' height='148px' />

Victors are one kind of drive controller. They run off of a PWM output on the digital sidecar, and they translate the PWM signal into a voltage.

This team is partial to these drive controllers (at least for the drivetrain), because they don't tend to chicken out when the robot gets into a shoving match.

### Jaguars ###
<img src='http://www.robotmarketplace.com/products/images/IFI-JAGUAR_lg.jpg' width='200px' height='172px' />

Jaguars are also drive controllers, and they too run off of a PWM channel on the digital sidecar. In terms of programming, they are essentially the same as Victors. They do provide different features from Victors; however, despite their size, they can freak out when they encounter too much stress.

### Talons ###
<img src='http://team358.org/files/programming/ControlSystem2009-/Talon.jpg' width='200px' height='176px' />

Talons are a new type of drive controller that have not been used extensively by our team. In tests, they have shown themselves to be very nice, combining some of the features of Jaguars and the robustness of Victors.

These, as well, run off of PWM channels.

### Spikes/Relays ###
<img src='http://www.robotmarketplace.com/products/images/IFIW-SP1_lg.jpg' width='200px' height='162px' />

Spikes, controllers that run off of relays on the digital sidecar, have three states: Off, Forward, and Reverse (the latter two being the device's "On" state). They do not provide voltage control, so these are usually used on motors which do not require speed control (like window motors) or lights. The Reverse state is like taking a battery out and inserting it the other way, reversing the polarity. This is useful for driving motors in different directions.

## Motors ##
All motors are hooked up to a controller, else they would be running constantly and in one state. You do not actually program the motors, but what you program the controller to do affects the motor.

Most motors are very similar. The only reason that specific motors are used is for different output ranges, torque, etc., properties that you, as programmers, are not particularly concerned with.

Of course, motors are not limited to this list. However, these are the kinds of motors you will see most commonly.

### CIM ###
<img src='http://ndroboknights.com/wp-content/uploads/2012/01/768-CIM_motor.jpg' width='200px' height='126px' />

CIMs are usually seen as the de-facto motor of FRC. These are powerful motors that don't give out easily. You will see these motors driving the drivetrain, but in some cases they power other things as well (in the case of the 2013 robot, they drove the climbing mechanism as well).

### Banebot ###
<img src='http://www.robotshop.com/Images/big/en/banebots-rs-550-motor-12v.jpg' width='200px' height='200px' />

### AndyMark ###
<img src='http://a248.e.akamai.net/origin-cdn.volusion.com/vyfsn.knvgw/v/vspfiles/photos/am-0912-2.jpg?1345723078' width='250px' height='164px' />

### Window Motors ###
<img src='http://www.hamiltontaxservices.com/images/MB/W126%20left%20rear%20window%20motor.jpg' width='200' height='150px' />

Believe it or not, but this is the actual motor that drives the power windows in your car.

Window motors are different from most other motors in that they run off of a relay (meaning they have the off, forward, and reverse states). You will usually see these running low-strain operations which do not require speed control.

### Servos and Vex Motors ###
<img src='http://www.servocity.com/assets/images/HS-311.jpg' width='180px' height='164px' />

Servos are special types of motors. They read directly off of a PWM cable. Doing so permits them to be controlled without a motor controller.

The width of the pulse provided in the PWM determines the endpoint of the motion of the servo.

Servos also have a builtin sensor that determines the rotational angle of the output. This allows you to control the position of whatever is attached to the servo without a PID (in theory).

Servos can have various rotational ranges, from a half-turn to multiple turns.

#### Vex Motors ####
<img src='http://www.pickmarked.net/Robotics/images/2wire269.jpg' width='193px' height='214px' />

Vex motors are very similar to a servo motor (and some are servo motors), but they are not all servos, despite their similar appearance. These have specialized drive controllers, which are treated like Victors in the code.

## Sensors ##
Sensors are crucial parts to the robot. They provide important information pertaining to the robot's (or a part of the robot's) position. Note that what is listed here is not necessarily what can be used on the robot.

### Digital Inputs ###
The following sensors are attached to the [Digital Sidecar](WPI_Lesson1#Digital_Sidecar.md), and provide their data digitally.

#### Encoder ####
<img src='http://upload.wikimedia.org/wikipedia/commons/thumb/a/a8/Gray_code_rotary_encoder_13-track_opened.jpg/125px-Gray_code_rotary_encoder_13-track_opened.jpg'>

Encoders are sensors that are used to measure distance. This distance can either be linear or rotational. You will see these most commonly on the drivetrain, measuring the distance the whole robot has traveled, usually used in autonomous.<br>
<br>
Encoders work by emitting pulses every time a predefined distance has passed after the previous pulse. If you don't want to use the intervals that are hard-wired into the encoders, there is a software function that allows you to change this (we still usually just use the pulses as the distance). However, that will be discussed later.<br>
<br>
Encoders always count up (unless driven backwards, then they count down), making them comparable to the odometer of a car.<br>
<br>
<h4>Gear Tooth Sensor</h4>
<img src='http://www.epcos.com/web/generator/Web/Sections/Components/Applications/2011/08__Digital__gear__tooth__sensors/Figure1,property=Data__en.jpg' />

It may be hard to believe, but this sensor watches (with a magnet) a gear and counts the number of teeth that pass by in a given period of time.<br>
<br>
<h4>Limit Switch</h4>
<img src='https://www.egr.msu.edu/eceshop/Parts_Inventory/images/spdt%20limit%20switch.jpg' width='214px' height='214px' />

These digital devices are simple, in that they have two states: on and off (pressed and not pressed).<br>
<br>
<h4>Ultrasonic Sensor</h4>
<img src='http://www.maxbotix.com/pictures/XL/XL%20Ultrasonic%20Sensor%20Iso.jpg' width='259' height='214px' />

This device is basically sonar. It sends out a super-high-pitched sound and waits for the echo to determine the distance to an object.<br>
<br>
<h3>Analog Inputs</h3>
The following sensors can all be found plugged into the <a href='WPI_Lesson1#Analog_Breakout.md'>analog breakout box</a>. They provide their data by varying voltage.<br>
<br>
<h4>Potentiometer</h4>
<img src='http://upload.wikimedia.org/wikipedia/commons/thumb/b/b5/Potentiometer.jpg/225px-Potentiometer.jpg' />

Potentiometers are sensors that measure rotation. You may not realize it, but a lot of the electronics you interact with on a day-to-day basis use potentiometers. For example, the volume knob on your car stereo works because there is a potentiometer behind the knob.<br>
<br>
The range of a potentiometer's output is technically from 0-6 volts, but may actually be around 1-5.<br>
<br>
<img src='http://fddrsn.net/pcomp/images/potentiometer1.gif' title='Mechanism of a potentiometer' width='236px' height='162px' />

The number of turns a potentiometer will allow varies depending on the potentiometer. Some allow 3, some allow 10, some even allow less than a full turn (potentiometers are not limited to these). Because most of the potentiometers you will interact with have the same voltage range, potentiometers that have a lower number of turns are more sensitive, therefore they will be more precise.<br>
<br>
<h4>Gyroscope</h4>
<img src='http://www.robotshop.com/blog/en/files/sfe-dual-axis-gyro.jpg' width='200px' height='200px' />

Gyroscopes are sensors that determine rotation about all three axes: essentially, it determines the robot's rotation compared to the ground. It can also measure lateral direction (y rotation).<br>
<br>
<h4>Accelerometer</h4>
<img src='http://www.pyroelectro.com/tutorials/accel_intro/img/accel.jpg' />

Accelerometers are sensors that measure acceleration, but can also be used to measure rotation (similar to a <a href='WPI_Lesson1#Gyroscope.md'>gyroscope</a>). The use of an accelerometer versus a gyroscope depends on the application.<br>
<br>
<h3>Other sensors</h3>

<h4>Camera</h4>
<img src='http://img.tomshardware.com/us/2003/12/11/axis_network_camera_review_axis205/205right.jpg' />

A camera's video feed can be used for various purposes -- in recent years, teams have used them to create an auto-alignment feature. They can also simply be put on the dashboard, so that the drivers can align the robot themselves. Of course, their use is not limited to that -- for example, it can be put near the bottom front of the robot so the drivers can see objects stuck in tight spaces or something small in front of the robot.<br>
<br>
The cameras available to robotics teams are hooked up directly to the radio, with no cRIO processing.<br>
<br>
<h2>Controls</h2>
<i>Not to be confused with <a href='WPI_Lesson1#Motor_Controllers.md'>motor controllers</a>.</i>

These devices are what controls the robot from the driver station.<br>
<br>
<h3>Computer</h3>
<i>Commonly referred to as a driver station if it is sending user input to the robot</i>

<img src='http://www.digitaltrends.com/wp-content/uploads/2011/05/hp-elitebook-8560p-display.jpg' width='266px' height='214px' />

If you do not know what this is, then you just made a paradox. In terms of robot operation, they gather data from the drivers and send it to the robot for processing.<br>
<br>
<h3>Radio</h3>
<i>Also known as a <b>wireless bridge</b>. Note that what is shown here is not the radio used by FRC.</i>

<img src='http://www.productwiki.com/upload/images/d_link_dap_1522.jpg' height='163px' width='250px' />

The radio is the device that handles all of the robot's incoming and outgoing data. It is similar to a router, but it has other functions, as well.<br>
<br>
During competitions, it acts less like a router and more like a client device (for example, a laptop on your home wifi), which connects to the field management system (aka FMS), which handles all of the data for every competing team, and sends it to the proper place.<br>
<br>
<h3>Game Controller</h3>
<img src='http://upload.wikimedia.org/wikipedia/commons/thumb/e/ed/Xbox-360-S-Controller.png/293px-Xbox-360-S-Controller.png' />

If you are under the age of 21 and do not know what this is, you have had a deprived childhood. Still, given how widespread videogames are these days, it is hard not to know what one of these is.<br>
<br>
FRC provides XBOX 360 controllers, so those are the game controllers used by essentially all teams.<br>
<br>
In this case, each button runs a certain function, either started by a button press or only while a button is being held. The joysticks are used to drive the robot (Left Y axis and right X axis, to allow precision driving), and to perform manual operations.<br>
<br>
<h3>Joystick</h3>
<i>Not to be confused with <a href='WPI_Lesson1#Game_Controller.md'>game controller</a> joysticks</i>

<img src='http://www.logitech.com/assets/35368/attack-3-joystick53.png' />

This is just a different input device. We do not use these, but you may see other teams using them.<br>
<br>
<h1>Miscellaneous</h1>
Of course, the amount of components is not limited to this. There are even entirely different control systems (like an Arudino or a CAN bus). This is just a list of simple components that run on a simple control system (which we use).<br>
<br>
<h1>Activity</h1>
For each of the following pictures, do the following WITHOUT REFERRING TO THE ABOVE TUTORIAL:<br>
<ol><li>Identify the component<br>
</li><li>Identify the component's category and applicable sub-categories<br>
</li><li>Explain its function</li></ol>

So, if you had the following picture:<br>
<br>
<img src='http://www.elexp.com/test/ping.jpg' width='225px' height='150px' />

You would write:<br>
<ul><li>Ultrasonic sensor<br>
</li><li>Analog sensor<br>
</li><li>Determines distances using ultrasonic waves</li></ul>

There are 10 questions.<br>
<br>
<img src='http://sine.ni.com/images/products/us/crio-9072_empty_fpga_l.jpg' width='200px' height='146px' />

<img src='http://www.3dcontentcentral.com/showmodels/CONTENTCENTRAL%5CVictor%20883-short%20fan%5CVictor%20883-short%20fan.jpg' />

<img src='http://img.ehowcdn.com/article-new/ehow/images/a05/3r/7k/potentiometer-works-800x800.jpg' width='150px' height='94px' />

<img src='http://www.brain.kyutech.ac.jp/~ishii/Darya_Bird/Ronbun/GyroSensor.png' width='225px' height='225px' />

<img src='http://team358.org/files/programming/ControlSystem2009-/AnalogModule.JPG' width='200px' height='212px' />

<img src='http://www.automationdirect.com/images/overviews/encoder_trdn_300.jpg' title="Hint: it's a digital input." width='150px' height='139px' />

Hint: it's a digital input.<br>
<br>
<img src='http://www.firstcadlibrary.com/images/CIM%20Motor.gif' />

<img src='http://www.bakersfieldads.net/Greenacres-/Allen-bradley-800T-Q10-ser-t-orange-pilot-light.jpg' width='200px' height='148px' />

<img src='http://www.lightobject.info/eBay/Images/LimitSWC2.jpg' width='250px' height='250px' />

<img src='http://i204.photobucket.com/albums/bb190/bennettrossinc/FIRST%20FRC%2009%20cRIO%20Pics%20from%20NI%20Visit/IMG_6967.jpg' width='285px' height='214px' />

<table><thead><th> ← <a href='Main_Page.md'>Main Page</a> </th><th> <b>Lesson 1</b> </th><th> <a href='WPI_Lesson2.md'>Lesson 2: Program Structure</a> → </th></thead><tbody>