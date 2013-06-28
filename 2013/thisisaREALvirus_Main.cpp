#include "Defines.h"
#include "PlatePIDHandler.h"
#include "PlatePIDAccelHandler.h"
#include "WPILib.h"
#include <cmath>
#include "AutonChoiceWrapper.h"
//#include "armHandler.h"

//Controller Values (also available in Defines.h)
/*
                        1:A
                        2:B
                        3:X
                        4:Y
                        5:Left Bumper
                        6:Right Bumper
                        7:Back
                        8:Start
                        9:L3
                        10:R3

                        Axes:
                        Left Stick X Axis: 1- Left-Negative Right-Positive
                        Left Stick Y Axis: 2- Up-Negative Down-Positive
                        Triggers: 3- Left-Positive Right-Negative
                        Right Stick X Axis: 4- Left-Negative Right-Positive
                        Right Stick Y Axis: 5- Up-Negative Down-Positive
*/
//Controller Map
/*
        Controller 1:
        Controller 2:
            Left vertical axis: Manual plate angle control
            Right vertical axis & Back: Manual climber extension control

            A: Medium Shot
            B: Shut off launcher
            X: Frisbee dump
            Y: Full Shot
            Left Bumper: Two-point Feeder Shot
            Right Bumper: Three-point Feeder Shot
            Left Trigger:
            Right Trigger:
*/

class BuiltinDefaultCode : public IterativeRobot
{
        // Declare variable for the robot drive system
        RobotDrive *m_robotDrive;               // robot will use PWM 1,2 for drive motors

        //Declare Drive Motors
        Victor *m_lDrive;
        Victor *m_rDrive;

        //Declare Launcher Motors
        Victor *m_Launcher1;
        Victor *m_Launcher2;
        Victor *m_Launcher3;

        //Declare Feeder
        Victor *m_Feeder;
        
        //Declare Climber Motors
        Victor *m_Climber;

        //Declare Plate Motor and PID controller
        Victor *m_Plate1;
        Victor *m_Plate2;
        PIDController* m_PlatePid;
        PIDController* m_PlateAccelPid;
        PlatePIDHandler* m_PlateHandler;
        PlatePIDAccelHandler* m_PlateAccelHandler;

        //Declare Controllers
        Joystick *m_Gamepad1;
        Joystick *m_Gamepad2;

        //Declare Sensors
        AnalogChannel *m_PlatePot;


        //Declare Encoders
        Encoder *m_lDriveEncoder;
        Encoder *m_rDriveEncoder;
        
        //Digital I/O
        DigitalOutput *m_clk;
        DigitalOutput *m_mosi;
        DigitalInput *m_miso;
        DigitalOutput *m_cs;
        //Accelerometer
        ADXL345_SPI *m_plateAccel;

        //Declare Relays
        Relay *m_Ratchet;
        //Declare timers
        Timer *m_timer;
        Timer *m_buttonTimer;
        Timer *m_ratchetTimer;

        // Declare a variable to use to access the driver station object
        DriverStationLCD *m_dsLCD;
        UINT32 m_priorPacketNumber;                                     // keep track of the most recent packet number from the DS
        UINT8 m_dsPacketsReceivedInCurrentSecond;       // keep track of the ds packets received in the current second

        //Variables to manage the SmartDashboard
        LiveWindow* lw;
        
        

        enum {                                                  // drive mode selection
                UNINITIALIZED_DRIVE = 0,
                ARCADE_DRIVE = 1,
                TANK_DRIVE = 2
        } m_driveMode;

        // Local variables to count the number of periodic loops performed
        UINT32 m_autoPeriodicLoops;
        UINT32 m_disabledPeriodicLoops;
        UINT32 m_telePeriodicLoops;
        UINT32 m_picSkipLoops;
        
        //Local variables to manage various things, e.g. the selected autonomous mode
        int m_autonMode;
        double m_launcherSpeed;
        double m_PlateAccelFilt;
        double m_plateBump;
        double m_drvStraightComp;
        int m_autonomousCase;
        int m_ratchetCase;
        int m_lockRatchetCase;

public:




        BuiltinDefaultCode(void)        {
                //SetPeriod (0.00);

                //Initialize Drive Motors
                m_lDrive = new Victor(1);
                m_rDrive = new Victor(2);
                
                //initialize climber motor
                m_Climber = new Victor(3);

                //Initialize Launcher and Feeder Motors
                m_Launcher1 = new Victor (4);
                m_Launcher2 = new Victor (5);
                m_Launcher3 = new Victor (6);
                m_Feeder = new Victor (7);
                //Initialize Ratchet
                m_Ratchet = new Relay(1);
                //Initialize Plate Motor
                m_Plate1 = new Victor (8);
                m_Plate2 = new Victor (10);

                //Initialize Controllers
                m_Gamepad1 = new Joystick(1);
                m_Gamepad2 = new Joystick(2);

                //Initialize Sensors
                m_PlatePot = new AnalogChannel(1);
                m_clk = new DigitalOutput(1);
                m_mosi = new DigitalOutput(2);
                m_miso = new DigitalInput(3);
                m_cs = new DigitalOutput(4);
                m_plateAccel = new ADXL345_SPI(m_clk, m_mosi, m_miso, m_cs);

                // Create a robot using standard right/left robot drive on PWMS 1, 2, 3, and #4
                m_robotDrive = new RobotDrive(m_lDrive, m_rDrive);
                m_robotDrive->SetSafetyEnabled(false);
                //Initialize encoders
                m_lDriveEncoder = new Encoder(1, 2, true);
                m_lDriveEncoder -> SetDistancePerPulse(1);
                m_lDriveEncoder -> SetMaxPeriod(1.0);
                m_lDriveEncoder -> Start();

                m_rDriveEncoder = new Encoder(3, 4, false);
                m_rDriveEncoder -> SetDistancePerPulse(1);
                m_rDriveEncoder -> SetMaxPeriod(1.0);
                m_rDriveEncoder -> Start();


                m_timer = new Timer();
                m_buttonTimer = new Timer();
                m_ratchetTimer = new Timer();

                //Initialize the PID for the arm. Check Defines.h for the values.
                m_PlateHandler = new PlatePIDHandler(m_Plate1, m_Plate2, m_PlatePot, m_plateAccel);
                m_PlateAccelHandler = new PlatePIDAccelHandler(m_Plate1, m_Plate2, m_PlatePot, m_plateAccel, m_PlateAccelFilt);
                                              //P        I    D        Sensor      Object that applies the changes
                m_PlatePid = new PIDController (PLATE_P, 0.0, 0.0, m_PlateHandler, m_PlateHandler);
                m_PlateAccelPid = new PIDController(8., 0.0, 0.0, m_PlateAccelHandler, m_PlateAccelHandler);
                m_PlateAccelPid->SetOutputRange(-0.4,0.4);

                // Acquire the Driver Station object
                m_dsLCD = DriverStationLCD::GetInstance();
                
                //m_armHandler = new armHandler(m_dsLCD, m_armPot, m_Arm1, m_Arm2);
                //m_armHandler->setPID(m_armPID);

                m_priorPacketNumber = 0;
                m_dsPacketsReceivedInCurrentSecond = 0;

                // Set drive mode to uninitialized
                m_driveMode = UNINITIALIZED_DRIVE;

                // Initialize counters to record the number of loops completed in autonomous and teleop modes
                m_autoPeriodicLoops = 0;
                m_disabledPeriodicLoops = 0;
                m_telePeriodicLoops = 0;
                m_launcherSpeed = 0;
                m_plateBump = 0;
                m_drvStraightComp = 0;
				m_autonomousCase = 0;
				m_ratchetCase = 0;
				m_lockRatchetCase = 0;
                //Start runnning the timer
                
        }


        /********************************** Init Routines *************************************/

        void RobotInit(void)
        {
                // Actions which would be performed once (and only once) upon initialization of the
                // robot would be put here.
        //	m_Choose->AddDefault("Default Auton ", new AutonChoiceWrapper(1, m_autonMode));
       // 	m_Choose->AddObject("Ignore this choice ", new AutonChoiceWrapper(2, m_autonMode));
        //	SmartDashboard::PutData("Autonomous Modes:", m_Choose);
        }

        void AutonomousInit(void)
        {
                m_autoPeriodicLoops = 0;
                m_telePeriodicLoops = 0;
                
                


                m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "                     ");
                m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "                     ");
                m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "                     ");
                m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "                     ");
                m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "                     ");
                m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "                     ");
                m_dsLCD->UpdateLCD();
                m_lDriveEncoder->Reset();
                m_rDriveEncoder->Reset();

                m_timer->Start();
                m_timer->Reset();

                m_PlatePid->SetSetpoint(PLATE_PYRAMID_AUTON);
                m_PlatePid->Enable();


        }

        void TeleopInit(void) {
                m_telePeriodicLoops = 0;                                // Reset the loop counter for teleop mode
                m_dsPacketsReceivedInCurrentSecond = 0; // Reset the number of dsPackets in current second
                m_driveMode = UNINITIALIZED_DRIVE;              // Set drive mode to uninitialized

                m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "                     ");
                m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "                     ");
                m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "                     ");
                m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "                     ");
                m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "                     ");
                m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "                     ");
                m_dsLCD->UpdateLCD();
                m_buttonTimer->Start();
                m_buttonTimer->Reset();
                m_lDriveEncoder->Reset();
                m_rDriveEncoder->Reset();
        }

        /********************************** Periodic Routines *************************************/
        void AutonomousPeriodic()
        {
        	m_PlatePid->SetSetpoint(PLATE_PYRAMID_AUTON);
			m_PlatePid->Enable();
        	switch(m_autonomousCase)
			{
			case 0:
				m_timer->Stop();
				m_timer->Start();
				m_timer->Reset();
				m_PlatePid->SetSetpoint(PLATE_PYRAMID_AUTON);
				m_PlatePid->Enable();
				m_autonomousCase = 1;
				break;
			case 1:   						
				m_Launcher1->Set(-(float)m_timer->Get()/2.0);
				m_Launcher2->Set(-(float)m_timer->Get()/2.0);
				m_Launcher3->Set(-(float)m_timer->Get()/2.0);
				
				if(m_timer->HasPeriodPassed(1.4))
				{
					m_Launcher1->Set(-0.7);
					m_Launcher2->Set(-0.7);
					m_Launcher3->Set(-0.7);
					m_autonomousCase = 2;
					m_timer->Stop();
					m_timer->Start();
					m_timer->Reset();
				}
				break;
			case 2:
				m_Feeder->Set(-0.5);
				if(m_timer->HasPeriodPassed(8.0))
				{
					m_timer->Stop();
					m_timer->Start();
					m_timer->Reset();
					m_autonomousCase = 3;
				}
			break;
			
			case 3:
				//DriveStraightEncoder(-0.8);
				if(m_timer->HasPeriodPassed(2.0))					
					m_autonomousCase = 4;
			break;
			case 4:
				//m_robotDrive->TankDrive(0.0,0.0);
			break;
			}
        }

/*        	if(m_autonCase == 0)
        	{
        		m_timer->Start();
        		m_timer->Reset();
        		m_PlatePid->SetSetpoint(PLATE_PYRAMID_AUTON);
        		m_PlatePid->Enable();
        		m_autonCase++;      
        	}
        	else if (m_timer->Get() <= 1.4)
			{
				m_Launcher1->Set(-(float)m_timer->Get()/2.0);
				m_Launcher2->Set(-(float)m_timer->Get()/2.0);
				m_Launcher3->Set(-(float)m_timer->Get()/2.0);
        	}
        	else if (m_timer->Get() >= 2.0)
        		m_Feeder->Set(-0.75);
        	m_robotDrive->TankDrive(0.0,0.0);
        	if (m_timer->Get() >= 10.0)
        	DriveStraightEncoder(-0.6);
        	if (m_timer->Get() >= 11.5)
        	DriveStraightEncoder(0);
*/

                void TeleopPeriodic(void)
                {
					//TeleopSensorUpdater();
					TeleopDriverStationUpdate();
					TeleopPlatePosition();
					TeleopLauncherSpeed();
					TeleopFeeder();
					TeleopDrive();
					TeleopSmartDashboardUpdate();
					TeleopClimberRatchetControl();
                }
                void TeleopClimberRatchetControl()
                {   	
                	SmartDashboard::PutNumber("Ratchet Case: ", m_ratchetCase);
                	SmartDashboard::PutNumber("Ratchet Lock Case: ", m_lockRatchetCase);
                	SmartDashboard::PutNumber("Timer: ", m_ratchetTimer->Get());
                	SmartDashboard::PutNumber("GamePad 1 Y: ", m_Gamepad1->GetRawAxis(RIGHT_Y));
                	if (m_Gamepad1->GetRawButton(BUTTON_BACK))
                	{
                		if (m_Gamepad1->GetRawButton(BUTTON_RB))
                		{
                			m_lockRatchetCase = 0;
							switch(m_ratchetCase)
							{
								case 0:
									m_Ratchet->Set(Relay::kForward);
									m_Climber->Set(-0.3);
									m_ratchetTimer->Stop();
									m_ratchetTimer->Start();
									m_ratchetTimer->Reset();
									m_ratchetCase = 1;
									break;
								case 1:
									if(m_ratchetTimer->HasPeriodPassed(0.125))
									{
										m_Ratchet->Set(Relay::kOff);
										m_ratchetCase = 2;
									}
									break;
								case 2:
									TeleopClimberControl();
									break;
							}
						
                		}
						else if (abs(m_Gamepad1->GetRawAxis(RIGHT_Y)) > 0.2)
						{
							
							if (m_Gamepad1->GetRawAxis(RIGHT_Y) < -0.2)
							{
								m_lockRatchetCase = 0;
								switch(m_ratchetCase)
								{
									case 0:
										m_Ratchet->Set(Relay::kForward);
										m_Climber->Set(-0.3);
										m_ratchetTimer->Stop();
										m_ratchetTimer->Start();
										m_ratchetTimer->Reset();
										m_ratchetCase = 1;
										break;
									case 1:
										if(m_ratchetTimer->HasPeriodPassed(0.125))
										{
											m_Ratchet->Set(Relay::kOff);
											m_ratchetCase = 2;
										}
										break;
									case 2:
										TeleopClimberControl();
										break;
								}
							}
							else if (m_Gamepad1->GetRawAxis(RIGHT_Y) > 0.2)
								TeleopClimberControl();
						}
						else
						{
							m_Climber->Set(0.0);
							m_ratchetCase = 0;
							switch(m_lockRatchetCase)
							{
								case 0:
									m_Ratchet->Set(Relay::kReverse);
									m_ratchetTimer->Stop();
									m_ratchetTimer->Start();
									m_ratchetTimer->Reset();
									m_lockRatchetCase = 1;
									break;
								case 1:
									if(m_ratchetTimer->HasPeriodPassed(0.125))
									{
										m_Ratchet->Set(Relay::kOff);
										m_lockRatchetCase = 2;
									}
									break;
							}
						}
                	}
                }
                
                void TeleopClimberControl()
                {
                	if (m_Gamepad1->GetRawAxis(RIGHT_Y) > 0.2)
						m_Climber->Set((-m_Gamepad1->GetRawAxis(RIGHT_Y)-0.2)*0.75);
					else if (m_Gamepad1->GetRawAxis(RIGHT_Y) < -0.2)
						m_Climber->Set((-m_Gamepad1->GetRawAxis(RIGHT_Y)+0.2)*0.75);
					else
						m_Climber->Set(0.0);
                }
                void TeleopSmartDashboardUpdate()
                {
                	SmartDashboard::PutNumber("Plate Postition: ", m_PlatePot->GetAverageVoltage()); 
                	SmartDashboard::PutNumber("Plate PID Position: ", m_PlatePot->PIDGet());
                	SmartDashboard::PutNumber("Plate Accel PID Position: ", m_plateAccel->GetAcceleration(m_plateAccel->kAxis_X));
                	SmartDashboard::PutNumber("Plate Pot PID Diff: ",abs(m_PlatePot->PIDGet()-PLATE_PYRAMID_THREE_POINT) );
                	SmartDashboard::PutNumber("Plate Accel PID Output: ", m_PlateAccelPid->Get());
                	SmartDashboard::PutNumber("Plate Stick: ", (m_Gamepad2->GetRawAxis(LEFT_Y)));
                	SmartDashboard::PutNumber("Filtered Accel: ", m_PlateAccelFilt);
                }

                void TeleopDriverStationUpdate(void)
                {
                	m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "lDrv Encoder: %f ", m_lDriveEncoder->Get());
                	m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "rDrv Encoder: %f ", m_rDriveEncoder->Get());
                	m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Plate Pot: %f ", m_PlatePot->PIDGet());
                	m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, " ");
                	m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, " ");
                	m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, " ");
                	m_dsLCD->UpdateLCD();


                }

                void TeleopDrive(void) {
					if(m_Gamepad1->GetRawButton(BUTTON_A))
						DriveStraightEncoder(1.0);
					else if(m_Gamepad1->GetRawAxis(TRIGGERS) > 0.4 && !m_Gamepad1->GetRawButton(BUTTON_BACK))
						m_robotDrive->ArcadeDrive(-(.75*(m_Gamepad1->GetRawAxis(LEFT_Y))), -(.75*((m_Gamepad1->GetRawAxis(RIGHT_X)))));
					else if(!m_Gamepad1->GetRawButton(BUTTON_BACK))
						m_robotDrive->ArcadeDrive(-m_Gamepad1->GetRawAxis(LEFT_Y), -m_Gamepad1->GetRawAxis(RIGHT_X));
					if(m_Gamepad1->GetRawButton(BUTTON_B))
					{
						m_lDriveEncoder->Reset();
						m_rDriveEncoder->Reset();
					}
                }




                void TeleopPlatePosition(void)
                {
                	
                	if(m_plateAccel->GetAcceleration(m_plateAccel->kAxis_X) - m_PlateAccelFilt > 0.1)
                	{
                		m_PlateAccelFilt=m_PlateAccelFilt + abs((m_plateAccel->GetAcceleration(m_plateAccel->kAxis_X)/10));
                	}
                	else if(m_plateAccel->GetAcceleration(m_plateAccel->kAxis_X) - m_PlateAccelFilt < -0.1)
                	{
                		m_PlateAccelFilt=m_PlateAccelFilt - abs((m_plateAccel->GetAcceleration(m_plateAccel->kAxis_X)/10));
                	}
                	else if(m_plateAccel->GetAcceleration(m_plateAccel->kAxis_X) - m_PlateAccelFilt > 0.02)
					{
						m_PlateAccelFilt=m_PlateAccelFilt + abs((m_plateAccel->GetAcceleration(m_plateAccel->kAxis_X)/5));
					}
					else if(m_plateAccel->GetAcceleration(m_plateAccel->kAxis_X) - m_PlateAccelFilt < -0.02)
					{
						m_PlateAccelFilt=m_PlateAccelFilt - abs((m_plateAccel->GetAcceleration(m_plateAccel->kAxis_X)/5));
					}
                	//FEEDER TWO POINT***********
                    if(m_Gamepad2->GetRawButton(BUTTON_LB))
                    {
                        m_PlatePid->SetSetpoint(PLATE_FEEDER_TWO_POINT);
                        m_PlatePid->Enable();
                    }
                    //FEEDER THREE POINT*********
                    else if (m_Gamepad2->GetRawButton(BUTTON_RB))
                    {
                    	m_PlatePid->SetSetpoint(PLATE_FEEDER_THREE_POINT);
                    	m_PlatePid->Enable();
                    }
                    //PYRAMID THREE POINT********
                    else if (m_Gamepad2->GetRawAxis(TRIGGERS) < -0.4)
                    {
                    	//if(abs(m_PlatePot->PIDGet()-PLATE_PYRAMID_THREE_POINT) >5)
                    	//{
                    		//m_PlateAccelPid->Disable();
                    		m_PlatePid->SetSetpoint(PLATE_PYRAMID_THREE_POINT);
                    		m_PlatePid->Enable();
                    	//}
                    	/*else
                    	//{
                    		m_PlatePid->Disable();
                    		m_PlateAccelPid->SetSetpoint(-0.371);
							m_PlateAccelPid->Enable();
                    	}*/
                    }
                    //PYRAMID DUMP/10-POINT CLIMB
                    else if (m_Gamepad2->GetRawAxis(TRIGGERS) > 0.4)
                    {
                    	m_PlatePid->SetSetpoint(PLATE_TEN_POINT_CLIMB);
                    	m_PlatePid->Enable();
                    }
                    //MANUAL CONTROL*************
                    else if (abs(m_Gamepad2->GetRawAxis(LEFT_Y)) > 0.2)
                    {
                    	m_PlatePid->Disable();
                    	/*if (m_Gamepad2->GetRawAxis(LEFT_Y) > 0.0 && m_PlatePot->GetAverageVoltage() > PLATE_HIGH_LIMIT)
                    		m_Plate->Set((m_Gamepad2->GetRawAxis(LEFT_Y) - 0.2)*1.25);
                    	else if (m_Gamepad2->GetRawAxis(LEFT_Y) < 0.0 && m_PlatePot->GetAverageVoltage() < PLATE_LOW_LIMIT)
                    		m_Plate->Set((m_Gamepad2->GetRawAxis(LEFT_Y) + 0.2)*1.25);
                    	
                    	else if (m_PlatePot->GetAverageVoltage() > PLATE_LOW_LIMIT && m_PlatePot->GetAverageVoltage() < PLATE_HIGH_LIMIT)
                    	{
                    	*/
                    		if (m_Gamepad2->GetRawAxis(LEFT_Y) > 0.0)
                    		{
                    			m_Plate1->Set((m_Gamepad2->GetRawAxis(LEFT_Y) - 0.2)*1.25);
                    			m_Plate2->Set((m_Gamepad2->GetRawAxis(LEFT_Y) - 0.2)*1.25);
                    		}
                    		else
                    		{
                    			m_Plate1->Set((m_Gamepad2->GetRawAxis(LEFT_Y) + 0.2)*1.25);
                    			m_Plate2->Set((m_Gamepad2->GetRawAxis(LEFT_Y) + 0.2)*1.25);
                    		}
                    	//}
                    }
                    else if(m_buttonTimer->Get() < 0.05)
					{
						if(m_plateBump == 1)
						{
							m_Plate1->Set(-0.3);
							m_Plate2->Set(-0.3);
						}
						else if(m_plateBump == -1)
						{
							m_Plate1->Set(0.4);
							m_Plate2->Set(0.4);
						}
						else
						{
							m_Plate1->Set(0.0);
							m_Plate2->Set(0.0);
						}
					}
                    else
					{
						m_PlatePid->Disable();
						m_PlateAccelPid->Disable();
						m_Plate1->Set(0.0);
						m_Plate2->Set(0.0);
					}
                    //BUMP UP! BUMP THE JAM! BUMP IT UP TILL YOU'RE FEELING FUNKY!
                    if(m_Gamepad2->GetRawButton(BUTTON_START) && m_buttonTimer->Get() > 0.5)
                    { //We should change these buttons (or add a new combination) so as not to interfere with the climber lock.
                    	
                    	m_buttonTimer->Start();
                    	m_buttonTimer->Reset();
                    	m_plateBump = 1;
                    }
                    else if(m_Gamepad2->GetRawButton(BUTTON_BACK) && m_buttonTimer->Get() > 0.5)
					{
                    	m_buttonTimer->Start();
                    	m_buttonTimer->Reset();               	
                    	m_plateBump = -1;
					}
                }
                void TeleopLauncherSpeed(void)
                {
               					if(m_Gamepad2->GetRawButton(BUTTON_A))
                                {
               						m_timer->Start();
               						m_timer->Reset();
               						m_launcherSpeed = 0.7;
                                }
               					else if(m_Gamepad2->GetRawButton(BUTTON_B))
								{
									m_timer->Stop();
									m_launcherSpeed = 0.0;
								}
               					else if(m_Gamepad2->GetRawButton(BUTTON_X))
								{
									m_Launcher1->Set(-0.3);
									m_Launcher2->Set(-0.3); //this is front on practice bot, WTF
									m_Launcher3->Set(-0.4);
								}
               					else if(m_Gamepad2->GetRawButton(BUTTON_Y))
								{
									m_timer->Start();
									m_timer->Reset();
									m_launcherSpeed = 1.0;
								}
               					else if (m_launcherSpeed < m_timer->Get() && m_launcherSpeed != 0)
                                {
                                		
                                		m_Launcher1->Set(-(float)m_timer->Get()/2.0);
                                		m_Launcher2->Set(-(float)m_timer->Get()/2.0);
                                		m_Launcher3->Set(-(float)m_timer->Get()/2.0);
                                }
                                else
                                {	
                                	m_timer->Stop();
                                	m_Launcher1->Set(-(float)m_launcherSpeed);
                                	m_Launcher2->Set(-(float)m_launcherSpeed);
                                	m_Launcher3->Set(-(float)m_launcherSpeed);
                                }
                                                }

                void TeleopFeeder(void)
                {
                	/***AVENGE THE HOPPER!!!!!!!!!!**/
                	//^ This is Nathan's fault
					
                	//Feed if right trigger pressed
					if((m_Gamepad1->GetRawAxis(TRIGGERS)) < -0.4)
						m_Feeder->Set(-1.0);
                                /*else if((m_Gamepad1->GetRawButton(BUTTON_BACK)) && ((m_Gamepad2->GetRawAxis(TRIGGERS)) < -0.4))
                                                m_Feeder->Set(0.5);*/
					else
					{
						m_Feeder->Set(0.0);
					}
                                // Left Stick Y Axis: 2- Up-Negative Down-Positive
                                /*if(m_HopRollerOverride == 0 && m_Gamepad2->GetRawAxis(LEFT_Y) > 0.4)
                                                m_HopRoller->Set(-1.0);
                                else if(m_HopRollerOverride == 0 && m_Gamepad2->GetRawAxis(LEFT_Y) < -0.4)
                                                m_HopRoller->Set(1.0);
                                else
                                                m_HopRoller->Set(0.0);*/
                }

        void DisabledPeriodic(void)  {
        	/*
                        m_autonomousCase = 0;
                //Pick Autonomous mode
                        if(m_Gamepad1->GetRawButton(BUTTON_START))
                        {
							if(m_Gamepad1 -> GetRawButton(BUTTON_B))
							{
								m_autonomousSelect = 1;
								m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Auton:               ");
								m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Top Key CHK Bridge   ");
							}
							if(m_Gamepad1 -> GetRawButton(BUTTON_Y))
							{
								m_autonomousSelect = 2;
								m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Auton:               ");
								m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Middle Key CHK Bridge");
							}
							if(m_Gamepad1 -> GetRawButton(BUTTON_X))
							{
								m_autonomousSelect = 3;
								m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Auton:               ");
								m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Bottom Key CHK Bridge");
							}
							if(m_Gamepad1 -> GetRawButton(BUTTON_R3))
							{
								m_autonomousSelect = 4;
								m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Auton:               ");
								m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Top Key Shoot Only   ");
							}
							if(m_Gamepad1 -> GetRawButton(BUTTON_L3))
							{
								m_autonomousSelect = 5;
								m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Auton:               ");
								m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Bottom Key Shoot Only");
							}
							if(m_Gamepad1 -> GetRawButton(BUTTON_LB))
							{
								m_autonomousSelect = 6;
								m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Auton:               ");
								m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Bridge First         ");
							}
							if(m_Gamepad1 -> GetRawButton(BUTTON_RB))
							{
								m_autonomousSelect = 7;
								m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Auton:               ");
								m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Alliance Bridge      ");
							}
                        }
                        else if(m_Gamepad1->GetRawButton(BUTTON_BACK))
                        {
                        	if(m_Gamepad1 -> GetRawButton(BUTTON_B))
							{
								m_autonomousSelect = 11;
								m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Auton:               ");
								m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "QCK Top Key CHK Bridg");
							}
							if(m_Gamepad1 -> GetRawButton(BUTTON_Y))
							{
								m_autonomousSelect = 12;
								m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Auton:               ");
								m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "QCK Middle Key CHK Br");
							}
							if(m_Gamepad1 -> GetRawButton(BUTTON_X))
							{
								m_autonomousSelect = 13;
								m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Auton:               ");
								m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "QCK Bottom Key CHK Br");
							}
							if(m_Gamepad1 -> GetRawButton(BUTTON_R3))
							{
								m_autonomousSelect = 14;
								m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Auton:               ");
								m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Top Key Shoot Only   ");
							}
							if(m_Gamepad1 -> GetRawButton(BUTTON_L3))
							{
								m_autonomousSelect = 15;
								m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Auton:               ");
								m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Bottom Key Shoot Only");
							}
							if(m_Gamepad1 -> GetRawButton(BUTTON_LB))
							{
								m_autonomousSelect = 16;
								m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Auton:               ");
								m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Bridge First         ");
							}
							if(m_Gamepad1 -> GetRawButton(BUTTON_RB))
							{
								m_autonomousSelect = 17;
								m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Auton:               ");
								m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Quick Alliance Bridge");
							}
                        }
                        else if(m_Gamepad1->GetRawButton(BUTTON_L3))
					    {
                        	if(m_Gamepad1->GetRawButton(BUTTON_A))
                        	{
                        		m_buttonTimer->Start();
                        		if(m_buttonTimer->HasPeriodPassed(0.2))
                        		{
									m_cannonOffset++;
									m_buttonTimer->Reset();
								}

							}
							if(m_Gamepad1->GetRawButton(BUTTON_B))
							{
								m_buttonTimer->Start();
								if(m_buttonTimer->HasPeriodPassed(0.2))
								{
									m_cannonOffset--;
									m_buttonTimer->Reset();
								}
							}
						 }
					else
					{
						m_buttonTimer->Stop();
					}
                    if(m_autonomousSelect == 0)
                    {
                        m_autonomousSelect = 12;
                        m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Auton:               ");
                        m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "QCK Middle Key CHK Br");
                    }


                m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "                     ");
                m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "                     ");
                m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "Cannon Offset:       ");
                m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "    %d               ", m_cannonOffset);
                m_dsLCD->UpdateLCD();*/
        }

        void DisabledInit(void)
        {
                m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "                     ");
                m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "                     ");
                m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "                     ");
                m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "                     ");
                m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "                     ");
                m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "                     ");
                m_dsLCD->UpdateLCD();

        }
        void DriveStraightEncoder(float speed)
        {
			if (m_lDriveEncoder->GetDistance() + 10 > m_rDriveEncoder->GetDistance())
			{
				m_robotDrive->TankDrive(speed + m_drvStraightComp, speed - m_drvStraightComp);
				m_drvStraightComp = m_drvStraightComp + 0.0001;
			}
			else if (m_rDriveEncoder->GetDistance() + 10 > m_lDriveEncoder->GetDistance())
			{
				m_robotDrive->TankDrive(speed - m_drvStraightComp, speed + m_drvStraightComp);
				m_drvStraightComp = m_drvStraightComp + 0.0001;
			}
			else
			{
				m_robotDrive->TankDrive(speed, speed);
				m_drvStraightComp = 0.05;
			}
        }


/*
        void cameraAutoAim()
        {
                float m_slowSpeed;
                float m_fastSpeed;

                if(m_telePeriodicLoops > 0)
                {
                        m_slowSpeed = 500;
                        m_fastSpeed = 500;
                        //m_slowSpeed = 550;
                                //m_fastSpeed = 800;
                }
                else
                {
                        m_slowSpeed = 500;
                        m_fastSpeed = 500;
                }

                //static int duty = 0;
                if(m_picSkipLoops != 0)
                {

                        double cur_point = getTurnDistance();
                    dash->PutDouble("m_Dese-Cur",(fabs(m_desiredEndpoint - cur_point)));
                        if(fabs(m_desiredEndpoint - cur_point) < 15)
                                          {
                                                        m_DrvTurnPIDRate->Disable();
                                                        DrvTurnPIDRateHandler->integrator = 0;
                                m_robotDrive->ArcadeDrive((float)(0),(float)(0));
                                                        m_cannonAimed = m_cannonAimed + 1;
                                                        m_picSkipLoops = 0;
                                          }
                        else if(fabs(m_desiredEndpoint - cur_point) < 50 )
                                                {
                                m_DrvTurnPIDRate->Enable();
                                m_cannonAimed = 0;
                                                        if(duty <= 1){
                                                                if(m_desiredEndpoint < cur_point)
                                                                        m_robotDrive->ArcadeDrive(0.0,-.7);
                                                                else
                                                                        m_robotDrive->ArcadeDrive(0.0,.7);

                                                                if(duty == 0)
                                                                        duty = 3;
                                                                else
                                                                        duty--;
                                                        } else {
                                                                duty--;
                                                                m_robotDrive->ArcadeDrive(0.0 ,0.0);
                                                        }
                                if((m_desiredEndpoint - cur_point) > 0)
                                        m_DrvTurnPIDRate->SetSetpoint(m_slowSpeed);
                                else
                                        m_DrvTurnPIDRate->SetSetpoint(-m_slowSpeed);
                                                }
                        else if(fabs(m_desiredEndpoint - cur_point) > 50)
                        {
                        //m_DrvTurnPID->SetOutputRange(0.3,0.8);
                                m_cannonAimed = 0;
                                //m_DrvTurnPID->SetTolerance(0.02);
                                if((m_desiredEndpoint - cur_point) > 0)
                                        m_DrvTurnPIDRate->SetSetpoint(m_fastSpeed);
                                else
                                        m_DrvTurnPIDRate->SetSetpoint(-m_fastSpeed);
                                                        //m_DrvTurnPIDRate->SetSetpoint(m_desiredEndpoint);
                                                        m_DrvTurnPIDRate->Enable();
                                                }
                                                dash->PutDouble("cur_point",cur_point);
                        dash->PutDouble("m_desiredEndpoint",m_desiredEndpoint);
                        dash->PutDouble("Output",m_DrvTurnPIDRate->Get());
                        dash->PutDouble("total turn rate",(m_Encoder1->GetRate()-m_Encoder2->GetRate()));
                        dash->PutBoolean("OnTarget",m_DrvTurnPIDRate->OnTarget());
                        m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "%d %d %d",m_cannonAimed,m_desiredEndpoint,cur_point);
                        m_dsLCD->UpdateLCD();


                }
                else
                {
                        //m_robotDrive->SetSafetyEnabled(false);
                        m_robotDrive->ArcadeDrive((float)(0),(float)(0));
                        float nCenter = (float)(m_camHandle->getCenter())+ cameraOffset;

                        //Display the normalized center on the dashboard
                        char outputstring[10];
                        sprintf(outputstring,"%0.2f",nCenter);
                        dash->PutString("Normalized Center:",outputstring);

                        //Specify the desired endpoint
                        m_Encoder1->Reset();
                        m_Encoder2->Reset();
                        m_desiredEndpoint = -8.98*REV_IN*nCenter*1.2;  //Divide by 2 because of double the encoder count
                        setDriveRampParams(0, m_desiredEndpoint, rampUp, initSpd, finalSpd, (m_desiredEndpoint*rampDown), 1);
                        m_picSkipLoops = 30;
                }
        }

        void CameraDiagnosticMode()
        {
                m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "CAMERA DIAGNOSTIC MODE");
                m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Cam Offset: %f     ", cameraOffset);
                m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Init Spd: %f       ",initSpd);
                m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "Final Spd: %f      ",finalSpd);
                m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "Ramp Up: %f        ",rampUp);
                m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "Ramp Down: %f      ",rampDown);
                m_dsLCD->UpdateLCD();

                if(m_Gamepad1->GetRawButton(BUTTON_A) && !m_Gamepad1->GetRawButton(BUTTON_LB))
                        cameraOffset += 0.01;
                else if(m_Gamepad1->GetRawButton(BUTTON_A) && m_Gamepad1->GetRawButton(BUTTON_LB))
                        cameraOffset -= 0.01;

                if(m_Gamepad1->GetRawButton(BUTTON_X) && !m_Gamepad1->GetRawButton(BUTTON_LB))
                        initSpd += 0.05;
                else if(m_Gamepad1->GetRawButton(BUTTON_X) && m_Gamepad1->GetRawButton(BUTTON_LB))
                        initSpd -= 0.05;

                if(m_Gamepad1->GetRawButton(BUTTON_B) && !m_Gamepad1->GetRawButton(BUTTON_LB))
                        finalSpd += 0.05;
                else if(m_Gamepad1->GetRawButton(BUTTON_B) && m_Gamepad1->GetRawButton(BUTTON_LB))
                        finalSpd -= 0.05;

                if(m_Gamepad1->GetRawButton(BUTTON_Y) && !m_Gamepad1->GetRawButton(BUTTON_LB))
                        rampUp += 0.001;
                else if(m_Gamepad1->GetRawButton(BUTTON_Y) && m_Gamepad1->GetRawButton(BUTTON_LB))
                        rampUp -= 0.001;

                if(m_Gamepad1->GetRawButton(BUTTON_RB) && !m_Gamepad1->GetRawButton(BUTTON_LB))
                        rampDown += 0.05;
                else if(m_Gamepad1->GetRawButton(BUTTON_RB) && m_Gamepad1->GetRawButton(BUTTON_LB))
                        rampDown -= 0.05;
        }


        double getDriveDistance(){
                return (-(m_Encoder1->GetDistance() + m_Encoder2->GetDistance()));
        }
        double getTurnDistance(){
                        return ((m_Encoder1->GetDistance() - m_Encoder2->GetDistance()));
                }
*/
        /*
        void DiagnosticMode(void)
        {
                //Back button with run all motors/relays backwards
                if(m_Gamepad2 -> GetRawButton(BUTTON_BACK))
                        m_Test = (-0.9);
                else
                        m_Test = (0.9);

                //Left drive
                if(m_Gamepad2 -> GetRawButton(BUTTON_A))
                        m_lDrive -> Set(m_Test);
                else
                        m_lDrive -> Set(0);

                //Right Drive
                if(m_Gamepad2 -> GetRawButton(BUTTON_B))
                        m_rDrive -> Set(m_Test);
                else
                        m_rDrive -> Set(0);

                //Hopper Roller
                if(m_Gamepad2 -> GetRawButton(BUTTON_X))
                {       m_HopRoller -> Set(Relay::kForward);
                        if(m_Gamepad2->GetRawButton(BUTTON_BACK))
                                m_HopRoller -> Set(Relay::kReverse);
                }
                else
                        m_HopRoller -> Set(Relay::kOff);

                //Unused Y Button
                //if(m_Gamepad2 -> GetRawButton(BUTTON_Y))

                //Cannon 1
                if(m_Gamepad2 -> GetRawButton(BUTTON_LB))
                        m_Cannon1 -> Set(m_Test);
                else
                        m_Cannon1 -> Set(0);

                //Cannon 2
                if(m_Gamepad2 -> GetRawButton(BUTTON_RB))
                        m_Cannon2 -> Set(m_Test);
                else
                        m_Cannon2 -> Set(0);

                //Light
                if(m_Gamepad2 -> GetRawButton(BUTTON_START))
                        m_Light -> Set(Relay::kForward);
                else
                        m_Light -> Set(Relay::kOff);

                //Index
                if(m_Gamepad2->GetRawButton(BUTTON_BACK) != 1 && m_Gamepad2 -> GetRawButton(BUTTON_L3))
                        m_Index -> Set(Relay::kForward);
                else if(m_Gamepad2->GetRawButton(BUTTON_BACK) && m_Gamepad2->GetRawButton(BUTTON_L3))
                        m_Index -> Set(Relay::kReverse);
                else
                        m_Index -> Set(Relay::kOff);

                //Shifter
                if(m_Gamepad2 -> GetRawButton(BUTTON_R3))
                        m_Shifter -> Set(Relay::kForward);
                else if(m_Gamepad2->GetRawButton(BUTTON_BACK) && m_Gamepad2->GetRawButton(10))
                        m_Shifter -> Set(Relay::kReverse);
                else
                        m_Shifter -> Set(Relay::kOff);

                //Arm 1
                if((m_Gamepad2 ->GetRawAxis(TRIGGERS)) > (0.4))
                        m_Arm1 -> Set(m_Test);
                else
                        m_Arm1 -> Set(0);

                //Arm 2
                if((m_Gamepad2 ->GetRawAxis(TRIGGERS)) < (-0.4))
                        m_Arm2 -> Set(m_Test);
                else
                        m_Arm2 -> Set(0);

                //Roller 1
                if((m_Gamepad2 ->GetRawAxis(LEFT_Y)) < (-0.4))
                        m_Roller1 -> Set(m_Test);
                else
                        m_Roller1 -> Set(0);

                //Roller 2
                if((m_Gamepad2 ->GetRawAxis(LEFT_Y)) > (0.4))
                        m_Roller2 -> Set(m_Test);
                else
                        m_Roller2 -> Set(0);
        }
*/
};
START_ROBOT_CLASS(BuiltinDefaultCode);
