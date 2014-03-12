#include "uploadpancakes_Main.h"
//#include "armHandler.h"

//Controller Map
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


class BuiltinDefaultCode : public IterativeRobot, public PIDOutput
{
        // Declare variable for the robot drive system
        RobotDrive *m_robotDrive;               // robot will use PWM 1,2 for drive motors
        
        //Declare Drive Motors
        Victor *m_lDrive;
        Victor *m_rDrive;
        
        //Declare Shooter Motors
        Jaguar *m_Cannon1;
        Jaguar *m_Cannon2;
        Relay *m_Index;
        //Declare Hopper Roller
        Victor *m_HopRoller;
        
        //Declare Utiliarm Motors
        Jaguar *m_Arm1;
        Jaguar *m_Arm2;
        Jaguar *m_Roller1;
        Jaguar *m_Roller2;
        
        //Declare Shifter
        Relay *m_Shifter;
        Relay *m_Light;
        Relay *m_armLight;
        
        //Declare Controllers
        Joystick *m_Gamepad1;
        Joystick *m_Gamepad2;
        
        //Declare Sensors
        AnalogChannel *m_armPot;
        Gyro *m_Gyro;
        Encoder *m_shooterSpeed;
        cameraHandler *m_camHandle;

        
        //Declare Encoders
        Encoder *m_Encoder1;
        Encoder *m_Encoder2;
        Encoder *m_CannonEncode;
        
        //Declare Digital Inputs
        DigitalInput *m_bridgeCheck;
        
        //PID Loops
        HotPID *m_armPosition;
        
        //Declare Kinect and Kinect Appendages
        Kinect *m_kinect;
        KinectStick *m_leftArm;
        KinectStick *m_rightArm;
        
        //Arm Handler
        //armHandler *m_armHandler;
        
        DrvStraightEncode *DrvStraightEncodeHandler;
        DrvStraightPID *DrvStraightPIDHandler;
        DrvTurnEncode *DrvTurnEncodeHandler;
        DrvTurnPID *DrvTurnPIDHandler;
        DrvTurnPIDRate *DrvTurnPIDRateHandler;
        DrvTurnEncodeRate *DrvTurnEncodeRateHandler;
        
        PIDController *m_CannonPID;
        PIDController *m_armPID;
        PIDController *m_DrvStraightPID;
        PIDController *m_DrvTurnPID;
        PIDController *m_DrvTurnPIDRate;
        Timer *m_timer;
        Timer *m_buttonTimer;
        
        double m_Encoder1Distance;
        double m_Encoder2Distance;
        double m_armPotValue;
        double m_Test;
        int m_autonomousCase;
        int m_autonomousSelect;
        int m_autonCounter;
        int m_cannonOffset;
        int m_shiftcounter;
        int m_shiftreversecounter;
        int m_shiftflag;
        int m_ArmPositionFlag;
        int m_countFlag;
        int m_shooterStatus;
        int m_cannonAimed;
        int m_cannonUpToSpeed;
        int m_HopRollerOverride;
        float m_cannonRate;
        float m_RPM;
        float m_RPMcounter;
        float m_RPMTimeCounter;
        float kP;
        float kI;
        float kD;
        //float ARM_DUMP;
        //float ARM_PICKUP;
        float m_cannonSetPoint;
        float m_AutonArmPos;
        float incr_speed;
        int m_armDown;
        int m_armTimer;
        int m_RollerPulseOut;
        
        float m_leftArmValue;
        float m_rightArmValue;
        float m_rightAnkleValue;
        float m_rightHipValue;
        int backdrive;
        double backdrive_spd;
        float cannonSpeed;
        int buttonFlag;
        int goDraino;
        
        //Camera Diagnostic Variables
        double cameraOffset; //Added for competition bot because camera aim was off.
        double initSpd;
        double finalSpd;
        double rampUp;
        double rampDown;
        float comp_spd;
        
        int m_bridgeDown;
        // Declare a variable to use to access the driver station object
        DriverStationLCD *m_dsLCD;
        SmartDashboard *dash;
        UINT32 m_priorPacketNumber;                                     // keep track of the most recent packet number from the DS
        UINT8 m_dsPacketsReceivedInCurrentSecond;       // keep track of the ds packets received in the current second
        

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
        double m_desiredEndpoint;
        float m_CannonDemandedSpeed;

public:

        void PIDWrite(float output) {
                //char outputstring[10];
                //sprintf(outputstring,"%0.4f",output);
                //dash->PutString("PID Output:",outputstring);
                
                //char outputstring[10];
                //sprintf(outputstring,"%0.4f",m_CannonDemandedSpeed);
                //dash->PutString("Speed Controller Input:",outputstring);
                
                //sprintf(outputstring,"%0.4f",m_cannonSetPoint);
                //dash->PutString("Cannon Set Point:",outputstring);
                
                if((m_cannonSetPoint - 5) <= m_CannonEncode->GetRate() && (m_cannonSetPoint + 5) >= m_CannonEncode->GetRate())
                {
                        m_cannonUpToSpeed = 1;
                }
                else if(m_cannonSetPoint == 500)
                {
                        m_cannonUpToSpeed = 1;
                }
                else
                {
                        m_cannonUpToSpeed = 0;
                }
                
                m_CannonDemandedSpeed += output;
                if(m_CannonDemandedSpeed > 1) m_CannonDemandedSpeed = 1;
                if(m_CannonDemandedSpeed < 0) m_CannonDemandedSpeed = 0;
                m_Cannon1->Set(-m_CannonDemandedSpeed);
                m_Cannon2->Set(-m_CannonDemandedSpeed);
        }

        
        BuiltinDefaultCode(void)        {
                //SetPeriod (0.00);
                
                //Initialize Drive Motors
                m_lDrive = new Victor(1);
                m_rDrive = new Victor(2);
                
                //Initialize Shooter Motors
                m_Cannon1 = new Jaguar(5); 
                m_Cannon2 = new Jaguar(6);
                m_Index = new Relay(1);
                m_HopRoller = new Victor(3);
                
                //Initialize Utiliarm Motors
                m_Arm1 = new Jaguar(7);
                m_Arm2 = new Jaguar(8);
                
                m_Roller1 = new Jaguar(9);
                m_Roller2 = new Jaguar(10);
                
                //Initialize Shifter
                m_Shifter = new Relay(2);
                m_Light = new Relay(3);
                m_armLight = new Relay(4);
                
                //Initialize Controllers
                m_Gamepad1 = new Joystick(1);
                m_Gamepad2 = new Joystick(2);
                
                
                
                //Initialize Sensors
                m_armPot = new AnalogChannel(1);
                m_Gyro = new Gyro(2);
                m_bridgeCheck = new DigitalInput(6);
                
                //Initialize Kinect and Kinect Appendages
                m_kinect = Kinect::GetInstance();
                m_leftArm = new KinectStick(1);
                m_rightArm = new KinectStick(2);
                
                m_shooterSpeed = new Encoder(5,5, true);
                m_shooterSpeed -> SetMaxPeriod(3.0);
                m_shooterSpeed -> Start();

                // Create a robot using standard right/left robot drive on PWMS 1, 2, 3, and #4
                m_robotDrive = new RobotDrive(m_lDrive, m_rDrive);
                
                //Initialize encoders
                m_Encoder1 = new Encoder(1, 2, true);
                m_Encoder1 -> SetDistancePerPulse(1);
                m_Encoder1 -> SetMaxPeriod(1.0);
                m_Encoder1 -> Start();
                
                m_Encoder2 = new Encoder(3, 4, false);
                m_Encoder2 -> SetDistancePerPulse(1);
                m_Encoder2 -> SetMaxPeriod(1.0);
                m_Encoder2 -> Start();
                
                m_CannonEncode = new Encoder(5,5,true);
                m_CannonEncode -> SetDistancePerPulse(1);
                m_CannonEncode -> Start();
                m_CannonEncode -> SetPIDSourceParameter(Encoder::kRate);
                
                //Initialize cameraHandler
                AxisCamera *camera = &AxisCamera::GetInstance("10.0.67.11");
                DrvStraightEncodeHandler = new DrvStraightEncode(m_Encoder1, m_Encoder2);
                DrvStraightPIDHandler = new DrvStraightPID(m_robotDrive, m_Encoder1, m_Encoder1);
                DrvTurnEncodeHandler = new DrvTurnEncode(m_Encoder1, m_Encoder2);
                DrvTurnEncodeRateHandler = new DrvTurnEncodeRate(m_Encoder1, m_Encoder2);
                DrvTurnPIDHandler = new DrvTurnPID(m_robotDrive, m_Encoder1, m_Encoder1);
                DrvTurnPIDRateHandler = new DrvTurnPIDRate(m_robotDrive, m_Encoder1, m_Encoder2);
                
                m_CannonPID = new PIDController(0.01 ,0 ,0.05, m_CannonEncode, this);
                m_DrvStraightPID = new PIDController(0.005, 0.0, 0.025, DrvStraightEncodeHandler, DrvStraightPIDHandler);
                m_DrvTurnPID = new PIDController(0.015, 0.00, 0.03, DrvTurnEncodeHandler, DrvTurnPIDHandler);
                m_DrvTurnPIDRate = new PIDController(0.0001, 0.00, 0.0003, DrvTurnEncodeRateHandler, DrvTurnPIDRateHandler);
                //m_DrvTurnPID -> SetOutputRange(0.0, 0.8);
                
                m_timer = new Timer();
                m_buttonTimer = new Timer();
                
                kP = 2.0;
                kI = 0.0;
                kD = 0.05;
                
                /*PIDs
                kP = 0.0001;
                                kI = 0.0;
                                kD = 0.0003;
                                */
                m_armPosition = new HotPID(2.0, 0.0, 0.05, 0.0);
                
                //MASSIVE VARIBALE LIST
                m_Encoder1Distance = 0;
                m_Encoder2Distance = 0;
                m_armPotValue = 0;
             
                m_autonomousCase = 0;
                m_autonomousSelect = 0;
                m_autonCounter = 0;
                m_cannonOffset = 0;
                m_shiftcounter = 0;
                m_shiftreversecounter = 0;
                m_shiftflag = 0;
                m_ArmPositionFlag = 2;
                m_countFlag = 0;
                m_RPMcounter = 0;
                m_RPM = 0;
                m_RPMTimeCounter = 0;
                m_shooterStatus = 0;
                m_cannonAimed = 0;
                m_cannonUpToSpeed = 0;
                m_Test = 0.9;
                //ARM_DUMP = 2.85;
                //ARM_PICKUP = 4.5917;
                m_cannonSetPoint = 0;
                goDraino = 0;
                m_AutonArmPos = ARM_BRIDGE;
                m_armDown = 0;
                m_armTimer = 0;
                m_RollerPulseOut = 10;
                
                m_leftArmValue = 0;
                m_rightArmValue = 0;
                m_rightAnkleValue = 0;
                m_rightHipValue = 0;
                m_HopRollerOverride = 0;
                
                backdrive = 0;
                backdrive_spd = -0.6;

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
                
                //Camera Diagnostic Variables
                cameraOffset = 0.01; //Added for competition bot because camera aim was off.
                initSpd = 0.75;
                finalSpd = 0.9;
                rampUp = 0.001;
                rampDown = 0.5;
                comp_spd = 0.05;
                
                m_bridgeDown = 0;
                
                dash = SmartDashboard::GetInstance();
                m_camHandle = new cameraHandler(camera, m_dsLCD, dash, m_Light);
        }
        
        
        /********************************** Init Routines *************************************/

        void RobotInit(void) 
        {
                // Actions which would be performed once (and only once) upon initialization of the
                // robot would be put here.
                
        }
        
        void AutonomousInit(void) 
        {
                m_autoPeriodicLoops = 0;
                m_telePeriodicLoops = 0;
                m_armPosition->Reset();
                m_armPosition->SetSPLimits(+6.0, +0.0); // desired position (inches)
                m_armPosition->SetPVLimits(+6.0, +0.0); // actual position (inches)
                m_armPosition->SetMVLimits(+1.0, -1.0); // Manipulated Variable, Jaguar drive
                m_armPosition->Enable();
                
                m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "                     ");
                m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "                     ");
                m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "                     ");
                m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "                     ");
                m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "                     ");
                m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "                     ");
                m_dsLCD->UpdateLCD();
                m_autonomousCase = 0;
                m_Encoder1->Reset();
                m_Encoder2->Reset();
                m_AutonArmPos = 0;
                //m_AutonArmPos = ARM_BRIDGEAUTON;
        }

        void TeleopInit(void) {
                m_telePeriodicLoops = 0;                                // Reset the loop counter for teleop mode
                m_dsPacketsReceivedInCurrentSecond = 0; // Reset the number of dsPackets in current second
                m_driveMode = UNINITIALIZED_DRIVE;              // Set drive mode to uninitialized
                m_shooterStatus = 0;
                m_RPMcounter = 0;
                m_RollerPulseOut = 10;
                
                
                m_armPosition->Reset();
                m_armPosition->SetSPLimits(+6.0, +0.0); // desired position (inches)
                m_armPosition->SetPVLimits(+6.0, +0.0); // actual position (inches)
                m_armPosition->SetMVLimits(+0.85, -0.85); // Manipulated Variable, Jaguar drive
                m_armPosition->Enable();
                
                m_autonomousCase = 0;
                m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "                     ");
                m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "                     ");
                m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "                     ");
                m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "                     ");
                m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "                     ");
                m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "                     ");
                m_dsLCD->UpdateLCD();
                m_armPosition->SetGains(kP, kI, kD);
                m_AutonArmPos = ARM_BRIDGE;
        }

        /********************************** Periodic Routines *************************************/
        void AutonomousPeriodic() 
        {
        		m_Light->Set(Relay::kForward);
        		switch(m_autonomousSelect)
                {
                        case 1:
                        {
                                TopofKeyCheckBridge();
                        }
                        break;
                        case 2:
                        {
                        	QuickMiddleofKey();	
                        	//MiddleofKeyCheckBridge();
                        }
                        break;
                        case 3:
                        {
                        		BottomofKeyCheckBridge();
                        }
                        break;
                        case 4:
                        {
                        		Shoot2TopKey();
                        }
                        break;
                        case 5:
                        {
                        		Shoot2BottomKey();;
                        }
                        break;
                        case 6:
                        {
								BridgeFirstCheckBridge();
                        }
                        break;
                        case 7:
                        {
                        		AllianceBridge();
                        }
                        break;
                        case 11:
                        {
                                QuickTopofKey();
                        }
                        break;
                        case 12:
                        {
                        		QuickMiddleofKeyV2();
                        }
                        break;
                        case 13:
                        {
                        		QuickBottomofKey();
                        }
                        break;
                        case 14:
                        {
                        		Shoot2TopKey();
                        }
                        break;
                        case 15:
                        {
                        		Shoot2BottomKey();;
                        }
                        break;
                        case 16:
                        {
								BridgeFirstCheckBridge();
                        }
                        break;
                        case 17:
                        {
                        		QuickAllianceBridge();
                        }
                        break;
                }
        }
        void AllianceBridge()
		{
			m_autoPeriodicLoops++;
			//m_robotDrive->SetSafetyEnabled(false);

			TeleopSensorUpdater();
			TeleopDriverStationUpdate();

			m_HopRoller->Set(-1.0);
			
			if(m_AutonArmPos != 0)
			{
				ArmPosition(m_AutonArmPos);
			}
			else
			{
				m_Arm1->Set(0.0);
				m_Arm2->Set(0.0);
			}
			m_rightHipValue = m_kinect->GetSkeleton().GetHipRight().z;
			m_rightAnkleValue = m_kinect->GetSkeleton().GetAnkleRight().z;
							
			if((m_rightHipValue - m_rightAnkleValue) >= LEG_FORWARD)
			{
				RunKinect();
				m_autonomousCase = 17;
			}
			else 
			{
				switch(m_autonomousCase)
				{
				case 0:
					m_AutonArmPos = 1.9552;
					m_Encoder1->Reset();
					m_Encoder2->Reset();
					m_timer->Start();
					m_timer->Reset();
					m_autonomousCase = 1;
				break;
				
				case 1:
					//Set cannon to speed where it could make a shot going backwards
					m_cannonSetPoint = BOTTOMKEY_SHOT + m_cannonOffset;
					m_CannonPID->SetSetpoint(BOTTOMKEY_SHOT + m_cannonOffset);
					m_CannonPID->Enable();
					//Shift into low gear
					m_armLight->Set(Relay::kForward);
					//Hop Roller runs constantly throughout Autonomous
					m_HopRoller->Set(-1.0);
					
					if(m_cannonUpToSpeed == 1)
					{
							m_HopRoller->Set(-1.0);                 
							m_Index->Set(Relay::kForward);
					}
					else
					{
							m_Index->Set(Relay::kOff);
					}
					if(m_timer->HasPeriodPassed(3.2)){
						backdrive = 0;
						m_armLight->Set(Relay::kOff);
						m_timer->Stop();
						m_timer->Reset();
						m_timer->Start();
						m_DrvTurnPID->SetSetpoint(20.417 * REV_IN);
						m_autonomousCase = 2;
					}
				break;
				
				case 2:

					m_cannonSetPoint = -100;
					m_CannonDemandedSpeed=0.;
					m_CannonPID->Disable();
					m_Index->Set(Relay::kOff);
		
					if (m_timer->HasPeriodPassed(0.5))
					{
							m_DrvTurnPID->Disable();
							m_DrvStraightPID->SetSetpoint(193.15 * REV_IN);
							m_Encoder1->Reset();
							m_Encoder2->Reset();
							m_timer->Stop();
							m_timer->Start();
							m_timer->Reset();
							m_autonomousCase = 3;
					}
					else
						m_DrvTurnPID->Enable();
				break;
				
				case 3:
					if(m_timer->HasPeriodPassed(1.3))
					{
						m_DrvStraightPID->Disable();
						m_Encoder1->Reset();
						m_Encoder2->Reset();
						m_timer->Stop();
						m_timer->Start();
						m_timer->Reset();
						m_DrvTurnPID->SetSetpoint(-35.973 * REV_IN);
						m_autonomousCase = 4;
					}
					else
						m_DrvStraightPID->Enable();
				break;
				
				case 4:
					if (m_timer->HasPeriodPassed(0.5))
					{
							m_DrvTurnPID->Disable();
							m_Encoder1->Reset();
							m_Encoder2->Reset();
							m_timer->Stop();
							m_timer->Start();
							m_timer->Reset();
							m_DrvStraightPID->SetSetpoint(100.76 * REV_IN);
							m_autonomousCase = 5;
					}
					else
						m_DrvTurnPID->Enable();
						
				break;
				
				case 5:
					if(m_timer->HasPeriodPassed(1.0))
					{
						m_DrvStraightPID->Disable();
						m_Encoder1->Reset();
						m_Encoder2->Reset();
						m_timer->Stop();
						m_autonomousCase = 6;
						m_armPosition->SetGains(1.0, kI, kD);
					}
					else
						m_DrvStraightPID->Enable();
				break;
				
				case 6:
					//Bring arm down to lower bridge and run on the bridge
					m_AutonArmPos = ARM_BRIDGE;
					//Once bridge is lowered reset encoders and set next driving cycle up bridge
					if ((m_armPot->GetAverageVoltage()) >= (ARM_BRIDGE - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_BRIDGE + 0.1))
					{
						if(!m_bridgeCheck->Get())
						{
							m_armTimer = 0;
							m_autonomousCase = 7;
						}
						else
						{	//Jump to extra routine if bridge is not found.
							m_Encoder1->Reset();
							m_Encoder2->Reset();
							setDriveRampParams(0, 87.6*REV_IN, 0.00, 0.4, 0.4, 10*REV_IN, 0);
							m_armTimer = 0;
							m_autonomousCase = 18;
						}
					}
				break;
				
				case 7:
					//Once bridge is lowered reset encoders and set next driving cycle up bridge
					m_AutonArmPos = ARM_GROUND;
					if ((m_armPot->GetAverageVoltage()) >= (ARM_GROUND - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_GROUND + 0.1))
					{
							m_Encoder1->Reset();
							m_Encoder2->Reset();
							m_autoPeriodicLoops = 0;
							m_autonomousCase = 9;
							setDriveRampParams(0, 87.6*REV_IN, 0.02, 0.6, 1.0, 35.04*REV_IN, 0);
					}
					m_armTimer++;
					if(m_armTimer > 30)
					{
						setDriveRampParams(0, -87.6*REV_IN, 0.00, 0.55, 0.55, 10*REV_IN, 0);
						m_autonomousCase = 19;
					}
				break;
				
				case 9:
					//Once up the bridge change arm gains, reset encoders and set next short trip
					if (updateRampDrive())
					{
							m_armPosition->SetGains(3.0, kI, kD);
							//Lift arm to ball dump position
							m_Encoder1->Reset();
							m_Encoder2->Reset();
							m_RollerPulseOut = 0;
							m_autonomousCase = 10;
							m_autoPeriodicLoops = 0;
					}
				break;
				
				case 10:
					if(m_autoPeriodicLoops <= 15)
					{
						m_Roller1->Set(-0.8);
						m_Roller2->Set(0.8);
					}
					else if(m_autoPeriodicLoops > 15 && m_autoPeriodicLoops <= 30)
					{
						if(m_RollerPulseOut < 5)
						{
							m_Roller1->Set(0.);
							m_Roller2->Set(0.);
							m_RollerPulseOut++;
						}
						else if(m_RollerPulseOut < 8)
						{
							m_Roller1->Set(0.8);
							m_Roller2->Set(-0.8);
							m_RollerPulseOut++;
						}
						else
						{
							m_Roller1->Set(0.);
							m_Roller2->Set(0.);
						}
						
					}
					else if(m_autoPeriodicLoops > 30 && m_autoPeriodicLoops <= 40)
					{
						m_AutonArmPos = ARM_DUMP;
					}
					else if(m_autoPeriodicLoops > 40)
					{
						m_autoPeriodicLoops = 0;
						m_armDown = 0;
						m_DrvStraightPID->SetSetpoint(-87.60 * REV_IN);
						m_Encoder1->Reset();
						m_Encoder2->Reset();
						m_timer->Start();
						m_timer->Reset();
						m_autonomousCase = 11;
					}
				break;
				
				case 11:
					if (m_timer->HasPeriodPassed(0.5))
					{
							m_DrvStraightPID->Disable();
							m_timer->Stop();
							m_timer->Start();
							m_timer->Reset();
							m_Encoder1->Reset();
							m_Encoder2->Reset();
							m_autonomousCase = 12;
							m_DrvTurnPID->SetSetpoint(22.337 * REV_IN);
					}
					else
						m_DrvStraightPID->Enable();
				break;
					
				case 12:
					if ((m_armPot->GetAverageVoltage()) >= (ARM_DUMP - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_DUMP + 0.1))
					{
						m_Roller1->Set(-0.8);
						m_Roller2->Set(0.8);
					}
					if(m_timer->HasPeriodPassed(0.5))
					{	m_Roller1->Set(0.);
						m_Roller2->Set(0.);
						m_DrvTurnPID->Disable();
						m_Encoder1->Reset();
						m_Encoder2->Reset();
						m_DrvStraightPID->SetSetpoint(-290*REV_IN);
						m_timer->Stop();
						m_timer->Start();
						m_timer->Reset();
						m_autonomousCase = 15;
					}
					else
						m_DrvTurnPID->Enable();
										
				break;
				
				case 15:
					m_cannonSetPoint = BOTTOMKEY_SHOT + m_cannonOffset;
					m_CannonPID->SetSetpoint(BOTTOMKEY_SHOT + m_cannonOffset);
					m_CannonPID->Enable();
					if(m_timer->HasPeriodPassed(0.5))
					{
					m_AutonArmPos = ARM_BRIDGE;
					}
					else if(m_timer->HasPeriodPassed(1.5))
					{
							m_DrvStraightPID->Disable();
							m_timer->Stop();
							m_AutonArmPos = ARM_BRIDGE;
							m_autonomousCase = 16;
					}
					else
						m_DrvStraightPID->Enable();
				break;
				
				case 16:
					//Auto aim with camera
					if(goDraino == 0)
					{
							cameraAutoAim();
					}
					ArmLight();	
					FireTheCannon();
					m_DrvStraightPID->Disable();
					m_DrvTurnPID->Disable();
					
				break;
				
				case 17:
					//KINECT AUTON CASE DONT BREAK ANYTHING
					m_robotDrive->TankDrive(0., 0.);
					m_AutonArmPos = 0;
					m_HopRoller->Set(Relay::kOff);
					m_Index->Set(Relay::kOff);
				break;
				
				case 18:
					if(!m_bridgeCheck->Get())
					{
						m_autonomousCase = 7;
						m_robotDrive->TankDrive(0., 0.);
					}
					updateRampDrive();
				break;	
				
				case 19:
					updateRampDrive();
					if((m_armPot->GetAverageVoltage()) >= (ARM_GROUND - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_GROUND + 0.1))
					{
						m_Encoder1->Reset();
						m_Encoder2->Reset();	
						m_robotDrive->TankDrive(0., 0.);
						setDriveRampParams(0, 87.6*REV_IN, 0.02, 0.6, 1.0, 35.04*REV_IN, 0);
						m_autonomousCase = 9;
					}
				break;	
				}
			}
		}
        void QuickAllianceBridge()
		{
        	m_autoPeriodicLoops++;

			TeleopSensorUpdater();
			TeleopDriverStationUpdate();

			m_HopRoller->Set(-1.0); 
			if(m_AutonArmPos != 0)
			{
					ArmPosition(m_AutonArmPos);
			}
			else
			{
					m_Arm1->Set(0.0);
					m_Arm2->Set(0.0);
			}
			m_rightHipValue = m_kinect->GetSkeleton().GetHipRight().z;
			m_rightAnkleValue = m_kinect->GetSkeleton().GetAnkleRight().z;
											
			if((m_rightHipValue - m_rightAnkleValue) >= LEG_FORWARD)
			{
					RunKinect();
					m_autonomousCase = 8;
			}
			else 
			{
				switch(m_autonomousCase)
				{
				case 0:
					m_AutonArmPos = ARM_BRIDGE;
					m_Encoder1->Reset();
					m_Encoder2->Reset();
					m_timer->Start();
					m_timer->Reset();
					m_autonomousCase = 1;
				break;
				case 1:
					//Set cannon to speed where it could make a shot going backwards
					m_cannonSetPoint = BOTTOMKEY_SHOT + m_cannonOffset;
					m_CannonPID->SetSetpoint(BOTTOMKEY_SHOT + m_cannonOffset);
					m_CannonPID->Enable();
					//Shift into low gear
					m_armLight->Set(Relay::kForward);
					//Hop Roller runs constantly throughout Autonomous
					m_HopRoller->Set(-1.0);
					
					if(m_timer->HasPeriodPassed(0.5))
					{
							m_HopRoller->Set(-1.0);                 
							m_Index->Set(Relay::kForward);
							m_timer->Stop();
							m_timer->Start();
							m_timer->Reset();
							m_autonomousCase = 2;
					}
				break;
				
				case 2:
					if(m_timer->HasPeriodPassed(1.5))
					{
						backdrive = 0;
						m_armLight->Set(Relay::kOff);
						m_timer->Stop();
						m_timer->Reset();
						m_timer->Start();
						m_DrvTurnPID->SetSetpoint(20.417 * REV_IN);
						m_autonomousCase = 3;
					}
				break;
				
				
				case 3:

					m_cannonSetPoint = -100;
					m_CannonDemandedSpeed=0.;
					m_CannonPID->Disable();
					m_Index->Set(Relay::kOff);
		
					if (m_timer->HasPeriodPassed(0.5))
					{
							m_DrvTurnPID->Disable();
							m_DrvStraightPID->SetSetpoint(193.15 * REV_IN);
							m_Encoder1->Reset();
							m_Encoder2->Reset();
							m_timer->Stop();
							m_timer->Start();
							m_timer->Reset();
							m_autonomousCase = 4;
					}
					else
						m_DrvTurnPID->Enable();
				break;
				
				case 4:
					if(m_timer->HasPeriodPassed(1.3))
					{
						m_DrvStraightPID->Disable();
						m_Encoder1->Reset();
						m_Encoder2->Reset();
						m_timer->Stop();
						m_timer->Start();
						m_timer->Reset();
						m_DrvTurnPID->SetSetpoint(-35.973 * REV_IN);
						m_autonomousCase = 5;
					}
					else
						m_DrvStraightPID->Enable();
				break;
				
				case 5:
					if (m_timer->HasPeriodPassed(0.5))
					{
							m_DrvTurnPID->Disable();
							m_Encoder1->Reset();
							m_Encoder2->Reset();
							m_timer->Stop();
							m_timer->Start();
							m_timer->Reset();
							m_DrvStraightPID->SetSetpoint(100.76 * REV_IN);
							m_autonomousCase = 6;
					}
					else
						m_DrvTurnPID->Enable();
						
				break;
				
				case 6:
					if(m_timer->HasPeriodPassed(1.0))
					{
						m_DrvStraightPID->Disable();
						m_Encoder1->Reset();
						m_Encoder2->Reset();
						m_timer->Stop();
						m_autonomousCase = 7;
						m_armPosition->SetGains(1.0, kI, kD);
					}
					else
						m_DrvStraightPID->Enable();
				break;
				
				case 7:
					//Bring arm down to lower bridge and run on the bridge
					m_AutonArmPos = ARM_BRIDGE;
					//Once bridge is lowered reset encoders and set next driving cycle up bridge
					if ((m_armPot->GetAverageVoltage()) >= (ARM_BRIDGE - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_BRIDGE + 0.1))
					{
						if(!m_bridgeCheck->Get())
						{
							m_armTimer = 0;
							m_autonomousCase = 8;
						}
						else
						{	//Jump to extra routine if bridge is not found.
							m_Encoder1->Reset();
							m_Encoder2->Reset();
							setDriveRampParams(0, 87.6*REV_IN, 0.00, 0.4, 0.4, 10*REV_IN, 0);
							m_armTimer = 0;
							m_autonomousCase = 19;
						}
					}
				break;
				
				case 8:
					//Once bridge is lowered reset encoders and set next driving cycle up bridge
					m_AutonArmPos = ARM_GROUND;
					if ((m_armPot->GetAverageVoltage()) >= (ARM_GROUND - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_GROUND + 0.1))
					{
							m_Encoder1->Reset();
							m_Encoder2->Reset();
							m_autoPeriodicLoops = 0;
							m_autonomousCase = 9;
							setDriveRampParams(0, 87.6*REV_IN, 0.02, 0.6, 1.0, 35.04*REV_IN, 0);
					}
					m_armTimer++;
					if(m_armTimer > 30)
					{
						setDriveRampParams(0, -87.6*REV_IN, 0.00, 0.55, 0.55, 10*REV_IN, 0);
						m_autonomousCase = 20;
					}
				break;
				
				case 9:
					//Once up the bridge change arm gains, reset encoders and set next short trip
					if (updateRampDrive())
					{
							m_armPosition->SetGains(3.0, kI, kD);
							//Lift arm to ball dump position
							m_Encoder1->Reset();
							m_Encoder2->Reset();
							m_RollerPulseOut = 0;
							m_autonomousCase = 10;
							m_autoPeriodicLoops = 0;
					}
				break;
				
				case 10:
					if(m_autoPeriodicLoops <= 15)
					{
						m_Roller1->Set(-0.8);
						m_Roller2->Set(0.8);
					}
					else if(m_autoPeriodicLoops > 15 && m_autoPeriodicLoops <= 30)
					{
						if(m_RollerPulseOut < 5)
						{
							m_Roller1->Set(0.);
							m_Roller2->Set(0.);
							m_RollerPulseOut++;
						}
						else if(m_RollerPulseOut < 8)
						{
							m_Roller1->Set(0.8);
							m_Roller2->Set(-0.8);
							m_RollerPulseOut++;
						}
						else
						{
							m_Roller1->Set(0.);
							m_Roller2->Set(0.);
						}
						
					}
					else if(m_autoPeriodicLoops > 30 && m_autoPeriodicLoops <= 40)
					{
						m_AutonArmPos = ARM_DUMP;
					}
					else if(m_autoPeriodicLoops > 40)
					{
						m_autoPeriodicLoops = 0;
						m_armDown = 0;
						m_DrvStraightPID->SetSetpoint(-87.60 * REV_IN);
						m_Encoder1->Reset();
						m_Encoder2->Reset();
						m_timer->Start();
						m_timer->Reset();
						m_autonomousCase = 11;
					}
				break;
				
				case 11:
					if (m_timer->HasPeriodPassed(0.5))
					{
							m_DrvStraightPID->Disable();
							m_timer->Stop();
							m_timer->Start();
							m_timer->Reset();
							m_Encoder1->Reset();
							m_Encoder2->Reset();
							m_autonomousCase = 12;
							m_DrvTurnPID->SetSetpoint(30 * REV_IN);
					}
					else
						m_DrvStraightPID->Enable();
				break;
					
				case 12:
					if ((m_armPot->GetAverageVoltage()) >= (ARM_DUMP - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_DUMP + 0.1))
					{
						m_Roller1->Set(-0.8);
						m_Roller2->Set(0.8);
					}
					if(m_timer->HasPeriodPassed(0.5))
					{	m_Roller1->Set(0.);
						m_Roller2->Set(0.);
						m_DrvTurnPID->Disable();
						m_Encoder1->Reset();
						m_Encoder2->Reset();
						m_DrvStraightPID->SetSetpoint(-260*REV_IN);
						m_timer->Stop();
						m_timer->Start();
						m_timer->Reset();
						m_autonomousCase = 15;
					}
					else
						m_DrvTurnPID->Enable();
										
				break;
				
				case 15:
					m_cannonSetPoint = BOTTOMKEY_SHOT + m_cannonOffset;
					m_CannonPID->SetSetpoint(BOTTOMKEY_SHOT + m_cannonOffset);
					m_CannonPID->Enable();
					if(m_timer->HasPeriodPassed(0.5))
					{
					m_AutonArmPos = ARM_BRIDGE;
					}
					else if(m_timer->HasPeriodPassed(1.5))
					{
							m_DrvStraightPID->Disable();
							m_timer->Stop();
							m_AutonArmPos = ARM_BRIDGE;
							m_autonomousCase = 16;
					}
					else
						m_DrvStraightPID->Enable();
				break;
				
				case 16:
					//Auto aim with camera
					if(goDraino == 0)
					{
							cameraAutoAim();
					}
					ArmLight();	
					FireTheCannon();
					m_DrvStraightPID->Disable();
					m_DrvTurnPID->Disable();
					
				break;
				
				case 17:
					//KINECT AUTON CASE DONT BREAK ANYTHING
					m_robotDrive->TankDrive(0., 0.);
					m_AutonArmPos = 0;
					m_HopRoller->Set(Relay::kOff);
					m_Index->Set(Relay::kOff);
				break;
				
				case 19:
					if(!m_bridgeCheck->Get())
					{
						m_autonomousCase = 8;
						m_robotDrive->TankDrive(0., 0.);
					}
					updateRampDrive();
				break;	
				
				case 20:
					updateRampDrive();
					if((m_armPot->GetAverageVoltage()) >= (ARM_GROUND - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_GROUND + 0.1))
					{
						m_Encoder1->Reset();
						m_Encoder2->Reset();	
						m_robotDrive->TankDrive(0., 0.);
						setDriveRampParams(0, 87.6*REV_IN, 0.02, 0.6, 1.0, 35.04*REV_IN, 0);
						m_autonomousCase = 9;
					}
				break;	
				}
			}
		}
        void QuickTopofKey()
		{
        	m_autoPeriodicLoops++;

			TeleopSensorUpdater();
			TeleopDriverStationUpdate();

			m_HopRoller->Set(-1.0); 
			if(m_AutonArmPos != 0)
			{
					ArmPosition(m_AutonArmPos);
			}
			else
			{
					m_Arm1->Set(0.0);
					m_Arm2->Set(0.0);
			}
			m_rightHipValue = m_kinect->GetSkeleton().GetHipRight().z;
			m_rightAnkleValue = m_kinect->GetSkeleton().GetAnkleRight().z;
											
			if((m_rightHipValue - m_rightAnkleValue) >= LEG_FORWARD)
			{
					RunKinect();
					m_autonomousCase = 8;
			}
			else 
			{
				switch(m_autonomousCase)
				{
				case 0:
					m_AutonArmPos = ARM_BRIDGE;
					m_Encoder1->Reset();
					m_Encoder2->Reset();
					m_timer->Start();
					m_timer->Reset();
					m_autonomousCase = 1;
				break;
				case 1:
					//Set cannon to speed where it could make a shot going backwards
					m_cannonSetPoint = TOPKEY_SHOT + m_cannonOffset;
					m_CannonPID->SetSetpoint(TOPKEY_SHOT + m_cannonOffset);
					m_CannonPID->Enable();
					//Shift into low gear
					m_armLight->Set(Relay::kForward);
					//Hop Roller runs constantly throughout Autonomous
					m_HopRoller->Set(-1.0);
					
					if(m_timer->HasPeriodPassed(0.5))
					{
							m_HopRoller->Set(-1.0);                 
							m_Index->Set(Relay::kForward);
							m_timer->Stop();
							m_timer->Start();
							m_timer->Reset();
							m_autonomousCase = 2;
					}
				break;
				
				case 2:
					if(m_timer->HasPeriodPassed(1.5))
					{
						m_armLight->Set(Relay::kOff);
						m_timer->Stop();
						m_timer->Start();
						m_timer->Reset();
						m_DrvStraightPID->SetSetpoint(56*2 * REV_IN);
						m_autonomousCase = 3;
					}
				break;
				
				case 3:

					m_cannonSetPoint = -100;
					m_CannonDemandedSpeed=0.;
					m_CannonPID->Disable();
					m_Index->Set(Relay::kOff);
		
					if (m_timer->HasPeriodPassed(1.1))
					{
							m_DrvStraightPID->Disable();
							m_Encoder1->Reset();
							m_Encoder2->Reset();
							m_timer->Stop();
							m_autonomousCase = 4;
					}
					else
						m_DrvStraightPID->Enable();
				break;
				
			   case 4:
					//Turn off cannon
					m_cannonSetPoint = -100;
					m_CannonDemandedSpeed=0.;
					m_CannonPID->Disable();
					m_Index->Set(Relay::kOff);
					//Bring arm down to lower bridge and run on the bridge
					m_AutonArmPos = ARM_BRIDGE;
					//Once bridge is lowered reset encoders and set next driving cycle up bridge
					if ((m_armPot->GetAverageVoltage()) >= (ARM_BRIDGE - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_BRIDGE + 0.1))
					{
							if(!m_bridgeCheck->Get())
							{
									m_armTimer = 0;
									m_autonomousCase = 5;
							}
							else
							{       //Jump to extra routine if bridge is not found.
									m_Encoder1->Reset();
									m_Encoder2->Reset();
									setDriveRampParams(0, 87.6*REV_IN, 0.00, 0.4, 0.4, 10*REV_IN, 0);
									m_armTimer = 0;
									m_autonomousCase = 11;
							}
					}
					break;
				case 5:
						//Once bridge is lowered reset encoders and set next driving cycle up bridge
						m_AutonArmPos = ARM_GROUND;
						if ((m_armPot->GetAverageVoltage()) >= (ARM_GROUND - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_GROUND + 0.1))
						{
										m_Encoder1->Reset();
										m_Encoder2->Reset();
										m_autoPeriodicLoops = 0;
										m_autonomousCase = 6;
										setDriveRampParams(0, 87.6*REV_IN, 0.02, 0.6, 1.0, 35.04*REV_IN, 0);
						}
						m_armTimer++;
						if(m_armTimer > 30)
						{
								setDriveRampParams(0, -87.6*REV_IN, 0.00, 0.55, 0.55, 10*REV_IN, 0);
								m_autonomousCase = 12;
						}
						break;
				case 6:
						//Once up the bridge change arm gains, reset encoders and set next short trip
						if (updateRampDrive())
						{
										//m_robotDrive->TankDrive(0.3 , 0.3);  //Not really needed
										m_armPosition->SetGains(3.0, kI, kD);
										//Lift arm to ball dump position
										m_Encoder1->Reset();
										m_Encoder2->Reset();
										setDriveRampParams(0, -160*REV_IN, 0.010, 0.7, 1.0, 130*REV_IN, 0);
										m_RollerPulseOut = 0;
										m_autonomousCase = 7;
										m_autoPeriodicLoops = 0;
						}
						break;
				case 7:
						if(m_autoPeriodicLoops <= 15)
						{
								m_Roller1->Set(-0.8);
								m_Roller2->Set(0.8);
						}
						else if(m_autoPeriodicLoops > 15 && m_autoPeriodicLoops <= 30)
						{
								if(m_RollerPulseOut < 5)
								{
										m_Roller1->Set(0.);
										m_Roller2->Set(0.);
										m_RollerPulseOut++;
								}
								else if(m_RollerPulseOut < 8)
								{
										m_Roller1->Set(0.8);
										m_Roller2->Set(-0.8);
										m_RollerPulseOut++;
								}
								else
								{
										m_Roller1->Set(0.);
										m_Roller2->Set(0.);
								}
								
						}
						else if(m_autoPeriodicLoops > 30 && m_autoPeriodicLoops <= 40)
						{
								m_AutonArmPos = ARM_DUMP;
						}
						else if(m_autoPeriodicLoops > 40)
						{
								m_autonomousCase = 8;
								m_autoPeriodicLoops = 0;
								m_armDown = 0;
						}
						break;
	
				case 8:
						//Set cannon speed to ready for key shot
						m_cannonSetPoint = BOTTOMKEY_SHOT + m_cannonOffset;
						m_CannonPID->SetSetpoint(BOTTOMKEY_SHOT + m_cannonOffset);
						m_CannonPID->Enable();
						/*if (m_autoPeriodicLoops > 30)
						{
										m_AutonArmPos = ARM_DUMP;
						}*/
						//Once arm reaches ball dump position, drop balls
						if ((m_armPot->GetAverageVoltage()) >= (ARM_DUMP - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_DUMP + 0.1))
						{
										m_Roller1->Set(-0.8);
										m_Roller2->Set(0.8);
										m_Light->Set(Relay::kForward);
										m_autoPeriodicLoops = 0;
										m_armDown = 1;
						}
						//if(m_autoPeriodicLoops >= 10 && m_armDown == 1)
								//m_AutonArmPos = ARM_BRIDGE;
						//Once robot is at key, stop the bot, put arm down to ball pick up pos
						if (updateRampDrive())
						{
										//m_robotDrive->TankDrive(0.0, 0.0);
										m_armPosition->SetGains(1.0, kI, kD);
										//m_AutonArmPos = ARM_BRIDGE;
										goDraino = 0;
										m_autoPeriodicLoops = 0;
										m_autonomousCase = 9;
										//Turn off rollers
										m_Roller1->Set(0.0);
										m_Roller2->Set(0.0);
						}
						
						break;
				case 9:
								m_AutonArmPos = ARM_BRIDGE;
								//Auto aim with camera
								if(goDraino == 0)
								{
												cameraAutoAim();
								}
								//Fire if shot is in range
								/*if(m_autoPeriodicLoops >=75)
								{               
										goDraino = 1;
										m_DrvTurnPIDRate->Disable();
								}*/
								ArmLight();     
								FireTheCannon();
				break;
				case 10:
								//KINECT AUTON CASE DONT BREAK ANYTHING
								m_robotDrive->TankDrive(0., 0.);
								m_AutonArmPos = 0;
								m_HopRoller->Set(Relay::kOff);
								m_Index->Set(Relay::kOff);
				break;
				case 11:
						if(!m_bridgeCheck->Get())
						{
								m_autonomousCase = 5;
								m_robotDrive->TankDrive(0., 0.);
						}
						updateRampDrive();
				break;  
				case 12:
						updateRampDrive();
						if((m_armPot->GetAverageVoltage()) >= (ARM_GROUND - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_GROUND + 0.1))
						{
								m_Encoder1->Reset();
								m_Encoder2->Reset();    
								m_robotDrive->TankDrive(0., 0.);
								setDriveRampParams(0, 87.6*REV_IN, 0.02, 0.6, 1.0, 35.04*REV_IN, 0);
								m_autonomousCase = 6;
						}
				break;  

				}
			}
		}
        void QuickMiddleofKey()
		{
        	m_autoPeriodicLoops++;

			TeleopSensorUpdater();
			TeleopDriverStationUpdate();

			m_HopRoller->Set(-1.0); 
			if(m_AutonArmPos != 0)
			{
					ArmPosition(m_AutonArmPos);
			}
			else
			{
					m_Arm1->Set(0.0);
					m_Arm2->Set(0.0);
			}
			m_rightHipValue = m_kinect->GetSkeleton().GetHipRight().z;
			m_rightAnkleValue = m_kinect->GetSkeleton().GetAnkleRight().z;
											
			if((m_rightHipValue - m_rightAnkleValue) >= LEG_FORWARD)
			{
					RunKinect();
					m_autonomousCase = 8;
			}
			else 
			{
				switch(m_autonomousCase)
				{
				case 0:
					m_AutonArmPos = ARM_BRIDGE;
					m_Encoder1->Reset();
					m_Encoder2->Reset();
					m_timer->Start();
					m_timer->Reset();
					m_autonomousCase = 1;
				break;
				case 1:
					//Set cannon to speed where it could make a shot going backwards
					m_cannonSetPoint = (BOTTOMKEY_SHOT + 1) + m_cannonOffset;
					m_CannonPID->SetSetpoint((BOTTOMKEY_SHOT + 1) + m_cannonOffset);
					m_CannonPID->Enable();
					//Shift into low gear
					m_armLight->Set(Relay::kForward);
					//Hop Roller runs constantly throughout Autonomous
					m_HopRoller->Set(-1.0);
					
					if(m_timer->HasPeriodPassed(0.5))
					{
							m_HopRoller->Set(-1.0);                 
							m_Index->Set(Relay::kForward);
							m_timer->Stop();
							m_timer->Start();
							m_timer->Reset();
							m_autonomousCase = 2;
					}
				break;
				
				case 2:
					if(m_timer->HasPeriodPassed(1.5))
					{
						m_armLight->Set(Relay::kOff);
						m_timer->Stop();
						m_timer->Start();
						m_timer->Reset();
						m_DrvStraightPID->SetSetpoint(93*2 * REV_IN);
						m_autonomousCase = 3;
					}
				break;
				
				case 3:

					m_cannonSetPoint = -100;
					m_CannonDemandedSpeed=0.;
					m_CannonPID->Disable();
					m_Index->Set(Relay::kOff);
		
					if (m_timer->HasPeriodPassed(1.6))
					{
							m_DrvStraightPID->Disable();
							m_Encoder1->Reset();
							m_Encoder2->Reset();
							m_timer->Stop();
							m_autonomousCase = 4;
					}
					else
						m_DrvStraightPID->Enable();
				break;
				
			   case 4:
					//Turn off cannon
					m_cannonSetPoint = -100;
					m_CannonDemandedSpeed=0.;
					m_CannonPID->Disable();
					m_Index->Set(Relay::kOff);
					//Bring arm down to lower bridge and run on the bridge
					m_AutonArmPos = ARM_BRIDGE;
					//Once bridge is lowered reset encoders and set next driving cycle up bridge
					if ((m_armPot->GetAverageVoltage()) >= (ARM_BRIDGE - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_BRIDGE + 0.1))
					{
							if(!m_bridgeCheck->Get())
							{
									m_armTimer = 0;
									m_autonomousCase = 5;
							}
							else
							{       //Jump to extra routine if bridge is not found.
									m_Encoder1->Reset();
									m_Encoder2->Reset();
									setDriveRampParams(0, 87.6*REV_IN, 0.00, 0.4, 0.4, 10*REV_IN, 0);
									m_armTimer = 0;
									m_autonomousCase = 11;
							}
					}
					break;
				case 5:
						//Once bridge is lowered reset encoders and set next driving cycle up bridge
						m_AutonArmPos = ARM_GROUND;
						if ((m_armPot->GetAverageVoltage()) >= (ARM_GROUND - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_GROUND + 0.1))
						{
										m_Encoder1->Reset();
										m_Encoder2->Reset();
										m_autoPeriodicLoops = 0;
										m_autonomousCase = 6;
										setDriveRampParams(0, 87.6*REV_IN, 0.02, 0.6, 1.0, 35.04*REV_IN, 0);
						}
						m_armTimer++;
						if(m_armTimer > 30)
						{
								setDriveRampParams(0, -87.6*REV_IN, 0.00, 0.55, 0.55, 10*REV_IN, 0);
								m_autonomousCase = 12;
						}
						break;
				case 6:
						//Once up the bridge change arm gains, reset encoders and set next short trip
						if (updateRampDrive())
						{
										//m_robotDrive->TankDrive(0.3 , 0.3);  //Not really needed
										m_armPosition->SetGains(3.0, kI, kD);
										//Lift arm to ball dump position
										m_Encoder1->Reset();
										m_Encoder2->Reset();
										setDriveRampParams(0, -160*REV_IN, 0.010, 0.7, 1.0, 130*REV_IN, 0);
										m_RollerPulseOut = 0;
										m_autonomousCase = 7;
										m_autoPeriodicLoops = 0;
						}
						break;
				case 7:
						if(m_autoPeriodicLoops <= 15)
						{
								m_Roller1->Set(-0.8);
								m_Roller2->Set(0.8);
						}
						else if(m_autoPeriodicLoops > 15 && m_autoPeriodicLoops <= 30)
						{
								if(m_RollerPulseOut < 5)
								{
										m_Roller1->Set(0.);
										m_Roller2->Set(0.);
										m_RollerPulseOut++;
								}
								else if(m_RollerPulseOut < 8)
								{
										m_Roller1->Set(0.8);
										m_Roller2->Set(-0.8);
										m_RollerPulseOut++;
								}
								else
								{
										m_Roller1->Set(0.);
										m_Roller2->Set(0.);
								}
								
						}
						else if(m_autoPeriodicLoops > 30 && m_autoPeriodicLoops <= 40)
						{
								m_AutonArmPos = ARM_DUMP;
						}
						else if(m_autoPeriodicLoops > 40)
						{
								m_autonomousCase = 8;
								m_autoPeriodicLoops = 0;
								m_armDown = 0;
						}
						break;
	
				case 8:
						//Set cannon speed to ready for key shot
						m_cannonSetPoint = (BOTTOMKEY_SHOT + 1) + m_cannonOffset;
						m_CannonPID->SetSetpoint((BOTTOMKEY_SHOT + 1) + m_cannonOffset);
						m_CannonPID->Enable();
						/*if (m_autoPeriodicLoops > 30)
						{
										m_AutonArmPos = ARM_DUMP;
						}*/
						//Once arm reaches ball dump position, drop balls
						if ((m_armPot->GetAverageVoltage()) >= (ARM_DUMP - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_DUMP + 0.1))
						{
										m_Roller1->Set(-0.8);
										m_Roller2->Set(0.8);
										m_Light->Set(Relay::kForward);
										m_autoPeriodicLoops = 0;
										m_armDown = 1;
						}
						//if(m_autoPeriodicLoops >= 40 && m_armDown == 1)
								//m_AutonArmPos = ARM_BRIDGE;
						//Once robot is at key, stop the bot, put arm down to ball pick up pos
						if (updateRampDrive())
						{
										//m_robotDrive->TankDrive(0.0, 0.0);
										m_armPosition->SetGains(1.0, kI, kD);
										//m_AutonArmPos = ARM_BRIDGE;
										goDraino = 0;
										m_autoPeriodicLoops = 0;
										m_autonomousCase = 9;
										//Turn off rollers
										m_Roller1->Set(0.0);
										m_Roller2->Set(0.0);
						}
						
						break;
				case 9:
								m_AutonArmPos = ARM_BRIDGE;
								//Auto aim with camera
								if(goDraino == 0)
								{
												cameraAutoAim();
								}
								//Fire if shot is in range
								/*if(m_autoPeriodicLoops >=75)
								{               
										goDraino = 1;
										m_DrvTurnPIDRate->Disable();
								}*/
								ArmLight();     
								FireTheCannon();
				break;
				case 10:
								//KINECT AUTON CASE DONT BREAK ANYTHING
								m_robotDrive->TankDrive(0., 0.);
								m_AutonArmPos = 0;
								m_HopRoller->Set(Relay::kOff);
								m_Index->Set(Relay::kOff);
				break;
				case 11:
						if(!m_bridgeCheck->Get())
						{
								m_autonomousCase = 5;
								m_robotDrive->TankDrive(0., 0.);
						}
						updateRampDrive();
				break;  
				case 12:
						updateRampDrive();
						if((m_armPot->GetAverageVoltage()) >= (ARM_GROUND - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_GROUND + 0.1))
						{
								m_Encoder1->Reset();
								m_Encoder2->Reset();    
								m_robotDrive->TankDrive(0., 0.);
								setDriveRampParams(0, 87.6*REV_IN, 0.02, 0.6, 1.0, 35.04*REV_IN, 0);
								m_autonomousCase = 6;
						}
				break;  

				}
			}
		}
        void QuickMiddleofKeyV2()
	   {
        	backdrive_spd = -0.75;
        	m_autoPeriodicLoops++;

			TeleopSensorUpdater();
			TeleopDriverStationUpdate();

			m_HopRoller->Set(-1.0); 
			if(m_AutonArmPos != 0)
			{
					ArmPosition(m_AutonArmPos);
			}
			else
			{
					m_Arm1->Set(0.0);
					m_Arm2->Set(0.0);
			}
			switch(m_autonomousCase)
			{
			case 0:
				m_AutonArmPos = ARM_BRIDGE;
				m_Encoder1->Reset();
				m_Encoder2->Reset();
				m_timer->Start();
				m_timer->Reset();
				m_autonomousCase = 1;
			break;
			case 1:
				//Set cannon to speed where it could make a shot going backwards
				m_cannonSetPoint = (BOTTOMKEY_SHOT + 1) + m_cannonOffset;
				m_CannonPID->SetSetpoint((BOTTOMKEY_SHOT + 1) + m_cannonOffset);
				m_CannonPID->Enable();
				//Shift into low gear
				m_armLight->Set(Relay::kForward);
				//Hop Roller runs constantly throughout Autonomous
				m_HopRoller->Set(-1.0);
				
				if(m_timer->HasPeriodPassed(0.5))
				{
						m_HopRoller->Set(-1.0);                 
						m_Index->Set(Relay::kForward);
						m_timer->Stop();
						m_timer->Start();
						m_timer->Reset();
						m_autonomousCase = 2;
				}
			break;
			
			case 2:
				if(m_timer->HasPeriodPassed(1.5))
				{
					m_armLight->Set(Relay::kOff);
					m_timer->Stop();
					m_timer->Start();
					m_timer->Reset();
					setDriveRampParams(0, (63)*REV_IN*2, 0.015, 0.5, 0.9, 10*REV_IN*2, 0);
					m_autonomousCase = 3;
				}
			break;

			case 3:
					//Once you reach bridge change arm pos gains
					if (eStopUpdateRampDrive())
					{
									m_autonomousCase = 4;
									m_armPosition->SetGains(1.0, kI, kD);
					}
					break;
					case 4:
							//Turn off cannon
							m_cannonSetPoint = -100;
							m_CannonDemandedSpeed=0.;
							m_CannonPID->Disable();
							m_Index->Set(Relay::kOff);
							//Bring arm down to lower bridge and run on the bridge
							m_AutonArmPos = ARM_BRIDGE;
							//Once bridge is lowered reset encoders and set next driving cycle up bridge
							if ((m_armPot->GetAverageVoltage()) >= (ARM_BRIDGE - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_BRIDGE + 0.1))
							{
									if(!m_bridgeCheck->Get())
									{
											m_armTimer = 0;
											m_autonomousCase = 5;
									}
									else
									{       //Jump to extra routine if bridge is not found.
											m_Encoder1->Reset();
											m_Encoder2->Reset();
											setDriveRampParams(0, 87.6*REV_IN, 0.00, 0.4, 0.4, 10*REV_IN, 0);
											m_armTimer = 0;
											m_autonomousCase = 10;
									}
							}
							break;
					case 5:
							//Once bridge is lowered reset encoders and set next driving cycle up bridge
							m_AutonArmPos = ARM_GROUND;
							if ((m_armPot->GetAverageVoltage()) >= (ARM_GROUND - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_GROUND + 0.1))
							{
											m_Encoder1->Reset();
											m_Encoder2->Reset();
											m_autoPeriodicLoops = 0;
											m_autonomousCase = 6;
											setDriveRampParams(0, 87.6*REV_IN, 0.02, 0.6, 1.0, 35.04*REV_IN, 0);
							}
							m_armTimer++;
							if(m_armTimer > 30)
							{
									setDriveRampParams(0, -87.6*REV_IN, 0.00, 0.55, 0.55, 10*REV_IN, 0);
									m_autonomousCase = 11;
							}
							break;
					case 6:
							//Once up the bridge change arm gains, reset encoders and set next short trip
							if (updateRampDrive())
							{
											//m_robotDrive->TankDrive(0.3 , 0.3);  //Not really needed
											m_armPosition->SetGains(3.0, kI, kD);
											//Lift arm to ball dump position
											m_Encoder1->Reset();
											m_Encoder2->Reset();
											setDriveRampParams(0, -210*REV_IN, 0.010, 0.4, 1.0, 100*REV_IN, 0);
											m_RollerPulseOut = 0;
											m_autonomousCase = 7;
											m_autoPeriodicLoops = 0;
							}
							break;
					case 7:
							if(m_autoPeriodicLoops <= 15)
							{
									m_Roller1->Set(-0.8);
									m_Roller2->Set(0.8);
							}
							else if(m_autoPeriodicLoops > 15 && m_autoPeriodicLoops <= 30)
							{
									if(m_RollerPulseOut < 5)
									{
											m_Roller1->Set(0.);
											m_Roller2->Set(0.);
											m_RollerPulseOut++;
									}
									else if(m_RollerPulseOut < 8)
									{
											m_Roller1->Set(0.8);
											m_Roller2->Set(-0.8);
											m_RollerPulseOut++;
									}
									else
									{
											m_Roller1->Set(0.);
											m_Roller2->Set(0.);
									}
									
							}
							else if(m_autoPeriodicLoops > 30 && m_autoPeriodicLoops <= 40)
							{
									m_AutonArmPos = ARM_DUMP;
							}
							else if(m_autoPeriodicLoops > 40)
							{
									m_autonomousCase = 8;
									m_autoPeriodicLoops = 0;
									m_armDown = 0;
							}
							break;

					case 8:
							//Set cannon speed to ready for key shot
							m_cannonSetPoint = (BOTTOMKEY_SHOT + 1) + m_cannonOffset;
							m_CannonPID->SetSetpoint((BOTTOMKEY_SHOT + 1) + m_cannonOffset);
							m_CannonPID->Enable();
							/*if (m_autoPeriodicLoops > 30)
							{
											m_AutonArmPos = ARM_DUMP;
							}*/
							//Once arm reaches ball dump position, drop balls
							if ((m_armPot->GetAverageVoltage()) >= (ARM_DUMP - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_DUMP + 0.1))
							{
											m_Roller1->Set(-0.8);
											m_Roller2->Set(0.8);
											m_Light->Set(Relay::kForward);
											m_autoPeriodicLoops = 0;
											m_armDown = 1;
							}
							if(m_autoPeriodicLoops >= 10 && m_armDown == 1)
									m_AutonArmPos = ARM_BRIDGE;
							//Once robot is at key, stop the bot, put arm down to ball pick up pos
							if (updateRampDrive())
							{
											//m_robotDrive->TankDrive(0.0, 0.0);
											m_armPosition->SetGains(1.0, kI, kD);
											m_AutonArmPos = ARM_BRIDGE;
											goDraino = 0;
											m_autoPeriodicLoops = 0;
											m_autonomousCase = 9;
											//Turn off rollers
											m_Roller1->Set(0.0);
											m_Roller2->Set(0.0);
							}
							
							break;
					case 9:
									//Auto aim with camera
									if(goDraino == 0)
									{
													cameraAutoAim();
									}
									//Fire if shot is in range
									/*if(m_autoPeriodicLoops >=75)
									{               
											goDraino = 1;
											m_DrvTurnPIDRate->Disable();
									}*/
									ArmLight();     
									FireTheCannon();
					break;
					case 10:
							if(!m_bridgeCheck->Get())
							{
									m_autonomousCase = 5;
									m_robotDrive->TankDrive(0., 0.);
							}
							updateRampDrive();
					break;  
					case 11:
							updateRampDrive();
							if((m_armPot->GetAverageVoltage()) >= (ARM_GROUND - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_GROUND + 0.1))
							{
									m_Encoder1->Reset();
									m_Encoder2->Reset();    
									m_robotDrive->TankDrive(0., 0.);
									setDriveRampParams(0, 87.6*REV_IN, 0.02, 0.6, 1.0, 35.04*REV_IN, 0);
									m_autonomousCase = 6;
							}
					break;  
			}
	   }

        
        void QuickBottomofKey()
		{
			m_autoPeriodicLoops++;

			TeleopSensorUpdater();
			TeleopDriverStationUpdate();

			m_HopRoller->Set(-1.0); 
			if(m_AutonArmPos != 0)
			{
					ArmPosition(m_AutonArmPos);
			}
			else
			{
					m_Arm1->Set(0.0);
					m_Arm2->Set(0.0);
			}
			m_rightHipValue = m_kinect->GetSkeleton().GetHipRight().z;
			m_rightAnkleValue = m_kinect->GetSkeleton().GetAnkleRight().z;
											
			if((m_rightHipValue - m_rightAnkleValue) >= LEG_FORWARD)
			{
					RunKinect();
					m_autonomousCase = 8;
			}
			else 
			{
				switch(m_autonomousCase)
				{
				case 0:
					m_AutonArmPos = ARM_BRIDGE;
					m_Encoder1->Reset();
					m_Encoder2->Reset();
					m_timer->Start();
					m_timer->Reset();
					m_autonomousCase = 1;
				break;
				case 1:
					//Set cannon to speed where it could make a shot going backwards
					m_cannonSetPoint = (BOTTOMKEY_SHOT + 1) + m_cannonOffset;
					m_CannonPID->SetSetpoint((BOTTOMKEY_SHOT + 1) + m_cannonOffset);
					m_CannonPID->Enable();
					//Shift into low gear
					m_armLight->Set(Relay::kForward);
					//Hop Roller runs constantly throughout Autonomous
					m_HopRoller->Set(-1.0);
					
					if(m_timer->HasPeriodPassed(0.5))
					{
							m_HopRoller->Set(-1.0);                 
							m_Index->Set(Relay::kForward);
							m_timer->Stop();
							m_timer->Start();
							m_timer->Reset();
							m_autonomousCase = 2;
					}
				break;
				
				case 2:
					if(m_timer->HasPeriodPassed(1.5))
					{
						m_armLight->Set(Relay::kOff);
						m_timer->Stop();
						m_timer->Start();
						m_timer->Reset();
						m_DrvStraightPID->SetSetpoint(112*2 * REV_IN);
						m_autonomousCase = 3;
					}
				break;
				
				case 3:

					m_cannonSetPoint = -100;
					m_CannonDemandedSpeed=0.;
					m_CannonPID->Disable();
					m_Index->Set(Relay::kOff);
		
					if (m_timer->HasPeriodPassed(1.6))
					{
							m_DrvStraightPID->Disable();
							m_Encoder1->Reset();
							m_Encoder2->Reset();
							m_timer->Stop();
							m_autonomousCase = 4;
					}
					else
						m_DrvStraightPID->Enable();
				break;
				
			   case 4:
					//Turn off cannon
					m_cannonSetPoint = -100;
					m_CannonDemandedSpeed=0.;
					m_CannonPID->Disable();
					m_Index->Set(Relay::kOff);
					//Bring arm down to lower bridge and run on the bridge
					m_AutonArmPos = ARM_BRIDGE;
					//Once bridge is lowered reset encoders and set next driving cycle up bridge
					if ((m_armPot->GetAverageVoltage()) >= (ARM_BRIDGE - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_BRIDGE + 0.1))
					{
							if(!m_bridgeCheck->Get())
							{
									m_armTimer = 0;
									m_autonomousCase = 5;
							}
							else
							{       //Jump to extra routine if bridge is not found.
									m_Encoder1->Reset();
									m_Encoder2->Reset();
									setDriveRampParams(0, 87.6*REV_IN, 0.00, 0.4, 0.4, 10*REV_IN, 0);
									m_armTimer = 0;
									m_autonomousCase = 11;
							}
					}
					break;
				case 5:
						//Once bridge is lowered reset encoders and set next driving cycle up bridge
						m_AutonArmPos = ARM_GROUND;
						if ((m_armPot->GetAverageVoltage()) >= (ARM_GROUND - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_GROUND + 0.1))
						{
										m_Encoder1->Reset();
										m_Encoder2->Reset();
										m_autoPeriodicLoops = 0;
										m_autonomousCase = 6;
										setDriveRampParams(0, 87.6*REV_IN, 0.02, 0.6, 1.0, 35.04*REV_IN, 0);
						}
						m_armTimer++;
						if(m_armTimer > 30)
						{
								setDriveRampParams(0, -87.6*REV_IN, 0.00, 0.55, 0.55, 10*REV_IN, 0);
								m_autonomousCase = 12;
						}
						break;
				case 6:
						//Once up the bridge change arm gains, reset encoders and set next short trip
						if (updateRampDrive())
						{
										//m_robotDrive->TankDrive(0.3 , 0.3);  //Not really needed
										m_armPosition->SetGains(3.0, kI, kD);
										//Lift arm to ball dump position
										m_Encoder1->Reset();
										m_Encoder2->Reset();
										setDriveRampParams(0, -190*REV_IN, 0.010, 0.7, 1.0, 100*REV_IN, 0);
										m_RollerPulseOut = 0;
										m_autonomousCase = 7;
										m_autoPeriodicLoops = 0;
						}
						break;
				case 7:
						if(m_autoPeriodicLoops <= 15)
						{
								m_Roller1->Set(-0.8);
								m_Roller2->Set(0.8);
						}
						else if(m_autoPeriodicLoops > 15 && m_autoPeriodicLoops <= 30)
						{
								if(m_RollerPulseOut < 5)
								{
										m_Roller1->Set(0.);
										m_Roller2->Set(0.);
										m_RollerPulseOut++;
								}
								else if(m_RollerPulseOut < 8)
								{
										m_Roller1->Set(0.8);
										m_Roller2->Set(-0.8);
										m_RollerPulseOut++;
								}
								else
								{
										m_Roller1->Set(0.);
										m_Roller2->Set(0.);
								}
								
						}
						else if(m_autoPeriodicLoops > 30 && m_autoPeriodicLoops <= 40)
						{
								m_AutonArmPos = ARM_DUMP;
						}
						else if(m_autoPeriodicLoops > 40)
						{
								m_autonomousCase = 8;
								m_autoPeriodicLoops = 0;
								m_armDown = 0;
						}
						break;
	
				case 8:
						//Set cannon speed to ready for key shot
						m_cannonSetPoint = (BOTTOMKEY_SHOT + 1) + m_cannonOffset;
						m_CannonPID->SetSetpoint((BOTTOMKEY_SHOT + 1) + m_cannonOffset);
						m_CannonPID->Enable();
						/*if (m_autoPeriodicLoops > 30)
						{
										m_AutonArmPos = ARM_DUMP;
						}*/
						//Once arm reaches ball dump position, drop balls
						if ((m_armPot->GetAverageVoltage()) >= (ARM_DUMP - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_DUMP + 0.1))
						{
										m_Roller1->Set(-0.8);
										m_Roller2->Set(0.8);
										m_Light->Set(Relay::kForward);
										m_autoPeriodicLoops = 0;
										m_armDown = 1;
						}
						//if(m_autoPeriodicLoops >= 10 && m_armDown == 1)
								//m_AutonArmPos = ARM_BRIDGE;
						//Once robot is at key, stop the bot, put arm down to ball pick up pos
						if (updateRampDrive())
						{
										//m_robotDrive->TankDrive(0.0, 0.0);
										m_armPosition->SetGains(1.0, kI, kD);
										//m_AutonArmPos = ARM_BRIDGE;
										goDraino = 0;
										m_autoPeriodicLoops = 0;
										m_autonomousCase = 9;
										//Turn off rollers
										m_Roller1->Set(0.0);
										m_Roller2->Set(0.0);
						}
						
						break;
				case 9:
					m_AutonArmPos = ARM_BRIDGE;			
					//Auto aim with camera
					if(goDraino == 0)
					{
									cameraAutoAim();
					}
					//Fire if shot is in range
					/*if(m_autoPeriodicLoops >=75)
					{               
							goDraino = 1;
							m_DrvTurnPIDRate->Disable();
					}*/
					ArmLight();     
					FireTheCannon();
					break;
				case 10:
								//KINECT AUTON CASE DONT BREAK ANYTHING
								m_robotDrive->TankDrive(0., 0.);
								m_AutonArmPos = 0;
								m_HopRoller->Set(Relay::kOff);
								m_Index->Set(Relay::kOff);
				break;
				case 11:
						if(!m_bridgeCheck->Get())
						{
								m_autonomousCase = 5;
								m_robotDrive->TankDrive(0., 0.);
						}
						updateRampDrive();
				break;  
				case 12:
						updateRampDrive();
						if((m_armPot->GetAverageVoltage()) >= (ARM_GROUND - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_GROUND + 0.1))
						{
								m_Encoder1->Reset();
								m_Encoder2->Reset();    
								m_robotDrive->TankDrive(0., 0.);
								setDriveRampParams(0, 87.6*REV_IN, 0.02, 0.6, 1.0, 35.04*REV_IN, 0);
								m_autonomousCase = 6;
						}
				break;  

				}
			}
		}
        void TopofKeyCheckBridge()
                {
                        backdrive_spd = -0.70;
                        m_autoPeriodicLoops++;
                        //m_robotDrive->SetSafetyEnabled(false);
                        //char outputstring[10];
                        
                        //sprintf(outputstring,"%0.4f",m_AutonArmPos);
                        //dash->PutString("Demanded Arm Pos:",outputstring);

                        //sprintf(outputstring,"%0.4f",m_armPot->GetAverageVoltage());
                        //dash->PutString("Actual Arm Pos:",outputstring);
                                        
                        //sprintf(outputstring,"%0.4f",m_Encoder1->GetDistance());
                        //dash->PutString("Left Drive Encoder:",outputstring);
                        
                        //sprintf(outputstring,"%0.4f",m_Encoder2->GetDistance());
                        //dash->PutString("Right Drive Encoder:",outputstring);

                        //sprintf(outputstring,"%0.4f",(float)(m_autonomousCase));
                        //dash->PutString("Auton Case:",outputstring);
                                                        
                        //Put bot into low gear
                        TeleopSensorUpdater();
                        TeleopDriverStationUpdate();

                        m_HopRoller->Set(-1.0); 
                        if(m_AutonArmPos != 0)
                        {
                                ArmPosition(m_AutonArmPos);
                        }
                        else
                        {
                                m_Arm1->Set(0.0);
                                m_Arm2->Set(0.0);
                        }
                        m_rightHipValue = m_kinect->GetSkeleton().GetHipRight().z;
                        m_rightAnkleValue = m_kinect->GetSkeleton().GetAnkleRight().z;
                                                        
                        if((m_rightHipValue - m_rightAnkleValue) >= LEG_FORWARD)
                        {
                                RunKinect();
                                m_autonomousCase = 8;
                        }
                        else 
                        {
                                switch(m_autonomousCase)
                                {
                                case 0:
                                                m_AutonArmPos = ARM_BRIDGEAUTON;
                                                m_Encoder1->Reset();
                                                m_Encoder2->Reset();
                                                m_timer->Start();
                                                m_timer->Reset();
                                                m_autonomousCase = 1;
                                break;
                                case 1:
                                                //Set cannon to speed where it could make a shot going backwards
                                                m_cannonSetPoint = TOPKEY_SHOT + m_cannonOffset;
                                                m_CannonPID->SetSetpoint(TOPKEY_SHOT + m_cannonOffset);
                                                m_CannonPID->Enable();
                                                //Shift into low gear
                                                m_shiftreversecounter++;
                                                m_shiftcounter = 0;
                                                m_shiftflag = 0;
                                                incr_speed = 0.0;
                                                m_armLight->Set(Relay::kForward);
                                                //Hop Roller runs constantly throughout Autonomous
                                                m_HopRoller->Set(-1.0);
                                                //Once shift is complete turn off shifter, set next driving cycle to bridge distance
                                                /*if(m_shiftreversecounter > 35)
                                                {
                                                                m_Shifter -> Set(Relay::kOff);
                                                                setDriveRampParams(0, 54*REV_IN*2, 0.007, 0.5, 0.9, 30*REV_IN*2, 0);
                                                }
                                                else
                                                {
                                                                m_Shifter -> Set(Relay::kForward);
                                                }*/
                                                setDriveRampParams(0, (51)*REV_IN*2, 0.015, 0.5, 0.9, 10*REV_IN*2, 0);
                                                if(m_cannonUpToSpeed == 1)
                                                {
                                                                m_HopRoller->Set(-1.0);                 
                                                                m_Index->Set(Relay::kForward);
                                                }
                                                else
                                                {
                                                                m_Index->Set(Relay::kOff);
                                                }
                                                if(m_timer->HasPeriodPassed(3.2)){
                                                        m_autonomousCase = 2;
                                                        backdrive = 0;
                                                        m_armLight->Set(Relay::kOff);
                                                        m_timer->Stop();
                                                }
                                break;          
                                case 2:
                                        //Move arm position to bridge height
                                        m_AutonArmPos = ARM_BRIDGEAUTON;
                                        //Shoot if cannon is up to speed
                                        if(m_cannonUpToSpeed == 1)
                                        {
                                                        m_HopRoller->Set(-1.0);                 
                                                        m_Index->Set(Relay::kForward);
                                        }
                                        else
                                        {
                                                        m_Index->Set(Relay::kOff);
                                        }
                                        //Once you reach bridge change arm pos gains
                                                if (eStopUpdateRampDrive())
                                        {
                                                        m_autonomousCase = 3;
                                                        m_armPosition->SetGains(1.0, kI, kD);
                                        }
                                        break;
                                        case 3:
                                                //Turn off cannon
                                                m_cannonSetPoint = -100;
                                                m_CannonDemandedSpeed=0.;
                                                m_CannonPID->Disable();
                                                m_Index->Set(Relay::kOff);
                                                //Bring arm down to lower bridge and run on the bridge
                                                m_AutonArmPos = ARM_BRIDGE;
                                                //Once bridge is lowered reset encoders and set next driving cycle up bridge
                                                if ((m_armPot->GetAverageVoltage()) >= (ARM_BRIDGE - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_BRIDGE + 0.1))
                                                {
                                                        if(!m_bridgeCheck->Get())
                                                        {
                                                                m_armTimer = 0;
                                                                m_autonomousCase = 4;
                                                        }
                                                        else
                                                        {       //Jump to extra routine if bridge is not found.
                                                                m_Encoder1->Reset();
                                                                m_Encoder2->Reset();
                                                                setDriveRampParams(0, 87.6*REV_IN, 0.00, 0.4, 0.4, 10*REV_IN, 0);
                                                                m_armTimer = 0;
                                                                m_autonomousCase = 10;
                                                        }
                                                }
                                                break;
                                        case 4:
                                                //Once bridge is lowered reset encoders and set next driving cycle up bridge
                                                m_AutonArmPos = ARM_GROUND;
                                                if ((m_armPot->GetAverageVoltage()) >= (ARM_GROUND - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_GROUND + 0.1))
                                                {
                                                                m_Encoder1->Reset();
                                                                m_Encoder2->Reset();
                                                                m_autoPeriodicLoops = 0;
                                                                m_autonomousCase = 5;
                                                                setDriveRampParams(0, 87.6*REV_IN, 0.02, 0.6, 1.0, 35.04*REV_IN, 0);
                                                }
                                                m_armTimer++;
                                                if(m_armTimer > 30)
                                                {
                                                        setDriveRampParams(0, -87.6*REV_IN, 0.00, 0.55, 0.55, 10*REV_IN, 0);
                                                        m_autonomousCase = 11;
                                                }
                                                break;
                                        case 5:
                                                //Once up the bridge change arm gains, reset encoders and set next short trip
                                                if (updateRampDrive())
                                                {
                                                                //m_robotDrive->TankDrive(0.3 , 0.3);  //Not really needed
                                                                m_armPosition->SetGains(3.0, kI, kD);
                                                                //Lift arm to ball dump position
                                                                m_Encoder1->Reset();
                                                                m_Encoder2->Reset();
                                                                setDriveRampParams(0, -210*REV_IN, 0.010, 0.4, 1.0, 100*REV_IN, 0);
                                                                m_RollerPulseOut = 0;
                                                                m_autonomousCase = 6;
                                                                m_autoPeriodicLoops = 0;
                                                }
                                                break;
                                        case 6:
                                                if(m_autoPeriodicLoops <= 15)
                                                {
                                                        m_Roller1->Set(-0.8);
                                                        m_Roller2->Set(0.8);
                                                }
                                                else if(m_autoPeriodicLoops > 15 && m_autoPeriodicLoops <= 30)
                                                {
                                                        if(m_RollerPulseOut < 5)
                                                        {
                                                                m_Roller1->Set(0.);
                                                                m_Roller2->Set(0.);
                                                                m_RollerPulseOut++;
                                                        }
                                                        else if(m_RollerPulseOut < 8)
                                                        {
                                                                m_Roller1->Set(0.8);
                                                                m_Roller2->Set(-0.8);
                                                                m_RollerPulseOut++;
                                                        }
                                                        else
                                                        {
                                                                m_Roller1->Set(0.);
                                                                m_Roller2->Set(0.);
                                                        }
                                                        
                                                }
                                                else if(m_autoPeriodicLoops > 30 && m_autoPeriodicLoops <= 40)
                                                {
                                                        m_AutonArmPos = ARM_DUMP;
                                                }
                                                else if(m_autoPeriodicLoops > 40)
                                                {
                                                        m_autonomousCase = 7;
                                                        m_autoPeriodicLoops = 0;
                                                        m_armDown = 0;
                                                }
                                                break;

                                        case 7:
                                                //Set cannon speed to ready for key shot
                                                m_cannonSetPoint = BOTTOMKEY_SHOT + m_cannonOffset;
                                                m_CannonPID->SetSetpoint(BOTTOMKEY_SHOT + m_cannonOffset);
                                                m_CannonPID->Enable();
                                                /*if (m_autoPeriodicLoops > 30)
                                                {
                                                                m_AutonArmPos = ARM_DUMP;
                                                }*/
                                                //Once arm reaches ball dump position, drop balls
                                                if ((m_armPot->GetAverageVoltage()) >= (ARM_DUMP - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_DUMP + 0.1))
                                                {
                                                                m_Roller1->Set(-0.8);
                                                                m_Roller2->Set(0.8);
                                                                m_Light->Set(Relay::kForward);
                                                                m_autoPeriodicLoops = 0;
                                                                m_armDown = 1;
                                                }
                                                if(m_autoPeriodicLoops >= 10 && m_armDown == 1)
                                                        m_AutonArmPos = ARM_BRIDGE;
                                                //Once robot is at key, stop the bot, put arm down to ball pick up pos
                                                if (updateRampDrive())
                                                {
                                                                //m_robotDrive->TankDrive(0.0, 0.0);
                                                                m_armPosition->SetGains(1.0, kI, kD);
                                                                m_AutonArmPos = ARM_BRIDGE;
                                                                goDraino = 0;
                                                                m_autoPeriodicLoops = 0;
                                                                m_autonomousCase = 8;
                                                                //Turn off rollers
                                                                m_Roller1->Set(0.0);
                                                                m_Roller2->Set(0.0);
                                                }
                                                
                                                break;
                                        case 8:
                                                        //Auto aim with camera
                                                        if(goDraino == 0)
                                                        {
                                                                        cameraAutoAim();
                                                        }
                                                        //Fire if shot is in range
                                                        /*if(m_autoPeriodicLoops >=75)
                                                        {               
                                                                goDraino = 1;
                                                                m_DrvTurnPIDRate->Disable();
                                                        }*/
                                                        ArmLight();     
                                                        FireTheCannon();
                                        break;
                                        case 9:
                                                        //KINECT AUTON CASE DONT BREAK ANYTHING
                                                        m_robotDrive->TankDrive(0., 0.);
                                                        m_AutonArmPos = 0;
                                                        m_HopRoller->Set(Relay::kOff);
                                                        m_Index->Set(Relay::kOff);
                                        break;
                                        case 10:
                                                if(!m_bridgeCheck->Get())
                                                {
                                                        m_autonomousCase = 4;
                                                        m_robotDrive->TankDrive(0., 0.);
                                                }
                                                updateRampDrive();
                                        break;  
                                        case 11:
                                                updateRampDrive();
                                                if((m_armPot->GetAverageVoltage()) >= (ARM_GROUND - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_GROUND + 0.1))
                                                {
                                                        m_Encoder1->Reset();
                                                        m_Encoder2->Reset();    
                                                        m_robotDrive->TankDrive(0., 0.);
                                                        setDriveRampParams(0, 87.6*REV_IN, 0.02, 0.6, 1.0, 35.04*REV_IN, 0);
                                                        m_autonomousCase = 5;
                                                }
                                        break;  
                                }
                        }
                }
        void BottomofKeyCheckBridge()
		{
				backdrive_spd = -0.75;
				m_autoPeriodicLoops++;
				//m_robotDrive->SetSafetyEnabled(false);
				//char outputstring[10];
				
				//sprintf(outputstring,"%0.4f",m_AutonArmPos);
				//dash->PutString("Demanded Arm Pos:",outputstring);

				//sprintf(outputstring,"%0.4f",m_armPot->GetAverageVoltage());
				//dash->PutString("Actual Arm Pos:",outputstring);
								
				//sprintf(outputstring,"%0.4f",m_Encoder1->GetDistance());
				//dash->PutString("Left Drive Encoder:",outputstring);
				
				//sprintf(outputstring,"%0.4f",m_Encoder2->GetDistance());
				//dash->PutString("Right Drive Encoder:",outputstring);

				//sprintf(outputstring,"%0.4f",(float)(m_autonomousCase));
				//dash->PutString("Auton Case:",outputstring);
												
				//Put bot into low gear
				TeleopSensorUpdater();
				TeleopDriverStationUpdate();

				m_HopRoller->Set(-1.0); 
				if(m_AutonArmPos != 0)
				{
						ArmPosition(m_AutonArmPos);
				}
				else
				{
						m_Arm1->Set(0.0);
						m_Arm2->Set(0.0);
				}
				m_rightHipValue = m_kinect->GetSkeleton().GetHipRight().z;
				m_rightAnkleValue = m_kinect->GetSkeleton().GetAnkleRight().z;
												
				if((m_rightHipValue - m_rightAnkleValue) >= LEG_FORWARD)
				{
						RunKinect();
						m_autonomousCase = 8;
				}
				else 
				{
						switch(m_autonomousCase)
						{
						case 0:
										m_AutonArmPos = ARM_BRIDGEAUTON;
										m_Encoder1->Reset();
										m_Encoder2->Reset();
										m_timer->Start();
										m_timer->Reset();
										m_autonomousCase = 1;
						break;
						case 1:
										//Set cannon to speed where it could make a shot going backwards
										m_cannonSetPoint = (BOTTOMKEY_SHOT + 1) + m_cannonOffset;
										m_CannonPID->SetSetpoint((BOTTOMKEY_SHOT + 1) + m_cannonOffset);
										m_CannonPID->Enable();
										//Shift into low gear
										m_shiftreversecounter++;
										m_shiftcounter = 0;
										m_shiftflag = 0;
										incr_speed = 0.0;
										m_armLight->Set(Relay::kForward);
										//Hop Roller runs constantly throughout Autonomous
										m_HopRoller->Set(-1.0);
										//Once shift is complete turn off shifter, set next driving cycle to bridge distance
										/*if(m_shiftreversecounter > 35)
										{
														m_Shifter -> Set(Relay::kOff);
														setDriveRampParams(0, 54*REV_IN*2, 0.007, 0.5, 0.9, 30*REV_IN*2, 0);
										}
										else
										{
														m_Shifter -> Set(Relay::kForward);
										}*/
										setDriveRampParams(0, (54+37)*REV_IN*2, 0.007, 0.5, 0.9, 10*REV_IN*2, 0);
										if(m_cannonUpToSpeed == 1)
										{
														m_HopRoller->Set(-1.0);                 
														m_Index->Set(Relay::kForward);
										}
										else
										{
														m_Index->Set(Relay::kOff);
										}
										if(m_timer->HasPeriodPassed(3.2)){
												m_autonomousCase = 2;
												backdrive = 0;
												m_armLight->Set(Relay::kOff);
												m_timer->Stop();
										}
						break;          
						case 2:
								//Move arm position to bridge height
								m_AutonArmPos = ARM_BRIDGEAUTON;
								//Shoot if cannon is up to speed
								if(m_cannonUpToSpeed == 1)
								{
												m_HopRoller->Set(-1.0);                 
												m_Index->Set(Relay::kForward);
								}
								else
								{
												m_Index->Set(Relay::kOff);
								}
								//Once you reach bridge change arm pos gains
										if (eStopUpdateRampDrive())
								{
												m_autonomousCase = 3;
												m_armPosition->SetGains(1.0, kI, kD);
								}
								break;
								case 3:
										//Turn off cannon
										m_cannonSetPoint = -100;
										m_CannonDemandedSpeed=0.;
										m_CannonPID->Disable();
										m_Index->Set(Relay::kOff);
										//Bring arm down to lower bridge and run on the bridge
										m_AutonArmPos = ARM_BRIDGE;
										//Once bridge is lowered reset encoders and set next driving cycle up bridge
										if ((m_armPot->GetAverageVoltage()) >= (ARM_BRIDGE - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_BRIDGE + 0.1))
										{
												if(!m_bridgeCheck->Get())
												{
														m_armTimer = 0;
														m_autonomousCase = 4;
												}
												else
												{       //Jump to extra routine if bridge is not found.
														m_Encoder1->Reset();
														m_Encoder2->Reset();
														setDriveRampParams(0, 87.6*REV_IN, 0.00, 0.4, 0.4, 10*REV_IN, 0);
														m_armTimer = 0;
														m_autonomousCase = 10;
												}
										}
										break;
								case 4:
										//Once bridge is lowered reset encoders and set next driving cycle up bridge
										m_AutonArmPos = ARM_GROUND;
										if ((m_armPot->GetAverageVoltage()) >= (ARM_GROUND - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_GROUND + 0.1))
										{
														m_Encoder1->Reset();
														m_Encoder2->Reset();
														m_autoPeriodicLoops = 0;
														m_autonomousCase = 5;
														setDriveRampParams(0, 87.6*REV_IN, 0.02, 0.6, 1.0, 35.04*REV_IN, 0);
										}
										m_armTimer++;
										if(m_armTimer > 30)
										{
												setDriveRampParams(0, -87.6*REV_IN, 0.00, 0.55, 0.55, 10*REV_IN, 0);
												m_autonomousCase = 11;
										}
										break;
								case 5:
										//Once up the bridge change arm gains, reset encoders and set next short trip
										if (updateRampDrive())
										{
														//m_robotDrive->TankDrive(0.3 , 0.3);  //Not really needed
														m_armPosition->SetGains(3.0, kI, kD);
														//Lift arm to ball dump position
														m_Encoder1->Reset();
														m_Encoder2->Reset();
														setDriveRampParams(0, -210*REV_IN, 0.010, 0.4, 1.0, 70*REV_IN, 0);
														m_RollerPulseOut = 0;
														m_autonomousCase = 6;
														m_autoPeriodicLoops = 0;
										}
										break;
								case 6:
										if(m_autoPeriodicLoops <= 15)
										{
												m_Roller1->Set(-0.8);
												m_Roller2->Set(0.8);
										}
										else if(m_autoPeriodicLoops > 15 && m_autoPeriodicLoops <= 30)
										{
												if(m_RollerPulseOut < 5)
												{
														m_Roller1->Set(0.);
														m_Roller2->Set(0.);
														m_RollerPulseOut++;
												}
												else if(m_RollerPulseOut < 8)
												{
														m_Roller1->Set(0.8);
														m_Roller2->Set(-0.8);
														m_RollerPulseOut++;
												}
												else
												{
														m_Roller1->Set(0.);
														m_Roller2->Set(0.);
												}
												
										}
										else if(m_autoPeriodicLoops > 30 && m_autoPeriodicLoops <= 40)
										{
												m_AutonArmPos = ARM_DUMP;
										}
										else if(m_autoPeriodicLoops > 40)
										{
												m_autonomousCase = 7;
												m_autoPeriodicLoops = 0;
												m_armDown = 0;
										}
										break;

								case 7:
										//Set cannon speed to ready for key shot
										m_cannonSetPoint = (BOTTOMKEY_SHOT + 1) + m_cannonOffset;
										m_CannonPID->SetSetpoint((BOTTOMKEY_SHOT + 1) + m_cannonOffset);
										m_CannonPID->Enable();
										/*if (m_autoPeriodicLoops > 30)
										{
														m_AutonArmPos = ARM_DUMP;
										}*/
										//Once arm reaches ball dump position, drop balls
										if ((m_armPot->GetAverageVoltage()) >= (ARM_DUMP - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_DUMP + 0.1))
										{
														m_Roller1->Set(-0.8);
														m_Roller2->Set(0.8);
														m_Light->Set(Relay::kForward);
														m_autoPeriodicLoops = 0;
														m_armDown = 1;
										}
										if(m_autoPeriodicLoops >= 10 && m_armDown == 1)
												m_AutonArmPos = ARM_BRIDGE;
										//Once robot is at key, stop the bot, put arm down to ball pick up pos
										if (updateRampDrive())
										{
														//m_robotDrive->TankDrive(0.0, 0.0);
														m_armPosition->SetGains(1.0, kI, kD);
														m_AutonArmPos = ARM_BRIDGE;
														goDraino = 0;
														m_autoPeriodicLoops = 0;
														m_autonomousCase = 8;
														//Turn off rollers
														m_Roller1->Set(0.0);
														m_Roller2->Set(0.0);
										}
										
										break;
								case 8:
												//Auto aim with camera
												if(goDraino == 0)
												{
																cameraAutoAim();
												}
												//Fire if shot is in range
												/*if(m_autoPeriodicLoops >=75)
												{               
														goDraino = 1;
														m_DrvTurnPIDRate->Disable();
												}*/
												ArmLight();     
												FireTheCannon();
								break;
								case 9:
												//KINECT AUTON CASE DONT BREAK ANYTHING
												m_robotDrive->TankDrive(0., 0.);
												m_AutonArmPos = 0;
												m_HopRoller->Set(Relay::kOff);
												m_Index->Set(Relay::kOff);
								break;
								case 10:
										if(!m_bridgeCheck->Get())
										{
												m_autonomousCase = 4;
												m_robotDrive->TankDrive(0., 0.);
										}
										updateRampDrive();
								break;  
								case 11:
										updateRampDrive();
										if((m_armPot->GetAverageVoltage()) >= (ARM_GROUND - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_GROUND + 0.1))
										{
												m_Encoder1->Reset();
												m_Encoder2->Reset();    
												m_robotDrive->TankDrive(0., 0.);
												setDriveRampParams(0, 87.6*REV_IN, 0.02, 0.6, 1.0, 35.04*REV_IN, 0);
												m_autonomousCase = 5;
										}
								break;  
						}
				}
		}
        void MiddleofKeyCheckBridge()
        {
				backdrive_spd = -0.75;
				m_autoPeriodicLoops++;
				//m_robotDrive->SetSafetyEnabled(false);
				//char outputstring[10];
				
				//sprintf(outputstring,"%0.4f",m_AutonArmPos);
				//dash->PutString("Demanded Arm Pos:",outputstring);

				//sprintf(outputstring,"%0.4f",m_armPot->GetAverageVoltage());
				//dash->PutString("Actual Arm Pos:",outputstring);
								
				//sprintf(outputstring,"%0.4f",m_Encoder1->GetDistance());
				//dash->PutString("Left Drive Encoder:",outputstring);
				
				//sprintf(outputstring,"%0.4f",m_Encoder2->GetDistance());
				//dash->PutString("Right Drive Encoder:",outputstring);

				//sprintf(outputstring,"%0.4f",(float)(m_autonomousCase));
				//dash->PutString("Auton Case:",outputstring);
												
				//Put bot into low gear
				TeleopSensorUpdater();
				TeleopDriverStationUpdate();

				m_HopRoller->Set(-1.0); 
				if(m_AutonArmPos != 0)
				{
						ArmPosition(m_AutonArmPos);
				}
				else
				{
						m_Arm1->Set(0.0);
						m_Arm2->Set(0.0);
				}
				m_rightHipValue = m_kinect->GetSkeleton().GetHipRight().z;
				m_rightAnkleValue = m_kinect->GetSkeleton().GetAnkleRight().z;
												
				if((m_rightHipValue - m_rightAnkleValue) >= LEG_FORWARD)
				{
						RunKinect();
						m_autonomousCase = 8;
				}
				else 
				{
						switch(m_autonomousCase)
						{
						case 0:
										m_AutonArmPos = ARM_BRIDGEAUTON;
										m_Encoder1->Reset();
										m_Encoder2->Reset();
										m_timer->Start();
										m_timer->Reset();
										m_autonomousCase = 1;
						break;
						case 1:
										//Set cannon to speed where it could make a shot going backwards
										m_cannonSetPoint = (BOTTOMKEY_SHOT + 1) + m_cannonOffset;
										m_CannonPID->SetSetpoint((BOTTOMKEY_SHOT + 1) + m_cannonOffset);
										m_CannonPID->Enable();
										//Shift into low gear
										m_shiftreversecounter++;
										m_shiftcounter = 0;
										m_shiftflag = 0;
										incr_speed = 0.0;
										m_armLight->Set(Relay::kForward);
										//Hop Roller runs constantly throughout Autonomous
										m_HopRoller->Set(-1.0);
										//Once shift is complete turn off shifter, set next driving cycle to bridge distance
										/*if(m_shiftreversecounter > 35)
										{
														m_Shifter -> Set(Relay::kOff);
														setDriveRampParams(0, 54*REV_IN*2, 0.007, 0.5, 0.9, 30*REV_IN*2, 0);
										}
										else
										{
														m_Shifter -> Set(Relay::kForward);
										}*/
										setDriveRampParams(0, (73)*REV_IN*2, 0.015, 0.5, 0.9, 10*REV_IN*2, 0);
										if(m_cannonUpToSpeed == 1)
										{
														m_HopRoller->Set(-1.0);                 
														m_Index->Set(Relay::kForward);
										}
										else
										{
														m_Index->Set(Relay::kOff);
										}
										if(m_timer->HasPeriodPassed(3.2)){
												m_autonomousCase = 2;
												backdrive = 0;
												m_armLight->Set(Relay::kOff);
												m_timer->Stop();
										}
						break;          
						case 2:
								//Move arm position to bridge height
								m_AutonArmPos = ARM_BRIDGEAUTON;
								//Shoot if cannon is up to speed
								if(m_cannonUpToSpeed == 1)
								{
												m_HopRoller->Set(-1.0);                 
												m_Index->Set(Relay::kForward);
								}
								else
								{
												m_Index->Set(Relay::kOff);
								}
								//Once you reach bridge change arm pos gains
										if (eStopUpdateRampDrive())
								{
												m_autonomousCase = 3;
												m_armPosition->SetGains(1.0, kI, kD);
								}
								break;
								case 3:
										//Turn off cannon
										m_cannonSetPoint = -100;
										m_CannonDemandedSpeed=0.;
										m_CannonPID->Disable();
										m_Index->Set(Relay::kOff);
										//Bring arm down to lower bridge and run on the bridge
										m_AutonArmPos = ARM_BRIDGE;
										//Once bridge is lowered reset encoders and set next driving cycle up bridge
										if ((m_armPot->GetAverageVoltage()) >= (ARM_BRIDGE - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_BRIDGE + 0.1))
										{
												if(!m_bridgeCheck->Get())
												{
														m_armTimer = 0;
														m_autonomousCase = 4;
												}
												else
												{       //Jump to extra routine if bridge is not found.
														m_Encoder1->Reset();
														m_Encoder2->Reset();
														setDriveRampParams(0, 87.6*REV_IN, 0.00, 0.4, 0.4, 10*REV_IN, 0);
														m_armTimer = 0;
														m_autonomousCase = 10;
												}
										}
										break;
								case 4:
										//Once bridge is lowered reset encoders and set next driving cycle up bridge
										m_AutonArmPos = ARM_GROUND;
										if ((m_armPot->GetAverageVoltage()) >= (ARM_GROUND - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_GROUND + 0.1))
										{
														m_Encoder1->Reset();
														m_Encoder2->Reset();
														m_autoPeriodicLoops = 0;
														m_autonomousCase = 5;
														setDriveRampParams(0, 87.6*REV_IN, 0.02, 0.6, 1.0, 35.04*REV_IN, 0);
										}
										m_armTimer++;
										if(m_armTimer > 30)
										{
												setDriveRampParams(0, -87.6*REV_IN, 0.00, 0.55, 0.55, 10*REV_IN, 0);
												m_autonomousCase = 11;
										}
										break;
								case 5:
										//Once up the bridge change arm gains, reset encoders and set next short trip
										if (updateRampDrive())
										{
														//m_robotDrive->TankDrive(0.3 , 0.3);  //Not really needed
														m_armPosition->SetGains(3.0, kI, kD);
														//Lift arm to ball dump position
														m_Encoder1->Reset();
														m_Encoder2->Reset();
														setDriveRampParams(0, -210*REV_IN, 0.010, 0.4, 1.0, 100*REV_IN, 0);
														m_RollerPulseOut = 0;
														m_autonomousCase = 6;
														m_autoPeriodicLoops = 0;
										}
										break;
								case 6:
										if(m_autoPeriodicLoops <= 15)
										{
												m_Roller1->Set(-0.8);
												m_Roller2->Set(0.8);
										}
										else if(m_autoPeriodicLoops > 15 && m_autoPeriodicLoops <= 30)
										{
												if(m_RollerPulseOut < 5)
												{
														m_Roller1->Set(0.);
														m_Roller2->Set(0.);
														m_RollerPulseOut++;
												}
												else if(m_RollerPulseOut < 8)
												{
														m_Roller1->Set(0.8);
														m_Roller2->Set(-0.8);
														m_RollerPulseOut++;
												}
												else
												{
														m_Roller1->Set(0.);
														m_Roller2->Set(0.);
												}
												
										}
										else if(m_autoPeriodicLoops > 30 && m_autoPeriodicLoops <= 40)
										{
												m_AutonArmPos = ARM_DUMP;
										}
										else if(m_autoPeriodicLoops > 40)
										{
												m_autonomousCase = 7;
												m_autoPeriodicLoops = 0;
												m_armDown = 0;
										}
										break;

								case 7:
										//Set cannon speed to ready for key shot
										m_cannonSetPoint = (BOTTOMKEY_SHOT + 1) + m_cannonOffset;
										m_CannonPID->SetSetpoint((BOTTOMKEY_SHOT + 1) + m_cannonOffset);
										m_CannonPID->Enable();
										/*if (m_autoPeriodicLoops > 30)
										{
														m_AutonArmPos = ARM_DUMP;
										}*/
										//Once arm reaches ball dump position, drop balls
										if ((m_armPot->GetAverageVoltage()) >= (ARM_DUMP - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_DUMP + 0.1))
										{
														m_Roller1->Set(-0.8);
														m_Roller2->Set(0.8);
														m_Light->Set(Relay::kForward);
														m_autoPeriodicLoops = 0;
														m_armDown = 1;
										}
										if(m_autoPeriodicLoops >= 10 && m_armDown == 1)
												m_AutonArmPos = ARM_BRIDGE;
										//Once robot is at key, stop the bot, put arm down to ball pick up pos
										if (updateRampDrive())
										{
														//m_robotDrive->TankDrive(0.0, 0.0);
														m_armPosition->SetGains(1.0, kI, kD);
														m_AutonArmPos = ARM_BRIDGE;
														goDraino = 0;
														m_autoPeriodicLoops = 0;
														m_autonomousCase = 8;
														//Turn off rollers
														m_Roller1->Set(0.0);
														m_Roller2->Set(0.0);
										}
										
										break;
								case 8:
												//Auto aim with camera
												if(goDraino == 0)
												{
																cameraAutoAim();
												}
												//Fire if shot is in range
												/*if(m_autoPeriodicLoops >=75)
												{               
														goDraino = 1;
														m_DrvTurnPIDRate->Disable();
												}*/
												ArmLight();     
												FireTheCannon();
								break;
								case 9:
												//KINECT AUTON CASE DONT BREAK ANYTHING
												m_robotDrive->TankDrive(0., 0.);
												m_AutonArmPos = 0;
												m_HopRoller->Set(Relay::kOff);
												m_Index->Set(Relay::kOff);
								break;
								case 10:
										if(!m_bridgeCheck->Get())
										{
												m_autonomousCase = 4;
												m_robotDrive->TankDrive(0., 0.);
										}
										updateRampDrive();
								break;  
								case 11:
										updateRampDrive();
										if((m_armPot->GetAverageVoltage()) >= (ARM_GROUND - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_GROUND + 0.1))
										{
												m_Encoder1->Reset();
												m_Encoder2->Reset();    
												m_robotDrive->TankDrive(0., 0.);
												setDriveRampParams(0, 87.6*REV_IN, 0.02, 0.6, 1.0, 35.04*REV_IN, 0);
												m_autonomousCase = 5;
										}
								break;  
						}
				}
		}

        void BridgeFirstCheckBridge()
                {
                backdrive_spd = -0.7;
                        m_autoPeriodicLoops++;
                        TeleopSensorUpdater();
                        TeleopDriverStationUpdate();
                        m_Light->Set(Relay::kForward);
                        m_HopRoller->Set(-1.0); 
                        if(m_AutonArmPos != 0)
                        {
                                ArmPosition(m_AutonArmPos);
                        }
                        else
                        {
                                m_Arm1->Set(0.0);
                                m_Arm2->Set(0.0);
                        }
                        m_rightHipValue = m_kinect->GetSkeleton().GetHipRight().z;
                        m_rightAnkleValue = m_kinect->GetSkeleton().GetAnkleRight().z;
                                                        
                        if((m_rightHipValue - m_rightAnkleValue) >= LEG_FORWARD)
                        {
                                RunKinect();
                                m_autonomousCase = 8;
                        }
                        else 
                        {
                                switch(m_autonomousCase)
                                {
                                case 0:
                                                m_armPosition->SetGains(2.0, kI, kD);
                                                m_AutonArmPos = ARM_BRIDGEAUTON;
                                                m_Encoder1->Reset();
                                                m_Encoder2->Reset();
                                                //m_timer->Start();
                                                //m_timer->Reset();
                                                setDriveRampParams(0, (51)*REV_IN*2, 0.015, 0.5, 0.9, 10*REV_IN*2, 0);
                                                m_autonomousCase = 2;
                                                
                                break;
                                case 2:
                                        //Move arm position to bridge height
                                        m_AutonArmPos = ARM_BRIDGEAUTON;
                                        //Shoot if cannon is up to speed
                                        //Once you reach bridge change arm pos gains
                                        if (eStopUpdateRampDrive())
                                        {
                                                        m_autonomousCase = 3;
                                                        m_armPosition->SetGains(1.0, kI, kD);
                                                        m_Encoder1->Reset();
                                                        m_Encoder2->Reset();                                                    
                                        }
                                break;
                                case 3:
                                        //Turn off cannon
                                        //Bring arm down to lower bridge and run on the bridge
                                        m_AutonArmPos = ARM_BRIDGE;
                                        //Once bridge is lowered reset encoders and set next driving cycle up bridge
                                        if ((m_armPot->GetAverageVoltage()) >= (ARM_BRIDGE - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_BRIDGE + 0.1))
                                        {
                                                if(!m_bridgeCheck->Get())
                                                {
                                                        m_armTimer = 0;
                                                        m_autonomousCase = 4;
                                                }
                                                else
                                                {       //Jump to extra routine if bridge is not found.
                                                        m_Encoder1->Reset();
                                                        m_Encoder2->Reset();
                                                        setDriveRampParams(0, 95*REV_IN, 0.00, 0.4, 0.4, 10*REV_IN, 0);
                                                        m_armTimer = 0;
                                                        m_autonomousCase = 10;
                                                }
                                        }
                                break;
                                case 4:
                                        //Once bridge is lowered reset encoders and set next driving cycle up bridge
                                        m_AutonArmPos = ARM_GROUND;
                                        if ((m_armPot->GetAverageVoltage()) >= (ARM_GROUND - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_GROUND + 0.1))
                                        {
                                                        m_Encoder1->Reset();
                                                        m_Encoder2->Reset();
                                                        m_autoPeriodicLoops = 0;
                                                        m_timer->Start();
                                                        m_timer->Reset();
                                                        setDriveRampParams(0, -105*REV_IN, 0.02, 0.6, 1.0, 35.04*REV_IN, 0);
                                                        m_autonomousCase = 5;
                                        }
                                        m_armTimer++;
                                        if(m_armTimer > 30)
                                        {
                                                setDriveRampParams(0, -105*REV_IN, 0.00, 0.55, 0.55, 10*REV_IN, 0);
                                                m_autonomousCase = 11;
                                        }
                                break;
                                case 5:
                                        if(m_timer->HasPeriodPassed(0.75))
                                        {
                                                m_timer->Stop();
                                                m_autonomousCase = 6;
                                                
                                        }
                                        break;
                                case 6:
                                        //Once up the bridge change arm gains, reset encoders and set next short trip
                                        if (updateRampDrive())
                                        {
                                                        //m_robotDrive->TankDrive(0.3 , 0.3);  //Not really needed
                                                        m_armPosition->SetGains(3.0, kI, kD);
                                                        //Lift arm to ball dump position
                                                        m_Encoder1->Reset();
                                                        m_Encoder2->Reset();
                                                        //setDriveRampParams(0, -210*REV_IN, 0.010, 0.4, 1.0, 100*REV_IN, 0);
                                                        m_RollerPulseOut = 0;
                                                        m_autonomousCase = 8;
                                                        m_autoPeriodicLoops = 0;
                                                        //Set cannon speed to ready for key shot
                                                        m_cannonSetPoint = (BOTTOMKEY_SHOT + 1) + m_cannonOffset;
                                                        m_CannonPID->SetSetpoint((BOTTOMKEY_SHOT + 1) + m_cannonOffset);
                                                        m_CannonPID->Enable();
                                        
                                        }
                                break;
                                        case 8:
                                                        //Auto aim with camera
                                                        if(goDraino == 0)
                                                        {
                                                                        cameraAutoAim();
                                                        }
                                                        //Fire if shot is in range
                                                        /*if(m_autoPeriodicLoops >=75)
                                                        {               
                                                                goDraino = 1;
                                                                m_DrvTurnPIDRate->Disable();
                                                        }*/
                                                        ArmLight();     
                                                        FireTheCannon();
                                        break;
                                        case 9:
                                                        //KINECT AUTON CASE DONT BREAK ANYTHING
                                                        m_robotDrive->TankDrive(0., 0.);
                                                        m_AutonArmPos = 0;
                                                        m_HopRoller->Set(Relay::kOff);
                                                        m_Index->Set(Relay::kOff);
                                        break;
                                        case 10:
                                                if(!m_bridgeCheck->Get())
                                                {
                                                        m_autonomousCase = 4;
                                                        m_robotDrive->TankDrive(0., 0.);
                                                }
                                                updateRampDrive();
                                        break;  
                                        case 11:
                                                updateRampDrive();
                                                if((m_armPot->GetAverageVoltage()) >= (ARM_GROUND - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_GROUND + 0.1))
                                                {
                                                        m_Encoder1->Reset();
                                                        m_Encoder2->Reset();    
                                                        m_timer->Start();
                                                        m_timer->Reset();
                                                        m_robotDrive->TankDrive(0., 0.);
                                                        setDriveRampParams(0, -105*REV_IN, 0.02, 0.6, 1.0, 35.04*REV_IN, 0);
                                                        m_autonomousCase = 5;
                                                }
                                        break;  
                                }
                        }
                }
        void Shoot2TopKey()
           {
                        backdrive_spd = -0.75;
                        m_autoPeriodicLoops++;
                        //m_robotDrive->SetSafetyEnabled(false);
                        //char outputstring[10];
                        
                        //sprintf(outputstring,"%0.4f",m_AutonArmPos);
                        //dash->PutString("Demanded Arm Pos:",outputstring);
        
                        //sprintf(outputstring,"%0.4f",m_armPot->GetAverageVoltage());
                        //dash->PutString("Actual Arm Pos:",outputstring);
                                        
                        //sprintf(outputstring,"%0.4f",m_Encoder1->GetDistance());
                        //dash->PutString("Left Drive Encoder:",outputstring);
                        
                        //sprintf(outputstring,"%0.4f",m_Encoder2->GetDistance());
                        //dash->PutString("Right Drive Encoder:",outputstring);
        
                        //sprintf(outputstring,"%0.4f",(float)(m_autonomousCase));
                        //dash->PutString("Auton Case:",outputstring);
                                                        
                        //Put bot into low gear
                        TeleopSensorUpdater();
                        TeleopDriverStationUpdate();
        
                        m_HopRoller->Set(-1.0); 
                        if(m_AutonArmPos != 0)
                        {
                                ArmPosition(m_AutonArmPos);
                        }
                        else
                        {
                                m_Arm1->Set(0.0);
                                m_Arm2->Set(0.0);
                        }
                        m_rightHipValue = m_kinect->GetSkeleton().GetHipRight().z;
                        m_rightAnkleValue = m_kinect->GetSkeleton().GetAnkleRight().z;
                                                        
                        if((m_rightHipValue - m_rightAnkleValue) >= LEG_FORWARD)
                        {
                                RunKinect();
                                m_autonomousCase = 8;
                        }
                        else 
                        {
                                switch(m_autonomousCase)
                                {
                                case 0:
                                                m_AutonArmPos = ARM_BRIDGEAUTON;
                                                m_Encoder1->Reset();
                                                m_Encoder2->Reset();
                                                m_autonomousCase = 1;
                                                m_timer->Start();
                                                m_timer->Reset();
                                break;
                                case 1:
                                                //Set cannon to speed where it could make a shot going backwards
                                                m_cannonSetPoint = TOPKEY_SHOT + m_cannonOffset;
                                                m_CannonPID->SetSetpoint(TOPKEY_SHOT + m_cannonOffset);
                                                m_CannonPID->Enable();
                                                //Shift into low gear
                                                m_shiftreversecounter++;
                                                m_shiftcounter = 0;
                                                m_shiftflag = 0;
                                                incr_speed = 0.0;
                                                //Hop Roller runs constantly throughout Autonomous
                                                m_HopRoller->Set(-1.0);
                                                //Once shift is complete turn off shifter, set next driving cycle to bridge distance
                                                /*if(m_shiftreversecounter > 35)
                                                {
                                                                m_Shifter -> Set(Relay::kOff);
                                                                setDriveRampParams(0, 54*REV_IN*2, 0.007, 0.5, 0.9, 30*REV_IN*2, 0);
                                                }
                                                else
                                                {
                                                                m_Shifter -> Set(Relay::kForward);
                                                }*/
                                                //setDriveRampParams(0, (54+42)*REV_IN*2, 0.010, 0.5, 0.9, 10*REV_IN*2, 0);
                                                if(m_cannonUpToSpeed == 1)
                                                {
                                                                m_HopRoller->Set(-1.0);                 
                                                                m_Index->Set(Relay::kForward);
                                                }
                                                else
                                                {
                                                                m_Index->Set(Relay::kOff);
                                                }
                                                if(m_timer->HasPeriodPassed(3.2))
                                                {
                                                        m_cannonSetPoint = -100;
                                                        m_CannonDemandedSpeed=0.;
                                                        m_CannonPID->Disable();
                                                        m_timer->Stop();
                                                        m_autonomousCase = 2;
                                                }
                                break;  
                                case 2:
                                        
                                break;
                                }
                        }
           }
        void Shoot2BottomKey()
        {
                backdrive_spd = -0.75;
                        m_autoPeriodicLoops++;
                        //m_robotDrive->SetSafetyEnabled(false);
                        //char outputstring[10];
                        
                        //sprintf(outputstring,"%0.4f",m_AutonArmPos);
                        //dash->PutString("Demanded Arm Pos:",outputstring);

                        //sprintf(outputstring,"%0.4f",m_armPot->GetAverageVoltage());
                        //dash->PutString("Actual Arm Pos:",outputstring);
                                        
                        //sprintf(outputstring,"%0.4f",m_Encoder1->GetDistance());
                        //dash->PutString("Left Drive Encoder:",outputstring);
                        
                        //sprintf(outputstring,"%0.4f",m_Encoder2->GetDistance());
                        //dash->PutString("Right Drive Encoder:",outputstring);

                        //sprintf(outputstring,"%0.4f",(float)(m_autonomousCase));
                        //dash->PutString("Auton Case:",outputstring);
                                                        
                        //Put bot into low gear
                        TeleopSensorUpdater();
                        TeleopDriverStationUpdate();
                        //ArmLight();

                        m_HopRoller->Set(-1.0); 
                        if(m_AutonArmPos != 0)
                        {
                                ArmPosition(m_AutonArmPos);
                        }
                        else
                        {
                                m_Arm1->Set(0.0);
                                m_Arm2->Set(0.0);
                        }
                        m_rightHipValue = m_kinect->GetSkeleton().GetHipRight().z;
                        m_rightAnkleValue = m_kinect->GetSkeleton().GetAnkleRight().z;
                                                        
                        if((m_rightHipValue - m_rightAnkleValue) >= LEG_FORWARD)
                        {
                                RunKinect();
                                m_autonomousCase = 8;
                        }
                        else 
                        {
                                switch(m_autonomousCase)
                                {
                                case 0:
                                                m_Index->Set(Relay::kOff);
                                                m_HopRoller->Set(Relay::kOff);
                                                m_AutonArmPos = ARM_BRIDGEAUTON;
                                                m_Encoder1->Reset();
                                                m_Encoder2->Reset();
                                                m_autonomousCase = 1;
                                break;
                                case 1:
                                                //Set cannon to speed where it could make a shot going backwards
                                                m_cannonSetPoint = (BOTTOMKEY_SHOT + 1) + m_cannonOffset;
                                                m_CannonPID->SetSetpoint((BOTTOMKEY_SHOT + 1) + m_cannonOffset);
                                                m_CannonPID->Enable();
                                                //Hop Roller runs constantly throughout Autonomous
                                                m_HopRoller->Set(-1.0);
                                                if(m_cannonUpToSpeed == 1)
                                                {
                                                                m_HopRoller->Set(-1.0);                 
                                                                m_Index->Set(Relay::kForward);
                                                }
                                                else
                                                {
                                                                m_Index->Set(Relay::kOff);
                                                }
                                                //cameraAutoAim();
                                                //FireTheCannon();
                                break;  
                                }
                        }
        }

        
        
                void TeleopPeriodic(void)
                {
                                // increment the number of teleop periodic loops completed
                                m_telePeriodicLoops++;
                                m_dsPacketsReceivedInCurrentSecond++;                                   // increment DS packets received
                                
                                //m_robotDrive->SetSafetyEnabled(false);
                                //DIAGNOSTIC MODE - PRESS BACK AND START ON DRIVER CONTROLLER
                                if(m_Gamepad1->GetRawButton(BUTTON_BACK) && m_Gamepad1->GetRawButton(BUTTON_START))
                                {
                                                m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "DIAGNOSTIC MODE ");
                                                m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "       %f       ");
                                                m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "       %f       ");
                                                m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "       %f       ");
                                                m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "       %f       ");
                                                m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "       %f       ");

                                                m_dsLCD->UpdateLCD(); 
                                                DiagnosticMode();
                                }
                        else if(m_Gamepad1->GetRawButton(BUTTON_BACK))
                                                CameraDiagnosticMode();
                                                //PIDTest();
                                else
                                {
                                                //PIDTest();
                                                //CannonSpeedTest();
                                                
                                                //Leave light on always during teleop
                                                if(m_Gamepad1->GetRawButton(BUTTON_A))
                                                        m_Light->Set(Relay::kOff);
                                                else
                                                        m_Light->Set(Relay::kForward);
                                                
                                                
                                                TeleopSensorUpdater();
                                                TeleopDriverStationUpdate();
                                                TeleopArmPosition();
                                                TeleopCannonSpeed();
                                                TeleopRoller();
                                                FireTheCannon();
                                                ArmLight();
                                                Shift();
                                                                                
                                                if(m_Gamepad1->GetRawAxis(TRIGGERS) > 0.4)
                                                {
                                                                //cameraAutoAim();
                                                                //FireTheCannon();
                                                				TeleopDrive();
                                                }
                                                else
                                                {
                                                                //Diagnostic to reset Encoders
                                                                if(m_Gamepad1->GetRawButton(BUTTON_B))
                                                                {
                                                                                m_Encoder1->Reset();
                                                                                m_Encoder2->Reset();
                                                                }
                                                                /*else if(m_Gamepad1->GetRawButton(BUTTON_B))
                                                                {
                                                                                m_DrvStraightPID->SetTolerance(0.02);
                                                                                m_DrvStraightPID->SetSetpoint(1000);
                                                                                m_DrvStraightPID->Enable();
                                                                                dash->PutDouble("output PID",m_DrvStraightPID->Get());
                                                                                dash->PutBoolean("output ontarget",m_DrvStraightPID->OnTarget());
                                                                }
                                                                else if(m_Gamepad1->GetRawButton(BUTTON_X))
                                                                {
                                                                                m_DrvTurnPID->SetTolerance(0.02);
                                                                                m_DrvTurnPID->SetSetpoint(-680);
                                                                                m_DrvTurnPID->Enable();
                                                                                dash->PutDouble("output PID",m_DrvStraightPID->Get());
                                                                                dash->PutDouble("output Encode",DrvStraightEncodeHandler->PIDGet());
                                                                }
                                                                */
                                                                else if(m_Gamepad1->GetRawButton(BUTTON_Y))
                                                                {
                                                                                DriveStraightEncoder(0.6);
                                                                }
                                                                else
                                                                {
                                                                        m_DrvTurnPID->Disable();
                                                                        m_DrvTurnPIDRate->Disable();
                                                                        m_DrvStraightPID->Disable();
                                                                        DrvTurnPIDRateHandler->integrator = 0;
                                                                        //Diagnostic to check DrvPID
                                                                        /*if(m_Gamepad1->GetRawButton(BUTTON_B))
                                                                        {
                                                                                        m_Encoder1->Reset();
                                                                                        m_Encoder2->Reset();
                                                                                        m_DrvStraightPID->SetSetpoint(500);
                                                                                        m_DrvStraightPID->Enable();
                                                                        }
                                                                        else if(m_Gamepad1->GetRawButton(BUTTON_B))
                                                                        {
                                                                                        m_Encoder1->Reset();
                                                                                        m_Encoder2->Reset();
                                                                                        m_DrvStraightPID->SetSetpoint(-500);
                                                                                        m_DrvStraightPID->Enable();
                                                                        }
                                                                        
                                                                        else
                                                                                m_DrvStraightPID->Disable();
                                                                        */
                                                                        TeleopDrive();
                                                                        m_cannonAimed = 0;
                                                                        m_picSkipLoops = 0;     
                                                                        m_HopRollerOverride = 0;
                                                                }
                                                }
                                }
                }
                
                void TeleopDriverStationUpdate(void)
                {
                                m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "lDrv Encoder: %f     ", m_Encoder1Distance);
                                m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "rDrv Encoder: %f     ", m_Encoder2Distance);
                                m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Arm Pot: %f          ", m_armPotValue);
                                m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "Cannon Spd: %f       ", m_cannonSetPoint);
                                //m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "BridgeArm: %d       ",m_bridgeCheck->Get());
                                m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "CanUptoSpd: %d       ",m_cannonUpToSpeed);
                                //m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "CanAimed: %d         ",m_cannonAimed);
                                m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "Auton: %d         ",m_autonomousCase);
                                m_dsLCD->UpdateLCD();    
                        
                                
                                
                                dash->PutDouble("Left Drive Encoder",m_Encoder1Distance);
                                dash->PutInt("Bridge Arm",m_bridgeCheck->Get());
                                dash->PutDouble("Right Drive Encoder", m_Encoder2Distance);
                                dash->PutDouble("Right Drive Encoder Rate", m_Encoder1->GetRate());
                                dash->PutDouble("Arm Pot Location", m_armPotValue);
                                dash->PutDouble("Cannon Encoder:",m_cannonRate);
                                dash->PutDouble("Cannon up to Speed?", m_cannonUpToSpeed);
                                dash->PutDouble("Cannon Aimed?", m_cannonAimed);

                }
                
                void TeleopDrive(void)
                {
                	 if(m_Gamepad1->GetRawAxis(TRIGGERS) > 0.4)
                         m_robotDrive->ArcadeDrive((.75*(-m_Gamepad1->GetRawAxis(LEFT_Y))), (.75*(-(m_Gamepad1->GetRawAxis(RIGHT_X)))));
                	 else
                		 m_robotDrive->ArcadeDrive((-m_Gamepad1->GetRawAxis(LEFT_Y)), -(m_Gamepad1->GetRawAxis(RIGHT_X)));
                }
                
                void TeleopSensorUpdater(void)
                {
                                m_Encoder1Distance = m_Encoder1 -> GetDistance();
                                m_Encoder2Distance = m_Encoder2 -> GetDistance();
                                m_armPotValue = m_armPot->GetVoltage();
                                m_cannonRate = m_CannonEncode->GetRate();
                }
                
                void TeleopArmPosition(void)
                {
                                //char outputstring[10];
                                //sprintf(outputstring,"%0.4f",m_Arm1->Get());
                                //dash->PutString("Arm 1 Motor Control:",outputstring);
                                //sprintf(outputstring,"%0.4f",m_Arm2->Get());
                                //dash->PutString("Arm 2 Motor Control:",outputstring);
                                //sprintf(outputstring,"%0.4f",m_armPot->GetAverageVoltage());
                                //dash->PutString("Arm Pot Average Voltage:",outputstring);
                                if(m_Gamepad1->GetRawButton(BUTTON_RB) && m_bridgeDown == 1)
                                {
                                        m_armPosition->SetGains(2., 0., 0.05);
                                        ArmPosition(ARM_GROUND);
                                }
                                else if(m_Gamepad1->GetRawButton(BUTTON_RB))
                                {
                                        if((m_armPot->GetAverageVoltage()) >= (ARM_BRIDGE - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_BRIDGE + 0.1) && (!m_bridgeCheck->Get()))
                                                        m_bridgeDown = 1;
                                        m_armPosition->SetGains(2., 0., 0.05);
                                        ArmPosition(ARM_BRIDGE);
                        }
                                else if(m_Gamepad2->GetRawButton(BUTTON_LB))
                                {
                                                m_armPosition->SetGains(2., 0., 0.05);
                                                ArmPosition(ARM_PICKUP);
                                                m_bridgeDown = 0;
                                }
                                else if(m_Gamepad2->GetRawButton(BUTTON_RB))
                                {//m_armHandler->updateArm(ARM_UP);
                                                m_armPosition->SetGains(2., 0., 0.05);
                                                ArmPosition(ARM_DUMP);
                                                m_bridgeDown = 0;
                                }
                                else if(m_Gamepad2->GetRawAxis(2) > 0.4)
                                {
                                                m_armPosition->SetGains(16., 0., 0.05);
                                                ArmPosition(ARM_BALANCE);
                                                m_bridgeDown = 0;
                                }
                                else if(m_Gamepad2->GetRawAxis(2) < -0.4)
                                {
                                                m_armPosition->SetGains(2., 0., 0.05);
                                                ArmPosition(ARM_BARRIER);
                                                m_bridgeDown = 0;
                                }
                                else
                                {
                                                //m_armHandler->stopArm();
                                                m_Arm1->Set(-(m_Gamepad2->GetRawAxis(RIGHT_Y)));
                                                m_Arm2->Set(m_Gamepad2->GetRawAxis(RIGHT_Y));
                                                m_bridgeDown = 0;
                                }
                }
                void TeleopCannonSpeed(void)
                {
                                if(m_Gamepad2->GetRawButton(BUTTON_A) && (!m_Gamepad2->GetRawButton(BUTTON_START)))
                                {
                                                m_cannonSetPoint = BOTTOMKEY_SHOT + m_cannonOffset;
                                                m_CannonPID->SetSetpoint(BOTTOMKEY_SHOT + m_cannonOffset);
                                                m_CannonPID->Enable();
                                }
                                else if(m_Gamepad2->GetRawButton(BUTTON_B) && (!m_Gamepad2->GetRawButton(BUTTON_START)))
                                {
                                                m_cannonSetPoint = -100;
                                                m_CannonDemandedSpeed=0.;
                                                m_CannonPID->Disable();
                                }
                                else if(m_Gamepad2->GetRawButton(BUTTON_X) && (!m_Gamepad2->GetRawButton(BUTTON_START)))
                                {
                                                m_cannonSetPoint = TOPKEY_SHOT + m_cannonOffset;
                                                m_CannonPID->SetSetpoint(TOPKEY_SHOT + m_cannonOffset);
                                                m_CannonPID->Enable();
                                }
                                else if(m_Gamepad2->GetRawButton(BUTTON_Y) && (!m_Gamepad2->GetRawButton(BUTTON_START)))
                                {
                                                m_cannonSetPoint = HAILMARY_SHOT;
                                                m_CannonPID->SetSetpoint(HAILMARY_SHOT);
                                                m_CannonPID->Enable();
                                }
                                else if(m_Gamepad2->GetRawButton(BUTTON_START))
                                {
									if(m_Gamepad2->GetRawButton(BUTTON_A))
									{
										m_buttonTimer->Start();
										if(m_buttonTimer->HasPeriodPassed(0.2))
										{
											m_cannonSetPoint++;
											m_buttonTimer->Reset();
										}
										
									}
									if(m_Gamepad2->GetRawButton(BUTTON_B))
									{
										m_buttonTimer->Start();
										if(m_buttonTimer->HasPeriodPassed(0.2))
										{
											m_cannonSetPoint--;
											m_buttonTimer->Reset();
										}
									}
									m_CannonPID->SetSetpoint(m_cannonSetPoint + m_cannonOffset);
									m_CannonPID->Enable();
                                                				
                                }
								else
								{
									m_buttonTimer->Stop();
								}
								
                }
                
                void TeleopRoller(void)
                {
                                if ((m_armPot->GetAverageVoltage()) >= (ARM_DUMP - 0.1) && (m_armPot->GetAverageVoltage()) <= (ARM_DUMP + 0.1))
                                {
                                                m_Roller1->Set(-0.8);
                                                m_Roller2->Set(0.8);
                                }
                                else if((m_Gamepad2->GetRawAxis(TRIGGERS)) < -0.6)
                                //if((m_Gamepad2->GetRawAxis(TRIGGERS)) < -0.6)
                                {
                                                m_Roller1->Set(-0.8);
                                                m_Roller2->Set(0.8);
                                                m_RollerPulseOut = 0;
                                }
                                else if((m_Gamepad2->GetRawAxis(TRIGGERS)) > 0.6)
                                {
                                                m_Roller1->Set(0.8);
                                                m_Roller2->Set(-0.8);
                                }
                                else
                                {
                                                m_Roller1->Set(0.0);
                                                m_Roller2->Set(0.0);
                                                if(m_RollerPulseOut < 5)
                                                {
                                                        m_RollerPulseOut++;
                                                }
                                                else if(m_RollerPulseOut < 8)
                                                {
                                                        m_Roller1->Set(0.8);
                                                        m_Roller2->Set(-0.8);
                                                        m_RollerPulseOut++;
                                                }
                                }
                                // Left Stick Y Axis: 2- Up-Negative Down-Positive
                                if(m_HopRollerOverride == 0 && m_Gamepad2->GetRawAxis(1) > 0.4)
                                                m_HopRoller->Set(-1.0); 
                                else if(m_HopRollerOverride == 0 && m_Gamepad2->GetRawAxis(1) < -0.4)
                                                m_HopRoller->Set(1.0);
                                else
                                                m_HopRoller->Set(0.0);
                }
                
        void DisabledPeriodic(void)  {
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
                m_dsLCD->UpdateLCD();
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
                m_autonomousSelect = 0;
                
        }
                
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
                                                        /*if(duty <= 1){
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
                                                        }*/
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
        
        //All parameters necessary to gradually speed up and down the drive 
        //  TODO: This will eventually needto be put into a class
        double rparam_spos, rparam_epos, rparam_uslope, rparam_minspeed, rparam_maxspeed, rparam_gdist, rparam_turn;
        int direction;
        void setDriveRampParams(double start_pos, double end_pos, double ramp_up_speed, double min_speed, double max_speed, double guard_distance, double turn){
                
                //Set up the local paramters
                rparam_spos = start_pos;
                rparam_epos = end_pos;
                rparam_uslope = ramp_up_speed;
                rparam_minspeed = min_speed;
                rparam_maxspeed = max_speed;
                rparam_gdist = guard_distance;
                rparam_turn = turn;
                
                //Determine which direction we're going
                direction = (start_pos < end_pos) ? 1 : -1;
                
                //Reset state
                incr_speed = min_speed;
        }
        
        int updateRampDrive()
        {
                //dash->PutDouble("getDriveDistance:",getDriveDistance());
                double cur_position = getDriveDistance();
                if(rparam_turn == 1)
                        cur_position = getTurnDistance();
                //Update state
                if(direction == 1 && cur_position > (rparam_epos - rparam_gdist))
                        incr_speed -= rparam_uslope;
                else if(direction == -1 && cur_position < (rparam_epos + rparam_gdist))
                        incr_speed -= rparam_uslope;
                else
                        incr_speed += rparam_uslope;
                
                //Bound the speed
                incr_speed = (incr_speed > rparam_maxspeed) ? rparam_maxspeed :
                             (incr_speed < rparam_minspeed) ? rparam_minspeed : incr_speed;
                
                if(direction == 1 && cur_position > rparam_epos)
                {
                        m_robotDrive->TankDrive(0.0, 0.0);
                        return 1;
                }
                else if(direction == -1 && cur_position < rparam_epos)
                {
                        m_robotDrive->TankDrive(0.0, 0.0);
                        return 1;
                }
                else if(direction == 1 && rparam_turn==0)
                        DriveStraightEncoder(incr_speed);
                else if(direction == -1 && rparam_turn==0)
                        DriveStraightEncoder(-incr_speed);
                else if(direction == 1 && rparam_turn==1)
                        m_robotDrive->ArcadeDrive((float)(0),(float)(incr_speed));
                else if(direction == -1 && rparam_turn==1)
                        m_robotDrive->ArcadeDrive((float)(0),(float)(-incr_speed));
                return 0;
        }
        
        int eStopUpdateRampDrive()
        {
                //dash->PutDouble("getDriveDistance:",;
                double cur_position = getDriveDistance();
                static double last_cur_position = 0;
                if(rparam_turn == 1)
                                cur_position = getTurnDistance();
                //Update state
                else if((direction == 1 && cur_position > (rparam_epos - rparam_gdist)))
                {
                                incr_speed = backdrive_spd;
                                backdrive = 1;
                }
                else if((direction == -1 && cur_position < (rparam_epos + rparam_gdist)))
                {
                                incr_speed = backdrive_spd;
                                backdrive = 1;
                }
                else
                                incr_speed += rparam_uslope;
                
                if(direction == 1 && (cur_position > rparam_epos || ((backdrive == 1) && (cur_position <= last_cur_position))))
                {
                                m_robotDrive->TankDrive(0.0, 0.0);
                                return 1;
                }
                else if(direction == -1 && (cur_position < rparam_epos || ((backdrive == 1) && (cur_position >= last_cur_position))))
                {
                                m_robotDrive->TankDrive(0.0, 0.0);
                                return 1;
                }
                else if(direction == 1 && rparam_turn==0)
                                DriveStraightEncoder(incr_speed);
                else if(direction == -1 && rparam_turn==0)
                                DriveStraightEncoder(-incr_speed);

                dash->PutDouble("cur_posit",cur_position);
                dash->PutDouble("rparam_epos",rparam_epos);
                dash->PutDouble("rparam_gdist",rparam_gdist);
                
                last_cur_position = cur_position;
                return 0;
        }
        int eStopUpdateRampDriveJim()
                {
                                //dash->PutDouble("getDriveDistance:",;
                                double cur_position = getDriveDistance();
                                static double last_cur_position = 0;
                                if(rparam_turn == 1)
                                                                cur_position = getTurnDistance();
                                //Update state
                                else if((direction == 1 && cur_position > (rparam_epos - rparam_gdist)))
                                {
                                                                incr_speed = backdrive_spd;
                                                                backdrive = 1;
                                }
                                else if((direction == -1 && cur_position < (rparam_epos + rparam_gdist)))
                                {
                                                                incr_speed = backdrive_spd;
                                                                backdrive = 1;
                                }
                                else
                                                                incr_speed = rparam_minspeed/(m_Encoder1->GetRate()) ;
                                
                                if(direction == 1 && (cur_position > rparam_epos || ((backdrive == 1) && (cur_position <= last_cur_position))))
                                {
                                                                m_robotDrive->TankDrive(0.0, 0.0);
                                                                return 1;
                                }
                                else if(direction == -1 && (cur_position < rparam_epos || ((backdrive == 1) && (cur_position >= last_cur_position))))
                                {
                                                                m_robotDrive->TankDrive(0.0, 0.0);
                                                                return 1;
                                }
                                else if(direction == 1 && rparam_turn==0)
                                                                DriveStraightEncoder(incr_speed);
                                else if(direction == -1 && rparam_turn==0)
                                                                DriveStraightEncoder(-incr_speed);

                                dash->PutDouble("cur_posit",cur_position);
                                dash->PutDouble("rparam_epos",rparam_epos);
                                dash->PutDouble("rparam_gdist",rparam_gdist);
                                
                                last_cur_position = cur_position;
                                return 0;
                }
        /*void DriveStraightDistance(float Destination, float Speed)
        {
                float driftClicks;
                
                if(Speed >= 0.5) && (Speed < 0.65)
                {       
                        driftClicks = 1010101;
                }
                else if(Speed >= 0.65) && (Speed < 0.8))
                {       
                        driftClicks = 1010101;
                }
                else if(Speed >= 0.8)
                {
                        driftClicks = 1010101;
                }
                if(m_Encoder1Distance <= (Desination - driftClicks))
                {
                        DriveStraightEncoder(Speed);
                }
                else if((m_Encoder1Distance <= Destination) && (m_Encoder1Distance > (Destination - driftClicks)))
                (
                        m_robotDrive->TankDrive(0. , 0.);
                }
        }*/

        void FireTheCannon(void)
        {
                        //m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Spd: %d Aim: %d",m_cannonUpToSpeed,m_cannonAimed);
                        //m_dsLCD->UpdateLCD();
                        if((goDraino == 1) && (m_telePeriodicLoops == 0))
                        {
                                 m_HopRoller->Set(-1.0);                 
                                 m_Index->Set(Relay::kForward);
                                 m_HopRollerOverride = 1;
                        }
                        else if(m_cannonUpToSpeed == 1 && m_cannonAimed >= 5) //If aimed and up to speed, shoot.
                        {
                                m_HopRoller->Set(-1.0);                 
                                m_Index->Set(Relay::kForward);
                                m_HopRollerOverride = 1;
                                goDraino = 1;
                        }  
                        
                        else if(m_cannonUpToSpeed == 1)
                        {
                                if(m_Gamepad1->GetRawAxis(TRIGGERS) < -0.4) //If right trigger pulled, shoot.
                                {
                                                m_HopRoller->Set(-1.0);                 
                                                m_Index->Set(Relay::kForward);
                                                m_HopRollerOverride = 1;
                                }
                                else
                                {
                                                m_HopRollerOverride = 0;
                                                m_Index->Set(Relay::kOff);
                                }
                        }
                        else
                        {
                                        m_HopRollerOverride = 0;
                                        m_Index->Set(Relay::kOff);
                        }
        }
        
        void ArmLight()
        {
                if(m_cannonAimed >= 5)
                        m_armLight->Set(Relay::kForward);
                else if(m_Gamepad1->GetRawButton(BUTTON_RB))
                        m_armLight->Set(Relay::kForward);
                else if(m_Gamepad1->GetRawButton(BUTTON_X))
                        m_armLight->Set(Relay::kForward);
                else if(m_Gamepad1->GetRawAxis(TRIGGERS) < -0.4)
                        m_armLight->Set(Relay::kForward);
                else
                        m_armLight->Set(Relay::kOff);
        }
        
        void Shift(void)
        {
                if(!m_Gamepad1 -> GetRawButton(BUTTON_LB))//If right trigger, shift
                {
                        m_shiftcounter++;
                        m_shiftreversecounter = 0;
                        if(m_shiftcounter > 50)
                        {
                                m_Shifter -> Set(Relay::kOff);
                                
                        }
                        else
                        {
                                m_Shifter -> Set(Relay::kReverse);
                        }
                        m_shiftflag = 1;
                }
                else if(m_shiftflag = 1)//Unshift after letting go of button 8
                {
                        m_shiftreversecounter++;
                        m_shiftcounter = 0;
                        m_shiftflag = 0;
                        if(m_shiftreversecounter > 50)
                        {
                                m_Shifter -> Set(Relay::kOff);
                        }
                        else
                        {
                                m_Shifter -> Set(Relay::kForward);
                        }
                }
        }
        
        void ArmPosition(float ArmPosition)
        {
                char outputstring[10];
                sprintf(outputstring,"%0.4f",m_armPosition->GetMV(ArmPosition,m_armPot->GetVoltage()));
                //dash->PutString("Arm MV:",outputstring);
                
                m_Arm1->Set(-(m_armPosition->GetMV(ArmPosition,m_armPot->GetVoltage())));
                m_Arm2->Set((m_armPosition->GetMV(ArmPosition,m_armPot->GetVoltage())));
        }
        
        void DriveStraightEncoder(float speed)
        {
                if (m_Encoder1->GetDistance() + 10 > m_Encoder2->GetDistance())
                {
                        m_robotDrive->TankDrive(speed + comp_spd, speed - comp_spd);
                        comp_spd = comp_spd + 0.0001;
                }
                else if (m_Encoder2->GetDistance() + 10 > m_Encoder1->GetDistance())
                {
                        m_robotDrive->TankDrive(speed - comp_spd, speed + comp_spd);
                        comp_spd = comp_spd + 0.0001;
                }
                else
                {
                        m_robotDrive->TankDrive(speed, speed);
                        comp_spd = 0.05;
                }
        }

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
        
        void CannonSpeedTest(void)
        {
                
                if (m_Gamepad1->GetRawButton(1) && buttonFlag == 0)
                {
                        cannonSpeed++;
                        buttonFlag = 1;
                }
                else buttonFlag = 0;
                if (m_Gamepad1->GetRawButton(2) && buttonFlag == 0)
                {
                        cannonSpeed--;
                        buttonFlag = 1;
                }
                else buttonFlag = 0;
                
                m_cannonSetPoint = cannonSpeed;
                m_CannonPID->SetSetpoint(cannonSpeed);
                m_CannonPID->Enable();
        }
        void RunKinect(void) 
        {
                m_leftArmValue = -(m_leftArm->GetY());
                m_rightArmValue = -(m_rightArm->GetY());
                                
                if (m_leftArmValue > .85) 
                {
                        m_leftArmValue = .85;
                }
                                                                
                if (m_rightArmValue > .85) 
                {
                        m_rightArmValue = .85;
                }

                if (m_leftArmValue < -.85) 
                {
                        m_leftArmValue = -.85;
                }
                                                                                                           
                if (m_rightArmValue < -.85) 
                {
                        m_rightArmValue = -.85;
                }
                                                                                
                if ((m_leftArmValue < .15) && (m_leftArmValue > -.15))
                {
                        m_leftArmValue = 0;
                }
                                                                                
                if ((m_rightArmValue < .15) && (m_rightArmValue > -.15))
                {
                        m_rightArmValue = 0;
                }
                if ((m_rightArmValue > .65) && (m_leftArmValue < -.65))
                {
                        m_rightArmValue = 1;
                        m_leftArmValue = -1;
                }
                if ((m_rightArmValue < -.65) && (m_leftArmValue > .65))
                {
                        m_rightArmValue = -1;
                        m_leftArmValue = 1;
                }
                
                m_robotDrive->TankDrive(m_leftArmValue, m_rightArmValue);
        }
                        
        
        void PIDTest(void)
        {
                        m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "kP: %f     ", kP);
                        m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "kI: %f     ", kI);
                        m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "kD: %f     ", kD);
                        m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "              ");
                        //m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "              ");
                                //m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "              ");
                                m_dsLCD->UpdateLCD();
                                
                        if (m_Gamepad1->GetRawButton(BUTTON_A))
                {
                        if(m_Gamepad1->GetRawButton(BUTTON_BACK))
                        {
                                kP = (kP - 0.00001);
                        }
                        else
                        {
                                kP += 0.00001;
                        }
                        m_DrvTurnPIDRate->SetPID(kP, kI, kD);
                }
                if (m_Gamepad1->GetRawButton(BUTTON_B))
                {
                        if(m_Gamepad1->GetRawButton(BUTTON_BACK))
                        {
                                kD = (kD - 0.00001);
                        }
                        else
                        {
                                kD = (kD + 0.00001);
                        }
                        m_DrvTurnPIDRate->SetPID(kP, kI, kD);
                }
                if (m_Gamepad1->GetRawButton(BUTTON_X))
                {
                        if(m_Gamepad1->GetRawButton(BUTTON_BACK))
                        {
                                kI = (kI - 0.001);
                        }
                        else
                        {
                                kI = (kI + 0.001);
                        }
                        m_DrvTurnPIDRate->SetPID(kP, kI, kD);
                }
                /*if (m_Gamepad1->GetRawButton(BUTTON_Y))
                {
                        if(m_Gamepad1->GetRawButton(BUTTON_BACK))
                        {
                                ARM_PICKUP = (ARM_PICKUP - 0.001);
                        }
                        else
                        {
                                ARM_PICKUP = (ARM_PICKUP + 0.001);
                        }
                }*/
        }
                        
};

START_ROBOT_CLASS(BuiltinDefaultCode);
