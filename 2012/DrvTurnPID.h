	#include "WPILib.h"
	#include "Math.h"
	
	class DrvTurnPID: public PIDOutput
	{
		public:
			DrvTurnPID(RobotDrive *m_robotDrive,Encoder *m_Encoder1, Encoder *m_Encoder2);
			void PIDWrite(float output);
		private:
			RobotDrive *m_robotDrive;
			Encoder *m_Encoder1;
			Encoder *m_Encoder2;
			float output;
			float comp_spd;
			float integrator;
	};
