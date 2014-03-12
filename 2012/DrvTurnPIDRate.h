	#include "WPILib.h"
	#include "Math.h"
	
	class DrvTurnPIDRate: public PIDOutput
	{
		public:
			DrvTurnPIDRate(RobotDrive *m_robotDrive,Encoder *m_Encoder1, Encoder *m_Encoder2);
			void PIDWrite(float output);
			float integrator;
		private:
			RobotDrive *m_robotDrive;
			Encoder *m_Encoder1;
			Encoder *m_Encoder2;
			float output;
			float comp_spd;
	};
