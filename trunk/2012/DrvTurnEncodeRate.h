	#include "WPILib.h"
	#include "Math.h"
	
	class DrvTurnEncodeRate : public PIDSource
	{
		public:
			DrvTurnEncodeRate(Encoder *m_Encoder1, Encoder *m_Encoder1);
			double PIDGet();
		private:
			Encoder *m_Encoder1;
			Encoder *m_Encoder2;
	};
