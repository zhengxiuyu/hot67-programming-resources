	#include "WPILib.h"
	#include "Math.h"
	
	class DrvTurnEncode : public PIDSource
	{
		public:
			DrvTurnEncode(Encoder *m_Encoder1, Encoder *m_Encoder1);
			double PIDGet();
		private:
			Encoder *m_Encoder1;
			Encoder *m_Encoder2;
	};
