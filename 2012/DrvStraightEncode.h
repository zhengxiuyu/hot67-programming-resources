	#include "WPILib.h"
	#include "Math.h"
	
	class DrvStraightEncode : public PIDSource
	{
		public:
			DrvStraightEncode(Encoder *m_Encoder1, Encoder *m_Encoder1);
			double PIDGet();
		private:
			Encoder *m_Encoder1;
			Encoder *m_Encoder2;
	};
