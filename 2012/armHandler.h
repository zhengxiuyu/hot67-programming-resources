	#include "WPILib.h"
	#include "Math.h"
	#include "nivision.h"

	class armHandler
	{
	public:
		armHandler(DriverStationLCD *m_dsLCD);
		double getCenter();
	private:
		DriverStationLCD *m_dsLCD;
	};
