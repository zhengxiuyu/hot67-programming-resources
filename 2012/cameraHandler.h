	#include "WPILib.h"
	#include "Math.h"
	#include "nivision.h"

	class cameraHandler	
	{
	public:
		cameraHandler(AxisCamera *camera, DriverStationLCD *m_dsLCD, SmartDashboard *dash, Relay *relay);
		double getCenter();
	private:
		AxisCamera *camera;
		DriverStationLCD *m_dsLCD;
		ColorImage *img;
		ColorImage *img2;
		Relay *light;
		SmartDashboard *dash;
	};
