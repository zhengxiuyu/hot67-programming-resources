#include "PlatePIDAccelHandler.h"

PlatePIDAccelHandler::PlatePIDAccelHandler(Victor* m_Plate, Victor* m_Plate2,  AnalogChannel* m_Pot, ADXL345_SPI *m_plateAccel, double m_plateAccelFilt)
{
    m_PlateMotor1 = m_Plate;
    m_PlateMotor2 = m_Plate2;
    m_PlatePot = m_Pot;
    m_Accel = m_plateAccel;
    m_PlateAccelFiltered = m_plateAccelFilt;
}

void PlatePIDAccelHandler::PIDWrite(float output)
{
    m_PlateMotor1->Set(output);
    m_PlateMotor2->Set(output);
}

double PlatePIDAccelHandler::PIDGet()
{
    //m_PlatePot->PIDGet();
	//return(m_Accel->GetAcceleration(m_Accel->kAxis_X));
	return(m_PlateAccelFiltered);
}
