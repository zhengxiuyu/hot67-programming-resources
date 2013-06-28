#include "PlatePIDHandler.h"

PlatePIDHandler::PlatePIDHandler(Victor* m_Plate1, Victor* m_Plate2,  AnalogChannel* m_Pot, ADXL345_SPI *m_plateAccel)
{
    m_PlateMotor1 = m_Plate1;
    m_PlateMotor2 = m_Plate2;
    m_PlatePot = m_Pot;
    m_Accel = m_plateAccel;
}

PlatePIDHandler::PlatePIDHandler(Victor* m_Plate1, Victor* m_Plate2,  AnalogChannel* m_Pot)
{
    m_PlateMotor1 = m_Plate1;
    m_PlateMotor2 = m_Plate2;
    m_PlatePot = m_Pot;
}

void PlatePIDHandler::PIDWrite(float output)
{
    m_PlateMotor1->Set(output);
    m_PlateMotor2->Set(output);
}

double PlatePIDHandler::PIDGet()
{
    return m_PlatePot->PIDGet();
	//return(m_Accel->GetAcceleration(m_Accel->kAxis_X));
}
