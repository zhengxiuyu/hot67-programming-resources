#include "DrvTurnPID.h"

DrvTurnPID::DrvTurnPID(RobotDrive *m_robotDrive,Encoder *m_Encoder1, Encoder *m_Encoder2)
{
	this->m_robotDrive = m_robotDrive;
	this->m_Encoder1 = m_Encoder1;
	this->m_Encoder2 = m_Encoder2;
}

void DrvTurnPID::PIDWrite(float output)
{
	m_robotDrive->ArcadeDrive(0.0 ,output);
}



