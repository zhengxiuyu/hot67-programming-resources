#include "DrvStraightPID.h"

DrvStraightPID::DrvStraightPID(RobotDrive *m_robotDrive,Encoder *m_Encoder1, Encoder *m_Encoder2)
{
	this->m_robotDrive = m_robotDrive;
	this->m_Encoder1 = m_Encoder1;
	this->m_Encoder2 = m_Encoder2;
}

void DrvStraightPID::PIDWrite(float output)
{
	m_robotDrive->SetSafetyEnabled(false);
    float comp_spd = 0.05;
    if (m_Encoder1->GetDistance() + 10 > m_Encoder2->GetDistance())
    {
            m_robotDrive->TankDrive(output + comp_spd, output - comp_spd);
    }
    else if (m_Encoder2->GetDistance() + 10 > m_Encoder1->GetDistance())
    {
            m_robotDrive->TankDrive(output - comp_spd, output + comp_spd);
    }
    else
    {
            m_robotDrive->TankDrive(output, output);
    }
}
