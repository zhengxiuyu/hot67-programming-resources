#include "DrvTurnEncode.h"

DrvTurnEncode::DrvTurnEncode(Encoder *m_Encoder1, Encoder *m_Encoder2)
{
	this->m_Encoder1 = m_Encoder1;
	this->m_Encoder2 = m_Encoder2;
}

double DrvTurnEncode::PIDGet()
{
  return ((m_Encoder1->GetDistance() - m_Encoder2->GetDistance()));
}
