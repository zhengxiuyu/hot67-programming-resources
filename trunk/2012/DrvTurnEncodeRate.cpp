#include "DrvTurnEncodeRate.h"

DrvTurnEncodeRate::DrvTurnEncodeRate(Encoder *m_Encoder1, Encoder *m_Encoder2)
{
	this->m_Encoder1 = m_Encoder1;
	this->m_Encoder2 = m_Encoder2;
}

double DrvTurnEncodeRate::PIDGet()
{
  return ((m_Encoder1->GetRate() - m_Encoder2->GetRate()));
}
