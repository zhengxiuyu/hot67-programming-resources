//Implementation file for AutonChoiceWrapper
#include "AutonChoiceWrapper.h"

AutonChoiceWrapper::AutonChoiceWrapper (int choice_num, int& output)
{
	output = choice_num;
	auton_mode = choice_num;
}

bool AutonChoiceWrapper::IsFinished ()
{
	return true;
}

int AutonChoiceWrapper::update ()
{
	return auton_mode;
}
