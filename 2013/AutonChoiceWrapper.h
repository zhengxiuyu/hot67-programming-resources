//Class to use Smart Dashboard to select autonomous modes
#include "Commands/Command.h"

class AutonChoiceWrapper : public Command
{
public:
	AutonChoiceWrapper(int choice_num, int& output);
	bool IsFinished();
	int update();
private:
	int auton_mode;
	bool checked;
};
