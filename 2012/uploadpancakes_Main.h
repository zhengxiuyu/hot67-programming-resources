#include "WPILib.h"
#include "Math.h"
#include "HotPID.h"
#include "cameraHandler.h"
#include "DrvStraightEncode.h"
#include "DrvStraightPID.h"
#include "DrvTurnEncode.h"
#include "DrvTurnEncodeRate.h"
#include "DrvTurnPID.h"
#include "DrvTurnPIDRate.h"

#define BUTTON_A 1
#define BUTTON_B 2
#define BUTTON_X 3
#define BUTTON_Y 4
#define BUTTON_LB 5
#define BUTTON_RB 6
#define BUTTON_BACK 7
#define BUTTON_START 8
#define BUTTON_L3 9
#define BUTTON_R3 10

#define LEFT_X 1
#define LEFT_Y 2
#define TRIGGERS 3
#define RIGHT_X 4
#define RIGHT_Y 5

#define LEG_FORWARD .01

//#define PracticeBot
#define CompetitionBot

#ifdef PracticeBot
	#define ARM_DUMP  2.85
	#define ARM_PICKUP  4.5917
	#define ARM_BRIDGE 4.15
	#define ARM_BRIDGEAUTON 3.85
	#define ARM_GROUND 4.60
	#define	ARM_BALL 4.40
	#define ARM_FRAME 3.8
	#define ARM_BALANCE 4.9
	#define ARM_BARRIER 4.3
	#define FENDER_SHOT 66
	#define BOTTOMKEY_SHOT 97
	#define TOPKEY_SHOT 102
	#define HAILMARY_SHOT 500
	#define REV_IN 11.416
#endif

#ifdef CompetitionBot
	#define ARM_DUMP 1.23
	#define ARM_PICKUP 2.96
	#define ARM_BRIDGE 2.46
	#define ARM_BRIDGEAUTON 1.8552
	#define ARM_GROUND 3.0
	#define ARM_FRAME 2.54	
	#define ARM_BALANCE 3.18
	#define ARM_BARRIER 2.50
	#define FENDER_SHOT 66
	#define BOTTOMKEY_SHOT 99
	#define TOPKEY_SHOT 104
	#define HAILMARY_SHOT 500
	#define REV_IN 16.54
#endif
