/*
 * RobotMotion.c  -- station transport for the LINE 7 peptide deck.
 *
 * Rail control reuses LINE 7's Rtelligent `servo` driver (servo.h) on the
 * shared RS485 bus at address ROBOT_LINEAR_ID. Arm poses are commanded through
 * RobotCommand's moveRobotTo(). moveVesselTo() combines the two: it retracts
 * the arm from the current station, travels along the rail, and inserts at the
 * target station.
 */

#include "RobotMotion.h"
#include "servo.h"
#include "cmsis_os.h"
#include "uart.h"
#include "RobotCommand.h"
#include "Load.h"          /* load cell: weight read + stepper dose  */
#include "main.h"          /* Drain_valve_pin_Pin_* GPIO defines      */
#include "stepper.h"       /* drainStepperLimit() -- stepper upper limit */
#include "FreeRTOS.h"      /* taskENTER_CRITICAL() for the bus lock      */
#include "task.h"

/* The shared servo bus handle, created in main.c. Rail = ROBOT_LINEAR_ID. */
extern ServoHandle *servo;

/* Load cell / vibratory feeder handle, created in main.c. */
extern LoadCellHandle *loadcell;

/* Ramped stepper dose driven by load-cell feedback -- this is the exact
 * routine behind the stepper menu's option 3. Defined in Machinemenu.c
 * (its `static` was removed so it can be shared here). */
bool LoadCellStepperDispenseRamped(LoadCellHandle *loadcell,
		uint16_t targetWeight, int direction, int stepSpeed);

/* ---------------------------------------------------------------------------
 *  TAUGHT ARM POSES  (FAIRINO MoveJ: 12 decimal values "j1..j6, x,y,z,rx,ry,rz")
 *
 *  Order MUST match the PositionID enum in RobotCoordinates.h. Jog to each pose
 *  on the pendant, read the joint + cartesian values, and paste the string
 *  verbatim -- NO scaling. POS_HOME is LINE 1's REFERENCE (a safe home for this
 *  arm); the station poses currently default to it -- TEACH each one on the
 *  physical deck before running an actual synthesis.
 * ------------------------------------------------------------------------- */
#define POSE_REFERENCE \
	"90.510,-98.471,153.157,-233.025,-3.077,-94.190,203.398,-172.244,351.847,92.018,-87.467,85.419"

/* LINE 7 HOME / travel pose (operator-taught, command "1").
 * Format: j1,j2,j3,j4,j5,j6, x,y,z,rx,ry,rz  -- decimals, NO scaling.
 * Shared by POS_HOME (global travel pose) and the liquid module's Home slot
 * (liquidPoses[0]). Re-taught 2026-07-16 (Pic 1) -- no longer tied to cleavage
 * Position 1, which keeps its own literal in cleavagePoses[0]. */
#define POSE_HOME \
	"-3.013,-91.987,155.591,-244.242,-89.248,94.134,-348.922,-142.245,469.843,81.213,85.826,-101.026"
#define POSE_POWDER1 \
	"-98.605, -87.179, 149.136, -242.429, -96.462, 94.146, -83.425, 431.667, 476.817, 83.606, 85.774, 158.556"
#define POSE_POWDER2 \
	"-98.611, -57.82, 134.216, -248.212, -105.015, 94.146, -35.998, 640.819, 301.173, 165.803, 81.847, -128.109"
//#define POSE_POWDER3 \
	"-91.158, -82.125, 155.562, -254.794, -103.525, 94.169, -126.839, 365.949, 428.127, 73.614, 85.321, 148.979"
//#define POSE_POWDER4 \
	"-4.124, -82.148, 154.719, -254.868, -89.256, 94.175, -383.032, -133.177, 432.493, 60.953, 85.258, -122.342"


//#define POSE_STIR1 \
	"-87.572, -91.227,155.597, -244.248, -89.266, 94.143, -174.818, 337.622, 465.269, 91.697, 85.854, -175.146"
//#define POSE_STIR2 \
	"-89.154, -48.558, 121.959, -253.498, -89.366, 94.166, -171.037, 734.064, 257.332, 88.673, 85.826, -179.846"
//#define POSE_STIR3 \
	"-87.758, -45.363, 109.989, -245.035, -94.241, 94.166, -184.214, 841.288, 263.346, 84.522, 85.779, 172.537"
//#define POSE_STIR4 \
	"-87.757, -63.735, 141.376, -252.337, -94.249, 94.165, -172.598, 544.648, 339.026, 144.626, 83.505, -127.573"
//#define POSE_STIR5 \
	"-87.751, -84.978, 155.101, -252.081, -94.276, 94.166, -165.193, 356.059, 443.776, 65.592, 85.267, 153.636"
//#define POSE_STIR6 \
	"-1.298, -88.447, 149.85, -248.542, -86.443, 94.166, -393.932, -156.667, 491.494, 27.437, 81.966, -150.048"


#define POSE_STIR1 \
	"-85.142, -89.296, 152.066, -244.128, -92.923, 94.135, -185.215, 365.495, 475.367, 72.103, 85.582, 164.087"
#define POSE_STIR2 \
	"-89.378, -47.847, 134.886, -261.892, -86.538, 94.14, -172.076, 613.826, 217.779, 139.189, 83.206, -126.909"
#define POSE_STIR3 \
	"-89.363, -47.839, 128.963, -254.723, -86.319, 94.142, -173.329, 676.217, 221.399, 144.649, 82.159, -121.264"
#define POSE_STIR4 \
	"-89.363, -47.839, 128.963, -254.723, -86.319, 94.142, -173.329, 676.217, 221.399, 144.649, 82.159, -121.264"
#define POSE_STIR5 \
	"-89.811, -46.856, 123.229, -249.674, -84.93, 94.147, -170.786, 734.349, 222.147, 144.753, 81.821, -120.22"
//#define POSE_STIR6 \
	"-93.292, -40.099, 104.967, -241.105, -88.599, 94.145, -109.79, 905.354, 207.146, 131.673, 84.336, -140.354"
//#define POSE_STIR7 \
	"-95.089, -42.91, 128.082, -263.697, -84.987, 94.137, -108.117, 682.796, 183.917, 109.023, 85.489, -161.107"
//#define POSE_STIR8 \
	"-91.268, -54.022, 135.597, -259.746, -94.876, 94.146, -136.536, 609.458, 277.358, 114.579, 85.613, -161.629"
//#define POSE_STIR9 \
	"-91.071, -73.672, 153.62, -259.751, -94.986, 94.147, -142.189, 407.796, 388.399, 92.727, 85.865, 176.657"
//#define POSE_STIR10 \
	"0.039, -83.467, 150.922, -253.517, -90.809, 94.148, -397.567, -157.772, 458.789, 34.817, 82.611, -145.735"

//Coordinates for Reposture of the robot
#define POSE_RESTIR1 \
	"-85.142, -89.296, 152.066, -244.128, -92.923, 94.135, -185.215, 365.495, 475.367, 72.103, 85.582, 164.087"
#define POSE_RESTIR2 \
	"-89.378, -47.847, 134.886, -261.892, -86.538, 94.14, -172.076, 613.826, 217.779, 139.189, 83.206, -126.909"
#define POSE_RESTIR3 \
	"-86.443, -44.367, 129.699, -262.176, -94.967, 94.157, -190.469, 649.252, 193.438, 129.038, 85.006, -142.487"
#define POSE_RESTIR4 \
	"-86.448, -42.93, 120.507, -255.579, -94.976, 94.158, -195.865, 737.389, 194.744, 116.567, 85.547, -154.927"
#define POSE_RESTIR5 \
	"-89.811, -46.856, 123.229, -249.674, -84.93, 94.147, -170.786, 734.349, 222.147, 144.753, 81.821, -120.22"
//#define POSE_RESTIR6 \
	"-93.479, -37.288, 103.808, -247.546, -89.088, 94.152, -106.123, 902.435, 182.486, 76.045, 85.739, 163.515"
//#define POSE_RESTIR7 \
	"-93.435, -41.624, 111.381, -246.446, -89.067, 94.152, -110.207, 846.539, 202.858, 128.261, 84.649, -144.361"
//#define POSE_RESTIR8 \
	"-93.505, -57.081, 136.576, -250.809, -89.069, 94.152, -123.183, 617.926, 288.117, 153.879, 80.319, -119.006"
//#define POSE_RESTIR9 \
	"-93.506, -82.72, 151.299, -250.63, -89.158, 94.153, -135.478, 413.634, 446.557, 63.526, 85.396, 150.928"
//#define POSE_RESTIR10 \
	"2.069, -78.771, 142.078, -250.634, -94.951, 94.154, -484.019, -167.436, 469.583, 33.098, 81.277, -149.519"

#define POWDER_RAIL_TURNS   50   /* rail turns to run before the arm sequence */
const char *powderPoses[POWDER_NUM_POS] = {
	/* 0 Home    */ POSE_HOME,

	/* 1 Powder1 */ POSE_POWDER1,
	/* 2 Powder2 */ POSE_POWDER2,
	/* 3 Powder3 */ //POSE_POWDER3,
	/* 4 Powder4 */ //POSE_POWDER4,
};

static const int   powderSeq[POWDER_NUM_STEPS]     = {   0,        1,          2,          2,          1,          0    };
static const char *powderSeqName[POWDER_NUM_STEPS] = { "HOME", "POWDER 1", "POWDER 2", "POWDER 2", "POWDER 1", "HOME" };

#define STIR_RAIL_TURNS   100   /* rail turns to run before the rail returns home */

const char *stirPoses[STIR_NUM_POS] = {
	/* 0 Home  */ POSE_HOME,
	/* 1 Stir1 */ POSE_STIR1,
	/* 2 Stir2 */ POSE_STIR2,
	/* 3 Stir3 */ POSE_STIR3,
	/* 4 Stir4 */ POSE_STIR4,
	/* 5 Stir5 */ POSE_STIR5
	/* 6 Stir6 */ //POSE_STIR6,
	              //POSE_STIR7,
				  //POSE_STIR8,
				  //POSE_STIR9,
				  //POSE_STIR10
};

/* Playback order: HOME -> STIR1..6 -> HOME. Each entry indexes stirPoses[] so
 * HOME is reused at both ends without duplicating its string. */
/* Reposture (RESTIR) pose table for runReactionReStirModule(): HOME + the
 * 10 POSE_RESTIR coordinates. Walked with the SAME stirSeq/stirSeqName order
 * below, so only the coordinates differ from stirPoses[]. */
const char *stirRePoses[STIR_NUM_POS] = {
	/* 0 Home   */ POSE_HOME,
	/* 1 ReStir1*/ POSE_RESTIR1,
	/* 2 ReStir2*/ POSE_RESTIR2,
	/* 3 ReStir3*/ POSE_RESTIR3,
	/* 4 ReStir4*/ POSE_RESTIR4,
	/* 5 ReStir5*/ POSE_RESTIR5,
	/* 6 ReStir6*/ //POSE_RESTIR6,
	               //POSE_RESTIR7,
	               //POSE_RESTIR8,
	               //POSE_RESTIR9,
	               //POSE_RESTIR10
};

static const int   stirSeq[STIR_NUM_STEPS]     = {   0,       1,        2,        3,        4,     5,     4,      3,        2,      1,    0    };
static const char *stirSeqName[STIR_NUM_STEPS] = { "HOME", "STIR 1", "STIR 2", "STIR 3", "STIR 4", "STIR5",  "STIR4",  "STIR 3", "STIR 2", "STIR 1", "HOME" };

/* ---------------------------------------------------------------------------
 *  FULL SEQUENCE  (all rail motion is here; the modules do arm moves only)
 *
 *  rail +650 -> POWDER -> wait 1min -> rail home -> LIQUID -> wait 1min
 *            -> rail +100 -> STIR -> rail home
 * ------------------------------------------------------------------------- */
#define SEQ_POWDER_TURNS   30    /* home -> +30 = powder+liquid station */
#define SEQ_STIR_TURNS     82    /* +70 from the +30 station -> +100 = stir */
#define SEQ_WAIT_MS        500u   /* 1 minute between stages */

const char *positions[NUM_POS] = {
	/* POS_HOME     */ POSE_HOME,

	/* POS_ST1_APP  */ POSE_REFERENCE,   /* TEACH */
	/* POS_ST1_WORK */ POSE_REFERENCE,   /* TEACH */

	/* POS_ST2_APP  */ POSE_REFERENCE,   /* TEACH */
	/* POS_ST2_WORK */ POSE_REFERENCE,   /* TEACH */

	/* POS_ST3_APP  */ POSE_REFERENCE,   /* TEACH */
	/* POS_ST3_WORK */ POSE_REFERENCE,   /* TEACH */

	/* POS_ST5_APP  */ POSE_REFERENCE,   /* TEACH */
	/* POS_ST5_WORK */ POSE_REFERENCE,   /* TEACH */

	/* POS_ST9_APP  */ POSE_REFERENCE,   /* TEACH */
	/* POS_ST9_WORK */ POSE_REFERENCE,   /* TEACH */

	/* POS_ST10_APP */ POSE_REFERENCE,   /* TEACH */
	/* POS_ST10_WORK*/ POSE_REFERENCE,   /* TEACH */

	/* POS_PARK_APP */ POSE_REFERENCE,   /* TEACH */
	/* POS_PARK_WORK*/ POSE_REFERENCE,   /* TEACH */
};

/* Trailing MoveJ params (tool,user,vel,acc,ovl,exaxis[4],blendT,offset...) --
 * identical to LINE 1's DEFAULT_EXTRA. */
const char *DEFAULT_EXTRA =
	"0, 0, 100.0, 100.0, 100.0, 0.0, 0.0, 0, 0,0,0,0,0,0,0,0,0,0";

static bool railConfigured = false;

/* ---------------------------------------------------------------------------
 *  PNEUMATIC GRIPPERS  (hook / unhook)
 *
 *  Two solenoid-driven grippers ported from the 555 Reaction Module:
 *    - the robot ARM gripper, which grabs/releases the reactor vessel, and
 *    - the CLEAVAGE-module clamp, which holds the vessel in the station.
 *
 *  A vessel hand-off is always "the receiver hooks BEFORE the holder unhooks",
 *  so the vessel is never released by both at once. Each call blocks for
 *  GRIP_SETTLE_MS so the pneumatic actuator seats before the next arm move.
 *  Pins are aliased in main.h; active levels + settle time are in RobotMotion.h.
 * ------------------------------------------------------------------------- */
//void robotArmGripperHook(void)
//{
//	HAL_GPIO_WritePin(Robot_Gripper_GPIO_Port, Robot_Gripper_Pin, ROBOT_GRIP_HOOK_LEVEL);
//	UART_Print("[GRIP] arm gripper -> HOOK (grab vessel)\r\n");
//	osDelay(GRIP_SETTLE_MS);
//}

//void robotArmGripperUnhook(void)
//{
//	HAL_GPIO_WritePin(Robot_Gripper_GPIO_Port, Robot_Gripper_Pin, ROBOT_GRIP_UNHOOK_LEVEL);
//	UART_Print("[GRIP] arm gripper -> UNHOOK (release vessel)\r\n");
//	osDelay(GRIP_SETTLE_MS);
//}

//void cleavageGripHook(void)
//{
//	HAL_GPIO_WritePin(Cleavage_Grip_GPIO_Port, Cleavage_Grip_Pin, CLEAVAGE_GRIP_HOOK_LEVEL);
//	UART_Print("[GRIP] cleavage clamp -> HOOK (hold vessel)\r\n");
//	osDelay(GRIP_SETTLE_MS);
//}

//void cleavageGripUnhook(void)
//{
//	HAL_GPIO_WritePin(Cleavage_Grip_GPIO_Port, Cleavage_Grip_Pin, CLEAVAGE_GRIP_UNHOOK_LEVEL);
//	UART_Print("[GRIP] cleavage clamp -> UNHOOK (release vessel)\r\n");
//	osDelay(GRIP_SETTLE_MS);
//}

/* ---------------------------------------------------------------------------
 *  CLEAVAGE MODULE  (command "2")  -- reactor vessel pick & place.
 *
 *  Operator-taught poses, executed one-by-one in ASCENDING order (Pos 1..6):
 *  the arm picks up the reactor vessel and returns it into the cleavage module.
 *  Each entry is a FAIRINO MoveJ pose "j1..j6, x,y,z,rx,ry,rz" -- decimals, NO
 *  scaling.
 *
 *  CORRECTED 2026-07-09: Position 3 (J4/X) and Position 5 (J2/J4) had
 *  sign-flipped joints that no longer matched their cartesian target -- the
 *  FR10 rejected Position 3 with "robot errcode:32" (invalid/out-of-range
 *  target). Values below are the operator's re-verified taught poses.
 * ------------------------------------------------------------------------- */
const char *cleavagePoses[CLEAVAGE_NUM_POS] = {
		"-2.895, -74.405, 134.359, -234.978, -88.569, 94.138, -604.414, -131.289, 451.429, 139.458, 83.458, -42.179",
		"-2.003, -53.469, 96.448, -227.982, -84.463, 94.145, -945.901, -136.257, 465.738, 36.242, 83.823, -140.044",
		"-2.025, -53.464, 91.437, -216.236, -85.099, 94.142, -993.015, -133.047, 492.629, 111.985, 85.374, -65.199",
		"-2.003, -53.469, 96.448, -227.982, -84.463, 94.145, -945.901, -136.257, 465.738, 36.242, 83.823, -140.044",
		"-2.895, -74.405, 134.359, -234.978, -88.569, 94.138, -604.414, -131.289, 451.429, 139.458, 83.458, -42.179",
};

/* Settle time between two module poses. Gives the FR10 time to finish
 * decelerating and clear its "in-motion" state before the next MoveJ, so a
 * command is never sent while the controller is still busy (avoids spurious
 * rejects / apparent hangs). Shared by all pose-playback modules below. */
/* 5000 -> 1000 -> 200. This is PURE EXTRA SETTLE, not move spacing:
 * robotMoveJStr() -> robotMoveJBody() ends in waitTillMotionComplete(), so the
 * arm has already finished the previous pose before this delay even starts.
 * The controller cannot be mid-move here, which is why it can be this short.
 * Raise it again if a pose visibly rings or overshoots before the next MoveJ. */
#define MODULE_STEP_DELAY_MS    200    /* between-pose settle, all modules */
#define STIR5_PUMP_DWELL_MS   3000u  /* hold at STIR5 with pump ON before returning */

/* Run the whole cleavage pick-and-place, blocking on motion-complete between
 * moves. Aborts (and returns false) the moment any pose is rejected/times out. */
bool runCleavageModule(void)
{
	UART_Print("\r\n===== CLEAVAGE MODULE : reactor vessel pick & place =====\r\n");

	if (connectRobot() != 0) {
		UART_Print("[CLEAVAGE] Robot not connected -- aborting.\r\n");
		return false;
	}

	/* Re-assert AUTO mode + enable. SetErrStateHoldEnable(1) latches a fault
	 * until the robot is re-enabled, so a previous aborted run would otherwise
	 * make every move fail with errcode:14. This clears that latch and fails
	 * fast (with guidance) if the arm is genuinely not in AUTOMATIC mode. */
	if (robotInitConfig() != 0) {
		UART_Print("[CLEAVAGE] Robot NOT READY -- aborting before any motion.\r\n"
		           "   Clear faults + switch to AUTOMATIC on the pendant/WebApp, then retry.\r\n");
		return false;
	}

	for (int i = 0; i < CLEAVAGE_NUM_POS; i++) {
		UART_Print("\r\n[CLEAVAGE] ---- Position %d of %d ----\r\n", i + 1, CLEAVAGE_NUM_POS);
		if (robotMoveJStr(cleavagePoses[i]) != 0) {
			UART_Print("[CLEAVAGE] Position %d FAILED -- sequence ABORTED.\r\n", i + 1);
			return false;
		}
		osDelay(MODULE_STEP_DELAY_MS);   /* settle before next pose */
	}

	UART_Print("\r\n[CLEAVAGE] Sequence COMPLETE -- vessel returned to cleavage module.\r\n");
	return true;
}

/* ---------------------------------------------------------------------------
 *  LIQUID DISPENSER MODULE  (command "3")
 *  Round trip: HOME -> Position 2 -> Position 3 -> Position 2 -> HOME.
 *
 *  Operator-taught poses. The arm starts at HOME, visits the two liquid-
 *  dispenser stations, then retraces Position 2 and HOME on the way back.
 *  FAIRINO MoveJ poses "j1..j6, x,y,z,rx,ry,rz" -- decimals, NO scaling.
 *  Re-taught with the operator 2026-07-16. liquidPoses[0] reuses POSE_HOME.
 * ------------------------------------------------------------------------- */
const char *liquidPoses[LIQUID_NUM_POS] = {
	/* Home       */ POSE_HOME,
	/* Position 2 */ //"-98.318,-51.407,153.291,-262.809,-90.039,97.035,-91.664,471.626,226.780,160.531,69.719,-119.009",
	/* Position 3 */ //"-98.325,-37.267,120.904,-263.507,-88.543,89.217,-54.545,743.980,135.235,-99.543,89.209,-16.411",
	//"83.434, -84.807, 146.962, -244.247, -84.078, 94.133, -218.939, 415.963, 476.751, 61.982, 85.565, 164.545"
	//"-97.547, -86.325, 149.98, -244.226, -81.848, 89.192, -118.841, 428.018, 468.459, -57.581, 88.947, 33.019",
	//"-62.489, -40.617, 133.449, -264.081, -111.324, 89.193, -391.974, 491.912, 148.064, -153.959, 80.925, -57.719"
	//"-67.599, -33.655, 112.633, -259.854, -11.427, 89.193, -412"
	"-72.988, -91.883, 157.808, -244.219, -89.293, 94.135, -248.494, 264.248, 455.395, 112.329, 85.508, -140.015",
	"-72.971, -47.717, 142.351, -244.193, -89.328, 94.143, -320.955, 500.714, 158.383, 172.332, 59.257, -81.095",
	//"-72.599, -33.386, 115.535, -254.927, -95.767, 94.136, -375.868, 703.269, 84.528, 154.758, 82.053, -103.866",
	"-72.518, -34.202, 117.769, -258.459, -95.022, 89.179, -371.093, 679.837, 95.317, -166.034, 84.756, -63.537"
	//"-71.833, -33.572, 118.551, -254.907, -95.755, 94.143, -377.407, 674.233, 77.134, 162.869, 79.509, -95.078",
	//"-67.409, -69.915, 152.693, -254.794, -95.754, 94.144, -303.838, 344.006, 354.323, 157.348, 81.388, -96.103",
	//"-65.203, -89.803, 155.017, -245.619, -98.525, 94.152, -277.751, 259.539, 462.716, 84.568, 85.769, -169.145",

};

/* Playback order for the round trip HOME -> Pos2 -> Pos3 -> Pos2 -> HOME. Each
 * entry indexes liquidPoses[] (0=Home, 1=Position 2, 2=Position 3), so the two
 * taught station poses are revisited on the way back without duplicating their
 * coordinate strings. */
static const int   liquidSeq[LIQUID_NUM_STEPS]     = {    0,        1,            2,            3    ,   3        ,   2      ,   1,  0};
static const char *liquidSeqName[LIQUID_NUM_STEPS] = { "HOME", "Pos1", "Pos2", "Pos3 (POUR)", "Pos3 (POUR)", "Pos2", "Pos1", "HOME" };

/* Run HOME -> Pos2 -> Pos3 -> Pos2 -> HOME, blocking on motion-complete between
 * moves. Aborts (and returns false) the moment any pose is rejected/times out. */
bool runLiquidDispenserModule(void)
{
	UART_Print("\r\n===== LIQUID DISPENSER MODULE : HOME -> Pos1..Pos6 -> HOME =====\r\n");
	HAL_GPIO_WritePin(Robot_gripper_GPIO_Port, Robot_gripper_Pin, GPIO_PIN_SET);
	if (connectRobot() != 0) {
		UART_Print("[LIQUID] Robot not connected -- aborting.\r\n");
		return false;
	}

	/* Same fault-clearing re-assert as the cleavage module (see there). */
	if (robotInitConfig() != 0) {
		UART_Print("[LIQUID] Robot NOT READY -- aborting before any motion.\r\n"
		           "   Clear faults + switch to AUTOMATIC on the pendant/WebApp, then retry.\r\n");
		return false;
	}

	for (int i = 0; i < LIQUID_NUM_STEPS; i++) {
		UART_Print("\r\n[LIQUID] ---- %s (step %d of %d) ----\r\n",
		           liquidSeqName[i], i + 1, LIQUID_NUM_STEPS);
		if (robotMoveJStr(liquidPoses[liquidSeq[i]]) != 0) {
			UART_Print("[LIQUID] %s FAILED -- sequence ABORTED.\r\n", liquidSeqName[i]);
			return false;
		}
		osDelay(MODULE_STEP_DELAY_MS);   /* settle before next pose */
	}

	UART_Print("\r\n[LIQUID] Sequence COMPLETE.\r\n");

	return true;

}


/* ---------------------------------------------------------------------------
 *  LIQUID MODULE, SPLIT IN TWO  (used by sequence())
 *
 *  runLiquidDispenserModule() above is the full ROUND TRIP
 *      HOME -> Pos2 -> Pos3 -> Pos2 -> HOME
 *  so by the time it RETURNS the arm is back at HOME. sequence() was calling it
 *  and only then signalling ROBOT_READY, so the drain valve opened after the
 *  robot had already visited the pour pose AND left again -- the liquid went
 *  nowhere. That is the bug being fixed here.
 *
 *  Split so the sequence can stop AT the pour pose, hold there while the valve
 *  is open, and retract only once the liquid has been collected:
 *
 *      approach : steps 0..2  HOME -> Pos2 -> Pos3   (ends AT Pos3, the pour pose)
 *      >>> valve opens, liquid collected, valve closes <<<
 *      retract  : steps 3..4  Pos2 -> HOME
 * ------------------------------------------------------------------------- */

#define LIQUID_APPROACH_STEPS   4   /* steps 0,1,2,3 -- ends AT the pour pose
                                     * (Pos3, the deepest reach). Retract = 4..7. */

/* HOME -> Pos2 -> Pos3. Leaves the arm AT the pour pose, holding the vessel. */
bool runLiquidDispenserApproach(void)
{
	UART_Print("\r\n===== LIQUID APPROACH : home -> pos2 -> pos3 (stop at pour) =====\r\n");
	//HAL_GPIO_WritePin(Robot_gripper_GPIO_Port, Robot_gripper_Pin, GPIO_PIN_SET);

	if (connectRobot() != 0) {
		UART_Print("[LIQUID] Robot not connected -- aborting.\r\n");
		return false;
	}

	if (robotInitConfig() != 0) {
		UART_Print("[LIQUID] Robot NOT READY -- aborting before any motion.\r\n"
		           "   Clear faults + switch to AUTOMATIC on the pendant/WebApp, then retry.\r\n");
		return false;
	}

	for (int i = 0; i < LIQUID_APPROACH_STEPS; i++) {
		UART_Print("\r\n[LIQUID] ---- %s (approach step %d of %d) ----\r\n",
		           liquidSeqName[i], i + 1, LIQUID_APPROACH_STEPS);

		if (robotMoveJStr(liquidPoses[liquidSeq[i]]) != 0) {
			UART_Print("[LIQUID] %s FAILED -- approach ABORTED.\r\n", liquidSeqName[i]);
			return false;
		}
		osDelay(MODULE_STEP_DELAY_MS);
	}

	UART_Print("\r\n[LIQUID] At pour pose -- holding for the liquid.\r\n");
	return true;
}

/* Pos2 -> HOME. Call only from the pour pose, after the liquid is collected. */
bool runLiquidDispenserRetract(void)
{
	UART_Print("\r\n===== LIQUID RETRACT : pos3 -> pos2 -> home =====\r\n");

	HAL_GPIO_WritePin(Robot_gripper_GPIO_Port, Robot_gripper_Pin, GPIO_PIN_SET);

	for (int i = LIQUID_APPROACH_STEPS; i < LIQUID_NUM_STEPS; i++) {
		UART_Print("\r\n[LIQUID] ---- %s (retract step %d of %d) ----\r\n",
		           liquidSeqName[i], i - LIQUID_APPROACH_STEPS + 1,
		           LIQUID_NUM_STEPS - LIQUID_APPROACH_STEPS);

		if (robotMoveJStr(liquidPoses[liquidSeq[i]]) != 0) {
			UART_Print("[LIQUID] %s FAILED -- retract ABORTED.\r\n", liquidSeqName[i]);
			return false;
		}
		osDelay(MODULE_STEP_DELAY_MS);
	}

	UART_Print("\r\n[LIQUID] Retracted to HOME.\r\n");
	return true;
}

/* ---------------------------------------------------------------------------
 *  Station -> {approach pose, work pose, rail turns}
 * ------------------------------------------------------------------------- */
typedef struct {
	PositionID app;
	PositionID work;
	int        railTurns;
} StationDef;

static const StationDef stationDefs[NUM_STATIONS] = {
	[ST_HOME]     = { POS_HOME,      POS_HOME,      RAIL_HOME },
	[ST_LIQUID]   = { POS_ST1_APP,   POS_ST1_WORK,  RAIL_ST1  },
	[ST_SOLID]    = { POS_ST2_APP,   POS_ST2_WORK,  RAIL_ST2  },
	[ST_VOLATILE] = { POS_ST3_APP,   POS_ST3_WORK,  RAIL_ST3  },
	[ST_STIR]     = { POS_ST5_APP,   POS_ST5_WORK,  RAIL_ST5  },
	[ST_DRAIN]    = { POS_ST9_APP,   POS_ST9_WORK,  RAIL_ST9  },
	[ST_CLEAVE]   = { POS_ST10_APP,  POS_ST10_WORK, RAIL_ST10 },
	[ST_PARK]     = { POS_PARK_APP,  POS_PARK_WORK, RAIL_PARK },
};

static Station currentStation   = ST_HOME;
static int     currentRailTurns = RAIL_HOME;

Station getCurrentStation(void) { return currentStation; }

/* =========================================================
 *  RAIL (Rtelligent servo) CONFIG
 * ========================================================= */
bool ConfigRobotLinear(void)
{
	bool status = true;

	osDelay(50);
	status &= ServoWrite16Verified(servo, ROBOT_LINEAR_ID, SERVO_POS_MODE,     INCREMENTAL);
	osDelay(50);
	status &= ServoWrite16Verified(servo, ROBOT_LINEAR_ID, SERVO_POS_ACC_TIME, ROBOT_ACCEL);
	osDelay(50);
	status &= ServoWrite16Verified(servo, ROBOT_LINEAR_ID, SERVO_POS_DEC_TIME, ROBOT_DECEL);
	osDelay(50);
	status &= ServoWrite16Verified(servo, ROBOT_LINEAR_ID, SERVO_POS_SPEED,    ROBOT_SPEED);
	osDelay(50);
	status &= ServoWrite16Verified(servo, ROBOT_LINEAR_ID, ORGIN_SPEED_CMD,    ROBOT_ORGIN_SPEED);
	osDelay(50);

	UART_Print(status ? "Robot Rail Config Success\r\n" : "Robot Rail Config Failed\r\n");
	return status;
}

/* =========================================================
 *  RAIL MOVE (relative, +/- turns)
 * ========================================================= */
bool MoveLinearServo(int turns)
{
	if (!railConfigured)
	    {
	        if (!ConfigRobotLinear())
	        {
	            UART_Print("Rail configuration failed\r\n");
	            return false;
	        }
	        railConfigured = true;
	    }

	if (turns == 0)
		return true;

	int32_t  position  = (int32_t)(SERVO_PPR * ROBOT_GEAR_RATIO * turns);
	uint16_t direction = (position >= 0) ? SERVO_FORWARD : SERVO_REVERSE;

	int32_t pos;

	if (ServoSafeReadPosition(servo, ROBOT_LINEAR_ID,
	                          READ_CURRENT_POS,
	                          &pos))
	{
	    UART_Print("Rail found. Pos=%ld\r\n", pos);
	}
	else
	{
	    UART_Print("Rail not responding\r\n");
	}

	UART_Print("Rail target : %ld pulses (%d turns)\r\n", (long)position, turns);



	//if (!ServoWrite32(servo, ROBOT_LINEAR_ID, SERVO_POSITION, position)) {
	//	UART_Print("Rail position write failed\r\n");
	//	return false;
	//}
	ServoWrite32(servo, ROBOT_LINEAR_ID, SERVO_POSITION, position);
	osDelay(50);

	//if (!ServoWrite16(servo, ROBOT_LINEAR_ID, SERVO_MOTION_CMD, direction)) {
	//	UART_Print("Rail motion start ACK failed\r\n");
	//	return false;
	//}
	ServoWrite16(servo, ROBOT_LINEAR_ID, SERVO_MOTION_CMD, direction);
	osDelay(20);

	bool done = ServoWaitMotionComplete(servo, ROBOT_LINEAR_ID, READ_CURRENT_POS, 180000);

	//if (!ServoWrite16(servo, ROBOT_LINEAR_ID, SERVO_MOTION_CMD, SERVO_STOP))
	//	UART_Print("Rail stop ACK failed\r\n");

	ServoWrite16(servo, ROBOT_LINEAR_ID, SERVO_MOTION_CMD, SERVO_STOP);
	osDelay(20);

	UART_Print(done ? "Rail reached\r\n" : "Rail motion timeout\r\n");
	return done;
}

void robotLinearStop(void)
{
	if (!ServoWrite16Ack(servo, ROBOT_LINEAR_ID, SERVO_MOTION_CMD, SERVO_STOP))
		UART_Print("Rail stop ACK failed\r\n");
	osDelay(20);
}

bool robotLinearHome(void)
{
	if (!ServoWrite16Ack(servo, ROBOT_LINEAR_ID, SERVO_HOME_TRIGGER, SERVO_HOMING_START))
		UART_Print("Rail home ACK failed\r\n");
	osDelay(20);

	bool done = ServoWaitMotionComplete(servo, ROBOT_LINEAR_ID, READ_CURRENT_POS, 180000);
	robotLinearStop();

	if (done) {
		currentRailTurns = RAIL_HOME;
		UART_Print("Rail homed\r\n");
	} else {
		UART_Print("Rail home timeout\r\n");
	}
	return done;
}

void setCurrentPositionAsHome(void)
{
	if (!ServoWrite16Ack(servo, ROBOT_LINEAR_ID, SERVO_HOME_TRIGGER, SERVO_CURRENT_HOME))
		UART_Print("Rail set-home ACK failed\r\n");
	osDelay(20);
	currentRailTurns = RAIL_HOME;
}

/* =========================================================
 *  ARM HELPERS
 * ========================================================= */
void moveRobotToHomePosition(void)
{
	if (connectRobot() != 0) {
		UART_Print("[HOME] robot not connected -- aborting.\r\n");
		return;
	}
	/* Re-assert AUTO/enable so a fault latched by an earlier aborted move is
	 * cleared before we try to home (else HOME would just errcode:14 too). */
	if (robotInitConfig() != 0) {
		UART_Print("[HOME] robot NOT READY (not in AUTOMATIC / faulted) -- aborting.\r\n");
		return;
	}
	if (moveRobotTo(POS_HOME) != 0) {
		UART_Print("[HOME] move FAILED (see message above).\r\n");
		return;
	}
	osDelay(500);
	DBG_REACHED();
}

bool robotResetToHome(void)
{
	UART_Print("\r\n[ROBOT] Reset to home\r\n");
	moveRobotTo(POS_HOME);
	osDelay(500);
	bool ok = robotLinearHome();
	currentStation   = ST_HOME;
	currentRailTurns = RAIL_HOME;
	return ok;
}

/* =========================================================
 *  FULL VESSEL TRANSPORT  current -> target
 *
 *  Sequence (mirrors the LINE 5 approach/retract discipline):
 *    1. retract arm from current station  (WORK -> APP -> HOME travel pose)
 *    2. move the rail by the station delta
 *    3. insert arm at target station      (HOME -> APP -> WORK)
 * ========================================================= */
bool moveVesselTo(Station target)
{
	if (target < 0 || target >= NUM_STATIONS) {
		UART_Print("[ROBOT] invalid station %d\r\n", target);
		return false;
	}
	if (target == currentStation)
		return true;

	const StationDef *from = &stationDefs[currentStation];
	const StationDef *to   = &stationDefs[target];

	UART_Print("\r\n[ROBOT] transport %d -> %d\r\n", currentStation, target);

	/* 1. retract from current station */
	if (currentStation != ST_HOME) {
		if (moveRobotTo(from->app)  != 0) return false;
		osDelay(300);
	}
	if (moveRobotTo(POS_HOME) != 0) return false;
	osDelay(300);

	/* 2. move rail to target lane */
	int delta = to->railTurns - currentRailTurns;
	if (delta != 0) {
		if (!MoveLinearServo(delta)) return false;
		currentRailTurns = to->railTurns;
		osDelay(300);
	}

	/* 3. insert at target station */
	if (target != ST_HOME) {
		if (moveRobotTo(to->app)  != 0) return false;
		osDelay(300);
		if (moveRobotTo(to->work) != 0) return false;
		osDelay(300);
	}

	currentStation = target;
	UART_Print("[ROBOT] at station %d\r\n", target);
	return true;
}

/* Return the rail to the home you set with setCurrentPositionAsHome().
 *
 * setCurrentPositionAsHome() zeroes the drive's position counter at home, so
 * the current feedback IS the distance from home. We read it, convert pulses
 * back to turns, and do the reverse relative move. Works regardless of how many
 * MoveLinearServo() calls got us here -- no software counter to drift. */
bool robotLinearReturnToHome(void)
{
	int32_t pos = 0;

	if (!ServoSafeReadPosition(servo, ROBOT_LINEAR_ID, READ_CURRENT_POS, &pos)) {
		UART_Print("Rail position read failed -- cannot return home\r\n");
		return false;
	}

	/* pulses per turn = SERVO_PPR * ROBOT_GEAR_RATIO (same factor MoveLinearServo
	 * uses). Round to the nearest whole turn (console moves are integer turns,
	 * so this is exact). */
	double pulsesPerTurn = (double)SERVO_PPR * ROBOT_GEAR_RATIO;
	int    turnsBack     = -(int)((pos / pulsesPerTurn) + (pos >= 0 ? 0.5 : -0.5));

	UART_Print("Rail is %ld pulses (%d turns) from home. Returning...\r\n",
	           (long)pos, -turnsBack);

	if (turnsBack == 0) {
		UART_Print("Already at home.\r\n");
		currentRailTurns = RAIL_HOME;
		return true;
	}

	bool ok = MoveLinearServo(turnsBack);
	if (ok)
		currentRailTurns = RAIL_HOME;   /* back at datum */
	return ok;
}

bool robotLinearGotoHardcodedHome(void)
{
	int32_t cur = 0;

	if (!railConfigured) {
		if (!ConfigRobotLinear()) return false;
		railConfigured = true;
	}
	if (!ServoSafeReadPosition(servo, ROBOT_LINEAR_ID, READ_CURRENT_POS, &cur)) {
		UART_Print("Rail read failed\r\n");
		return false;
	}

	int32_t delta = (int32_t)ROBOT_RAIL_HOME_PULSES - cur;   /* pulses to move */
	if (delta == 0) { UART_Print("Already at home\r\n"); return true; }

	uint16_t dir = (delta >= 0) ? SERVO_FORWARD : SERVO_REVERSE;
	UART_Print("Rail %ld -> home %d (delta %ld)\r\n",
	           (long)cur, ROBOT_RAIL_HOME_PULSES, (long)delta);

	ServoWrite32(servo, ROBOT_LINEAR_ID, SERVO_POSITION, delta);   /* incremental */
	osDelay(50);
	ServoWrite16(servo, ROBOT_LINEAR_ID, SERVO_MOTION_CMD, dir);
	osDelay(20);
	bool done = ServoWaitMotionComplete(servo, ROBOT_LINEAR_ID, READ_CURRENT_POS, 180000);
	ServoWrite16(servo, ROBOT_LINEAR_ID, SERVO_MOTION_CMD, SERVO_STOP);
	osDelay(20);

	UART_Print(done ? "Rail at home\r\n" : "Rail home move timeout\r\n");
	return done;
}


bool runPowderDispenserModule(void)   /* ARM ONLY -- rail moved by sequence() */
{
	UART_Print("\r\n===== POWDER DISPENSER MODULE : home -> P1..P4 -> home =====\r\n");

	if (connectRobot() != 0) {
		UART_Print("[POWDER] Robot not connected -- aborting.\r\n");
		return false;
	}
	if (robotInitConfig() != 0) {
		UART_Print("[POWDER] Robot NOT READY -- aborting before any motion.\r\n"
		           "   Clear faults + switch to AUTOMATIC on the pendant/WebApp, then retry.\r\n");
		return false;
	}

	for (int i = 0; i < POWDER_NUM_STEPS; i++) {
		UART_Print("\r\n[POWDER] ---- %s (step %d of %d) ----\r\n",
		           powderSeqName[i], i + 1, POWDER_NUM_STEPS);
		if (robotMoveJStr(powderPoses[powderSeq[i]]) != 0) {
			UART_Print("[POWDER] %s FAILED -- sequence ABORTED.\r\n", powderSeqName[i]);
			return false;
		}
		osDelay(MODULE_STEP_DELAY_MS);
	}

	UART_Print("\r\n[POWDER] Sequence COMPLETE.\r\n");
	return true;
}

/* Pump-on settle time before the STIR6 move begins. */
#define STIR_PUMP_PRE_MS      5000u

#define SIG_REACTION_DONE     0x0010     /* reaction task -> stir module         */

/* ---------------------------------------------------------------------------
 *  PARALLEL REACTION MOVE  (runs on the mixer/reaction servo during STIR8)
 *
 *  When the stir sequence reaches STIR8, this task is launched to run
 *  ReactionMove() on the MIXER servo (ID 0x01) IN PARALLEL with the remaining
 *  stir arm poses (STIR8 hold -> STIR9 -> HOME). ReactionMove() does a fixed
 *  number of pendulum cycles per call, so to keep it going for a full minute
 *  the task loops the call until REACTION_PARALLEL_DURATION_MS has elapsed.
 *
 *  Values requested: offset 5, swing 60 deg, 1 cycle, 500 ms delay. Speed was
 *  not specified, so REACTION_PARALLEL_SPEED is a safe default (a 60 deg swing
 *  at speed 4 nearly hits the 15 s motion timeout; 40 keeps well clear).
 *
 *  SHARED BUS: ReactionMove() talks to the servo over the same USART6 bus as
 *  the rail/carousel, so each ReactionMove() call is wrapped in servoBusLock/
 *  Unlock. The stir arm poses themselves use the FR5 TCP link (not this bus),
 *  so there is no contention during the parallel window. */
#define REACTION_PARALLEL_ID          MIXER_ID
#define REACTION_PARALLEL_OFFSET      5
#define REACTION_PARALLEL_DEGREE      90      /* swing, each side of centre     */
#define REACTION_PARALLEL_CYCLES      1
#define REACTION_PARALLEL_DELAY_MS    0      /* 0 = no delay between cycles */
#define REACTION_PARALLEL_SPEED       100      /* NOT specified -- safe default  */
#define REACTION_PARALLEL_DURATION_MS 60000u  /* run for 1 minute               */
#define REACTION_JOIN_TIMEOUT_MS      90000u  /* stir waits at most this long   */

#define REACTION_TASK_STACK_WORDS     2048   /* was 1024; +margin, see appMain */

/* After its cycles, the mixer servo parks at this ABSOLUTE encoder count.
 * 2168 pulses = the position reported by READ_CURRENT_POS (Modbus reply
 * 01 03 04 08 78 00 00 ... -> 0x0878 = 2168). */
#define REACTION_HOME_PULSES          2168

static StackType_t  xReactionTaskStack[REACTION_TASK_STACK_WORDS];
static StaticTask_t xReactionTaskTCB;
static osThreadId   reactionTaskHandle  = NULL;
static osThreadId   reactionJoinHandle  = NULL;   /* who to signal when finished */
static volatile bool reactionTaskRunning = false;

/* Set true by sequence() before it calls runReactionStirModule(), so the STIR8
 * parallel reaction only fires as part of the FULL run -- a standalone menu
 * call of runReactionStirModule() leaves it false and skips the reaction. */
static volatile bool reactionAtStir8Enabled = false;

/* startParallelReaction()/ReactionTask() are defined later (after
 * servoBusLock, which ReactionTask uses); forward-declare here so the
 * stir module below can launch the reaction. */
static void startParallelReaction(osThreadId joinTo) __attribute__((unused));

bool runReactionStirModule(void)   /* ARM ONLY -- rail moved by sequence() */
{
	UART_Print("\r\n===== REACTION STIR MODULE : home -> STIR1..4 -> home =====\r\n");

	UART_Print("[STIR] Gripper ON + peristaltic pump OFF (initial state).\r\n");
	HAL_GPIO_WritePin(Robot_gripper_GPIO_Port, Robot_gripper_Pin, GPIO_PIN_SET);   /* <-- first action */
	/* Pump starts in a defined RESET (OFF) state; it is turned ON (SET) only
	 * after the STIR5 move at i == 5 below. */
	HAL_GPIO_WritePin(Liquid_peristalic_pump_GPIO_Port,
	                  Liquid_peristalic_pump_Pin, GPIO_PIN_RESET);

	if (connectRobot() != 0) {
		UART_Print("[STIR] Robot not connected -- aborting.\r\n");
		return false;
	}
	if (robotInitConfig() != 0) {
		UART_Print("[STIR] Robot NOT READY -- aborting before any motion.\r\n"
		           "   Clear faults + switch to AUTOMATIC on the pendant/WebApp, then retry.\r\n");
		return false;
	}

	for (int i = 0; i < STIR_NUM_STEPS; i++) {

		/* BEFORE moving to the 4th position (STIR4, loop index 4):
		 *   - release the gripper (GPIO_RESET)
		 *   - turn the peristaltic pump ON (GPIO_SET)
		 *   - wait 3 s with the pump running, THEN do the move.
		 * The gripper stays RESET from here through the rest of the module. */
		if (i == 5) {
			UART_Print("[STIR] Before STIR4: gripper OFF + pump ON, wait 3 s...\r\n");
			//HAL_GPIO_WritePin(Robot_gripper_GPIO_Port,
			                  //Robot_gripper_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Liquid_peristalic_pump_GPIO_Port,
			                  Liquid_peristalic_pump_Pin, GPIO_PIN_RESET);
			osDelay(1000);
		}

		UART_Print("\r\n[STIR] ---- %s (step %d of %d) ----\r\n",
		           stirSeqName[i], i + 1, STIR_NUM_STEPS);
		if (robotMoveJStr(stirPoses[stirSeq[i]]) != 0) {
			UART_Print("[STIR] %s FAILED -- sequence ABORTED.\r\n", stirSeqName[i]);
			return false;
		}
		osDelay(MODULE_STEP_DELAY_MS);

		/* AFTER arriving at the 4th position (STIR4): pump OFF (GPIO_RESET) and
		 * dwell 5 s, then the loop continues back STIR3 -> STIR2 -> STIR1 ->
		 * HOME. Gripper stays RESET throughout. */
		if (i == 5) {
			UART_Print("[STIR] At STIR4: pump OFF, dwell 5 s...\r\n");
			HAL_GPIO_WritePin(Liquid_peristalic_pump_GPIO_Port,
			                  Liquid_peristalic_pump_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(Robot_gripper_GPIO_Port,
						                  Robot_gripper_Pin, GPIO_PIN_RESET);
			osDelay(1000);
		}

		/* AT STIR8 (loop index 8): kick off ReactionMove on the mixer servo in a
		 * PARALLEL task. It runs for ~1 min while this loop continues with STIR9
		 * and HOME. Only fires as part of the full sequence (see the flag). */
		if (i == 8 && reactionAtStir8Enabled) {
			UART_Print("[STIR] STIR8 reached -- launching parallel reaction.\r\n");
			(void) osSignalWait(SIG_REACTION_DONE, 0);   /* drain any stale bit */
			startParallelReaction(osThreadGetId());
		}
	}

	/* If the parallel reaction was started at STIR8, wait for its full minute to
	 * finish before returning, so the caller does not start using the servo bus
	 * (rail moves) while ReactionMove is still driving the mixer servo. */
	if (reactionAtStir8Enabled && reactionTaskRunning) {
		UART_Print("[STIR] Waiting for the parallel reaction (1 min) to finish "
		           "before ReStir...\r\n");
		(void) osSignalWait(SIG_REACTION_DONE, REACTION_JOIN_TIMEOUT_MS);
	}

	UART_Print("\r\n[STIR] Sequence COMPLETE.\r\n");
	return true;
}

/* ---------------------------------------------------------------------------
 *  REPOSTURE STIR  (POSE_RESTIR coordinates)
 *
 *  Same shape as runReactionStirModule()'s single pass, but:
 *    - uses stirRePoses[] (the POSE_RESTIR coordinates)
 *    - the gripper and pump actions are the REVERSE of the 1st stir cycle:
 *        1st cycle   start: gripper SET   | i==5: pump SET   | i==6: pump RESET + gripper RESET
 *        RESTIR      start: gripper RESET | i==5: pump RESET | i==6: pump SET   + gripper SET
 *    - no parallel reaction is launched here. */
bool runReactionReStirModule(void)   /* ARM ONLY -- rail moved by sequence() */
{
	UART_Print("\r\n===== REPOSTURE STIR MODULE (RESTIR) =====\r\n");

	/* RESTIR holds the OPPOSITE pin states to the stir module:
	 * robot gripper SET (ON) and peristaltic pump RESET (OFF) throughout. */
	UART_Print("[RESTIR] Gripper SET + pump RESET (initial state).\r\n");
	HAL_GPIO_WritePin(Robot_gripper_GPIO_Port, Robot_gripper_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Liquid_peristalic_pump_GPIO_Port,
	                  Liquid_peristalic_pump_Pin, GPIO_PIN_SET);

	if (connectRobot() != 0) {
		UART_Print("[RESTIR] Robot not connected -- aborting.\r\n");
		return false;
	}
	if (robotInitConfig() != 0) {
		UART_Print("[RESTIR] Robot NOT READY -- aborting before any motion.\r\n");
		return false;
	}

	/* Same motion as the stir module (HOME -> ReStir1..5 -> ReStir4..1 -> HOME),
	 * using stirRePoses[] whose coordinates are now identical to stirPoses[]. */
	for (int i = 0; i < STIR_NUM_STEPS; i++) {

		UART_Print("\r\n[RESTIR] ---- %s (step %d of %d) ----\r\n",
		           stirSeqName[i], i + 1, STIR_NUM_STEPS);
		if (robotMoveJStr(stirRePoses[stirSeq[i]]) != 0) {
			UART_Print("[RESTIR] %s FAILED -- sequence ABORTED.\r\n", stirSeqName[i]);
			return false;
		}
		osDelay(MODULE_STEP_DELAY_MS);

		/* AFTER the ReStir5 move (loop index 5 -- stirSeq[5] == 5): gripper SET +
		 * pump RESET (the reverse of the stir module, which does pump SET +
		 * gripper RESET at STIR5), then HOLD here. */
		if (i == 5) {
			UART_Print("[RESTIR] At ReStir5: gripper SET + pump RESET, holding %u ms...\r\n",
			           (unsigned) STIR5_PUMP_DWELL_MS);
			HAL_GPIO_WritePin(Robot_gripper_GPIO_Port,
			                  Robot_gripper_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(Liquid_peristalic_pump_GPIO_Port,
			                  Liquid_peristalic_pump_Pin, GPIO_PIN_RESET);
			osDelay(STIR5_PUMP_DWELL_MS);
		}
	}

	UART_Print("\r\n[RESTIR] Sequence COMPLETE.\r\n");
	return true;
}

/* ---------------------------------------------------------------------------
 *  PARALLEL DOSE + DRAIN  (FreeRTOS task + rendezvous with the robot)
 *
 *    dispense task                    sequence task (caller)
 *    -------------                    ----------------------
 *    stepper -> limit        ||       rail -> powder station
 *                                     runPowderDispenserModule()
 *    wait POWDER_DONE   <-----------  signal POWDER_DONE
 *    carousel index          ||       rail -> HOME  (parallel with the dose)
 *    dose 10 g               ||
 *    signal WEIGHT_READY ---------->  wait WEIGHT_READY
 *                                     runLiquidDispenserModule() (robot->liquid)
 *    wait ROBOT_READY   <-----------  signal ROBOT_READY
 *    OPEN drain valve
 *    poll until load cell ~ 0
 *    CLOSE valve
 *    stepper -> limit (park)
 *    signal DRAIN_DONE  ---------->   wait DRAIN_DONE
 *                                     rail -> stir + runReactionStirModule()
 *
 *  Three deliberate overlaps:
 *    - stepper homing runs while the rail travels to the powder station
 *    - the dose runs while the rail comes back home
 *    - the valve opens only once BOTH the weight is made up AND the robot has
 *      reached the liquid-dispensing pose holding the vessel
 *
 *  SHARED BUS WARNING: the carousel (MicroID 0x02) and the rail
 *  (ROBOT_LINEAR_ID 0x04) sit on the SAME RS485 bus (USART6). Every servo-bus
 *  call is wrapped in servoBusLock/Unlock so the two tasks cannot transmit
 *  Modbus frames at the same time. The load cell is on USART2 -- no lock needed.
 * ------------------------------------------------------------------------- */

#define SEQ_DOSE_RAW            100      /* 100 tenths = 10.0 g                 */
#define SEQ_DOSE_DIR              0      /* stepper direction for dispensing    */
#define SEQ_DOSE_SPEED           500      /* stepper speed                       */

#define SEQ_DISPENSER_DEG       -36      /* carousel index to the pour position  */
#define WASH_DISPENSER_DEG      -18
/* ---- 8-syringe carousel ----
 * Eight syringes sit -36 deg apart on the dispenser (amino-acid) carousel,
 * alternating 10 ml / 60 ml. Slot 1 is the rest position. The sequence loops
 * NUM_SYRINGES times; each pass the carousel advances -36 deg to the next
 * syringe (a NORMAL DispenserMove -- the plunge below is a STEPPER action,
 * nothing to do with the carousel move).
 *
 * ODD slots (1,3,5,7) get a fast initial plunge before the normal ramped,
 * weight-based dose: SYRINGE_ODD_PLUNGE_STEPS at SYRINGE_ODD_PLUNGE_SPEED,
 * then dosing continues at the normal speed already in the code. EVEN slots
 * skip the plunge. */
#define NUM_SYRINGES               8
#define AMINO_ACID_SYRINGES        3
#define SYRINGE_ODD_PLUNGE_STEPS   40000
/* MAX plunge speed. stepperData() derives its per-step delay as
 * MotorSpeed = 101 - speed, so speed 100 -> 1 = the fastest the driver can
 * go. This was 5000 before: if stepper.c does NOT clamp `speed` to 100
 * first, 101 - 5000 underflows into a huge unsigned delay and the "fast"
 * plunge crawls -- which is exactly the symptom reported on odd plunges.
 * 100 is correct whether or not that clamp exists. Do NOT raise it again;
 * to plunge faster, change the driver's step timing, not this number. */
#define SYRINGE_ODD_PLUNGE_SPEED   8000

/* Per-syringe target weight, in TENTHS OF A GRAM (the load cell's unit:
 * raw 50 = 5.0 g). Index 0 = syringe 1. Requested amounts:
 *   S1=5 S2=23 S3=16 S4=19 S5=21 S6=16 S7=23 S8=16  (grams)
 * This replaces the old fixed SEQ_DOSE_RAW (10 g) for every syringe. */
static const uint16_t syringeTargetRaw[NUM_SYRINGES] = {
	 50,   /* syringe 1 -> 5.0 g  */
	230,   /* syringe 2 -> 23.0 g */
	160,   /* syringe 3 -> 16.0 g */
	190,   /* syringe 4 -> 19.0 g */
	210,   /* syringe 5 -> 21.0 g */
	160,   /* syringe 6 -> 16.0 g */
	230,   /* syringe 7 -> 23.0 g */
	160,   /* syringe 8 -> 16.0 g */
};
#define SEQ_DISPENSER_SPEED      4
#define STEPPER_LIMIT_WAIT_MS   300000u  /* max wait for stepper->limit before -36 */

#define SEQ_EMPTY_RAW             5      /* <= 0.5 g counts as "drained"        */
#define SEQ_EMPTY_STABLE          3      /* consecutive empty reads required    */
#define SEQ_DRAIN_TIMEOUT_MS 100000u     /* 3 min ceiling so it cannot hang     */

/* ---- How the valve-open period is ended ----
 * 1 = poll the load cell and close the valve once it reads ~zero (the
 *     logic you commented out -- it was CORRECT, kept here verbatim).
 * 0 = hold the valve open for a fixed time instead.
 *
 * Commenting the loop out in-place had a side effect worth knowing about:
 * the valve was opened and then closed on the very next line with no wait
 * at all, so no liquid could flow -- and `drained` stayed false, which made
 * dispenseOk false and aborted the whole sequence with "Dose/collect
 * reported FAILURE". The #if keeps both behaviours valid. */
#define SEQ_DRAIN_BY_WEIGHT       0      /* 1 = wait for load cell ~ zero   */
#define SEQ_DRAIN_FIXED_MS   10000u      /* used when the above is 0        */

/* Rendezvous signals (CMSIS-RTOS v1 osSignal bits). */
#define SIG_WEIGHT_READY      0x0001     /* dispense -> sequence */
#define SIG_ROBOT_READY       0x0002     /* sequence -> dispense */
#define SIG_DRAIN_DONE        0x0004     /* dispense -> sequence */
#define SIG_POWDER_DONE       0x0008     /* sequence -> dispense */

#define SIG_DOSE_TIMEOUT_MS  10000000u     /* 5 min ceiling on each rendezvous    */
/* ---- shared RS485 servo-bus lock ----
 *
 * Deliberately NOT an osMutex -- but NOT because mutexes are disabled.
 * configUSE_MUTEXES IS 1 in FreeRTOSConfig.h. osMutexCreate() still returned
 * NULL and produced "Could not create servo bus mutex" because
 * configTOTAL_HEAP_SIZE is only 15360 bytes and LWIP has already consumed most
 * of it -- xSemaphoreCreateMutex() calls pvPortMalloc(), which fails on an
 * exhausted heap.
 *
 * This flag-based lock allocates NOTHING: the test-and-set is made atomic with
 * a critical section, and a waiting task yields with osDelay() rather than
 * spinning. Two tasks at equal priority means no priority-inversion concern. */
static volatile uint8_t servoBusBusy = 0;

static void servoBusLock(void)
{
	for (;;) {
		uint8_t taken = 0;

		taskENTER_CRITICAL();
		if (servoBusBusy == 0u) {
			servoBusBusy = 1u;
			taken = 1u;
		}
		taskEXIT_CRITICAL();

		if (taken) {
			return;
		}

		osDelay(10);            /* bus is busy -- yield and retry */
	}
}

static void servoBusUnlock(void)
{
	taskENTER_CRITICAL();
	servoBusBusy = 0u;
	taskEXIT_CRITICAL();
}

static osThreadId dispenseTaskHandle = NULL;
static osThreadId sequenceTaskHandle = NULL;
static volatile bool dispenseOk = false;

/* True while DispenseTask owns xDispenseTaskTCB. Checked before force-
 * terminating it on an aborted sequence, and cleared by the task itself
 * just before it self-terminates, so a stale handle is never terminated. */
static volatile bool dispenseTaskRunning = false;

/* ---- One-shot parallel dose+pour task for amino_acid_addition_sequence().
 * Reuses sequence()'s SIG_WEIGHT_READY/SIG_ROBOT_READY/SIG_DRAIN_DONE and
 * dispenseOk/sequenceTaskHandle -- safe because sequence() and
 * amino_acid_addition_sequence() are never in flight at the same time. Unlike
 * DispenseTask this does ONE dose per launch (no syringe loop, no powder
 * wait), since this function's steps are named liquid additions, not
 * indexed syringes. */
#define AMINO_DOSE_TASK_STACK_WORDS   2048   /* was 1024; +margin, see appMain */

static StackType_t  xAminoDoseTaskStack[AMINO_DOSE_TASK_STACK_WORDS];
static StaticTask_t xAminoDoseTaskTCB;
static osThreadId     aminoDoseTaskHandle  = NULL;
static volatile bool  aminoDoseTaskRunning = false;
static volatile uint16_t aminoDoseTargetRaw = 0;   /* set by caller before launch */

static void AminoDoseTask(void const *argument);


/* Move the mixer servo to an ABSOLUTE encoder position. The drive is in
 * INCREMENTAL mode, so read the current count and command the delta (mirrors
 * the move/trigger/wait pattern of MoveLinearServo). Caller holds the bus. */
static void reactionServoGotoPulses(int32_t target)
{
	int32_t cur = 0;

	if (!ServoSafeReadPosition(servo, REACTION_PARALLEL_ID, READ_CURRENT_POS, &cur)) {
		UART_Print("[REACT] Could not read mixer position -- skip park.\r\n");
		return;
	}

	int32_t  delta     = target - cur;
	uint16_t direction = (delta >= 0) ? SERVO_FORWARD : SERVO_REVERSE;

	UART_Print("[REACT] Mixer -> %ld pulses (cur %ld, delta %ld)...\r\n",
	           (long) target, (long) cur, (long) delta);

	if (delta == 0) {
		UART_Print("[REACT] Mixer already parked.\r\n");
		return;
	}

	ServoWrite32(servo, REACTION_PARALLEL_ID, SERVO_POSITION, delta);
	osDelay(50);
	ServoWrite16(servo, REACTION_PARALLEL_ID, SERVO_MOTION_CMD, direction);
	osDelay(20);
	(void) ServoWaitMotionComplete(servo, REACTION_PARALLEL_ID, READ_CURRENT_POS, 30000);
	ServoWrite16(servo, REACTION_PARALLEL_ID, SERVO_MOTION_CMD, SERVO_STOP);
	osDelay(20);

	UART_Print("[REACT] Mixer parked at ~%ld pulses.\r\n", (long) target);
}

static void ReactionTask(void const *argument)
{
	uint32_t start = HAL_GetTick();

	(void) argument;

	reactionTaskRunning = true;

	UART_Print("\r\n[REACT] Parallel ReactionMove START "
	           "(offset %d, swing %d, cycles %d, delay %d ms) for %lu ms...\r\n",
	           REACTION_PARALLEL_OFFSET, REACTION_PARALLEL_DEGREE,
	           REACTION_PARALLEL_CYCLES, REACTION_PARALLEL_DELAY_MS,
	           (unsigned long) REACTION_PARALLEL_DURATION_MS);

	while ((HAL_GetTick() - start) < REACTION_PARALLEL_DURATION_MS) {
		servoBusLock();
		ReactionMove(servo, REACTION_PARALLEL_ID,
		             REACTION_PARALLEL_OFFSET, REACTION_PARALLEL_DEGREE,
		             REACTION_PARALLEL_SPEED, REACTION_PARALLEL_CYCLES,
		             REACTION_PARALLEL_DELAY_MS);
		servoBusUnlock();
	}

	UART_Print("[REACT] Parallel ReactionMove DONE (1 min elapsed).\r\n");

	/* Cycles finished -- park the mixer servo at its hardcoded position. */
	servoBusLock();
	reactionServoGotoPulses(REACTION_HOME_PULSES);
	servoBusUnlock();

	reactionTaskRunning = false;

	if (reactionJoinHandle != NULL) {
		osSignalSet(reactionJoinHandle, SIG_REACTION_DONE);
	}

	osThreadTerminate(osThreadGetId());
}

/* Launch the parallel reaction, remembering which task to notify when it ends. */
static void startParallelReaction(osThreadId joinTo)
{
	if (reactionTaskRunning) {
		UART_Print("[REACT] Already running -- not starting again.\r\n");
		return;
	}

	reactionJoinHandle = joinTo;

	reactionTaskHandle = (osThreadId) xTaskCreateStatic(
			(TaskFunction_t) ReactionTask,
			"reactionTask",
			REACTION_TASK_STACK_WORDS,
			NULL,
			(UBaseType_t) (tskIDLE_PRIORITY + 3),
			xReactionTaskStack,
			&xReactionTaskTCB);

	if (reactionTaskHandle == NULL) {
		UART_Print("[REACT] Could not start reaction task.\r\n");
	}
}

static void DispenseTask(void const *argument);

/* STATIC task allocation -- deliberately not osThreadCreate().
 * configTOTAL_HEAP_SIZE is only 15360 bytes and LWIP has taken most of it, so a
 * dynamic 4 KB task stack would fail exactly like the mutex did.
 * configSUPPORT_STATIC_ALLOCATION is 1, so stack and TCB live in .bss instead
 * -- the same pattern appMainTask uses in freertos.c. Priority
 * tskIDLE_PRIORITY + 3 is what CMSIS-RTOS v1 computes for osPriorityNormal. */
#define DISPENSE_TASK_STACK_WORDS   2048   /* was 1024; +margin, see appMain */

static StackType_t  xDispenseTaskStack[DISPENSE_TASK_STACK_WORDS];
static StaticTask_t xDispenseTaskTCB;

/* ---------------------------------------------------------------------------
 *  DISPENSE TASK
 * ------------------------------------------------------------------------- */
static void DispenseTask(void const *argument)
{
	(void) argument;

	dispenseOk = false;

	/* One pass per syringe. This single task lives for the whole run (it is NOT
	 * recreated per syringe), so there is no static-TCB reuse hazard. It stays
	 * in lockstep with sequence() through the same four signals, re-armed every
	 * pass: POWDER_DONE -> WEIGHT_READY -> ROBOT_READY -> DRAIN_DONE. */
	for (int slot = 1; slot <= NUM_SYRINGES; slot++) {

		uint16_t weight = 0;
		uint32_t start;
		int      emptyHits = 0;
		bool     drained = false;
		osEvent  evt;

		UART_Print("\r\n[DOSE] ===== Syringe %d of %d (%s slot) =====\r\n",
		           slot, NUM_SYRINGES, (slot & 1) ? "ODD" : "EVEN");

		/* ---- 1. Ensure the stepper is at its upper limit before plunging.
		 * If it is ALREADY at the limit, skip homing and go straight on to the
		 * plunge; otherwise home to the limit first. */
		if (drainStepperAtLimit()) {
			UART_Print("[DOSE] Stepper already at limit -- plunging directly.\r\n");
		} else {
			UART_Print("[DOSE] Stepper not at limit -- homing first...\r\n");
			drainStepperLimit();
		}
		osDelay(300);
		UART_Print("[DOSE] Stepper at limit. Waiting for the powder stage...\r\n");

		/* ---- 2. Hold until the robot's powder stage for this syringe is done. */
		evt = osSignalWait(SIG_POWDER_DONE, SIG_DOSE_TIMEOUT_MS);

		if (evt.status == osEventTimeout) {
			UART_Print("[DOSE] Powder stage never signalled -- aborting.\r\n");
			dispenseOk = false;
			osSignalSet(sequenceTaskHandle, SIG_WEIGHT_READY);
			osSignalSet(sequenceTaskHandle, SIG_DRAIN_DONE);
			dispenseTaskRunning = false;
			osThreadTerminate(osThreadGetId());
			return;
		}

		/* ---- 2a. Carousel -36 to index this syringe for the dose.
		 * SAFETY INTERLOCK: the plunger (stepper) MUST be at its upper limit,
		 * clear of the syringes, before the carousel rotates. Check it; if it
		 * is not at the limit, drive it there first; THEN index -36. This runs
		 * in the dispense thread, in parallel with the robot moving to the pour
		 * pose -- the powder stage above never waited for the stepper. */
		if (!drainStepperAtLimit()) {
			UART_Print("[DOSE] Stepper NOT at limit before -36 -- homing first...\r\n");
			drainStepperLimit();
		}
		UART_Print("[DOSE] Stepper at limit -- indexing carousel %d deg (syringe %d)...\r\n",
		           SEQ_DISPENSER_DEG, slot);
		servoBusLock();
		DispenserMove(servo, MicroID, SEQ_DISPENSER_DEG, SEQ_DISPENSER_SPEED);
		servoBusUnlock();
		osDelay(300);

		/* ---- 2b. ODD slot only: fast initial plunge, THEN the normal dose.
		 * stepperData() moves SYRINGE_ODD_PLUNGE_STEPS in the dose direction at
		 * SYRINGE_ODD_PLUNGE_SPEED; the weight-based ramp below then runs at the
		 * normal speed. (Note: LoadCellStepperDispenseRamped() takes its own
		 * baseline AFTER this plunge, so the plunge is treated as positioning /
		 * priming, not counted toward the target weight.) */
		if (slot & 1) {
			UART_Print("[DOSE] Odd slot -> plunge %d steps @ %d, then normal dose.\r\n",
			           SYRINGE_ODD_PLUNGE_STEPS, SYRINGE_ODD_PLUNGE_SPEED);
			stepperData(SYRINGE_ODD_PLUNGE_STEPS, SEQ_DOSE_DIR, SYRINGE_ODD_PLUNGE_SPEED);
			osDelay(200);
		}

		/* ---- 3. Dose by load-cell feedback (normal speed, as in the base code).
		 * Target is this syringe's own amount, not a fixed 10 g. */
		uint16_t targetRaw = syringeTargetRaw[slot - 1];

		UART_Print("[DOSE] Dosing syringe %d -> %u.%u g...\r\n",
		           slot, (unsigned)(targetRaw / 10), (unsigned)(targetRaw % 10));

		if (!LoadCellStepperDispenseRamped(loadcell, targetRaw,
		                                   SEQ_DOSE_DIR, SEQ_DOSE_SPEED)) {
			UART_Print("[DOSE] Dose FAILED.\r\n");
			dispenseOk = false;
			osSignalSet(sequenceTaskHandle, SIG_WEIGHT_READY);
			osSignalSet(sequenceTaskHandle, SIG_DRAIN_DONE);
			dispenseTaskRunning = false;
			osThreadTerminate(osThreadGetId());
			return;
		}

		/* ---- 4. Weight made up -- signal, but DO NOT open the valve yet. */
		UART_Print("[DOSE] Target weight collected. Feedback pin = %s\r\n",
		           (HAL_GPIO_ReadPin(loadcell->feedbackPort,
		                             loadcell->feedbackPin) != GPIO_PIN_RESET)
		           ? "HIGH" : "LOW");
		UART_Print("[DOSE] >>> WEIGHT READY (syringe %d).\r\n", slot);

		osSignalSet(sequenceTaskHandle, SIG_WEIGHT_READY);

		/* ---- 5. Hold until the robot is at the liquid-dispensing pose. */
		evt = osSignalWait(SIG_ROBOT_READY, SIG_DOSE_TIMEOUT_MS);

		if (evt.status == osEventTimeout) {
			UART_Print("[DOSE] Robot never signalled ready -- valve NOT opened.\r\n");
			dispenseOk = false;
			osSignalSet(sequenceTaskHandle, SIG_DRAIN_DONE);
			dispenseTaskRunning = false;
			osThreadTerminate(osThreadGetId());
			return;
		}

		/* ---- 6. Robot in position: OPEN the drain valve. */
		UART_Print("[DOSE] Robot at liquid station -- OPENING drain valve.\r\n");
		HAL_GPIO_WritePin(Drain_valve_pin_Pin_GPIO_Port,
		                  Drain_valve_pin_Pin_Pin, GPIO_PIN_SET);

		/* ---- 7. Hold the valve open until the liquid has been collected. */
#if SEQ_DRAIN_BY_WEIGHT
		start = HAL_GetTick();

		while ((HAL_GetTick() - start) < SEQ_DRAIN_TIMEOUT_MS) {
			osDelay(1000);

			if (!LoadCellReadWeight(loadcell, &weight)) {
				continue;
			}

			UART_Print("[DOSE] Collecting... weight = %u.%u g\r\n",
			           (unsigned)(weight / 10), (unsigned)(weight % 10));

			if (weight <= SEQ_EMPTY_RAW) {
				if (++emptyHits >= SEQ_EMPTY_STABLE) {
					drained = true;
					break;
				}
			} else {
				emptyHits = 0;
			}
		}
#else
		(void) weight;
		(void) start;
		(void) emptyHits;

		UART_Print("[DOSE] Valve open for %u ms (weight polling disabled)...\r\n",
		           (unsigned) SEQ_DRAIN_FIXED_MS);
		osDelay(SEQ_DRAIN_FIXED_MS);
		drained = true;
#endif

		/* ---- 8. CLOSE the valve. */
		HAL_GPIO_WritePin(Drain_valve_pin_Pin_GPIO_Port,
		                  Drain_valve_pin_Pin_Pin, GPIO_PIN_RESET);
		osDelay(300);

		if (drained) {
			UART_Print("[DOSE] Liquid collected (syringe %d).\r\n", slot);
		} else {
			UART_Print("[DOSE] DRAIN TIMEOUT (syringe %d) -- valve closed.\r\n", slot);
		}

		/* ---- 9. Report result FIRST so the robot does NOT wait for the stepper
		 * to home. The park below then runs in parallel inside this task; the
		 * stepper is driven purely by GPIO pins (no servo-bus dependency), so it
		 * cannot clash with the robot's rail/servo moves. */
		dispenseOk = drained;
		osSignalSet(sequenceTaskHandle, SIG_DRAIN_DONE);

		/* ---- 10. Park the head at its limit, in PARALLEL with the robot. */
		UART_Print("[DOSE] Stepper -> upper limit (park, parallel)...\r\n");
		drainStepperLimit();
		osDelay(300);

		if (!drained) {
			/* A collection failure stops the whole run; sequence() will abort
			 * when it sees dispenseOk == false for this syringe. */
			break;
		}
	}

	dispenseTaskRunning = false;
	osThreadTerminate(osThreadGetId());
}

/* Abort helper: reports why, stops the dispense task if it is still blocked on
 * a rendezvous, and frees the servo bus. Without this an aborted run would
 * leave DispenseTask waiting (up to 5 min) still owning xDispenseTaskTCB, and
 * the next sequence() would call xTaskCreateStatic() on a TCB still in use. */
static bool sequenceAbort(const char *why)
{
	UART_Print("[SEQ] %s -- aborting.\r\n", why);

	if (dispenseTaskRunning && dispenseTaskHandle != NULL) {
		dispenseTaskRunning = false;
		osThreadTerminate(dispenseTaskHandle);
	}
	dispenseTaskHandle = NULL;

	/* Stop the parallel reaction too, in case the abort happened during the
	 * stir stage while it was mid-run. */
	if (reactionTaskRunning && reactionTaskHandle != NULL) {
		reactionTaskRunning = false;
		osThreadTerminate(reactionTaskHandle);
	}
	reactionTaskHandle = NULL;
	reactionAtStir8Enabled = false;

	/* Stop a still-running amino dose task too, in case the abort happened
	 * mid-dose during amino_acid_addition_sequence(). */
	if (aminoDoseTaskRunning && aminoDoseTaskHandle != NULL) {
		aminoDoseTaskRunning = false;
		osThreadTerminate(aminoDoseTaskHandle);
	}
	aminoDoseTaskHandle = NULL;

	servoBusUnlock();

	return false;
}

/* ---------------------------------------------------------------------------
 *  FULL SEQUENCE
 * ------------------------------------------------------------------------- */
bool sequence(void)
{
	UART_Print("\r\n########## FULL SEQUENCE START (%d syringes) ##########\r\n",
	           NUM_SYRINGES);

	/* Robot gripper ON for the whole sequence (powder + liquid dispensing).
	 * It is held SET from here until the stir module, which then manages it. */
	UART_Print("[SEQ] Robot gripper ON (held through powder + liquid).\r\n");
	HAL_GPIO_WritePin(Robot_gripper_GPIO_Port, Robot_gripper_Pin, GPIO_PIN_SET);

	/* Servo-bus lock: make sure it starts free (a prior aborted run may have
	 * left it held). */
	servoBusUnlock();

	sequenceTaskHandle = osThreadGetId();

	/* Drain stale signals from a previous run (osSignalClear is unimplemented
	 * in ST's CMSIS-RTOS v1, so a zero-timeout wait is used to clear them). */
	(void) osSignalWait(SIG_WEIGHT_READY, 0);
	(void) osSignalWait(SIG_DRAIN_DONE,   0);

	dispenseOk = false;

	/* One dispense task for the WHOLE run -- it loops over all syringes itself,
	 * so it is created once here and never recreated. */
	dispenseTaskHandle = (osThreadId) xTaskCreateStatic(
			(TaskFunction_t) DispenseTask,
			"dispenseTask",
			DISPENSE_TASK_STACK_WORDS,
			NULL,
			(UBaseType_t) (tskIDLE_PRIORITY + 3),
			xDispenseTaskStack,
			&xDispenseTaskTCB);

	if (dispenseTaskHandle == NULL) {
		UART_Print("[SEQ] Could not start dispense task -- aborting.\r\n");
		return false;
	}

	dispenseTaskRunning = true;
	UART_Print("[SEQ] Dispense task started (loops all %d syringes).\r\n", NUM_SYRINGES);

	/* ===================== PER-SYRINGE LOOP ===================== */
	for (int syringe = 1; syringe <= NUM_SYRINGES; syringe++) {

		osEvent evt;
		bool    railOk;
		bool    stirOk;

		UART_Print("\r\n##### SEQUENCE: syringe %d of %d #####\r\n",
		           syringe, NUM_SYRINGES);

		/* ---- 1. Rail out to the powder station (stepper homes in parallel). */
		UART_Print("[SEQ] Rail -> powder station (%d turns)...\r\n", SEQ_POWDER_TURNS);
		servoBusLock();
		railOk = MoveLinearServo(SEQ_POWDER_TURNS);
		servoBusUnlock();
		if (!railOk) {
			return sequenceAbort("Rail move to powder FAILED");
		}

		/* Carousel -36 is NO LONGER done here. It is done in the DispenseTask
		 * after POWDER_DONE (during liquid dispensing), gated on the stepper
		 * being at its upper limit -- so the powder stage never waits for it. */

		/* ---- 3. Powder dispensing arm poses (HOME -> P1..P4 -> HOME). */
		if (!runPowderDispenserModule()) {
			return sequenceAbort("Powder dispense FAILED");
		}

		/* Rail STAYS at the +30 (powder) station -- it does NOT go home here.
		 * Liquid is reached by the arm from +30; the stir move below is +70
		 * incremental from +30 (=> +100 total), then home after the stir. */

		/* ---- 5. Powder stage done -- release the dose for this syringe. */
		UART_Print("[SEQ] >>> POWDER STAGE DONE (syringe %d) -- starting the dose.\r\n",
		           syringe);
		osSignalSet(dispenseTaskHandle, SIG_POWDER_DONE);

		/* ---- 6. Move the robot to the pour pose IN PARALLEL with the dose.
		 * The stepper (home-to-limit + plunge + weigh) runs in the DispenseTask
		 * thread; there is no reason for the arm to sit idle at home waiting for
		 * it. So the arm travels to the pour pose now, while the stepper doses.
		 * The valve stays CLOSED until step 8, so nothing pours early. */
		UART_Print("[SEQ] Moving robot to the pour pose (dose runs in parallel)...\r\n");
		if (!runLiquidDispenserApproach()) {
			return sequenceAbort("Liquid approach FAILED");
		}

		/* ---- 7. Arm is at the pour pose. NOW make sure the dose has finished
		 * before the valve opens (it may already be done). */
		UART_Print("[SEQ] At pour pose -- waiting for WEIGHT READY...\r\n");
		evt = osSignalWait(SIG_WEIGHT_READY, SIG_DOSE_TIMEOUT_MS);
		if (evt.status == osEventTimeout) {
			return sequenceAbort("Timed out waiting for the dose");
		}

		/* ---- 8. Weight collected AND arm in position -> release the valve. */
		UART_Print("[SEQ] >>> ROBOT READY (at pour pose, weight collected).\r\n");
		osSignalSet(dispenseTaskHandle, SIG_ROBOT_READY);

		/* ---- 9. Hold while the liquid is collected and the valve closes. */
		evt = osSignalWait(SIG_DRAIN_DONE, SIG_DOSE_TIMEOUT_MS);
		if (evt.status == osEventTimeout) {
			return sequenceAbort("Timed out waiting for liquid collection");
		}
		if (!dispenseOk) {
			return sequenceAbort("Dose/collect reported FAILURE");
		}

		/* ---- 10. Liquid is in the vessel -- retract the arm. */
		UART_Print("[SEQ] Liquid collected -- retracting the arm.\r\n");
		if (!runLiquidDispenserRetract()) {
			return sequenceAbort("Liquid retract FAILED");
		}

		/* ---- 11. Rail out to the stir station, then stir (reaction at STIR8). */
		UART_Print("[SEQ] Rail -> stir station (%d turns)...\r\n", SEQ_STIR_TURNS);
		servoBusLock();
		railOk = MoveLinearServo(SEQ_STIR_TURNS);
		servoBusUnlock();
		if (!railOk) {
			return sequenceAbort("Rail move to stir FAILED");
		}

		reactionAtStir8Enabled = true;
		stirOk = runReactionStirModule();
		reactionAtStir8Enabled = false;
		if (!stirOk) {
			return sequenceAbort("Stir FAILED");
		}

		/* Reposture pass with the RESTIR coordinates (mirrored gripper/pump). */
		if (!runReactionReStirModule()) {
			return sequenceAbort("ReStir FAILED");
		}

		/* ---- 12. Rail back home before the next syringe. */
		UART_Print("[SEQ] Returning rail to home...\r\n");
		servoBusLock();
		robotLinearGotoHardcodedHome();
		servoBusUnlock();

		/* Give the idle task a moment to reclaim the reaction task's TCB before
		 * the next syringe's STIR8 recreates it. */
		osDelay(100);
	}
	/* =================== END PER-SYRINGE LOOP =================== */

	dispenseTaskRunning = false;
	dispenseTaskHandle = NULL;

	UART_Print("\r\n########## FULL SEQUENCE COMPLETE (%d syringes) ##########\r\n",
	           NUM_SYRINGES);
	return true;
}

/* ---------------------------------------------------------------------------
 *  "stir_drain" -- same physical drain valve as the rest of this file
 *  (Drain_valve_pin_Pin_*), aliased under this name for this sequence, as
 *  requested. Change these two lines if a SEPARATE physical valve is meant. */
#define stir_drain_GPIO_Port   Drain_valve_pin_Pin_GPIO_Port
#define stir_drain_Pin         Drain_valve_pin_Pin_Pin

#define Therinol_TARGET   50u   /* 5.0 g (tenths-of-gram raw unit) */
#define Cysteine_TARGET   230u
#define Theronine_TARGET  160u
#define Lysine_TARGET     190u
#define Dtreptran_TARGET  210u
#define Phenaline_TARGET  160u
#define Cysteine_TARGET   230u
#define Dphenamine_TARGET 160u

#define WASH_LIQUID_TARGET_RAW        10u   /* 1.0 g (tenths-of-gram raw unit) */

/* ---------------------------------------------------------------------------
 *  AMINO SEQUENCE -- SHARED RAIL / CAROUSEL-SLOT / PLUNGE HELPERS
 *
 *  Three defects lived in this sequence and all three are fixed here.
 *
 *  (1) ODD/EVEN PLUNGE was keyed off a "how many doses have I done" counter
 *      (aminoPlungeCount), NOT off the carousel slot. DispenseTask keys it
 *      off `slot` -- the physical syringe under the plunger -- because odd
 *      slots hold the 60 ml syringes that need the long fast plunge to take
 *      up dead volume and even slots hold the 10 ml ones that do not. A call
 *      counter drifts out of phase with the carousel the instant a -36 index
 *      happens without a dose (stage 5 below does exactly that) or the
 *      instant wash() jumps the carousel to an absolute -180. The tracker
 *      below is therefore the SLOT itself, maintained by the only two
 *      functions that move the carousel -- an exact mirror of DispenseTask.
 *
 *  (2) wash() called LoadCellStepperDispenseRamped() DIRECTLY, so its two
 *      doses per call got no limit-home, no odd/even fast plunge and no park
 *      afterwards: they crawled from wherever the previous dose left the
 *      plunger. Eight of the fifteen doses in a full run are wash doses,
 *      which is why the stepper looked slow nearly everywhere. Every dose
 *      now routes through aminoPlungeAndDose() / aminoDoseAndPour().
 *
 *  (3) RAIL: the sequence moved the rail +SEQ_POWDER_TURNS once at the top
 *      and then never again, so every runReactionStirModule() ran with the
 *      rail still parked in the liquid lane -- the arm went straight to the
 *      stir poses without the rail ever reaching the stir station.
 *      sequence() does rail +30 -> dose -> rail +70 (=+100) -> stir -> home
 *      on every syringe; aminoRailGoto() reproduces that, tracking the rail
 *      in ABSOLUTE turns from the datum so the relative MoveLinearServo()
 *      deltas stay correct whatever order the stages run in.
 * ------------------------------------------------------------------------- */

/* Rail stops in turns from the rail datum -- the same two lanes sequence()
 * uses, expressed absolutely instead of as chained relative moves. */
#define AMINO_RAIL_HOME     0
#define AMINO_RAIL_LIQUID   (SEQ_POWDER_TURNS)                    /* +30  */
#define AMINO_RAIL_STIR     (SEQ_POWDER_TURNS + SEQ_STIR_TURNS)   /* +100 */

/* ---- Where the RESTIR reposture pass sits relative to the drain pulse ----
 * 1 = stir -> RESTIR -> drain   (DEFAULT: the exact order sequence() uses)
 * 0 = stir -> drain -> RESTIR   (drain while the vessel is still seated on
 *                                the stir station, then pick it back up)
 *
 * This is a switch rather than a fixed order because the two differ only in
 * whether the vessel is HELD BY THE ARM or SEATED ON THE STATION while the
 * valve is open, and stirDrainPulse() currently aliases stir_drain to the
 * SAME pin as the liquid dispenser's Drain_valve_pin (see the #define above
 * it). Once that alias is resolved to the real reactor drain, pick the
 * matching order here and delete the other branch. */
#define AMINO_RESTIR_BEFORE_DRAIN   1

static int aminoRailTurns = AMINO_RAIL_HOME;   /* where the rail is right now */

/* Carousel slot currently under the plunger, 1..NUM_SYRINGES. Slot 1 is the
 * rest position -- identical meaning to DispenseTask's `slot`. */
static uint32_t aminoSlot = 1;

/* Carousel angle in DEGREES from the slot-1 datum, tracked in software.
 *
 * WHY SOFTWARE TRACKING: the carousel drive runs in INCREMENTAL mode. Every
 * DispenserMove() is a RELATIVE step and SERVO_POSITION is written as a delta
 * scaled by 4.5 (see DispenserMove() in servo.c). The encoder value returned
 * by READ_CURRENT_POS is therefore a running total in the scaled pulse domain,
 * NOT an absolute slot angle -- there is no home switch on this axis to give
 * one. The old aminoCarouselGotoAbsoluteDeg() tried to derive an absolute
 * target from it with `DegreeToPulse(deg) - cur`, which mixes the UNSCALED
 * and SCALED pulse domains and drifts further out with every move of the run.
 * Tracking the commanded angle here instead is exact, because every carousel
 * motion in this file goes through one of the two helpers below.
 *
 * LIMITATION: this tracker only knows about moves THIS file makes. If a
 * MachineMenu command drives MicroID by hand between runs, the datum is lost.
 * A proper fix is a home flag/switch on the carousel -- worth adding. */
static float aminoCarouselDeg = 0.0f;

static bool aminoDoseAndPour(uint16_t targetRaw, const char *why);   /* fwd */

/* THE only way this file turns the carousel.
 *
 * Everything goes through DispenserMove() because that is the one code path
 * known to work on this drive: it issues SERVO_STOP + 300 ms to unlatch the
 * previous motion, re-arms with ServoPositionConfig(), applies the x4.5 scale,
 * writes SERVO_POS_SPEED, picks the direction from the sign, waits for motion
 * complete and then stops. The hand-rolled register poking that used to live
 * in aminoCarouselGotoAbsoluteDeg() skipped the STOP/re-arm, skipped
 * ServoPositionConfig() and skipped the speed write -- and DispenserMove()'s
 * own comment says exactly what happens then: "if the servo is still latched
 * in its previous motion state, writing a new target position without
 * stopping/re-arming first can leave it ignoring the new value and repeating
 * the prior move." That hand-rolled block was the LAST carousel operation of
 * a run (wash #4), which is why the drive came out of run 1 latched and would
 * not turn at all on run 2. */
static void aminoCarouselMoveRel(int deg, const char *why)
{
	if (deg == 0) {
		UART_Print("[SEQ] Carousel already in place (%s).\r\n", why);
		return;
	}

	UART_Print("[SEQ] Carousel %+d deg (at %+d, %s)...\r\n",
	           deg, (int) aminoCarouselDeg, why);

	servoBusLock();
	DispenserMove(servo, MicroID, deg, SEQ_DISPENSER_SPEED);
	servoBusUnlock();
	osDelay(300);

	aminoCarouselDeg += (float) deg;
}

/* ---- RAIL ---------------------------------------------------------------- */
static bool aminoRailGoto(int targetTurns, const char *lane, const char *why)
{
	int delta = targetTurns - aminoRailTurns;

	if (delta == 0) {
		UART_Print("[SEQ] Rail already in the %s lane (%+d turns) -- %s.\r\n",
		           lane, aminoRailTurns, why);
		return true;
	}

	UART_Print("[SEQ] Rail -> %s lane: %+d turns (at %+d, target %+d) -- %s...\r\n",
	           lane, delta, aminoRailTurns, targetTurns, why);

	servoBusLock();
	bool ok = MoveLinearServo(delta);
	servoBusUnlock();

	if (!ok) {
		UART_Print("[SEQ] Rail move to the %s lane FAILED (%s).\r\n", lane, why);
		return false;
	}

	aminoRailTurns = targetTurns;
	osDelay(300);
	return true;
}

static bool aminoRailToLiquid(const char *why)
{
	return aminoRailGoto(AMINO_RAIL_LIQUID, "liquid/powder", why);
}

static bool aminoRailToStir(const char *why)
{
	return aminoRailGoto(AMINO_RAIL_STIR, "stir", why);
}

/* ---- CAROUSEL SLOT TRACKING --------------------------------------------- */
/* One -36 deg index = the next syringe is now under the plunger. */
static void aminoSlotAdvance(const char *why)
{
	aminoSlot = (aminoSlot % (uint32_t) NUM_SYRINGES) + 1u;
	UART_Print("[SEQ] Carousel -> slot %lu (%s) -- %s.\r\n",
	           (unsigned long) aminoSlot,
	           (aminoSlot & 1u) ? "ODD" : "EVEN", why);
}

/* Slot 1 sits at 0 deg and each slot is |SEQ_DISPENSER_DEG| further round, so
 * an ABSOLUTE carousel move to `deg` lands on slot 1 + (|deg| / 36), wrapped.
 * wash() uses an absolute -180 -> 180/36 = 5 -> slot 6 (EVEN, 10 ml, no fast
 * plunge). Without this the slot tracker would go stale across every wash. */
static void aminoSlotSetFromAbsoluteDeg(float deg, const char *why)
{
	float mag   = (deg < 0.0f) ? -deg : deg;
	int   steps = (int)((mag / (float)(-SEQ_DISPENSER_DEG)) + 0.5f);

	aminoSlot = (uint32_t)(steps % NUM_SYRINGES) + 1u;
	UART_Print("[SEQ] Carousel absolute %d deg -> slot %lu (%s) -- %s.\r\n",
	           (int) deg, (unsigned long) aminoSlot,
	           (aminoSlot & 1u) ? "ODD" : "EVEN", why);
}

/* ---- STEPPER: LIMIT + ODD/EVEN PLUNGE + DOSE ----------------------------- */
static void aminoStepperToLimit(const char *why)
{
	if (drainStepperAtLimit()) {
		UART_Print("[ADOSE] Stepper already at limit (%s).\r\n", why);
		return;
	}
	UART_Print("[ADOSE] Stepper NOT at limit -- homing first (%s)...\r\n", why);
	drainStepperLimit();
	osDelay(300);
}

/* DispenseTask's step 1 + step 2b + step 3, verbatim in behaviour, keyed on
 * the carousel slot instead of a loop index. THE single dosing path for the
 * whole amino sequence -- wash() included. Returns the ramped dose result. */
static bool aminoPlungeAndDose(uint16_t targetRaw, const char *why)
{
	aminoStepperToLimit(why);

	UART_Print("\r\n[ADOSE] ===== Slot %lu of %d (%s slot) -- %s =====\r\n",
	           (unsigned long) aminoSlot, NUM_SYRINGES,
	           (aminoSlot & 1u) ? "ODD" : "EVEN", why);

	if (aminoSlot & 1u) {
		UART_Print("[ADOSE] Odd slot -> plunge %d steps @ %d, then normal dose.\r\n",
		           SYRINGE_ODD_PLUNGE_STEPS, SYRINGE_ODD_PLUNGE_SPEED);
		stepperData(SYRINGE_ODD_PLUNGE_STEPS, SEQ_DOSE_DIR, SYRINGE_ODD_PLUNGE_SPEED);
		osDelay(200);
	} else {
		UART_Print("[ADOSE] Even slot -> normal dose only (no fast plunge).\r\n");
	}

	UART_Print("[ADOSE] Dosing to %u.%u g...\r\n",
	           (unsigned)(targetRaw / 10), (unsigned)(targetRaw % 10));

	return LoadCellStepperDispenseRamped(loadcell, targetRaw,
	                                    SEQ_DOSE_DIR, SEQ_DOSE_SPEED);
}

/* AminoDoseTask parks the stepper AFTER it signals DRAIN_DONE, so it can still
 * be alive when the caller starts the next dose. Reusing xAminoDoseTaskTCB
 * while that is true corrupts the scheduler, so wait for the real termination
 * (plus a tick for the idle task to reclaim the TCB) before recreating it.
 * wash() now launches two extra dose tasks per call, which makes this race
 * far more likely than it was before. */
static bool aminoDoseTaskWaitFree(void)
{
	uint32_t start = HAL_GetTick();

	while (aminoDoseTaskRunning) {
		if ((HAL_GetTick() - start) > 120000u) {
			return false;
		}
		osDelay(20);
	}

	osDelay(100);                /* let the idle task reclaim the TCB */
	aminoDoseTaskHandle = NULL;
	return true;
}

/* Open stir_drain, hold SEQ_DRAIN_FIXED_MS, close -- same shape as the valve
 * toggle in DispenseTask, just under the requested pin alias and fixed-time
 * only (no load-cell/UV endpoint here). */
static void stirDrainPulse(const char *why)
{
	UART_Print("[SEQ] Draining (%s)...\r\n", why);
	HAL_GPIO_WritePin(stir_drain_GPIO_Port, stir_drain_Pin, GPIO_PIN_SET);
	osDelay(SEQ_DRAIN_FIXED_MS);
	HAL_GPIO_WritePin(stir_drain_GPIO_Port, stir_drain_Pin, GPIO_PIN_RESET);
	osDelay(300);
}

/* ---------------------------------------------------------------------------
 *  RAIL -> STIR LANE, STIR, DRAIN, RAIL -> BACK TO THE LIQUID LANE.
 *
 *  This is the stage that was missing outright. Every runReactionStirModule()
 *  call in amino_acid_addition_sequence() used to fire with the rail still
 *  sitting at +SEQ_POWDER_TURNS, so the arm drove to the stir poses without
 *  the rail ever travelling to the stir station. sequence() always does
 *  MoveLinearServo(SEQ_STIR_TURNS) first -- this does the same thing, then
 *  returns the rail to the liquid lane afterwards so the next dose/pour finds
 *  the taught liquid poses where they belong.
 * ------------------------------------------------------------------------- */
static bool aminoStirAndDrain(const char *why)
{
	if (!aminoRailToStir(why)) {
		return false;
	}

	/* ---- Stir pass. Leaves the ROBOT GRIPPER RESET (vessel released onto the
	 * stir station) and the peristaltic pump SET -- see the i == 5 block inside
	 * runReactionStirModule(). ---- */
	reactionAtStir8Enabled = true;
	bool stirOk = runReactionStirModule();
	reactionAtStir8Enabled = false;

	if (!stirOk) {
		UART_Print("[SEQ] Stir FAILED (%s).\r\n", why);
		return false;
	}

#if AMINO_RESTIR_BEFORE_DRAIN
	/* ---- RESTIR reposture pass, exactly as sequence() runs it: immediately
	 * after the stir, with the rail still in the stir lane.
	 *
	 * This is NOT cosmetic. runReactionStirModule() ends with the gripper
	 * RESET and the pump SET; runReactionReStirModule() walks the mirrored
	 * RESTIR poses and puts the gripper back to SET and the pump back to RESET.
	 * Without it the amino sequence carried on with the gripper OPEN -- so the
	 * rail move below and the next runLiquidDispenserApproach() were both being
	 * done with the vessel un-gripped. ---- */
	if (!runReactionReStirModule()) {
		UART_Print("[SEQ] ReStir FAILED (%s).\r\n", why);
		return false;
	}

	stirDrainPulse(why);
#else
	/* Drain with the vessel still seated on the stir station, THEN reposture
	 * and re-grip it. Same two operations, opposite order -- see
	 * AMINO_RESTIR_BEFORE_DRAIN. */
	stirDrainPulse(why);

	if (!runReactionReStirModule()) {
		UART_Print("[SEQ] ReStir FAILED (%s).\r\n", why);
		return false;
	}
#endif

	/* Let the idle task reclaim the reaction task's static TCB before the next
	 * stir's STIR8 recreates it -- the same osDelay(100) sequence() does at the
	 * bottom of its per-syringe loop. This function runs ten times per amino
	 * run, so skipping it would eventually land xTaskCreateStatic() on a TCB
	 * that has not been freed yet. */
	osDelay(100);

	/* Rail back to the liquid/powder lane, ready for whatever comes next. */
	return aminoRailToLiquid(why);
}

/* Move the carousel to an ABSOLUTE angle in the slot-1 datum frame (NOT a
 * relative step like the -36 deg indexing). Now computed as a delta against
 * the software tracker and issued through aminoCarouselMoveRel(), so it uses
 * the same proven DispenserMove() path as every other carousel motion. */
static void aminoCarouselGotoAbsoluteDeg(float deg, const char *why)
{
	float fdelta = deg - aminoCarouselDeg;
	int   delta  = (int) ((fdelta < 0.0f) ? (fdelta - 0.5f) : (fdelta + 0.5f));

	UART_Print("[SEQ] Carousel -> absolute %d deg (at %+d, delta %+d) -- %s...\r\n",
	           (int) deg, (int) aminoCarouselDeg, delta, why);

	aminoCarouselMoveRel(delta, why);

	/* Snap the tracker to the commanded angle so rounding cannot accumulate. */
	aminoCarouselDeg = deg;

	/* Keep the odd/even syringe tracker in step. An absolute move is NOT a -36
	 * index, so aminoSlotAdvance() is wrong here -- recompute from the angle. */
	aminoSlotSetFromAbsoluteDeg(deg, why);
}

/* ---------------------------------------------------------------------------
 *  RETURN THE CAROUSEL TO THE SLOT-1 DATUM.
 *
 *  THIS IS THE FIX FOR "THE SECOND RUN STARTS ON THE SAME SYRINGE".
 *
 *  amino_acid_addition_sequence() sets aminoSlot = 1 at the top, but nothing
 *  ever moved the carousel BACK, so on run 2 that assignment was simply a lie:
 *  the software believed slot 1 while the ring was still parked wherever run 1
 *  left it (six -36 indexes plus the wash moves). Run 2 then drew its first
 *  dose from the syringe run 1 finished on, and every odd/even decision after
 *  that was made against the wrong physical syringe.
 *
 *  Called at BOTH ends of a run. Calling it at the START matters more than at
 *  the end, because an ABORTED run never reaches the end -- homing on entry
 *  makes the datum self-correcting after a fault.
 *
 *  The rewind unwinds the accumulated angle the way it came rather than taking
 *  the short way round. Slower, but it makes no assumption about how far the
 *  syringe tubing tolerates continuous rotation. */
static void aminoCarouselHome(const char *why)
{
	if ((int) aminoCarouselDeg == 0) {
		UART_Print("[SEQ] Carousel already at the slot-1 datum (%s).\r\n", why);
	} else {
		UART_Print("[SEQ] Carousel -> slot-1 datum: unwinding %+d deg (%s)...\r\n",
		           -(int) aminoCarouselDeg, why);
		aminoCarouselGotoAbsoluteDeg(0.0f, why);
	}

	aminoCarouselDeg = 0.0f;
	aminoSlot        = 1;
}

/* ---------------------------------------------------------------------------
 *  WASH()  --  carousel -> hardcoded -180 deg, then TWO liquid+stir+drain
 *  passes (1 g each), as requested.
 * ------------------------------------------------------------------------- */
bool wash(void)
{
	UART_Print("\r\n===== WASH =====\r\n");

	aminoCarouselGotoAbsoluteDeg(-180.0f, "wash start");

	/* TWO 1 g liquid additions, each followed by stir + drain.
	 *
	 * Both now go through aminoDoseAndPour(), so each one gets the full
	 * DispenseTask treatment: stepper homed to limit -> odd/even fast plunge
	 * for THIS carousel slot -> ramped weight-based dose -> arm at the pour
	 * pose -> drain valve OPEN -> collect -> valve CLOSE -> stepper parked back
	 * at the limit.
	 *
	 * The old code called LoadCellStepperDispenseRamped() directly. That
	 * skipped the limit-home, skipped the fast plunge, skipped the park AND
	 * never opened the drain valve -- so the wash solvent was weighed out but
	 * never actually poured into the vessel, and the plunger crawled the whole
	 * way from wherever the previous dose had left it. If wash() is genuinely
	 * meant to be valve-less, revert just these two calls; everything else in
	 * this function should stay. */
	for (int pass = 1; pass <= 2; pass++) {
		const char *why = (pass == 1) ? "wash pass 1" : "wash pass 2";

		if (!aminoDoseAndPour(WASH_LIQUID_TARGET_RAW, why)) {
			UART_Print("[WASH] Liquid dose/pour #%d FAILED.\r\n", pass);
			return false;
		}

		if (!aminoStirAndDrain(why)) {
			return false;
		}
	}

	UART_Print("\r\n[WASH] Complete.\r\n");
	return true;
}

/* NOTE: the old aminoPlungeCount / aminoPlungeIfOdd() pair that used to live
 * here is GONE. It keyed the fast plunge off "how many times has a dose been
 * launched", which drifts out of phase with the physical carousel and which
 * wash() bypassed entirely. Its replacement is the slot tracker plus
 * aminoPlungeAndDose(), defined further up this file next to the rail
 * helpers -- see the big comment block there. */

/* ---------------------------------------------------------------------------
 *  CAROUSEL -36 INDEX, with the same safety interlock as DispenseTask's step
 *  2a: the plunger (stepper) MUST be at its upper limit, clear of the
 *  syringes, before the carousel indexes -36 deg. Checks first and only
 *  drives to the limit if it is not already there, then indexes.
 * ------------------------------------------------------------------------- */
static void aminoCarouselIndex36(const char *why)
{
	/* Same interlock as before, just shared with every other stepper user. */
	aminoStepperToLimit(why);

	/* Through the shared wrapper so the angle tracker stays truthful. */
	aminoCarouselMoveRel(SEQ_DISPENSER_DEG, why);

	/* A -36 index means the NEXT syringe is now under the plunger. This single
	 * line is what keeps the odd/even fast plunge locked to the physical
	 * carousel, exactly the way DispenseTask's `slot++` loop does. */
	aminoSlotAdvance(why);
}

static void aminoCarouselIndex18(const char *why)
{
	/* Same interlock as before, just shared with every other stepper user. */
	aminoStepperToLimit(why);

	/* Through the shared wrapper so the angle tracker stays truthful. */
	aminoCarouselMoveRel(WASH_DISPENSER_DEG, why);

	/* A -36 index means the NEXT syringe is now under the plunger. This single
	 * line is what keeps the odd/even fast plunge locked to the physical
	 * carousel, exactly the way DispenseTask's `slot++` loop does. */
	aminoSlotAdvance(why);
}

/* ---------------------------------------------------------------------------
 *  WASH SOLVENT SELECTION  --  interactive console menu
 *
 *  Asks how many wash solvents are wanted, then for each one asks WHICH
 *  solvent and HOW MUCH, drives the carousel to that solvent's slot, and runs
 *  the pump + DC motor for a fixed window before shutting both off.
 *
 *  Per solvent:
 *      pick name -> pick amount -> wash_solvents() moves the carousel
 *      -> X_pump SET -> DC_Motor SET -> 30 s -> DC_Motor RESET -> X_pump RESET
 *
 *  The carousel move goes through wash_solvents() (servo.h), so the slot
 *  positions and the solvent names live in exactly one place.
 *
 *  NOTE ON THE AMOUNT: the requested amount is recorded and echoed, but
 *  nothing meters it -- the volume actually delivered is whatever the pump
 *  moves in WASH_SEL_PUMP_RUN_MS, which is fixed at the 30 s that was asked
 *  for. If the amount is supposed to govern the dose, this is the place to
 *  either scale the run time or route it through aminoDoseAndPour() like
 *  wash() does.
 * ------------------------------------------------------------------------- */

#define WASH_SEL_PUMP_RUN_MS      30000u   /* pump + DC motor ON window        */
#define WASH_SEL_SETTLE_MS         5000u   /* gap between each output change   */
#define WASH_SEL_SERVO_SETTLE_MS    500u   /* let the plate stop before pumping */
#define WASH_SEL_INPUT_LEN           48
#define WASH_SEL_MAX_TRIES            3    /* re-prompts before giving up      */
#define WASH_SEL_BLANK_SKIP           4    /* blank lines swallowed per prompt */

static char washSelLower(char c)
{
	return (c >= 'A' && c <= 'Z') ? (char) (c - 'A' + 'a') : c;
}

/* Strip leading/trailing blanks and any stray CR/LF. UART_ReadLine() already
 * drops the newline and zero-fills, but a pasted line can still carry them. */
static char *washSelTrim(char *s)
{
	char *end;

	while (*s == ' ' || *s == '\t') {
		s++;
	}

	end = s;
	while (*end != '\0') {
		end++;
	}

	while (end > s) {
		char c = *(end - 1);

		if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
			end--;
		} else {
			break;
		}
	}

	*end = '\0';

	return s;
}

/* Whole-string unsigned parse. Rejects "12abc" and "" -- deliberately stricter
 * than atoi(), which would silently read those as 12 and 0. */
static bool washSelParseUInt(const char *s, uint32_t *out)
{
	uint32_t value = 0;
	bool     any   = false;

	while (*s >= '0' && *s <= '9') {
		value = (value * 10u) + (uint32_t) (*s - '0');
		any   = true;
		s++;
	}

	if (!any || *s != '\0') {
		return false;
	}

	*out = value;

	return true;
}

static bool washSelNamesEqual(const char *a, const char *b)
{
	while (*a != '\0' && *b != '\0') {
		if (washSelLower(*a) != washSelLower(*b)) {
			return false;
		}

		a++;
		b++;
	}

	return (*a == '\0' && *b == '\0');
}

/* Read one NON-BLANK line from the console.
 *
 * THIS IS THE FIX FOR "Wash solvent selection #1 FAILED".
 *
 * UART_ReadLine() returns on either CR or LF. A terminal that ends lines with
 * CRLF therefore satisfies one read with the CR and leaves the LF sitting in
 * the UART -- so the VERY NEXT read completes instantly with an empty string,
 * before the operator has typed anything. The old code fed that "" straight
 * into the strict digit parser, which rejected it and returned false, and a
 * false from any prompt aborted the whole octertoide run.
 *
 * Blank lines are therefore skipped rather than treated as answers. The cap
 * stops a noisy or disconnected line from wedging the sequence forever. */
static bool washSelReadLine(char *buf, uint16_t len, char **out)
{
	for (int i = 0; i < WASH_SEL_BLANK_SKIP; i++) {
		UART_ReadLine(buf, len);

		*out = washSelTrim(buf);

		if ((*out)[0] != '\0') {
			return true;
		}
	}

	return false;
}

/* Re-prompts instead of giving up on the first bad entry. A single mistyped
 * character used to abort the entire run from here. */
static bool washSelAskUInt(const char *prompt, uint32_t *out)
{
	for (int attempt = 0; attempt < WASH_SEL_MAX_TRIES; attempt++) {
		char  buf[WASH_SEL_INPUT_LEN];
		char *text;

		UART_Print("%s", prompt);

		if (!washSelReadLine(buf, sizeof(buf), &text)) {
			UART_Print("[WASHSEL] No input received -- try again.\r\n");
			continue;
		}

		if (washSelParseUInt(text, out)) {
			return true;
		}

		UART_Print("[WASHSEL] '%s' is not a number -- digits only, no decimal "
		           "point (e.g. 15 for 1.5 g).\r\n", text);
	}

	UART_Print("[WASHSEL] No valid number after %d attempts.\r\n",
	           WASH_SEL_MAX_TRIES);

	return false;
}

static bool washSelAskYes(const char *prompt)
{
	char  buf[WASH_SEL_INPUT_LEN];
	char *text;

	UART_Print("%s", prompt);
	UART_ReadLine(buf, sizeof(buf));

	text = washSelTrim(buf);

	return (washSelLower(text[0]) == 'y');
}

static void washSelPrintList(void)
{
	UART_Print("\r\n  Available wash solvents:\r\n");

	for (int i = 0; i < WASH_SLOT_COUNT; i++) {
		UART_Print("    %2d) %s\r\n", i + 1,
		           WashSolventSlotName((WashSolventSlot_t) i));
	}
}

/* Accepts either the menu number (1..WASH_SLOT_COUNT) or the solvent name
 * spelled out, case-insensitively. Re-prompts up to WASH_SEL_MAX_TRIES. */
static bool washSelAskSolvent(WashSolventSlot_t *out)
{
	for (int attempt = 0; attempt < WASH_SEL_MAX_TRIES; attempt++) {
		char     buf[WASH_SEL_INPUT_LEN];
		char    *text;
		uint32_t number;

		washSelPrintList();
		UART_Print("  Solvent (number or name) : ");

		/* Same blank-line guard as washSelAskUInt() -- without it a stray LF
		 * from a CRLF terminal would burn one of the three attempts. */
		if (!washSelReadLine(buf, sizeof(buf), &text)) {
			UART_Print("[WASHSEL] No input received -- try again.\r\n");
			continue;
		}

		if (washSelParseUInt(text, &number)) {
			if (number >= 1u && number <= (uint32_t) WASH_SLOT_COUNT) {
				*out = (WashSolventSlot_t) (number - 1u);

				return true;
			}

			UART_Print("[WASHSEL] %lu is out of range (1..%d).\r\n",
			           (unsigned long) number, WASH_SLOT_COUNT);

			continue;
		}

		for (int i = 0; i < WASH_SLOT_COUNT; i++) {
			if (washSelNamesEqual(text,
			                      WashSolventSlotName((WashSolventSlot_t) i))) {
				*out = (WashSolventSlot_t) i;

				return true;
			}
		}

		UART_Print("[WASHSEL] '%s' is not a known solvent.\r\n", text);
	}

	UART_Print("[WASHSEL] Too many bad entries -- aborting.\r\n");

	return false;
}

/* Keep THIS file's carousel tracker in step with servo.c's.
 *
 * Two independent trackers now exist for one physical plate: aminoCarouselDeg
 * here (accumulated raw DispenserMove degrees) and gDispenserAngle inside
 * servo.c (absolute, 0..360). wash_solvents() moves only the latter, so
 * without this the next aminoCarouselGotoAbsoluteDeg() would compute its delta
 * from a stale datum and send the plate to the wrong place.
 *
 * servo.c turns its angle into DispenserMove() degrees through
 * DISPENSER_SLOT_DIR, so that constant is exactly the factor between the two
 * trackers. It is used here rather than a hardcoded sign so that flipping the
 * plate's direction cannot silently desync them -- which is what would have
 * happened when it went from -1 to +1. The wrap loop picks the representation
 * nearest the current value, so the sync itself never implies a spurious extra
 * revolution. */
static void washSelSyncCarouselTracker(void)
{
	float target = (float) DISPENSER_SLOT_DIR * ServoDispenserAngle();

	while ((target - aminoCarouselDeg) > 180.0f) {
		target -= 360.0f;
	}

	while ((target - aminoCarouselDeg) < -180.0f) {
		target += 360.0f;
	}

	aminoCarouselDeg = target;
}

/* Pump + DC motor cycle for one solvent, in the order requested:
 * X_pump ON -> DC motor ON -> hold -> DC motor OFF -> X_pump OFF. */
static void washSelRunPumpCycle(const char *solvent)
{
	//UART_Print("[WASHSEL] %s : X_pump ON.\r\n", solvent);
	//HAL_GPIO_WritePin(B_Pump_GPIO_Port, B_Pump_Pin, GPIO_PIN_SET);
	//osDelay(WASH_SEL_SETTLE_MS);

	UART_Print("[WASHSEL] %s : DC motor ON.\r\n", solvent);
	HAL_GPIO_WritePin(DC_Motor_pin_GPIO_Port, DC_Motor_pin_Pin, GPIO_PIN_SET);

	UART_Print("[WASHSEL] %s : running for %lu ms...\r\n", solvent,
	           (unsigned long) WASH_SEL_PUMP_RUN_MS);
	osDelay(WASH_SEL_PUMP_RUN_MS);

	UART_Print("[WASHSEL] %s : DC motor OFF.\r\n", solvent);
	HAL_GPIO_WritePin(DC_Motor_pin_GPIO_Port, DC_Motor_pin_Pin, GPIO_PIN_RESET);
	osDelay(WASH_SEL_SETTLE_MS);

	//UART_Print("[WASHSEL] %s : X_pump OFF.\r\n", solvent);
	//HAL_GPIO_WritePin(B_Pump_GPIO_Port, B_Pump_Pin, GPIO_PIN_RESET);
	//osDelay(WASH_SEL_SETTLE_MS);
}

bool wash_solvent_selection(void)
{
	uint32_t wanted = 0;

	UART_Print("\r\n===== WASH SOLVENT SELECTION =====\r\n");

	if (!washSelAskUInt("How many wash solvents do you want to use? : ",
	                    &wanted)) {
		UART_Print("[WASHSEL] Aborted.\r\n");

		return false;
	}

	if (wanted == 0u) {
		UART_Print("[WASHSEL] Nothing to do.\r\n");

		return true;
	}

	/* More selections than there are solvents is almost always a typo, but it
	 * is legal -- the same solvent can be run twice -- so warn, don't refuse. */
	if (wanted > (uint32_t) WASH_SLOT_COUNT) {
		UART_Print("[WASHSEL] Note: %lu selections requested but only %d "
		           "solvents exist -- repeats allowed.\r\n",
		           (unsigned long) wanted, WASH_SLOT_COUNT);
	}

	UART_Print("[WASHSEL] Running %lu selection(s).\r\n",
	           (unsigned long) wanted);

	for (uint32_t n = 1u; n <= wanted; n++) {
		WashSolventSlot_t slot;
		const char       *name;
		uint32_t          amount = 0;

		UART_Print("\r\n----- Solvent %lu of %lu -----\r\n",
		           (unsigned long) n, (unsigned long) wanted);

		if (!washSelAskSolvent(&slot)) {
			return false;
		}

		name = WashSolventSlotName(slot);

		if (!washSelAskUInt("  Amount to add (tenths of a gram, e.g. 10 = 1.0 g) : ",
		                    &amount)) {
			UART_Print("[WASHSEL] Aborted.\r\n");

			return false;
		}

		UART_Print("[WASHSEL] Selected %s, amount %lu (%lu.%lu g).\r\n", name,
		           (unsigned long) amount, (unsigned long) (amount / 10u),
		           (unsigned long) (amount % 10u));

		/* ---- Dispenser servo (MicroID) HOME, then out to this solvent ----
		 * The solvent is chosen above but NOTHING has turned yet; the very
		 * first rotation of this iteration is the home move below.
		 *
		 * "Home" is the encoder count latched when the servo started -- the
		 * caller declares it once (octertoide_sequence() does so at the top of
		 * the run) and DispenserMoveHome() returns to exactly that count every
		 * time, verifying arrival. It re-datums servo.c's angle tracker to
		 * 0 deg on arrival, so the slot angles (DCM 16 deg clockwise, then a
		 * 30 deg pitch anticlockwise) are measured from that datum on every
		 * single solvent -- never from wherever the previous one left the
		 * plate, and with no accumulated drift.
		 *
		 * Only after both moves succeed does the pump/motor GPIO sequence run,
		 * further down -- so the outputs never switch while the plate is still
		 * travelling or parked on the wrong slot.
		 *
		 * The bus lock is this file's, and every other USART6 call here takes
		 * it, so these do too -- neither DispenserMoveHome() nor
		 * wash_solvents() can take it itself (the lock is static to this
		 * file). It is held across the pair so nothing can drive the plate in
		 * between the home and the slot move. */
		UART_Print("[WASHSEL] Dispenser servo -> home before %s...\r\n", name);

		servoBusLock();
		bool homed = DispenserMoveHome(servo);
		bool moved = homed ? wash_solvents(servo, slot) : false;
		servoBusUnlock();

		washSelSyncCarouselTracker();

		if (!homed) {
			UART_Print("[WASHSEL] Dispenser home FAILED before %s -- "
			           "skipping the pump for this solvent.\r\n", name);

			continue;
		}

		if (!moved) {
			UART_Print("[WASHSEL] Carousel move to %s FAILED -- "
			           "skipping the pump for this solvent.\r\n", name);

			continue;
		}

		UART_Print("[WASHSEL] Carousel at %s (%d deg).\r\n", name,
		           (int) ServoDispenserAngle());

		/* Let the plate come fully to rest before liquid starts moving. */
		osDelay(WASH_SEL_SERVO_SETTLE_MS);

		washSelRunPumpCycle(name);

		UART_Print("[WASHSEL] %s complete.\r\n", name);

		/* Offer an early way out. Only worth asking while there is something
		 * left to do -- after the last one the function exits anyway, which is
		 * the requested behaviour. */
		if (n < wanted) {
			if (washSelAskYes("Exit the wash solvent menu? (y/n) : ")) {
				UART_Print("[WASHSEL] Exiting early at the user's request "
				           "(%lu of %lu done).\r\n",
				           (unsigned long) n, (unsigned long) wanted);

				return true;
			}
		}
	}

	UART_Print("\r\n[WASHSEL] All %lu selection(s) complete -- exiting.\r\n",
	           (unsigned long) wanted);

	return true;
}

/* ---------------------------------------------------------------------------
 *  AMINO ACID SELECTION  --  interactive console menu
 *
 *  The syringe-side twin of wash_solvent_selection(): the operator picks one of
 *  the eight amino acids and the weight, the plate is homed and driven to that
 *  syringe (36 deg per syringe, anticlockwise, via AminoAcidSlots() ->
 *  DispenserMove()), and the load-cell chain delivers the weight.
 *
 *  The dose goes through aminoDoseAndPour() -> AminoDoseTask ->
 *  aminoPlungeAndDose() -> LoadCellStepperDispenseRamped(), i.e. exactly the
 *  path sequence() and amino_acid_addition_sequence() dose through: limit home,
 *  odd/even fast plunge, ramped weight-based dose, arm at the pour pose, drain
 *  valve open, collect, valve close, stepper parked.
 *
 *  One selection per call -- this replaces a single fixed dose step in the
 *  sequence, so it deliberately does not loop the way the wash menu does.
 * ------------------------------------------------------------------------- */

/* Console labels. Short name first (that is also what AminoAcidSlotName()
 * returns and what the name-matching below accepts), then the reagent.
 *
 * NOTE: eight entries were asked for but only seven names given, so AA_SLOT_8
 * stays a placeholder -- rename it here and in servo.h once it is known. */
/* The amino menu doses through the proven load-cell path (aminoDoseAndPour),
 * the same weight-metered dose sequence() and amino_acid_addition_sequence()
 * use -- the stepper rotates until the entered weight is collected on the load
 * cell. The earlier proximity-stepper parameters (AASEL_STEPS_PER_TENTH_GRAM
 * etc.) are gone with it: that path did not rotate the stepper to a weight, so
 * the arm approached and retracted with no real dose. */

static const char *const aminoMenuNames[AA_SLOT_COUNT] = {
	[AA_THREONOL]    = "Therinol    - Fmoc-Thr(tBu)-ol",
	[AA_CYSTEINE]    = "Cysteine    - Fmoc-Cys(Trt)-OH",
	[AA_THREONINE]   = "Theronine   - Fmoc-Thr(tBu)-OH",
	[AA_LYSINE]      = "Lysine      - Fmoc-Lys(Boc)-OH",
	[AA_DTRAPTOC]    = "DTraptoc    - Fmoc-D-Trp(Boc)-OH",
	[AA_PHENAMINE]   = "Phenamine   - Fmoc-Phe-OH",
	[AA_D_PHENAMINE] = "DPhenamine  - Fmoc-D-Phe-OH",
	[AA_SLOT_8]      = "Slot-8      - (8th syringe, name not supplied)",
};

static void aminoSelPrintList(void)
{
	UART_Print("\r\n  Available amino acids:\r\n");

	for (int i = 0; i < AA_SLOT_COUNT; i++) {
		UART_Print("    %d) %s\r\n", i + 1, aminoMenuNames[i]);
	}
}

/* Accepts the menu number (1..8) or the short name, case-insensitively.
 * Uses the same blank-line-tolerant reader as the wash menu, so a stray LF from
 * a CRLF terminal cannot burn an attempt. */
static bool aminoSelAskSlot(AminoAcidSlot_t *out)
{
	for (int attempt = 0; attempt < WASH_SEL_MAX_TRIES; attempt++) {
		char     buf[WASH_SEL_INPUT_LEN];
		char    *text;
		uint32_t number;

		aminoSelPrintList();
		UART_Print("  Amino acid (number or name) : ");

		if (!washSelReadLine(buf, sizeof(buf), &text)) {
			UART_Print("[AASEL] No input received -- try again.\r\n");
			continue;
		}

		if (washSelParseUInt(text, &number)) {
			if (number >= 1u && number <= (uint32_t) AA_SLOT_COUNT) {
				*out = (AminoAcidSlot_t) (number - 1u);

				return true;
			}

			UART_Print("[AASEL] %lu is out of range (1..%d).\r\n",
			           (unsigned long) number, AA_SLOT_COUNT);

			continue;
		}

		for (int i = 0; i < AA_SLOT_COUNT; i++) {
			if (washSelNamesEqual(text,
			                      AminoAcidSlotName((AminoAcidSlot_t) i))) {
				*out = (AminoAcidSlot_t) i;

				return true;
			}
		}

		UART_Print("[AASEL] '%s' is not a known amino acid.\r\n", text);
	}

	UART_Print("[AASEL] Too many bad entries -- aborting.\r\n");

	return false;
}

bool amino_acid_selection(void)
{
	AminoAcidSlot_t slot;
	uint32_t        amount = 0;
	const char     *name;

	UART_Print("\r\n===== AMINO ACID SELECTION =====\r\n");

	if (!aminoSelAskSlot(&slot)) {
		return false;
	}

	name = AminoAcidSlotName(slot);

	if (!washSelAskUInt("  Weight to dispense (tenths of a gram, "
	                    "e.g. 50 = 5.0 g) : ", &amount)) {
		UART_Print("[AASEL] Aborted.\r\n");

		return false;
	}

	if (amount == 0u) {
		UART_Print("[AASEL] Zero weight -- nothing to dispense.\r\n");

		return true;
	}

	/* aminoDoseAndPour() takes a uint16_t, so anything above 65535 raw would
	 * wrap into a silently different (and much smaller) target. */
	if (amount > 0xFFFFu) {
		UART_Print("[AASEL] %lu is too large -- max 65535 raw (6553.5 g).\r\n",
		           (unsigned long) amount);

		return false;
	}

	UART_Print("[AASEL] Selected %s, weight %lu (%lu.%lu g).\r\n", name,
	           (unsigned long) amount, (unsigned long) (amount / 10u),
	           (unsigned long) (amount % 10u));

	/* ---- Plate to the latched home, then out to this syringe ----
	 * Homing first makes the syringe angle absolute from the run datum rather
	 * than relative to wherever the last wash selection left the plate. Both
	 * calls are inside one bus lock so nothing can drive the plate in between;
	 * neither can take the lock itself, it is static to this file. */
	UART_Print("[AASEL] Dispenser servo -> home before %s...\r\n", name);

	servoBusLock();
	bool homed = DispenserMoveHome(servo);
	bool moved = homed ? AminoAcidSlots(servo, slot) : false;
	servoBusUnlock();

	washSelSyncCarouselTracker();

	if (!homed) {
		UART_Print("[AASEL] Dispenser home FAILED before %s.\r\n", name);

		return false;
	}

	if (!moved) {
		UART_Print("[AASEL] Carousel move to %s FAILED.\r\n", name);

		return false;
	}

	/* Keep the odd/even plunge tracker on the syringe actually under the
	 * plunger. aminoPlungeAndDose() keys the fast 60 ml plunge off aminoSlot,
	 * so leaving it stale would prime the wrong syringe type. Slots are
	 * 0-based here and 1-based there. */
	aminoSlot = (uint32_t) slot + 1u;

	UART_Print("[AASEL] Carousel at %s (%d deg), slot %lu (%s).\r\n", name,
	           (int) ServoDispenserAngle(), (unsigned long) aminoSlot,
	           (aminoSlot & 1u) ? "ODD" : "EVEN");

	/* Let the plate come fully to rest before the plunger drives. */
	osDelay(WASH_SEL_SERVO_SETTLE_MS);

	/* ---- DISPENSE + LIQUID POUR : the PROVEN load-cell path ----
	 * This is the exact working dose used by sequence() and
	 * amino_acid_addition_sequence(): aminoDoseAndPour() launches AminoDoseTask,
	 * which runs aminoPlungeAndDose() -> LoadCellStepperDispenseRamped() -- so
	 * the stepper ROTATES UNTIL THE ENTERED WEIGHT IS COLLECTED, weighed on the
	 * load cell, in parallel with the arm's approach to the pour pose. It then
	 * opens the drain valve, collects, closes, and the arm retracts.
	 *
	 * The proximity dispense that was here did not meter by weight -- the
	 * stepper was not rotating to the target -- which is why the arm approached
	 * and retracted with no real dose in between. Switched back to the load-cell
	 * logic as requested; `amount` is the target in tenths of a gram, exactly
	 * what aminoDoseAndPour() expects. */
	if (!aminoDoseAndPour((uint16_t) amount, name)) {
		UART_Print("[AASEL] Dose/pour FAILED for %s.\r\n", name);
		return false;
	}

	UART_Print("[AASEL] %s dispense + pour complete.\r\n", name);

	return true;
}

bool aminoPowderAddition(void)
{
    if (!runPowderDispenserModule())
        return false;

    // future:
    // powder weighing
    // carousel indexing
    // vibration
    // confirmation

    return true;
}

/* ---------------------------------------------------------------------------
 *  AMINO DOSE TASK  --  one-shot version of DispenseTask's weigh+pour
 *  rendezvous, for amino_acid_addition_sequence(). Doses on the load cell
 *  (LoadCellStepperDispenseRamped) WHILE the caller runs
 *  runLiquidDispenserApproach() in parallel -- same overlap sequence() gets
 *  from DispenseTask, just without the syringe loop / powder wait, since this
 *  function doses named steps one at a time, not an indexed syringe rack.
 * ------------------------------------------------------------------------- */
static void AminoDoseTask(void const *argument)
{
	(void) argument;
	aminoDoseTaskRunning = true;

	/* Stepper to limit -> odd/even fast plunge for the CURRENT CAROUSEL SLOT ->
	 * ramped weight-based dose. Same three steps, same order, as DispenseTask's
	 * steps 1, 2b and 3 -- all of it inside aminoPlungeAndDose() now so wash()
	 * gets identical behaviour. */
	if (!aminoPlungeAndDose(aminoDoseTargetRaw, "amino dose")) {
		UART_Print("[ADOSE] Dose FAILED.\r\n");
		dispenseOk = false;
		osSignalSet(sequenceTaskHandle, SIG_WEIGHT_READY);
		osSignalSet(sequenceTaskHandle, SIG_DRAIN_DONE);
		aminoDoseTaskRunning = false;
		osThreadTerminate(osThreadGetId());
		return;
	}

	UART_Print("[ADOSE] >>> WEIGHT READY.\r\n");
	osSignalSet(sequenceTaskHandle, SIG_WEIGHT_READY);

	/* Hold until the robot is at the liquid-dispensing pose. */
	osEvent evt = osSignalWait(SIG_ROBOT_READY, SIG_DOSE_TIMEOUT_MS);
	if (evt.status == osEventTimeout) {
		UART_Print("[ADOSE] Robot never signalled ready -- valve NOT opened.\r\n");
		dispenseOk = false;
		osSignalSet(sequenceTaskHandle, SIG_DRAIN_DONE);
		aminoDoseTaskRunning = false;
		osThreadTerminate(osThreadGetId());
		return;
	}

	UART_Print("[ADOSE] Robot at pour pose -- OPENING drain valve.\r\n");
	HAL_GPIO_WritePin(Drain_valve_pin_Pin_GPIO_Port, Drain_valve_pin_Pin_Pin, GPIO_PIN_SET);

	bool drained;
#if SEQ_DRAIN_BY_WEIGHT
	{
		uint32_t start     = HAL_GetTick();
		uint16_t weight    = 0;
		int      emptyHits = 0;
		drained = false;

		while ((HAL_GetTick() - start) < SEQ_DRAIN_TIMEOUT_MS) {
			osDelay(1000);
			if (!LoadCellReadWeight(loadcell, &weight)) continue;

			UART_Print("[ADOSE] Collecting... weight = %u.%u g\r\n",
			           (unsigned)(weight / 10), (unsigned)(weight % 10));

			if (weight <= SEQ_EMPTY_RAW) {
				if (++emptyHits >= SEQ_EMPTY_STABLE) { drained = true; break; }
			} else {
				emptyHits = 0;
			}
		}
	}
#else
	UART_Print("[ADOSE] Valve open for %u ms (weight polling disabled)...\r\n",
	           (unsigned) SEQ_DRAIN_FIXED_MS);
	osDelay(SEQ_DRAIN_FIXED_MS);
	drained = true;
#endif

	HAL_GPIO_WritePin(Drain_valve_pin_Pin_GPIO_Port, Drain_valve_pin_Pin_Pin, GPIO_PIN_RESET);
	osDelay(300);

	UART_Print(drained ? "[ADOSE] Liquid collected.\r\n"
	                    : "[ADOSE] DRAIN TIMEOUT -- valve closed.\r\n");

	/* Report the result FIRST so the caller does not wait on the stepper --
	 * same ordering as DispenseTask's step 9/10. */
	dispenseOk = drained;
	osSignalSet(sequenceTaskHandle, SIG_DRAIN_DONE);

	/* Park the head at its limit, in PARALLEL with the caller's next move --
	 * same as DispenseTask's step 10. WITHOUT this the stepper is left
	 * wherever the ramped dose stopped it, so the NEXT plunge's
	 * drainStepperAtLimit() check always fails and has to run a full, slow,
	 * chunked drainStepperLimit() homing pass before it can even get to the
	 * odd/even branch -- which buries the fast plunge under that homing time
	 * on every single dose, not just the first. */
	UART_Print("[ADOSE] Stepper -> upper limit (park, parallel)...\r\n");
	drainStepperLimit();
	osDelay(300);

	aminoDoseTaskRunning = false;
	osThreadTerminate(osThreadGetId());
}

/* Launches AminoDoseTask for `targetRaw`, runs the arm approach IN PARALLEL,
 * then rendezvouses exactly like sequence()'s per-syringe loop: wait
 * WEIGHT_READY -> signal ROBOT_READY -> wait DRAIN_DONE -> retract. Any
 * `false` return here is meant to be wrapped by the caller in
 * `return sequenceAbort(...)`, which is what actually terminates a still-
 * running aminoDoseTaskHandle -- this function does not clean up itself. */
static bool aminoDoseAndPour(uint16_t targetRaw, const char *why)
{
	UART_Print("[SEQ] Dose+pour: %s (%u.%u g)...\r\n", why,
	           (unsigned)(targetRaw / 10), (unsigned)(targetRaw % 10));

	/* The pour poses were taught with the rail in the liquid/powder lane, so
	 * make sure that is where it is before the arm approaches. After a stir
	 * aminoStirAndDrain() has already brought it back, so this is normally a
	 * no-op -- but it makes every dose self-sufficient. */
	if (!aminoRailToLiquid(why)) {
		return false;
	}

	/* Never recreate the static TCB while the previous dose task is still
	 * parking the stepper. */
	if (!aminoDoseTaskWaitFree()) {
		UART_Print("[SEQ] Previous amino dose task never finished (%s).\r\n", why);
		return false;
	}

	(void) osSignalWait(SIG_WEIGHT_READY, 0);   /* drain stale signals */
	(void) osSignalWait(SIG_ROBOT_READY,  0);
	(void) osSignalWait(SIG_DRAIN_DONE,   0);
	dispenseOk        = false;
	aminoDoseTargetRaw = targetRaw;

	aminoDoseTaskHandle = (osThreadId) xTaskCreateStatic(
			(TaskFunction_t) AminoDoseTask,
			"aminoDoseTask",
			AMINO_DOSE_TASK_STACK_WORDS,
			NULL,
			(UBaseType_t) (tskIDLE_PRIORITY + 3),
			xAminoDoseTaskStack,
			&xAminoDoseTaskTCB);

	if (aminoDoseTaskHandle == NULL) {
		UART_Print("[SEQ] Could not start amino dose task (%s).\r\n", why);
		return false;
	}
	aminoDoseTaskRunning = true;

	/* Arm travels to the pour pose WHILE the dose task doses -- parallel,
	 * same overlap as sequence()'s rendezvous. */

	if (!runLiquidDispenserApproach()) {
		UART_Print("[SEQ] Liquid approach FAILED (%s).\r\n", why);
		return false;
	}

	UART_Print("[SEQ] At pour pose -- waiting for WEIGHT READY (%s)...\r\n", why);
	osEvent evt = osSignalWait(SIG_WEIGHT_READY, SIG_DOSE_TIMEOUT_MS);
	if (evt.status == osEventTimeout) {
		UART_Print("[SEQ] Timed out waiting for the dose (%s).\r\n", why);
		return false;
	}

	UART_Print("[SEQ] >>> ROBOT READY (%s).\r\n", why);
	osSignalSet(aminoDoseTaskHandle, SIG_ROBOT_READY);

	evt = osSignalWait(SIG_DRAIN_DONE, SIG_DOSE_TIMEOUT_MS);
	if (evt.status == osEventTimeout) {
		UART_Print("[SEQ] Timed out waiting for liquid collection (%s).\r\n", why);
		return false;
	}
	if (!dispenseOk) {
		UART_Print("[SEQ] Dose/collect reported FAILURE (%s).\r\n", why);
		return false;
	}

	/* Do NOT clear aminoDoseTaskRunning here -- the dose task is still alive,
	 * parking the stepper in parallel with the retract below, and it clears the
	 * flag itself as its last act. Clearing it early was what let the next
	 * xTaskCreateStatic() land on a TCB that was still in use.
	 * aminoDoseTaskWaitFree() (called at the top of the next dose) does the
	 * real join. */

	if (!runLiquidDispenserRetract()) {
		UART_Print("[SEQ] Liquid retract FAILED (%s).\r\n", why);
		return false;
	}

	return true;
}
/* ---------------------------------------------------------------------------
 *  AMINO ACID ADDITION SEQUENCE
 *
 *  Stage order and dose targets are unchanged from before. What changed:
 *
 *   - RAIL. The rail is homed to the hardcoded datum first so the relative
 *     MoveLinearServo() deltas start from a known place (an aborted previous
 *     run could have left it at +100, in which case another +30 would drive it
 *     off the end of the track). It then travels to the liquid/powder lane, and
 *     from that point aminoStirAndDrain() shuttles it liquid -> stir -> liquid
 *     around EVERY stir, which is what sequence() does per syringe and what
 *     this function never did at all.
 *
 *   - ODD/EVEN. aminoSlot is reset to 1 (rest position) here and is then
 *     maintained by aminoCarouselIndex36() and aminoCarouselGotoAbsoluteDeg(),
 *     and the ring is rewound to the slot-1 datum at both ends of the run,
 *     so the fast plunge follows the physical syringe rather than a dose count.
 *
 *   - RESTIR. Every stage now runs runReactionReStirModule() after the stir,
 *     which sequence() always did and this function never did. The stir pass
 *     leaves the gripper RESET; RESTIR is what puts it back to SET. Without
 *     it the rail moves and liquid approaches that follow a stir were all
 *     happening with the vessel un-gripped.
 *
 *   - Every stir + RESTIR + drain group is now one aminoStirAndDrain() call
 *     instead of four repeated lines, so no stage can silently miss the rail
 *     move or the reposture pass while another stage has it.
 * ------------------------------------------------------------------------- */
bool amino_acid_addition_sequence(void)
{
	UART_Print("\r\n########## AMINO ACID ADDITION SEQUENCE START ##########\r\n");

	UART_Print("[SEQ] Robot gripper ON (held for the run).\r\n");
	HAL_GPIO_WritePin(Robot_gripper_GPIO_Port, Robot_gripper_Pin, GPIO_PIN_SET);

	/* Servo-bus lock: make sure it starts free (a prior aborted run may have
	 * left it held) -- same safety init as sequence(). */
	servoBusUnlock();

	/* Register this task so the amino dose task / reaction task have a valid
	 * handle to signal back to -- same as sequence(). */
	sequenceTaskHandle = osThreadGetId();

	/* Drain stale rendezvous signals from any previous run. */
	(void) osSignalWait(SIG_WEIGHT_READY, 0);
	(void) osSignalWait(SIG_ROBOT_READY,  0);
	(void) osSignalWait(SIG_DRAIN_DONE,   0);

	/* ---- CAROUSEL DATUM. Rewind the ring to slot 1 before anything else.
	 * This is what makes run 2, run 3 ... start on the SAME physical syringe
	 * run 1 did. Setting aminoSlot = 1 without this move (what the code used to
	 * do) just told the software a comfortable lie. aminoCarouselHome() sets
	 * aminoSlot = 1 itself once the ring is actually there.
	 * Slot 1 is ODD, so the first dose still gets the fast plunge. ---- */
	aminoCarouselHome("run start");

	/* Home the stepper to its upper limit ONCE, up front. AminoDoseTask parks it
	 * back at the limit after every dose from here on, so this should be the
	 * only full homing pass the whole run needs -- every later plunge starts
	 * from the limit and therefore actually gets to run fast. */
	UART_Print("[SEQ] Homing stepper to its upper limit before the run starts...\r\n");
	aminoStepperToLimit("run start");

	/* ---- Rail datum. Absolute move to the hardcoded home so aminoRailTurns is
	 * true, THEN out to the liquid/powder lane. ---- */
	UART_Print("[SEQ] Rail -> hardcoded home (establishing the run datum)...\r\n");
	servoBusLock();
	bool railHomed = robotLinearGotoHardcodedHome();
	servoBusUnlock();
	if (!railHomed) {
		return sequenceAbort("Rail home FAILED (cannot establish datum)");
	}
	aminoRailTurns = AMINO_RAIL_HOME;
	osDelay(300);

	if (!aminoPowderAddition())
	    return sequenceAbort("Powder addition FAILED");

	if (!aminoRailToLiquid("run start")) {
		return sequenceAbort("Rail move to the liquid lane FAILED");
	}

	/* ---- 1. Liquid dispense, syringe 1 (5 g), stir, drain. ---- */
	if (!aminoDoseAndPour(Therinol_TARGET, "Therinol(Fmoc-Thr(tBu)-ol) Addition "))
		return sequenceAbort("Therinol addition FAILED (syringe 1)");

	if (!aminoStirAndDrain("after syringe 1"))
		return sequenceAbort("Stir/drain FAILED (after syringe 1)");

	/* ---- 2. Carousel -36, liquid dispense (1 g), stir, drain. ---- */
	aminoCarouselIndex18("Socl2 solvent, 1st");

	//if (!aminoDoseAndPour(WASH_LIQUID_TARGET_RAW, "wash solvent, 2nd"))
		//return sequenceAbort("Liquid dose/pour FAILED (wash solvent, 2nd)");

	//if (!aminoStirAndDrain("after 2nd syringe"))
		//return sequenceAbort("Stir/drain FAILED (after 2nd syringe)");

	/* ---- 3. Carousel -36, liquid dispense (1 g), stir, drain. ---- */
	aminoCarouselIndex36("DCM Wash solvent, 2ndd");

	//if (!aminoDoseAndPour(WASH_LIQUID_TARGET_RAW, "wash solvent, 3rd"))
		//return sequenceAbort("Liquid dose/pour FAILED (wash solvent, 3rd)");

	//if (!aminoStirAndDrain("after 3rd syringe"))
		//return sequenceAbort("Stir/drain FAILED (after 3rd syringe)");

	/* ---- 4. Carousel -36, protection solvent, stir, drain. ---- */
	aminoCarouselIndex36("protection solvent");

	if (!aminoDoseAndPour(WASH_LIQUID_TARGET_RAW, "protection solvent"))
		return sequenceAbort("Liquid dose/pour FAILED (protection solvent)");

	if (!aminoStirAndDrain("after 4th -36 index"))
		return sequenceAbort("Stir/drain FAILED (after 4th -36 index)");

	/* ---- 5. Carousel -36 (NO dose), stir, drain.
	 * This index with no dose after it is exactly what used to knock the old
	 * dose-count parity out of step with the carousel. aminoSlotAdvance() inside
	 * aminoCarouselIndex36() now handles it correctly. ---- */
	aminoCarouselIndex36("5th -36 index, no dose");

	if (!aminoStirAndDrain("after 5th -36 index"))
		return sequenceAbort("Stir/drain FAILED (after 5th -36 index)");

	/* ---- 6. wash(), then stir, drain. ---- */
	if (!wash()) return sequenceAbort("Wash #1 FAILED");
	if (!aminoStirAndDrain("after wash #1"))
		return sequenceAbort("Stir/drain FAILED (after wash #1)");

	/* ---- 7. wash(), then stir, drain. ---- */
	if (!wash()) return sequenceAbort("Wash #2 FAILED");
	if (!aminoStirAndDrain("after wash #2"))
		return sequenceAbort("Stir/drain FAILED (after wash #2)");

	/* ---- 8. Carousel -36, deprotection solvent (5 g), stir, drain. ---- */
	aminoCarouselIndex36("deprotection solvent");

	//if (!aminoDoseAndPour(AMINO_SEQ_LIQUID_TARGET_RAW, "deprotection solvent"))
		//return sequenceAbort("Liquid dose/pour FAILED (deprotection solvent)");

	if (!aminoStirAndDrain("after final liquid dispense"))
		return sequenceAbort("Stir/drain FAILED (after final liquid dispense)");

	/* ---- 9. wash(), then stir, drain. ---- */
	if (!wash()) return sequenceAbort("Wash #3 FAILED");
	if (!aminoStirAndDrain("after wash #3"))
		return sequenceAbort("Stir/drain FAILED (after wash #3)");

	/* ---- 10. wash(), then stir, drain. ---- */
	if (!wash()) return sequenceAbort("Wash #4 FAILED");
	if (!aminoStirAndDrain("after wash #4"))
		return sequenceAbort("Stir/drain FAILED (after wash #4)");

	/* ---- Park: make sure no dose task is still alive, then rail home. ---- */
	(void) aminoDoseTaskWaitFree();

	UART_Print("[SEQ] Returning rail to home...\r\n");
	servoBusLock();
	(void) robotLinearGotoHardcodedHome();
	servoBusUnlock();
	aminoRailTurns = AMINO_RAIL_HOME;

	/* Rewind the carousel too, so the machine is left in the state the next run
	 * expects. The home at the TOP of the run is the one that actually
	 * guarantees it (this one is skipped on any abort path), but leaving the
	 * ring parked mid-way between runs is worth avoiding on its own. */
	aminoCarouselHome("run end");

	UART_Print("\r\n########## AMINO ACID ADDITION SEQUENCE COMPLETE ##########\r\n");
	return true;
}

/* ---------------------------------------------------------------------------
 *  OCTERTOIDE SEQUENCE
 *
 *  Stage order:
 *
 *  BLOCK 1 -- runs once
 *    1  rail -> powder lane, runPowderDispenserModule(), hold, resin notice
 *    2  POWDER + WASH
 *    3  stir + restir + drain
 *    4  POWDER + WASH
 *    5  stir + restir + drain
 *    6  PROXIMITY DISPENSE (amino acid selection)
 *    7  POWDER + WASH
 *    8  stir + restir + drain
 *    9  POWDER + WASH
 *   10  stir + restir + drain
 *   11  POWDER + WASH
 *   12  stir + restir + drain
 *   13  POWDER + WASH
 *
 *  BLOCK 2 -- runs OCT_BLOCK2_CYCLES (7) times
 *    a  PROXIMITY DISPENSE (amino acid selection)
 *    b  POWDER + WASH
 *    c  stir + restir + drain
 *    d  POWDER + WASH
 *    e  stir + restir + drain
 *    f  POWDER + WASH
 *
 *  So a full run is block 1 once, then block 2 seven times: 6 + 21 = 27
 *  powder+wash stages and 8 amino-acid doses in total.
 *
 *  Every dose is the INTERACTIVE amino_acid_selection() -- the operator picks
 *  the syringe and the weight each time, and it doses through the proven
 *  load-cell path (aminoDoseAndPour): the stepper rotates until the entered
 *  weight is collected, in parallel with the arm's pour approach. This is what
 *  makes repeating block 2 useful: each cycle couples the next residue.
 *
 *  A POWDER + WASH stage (octertoidePowderWash) is one arm trip with the
 *  selection nested INSIDE it, not two things in a row:
 *
 *      rail -> +30 lane
 *      HOME -> POWDER 1 -> POWDER 2
 *                             `-- wash_solvent_selection() runs here, with the
 *                                 arm holding the pose: operator prompt, plate
 *                                 home, slot move, pump ON / motor ON / 30 s /
 *                                 motor OFF / pump OFF
 *      POWDER 1 -> HOME
 *
 *  So the arm never travels while an output is energised, and cannot leave
 *  POWDER 2 with the motor still running -- wash_solvent_selection() drives
 *  both outputs back off before it returns.
 *
 *  Stage 1 uses the plain runPowderDispenserModule(), unchanged, with no wash
 *  nested in it.
 *
 *  OPERATOR PROMPTS: one wash selection per powder+wash stage (27) plus one
 *  amino-acid selection per dose (8). Each blocks on the console until it is
 *  answered, so the machine WILL sit and wait -- with the arm parked at
 *  POWDER 2 during the wash ones. A full run cannot be left unattended.
 *
 *  REUSED PATHS. Nothing here re-implements motion that already works:
 *      amino dose    -> amino_acid_selection() -> aminoDoseAndPour() ->
 *                       AminoDoseTask -> aminoPlungeAndDose() ->
 *                       LoadCellStepperDispenseRamped(), the same weight-metered
 *                       dose sequence() and amino_acid_addition_sequence() use
 *      stir group    -> runReactionStirModule() + runReactionReStirModule() +
 *                       stirDrainPulse(), the same stir/RESTIR/drain order
 *                       aminoStirAndDrain() uses, plus a mixer park at
 *                       REACTION_HOME_PULSES first
 *
 *  RAIL DISCIPLINE. Every module with taught poses is preceded by the rail move
 *  that puts the deck under them:
 *      runPowderDispenserModule()   <- aminoRailToLiquid() (+30), stage 1
 *      powder+wash arm trip         <- aminoRailToLiquid() (+30), each stage
 *      runLiquidDispenser*()        <- aminoRailToLiquid() (+30), issued by
 *                                      aminoDoseAndPour() at its own top
 *      runReactionStirModule()      <- aminoRailToStir()   (+100), issued by
 *      runReactionReStirModule()       octertoideStir(), which returns the
 *                                      rail to +30 afterwards
 * ------------------------------------------------------------------------- */

#define OCT_POWDER_HOLD_MS      2000u   /* hold at the powder station */

/* How many times block 2 runs. Block 1 runs once, then this many cycles of
 * block 2 -- i.e. once through, then another six. Each cycle prompts for its
 * own amino acid and weight, so a run couples seven residues. */
#define OCT_BLOCK2_CYCLES             7

/* Stir group exactly as asked: runReactionStirModule() then
 * runReactionReStirModule(), with the rail shuttled to the stir lane and back.
 *
 * The rail moves are NOT optional padding. This file's own history note records
 * that every runReactionStirModule() call in amino_acid_addition_sequence()
 * used to fire with the rail still parked in the liquid lane, so the arm walked
 * the stir poses without the rail ever reaching the stir station.
 * aminoRailToStir()/aminoRailToLiquid() are what sequence() does around every
 * stir, so they are what this does too.
 *
 * This is aminoStirAndDrain() MINUS the drain pulse: the requested step is
 * "stir then restir" and nothing in it asked for the vessel to be drained. If a
 * drain does belong after each stir, swap these calls for aminoStirAndDrain(). */
static bool octertoideStir(const char *why)
{
	/* ---- Mixer servo to its fixed home FIRST ----
	 * REACTION_HOME_PULSES (2168) is an ABSOLUTE encoder count, so this parks
	 * the reaction vessel in the same orientation before every stir regardless
	 * of where the previous stir's swings left it. Doing it here, before the
	 * rail moves and before STIR8 can launch the parallel reaction, means the
	 * stir always starts from a known datum.
	 *
	 * ReactionMoveHome() talks to MIXER_ID over the shared USART6 bus and does
	 * not take the lock itself (the lock is static to this file), so it is
	 * wrapped here like every other bus call. */
	UART_Print("[OCT] Mixer -> reaction home (%d pulses) before stir (%s)...\r\n",
	           REACTION_HOME_PULSES, why);

	servoBusLock();
	bool mixerHomed = ReactionMoveHome(servo);
	servoBusUnlock();

	if (!mixerHomed) {
		UART_Print("[OCT] Mixer home FAILED (%s).\r\n", why);
		return false;
	}

	if (!aminoRailToStir(why)) {
		return false;
	}

	reactionAtStir8Enabled = true;
	bool stirOk = runReactionStirModule();
	reactionAtStir8Enabled = false;

	if (!stirOk) {
		UART_Print("[OCT] Stir FAILED (%s).\r\n", why);
		return false;
	}

	/* RESTIR is what puts the gripper back to SET and the pump back to RESET --
	 * the stir pass leaves them the other way round. Skipping it would leave
	 * every move that follows happening with the vessel un-gripped. */
	if (!runReactionReStirModule()) {
		UART_Print("[OCT] ReStir FAILED (%s).\r\n", why);
		return false;
	}

	/* Drain pulse, completing the stir group exactly as the sequence runs it:
	 * stir -> RESTIR -> drain, the AMINO_RESTIR_BEFORE_DRAIN = 1 order that
	 * aminoStirAndDrain() uses. */
	stirDrainPulse(why);

	/* Let the idle task reclaim the reaction task's static TCB before the next
	 * stir's STIR8 recreates it -- the same guard aminoStirAndDrain() uses. */
	osDelay(100);

	return aminoRailToLiquid(why);
}

/* ---------------------------------------------------------------------------
 *  PARALLEL STEPPER HOMING
 *
 *  THIS IS THE FIX FOR "the robot does not move to the powder pose until the
 *  stepper reaches its limit".
 *
 *  drainStepperLimit() has nothing to do with the FR5 arm: it drives the
 *  stepper GPIO, the arm talks over TCP and the rail over USART6, so no two of
 *  them contend. Calling it inline in the preamble simply made the arm sit
 *  still for the whole plunger travel before it would even start toward the
 *  powder poses. It now runs on its own thread while the rail moves and the
 *  powder module walks its poses.
 *
 *  Static allocation, the same pattern DispenseTask / ReactionTask /
 *  AminoDoseTask use in this file: configTOTAL_HEAP_SIZE is only 15360 bytes
 *  and LWIP has taken most of it, so the stack and TCB live in .bss instead.
 * ------------------------------------------------------------------------- */
#define OCT_STEPPER_TASK_STACK_WORDS   1024   /* was 512; a UART_Print alone is
                                               * a 512-byte stack buffer */

static StackType_t   xOctStepperTaskStack[OCT_STEPPER_TASK_STACK_WORDS];
static StaticTask_t  xOctStepperTaskTCB;
static osThreadId    octStepperTaskHandle  = NULL;
static volatile bool octStepperTaskRunning = false;

static void OctStepperHomeTask(void const *argument)
{
	(void) argument;

	aminoStepperToLimit("octertoide run start (parallel)");

	/* Cleared last, so the join below cannot see "finished" early. */
	octStepperTaskRunning = false;

	osThreadTerminate(osThreadGetId());
}

/* Start the plunger homing and return immediately, so the caller can get the
 * arm moving. */
static void octertoideStartStepperHome(void)
{
	if (octStepperTaskRunning) {
		UART_Print("[OCT] Stepper homing already in progress.\r\n");
		return;
	}

	UART_Print("[OCT] Stepper -> upper limit, in PARALLEL with the arm...\r\n");

	/* Set before the task exists: if it were set inside the task, a join
	 * running first would see false and conclude the homing had finished. */
	octStepperTaskRunning = true;

	octStepperTaskHandle = (osThreadId) xTaskCreateStatic(
			(TaskFunction_t) OctStepperHomeTask,
			"octStepperTask",
			OCT_STEPPER_TASK_STACK_WORDS,
			NULL,
			(UBaseType_t) (tskIDLE_PRIORITY + 3),
			xOctStepperTaskStack,
			&xOctStepperTaskTCB);

	if (octStepperTaskHandle == NULL) {
		UART_Print("[OCT] Could not start the stepper task -- homing inline.\r\n");

		octStepperTaskRunning = false;
		aminoStepperToLimit("octertoide run start (inline fallback)");
	}
}

/* Wait for the plunger to be clear.
 *
 * MUST be called before anything rotates the plate. That interlock is not
 * cosmetic: aminoCarouselIndex36()/aminoCarouselIndex18() both home the stepper
 * before indexing precisely because turning the carousel with the plunger down
 * drives it into the syringes. */
static bool octertoideJoinStepperHome(uint32_t timeout_ms)
{
	uint32_t start = HAL_GetTick();

	while (octStepperTaskRunning) {
		if ((HAL_GetTick() - start) > timeout_ms) {
			UART_Print("[OCT] Stepper homing did not finish within %lu ms.\r\n",
			           (unsigned long) timeout_ms);
			return false;
		}

		osDelay(20);
	}

	osDelay(100);                  /* let the idle task reclaim the TCB */
	octStepperTaskHandle = NULL;

	UART_Print("[OCT] Stepper homing joined -- plunger clear.\r\n");

	return true;
}

/* Robot link up and armed, with NO operator input -- the same two steps the
 * ROBOT MENU performs as "1. Connect to Robot" and "2. Initialize Robot", so an
 * octertoide run does not have to be preceded by a trip through that menu.
 *
 * connectRobot() is idempotent (returns immediately if the socket is already
 * up) and runs robotInitConfig() internally, so the explicit init below is a
 * re-assert of mode/enable/speed rather than first-time setup. It is kept
 * because it is what the menu does, and because it catches an arm that was
 * connected earlier in the session but has since been disabled or faulted --
 * in which case the connect branch is skipped and this is the only thing that
 * would put it back in a usable state. */
static bool octertoideRobotBringUp(void)
{
	if (robotSocket < 0) {
		UART_Print("[OCT] Connecting to robot...\r\n");

		if (connectRobot() != 0) {
			UART_Print("[OCT] Robot connection FAILED.\r\n");
			return false;
		}

		UART_Print("[OCT] Robot connected.\r\n");
	} else {
		UART_Print("[OCT] Robot already connected.\r\n");
	}

	UART_Print("[OCT] Initializing robot...\r\n");

	if (robotInitConfig() != 0) {
		UART_Print("[OCT] Robot initialization FAILED -- see message above.\r\n");
		return false;
	}

	UART_Print("[OCT] Robot initialization complete.\r\n");

	return true;
}

/* Declare wherever the MicroID plate is standing RIGHT NOW to be home for this
 * run, and datum every tracker to it. Called once at the top of the run, before
 * the servo has rotated at all -- so "home" is literally the position the servo
 * started from.
 *
 * ServoHomeFixedCapture() reads the live encoder count and latches it; every
 * later DispenserMoveHome() drives back to exactly that count. The two
 * assignments bring THIS file's angle tracker and the odd/even syringe slot to
 * the same datum, which is what aminoCarouselHome() does once the ring is
 * actually there. */
static void octertoideDeclareDispenserHome(void)
{
	UART_Print("[OCT] Declaring the dispenser's start position as home...\r\n");

	servoBusLock();
	ServoHomeFixedCapture(servo);
	servoBusUnlock();

	aminoCarouselDeg = 0.0f;
	aminoSlot        = 1u;
}

/* MicroID plate -> the home latched above, then re-datum both angle trackers.
 *
 * The latched value is an absolute ENCODER COUNT, not a tracked angle, so this
 * returns to exactly the position the run started from however many slot moves
 * have happened in between -- no accumulated rounding, no drift. Every stage
 * that turns the plate is preceded by this, so no stage inherits the previous
 * one's position.
 *
 * DispenserMoveHome() re-datums servo.c's tracker to 0 deg itself; the two
 * assignments here keep this file's trackers in step. */
static bool octertoideDispenserHome(const char *why)
{
	UART_Print("[OCT] Dispenser -> run home -- %s...\r\n", why);

	servoBusLock();
	bool ok = DispenserMoveHome(servo);
	servoBusUnlock();

	if (!ok) {
		UART_Print("[OCT] Dispenser home FAILED (%s).\r\n", why);
		return false;
	}

	aminoCarouselDeg = 0.0f;
	aminoSlot        = 1u;

	return true;
}

/* ---------------------------------------------------------------------------
 *  POWDER DISPENSE WITH THE WASH SELECTION AT POSITION 2
 *
 *  A SEPARATE function from runPowderDispenserModule(), which is untouched and
 *  still walks HOME -> P1 -> P2 -> P2 -> P1 -> HOME driving nothing.
 *
 *  This one stops at POWDER 2 and HOLDS the arm there for the whole wash
 *  solvent selection -- operator prompt, plate home, slot move and the
 *  pump/DC-motor on-off cycle all happen with the arm parked on that pose.
 *  Only once wash_solvent_selection() has returned (outputs back off) does the
 *  arm continue to POWDER 1 and HOME.
 *
 *      HOME -> POWDER 1 -> POWDER 2 -> [wash_solvent_selection()]
 *                                   -> POWDER 1 -> HOME
 *
 *  wash_solvent_selection() is what drives the outputs: per solvent it does
 *  DispenserMoveHome() -> wash_solvents() -> pump ON -> motor ON -> 30 s ->
 *  motor OFF -> pump OFF. So "wait for the motor to be off before leaving P2"
 *  is satisfied by construction -- that call cannot return with either output
 *  still energised.
 * ------------------------------------------------------------------------- */
static bool octertoidePowderWash(int n)
{
	/* Indices into powderPoses[]: 0 = HOME, 1 = POWDER 1, 2 = POWDER 2.
	 * Only those three are populated -- POSE_POWDER3/4 are commented out in
	 * the pose table, so 3 and 4 must not be used. */
	static const int   outSeq[]      = {      0,         1,          2     };
	static const char *outSeqName[]  = {   "HOME", "POWDER 1", "POWDER 2"  };
	static const int   backSeq[]     = {      1,         0                 };
	static const char *backSeqName[] = { "POWDER 1",  "HOME"               };

	bool washOk;

	UART_Print("\r\n[OCT] ===== Powder + wash stage #%d : "
	           "home -> P1 -> P2 -> wash selection -> P1 -> home =====\r\n", n);

	/* Rail into the lane the powder poses were taught in (+30) before the arm
	 * walks them. Normally a no-op -- octertoideStir() ends by returning the
	 * rail here and aminoDoseAndPour() opens by doing the same -- but it makes
	 * the stage self-sufficient wherever it is called from. */
	if (!aminoRailToLiquid("before powder + wash selection")) {
		UART_Print("[OCT] Rail move to the powder lane FAILED (stage #%d).\r\n", n);
		return false;
	}

	if (connectRobot() != 0) {
		UART_Print("[OCT] Robot not connected -- powder stage #%d aborted.\r\n", n);
		return false;
	}

	if (robotInitConfig() != 0) {
		UART_Print("[OCT] Robot NOT READY -- powder stage #%d aborted before any "
		           "motion.\r\n   Clear faults + switch to AUTOMATIC on the "
		           "pendant/WebApp, then retry.\r\n", n);
		return false;
	}

	/* ---- Out to POWDER 2. A failure here returns before the wash runs, so
	 * no output is ever left energised. ---- */
	for (int i = 0; i < (int) (sizeof(outSeq) / sizeof(outSeq[0])); i++) {
		UART_Print("\r\n[OCT] ---- %s ----\r\n", outSeqName[i]);

		if (robotMoveJStr(powderPoses[outSeq[i]]) != 0) {
			UART_Print("[OCT] %s FAILED -- powder stage #%d ABORTED.\r\n",
			           outSeqName[i], n);
			return false;
		}

		osDelay(MODULE_STEP_DELAY_MS);
	}

	/* ---- Arm holds POWDER 2 for the whole selection. ---- */
	UART_Print("\r\n[OCT] At POWDER 2 -- running wash solvent selection #%d "
	           "(arm holds this pose)...\r\n", n);

	washOk = wash_solvent_selection();

	UART_Print("[OCT] Wash selection #%d finished -- outputs off, arm may leave "
	           "POWDER 2.\r\n", n);

	/* ---- Back to POWDER 1 and HOME. This runs even if the selection failed,
	 * so an aborted wash never strands the arm out at POWDER 2. ---- */
	for (int i = 0; i < (int) (sizeof(backSeq) / sizeof(backSeq[0])); i++) {
		UART_Print("\r\n[OCT] ---- %s ----\r\n", backSeqName[i]);

		if (robotMoveJStr(powderPoses[backSeq[i]]) != 0) {
			UART_Print("[OCT] %s FAILED -- powder stage #%d ABORTED.\r\n",
			           backSeqName[i], n);
			return false;
		}

		osDelay(MODULE_STEP_DELAY_MS);
	}

	if (!washOk) {
		UART_Print("[OCT] Wash solvent selection #%d FAILED (arm returned "
		           "home).\r\n", n);
		return false;
	}

	UART_Print("\r\n[OCT] Powder + wash stage #%d COMPLETE.\r\n", n);

	return true;
}

bool octertoide_sequence(void)
{
	/* Running count of powder+wash stages, so the console log numbers them
	 * continuously across block 1 and all of block 2's cycles instead of
	 * restarting at 1 each time round the loop. */
	int washStage = 0;

	UART_Print("\r\n########## OCTERTOIDE SEQUENCE START ##########\r\n");

	/* ---- Robot link FIRST. Every arm move below -- the powder module, all
	 * four stir/restir groups, the liquid approach inside the 5 g dose --
	 * needs the FR5 socket up and the arm enabled. Doing it here means the run
	 * is self-sufficient: no visit to the ROBOT MENU beforehand, no operator
	 * input to activate the arm. ---- */
	if (!octertoideRobotBringUp()) {
		return sequenceAbort("Robot bring-up FAILED");
	}

	/* ---- Run preamble, the same one amino_acid_addition_sequence() opens with.
	 * None of it is boilerplate:
	 *   - sequenceTaskHandle is how AminoDoseTask signals the dose back, so
	 *     without it the 5 g dispense at step 5 would wait forever;
	 *   - the carousel and rail datums are what make every later relative move
	 *     land in the right place, and an aborted previous run can easily have
	 *     left both somewhere else. ---- */
	UART_Print("[OCT] Robot gripper ON (held for the run).\r\n");
	HAL_GPIO_WritePin(Robot_gripper_GPIO_Port, Robot_gripper_Pin, GPIO_PIN_SET);

	servoBusUnlock();                       /* a prior abort may have held it */
	sequenceTaskHandle = osThreadGetId();

	(void) osSignalWait(SIG_WEIGHT_READY, 0);   /* drain stale signals */
	(void) osSignalWait(SIG_ROBOT_READY,  0);
	(void) osSignalWait(SIG_DRAIN_DONE,   0);

	/* ---- HOME IS DECLARED HERE, NOT DRIVEN TO ----
	 * The plate has not been turned yet this run, so wherever it is standing IS
	 * home: latch that encoder count and datum every tracker to it. Every later
	 * octertoideDispenserHome() -- and every DispenserMoveHome() inside
	 * wash_solvent_selection() -- returns to exactly this position.
	 *
	 * Re-declaring per run rather than relying on the previous run's latch is
	 * deliberate: an aborted run can leave the plate anywhere, and this way the
	 * run's own start position is always its datum. */
	octertoideDeclareDispenserHome();

	aminoCarouselHome("octertoide run start");

	/* ---- Plunger homing STARTS HERE and runs on its own thread ----
	 * Everything below -- the rail datum, the rail move to the powder lane and
	 * the whole powder module -- now overlaps it instead of queueing behind it.
	 * Joined further down, before the first thing that turns the plate. */
	octertoideStartStepperHome();

	UART_Print("[OCT] Rail -> hardcoded home (establishing the run datum)...\r\n");
	servoBusLock();
	bool railHomed = robotLinearGotoHardcodedHome();
	servoBusUnlock();

	if (!railHomed) {
		return sequenceAbort("Rail home FAILED (cannot establish datum)");
	}

	aminoRailTurns = AMINO_RAIL_HOME;
	osDelay(300);

	/* ---- 1. Rail +30 turns out to the powder station, THEN the powder
	 * dispenser module, then hold 30 s.
	 *
	 * The rail move comes FIRST because that is the order sequence() uses:
	 * MoveLinearServo(SEQ_POWDER_TURNS) and only then
	 * runPowderDispenserModule(). The powder poses were taught with the rail at
	 * +30, so running the module straight off the rail datum would walk the arm
	 * through them with the deck 30 turns away.
	 *
	 * aminoRailToLiquid() IS that +30 -- AMINO_RAIL_LIQUID is defined as
	 * SEQ_POWDER_TURNS and the lane is the shared powder/liquid station -- but
	 * expressed as an absolute target rather than a relative step, so it stays
	 * correct whatever the rail did before. From the datum established above
	 * the delta it issues is exactly +30 turns.
	 *
	 * It also leaves the rail where the stages after the powder module expect
	 * it, so no second move is needed once the hold expires. ---- */
	UART_Print("\r\n[OCT] ===== Step 1: rail -> powder station, then powder module =====\r\n");

	if (!aminoRailToLiquid("before powder")) {
		return sequenceAbort("Rail move to the powder station FAILED");
	}

	if (!runPowderDispenserModule()) {
		return sequenceAbort("Powder dispenser module FAILED");
	}

	/* ---- 2. Hold at the powder station. ---- */
	UART_Print("[OCT] Holding at the powder station for %lu ms...\r\n",
	           (unsigned long) OCT_POWDER_HOLD_MS);
	osDelay(OCT_POWDER_HOLD_MS);

	/* ---- 3. Resin powder notice. Console only -- nothing here weighs or
	 * meters the resin; the 20 g figure is what the powder station is set up
	 * to deliver during the hold above. ---- */
	UART_Print("\r\n[OCT] 20 gm resin powder is getting dispensed.\r\n");

	/* ---- Join the parallel plunger homing ----
	 * Last safe moment: wash stage 1 below turns the plate (home, then out to
	 * the solvent slot), and the plunger must be clear of the syringes before
	 * anything rotates. By now it has had the rail moves, the whole powder
	 * module and the 50 s hold to finish in, so this is normally instant. */
	if (!octertoideJoinStepperHome(STEPPER_LIMIT_WAIT_MS)) {
		return sequenceAbort("Stepper homing did not finish (plunger not clear)");
	}

	/* ---- 2. Powder + wash stage 1 (selection happens at POWDER 2). ---- */
	if (!octertoidePowderWash(++washStage)) {
		return sequenceAbort("Powder + wash stage #1 FAILED");
	}

	/* ---- 3. Stir + restir + drain. ---- */
	if (!octertoideStir("after powder/wash #1")) {
		return sequenceAbort("Stir/restir #1 FAILED");
	}

	/* ---- 4. Powder + wash stage 2. ---- */
	if (!octertoidePowderWash(++washStage)) {
		return sequenceAbort("Powder + wash stage #2 FAILED");
	}

	/* ---- 5. Stir + restir + drain. ---- */
	if (!octertoideStir("after powder/wash #2")) {
		return sequenceAbort("Stir/restir #2 FAILED");
	}

	/* ---- 6. Amino acid selection + load-cell dose. ----
	 * The operator picks the syringe and the weight; amino_acid_selection()
	 * homes the plate, drives to that syringe (36 deg per syringe,
	 * anticlockwise) and doses it through the load-cell path (aminoDoseAndPour):
	 * the stepper rotates until the entered weight is collected. ---- */
	UART_Print("\r\n[OCT] ===== Step 6: amino acid selection + dose =====\r\n");

	if (!amino_acid_selection()) {
		return sequenceAbort("Amino acid selection/dose FAILED (block 1)");
	}

	/* ---- 7. Powder + wash stage 3. ---- */
	if (!octertoidePowderWash(++washStage)) {
		return sequenceAbort("Powder + wash stage #3 FAILED");
	}

	/* ---- 8. Stir + restir + drain. ---- */
	if (!octertoideStir("after powder/wash #3")) {
		return sequenceAbort("Stir/restir #3 FAILED");
	}

	/* ---- 9. Powder + wash stage 4. ---- */
	if (!octertoidePowderWash(++washStage)) {
		return sequenceAbort("Powder + wash stage #4 FAILED");
	}

	/* ---- 10. Stir + restir + drain. ---- */
	if (!octertoideStir("after powder/wash #4")) {
		return sequenceAbort("Stir/restir #4 FAILED");
	}

	/* ---- 11. Powder + wash stage 5. ---- */
	if (!octertoidePowderWash(++washStage)) {
		return sequenceAbort("Powder + wash stage #5 FAILED");
	}

	/* ---- 12. Stir + restir + drain. ---- */
	if (!octertoideStir("after powder/wash #5")) {
		return sequenceAbort("Stir/restir #5 FAILED");
	}

	/* ---- 13. Final powder + wash stage of block 1. ---- */
	if (!octertoidePowderWash(++washStage)) {
		return sequenceAbort("Powder + wash stage #6 FAILED");
	}

	/* =====================================================================
	 *  BLOCK 2, REPEATED OCT_BLOCK2_CYCLES TIMES
	 *
	 *  One cycle is:
	 *      amino acid selection + load-cell dose
	 *      POWDER + WASH
	 *      stir + restir + drain
	 *      POWDER + WASH
	 *      stir + restir + drain
	 *      POWDER + WASH
	 *
	 *  Block 1 runs once, then this runs 7 times -- i.e. once, then another 6.
	 *  Every cycle asks for its own amino acid and weight, which is what makes
	 *  repeating it useful: each pass adds the next residue in the chain.
	 *
	 *  Same building blocks as block 1: octertoidePowderWash() nests the
	 *  operator selection inside the arm trip at POWDER 2, and octertoideStir()
	 *  does mixer-park + rail + stir + restir + drain.
	 *
	 *  washStage just keeps the console log's stage numbers running across the
	 *  whole run instead of restarting at 1 every cycle.
	 * ===================================================================== */
	for (int cycle = 1; cycle <= OCT_BLOCK2_CYCLES; cycle++) {

		UART_Print("\r\n##### BLOCK 2 -- CYCLE %d of %d #####\r\n",
		           cycle, OCT_BLOCK2_CYCLES);

		/* ---- Amino acid selection + load-cell dose. ----
		 * amino_acid_selection() homes the plate to the run datum first, so the
		 * syringe angle is absolute rather than relative to wherever the
		 * previous cycle's last wash left the plate. */
		if (!amino_acid_selection()) {
			UART_Print("[OCT] Amino acid selection/dose FAILED "
			           "(block 2, cycle %d).\r\n", cycle);
			return sequenceAbort("Amino acid selection/dose FAILED (block 2)");
		}

		if (!octertoidePowderWash(++washStage)) {
			return sequenceAbort("Powder + wash stage FAILED (block 2)");
		}

		if (!octertoideStir("block 2, first stir")) {
			return sequenceAbort("Stir/restir FAILED (block 2, first)");
		}

		if (!octertoidePowderWash(++washStage)) {
			return sequenceAbort("Powder + wash stage FAILED (block 2)");
		}

		if (!octertoideStir("block 2, second stir")) {
			return sequenceAbort("Stir/restir FAILED (block 2, second)");
		}

		if (!octertoidePowderWash(++washStage)) {
			return sequenceAbort("Powder + wash stage FAILED (block 2)");
		}

		UART_Print("\r\n##### BLOCK 2 -- CYCLE %d of %d COMPLETE #####\r\n",
		           cycle, OCT_BLOCK2_CYCLES);
	}

	/* ---- Park: make sure no dose task is still alive, then rail + carousel
	 * back to their datums so the next run starts from a known state. ---- */
	(void) aminoDoseTaskWaitFree();

	UART_Print("[OCT] Returning rail to home...\r\n");
	servoBusLock();
	(void) robotLinearGotoHardcodedHome();
	servoBusUnlock();
	aminoRailTurns = AMINO_RAIL_HOME;

	/* Leave the plate on the hardcoded datum, not merely on the tracker's idea
	 * of it, so the next run starts from a genuinely known position. */
	(void) octertoideDispenserHome("run end");

	UART_Print("\r\n########## OCTERTOIDE SEQUENCE COMPLETE ##########\r\n");

	return true;
}

/* ---------------------------------------------------------------------------
 *  CLEAVAGE MODULE  --  interactive console routine
 *
 *  1. Pneumatic 3 ON, hold CLEAVE_PNEU3_MS, OFF.
 *  2. Then the wash-menu flow, with the four cleavage cocktail solvents:
 *     ask how many, then per selection ask WHICH solvent -- and that solvent's
 *     valve goes ON and STAYS ON until the operator types "off".
 *
 *  Solvent -> valve mapping (as specified):
 *      TFA -> Pneumatic 1,  EDT -> Pneumatic 2,
 *      TIS -> Pneumatic 4,  H2O -> Pneumatic 5.
 *
 *  NO LOAD CELL and no fill timer here, by request: the open time is whatever
 *  the operator wants it to be, and only an "off" closes the valve. The one
 *  thing that closes it without an "off" is the console going quiet -- see
 *  cleaveHoldValve() -- so a dead terminal cannot leave a solvent valve open.
 * ------------------------------------------------------------------------- */

#define CLEAVE_PNEU3_MS          5000u   /* Pneumatic 3 hold, as specified   */
#define CLEAVE_SETTLE_MS          500u   /* gap between output changes       */

typedef enum {
	CLEAVE_TFA = 0,
	CLEAVE_EDT,
	CLEAVE_TIS,
	CLEAVE_H2O,
	CLEAVE_SOLVENT_COUNT
} CleavageSolvent_t;

/* Solvent -> valve -> motor. The motor pairing is deliberately NOT in numeric
 * order; it is the wiring as specified:
 *      Pneumatic 1 (TFA) -> Cleavage solvent motor 2
 *      Pneumatic 2 (EDT) -> Cleavage solvent motor 4
 *      Pneumatic 4 (TIS) -> Cleavage solvent motor 1
 *      Pneumatic 5 (H2O) -> Cleavage solvent motor 3
 * Only cleavageModuleSequence() runs the motors; clevagemodule() ignores these
 * three fields and drives the valve alone, exactly as before. */
static const struct {
	const char   *name;
	const char   *valve;
	GPIO_TypeDef *port;
	uint16_t      pin;
	const char   *motorName;
	GPIO_TypeDef *motorPort;
	uint16_t      motorPin;
} cleavageSolvents[CLEAVE_SOLVENT_COUNT] = {
	[CLEAVE_TFA] = { "TFA", "Pneumatic 1", Pnuematic_1_GPIO_Port, Pnuematic_1_Pin,
	                 "Solvent motor 2", Clevage_solvent_motor_2_GPIO_Port,
	                 Clevage_solvent_motor_2_Pin },
	[CLEAVE_EDT] = { "EDT", "Pneumatic 2", Pnuematic_2_GPIO_Port, Pnuematic_2_Pin,
	                 "Solvent motor 4", Clevage_solvent_motor_4_GPIO_Port,
	                 Clevage_solvent_motor_4_Pin },
	[CLEAVE_TIS] = { "TIS", "Pneumatic 4", Pnuematic_4_GPIO_Port, Pnuematic_4_Pin,
	                 "Solvent motor 1", Clevage_solvent_motor_1_GPIO_Port,
	                 Clevage_solvent_motor_1_Pin },
	[CLEAVE_H2O] = { "H2O", "Pneumatic 5", Pnuematic_5_GPIO_Port, Pnuematic_5_Pin,
	                 "Solvent motor 3", Clevage_solvent_motor_3_GPIO_Port,
	                 Clevage_solvent_motor_3_Pin },
};

static void cleavePrintList(void)
{
	UART_Print("\r\n  Available cleavage solvents:\r\n");

	for (int i = 0; i < CLEAVE_SOLVENT_COUNT; i++) {
		UART_Print("    %d) %-4s  (%s)\r\n", i + 1,
		           cleavageSolvents[i].name, cleavageSolvents[i].valve);
	}
}

/* Number (1..4) or the name, case-insensitively. Same blank-line-tolerant
 * reader the wash menu uses, so a stray LF from a CRLF terminal cannot burn an
 * attempt. */
static bool cleaveAskSolvent(CleavageSolvent_t *out)
{
	for (int attempt = 0; attempt < WASH_SEL_MAX_TRIES; attempt++) {
		char     buf[WASH_SEL_INPUT_LEN];
		char    *text;
		uint32_t number;

		cleavePrintList();
		UART_Print("  Solvent (number or name) : ");

		if (!washSelReadLine(buf, sizeof(buf), &text)) {
			UART_Print("[CLEAVE] No input received -- try again.\r\n");
			continue;
		}

		if (washSelParseUInt(text, &number)) {
			if (number >= 1u && number <= (uint32_t) CLEAVE_SOLVENT_COUNT) {
				*out = (CleavageSolvent_t) (number - 1u);

				return true;
			}

			UART_Print("[CLEAVE] %lu is out of range (1..%d).\r\n",
			           (unsigned long) number, CLEAVE_SOLVENT_COUNT);

			continue;
		}

		for (int i = 0; i < CLEAVE_SOLVENT_COUNT; i++) {
			if (washSelNamesEqual(text, cleavageSolvents[i].name)) {
				*out = (CleavageSolvent_t) i;

				return true;
			}
		}

		UART_Print("[CLEAVE] '%s' is not a cleavage solvent.\r\n", text);
	}

	UART_Print("[CLEAVE] Too many bad entries -- aborting.\r\n");

	return false;
}

/* Opens one solvent valve and holds it open until the operator types "off".
 *
 * No load cell, no timer: the valve stays on for exactly as long as the console
 * leaves it on. The only thing that closes it on its own is the console going
 * quiet -- if the prompt comes back blank WASH_SEL_BLANK_SKIP times in a row
 * (terminal unplugged, line noise) the valve is closed rather than left open
 * with nobody able to answer. */
/* Switches one output ON and holds it there until the operator types "off".
 * Shared by the solvent valves and the cleavage solvent motors so both are
 * controlled the same way. Returns false only if the console went quiet, in
 * which case the output is switched off anyway. */
static bool cleaveHoldPin(GPIO_TypeDef *port, uint16_t pin,
                          const char *what, const char *outputName)
{
	bool offByOperator = false;

	HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);

	UART_Print("[CLEAVE] %s : %s ON.\r\n", what, outputName);

	for (;;) {
		char  buf[WASH_SEL_INPUT_LEN];
		char *text;

		UART_Print("  %s is ON -- type 'off' to switch it off : ", outputName);

		if (!washSelReadLine(buf, sizeof(buf), &text)) {
			UART_Print("\r\n[CLEAVE] No input -- switching %s off for "
			           "safety.\r\n", outputName);
			break;
		}

		/* "off" or a plain 0, either case. Anything else re-prompts: the
		 * output must not switch off on a typo. */
		if (washSelNamesEqual(text, "off") || washSelNamesEqual(text, "0")) {
			offByOperator = true;
			break;
		}

		UART_Print("[CLEAVE] '%s' is not 'off' -- %s stays ON.\r\n",
		           text, outputName);
	}

	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);

	UART_Print("[CLEAVE] %s : %s OFF.\r\n", what, outputName);

	osDelay(CLEAVE_SETTLE_MS);

	return offByOperator;
}

static bool cleaveHoldValve(CleavageSolvent_t which)
{
	return cleaveHoldPin(cleavageSolvents[which].port,
	                     cleavageSolvents[which].pin,
	                     cleavageSolvents[which].name,
	                     cleavageSolvents[which].valve);
}

/* The solvent's valve AND its paired motor, switched on TOGETHER and held until
 * the operator types "off", then both switched off together.
 *
 * They run at the same time deliberately: the motor is what moves the solvent
 * while the valve is open, so starting it only after the valve shut (as an
 * earlier version did) meant it ran against a closed valve. */
static bool cleaveHoldValveAndMotor(CleavageSolvent_t which)
{
	const char *name  = cleavageSolvents[which].name;
	const char *valve = cleavageSolvents[which].valve;
	const char *motor = cleavageSolvents[which].motorName;
	bool        offByOperator = false;

	HAL_GPIO_WritePin(cleavageSolvents[which].port,
	                  cleavageSolvents[which].pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(cleavageSolvents[which].motorPort,
	                  cleavageSolvents[which].motorPin, GPIO_PIN_SET);

	UART_Print("[CLEAVE] %s : %s ON + %s ON.\r\n", name, valve, motor);

	for (;;) {
		char  buf[WASH_SEL_INPUT_LEN];
		char *text;

		UART_Print("  %s (%s + %s) is ON -- type 'off' to stop : ",
		           name, valve, motor);

		if (!washSelReadLine(buf, sizeof(buf), &text)) {
			UART_Print("\r\n[CLEAVE] No input -- switching %s and %s off for "
			           "safety.\r\n", valve, motor);
			break;
		}

		if (washSelNamesEqual(text, "off") || washSelNamesEqual(text, "0")) {
			offByOperator = true;
			break;
		}

		UART_Print("[CLEAVE] '%s' is not 'off' -- both stay ON.\r\n", text);
	}

	/* Valve first, then the motor: the motor must never be left running with
	 * the valve already shut. */
	HAL_GPIO_WritePin(cleavageSolvents[which].port,
	                  cleavageSolvents[which].pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(cleavageSolvents[which].motorPort,
	                  cleavageSolvents[which].motorPin, GPIO_PIN_RESET);

	UART_Print("[CLEAVE] %s : %s OFF + %s OFF.\r\n", name, valve, motor);

	osDelay(CLEAVE_SETTLE_MS);

	return offByOperator;
}

/* ---------------------------------------------------------------------------
 *  CLEAVAGE DOOR  --  open-loop, fixed step counts
 *
 *  No limit switches: each move is a fixed number of steps in a fixed
 *  direction. The Clevage_limit_1 / Clevage_limit_2 inputs are not read here
 *  at all any more -- the stepper menu's "15. Read the cleavage limits" still
 *  reads them if they are needed for checking by hand.
 *
 *  The stepper goes through Stepper_Step() with a StepperMotor object on
 *  PUL1A/PUL1B/DIR1A/DIR1B -- the same call and the same pins as the stepper
 *  menu's "12. Operate Stepper (Stepper.h library)".
 *
 *  Closing is 5000 steps SHORT of opening (295000 vs 300000), so the door does
 *  not return to exactly where it started. That is deliberate per the spec, but
 *  it means the offset ACCUMULATES: every open/close cycle leaves the door
 *  5000 steps further along than the last. With nothing reading a switch there
 *  is no datum to correct against, so the door will need re-zeroing by hand
 *  after enough cycles.
 * ------------------------------------------------------------------------- */

#define CLEAVE_DOOR_OPEN_STEPS   300000
#define CLEAVE_DOOR_OPEN_DIR          0
#define CLEAVE_DOOR_OPEN_SPEED       10

#define CLEAVE_DOOR_CLOSE_STEPS  295000
#define CLEAVE_DOOR_CLOSE_DIR         1
#define CLEAVE_DOOR_CLOSE_SPEED       10

static StepperMotor cleaveDoorMotor;
static bool         cleaveDoorReady = false;

static void cleaveDoorInit(void)
{
	if (cleaveDoorReady) {
		return;
	}

	Stepper_Init(&cleaveDoorMotor,
	             PUL1A_Pin_GPIO_Port, PUL1A_Pin_Pin,   /* A0 */
	             PUL1B_Pin_GPIO_Port, PUL1B_Pin_Pin,   /* A1 */
	             DIR1A_Pin_GPIO_Port, DIR1A_Pin_Pin,   /* A2 */
	             DIR1B_Pin_GPIO_Port, DIR1B_Pin_Pin);  /* A3 */

	cleaveDoorReady = true;
}

/* One fixed-length move. Stepper_Step() blocks until every step is done and
 * de-energises the coils on the way out. */
static bool cleaveDoorMove(int steps, int dir, int speed, const char *what)
{
	cleaveDoorInit();

	UART_Print("[CLEAVE] Door %s: %d steps, dir %d, speed %d...\r\n",
	           what, steps, dir, speed);

	Stepper_Step(&cleaveDoorMotor, steps, dir, speed);

	UART_Print("[CLEAVE] Door %s: done.\r\n", what);

	return true;
}

/* Opens the door: 300000 steps, direction 0, speed 10. */
bool cleavageDoorOpen(void)
{
	return cleaveDoorMove(CLEAVE_DOOR_OPEN_STEPS, CLEAVE_DOOR_OPEN_DIR,
	                      CLEAVE_DOOR_OPEN_SPEED, "OPEN");
}

/* Closes the door: 295000 steps, direction 1, speed 1. */
bool cleavageDoorClose(void)
{
	return cleaveDoorMove(CLEAVE_DOOR_CLOSE_STEPS, CLEAVE_DOOR_CLOSE_DIR,
	                      CLEAVE_DOOR_CLOSE_SPEED, "CLOSE");
}

bool clevagemodule(void)
{
	uint32_t wanted = 0;

	UART_Print("\r\n===== CLEAVAGE MODULE =====\r\n");

	/* ---- Step 0 : open the door before anything else runs. ---- */
	(void) cleavageDoorOpen();

	/* ---- Step 1 : Pneumatic 3 ON for 5 s, then OFF. ---- */
	UART_Print("[CLEAVE] Pneumatic 3 ON...\r\n");
	HAL_GPIO_WritePin(Pnuematic_3_GPIO_Port, Pnuematic_3_Pin, GPIO_PIN_SET);

	osDelay(CLEAVE_PNEU3_MS);

	HAL_GPIO_WritePin(Pnuematic_3_GPIO_Port, Pnuematic_3_Pin, GPIO_PIN_RESET);
	UART_Print("[CLEAVE] Pneumatic 3 OFF after %lu ms.\r\n",
	           (unsigned long) CLEAVE_PNEU3_MS);

	osDelay(CLEAVE_SETTLE_MS);

	/* ---- Step 2 : solvent selection, same shape as the wash menu. ---- */
	if (!washSelAskUInt("How many cleavage solvents do you want to add? : ",
	                    &wanted)) {
		UART_Print("[CLEAVE] Aborted.\r\n");

		/* The door was opened at the top, so it is closed on every way out --
		 * an abort must not leave it standing open. */
		(void) cleavageDoorClose();

		return false;
	}

	if (wanted == 0u) {
		UART_Print("[CLEAVE] Nothing to add.\r\n");

		(void) cleavageDoorClose();

		return true;
	}

	/* Repeats are legal -- the same solvent can be topped up twice -- so this
	 * warns rather than refuses, exactly like the wash menu. */
	if (wanted > (uint32_t) CLEAVE_SOLVENT_COUNT) {
		UART_Print("[CLEAVE] Note: %lu selections requested but only %d "
		           "solvents exist -- repeats allowed.\r\n",
		           (unsigned long) wanted, CLEAVE_SOLVENT_COUNT);
	}

	for (uint32_t n = 1u; n <= wanted; n++) {
		CleavageSolvent_t slot;

		UART_Print("\r\n----- Cleavage solvent %lu of %lu -----\r\n",
		           (unsigned long) n, (unsigned long) wanted);

		if (!cleaveAskSolvent(&slot)) {
			(void) cleavageDoorClose();

			return false;
		}

		/* The valve is already closed by cleaveHoldValve() on every path out;
		 * a console-quiet close is reported and the run carries on to the next
		 * selection. */
		if (!cleaveHoldValve(slot)) {
			UART_Print("[CLEAVE] %s closed without an 'off' -- continuing "
			           "with the next selection.\r\n",
			           cleavageSolvents[slot].name);
		}
	}

	UART_Print("\r\n[CLEAVE] All %lu selection(s) complete.\r\n",
	           (unsigned long) wanted);

	/* ---- Last step : close the door. ---- */
	(void) cleavageDoorClose();

	UART_Print("[CLEAVE] Exiting.\r\n");

	return true;
}

/* ---------------------------------------------------------------------------
 *  CLEAVAGE MODULE  --  the whole thing in one call
 *
 *  Sequence, in order:
 *
 *   1. Rail to home. robotLinearGotoHardcodedHome() reads the encoder first and
 *      prints "Already at home" without moving if it is already there, so this
 *      is both the check and the move.
 *   2. Open the door (cleavageDoorOpen(), open-loop step count).
 *   3. Arm through the FIRST THREE taught cleavage poses.
 *   4. On arriving at pose 3: Pneumatic 3 ON and the robot gripper OFF at the
 *      same moment, Pneumatic 3 back off after 5 s.
 *   5. The cleavage solvent menu. For each selection the valve is held open
 *      until the operator types "off", and THEN that solvent's motor runs,
 *      also until "off".
 *   6. Close the door.
 *
 *  Solvent -> motor pairing is in cleavageSolvents[] above.
 *
 *  ASSUMPTION WORTH CHECKING: "motor on after the pneumatic operation is done"
 *  and "motors off after their pneumatic operation is done" pull in opposite
 *  directions, so the motor is given the same operator-controlled stop as the
 *  valve: valve ON -> "off" -> valve OFF -> motor ON -> "off" -> motor OFF.
 *  If the motor should instead run for a fixed time, that is a one-line change
 *  in cleaveRunSolvent().
 * ------------------------------------------------------------------------- */

/* Poses driven before the pneumatic step. cleavagePoses[] holds
 * CLEAVAGE_NUM_POS entries; only the first three are used here. */
#define CLEAVE_SEQ_POSES   3

/* Robot gripper, on/off in one place.
 *
 * The active levels come from ROBOT_GRIP_HOOK_LEVEL / ROBOT_GRIP_UNHOOK_LEVEL
 * in RobotMotion.h, so a solenoid wired the other way round is fixed there and
 * every caller follows. The pin is written directly rather than through
 * robotArmGripperHook()/Unhook() -- those are commented out in this file (they
 * still use the old Robot_Gripper_* label) and carry a 1.5 s settle that would
 * break the "at the same time" pairing with Pneumatic 3. */
static void cleaveGripper(bool on)
{
	HAL_GPIO_WritePin(Robot_gripper_GPIO_Port, Robot_gripper_Pin,
	                  on ? ROBOT_GRIP_HOOK_LEVEL : ROBOT_GRIP_UNHOOK_LEVEL);

	UART_Print("[CLEAVE] Robot gripper %s.\r\n", on ? "ON (hook)"
	                                                : "OFF (release)");
}

/* THE ARM MUST BE HOME BEFORE THE DOOR MOVES.
 *
 * The door used to close while the arm was still parked at cleavage pose 3 --
 * the door shutting onto the arm. So every exit path that has already moved the
 * arm goes through here: arm to POS_HOME FIRST, and only then the door. */
/* Ends the held part of the sequence: Pneumatic 3 off, then the gripper back
 * on. Runs on EVERY exit path after pose 3 -- including the aborts -- so the
 * valve is never left open. */
static void cleaveFinishPneumaticAndGrip(void)
{
	HAL_GPIO_WritePin(Pnuematic_3_GPIO_Port, Pnuematic_3_Pin, GPIO_PIN_RESET);
	UART_Print("[CLEAVE] Pneumatic 3 OFF (solvent sequence finished).\r\n");

	osDelay(CLEAVE_SETTLE_MS);

	cleaveGripper(true);

	osDelay(CLEAVE_SETTLE_MS);
}

static void cleaveArmHomeThenCloseDoor(void)
{
	cleaveFinishPneumaticAndGrip();

	UART_Print("[CLEAVE] Arm -> home before the door closes...\r\n");

	if (moveRobotTo(POS_HOME) != 0) {
		UART_Print("[CLEAVE] Arm home move FAILED -- DOOR NOT CLOSED.\r\n"
		           "  Closing it now would shut onto the arm. Clear the arm by "
		           "hand, then close the door from the menu.\r\n");

		return;
	}

	osDelay(500);

	(void) cleavageDoorClose();
}

/* One solvent: valve and its paired motor, together. */
static void cleaveRunSolvent(CleavageSolvent_t which)
{
	if (!cleaveHoldValveAndMotor(which)) {
		UART_Print("[CLEAVE] %s stopped without an 'off'.\r\n",
		           cleavageSolvents[which].name);
	}
}

bool cleavageModuleSequence(void)
{
	uint32_t wanted = 0;

	UART_Print("\r\n===== CLEAVAGE MODULE (full sequence) =====\r\n");

	/* ---- 1. Robot up, then arm AND rail to home ----
	 * The arm is homed BEFORE the rail: the rail carries the arm, so travelling
	 * with the arm still extended at whatever pose it was left in is what
	 * risks hitting something. Both happen before anything else moves, so the
	 * sequence always starts from a known place rather than from wherever the
	 * last run stopped. */
	if (connectRobot() != 0) {
		UART_Print("[CLEAVE] Robot not connected -- aborting.\r\n");

		return false;
	}

	if (robotInitConfig() != 0) {
		UART_Print("[CLEAVE] Robot NOT READY -- aborting before any motion.\r\n"
		           "   Clear faults + switch to AUTOMATIC on the pendant, then "
		           "retry.\r\n");

		return false;
	}

	/* Gripper ON before anything moves, so the vessel is held for the whole
	 * approach and is only released where the sequence says to. */
	cleaveGripper(true);

	UART_Print("[CLEAVE] Arm -> home...\r\n");

	if (moveRobotTo(POS_HOME) != 0) {
		UART_Print("[CLEAVE] Arm did not reach home -- aborting.\r\n");

		return false;
	}

	osDelay(500);

	UART_Print("[CLEAVE] Rail -> home...\r\n");

	servoBusLock();
	bool railHomed = robotLinearGotoHardcodedHome();
	servoBusUnlock();

	if (!railHomed) {
		UART_Print("[CLEAVE] Rail did not reach home -- aborting.\r\n");

		return false;
	}

	aminoRailTurns = AMINO_RAIL_HOME;   /* keep this file's tracker in step */

	/* ---- 2. Open the door ---- */
	(void) cleavageDoorOpen();

	/* ---- 3. Arm: the first three cleavage poses ---- */
	for (int i = 0; i < CLEAVE_SEQ_POSES && i < CLEAVAGE_NUM_POS; i++) {
		UART_Print("\r\n[CLEAVE] ---- Position %d of %d ----\r\n",
		           i + 1, CLEAVE_SEQ_POSES);

		if (robotMoveJStr(cleavagePoses[i]) != 0) {
			UART_Print("[CLEAVE] Position %d FAILED -- sequence ABORTED.\r\n",
			           i + 1);

			return false;
		}

		osDelay(MODULE_STEP_DELAY_MS);   /* settle before the next pose */
	}

	/* ---- 4. At pose 3: Pneumatic 3 ON and the gripper OFF together ----
	 * Written back to back so they switch in the same instant; the gripper is
	 * driven directly rather than through robotArmGripperUnhook(), which is
	 * commented out in this file and carries a 1.5 s settle that would break
	 * the "at the same time" requirement. */
	UART_Print("\r\n[CLEAVE] Pose 3 reached: Pneumatic 3 ON + gripper OFF.\r\n");

	HAL_GPIO_WritePin(Pnuematic_3_GPIO_Port, Pnuematic_3_Pin, GPIO_PIN_SET);
	cleaveGripper(false);   /* released here, in the same instant */

	/* NO 5 s timer here any more: Pneumatic 3 is HELD ON for the whole solvent
	 * sequence and is only switched off afterwards, in
	 * cleaveFinishPneumaticAndGrip(). */
	UART_Print("[CLEAVE] Pneumatic 3 stays ON until the solvent sequence "
	           "finishes.\r\n");

	osDelay(CLEAVE_SETTLE_MS);

	/* ---- 5. Solvent selection, each valve followed by its motor ---- */
	if (!washSelAskUInt("How many cleavage solvents do you want to add? : ",
	                    &wanted)) {
		UART_Print("[CLEAVE] Aborted.\r\n");

		cleaveArmHomeThenCloseDoor();

		return false;
	}

	if (wanted > (uint32_t) CLEAVE_SOLVENT_COUNT) {
		UART_Print("[CLEAVE] Note: %lu selections requested but only %d "
		           "solvents exist -- repeats allowed.\r\n",
		           (unsigned long) wanted, CLEAVE_SOLVENT_COUNT);
	}

	for (uint32_t n = 1u; n <= wanted; n++) {
		CleavageSolvent_t slot;

		UART_Print("\r\n----- Cleavage solvent %lu of %lu -----\r\n",
		           (unsigned long) n, (unsigned long) wanted);

		if (!cleaveAskSolvent(&slot)) {
			cleaveArmHomeThenCloseDoor();

			return false;
		}

		cleaveRunSolvent(slot);
	}

	UART_Print("\r\n[CLEAVE] All %lu selection(s) complete.\r\n",
	           (unsigned long) wanted);

	/* ---- 6. Arm back to home, THEN close the door ---- */
	cleaveArmHomeThenCloseDoor();

	UART_Print("[CLEAVE] Sequence COMPLETE.\r\n");

	return true;
}
