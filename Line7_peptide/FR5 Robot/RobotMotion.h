/*
 * RobotMotion.h
 *
 *  Created on: Nov 27, 2025
 *      Author: SE-02
 */

#ifndef FR5_ROBOT_ROBOTMOTION_H_
#define FR5_ROBOT_ROBOTMOTION_H_

#include <stdbool.h>

#include "RobotCoordinates.h"

/* ================= Linear rail (Rtelligent servo) ================= */
/* Modbus address of the rail drive on the servo bus.
 * LINE 7 already uses 0x01 = MIXER_ID, 0x02 = MicroID (amino platform),
 * so the rail lives at 0x03. Change to match your DIP/param setting. */
#define ROBOT_LINEAR_ID        0x04

#define ROBOT_RAIL_HOME_PULSES   5   /* absolute encoder count at home -- measured */

#define ROBOT_GEAR_RATIO       7.5      /* rail: motor turns per output "turn" unit */
#define ROBOT_ORGIN_SPEED      3000
#define ROBOT_SPEED            2500
#define ROBOT_ACCEL            500
#define ROBOT_DECEL            500

/* ================= Deck stations (process-facing) ================= */
typedef enum {
	ST_HOME = 0,   /* safe travel pose, rail datum */
	ST_LIQUID,     /* St 1  : DMF, DIPEA, amino-acid solutions */
	ST_SOLID,      /* St 2  : resin, Fmoc-AA, HATU (powders) + activator */
	ST_VOLATILE,   /* St 3  : SOCl2, DCM, MeOH, 20% pip/DMF, cleavage cocktail */
	ST_STIR,       /* St 5  : stirring / reaction station (reactor home) */
	ST_DRAIN,      /* St 9  : drain & monitor (UV, conductivity, vacuum) */
	ST_CLEAVE,     /* St 10 : cleavage */
	ST_PARK,       /* free stir station (St 4-8): vessel pick + folding */
	NUM_STATIONS
} Station;

/* Per-station rail position, in rail "turns" from the datum (placeholders --
 * teach on the physical deck, exactly like the Line 5 rail offsets). */
#define RAIL_HOME       0
#define RAIL_ST1        40      /* St 1  liquid   */
#define RAIL_ST2        80      /* St 2  solid    */
#define RAIL_ST3        120     /* St 3  volatile */
#define RAIL_ST5        160     /* St 5  stir     */
#define RAIL_ST9        200     /* St 9  drain    */
#define RAIL_ST10       260     /* St 10 cleavage */
#define RAIL_PARK       160     /* free stir station (shares St5 lane by default) */

/* ================= Rail (servo) primitives ================= */
bool ConfigRobotLinear(void);
bool MoveLinearServo(int turns);       /* relative move, +/- turns */
void robotLinearStop(void);
bool robotLinearHome(void);
void setCurrentPositionAsHome(void);

/* ================= Arm + rail transport ================= */
void   moveRobotToHomePosition(void);          /* arm -> POS_HOME */
bool   robotResetToHome(void);                 /* home rail + arm, current = ST_HOME */
bool   moveVesselTo(Station target);           /* full transport current -> target */
Station getCurrentStation(void);

/* ================= PNEUMATIC GRIPPERS (hook / unhook) =================
 * Ported from the 555 Reaction Module's two gripper relays:
 *    Robot_gripper (arm)     : 1 = close/HOOK the vessel, 0 = open/UNHOOK
 *    Cleavage_Grip (cleavage): 0 = close/HOOK the vessel, 1 = open/UNHOOK
 * On Line 7 these map to spare Cylinder outputs (pins chosen in main.h). The
 * active levels below reproduce the Reaction Module polarity EXACTLY -- if a
 * solenoid is wired the opposite way, flip only the matching pair. */
#define ROBOT_GRIP_HOOK_LEVEL       GPIO_PIN_SET     /* Robot_gripper = 1 -> close/grab   */
#define ROBOT_GRIP_UNHOOK_LEVEL     GPIO_PIN_RESET   /* Robot_gripper = 0 -> open/release  */
#define CLEAVAGE_GRIP_HOOK_LEVEL    GPIO_PIN_RESET   /* Cleavage_Grip = 0 -> close/grab    */
#define CLEAVAGE_GRIP_UNHOOK_LEVEL  GPIO_PIN_SET     /* Cleavage_Grip = 1 -> open/release  */

/* Settle time after a solenoid is switched, so the pneumatic actuator has time
 * to fully seat before the arm moves (Reaction Module used ~1.5 s here). */
#define GRIP_SETTLE_MS   1500

/* ================= POWDER DISPENSER MODULE ================= */
/* Rail +50 turns, then arm: HOME -> POWDER1 -> POWDER2 -> POWDER3 -> POWDER4 -> HOME. */
#define POWDER_NUM_POS     5    /* HOME + 4 powder poses          */
#define POWDER_NUM_STEPS   6    /* HOME -> P1 -> P2 -> P3 -> P4 -> HOME */
extern const char *powderPoses[POWDER_NUM_POS];
bool   runPowderDispenserModule(void);

/* ================= REACTION STIRRING MODULE ================= */
/* Arm: HOME -> STIR1..STIR6 -> HOME, then rail +100 turns -> rail home. */
#define STIR_NUM_POS      6    /* HOME + 6 stir poses                    */
#define STIR_NUM_STEPS    11    /* HOME -> STIR1..6 -> HOME                */
extern const char *stirPoses[STIR_NUM_POS];
bool   runReactionStirModule(void);

void robotArmGripperHook(void);     /* arm gripper closes  -> grab the vessel   */
void robotArmGripperUnhook(void);   /* arm gripper opens   -> release the vessel */
void cleavageGripHook(void);        /* cleavage clamp closes -> hold the vessel  */
void cleavageGripUnhook(void);      /* cleavage clamp opens  -> release the vessel */

/* ================= CLEAVAGE MODULE (command "2") ================= */
/* Reactor vessel pick & place: runs the 6 taught poses in ascending order.
 * Poses live in cleavagePoses[] (RobotMotion.c). */
#define CLEAVAGE_NUM_POS   5
extern const char *cleavagePoses[CLEAVAGE_NUM_POS];
bool   runCleavageModule(void);                /* run Positions 1..6, blocking */

/* ================= LIQUID DISPENSER MODULE (command "3") ================= */
/* Round trip HOME -> Position 2 -> Position 3 -> Position 2 -> HOME.
 * The 3 distinct taught poses live in liquidPoses[] (RobotMotion.c): index
 * 0 = HOME, 1 = Position 2, 2 = Position 3. The 5-step playback order revisits
 * Position 2 and HOME on the way back (see liquidSeq[] in RobotMotion.c). */
#define LIQUID_NUM_POS     4    /* HOME + 6 taught liquid poses  */
#define LIQUID_NUM_STEPS   8    /* HOME -> Pos1..Pos6 -> HOME     */
extern const char *liquidPoses[LIQUID_NUM_POS];
bool   runLiquidDispenserModule(void);         /* run the 5-step round trip, blocking */

/* Split halves used by sequence(): the arm must STOP at the pour pose while
 * the drain valve is open, then retract only after the liquid is collected.
 * Calling the full round trip above would return the arm to HOME first. */
bool   runLiquidDispenserApproach(void);       /* HOME -> Pos2 -> Pos3 (stop) */
bool   runLiquidDispenserRetract(void);        /* Pos2 -> HOME                */

bool robotLinearReturnToHome(void);   /* move rail back to the set home */
bool robotLinearGotoHardcodedHome(void);

bool   runReactionStirModule(void);
bool   runReactionReStirModule(void);   /* reposture pass, RESTIR poses */

bool sequence(void);   /* full run: rail+powder -> liquid -> rail+stir */

static void stirDrainPulse(const char *why);
static void aminoCarouselGotoAbsoluteDeg(float deg, const char *why);
bool wash(void);
bool amino_acid_addition_sequence(void);

/* Interactive wash-solvent menu: asks how many solvents, then for each one the
 * solvent name and the amount, moves the carousel to that slot via
 * wash_solvents(), and runs X_pump + DC motor for WASH_SEL_PUMP_RUN_MS.
 * Returns false only if the operator aborted or a carousel move failed. */
bool wash_solvent_selection(void);

/* Interactive amino-acid menu: asks which of the eight syringes and the weight,
 * moves the carousel to that syringe (36 deg per syringe, anticlockwise), and
 * doses it on the load cell via aminoDoseAndPour(). One dose per call. */
bool amino_acid_selection(void);

/* Octertoide run: powder module (30 s hold) -> 8 operator wash-solvent
 * selections interleaved with 4 stir+restir groups, and a 5.0 g Threonol
 * dispense from syringe 1. Blocks on the console at each selection. */
bool octertoide_sequence(void);

/* Cleavage module: Pneumatic 3 ON for 5 s, then the wash-menu style solvent
 * loop over the four cleavage solvents -- TFA (Pneumatic 1), EDT (Pneumatic 2),
 * TIS (Pneumatic 4), H2O (Pneumatic 5). Each selection asks for a weight in
 * tenths of a gram and holds that valve open until the load cell says that much
 * has been added. Returns false only if the operator aborted. */
bool clevagemodule(void);

/* Cleavage door, stepper driven through Stepper_Step() on the same pins as the
 * stepper menu's option 12. OPEN-LOOP: fixed step counts, no limit switches
 * read at all. Open = 300000 steps, dir 0, speed 10. Close = 295000 steps,
 * dir 1, speed 1. Both block until the move finishes and always return true.
 * Called at the top and bottom of clevagemodule(). */
bool cleavageDoorOpen(void);
bool cleavageDoorClose(void);

/* The whole cleavage module in one call:
 *   rail -> home  ->  door open  ->  arm through cleavage poses 1..3
 *   ->  Pneumatic 3 ON 5 s with the gripper released at the same instant
 *   ->  solvent menu, each valve followed by its paired solvent motor
 *   ->  door close.
 * Solvent -> motor pairing: TFA/Pneu1 -> motor 2, EDT/Pneu2 -> motor 4,
 * TIS/Pneu4 -> motor 1, H2O/Pneu5 -> motor 3. Blocks on the console at each
 * selection. false = the rail, the robot or the operator stopped it. */
bool cleavageModuleSequence(void);


#define DBG_REACHED()  UART_Print("Reached %s\r\n", __func__)


#endif /* FR5_ROBOT_ROBOTMOTION_H_ */
