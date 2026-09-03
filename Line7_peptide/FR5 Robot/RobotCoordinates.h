/*
 * RobotCoordinates.h
 *
 *  6-axis FAIRINO FR5 arm poses for the LINE 7 peptide (flow-SPPS) deck.
 *
 *  PROTOCOL NOTE (matches LINE 1 production, the identical robot):
 *  The FR5 is driven with the FAIRINO frame protocol (see RobotCommand.*),
 *  MoveJ(<pose>,<extra>). Each pose is a **decimal string** of 12 values:
 *      j1,j2,j3,j4,j5,j6, x,y,z,rx,ry,rz
 *  i.e. the six joint angles (deg) followed by the tool cartesian pose. These
 *  are taught on the pendant and pasted verbatim -- NO scaling, NO x1000 / x100.
 *  (The old LINE 5 "RemoteMonitor" integer milli-degree format was wrong for
 *  this controller -- that is what caused the command to be rejected.)
 *
 *  IMPORTANT -- teach these on the physical cell. POS_HOME below is LINE 1's
 *  REFERENCE pose (a known-safe home for this arm); the station poses default to
 *  it until you jog + teach them. Keep the E-stop reachable.
 *
 *  NOTE: this content was uploaded mislabeled as "RobotCommand.h" (which
 *  should instead hold the function declarations below in RobotCommand.h) --
 *  restored to its correct file/name here. positions[]/DEFAULT_EXTRA are
 *  defined in RobotMotion.c, matching this enum exactly.
 */

#ifndef FR5_ROBOT_ROBOTCOORDINATES_H_
#define FR5_ROBOT_ROBOTCOORDINATES_H_

/*
 * Every named pose the arm can be commanded to.
 *
 * Convention (mirrors LINE 5/7 station model): each functional station has an
 * APPROACH pose (safe pre-insert/extract) and a WORK pose (engaged). HOME is
 * the safe travel pose used between stations while the rail moves.
 */
typedef enum {
	POS_HOME = 0,        /* safe travel / reference pose */

	POS_ST1_APP,         /* St 1  liquid dispenser  - approach */
	POS_ST1_WORK,        /* St 1  liquid dispenser  - engaged  */

	POS_ST2_APP,         /* St 2  solid dispenser   - approach */
	POS_ST2_WORK,        /* St 2  solid dispenser   - engaged  */

	POS_ST3_APP,         /* St 3  volatile disp.    - approach */
	POS_ST3_WORK,        /* St 3  volatile disp.    - engaged  */

	POS_ST5_APP,         /* St 5  stir station      - approach */
	POS_ST5_WORK,        /* St 5  stir station      - engaged  */

	POS_ST9_APP,         /* St 9  drain & monitor   - approach */
	POS_ST9_WORK,        /* St 9  drain & monitor   - engaged  */

	POS_ST10_APP,        /* St 10 cleavage          - approach */
	POS_ST10_WORK,       /* St 10 cleavage          - engaged  */

	POS_PARK_APP,        /* free stir station (St 4-8) - approach */
	POS_PARK_WORK,       /* free stir station (St 4-8) - engaged  */

	NUM_POS
} PositionID;

/* Taught poses: 12-value decimal strings "j1,..,j6, x,y,z,rx,ry,rz". Defined
 * in RobotMotion.c. */
extern const char *positions[NUM_POS];

/* Trailing MoveJ parameters (tool,user,vel,acc,ovl,...) -- same for every
 * move. Defined in RobotMotion.c. */
extern const char *DEFAULT_EXTRA;

#endif /* FR5_ROBOT_ROBOTCOORDINATES_H_ */
