/*
 * servo.h
 *
 *  Created on: 29-Jun-2026
 *      Author: Electronics Dept
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_


/* =========================================================
 INCLUDES
 ========================================================= */

/*#include "../Serial_Comm/SerialComm.h"*/

#include "stm32f7xx_hal.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/* =========================================================
 MODBUS CRC
 ========================================================= */

#define CRC_LSB_MASK               0x0001
#define CRC_POLYNOMIAL             0xA001

/* =========================================================
 SERVO CONFIGURATION
 ========================================================= */

/* CONTROL MODE */

#define SERVO_CONTROL_MODE         500

#define POSITION_CONTROL           0
#define SPEED_CONTROL              1

/* HOMING */

#define SERVO_HOME_TRIGGER         340
#define SERVO_HOMING_START         5
#define SERVO_CURRENT_HOME         6

/* POSITION MODE */

#define SERVO_POS_MODE             353

#define INCREMENTAL                0x00
#define ABSOLUTE                   0x01

/* POSITION COMMAND */

#define SERVO_POSITION             357
#define SERVO_PPR                  5000
#define SERVO_OFFSET               364

/* MOTION */

#define ORGIN_SPEED_CMD            342

#define SERVO_POS_SPEED            356
#define SERVO_POS_ACC_TIME         354
#define SERVO_POS_DEC_TIME         355

#define SERVO_MOTION_CMD           359

#define SERVO_FORWARD              1
#define SERVO_REVERSE              2
#define SERVO_STOP                 5

/* SPEED MODE */

#define SERVO_SPEED_SPEED          40
#define SERVO_SPEED_ACCEL          15000
#define SERVO_SPEED_DECEL          15000

#define SERVO_SPEED_STOP           0

/* STATUS */

#define READ_CURRENT_POS           1309
#define READ_RUNNING_STATUS        1300

/* FAULT */

#define FAULT_RESET                1208

/* GENERAL */

#define MAX_RUN                    2147483640

#define SET_HIGH                   1
#define SET_LOW                    0

#define INDEX_GEAR_RATIO           60

/* =========================================================
 COMMUNICATION CONFIGURATION
 ========================================================= */

#define SERVO_MAX_RETRY            3

/* Timeout for a single whole-frame HAL_UART_Receive() call in
 * ServoReadResponse() (no longer a "max gap between polled bytes" --
 * that was the old byte-by-byte polling design, replaced because it was
 * dropping bytes to UART overruns). 500ms is generous for an 8-9 byte
 * RTU frame; it only needs to cover genuine turnaround/processing time
 * before the servo starts replying, not the transmission itself (which
 * takes well under 10ms at typical RTU baud rates). */
#define SERVO_RESPONSE_TIMEOUT     1000

/* =========================================================
 MODBUS FUNCTION CODES
 ========================================================= */

#define MODBUS_READ_HOLDING        0x03
#define MODBUS_WRITE_SINGLE        0x06
#define MODBUS_WRITE_MULTIPLE      0x10

/* =========================================================
 MODBUS EXCEPTION CODES
 ========================================================= */

#define MODBUS_EX_ILLEGAL_FUNC     0x01
#define MODBUS_EX_ILLEGAL_ADDR     0x02
#define MODBUS_EX_ILLEGAL_VALUE    0x03
#define MODBUS_EX_SLAVE_FAILURE    0x04

/* SEVO Address */
#define MIXER_ID                   0x01
#define MicroID                    0x02

/* Default Positions */
#define REACTION_DEFAULT_SPEED     300
#define REACTION_DEFAULT_CYCLES    10
#define REACTION_DEFAULT_DEG       360
#define REACTION_DEFAULT_OFFSET    90
#define REACTION_MOTION_TIMEOUT    15000

/* =========================================================
 MOTION-COMPLETE DETECTION
 =========================================================
 * "Motion complete" is inferred from the position feedback going quiet.
 * Require the position to stay within TOLERANCE pulses for COUNT consecutive
 * 100 ms samples (~800 ms of genuine stillness) before declaring done -- this
 * is the proven mbed logic. A single equal pair is NOT enough: comms hiccups
 * and brief pauses would otherwise fake completion while the servo is still
 * turning. */
#define MOTION_STABLE_TOLERANCE    5     /* pulses within this = "not moving"  */
#define MOTION_STABLE_COUNT        8     /* consecutive stable samples needed  */

/* =========================================================
 DISPENSER ANGLE CALIBRATION
 =========================================================
 * Command counts written to the drive = DegreeToPulse(deg) * DISPENSER_POS_SCALE.
 * With SERVO_PPR = 5000 this works out to (5000/360)*4.5 = 62.5 counts per
 * degree of dispenser OUTPUT rotation. 4.5 is the empirical factor carried over
 * from the C++ build. IF THE SERVO ROTATES THE WRONG AMOUNT, THIS IS THE ONE
 * NUMBER TO RECALIBRATE: command a known angle (e.g. 360), measure the real
 * output rotation, and scale -- new = old * (commanded / measured). */
#define DISPENSER_POS_SCALE        4.5f

/* =========================================================
 SERVO HANDLE
 ========================================================= */

typedef struct {

    UART_HandleTypeDef *huart;

    uint32_t tx_count;
    uint32_t rx_count;

    uint32_t timeout_count;
    uint32_t crc_error_count;
    uint32_t exception_count;

    uint32_t retry_count;

    bool communication_ok;

} ServoHandle;

/* =========================================================
 INITIALIZATION
 ========================================================= */

void ServoInit(ServoHandle *servo, UART_HandleTypeDef *huart);

/* =========================================================
 LOW LEVEL FUNCTIONS
 ========================================================= */

void ServoFlushUART(ServoHandle *servo);

void ServoSendCmd(ServoHandle *servo, uint8_t *cmd, int len);

bool ServoReadResponse(ServoHandle *servo, uint8_t *resp, int resp_len);

bool ServoValidateCRC(uint8_t *resp, int len);

bool ServoValidateWriteAck(uint8_t *cmd, uint8_t *resp, int len);

/* =========================================================
 READ FUNCTIONS
 ========================================================= */

bool ServoReadRegisters(ServoHandle *servo, uint8_t id, uint16_t reg,
		uint16_t count, uint16_t *out);

uint16_t ServoRead16(ServoHandle *servo, uint8_t id, uint16_t reg);

int32_t ServoRead32(ServoHandle *servo, uint8_t id, uint16_t reg);

/* =========================================================
 SAFE READ FUNCTIONS
 ========================================================= */

bool ServoSafeRead16(ServoHandle *servo, uint8_t id, uint16_t reg,
		uint16_t *value);

bool ServoSafeRead32(ServoHandle *servo, uint8_t id, uint16_t reg,
		int32_t *value);

bool ServoSafeReadPosition(ServoHandle *servo, uint8_t id, uint16_t reg,
		int32_t *pos);

/* =========================================================
 NORMAL WRITE FUNCTIONS
 ========================================================= */

void ServoWrite16(ServoHandle *servo, uint8_t id, uint16_t reg, uint16_t value);

void ServoWrite32(ServoHandle *servo, uint8_t id, uint16_t reg, int32_t value);

/* =========================================================
 ACK VERIFIED WRITE FUNCTIONS
 ========================================================= */

bool ServoWrite16Ack(ServoHandle *servo, uint8_t id, uint16_t reg,
		uint16_t value);

bool ServoWrite32Ack(ServoHandle *servo, uint8_t id, uint16_t reg,
		int32_t value);

/* =========================================================
 FULL VERIFIED WRITE FUNCTIONS
 ========================================================= */

bool ServoWrite16Verified(ServoHandle *servo, uint8_t id, uint16_t reg,
		uint16_t value);

bool ServoWrite32Verified(ServoHandle *servo, uint8_t id, uint16_t reg,
		int32_t value);

/* =========================================================
 MOTION FUNCTIONS
 ========================================================= */

bool ServoWaitMotionComplete(ServoHandle *servo, uint8_t id, uint16_t posReg,
		int timeout_ms);

/* =========================================================
 UTILITY FUNCTIONS
 ========================================================= */

int32_t DegreeToPulse(float deg);

/* =========================================================
 DIAGNOSTIC FUNCTIONS
 ========================================================= */

void ServoResetDiagnostics(ServoHandle *servo);

bool ServoIsCommunicationOK(ServoHandle *servo);

void DispensingMove(int deg, int speed);

/* =========================================================
 POSITION CONFIG
 ========================================================= */

void ServoPositionConfig(ServoHandle *servo, uint8_t id);

void ComputeCRC(uint8_t *buf, uint16_t len);
/* =========================================================
 REACTION MOVE
 ========================================================= */

void ReactionMove(ServoHandle *servo, uint8_t id, int offset_deg,
        int swing_deg, uint16_t speed, int cycles, int cycle_delay_ms);

/* Continuous LEFT<->RIGHT oscillation with no stop/wait between swings.
 * cycle_delay_ms is reused as the time per stroke in ms (0 -> a default). */
void ReactionMoveContinuous(ServoHandle *servo, uint8_t id, int offset_deg,
        int swing_deg, uint16_t speed, int cycles, int cycle_delay_ms);

/* =========================================================
 MENU
 ========================================================= */

/*void ServoMenuTask(ServoHandle *servo, SerialPort *pc);*/

void ServoMenuTask(ServoHandle *servo, UART_HandleTypeDef *huart);

/*==========AMINO ACID SERVO==========*/

void AminoAcidServo(ServoHandle *servo, uint8_t id);

/*==========AMINO ACID SERVO MOVEMENT============*/

void DispenserMove(ServoHandle *servo, uint8_t id, int deg, uint16_t speed);

void ServoScanIDs(ServoHandle *servo);

/* =========================================================
 DISPENSER PLATE — NAMED SLOTS
 =========================================================
 * The dispensing plate is the rotary carousel driven by the MicroID servo
 * through DispenserMove(). Two plates, each with its own slot count, each
 * mapped straight through: name N sits in slot N.
 *
 *   AminoAcidSlots()  — 8 syringes, 36 deg apart (every 2nd slot of the
 *                       20-slot plate), named (slots 0..7).
 *   wash_solvents()   — DCM 16 deg CLOCKWISE of home, then a 30 deg pitch
 *                       ANTICLOCKWISE for the rest (slots 0..10).
 *
 * DISPENSER_SLOT_DIR below is the single knob for physical direction and
 * applies to both maps: positive angles turn anticlockwise, negative ones
 * (only DCM) clockwise.
 *
 * Both take a NAME (enum) and drive the plate to that name's absolute angle.
 * DispenserMove() is INCREMENTAL (ServoPositionConfig sets INCREMENTAL mode),
 * so these helpers keep track of where the plate currently is and command only
 * the DIFFERENCE, by the shorter way round. That is what makes it safe to call
 * any slot, in any order, from any file:
 *
 *      AminoAcidSlots(&servo, AA_LYSINE);      // slot 3 -> 3 * 36      = 108 deg
 *      wash_solvents(&servo, WASH_DMF);        // slot 2 -> 2 * 30 - 16 =  44 deg
 *      servo_home_fixed(&servo);               // -> back to the fixed home
 *
 * The tracked angle is only correct if the plate starts from the fixed home.
 * Call servo_home_fixed() (or ServoHomeFixedCapture()) once at start-up, or
 * just let the first slot call latch home implicitly.
 ========================================================= */

/* Servo that carries the dispensing plate. */
#define DISPENSER_SLOT_ID          MicroID

/* SERVO_POS_SPEED used for slot-to-slot indexing. 40 is what the existing
 * 18-degree indexing in the machine menu already uses on this plate. */
#define DISPENSER_SLOT_SPEED       40

/* Sign that turns "increasing slot number" into a DispenserMove() angle, and
 * therefore which way the plate physically turns to reach a named slot.
 *
 * (+1) = ANTICLOCKWISE, which is what BOTH maps below now use: the syringes
 * and the solvents run the same way round. It applies to the whole shared
 * angle frame, so it must stay a single value — give the two maps different
 * signs here and one tracked angle would mean two different physical
 * positions. Flip to (-1) to send everything back the other way.
 *
 * NOTE this is the opposite of the legacy indexing in Machinemenu.c, which
 * steps to the next slot with DispenserMove(servo, MicroID, -18, 40). That
 * code is untouched and still steps its own way; only the named-slot helpers
 * here follow this constant. */
#define DISPENSER_SLOT_DIR         (+1)

/* Syringe geometry: 36 deg between adjacent syringes, anticlockwise.
 *
 * The plate itself has 20 slots 18 deg apart, so the 8 syringes occupy every
 * SECOND slot -- which is the same 36 deg pitch the rest of this project's
 * carousel code uses (SEQ_DISPENSER_DEG is -36, NUM_SYRINGES is 8). Syringe n
 * is therefore at n * 36 deg, and the last one (n = 7) at 252 deg, comfortably
 * inside one revolution. */
#define AMINO_PLATE_SLOTS          10         /* 360 / 36 positions */
#define AMINO_SLOT_STEP_DEG        36.0f      /* syringe(n) -> syringe(n+1) */

/* Wash plate: an evenly spaced 30 deg ring, shifted so DCM sits 16 deg
 * CLOCKWISE of home and everything after it runs ANTICLOCKWISE:
 *      solvent n  ->  n * WASH_SLOT_STEP_DEG - WASH_SLOT_OFFSET_DEG
 *
 * so DCM is at -16 (16 deg clockwise), SOCl2 at +14, DMF at +44 ... H2O at
 * +284. All 11 fit inside one revolution, none wraps onto another, and a 12th
 * would sit at +314, still clear.
 *
 * Because wash_solvent_selection() returns to home before every solvent, the
 * 30 deg figure is the SPACING between adjacent solvents, not the move that
 * gets commanded: picking SOCl2 from home turns +14, not 30. */
#define WASH_SLOT_OFFSET_DEG       16.0f      /* DCM, clockwise of home  */
#define WASH_SLOT_STEP_DEG         30.0f      /* slot(n) -> slot(n+1)    */
#define WASH_PLATE_SLOTS           12         /* positions the plate holds */

/* Angle below which a requested move is treated as "already there". */
#define DISPENSER_SLOT_EPS_DEG     0.05f

/* ---------------------------------------------------------
 AMINO ACID SLOTS  (18 deg apart)
 ---------------------------------------------------------
 * NOTE: 8 syringe slots were requested but only 7 names were given
 * (Threonol, Cysteine, Threonine, Lysine, Dtraptoc, Phenamine, D-Phenamine).
 * AA_SLOT_8 is a placeholder for the 8th — rename it when you have the name.
 * Physical slot numbers live in AminoAcidSlotIndex[] in servo.c; edit that
 * table if the syringes are not in plate slots 0..7. */
typedef enum {
    AA_THREONOL = 0,
    AA_CYSTEINE,
    AA_THREONINE,
    AA_LYSINE,
    AA_DTRAPTOC,
    AA_PHENAMINE,
    AA_D_PHENAMINE,
    AA_SLOT_8,                 /* 8th syringe — name not supplied yet */
    AA_SLOT_COUNT
} AminoAcidSlot_t;

/* ---------------------------------------------------------
 WASH / SOLVENT SLOTS  (DCM 16 deg clockwise, then 30 deg each)
 ---------------------------------------------------------
 * Solvent N sits in slot N, so the order below IS the order round the plate:
 * DCM at -16 deg, SOCl2 at +14, DMF at +44 ... H2O at +284. Positions live in
 * WashSolventSlotIndex[] in servo.c; that table is the only thing to edit if
 * the bottles are arranged differently. */
typedef enum {
    WASH_DCM = 0,
    WASH_SOCL2,
    WASH_DMF,
    WASH_PIPERIDINE_20_DMF,    /* 20% Piperidine / DMF */
    WASH_MEOH,
    WASH_DIPEA,
    WASH_HATU,
    WASH_TFA,
    WASH_EDT,
    WASH_TIS,
    WASH_H2O,
    WASH_SLOT_COUNT
} WashSolventSlot_t;

/* Move the plate to a named amino-acid slot. */
bool AminoAcidSlots(ServoHandle *servo, AminoAcidSlot_t slot);

/* Move the plate to a named wash/solvent slot. */
bool wash_solvents(ServoHandle *servo, WashSolventSlot_t slot);

/* Return the plate to the fixed home it was in at start-up. The very first
 * call latches the current position as that home instead of moving. */
bool servo_home_fixed(ServoHandle *servo);

/* Force "here is home, and this is 0 deg" — use if you re-fix the plate by
 * hand mid-run, or to latch home explicitly during start-up. */
void ServoHomeFixedCapture(ServoHandle *servo);

/* Where the helpers think the plate is, in degrees from home (0..360). */
float ServoDispenserAngle(void);

/* Printable slot names, for menus and logs. */
const char *AminoAcidSlotName(AminoAcidSlot_t slot);
const char *WashSolventSlotName(WashSolventSlot_t slot);

/* =========================================================
 REACTION (MIXER) SERVO — PARK AT HOME
 =========================================================
 * ReactionMove() leaves the mixer wherever its last swing ended. These park it
 * at a known ABSOLUTE encoder count so the reaction vessel always finishes in
 * the same orientation, callable from any file:
 *
 *      ReactionMoveHome(&servo);           // -> REACTION_HOME_PULSES
 *      ReactionMoveToPulses(&servo, 900);  // -> any absolute count
 *
 * The drive is in INCREMENTAL mode, so these read READ_CURRENT_POS and command
 * the DELTA -- the same read/delta/trigger/wait pattern already used to park
 * the mixer at the end of the parallel reaction task in RobotMotion.c.
 ========================================================= */

/* Mixer park position, in ABSOLUTE encoder counts as reported by
 * READ_CURRENT_POS. 2168 = 0x0878, the count read back from the drive at the
 * fixed home. RobotMotion.c carries an identical #define of its own (that is
 * where this value comes from); the two must stay in step. */
#define REACTION_HOME_PULSES       2168

/* SERVO_POS_SPEED used for the park move. Matches REACTION_PARALLEL_SPEED,
 * i.e. the speed the mixer is already running at when RobotMotion.c parks it,
 * so behaviour is unchanged -- but set explicitly here so the result does not
 * depend on whatever ran before. */
#define REACTION_HOME_SPEED        100

/* Timeout for the park move. */
#define REACTION_HOME_TIMEOUT_MS   30000

/* How far off the target the mixer may settle and still count as parked.
 * Raise if a good park is reported as a miss. */
#define REACTION_HOME_TOLERANCE    50

/* Park the MIXER_ID servo at REACTION_HOME_PULSES.
 * Returns true only if it actually arrived within REACTION_HOME_TOLERANCE. */
bool ReactionMoveHome(ServoHandle *servo);

/* Same, for any absolute encoder count. */
bool ReactionMoveToPulses(ServoHandle *servo, int32_t target_pulses);

/* =========================================================
 DISPENSER (MicroID) SERVO — PARK AT HOME
 =========================================================
 * The same idea as the mixer park above, for the plate DispenserMove() turns:
 *
 *      DispenserMoveHome(&servo);              // -> DISPENSER_HOME_PULSES
 *      DispenserMoveToPulses(&servo, -371000); // -> any absolute count
 *
 * WHAT "HOME" MEANS: the position the plate was standing in when it was
 * LATCHED -- i.e. before the servo started rotating. The first
 * DispenserMoveHome() after power-up (or after any ServoHomeFixedCapture())
 * declares the current position as home and does NOT move; every call after
 * that drives back to exactly that latched ENCODER COUNT and verifies arrival.
 *
 * Because the latched value is an absolute count rather than a tracked angle,
 * repeated homing is exact and cannot drift, however many slot moves happen in
 * between. servo_home_fixed() delegates here, so there is only one notion of
 * home in the system.
 *
 * DispenserMoveHome() also re-datums the software angle tracker to 0 deg on
 * arrival, so AminoAcidSlots() and wash_solvents() measure from this position.
 *
 * To re-declare home mid-run -- e.g. at the top of a sequence, so the run's
 * own start position becomes its home -- call ServoHomeFixedCapture().
 ========================================================= */

/* Reference value only: an absolute encoder count read from the drive at the
 * fixed home on one occasion (Modbus reply 02 03 04 56 AE FF FA -> 0xFFFA56AE).
 * DispenserMoveHome() does NOT use this -- it latches the live start position
 * instead. Kept for DispenserMoveToPulses() callers that want that exact spot. */
#define DISPENSER_HOME_PULSES      (-371026)

/* SERVO_POS_SPEED used for the park move -- the same speed the named-slot
 * moves use, set explicitly so the result does not depend on what ran before. */
#define DISPENSER_HOME_SPEED       DISPENSER_SLOT_SPEED

/* Timeout for the park move. The plate can be most of a revolution away, so
 * this is deliberately generous. */
#define DISPENSER_HOME_TIMEOUT_MS  60000

/* How far off the target the plate may settle and still count as parked.
 * Raise if a good park is reported as a miss. */
#define DISPENSER_HOME_TOLERANCE   50

/* Return the MicroID plate to the latched home and re-datum the angle tracker.
 * The FIRST call latches the current position instead of moving. Returns true
 * only if it arrived within DISPENSER_HOME_TOLERANCE. */
bool DispenserMoveHome(ServoHandle *servo);

/* Same, for any absolute encoder count. Does NOT re-datum the angle tracker --
 * only DispenserMoveHome() knows the plate ended up at the 0 deg reference. */
bool DispenserMoveToPulses(ServoHandle *servo, int32_t target_pulses);

#endif /* INC_SERVO_H_ */
