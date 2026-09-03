/*
 * RobotCommand.h
 *
 *  FAIRINO FR5 six-axis arm command layer for LINE 7 (peptide).
 *  Ported from LINE 1 production (the identical robot). Talks to the FR5
 *  controller over a raw TCP socket (lwIP) using the FAIRINO frame protocol:
 *
 *      /f/bIII<instance>III<cmdnum>III<bodylen>III<body>III/b/f
 *
 *  where <body> is e.g. "MoveJ(<12 decimal pose>,<extra>)". Replies are read
 *  inline (synchronously) right after each send -- there is NO separate robot
 *  RX task (the RX task in freertos.c is idle for the robot).
 *
 *  DEPENDENCIES (enabled in the .ioc): FreeRTOS (CMSIS-OS v1) + LWIP.
 *
 *  NOTE: this file was missing from the last upload -- the content that
 *  should live here (these declarations) was uploaded mislabeled under
 *  RobotCoordinates.h's name instead, which held the PositionID enum
 *  content that actually belongs in RobotCoordinates.h. Restored here to
 *  match RobotCommand.c exactly.
 */

#ifndef FR5_ROBOT_ROBOTCOMMAND_H_
#define FR5_ROBOT_ROBOTCOMMAND_H_

#include "main.h"
#include "stdint.h"
#include "stddef.h"

#include "RobotCoordinates.h"

/* ================= FR5 controller network endpoint ================= */
/* Matches LINE 1 (the identical robot): FAIRINO frame protocol on port 8080.
 * The STM32's own static IP/mask/gw are set in the LWIP config. */
#define ROBOT_IP            "192.168.58.2"
#define ROBOT_PORT          8080

/* ================= Framing / buffers ================= */
#define RP_MAX_FRAME_SIZE   1024
#define RP_MAX_RECV_SIZE    1024

/* ================= FAIRINO command numbers (as used by LINE 1) ================= */
#define CMD_MODE                     303   /* Mode(0) = auto            */
#define CMD_ROBOT_ENABLE             302   /* RobotEnable(1)            */
#define CMD_SET_SPEED                206   /* SetSpeed(pct)             */
#define CMD_SET_ERR_HOLD             1206  /* SetErrStateHoldEnable(1)  */
#define CMD_SET_CUST_SPEED           750   /* SetCustSpeedManualToAuto  */
#define CMD_SET_OACC_SCALE           640   /* SetOaccScale(25)          */
#define CMD_SET_MAX_CART_VEL_ACC     849   /* SetMaxCartVelAcc(...)     */
#define CMD_MOVEJ                    201   /* MoveJ(pose,extra)         */
#define CMD_GET_ROBOT_MOTION_STATUS 1161  /* GetRobotMotionStatus()    */
#define CMD_MOVELINEAR               358
#define CMD_MOVEAXES                 359

/* ================= Public API ================= */
int  connectRobot(void);          /* connect + enable/init (idempotent)      */
void disconnectRobot(void);
int  robotInitConfig(void);       /* Mode/Enable/Speed... (called by connect) */

/* Low-level frame send + inline receive. If the reply carries a status byte
 * for this cmdnum, *flag is updated. Returns ERR_OK / <0. */
int  sendCommand(int instance, int cmdnum, const char *body, char *flag);
void buildFrame(int instance, int cmdnum, const char *body, char *out, size_t outSize);
int  getStatusIndex(int cmdnum);

/* Motion (block until the controller reports motion complete, or timeout). */
int  moveRobotTo(PositionID id);                 /* MoveJ to a taught pose      */
int  robotMoveJStr(const char *pose12);          /* MoveJ to an ad-hoc pose str */
int  waitTillMotionComplete(void);               /* poll GetRobotMotionStatus   */

/* Console self-tests. */
void testRobotVersion(void);
void testRobotMode(void);
void testRobotAlarm(void);

void robotLinearMoveConsole(void);   /* menu helper: prompt turns -> move the rail */

/* Provided by RobotCommand.c */
extern int  robotSocket;
extern char statusFlag;

#endif /* FR5_ROBOT_ROBOTCOMMAND_H_ */
