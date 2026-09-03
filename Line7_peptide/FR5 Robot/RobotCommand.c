/*
 * RobotCommand.c  -- FAIRINO FR5 six-axis arm command layer (LINE 7 peptide)
 *
 * Ported from LINE 1 production (the identical robot). Uses the FAIRINO frame
 * protocol over TCP (port 8080). Replies are read INLINE right after each send
 * (no separate robot RX task). Console logging via UART_Print (USART3).
 */

#include "RobotCommand.h"

#include "cmsis_os.h"
#include "uart.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/err.h"
#include "RobotCoordinates.h"
#include "RobotMotion.h"   /* for MoveLinearServo() -- the Rtelligent rail (ID 0x03) */

extern struct netif gnetif;

int  robotSocket      = -1;
char statusFlag       = 0;
int  robotLastErrcode = 0;   /* last "robot errcode:NN" seen in a reply */

#define MOTION_TIMEOUT_MS         90000   /* absolute cap on one MoveJ finishing   */
#define MOTION_START_TIMEOUT_MS    2500   /* max wait to observe a move has STARTED */

/* A FAIRINO reply that contains "...robot errcode:NN..." means the controller
 * REJECTED the command (typically: not in AUTO mode, or an active fault).
 * Returns the code, or 0 if the command was accepted. */
static int replyErrcode(const char *reply)
{
	const char *p = strstr(reply, "errcode:");
	return p ? atoi(p + 8) : 0;
}

/* Human-readable hint for the FAIRINO reject codes we actually see on this cell.
 * NOTE: errcode 32 is NOT a mode problem -- it means the target itself is bad
 * (joint out of range / unreachable / joints inconsistent with the cartesian),
 * so the fix is the COORDINATES, not the pendant mode. */
static const char *robotErrcodeHint(int ec)
{
	switch (ec) {
	case 14:  return "not in AUTOMATIC mode, or an active fault / E-stop";
	case 31:  return "enable/motion refused -- fault or E-stop state";
	case 32:  return "INVALID TARGET -- joint out of range / pose unreachable / "
	                 "joints do not match the cartesian -> CHECK THE COORDINATES";
	case 101: return "MoveJ rejected -- tool/user/coord mismatch or bad target";
	default:  return "command rejected by controller";
	}
}

/* =========================================================
 CONNECT + ENABLE
 ========================================================= */
int connectRobot(void)
{
	struct sockaddr_in cobot_addr;

	if (robotSocket >= 0)
		return ERR_OK;                         /* already connected + enabled */

	/* Bounded waits (~5s each) so a missing cable can't hang the console. */
	int guard;
	for (guard = 0; !netif_is_up(&gnetif); guard++) {
		if (guard > 50) { UART_Print("NETIF not up\r\n"); return -1; }
		osDelay(100);
	}
	for (guard = 0; gnetif.ip_addr.addr == 0; guard++) {
		if (guard > 50) { UART_Print("No IP assigned\r\n"); return -1; }
		osDelay(100);
	}

	UART_Print("\r\n========== NETWORK INFO ==========\r\n");
	UART_Print("STM32 IP  : %s\r\n", ip4addr_ntoa(netif_ip4_addr(&gnetif)));
	UART_Print("NETMASK   : %s\r\n", ip4addr_ntoa(netif_ip4_netmask(&gnetif)));
	UART_Print("GATEWAY   : %s\r\n", ip4addr_ntoa(netif_ip4_gw(&gnetif)));
	UART_Print("==================================\r\n");

	robotSocket = lwip_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (robotSocket < 0) {
		UART_Print("Robot socket create fail errno=%d\r\n", errno);
		return -1;
	}

	/* Short RX/TX timeout: replies are small and immediate (matches LINE 1). */
	struct timeval timeout;
	timeout.tv_sec  = 2;
	timeout.tv_usec = 0;
	lwip_setsockopt(robotSocket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
	lwip_setsockopt(robotSocket, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

	memset(&cobot_addr, 0, sizeof(cobot_addr));
	cobot_addr.sin_family      = AF_INET;
	cobot_addr.sin_port        = htons(ROBOT_PORT);
	cobot_addr.sin_addr.s_addr = inet_addr(ROBOT_IP);

	UART_Print("Connecting Robot %s:%d ...\r\n", ROBOT_IP, ROBOT_PORT);

	if (lwip_connect(robotSocket, (struct sockaddr *)&cobot_addr, sizeof(cobot_addr)) < 0) {
		UART_Print("Robot connect fail errno=%d\r\n", errno);
		lwip_close(robotSocket);
		robotSocket = -1;
		return -1;
	}

	UART_Print("Robot Connected Successfully\r\n");

	/* The arm will not move until it is put in auto mode + enabled. */
	robotInitConfig();

	return ERR_OK;
}

void disconnectRobot(void)
{
	if (robotSocket >= 0) {
		/* Return the arm to the safe/idle state BEFORE closing the link, so the
		 * pendant LED goes from blue (auto + enabled) back to green (manual /
		 * disabled). This undoes what robotInitConfig() set on connect. Order
		 * matters: disable first, then drop to manual, THEN close -- after
		 * lwip_close() we can no longer talk to the controller. */
		UART_Print("\r\n[ROBOT] disabling + returning to manual...\r\n");

		sendCommand(90, CMD_ROBOT_ENABLE, "RobotEnable(0)", &statusFlag);   /* disable torque */
		osDelay(200);

		sendCommand(91, CMD_MODE, "Mode(1)", &statusFlag);                  /* auto -> manual  */
		osDelay(200);

		lwip_close(robotSocket);
		robotSocket = -1;
		UART_Print("Robot disabled + disconnected\r\n");
	}
}

/* =========================================================
 FRAME BUILD + STATUS INDEX
 ========================================================= */
void buildFrame(int instance, int cmdnum, const char *body, char *out, size_t outSize)
{
	int bodylen = (int)strlen(body);
	snprintf(out, outSize, "/f/bIII%dIII%dIII%dIII%sIII/b/f",
	         instance, cmdnum, bodylen, body);
}

int getStatusIndex(int cmdnum)
{
	switch (cmdnum) {
	case 1161: return 24;   /* GetRobotMotionStatus */
	case 2001: return 30;
	case 3005: return 18;
	default:   return -1;
	}
}

/* =========================================================
 DRAIN STALE RX  (keep replies aligned with requests)
 ========================================================= */
/* Discard any bytes already waiting in the socket RX buffer -- late or duplicate
 * replies, or async status the controller pushed on its own. Without this, a
 * straggler frame left over from a previous exchange gets consumed as the reply
 * to the NEXT command, so the controller's answers drift one command behind and
 * every command after the first is mis-judged (enable "rejected", MoveJ
 * "rejected", ...). Non-blocking: stops as soon as the buffer is empty. */
static void drainRobotSocket(void)
{
	if (robotSocket < 0)
		return;

	char scratch[128];
	for (int guard = 0; guard < 64; guard++) {
		int n = lwip_recv(robotSocket, scratch, sizeof(scratch), MSG_DONTWAIT);
		if (n <= 0)
			break;                 /* EWOULDBLOCK / closed -> nothing left */
	}
}

/* =========================================================
 SEND FRAME + INLINE RECEIVE
 ========================================================= */
int sendCommand(int instance, int cmdnum, const char *body, char *flag)
{
	static char frame[RP_MAX_FRAME_SIZE];
	static char recvbuf[RP_MAX_RECV_SIZE];

	if (robotSocket < 0)
		return -1;

	/* Read THIS command's reply, not a leftover one from the last exchange. */
	drainRobotSocket();

	buildFrame(instance, cmdnum, body, frame, sizeof(frame));

	UART_Print("[TX cmd %d] %s\r\n", cmdnum, body);

	int sent = lwip_send(robotSocket, frame, strlen(frame), 0);
	if (sent < 0) {
		UART_Print("Send error %d (frame: %s)\r\n", sent, frame);
		return sent;
	}

	int recvd = lwip_recv(robotSocket, recvbuf, sizeof(recvbuf) - 1, 0);
	if (recvd <= 0) {
		UART_Print("[RX cmd %d] <no reply, recv=%d>\r\n", cmdnum, recvd);
		return (recvd == 0) ? -1 : recvd;
	}
	recvbuf[recvd] = '\0';

	/* Show exactly what the controller returned -- this is what tells us
	 * whether a command (enable/mode/MoveJ) was accepted or rejected. */
	UART_Print("[RX cmd %d] %s\r\n", cmdnum, recvbuf);

	/* Controller rejected it (not in AUTO mode / active fault). Return a
	 * distinct code so callers abort instead of polling forever. */
	int ec = replyErrcode(recvbuf);
	if (ec != 0) {
		robotLastErrcode = ec;
		return -2;
	}

	int idx = getStatusIndex(cmdnum);
	if (flag && idx >= 0 && recvd > idx)
		*flag = recvbuf[idx];

	return ERR_OK;
}

/* =========================================================
 INIT / ENABLE SEQUENCE  (must run before any move)
 ========================================================= */
int robotInitConfig(void)
{
	UART_Print("\r\n[ROBOT] init/enable...\r\n");
	int modeOk = (sendCommand(1, CMD_MODE, "Mode(0)", &statusFlag) == ERR_OK);
	osDelay(100);
	sendCommand(2, CMD_SET_ERR_HOLD, "SetErrStateHoldEnable(1)", &statusFlag);
	osDelay(100);
	int enOk = (sendCommand(3, CMD_ROBOT_ENABLE, "RobotEnable(1)", &statusFlag) == ERR_OK);
	osDelay(100);
	sendCommand(4, CMD_SET_SPEED, "SetSpeed(25)", &statusFlag);
	osDelay(100);
	sendCommand(7, CMD_SET_CUST_SPEED, "SetCustSpeedManualToAuto(1,100)", &statusFlag);
	osDelay(100);
	sendCommand(8, CMD_SET_OACC_SCALE, "SetOaccScale(25)", &statusFlag);
	osDelay(100);
	sendCommand(9, CMD_SET_MAX_CART_VEL_ACC, "SetMaxCartVelAcc(999,2999)", &statusFlag);
	osDelay(100);
	//sendCommand(11, CMD_MOVELINEAR, "SetMaxCartVelAcc(999,2999)", &statusFlag);
		//osDelay(100);


	if (modeOk && enOk) {
		UART_Print("[ROBOT] init done -- mode + enable OK, ready to move.\r\n");
		return ERR_OK;
	}

	UART_Print("\r\n*** ROBOT NOT READY (Mode=%s  Enable=%s) ***\r\n"
	           "The FR10 rejected these -- it is NOT in AUTOMATIC mode or has a\r\n"
	           "fault. On the FAIRINO WebApp (open http://%s in a browser):\r\n"
	           "  1. Clear/RESET all faults/alarms.\r\n"
	           "  2. Switch to AUTOMATIC mode  (end-effector LED must be BLUE).\r\n"
	           "  3. Release the E-stop; make sure drag/teach mode is OFF.\r\n"
	           "Then re-run menu 0 -> 3.  No re-flash needed.\r\n",
	           modeOk ? "OK" : "REJECTED", enOk ? "OK" : "REJECTED", ROBOT_IP);
	return -1;
}

/* =========================================================
 WAIT MOTION COMPLETE  (poll GetRobotMotionStatus, byte[24]=='1')
 ========================================================= */
int waitTillMotionComplete(void)
{
	char flag = 0;
	uint32_t start = osKernelSysTick();

	/* Phase 1 -- confirm the move actually STARTED.
	 * A large MoveJ (e.g. HOME<->dispense station, ~95 deg on J1) can still
	 * report byte[24]=='1' (idle) for a few hundred ms after it was accepted.
	 * If we jumped straight to the "wait for idle" loop we would read that stale
	 * idle byte, declare "Motion complete" immediately, and fire the next MoveJ
	 * while the arm is still moving -- the controller then rejects the busy
	 * command, so only the FIRST pose of a sequence ever runs. Poll until we see
	 * the non-idle (moving) state, bounded so a genuinely short move that
	 * finishes before we look can't hang here. */
	while ((osKernelSysTick() - start) < MOTION_START_TIMEOUT_MS) {
		int r = sendCommand(6, CMD_GET_ROBOT_MOTION_STATUS, "GetRobotMotionStatus()", &flag);
		if (r == -2) {
			UART_Print("Status query rejected (errcode:%d) -- robot not running, aborting.\r\n",
			           robotLastErrcode);
			return -1;
		}
		if (r != ERR_OK)
			return -1;
		if (flag != '1')                 /* moving -- the move is underway */
			break;
		osDelay(100);
	}

	/* Phase 2 -- wait for motion complete (byte[24]=='1' == idle again). */
	do {
		if ((osKernelSysTick() - start) > MOTION_TIMEOUT_MS) {
			UART_Print("Robot motion timeout\r\n");
			return -1;
		}
		int r = sendCommand(6, CMD_GET_ROBOT_MOTION_STATUS, "GetRobotMotionStatus()", &flag);
		if (r == -2) {
			UART_Print("Status query rejected (errcode:%d) -- robot not running, aborting.\r\n",
			           robotLastErrcode);
			return -1;
		}
		if (r != ERR_OK)
			return -1;
		osDelay(500);
	} while (flag != '1');

	UART_Print("Motion complete\r\n");
	return ERR_OK;
}

/* =========================================================
 MOVE (build MoveJ body, send, then wait)
 ========================================================= */
static int robotMoveJBody(const char *body, const char *label)
{
	static int instance = 100;

	if (connectRobot() != ERR_OK)
		return -1;

	UART_Print("\r\n[ROBOT] MoveJ -> %s\r\n", label);

	int r = sendCommand(instance++, CMD_MOVEJ, body, &statusFlag);
	if (r == -2) {
		UART_Print("\r\n*** MoveJ REJECTED by controller (errcode:%d) ***\r\n"
		           "  Likely cause: %s\r\n"
		           "  Fix on the robot (WebApp http://%s) then retry -- no re-flash.\r\n",
		           robotLastErrcode, robotErrcodeHint(robotLastErrcode), ROBOT_IP);
		return -1;
	}
	if (r != ERR_OK) {
		UART_Print("Robot MoveJ send fail\r\n");
		return -1;
	}

	/* Small settle, then waitTillMotionComplete() confirms the move actually
	 * started (see there) before waiting for it to finish -- so a stale "idle"
	 * status byte can no longer be mistaken for an instant "Motion complete". */
	osDelay(200);
	return waitTillMotionComplete();
}

int robotMoveJStr(const char *pose12)
{
	char body[RP_MAX_FRAME_SIZE];
	snprintf(body, sizeof(body), "MoveJ(%s,%s)", pose12, DEFAULT_EXTRA);
	return robotMoveJBody(body, pose12);
}

int moveRobotTo(PositionID id)
{
	if (id < 0 || id >= NUM_POS) {
		UART_Print("Invalid Position %d\r\n", id);
		return -1;
	}
	char body[RP_MAX_FRAME_SIZE];
	snprintf(body, sizeof(body), "MoveJ(%s,%s)", positions[id], DEFAULT_EXTRA);

	char label[16];
	snprintf(label, sizeof(label), "POS %d", id);
	return robotMoveJBody(body, label);
}

/* =========================================================
 LINEAR RAIL SERVO -- console helper (menu 5 -> 7)
 =========================================================
 * Prompts for a signed number of turns on the serial console and drives the
 * Rtelligent linear rail (ROBOT_LINEAR_ID = 0x03) that far. MoveLinearServo()
 * auto-configures the rail on first use, so nothing else has to be called
 * first. + = forward, - = reverse, 0 = no move. Independent of the FR5 TCP
 * link -- the rail is on the Modbus servo bus, so the robot need not be
 * connected. */
void robotLinearMoveConsole(void)
{
	char inbuf[32];
	int  turns;

	UART_Print("\r\nEnter turns (+ forward / - reverse) : ");
	UART_ReadLine(inbuf, sizeof(inbuf));
	turns = atoi(inbuf);

	UART_Print("Moving rail %d turn(s)...\r\n", turns);

	if (MoveLinearServo(turns))
		UART_Print("Rail move complete.\r\n");
	else
		UART_Print("Rail move failed / timeout.\r\n");
}


/* =========================================================
 SELF-TESTS (menu 0 -> 3): lightweight proven commands
 ========================================================= */
void testRobotVersion(void)
{
	char flag = 0;
	if (sendCommand(6, CMD_GET_ROBOT_MOTION_STATUS, "GetRobotMotionStatus()", &flag) == ERR_OK)
		UART_Print("PING ok (motion-status byte='%c')\r\n", flag ? flag : '?');
	else
		UART_Print("PING FAILED\r\n");
}

void testRobotMode(void)
{
	if (sendCommand(1, CMD_MODE, "Mode(0)", &statusFlag) == ERR_OK)
		UART_Print("Mode(0) sent (auto)\r\n");
	else
		UART_Print("Mode set FAILED\r\n");
}

void testRobotAlarm(void)
{
	if (sendCommand(3, CMD_ROBOT_ENABLE, "RobotEnable(1)", &statusFlag) == ERR_OK)
		UART_Print("RobotEnable(1) sent (clears + enables)\r\n");
	else
		UART_Print("Enable FAILED\r\n");
}
