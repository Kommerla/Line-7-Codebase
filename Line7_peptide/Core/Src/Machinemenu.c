/*
 * MachineMenu.c
 *
 *  Numbered "PEPTIDE MACHINE MENU" console (Mixing Servo / Amino Acids
 *  Platform Servo / Dispensing Stepper / Drain Valve / Robot Functions).
 *
 *  Split out of main.c into its own module so the menu implementation
 *  isn't buried in main.c alongside HAL/CubeMX init code. Called from
 *  StartAppMainTask() in freertos.c once the RTOS scheduler + LWIP are
 *  up (code placed directly in main()'s body after osKernelStart() never
 *  runs, since the FreeRTOS scheduler doesn't return there -- see
 *  freertos.c for the actual call site).
 */

#include "MachineMenu.h"

#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "uart.h"
#include "servo.h"
#include "Load.h"
#include "stepper.h"
#include "timer_app.h"   /* micros() -- TIM2 microsecond counter */
#include "fmoc.h"        /* Fmoc UV detector on UART5 (RS232/MAX3232) */
//#include "RobotConsole.h"
#include "load_cell.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* StepperMotor / Stepper_Init / Stepper_Step now live in stepper.h (moved from
 * new_stepper.h). stepper.h is already included above, so nothing extra is
 * needed here; case 12 below uses them. */

#include "RobotCommand.h"
#include "RobotCoordinates.h"
#include "RobotMotion.h"

/* servo/loadcell are initialised in main.c (ServoInit()/LoadCell_Init())
 * and also used by RobotMotion.c -- they stay defined there; this file
 * only needs to see them. */
extern ServoHandle    *servo;
extern LoadCellHandle *loadcell;

/* SelectedID is used only transiently within the menu below, but was
 * already file-scope in main.c (harmless either way) -- keep it there to
 * avoid touching anything outside the menu's own concerns. */
extern uint8_t SelectedID;

/* Default staging percentages used by LoadCellLoadCheck(): 30% coarse,
 * 60% fine, 100% (target) super-fine. Menu-only, so they live here now
 * instead of main.c. */
#define LOADCELL_COARSE_PCT 30
#define LOADCELL_FINE_PCT   60

/* Used by "3. Dispense by Weight" in the STEPPER MENU (case 3 below), which
 * calls LoadCellStepperDispense() (Load.c). STEP_INCREMENT is how many
 * motor steps get jogged between each weight-feedback check -- small on
 * purpose, so the motor doesn't overshoot the target between reads.
 * TIMEOUT_MS is a safety ceiling in case the target is never reached. */
#define STEPPER_DISPENSE_STEP_INCREMENT   1
#define STEPPER_DISPENSE_TIMEOUT_MS       120000UL

static int exitMenu1 = 0;
static int exitMenu2 = 0;
static int exitMenu3 = 0;
static int exitMenu4 = 0;
static int exitMenu6 = 0;
static int exitMenu5 = 0;
static int exitMenu7 = 0;
static int exitMenu8 = 0;
static int exitMenu9 = 0;

static char    buf[50];
static int     choice;

/* Was local to main() -- promoted to file scope because the stepper menu
 * code that uses these (case 3, "Dispensing Stepper") now runs inside
 * MachineMenuTask() instead of inline in main(). static, because
 * stepper.c already has its own global rxBuffer[]/msg[] and, with
 * -fno-common (default on modern GCC), two non-static globals with the
 * same name in different .c files is a link error ("multiple definition"),
 * not just a naming clash the linker quietly merges like it used to. */
static char rxBuffer[40];
static char msg[30];

      /* Parameters with defaults shown to the user */
static int      offset_deg     = REACTION_DEFAULT_OFFSET;
static int      swing_deg      = REACTION_DEFAULT_DEG;
static uint16_t speed          = REACTION_DEFAULT_SPEED;
static int      cycles         = REACTION_DEFAULT_CYCLES;
static int      cycle_delay_ms = 0;      /* 0 = no delay between cycles */

static int speed_1 = 0, direction_1 = 0, steps_1 = 0;


#define STEPPER_RAMP_START_STEPS   1
#define STEPPER_RAMP_INCREMENT     1
#define STEPPER_RAMP_MAX_STEPS     20

/* No longer static: sequence() in RobotMotion.c calls this to do the 10 g
 * liquid dose, so it must have external linkage. Declared there. */
bool LoadCellStepperDispenseRamped(LoadCellHandle *loadcell,
		uint16_t targetWeight, int direction, int stepSpeed) {
	uint16_t coarse, fine, sfine;
	uint16_t currentWeight = 0;
	uint16_t baseline = 0;
	uint16_t added = 0;
	bool targetReached = false;
	int stepCount = STEPPER_RAMP_START_STEPS;
	bool feedbackWarned = false;

	(void) direction;   /* dose direction/speed are fixed below (0 / 100) */
	(void) stepSpeed;

	if (targetWeight == 0) {
		UART_Print("[STEPPER ERROR] Invalid Target Weight\r\n");
		return false;
	}

	/* 1. reset pulse pair */
	LoadCellStartCollection(loadcell);
	LoadCellStopCollection(loadcell);

	/* 2. coarse(30%)/fine(60%)/superfine(100%) setpoints */
	coarse = (uint16_t) (((uint32_t) targetWeight * 30) / 100);
	fine   = (uint16_t) (((uint32_t) targetWeight * 60) / 100);
	sfine  = targetWeight;

	if (!LoadCellSetCoarse(loadcell, coarse)) {
		return false;
	}
	osDelay(200);

	if (!LoadCellSetFine(loadcell, fine)) {
		return false;
	}
	osDelay(200);

	if (!LoadCellSetSuperFine(loadcell, sfine)) {
		return false;
	}
	osDelay(200);

	/* 3. begin */
	LoadCellStartCollection(loadcell);
	osDelay(500);

	/* 3b. BASELINE (tare). Dose by weight ADDED since this baseline, not by
	 * the absolute reading. Without it, powder + the 30000-step plunge that
	 * runs before this call already satisfy currentWeight >= targetWeight,
	 * so the loop broke out immediately and the stepper never stepped during
	 * the normal dose. Empty pan -> baseline ~0 -> menu use unchanged. */
	{
		uint32_t sum = 0;
		int      valid = 0;
		for (int k = 0; k < 5; k++) {
			uint16_t w = 0;
			if (LoadCellReadWeight(loadcell, &w)) {
				sum += w;
				valid++;
			}
			osDelay(200);
		}
		baseline = (valid > 0) ? (uint16_t)(sum / valid) : 0;
	}
	UART_Print("Baseline = %u (target to ADD = %u)\r\n", baseline, targetWeight);

	/* 4. real-time weight + RAMPED stepper loop -- WEIGHT IS THE ONLY THING
	 * THAT ENDS THIS LOOP NOW. The feedback pin used to also break the
	 * loop, but on this rig it can go HIGH (e.g. the vibratory feeder's
	 * own coarse/fine/superfine setpoints being satisfied) well before the
	 * operator's actual target weight is reached, which was cutting the
	 * dispense short and reporting "failed" even though nothing was
	 * actually wrong. The pin is still read and logged once, purely for
	 * visibility, but it no longer stops anything. */
	for (;;) {
		if (!LoadCellReadWeight(loadcell, &currentWeight)) {
			UART_Print("[STEPPER ERROR] Weight read failed, retrying\r\n");
		}

		added = (currentWeight > baseline)
		        ? (uint16_t)(currentWeight - baseline) : 0;

		UART_Print("\r\nWeight = %u (added %u / %u, step count = %d)\r\n",
		           currentWeight, added, targetWeight, stepCount);

		if (added >= targetWeight) {
			UART_Print("\r\nTarget reached\r\n");
			targetReached = true;
			break;
		}

		if (!feedbackWarned && HAL_GPIO_ReadPin(loadcell->feedbackPort,
				loadcell->feedbackPin) != GPIO_PIN_RESET) {
			UART_Print("[STEPPER WARNING] Feedback pin went HIGH before target "
					"weight reached -- ignoring it and continuing\r\n");
			feedbackWarned = true;
		}

		/* Dose stepper: fixed direction 0 (down) and speed 100 (fast) -- the
		 * passed-in direction/stepSpeed were making this crawl (speed 1 ->
		 * osDelay(100) per micro-step, so it looked stopped). This matches the
		 * speed that works from the dispense-stepper menu. */
		stepperData(stepCount, 0, 200);

		if (stepCount < STEPPER_RAMP_MAX_STEPS) {
			stepCount += STEPPER_RAMP_INCREMENT;
		}
	}

	/* 5. stop, unconditionally */
	LoadCellStopCollection(loadcell);

	UART_Print("\r\nCollection Done\r\n");

	return targetReached;
}

/* USER CODE BEGIN MachineMenuTask */
/**
  * @brief  Numbered "PEPTIDE MACHINE MENU" console (Mixing Servo /
  *         Amino Acids Platform Servo / Dispensing Stepper / Drain Valve /
  *         Robot Functions).
  *
  *         NOTE: this switch-based menu used to sit directly below
  *         osKernelStart() in main(), which is unreachable once the
  *         FreeRTOS scheduler starts -- so on real hardware it never
  *         actually ran. It has been moved into its own task so it
  *         executes; see StartAppMainTask() in freertos.c, which now
  *         calls MachineMenuTask() instead of calling robotConsoleTask()
  *         directly. "5. Robot Functions" drops into the same typed FR5
  *         command shell as the standalone robot console (RobotConsole.c) --
  *         type 'exit' there to come back to this menu.
  * @param  argument: Not used
  * @retval None
  */
void MachineMenuTask(void const *argument)
{
  (void)argument;

  UART_Print("CHECKPOINT: MachineMenuTask entered\r\n");

  for (;;)
  {
	  /* ---- Print Menu ---- */
	 	  UART_Print("\r\n "
	 			  "========================\r\n"
	 			  "   PEPTIDE MACHINE MENU\r\n   "
	 			  "========================\r\n"
	 			  "1. Mixing Servo\r\n "
	 			  "2. Amino Acids Platform Servo\r\n "
	 			  "3. Dispensing Stepper\r\n"
	 			  "4. Drain Valve Functions\r\n"
	 			  "5. Robot Functions\r\n"
	 			  "6. Vibrator Motor Control\r\n"
	 			  "7. Loadcell Test\r\n"
	 			  "9. Read Address of Devices\r\n"
	 			  "10.DC Motor Working\r\n"
	 			  "11.Wash Solvent Sequence\r\n"
	 			  "12. Run Octertoide Sequence\r\n"
	 			  "13. Read RS232 device (UART4 PC10/PC11)\r\n"
	 			  "14. Pneumatic Controls\r\n"
	 			  "15. Cleavage Module\r\n"
	 			  "16. Fmoc UV Detector (USART6 RS232)\r\n"
	 			  "17. USART6 monitor (print everything received)\r\n"
	 			  "18. USART6 TX test (10 s solid, watch the TX LED)\r\n"
	 			  "19. USART6 baud sweep (garbled replies?)\r\n"
	 			  "20. Is anything driving the RX pin (PG9)?\r\n"
	 			  "21. USART6 terminal (send + receive commands)\r\n"
	 			  "-------------------------\r\n"
	 			  "Enter Choice:\r\n "
	 			  );
	 	  UART_ReadLine(buf, sizeof(buf));
	 	  switch(atoi(buf)) {

	 	  case 1:
	 		  exitMenu1 = 0;
	 		  SelectedID = MIXER_ID;
	 		  UART_Print("\r\nMixer Servo Selected.\r\n");
	 		  while(!exitMenu1){
	 		  		  UART_Print(
	 		  		  	              "\r\n"
	 		  		  	              "========================================\r\n"
	 		  		  	              "  SERVO MENU\r\n"
	 		  		  	              "========================================\r\n"
	 		  		  	              "  1. Run Reaction Move\r\n"
	 		  		  	              "  2. Stop Servo\r\n"
	 		  		  	              "  3. Set Speed Only\r\n"
	 		  		  	              "  4. Homing Start\r\n"
	 		  		  	              "  5. Set Current Position as Home\r\n"
	 		  		  	              "  6. Reset Fault\r\n"
	 		  		  	              "  7. Read Current Position\r\n"
	 		  		  	              "  8. Reset Diagnostics\r\n"
	 		  		  	              "  9. Print Diagnostics\r\n"
	 		  		  	              "  10. Exit\r\n"
	 		  		  	              "  11. Reaction Go To Home\r\n"
	 		  		  	              "----------------------------------------\r\n"
	 		  		  	              "Enter choice : ");

	 		  		  	          UART_ReadLine(buf, sizeof(buf));
	 		  		  	          choice= atoi(buf);

	 		  		  	          switch (choice){

	 		  		  	          /* ------------------------------------------------
	 		  		  	             CASE 1 : RUN REACTION MOVE
	 		  		  	             ------------------------------------------------ */
	 		  		  	          case 1:
	 		  		  	          {
	 		  		  	              UART_Print(
	 		  		  	                  "\r\n--- Reaction Move Setup ---\r\n"
	 		  		  	                  "Current defaults :\r\n"
	 		  		  	                  "  Offset deg     = %d\r\n"
	 		  		  	                  "  Swing  deg     = %d\r\n"
	 		  		  	                  "  Speed          = %u\r\n"
	 		  		  	                  "  Cycles         = %d\r\n"
	 		  		  	                  "  Cycle delay ms = %d\r\n\r\n",
	 		  		  	                  offset_deg, swing_deg,
	 		  		  	                  (unsigned)speed, cycles, cycle_delay_ms);

	 		  		  	              /* Offset degree */
	 		  		  	              UART_Print("Enter offset degree (e.g. 90)  : ");
	 		  		  	              UART_ReadLine(buf, sizeof(buf));
	 		  		  	              if (buf[0] != '\0') offset_deg = atoi(buf);

	 		  		  	              /* Swing degree. 0 is refused here rather than
	 		  		  	               * further down: a 0 swing makes every stroke a
	 		  		  	               * no-op, so the mixer does the offset move and
	 		  		  	               * then just sits there. */
	 		  		  	              UART_Print("Enter swing  degree (e.g. 360) : ");
	 		  		  	              UART_ReadLine(buf, sizeof(buf));
	 		  		  	              if (buf[0] != '\0') {
	 		  		  	                  int entered = atoi(buf);
	 		  		  	                  if (entered == 0)
	 		  		  	                      UART_Print("Swing of 0 deg would not move -- "
	 		  		  	                                 "keeping %d.\r\n", swing_deg);
	 		  		  	                  else
	 		  		  	                      swing_deg = entered;
	 		  		  	              }

	 		  		  	              /* Speed */
	 		  		  	              UART_Print("Enter speed  (e.g. 300)        : ");
	 		  		  	              UART_ReadLine(buf, sizeof(buf));
	 		  		  	              if (buf[0] != '\0') speed = (uint16_t) atoi(buf);

	 		  		  	              /* Cycles */
	 		  		  	              UART_Print("Enter cycles (e.g. 10)         : ");
	 		  		  	              UART_ReadLine(buf, sizeof(buf));
	 		  		  	              if (buf[0] != '\0') cycles = atoi(buf);

	 		  		  	              /* Cycle delay */
	 		  		  	              UART_Print("Enter cycle delay ms (e.g. 3000) : ");
	 		  		  	              UART_ReadLine(buf, sizeof(buf));
	 		  		  	              if (buf[0] != '\0') cycle_delay_ms = atoi(buf);

	 		  		  	              /* Direction. The drive turns FORWARD for a
	 		  		  	               * positive angle and REVERSE for a negative one,
	 		  		  	               * and REVERSE is the anticlockwise side -- which
	 		  		  	               * is why a plain positive entry always went
	 		  		  	               * anticlockwise. Choosing clockwise simply flips
	 		  		  	               * the sign of the angles handed to ReactionMove,
	 		  		  	               * so the whole move mirrors. */
	 		  		  	              int dir_choice = 2;     /* default anticlockwise */

	 		  		  	              UART_Print("Enter direction (1 = Clockwise, "
	 		  		  	                         "2 = Anticlockwise) : ");
	 		  		  	              UART_ReadLine(buf, sizeof(buf));
	 		  		  	              if (buf[0] != '\0') dir_choice = atoi(buf);

	 		  		  	              int run_offset_deg = offset_deg;
	 		  		  	              int run_swing_deg  = swing_deg;

	 		  		  	              if (dir_choice == 1) {      /* clockwise */
	 		  		  	                  run_offset_deg = -offset_deg;
	 		  		  	                  run_swing_deg  = -swing_deg;
	 		  		  	              }

	 		  		  	              UART_Print(
	 		  		  	                  "\r\nStarting Reaction Move ...\r\n"
	 		  		  	                  "  Servo ID       = 0x%02X\r\n"
	 		  		  	                  "  Direction      = %s\r\n"
	 		  		  	                  "  Offset deg     = %d\r\n"
	 		  		  	                  "  Swing  deg     = %d\r\n"
	 		  		  	                  "  Speed          = %u\r\n"
	 		  		  	                  "  Cycles         = %d\r\n"
	 		  		  	                  "  Cycle delay ms = %d\r\n\r\n",
	 		  							  SelectedID,
	 		  		  	                  (dir_choice == 1) ? "CLOCKWISE"
	 		  		  	                                    : "ANTICLOCKWISE",
	 		  		  	                  run_offset_deg, run_swing_deg,
	 		  		  	                  (unsigned)speed, cycles, cycle_delay_ms);

	 		  		  	              /* SPEED-mode pendulum. Same swing/cycle
	 		  		  	               * semantics as ReactionMove(), but the drive
	 		  		  	               * runs in velocity control and each stroke ends
	 		  		  	               * on encoder feedback, so the turnarounds have
	 		  		  	               * no position re-arm between them.
	 		  		  	               * ReactionMove() is still there if the
	 		  		  	               * position-mode version is ever wanted back. */
	 		  		  	              ReactionMoveContinuous(servo, SelectedID,
	 		  		  	                           run_offset_deg, run_swing_deg,
	 		  		  	                           speed, cycles, cycle_delay_ms);

	 		  		  	              UART_Print("\r\nReaction Move Done.\r\n");
	 		  		  	          }
	 		  		  	          break;

	 		  		  	          /* ------------------------------------------------
	 		  		  	             CASE 2 : STOP SERVO
	 		  		  	             ------------------------------------------------ */
	 		  		  	          case 2:
	 		  		  	          {
	 		  		  	        	ServoWrite16Ack(servo, SelectedID, SERVO_MOTION_CMD, SERVO_STOP);
	 		  		  	              HAL_Delay(500);
	 		  		  	              UART_Print("\r\nServo Stopped.\r\n");
	 		  		  	          }
	 		  		  	          break;

	 		  		  	          /* ------------------------------------------------
	 		  		  	             CASE 3 : SET SPEED ONLY
	 		  		  	             ------------------------------------------------ */
	 		  		  	          case 3:
	 		  		  	          {
	 		  		  	              UART_Print("\r\nEnter new speed : ");
	 		  		  	              UART_ReadLine(buf, sizeof(buf));

	 		  		  	              if (buf[0] != '\0') {
	 		  		  	                  speed = (uint16_t) atoi(buf);
	 		  		  	              ServoWrite16Ack(servo, SelectedID, SERVO_POS_SPEED, speed);
	 		  		  	                  HAL_Delay(500);
	 		  		  	                  UART_Print("Speed set to %u\r\n", (unsigned)speed);
	 		  		  	              } else {
	 		  		  	                  UART_Print("No value entered. Speed unchanged.\r\n");
	 		  		  	              }
	 		  		  	          }
	 		  		  	          break;

	 		  		  	          /* ------------------------------------------------
	 		  		  	             CASE 4 : HOMING START
	 		  		  	             ------------------------------------------------ */
	 		  		  	          case 4:
	 		  		  	          {
	 		  		  	        	ServoWrite16Ack(servo, SelectedID,
	 		  		  	                           SERVO_HOME_TRIGGER, SERVO_HOMING_START);
	 		  		  	              HAL_Delay(500);
	 		  		  	              UART_Print("\r\nHoming started. Waiting...\r\n");

	 		  		  	              bool done = ServoWaitMotionComplete(servo, SelectedID,
	 		  		  	                              READ_CURRENT_POS, REACTION_MOTION_TIMEOUT);

	 		  		  	          ServoWrite16Ack(servo, SelectedID, SERVO_MOTION_CMD, SERVO_STOP);
	 		  		  	              HAL_Delay(500);

	 		  		  	              if (done)
	 		  		  	                  UART_Print("Homing Complete.\r\n");
	 		  		  	              else
	 		  		  	                  UART_Print("Homing Timeout.\r\n");
	 		  		  	          }
	 		  		  	          break;

	 		  		  	          /* ------------------------------------------------
	 		  		  	             CASE 5 : SET CURRENT POSITION AS HOME
	 		  		  	             ------------------------------------------------ */
	 		  		  	          case 5:
	 		  		  	          {
	 		  		  	        	ServoWrite16Ack(servo, SelectedID,
	 		  		  	                           SERVO_HOME_TRIGGER, SERVO_CURRENT_HOME);
	 		  		  	              HAL_Delay(500);
	 		  		  	              UART_Print("\r\nCurrent position set as home.\r\n");
	 		  		  	          }
	 		  		  	          break;

	 		  		  	          /* ------------------------------------------------
	 		  		  	             CASE 6 : RESET FAULT
	 		  		  	             ------------------------------------------------ */
	 		  		  	          case 6:
	 		  		  	          {
	 		  		  	        	ServoWrite16Ack(servo, SelectedID, FAULT_RESET, SET_HIGH);
	 		  		  	              HAL_Delay(500);
	 		  		  	              UART_Print("\r\nFault Reset sent.\r\n");
	 		  		  	          }
	 		  		  	          break;

	 		  		  	          /* ------------------------------------------------
	 		  		  	             CASE 7 : READ CURRENT POSITION
	 		  		  	             ------------------------------------------------ */
	 		  		  	          case 7:
	 		  		  	          {
	 		  		  	              int32_t pos = 0;

	 		  		  	              bool ok = ServoSafeReadPosition(servo, SelectedID,
	 		  		  	                                              READ_CURRENT_POS, &pos);
	 		  		  	              if (ok)
	 		  		  	                  UART_Print("\r\nCurrent Position = %ld pulses\r\n",
	 		  		  	                               (long)pos);
	 		  		  	              else
	 		  		  	                  UART_Print("\r\nPosition Read Failed.\r\n");
	 		  		  	          }
	 		  		  	          break;

	 		  		  	          /* ------------------------------------------------
	 		  		  	             CASE 8 : RESET DIAGNOSTICS
	 		  		  	             ------------------------------------------------ */
	 		  		  	          case 8:
	 		  		  	          {
	 		  		  	              ServoResetDiagnostics(servo);
	 		  		  	              UART_Print("\r\nDiagnostics Reset.\r\n");
	 		  		  	          }
	 		  		  	          break;

	 		  		  	          /* ------------------------------------------------
	 		  		  	             CASE 9 : PRINT DIAGNOSTICS
	 		  		  	             ------------------------------------------------ */
	 		  		  	          case 9:
	 		  		  	          {
	 		  		  	              UART_Print(
	 		  		  	                  "\r\n--- Diagnostics ---\r\n"
	 		  		  	                  "  TX count       = %lu\r\n"
	 		  		  	                  "  RX count       = %lu\r\n"
	 		  		  	                  "  Timeout count  = %lu\r\n"
	 		  		  	                  "  CRC errors     = %lu\r\n"
	 		  		  	                  "  Exceptions     = %lu\r\n"
	 		  		  	                  "  Retry count    = %lu\r\n"
	 		  		  	                  "  Comm OK        = %s\r\n",
	 		  		  	                  (unsigned long)servo->tx_count,
	 		  		  	                  (unsigned long)servo->rx_count,
	 		  		  	                  (unsigned long)servo->timeout_count,
	 		  		  	                  (unsigned long)servo->crc_error_count,
	 		  		  	                  (unsigned long)servo->exception_count,
	 		  		  	                  (unsigned long)servo->retry_count,
	 		  		  	                  servo->communication_ok ? "YES" : "NO");
	 		  		  	          }
	 		  		  	          break;

	 		  		  	          /* ------------------------------------------------
	 		  		  	             CASE 0 : EXIT
	 		  		  	             ------------------------------------------------ */
	 		  		  	          case 10:
	 		  		  	          {
	 		  		  	              UART_Print("\r\nExiting menu.\r\n");
	 		  		  	              exitMenu1 = 1;
	 		  		  	              break;
	 		  		  	          }

	 		  		  	          /* ------------------------------------------------
	 		  		  	             CASE 11 : REACTION GO TO HOME
	 		  		  	             ------------------------------------------------ */
	 		  		  	          case 11:
	 		  		  	          {
	 		  		  	              UART_Print("\r\nReaction : moving to home "
	 		  		  	                         "(%d pulses) ...\r\n",
	 		  		  	                         REACTION_HOME_PULSES);

	 		  		  	              if (ReactionMoveHome(servo))
	 		  		  	                  UART_Print("Reaction : at home.\r\n");
	 		  		  	              else
	 		  		  	                  UART_Print("Reaction : home move FAILED.\r\n");
	 		  		  	          }
	 		  		  	          break;

	 		  		  	          default:
	 		  		  	          {
	 		  		  	              UART_Print("\r\nUnknown choice. Please try again.\r\n");
	 		  		  	          }
	 		  		  	          break;

	 		  		  	   } /* end switch */

	 		  } break;

	 	  case 2:
	 		  exitMenu2 = 0;
	 		  SelectedID = MicroID;
	 		  UART_Print("\r\nDispensing Servo Selected.\r\n");
	 		  /* ------------------------------------------------
	 		     DISPENSING SERVO MENU
	 		     ------------------------------------------------ */

	 		  while(!exitMenu2)
	 		  {
	 		      UART_Print(
	 		          "\r\n"
	 		          "========================================\r\n"
	 		          "  DISPENSING SERVO MENU\r\n"
	 		          "========================================\r\n"
	 		          "  1. Dispensing Move\r\n"
	 		          "  2. Home Servo\r\n"
	 		          "  3. Stop Servo\r\n"
	 		          "  4. Set Speed\r\n"
	 		          "  5. Read Current Position\r\n"
	 		          "  6. Reset Fault\r\n"
	 		          "  7. Set Current Position as Home\r\n"
	 		          "  8. Reset Diagnostics\r\n"
	 		          "  9. Print Diagnostics\r\n"
	 		          "  0. Back\r\n"
	 		          "----------------------------------------\r\n"
	 		          "Enter choice : ");

	 		      UART_ReadLine(buf, sizeof(buf));
	 		      choice= atoi(buf);

	 		      switch(choice)
	 		      {

	 		      /* ---------------------------------------------
	 		         CASE 1 : DISPENSING MOVE
	 		         --------------------------------------------- */

	 		      case 1:
	 		      {
	 		          int deg;
	 		          uint16_t speed;

	 		          UART_Print("\r\nEnter Degree : ");
	 		          UART_ReadLine(buf,sizeof(buf));
	 		          deg = atoi(buf);

	 		          UART_Print("Enter Speed : ");
	 		          UART_ReadLine(buf,sizeof(buf));
	 		          speed = (uint16_t)atoi(buf);

	 		          DispenserMove(servo,
	 		                        SelectedID,
	 		                        deg,
	 		                        speed);

	 		          UART_Print("\r\nDispensing Move Complete.\r\n");
	 		      }
	 		      break;

	 		      /* ---------------------------------------------
	 		         CASE 2 : HOME
	 		         --------------------------------------------- */

	 		      case 2:
	 		      {
	 		          AminoAcidServo(servo,
	 		                         SelectedID);

	 		          UART_Print("\r\nHoming Complete.\r\n");
	 		      }
	 		      break;

	 		      /* ---------------------------------------------
	 		         CASE 3 : STOP
	 		         --------------------------------------------- */

	 		      case 3:
	 		      {
	 		    	  ServoWrite16Ack(servo,
	 		                       SelectedID,
	 		                       SERVO_MOTION_CMD,
	 		                       SERVO_STOP);

	 		          HAL_Delay(500);

	 		          UART_Print("\r\nServo Stopped.\r\n");
	 		      }
	 		      break;

	 		      /* ---------------------------------------------
	 		         CASE 4 : SET SPEED
	 		         --------------------------------------------- */

	 		      case 4:
	 		      {
	 		          uint16_t speed;

	 		          UART_Print("\r\nEnter Speed : ");
	 		          UART_ReadLine(buf,sizeof(buf));

	 		          speed = (uint16_t)atoi(buf);

	 		          ServoWrite16Ack(servo,
	 		                       SelectedID,
	 		                       SERVO_POS_SPEED,
	 		                       speed);

	 		          HAL_Delay(500);

	 		          UART_Print("Speed Updated.\r\n");
	 		      }
	 		      break;

	 		      /* ---------------------------------------------
	 		         CASE 5 : READ POSITION
	 		         --------------------------------------------- */

	 		      case 5:
	 		      {
	 		          int32_t pos;

	 		          if(ServoSafeReadPosition(servo,
	 		                                   SelectedID,
	 		                                   READ_CURRENT_POS,
	 		                                   &pos))
	 		          {
	 		              UART_Print("\r\nCurrent Position = %ld\r\n",
	 		                         (long)pos);
	 		          }
	 		          else
	 		          {
	 		              UART_Print("\r\nRead Failed.\r\n");
	 		          }
	 		      }
	 		      break;

	 		      /* ---------------------------------------------
	 		         CASE 6 : RESET FAULT
	 		         --------------------------------------------- */

	 		      case 6:
	 		      {
	 		    	  ServoWrite16Ack(servo,
	 		                       SelectedID,
	 		                       FAULT_RESET,
	 		                       SET_HIGH);

	 		          HAL_Delay(500);

	 		          UART_Print("\r\nFault Reset Sent.\r\n");
	 		      }
	 		      break;

	 		      /* ---------------------------------------------
	 		         CASE 7 : SET CURRENT POSITION AS HOME
	 		         --------------------------------------------- */

	 		      case 7:
	 		      {
	 		    	  ServoWrite16Ack(servo,
	 		                       SelectedID,
	 		                       SERVO_HOME_TRIGGER,
	 		                       SERVO_CURRENT_HOME);

	 		          HAL_Delay(500);

	 		          UART_Print("\r\nCurrent Position Set As Home.\r\n");
	 		      }
	 		      break;

	 		      /* ---------------------------------------------
	 		         CASE 8 : RESET DIAGNOSTICS
	 		         --------------------------------------------- */

	 		      case 8:
	 		      {
	 		          ServoResetDiagnostics(servo);

	 		          UART_Print("\r\nDiagnostics Reset.\r\n");
	 		      }
	 		      break;

	 		      /* ---------------------------------------------
	 		         CASE 9 : PRINT DIAGNOSTICS
	 		         --------------------------------------------- */

	 		      case 9:
	 		      {
	 		          UART_Print(
	 		              "\r\n--- Diagnostics ---\r\n"
	 		              "TX Count      : %lu\r\n"
	 		              "RX Count      : %lu\r\n"
	 		              "Timeout Count : %lu\r\n"
	 		              "CRC Errors    : %lu\r\n"
	 		              "Exceptions    : %lu\r\n"
	 		              "Retry Count   : %lu\r\n"
	 		              "Comm Status   : %s\r\n",

	 		              (unsigned long)servo->tx_count,
	 		              (unsigned long)servo->rx_count,
	 		              (unsigned long)servo->timeout_count,
	 		              (unsigned long)servo->crc_error_count,
	 		              (unsigned long)servo->exception_count,
	 		              (unsigned long)servo->retry_count,
	 		              servo->communication_ok ? "OK" : "FAILED");
	 		      }
	 		      break;

	 		      /* ---------------------------------------------
	 		         CASE 0 : BACK
	 		         --------------------------------------------- */

	 		      case 0:
	 		      {
	 		          UART_Print("\r\nReturning to Servo Selection...\r\n");
	 		          exitMenu2 = 1;
	 		          break;
	 		      }

	 		      default:
	 		      {
	 		          UART_Print("\r\nInvalid Choice.\r\n");
	 		      }
	 		      break;
	 		      }
	 		 break;

	 	  default:
	 	          UART_Print("\r\nInvalid Choice.\r\n");
	 	          break;
	 	  }break;
	     /* USER CODE BEGIN 3 */

	   /* USER CODE END 3 */
	 	  case 3:
	 	    exitMenu3 = 0;
	 	    while (!exitMenu3)
	 	    {
	 	        UART_Print(
	 	            "\r\n---------- STEPPER MENU ----------\r\n"
	 	            "  1. Enter Steps, Direction and Speed for motor\r\n"
	 	        	"  2. Set Weight (Load Cell Feedback)\r\n"
	 	        	"  3. Dispense by Weight (Stepper + Load Cell Feedback)\r\n"
	 	        	"  4. Run Full Recipe Test (20-position dispense sequence)\r\n"
	 	        	"  5. Go to the limit\r\n"
	 	        	"  6. Peristalic pump working\r\n"
	 	        	"  7. GPIO Pin Read\r\n"
	 	        	"  8. Proximity Sensing Dispense\r\n"
	 	        	"  9. Testing the Y Pump\r\n"
	 	        	" 10. Testing the Z Pump\r\n"
	 	        	" 11. Enter Steps, Direction and Speed for motor 2\r\n"
	 	        	" 12. Operate Stepper (Stepper.h library)\r\n"
	 	        	" 14. PF15 GPIO Set / Unset\r\n"
	 	        	" 15. Read the cleavage limits (1 = PD1, 2 = PB6)\r\n"
	 	        	" 17. Operate Stepper 3 motor (PUL2A/PUL2B/DIR2A/DIR2B)\r\n"
	 	        		"16. gpio read"
	 	            "  0. Exit the Menu\r\n"
	 	            "-----------------------------------\r\n"
	 	            "Enter choice : ");

	 	        UART_ReadLine(buf, sizeof(buf));
	 	        choice= atoi(buf);

	 	        switch (choice)
	 	        {
	 	        case 1:
	 	        {
	 	            strcpy(msg, "Enter Steps of Motor: ");
	 	            HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
	 	            UART_ReadString(rxBuffer, sizeof(rxBuffer));
	 	            steps_1 = atoi(rxBuffer);

	 	            strcpy(msg, "Enter direction of Motor: ");
	 	            HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
	 	            UART_ReadString(rxBuffer, sizeof(rxBuffer));
	 	            direction_1 = atoi(rxBuffer);

	 	            strcpy(msg, "Enter speed of Motor: ");
	 	            HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
	 	            UART_ReadString(rxBuffer, sizeof(rxBuffer));
	 	            speed_1 = atoi(rxBuffer);

	 	            stepperData(steps_1, direction_1, speed_1);
	 	        }
	 	        break;

	 	        case 2:
	 	        {
	 	        	 strcpy(msg, "Enter the weight: ");
	 	        	 HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
	 	        	 UART_ReadString(rxBuffer, sizeof(rxBuffer));
	 	        	 int weight = atoi(rxBuffer);

	 	        	 UART_Print("\r\nTarget weight set to %d. Starting load check...\r\n", weight);

	 	        	 if (LoadCellLoadCheck(loadcell, (uint16_t)weight, LOADCELL_COARSE_PCT, LOADCELL_FINE_PCT)) {
	 	        		 UART_Print("\r\nTarget weight of %d reached.\r\n", weight);
	 	        	 } else {
	 	        		 UART_Print("\r\nLoad check failed - see LOADCELL error prints above.\r\n");
	 	        	 }
	 	        }
	             break;

	 	        case 3:
	 	        {
	 	        	int targetWeight, dispenseDirection, dispenseSpeed;

	 	        	strcpy(msg, "Enter target weight: ");
	 	        	HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
	 	        	UART_ReadString(rxBuffer, sizeof(rxBuffer));
	 	        	targetWeight = atoi(rxBuffer);

	 	        	strcpy(msg, "Enter direction of Motor: ");
	 	        	HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
	 	        	UART_ReadString(rxBuffer, sizeof(rxBuffer));
	 	        	dispenseDirection = atoi(rxBuffer);

	 	        	strcpy(msg, "Enter speed of Motor: ");
	 	        	HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
	 	        	UART_ReadString(rxBuffer, sizeof(rxBuffer));
	 	        	dispenseSpeed = atoi(rxBuffer);

	 	        	if (LoadCellStepperDispenseRamped(loadcell, (uint16_t)targetWeight,
	 	        			dispenseDirection, dispenseSpeed)) {
	 	        		UART_Print("\r\nDispense complete -- target weight of %d reached.\r\n", targetWeight);
	 	        	} else {
	 	        		UART_Print("\r\nDispense failed - see STEPPER DISPENSE error prints above.\r\n");
	 	        	}
	 	        }
	 	        break;

	 	        case 4:
	 	        {
	 	            /* Faithful port of the Mbed reference's case 44
	 	             * (Dispensing_module_Test(float weight[])) -- a 20-position
	 	             * carousel walk, dosing by weight only at the positions
	 	             * Pos[i]==1, indexing the servo between each. Same
	 	             * hardcoded Pos[]/weight[] test recipe as the reference
	 	             * (only positions 7 and 9 actually dose, both to 10g).
	 	             *
	 	             * UPDATE: drainStepperLimit() IS real -- stepper.c/.h,
	 	             * already included via Machinemenu.c's existing
	 	             * "#include \"stepper.h\"". Previous version of this port
	 	             * incorrectly skipped it thinking no such function existed;
	 	             * now called at both points the reference calls it: once
	 	             * before the initial servo home, and once at the top of
	 	             * every loop iteration (all 20 positions, not just the
	 	             * ones that dose -- matches the reference exactly).
	 	             *
	 	             * Dispensing_Move()/DispensingMove() still has no direct
	 	             * equivalent: servo.h DECLARES DispensingMove(deg,speed) but
	 	             * it is never actually DEFINED anywhere in servo.c --
	 	             * calling it would be a linker error. Using
	 	             * DispenserMove(servo, MicroID, deg, speed) instead -- the
	 	             * one that IS implemented and already used elsewhere in
	 	             * this menu (Amino Acid Servo option), on MicroID, the same
	 	             * servo address the reference's Micro_ID always used for
	 	             * this exact indexing/homing role.
	 	             *
	 	             * The metering pre-move before each dose uses this same
	 	             * dispensing stepper (stepperData()) -- same steps/dir/speed
	 	             * values as the reference's drainStepperMove() calls. */
	 	            static const int Pos[20] = {
	 	            	0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	 	            };
	 	            static const float recipeWeight[20] = {
	 	            	10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
	 	            	 0,  0, 15, 15, 15, 15, 15, 15, 15, 15
	 	            };

	 	            UART_Print("Stepper go to limit\r\n");
	 	            drainStepperLimit();

	 	            UART_Print("Servo Go to Home\r\n");
	 	            ServoWrite16Ack(servo, MicroID, SERVO_HOME_TRIGGER, SERVO_HOMING_START);
	 	            osDelay(8000);

	 	            UART_Print("\r\n====================================\r\n"
	 	            		   "Testing Recipe 1\r\n"
	 	            		   "====================================\r\n");

	 	            for (int i = 0; i < 20; i++) {
	 	            	UART_Print("\r\nPosition = %d\r\n", i + 1);

	 	            	/* Home before every position -- matches the reference,
	 	            	 * which re-homes on every single iteration, not just
	 	            	 * the ones that actually dose. */
	 	            	UART_Print("Stepper go to limit\r\n");
	 	            	drainStepperLimit();
	 	            	osDelay(1000);

	 	            	if (Pos[i] == 1) {
	 	            		if (i < 10) {
	 	            			UART_Print("60ml Position %d\r\n", i + 1);
	 	            			stepperData(600, 0, 10);
	 	            		} else {
	 	            			UART_Print("10ml Position %d\r\n", i + 1);
	 	            			stepperData(15000, 0, 50);
	 	            			osDelay(500);
	 	            		}

	 	            		UART_Print("Target Weight = %.2f\r\n", recipeWeight[i]);

	 	            		if (LoadCellLoadCheck(loadcell, (uint16_t) recipeWeight[i],
	 	            				LOADCELL_COARSE_PCT, LOADCELL_FINE_PCT)) {
	 	            			UART_Print("Dose Complete\r\n");
	 	            		} else {
	 	            			UART_Print("Dose FAILED\r\n");
	 	            		}
	 	            		osDelay(500);

	 	            		HAL_GPIO_WritePin(Drain_valve_pin_Pin_GPIO_Port,
	 	            				Drain_valve_pin_Pin_Pin, GPIO_PIN_SET);
	 	            		osDelay(1000);
	 	            		HAL_GPIO_WritePin(Drain_valve_pin_Pin_GPIO_Port,
	 	            				Drain_valve_pin_Pin_Pin, GPIO_PIN_RESET);
	 	            		osDelay(1000);
	 	            	} else {
	 	            		UART_Print("Skipping Position %d\r\n", i + 1);
	 	            	}

	 	            	if (i < 19) {
	 	            		DispenserMove(servo, MicroID, -18, 40);
	 	            	}
	 	            }

	 	            UART_Print("\r\n===== Recipe Test Complete =====\r\n");
	 	        }
	 	        break;

	 	        case 5:
	 	        	drainStepperLimit();
	 	        	break;

	 	       case 6:
	 	       	 	        	exitMenu7 = 0;
	 	       	 	        		 	      while (!exitMenu7)
	 	       	 	        		 	      {
	 	       	 	        		 	          UART_Print("=======VIBRATOR MOTOR=======\r\n"
	 	       	 	        		 	                      "1. Ronbot gripper ON\r\n"
	 	       	 	        		 	                      "2. Ronbot gripper OFF\r\n"
	 	       	 	        		 	                      "0. Back\r\n"
	 	       	 	        		 	                      "============================\r\n"
	 	       	 	        		 	                      "Enter your choice: ");
	 	       	 	        		 	          UART_ReadLine(buf, sizeof(buf));
	 	       	 	        		 	          choice = atoi(buf);

	 	       	 	        		 	          switch(choice)
	 	       	 	        		 	          {
	 	       	 	        		 	          case 1:
	 	       	 	        		 	              //HAL_GPIO_WritePin(Liquid_peristalic_pump_GPIO_Port, Liquid_peristalic_pump_Pin, GPIO_PIN_SET);
	 	       	 	        		 	              HAL_GPIO_WritePin(Robot_gripper_GPIO_Port, Robot_gripper_Pin, GPIO_PIN_SET);
	 	       	 	        		 	              UART_Print("\r\nPump going up.\r\n");
	 	       	 	        		 	              break;
	 	       	 	        		 	          case 2:
	 	       	 	        		 	              //HAL_GPIO_WritePin(Liquid_peristalic_pump_GPIO_Port, Liquid_peristalic_pump_Pin, GPIO_PIN_RESET);
	 	       	 	        		 	              HAL_GPIO_WritePin(Robot_gripper_GPIO_Port, Robot_gripper_Pin, GPIO_PIN_RESET);
	 	       	 	        		 	              UART_Print("\r\nPump going down.\r\n");
	 	       	 	        		 	              break;
	 	       	 	        		 	          case 0:
	 	       	 	        		 	              UART_Print("\r\nReturning to MACHINE MENU\r\n");
	 	       	 	        		 	              exitMenu7 = 1;
	 	       	 	        		 	              break;
	 	       	 	        		 	          default:
	 	       	 	        		 	              UART_Print("\r\nInvalid Choice.\r\n");
	 	       	 	        		 	              break;
	 	       	 	        		 	          }
	 	       	 	        		 	      }
	 	       	 	        	 break;

	 	      case 7:
	 	      {
	 	          UART_Print("\r\nMonitoring Proximity Sensor... Press Reset or change mode to exit.\r\n");

	 	          while (HAL_GPIO_ReadPin(Proximity_sensing_GPIO_Port, Proximity_sensing_Pin) == GPIO_PIN_RESET)
	 	          {
	 	             //f (HAL_GPIO_ReadPin(Proximity_sensing_GPIO_Port, Proximity_sensing_Pin) == GPIO_PIN_SET)
	 	             //
	 	               // UART_Print("\r\nPin is HIGH\r\n");
	 	           // }
	 	           // else
	 	           // {
	 	           //     UART_Print("\r\nPin is LOW\r\n");
	 	           // }
                      UART_Print("\r\nPin is High\r\n");
	 	              osDelay(50);

	 	          }

	 	          // This break is never reached because of the infinite loop.
	 	          break;
	 	      }
               break;

	 	        case 8:
	 	        {
	 	            stepper_proximity_sensing();
	 	        }
	 	        break;

	 	       case 9:
	 	       	 	       	 	        	exitMenu8 = 0;
	 	       	 	       	 	        		 	      while (!exitMenu8)
	 	       	 	       	 	        		 	      {
	 	       	 	       	 	        		 	          UART_Print("=======VIBRATOR MOTOR=======\r\n"
	 	       	 	       	 	        		 	                      "1. Ronbot gripper ON\r\n"
	 	       	 	       	 	        		 	                      "2. Ronbot gripper OFF\r\n"
	 	       	 	       	 	        		 	                      "0. Back\r\n"
	 	       	 	       	 	        		 	                      "============================\r\n"
	 	       	 	       	 	        		 	                      "Enter your choice: ");
	 	       	 	       	 	        		 	          UART_ReadLine(buf, sizeof(buf));
	 	       	 	       	 	        		 	          choice = atoi(buf);

	 	       	 	       	 	        		 	          switch(choice)
	 	       	 	       	 	        		 	          {
	 	       	 	       	 	        		 	          case 1:
	 	       	 	       	 	        		 	              HAL_GPIO_WritePin(Liquid_peristalic_pump_GPIO_Port, Liquid_peristalic_pump_Pin, GPIO_PIN_SET);
	 	       	 	       	 	        		 	              //HAL_GPIO_WritePin(A_Pump_GPIO_Port, A_Pump_Pin, GPIO_PIN_SET);
	 	       	 	       	 	        		 	              UART_Print("\r\nPump going up.\r\n");
	 	       	 	       	 	        		 	              break;
	 	       	 	       	 	        		 	          case 2:
	 	       	 	       	 	        		 	              HAL_GPIO_WritePin(Liquid_peristalic_pump_GPIO_Port, Liquid_peristalic_pump_Pin, GPIO_PIN_RESET);
	 	       	 	       	 	        		 	              //HAL_GPIO_WritePin(A_Pump_GPIO_Port, A_Pump_Pin, GPIO_PIN_RESET);
	 	       	 	       	 	        		 	              UART_Print("\r\nPump going down.\r\n");
	 	       	 	       	 	        		 	              break;
	 	       	 	       	 	        		 	          case 0:
	 	       	 	       	 	        		 	              UART_Print("\r\nReturning to MACHINE MENU\r\n");
	 	       	 	       	 	        		 	              exitMenu9 = 1;
	 	       	 	       	 	        		 	              break;
	 	       	 	       	 	        		 	          default:
	 	       	 	       	 	        		 	              UART_Print("\r\nInvalid Choice.\r\n");
	 	       	 	       	 	        		 	              break;
	 	       	 	       	 	        		 	          }
	 	       	 	       	 	        		 	      }
	 	       	 	       	 	        	 break;

	 	      case 10:
	 	      	 	       	 	       	 	        	exitMenu9 = 0;
	 	      	 	       	 	       	 	        		 	      while (!exitMenu9)
	 	      	 	       	 	       	 	        		 	      {
	 	      	 	       	 	       	 	        		 	          UART_Print("=======VIBRATOR MOTOR=======\r\n"
	 	      	 	       	 	       	 	        		 	                      "1. Ronbot gripper ON\r\n"
	 	      	 	       	 	       	 	        		 	                      "2. Ronbot gripper OFF\r\n"
	 	      	 	       	 	       	 	        		 	                      "0. Back\r\n"
	 	      	 	       	 	       	 	        		 	                      "============================\r\n"
	 	      	 	       	 	       	 	        		 	                      "Enter your choice: ");
	 	      	 	       	 	       	 	        		 	          UART_ReadLine(buf, sizeof(buf));
	 	      	 	       	 	       	 	        		 	          choice = atoi(buf);

	 	      	 	       	 	       	 	        		 	          switch(choice)
	 	      	 	       	 	       	 	        		 	          {
	 	      	 	       	 	       	 	        		 	          case 1:
	 	      	 	       	 	       	 	        		 	              HAL_GPIO_WritePin(Liquid_peristalic_pump_GPIO_Port, Liquid_peristalic_pump_Pin, GPIO_PIN_SET);
	 	      	 	       	 	       	 	        		 	              //HAL_GPIO_WritePin(B_Pump_GPIO_Port, B_Pump_Pin, GPIO_PIN_SET);
	 	      	 	       	 	       	 	        		 	              UART_Print("\r\nPump going up.\r\n");
	 	      	 	       	 	       	 	        		 	              break;
	 	      	 	       	 	       	 	        		 	          case 2:
	 	      	 	       	 	       	 	        		 	              HAL_GPIO_WritePin(Liquid_peristalic_pump_GPIO_Port, Liquid_peristalic_pump_Pin, GPIO_PIN_RESET);
	 	      	 	       	 	       	 	        		 	              //HAL_GPIO_WritePin( B_Pump_GPIO_Port, B_Pump_Pin, GPIO_PIN_RESET);
	 	      	 	       	 	       	 	        		 	              UART_Print("\r\nPump going down.\r\n");
	 	      	 	       	 	       	 	        		 	              break;
	 	      	 	       	 	       	 	        		 	          case 0:
	 	      	 	       	 	       	 	        		 	              UART_Print("\r\nReturning to MACHINE MENU\r\n");
	 	      	 	       	 	       	 	        		 	              exitMenu9 = 1;
	 	      	 	       	 	       	 	        		 	              break;
	 	      	 	       	 	       	 	        		 	          default:
	 	      	 	       	 	       	 	        		 	              UART_Print("\r\nInvalid Choice.\r\n");
	 	      	 	       	 	       	 	        		 	              break;
	 	      	 	       	 	       	 	        		 	          }
	 	      	 	       	 	       	 	        		 	      }
	 	      	 	       	 	       	 	        	 break;

	 	        case 11:
	 	        {
	 	            /* Motor 2 (PUL1A/PUL1B/DIR1A/DIR1B) -- identical prompt flow to
	 	             * case 1, just calling stepperData2() instead of stepperData(). */
	 	            int steps_2, direction_2, speed_2;

	 	            strcpy(msg, "Enter Steps of Motor 2: ");
	 	            HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
	 	            UART_ReadString(rxBuffer, sizeof(rxBuffer));
	 	            steps_2 = atoi(rxBuffer);

	 	            strcpy(msg, "Enter direction of Motor 2: ");
	 	            HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
	 	            UART_ReadString(rxBuffer, sizeof(rxBuffer));
	 	            direction_2 = atoi(rxBuffer);

	 	            strcpy(msg, "Enter speed of Motor 2: ");
	 	            HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
	 	            UART_ReadString(rxBuffer, sizeof(rxBuffer));
	 	            speed_2 = atoi(rxBuffer);

	 	            stepperData2(steps_2, direction_2, speed_2);
	 	        }
	 	        break;

	 	        case 12:
	 	        {
	 	            /* Drive the stepper through the Stepper.h library (Stepper_Init
	 	             * + Stepper_Step) instead of stepperData2(). The port/pin pairs
	 	             * are passed straight from main.h -- PUL1A/PUL1B/DIR1A/DIR1B --
	 	             * so this does not depend on the A0_/A1_ macros in Stepper.h.
	 	             * The motor object is created ONCE (static) and initialised on
	 	             * the first call. */
	 	            static StepperMotor libMotor;
	 	            static int          libMotorReady = 0;
	 	            int steps_3, direction_3, speed_3;

	 	            if (!libMotorReady) {
	 	                Stepper_Init(&libMotor,
	 	                             PUL1A_Pin_GPIO_Port, PUL1A_Pin_Pin,   /* A0 */
	 	                             PUL1B_Pin_GPIO_Port, PUL1B_Pin_Pin,   /* A1 */
	 	                             DIR1A_Pin_GPIO_Port, DIR1A_Pin_Pin,   /* A2 */
	 	                             DIR1B_Pin_GPIO_Port, DIR1B_Pin_Pin);  /* A3 */
	 	                libMotorReady = 1;
	 	            }

	 	            strcpy(msg, "Enter Steps: ");
	 	            HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
	 	            UART_ReadString(rxBuffer, sizeof(rxBuffer));
	 	            steps_3 = atoi(rxBuffer);

	 	            strcpy(msg, "Enter direction: ");
	 	            HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
	 	            UART_ReadString(rxBuffer, sizeof(rxBuffer));
	 	            direction_3 = atoi(rxBuffer);

	 	            strcpy(msg, "Enter speed: ");
	 	            HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
	 	            UART_ReadString(rxBuffer, sizeof(rxBuffer));
	 	            speed_3 = atoi(rxBuffer);

	 	            Stepper_Step(&libMotor, steps_3, direction_3, speed_3);
	 	        }
	 	        break;


	 	       //case 14:
	 	       //{
	 	           /* Set / unset PF15 (Powder_motor_right in main.h). */
	 	          // UART_Print("\r\nPF15 (Powder_motor_right): "
	 	                   //   "1 = SET (HIGH), 0 = UNSET (LOW). Enter choice: ");
	 	         //  UART_ReadLine(buf, sizeof(buf));

	 	           //if (atoi(buf) == 1) {
	 	             //  HAL_GPIO_WritePin(Powder_motor_right_GPIO_Port,
	 	                  //               Powder_motor_right_Pin, GPIO_PIN_SET);
	 	              // UART_Print("PF15 is SET (HIGH)\r\n");
	 	          // } else {
	 	          //     HAL_GPIO_WritePin(Powder_motor_right_GPIO_Port,
	 	              //                   Powder_motor_right_Pin, GPIO_PIN_RESET);
	 	           //    UART_Print("PF15 is UNSET (LOW)\r\n");
	 	         //  }
	 	      // }
	 	      // break;

	 	       case 15:
	 	       {
	 	           /* Read both cleavage limit switches.
	 	            *
	 	            * PD1 (Clevage_limit_1) IS SET UP AS AN OUTPUT IN CUBEMX
	 	            * (PD1.Signal=GPIO_Output), and gpio.c drives it LOW at boot.
	 	            * HAL_GPIO_ReadPin() on an output pin returns the OUTPUT DATA
	 	            * REGISTER, so it would read LOW forever no matter what the
	 	            * switch does. It is therefore reconfigured to input+pull-up
	 	            * here, the same thing drainStepperLimit() does for PG1.
	 	            * The real fix is to set PD1 to GPIO_Input in CubeMX.
	 	            *
	 	            * PB6 (Clevage_limit_2) is already GPIO_Input with a pull-up,
	 	            * so it is left exactly as CubeMX configured it.
	 	            *
	 	            * Pull-up on both means a switch that closes to GND:
	 	            *      LOW  = switch CLOSED = at the limit
	 	            *      HIGH = switch OPEN   = not at the limit */
	 	           {
	 	               GPIO_InitTypeDef gi = {0};
	 	               gi.Pin   = Clevage_limit_1_Pin;
	 	               gi.Mode  = GPIO_MODE_INPUT;
	 	               gi.Pull  = GPIO_NOPULL;
	 	               gi.Speed = GPIO_SPEED_FREQ_LOW;
	 	               HAL_GPIO_Init(Clevage_limit_1_GPIO_Port, &gi);
	 	               osDelay(5);   /* let the pull-up settle the line */
	 	           }

	 	           GPIO_PinState l1 = HAL_GPIO_ReadPin(Clevage_limit_1_GPIO_Port,
	 	                                               Clevage_limit_1_Pin);
	 	           GPIO_PinState l2 = HAL_GPIO_ReadPin(Clevage_limit_2_GPIO_Port,
	 	                                               Clevage_limit_2_Pin);

	 	           UART_Print("\r\n[CLIMIT] Limit 1 (PD1) : %s -> %s\r\n",
	 	                      (l1 == GPIO_PIN_SET) ? "HIGH" : "LOW",
	 	                      (l1 == GPIO_PIN_SET) ? "open (not at limit)"
	 	                                           : "CLOSED (at limit)");
	 	           UART_Print("[CLIMIT] Limit 2 (PB6) : %s -> %s\r\n",
	 	                      (l2 == GPIO_PIN_SET) ? "HIGH" : "LOW",
	 	                      (l2 == GPIO_PIN_SET) ? "open (not at limit)"
	 	                                           : "CLOSED (at limit)");

	 	           /* A single reading cannot tell a working switch from a dead
	 	            * one, so watch both pins for 15 s and report every change --
	 	            * trigger each limit by hand while this runs. */
	 	           UART_Print("[CLIMIT] Watching 15 s -- trigger each limit now.\r\n");

	 	           uint32_t clStart   = HAL_GetTick();
	 	           uint32_t clChanges = 0;

	 	           while ((HAL_GetTick() - clStart) < 15000u) {
	 	               GPIO_PinState c1 = HAL_GPIO_ReadPin(Clevage_limit_1_GPIO_Port,
	 	                                                   Clevage_limit_1_Pin);
	 	               GPIO_PinState c2 = HAL_GPIO_ReadPin(Clevage_limit_2_GPIO_Port,
	 	                                                   Clevage_limit_2_Pin);

	 	               if (c1 != l1) {
	 	                   l1 = c1;
	 	                   clChanges++;
	 	                   UART_Print("[CLIMIT] Limit 1 -> %s  (t=%lu ms)\r\n",
	 	                              (c1 == GPIO_PIN_SET) ? "HIGH (open)"
	 	                                                   : "LOW (at limit)",
	 	                              (unsigned long)(HAL_GetTick() - clStart));
	 	               }

	 	               if (c2 != l2) {
	 	                   l2 = c2;
	 	                   clChanges++;
	 	                   UART_Print("[CLIMIT] Limit 2 -> %s  (t=%lu ms)\r\n",
	 	                              (c2 == GPIO_PIN_SET) ? "HIGH (open)"
	 	                                                   : "LOW (at limit)",
	 	                              (unsigned long)(HAL_GetTick() - clStart));
	 	               }

	 	               osDelay(5);   /* ~200 Hz sampling, and yields to the RTOS */
	 	           }

	 	           if (clChanges == 0u) {
	 	               UART_Print("[CLIMIT] No change in 15 s -- both pins stuck. "
	 	                          "If a switch WAS pressed, its output is not "
	 	                          "reaching the pin (check the switch common is "
	 	                          "on board GND).\r\n");
	 	           } else {
	 	               UART_Print("[CLIMIT] Done, %lu change(s) seen.\r\n",
	 	                          (unsigned long) clChanges);
	 	           }

	 	           UART_Print("[CLIMIT] Final: Limit 1 = %s, Limit 2 = %s\r\n",
	 	                      (l1 == GPIO_PIN_SET) ? "HIGH" : "LOW",
	 	                      (l2 == GPIO_PIN_SET) ? "HIGH" : "LOW");
	 	       }
	 	       break;

	 	       case 16:
	 	       {
	 	    	   while(HAL_GPIO_ReadPin(Clevage_limit_1_GPIO_Port,
                            Clevage_limit_1_Pin) == 0)
	 	    	   {   UART_Print("INSIDE\n\r");
	 	    		   osDelay(100);
	 	    	   }
	 	       }break;

	 	       case 17:
	 	       {
	 	           /* Stepper 3 -- PUL2A/PUL2B/DIR2A/DIR2B, via stepper_3() in
	 	            * stepper.c. Same Stepper.h engine and the same
	 	            * steps/direction/speed contract as option 12, just on the
	 	            * second set of pins. */
	 	           int steps_4, direction_4, speed_4;

	 	           strcpy(msg, "Enter Steps: ");
	 	           HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
	 	           UART_ReadString(rxBuffer, sizeof(rxBuffer));
	 	           steps_4 = atoi(rxBuffer);

	 	           /* Kept short: msg is char[30], and a longer literal than this
	 	            * overflows it. The 0/1 meaning goes through UART_Print,
	 	            * which formats into its own 1 KB buffer. */
	 	           UART_Print("\r\n0 = clockwise, 1 = anticlockwise\r\n");
	 	           strcpy(msg, "Enter direction: ");
	 	           HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
	 	           UART_ReadString(rxBuffer, sizeof(rxBuffer));
	 	           direction_4 = atoi(rxBuffer);

	 	           strcpy(msg, "Enter speed: ");
	 	           HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
	 	           UART_ReadString(rxBuffer, sizeof(rxBuffer));
	 	           speed_4 = atoi(rxBuffer);

	 	           UART_Print("\r\n[STEPPER3] %d steps, dir %d, speed %d...\r\n",
	 	                      steps_4, direction_4, speed_4);

	 	           stepper_3(steps_4, direction_4, speed_4);

	 	           UART_Print("[STEPPER3] Done.\r\n");
	 	       }
	 	       break;

	 	        case 0:
	 	            UART_Print("\r\nReturning to MACHINE MENU\r\n");
	 	            exitMenu3 = 1;
	 	            break;

	 	        default:
	 	            UART_Print("\r\nInvalid Choice.\r\n");
	 	            break;
	 	        }
	 	    }
	 	    break;

	 	  case 4:
	 	      exitMenu4 = 0;
	 	      while (!exitMenu4)
	 	      {
	 	          UART_Print("=======DRAIN VALVE=======\r\n"
	 	                      "1. Open the Drain Valve\r\n"
	 	                      "2. Close the Drain Valve\r\n"
	 	                      "0. Back\r\n"
	 	                      "============================\r\n"
	 	                      "Enter your choice: ");
	 	         UART_ReadLine(buf, sizeof(buf));
	 	          choice = atoi(buf);

	 	          switch(choice)
	 	          {
	 	          case 1:
	 	              HAL_GPIO_WritePin(Drain_valve_pin_Pin_GPIO_Port, Drain_valve_pin_Pin_Pin, GPIO_PIN_SET);
	 	              UART_Print("\r\nDrain Valve OPEN.\r\n");
	 	              break;
	 	          case 2:
	 	              HAL_GPIO_WritePin(Drain_valve_pin_Pin_GPIO_Port, Drain_valve_pin_Pin_Pin, GPIO_PIN_RESET);
	 	              UART_Print("\r\nDrain Valve CLOSED.\r\n");
	 	              break;
	 	          case 0:
	 	              UART_Print("\r\nReturning to MACHINE MENU\r\n");
	 	              exitMenu4 = 1;
	 	              break;
	 	          default:
	 	              UART_Print("\r\nInvalid Choice.\r\n");
	 	              break;
	 	          }
	 	      }
	 	      break;

	 	 case 5:
	 	 {
	 	     int robotExit = 0;

	 	     while (!robotExit)
	 	     {
	 	         UART_Print(
	 	             "\r\n"
	 	             "========== ROBOT MENU ==========\r\n"
	 	             "1. Connect to Robot\r\n"
	 	             "2. Initialize Robot\r\n"
	 	             "3. Move Robot to Home Position\r\n"
	 	        	 "4. Move Robot to Liquid Dispenser\r\n"
	 	        	 "5. Robot Status\r\n"
	 	        	 "6. Disconnect Robot\r\n"
	 	        	 "7. Move Linear rail\r\n"
	 	        	 "8. Set Rail Home (current position)\r\n"
	 	        	 "9. Return Rail to Home\r\n"
	 	             "10. Read Home position\r\n"
	 	        	 "11. Move servo to Home\r\n"
	 	        	 "12. Move to powder dispensing module\r\n"
	 	        	 "13. Move to reaction stirring module\r\n"
	 	        	 "14. Collect the vessel from reaction stir module\r\n"
                     "15. Go to powder dispensing module\r\n"
	 	        	 "16. Run the entire Sequence\r\n"
	 	        	 "17. Run the amino acid first addition sequence\r\n"
	 	        	 "18. Run the cleavage module (arm poses)\r\n"
	 	        	 "19. Run the FULL cleavage module sequence\r\n"
	 	        	 //"15. Test the robot gripper\r\n"
	 	             "0. Back\r\n"
	 	             "===============================\r\n"
	 	             "Enter Choice : ");

	 	         UART_ReadLine(buf, sizeof(buf));
	 	         choice = atoi(buf);

	 	         switch(choice)
	 	         {
	 	         case 1:

	 	             /* connectRobot() (RobotCommand.c) replaces
	 	              * StartCobotConnectionTask(), which no longer exists in
	 	              * the new robot layer -- connectRobot() is idempotent
	 	              * (returns immediately if already connected) and also
	 	              * runs robotInitConfig() internally, so this both
	 	              * connects AND enables the arm in one step. */
	 	             if (robotSocket < 0)
	 	             {
	 	                 UART_Print("\r\nConnecting to Robot...\r\n");

	 	                 if (connectRobot() == 0)
	 	                     UART_Print("Robot Connected.\r\n");
	 	                 else
	 	                     UART_Print("Robot Connection Failed.\r\n");
	 	             }
	 	             else
	 	             {
	 	                 UART_Print("Robot Already Connected.\r\n");
	 	             }

	 	             break;

	 	         case 2:

	 	             if (robotSocket < 0)
	 	             {
	 	                 UART_Print("Robot is not connected.\r\n");
	 	                 break;
	 	             }

	 	             UART_Print("Initializing Robot...\r\n");
	 	             /* robotInitConfig() now returns int and already prints
	 	              * its own detailed pass/fail diagnostics -- check it so
	 	              * this doesn't print "Complete" over a failure. */
	 	             if (robotInitConfig() == 0)
	 	                 UART_Print("Initialization Complete.\r\n");
	 	             else
	 	                 UART_Print("Initialization FAILED -- see message above.\r\n");

	 	             break;

	 	         case 3:

	 	             if (robotSocket < 0)
	 	             {
	 	                 UART_Print("Robot is not connected.\r\n");
	 	                 break;
	 	             }

	 	             UART_Print("Moving Robot to HOME...\r\n");

	 	             /* moveRobotToHomePosition() (new RobotMotion.c -- NOT
	 	              * the old moveToHomePosition(), which no longer exists)
	 	              * is void and already prints its own accurate outcome
	 	              * internally (connect/ready/move failures, or
	 	              * DBG_REACHED() on success) -- no unconditional "reached"
	 	              * print here anymore, since that could contradict a
	 	              * failure message it just printed itself. */
	 	             moveRobotToHomePosition();

	 	             break;

	 	         case 4:

	 	        	if (robotSocket < 0)
	 	            {
	 	        		 UART_Print("Robot is not connected.\r\n");
	 	        		 break;
	 	            }

	 	        	UART_Print("Moving Robot to Liquid Dispenser Now.\r\n");

	 	        	/* moveToLIQ1Position()/moveToLIQ2Position() no longer exist --
	 	        	 * the new RobotMotion.c replaced the old two-pose sequence
	 	        	 * with the station/vessel transport model. ST_LIQUID ("St 1:
	 	        	 * DMF, DIPEA, amino-acid solutions") is the liquid dispenser
	 	        	 * station this menu item meant -- moveVesselTo() retracts
	 	        	 * from wherever the arm currently is, moves the rail, and
	 	        	 * inserts at the station in one call. */

	 	        	runLiquidDispenserModule();
	 	        	//if (moveVesselTo(ST_LIQUID))
	 	        		//UART_Print("Robot reached Liquid Dispenser station.\r\n");
	 	        	//else
	 	        		//UART_Print("Move to Liquid Dispenser FAILED -- see message above.\r\n");

	 	        	 break;

	 	         case 5:
	 	         {
	 	             /* robotStatus() didn't exist anywhere in the new robot
	 	              * layer -- undefined reference. Inline replacement using
	 	              * what's actually available: robotSocket (RobotCommand.h)
	 	              * and getCurrentStation() (RobotMotion.h). Also restores
	 	              * the "break;" this case was missing before -- without it,
	 	              * execution fell straight through into case 0 and exited
	 	              * the robot menu immediately after every status check. */
	 	             static const char *robotStationName[NUM_STATIONS] = {
	 	                 "HOME", "LIQUID", "SOLID", "VOLATILE",
	 	                 "STIR", "DRAIN", "CLEAVE", "PARK"
	 	             };
	 	             UART_Print("\r\n----- ROBOT STATUS -----\r\n");
	 	             UART_Print("Connection : %s\r\n",
	 	                     (robotSocket >= 0) ? "CONNECTED" : "NOT CONNECTED");
	 	             UART_Print("Station    : %s\r\n",
	 	                     robotStationName[getCurrentStation()]);
	 	             UART_Print("-------------------------\r\n");
	 	         }
	 	         break;

	 	         case 6:
	 	        	disconnectRobot();
	 	        	 break;

	 	         case 7:
	 	        	robotLinearMoveConsole();
	 	        	 break;

	 	         case 8:
	 	        	UART_Print("\r\nSetting rail home at current position...\r\n");

	 	            setCurrentPositionAsHome();

	                UART_Print("Rail home set. Turn counter reset to 0.\r\n");
	 	        	 break;

	 	         case 9:
	 	        	if (robotLinearReturnToHome())
	 	                 UART_Print("Rail at home.\r\n");
	 	            else
	 	        		 UART_Print("Return to home failed.\r\n");
	 	        	 break;

	 	         case 10:
	 	               {
	 	        		      int32_t p = 0;
	 	                      if (ServoSafeReadPosition(servo, ROBOT_LINEAR_ID, READ_CURRENT_POS, &p))
	 	        		            UART_Print("Rail absolute position = %ld pulses\r\n", (long)p);
	 	        		      else
	 	        		            UART_Print("Rail position read failed\r\n");
	 	               }
	 	              break;

	 	         case 11:
	 	        	robotLinearGotoHardcodedHome();
	 	        	 break;

	 	         case 12:
	 	        	runPowderDispenserModule();
	 	        	 break;

	 	         case 13:
	 	        	runReactionStirModule();
	 	        	 break;
	 	         case 14:
	 	        	runReactionReStirModule();
	 	        	break;
	 	         case 15:
	 	        	runPowderDispenserModule();
	 	        	break;

	 	         case 16:
	 	        	sequence();
	 	        	 break;
	 	         case 17:
	 	        	amino_acid_addition_sequence();
	 	        	 break;

	 	         case 18:
	 	        	/* Cleavage module: the arm runs the taught cleavage poses
	 	        	 * (cleavagePoses[] in RobotMotion.c). Not the same thing as
	 	        	 * clevagemodule(), which is the solvent/valve routine. */
	 	        	if (runCleavageModule())
	 	        	    UART_Print("\r\nCleavage module complete.\r\n");
	 	        	else
	 	        	    UART_Print("\r\nCleavage module FAILED.\r\n");
	 	        	 break;

	 	         case 19:
	 	        	/* Rail home -> door open -> poses 1..3 -> Pneumatic 3 +
	 	        	 * gripper release -> solvent menu with paired motors ->
	 	        	 * door close. */
	 	        	if (cleavageModuleSequence())
	 	        	    UART_Print("\r\nCleavage sequence complete.\r\n");
	 	        	else
	 	        	    UART_Print("\r\nCleavage sequence STOPPED.\r\n");
	 	        	 break;

	 	         case 0:

	 	             robotExit = 1;
	 	             UART_Print("Returning to Main Menu...\r\n");
	 	             break;

	 	         default:

	 	             UART_Print("Invalid Choice.\r\n");
	 	             break;
	 	         }
	 	     }
	 	 }
	 	 break;

	 	  case 6:
	 	      exitMenu6 = 0;
	 	      while (!exitMenu6)
	 	      {
	 	          UART_Print("=======VIBRATOR MOTOR=======\r\n"
	 	                      "1. Vibrator Motor ON\r\n"
	 	                      "2. Vibrator Motor OFF\r\n"
	 	                      "0. Back\r\n"
	 	                      "============================\r\n"
	 	                      "Enter your choice: ");
	 	          UART_ReadLine(buf, sizeof(buf));
	 	          choice = atoi(buf);

	 	          switch(choice)
	 	          {
	 	          case 1:
	 	              //HAL_GPIO_WritePin(Vibrator_motor_control_GPIO_Port, Vibrator_motor_control_Pin, GPIO_PIN_SET);
	 	              UART_Print("\r\nVibrator Motor ON.\r\n");
	 	              break;
	 	          case 2:
	 	              //HAL_GPIO_WritePin(Vibrator_motor_control_GPIO_Port, Vibrator_motor_control_Pin, GPIO_PIN_RESET);
	 	              UART_Print("\r\nVibrator Motor OFF.\r\n");
	 	              break;
	 	          case 0:
	 	              UART_Print("\r\nReturning to MACHINE MENU\r\n");
	 	              exitMenu6 = 1;
	 	              break;
	 	          default:
	 	              UART_Print("\r\nInvalid Choice.\r\n");
	 	              break;
	 	          }
	 	      }
	 	      break;



	 	  case 7:
		 		 	  {

		 		 		 if(LoadCellReadTest(loadcell))
		 		 		 {
		 		 		     UART_Print("Communication OK\r\n");
		 		 		 }
		 		 		 else
		 		 		 {
		 		 		     UART_Print("Communication FAILED\r\n");
		 		 		 }
		 		 	  }break;

	 	  case 8:
	 	  {
	 		 UART_Print("\r\n Robot Moving to Home Position");
	 		 robotInitConfig();
	 	  }break;

	 	 case 9:
	 	     UART_Print("\r\nScanning Modbus Devices...\r\n");

	 	     ServoScanIDs(servo);

	 	     break;

	 	case 10:
	 		 	      exitMenu5 = 0;
	 		 	      while (!exitMenu5)
	 		 	      {
	 		 	          UART_Print("=======DC Motor control=======\r\n"
	 		 	                      "1. Run DC Motor\r\n"
	 		 	                      "2. Stop DC Motor\r\n"
	 		 	                      "3. Powder Motor RIGHT\r\n"
	 		 	                      "4. Powder Motor LEFT\r\n"
	 		 	                      "5. Powder Motor STOP\r\n"
	 		 	                      "0. Back\r\n"
	 		 	                      "==============================\r\n"
	 		 	                      "Enter your choice: ");
	 		 	         UART_ReadLine(buf, sizeof(buf));
	 		 	          choice = atoi(buf);

	 		 	          switch(choice)
	 		 	          {
	 		 	          case 1:
	 		 	              HAL_GPIO_WritePin(DC_Motor_pin_GPIO_Port, DC_Motor_pin_Pin, GPIO_PIN_SET);
	 		 	              //HAL_GPIO_WritePin(X_pump_GPIO_Port, X_pump_Pin, GPIO_PIN_SET);
	 		 	              UART_Print("\r\nMotor Started.\r\n");
	 		 	              break;
	 		 	          case 2:
	 		 	              HAL_GPIO_WritePin(DC_Motor_pin_GPIO_Port, DC_Motor_pin_Pin, GPIO_PIN_RESET);
	 		 	              //HAL_GPIO_WritePin(X_pump_GPIO_Port, X_pump_Pin, GPIO_PIN_RESET);
	 		 	              UART_Print("\r\nMotor Stopped.\r\n");
	 		 	              break;
	 		 	          //case 3:
	 		 	              /* Powder motor RIGHT: drive RIGHT pin, clear LEFT pin.
	 		 	               * (Never both high together -> that would be a shoot
	 		 	               * through / both-terminals-same-polarity = no motion.) */
	 		 	              //HAL_GPIO_WritePin(Powder_motor_left_GPIO_Port,
	 		 	                //                Powder_motor_left_Pin, GPIO_PIN_RESET);
	 		 	              //HAL_GPIO_WritePin(Powder_motor_right_GPIO_Port,
	 		 	                     //           Powder_motor_right_Pin, GPIO_PIN_SET);
	 		 	             // UART_Print("\r\nPowder Motor RIGHT.\r\n");
	 		 	             // break;
	 		 	          //case 4:
	 		 	              /* Powder motor LEFT: drive LEFT pin, clear RIGHT pin. */
	 		 	              //HAL_GPIO_WritePin(Powder_motor_right_GPIO_Port,
	 		 	                           //     Powder_motor_right_Pin, GPIO_PIN_RESET);
	 		 	            //  HAL_GPIO_WritePin(Powder_motor_left_GPIO_Port,
	 		 	                        //        Powder_motor_left_Pin, GPIO_PIN_SET);
	 		 	              //UART_Print("\r\nPowder Motor LEFT.\r\n");
	 		 	              break;
	 		 	          //case 5:
	 		 	              /* Powder motor STOP: clear both direction pins. */
	 		 	              //HAL_GPIO_WritePin(Powder_motor_right_GPIO_Port,
	 		 	                 //               Powder_motor_right_Pin, GPIO_PIN_RESET);
	 		 	             // HAL_GPIO_WritePin(Powder_motor_left_GPIO_Port,
	 		 	                         //       Powder_motor_left_Pin, GPIO_PIN_RESET);
	 		 	            //  UART_Print("\r\nPowder Motor STOPPED.\r\n");
	 		 	             // break;
	 		 	          case 0:
	 		 	              UART_Print("\r\nReturning to MACHINE MENU\r\n");
	 		 	              exitMenu5 = 1;
	 		 	              break;
	 		 	          default:
	 		 	              UART_Print("\r\nInvalid Choice.\r\n");
	 		 	              break;
	 		 	          }
	 		 	      }
	 		 	      break;
	 	case 11:
	 		wash_solvent_selection();
	 		break;
	 	case 12:
	 		octertoide_sequence();
	 		break;

	 	case 13:
	 	{
	 	    /* Read from UART4 (RS232 device on PC10=TX / PC11=RX) and echo every
	 	     * received byte to the USART3 console. Runs for a 30 s window; each
	 	     * byte is printed as its ASCII char and its hex value. */
	 	    uint8_t  rxByte;

	 	    UART_Print("\r\n[UART4] Command mode. Type a command and press Enter to "
	 	               "send it on UART4; the reply is printed back.\r\n"
	 	               "        Type 'exit' to return to the menu.\r\n");

	 	    for (;;)
	 	    {
	 	        UART_Print("\r\nUART4> ");
	 	        UART_ReadLine(buf, sizeof(buf));          /* command from console */

	 	        if (strcmp(buf, "exit") == 0 || strcmp(buf, "EXIT") == 0)
	 	        {
	 	            UART_Print("[UART4] Leaving command mode.\r\n");
	 	            break;
	 	        }

	 	        if (strlen(buf) > 0)
	 	        {
	 	            /* Send the typed command, then a CR+LF terminator. If your
	 	             * device wants a different ending (just CR, just LF, or none),
	 	             * change the two bytes below. */
	 	            HAL_UART_Transmit(&huart4, (uint8_t *)buf, strlen(buf),
	 	                              HAL_MAX_DELAY);
	 	            uint8_t term[2] = { '\r', '\n' };
	 	            HAL_UART_Transmit(&huart4, term, 2, HAL_MAX_DELAY);
	 	            UART_Print("[UART4] Sent: %s\r\n", buf);
	 	        }

	 	        /* Read the reply for up to 3 s (until it goes quiet). */
	 	        UART_Print("[UART4] Reply: ");
	 	        uint32_t lastRx = HAL_GetTick();
	 	        while ((HAL_GetTick() - lastRx) < 3000u)
	 	        {
	 	            if (HAL_UART_Receive(&huart4, &rxByte, 1, 100) == HAL_OK)
	 	            {
	 	                HAL_UART_Transmit(&huart3, &rxByte, 1, HAL_MAX_DELAY);
	 	                lastRx = HAL_GetTick();   /* keep reading while data flows */
	 	            }
	 	        }
	 	        UART_Print("\r\n");
	 	    }
	 	}
	 	break;

	 	case 14:
	 	{
	 	    /* Pneumatic / actuator GPIO controls. Every entry is a plain output
	 	     * pin declared in main.h: pick the actuator, then 1 = ON (pin SET),
	 	     * 0 = OFF (pin RESET). */
	 	    int exitPneu = 0;

	 	    while (!exitPneu)
	 	    {
	 	        UART_Print("\r\n=======PNEUMATIC CONTROLS=======\r\n"
	 	                   "1. Pneumatic 1\r\n"
	 	                   "2. Pneumatic 2\r\n"
	 	                   "3. Pneumatic 3\r\n"
	 	                   "4. Pneumatic 4\r\n"
	 	                   "5. Pneumatic 5\r\n"
	 	                   "6. Robot Gripper\r\n"
	 	                   "7. Liquid Peristaltic Pump\r\n"
	 	                   "8. Cleavage Solvent Motor 1\r\n"
	 	                   "9. Cleavage Solvent Motor 2\r\n"
	 	                   "10. Cleavage Solvent Motor 3\r\n"
	 	                   "11. Cleavage Solvent Motor 4\r\n"
	 	                   "0. Back\r\n"
	 	                   "================================\r\n"
	 	                   "Enter your choice: ");
	 	        UART_ReadLine(buf, sizeof(buf));

	 	        GPIO_TypeDef *pneuPort = NULL;
	 	        uint16_t      pneuPin  = 0;
	 	        const char   *pneuName = NULL;

	 	        switch (atoi(buf))
	 	        {
	 	        case 1:
	 	            pneuPort = Pnuematic_1_GPIO_Port;
	 	            pneuPin  = Pnuematic_1_Pin;
	 	            pneuName = "Pneumatic 1";
	 	            break;
	 	        case 2:
	 	            pneuPort = Pnuematic_2_GPIO_Port;
	 	            pneuPin  = Pnuematic_2_Pin;
	 	            pneuName = "Pneumatic 2";
	 	            break;
	 	        case 3:
	 	            pneuPort = Pnuematic_3_GPIO_Port;
	 	            pneuPin  = Pnuematic_3_Pin;
	 	            pneuName = "Pneumatic 3";
	 	            break;
	 	        case 4:
	 	            pneuPort = Pnuematic_4_GPIO_Port;
	 	            pneuPin  = Pnuematic_4_Pin;
	 	            pneuName = "Pneumatic 4";
	 	            break;
	 	        case 5:
	 	            pneuPort = Pnuematic_5_GPIO_Port;
	 	            pneuPin  = Pnuematic_5_Pin;
	 	            pneuName = "Pneumatic 5";
	 	            break;
	 	        case 6:
	 	            pneuPort = Robot_gripper_GPIO_Port;
	 	            pneuPin  = Robot_gripper_Pin;
	 	            pneuName = "Robot Gripper";
	 	            break;
	 	        case 7:
	 	            pneuPort = Liquid_peristalic_pump_GPIO_Port;
	 	            pneuPin  = Liquid_peristalic_pump_Pin;
	 	            pneuName = "Liquid Peristaltic Pump";
	 	            break;
	 	        /* Cleavage solvent motors -- plain outputs like the rest, so
	 	         * they ride the same 1 = ON / 0 = OFF prompt below. */
	 	        case 8:
	 	            pneuPort = Clevage_solvent_motor_1_GPIO_Port;
	 	            pneuPin  = Clevage_solvent_motor_1_Pin;
	 	            pneuName = "Cleavage Solvent Motor 1";
	 	            break;
	 	        case 9:
	 	            pneuPort = Clevage_solvent_motor_2_GPIO_Port;
	 	            pneuPin  = Clevage_solvent_motor_2_Pin;
	 	            pneuName = "Cleavage Solvent Motor 2";
	 	            break;
	 	        case 10:
	 	            pneuPort = Clevage_solvent_motor_3_GPIO_Port;
	 	            pneuPin  = Clevage_solvent_motor_3_Pin;
	 	            pneuName = "Cleavage Solvent Motor 3";
	 	            break;
	 	        case 11:
	 	            pneuPort = Clevage_solvent_motor_4_GPIO_Port;
	 	            pneuPin  = Clevage_solvent_motor_4_Pin;
	 	            pneuName = "Cleavage Solvent Motor 4";
	 	            break;
	 	        case 0:
	 	            UART_Print("\r\nReturning to MACHINE MENU\r\n");
	 	            exitPneu = 1;
	 	            break;
	 	        default:
	 	            UART_Print("\r\nInvalid Choice.\r\n");
	 	            break;
	 	        }

	 	        if (pneuName == NULL) continue;   /* Back, or a bad entry */

	 	        UART_Print("\r\n%s : 1 = ON, 0 = OFF. Enter choice: ", pneuName);
	 	        UART_ReadLine(buf, sizeof(buf));

	 	        if (atoi(buf) == 1) {
	 	            HAL_GPIO_WritePin(pneuPort, pneuPin, GPIO_PIN_SET);
	 	            UART_Print("%s is ON\r\n", pneuName);
	 	        } else {
	 	            HAL_GPIO_WritePin(pneuPort, pneuPin, GPIO_PIN_RESET);
	 	            UART_Print("%s is OFF\r\n", pneuName);
	 	        }
	 	    }
	 	}
	 	break;

	 	case 15:
	 	{
	 	    /* Cleavage module. Its own submenu so the run is a deliberate
	 	     * choice -- selecting it from the main menu alone must not start
	 	     * pulsing Pneumatic 3 and opening solvent valves. */
	 	    int exitCleave = 0;

	 	    while (!exitCleave)
	 	    {
	 	        UART_Print("\r\n=======CLEAVAGE MODULE=======\r\n"
	 	                   "1. Run Cleavage Module\r\n"
	 	                   "0. Back\r\n"
	 	                   "=============================\r\n"
	 	                   "Enter your choice: ");
	 	        UART_ReadLine(buf, sizeof(buf));

	 	        switch (atoi(buf))
	 	        {
	 	        case 1:
	 	            if (clevagemodule())
	 	                UART_Print("\r\nCleavage Module finished.\r\n");
	 	            else
	 	                UART_Print("\r\nCleavage Module ABORTED.\r\n");
	 	            break;
	 	        case 0:
	 	            UART_Print("\r\nReturning to MACHINE MENU\r\n");
	 	            exitCleave = 1;
	 	            break;
	 	        default:
	 	            UART_Print("\r\nInvalid Choice.\r\n");
	 	            break;
	 	        }
	 	    }
	 	}
	 	break;

	 	case 16:
	 	{
	 	    /* Fmoc UV detector on UART5 (RS232 via MAX3232), 19200 8N1.
	 	     * Straight port of the vendor's Fmoc.py: flush, send GA + CR
	 	     * LF, read one line, print it, wait 250 ms. Loops forever the
	 	     * way the Python's `while True` does. */
	 	    FmocRun();
	 	}
	 	break;

	 	case 17:
	 	{
	 	    /* Listen only: print whatever turns up on UART5, sending nothing. */
	 	    FmocMonitor();
	 	}
	 	break;

	 	case 18:
	 	{
	 	    /* Hammer the TX line so the module's TX LED is actually visible --
	 	     * a 4-byte "GA" is far too short to see. */
	 	    FmocTxTest(10u);
	 	}
	 	break;

	 	case 19:
	 	{
	 	    /* Garbled replies: is it the bit timing, or the levels? */
	 	    FmocBaudScan();
	 	}
	 	break;

	 	case 20:
	 	{
	 	    /* Floating RX pin, or a real transceiver output? */
	 	    FmocRxDriveTest();
	 	}
	 	break;

	 	case 21:
	 	{
	 	    /* Interactive: type a command, see the raw reply. "exit" to leave. */
	 	    FmocSendReceive();

	 	}
	 	break;

	 	 }


	    }
    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}
/* USER CODE END MachineMenuTask */
