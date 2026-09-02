/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : FreeRTOS applications for LINE 7 peptide synthesizer.
 *
 * NOTE: This file is authored to match what STM32CubeMX generates once
 *       FreeRTOS (CMSIS-OS v1) and LWIP are enabled in the .ioc. If you
 *       regenerate from CubeMX, keep everything inside the USER CODE blocks --
 *       that is where the robot RX task, mutex/semaphore, and the appMain +
 *       peptide application task live. See INTEGRATION_GUIDE.md.
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/tcpip.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/err.h"
#include <string.h>
#include <errno.h>
#include "lwip/netif.h"

extern struct netif gnetif;
#include "RobotCommand.h"
//#include "RobotConsole.h"
#include "MachineMenu.h"
/*#include "peptide_app.h"*/
#include "uart.h"
#include "usart.h"          /* huart6 -- the servo RS485 bus */

extern ETH_HandleTypeDef heth;

/* Numbered "PEPTIDE MACHINE MENU" task (see MachineMenu.h/MachineMenu.c).
 * Includes "5. Robot Functions", which drops into the FR5 command shell
 * built in RobotConsole.c (see robotConsoleMenu()). */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern int robotSocket;

/* Reply buffer filled by the socket RX task, consumed in RobotCommand.c */
//char         robotReplyBuffer[1024];
//volatile int robotReplyLength = 0;

#define SIG_READY        1
//#define SIG_ROBOT_START  2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId appMainTaskHandle;
//osThreadId robotRxTaskHandle;

//osMutexId     robotMutexHandle;
//osSemaphoreId robotReplySemHandle;

/* Serialises the shared USART3 debug console between appMainTask and
 * robotRxTask so their prints don't interleave / trip HAL_BUSY. */
//osMutexId     consoleMutexHandle;

/* ---------------------------------------------------------------------------
 * Servo bus (USART6) interrupt-driven RX ring buffer
 * ---------------------------------------------------------------------------
 * The servo Modbus reader (servo.c) used to poll the one-byte RX register,
 * which dropped bytes whenever another task/LWIP preempted it mid-frame. Here
 * the USART6 RX interrupt copies each incoming byte into this ring the instant
 * it lands, so capture no longer depends on which task is scheduled. servo.c
 * drains it via ServoRxRingGet(). Size is a power of two so the index wrap is
 * a cheap AND; 256 bytes is far more than a 9-byte Modbus reply needs. */
#define SERVO_RX_RING_SIZE   256u
#define SERVO_RX_RING_MASK   (SERVO_RX_RING_SIZE - 1u)

static volatile uint8_t  servoRxRing[SERVO_RX_RING_SIZE];
static volatile uint16_t servoRxHead;   /* producer: written only by the ISR    */
static volatile uint16_t servoRxTail;   /* consumer: written only by the reader  */

/* ---------------------------------------------------------------------------
 * Static stack/TCB for appMainTask (used by the xTaskCreateStatic() call in
 * MX_FREERTOS_Init's RTOS_THREADS block below).
 *
 * These MUST live in a STANDARD USER CODE block. They used to sit in a custom
 * "USER CODE BEGIN APP_MAIN_TASK_MEMORY" section, but CubeMX only preserves the
 * USER CODE blocks it generates itself -- it does not know that custom name, so
 * it DROPPED the whole block on regenerate, leaving xTaskCreateStatic()
 * referencing undeclared symbols. Kept here in USER CODE BEGIN Variables (a
 * block CubeMX always regenerates) so it survives.
 *
 * 4096 words = 16 KB: the appMain task runs the deep menu + LWIP/robot call
 * chains, and a smaller stack overflowed. */
#define APP_MAIN_TASK_STACK_WORDS   4096
static StackType_t  xAppMainTaskStack[APP_MAIN_TASK_STACK_WORDS];
static StaticTask_t xAppMainTaskTCB;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartAppMainTask(void const *argument);
//void RobotRxTask(void const *argument);
//void startRobotRxTask(void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
		StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
	*ppxIdleTaskStackBuffer = &xIdleStack[0];
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */

//	osMutexDef(robotMutex);
//	robotMutexHandle = osMutexCreate(osMutex(robotMutex));
//
//	osMutexDef(consoleMutex);
//	consoleMutexHandle = osMutexCreate(osMutex(consoleMutex));
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
//	osSemaphoreDef(robotReplySem);
//	robotReplySemHandle = osSemaphoreCreate(osSemaphore(robotReplySem), 1);
	/* start empty */
//	osSemaphoreWait(robotReplySemHandle, 0);

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* Statically allocated -- see xAppMainTaskStack/xAppMainTaskTCB and the
	 * comment above vApplicationGetIdleTaskMemory(). tskIDLE_PRIORITY + 3
	 * is exactly the FreeRTOS priority osThreadCreate() would have computed
	 * for osPriorityNormal (CMSIS-RTOS v1 offsets everything from
	 * osPriorityIdle == -3), so this behaves identically to before. */
	appMainTaskHandle = (osThreadId)xTaskCreateStatic(
			(TaskFunction_t)StartAppMainTask,
			"appMainTask",
			APP_MAIN_TASK_STACK_WORDS,
			NULL,
			(UBaseType_t)(tskIDLE_PRIORITY + 3),
			xAppMainTaskStack,
			&xAppMainTaskTCB);

//	osThreadDef(robotRxTask, RobotRxTask, osPriorityHigh, 0, 1024);
//	robotRxTaskHandle = osThreadCreate(osThread(robotRxTask), NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */


//
//void RobotRxTask(void const *argument)
//{
//	char rxBuf[1024];
//	int  len;
//
//	/* Stay dormant -- and silent -- until the Robot Functions menu is
//	 * entered (see robotConsoleMenu()/robotConsoleTask(), which call
//	 * startRobotRxTask()). Otherwise this task used to print
//	 * "Waiting for robot connection..." from boot, before the main
//	 * menu even had a chance to show. */
//	osSignalWait(SIG_ROBOT_START, osWaitForever);
//
//	UART_Print("RX TASK STARTED\r\n");
//
//
//	// Wait for robot socket to be connected
//	if (robotSocket < 0) {
//	    UART_Print("Waiting for robot connection...\r\n");
//	}
//	while (robotSocket < 0) {
//	    osDelay(1000);
//	}
//
//	for (;;) {
//		if (robotSocket < 0) {
//			osDelay(100);
//			continue;
//		}
//
//		len = lwip_recv(robotSocket, rxBuf, sizeof(rxBuf) - 1, 0);
//
//		if (len > 0) {
//			rxBuf[len] = 0;
//			UART_Print("ROBOT RX: %s\r\n", rxBuf);
//
//			memset(robotReplyBuffer, 0, sizeof(robotReplyBuffer));
//			memcpy(robotReplyBuffer, rxBuf, len);
//			robotReplyLength = len;
//
//			osSemaphoreRelease(robotReplySemHandle);
//		} else if (len == 0) {
//			UART_Print("Robot disconnected\r\n");
//			lwip_close(robotSocket);
//			robotSocket = -1;
//		} else {
//			if (errno == EWOULDBLOCK || errno == EAGAIN || errno == ETIMEDOUT) {
//				osDelay(10);
//				continue;
//			}
//			UART_Print("RX fatal error errno=%d\r\n", errno);
//			lwip_close(robotSocket);
//			robotSocket = -1;
//		}
//	}
//}
//
//


void LAN8742_SW_Reset(void) {

	uint32_t reg;
	uint32_t start;

	/* Read BCR */
	HAL_ETH_ReadPHYRegister(&heth, LAN8742A_PHY_ADDRESS, PHY_BCR, &reg);

	/* Set reset bit */
	reg |= PHY_RESET;
	HAL_ETH_WritePHYRegister(&heth, LAN8742A_PHY_ADDRESS, PHY_BCR, reg);

	/* Wait reset done -- bounded (2s). Self-clears almost immediately in
	 * practice, but there's no reason to risk spinning forever here either
	 * if the PHY doesn't respond as expected. */
	start = osKernelSysTick();
	do {
		HAL_ETH_ReadPHYRegister(&heth, LAN8742A_PHY_ADDRESS, PHY_BCR, &reg);
		if (!(reg & PHY_RESET)) {
			break;
		}
		osDelay(50);
	} while ((osKernelSysTick() - start) < 2000);

	/* Restart autonegotiation */
	HAL_ETH_ReadPHYRegister(&heth, LAN8742A_PHY_ADDRESS, PHY_BCR, &reg);
	reg |= PHY_AUTONEGOTIATION;
	reg |= PHY_RESTART_AUTONEGOTIATION;
	HAL_ETH_WritePHYRegister(&heth, LAN8742A_PHY_ADDRESS, PHY_BCR, reg);

	/* Wait link -- BOUNDED (5s). This used to be an unconditional
	 * "while (!(reg & PHY_LINKED_STATUS));" with no timeout, which spins
	 * forever if there's no Ethernet cable connected / no live link
	 * partner -- permanently parking defaultTask here, so it could never
	 * reach the osSignalSet(appMainTaskHandle, SIG_READY) a few lines later
	 * in StartDefaultTask(). Give it 5s, then continue regardless: the menu
	 * starts anyway via StartAppMainTask()'s own bounded SIG_READY wait --
	 * only "Robot Functions" actually needs the link to be up. */
	start = osKernelSysTick();
	do {
		HAL_ETH_ReadPHYRegister(&heth, LAN8742A_PHY_ADDRESS, PHY_BSR, &reg);
		if (reg & PHY_LINKED_STATUS) {
			break;
		}
		osDelay(100);
	} while ((osKernelSysTick() - start) < 5000);

	if (reg & PHY_LINKED_STATUS) {
		UART_Print("PHY LINK UP\r\n");
	} else {
		UART_Print("PHY link not up after 5s -- continuing anyway\r\n");
	}
}


/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartDefaultTask */
	/* EVERYTHING custom now lives inside this USER CODE block so CubeMX
	 * regeneration cannot wipe it. Previously the PHY reset / netif bring-up
	 * and the "Before/After LWIP" prints sat BETWEEN the generated
	 * MX_LWIP_Init() and this block -- i.e. outside any USER CODE region --
	 * which is exactly why that code disappeared every time the .ioc was
	 * regenerated.
	 *
	 * MX_LWIP_Init() is left as the CubeMX-generated call above (it regenerates
	 * there regardless), so it must NOT be called again here. */
	UART_Print("LWIP init done\r\n");

	LAN8742_SW_Reset();
	netif_set_down(&gnetif);
	netif_set_up(&gnetif);

	osDelay(2000);                                   /* let DHCP/static IP settle */
	osSignalSet(appMainTaskHandle, SIG_READY);       /* release the application */

	for (;;) {
		osDelay(2000);
	}
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


void StartAppMainTask(void const *argument)
{
	osEvent evt;

	UART_Print("CHECKPOINT: AppMainTask waiting for SIG_READY...\r\n");

	/* Bounded wait instead of osWaitForever. StartDefaultTask() only sends
	 * SIG_READY after MX_LWIP_Init() returns (plus a 2s settle) -- if
	 * Ethernet/LWIP never comes up (e.g. no cable connected, so the PHY
	 * link-up wait inside the Ethernet driver never returns), that signal
	 * never arrives and this task -- along with the entire menu -- would
	 * otherwise wait here forever. Give it 5s (comfortably more than the
	 * normal ~2s path takes), then start the menu regardless: the load
	 * cell / servo / stepper menu items don't need networking at all, only
	 * "Robot Functions" (the FR5 socket link) does, and that will simply
	 * not work until LWIP actually comes up. */
	evt = osSignalWait(SIG_READY, 5000);

	if (evt.status == osEventTimeout) {
		UART_Print("CHECKPOINT: AppMainTask timed out waiting for SIG_READY "
				"(network/LWIP not up) -- starting menu anyway\r\n");
	} else {
		UART_Print("CHECKPOINT: AppMainTask got SIG_READY, starting menu\r\n");
	}

	/* Numbered "PEPTIDE MACHINE MENU" (Mixing Servo / Platform Servo /
	 * Dispensing Stepper / Drain Valve / Robot Functions). Robot Functions
	 * (option 5) drops into the same FR5 command shell that robotConsoleTask()
	 * used to own outright -- see robotConsoleMenu() in RobotConsole.c.
	 * The full peptide process console (peptide_app(), in
	 * Application/Peptide/) is not started -- swap the line below to
	 * bring it back when the process layer is needed again. */
	MachineMenuTask(argument);                       /* never returns */
	/* robotConsoleTask(argument); */
	/* peptide_app(); */

	for (;;) {
		osDelay(100);
	}
}

/*
 * Socket RX task: the only reader of the robot socket. It hands each reply to
 * sendRobotData() via robotReplyBuffer + robotReplySemHandle.
 */

/* Releases RobotRxTask from its dormant wait. Call this once, right when
 * the Robot Functions menu / FR5 console is actually entered. */
//void startRobotRxTask(void)
//{
//	osSignalSet(robotRxTaskHandle, SIG_ROBOT_START);
//}

/* ===========================================================================
 * Servo bus (USART6) interrupt-driven RX -- ring buffer + ISR
 * ===========================================================================
 * Declared (extern) and consumed in servo.c: ServoRxRingInit/Flush/Get.
 *
 * The ISR does NOT call any FreeRTOS API, so its priority is unconstrained by
 * configMAX_SYSCALL_INTERRUPT_PRIORITY and it is safe to run at any level.
 */

/* Enable the USART6 RX-not-empty interrupt and its NVIC line. Called once from
 * ServoInit(). MX_USART6_UART_Init() has already put the UART in TX/RX mode, so
 * we only need to switch on the RXNE interrupt source and unmask the IRQ. */
void ServoRxRingInit(void)
{
	servoRxHead = 0;
	servoRxTail = 0;

	__HAL_UART_CLEAR_OREFLAG(&huart6);

	/* Priority 6 = numerically below ETH (5) and comfortably inside the
	 * range FreeRTOS allows; it must be >= configMAX_SYSCALL priority only
	 * if it called RTOS APIs, which it doesn't -- but a safe value costs
	 * nothing. At 9600 baud bytes are ~1 ms apart, so this trivially short
	 * ISR never overruns. */
	HAL_NVIC_SetPriority(USART6_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(USART6_IRQn);

	__HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
}

/* Discard everything currently buffered and clear any overrun. Called right
 * before a request is transmitted so a reply can't be confused with stale
 * bytes or transmit echo. Advancing the tail to the head is the whole flush --
 * cheap and race-free (only the reader writes the tail). */
void ServoRxRingFlush(void)
{
	servoRxTail = servoRxHead;
	__HAL_UART_CLEAR_OREFLAG(&huart6);
}

/* Pop one byte. Returns 1 and writes *out_byte if the ring was non-empty,
 * else 0. Single-producer (ISR) / single-consumer (reader) with head/tail
 * indices needs no locking on Cortex-M: aligned 16-bit loads/stores are
 * atomic, and each side writes only its own index. */
int ServoRxRingGet(uint8_t *out_byte)
{
	if (servoRxTail == servoRxHead) {
		return 0;                       /* empty */
	}

	*out_byte = servoRxRing[servoRxTail];
	servoRxTail = (uint16_t)((servoRxTail + 1u) & SERVO_RX_RING_MASK);

	return 1;
}

/* USART6 receive ISR.
 *
 * NOTE: this handler is defined here, NOT in stm32f7xx_it.c. That is correct
 * as long as USART6's global interrupt is left DISABLED in CubeMX (it is --
 * main.c's MX_NVIC_Init() only enables ETH), so CubeMX does not also generate
 * a USART6_IRQHandler. If you ever tick "USART6 global interrupt" in the .ioc,
 * CubeMX will emit its own USART6_IRQHandler and the linker will report a
 * duplicate symbol -- in that case delete this copy and instead call the ring
 * feed from the generated handler, or just keep this one and leave the box
 * unchecked. We deliberately do NOT route through HAL_UART_IRQHandler(): HAL's
 * IT state machine is not armed for huart6 (we never call HAL_UART_Receive_IT),
 * and blocking HAL_UART_Transmit() on the same UART keeps working untouched. */
void USART6_IRQHandler(void)
{
	UART_HandleTypeDef *h = &huart6;

	/* Overrun: clear it and carry on. The overrun byte itself is already
	 * lost in hardware, but clearing ORE keeps RXNE latching for the rest
	 * of the frame; a short frame just fails CRC upstream and is retried. */
	if (__HAL_UART_GET_FLAG(h, UART_FLAG_ORE)) {
		__HAL_UART_CLEAR_OREFLAG(h);
	}

	if (__HAL_UART_GET_FLAG(h, UART_FLAG_RXNE)) {
		uint8_t d = (uint8_t)(h->Instance->RDR & 0xFFu);   /* clears RXNE */

		uint16_t next = (uint16_t)((servoRxHead + 1u) & SERVO_RX_RING_MASK);

		if (next != servoRxTail) {          /* drop byte if ring is full */
			servoRxRing[servoRxHead] = d;
			servoRxHead = next;
		}
	}
}
/* USER CODE END Application */

