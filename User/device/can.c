/* 
	CAN总线上的设
	将所有CAN总线上挂载的设抽象成一设进行配和控制
*/

/* Includes ------------------------------------------------------------------*/
#include "can.h"

#include <stdbool.h>
#include <string.h>

#include "bsp\can.h"

#include "component\user_math.h"

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile uint8_t motor_received = 0;
static volatile uint32_t unknown_message = 0;

static CAN_RxHeaderTypeDef rx_header;
static uint8_t rx_data[8];

static CAN_t *gcan;
static bool inited = false;

/* Private function  ---------------------------------------------------------*/
static void CAN_Motor_Decode(CAN_MotorFeedback_t *fb, const uint8_t *raw) {
	fb->rotor_angle = (uint16_t)((raw[0] << 8) | raw[1]);
	fb->rotor_speed = (int16_t)((raw[2] << 8) | raw[3]);
	fb->torque_current = (int16_t)((raw[4] << 8) | raw[5]);
	fb->temp = raw[6];
	
	motor_received++;
}

static void CAN_UWB_Decode(CAN_UWBFeedback_t *fb, const uint8_t *raw) {
	memcmp(fb->raw,raw,8);
}

static void CAN_SuperCap_Decode(CAN_SuperCapFeedback_t *fb, const uint8_t *raw) {
	// TODO
	(void)fb;
	(void)raw;
}

static void CAN_RxFifo0MsgPendingCallback(void) {
	HAL_CAN_GetRxMessage(BSP_CAN_GetHandle(BSP_CAN_1), CAN_RX_FIFO0, &rx_header, rx_data);
	
	uint32_t index;
	switch (rx_header.StdId) {
		case CAN_M3508_M1_ID:
		case CAN_M3508_M2_ID:
		case CAN_M3508_M3_ID:
		case CAN_M3508_M4_ID:
			index = rx_header.StdId - CAN_M3508_M1_ID;
			CAN_Motor_Decode(&(gcan->chassis_motor_fb[index]), rx_data);
			break;
		
		case CAN_M3508_FRIC1_ID:
		case CAN_M3508_FRIC2_ID:
			index = rx_header.StdId - CAN_M3508_FRIC1_ID;
			CAN_Motor_Decode(&(gcan->gimbal_motor_fb.fric_fb[index]), rx_data);
			break;
		
		case CAN_M2006_TRIG_ID:
			CAN_Motor_Decode(&(gcan->gimbal_motor_fb.trig_fb), rx_data);
			break;
		
		case CAN_GM6020_YAW_ID:
			CAN_Motor_Decode(&(gcan->gimbal_motor_fb.yaw_fb), rx_data);
			break;
		
		case CAN_GM6020_PIT_ID:
			CAN_Motor_Decode(&(gcan->gimbal_motor_fb.pit_fb), rx_data);
			break;

		case CAN_SUPERCAP_FEEDBACK_ID_BASE:
			CAN_SuperCap_Decode(&(gcan->supercap_feedback), rx_data);
			osThreadFlagsSet(gcan->supercap_alert, SIGNAL_CAN_SUPERCAP_RECV);
			break;
		
		default:
			unknown_message++;
			break;
	}
	
	if (motor_received > CAN_CHASSIS_NUM_MOTOR) {
		for (uint8_t i = 0; i < gcan->motor_alert_len; i++) {
			if (gcan->motor_alert[i]) {
				osThreadFlagsSet(gcan->motor_alert, SIGNAL_CAN_MOTOR_RECV);
			}
		}
		motor_received = 0;
	}
}

static void CAN_RxFifo1MsgPendingCallback(void) {
	HAL_CAN_GetRxMessage(BSP_CAN_GetHandle(BSP_CAN_1), CAN_RX_FIFO1, &rx_header, rx_data);
	
	switch (rx_header.StdId) {
		case CAN_UWB_FEEDBACK_ID_BASE:
			CAN_UWB_Decode(&(gcan->uwb_feedback), rx_data);
			osThreadFlagsSet(gcan->uwb_alert, SIGNAL_CAN_UWB_RECV);
			break;
		
		default:
			unknown_message++;
			break;
	}
}

/* Exported functions --------------------------------------------------------*/
int8_t CAN_Init(
	CAN_t *can,
	osThreadId_t *motor_alert,
	uint8_t motor_alert_len,
	osThreadId_t uwb_alert,
	osThreadId_t supercap_alert) {
		
	if (can == NULL)
		return DEVICE_ERR_NULL;
	
	if (motor_alert == NULL)
		return DEVICE_ERR_NULL;
	
	if (inited)
		return DEVICE_ERR_INITED;
	
	can->motor_alert_len = motor_alert_len;
	can->motor_alert = motor_alert;
	can->uwb_alert = uwb_alert;
	can->supercap_alert = supercap_alert;
	
	CAN_FilterTypeDef  can_filter = {0};

	can_filter.FilterBank = 0;
	can_filter.FilterIdHigh = 0;
	can_filter.FilterIdLow  = 0;
	can_filter.FilterMode =  CAN_FILTERMODE_IDLIST;
	can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter.FilterMaskIdHigh = 0;
	can_filter.FilterMaskIdLow  = 0;
	can_filter.FilterActivation = ENABLE;
	can_filter.SlaveStartFilterBank  = 14;
	can_filter.FilterFIFOAssignment = CAN_MOTOR_CAN_RX_FIFO;
	
	HAL_CAN_ConfigFilter(BSP_CAN_GetHandle(BSP_CAN_1), &can_filter);
	HAL_CAN_Start(BSP_CAN_GetHandle(BSP_CAN_1));
	BSP_CAN_RegisterCallback(BSP_CAN_1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB, CAN_RxFifo0MsgPendingCallback);
	HAL_CAN_ActivateNotification(BSP_CAN_GetHandle(BSP_CAN_1), CAN_IT_RX_FIFO0_MSG_PENDING);

	
	can_filter.FilterBank = 14;
	can_filter.FilterFIFOAssignment = CAN_UWB_CAN_RX_FIFO;
	
	HAL_CAN_ConfigFilter(BSP_CAN_GetHandle(BSP_CAN_2), &can_filter);
	HAL_CAN_Start(BSP_CAN_GetHandle(BSP_CAN_2));
	BSP_CAN_RegisterCallback(BSP_CAN_2, HAL_CAN_RX_FIFO1_MSG_PENDING_CB, CAN_RxFifo1MsgPendingCallback);
	HAL_CAN_ActivateNotification(BSP_CAN_GetHandle(BSP_CAN_2), CAN_IT_RX_FIFO1_MSG_PENDING);

	gcan = can;
	inited = true;
	return DEVICE_OK;
}

CAN_t *CAN_GetDevice(void) {
	if (inited) {
		return gcan;
	}
	return NULL;
}

int8_t CAN_Motor_ControlChassis(float m1, float m2, float m3, float m4) {
	int16_t motor1 = (int16_t)(m1 * (float)CAN_M3508_MAX_ABS_VOLT);
	int16_t motor2 = (int16_t)(m2 * (float)CAN_M3508_MAX_ABS_VOLT);
	int16_t motor3 = (int16_t)(m3 * (float)CAN_M3508_MAX_ABS_VOLT);
	int16_t motor4 = (int16_t)(m4 * (float)CAN_M3508_MAX_ABS_VOLT);
	
	CAN_TxHeaderTypeDef tx_header;

	tx_header.StdId = CAN_M3508_M2006_RECEIVE_ID_BASE;
	tx_header.IDE   = CAN_ID_STD;
	tx_header.RTR   = CAN_RTR_DATA;
	tx_header.DLC   = 8;

	uint8_t tx_data[8];
	tx_data[0] = (uint8_t)((motor1 >> 8) & 0xFF);
	tx_data[1] = (uint8_t)(motor1 & 0xFF);
	tx_data[2] = (uint8_t)((motor2 >> 8) & 0xFF);
	tx_data[3] = (uint8_t)(motor2 & 0xFF);
	tx_data[4] = (uint8_t)((motor3 >> 8) & 0xFF);
	tx_data[5] = (uint8_t)(motor3 & 0xFF);
	tx_data[6] = (uint8_t)((motor4 >> 8) & 0xFF);
	tx_data[7] = (uint8_t)(motor4 & 0xFF);
	
	HAL_CAN_AddTxMessage(BSP_CAN_GetHandle(BSP_CAN_1), &tx_header, tx_data, (uint32_t*)CAN_TX_MAILBOX0); 
	
	return DEVICE_OK;
}

int8_t CAN_Motor_ControlGimbal(float yaw, float pitch) {
	int16_t yaw_motor = (int16_t)(yaw * (float)CAN_GM6020_MAX_ABS_VOLT);
	int16_t pit_motor = (int16_t)(pitch * (float)CAN_GM6020_MAX_ABS_VOLT);
	
	CAN_TxHeaderTypeDef tx_header;

	tx_header.StdId = CAN_GM6020_RECEIVE_ID_EXTAND;
	tx_header.IDE   = CAN_ID_STD;
	tx_header.RTR   = CAN_RTR_DATA;
	tx_header.DLC   = 8;

	uint8_t tx_data[8];
	tx_data[0] = (uint8_t)((yaw_motor  >> 8) & 0xFF);
	tx_data[1] = (uint8_t)(yaw_motor & 0xFF);
	tx_data[2] = (uint8_t)((pit_motor  >> 8) & 0xFF);
	tx_data[3] = (uint8_t)(pit_motor & 0xFF);
	tx_data[4] = 0;
	tx_data[5] = 0;
	tx_data[6] = 0;
	tx_data[7] = 0;
	
	HAL_CAN_AddTxMessage(BSP_CAN_GetHandle(BSP_CAN_1), &tx_header, tx_data, (uint32_t*)CAN_TX_MAILBOX0); 
	
	return DEVICE_OK;
}

int8_t CAN_Motor_ControlShoot(float fric1, float fric2, float trig) {
	int16_t fric1_motor = (int16_t)(fric1 * (float)CAN_M3508_MAX_ABS_VOLT);
	int16_t fric2_motor = (int16_t)(fric2 * (float)CAN_M3508_MAX_ABS_VOLT);
	int16_t trig_motor = (int16_t)(trig * (float)CAN_M2006_MAX_ABS_VOLT);
	
	CAN_TxHeaderTypeDef tx_header;

	tx_header.StdId = CAN_M3508_M2006_RECEIVE_ID_EXTAND;
	tx_header.IDE   = CAN_ID_STD;
	tx_header.RTR   = CAN_RTR_DATA;
	tx_header.DLC   = 8;
	
	uint8_t tx_data[8];
	tx_data[0] = (uint8_t)((fric1_motor  >> 8) & 0xFF);
	tx_data[1] = (uint8_t)(fric1_motor & 0xFF);
	tx_data[2] = (uint8_t)((fric2_motor  >> 8) & 0xFF);
	tx_data[3] = (uint8_t)(fric2_motor & 0xFF);
	tx_data[4] = (uint8_t)((trig_motor  >> 8) & 0xFF);
	tx_data[5] = (uint8_t)(trig_motor & 0xFF);
	tx_data[6] = 0;
	tx_data[7] = 0;
	
	HAL_CAN_AddTxMessage(BSP_CAN_GetHandle(BSP_CAN_1), &tx_header, tx_data, (uint32_t*)CAN_TX_MAILBOX0);
	
	return DEVICE_OK;
}

int8_t CAN_Motor_QuickIdSetMode(void) {
	CAN_TxHeaderTypeDef tx_header;

	tx_header.StdId = CAN_M3508_M2006_ID_SETTING_ID;
	tx_header.IDE   = CAN_ID_STD;
	tx_header.RTR   = CAN_RTR_DATA;
	tx_header.DLC   = 8;
	
	uint8_t tx_data[8];

	HAL_CAN_AddTxMessage(BSP_CAN_GetHandle(BSP_CAN_1), &tx_header, tx_data, (uint32_t*)CAN_TX_MAILBOX0); 
	return DEVICE_OK;
}