#include "vsa_actuators_controller.h"

//================================================================================================================//
// CAN Bus Variables
static CAN_TxHeaderTypeDef TxHeader;
static CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailBox;

// Structures
relays_t relays;
motors_t motors;

// State Machine Parameters
mode_e mode = mode_init;
void (*state_machine[3])() = {init, normal, safety};

// General Global Variables
uint32_t power_off_time = 30000; // 30 seconds of interval to turn off system after power off signal arrive
uint32_t msg_motor_last_time = 0;
//================================================================================================================//

// Hardware Access
void set_relay_values(void)
{
	uint8_t b_pin3_value = (uint8_t)(relays.relay_1 == 0);
	uint8_t b_pin4_value = (uint8_t)(relays.relay_2 == 0);
	uint8_t b_pin15_value = (uint8_t)(relays.relay_3 == 0);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, b_pin3_value);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, b_pin4_value);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, b_pin15_value);
}

void set_motors_values(void)
{
	uint8_t a_pin_1_value = (uint8_t)(motors.thruster_direction == 0);
	uint8_t a_pin_2_value = (uint8_t)(motors.thruster_enable == 0);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, a_pin_1_value);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, a_pin_2_value);

	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, motors.servo_1);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, motors.servo_2);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, motors.servo_3);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, motors.servo_4);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, motors.thruster_control);
}

// Specific Purpose
void external_pin_interrupt_callback(void)
{

}

void can_rx_interrupt_callback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);


	switch(RxHeader.StdId)
	{
		// If is a relay message
		case((uint32_t)msg_relays):
		{
			if(RxData[3]) // Check the fourth byte of CAN RX array to know if this message update Relay 1 Channel
			{
				relays.relay_1 = RxData[0];
			}

			if(RxData[4]) // Check the fifth byte of CAN RX array to know if this message update Relay 2 Channel
			{
				relays.relay_2 = RxData[1];
			}

			if(RxData[5]) // Check the sixth byte of CAN RX array to know if this message update Relay 3 Channel
			{
				relays.relay_3 = RxData[2];
			}

			set_relay_values(); // Update digital outputs
			break;
		}

		// If is a motor message
		case((uint32_t)msg_motors):
		{
			msg_motor_last_time = HAL_GetTick(); // Update the variable used to monitoring motor message arrive time with current time

			if(mode == mode_normal) // Checks the current state of system. Motors only can be updated by CAN message in normal mode.
			{
				memcpy(motors.data, RxData, 7); // Update the motors structure with values incoming from CAN message
				set_motors_values(); // Update PWM outputs
			}

			break;
		}

		// If is a set mode message
		case((uint32_t)msg_set_mode):
		{
			if(RxData[1] != (uint8_t)mode_init) // Check if the new mode is different of init mode (init mode should only run when the system is powered on)
			{
				set_mode((mode_e)RxData[1]); // Set the new mode
			}
			break;
		}
	}
}

// State Machine (operations mode)
void init(void)
{
	// Initialize Relay Structure
	relays.relay_1 = GPIO_PIN_SET;
	relays.relay_2 = GPIO_PIN_SET;
	relays.relay_3 = GPIO_PIN_SET;
	set_relay_values();

	// Initialize Motor Structure with safe values and set PWM outputs
	safety();

	// Initialize CAN TX
	HAL_CAN_ActivateNotification(&hcan, TxMailBox);

	// Initialize System Power OFF CAN Message
	TxHeader.StdId = (uint32_t)msg_power_system_off;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;

	// Initialize CAN Output Buffer
	TxData[0] = 0;
	TxData[1] = 0;
	TxData[2] = 0;
	TxData[3] = 0;
	TxData[4] = 0;
	TxData[5] = 0;
	TxData[6] = 0;
	TxData[7] = 0;

	// Store to time, in milliseconds, of entry in normal mode
	// to monitoring motor message arrive interval
	msg_motor_last_time = HAL_GetTick();

	// Set normal mode to run after initialize
	set_mode(mode_normal);
}

void normal(void)
{

	// Monitor the arrival interval of engine messages to
	// place the system in safety mode if a message fails
	// to arrive within a specified time frame.
	if((HAL_GetTick() - msg_motor_last_time) >= 1000)
	{
		set_mode(mode_safety);
	}
}

void safety(void)
{
	motors.servo_1 = 128;
	motors.servo_2 = 128;
	motors.servo_3 = 128;
	motors.servo_4 = 128;
	motors.thruster_control = 0;
	motors.thruster_direction = 0;
	motors.thruster_enable = 0;

	set_motors_values();
}

// State Machine Running and Management
void run(void)
{
	(*state_machine[(uint8_t)mode])();
}

void set_mode(mode_e new_mode)
{
	mode = new_mode;
}

mode_e get_mode(void)
{
	return mode;
}
