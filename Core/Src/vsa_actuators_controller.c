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
void (*state_machine[size_of_modes])() = {init, normal};

// General Global Variables
//Timeouts
uint32_t motor_msg_timeout = 1000;
uint32_t power_off_timeout = 30000; // 30 seconds of interval to turn off system after power off signal arrive

// Timestamps
uint32_t power_off_start_time = 0;
uint32_t msg_motor_last_time = 0;

// Signal Flags
uint8_t power_off_trigger = 0;
uint8_t power_key_last_status = 0;
//================================================================================================================//

// Hardware Access
void set_relay_values(void)
{
	uint8_t a_pin15_value = (uint8_t)(relays.relay_1 == 0);
	uint8_t b_pin3_value = (uint8_t)(relays.relay_2 == 0);
	uint8_t b_pin4_value = (uint8_t)(relays.relay_3 == 0);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, a_pin15_value);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, b_pin3_value);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, b_pin4_value);
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

			memcpy(motors.data, RxData, 7); // Update the motors structure with values incoming from CAN message
			set_motors_values(); // Update PWM outputs

			break;
		}

		// If is a set mode message
		case((uint32_t)msg_set_mode):
		{
			// Check if the new mode is different of init mode (init mode should only run when the system is powered on)
			// or new mode in domain of modes vector
			if((RxData[1] != (uint8_t)mode_init) && (RxData[1] < (uint8_t)size_of_modes))
			{
				set_mode((mode_e)RxData[1]); // Set the new mode
			}
			break;
		}
	}
}

// General Purpose
void check_motor_msg_timeout(void)
{
	if((HAL_GetTick() - msg_motor_last_time) >= motor_msg_timeout)
	{
		motors.servo_1 = 128;
		motors.servo_2 = 128;
		motors.servo_3 = 128;
		motors.servo_4 = 128;
		motors.thruster_control = 0;
		motors.thruster_direction = 0;
		motors.thruster_enable = 0;

		set_motors_values();
		msg_motor_last_time = HAL_GetTick();
	}
}

void check_power_off_timeout(void)
{
	if((HAL_GetTick() - power_off_start_time) >= power_off_start_time)
	{
		HAL_GPIO_WritePin(SEAL_CONTACT_GPIO_Port, SEAL_CONTACT_Pin, GPIO_PIN_SET);
	}
}

void check_power_status(void)
{
	uint8_t power_key_status = (uint8_t)HAL_GPIO_ReadPin(POWER_KEY_GPIO_Port, POWER_KEY_Pin);

	if(power_key_status)
	{
		HAL_GPIO_WritePin(SEAL_CONTACT_GPIO_Port, SEAL_CONTACT_Pin, GPIO_PIN_RESET);
		if(!power_key_last_status)
		{
			power_key_last_status = 1;
			send_led_report(0, 0, 0);
		}
	}
	else
	{
		send_power_off();

		if(power_key_last_status)
		{
			power_off_start_time = HAL_GetTick();
			power_off_trigger = 1;
			power_key_last_status = 0;
		}
	}
}

void send_led_report(uint8_t red, uint8_t green, uint8_t blue)
{
	TxHeader.StdId = (uint32_t)msg_leds;
	TxData[0] = 0;
	TxData[1] = red;
	TxData[2] = green;
	TxData[3] = blue;
	TxData[4] = 0;
	TxData[5] = 0;
	TxData[6] = 0;
	TxData[7] = 0;

	HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);
	HAL_Delay(10);
}

void send_power_off(void)
{
	send_led_report(0, 0, 100);

	TxHeader.StdId = (uint32_t)msg_power_system_off;
	TxData[0] = 0;
	TxData[1] = 0;
	TxData[2] = 0;
	TxData[3] = 0;
	TxData[4] = 0;
	TxData[5] = 0;
	TxData[6] = 0;
	TxData[7] = 0;

	HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);
	HAL_Delay(10);
}

// State Machine (operations mode)
void init(void)
{
	// Initialize Relay Structure
	relays.relay_1 = GPIO_PIN_RESET;
	relays.relay_2 = GPIO_PIN_RESET;
	relays.relay_3 = GPIO_PIN_RESET;
	set_relay_values();

	// Force Seal Contact to Start off
	HAL_GPIO_WritePin(SEAL_CONTACT_GPIO_Port, SEAL_CONTACT_Pin, GPIO_PIN_SET);

	// Initialize Motor Structure with safe values and set PWM outputs
	motors.servo_1 = 128;
	motors.servo_2 = 128;
	motors.servo_3 = 128;
	motors.servo_4 = 128;
	motors.thruster_control = 0;
	motors.thruster_direction = 0;
	motors.thruster_enable = 0;
	set_motors_values();

	// Initialize CAN TX
	HAL_CAN_ActivateNotification(&hcan, TxMailBox);

	// Initialize CAN Message Header
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;

	// Store to time, in milliseconds, of entry in normal mode
	// to monitoring motor message arrive interval
	msg_motor_last_time = HAL_GetTick();

	// Send initialized led report
	send_led_report(0, 0, 0);

	// Set normal mode to run after initialize
	set_mode(mode_normal);
}

void normal(void)
{
	// Monitor the arrival interval of engine messages to
	// force actuators in safety values if a message fails
	// to arrive within a specified time frame.
	check_motor_msg_timeout();

	// Checks if the power switch is engaged to keep the seal-in contact active.
	// Otherwise, it initiates the system power off process.
	check_power_status();

	// Check if power off process has been started
	if(power_off_trigger)
	{
		check_power_off_timeout();
	}
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
