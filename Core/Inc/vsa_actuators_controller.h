#ifndef __VSA_ACTUATORS_CONTROLLER__
#define __VSA_ACTUATORS_CONTROLLER__

#include <stdint.h>
#include <string.h>

#include "can.h"
#include "tim.h"
#include "gpio.h"

typedef enum
{
	mode_init = 0,
	mode_normal = 1,
	mode_safety = 2
} mode_e;

typedef enum
{
	msg_motors = 1,
	msg_set_mode = 2,
	msg_relays = 3,
	msg_power_system_off = 4
} message_e;

typedef union
{
	struct
	{
		uint8_t servo_1;
		uint8_t servo_2;
		uint8_t servo_3;
		uint8_t servo_4;
		uint8_t thruster_direction;
		uint8_t thruster_control;
		uint8_t thruster_enable;
	};

	uint8_t data[7];
} motors_t;

typedef union
{
	struct
	{
		uint8_t relay_1;
		uint8_t relay_2;
		uint8_t relay_3;
	};

	uint8_t data[3];
} relays_t;


// Hardware Access
void set_relay_values(void);
void set_motors_values(void);

// Specific Purpose
void external_pin_interrupt_callback(void);
void can_rx_interrupt_callback(CAN_HandleTypeDef *hcan);

// State Machine (operations mode)
void init(void);
void normal(void);
void safety(void);

// State Machine Running and Management
void run(void);
void set_mode(mode_e new_mode);
mode_e get_mode(void);

#endif
