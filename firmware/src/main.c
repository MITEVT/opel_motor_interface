#include "board.h"
#include "dmoc645.h"

// -------------------------------------------------------------
// Macro Definitions

#define CCAN_BAUD_RATE 500000 					// Desired CAN Baud Rate
#define UART_BAUD_RATE 19200 					// Desired UART Baud Rate

#define BUFFER_SIZE 8

// -------------------------------------------------------------
// Static Variable Declaration

extern volatile uint32_t msTicks;

static CCAN_MSG_OBJ_T msg_obj; 					// Message Object data structure for manipulating CAN messages
static RINGBUFF_T can_rx_buffer;				// Ring Buffer for storing received CAN messages
static CCAN_MSG_OBJ_T _rx_buffer[BUFFER_SIZE]; 	// Underlying array used in ring buffer

static CCAN_MSG_OBJ_T msg_obj1; 					// Message Object data structure for manipulating CAN messages
static CCAN_MSG_OBJ_T msg_obj2; 					// Message Object data structure for manipulating CAN messages
static CCAN_MSG_OBJ_T msg_obj3; 					// Message Object data structure for manipulating CAN messages

static CCAN_MSG_OBJ_T msg_obj4; 					// Message Object data structure for manipulating CAN messages

static char str[100];							// Used for composing UART messages
static uint8_t uart_rx_buffer[BUFFER_SIZE]; 	// UART received message buffer

static bool can_error_flag;
static uint32_t can_error_info;

static uint8_t heartVal = 0;

static DMOC_OP_STATE_T state;			// Keeps track of the state of the motor controller, including mode and speed 
static DMOC_HV_STAT_T hvstat;			// Keeps track of the high voltage state of the motor controller
static uint16_t torqueStat;				// Keeps track of the torque state
static uint32_t messagesRecieved;		// Tracks the number of messages received for debugging. Remove before final build

static uint8_t targetState;				// The state that we are telling the motor controller to become
static uint8_t targetGear;				// The "gear" (forward, reverse, neutral) that we are telling the motor controller to become
static uint16_t targetSpeed;			// The current speed request sent to the motor controller in the speed ramp
static uint16_t targetTorque;			// The torque that we would like to motor controller to apply to the motor (can act as bounds)
static uint16_t reqSpeed;				// The speed that we would like the motor controller to spin the motor
static uint32_t lastRamp;				// The last time the speed was increased, used to make a more gentle ramp with speed changes
static bool speedset;					// If the speed is currently being set, and the chip should look for numerical inputs
static uint16_t testSpeed;				// Used to assemble the speed input and verify that it is within reasonable bounds 

// -------------------------------------------------------------
// Helper Functions

/**
 * Delay the processor for a given number of milliseconds
 * @param ms Number of milliseconds to delay
 */
void _delay(uint32_t ms) {
	uint32_t curTicks = msTicks;
	while ((msTicks - curTicks) < ms);
}

// -------------------------------------------------------------
// CAN Driver Callback Functions

/*	CAN receive callback */
/*	Function is executed by the Callback handler after
    a CAN message has been received */
void CAN_rx(uint8_t msg_obj_num) {
	/* Determine which CAN message has been received */
	msg_obj.msgobj = msg_obj_num;
	/* Now load up the msg_obj structure with the CAN message */
	LPC_CCAN_API->can_receive(&msg_obj);
	if (msg_obj_num == 1) {
		RingBuffer_Insert(&can_rx_buffer, &msg_obj);
	}
}

/*	CAN transmit callback */
/*	Function is executed by the Callback handler after
    a CAN message has been transmitted */
void CAN_tx(uint8_t msg_obj_num) {
	msg_obj_num = msg_obj_num;
}

/*	CAN error callback */
/*	Function is executed by the Callback handler after
    an error has occurred on the CAN bus */
void CAN_error(uint32_t error_info) {
	can_error_info = error_info;
	can_error_flag = true;
}

// -------------------------------------------------------------
// DMOC required messages

/* 	Checksum calculations */
/*	Function takes a CAN message, and calculates the required
 *	checksum for the message
 *
 *	@return: The checksum */
static uint8_t inline calcChecksum(CCAN_MSG_OBJ_T msg) {
    uint8_t cs;
    uint8_t i;
    cs = msg.mode_id;
    for (i = 0; i < 7; i++)
        cs += msg.data[i];
    i = cs + 3;
    cs = ((int) 256 - i);
    return cs;
}

/*	First Required Message - States */
/* 	Function sends the first of the required CAN messages
 *
 *  The first one sends state information and a speed request.
 *  
 *  The first two bytes are the target speed, which is calculated by offsettng it by 20000
 *  
 *  Then there are three supposedly unused bytes
 *  
 *  The next byte determines the "key mode" of the car, if it is on or off
 *
 *  Next, There is a byte that contains lots of information. It gives a heartbeat val
 *  that is calculated based upon the number of messages previously sent. Next, it 
 *  also contains the target state and gear of the the vehicle. Telling the motor
 *  controller that it should be in enabled and moving forward, by example. 
 *
 * 	Finally, the last byte is the checksum
 *
 * 	@return: Nothing
 */

static void inline sendOne(void){
	msg_obj1.msgobj = 2;
	msg_obj1.mode_id = 0x232;
	msg_obj1.dlc = 8;
	msg_obj1.data[0] = ((20000+targetSpeed) & 0xFF00)>>8;	// Speed Request: MSB
	msg_obj1.data[1] = ((20000+targetSpeed) & 0xFF); // Speed Request: LSB
	msg_obj1.data[2] = 0x00; // Not Used
	msg_obj1.data[3] = 0x00; // Not Used
	msg_obj1.data[4] = 0x00; // Not Used
	msg_obj1.data[5] = 0x01; // Set Key Mode (off: 0; On: 1; Reserved: 2; NoAction: 3)
	msg_obj1.data[6] = heartVal|(targetState<<6)|(targetGear<<4); // alive time, gear, state
	msg_obj1.data[7] = DMOC_Checksum(msg_obj1); // Checksum

	LPC_CCAN_API->can_transmit(&msg_obj1);
}

/* 	Second Required Message - Torques */
/*	Function sends the second of the required CAN messages
 *
 * 	The second one sends torque information.
 *
 *	The first two bytes transmit the torque upper limit.
 *
 *	The next two bytes transmit the torque lower limit.
 *
 *	If the upper and lower limits are the same, then the motor controller
 *	is being used in torque mode, rather than speed mode
 *
 *	Next are the "standby torque" values, which, because the documentation
 *	that I looked at didn't have a good explaination, I am leaving at a 
 *	value of zero for now.
 *
 *	Finally, the last two bytes contains the same heartbeat value as the other messages
 *	and the checksum for the overall message
 *
 *	@return: Nothing
 */	
static void inline sendTwo(void){
	msg_obj2.msgobj = 2;
	msg_obj2.mode_id = 0x233;
	msg_obj2.dlc = 8;
	msg_obj2.data[0] = ((0x7530+targetTorque) & 0xFF00)>>8;	// Torque Request
	msg_obj2.data[1] = ((0x7530+targetTorque) & 0x00FF);	// Torque Request
	msg_obj2.data[2] = ((0x7530-targetTorque) & 0xFF00)>>8; // Torque Request
	msg_obj2.data[3] = ((0x7530-targetTorque) & 0x00FF); // Torque Request
	msg_obj2.data[4] = 0x75; // Standby Torque: Value 0
	msg_obj2.data[5] = 0x30; // Standby Torque: Value 0
	msg_obj2.data[6] = heartVal; // Alive time
	msg_obj2.data[7] = DMOC_Checksum(msg_obj2); // Checksum
	LPC_CCAN_API->can_transmit(&msg_obj2);
}

/*	Third required Message - Power */

/*	Function sends the third and final required CAN message
 *	It contains information on the power
 *	
 *	The first two bytes are the power allowed for Regen
 *
 *	The next two bytes are the power allowed for acceleration
 *
 * 	After that, it becomes a bit confusing. The resources that I
 * 	looked at were a little bit unclear on two of the bytes.
 *
 * 	There is then the same heartbeat values.
 *	
 *	The last byte is the checksum
 *
 *	@return: Nothing
 */
static void inline sendThree(void){
	msg_obj3.msgobj = 2;
	msg_obj3.mode_id = 0x234;
	msg_obj3.dlc = 8;
	msg_obj3.data[0] = 0x27;	// Regen Power MSB
	msg_obj3.data[1] = 0x00;	// Regen Power LSB
	msg_obj3.data[2] = 0x27; // Accel Power MSB
	msg_obj3.data[3] = 0x00; // Accel Power LSB
	msg_obj3.data[4] = 0x00; // Not certain (other code says not used)
	msg_obj3.data[5] = 0x60; // Something relating to temperature? Not much info
	msg_obj3.data[6] = heartVal; // Alive time
	msg_obj3.data[7] = DMOC_Checksum(msg_obj3); // Checksum
	LPC_CCAN_API->can_transmit(&msg_obj3);
}


/* Prints a help menu to allow people to more easily interact with the program */
static void inline printMenu(void){
	Board_UART_Println("---------------------------------------------");
	Board_UART_Println("You are in a forest and encounter a strange metal box");
	Board_UART_Print("You've recieved ");
	Board_UART_PrintNum(messagesRecieved,10,false);
	Board_UART_Println(" messages from your environment.");
	Board_UART_Println("What do you do?");
	Board_UART_Println("v) Inspect high voltage");
	Board_UART_Println("s) Discuss State of box");
	Board_UART_Println("t) Inspect Torque Output");
	Board_UART_Println("h) Examine your surroundings");
	Board_UART_Println("0) Disable The Box");
	Board_UART_Println("1) Start the Box");
	Board_UART_Println("2) Enable the box");
	Board_UART_Println("n) Put the box in neutral");
	Board_UART_Println("f) Put the box in forward");
	Board_UART_Println("o) Raise torque limit");
	Board_UART_Println("k) Deprive it of torque");
	Board_UART_Println("m) 500 rmp");
	Board_UART_Println("p) 1000 rmp");
	Board_UART_Println("l) Deprive it of speed");
	Board_UART_Println("---------------------------------------------");
	Board_UART_Println("Recommended keypresses for demos:");
	Board_UART_Println("Ensure that the motor controller responds before you proceed in the steps");
	Board_UART_Println("1  2  f  o  m  p");
	Board_UART_Println("Disable to the motor by working backwords with:");
	Board_UART_Println("l  k  n  1  0");
	Board_UART_Println("---------------------------------------------\n\n");
}

/* Prints the current state of the motor controller */
static void inline printState(void){
	Board_UART_Println("---------------------------------------------");
	Board_UART_Print("STATE: ");
	if(state.op_stat == POWERUP){
		Board_UART_Println("POWERUP");
	}
	else if (state.op_stat == DISABLED){
		Board_UART_Println("DISABLED");
	}
	else if (state.op_stat == ENABLED){
		Board_UART_Println("ENABLED");
	}
	else if (state.op_stat == POWERDOWN){
		Board_UART_Println("POWERDOWN");
	}
	else if (state.op_stat == FAULT){
		Board_UART_Println("FAULT");
	}
	else if (state.op_stat == CRITICAL_FAULT){
		Board_UART_Println("Critical Fault");
	}
	else{
		Board_UART_Print("0x");
		Board_UART_PrintNum(state.op_stat,16,true);
	}
	Board_UART_Print("Speed: ");
	Board_UART_PrintNum(state.speed,10,true);
	Board_UART_Println("---------------------------------------------\n\n");
}


/* Prints the high voltage information known about the motor controller */
static void inline printHVStuff(void){
	Board_UART_Println("---------------------------------------------");
	Board_UART_Println("High Voltage State:");
	Board_UART_Print("HV Voltage: ");
	Board_UART_PrintNum(hvstat.hv_voltage, 10, true);
	Board_UART_Print("HV Current: ");
	Board_UART_PrintNum(hvstat.hv_current, 10, true);
	Board_UART_Println("---------------------------------------------");
}


/* Prints the current torque status of the motor controller */
static void inline printTorqueStuff(void){
	Board_UART_Println("---------------------------------------------");
	Board_UART_Println("Torque Status: ");
	Board_UART_PrintNum(torqueStat,10,true);
	Board_UART_Println("---------------------------------------------\n\n");
}

/* Enables the motor controller by sending the RS-232 message "ab" */
static void inline enableDMOC(void){
	Board_UART_Println("Enabling DMOC");
	while((LPC_USART->LSR & 0x40)==0);
	Board_DMOC_Comm_Enable();
	Board_UART_Println("ab");
	while((LPC_USART->LSR & 0x40)==0);
	Board_DMOC_Comm_Disable();
}


// -------------------------------------------------------------
// Interrupt Service Routines


// -------------------------------------------------------------
// Main Program Loop

int main(void)
{
	targetState = 0;
	targetGear = 0;
	targetSpeed = 0;
	targetTorque = 0;
	reqSpeed = 0;

	//---------------
	// Initialize SysTick Timer to generate millisecond count
	if (Board_SysTick_Init()) {
		// Unrecoverable Error. Hang.
		while(1);
	}

	//---------------
	// Initialize GPIO and LED as output
	Board_LEDs_Init();
	Board_LED_On(LED0);
	Board_DMOC_Comm_Init();
	Board_DMOC_Comm_Disable();
	//---------------
	// Initialize UART Communication
	Board_UART_Init(UART_BAUD_RATE);
	Board_UART_Println("Started up");

	//---------------
	// Initialize CAN  and CAN Ring Buffer

	RingBuffer_Init(&can_rx_buffer, _rx_buffer, sizeof(CCAN_MSG_OBJ_T), BUFFER_SIZE);
	RingBuffer_Flush(&can_rx_buffer);

	Board_CAN_Init(CCAN_BAUD_RATE, CAN_rx, CAN_tx, CAN_error);	
	msg_obj.msgobj = 1;
	msg_obj.mode_id = 0x000;
	msg_obj.mask = 0x000;
	LPC_CCAN_API->config_rxmsgobj(&msg_obj);

	can_error_flag = false;
	can_error_info = 0;


	uint32_t lasttime = msTicks;

	uint8_t count;


	speedset = false;
	messagesRecieved=0;
	lastRamp = 0;
	while (1) {
		if (!RingBuffer_IsEmpty(&can_rx_buffer)) {
			CCAN_MSG_OBJ_T temp_msg;
			RingBuffer_Pop(&can_rx_buffer, &temp_msg);
			messagesRecieved++;
				if (temp_msg.mode_id == DMOC_STATE_ID)
					DMOC_Decode_State(&temp_msg,&state);

				else if(temp_msg.mode_id == DMOC_HV_STAT_ID){
					DMOC_Decode_HV_Status(&temp_msg, &hvstat);
				}
				else if (temp_msg.mode_id == DMOC_TORQUE_STAT_ID){
					torqueStat = DMOC_Decode_Torque_Status(&temp_msg);

			}

		}

		if (can_error_flag) {
			can_error_flag = false;
			Board_UART_Print("CAN Error: 0b");
			itoa(can_error_info, str, 2);
			Board_UART_Println(str);



			Board_CAN_Init(CCAN_BAUD_RATE, CAN_rx, CAN_tx, CAN_error);
			msg_obj.msgobj = 1;
			msg_obj.mode_id = 0x000;
			msg_obj.mask = 0x000;
			LPC_CCAN_API->config_rxmsgobj(&msg_obj);
		}

		if(lasttime < msTicks-250){
			lasttime = msTicks;
			sendOne();
			_delay(20);
			sendTwo();
			_delay(20);
			sendThree();
			heartVal = (heartVal+2) & 0x0F;
		}
		
		if(lastRamp < msTicks-2 && targetSpeed<reqSpeed){
			targetSpeed++;
			lastRamp = msTicks;
		}
		else if(lastRamp < msTicks-2 && targetSpeed>reqSpeed){
			targetSpeed--;
			lastRamp = msTicks;
		}
		uint8_t count;
		if ((count = Board_UART_Read(uart_rx_buffer, BUFFER_SIZE)) != 0) {
			if(!speedset){
				switch (uart_rx_buffer[0]) {
					case 's':
						printState();
						break;
					case 'v':
						printHVStuff();
						break;
					case 't':
						printTorqueStuff();
						break;
					case 'h':
						printMenu();
						break;
					case 'z':
						Board_UART_Println("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
						break;
					case '1':
						targetState = 1;
						break;
					case '2':
						targetState = 2;
						break;
					case '0':
						targetState = 0;
						break;
					case 'n':
						targetGear = 0;
						break;
					case 'f':
						targetGear = 1;
						break;
					case 'r':
						targetGear = 2;
						break;
					case 'p':
						reqSpeed = 1000;
						Board_UART_Println("Setting Speed to 1000 rmp");
						break;
					case 'm':
						reqSpeed = 500;
						break;
					case 'l':
						reqSpeed = 0;
						targetSpeed = 0;
						break;
					case 'o':
						targetTorque = 1000;
						break;
					case 'k':
						targetTorque = 0;
						break;
					case 'b':
						Board_UART_Println("Set a target speed in rpm:");
						speedset = true;
						testSpeed = 0;
						break;
					case 'g':
						enableDMOC();
						break;
					default:
						Board_UART_Println("Invalid Command");
						break;
				}
			}
			else{
				char tmp = uart_rx_buffer[0];
				Board_UART_SendBlocking(&tmp, 1);
				if (tmp>47 && tmp<58){
						testSpeed = (testSpeed * 10)+(tmp-48);
				}
				else if(uart_rx_buffer[0] == '\n'){
					Board_UART_Println("");
					if(testSpeed < 2500){
						Board_UART_Print("Setting Speed in rpm to: ");
						Board_UART_PrintNum(testSpeed,10,true);
						reqSpeed = testSpeed;
					}
					else{
						Board_UART_Println("Speed out of bounds");
					}
					Board_UART_Println("---------------------------------------\n");
					speedset = false;
				}
				else{
					Board_UART_Println("Invalid speed");
					speedset = false;
				}
			}
		}
	}
}
