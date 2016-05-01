#include "board.h"
#include "dmoc645.h"

// -------------------------------------------------------------
// Macro Definitions

#define CCAN_BAUD_RATE 500000 					// Desired CAN Baud Rate
#define UART_BAUD_RATE 57600 					// Desired UART Baud Rate

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

static DMOC_OP_STATE_T state;
static DMOC_HV_STAT_T hvstat;
static uint16_t torqueStat;
static uint32_t messagesRecieved;

static uint8_t targetState;
static uint8_t targetGear;
static uint16_t targetSpeed;
static uint16_t targetTorque;

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
	// LED_On();
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

uint8_t calcChecksum(CCAN_MSG_OBJ_T msg) {
    uint8_t cs;
    uint8_t i;
    cs = msg.mode_id;
    for (i = 0; i < 7; i++)
        cs += msg.data[i];
    i = cs + 3;
    cs = ((int) 256 - i);
    return cs;
}


void sendOne(void){
//	Board_UART_Println("Sending CAN with ID: 0x232");
	msg_obj1.msgobj = 2;
	msg_obj1.mode_id = 0x232;
	msg_obj1.dlc = 8;
	msg_obj1.data[0] = ((20000+targetSpeed) & 0xFF00)>>8;	// Speed Request: MSB
	msg_obj1.data[1] = ((20000+targetSpeed) & 0xFF); // Speed Request: LSB
	msg_obj1.data[2] = 0x00; // Not Used
	msg_obj1.data[3] = 0x00; // Not Used
	msg_obj1.data[4] = 0x00; // Not Used
	msg_obj1.data[5] = 0x01; // Set Key Mode (off: 0; On: 1; Reserved: 2; NoAction: 3)
	msg_obj1.data[6] = heartVal+(targetState<<6)+(targetGear<<4); // alive time, gear, state
	msg_obj1.data[7] = DMOC_Checksum(msg_obj1); // Checksum

	LPC_CCAN_API->can_transmit(&msg_obj1);
}

void sendTwo(void){
//	Board_UART_Println("Sending CAN with ID: 0x233");
	msg_obj2.msgobj = 2;
	msg_obj2.mode_id = 0x233;
	msg_obj2.dlc = 8;
	msg_obj2.data[0] = ((0x7530+targetTorque) & 0xFF00)>>8;	// Torque Request
	msg_obj2.data[1] = ((0x7530+targetTorque) & 0x00FF);	// Torque Request
	msg_obj2.data[2] = 0x75; // Torque Request
	msg_obj2.data[3] = 0x30; // Torque Request
	msg_obj2.data[4] = 0x75; // Standby Torque: Value 0
	msg_obj2.data[5] = 0x30; // Standby Torque: Value 0
	msg_obj2.data[6] = heartVal; // Alive time
	msg_obj2.data[7] = DMOC_Checksum(msg_obj2); // Checksum
	LPC_CCAN_API->can_transmit(&msg_obj2);
}

void sendThree(void){
//	Board_UART_Println("Sending CAN with ID: 0x234");
	msg_obj3.msgobj = 2;
	msg_obj3.mode_id = 0x234;
	msg_obj3.dlc = 8;
	msg_obj3.data[0] = 0x00;	// Regen Power MSB
	msg_obj3.data[1] = 0x00;	// Regen Power LSB
	msg_obj3.data[2] = 0x27; // Accel Power MSB
	msg_obj3.data[3] = 0x00; // Accel Power LSB
	msg_obj3.data[4] = 0x00; // Not certain (other code says not used)
	msg_obj3.data[5] = 0x60; // Something relating to temperature? Not much info
	msg_obj3.data[6] = heartVal; // Alive time
	msg_obj3.data[7] = DMOC_Checksum(msg_obj3); // Checksum
	LPC_CCAN_API->can_transmit(&msg_obj3);
}


void printMenu(void){
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
	Board_UART_Println("---------------------------------------------\n\n");
}

void printState(void){
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

void printHVStuff(void){
	Board_UART_Println("---------------------------------------------");
	Board_UART_Println("High Voltage State:");
	Board_UART_Print("HV Voltage: ");
	Board_UART_PrintNum(hvstat.hv_voltage, 10, true);
	Board_UART_Print("HV Current: ");
	Board_UART_PrintNum(hvstat.hv_current, 10, true);
	Board_UART_Println("---------------------------------------------");
}

void printTorqueStuff(void){
	Board_UART_Println("---------------------------------------------");
	Board_UART_Println("Torque Status: ");
	Board_UART_PrintNum(torqueStat,10,true);
	Board_UART_Println("---------------------------------------------\n\n");
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

	//---------------
	// Initialize UART Communication
	Board_UART_Init(UART_BAUD_RATE);
	Board_UART_Println("Started up");

	//---------------
	// Initialize CAN  and CAN Ring Buffer

	RingBuffer_Init(&can_rx_buffer, _rx_buffer, sizeof(CCAN_MSG_OBJ_T), BUFFER_SIZE);
	RingBuffer_Flush(&can_rx_buffer);

	Board_CAN_Init(CCAN_BAUD_RATE, CAN_rx, CAN_tx, CAN_error);

	// For your convenience.
	// typedef struct CCAN_MSG_OBJ {
	// 	uint32_t  mode_id;
	// 	uint32_t  mask;
	// 	uint8_t   data[8];
	// 	uint8_t   dlc;
	// 	uint8_t   msgobj;
	// } CCAN_MSG_OBJ_T;

	/* [Tutorial] How do filters work?

		Incoming ID & Mask == Mode_ID for msgobj to accept message

		Incoming ID : 0xabc
		Mask: 		  0xF0F &
		            -----------
		              0xa0c

		mode_id == 0xa0c for msgobj to accept message

	*/

	msg_obj.msgobj = 1;
	msg_obj.mode_id = 0x000;
	msg_obj.mask = 0x000;
	LPC_CCAN_API->config_rxmsgobj(&msg_obj);

	/* [Tutorial] How do I send a CAN Message?

		There are 32 Message Objects in the CAN Peripherals Message RAM.
		We need to pick one that isn't setup for receiving messages and use it to send.

		For this exmaple we'll pick 31

		msg_obj.msgobj = 31;
		msg_obj.mode_id = 0x600; 		// CAN ID of Message to Send
		msg_obj.dlc = 8; 				// Byte length of CAN Message
		msg_obj.data[0] = 0xAA; 		// Fill your bytes here
		msg_obj.data[1] = ..;
		..
		msg_obj.data[7] = 0xBB:

		Now its time to send
		LPC_CCAN_API->can_transmit(&msg_obj);

	*/

	can_error_flag = false;
	can_error_info = 0;

	uint32_t lasttime = msTicks;

	uint8_t count;

	bool menu=true;
	bool listen=false;
	messagesRecieved=0;
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

		uint8_t count;
		if ((count = Board_UART_Read(uart_rx_buffer, BUFFER_SIZE)) != 0) {
			//Board_UART_SendBlocking(uart_rx_buffer, count); // Echo user input
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
				case 'p':
					targetSpeed = 1000;
					break;
				case 'm':
					targetSpeed = 500;
					break;
				case 'l':
					targetSpeed = 0;
					targetTorque = 0;
					break;
				case 'o':
					targetTorque = 1000;
					break;
				case 'k':
					targetTorque = 0;
					break;
				default:
					Board_UART_Println("Invalid Command");
					break;
			}
		}
	}
}
