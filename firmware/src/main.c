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

static char str[100];							// Used for composing UART messages
static uint8_t uart_rx_buffer[BUFFER_SIZE]; 	// UART received message buffer

static bool can_error_flag;
static uint32_t can_error_info;

static uint8_t heartVal = 0;

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
	msg_obj.msgobj = 2;
	msg_obj.mode_id = 0x232;
	msg_obj.dlc = 8;
	msg_obj.data[0] = (20000 & 0xFF00)>>8;	// Speed Request: MSB
	msg_obj.data[1] = (20000 & 0xFF); // Speed Request: LSB
	msg_obj.data[2] = 0x00; // Not Used
	msg_obj.data[3] = 0x00; // Not Used
	msg_obj.data[4] = 0x00; // Not Used
	msg_obj.data[5] = 0x01; // Set Key Mode (off: 0; On: 1; Reserved: 2; NoAction: 3)
	msg_obj.data[6] = heartVal; // alive time, gear, state
	msg_obj.data[7] = DMOC_Checksum(msg_obj); // Checksum

	LPC_CCAN_API->can_transmit(&msg_obj);
}

void sendTwo(void){
//	Board_UART_Println("Sending CAN with ID: 0x233");
	msg_obj.msgobj = 2;
	msg_obj.mode_id = 0x233;
	msg_obj.dlc = 8;
	msg_obj.data[0] = 0x75;	// Torque Request
	msg_obj.data[1] = 0x30;	// Torque Request
	msg_obj.data[2] = 0x75; // Torque Request
	msg_obj.data[3] = 0x30; // Torque Request
	msg_obj.data[4] = 0x75; // Standby Torque: Value 0
	msg_obj.data[5] = 0x30; // Standby Torque: Value 0
	msg_obj.data[6] = heartVal; // Alive time
	msg_obj.data[7] = DMOC_Checksum(msg_obj); // Checksum
	LPC_CCAN_API->can_transmit(&msg_obj);
}

void sendThree(void){
//	Board_UART_Println("Sending CAN with ID: 0x234");
	msg_obj.msgobj = 2;
	msg_obj.mode_id = 0x234;
	msg_obj.dlc = 8;
	msg_obj.data[0] = 0x00;	// Regen Power MSB
	msg_obj.data[1] = 0x00;	// Regen Power LSB
	msg_obj.data[2] = 0x00; // Accel Power MSB
	msg_obj.data[3] = 0x00; // Accel Power LSB
	msg_obj.data[4] = 0x00; // Not certain (other code says not used)
	msg_obj.data[5] = 0x60; // Something relating to temperature? Not much info
	msg_obj.data[6] = heartVal; // Alive time
	msg_obj.data[7] = DMOC_Checksum(msg_obj); // Checksum
	LPC_CCAN_API->can_transmit(&msg_obj);
}

// -------------------------------------------------------------
// Interrupt Service Routines


// -------------------------------------------------------------
// Main Program Loop

int main(void)
{

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

	DMOC_OP_STATE_T state;
	DMOC_HV_STAT_T hvstat;
	while (1) {
		if (!RingBuffer_IsEmpty(&can_rx_buffer)) {
			CCAN_MSG_OBJ_T temp_msg;
			RingBuffer_Pop(&can_rx_buffer, &temp_msg);
			Board_UART_Print("Received Message ID: 0x");
			itoa(temp_msg.mode_id, str, 16);
			Board_UART_Println(str);
			
			DMOC_Decode_State(&temp_msg,&state);
			if (temp_msg.mode_id == DMOC_STATE_ID){
				Board_UART_Println("STATE:");
				if(state.op_stat == ON){
					Board_UART_Println("ON");
				}
				else if (state.op_stat == OFF){
					Board_UART_Println("OFF");
				}
				else if (state.op_stat == POWERUP){
					Board_UART_Println("Powerup");
				}
				else if (state.op_stat == FAULT){
					Board_UART_Println("Fault");
				}
				Board_UART_Print("Speed: ");
				Board_UART_PrintNum(state.speed,10,true);
			}	
			else if(temp_msg.mode_id == DMOC_HV_STAT_ID){
				Board_UART_Println("High Voltage State:");
				DMOC_Decode_HV_Status(&temp_msg, &hvstat);
				Board_UART_Print("HV Voltage: ");
				Board_UART_PrintNum(hvstat.hv_voltage, 10, true);
				Board_UART_Print("HV Current: ");
				Board_UART_PrintNum(hvstat.hv_current, 10, true);
			}
			else if (temp_msg.mode_id == DMOC_TORQUE_STAT_ID){
				Board_UART_Println("Torque Status: ");
				Board_UART_PrintNum(DMOC_Decode_Torque_Status(&temp_msg),10,true);

			}

		}

		if (can_error_flag) {
			can_error_flag = false;
			Board_UART_Print("CAN Error: 0b");
			itoa(can_error_info, str, 2);
			Board_UART_Println(str);
		}

/*		if(lasttime < msTicks-500){
			lasttime = msTicks;
			sendOne();
			sendTwo();
			sendThree();
			heartVal = (heartVal+2) & 0x0F;
		}
*/
		uint8_t count;
		if ((count = Board_UART_Read(uart_rx_buffer, BUFFER_SIZE)) != 0) {
			Board_UART_SendBlocking(uart_rx_buffer, count); // Echo user input
			switch (uart_rx_buffer[0]) {
				case 'a':
					Board_UART_Println("Sending CAN with ID: 0x232");
					msg_obj.msgobj = 2;
					msg_obj.mode_id = 0x232;
					msg_obj.dlc = 8;
					msg_obj.data[0] = 0x00;	// Speed Request
					msg_obj.data[1] = 0x00; // Speed Request
					msg_obj.data[2] = 0x00; // Not Used
					msg_obj.data[3] = 0x00; // Not Used
					msg_obj.data[4] = 0x00; // Not Used
					msg_obj.data[5] = 0x01; // Set Key Mode (off: 0; On: 1; Reserved: 2; NoAction: 3)
					msg_obj.data[6] = 0x00; // alive time, gear, state
					msg_obj.data[7] = 0x00; // Still confused

					LPC_CCAN_API->can_transmit(&msg_obj);
					break;

				default:
					Board_UART_Println("Invalid Command");
					break;
			}
		}
	}
}
