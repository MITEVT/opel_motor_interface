#ifndef __BOARD_H_
#define __BOARD_H_

#include "chip.h"
#include "util.h"
#include <string.h>

// -------------------------------------------------------------
// Global Variables

extern const uint32_t OscRateIn; 				/** @brief Board Oscillator Frequency (Hz) **/
volatile uint32_t msTicks; 						/** @brief System Time (ms) **/

// -------------------------------------------------------------
// Configuration Macros


// -------------------------------------------------------------
// Pin Descriptions

#define LED0_PORT 2
#define LED0_PIN 5

#define LED1_PORT 0
#define LED1_PIN 6

#define UART_RX_PORT 1
#define UART_RX_PIN 6
#define UART_RX_IOCON IOCON_PIO1_6

#define UART_TX_PORT 1
#define UART_TX_PIN 7
#define UART_TX_IOCON IOCON_PIO1_7

#define DMOC_COM_PIN 2,4

// -------------------------------------------------------------
// Computed Macros

#define LED0 LED0_PORT, LED0_PIN
#define LED1 LED1_PORT, LED1_PIN

#define UART_RX UART_RX_PORT, UART_RX_PIN
#define UART_TX UART_TX_PORT, UART_TX_PIN

#define Board_LED_On(led) {Chip_GPIO_SetPinState(LPC_GPIO, led, true);}
#define Board_LED_Off(led) {Chip_GPIO_SetPinState(LPC_GPIO, led, false);}
 
// -------------------------------------------------------------
// Board Level Function Prototypes

/**
 * Initialize the Core Systick Timer
 * 
 * @return true if error
 */
int8_t Board_SysTick_Init(void);

void Board_LEDs_Init(void);

void Board_UART_Init(uint32_t baudrate);

/**
 * Transmit the given string through the UART peripheral (blocking)
 * 
 * @param str pointer to string to transmit
 * @note	This function will send or place all bytes into the transmit
 *			FIFO. This function will block until the last bytes are in the FIFO.
 */
void Board_UART_Print(const char *str);

/**
 * Transmit a string through the UART peripheral and append a newline and a linefeed character (blocking)
 * 
 * @param str pointer to string to transmit
 * @note	This function will send or place all bytes into the transmit
 *			FIFO. This function will block until the last bytes are in the FIFO.
 */
void Board_UART_Println(const char *str);

/**
 * Transmit a string containing a number through the UART peripheral (blocking)
 * 
 * @param num number to print
 * @param base number base
 * @param crlf append carraige return and line feed
 */
void Board_UART_PrintNum(const int num, uint8_t base, bool crlf);

/**
 * Transmit a byte array through the UART peripheral (blocking)
 * 
 * @param	data		: Pointer to data to transmit
 * @param	num_bytes	: Number of bytes to transmit
 * @note	This function will send or place all bytes into the transmit
 *			FIFO. This function will block until the last bytes are in the FIFO.
 */
void Board_UART_SendBlocking(const void *data, uint8_t num_bytes);

/**
 * Read data through the UART peripheral (non-blocking)
 * 
 * @param	data		: Pointer to bytes array to fill
 * @param	num_bytes	: Size of the passed data array
 * @return	The actual number of bytes read
 * @note	This function reads data from the receive FIFO until either
 *			all the data has been read or the passed buffer is completely full.
 *			This function will not block. This function ignores errors.
 */
int8_t Board_UART_Read(void *data, uint8_t num_bytes);


void Board_CAN_Init(uint32_t baudrate, void (*rx_callback)(uint8_t), void (*tx_callback)(uint8_t), void (*error_callback)(uint32_t));

/**
 * Enable the RS-232 communcation with the motor controller
 *
 * @return	Nothing
 */
void Board_DMOC_Comm_Enable(void);

/**
 * Disable the RS-232 communication with the motor controller
 *
 * @return	Nothing
 */
void Board_DMOC_Comm_Disable(void);

/**
 * Initialize the RS-232 communication with the motor controller
 *
 * @return	Nothing
 * @note	Requires that GPIO has been initialized first
 */
void Board_DMOC_Comm_Init(void);


/** 
 * Initialize the DC/DC Converter control pin
 *
 * @return	Nothing
 * @note	Requires that the GPIO has been initialized first
 */
void Board_DCDC_Init(void);

/**
 * Enable the DC/DC Converter
 *
 * @return	Nothing
 */
void Board_DCDC_Enable(void);

/**
 * Disable the DC/DC Converter
 *
 * @return	Nothing
 */
void Board_DCDC_Disable(void);

#endif
