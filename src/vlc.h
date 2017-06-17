/**
 * Visible Light Communication Library
 * BBM 434 - Project
 * Runs on TM4C123
 * Date: May 29, 2017
 * Authors: Reyyan Oflaz, Halil Ibrahim Sener
 */

#ifndef VISIBLE_LIGHT_COMMUNICATION_H
#define VISIBLE_LIGHT_COMMUNICATION_H

#define BUFFER_SIZE 16
#define STX         0x02    // start of text
#define ETX         0x03    // end of text

/**
 * VLC transmitter initialization function.
 * @param frequency the frequency of the SysTick Timer
 * @param message message to be sent
 */
void VLC_Transmitter_Init(unsigned long frequency, unsigned char *message);

/**
 * VLC receiver initialization function.
 * @param frequency the frequency of the SysTick Timer
 */
void VLC_Receiver_Init(unsigned long frequency);

/**
 * Returns data available bit.
 * @return data available bit
 */
unsigned char VLC_Data_Available(void);

/**
 * Returns the buffer
 * @return char pointer of the buffer
 */
char* VLC_Get_Data(void);

#endif
