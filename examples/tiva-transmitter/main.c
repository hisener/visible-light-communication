// BBM 434 - Project
// Visible Light Communication Example Application
// Runs on TM4C123
// Date: May 29, 2017
// Authors: Reyyan Oflaz, Halil Ibrahim Sener

#include "tm4c123gh6pm.h"
#include "Nokia5110.h"
#include "vlc.h"

#define FREQUENCY   1600000   // assumes 16 MHz => 1ms

// Function Prototypes
void EnableInterrupts(void);    // Enable interrupts
void DisableInterrupts(void);   // Disable interrupts
void WaitForInterrupt(void);    // Wait for interrupt

int main(void) {
    unsigned char message[16] = "Yet another msg";
    VLC_Transmitter_Init(FREQUENCY, message);
    
    while(1) {
        WaitForInterrupt();
    }
}
