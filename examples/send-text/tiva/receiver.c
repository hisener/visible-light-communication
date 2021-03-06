// BBM 434 - Project
// Visible Light Communication Example Application
// Runs on TM4C123
// Date: May 29, 2017
// Authors: Reyyan Oflaz, Halil Ibrahim Sener

#include "tm4c123gh6pm.h"
#include "Nokia5110.h"
#include "vlc.h"

#define FREQUENCY   8000   // assumes 16 MHz

// Function Prototypes
void EnableInterrupts(void);    // Enable interrupts
void DisableInterrupts(void);   // Disable interrupts
void WaitForInterrupt(void);    // Wait for interrupt

int main(void) {
    VLC_Receiver_Init(FREQUENCY);
    
    Nokia5110_Init();
    Nokia5110_Clear();

    // wait until data available
    while(!VLC_Data_Available());
    Nokia5110_OutString(VLC_Get_Data());

    while(1) {
        WaitForInterrupt();
    }
}
