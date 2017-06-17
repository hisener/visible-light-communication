/**
 * Visible Light Communication Library
 * BBM 434 - Project
 * Runs on TM4C123
 * Uses ADC0, PortE 3, SysTick.
 * Date: May 29, 2017
 * Authors: Reyyan Oflaz, Halil Ibrahim Sener
 */

#include <string.h>
#include "tm4c123gh6pm.h"
#include "vlc.h"

/* Function prototypes */
void Read_Data(unsigned long input);

void Take_Sample(unsigned long input);
void Bit_Sync(unsigned long input);
void Word_Sync(unsigned long input);
void Data(unsigned long input);

void Transmitter_Timer_Handler(void);
void Receiver_Timer_Handler(void);

void Manchester_Encode(unsigned char *message, unsigned char* output);
unsigned long ADC0_In(void);

void ADC0_Init(void);
void Output_Init(void);
void SysTick_Init(unsigned long delay);
void SysTick_Disable(void);

/* State struct that contains a callback function and next state index */
typedef struct State {
    void (*callback)(unsigned long); // callback function of the state
    unsigned long next; // next state
} State;

/* There are four states:
 * Take_Sample: It takes samples to decide the threshold value.
 * Bit_Sync: After deciding the threshold, it synchronizes bits. Further
 * information, please refer Manchester encode.
 * Word_Sync: After Bit_Sync, it synchronizes words as well. It waits STX.
 * Data: Put bits to the buffer after Word_Sync. After receiving ETX, return
 * Word_Sync state.
 */
State State_Array[4] = {
    {&Take_Sample, 1},
    {&Bit_Sync, 2},
    {&Word_Sync, 3},
    {&Data, 2}
};

void (*timer_handler)(void);
unsigned long current_state;

unsigned int count = 0;
double threshold;
unsigned int sample;

unsigned char output[288];
unsigned char buffer[BUFFER_SIZE];
unsigned char Index = 0;
unsigned char data_availabe = 0;

unsigned char bit_sync = 0;
unsigned long previous_input;
unsigned char previous_halfbit = 2;
unsigned char data = 0xFF;

/**
 * VLC transmitter initialization function.
 * @param frequency the frequency of the SysTick Timer
 * @param message message to be sent
 */
void VLC_Transmitter_Init(unsigned long frequency, unsigned char *message) {
    Manchester_Encode(message, output);
    timer_handler = Transmitter_Timer_Handler;
    Output_Init();
    SysTick_Init(frequency);
}

/**
 * VLC receiver initialization function.
 * @param frequency the frequency of the SysTick Timer
 */
void VLC_Receiver_Init(unsigned long frequency) {
    threshold = 0;
    current_state = 0; // Take Sample is the initial state
    sample = 3200000 / frequency; // 0.2 sec (assumes 16 MHz)
    timer_handler = Receiver_Timer_Handler;
    ADC0_Init();
    SysTick_Init(frequency);
}

/**
 * Returns data available bit.
 * @return data available bit
 */
unsigned char VLC_Data_Available(void) {
    return data_availabe;
}

/**
 * Returns the buffer
 * @return char pointer of the buffer
 */
char* VLC_Get_Data(void) {
    return (char*) buffer;
}

/**
 * Subroutine to handle SysTick Timer interrupts
 */
void SysTick_Handler(void) {
    timer_handler();
}

/**
 * Subroutine to handle transmitter SysTick Timer interrupts
 */
void Transmitter_Timer_Handler(void) {
    GPIO_PORTE_DATA_R = output[Index];
    Index = ++Index % 288;
}

/**
 * Subroutine to handle receiver SysTick Timer interrupts
 */
void Receiver_Timer_Handler(void) {
    unsigned long input = ADC0_In();

    // call current state's callback method
    State_Array[current_state].callback(input);
}

/**
 * Maps inputs to the actual bit value.
 * Uses Manchester decode
 * 01 => 1
 * 10 => 0
 * @param input analog input value of the photo diode.
 */
void Read_Data(unsigned long input) {
    unsigned char halfbit, bit;
    // map input to the halfbit
    halfbit = (input > threshold) ? 1 : 0;

    if (++count % 2 == 0) {
        // If previous halfbit and current halfbit are not equal, it is valid
        // (manchester code) and current halfbit that we are looking for.
        // Otherwise, something went wrong. So, we have to decide according to
        // analog inputs. We assumed that the input farther than the threshold
        // is more reliable.
        if (previous_halfbit != halfbit) {
            bit = halfbit;
        } else if(input > threshold && input > previous_input) {
            bit = halfbit;
        } else if (input < threshold && input < previous_input) {
            bit = halfbit;
        } else {
            bit = previous_halfbit;
        }

        data >>= 1;
        data |= bit << 7;
    }

    previous_input = input;
    previous_halfbit = halfbit;
}

/**
 * Take Sample state to determine the threshold value.
 * @param input analog input value of the photo diode.
 */
void Take_Sample(unsigned long input) {
    // calculate average
    threshold = (input - threshold) / ++count + threshold;

    // after taking enough sample change state
    if (count > sample - 1) {
        current_state = State_Array[current_state].next;
    }
}

/**
 * Bit Synchronization with waiting two identical signals
 * (low-low or high-high)
 * @param input analog input value of the photo diode.
 */
void Bit_Sync(unsigned long input) {
    unsigned char halfbit = (input > threshold) ? 1 : 0;
    
    if (halfbit == previous_halfbit) {
        bit_sync = 1;
    } else if (bit_sync) {
        current_state = State_Array[current_state].next;
    }
    previous_halfbit = halfbit;
}

/**
 * Read the bits and check if the data is STX. If it reads STX, it will change
 * current state to the next one.
 * @param input analog input value of the photo diode.
 * @see Read_Data(unsigned long)
 */
void Word_Sync(unsigned long input) {
    Read_Data(input);

    if (data == STX) {
        current_state = State_Array[current_state].next;
        count = 0;
        data_availabe = 0;
        Index = 0;
    }
}

/**
 * Reads data and if counter reach the byte, put it the buffer.
 * When it reads ETX, it makes data available and changes current state.
 * @param input analog input value of the photo diode.
 * @see Read_Data(unsigned long)
 */
void Data(unsigned long input) {
    Read_Data(input);
    
    // 16, because of 1 char is 8 bits and 16 halfbits.
    // if the count does not equal 16, nothing to do
    if (count != 16) {
        return;
    }

    // count equals 16
    // check if data is end of text
    if (data == ETX) {
        data_availabe = 1;
        current_state = State_Array[current_state].next;
        return;
    }

    // put it to the buffer and increment index
    buffer[Index] = data;
    Index = ++Index % BUFFER_SIZE;
    count = 0;
}

/**
 * Encodes a message using Manchester coding.
 * @param message message to be encoded
 * @param output output bit array
 */
void Manchester_Encode(unsigned char *message, unsigned char* output) {
    unsigned char data[18];
    int i, j, next;
    unsigned char bit;
    
    data[0] = 2;
    strcpy((char *) data + 1, (char *) message);
    data[17] = 3;

    next = 0;
    for (i = 0; i < 18; ++i) {
        for (j = 0; j < 8; ++j) {
            bit = (data[i] >> j) & 1;
            output[next++] = bit ^ 1;
            output[next++] = bit ^ 0;
        }
    }
}

/**
 * Busy-wait Analog to digital conversion
 * @return 12-bit result of ADC conversion
 */
unsigned long ADC0_In(void) {
    unsigned long result;
    ADC0_PSSI_R = 0x0008;               // 1) initiate SS3
    while((ADC0_RIS_R & 0x08) == 0);    // 2) wait for conversion done
    result = ADC0_SSFIFO3_R & 0xFFF;    // 3) read result
    ADC0_ISC_R = 0x0008;                // 4) acknowledge completion
    return result;
}

/**
 * Subroutine to initialize ADC
 * Max sample rate: <= 125,000 samples/second
 * SS3 triggering event: software trigger
 * SS3 0st sample source: channel 0
 * SS3 interrupts: enabled but not promoted to controller
 */
void ADC0_Init(void) {
    SYSCTL_RCGC2_R |= 0x00000010;   // 1) activate clock for Port E
    while((SYSCTL_RCGC2_R & 0x10) == 0);
    GPIO_PORTE_DIR_R &= ~0x08;      // 2) make PE3 input
    GPIO_PORTE_AFSEL_R |= 0x08;     // 3) enable alternate function on PE3
    GPIO_PORTE_DEN_R &= ~0x08;      // 4) disable digital I/O on PE3
    GPIO_PORTE_AMSEL_R |= 0x08;     // 5) enable analog function on PE3

    SYSCTL_RCGC0_R |= 0x00010000;   // 6) activate ADC0
    while((SYSCTL_RCGC0_R & 0x10000) == 0);
    SYSCTL_RCGC0_R &= ~0x00000300;  // 7) configure for 125K
    ADC0_SSPRI_R = 0x0123;          // 8) Sequencer 3 is highest priority
    ADC0_ACTSS_R &= ~0x0008;        // 9) disable sample sequencer 3
    ADC0_EMUX_R &= ~0xF000;         // 10) Seq3 is software trigger
    ADC0_SSMUX3_R &= ~0x000F;       // 11) clear SS3 field (Ain0)
    ADC0_SSCTL3_R = 0x0006;         // 12) no TS0 D0, yes IE0 END0
    ADC0_ACTSS_R |= 0x0008;         // 13) enable sample sequencer 3
}

/**
 * Subroutine to initialize output pin
 */
void Output_Init(void) {
    SYSCTL_RCGC2_R |= 0x00000010;           // 1) activate clock for Port E
    while((SYSCTL_PRGPIO_R & 0x10) == 0);   // allow time for clock to start
    GPIO_PORTE_CR_R |= 0x01;                // allow changes to PE0
    GPIO_PORTE_AMSEL_R &= ~0x01;            // 3) disable analog on PE0
    GPIO_PORTE_PCTL_R &= ~0x0F;             // 4) PCTL GPIO on PE0
    GPIO_PORTE_DIR_R |= 0x01;               // 5) PE0 output
    GPIO_PORTE_AFSEL_R &= ~0x01;            // 6) disable alt funct on PE0
    GPIO_PORTE_DEN_R |= 0x01;               // 7) enable digital I/O on PE0
}

/**
 * Subroutine to initialize SysTick Timer
 * @param delay clock cycle to wait
 */
void SysTick_Init(unsigned long delay) {
    NVIC_ST_CTRL_R = 0;             // disable SysTick during setup
    NVIC_ST_RELOAD_R = delay - 1;   // reload value
    NVIC_ST_CURRENT_R = 0;          // any write to current clears it
    NVIC_SYS_PRI3_R &= 0x00FFFFFF;  // Clear priority bits
    NVIC_SYS_PRI3_R |= 0x20000000;  // priority 1
    NVIC_ST_CTRL_R = 0x07;          // enable SysTick with processor clock
}

/**
 * Subroutine to disable SysTick Timer
 */
void SysTick_Disable(void) {
    NVIC_ST_CTRL_R = 0; // disable SysTick
}
