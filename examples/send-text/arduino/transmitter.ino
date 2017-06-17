#include "TimerOne.h"

#define LED 2
#define FREQUENCY 500

unsigned char message[16] = "Yet another msg";
unsigned char output[288]; // (message length + 2) * 8 bit * 2 clock
unsigned int index = 0;

void setup() {
  pinMode(LED, OUTPUT);

  manchester_encode(message, output);

  Timer1.initialize(FREQUENCY);
  Timer1.attachInterrupt(callback);
}

void loop() {
}

void callback() {
  digitalWrite(LED, output[index]);
  index = ++index % 288;
}

void manchester_encode(unsigned char *message, unsigned char* output) {
  unsigned char data[18];
  data[0] = 2;
  strcpy(data + 1, message);
  data[17] = 3;
  
  int next = 0;
  for (int i = 0; i < 18; ++i) {
    for (int j = 0; j < 8; ++j) {
      unsigned char bit = (data[i] >> j) & 1;
      output[next++] = bit ^ 1;
      output[next++] = bit ^ 0;
    }
  }
}
