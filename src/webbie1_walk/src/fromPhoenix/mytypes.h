#include "ros_cfg.h"

#include <cstdlib>
#include <cstdint>
//#include <chrono>
#include <unistd.h>
#include <sys/time.h>

typedef uint8_t byte;
typedef uint16_t word;
typedef bool boolean;

// this defines pgm_read_word etc
#define __SAM3X8E__
#define PROGMEM

/*
void delay(int x) {
  usleep(x*1000);
}

void delayMicroseconds(int x) {
  usleep(x);
}
*/
#define delayMicroseconds usleep
#define delay(x) usleep(x*1000)

long unsigned int millis();

//#define abs(x) ((x>=0)?(x):(-x))
//#define min(x,y) ((x)<(y)?(x):(y))

