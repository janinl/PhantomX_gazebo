#include "mytypes.h"

#include "ax12Serial.hh"

#define DEFINE_HEX_GLOBALS
#define HEXMODE   // default to hex mode
#include "Hex_Cfg.h"

#undef SOUND_PIN

//#include <ax12.h>
#include "_Phoenix.h"

#include "Input_Controller_raspi.h"
#include "_Phoenix_Driver_AX12.h"
#include "_Phoenix_Code.h"



long unsigned int millis() {
  struct timeval tp;
  gettimeofday(&tp, NULL);
  long unsigned int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  return ms;
}


int main()
{
setup();
while (loop()) {}

ax12Finish();

return 0;
}

