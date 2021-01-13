#include "Thermal.h"

#include "commands.h"
#include "controller.h"


struct Extruder {
 public:
  Controller syringe;
  Controller needle;
};


/* Global state */
const static int NUM_EXTRUDERS = 1;
const static int NUM_CONTROLLERS = NUM_EXTRUDERS * 2 + 2;
static Extruder extruders[1];
static Controller bed;
static Controller ambient;
static Controller *controllers[NUM_CONTROLLERS] = {
  &ambient,
  &bed,
  &extruders[0].syringe,
  &extruders[0].needle
};


/* Perform a single code from the queue */
static CommandQueue cq;
static void doNextCode() {
  // Must find an M code to continue
  cq.loadNextLine();
  int32_t mCode;
  if (!cq.getInt('M', &mCode)) return;
  
  switch (mCode) {
    /* Set extruder temperature
     *   E: extruder number
     *   T: target temperature
     */
    case 104:
      {
        int32_t extruderNumber;
        float temperature;
        if (!cq.getInt('E', &extruderNumber) ||
            !cq.getFloat('T', &temperature) ||
            extruderNumber >= NUM_EXTRUDERS) {
          break;
        }

        extruders[extruderNumber].syringe.setTargetTemp(temperature);
        extruders[extruderNumber].needle.setTargetTemp(temperature);
      } break;
  }
}


void setup() {
  Serial.begin(115200);
  
  ambient = Controller("a0a", A0);
  bed = Controller("b0a", A1);
  extruders[0].syringe = Controller("s0a", A2, 2, 4.0f, 0.10f, 10.0f);
  extruders[0].needle = Controller("n0a", A3);

  // Change pulse-width modulation settings
  //TCCR3B &= 0xf8;
  //TCCR3B |= 0x02;
}


void loop() {
  doNextCode();
  for (int i = 0; i < NUM_CONTROLLERS; ++i) {
    controllers[i]->adjustPower();
    controllers[i]->printTemp();
  }
  delay(100);
}
