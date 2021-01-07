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
static Extruder extruders[1];


/* Perform a single code from the queue */
static CommandQueue cq;
static void doNextCode() {
  // Must find an M code to continue
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
        Serial.println(String("# Set extruder temperature: ") + String(temperature));
      } break;
  }
}


void setup() {
  extruders[0].syringe = Controller(A0, 2, 1.0f, 1.0f, 1.0f);
  extruders[0].needle = Controller(A1, 11, 1.0f, 1.0f, 1.0f);

  Serial.begin(115200);
}


void loop() {
  doNextCode();
  extruders[0].syringe.adjustPower();
  delay(100);
}
