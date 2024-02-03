#include <Arduino.h>
#include <DccAccessoryDecoder.h>

void onAccessoryPacket(unsigned int decoderAddress, bool thrown) {
  digitalWrite(BUILTIN_LED, thrown ? 1 : 0);
  Serial.print("Change in Accessory ");
  Serial.print(decoderAddress);
  Serial.print(" -> ");
  Serial.println(thrown);
}

void setup() {
  Serial.begin(115200);
  DccAccessoryDecoder.begin(19, onAccessoryPacket);
  Serial.println("Start sending turnout commands.");
}

void loop() {
  DccAccessoryDecoder.loop();
}
