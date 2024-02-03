/*
 * Example: Builtin Led Accessory
 *
 * Switches the builtin LED state to the last recieved accessory value and outputs the details to Serial.
 * Listens to all addresses
 */

#include <Arduino.h>
#include <DccAccessoryDecoder.h>

void onAccessoryPacket(unsigned int linearDecoderAddress, bool enabled) {
  digitalWrite(BUILTIN_LED, enabled ? 1 : 0);
  Serial.print("Change in Accessory ");
  Serial.print(linearDecoderAddress);
  Serial.print(" -> ");
  Serial.println(enabled);
}

void setup() {
  Serial.begin(115200);
  DccAccessoryDecoder.begin(19, onAccessoryPacket);
  Serial.println("Start sending turnout commands.");
}

void loop() {
  DccAccessoryDecoder.loop();
}
