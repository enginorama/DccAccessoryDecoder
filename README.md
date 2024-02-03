# DCC Accessory Decoder

ESP32 library for a DIY DCC Accessory Decoder

This project is based on [DCCInspector-EX](https://github.com/DCC-EX/DCCInspector-EX). Checkout the repository for more information.

## Usage with platform io

You can add this library by adding it to you platformio.ini

```ini
lib_deps =
  https://github.com/enginorama/DccAccessoryDecoder
```

and follow the [example](./examples/).

```cpp
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
```
