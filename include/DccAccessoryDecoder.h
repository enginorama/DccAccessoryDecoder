/* Copyright (c) 2024 Christian Hartinger
 * Based on DCCInspector-EX, Neil McKechnie, 2021
 * Based on DCC_Sniffer, Ruud Boer, October 2015
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#ifndef DCC_ACCESSORY_DECODER
#define DCC_ACCESSORY_DECODER

#include <Arduino.h>
#include "EventTimer.h"

typedef void AccessoryPacketHandler(unsigned int decoderAddress, bool thrown);

bool IRAM_ATTR DccAccessoryDecoderCaptureWrapper(unsigned long halfBitLengthTicks);

class DccAccessoryDecoderClass {
 public:
  bool begin(uint8_t pin, AccessoryPacketHandler* accessoryPacketHandler) {
    this->pin = pin;
    this->accessoryPacketHandler = accessoryPacketHandler;
    pinMode(pin, INPUT_PULLUP);
    pinMode(BUILTIN_LED, OUTPUT);
    if (!EventTimer.inputCaptureMode()) {
      // Output health warning...
      ESP_LOGW("DccAccessoryDecoder", F("\r\n** WARNING Measurements will occasionally be out up to ~10us "
                                        "either way **"));
      ESP_LOGW("DccAccessoryDecoder", F("**         because of inaccuracies in the micros() function.        "
                                        "    **"));
    }
    return EventTimer.begin(pin, DccAccessoryDecoderCaptureWrapper);
  }
  void loop() { processDCC(); }

  bool IRAM_ATTR capture(unsigned long halfBitLengthTicks) {
    byte bitValue;

    // The most critical parts are done first - read state of digital input.
    byte diginState = digitalRead(pin);

    // Set a high bound on the half bit length
    if (halfBitLengthTicks > 1200 * TICKSPERMICROSEC)
      halfBitLengthTicks = 1200 * TICKSPERMICROSEC;  // microseconds.

    // Calculate time between interrupts in microseconds.
    unsigned int interruptInterval = halfBitLengthTicks / TICKSPERMICROSEC;

    // Precondition input?
    if (filterInput) {
      // Check that the digital input has actually changed since last interrupt,
      // and that the gap between interrupts is realistic.
      if (interruptCount > 0 && (diginState == previousDiginState || interruptInterval <= 3)) {
        // No change in digital, or it was fleeting.  Ignore.
        return false;  // reject interrupt
      }
    }

    // If we get here, the interrupt looks valid, i.e. the digital input really
    // did change its state more than 3us after its last change. Calculate
    // difference between current bit half and preceding one, rounding up to next
    // microsecond. This will only be recorded on alternate half-bits, i.e. where
    // the previous and current half-bit make a complete bit.
    long deltaTicks = halfBitLengthTicks - previousHalfBitLengthTicks;
    if (deltaTicks < 0)
      deltaTicks = -deltaTicks;
    unsigned int delta = (deltaTicks + TICKSPERMICROSEC - 1) / TICKSPERMICROSEC;

    // Check length of half-bit
    if (interruptInterval < 80)
      bitValue = 1;
    else
      bitValue = 0;

    // Record input state and timer values ready for next interrupt
    previousDiginState = diginState;
    previousHalfBitLengthTicks = halfBitLengthTicks;

    // If first or second interrupt, then exit as the previous state is
    // incomplete.
    if (interruptCount < 2) {
      interruptCount++;
      previousBitValue = bitValue;
      return true;
    }

    // Check if we're on the first or second half of the bit.
    if (bitValue != previousBitValue) {
      // First half of new bit received
      altbit = false;
    } else {
      // Toggle for alternate half-bits
      altbit = !altbit;
    }
    previousBitValue = bitValue;

    // Store interrupt interval for use on next interrupt.
    previousHalfBitLengthTicks = halfBitLengthTicks;

    // If this is the second half-bit then we've got a whole bit!!
    if (altbit) {
      bool rejectBit = false;
      if (strictMode == 2) {
        // Validate bit lengths against NMRA spec for controllers
        if (bitValue == 0) {
          if (interruptInterval < 95 || interruptInterval > 9900) {
            rejectBit = true;
          }
        } else {
          if (interruptInterval < 55 || interruptInterval > 61 || delta > 3) {
            rejectBit = true;
          }
        }
      } else if (strictMode == 1) {
        // Validate bit lengths against NMRA spec for decoders.
        if (bitValue == 0) {
          if (interruptInterval < 90 || interruptInterval > 10000) {
            rejectBit = true;
          }
        } else {
          if (interruptInterval < 52 || interruptInterval > 64 || delta > 6) {
            rejectBit = true;
          }
        }
      }
      // Record error only if we're in a packet (preamble has been read).
      if (rejectBit && preambleFound) {
        // Search for next packet
        preambleFound = 0;
        preambleOneCount = 0;
      }

      // Now we've got a bit, process it.  The message comprises the following:
      //   Preamble: 10 or more '1' bits followed by a '0' start bit.
      //   Groups of 9 bits each containing data byte of 8 bits, followed by a
      //   '0' bit (if message not yet finished), or a '1' bit (if the byte is
      //   the last byte of the message, i.e. the checksum).
      //
      if (!preambleFound) {
        if (bitValue == 1) {
          // Reading preamble perhaps...
          preambleOneCount++;
        } else if (preambleOneCount < 10) {  // and bitValue==0)
          // Preamble not yet found, but zero bit encountered.  Restart preable
          // count.
          preambleOneCount = 0;
        } else {  // preambleOneCount >= 10 and bitValue==0
          // Start bit found at end of preamble, so prepare to process data.
          preambleFound = true;
          newByte = 0;
          inputBitCount = 0;
          inputByteNumber = 0;
        }
      } else {  // Preamble previously found, so this is a message bit
        if (packetsPending == nPackets) {
          // Previous DCC packets haven't been processed by the main loop,
          // so there is no buffer for the incoming message.
          // Discard incoming message and scan for another preamble.
          preambleFound = false;
          preambleOneCount = 0;

        } else {
          // Preamble read, packet buffer available, so message bit can be stored!
          if (inputBitCount == 8) {  // Byte previously completed, so this bit is
                                     // the interbyte marker
            if (bitValue == 0) {     // Interbyte marker is zero, so prepare for next
                                     // byte of data
              inputBitCount = 0;
            } else {  // one-bit found, marks end of packet
              // End of packet found
              dccPacket[activePacket][0] = inputByteNumber;  // save number of bytes
              packetsPending++;                              // flag that packet is ready for processing
              if (++activePacket >= nPackets)
                activePacket = 0;     // move to next packet buffer
              preambleFound = false;  // scan for another preamble
              preambleOneCount = 1;   // allow the current bit to be counted in the preamble.
            }
          } else {  // Reading packet data at this point.
            // Append received bit to the current new byte.
            newByte = (newByte << 1) | bitValue;
            if (++inputBitCount == 8) {  // Completed byte, save byte (if room)
              if (inputByteNumber < pktLength - 1)
                dccPacket[activePacket][++inputByteNumber] = newByte;
              else {               // packet has filled buffer so no more bits can be stored!
                packetsPending++;  // flag that packet is ready for processing
                if (++activePacket >= nPackets)
                  activePacket = 0;     // move to next packet buffer
                preambleFound = false;  // scan for another preamble
                preambleOneCount = 0;
              }
              newByte = 0;
            }
          }
        }
      }
    }
    return true;
  }

 private:
  static const int nPackets = 16;  // Number of packet buffers
  static const int pktLength = 8;  // Max length+1 in bytes of DCC packet

  byte inputPacket = 0;  // Index of next packet to be analysed in dccPacket array
  byte pktByteCount = 0;

  uint8_t pin = 0;

  // Variables shared by interrupt routine and main loop
  volatile byte dccPacket[nPackets][pktLength];  // buffer to hold packets
  volatile byte packetsPending = 0;              // Count of unprocessed packets
  volatile byte activePacket = 0;                // indicate which buffer is currently being filled
  volatile bool filterInput = true;              // conditions input to remove transient changes
  volatile byte strictMode = 1;                  // rejects frames containing out-of-spec bit lengths

  // Variables needed by the ISR
  volatile byte preambleOneCount = 0;
  volatile boolean preambleFound = false;
  volatile int newByte = 0;          // Accumulator for input bits until complete byte found.
  volatile int inputBitCount = 0;    // Number of bits read in current newByte.
  volatile int inputByteNumber = 0;  // Number of bytes read into active dccPacket buffer so far
  volatile byte interruptCount = 0;
  volatile byte previousBitValue = 0, previousDiginState = 0;
  volatile unsigned int previousHalfBitLengthTicks = 0;
  volatile byte altbit = 0;  // 0 for first half-bit and 1 for second.

  AccessoryPacketHandler* accessoryPacketHandler = nullptr;

  bool processDCC() {
    if (!packetsPending) {
      return false;
    }

    pktByteCount = dccPacket[inputPacket][0];
    // Check packet isn't empty
    if (pktByteCount > 0) {
      // Calculate and verify checksum
      byte checksum = 0;
      for (byte n = 1; n <= pktByteCount; n++)
        checksum ^= dccPacket[inputPacket][n];
      if (checksum == 0) {  // Result should be zero, if not it's an error!
        DecodePacket(inputPacket);
      }
    }
    packetsPending--;  // Free packet buffer.
    if (++inputPacket >= nPackets) {
      inputPacket = 0;
    }

    return true;
  }

  void DecodePacket(int inputPacket) {
    byte instrByte1;
    byte decoderType;  // 0=Loc, 1=Acc
    unsigned int decoderAddress;

    // First determine the decoder type and address.
    if (dccPacket[inputPacket][1] == 0B11111111) {  // Idle packet
      decoderType = 255;
    } else if (!bitRead(dccPacket[inputPacket][1],
                        7)) {  // bit7=0 -> Loc Decoder Short Address
      decoderAddress = dccPacket[inputPacket][1];
      instrByte1 = dccPacket[inputPacket][2];
      decoderType = 0;
    } else {
      if (bitRead(dccPacket[inputPacket][1],
                  6)) {  // bit7=1 AND bit6=1 -> Loc Decoder Long Address
        decoderAddress = 256 * (dccPacket[inputPacket][1] & 0B00111111) + dccPacket[inputPacket][2];
        instrByte1 = dccPacket[inputPacket][3];
        decoderType = 0;
      } else {  // bit7=1 AND bit6=0 -> Accessory Decoder
        decoderAddress = dccPacket[inputPacket][1] & 0B00111111;
        instrByte1 = dccPacket[inputPacket][2];
        decoderType = 1;
      }
    }

    // Handle decoder type 0 and 1 separately.
    if (decoderType == 1) {           // Accessory Basic
      if (instrByte1 & 0B10000000) {  // Basic Accessory
        decoderAddress = (((~instrByte1) & 0B01110000) << 2) + decoderAddress;
        byte port = (instrByte1 & 0B00000110) >> 1;
        Serial.print(F("Acc "));
        Serial.print((decoderAddress - 1) * 4 + port + 1);
        Serial.print(' ');
        if (bitRead(instrByte1, 0)) {
          if (accessoryPacketHandler != nullptr) {
            accessoryPacketHandler(decoderAddress, true);
          }
        } else {
          if (accessoryPacketHandler != nullptr) {
            accessoryPacketHandler(decoderAddress, false);
          }
        }
      } else {  // Accessory Extended NMRA spec is not clear about address and
                // instruction format !!!
        // sbTemp.print(F("Acc Ext "));
        decoderAddress = (decoderAddress << 5) + ((instrByte1 & 0B01110000) >> 2) + ((instrByte1 & 0B00000110) >> 1);
        // sbTemp.print(decoderAddress);
        // sbTemp.print(F(" Asp "));
        // sbTemp.print(dccPacket[inputPacket][3], BIN);
      }
      // outputDecodedData = true;
    }
  }
};

DccAccessoryDecoderClass DccAccessoryDecoder;

bool IRAM_ATTR DccAccessoryDecoderCaptureWrapper(unsigned long halfBitLengthTicks) {
  return DccAccessoryDecoder.capture(halfBitLengthTicks);
}

#endif
