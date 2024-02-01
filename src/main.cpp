/* Copyright (c) 2024 Christian Hartinger
 * Parts based on DCCInspector-EX, Neil McKechnie, 2021
 * Parts based on DCC_Sniffer, Ruud Boer, October 2015
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

#include <Arduino.h>
#include <EventTimer.h>

#define INPUTPIN 19

#if defined(ESP32) || defined(ESP8266) || defined(ESP_PLATFORM)
#define INTERRUPT_SAFE IRAM_ATTR
#else
#define INTERRUPT_SAFE
#endif

const int nPackets = 16;  // Number of packet buffers
const int pktLength = 8;  // Max length+1 in bytes of DCC packet

// Variables shared by interrupt routine and main loop
volatile byte dccPacket[nPackets][pktLength];  // buffer to hold packets
volatile byte packetsPending = 0;              // Count of unprocessed packets
volatile byte activePacket = 0;                // indicate which buffer is currently being filled
volatile bool filterInput = true;              // conditions input to remove transient changes
volatile byte strictMode = 1;                  // rejects frames containing out-of-spec bit lengths

byte packetHashListSize = 32;  // DCC packets checksum buffer size

byte inputPacket = 0;  // Index of next packet to be analysed in dccPacket array
byte pktByteCount = 0;

int packetHashListCounter = 0;
unsigned int packetHashList[64];
// bool calibrated = false;
// unsigned long lastRefresh = 0;
// unsigned int inactivityCount = 0;

// bool showLoc = true;
bool showAcc = true;

//=======================================================================
// Read data from the dccPacket structure and decode into
// textual representation.  Send results out over the USB serial
// connection.

void DecodePacket(int inputPacket, bool isDifferentPacket) {
  Serial.println("Decoding packet!");
  byte instrByte1;
  byte decoderType;  // 0=Loc, 1=Acc
  unsigned int decoderAddress;
  // bool outputDecodedData = false;

  // char tempBuffer[100];
  // StringBuilder sbTemp(tempBuffer, sizeof(tempBuffer));

  // First determine the decoder type and address.
  if (dccPacket[inputPacket][1] == 0B11111111) {  // Idle packet
    if (isDifferentPacket) {
      // sbTemp.print(F("Idle "));
      // outputDecodedData = true;
    }
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
  if (decoderType == 1) {  // Accessory Basic
    if (showAcc) {
      if (instrByte1 & 0B10000000) {  // Basic Accessory
        decoderAddress = (((~instrByte1) & 0B01110000) << 2) + decoderAddress;
        byte port = (instrByte1 & 0B00000110) >> 1;
        Serial.print(F("Acc "));
        Serial.print((decoderAddress - 1) * 4 + port + 1);
        Serial.print(' ');
        // sbTemp.print(decoderAddress);
        // sbTemp.print(F(":"));
        // sbTemp.print(port);
        // sbTemp.print(' ');
        // sbTemp.print(bitRead(instrByte1, 3));
        if (bitRead(instrByte1, 0)) {
          digitalWrite(BUILTIN_LED, 1);
          Serial.println(F(" On"));
        }
        // sbTemp.print(F(" On"));
        else {
          digitalWrite(BUILTIN_LED, 0);
          Serial.println(F(" Off"));
        }
        // sbTemp.print(F(" Off"));
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
}

//=======================================================================
// Validate received packet and pass to decoder.
// Return false if nothing done.

bool processDCC() {
  byte isDifferentPacket = 0;

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
    if (checksum) {  // Result should be zero, if not it's an error!
      // DCCStatistics.recordChecksumError();
    } else {
// There is a new packet with a correct checksum
#ifdef LEDPIN_DECODING
      digitalWrite(LEDPIN_DECODING, 1);
#endif

      // Hooray - we've got a packet to decode, with no errors!
      // DCCStatistics.recordPacket();

      // Generate a cyclic hash based on the packet contents for checking if
      // we've seen a similar packet before.
      isDifferentPacket = true;
      unsigned int hash = dccPacket[inputPacket][pktByteCount];  // calculate checksum
      for (byte n = 1; n < pktByteCount; n++)
        hash = ((hash << 5) | (hash >> 11)) ^ dccPacket[inputPacket][n];

      // Check if packet's checksum is already in the list.
      for (byte n = 0; n < packetHashListSize; n++) {
        if (hash == packetHashList[n])
          isDifferentPacket = false;
      }

      if (isDifferentPacket) {
        packetHashList[packetHashListCounter++] = hash;  // add new packet's hash to the list
        if (packetHashListCounter >= packetHashListSize)
          packetHashListCounter = 0;

        DecodePacket(inputPacket, isDifferentPacket);
      }
    }
  }
  packetsPending--;  // Free packet buffer.
  if (++inputPacket >= nPackets) {
    inputPacket = 0;
  }

  return true;
}

//=======================================================================
// Function invoked (from interrupt handler) on change of state of INPUTPIN.
//  It measures the time between successive changes (half-cycle of DCC
//  signal).  Depending on the value, it decodes 0 or a 1 for alternate
//  half-cycles.  A 0 half-bit is nominally 100us per half-cycle (NMRA says
//  90-10000us) and a 1 half-bit is nominally 58us (52-64us).  We treat a
//  half-bit duration < 80us as a '1' half-bit, and a duration >= 80us as a '0'
//  half-bit. Prologue and framing bits are detected and stripped, and data
//  bytes are then stored in the packet queue for processing by the main loop.
//
bool INTERRUPT_SAFE capture(unsigned long halfBitLengthTicks) {
  static byte preambleOneCount = 0;
  static boolean preambleFound = false;
  static int newByte = 0;          // Accumulator for input bits until complete byte found.
  static int inputBitCount = 0;    // Number of bits read in current newByte.
  static int inputByteNumber = 0;  // Number of bytes read into active dccPacket buffer so far
  static byte interruptCount = 0;
  static byte previousBitValue = 0, previousDiginState = 0;
  static unsigned int previousHalfBitLengthTicks = 0;
  static byte altbit = 0;  // 0 for first half-bit and 1 for second.
  byte bitValue;

  // The most critical parts are done first - read state of digital input.
  byte diginState = digitalRead(INPUTPIN);

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
      // DCCStatistics.recordGlitch();
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

#ifdef LEDPIN_ACTIVE
  digitalWrite(LEDPIN_ACTIVE, 1);
#endif

  // Check if we're on the first or second half of the bit.
  if (bitValue != previousBitValue) {
    // First half of new bit received
    altbit = false;
  } else {
    // Toggle for alternate half-bits
    altbit = !altbit;
  }
  previousBitValue = bitValue;

  // Update statistics
  // DCCStatistics.recordHalfBit(altbit, bitValue, interruptInterval, delta);

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
      // DCCStatistics.recordOutOfSpecRejection();
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

        // Record this event in a counter.
        // DCCStatistics.recordLostPacket();

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
              // Record this event in a counter.
              // DCCStatistics.recordLongPacket();
            }
            newByte = 0;
          }
        }
      }
    }
  }

#ifdef LEDPIN_ACTIVE
  // Turn out ACTIVE LED.
  digitalWrite(LEDPIN_ACTIVE, 0);
#endif

  // Calculate time taken in interrupt code between the measured time of event
  // to POINTB.

  // unsigned int interruptDuration = EventTimer.elapsedTicksSinceLastEvent() / TICKSPERMICROSEC;  // POINTB

  // Assume that there are about 25 cycles of instructions in this function that
  // are not measured, and that the prologue in dispatching the function (saving
  // registers etc) is around 51 cycles and the epilogue (restoring registers
  // etc) is around 35 cycles.  This adds a further (51+25+35)/16MHz=6.9us to
  // the calculation. See
  // https://billgrundmann.wordpress.com/2009/03/02/the-overhead-of-arduino-interrupts/.
  // However, if the Input Capture mode is used, then this will be much smaller.
  // So ignore it.
  // interruptDuration += 7;

  // Record result
  // DCCStatistics.recordInterruptHandlerTime(interruptDuration);

  return true;  // Accept interrupt.
}

void setup() {
  Serial.begin(115200);
  pinMode(INPUTPIN, INPUT_PULLUP);
  pinMode(BUILTIN_LED, OUTPUT);
  if (!EventTimer.inputCaptureMode()) {
    // Output health warning...
    Serial.println(
        F("\r\n** WARNING Measurements will occasionally be out up to ~10us "
          "either way **"));
    Serial.println(
        F("**         because of inaccuracies in the micros() function.        "
          "    **"));
  }
  if (!EventTimer.begin(INPUTPIN, capture)) {
    Serial.println(F("Unable to start EventTimer, check configured pin"));
    while (1)
      ;
  }  // put your setup code here, to run once:
  Serial.println("SETUP done!");
  digitalWrite(BUILTIN_LED, 1);
}

void loop() {
  if (processDCC()) {
  }
}
