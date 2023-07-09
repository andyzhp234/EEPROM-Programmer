// EEPROM Programmer following Ben Eater's hardware schematic with the ability to unlock Software Data Protection (SDP)

#define SHIFT_DATA 2
#define SHIFT_CLK 3
#define SHIFT_LATCH 4
#define WRITE_EN 13

// common andode 7-segment display in hex decimal
// byte data[] = { 0x01, 0x4f, 0x12, 0x06, 0x4c, 0x24, 0x20, 0x0f, 0x00, 0x04, 0x08, 0x60, 0x31, 0x42, 0x30, 0x38 };

// common cathode 7-segment display in hex decimal
// byte data[] = { 0x7e, 0x30, 0x6d, 0x79, 0x33, 0x5b, 0x5f, 0x70, 0x7f, 0x7b, 0x77, 0x1f, 0x4e, 0x3d, 0x4f, 0x47 };

void enableWrite() {
  digitalWrite(WRITE_EN, LOW);
}
void disableWrite() {
  digitalWrite(WRITE_EN, HIGH);
}

void memoryValueTesting() {
  // byte digits[] = { 0x7e, 0x30, 0x6d, 0x79, 0x33, 0x5b, 0x5f, 0x70, 0x7f, 0x7b };
  // -029 = 11100010
  // 100 11100010 (9) = 80 4E2
  // 101 11100010 (2) = 180 5E2
  // 110 11100010 (0) = 280
  // -----------------------

  // byte ones = readEEPROM(0x04e2);
  // byte tens = readEEPROM(0x05e2);
  // byte hundreds = readEEPROM(0x06e2);
  // byte thousands = readEEPROM(0x07e2);
  // char buf[80];
  // sprintf(buf, "%02x %02x %02x %02x", thousands, hundreds, tens, ones);
  // Serial.println(buf);
}

void writeDecimalDisplayValues() {
  byte digits[] = { 0x7e, 0x30, 0x6d, 0x79, 0x33, 0x5b, 0x5f, 0x70, 0x7f, 0x7b };

  for (int i = 0; i <= 20; i++) {
    Serial.println("Programming ones place");
    for (int value = 0; value <= 255; value += 1) {
      writeEEPROM(value, digits[value % 10]);
    }

    delay(i);
  }

  // -------------------------------------------------------------------------------

  // Serial.println("Programming ones place");
  // for (int value = 0; value <= 255; value += 1) {
  //   writeEEPROM(value, digits[value % 10]);
  // }

  // Serial.println("Programming tens place");
  // for (int value = 0; value <= 255; value += 1) {
  //   writeEEPROM(value + 256, digits[(value / 10) % 10]);
  // }

  // Serial.println("Programming hundreds place");
  // for (int value = 0; value <= 255; value += 1) {
  //   writeEEPROM(value + 512, digits[(value / 100) % 10]);
  // }

  // Serial.println("Programming sign");
  // for (int value = 0; value <= 255; value += 1) {
  //   writeEEPROM(value + 768, 0);
  // }

  // ----------------------------------------------------------------

  // Serial.println("Programming ones place (twos complement)");
  // for (int value = -128; value <= 127; value += 1) {
  //   writeEEPROM((byte)value + 1024, digits[abs(value) % 10]);
  // }

  // Serial.println("Programming tens place (twos complement)");
  // for (int value = -128; value <= 127; value += 1) {
  //   writeEEPROM((byte)value + 1280, digits[abs(value / 10) % 10]);
  // }

  // Serial.println("Programming hundreds place (twos complement)");
  // for (int value = -128; value <= 127; value += 1) {
  //   writeEEPROM((byte)value + 1536, digits[abs(value / 100) % 10]);
  // }

  // Serial.println("Programming sign (twos complement)");
  // for (int value = -128; value <= 127; value += 1) {
  //   if (value < 0) {
  //     writeEEPROM((byte)value + 1792, 0x01);
  //   } else {
  //     writeEEPROM((byte)value + 1792, 0);
  //   }
  // }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(SHIFT_DATA, OUTPUT);
  pinMode(SHIFT_CLK, OUTPUT);
  pinMode(SHIFT_LATCH, OUTPUT);
  disableWrite();
  pinMode(WRITE_EN, OUTPUT);
  Serial.begin(57600);

  // Serial.print("\nDisabling EEPROM Software Data Protection(SDP)...");
  // disableSoftwareWriteProtect();
  // Serial.println("SDP disabled \n");

  // Serial.print("Programming EEPROM...");
  // writeDecimalDisplayValues();
  // Serial.println("Done\n");

  // Serial.println("Reading EEPROM");
  // printContents();
  // Serial.println("Done\n");
}

void loop() {}

// Output the address bits and outputEnable signal using shift registers.
void setAddress(int addr, bool outputEnable) {
  // Set the highest bit as the output enable bit (active low)
  if (outputEnable) {
    addr &= ~0x8000;
  } else {
    addr |= 0x8000;
  }
  byte dataMask = 0x04;
  byte clkMask = 0x08;
  byte latchMask = 0x10;

  // Make sure the clock is low to start.
  PORTD &= ~clkMask;

  // Shift 16 bits in, starting with the MSB.
  for (uint16_t ix = 0; (ix < 16); ix++) {
    // Set the data bit
    if (addr & 0x8000) {
      PORTD |= dataMask;
    } else {
      PORTD &= ~dataMask;
    }

    // Toggle the clock high then low
    PORTD |= clkMask;
    delayMicroseconds(3);
    PORTD &= ~clkMask;
    addr <<= 1;
  }

  // Latch the shift register contents into the output register.
  PORTD &= ~latchMask;
  delayMicroseconds(1);
  PORTD |= latchMask;
  delayMicroseconds(1);
  PORTD &= ~latchMask;
}



// Read a byte from the EEPROM at the specified address.
byte readEEPROM(int address) {
  setDataBusMode(INPUT);
  setAddress(address, /*outputEnable*/ true);
  delay(15);
  return readDataBus();
}

// Write a byte to the EEPROM at the specified address.
void writeEEPROM(int address, byte data) {
  setAddress(address, /*outputEnable*/ false);
  setDataBusMode(OUTPUT);
  writeDataBus(data);
  enableWrite();
  delayMicroseconds(1);
  disableWrite();
  delay(10);
}

// Read the first 256 byte block of the EEPROM and dump it to the serial monitor.
void printContents() {
  byte digits[] = { 0x7e, 0x30, 0x6d, 0x79, 0x33, 0x5b, 0x5f, 0x70, 0x7f, 0x7b, 0x00, 0x01 };
  // 0 - 256
  // 256 - 512
  // 512 - 768
  // 768 - 1024
  // 1024 - 1280
  // 1280 - 1536
  // 1536 - 1792
  // 1792 - 2048

  for (int base = 0; (base < 128); base += 16) {
    // for (int base = 1920; (base < 2048); base += 16) {
    byte data[16];

    for (int offset = 0; offset <= 15; offset += 1) {
      data[offset] = readEEPROM(base + offset);
    }

    char buf[80];
    sprintf(buf, "%04x:  %02x %02x %02x %02x %02x %02x %02x %02x   %02x %02x %02x %02x %02x %02x %02x %02x",
            base, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
            data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]);
    Serial.println(buf);
  }
}

// Write the special six-byte code to turn off Software Data Protection.
void disableSoftwareWriteProtect() {
  disableWrite();
  setDataBusMode(OUTPUT);

  setByte(0xaa, 0x5555);
  setByte(0x55, 0x2aaa);
  setByte(0x80, 0x5555);
  setByte(0xaa, 0x5555);
  setByte(0x55, 0x2aaa);
  setByte(0x20, 0x5555);

  setDataBusMode(INPUT);
  delay(10);
}

// Write the special three-byte code to turn on Software Data Protection.
void enableSoftwareWriteProtect() {
  disableWrite();
  setDataBusMode(OUTPUT);

  setByte(0xaa, 0x5555);
  setByte(0x55, 0x2aaa);
  setByte(0xa0, 0x5555);

  setDataBusMode(INPUT);
  delay(10);
}

// Set the I/O state of the data bus.
// The 8 bits data bus are is on pins D5..D12.
void setDataBusMode(uint8_t mode) {
  // On the Uno and Nano, D5..D12 maps to the upper 3 bits of port D and the
  // lower 5 bits of port B.
  if (mode == OUTPUT) {
    DDRB |= 0x1f;
    DDRD |= 0xe0;
  } else {
    DDRB &= 0xe0;
    DDRD &= 0x1f;
  }
}

// Read a byte from the data bus.  The caller must set the bus to input_mode
// before calling this or no useful data will be returned.
byte readDataBus() {
  return (PINB << 3) | (PIND >> 5);
}

// Write a byte to the data bus.  The caller must set the bus to output_mode
// before calling this or no data will be written.
void writeDataBus(byte data) {
  PORTB = (PORTB & 0xe0) | (data >> 3);
  PORTD = (PORTD & 0x1f) | (data << 5);
}

// Set an address and data value and toggle the write control.  This is used
// to write control sequences, like the software write protect.  This is not a
// complete byte write function because it does not set the chip enable or the
// mode of the data bus.
void setByte(byte value, word address) {
  setAddress(address, false);
  writeDataBus(value);

  delayMicroseconds(1);
  enableWrite();
  delayMicroseconds(1);
  disableWrite();
}
