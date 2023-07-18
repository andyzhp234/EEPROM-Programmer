#define SHIFT_DATA 2
#define SHIFT_CLK 3
#define SHIFT_LATCH 4
#define EEPROM_D0 5
#define EEPROM_D7 12
#define WRITE_EN 13

void writeDecimalDisplayValues() {
  byte digits[] = { 0x7e, 0x30, 0x6d, 0x79, 0x33, 0x5b, 0x5f, 0x70, 0x7f, 0x7b };
  Serial.println("Programming ones place");
  for (int value = 0; value <= 255; value += 1) {
    writeEEPROM(value, digits[value % 10]);
  }

  Serial.println("Programming tens place");
  for (int value = 0; value <= 255; value += 1) {
    writeEEPROM(value + 256, digits[(value / 10) % 10]);
  }

  Serial.println("Programming hundreds place");
  for (int value = 0; value <= 255; value += 1) {
    writeEEPROM(value + 512, digits[(value / 100) % 10]);
  }

  Serial.println("Programming sign");
  for (int value = 0; value <= 255; value += 1) {
    writeEEPROM(value + 768, 0);
  }

  // ----------------------------------------------------------------
  Serial.println("Programming ones place (twos complement)");
  for (int value = -128; value <= 127; value += 1) {
    writeEEPROM((byte)value + 1024, digits[abs(value) % 10]);
  }

  Serial.println("Programming tens place (twos complement)");
  for (int value = -128; value <= 127; value += 1) {
    writeEEPROM((byte)value + 1280, digits[abs(value / 10) % 10]);
  }

  Serial.println("Programming hundreds place (twos complement)");
  for (int value = -128; value <= 127; value += 1) {
    writeEEPROM((byte)value + 1536, digits[abs(value / 100) % 10]);
  }

  Serial.println("Programming sign (twos complement)");
  for (int value = -128; value <= 127; value += 1) {
    if (value < 0) {
      writeEEPROM((byte)value + 1792, 0x01);
    } else {
      writeEEPROM((byte)value + 1792, 0);
    }
  }
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

  for (int base = 256*0; (base < 256*1); base += 16) {
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

void setup() {
  initUCode();
  // put your setup code here, to run once:
  pinMode(SHIFT_DATA, OUTPUT);
  pinMode(SHIFT_CLK, OUTPUT);
  pinMode(SHIFT_LATCH, OUTPUT);
  disableWrite();
  pinMode(WRITE_EN, OUTPUT);
  Serial.begin(57600);

  Serial.println("Programming EEPROM...");
  writeDecimalDisplayValues();
  Serial.println("Done\n");

  Serial.println("Reading EEPROM");
  printContents();
  Serial.println("Done\n");
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
  delay(10);
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

void enableWrite() {
  digitalWrite(WRITE_EN, LOW);
}

void disableWrite() {
  digitalWrite(WRITE_EN, HIGH);
}
