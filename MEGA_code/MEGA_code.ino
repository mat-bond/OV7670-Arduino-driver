/*
  OV7670 VGA RGB565 still-image streamer for Arduino Mega 2560

  Architecture: one contiguous 8-bit camera port, a 1'280-bytes line buffer,
  direct UART register access, and camera/UART work overlapped while each
  row is captured. The host protocol is intentionally simple for Python compatibility.
*/

#ifndef F_CPU
#define F_CPU 16000000UL      // Arduino MEGA-2560 runs on 16 MHz
#endif

#include <Arduino.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#if !defined(__AVR_ATmega2560__)
#error "This sketch is for Arduino Mega 2560 only."
#endif

// -----------------------------------------------------------------------------
// User-tunable settings
// -----------------------------------------------------------------------------

static constexpr uint16_t FRAME_WIDTH = 640;
static constexpr uint16_t FRAME_HEIGHT = 480;
static constexpr uint16_t BYTES_PER_ROW = FRAME_WIDTH * 2; 
static constexpr uint32_t FRAME_BYTES = uint32_t(FRAME_WIDTH) * FRAME_HEIGHT * 2UL;

static constexpr uint32_t UART_BAUD = 2000000UL;

// Maximum OV7670 clock divider. This gives enough time to capture one HREF row,
// then transmit the complete 1,280-byte RGB565 row before the following row.
static constexpr uint8_t CAMERA_CLOCK_PRESCALER = 63;

static constexpr bool USE_MANUAL_EXPOSURE = true;
static constexpr uint16_t MANUAL_EXPOSURE = 6;
static constexpr uint8_t MANUAL_GAIN = 0x00;

static constexpr uint8_t OV7670_I2C_ADDRESS = 0x21;
static constexpr uint8_t VGA_VERTICAL_PADDING_ROWS = 10;

// VGA window normally has one non-image byte at the start
// of each HREF row. Change this to 0 only if the image is horizontally
// shifted by exactly one byte after the timing corruption is fixed.
static constexpr uint8_t ROW_LEADING_PADDING_BYTES = 1;

// -----------------------------------------------------------------------------
// OV7670 registers used by the configuration tables
// -----------------------------------------------------------------------------

#define REG_GAIN       0x00
#define REG_BLUE       0x01
#define REG_RED        0x02
#define REG_VREF       0x03
#define REG_COM1       0x04
#define REG_COM2       0x09
#define REG_PID        0x0A
#define REG_VER        0x0B
#define REG_COM3       0x0C
#define COM3_SWAP      0x40
#define COM3_DCWEN     0x04
#define REG_COM4       0x0D
#define REG_COM5       0x0E
#define REG_COM6       0x0F
#define COM6_HREF_HB   0x80
#define REG_AECH       0x10
#define REG_CLKRC      0x11
#define REG_COM7       0x12
#define COM7_RESET     0x80
#define COM7_FMT_QVGA  0x10
#define COM7_RGB       0x04
#define REG_COM8       0x13
#define COM8_FASTAEC   0x80
#define COM8_AECSTEP   0x40
#define COM8_BFILT     0x20
#define COM8_AGC       0x04
#define COM8_AWB       0x02
#define COM8_AEC       0x01
#define REG_COM9       0x14
#define REG_COM10      0x15
#define COM10_PCLK_HB  0x20
#define REG_HSTART     0x17
#define REG_HSTOP      0x18
#define REG_VSTART     0x19
#define REG_VSTOP      0x1A
#define REG_MVFP       0x1E
#define REG_AEW        0x24
#define REG_AEB        0x25
#define REG_VPT        0x26
#define REG_HREF       0x32
#define REG_TSLB       0x3A
#define REG_COM11      0x3B
#define COM11_HZAUTO   0x10
#define COM11_EXP      0x02
#define REG_COM12      0x3C
#define REG_COM13      0x3D
#define COM13_UVSAT    0x40
#define REG_COM14      0x3E
#define REG_EDGE       0x3F
#define REG_COM15      0x40
#define COM15_R00FF    0xC0
#define COM15_RGB565   0x10
#define REG_COM16      0x41
#define COM16_AWBGAIN  0x08
#define REG_COM17      0x42
#define COM17_CBAR     0x08
#define REG_BRIGHT     0x55
#define REG_CONTRAS    0x56
#define REG_GFIX       0x69
#define DBLV           0x6B
#define SCALING_DCWCTR 0x72
#define SCALING_PCLK_DIV 0x73
#define REG_REG76      0x76
#define REG_RGB444     0x8C
#define REG_HAECC1     0x9F
#define REG_HAECC2     0xA0
#define REG_BD50MAX    0xA5
#define REG_HAECC3     0xA6
#define REG_HAECC4     0xA7
#define REG_HAECC5     0xA8
#define REG_HAECC6     0xA9
#define REG_HAECC7     0xAA
#define REG_BD60MAX    0xAB
#define REG_AECHH      0x07

struct RegisterSetting {
  uint8_t address;
  uint8_t value;
};

// Standard OV7670 base configuration used by no-RAM AVR implementations.
static const RegisterSetting DEFAULT_REGISTERS[] PROGMEM = {
  {REG_COM7, COM7_RESET},
  {REG_TSLB, 0x04},
  {REG_COM7, 0x00},

  {REG_HSTART, 0x13}, {REG_HSTOP, 0x01},
  {REG_HREF, 0xB6}, {REG_VSTART, 0x02},
  {REG_VSTOP, 0x7A}, {REG_VREF, 0x0A},
  {REG_COM3, 0x00}, {REG_COM14, 0x00},

  {0x70, 0x3A}, {0x71, 0x35},
  {SCALING_DCWCTR, 0x11}, {SCALING_PCLK_DIV, 0xF0},
  {0xA2, 0x01}, {REG_COM10, 0x00},

  {0x7A, 0x20}, {0x7B, 0x10},
  {0x7C, 0x1E}, {0x7D, 0x35},
  {0x7E, 0x5A}, {0x7F, 0x69},
  {0x80, 0x76}, {0x81, 0x80},
  {0x82, 0x88}, {0x83, 0x8F},
  {0x84, 0x96}, {0x85, 0xA3},
  {0x86, 0xAF}, {0x87, 0xC4},
  {0x88, 0xD7}, {0x89, 0xE8},

  {REG_COM8, COM8_FASTAEC | COM8_AECSTEP},
  {REG_GAIN, 0x00}, {REG_AECH, 0x00},
  {REG_COM4, 0x40}, {REG_COM9, 0x18},
  {REG_BD50MAX, 0x05}, {REG_BD60MAX, 0x07},
  {REG_AEW, 0x95}, {REG_AEB, 0x33},
  {REG_VPT, 0xE3}, {REG_HAECC1, 0x78},
  {REG_HAECC2, 0x68}, {0xA1, 0x03},
  {REG_HAECC3, 0xD8}, {REG_HAECC4, 0xD8},
  {REG_HAECC5, 0xF0}, {REG_HAECC6, 0x90},
  {REG_HAECC7, 0x94},
  {REG_COM8, COM8_FASTAEC | COM8_AECSTEP | COM8_AGC | COM8_AEC},

  {0x30, 0x00}, {0x31, 0x00},
  {REG_COM5, 0x61}, {REG_COM6, 0x4B},
  {0x16, 0x02}, {REG_MVFP, 0x07},
  {0x21, 0x02}, {0x22, 0x91},
  {0x29, 0x07}, {0x33, 0x0B},
  {0x35, 0x0B}, {0x37, 0x1D},
  {0x38, 0x71}, {0x39, 0x2A},
  {REG_COM12, 0x78}, {0x4D, 0x40},
  {0x4E, 0x20}, {REG_GFIX, 0x00},
  {0x74, 0x10},
  {0x8D, 0x4F}, {0x8E, 0x00},
  {0x8F, 0x00}, {0x90, 0x00},
  {0x91, 0x00}, {0x96, 0x00},
  {0x9A, 0x00}, {0xB0, 0x84},
  {0xB1, 0x0C}, {0xB2, 0x0E},
  {0xB3, 0x82}, {0xB8, 0x0A},

  {0x43, 0x0A}, {0x44, 0xF0},
  {0x45, 0x34}, {0x46, 0x58},
  {0x47, 0x28}, {0x48, 0x3A},
  {0x59, 0x88}, {0x5A, 0x88},
  {0x5B, 0x44}, {0x5C, 0x67},
  {0x5D, 0x49}, {0x5E, 0x0E},
  {0x6C, 0x0A}, {0x6D, 0x55},
  {0x6E, 0x11}, {0x6F, 0x9E},
  {0x6A, 0x40}, {REG_BLUE, 0x40},
  {REG_RED, 0x60},
  {REG_COM8, COM8_FASTAEC | COM8_AECSTEP | COM8_AGC | COM8_AEC | COM8_AWB},

  {0x4F, 0x80}, {0x50, 0x80},
  {0x51, 0x00}, {0x52, 0x22},
  {0x53, 0x5E}, {0x54, 0x80},
  {0x58, 0x9E},

  {REG_COM16, COM16_AWBGAIN}, {REG_EDGE, 0x00},
  {0x75, 0x05}, {REG_REG76, 0xE1},
  {0x4C, 0x00}, {0x77, 0x01},
  {REG_COM13, 0x48}, {0x4B, 0x09},
  {0xC9, 0x60}, {0x56, 0x40},

  {0x34, 0x11}, {REG_COM11, COM11_EXP | COM11_HZAUTO},
  {0xA4, 0x82}, {0x96, 0x00},
  {0x97, 0x30}, {0x98, 0x20},
  {0x99, 0x30}, {0x9A, 0x84},
  {0x9B, 0x29}, {0x9C, 0x03},
  {0x9D, 0x4C}, {0x9E, 0x3F},
  {0x78, 0x04},

  {0x79, 0x01}, {0xC8, 0xF0},
  {0x79, 0x0F}, {0xC8, 0x00},
  {0x79, 0x10}, {0xC8, 0x7E},
  {0x79, 0x0A}, {0xC8, 0x80},
  {0x79, 0x0B}, {0xC8, 0x01},
  {0x79, 0x0C}, {0xC8, 0x0F},
  {0x79, 0x0D}, {0xC8, 0x20},
  {0x79, 0x09}, {0xC8, 0x80},
  {0x79, 0x02}, {0xC8, 0xC0},
  {0x79, 0x03}, {0xC8, 0x40},
  {0x79, 0x05}, {0xC8, 0x30},
  {0x79, 0x26},

  {0xFF, 0xFF}
};

// RGB565 configuration from the same no-RAM AVR lineage.
static const RegisterSetting RGB565_REGISTERS[] PROGMEM = {
  {REG_COM7, COM7_RGB},
  {REG_RGB444, 0x00},
  {REG_COM1, 0x00},
  {REG_COM15, COM15_RGB565 | COM15_R00FF},
  {REG_COM9, 0x6A},
  {0x4F, 0xB3},
  {0x50, 0xB3},
  {0x51, 0x00},
  {0x52, 0x3D},
  {0x53, 0xA7},
  {0x54, 0xE4},
  {REG_COM13, COM13_UVSAT},
  {0xFF, 0xFF}
};

// VGA window. It deliberately includes ten initial rows that
// are consumed and discarded before the 480 useful rows.
static const RegisterSetting VGA_REGISTERS[] PROGMEM = {
  // VGA timing:
  // 480 useful rows + 10 leading rows, and 640 pixels + 4 padding bytes.
  {REG_VSTART, 0x00},      // 0 >> 2
  {REG_VSTOP, 0x7A},       // (480 + 10) >> 2
  {REG_VREF, 0x08},        // low two bits of VSTOP
  {REG_HSTART, 0x13},      // 156 >> 3
  {REG_HSTOP, 0x01},       // 14 >> 3
  {REG_HREF, 0x34},        // HSTART/HSTOP low bits
  {0xFF, 0xFF}
};

// One complete RGB565 line, not a complete frame.
static uint8_t lineBuffer[BYTES_PER_ROW];

// -----------------------------------------------------------------------------
// Low-level UART
// -----------------------------------------------------------------------------

static void configureUart2M() {
  UBRR0H = 0;
  UBRR0L = 0;                         // Exact 2 Mbps with U2X at 16 MHz
  UCSR0A = _BV(U2X0);
  UCSR0B = _BV(RXEN0) | _BV(TXEN0);
  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); // 8N1
}

static inline bool uartTxReady() {
  return (UCSR0A & _BV(UDRE0)) != 0;
}

static inline void uartWriteByte(uint8_t value) {
  while (!uartTxReady()) {}
  UDR0 = value;
}

static void uartWriteText(const char *text) {
  while (*text) {
    uartWriteByte(static_cast<uint8_t>(*text++));
  }
}

static void uartWaitComplete() {
  while (!(UCSR0A & _BV(TXC0))) {}
  UCSR0A |= _BV(TXC0);
}

static bool waitForHostGo(uint32_t timeoutMs) {
  const uint32_t start = millis();

  while ((millis() - start) < timeoutMs) {
    if (UCSR0A & _BV(RXC0)) {
      const uint8_t value = UDR0;
      if (value == 'G' || value == 'g') {
        return true;
      }
    }
  }

  return false;
}

// -----------------------------------------------------------------------------
// XCLK and camera input pins
// -----------------------------------------------------------------------------

static void configureCameraPins() {
  // D0..D7 on Mega pins 22..29 = PA0..PA7.
  DDRA = 0x00;
  PORTA = 0x00;

  // PCLK on Mega pin 12 = PB6.
  DDRB &= ~_BV(DDB6);
  PORTB &= ~_BV(PORTB6);

  // VSYNC on Mega pin 2 = PE4.
  DDRE &= ~_BV(DDE4);
  PORTE &= ~_BV(PORTE4);

  // HREF/HS on Mega pin 8 = PH5.
  DDRH &= ~_BV(DDH5);
  PORTH &= ~_BV(PORTH5);
}

static void startCameraClock4MHz() {
  // Mega pin 10 is OC2A/PB4.
  DDRB |= _BV(DDB4);

  // CTC mode: toggle OC2A whenever Timer2 reaches OCR2A.
  // With F_CPU=16 MHz, prescaler=1 and OCR2A=1, XCLK is 4 MHz.
  ASSR &= ~(_BV(EXCLK) | _BV(AS2));
  TCCR2A = _BV(COM2A0) | _BV(WGM21);
  TCCR2B = _BV(CS20);
  TCNT2 = 0;
  OCR2A = 1;
}

static inline bool pixelClockHigh() {
  return (PINB & _BV(PINB6)) != 0;
}

static inline bool verticalSyncHigh() {
  return (PINE & _BV(PINE4)) != 0;
}

static inline bool hrefHigh() {
  return (PINH & _BV(PINH5)) != 0;
}

static inline void waitForPixelClockRisingEdge() {
  while (pixelClockHigh()) {}
  while (!pixelClockHigh()) {}
}

static inline uint8_t readPixelByte() {
  const uint8_t value = PINA;
  asm volatile("nop\n\t"
               "nop\n\t"
               "nop\n\t"
               "nop\n\t");
  return value;
}

// -----------------------------------------------------------------------------
// Camera register access
// -----------------------------------------------------------------------------

static bool writeRegister(uint8_t address, uint8_t value) {
  // SCCB can occasionally miss an acknowledgement immediately after reset.
  // Retry a few times instead of treating one transient failure as fatal.
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    Wire.beginTransmission(OV7670_I2C_ADDRESS);
    Wire.write(address);
    Wire.write(value);

    if (Wire.endTransmission(true) == 0) {  // Send a STOP condition.
      delayMicroseconds(100);
      return true;
    }

    delay(2);
  }

  return false;
}

static uint8_t readRegister(uint8_t address) {
  // OV7670 uses SCCB. A STOP between the register-address write and the
  // read transaction is more reliable than an I2C repeated START.
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    Wire.beginTransmission(OV7670_I2C_ADDRESS);
    Wire.write(address);

    if (Wire.endTransmission(true) != 0) {  // STOP, not repeated START.
      delay(2);
      continue;
    }

    delayMicroseconds(100);

    if (Wire.requestFrom(OV7670_I2C_ADDRESS, uint8_t(1), uint8_t(true)) == 1) {
      return Wire.read();
    }

    delay(2);
  }

  return 0xFF;
}

static bool writeRegisterList(const RegisterSetting *list) {
  while (true) {
    const uint8_t address = pgm_read_byte(&list->address);
    const uint8_t value = pgm_read_byte(&list->value);

    if (address == 0xFF && value == 0xFF) {
      return true;
    }

    if (!writeRegister(address, value)) {
      return false;
    }

    ++list;
  }
}

static void setManualExposure(uint16_t exposure, uint8_t gain) {
  uint8_t com8 = readRegister(REG_COM8);
  com8 &= uint8_t(~(COM8_AGC | COM8_AEC));
  com8 |= COM8_AWB;
  writeRegister(REG_COM8, com8);
  delay(10);

  const uint8_t com1 = readRegister(REG_COM1);
  const uint8_t aechh = readRegister(REG_AECHH);

  writeRegister(REG_GAIN, gain);
  writeRegister(REG_COM1, (com1 & 0xFC) | (exposure & 0x03));
  writeRegister(REG_AECH, uint8_t((exposure >> 2) & 0xFF));
  writeRegister(REG_AECHH, (aechh & 0xC0) | uint8_t((exposure >> 10) & 0x3F));
}

static bool initialiseCamera() {
  Wire.begin();
  Wire.setClock(100000UL);

  // Disable the Mega's internal 5 V pull-ups. External 4.7k pull-ups to
  // 3.3 V must be fitted on SDA and SCL.
  digitalWrite(20, LOW);
  digitalWrite(21, LOW);

  if (!writeRegister(REG_COM7, COM7_RESET)) {
    return false;
  }
  delay(500);

  if (!writeRegisterList(DEFAULT_REGISTERS)) {
    return false;
  }

  if (!writeRegisterList(RGB565_REGISTERS)) {
    return false;
  }

  if (!writeRegisterList(VGA_REGISTERS)) {
    return false;
  }

  // Match the capture assumptions.
  writeRegister(REG_COM10, readRegister(REG_COM10) | COM10_PCLK_HB);
  writeRegister(REG_COM6, readRegister(REG_COM6) | COM6_HREF_HB);
  writeRegister(REG_CLKRC, 0x80 | CAMERA_CLOCK_PRESCALER);
  writeRegister(REG_COM17, readRegister(REG_COM17) & uint8_t(~COM17_CBAR));

  if (USE_MANUAL_EXPOSURE) {
    setManualExposure(MANUAL_EXPOSURE, MANUAL_GAIN);
  }

  delay(2500);

  const uint8_t pid = readRegister(REG_PID);
  const uint8_t ver = readRegister(REG_VER);

  if (pid != 0x76 || ver != 0x73) {
    uartWriteText("*BADID*");
    return false;
  }

  return true;
}

// -----------------------------------------------------------------------------
// HREF-synchronised frame timing
// -----------------------------------------------------------------------------

static void waitForFrameStart() {
  // Wait for a VSYNC pulse, then for the active frame interval.
  while (!verticalSyncHigh()) {}
  while (verticalSyncHigh()) {}
}

static bool waitForNextHrefHigh() {
  // Require a real low-to-high HREF transition. This prevents starting in
  // the middle of a row if UART work ran into the following camera row.
  while (hrefHigh()) {
    if (verticalSyncHigh()) {
      return false;
    }
  }

  while (!hrefHigh()) {
    if (verticalSyncHigh()) {
      return false;
    }
  }

  return true;
}

static inline bool readPixelByteWhileHref(uint8_t &value) {
  // Wait for PCLK low.
  while (pixelClockHigh()) {
    if (!hrefHigh()) {
      return false;
    }
  }

  // Sample on the next PCLK rising edge.
  while (!pixelClockHigh()) {
    if (!hrefHigh()) {
      return false;
    }
  }

  value = PINA;

  asm volatile("nop\n\t"
               "nop\n\t"
               "nop\n\t"
               "nop\n\t");

  return true;
}

static bool discardHrefRow() {
  if (!waitForNextHrefHigh()) {
    return false;
  }

  while (hrefHigh()) {}
  return true;
}

static bool captureAndStreamRgb565Frame() {
  waitForFrameStart();

  // VGA window contains ten initial garbage rows.
  for (uint8_t row = 0; row < VGA_VERTICAL_PADDING_ROWS; ++row) {
    if (!discardHrefRow()) {
      return false;
    }
  }

  for (uint16_t y = 0; y < FRAME_HEIGHT; ++y) {
    if (!waitForNextHrefHigh()) {
      return false;
    }

    // Discard the VGA window's leading padding byte(s).
    for (uint8_t padding = 0;
         padding < ROW_LEADING_PADDING_BYTES;
         ++padding) {
      uint8_t ignoredByte = 0;
      if (!readPixelByteWhileHref(ignoredByte)) {
        return false;
      }
    }

    uint8_t *sendPointer = lineBuffer;

    // Capture from PINA and transmit already-captured bytes in parallel.
    // At prescaler 63, UART is faster than the camera byte stream, so only
    // zero or a few bytes should remain to drain at the end of each row.
    for (uint16_t index = 0; index < BYTES_PER_ROW; ++index) {
      if (!readPixelByteWhileHref(lineBuffer[index])) {
        return false;
      }

      if (sendPointer <= &lineBuffer[index] && uartTxReady()) {
        UDR0 = *sendPointer++;
      }
    }

    // Ignore the right-padding bytes and wait for a clean HREF-low interval.
    while (hrefHigh()) {}

    // Drain only the small unsent tail. Do not send a complete 640-byte row
    // here, because that would run into the next HREF interval.
    while (sendPointer < lineBuffer + BYTES_PER_ROW) {
      uartWriteByte(*sendPointer++);
    }
  }

  uartWaitComplete();
  return true;
}

static void fatalError() {
  uartWriteText("*ERR*");
  pinMode(LED_BUILTIN, OUTPUT);

  while (true) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);
    digitalWrite(LED_BUILTIN, LOW);
    delay(150);
  }
}

void setup() {
  cli();
  configureCameraPins();
  configureUart2M();
  startCameraClock4MHz();
  sei();

  delay(100);

  if (!initialiseCamera()) {
    fatalError();
  }

  uartWriteText("*INIT*");
}

void loop() {
  uartWriteText("*RDY*");

  if (!waitForHostGo(30000UL)) {
    return;
  }

  uartWriteText("*IMG*");
  uartWaitComplete();

  noInterrupts();
  const bool frameComplete = captureAndStreamRgb565Frame();
  interrupts();

  // If row synchronisation failed, the host will time out and the next request can retry.
  (void)frameComplete;
}
