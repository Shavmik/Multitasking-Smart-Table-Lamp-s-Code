/*
  TFT RAW TEST - no libraries at all (no LovyanGFX). Plain digitalWrite() "bit-banged" SPI.

  Purpose: find out whether the problem is the SOFTWARE LIBRARY or the HARDWARE
  (wires, power, screen controller). If this sketch fills the screen with colours,
  the hardware is fine and only the LovyanGFX settings need adjusting.

  Wiring (same as before):
    screen VCC, LED -> 3V3        screen GND -> GND
    screen SDI -> GPIO 21         screen SCK -> GPIO 47        screen D/C -> GPIO 42
    screen CS  -> GND (or GPIO 1, then set PIN_CS 1)
    screen RESET -> 3V3 (or GPIO 2, then set PIN_RST 2)

  Expected: the screen fills RED, GREEN, BLUE, WHITE, BLACK, over and over. One fill takes
  1-3 seconds because the pins are toggled by software.
  Open the Serial Monitor (115200) to see which colour it is drawing.

  Things to try, one at a time:
    CONTROLLER 1 -> 2        (ST7789 instead of ILI9341)
    INVERT 0 -> 1            (if colours look inverted)
    PIN_RST 2 + a real wire  (hardware reset from GPIO 2)
*/

#define PIN_SCK   47
#define PIN_MOSI  21
#define PIN_DC    42
#define PIN_CS    -1     // -1 = CS wired to GND
#define PIN_RST   -1     // -1 = RESET wired to 3V3

#define CONTROLLER 1     // 1 = ILI9341, 2 = ST7789
#define INVERT     0

#define W 240
#define H 320

void spiWrite(uint8_t b) {
  for (int i = 7; i >= 0; i--) {
    digitalWrite(PIN_MOSI, (b >> i) & 1);
    digitalWrite(PIN_SCK, HIGH);
    digitalWrite(PIN_SCK, LOW);
  }
}

void csLow()  { if (PIN_CS >= 0) digitalWrite(PIN_CS, LOW); }
void csHigh() { if (PIN_CS >= 0) digitalWrite(PIN_CS, HIGH); }

void cmd(uint8_t c) {
  digitalWrite(PIN_DC, LOW);
  csLow();
  spiWrite(c);
  csHigh();
}

void data(uint8_t d) {
  digitalWrite(PIN_DC, HIGH);
  csLow();
  spiWrite(d);
  csHigh();
}

void cmdData(uint8_t c, const uint8_t *d, int n) {
  cmd(c);
  for (int i = 0; i < n; i++) data(d[i]);
}

#define CMD_DATA(c, ...) { const uint8_t _d[] = {__VA_ARGS__}; cmdData(c, _d, sizeof(_d)); }

void initDisplay() {
  if (PIN_RST >= 0) {
    pinMode(PIN_RST, OUTPUT);
    digitalWrite(PIN_RST, HIGH); delay(10);
    digitalWrite(PIN_RST, LOW);  delay(30);
    digitalWrite(PIN_RST, HIGH); delay(150);
  }
  cmd(0x01); delay(150);                 // software reset

#if CONTROLLER == 1
  // ILI9341
  CMD_DATA(0xEF, 0x03, 0x80, 0x02);
  CMD_DATA(0xCF, 0x00, 0xC1, 0x30);
  CMD_DATA(0xED, 0x64, 0x03, 0x12, 0x81);
  CMD_DATA(0xE8, 0x85, 0x00, 0x78);
  CMD_DATA(0xCB, 0x39, 0x2C, 0x00, 0x34, 0x02);
  CMD_DATA(0xF7, 0x20);
  CMD_DATA(0xEA, 0x00, 0x00);
  CMD_DATA(0xC0, 0x23);
  CMD_DATA(0xC1, 0x10);
  CMD_DATA(0xC5, 0x3E, 0x28);
  CMD_DATA(0xC7, 0x86);
  CMD_DATA(0x36, 0x48);
  CMD_DATA(0x37, 0x00);
  CMD_DATA(0x3A, 0x55);
  CMD_DATA(0xB1, 0x00, 0x18);
  CMD_DATA(0xB6, 0x08, 0x82, 0x27);
  CMD_DATA(0xF2, 0x00);
  CMD_DATA(0x26, 0x01);
  CMD_DATA(0xE0, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00);
  CMD_DATA(0xE1, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F);
  cmd(0x11); delay(150);                 // sleep out
  cmd(INVERT ? 0x21 : 0x20);
  cmd(0x29); delay(50);                  // display on
#else
  // ST7789
  cmd(0x11); delay(150);                 // sleep out
  CMD_DATA(0x3A, 0x55);                  // 16-bit colour
  CMD_DATA(0x36, 0x00);
  cmd(INVERT ? 0x21 : 0x20);
  cmd(0x13); delay(10);                  // normal mode
  cmd(0x29); delay(50);                  // display on
#endif
}

void fill(uint16_t color) {
  CMD_DATA(0x2A, 0x00, 0x00, (W - 1) >> 8, (W - 1) & 0xFF);
  CMD_DATA(0x2B, 0x00, 0x00, (H - 1) >> 8, (H - 1) & 0xFF);
  cmd(0x2C);
  digitalWrite(PIN_DC, HIGH);
  csLow();
  uint8_t hi = color >> 8, lo = color & 0xFF;
  for (uint32_t i = 0; i < (uint32_t)W * H; i++) {
    spiWrite(hi);
    spiWrite(lo);
  }
  csHigh();
}

void setup() {
  Serial.begin(115200);
  delay(2500);
  Serial.println("RAW TFT test starting...");

  pinMode(PIN_SCK, OUTPUT);
  pinMode(PIN_MOSI, OUTPUT);
  pinMode(PIN_DC, OUTPUT);
  digitalWrite(PIN_SCK, LOW);
  if (PIN_CS >= 0) { pinMode(PIN_CS, OUTPUT); digitalWrite(PIN_CS, HIGH); }

  initDisplay();
  Serial.println("display initialised");
}

void loop() {
  Serial.println("RED");   fill(0xF800); delay(500);
  Serial.println("GREEN"); fill(0x07E0); delay(500);
  Serial.println("BLUE");  fill(0x001F); delay(500);
  Serial.println("WHITE"); fill(0xFFFF); delay(500);
  Serial.println("BLACK"); fill(0x0000); delay(500);
}
