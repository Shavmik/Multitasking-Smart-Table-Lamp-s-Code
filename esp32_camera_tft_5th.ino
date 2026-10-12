/*
  ESP32-S3 Camera -> USB image server + TFT screen (for the Python OCR terminal app)

  What it does
  ------------
  * 'C' = take a full-quality photo, save a copy to the SD card (if present),
          send it to the laptop AND show it on the TFT screen.
  * 'V' = send one quick preview frame (320x240) to the laptop AND draw it on the
          TFT screen. The laptop asks for these over and over = live video.
  * 'T' = show a text page on the TFT (used by the laptop for OCR results).
  * 'P' = ping, answers "PONG SD=1" or "PONG SD=0".

  Libraries needed (Arduino IDE -> Library Manager)
  -------------------------------------------------
  * "LovyanGFX" by lovyan03   (screen driver + JPEG drawing, no other library needed)

  Arduino IDE settings (Tools menu)
  ---------------------------------
  Board ............. "ESP32S3 Dev Module"
  USB CDC On Boot ... Enabled
  PSRAM ............. "OPI PSRAM"  (if the camera fails to start, try "QSPI PSRAM")
  Flash Size ........ match your board (commonly 8MB or 16MB)
  Partition Scheme .. "Huge APP" or the default
  Core Debug Level .. None

  Protocol
  --------
  Photo reply:  "IMG!" + 4-byte little-endian length + JPEG bytes
  Error reply:  "ERR!"
  Text command: 'T' + 2-byte little-endian length + ASCII text ("Title\nBody...")
*/

#include "esp_camera.h"
#include "img_converters.h"
#include "FS.h"

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

// ===================== STEP 1: choose YOUR board (uncomment ONE) =====================
#define BOARD_FREENOVE_S3_CAM      // Freenove ESP32-S3 WROOM CAM and ESP32-S3-EYE style boards
//#define BOARD_XIAO_S3_SENSE      // Seeed XIAO ESP32S3 Sense (very few free pins, TFT needs rewiring)
//#define BOARD_CUSTOM             // fill in the pins yourself below
// =====================================================================================

// ===================== STEP 2: TFT screen pins (MUST match your wires) ===============
// Pins already used by the Freenove board: camera = 4-13, 15-18; SD card = 38,39,40;
// PSRAM = 35,36,37; USB = 19,20; boot/strapping = 0,3,45,46.
// Free pins you can use: 1, 2, 14, 21, 41, 42, 47, 48  (GPIO2 also drives the onboard LED, 48 the RGB LED)
//
// BREADBOARD WIRING USED HERE:
//   screen VCC  -> ESP32 3V3        screen LED   -> 3V3 (same strip as VCC)
//   screen GND  -> ESP32 GND
//   screen SDI  -> GPIO 21 (MOSI)   screen SCK   -> GPIO 47     screen D/C -> GPIO 42
//   screen CS   -> GND  (tied low, always selected)   => PIN_TFT_CS  = -1
//   screen RESET-> 3V3  (tied high, no reset wire)    => PIN_TFT_RST = -1
// Later, if you want to use real wires for them: CS -> GPIO 1 and RESET -> GPIO 2, then set the
// two numbers below to 1 and 2.
#define PIN_TFT_SCLK  47   // screen pin "SCK"
#define PIN_TFT_MOSI  21   // screen pin "SDI (MOSI)"
#define PIN_TFT_MISO  -1   // screen pin "SDO (MISO)" - not needed, leave unconnected
#define PIN_TFT_DC    42   // screen pin "D/C"
#define PIN_TFT_CS    -1   // -1 = CS wired to GND.   (use 1 if wired to GPIO 1)
#define PIN_TFT_RST   -1   // -1 = RESET wired to 3V3. (use 2 if wired to GPIO 2)
#define PIN_TFT_BL    -1   // screen pin "LED". -1 = LED is wired to 3V3 (always on)

#define TFT_ROTATION   1       // 1 = landscape 320x240. Use 3 if the picture is upside down.
#define TFT_INVERT     false   // set true if colours look inverted (black <-> white)
#define TFT_RGB_ORDER  false   // set true if red and blue are swapped
// =====================================================================================

#if defined(BOARD_FREENOVE_S3_CAM)
  #include "SD_MMC.h"
  #define STORAGE SD_MMC
  #define USE_SD_MMC
  #define SD_MMC_CLK 39
  #define SD_MMC_CMD 38
  #define SD_MMC_D0  40

  #define PWDN_GPIO_NUM  -1
  #define RESET_GPIO_NUM -1
  #define XCLK_GPIO_NUM  15
  #define SIOD_GPIO_NUM   4
  #define SIOC_GPIO_NUM   5
  #define Y9_GPIO_NUM    16
  #define Y8_GPIO_NUM    17
  #define Y7_GPIO_NUM    18
  #define Y6_GPIO_NUM    12
  #define Y5_GPIO_NUM    10
  #define Y4_GPIO_NUM     8
  #define Y3_GPIO_NUM     9
  #define Y2_GPIO_NUM    11
  #define VSYNC_GPIO_NUM  6
  #define HREF_GPIO_NUM   7
  #define PCLK_GPIO_NUM  13

#elif defined(BOARD_XIAO_S3_SENSE)
  #include "SD.h"
  #include "SPI.h"
  #define STORAGE SD
  #define USE_SD_SPI
  #define SD_CS_PIN 21

  #define PWDN_GPIO_NUM  -1
  #define RESET_GPIO_NUM -1
  #define XCLK_GPIO_NUM  10
  #define SIOD_GPIO_NUM  40
  #define SIOC_GPIO_NUM  39
  #define Y9_GPIO_NUM    48
  #define Y8_GPIO_NUM    11
  #define Y7_GPIO_NUM    12
  #define Y6_GPIO_NUM    14
  #define Y5_GPIO_NUM    16
  #define Y4_GPIO_NUM    18
  #define Y3_GPIO_NUM    17
  #define Y2_GPIO_NUM    15
  #define VSYNC_GPIO_NUM 38
  #define HREF_GPIO_NUM  47
  #define PCLK_GPIO_NUM  13

#elif defined(BOARD_CUSTOM)
  // Use the pin table from your board's documentation.
  #include "SD_MMC.h"
  #define STORAGE SD_MMC
  #define USE_SD_MMC
  #define SD_MMC_CLK 0
  #define SD_MMC_CMD 0
  #define SD_MMC_D0  0

  #define PWDN_GPIO_NUM  -1
  #define RESET_GPIO_NUM -1
  #define XCLK_GPIO_NUM   0
  #define SIOD_GPIO_NUM   0
  #define SIOC_GPIO_NUM   0
  #define Y9_GPIO_NUM     0
  #define Y8_GPIO_NUM     0
  #define Y7_GPIO_NUM     0
  #define Y6_GPIO_NUM     0
  #define Y5_GPIO_NUM     0
  #define Y4_GPIO_NUM     0
  #define Y3_GPIO_NUM     0
  #define Y2_GPIO_NUM     0
  #define VSYNC_GPIO_NUM  0
  #define HREF_GPIO_NUM   0
  #define PCLK_GPIO_NUM   0
#else
  #error "Please select a board at the top of the sketch."
#endif

// ------------------------------- TFT driver -------------------------------
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9341 _panel;     // most 2.8" 240x320 SPI screens. If yours stays blank,
                                  // try lgfx::Panel_ST7789 instead.
  lgfx::Bus_SPI _bus;
public:
  LGFX() {
    {
      auto cfg = _bus.config();
      cfg.spi_host    = SPI2_HOST;
      cfg.spi_mode    = 0;
      cfg.freq_write  = 20000000;   // long jumper wires: keep it low. Try 10000000 if the picture is noisy
      cfg.freq_read   = 16000000;
      cfg.spi_3wire   = false;
      cfg.use_lock    = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk    = PIN_TFT_SCLK;
      cfg.pin_mosi    = PIN_TFT_MOSI;
      cfg.pin_miso    = PIN_TFT_MISO;
      cfg.pin_dc      = PIN_TFT_DC;
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }
    {
      auto cfg = _panel.config();
      cfg.pin_cs           = PIN_TFT_CS;
      cfg.pin_rst          = PIN_TFT_RST;
      cfg.pin_busy         = -1;
      cfg.panel_width      = 240;
      cfg.panel_height     = 320;
      cfg.offset_x         = 0;
      cfg.offset_y         = 0;
      cfg.offset_rotation  = 0;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits  = 1;
      cfg.readable         = false;
      cfg.invert           = TFT_INVERT;
      cfg.rgb_order        = TFT_RGB_ORDER;
      cfg.dlen_16bit       = false;
      cfg.bus_shared       = false;
      _panel.config(cfg);
    }
    setPanel(&_panel);
  }
};

LGFX tft;

#define MAX_TEXT 600
static const framesize_t PREVIEW_SIZE = FRAMESIZE_QVGA;   // 320x240 = exactly the screen

bool sdOk = false;
bool jpegMode = true;   // false = camera can't make JPEG itself, we convert
uint32_t nextIndex = 1;
framesize_t photoSize   = FRAMESIZE_UXGA;
framesize_t currentSize = FRAMESIZE_UXGA;

// ------------------------------- TFT helpers -------------------------------
void initTFT() {
  if (PIN_TFT_BL >= 0) {
    pinMode(PIN_TFT_BL, OUTPUT);
    digitalWrite(PIN_TFT_BL, HIGH);
  }
  tft.init();
  tft.setRotation(TFT_ROTATION);
  tft.fillScreen(TFT_BLACK);
}

// First line = title (cyan), the rest = body (white).
void showTextPage(const String &msg) {
  int nl = msg.indexOf('\n');
  String title = (nl >= 0) ? msg.substring(0, nl) : msg;
  String body  = (nl >= 0) ? msg.substring(nl + 1) : String("");

  tft.fillScreen(TFT_BLACK);
  tft.setFont(&fonts::Font2);
  tft.setTextWrap(true, false);

  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(4, 4);
  tft.print(title);
  tft.drawFastHLine(0, 24, tft.width(), tft.color565(90, 90, 90));

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(4, 30);
  tft.print(body);
}

void handleText() {
  uint8_t hdr[2];
  if (Serial.readBytes(hdr, 2) != 2) return;
  size_t n = hdr[0] | (hdr[1] << 8);
  if (n > MAX_TEXT) n = MAX_TEXT;
  static char buf[MAX_TEXT + 1];
  size_t got = Serial.readBytes(buf, n);
  buf[got] = 0;
  showTextPage(String(buf));
}

// ------------------------------- SD card -------------------------------
bool initSD() {
#if defined(USE_SD_MMC)
  SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
  if (!SD_MMC.begin("/sdcard", true)) return false;   // 1-bit mode
  return SD_MMC.cardType() != CARD_NONE;
#else
  if (!SD.begin(SD_CS_PIN)) return false;
  return SD.cardType() != CARD_NONE;
#endif
}

// Find the next free photo number so old photos are never overwritten.
uint32_t findNextIndex() {
  uint32_t maxIdx = 0;
  File root = STORAGE.open("/");
  if (!root) return 1;
  File f = root.openNextFile();
  while (f) {
    String n = f.name();
    int a = n.indexOf("img_");
    if (a >= 0) {
      uint32_t v = (uint32_t)n.substring(a + 4).toInt();
      if (v > maxIdx) maxIdx = v;
    }
    f.close();
    f = root.openNextFile();
  }
  root.close();
  return maxIdx + 1;
}

void saveToSD(const uint8_t *buf, size_t len) {
  if (!sdOk) return;
  char path[32];
  snprintf(path, sizeof(path), "/img_%05lu.jpg", (unsigned long)nextIndex);
  File f = STORAGE.open(path, FILE_WRITE);
  if (!f) return;
  f.write(buf, len);
  f.close();
  nextIndex++;
}

// ------------------------------- camera -------------------------------
static void fillConfig(camera_config_t &config, bool jpeg, framesize_t rgbSize) {
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk  = XCLK_GPIO_NUM;
  config.pin_pclk  = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href  = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn  = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.grab_mode    = CAMERA_GRAB_LATEST;

  if (jpeg) {
    config.pixel_format = PIXFORMAT_JPEG;
    if (psramFound()) {
      config.frame_size   = FRAMESIZE_UXGA;   // 1600x1200: good detail for OCR
      config.jpeg_quality = 10;               // lower number = better quality
      config.fb_count     = 2;
      config.fb_location  = CAMERA_FB_IN_PSRAM;
    } else {
      config.frame_size   = FRAMESIZE_SVGA;   // 800x600 without PSRAM
      config.jpeg_quality = 12;
      config.fb_count     = 1;
      config.fb_location  = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Fallback for sensors that cannot output JPEG: raw picture, converted by the ESP32.
    config.pixel_format = PIXFORMAT_RGB565;
    config.frame_size   = rgbSize;            // raw pictures: size is fixed until the camera is restarted
    config.jpeg_quality = 12;
    config.fb_count     = 1;
    config.fb_location  = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  }
}

void applySensorSettings(bool announce) {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) return;
  if (announce) {
    Serial.printf("CAMERA_OK MODE=%s PID=0x%x\n", jpegMode ? "JPEG" : "RGB565", s->id.PID);
  }
  // If your picture is upside down or mirrored, uncomment what you need:
  // s->set_vflip(s, 1);
  // s->set_hmirror(s, 1);
}

bool initCamera() {
  camera_config_t config;
  memset(&config, 0, sizeof(config));
  fillConfig(config, true, PREVIEW_SIZE);

  esp_err_t err = esp_camera_init(&config);
  if (err == ESP_OK) {
    jpegMode = true;
  } else {
    Serial.printf("CAM_ERR 0x%x in JPEG mode, trying RGB565 mode...\n", err);
    esp_camera_deinit();
    memset(&config, 0, sizeof(config));
    fillConfig(config, false, PREVIEW_SIZE);
    err = esp_camera_init(&config);
    if (err != ESP_OK) {
      Serial.printf("CAM_ERR 0x%x\n", err);
      return false;
    }
    jpegMode = false;
  }
  currentSize = config.frame_size;
  // JPEG sensors: biggest size for photos. Raw sensors (e.g. GC2145): 1280x960 if PSRAM allows.
  photoSize = jpegMode ? config.frame_size : (psramFound() ? FRAMESIZE_SXGA : FRAMESIZE_VGA);

  applySensorSettings(true);
  return true;
}

// Raw (RGB565) sensors cannot change picture size on the fly: restart the camera instead.
bool restartRawCamera(framesize_t fs) {
  esp_camera_deinit();
  camera_config_t config;
  memset(&config, 0, sizeof(config));
  fillConfig(config, false, fs);
  if (esp_camera_init(&config) != ESP_OK) return false;
  currentSize = fs;
  applySensorSettings(false);
  for (int i = 0; i < 3; i++) {             // let auto-exposure settle
    camera_fb_t *tmp = esp_camera_fb_get();
    if (tmp) esp_camera_fb_return(tmp);
  }
  return true;
}

// Preview uses a small picture (fast, fits the screen); photos use the big size.
bool setFrameSize(framesize_t fs, int flushFrames) {
  if (fs == currentSize) return true;
  if (!jpegMode) return restartRawCamera(fs);
  sensor_t *s = esp_camera_sensor_get();
  if (!s) return false;
  s->set_framesize(s, fs);
  currentSize = fs;
  for (int i = 0; i < flushFrames; i++) {   // throw away frames taken during the switch
    camera_fb_t *tmp = esp_camera_fb_get();
    if (tmp) esp_camera_fb_return(tmp);
  }
  return true;
}

// ---- RGB565 -> JPEG with our own growing buffer ----
// The built-in frame2jpg() stops at 128 KB (131072 bytes), which cuts big, detailed photos in half.
// frame2jpg_cb() hands us the JPEG piece by piece, so we can store as much as needed.
struct JpgBuf {
  uint8_t *buf;
  size_t   len;
  size_t   cap;
  bool     failed;
};

static size_t jpgCollect(void *arg, size_t index, const void *data, size_t len) {
  JpgBuf *b = (JpgBuf *)arg;
  if (index == 0) b->len = 0;
  if (b->failed) return len;
  if (b->len + len > b->cap) {
    size_t newCap = b->cap ? b->cap : 65536;
    while (newCap < b->len + len) newCap *= 2;
    uint8_t *nb = (uint8_t *)heap_caps_realloc(b->buf, newCap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!nb) nb = (uint8_t *)realloc(b->buf, newCap);
    if (!nb) { b->failed = true; return len; }
    b->buf = nb;
    b->cap = newCap;
  }
  memcpy(b->buf + b->len, data, len);
  b->len += len;
  return len;
}

bool rgbToJpeg(camera_fb_t *fb, uint8_t quality, uint8_t **outBuf, size_t *outLen) {
  JpgBuf b = {nullptr, 0, 0, false};
  bool ok = frame2jpg_cb(fb, quality, jpgCollect, &b);
  if (!ok || b.failed || b.len == 0) {
    if (b.buf) free(b.buf);
    return false;
  }
  *outBuf = b.buf;
  *outLen = b.len;
  return true;
}

void captureAndSend(bool preview) {
  bool sizeOk = preview ? setFrameSize(PREVIEW_SIZE, 1) : setFrameSize(photoSize, 3);
  if (!sizeOk) {
    Serial.write((const uint8_t *)"ERR!", 4);
    return;
  }
  if (!preview) {
    // Throw away stale frames so the photo is fresh and well exposed.
    for (int i = 0; i < 2; i++) {
      camera_fb_t *tmp = esp_camera_fb_get();
      if (tmp) esp_camera_fb_return(tmp);
    }
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.write((const uint8_t *)"ERR!", 4);
    return;
  }

  const uint8_t *out = fb->buf;
  size_t outLen = fb->len;
  uint8_t *converted = nullptr;
  const int frameW = fb->width;
  const int frameH = fb->height;

  if (!jpegMode) {
    size_t cl = 0;
    if (!rgbToJpeg(fb, preview ? 50 : 85, &converted, &cl)) {
      esp_camera_fb_return(fb);
      Serial.write((const uint8_t *)"ERR!", 4);
      return;
    }
    out = converted;
    outLen = cl;
    // Give the camera buffer back BEFORE sending. If the camera has no free buffer, its driver
    // prints overflow warnings into the USB stream and corrupts the picture on its way to the laptop.
    esp_camera_fb_return(fb);
    fb = nullptr;
  }

  if (!preview) saveToSD(out, outLen);

  // 1) send to the laptop
  uint32_t len = (uint32_t)outLen;
  Serial.write((const uint8_t *)"IMG!", 4);
  Serial.write((const uint8_t *)&len, 4);

  size_t sent = 0;
  while (sent < len) {
    size_t n = len - sent;
    if (n > 1024) n = 1024;
    Serial.write(out + sent, n);
    sent += n;
  }
  Serial.flush();

  // 2) show on the TFT screen (photos are scaled down to fit)
  if (preview) {
    tft.drawJpg(out, outLen, 0, 0, tft.width(), tft.height());
  } else {
    tft.drawJpg(out, outLen, 0, 0, tft.width(), tft.height(), 0, 0,
                (float)tft.width() / frameW, (float)tft.height() / frameH);
  }

  if (converted) free(converted);
  if (fb) esp_camera_fb_return(fb);
}

// -------------------------------- main --------------------------------
void setup() {
  esp_log_level_set("*", ESP_LOG_NONE);   // keep camera/IDF log text out of the USB data stream
  Serial.setRxBufferSize(1024);     // room for text pages sent while the screen is busy
  Serial.begin(115200);
  Serial.setTimeout(500);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);

  initTFT();
  showTextPage("ESP32 OCR\nStarting camera...");

  if (!initCamera()) {
    showTextPage("CAMERA ERROR\nCheck the board choice, the PSRAM setting and that the camera ribbon cable is fully inserted.");
    while (true) {                 // camera failed: keep reporting the problem
      Serial.println("CAMERA_INIT_FAILED (check board choice and PSRAM setting)");
      delay(2000);
    }
  }

  sdOk = initSD();
  if (sdOk) nextIndex = findNextIndex();

  showTextPage("ESP32 OCR\nReady.\nNow start the Python program on the laptop.");
  Serial.printf("READY SD=%d\n", sdOk ? 1 : 0);
}

void loop() {
  if (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'C') {
      captureAndSend(false);
    } else if (c == 'V') {
      captureAndSend(true);
    } else if (c == 'T') {
      handleText();
    } else if (c == 'P') {
      Serial.printf("PONG SD=%d\n", sdOk ? 1 : 0);
    }
  }
  delay(2);
}
