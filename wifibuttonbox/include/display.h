#ifdef TILLY_DISPLAY
#include "ST7305_U8g2.h"
#include "board_pins.h"
#include "applog.h"

// ----- SCREEN GEOMETRY (landscape, after U8G2_R1 rotation) -----
static const int DISP_W = RLCD_WIDTH;   // 400
static const int DISP_H = RLCD_HEIGHT;  // 300

struct TillyDisplayState {
  int sats = 0;
  bool ekfOk = false;
  bool magOk = false;
  bool armed = false;
  bool autoMode = false;
  float speedKn = 0.0f;
  int curHeading = 0;
  int desHeading = 0;
  float pidFF = 0, pidP = 0, pidI = 0, pidD = 0, pidSRate = 0;
};

static ST7305_U8g2 tillyLcd(RLCD_SCK_PIN, RLCD_MOSI_PIN, RLCD_DC_PIN, RLCD_CS_PIN, RLCD_RST_PIN);
static U8G2 *tillyU8g2 = nullptr;

static void drawCheckbox(int x, int y, bool checked, const char *label) {
  tillyU8g2->setFont(u8g2_font_7x14B_tr);
  tillyU8g2->drawStr(x, y + 12, label);
  int bx = x + tillyU8g2->getStrWidth(label) + 6;
  tillyU8g2->drawFrame(bx, y, 14, 14);
  if (checked) {
    tillyU8g2->drawLine(bx + 2, y + 7, bx + 5, y + 11);
    tillyU8g2->drawLine(bx + 5, y + 11, bx + 12, y + 2);
  }
}

static void drawPidBox(int x, int y, int w, int h, const char *label, float value) {
  tillyU8g2->drawFrame(x, y, w, h);
  tillyU8g2->setFont(u8g2_font_6x12_tr);
  tillyU8g2->drawStr(x + 4, y + 13, label);
  char buf[12];
  snprintf(buf, sizeof(buf), "%.3g", value);
  tillyU8g2->setFont(u8g2_font_helvB10_tr);
  tillyU8g2->drawStr(x + 4, y + h - 8, buf);
}

void updateTillyDisplay(const TillyDisplayState &s) {
  tillyU8g2->clearBuffer();

  // ----- Header: sats / EKF / MAG / armed -----
  char satsBuf[16];
  snprintf(satsBuf, sizeof(satsBuf), "SATS %d", s.sats);
  tillyU8g2->setFont(u8g2_font_7x14B_tr);
  tillyU8g2->drawStr(6, 16, satsBuf);

  drawCheckbox(120, 4, s.ekfOk, "EKF");
  drawCheckbox(220, 4, s.magOk, "MAG");
  drawCheckbox(320, 4, s.armed, "ARM");

  tillyU8g2->drawHLine(0, 36, DISP_W);

  // ----- Mode + speed -----
  tillyU8g2->setFont(u8g2_font_helvB24_tr);
  tillyU8g2->drawStr(10, 82, s.autoMode ? "AUTO" : "STANDBY");

  char speedBuf[16];
  snprintf(speedBuf, sizeof(speedBuf), "%.1f kn", s.speedKn);
  int speedW = tillyU8g2->getStrWidth(speedBuf);
  tillyU8g2->drawStr(DISP_W - speedW - 10, 82, speedBuf);

  tillyU8g2->drawHLine(0, 98, DISP_W);

  // ----- Cur / des heading -----
  tillyU8g2->setFont(u8g2_font_7x14B_tr);
  tillyU8g2->drawStr(10, 116, "CUR");
  tillyU8g2->drawStr(210, 116, "DES");

  char curBuf[8], desBuf[8];
  snprintf(curBuf, sizeof(curBuf), "%d", s.curHeading);
  snprintf(desBuf, sizeof(desBuf), "%d", s.desHeading);
  tillyU8g2->setFont(u8g2_font_helvB24_tr);
  tillyU8g2->drawStr(10, 156, curBuf);
  tillyU8g2->drawStr(210, 156, desBuf);

  tillyU8g2->drawHLine(0, 168, DISP_W);

  // ----- PID terms -----
  const int boxY = 174, boxH = DISP_H - boxY - 6, gap = 5;
  const int boxW = (DISP_W - 6 * gap) / 5;
  struct { const char *label; float value; } terms[] = {
    {"FF", s.pidFF}, {"P", s.pidP}, {"I", s.pidI}, {"D", s.pidD}, {"SRATE", s.pidSRate},
  };
  for (int i = 0; i < 5; i++) {
    int x = gap + i * (boxW + gap);
    drawPidBox(x, boxY, boxW, boxH, terms[i].label, terms[i].value);
  }

  tillyU8g2->sendBuffer();
}

// Full-screen scrollback view of appLog()'s ring buffer. Toggled by holding
// buttons 5+6 together (see handleButtons() in wifibuttonbox.ino).
void updateTillyLogScreen() {
  tillyU8g2->clearBuffer();

  tillyU8g2->setFont(u8g2_font_6x12_tr);
  tillyU8g2->drawStr(4, 11, "LOG - hold 5+6 to exit");
  tillyU8g2->drawHLine(0, 15, DISP_W);

  const int lineH = 12;
  const int top = 15 + lineH;
  const int maxLines = (DISP_H - top) / lineH;

  size_t total = appLogCount();
  size_t shown = (total < (size_t)maxLines) ? total : (size_t)maxLines;
  size_t startIdx = total - shown;  // oldest of the visible window

  int y = top;
  for (size_t i = 0; i < shown; i++) {
    tillyU8g2->drawStr(2, y, appLogLine(startIdx + i));
    y += lineH;
  }

  tillyU8g2->sendBuffer();
}

void initTillyDisplay() {
  tillyLcd.begin(0, U8G2_R1);
  tillyU8g2 = tillyLcd.getU8g2();

  TillyDisplayState blank;
  updateTillyDisplay(blank);
}

#endif
