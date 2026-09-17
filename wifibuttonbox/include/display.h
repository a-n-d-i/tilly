#ifdef TILLY_DISPLAY
#include "SPI.h"
#include "Adafruit_GFX.h"+
#include "Adafruit_ILI9341.h"

#define TFT_CS    22
#define TFT_DC    21
#define TFT_MOSI  23  // SDA
#define TFT_CLK   19
#define TFT_RST   18
#define TFT_MISO  25   // SDO or -1 if not used

struct Field {
  String text;      
  uint16_t bgColor; 
};

struct LowerDisplay {
  String curText;
  String desText;
  uint16_t bgColor;
};

// ----- SCREEN GEOMETRY (LANDSCAPE 320x240) -----
const int W = 320;
const int H = 240;

const int upperH = H / 2;     // 120px
const int rowH   = upperH / 2;
const int colW   = W / 4;     // 4 columns across 320px

// ----- DATA -----
Field fields[8];
Field prevFields[8];

LowerDisplay lower;

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

void drawSingleField(int index) {
  int row = index / 4;
  int col = index % 4;

  int x = col * colW;
  int y = row * rowH;

  tft.fillRect(x, y, colW, rowH, fields[index].bgColor);

  tft.setTextColor(ILI9341_BLACK, fields[index].bgColor);
  tft.setTextSize(2);
  tft.setCursor(x + 4, y + 4);
  tft.print(fields[index].text);

  tft.drawRect(x, y, colW, rowH, ILI9341_BLACK);

  prevFields[index] = fields[index];
}

void drawLowerDisplay() {
  int y0 = upperH;
  int h  = H - upperH;

  tft.fillRect(0, y0, W, h, lower.bgColor);

  tft.setTextColor(ILI9341_BLACK, lower.bgColor);
  tft.setTextSize(4);

  tft.setCursor(10, y0 + 10);
  tft.print("CUR:");

  tft.setCursor(10, y0 + 70);
  tft.print("DES:");
}



// this just overrides the three digit number for degrees at the specified y position
void updateDeg(int y, int hdg, int &hdg_old) {
  tft.setTextSize(4);
  // erase old number from Display
  tft.setCursor(115, upperH + y);
  tft.setTextColor(lower.bgColor, lower.bgColor);
  tft.print(String(hdg_old));

  // write new number
  tft.setTextColor(ILI9341_BLACK, lower.bgColor);
  lower.desText = String(hdg);
  tft.setCursor(115, upperH + y);
  tft.print(String(hdg));
  hdg_old = hdg;
}


void drawFullscreen() {
  for (int i = 0; i < 8; i++) {
    if (fields[i].text    != prevFields[i].text ||
        fields[i].bgColor != prevFields[i].bgColor)
    {
      drawSingleField(i);
    }
  }
  drawLowerDisplay();
}

void initTillyDisplay() {
  tft.begin();
  tft.setRotation(1);   // <<< LANDSCAPE MODE
  tft.fillScreen(ILI9341_BLACK);
  for (int i = 0; i < 8; i++) {
    fields[i].text = "";
    fields[i].bgColor = ILI9341_GREEN;
  }

  lower.bgColor = ILI9341_RED;

  drawFullscreen();
}

#endif
