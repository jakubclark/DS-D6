#define NUMFLAKES 10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT 16
#define LOGO_WIDTH 16

static const unsigned char PROGMEM logo_bmp[] = {
  B00000000, B11000000, B00000001, B11000000, B00000001, B11000000, B00000011,
  B11100000, B11110011, B11100000, B11111110, B11111000, B01111110, B11111111,
  B00110011, B10011111, B00011111, B11111100, B00001101, B01110000, B00011011,
  B10100000, B00111111, B11100000, B00111111, B11110000, B01111100, B11110000,
  B01110000, B01110000, B00000000, B00110000
};

void displaySnowflakesStatic() {
  display.clearDisplay();
  display.drawBitmap((display.width() - LOGO_WIDTH) / 2,
                     (display.height() - LOGO_HEIGHT) / 2, logo_bmp, LOGO_WIDTH,
                     LOGO_HEIGHT, 1);
  display.display();
}

#define XPOS 0
#define YPOS 1
#define DELTAY 2

int8_t icons[NUMFLAKES][3];

void initSnowflakes() {
  int8_t f;

  for (f = 0; f < NUMFLAKES; f++) {
    icons[f][XPOS] = random(1 - LOGO_WIDTH, display.width());
    icons[f][YPOS] = -LOGO_HEIGHT;
    icons[f][DELTAY] = random(1, 6);
  }
}

void displaySnowflakeAnimation() {
  int8_t f;
  display.clearDisplay();

  for (f = 0; f < NUMFLAKES; f++) {
    display.drawBitmap(icons[f][XPOS], icons[f][YPOS], logo_bmp, LOGO_WIDTH,
                        LOGO_HEIGHT, WHITE);
  }

  display.display();
  delay(10);
  for (f = 0; f < NUMFLAKES; f++) {
    icons[f][YPOS] += icons[f][DELTAY];
    if (icons[f][YPOS] >= display.height()) {
      icons[f][XPOS] = random(1 - LOGO_WIDTH, display.width());
      icons[f][YPOS] = -LOGO_HEIGHT;
      icons[f][DELTAY] = random(1, 3);
    }
  }
}
