#include <SPI.h>
#include <stdint.h>
#include <BLEPeripheral.h>

#include <nrf_nvic.h> //interrupt controller stuff
#include <nrf_sdm.h>
#include <nrf_soc.h>
#include <WInterrupts.h>
#include "Adafruit_GFX.h"
#include "SSD1306.h"
#include <TimeLib.h>
#include <nrf.h>
#include "count_steps.h"
#include "count_steps.c"
#include "i2csoft.h"

#define wdt_reset() NRF_WDT->RR[0] = WDT_RR_RR_Reload
#define wdt_enable(timeout)                                                    \
  NRF_WDT->CONFIG = NRF_WDT->CONFIG =                                          \
      (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) |                         \
      (WDT_CONFIG_SLEEP_Pause << WDT_CONFIG_SLEEP_Pos);                        \
  NRF_WDT->CRV = (32768 * timeout) / 1000;                                     \
  NRF_WDT->RREN |= WDT_RREN_RR0_Msk;                                           \
  NRF_WDT->TASKS_START = 1

#define motorOn() digitalWrite(25, HIGH);
#define motorOff() digitalWrite(25, LOW);

#define TIME_INDEX 0
#define MAC_INDEX 1
#define SANDBOX_INDEX 2
#define SNOWFLAKE_INDEX 3
#define SNOWFLAKE_ANIMATION_INDEX 4
#define BOOTLOADER_INDEX 5
/* The watch has moved */
#define ACCELEROMETER_INDEX 77
/* The watch is charging */
#define CHARGING_INDEX 88
/* The watch received an "AT+PUSH=..." BLE command */
#define BLE_RECEIVED_INDEX 99

Adafruit_SSD1306 display(128, 32, &SPI, 28, 4, 29);

/* The display will power down after this many milliseconds */
#define sleepDelay 7000
#define BUTTON_PIN 30
/* How often the screen is updated (for most commands) */
#define refreshRate 250

/* The current index in the menu */
int menuIndex;
volatile bool buttonPressed = false;
long startbutton;
unsigned long sleepTime;
/* The time when the display was last refreshed */
unsigned long displayRefreshTime;
volatile bool sleeping = false;
int timezone, steps, steps1;
String serialNr = "235246472";
String versionNr = "110.200.051";
String btversionNr = "100.016.051";
String msgText;
boolean gotoBootloader = false;

String bleSymbol = "";
int contrast;

BLEPeripheral blePeripheral = BLEPeripheral();
BLEService batteryLevelService = BLEService("190A");
BLECharacteristic TXchar = BLECharacteristic("0002", BLENotify, 20);
BLECharacteristic RXchar =
    BLECharacteristic("0001", BLEWriteWithoutResponse, 20);

BLEService batteryLevelService1 = BLEService("190B");
BLECharacteristic TXchar1 = BLECharacteristic("0004", BLENotify, 20);
BLECharacteristic RXchar1 =
    BLECharacteristic("0003", BLEWriteWithoutResponse, 20);

#define N_GRAINS 250 // Number of grains of sand
#define WIDTH 127    // Display width in pixels
#define HEIGHT 32    // Display height in pixels
#define MAX_FPS 150  // Maximum redraw rate, frames/second

// The 'sand' grains exist in an integer coordinate space that's 256X
// the scale of the pixel grid, allowing them to move and interact at
// less than whole-pixel increments.
#define MAX_X (WIDTH * 256 - 1)  // Maximum X coordinate in grain space
#define MAX_Y (HEIGHT * 256 - 1) // Maximum Y coordinate
struct Grain {
  int16_t x, y;   // Position
  int16_t vx, vy; // Velocity
  uint16_t pos;
} grain[N_GRAINS];

uint32_t prevTime = 0; // Used for frames-per-second throttle
uint16_t backbuffer = 0, img[WIDTH * HEIGHT]; // Internal 'map' of pixels

#ifdef __cplusplus
extern "C" {
#endif

#define LF_FREQUENCY 32768UL
#define SECONDS(x) ((uint32_t)((LF_FREQUENCY * x) + 0.5))
#define wakeUpSeconds 120
void RTC2_IRQHandler(void) {
  volatile uint32_t dummy;
  if (NRF_RTC2->EVENTS_COMPARE[0] == 1) {
    NRF_RTC2->EVENTS_COMPARE[0] = 0;
    NRF_RTC2->CC[0] = NRF_RTC2->COUNTER + SECONDS(wakeUpSeconds);
    dummy = NRF_RTC2->EVENTS_COMPARE[0];
    dummy;
    // powerUp();
  }
}

void initRTC2() {

  NVIC_SetPriority(RTC2_IRQn, 15);
  NVIC_ClearPendingIRQ(RTC2_IRQn);
  NVIC_EnableIRQ(RTC2_IRQn);

  NRF_RTC2->PRESCALER = 0;
  NRF_RTC2->CC[0] = SECONDS(wakeUpSeconds);
  NRF_RTC2->INTENSET = RTC_EVTENSET_COMPARE0_Enabled
                       << RTC_EVTENSET_COMPARE0_Pos;
  NRF_RTC2->EVTENSET = RTC_INTENSET_COMPARE0_Enabled
                       << RTC_INTENSET_COMPARE0_Pos;
  NRF_RTC2->TASKS_START = 1;
}
#ifdef __cplusplus
}
#endif

void powerUp() {
  if (sleeping) {
    sleeping = false;
    display.begin(SSD1306_SWITCHCAPVCC);
    display.clearDisplay();
    display.display();
    delay(5);
  }
  sleepTime = millis();
}

void powerDown() {
  if (!sleeping) {
    sleeping = true;

    digitalWrite(28, LOW);
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
    digitalWrite(29, LOW);
    digitalWrite(4, LOW);
    NRF_SAADC->ENABLE = 0; // disable ADC
    NRF_PWM0->ENABLE = 0;  // disable all pwm instance
    NRF_PWM1->ENABLE = 0;
    NRF_PWM2->ENABLE = 0;
  }
}

void charge() {
  if (sleeping)
    menuIndex = CHARGING_INDEX;
  powerUp();
}

/* Check if no action was taken in the last `delay` milliseconds */
bool shouldSleep(int delay) { return (millis() - sleepTime) > delay; }

/* Check if the display should be refreshed, based on the time of the last
   refresh */
bool shouldRefresh() { return (millis() - displayRefreshTime) > refreshRate; }

/* Callback for when the button is pressed */
void buttonCallback() {
  if (!sleeping)
    buttonPressed = true;
  else
    menuIndex = 0;
  powerUp();
}

void acclHandler() {
  ReadRegister(0x17);
  if (sleeping) {
    menuIndex = ACCELEROMETER_INDEX;
    powerUp();
  }
}

/* Handler for when BLE has connected */
void blePeripheralConnectHandler(BLECentral &central) {
  menuIndex = 0;
  powerUp();
  bleSymbol = "B";
}

/* Handler for when BLE has disconnected */
void blePeripheralDisconnectHandler(BLECentral &central) {
  menuIndex = 0;
  powerUp();
  bleSymbol = " ";
}

String answer = "";
String tempCmd = "";
int tempLen = 0, tempLen1;
boolean syn;

void characteristicWritten(BLECentral &central,
                           BLECharacteristic &characteristic) {
  char remoteCharArray[21];
  tempLen1 = characteristic.valueLength();
  tempLen = tempLen + tempLen1;
  memset(remoteCharArray, 0, sizeof(remoteCharArray));
  memcpy(remoteCharArray, characteristic.value(), tempLen1);
  tempCmd = tempCmd + remoteCharArray;
  if (tempCmd[tempLen - 2] == '\r' && tempCmd[tempLen - 1] == '\n') {
    answer = tempCmd.substring(0, tempLen - 2);
    tempCmd = "";
    tempLen = 0;
    handleBLECommand(answer);
  }
}

/* Handle the received BLE Command, stored in `cmd` */
void handleBLECommand(String cmd) {
  if (cmd == "AT+BOND") {
    sendBLEcmd("AT+BOND:OK");
  } else if (cmd == "AT+ACT") {
    sendBLEcmd("AT+ACT:0");
  } else if (cmd.substring(0, 7) == "BT+UPGB") {
    gotoBootloader = true;
  } else if (cmd.substring(0, 8) == "BT+RESET") {
    if (gotoBootloader)
      NRF_POWER->GPREGRET = 0x01;
    sd_nvic_SystemReset();
  } else if (cmd.substring(0, 7) == "AT+RUN=") {
    sendBLEcmd("AT+RUN:" + cmd.substring(7));
  } else if (cmd.substring(0, 8) == "AT+USER=") {
    sendBLEcmd("AT+USER:" + cmd.substring(8));
  } else if (cmd.substring(0, 7) == "AT+REC=") {
    sendBLEcmd("AT+REC:" + cmd.substring(7));
  } else if (cmd.substring(0, 8) == "AT+PUSH=") {
    sendBLEcmd("AT+PUSH:OK");
    menuIndex = BLE_RECEIVED_INDEX;
    powerUp();
    handlePush(cmd.substring(8));
  } else if (cmd.substring(0, 9) == "AT+MOTOR=") {
    sendBLEcmd("AT+MOTOR:" + cmd.substring(9));
  } else if (cmd.substring(0, 8) == "AT+DEST=") {
    sendBLEcmd("AT+DEST:" + cmd.substring(8));
  } else if (cmd.substring(0, 9) == "AT+ALARM=") {
    sendBLEcmd("AT+ALARM:" + cmd.substring(9));
  } else if (cmd.substring(0, 13) == "AT+HRMONITOR=") {
    sendBLEcmd("AT+HRMONITOR:" + cmd.substring(13));
  } else if (cmd.substring(0, 13) == "AT+FINDPHONE=") {
    sendBLEcmd("AT+FINDPHONE:" + cmd.substring(13));
  } else if (cmd.substring(0, 13) == "AT+ANTI_LOST=") {
    sendBLEcmd("AT+ANTI_LOST:" + cmd.substring(13));
  } else if (cmd.substring(0, 9) == "AT+UNITS=") {
    sendBLEcmd("AT+UNITS:" + cmd.substring(9));
  } else if (cmd.substring(0, 11) == "AT+HANDSUP=") {
    sendBLEcmd("AT+HANDSUP:" + cmd.substring(11));
  } else if (cmd.substring(0, 7) == "AT+SIT=") {
    sendBLEcmd("AT+SIT:" + cmd.substring(7));
  } else if (cmd.substring(0, 7) == "AT+LAN=") {
    sendBLEcmd("AT+LAN:ERR");
  } else if (cmd.substring(0, 14) == "AT+TIMEFORMAT=") {
    sendBLEcmd("AT+TIMEFORMAT:" + cmd.substring(14));
  } else if (cmd == "AT+BATT") {
    sendBLEcmd("AT+BATT:" + String(getBatteryLevel()));
  } else if (cmd == "BT+VER") {
    sendBLEcmd("BT+VER:" + btversionNr);
  } else if (cmd == "AT+VER") {
    sendBLEcmd("AT+VER:" + versionNr);
  } else if (cmd == "AT+SN") {
    sendBLEcmd("AT+SN:" + serialNr);
  } else if (cmd.substring(0, 10) == "AT+DISMOD=") {
    sendBLEcmd("AT+DISMOD:" + cmd.substring(10));
  } else if (cmd.substring(0, 7) == "AT+LAN=") {
    sendBLEcmd("AT+LAN:ERR");
  } else if (cmd.substring(0, 10) == "AT+MOTOR=1") {
    sendBLEcmd("AT+MOTOR:1" + cmd.substring(10));
    motorOn();
    delay(300);
    motorOff();
  } else if (cmd.substring(0, 12) == "AT+CONTRAST=") {
    contrast = cmd.substring(12).toInt();
  } else if (cmd.substring(0, 6) == "AT+DT=") {
    SetDateTimeString(cmd.substring(6));
    sendBLEcmd("AT+DT:" + GetDateTimeString());
  } else if (cmd.substring(0, 5) == "AT+DT") {
    sendBLEcmd("AT+DT:" + GetDateTimeString());
  } else if (cmd.substring(0, 12) == "AT+TIMEZONE=") {
    timezone = cmd.substring(12).toInt();
    sendBLEcmd("AT+TIMEZONE:" + String(timezone));
  } else if (cmd.substring(0, 11) == "AT+TIMEZONE") {
    sendBLEcmd("AT+TIMEZONE:" + String(timezone));
  } else if (cmd == "AT+STEPSTORE") {
    sendBLEcmd("AT+STEPSTORE:OK");
  } else if (cmd == "AT+TOPACE=1") {
    sendBLEcmd("AT+TOPACE:OK");
    sendBLEcmd("NT+TOPACE:" + String(steps));
  } else if (cmd == "AT+TOPACE=0") {
    sendBLEcmd("AT+TOPACE:" + String(steps));
  } else if (cmd == "AT+DATA=0") {
    sendBLEcmd("AT+DATA:0,0,0,0");
  } else if (cmd.substring(0, 8) == "AT+PACE=") {
    steps1 = cmd.substring(8).toInt();
    sendBLEcmd("AT+PACE:" + String(steps1));
  } else if (cmd == "AT+PACE") {
    sendBLEcmd("AT+PACE:" + String(steps1));
  } else if (cmd == "AT+DATA=1") {
    sendBLEcmd("AT+DATA:0,0,0,0");
  } else if (cmd.substring(0, 7) == "AT+SYN=") {
    if (cmd.substring(7) == "1") {
      sendBLEcmd("AT+SYN:1");
      syn = true;
    } else {
      sendBLEcmd("AT+SYN:0");
      syn = false;
    }
  }
}

void sendBLEcmd(String cmd) {
  cmd = cmd + "\r\n";
  while (cmd.length() > 0) {
    const char *TempSendCmd;
    String Tempcmd = cmd.substring(0, 20);
    TempSendCmd = &Tempcmd[0];
    TXchar.setValue(TempSendCmd);
    TXchar1.setValue(TempSendCmd);
    cmd = cmd.substring(20);
  }
}

String GetDateTimeString() {
  String datetime = String(year());
  if (month() < 10)
    datetime += "0";
  datetime += String(month());
  if (day() < 10)
    datetime += "0";
  datetime += String(day());
  if (hour() < 10)
    datetime += "0";
  datetime += String(hour());
  if (minute() < 10)
    datetime += "0";
  datetime += String(minute());
  return datetime;
}

void SetDateTimeString(String datetime) {
  int year = datetime.substring(0, 4).toInt();
  int month = datetime.substring(4, 6).toInt();
  int day = datetime.substring(6, 8).toInt();
  int hr = datetime.substring(8, 10).toInt();
  int min = datetime.substring(10, 12).toInt();
  int sec = datetime.substring(12, 14).toInt();
  setTime(hr, min, sec, day, month, year);
}

void handlePush(String pushedMessage) {
  int firstComma = pushedMessage.indexOf(',');
  int secondComma = pushedMessage.indexOf(',', firstComma + 1);
  msgText = pushedMessage.substring(firstComma + 1, secondComma);
}

int getBatteryLevel() { return map(analogRead(3), 500, 715, 0, 100); }

void setup() {
  pinMode(BUTTON_PIN, INPUT);
  pinMode(3, INPUT);
  if (digitalRead(BUTTON_PIN) == LOW) {
    NRF_POWER->GPREGRET = 0x01;
    sd_nvic_SystemReset();
  }
  pinMode(2, INPUT);
  pinMode(25, OUTPUT);
  digitalWrite(25, HIGH);
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  pinMode(15, INPUT);
  wdt_enable(5000);
  blePeripheral.setLocalName("DS-D6-K");
  blePeripheral.setAdvertisingInterval(555);
  blePeripheral.setAppearance(0x0000);
  blePeripheral.setConnectable(true);
  blePeripheral.setDeviceName("ATCDSD6-K");
  blePeripheral.setAdvertisedServiceUuid(batteryLevelService.uuid());
  blePeripheral.addAttribute(batteryLevelService);
  blePeripheral.addAttribute(TXchar);
  blePeripheral.addAttribute(RXchar);
  RXchar.setEventHandler(BLEWritten, characteristicWritten);
  blePeripheral.setAdvertisedServiceUuid(batteryLevelService1.uuid());
  blePeripheral.addAttribute(batteryLevelService1);
  blePeripheral.addAttribute(TXchar1);
  blePeripheral.addAttribute(RXchar1);
  RXchar1.setEventHandler(BLEWritten, characteristicWritten);
  blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  blePeripheral.setEventHandler(BLEDisconnected,
                                blePeripheralDisconnectHandler);
  blePeripheral.begin();
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonCallback, FALLING);
  attachInterrupt(digitalPinToInterrupt(15), acclHandler, RISING);
  NRF_GPIO->PIN_CNF[15] &= ~((uint32_t)GPIO_PIN_CNF_SENSE_Msk);
  NRF_GPIO->PIN_CNF[15] |=
      ((uint32_t)GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);
  attachInterrupt(digitalPinToInterrupt(2), charge, RISING);
  NRF_GPIO->PIN_CNF[2] &= ~((uint32_t)GPIO_PIN_CNF_SENSE_Msk);
  NRF_GPIO->PIN_CNF[2] |=
      ((uint32_t)GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);
  display.begin(SSD1306_SWITCHCAPVCC);
  delay(100);
  display.clearDisplay();
  display.display();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(10, 0);
  display.println("D6 Watch");
  display.display();
  motorOff();
  sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
  initRTC2();
  initi2c();
  initkx023();

  uint8_t i, j, bytes;
  memset(img, 0, sizeof(img));     // Clear the img[] array
  for (i = 0; i < N_GRAINS; i++) { // For each sand grain...
    do {
      grain[i].x = random(WIDTH * 256);  // Assign random position within
      grain[i].y = random(HEIGHT * 256); // the 'grain' coordinate space
      // Check if corresponding pixel position is already occupied...
      for (j = 0; (j < i) && (((grain[i].x / 256) != (grain[j].x / 256)) ||
                              ((grain[i].y / 256) != (grain[j].y / 256)));
           j++)
        ;
    } while (j < i); // Keep retrying until a clear spot is found
    img[(grain[i].y / 256) * WIDTH + (grain[i].x / 256)] = 255; // Mark it
    grain[i].pos = (grain[i].y / 256) * WIDTH + (grain[i].x / 256);
    grain[i].vx = grain[i].vy = 0; // Initial velocity is zero
  }

  initSnowflakes();
}

/* Do some action, based on the current value of `menuIndex` */
void doAction() {
  /* Always display the Sandbox Game */
  if (menuIndex == SANDBOX_INDEX)
    displaySandboxGame();
  else if (menuIndex == SNOWFLAKE_ANIMATION_INDEX)
    displaySnowflakeAnimation();
  else if (shouldRefresh()) {
    displayRefreshTime = millis();
    switch (menuIndex) {
    case TIME_INDEX:
    case ACCELEROMETER_INDEX:
      displayTime();
      break;
    case MAC_INDEX:
      displayMAC();
      break;
    case SNOWFLAKE_INDEX:
      displaySnowflakesStatic();
      break;
    case BOOTLOADER_INDEX:
      displayBootloader();
      break;
    case CHARGING_INDEX:
      displayCharging();
      break;
    case BLE_RECEIVED_INDEX:
      displayBleMenu();
      break;
    }
  } else {
    delay(refreshRate);
  }
}

/**
   Handler for when the button is pressed.
   This is different than `buttonCallback()`, which handles the IRQ.
 * */
void handleButtonPress() {
  buttonPressed = false;
  switch (menuIndex) {
  case BOOTLOADER_INDEX:
    startbutton = millis();
    while (!digitalRead(BUTTON_PIN)) {
    }
    if (millis() - startbutton > 1000) {
      delay(100);
      int err_code = sd_power_gpregret_set(0x01);
      sd_nvic_SystemReset();
      while (1) {
      };
    } else {
      menuIndex = 0;
    }
    break;
  case ACCELEROMETER_INDEX:
  case CHARGING_INDEX:
  case BLE_RECEIVED_INDEX:
    menuIndex = 0;
    motorOff();
    break;
  default:
    menuIndex += 1;
  }
}

/* If enough time has passed, power off the display, based on the value of
   `menuIndex`. */
void powerDownIfNeeded() {
  switch (menuIndex) {
  case TIME_INDEX:
  case MAC_INDEX:
    if (shouldSleep(sleepDelay)) {
      powerDown();
    }
    break;
  case SNOWFLAKE_INDEX:
  case SNOWFLAKE_ANIMATION_INDEX:
    if (shouldSleep(10000)) {
      powerDown();
    }
    break;
  case SANDBOX_INDEX:
    if (shouldSleep(20000)) {
      powerDown();
    }
    break;
  case ACCELEROMETER_INDEX:
  case CHARGING_INDEX:
    if (shouldSleep(5000))
      powerDown();
    break;
  case BLE_RECEIVED_INDEX:
    if (shouldSleep(500)) {
      motorOff();
    }
    if (shouldSleep(5000)) {
      powerDown();
    }
    break;
  default:
    if (shouldSleep(5000)) {
      motorOff();
      powerDown();
    }
  }
}

/**
   Generally, the loop contains 3 parts:
   1. Take an action based on the current state.
   2. Increment the menu index, if the button is pressed
   3. Turn off the display, if enough time has passed since the last
  event/action If the device is in `sleeping` mode (i.e.: no action has been
  taken), wait for the next IRQ.
 **/
void loop() {
  blePeripheral.poll();
  wdt_reset();

  if (sleeping) {
    sd_app_evt_wait();
    sd_nvic_ClearPendingIRQ(SD_EVT_IRQn);
  } else {
    doAction();
    if (buttonPressed) {
      handleButtonPress();
    }
    powerDownIfNeeded();
  }
}

/* Shows the current time and date, along with battery level */
void displayTime() {
  display.setRotation(3);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(bleSymbol);
  display.setCursor(0, 11);
  display.print(getBatteryLevel());
  display.println("%");

  display.setTextSize(2);
  display.setCursor(4, 30);
  if (hour() < 10)
    display.print("0");
  display.println(hour());
  display.setCursor(4, 50);
  if (minute() < 10)
    display.print("0");
  display.println(minute());
  display.setCursor(4, 70);
  if (second() < 10)
    display.print("0");
  display.println(second());

  display.setCursor(0, 111);
  display.setTextSize(1);
  if (day() < 10)
    display.print("0");
  display.print(day());
  display.print(".");
  if (month() < 10)
    display.print("0");
  display.println(month());
  display.print(".");
  display.println(year());

  display.display();
}

/* Display the MAC Address of the device */
void displayMAC() {
  display.setRotation(0);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("MAC:");
  char tmp[16];
  sprintf(tmp, "%04X", NRF_FICR->DEVICEADDR[1] & 0xffff);
  String MyID = tmp;
  sprintf(tmp, "%08X", NRF_FICR->DEVICEADDR[0]);
  MyID += tmp;
  display.println(MyID);
  display.display();
}

/* Display from which the bootloader can be entered. */
void displayBootloader() {
  display.setRotation(0);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Hold for the Bootloader!");
  display.display();
}

/* Shown when the device is charging */
void displayCharging() {
  display.setRotation(3);
  display.clearDisplay();
  display.setCursor(0, 30);
  display.println("Charging");
  display.print(getBatteryLevel());
  display.display();
}

/* Shown when the "AT+PUSH=" Command is sent. */
void displayBleMenu() {
  motorOn();
  display.setRotation(0);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(msgText);
  display.display();
}
