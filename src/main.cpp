//オーディオモード、残り時間のバーは出来てる
//キーパッドの色エラー出るのでそのまま、秒数のコントロールも出来ていない…


#include <Dynamixel.h>
#define DYNAMIXEL_SERIAL Serial2 // change as you want

#include <SPIFFS.h>
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

String numberBuffer1 = "test";

BluetoothSerial SerialBT;



static const int Empty = 0;
static const int Start = 1;
static const int ArrowPressUp = 2;
static const int ArrowPressDown = 3;
static const int ArrowPressLeft = 4;
static const int ArrowPressRight = 5;
static const int ArrowPressCenter = 6;
static const int ArrowOut = 7;
static const int ButtonPressA = 8;
static const int ButtonPressB = 9;
static const int ButtonPressC = 10;
static const int ButtonPressD = 11;
static const int ButtonPressE = 12;
static const int ButtonPressF = 13;
static const int ButtonPressG = 14;
static const int ButtonPressH = 15;
static const int ButtonPressI = 16;
static const int ButtonPressJ = 17;
static const int ButtonPressK = 18;
static const int ButtonPressL = 19;
static const int ButtonPressM = 20;
static const int ButtonPressN = 21;
static const int ButtonPressO = 22;
static const int ButtonPressP = 23;
static const int ButtonPressQ = 24;
static const int ButtonPressR = 25;
static const int ButtonPressS = 26;
static const int ButtonPressT = 27;
static const int ButtonPressU = 28;
static const int ButtonPressV = 29;
static const int ButtonPressW = 30;
static const int ButtonPressX = 31;
static const int ButtonPressY = 32;
static const int ButtonPressZ = 33;
static const int ButtonOut = 34;
struct Action {
    int id;
    String command;
};
Action ACTIONS[] = {
    { Empty, "__" },
    { Start, "START" },
    { ArrowPressUp, "ARROW_PRESS_UP" },
    { ArrowPressDown, "ARROW_PRESS_DOWN" },
    { ArrowPressLeft, "ARROW_PRESS_LEFT" },
    { ArrowPressRight, "ARROW_PRESS_RIGHT" },
    { ArrowPressCenter, "ARROW_PRESS_CENTER" },
    { ArrowOut, "ARROW_OUT" },
    { ButtonPressA, "BUTTON_PRESS_A" },
    { ButtonPressB, "BUTTON_PRESS_B" },
    { ButtonPressC, "BUTTON_PRESS_C" },
    { ButtonPressD, "BUTTON_PRESS_D" },
    { ButtonPressE, "BUTTON_PRESS_E" },
    { ButtonPressF, "BUTTON_PRESS_F" },
    { ButtonPressG, "BUTTON_PRESS_G" },
    { ButtonPressH, "BUTTON_PRESS_H" },
    { ButtonPressI, "BUTTON_PRESS_I" },
    { ButtonPressJ, "BUTTON_PRESS_J" },
    { ButtonPressK, "BUTTON_PRESS_K" },
    { ButtonPressL, "BUTTON_PRESS_L" },
    { ButtonPressM, "BUTTON_PRESS_M" },
    { ButtonPressN, "BUTTON_PRESS_N" },
    { ButtonPressO, "BUTTON_PRESS_O" },
    { ButtonPressP, "BUTTON_PRESS_P" },
    { ButtonPressQ, "BUTTON_PRESS_Q" },
    { ButtonPressR, "BUTTON_PRESS_R" },
    { ButtonPressS, "BUTTON_PRESS_S" },
    { ButtonPressT, "BUTTON_PRESS_T" },
    { ButtonPressU, "BUTTON_PRESS_U" },
    { ButtonPressV, "BUTTON_PRESS_V" },
    { ButtonPressW, "BUTTON_PRESS_W" },
    { ButtonPressX, "BUTTON_PRESS_X" },
    { ButtonPressY, "BUTTON_PRESS_Y" },
    { ButtonPressZ, "BUTTON_PRESS_Z" },
    { ButtonOut, "BUTTON_OUT" },
};

// ---- S/W Version ------------------
#define VERSION_NUMBER  "ver. 0.12.0"
// -----------------------------------

String bluetoothDeviceName = "YushunArm";


bool onlyLeftArm = false; //左手のみを使用するかどうか
bool mainloop = false;
bool stopRecording = false; //STOPボタンが押されたかどうか
bool stopPlaying = false; //STOPボタンが押されたかどうか
int rightHandException = 2400;   //手が全開で脱力する時の閾値
int leftHandException = 800;   //手が全開で脱力する時の閾値


#define RXD2 16
#define TXD2 17

const int rs485TX = 22;  //485--白
const int rs485RX = 12;  //485--黄

const int swAudio = 21;

const uint8_t PIN_RTS = 11;

const int pin1 = 25;
const int pin2 = 26;
const int pin3 = 27;

Dynamixel dxl(RXD2, TXD2);

const int defaultRecordNumber = 600;  //デフォルトの録画する数
int number = 17; //←ここの数字はID+1とする 呼び出す数
int values[defaultRecordNumber * 18 ]; //←ここの数字はID＋１とする
char buffer[24]; // 数値の一時的な保持のためのバッファ

int startRecordTime = 0;
int endRecordTime = 0;
int totalRecordTime = 0;
int playMotionTime = 200;


const int timer = defaultRecordNumber;
int TIMERLENGTH = 0;


int sw00State = 1, sw02State = 1, sw03State = 1, sw04State = 1, sw05State = 1, swAudioState = 1;
int sw01State = 1;
int targetPos01, targetPos02, targetPos03, targetPos04, targetPos05, targetPos06, targetPos07, targetPos08 = 0;
int targetPos11, targetPos12, targetPos13, targetPos14, targetPos15, targetPos16, targetPos17, targetPos18 = 0;


int s1, s2, s3, s4, s5, s6, s7, s8 = 0;
int s11, s12, s13, s14, s15, s16, s17, s18 = 0;
char receivedChar = 0;

const int s1diference = 500; //id:01モータの目標値と実測値の差分 この差分を超えるとモーションを終了する
const int s2diference = 400; //id:02モータの目標値と実測値の差分 この差分を超えるとモーションを終了する
const int s4diference = 800; //id:04モータの目標値と実測値の差分 この差分を超えるとモーションを終了する

const int s11diference = 500; //id:11モータの目標値と実測値の差分 この差分を超えるとモーションを終了する
const int s12diference = 400; //id:12モータの目標値と実測値の差分 この差分を超えるとモーションを終了する
const int s14diference = 800; //id:14モータの目標値と実測値の差分 この差分を超えるとモーションを終了する

int rightArmFlag = 0; //フリーの時、落下防止に動きを遅くするフラグ
int leftArmFlag = 0; //フリーの時、落下防止に動きを遅くするフラグ

int ran1, ran2, ran3, ran4 = 0;
int ran11, ran12, ran13, ran14 = 0;
int s08 = 0; //腕の角度
int s018 = 0; //腕の角度

int mode = 10; //1-9:モーション登録, 11-19:モーション再生
int audioMode = 0; //0:初期値,

int moveMode = 0; //0:初期値, 1:前進, 2:後進, 3:右回転, 4:左回転

int O_time = 0;
const int O_t = 20;

int past_mode = 0;

int serialnumberP = 700;
int serialnumberI = 2;
int serialnumberD = 1;

const int profileVelocity = 150;

File file;

// グローバル変数としてdata配列を定義
const int readInterval = 100; // 読み取り間隔(ms)
const int totalReads = 100; // 10秒間に100回の読み取り
int data[totalReads][8]; // [読み取り回数][サーボモータの数]
int dataIndex = 0; // 現在の読み取りインデックス

const uint8_t TARGET_ID1 = 1;
const uint8_t TARGET_ID2 = 2;
const uint8_t TARGET_ID3 = 3;
const uint8_t TARGET_ID4 = 4;
const uint8_t TARGET_ID5 = 5;
const uint8_t TARGET_ID6 = 6;
const uint8_t TARGET_ID7 = 7;
const uint8_t TARGET_ID8 = 8;

const uint8_t TARGET_ID11 = 11;
const uint8_t TARGET_ID12 = 12;
const uint8_t TARGET_ID13 = 13;
const uint8_t TARGET_ID14 = 14;
const uint8_t TARGET_ID15 = 15;
const uint8_t TARGET_ID16 = 16;
const uint8_t TARGET_ID17 = 17;
const uint8_t TARGET_ID18 = 18;





int dxl_goal_position1[2];
int dxl_goal_position2[2];
bool dir = true;
int t = 0;

#include "FS.h"
#include <SPI.h>
#include <TFT_eSPI.h>      // Hardware-specific library
TFT_eSPI tft = TFT_eSPI(); // Invoke custom library
#define CALIBRATION_FILE "/TouchCalData1"
#define REPEAT_CAL false
// Keypad start position, key sizes and spacing
#define KEY_X 40 // Centre of key
#define KEY_Y 90
#define KEY_W 74 // Width and height
#define KEY_H 45
#define KEY_SPACING_X 6 // X and Y gap
#define KEY_SPACING_Y 5
#define KEY_TEXTSIZE 0   // Font size multiplier

// Using two fonts since numbers are nice when bold
#define LABEL1_FONT &FreeSansOblique12pt7b // Key label font 1
#define LABEL2_FONT &FreeSansBold12pt7b    // Key label font 2

// Numeric display box size and location
#define DISP_X 1
#define DISP_Y 10
#define DISP_W 238
#define DISP_H 50
#define DISP_TSIZE 3
#define DISP_TCOLOR TFT_CYAN

#define UNDERDISP_X 1
#define UNDERDISP_Y 269
#define UNDERDISP_W 238
#define UNDERDISP_H 50
#define UNDERDISP_TSIZE 1
#define UNDERDISP_TCOLOR TFT_CYAN

#define TIMERDISP_X 1
#define TIMERDISP_Y 218
#define TIMERDISP_W 238
#define TIMERDISP_H 50
#define TIMERDISP_TSIZE 3
#define TIMERDISP_TCOLOR TFT_CYAN

// Number length, buffer for storing it and character index
#define NUM_LEN 12
char numberBuffer[NUM_LEN + 1] = "";
uint8_t numberIndex = 0;

// We have a status line for messages
#define STATUS_X 120 // Centred on this
#define STATUS_Y 65


// Create 9 keys for the keypad
char keyLabel[9][5] = {"RUN", "MODE", "REC", "1", "2", "3", "4", "5", "6"};
uint16_t keyColor[9] = {
  TFT_RED, TFT_DARKGREY, TFT_DARKGREEN,
  TFT_DARKGREY, TFT_DARKGREY, TFT_DARKGREY,
  TFT_DARKGREY, TFT_DARKGREY, TFT_DARKGREY,
};

// Invoke the TFT_eSPI button class and create all the button objects
TFT_eSPI_Button key[9];






void DISPprint() {
  // Update the number display field

  tft.setTextDatum(TL_DATUM);        // Use top left corner as text coord datum
  tft.setFreeFont(&FreeSans18pt7b);  // Choose a nice font that fits box
  tft.setTextColor(DISP_TCOLOR);     // Set the font colour
  int xwidth = tft.drawString(numberBuffer1, DISP_X + 4, DISP_Y + 12);

  // Now cover up the rest of the line up by drawing a black rectangle.  No flicker this way
  // but it will not work with italic or oblique fonts due to character overlap.
  tft.fillRect(DISP_X + 4 + xwidth, DISP_Y + 1, DISP_W - xwidth - 5, DISP_H - 2, TFT_BLACK);
  delay(10); // UI debouncing
}


void DISPreset() {

  tft.setTextDatum(TL_DATUM);        // Use top left corner as text coord datum
  tft.setFreeFont(&FreeSans18pt7b);  // Choose a nice font that fits box
  tft.setTextColor(DISP_TCOLOR);     // Set the font colour
  numberBuffer1 = "";
  int xwidth = tft.drawString(numberBuffer1, DISP_X + 4, DISP_Y + 12);
  tft.fillRect(DISP_X + 4 + xwidth, DISP_Y + 1, DISP_W - xwidth - 5, DISP_H - 2, TFT_BLACK);
  delay(10); // UI debouncing
}

void DISPwrite(String A) {

  Serial.print("DISP= ");
  Serial.println(A);
  tft.setTextDatum(TL_DATUM);        // Use top left corner as text coord datum
  tft.setFreeFont(&FreeSans18pt7b);  // Choose a nice font that fits box
  tft.setTextColor(DISP_TCOLOR);     // Set the font colour
  numberBuffer1 = "";
  int xwidth = tft.drawString(numberBuffer1, DISP_X + 4, DISP_Y + 12);
  tft.fillRect(DISP_X + 4 + xwidth, DISP_Y + 1, DISP_W - xwidth - 5, DISP_H - 2, TFT_BLACK);
  delay(10);
  xwidth = tft.drawString(A, DISP_X + 4, DISP_Y + 12);
  tft.fillRect(DISP_X + 4 + xwidth, DISP_Y + 1, DISP_W - xwidth - 5, DISP_H - 2, TFT_BLACK);
  delay(10);
}

void UNDERDISPprint() {
  // Update the number display field

  tft.setTextDatum(TL_DATUM);        // Use top left corner as text coord datum
  tft.setFreeFont(&FreeSans18pt7b);  // Choose a nice font that fits box
  tft.setTextColor(UNDERDISP_TCOLOR);     // Set the font colour
  int xwidth = tft.drawString(numberBuffer1, UNDERDISP_X + 4, UNDERDISP_Y + 12);

  // Now cover up the rest of the line up by drawing a black rectangle.  No flicker this way
  // but it will not work with italic or oblique fonts due to character overlap.
  tft.fillRect(UNDERDISP_X + 4 + xwidth, UNDERDISP_Y + 1, UNDERDISP_W - xwidth - 5, UNDERDISP_H - 2, TFT_BLACK);
  delay(10); // UI debouncing
}


void UNDERDISPreset() {

  tft.setTextDatum(TL_DATUM);        // Use top left corner as text coord datum
  tft.setFreeFont(&FreeSans18pt7b);  // Choose a nice font that fits box
  tft.setTextColor(UNDERDISP_TCOLOR);     // Set the font colour
  numberBuffer1 = "";
  int xwidth = tft.drawString(numberBuffer1, UNDERDISP_X + 4, UNDERDISP_Y + 12);
  tft.fillRect(UNDERDISP_X + 4 + xwidth, UNDERDISP_Y + 1, UNDERDISP_W - xwidth - 5, UNDERDISP_H - 2, TFT_BLACK);
  delay(10); // UI debouncing
}

void UNDERDISPwrite(String A) {

  tft.setTextDatum(TL_DATUM);        // Use top left corner as text coord datum
  tft.setFreeFont(&FreeSans18pt7b);  // Choose a nice font that fits box
  tft.setTextColor(UNDERDISP_TCOLOR);     // Set the font colour
  numberBuffer1 = "";
  int xwidth = tft.drawString(numberBuffer1, UNDERDISP_X + 4, UNDERDISP_Y + 12);
  tft.fillRect(UNDERDISP_X + 4 + xwidth, UNDERDISP_Y + 1, UNDERDISP_W - xwidth - 5, UNDERDISP_H - 2, TFT_BLACK);
  delay(10);
  xwidth = tft.drawString(A, UNDERDISP_X + 4, UNDERDISP_Y + 12);
  tft.fillRect(UNDERDISP_X + 4 + xwidth, UNDERDISP_Y + 1, UNDERDISP_W - xwidth - 5, UNDERDISP_H - 2, TFT_BLACK);
  delay(10);
}

void TIMERDISPprint() {
  // Update the number display field

  tft.setTextDatum(TL_DATUM);        // Use top left corner as text coord datum
  tft.setFreeFont(&FreeSans18pt7b);  // Choose a nice font that fits box
  tft.setTextColor(UNDERDISP_TCOLOR);     // Set the font colour
  int xwidth = tft.drawString(numberBuffer1, UNDERDISP_X + 4, UNDERDISP_Y + 12);

  // Now cover up the rest of the line up by drawing a black rectangle.  No flicker this way
  // but it will not work with italic or oblique fonts due to character overlap.
  tft.fillRect(UNDERDISP_X + 4 + xwidth, UNDERDISP_Y + 1, UNDERDISP_W - xwidth - 5, UNDERDISP_H - 2, TFT_BLACK);
  delay(10); // UI debouncing
}


void TIMERDISPreset() {

  tft.fillRect(TIMERDISP_X, TIMERDISP_Y, TIMERDISP_W, TIMERDISP_H, TFT_BLACK);  // Draw number display area and frame
  tft.drawRect(TIMERDISP_X, TIMERDISP_Y, TIMERDISP_W, TIMERDISP_H, TFT_WHITE);
}

void TIMERDISPwrite() {

  float progress = (float)TIMERLENGTH /   (float)timer ;
  int fillWidth = progress * TIMERDISP_W;

  tft.fillRect(TIMERDISP_X + 1 , TIMERDISP_Y + 1, fillWidth, TIMERDISP_H - 2, TFT_GREEN);
}

void touch_calibrate() {

  uint16_t calData[5];
  uint8_t calDataOK = 0;

  // check file system exists
  if (!SPIFFS.begin()) {
    Serial.println("formatting file system");
    SPIFFS.format();
    SPIFFS.begin();
  }

  // check if calibration file exists and size is correct
  if (SPIFFS.exists(CALIBRATION_FILE)) {
    if (REPEAT_CAL)
    {
      // Delete if we want to re-calibrate
      SPIFFS.remove(CALIBRATION_FILE);
    }
    else
    {
      File f = SPIFFS.open(CALIBRATION_FILE, "r");
      if (f) {
        if (f.readBytes((char *)calData, 14) == 14)
          calDataOK = 1;
        f.close();
      }
    }
  }

  if (calDataOK && !REPEAT_CAL) {
    // calibration data valid
    tft.setTouch(calData);
  } else {
    // data not valid so recalibrate
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(20, 0);
    tft.setTextFont(2);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    tft.println("Touch corners as indicated");

    tft.setTextFont(1);
    tft.println();

    if (REPEAT_CAL) {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.println("Set REPEAT_CAL to false to stop this running again!");
    }

    tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.println("Calibration complete!");

    // store data
    File f = SPIFFS.open(CALIBRATION_FILE, "w");
    if (f) {
      f.write((const unsigned char *)calData, 14);
      f.close();
    }
  }
}

//------------------------------------------------------------------------------------------

// Print something in the mini status bar
void status(const char *msg) {

  tft.setTextPadding(240);
  //tft.setCursor(STATUS_X, STATUS_Y);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft.setTextFont(0);
  tft.setTextDatum(TC_DATUM);
  tft.setTextSize(1);
  tft.drawString(msg, STATUS_X, STATUS_Y);
}
//------------------------------------------------------------------------------------------

void drawKeypad() {
  // Draw the keys

  for (uint8_t row = 0; row < 3; row++) {
    for (uint8_t col = 0; col < 3; col++) {
      uint8_t b = col + row * 3;

      if (b < 3) tft.setFreeFont(LABEL1_FONT);
      else tft.setFreeFont(LABEL2_FONT);

      // モードが1から9（録画モード）なら「MODE」を「STOP」に変更
      if (b == 1 && ((0 < mode  && mode < 10) || (10 < mode  && mode < 20))) {
        keyLabel[b][0] = 'S';
        keyLabel[b][1] = 'T';
        keyLabel[b][2] = 'O';
        keyLabel[b][3] = 'P';
        keyLabel[b][4] = '\0';
      } else if (b == 1 && (mode < 1 || mode == 10 || 20 < mode)) {
        keyLabel[b][0] = 'M';
        keyLabel[b][1] = 'O';
        keyLabel[b][2] = 'D';
        keyLabel[b][3] = 'E';
        keyLabel[b][4] = '\0';
      }

      
      if(audioMode == 0) {
          keyColor[0] = TFT_DARKGREEN;
          keyColor[1] = TFT_DARKGREY;
          keyColor[2] = TFT_RED;
        
        if (mode == 0){
          keyColor[3] = TFT_RED;
          keyColor[4] = TFT_RED;
          keyColor[5] = TFT_RED;
          keyColor[6] = TFT_RED;
          keyColor[7] = TFT_RED;
          keyColor[8] = TFT_RED;
        }

        if(mode == 10){
          keyColor[3] = TFT_DARKGREEN;
          keyColor[4] = TFT_DARKGREEN;
          keyColor[5] = TFT_DARKGREEN;
          keyColor[6] = TFT_DARKGREEN;
          keyColor[7] = TFT_DARKGREEN;
          keyColor[8] = TFT_DARKGREEN;
        }
        

        if (0 < mode && mode < 10) {
          for (int i = 3; i < 9; i++) {
            if (i == mode + 2){
              keyColor[i] = TFT_RED;
            } else {
              keyColor[i] = TFT_DARKGREY;
            }
          }
        }

        if (10 < mode && mode < 20) {
          for (int i = 3; i < 9; i++) {
            if (i == mode - 8){
              keyColor[i] = TFT_DARKGREEN;
            } else {
              keyColor[i] = TFT_DARKGREY;
            }
          }
        }

      }
      else {
        keyColor[0] = TFT_DARKGREY;
        keyColor[1] = TFT_BLUE;
        keyColor[2] = TFT_DARKGREY;

        if(audioMode == 1){
          keyColor[3] = TFT_BLUE;
          keyColor[4] = TFT_BLUE;
          keyColor[5] = TFT_BLUE;
          keyColor[6] = TFT_BLUE;
          keyColor[7] = TFT_BLUE;
          keyColor[8] = TFT_BLUE;        
        }

        if (1 < audioMode && audioMode < 8) {
          for (int i = 3; i < 9; i++) {
            if (i == audioMode + 1){
              keyColor[i] = TFT_BLUE;
            } else {
              keyColor[i] = TFT_DARKGREY;
            }
          }
        }
      }
      key[b].initButton(&tft, KEY_X + col * (KEY_W + KEY_SPACING_X),
                        KEY_Y + row * (KEY_H + KEY_SPACING_Y), // x, y, w, h, outline, fill, text
                        KEY_W, KEY_H, TFT_WHITE, keyColor[b], TFT_WHITE,
                        keyLabel[b], KEY_TEXTSIZE);
      key[b].drawButton();
    }
  }
}

void Pgain_on() {
  
  if (serialnumberI > 1) {
    dxl.positionPGain(TARGET_ID1, serialnumberP);
    dxl.positionPGain(TARGET_ID2, serialnumberP);
    dxl.positionPGain(TARGET_ID3, 700);
    dxl.positionPGain(TARGET_ID4, serialnumberP);
    dxl.positionPGain(TARGET_ID6, 500);
    dxl.positionPGain(TARGET_ID7, 500);
    dxl.positionPGain(TARGET_ID8, 1200);

    dxl.positionPGain(TARGET_ID11, serialnumberP);
    dxl.positionPGain(TARGET_ID12, serialnumberP);
    dxl.positionPGain(TARGET_ID13, 700);
    dxl.positionPGain(TARGET_ID14, serialnumberP);
    dxl.positionPGain(TARGET_ID16, 500);
    dxl.positionPGain(TARGET_ID17, 500);
    dxl.positionPGain(TARGET_ID18, 1200);
  }
}

void rightArmRange(int a1, int a2) {

  int r = 230;
  if (a1 < a2 + r && a2 - r < a1)rightArmFlag = rightArmFlag + 1;
}

void leftArmRange(int a1, int a2) {

  int r = 230;
  if (a1 < a2 + r && a2 - r < a1)leftArmFlag = leftArmFlag + 1;
}

void zero() { //フリーの時、落下防止に動きを遅くするフラグをONにする

  rightArmFlag = 1;
  leftArmFlag = 1;
  int de = 1; delay(de);
  targetPos01 = dxl.presentPosition(TARGET_ID1); delay(de);
  targetPos02 = dxl.presentPosition(TARGET_ID2); delay(de);
  targetPos04 = dxl.presentPosition(TARGET_ID4); delay(de);
  targetPos08 = dxl.presentPosition(TARGET_ID8); delay(de);

  targetPos11 = dxl.presentPosition(TARGET_ID11); delay(de);
  targetPos12 = dxl.presentPosition(TARGET_ID12); delay(de);
  targetPos14 = dxl.presentPosition(TARGET_ID14); delay(de);
  targetPos18 = dxl.presentPosition(TARGET_ID18);

  s08 = targetPos08;
  s018 = targetPos18;

  rightArmRange(targetPos01, ran1);
  rightArmRange(targetPos02, ran2);
  rightArmRange(targetPos04, ran4);

  leftArmRange(targetPos11, ran11);
  leftArmRange(targetPos12, ran12);
  leftArmRange(targetPos14, ran14);

  if (rightArmFlag == 4)rightArmFlag = 0;
  // Serial.print(" Rflag : ");
  // Serial.print(rightArmFlag);
  // Serial.print(":");
  if (leftArmFlag == 4)leftArmFlag = 0;
  // Serial.print(" Lflag : ");
  // Serial.print(leftArmFlag);
  // Serial.print(":");
}

void slow() { //条件がONの時、スローにする。ただし腕が全開の時は例外とする

  if (rightArmFlag > 0 && s08 > rightHandException) {
    dxl.positionPGain(TARGET_ID1, 1);
    dxl.positionPGain(TARGET_ID2, 1);
    dxl.positionPGain(TARGET_ID3, 1);
    dxl.positionPGain(TARGET_ID4, 1);
    dxl.positionPGain(TARGET_ID6, 1);
    dxl.positionPGain(TARGET_ID7, 1);
    dxl.positionPGain(TARGET_ID8, 1);

    int de = 1;

    dxl.torqueEnable(TARGET_ID1, false);
    dxl.torqueEnable(TARGET_ID2, false);
    dxl.torqueEnable(TARGET_ID3, false);
    dxl.torqueEnable(TARGET_ID4, false);
    dxl.torqueEnable(TARGET_ID6, false);

    delay(de);
    dxl.torqueEnable(TARGET_ID1, true);
    dxl.torqueEnable(TARGET_ID2, true);
    dxl.torqueEnable(TARGET_ID3, true);
    dxl.torqueEnable(TARGET_ID4, true);
    dxl.torqueEnable(TARGET_ID6, true);

    delay(12);

  } else {
    dxl.torqueEnable(TARGET_ID1, false);
    dxl.torqueEnable(TARGET_ID2, false);
    dxl.torqueEnable(TARGET_ID3, false);
    dxl.torqueEnable(TARGET_ID4, false);
    dxl.torqueEnable(TARGET_ID6, false);
  }

  if (leftArmFlag > 0 && s018 < leftHandException) {
    dxl.positionPGain(TARGET_ID11, 1);
    dxl.positionPGain(TARGET_ID12, 1);
    dxl.positionPGain(TARGET_ID13, 1);
    dxl.positionPGain(TARGET_ID14, 1);
    dxl.positionPGain(TARGET_ID16, 1);
    dxl.positionPGain(TARGET_ID17, 1);
    dxl.positionPGain(TARGET_ID18, 1);

    int de = 1;

    dxl.torqueEnable(TARGET_ID11, false);
    dxl.torqueEnable(TARGET_ID12, false);
    dxl.torqueEnable(TARGET_ID13, false);
    dxl.torqueEnable(TARGET_ID14, false);
    dxl.torqueEnable(TARGET_ID16, false);
    delay(de);

    dxl.torqueEnable(TARGET_ID11, true);
    dxl.torqueEnable(TARGET_ID12, true);
    dxl.torqueEnable(TARGET_ID13, true);
    dxl.torqueEnable(TARGET_ID14, true);
    dxl.torqueEnable(TARGET_ID16, true);
    delay(12);

  } else {

    dxl.torqueEnable(TARGET_ID11, false);
    dxl.torqueEnable(TARGET_ID12, false);
    dxl.torqueEnable(TARGET_ID13, false);
    dxl.torqueEnable(TARGET_ID14, false);
    dxl.torqueEnable(TARGET_ID16, false);
  }
}

void demo() {

  // Serial.print("mode= ");//デモの時は基本10になっている。
  // Serial.print(mode);
  int de = 5;
  delay(de);

  dxl.driveMode(TARGET_ID1, 0x04);
  dxl.driveMode(TARGET_ID2, 0x04);
  dxl.driveMode(TARGET_ID3, 0x04);
  dxl.driveMode(TARGET_ID4, 0x04);
  dxl.driveMode(TARGET_ID5, 0x04);
  dxl.driveMode(TARGET_ID6, 0x04);
  dxl.driveMode(TARGET_ID7, 0x04);
  dxl.driveMode(TARGET_ID8, 0x04);
  dxl.driveMode(TARGET_ID11, 0x04);
  dxl.driveMode(TARGET_ID12, 0x04);
  dxl.driveMode(TARGET_ID13, 0x04);
  dxl.driveMode(TARGET_ID14, 0x04);
  dxl.driveMode(TARGET_ID15, 0x04);
  dxl.driveMode(TARGET_ID16, 0x04);
  dxl.driveMode(TARGET_ID17, 0x04);
  dxl.driveMode(TARGET_ID18, 0x04);



  dxl.profileVelocity(TARGET_ID1, profileVelocity);
  dxl.profileVelocity(TARGET_ID2, profileVelocity);
  dxl.profileVelocity(TARGET_ID3, profileVelocity);
  dxl.profileVelocity(TARGET_ID4, profileVelocity);
  dxl.profileVelocity(TARGET_ID5, profileVelocity);
  dxl.profileVelocity(TARGET_ID6, profileVelocity);
  dxl.profileVelocity(TARGET_ID7, profileVelocity);
  dxl.profileVelocity(TARGET_ID8, profileVelocity);
  dxl.profileVelocity(TARGET_ID11, profileVelocity);
  dxl.profileVelocity(TARGET_ID12, profileVelocity);
  dxl.profileVelocity(TARGET_ID13, profileVelocity);
  dxl.profileVelocity(TARGET_ID14, profileVelocity);
  dxl.profileVelocity(TARGET_ID15, profileVelocity);
  dxl.profileVelocity(TARGET_ID16, profileVelocity);
  dxl.profileVelocity(TARGET_ID17, profileVelocity);
  dxl.profileVelocity(TARGET_ID18, profileVelocity);

  targetPos01 = dxl.presentPosition(TARGET_ID1); delay(de);
  targetPos02 = dxl.presentPosition(TARGET_ID2); delay(de);
  targetPos03 = dxl.presentPosition(TARGET_ID3); delay(de);
  targetPos04 = dxl.presentPosition(TARGET_ID4); delay(de);
  targetPos05 = dxl.presentPosition(TARGET_ID5); delay(de);
  targetPos06 = dxl.presentPosition(TARGET_ID6); delay(de);
  targetPos07 = dxl.presentPosition(TARGET_ID7); delay(de);
  targetPos08 = dxl.presentPosition(TARGET_ID8); delay(de);

  targetPos11 = dxl.presentPosition(TARGET_ID11); delay(de);
  targetPos12 = dxl.presentPosition(TARGET_ID12); delay(de);
  targetPos13 = dxl.presentPosition(TARGET_ID13); delay(de);
  targetPos14 = dxl.presentPosition(TARGET_ID14); delay(de);
  targetPos15 = dxl.presentPosition(TARGET_ID15); delay(de);
  targetPos16 = dxl.presentPosition(TARGET_ID16); delay(de);
  targetPos17 = dxl.presentPosition(TARGET_ID17); delay(de);
  targetPos18 = dxl.presentPosition(TARGET_ID18);

  // Serial.print(" demo = ");
  Serial.print(targetPos01); Serial.print(", ");
  Serial.print(targetPos02); Serial.print(", ");
  Serial.print(targetPos03); Serial.print(", ");
  Serial.print(targetPos04); Serial.print(", ");
  Serial.print(targetPos05); Serial.print(", ");
  Serial.print(targetPos06); Serial.print(", ");
  Serial.print(targetPos07); Serial.print(", ");
  Serial.print(targetPos08); Serial.print(", ");

  Serial.print(targetPos11); Serial.print(", ");
  Serial.print(targetPos12); Serial.print(", ");
  Serial.print(targetPos13); Serial.print(", ");
  Serial.print(targetPos14); Serial.print(", ");
  Serial.print(targetPos15); Serial.print(", ");
  Serial.print(targetPos16); Serial.print(", ");
  Serial.print(targetPos17); Serial.print(", ");
  Serial.println(targetPos18);
}


void recordMotion() {
  // ファイルの書き込み

  DISPreset();
  numberBuffer1 = "Writer";
  DISPprint();

  TIMERDISPreset();

  if (mode < 10) {//writerはmodeが1～9
    drawKeypad();
    if (mode == 1) {
      // ファイルの作成とデータの書き込み
      file = SPIFFS.open("/test1.txt", FILE_WRITE);
      if (!file) {
        Serial.println("ファイルの作成に失敗しました");
        return;
      }
    }
    if (mode == 2) {
      // ファイルの作成とデータの書き込み
      file = SPIFFS.open("/test2.txt", FILE_WRITE);
      if (!file) {
        Serial.println("ファイルの作成に失敗しました");
        return;
      }
    }
    if (mode == 3) {
      // ファイルの作成とデータの書き込み
      file = SPIFFS.open("/test3.txt", FILE_WRITE);
      if (!file) {
        Serial.println("ファイルの作成に失敗しました");
        return;
      }
    }
    if (mode == 4) {
      // ファイルの作成とデータの書き込み
      file = SPIFFS.open("/test4.txt", FILE_WRITE);
      if (!file) {
        Serial.println("ファイルの作成に失敗しました");
        return;
      }
    }
    if (mode == 5) {
      // ファイルの作成とデータの書き込み
      file = SPIFFS.open("/test5.txt", FILE_WRITE);
      if (!file) {
        Serial.println("ファイルの作成に失敗しました");
        return;
      }
    }

    //***********************************************

    //    //レコード開始の合図をつけるならここ

    //全脱力
    dxl.torqueEnable(TARGET_ID1, false);
    dxl.torqueEnable(TARGET_ID2, false);
    dxl.torqueEnable(TARGET_ID3, false);
    dxl.torqueEnable(TARGET_ID4, false);
    dxl.torqueEnable(TARGET_ID5, false);
    dxl.torqueEnable(TARGET_ID6, false);
    dxl.torqueEnable(TARGET_ID7, false);
    dxl.torqueEnable(TARGET_ID8, false);

    dxl.torqueEnable(TARGET_ID11, false);
    dxl.torqueEnable(TARGET_ID12, false);
    dxl.torqueEnable(TARGET_ID13, false);
    dxl.torqueEnable(TARGET_ID14, false);
    dxl.torqueEnable(TARGET_ID15, false);
    dxl.torqueEnable(TARGET_ID16, false);
    dxl.torqueEnable(TARGET_ID17, false);
    dxl.torqueEnable(TARGET_ID18, false);

    startRecordTime = millis();

    for (int i = 0; i < defaultRecordNumber; i++) {
      // STOPボタンが押されたかどうかをチェック
      uint16_t t_x = 0, t_y = 0; // To store the touch coordinates
      bool pressed = tft.getTouch(&t_x, &t_y);
      if (pressed && key[1].contains(t_x, t_y)) {
        stopRecording = true;
      }


      DISPwrite(String(i)+"/"+String(defaultRecordNumber));
      
      UNDERDISPwrite(String(mode)+", "+String(targetPos01)+", "+String(targetPos02)+", "+String(targetPos03)+", "+String(targetPos04)+", "+String(targetPos11)+", "+String(targetPos12)+", "+String(targetPos13)+", "+String(targetPos14));

      TIMERLENGTH = i;
      TIMERDISPwrite();
    

      int de = 3; delay(de);
      targetPos01 = dxl.presentPosition(TARGET_ID1); delay(de);
      targetPos02 = dxl.presentPosition(TARGET_ID2); delay(de);
      targetPos03 = dxl.presentPosition(TARGET_ID3); delay(de);
      targetPos04 = dxl.presentPosition(TARGET_ID4); delay(de);
      targetPos05 = dxl.presentPosition(TARGET_ID5); delay(de);
      targetPos06 = dxl.presentPosition(TARGET_ID6); delay(de);
      targetPos07 = dxl.presentPosition(TARGET_ID7); delay(de);
      targetPos08 = dxl.presentPosition(TARGET_ID8); delay(de);
      
      targetPos11 = dxl.presentPosition(TARGET_ID11); delay(de);
      targetPos12 = dxl.presentPosition(TARGET_ID12); delay(de);
      targetPos13 = dxl.presentPosition(TARGET_ID13); delay(de);
      targetPos14 = dxl.presentPosition(TARGET_ID14); delay(de);
      targetPos15 = dxl.presentPosition(TARGET_ID15); delay(de);
      targetPos16 = dxl.presentPosition(TARGET_ID16); delay(de);
      targetPos17 = dxl.presentPosition(TARGET_ID17); delay(de);
      targetPos18 = dxl.presentPosition(TARGET_ID18);

      //Serial.print("timer = ");  Serial.println(t);

      Serial1.write(0);

      Serial.print("レコード中 = ");
      Serial.print(i); Serial.print(" : ");
      // Serial.print(targetPos01); Serial.print(", ");
      // Serial.print(targetPos02); Serial.print(", ");
      // Serial.print(targetPos03); Serial.print(", ");
      // Serial.print(targetPos04); Serial.print(", ");
      // Serial.print(targetPos05); Serial.print(", ");
      // Serial.print(targetPos06); Serial.print(", ");
      // Serial.print(targetPos07); Serial.print(", ");
      // Serial.print(targetPos08); Serial.print(", ");

      // Serial.print(targetPos11); Serial.print(", ");
      // Serial.print(targetPos12); Serial.print(", ");
      // Serial.print(targetPos13); Serial.print(", ");
      // Serial.print(targetPos14); Serial.print(", ");
      // Serial.print(targetPos15); Serial.print(", ");
      // Serial.print(targetPos16); Serial.print(", ");
      // Serial.print(targetPos17); Serial.print(", ");
      // Serial.print(targetPos18); Serial.print(", MODE=");  
      // Serial.print(mode);
      // Serial.print(", Audio="); Serial.print(swAudioState);

      //レコード書き出し

      file.print(targetPos01);
      Serial.print(targetPos01);
      file.print(",");
      Serial.print(",");
      file.print(targetPos02);
      Serial.print(targetPos02);
      file.print(",");
      Serial.print(",");
      file.print(targetPos03);
      Serial.print(targetPos03);
      file.print(",");
      Serial.print(",");
      file.print(targetPos04);
      Serial.print(targetPos04);
      file.print(",");
      Serial.print(",");
      file.print(targetPos05);
      Serial.print(targetPos05);
      file.print(",");
      Serial.print(",");
      file.print(targetPos06);
      Serial.print(targetPos06);
      file.print(",");
      Serial.print(",");
      file.print(targetPos07);
      Serial.print(targetPos07);
      file.print(",");
      Serial.print(",");
      file.print(targetPos08);
      Serial.print(targetPos08);
      file.print(",");
      Serial.print(",");

      file.print(targetPos11);
      Serial.print(targetPos11);
      file.print(",");
      Serial.print(",");
      file.print(targetPos12);
      Serial.print(targetPos12);
      file.print(",");
      Serial.print(",");
      file.print(targetPos13);
      Serial.print(targetPos13);
      file.print(",");
      Serial.print(",");
      file.print(targetPos14);
      Serial.print(targetPos14);
      file.print(",");
      Serial.print(",");
      file.print(targetPos15);
      Serial.print(targetPos15);
      file.print(",");
      Serial.print(",");
      file.print(targetPos16);
      Serial.print(targetPos16);
      file.print(",");
      Serial.print(",");
      file.print(targetPos17);
      Serial.print(targetPos17);
      file.print(",");
      Serial.print(",");
      file.print(targetPos18);
      Serial.print(targetPos18);
      file.print(",");
      Serial.print(",");
      // file.println(0);
      // Serial.println(0);
      if (stopRecording) { // STOPボタンが押されたかどうかをチェック
        for (int i = 3; i < 9; i++) {
          keyColor[i] = TFT_RED; // 数字ボタンを赤色に変更
        }
        endRecordTime = millis();
        totalRecordTime = endRecordTime - startRecordTime;
        
        file.print(totalRecordTime);
        Serial.print(totalRecordTime);
        file.print(",");
        Serial.print(",");
        file.println(0);
        Serial.println(0);
        Serial.println("");
        Serial.println("STOP");
        mode = 0;
        drawKeypad();
        break;
      } else {
        if (i > defaultRecordNumber - 2) {
          endRecordTime = millis();
          totalRecordTime = endRecordTime - startRecordTime;
          file.print(totalRecordTime);
          Serial.print(totalRecordTime);
          file.print(",");
          Serial.print(",");
        }
        file.println(0);
        Serial.println(0);
      }

    }

    if (!stopRecording) { // STOPボタンが押されていない場合
      DISPwrite("COMPLETE");
      Serial.println("COMPLETE");

    } else {
      DISPwrite("STOPPED");
      stopRecording = false;
    }
    
    file.close();
  }
}

void playMotion() {
  // ファイルの読み取り

  DISPwrite("reader");
  Serial.print("mode = " + mode);
  drawKeypad();

  TIMERDISPreset();

  if (mode == 1 || mode == 11) {
    //digitalWrite(led01, HIGH);
    file = SPIFFS.open("/test1.txt");
    if (!file) {
      Serial.println("ファイルの読み取りに失敗しました");
      return;
    }
  }
  if (mode == 2 || mode == 12) {
    //digitalWrite(led02, HIGH);
    file = SPIFFS.open("/test2.txt");
    if (!file) {
      Serial.println("ファイルの読み取りに失敗しました");
      return;
    }
  }
  if (mode == 3 || mode == 13) {
    //digitalWrite(led03, HIGH);
    file = SPIFFS.open("/test3.txt");
    if (!file) {
      Serial.println("ファイルの読み取りに失敗しました");
      return;
    }
  }
  if (mode == 4 || mode == 14) {
    //digitalWrite(led04, HIGH);
    file = SPIFFS.open("/test4.txt");
    if (!file) {
      Serial.println("ファイルの読み取りに失敗しました");
      return;
    }
  }
  if (mode == 5 || mode == 15) {
    //digitalWrite(led05, HIGH);
    file = SPIFFS.open("/test5.txt");
    if (!file) {
      Serial.println("ファイルの読み取りに失敗しました");
      return;
    }
  }

  // ファイルの内容を格納する配列
  int index = 0;
  while (file.available()) {
    int pos = 0;
    while (true) {
      char c = file.read();
      if (c == ',' || c == '\n' || c == -1) {
        buffer[pos] = '\0'; // 終端文字を追加
        values[index++] = atoi(buffer); // char配列を整数に変換して配列に格納
        pos = 0; // バッファの位置をリセット

        if (c == '\n' || c == -1) {
          break; // 行の終わりまたはファイルの終わり
        }
      } else {
        buffer[pos++] = c; // バッファに文字を追加
      }
    }
  }
  file.close();

  // 総記録時間を取得
  totalRecordTime = values[index - 2]; // 追加: 総記録時間は配列の最後から2番目に格納されています

  // playTimeを画面に表示
  DISPwrite("Play Time: " + String(totalRecordTime) + " ms");
  Serial.println("Play Time: " + String(totalRecordTime) + " ms");

  playMotionTime = totalRecordTime / 164;


  mode = 255;
  Serial.print(rightArmFlag);
  Serial.print(leftArmFlag);
  if (rightArmFlag == 0)mode = 256;
  if (leftArmFlag == 0)mode = 256;
  if (mode == 255 )slow();
  if (mode == 256) {

    dxl.torqueEnable(TARGET_ID1, true);
    dxl.torqueEnable(TARGET_ID2, true);
    dxl.torqueEnable(TARGET_ID3, true);
    dxl.torqueEnable(TARGET_ID4, true);
    dxl.torqueEnable(TARGET_ID5, true);
    dxl.torqueEnable(TARGET_ID6, true);
    dxl.torqueEnable(TARGET_ID7, true);
    dxl.torqueEnable(TARGET_ID8, true);

    dxl.torqueEnable(TARGET_ID11, true);
    dxl.torqueEnable(TARGET_ID12, true);
    dxl.torqueEnable(TARGET_ID13, true);
    dxl.torqueEnable(TARGET_ID14, true);
    dxl.torqueEnable(TARGET_ID15, true);
    dxl.torqueEnable(TARGET_ID16, true);
    dxl.torqueEnable(TARGET_ID17, true);
    dxl.torqueEnable(TARGET_ID18, true);

    // シリアルモニタにデータを表示&モータ実行
    for (int i = 0; i < playMotionTime; i++) {
      // STOPボタンが押されたかどうかをチェック
      uint16_t t_x = 0, t_y = 0; // To store the touch coordinates
      bool pressed = tft.getTouch(&t_x, &t_y);
      if (pressed && key[1].contains(t_x, t_y)) {
        stopPlaying = true;
      }
      if (stopPlaying) { // STOPボタンが押されたかどうかをチェック
        for (int i = 3; i < 9; i++) {
          keyColor[i] = TFT_DARKGREEN; // 数字ボタンを緑色に変更
        }
        drawKeypad();
        break;
      }


      int ss1 = values[i * number];
      int ss2 = values[i * number + 1];
      int ss3 = values[i * number + 2];
      int ss4 = values[i * number + 3];
      int ss5 = values[i * number + 4];
      int ss6 = values[i * number + 5];
      int ss7 = values[i * number + 6];
      int ss8 = values[i * number + 7];

      int ss11 = values[i * number + 8];
      int ss12 = values[i * number + 9];
      int ss13 = values[i * number + 10];
      int ss14 = values[i * number + 11];
      int ss15 = values[i * number + 12];
      int ss16 = values[i * number + 13];
      int ss17 = values[i * number + 14];
      int ss18 = values[i * number + 15];


      Serial.print("フレーム ");
      Serial.print(i + 1);
      Serial.println(": ");

      Serial.print("  mode= ");
      Serial.println(mode);

      DISPwrite(String(i)+"/"+String(playMotionTime));

      TIMERLENGTH = i;
      TIMERDISPwrite();

      //実測値計測
      int de = 1; delay(de);
      s1 = dxl.presentPosition(TARGET_ID1); delay(de);
      s2 = dxl.presentPosition(TARGET_ID2); delay(de);
      s3 = dxl.presentPosition(TARGET_ID3); delay(de);
      s4 = dxl.presentPosition(TARGET_ID4); delay(de);
      s5 = dxl.presentPosition(TARGET_ID5); delay(de);
      s6 = dxl.presentPosition(TARGET_ID6); delay(de);
      s7 = dxl.presentPosition(TARGET_ID7); delay(de);
      s8 = dxl.presentPosition(TARGET_ID8); delay(de);

      s11 = dxl.presentPosition(TARGET_ID11); delay(de);
      s12 = dxl.presentPosition(TARGET_ID12); delay(de);
      s13 = dxl.presentPosition(TARGET_ID13); delay(de);
      s14 = dxl.presentPosition(TARGET_ID14); delay(de);
      s15 = dxl.presentPosition(TARGET_ID15); delay(de);
      s16 = dxl.presentPosition(TARGET_ID16); delay(de);
      s17 = dxl.presentPosition(TARGET_ID17); delay(de);
      s18 = dxl.presentPosition(TARGET_ID18);


      Serial.print("差分 = ");
      Serial.print(ss1 - s1); Serial.print(", ");
      Serial.print(ss2 - s2); Serial.print(", ");
      Serial.print(ss3 - s3); Serial.print(", ");
      Serial.print(ss4 - s4); Serial.print(", ");
      Serial.print(ss5 - s5); Serial.print(", ");
      Serial.print(ss6 - s6); Serial.print(", ");
      Serial.print(ss7 - s7); Serial.print(", ");
      Serial.print(ss8 - s8); Serial.print(", ");

      Serial.print(ss11 - s11); Serial.print(", ");
      Serial.print(ss12 - s12); Serial.print(", ");
      Serial.print(ss13 - s13); Serial.print(", ");
      Serial.print(ss14 - s14); Serial.print(", ");
      Serial.print(ss15 - s15); Serial.print(", ");
      Serial.print(ss16 - s16); Serial.print(", ");
      Serial.print(ss17 - s17); Serial.print(", ");
      Serial.println(ss18 - s18);


      rightArmRange(s1, ran1);
      rightArmRange(s2, ran2);
      rightArmRange(s4, ran4);

      leftArmRange(s11, ran11);
      leftArmRange(s12, ran12);
      leftArmRange(s14, ran14);

      digitalWrite(pin1, HIGH);

      //モータへ送信
      dxl.goalPosition(TARGET_ID1, ss1);
      dxl.goalPosition(TARGET_ID2, ss2);
      dxl.goalPosition(TARGET_ID3, ss3);
      dxl.goalPosition(TARGET_ID4, ss4);
      dxl.goalPosition(TARGET_ID5, ss5);
      dxl.goalPosition(TARGET_ID6, ss6);
      dxl.goalPosition(TARGET_ID7, ss7);
      dxl.goalPosition(TARGET_ID8, ss8 + 15);

      dxl.goalPosition(TARGET_ID11, ss11);
      dxl.goalPosition(TARGET_ID12, ss12);
      dxl.goalPosition(TARGET_ID13, ss13);
      dxl.goalPosition(TARGET_ID14, ss14);
      dxl.goalPosition(TARGET_ID15, ss15);
      dxl.goalPosition(TARGET_ID16, ss16);
      dxl.goalPosition(TARGET_ID17, ss17);
      dxl.goalPosition(TARGET_ID18, ss18 + 15);

      digitalWrite(pin1, LOW);

    }

    if (!stopPlaying) { // STOPボタンが押されていない場合
      DISPwrite("COMPLETE");
    } else {
      DISPwrite("STOPPED");
      stopPlaying = false;
    }

    //dxl.torqueEnable(TARGET_ID1, false);
    //dxl.torqueEnable(TARGET_ID2, false);
    dxl.torqueEnable(TARGET_ID3, false);
    //dxl.torqueEnable(TARGET_ID4, false);
    dxl.torqueEnable(TARGET_ID5, false);
    dxl.torqueEnable(TARGET_ID6, false);
    dxl.torqueEnable(TARGET_ID7, false);
    dxl.torqueEnable(TARGET_ID8, false);

    //dxl.torqueEnable(TARGET_ID11, false);
    //dxl.torqueEnable(TARGET_ID12, false);
    dxl.torqueEnable(TARGET_ID13, false);
    //dxl.torqueEnable(TARGET_ID14, false);
    dxl.torqueEnable(TARGET_ID15, false);
    dxl.torqueEnable(TARGET_ID16, false);
    dxl.torqueEnable(TARGET_ID17, false);
    dxl.torqueEnable(TARGET_ID18, false);
    mode = 0;
    zero();
    slow();
  }
}



//オーディオインタフェースモード
void audioLoop() {
  
  O_time++;
  if (audioMode == 1) {
    drawKeypad();
    delay(1000);
    audioMode = 2;
  }

  swAudioState = digitalRead(swAudio);  //場所を変えない

  if (audioMode == 2) {
    drawKeypad();
    if (swAudioState == 0) {
      mode = 11;
      audioMode = -1;
      O_time = 0;
      playMotion();
    }
  }

  if (audioMode == 3) {
    drawKeypad();
    if (swAudioState == 0) {
      mode = 12;
      audioMode = -1;
      O_time = 0;
      playMotion();
    }
  }

  if (audioMode == 4) {
    drawKeypad();
    if (swAudioState == 0) {
      mode = 13;
      audioMode = -1;
      O_time = 0;
      playMotion();
    }
  }

  if (audioMode == 5) {
    drawKeypad();
    if (swAudioState == 0) {
      mode = 14;
      audioMode = -1;
      O_time = 0;
      playMotion();
    }
  }

  if (audioMode == 6) {
    drawKeypad();
    if (swAudioState == 0) {
      mode = 15;
      audioMode = -1;
      O_time = 0;
      playMotion();
    }
  }

  if (audioMode == 7) {
    drawKeypad();
    if (swAudioState == 0) {
      mode = 16;
      audioMode = -1;
      O_time = 0;
      playMotion();
    }
  }

  if (audioMode == 8) {
    audioMode = 0;
    drawKeypad();
  }


  if (audioMode > 1 && O_time > O_t) {
    audioMode++;
    O_time = 0;
  }

  if (mode > 10) {
    Serial.print("mode>10");
    playMotion();
  }

}


void armloop() {

  if (audioMode > 0) {
    audioLoop();
  } else { 
    Pgain_on();
    delay(10);
    // demo();//デモを再生せよ
    dxl.driveMode(TARGET_ID1, 0x04);
    dxl.driveMode(TARGET_ID2, 0x04);
    dxl.driveMode(TARGET_ID3, 0x04);
    dxl.driveMode(TARGET_ID4, 0x04);
    dxl.driveMode(TARGET_ID5, 0x04);
    dxl.driveMode(TARGET_ID6, 0x04);
    dxl.driveMode(TARGET_ID7, 0x04);
    dxl.driveMode(TARGET_ID8, 0x04);
    dxl.driveMode(TARGET_ID11, 0x04);
    dxl.driveMode(TARGET_ID12, 0x04);
    dxl.driveMode(TARGET_ID13, 0x04);
    dxl.driveMode(TARGET_ID14, 0x04);
    dxl.driveMode(TARGET_ID15, 0x04);
    dxl.driveMode(TARGET_ID16, 0x04);
    dxl.driveMode(TARGET_ID17, 0x04);
    dxl.driveMode(TARGET_ID18, 0x04);



    dxl.profileVelocity(TARGET_ID1, profileVelocity);
    dxl.profileVelocity(TARGET_ID2, profileVelocity);
    dxl.profileVelocity(TARGET_ID3, profileVelocity);
    dxl.profileVelocity(TARGET_ID4, profileVelocity);
    dxl.profileVelocity(TARGET_ID5, profileVelocity);
    dxl.profileVelocity(TARGET_ID6, profileVelocity);
    dxl.profileVelocity(TARGET_ID7, profileVelocity);
    dxl.profileVelocity(TARGET_ID8, profileVelocity);
    dxl.profileVelocity(TARGET_ID11, profileVelocity);
    dxl.profileVelocity(TARGET_ID12, profileVelocity);
    dxl.profileVelocity(TARGET_ID13, profileVelocity);
    dxl.profileVelocity(TARGET_ID14, profileVelocity);
    dxl.profileVelocity(TARGET_ID15, profileVelocity);
    dxl.profileVelocity(TARGET_ID16, profileVelocity);
    dxl.profileVelocity(TARGET_ID17, profileVelocity);
    dxl.profileVelocity(TARGET_ID18, profileVelocity);
    delay(10);

    if (swAudioState == 0) {
      audioMode = 1;
    }
    if (SerialBT.available()) {
      receivedChar = SerialBT.read();
    }


    //mode=0（RECモードの時）、ボタンを押されたらモーション記録プロセスへ
    if (sw01State == 0 && mode == 0) {
      mode = 1;
      DISPwrite("W_1");
      recordMotion();
      mode = 10;
      sw01State = 1;
    } else if (sw02State == 0 && mode == 0) {
      mode = 2;
      DISPwrite("W_2");
      recordMotion();
      mode = 10;
      sw02State = 1;
    } else if (sw03State == 0 && mode == 0) {
      mode = 3;
      DISPwrite("W_3");
      recordMotion();
      mode = 10;
      sw03State = 1;
    } else if (sw04State == 0 && mode == 0) {
      mode = 4;
      DISPwrite("W_4");
      recordMotion();
      mode = 10;
      sw04State = 1;
    } else if (sw05State == 0 && mode == 0) {
      mode = 5;
      DISPwrite("W_5");
      recordMotion();
      mode = 10;
      sw05State = 1;
    }


    //mode=10（RUNモードの時）、ボタンを押されたらモーション再生プロセスへ

    if ((sw01State == 0 && mode == 10) || receivedChar == 49) { //ASCII 1
      mode = 11;
      Serial.print("mode = 11:");
      DISPwrite("R_11");
      playMotion();
      Serial.println("Reader!1");
      mode = 10;
      receivedChar = 0;
      sw01State = 1;
    } else if ((sw02State == 0 && mode == 10) || receivedChar == 50) { //ASCII 2
      mode = 12;
      Serial.print("mode= 12:");
      DISPwrite("R_12");
      playMotion();
      Serial.println("Reader!2");
      mode = 10;
      receivedChar = 0;
      sw02State = 1;
    } else if ((sw03State == 0 && mode == 10) || receivedChar == 51) { //ASCII 3
      mode = 13;
      DISPwrite("R_13");
      Serial.print("mode= 13:");
      Serial.println("Reader!3");
      playMotion();
      mode = 10;
      receivedChar = 0;
      sw03State = 1;
    } else if ((sw04State == 0 && mode == 10) || receivedChar == 52) { //ASCII 4
      mode = 14;
      DISPwrite("R_14");
      Serial.print("mode= 14:");
      Serial.println("Reader!4");
      playMotion();
      mode = 10;
      receivedChar = 0;
      sw04State = 1;
    } else if ((sw05State == 0 && mode == 10) || receivedChar == 53) { //ASCII 5
      mode = 15;
      DISPwrite("R_15");
      Serial.print("mode= 15:");
      Serial.println("Reader!5");
      playMotion();
      mode = 10;
      receivedChar = 0;
      sw05State = 1;
    }
  }
}




void setup() {

  // Serial.begin(115200);
  Serial.begin(19200);
  Serial1.begin(115200, SERIAL_8N1, rs485TX, rs485RX);
  Serial1.flush(); // 受信バッファをクリア
  DYNAMIXEL_SERIAL.begin(1000000);
  dxl.attach(DYNAMIXEL_SERIAL, 1000000);
  SerialBT.begin(bluetoothDeviceName); // Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

  dxl.addModel<DxlModel::X>(TARGET_ID1);
  dxl.addModel<DxlModel::X>(TARGET_ID2);
  dxl.addModel<DxlModel::X>(TARGET_ID3);
  dxl.addModel<DxlModel::X>(TARGET_ID4);
  dxl.addModel<DxlModel::X>(TARGET_ID5);
  dxl.addModel<DxlModel::X>(TARGET_ID6);
  dxl.addModel<DxlModel::X>(TARGET_ID7);
  dxl.addModel<DxlModel::X>(TARGET_ID8);

  dxl.addModel<DxlModel::X>(TARGET_ID11);
  dxl.addModel<DxlModel::X>(TARGET_ID12);
  dxl.addModel<DxlModel::X>(TARGET_ID13);
  dxl.addModel<DxlModel::X>(TARGET_ID14);
  dxl.addModel<DxlModel::X>(TARGET_ID15);
  dxl.addModel<DxlModel::X>(TARGET_ID16);
  dxl.addModel<DxlModel::X>(TARGET_ID17);
  dxl.addModel<DxlModel::X>(TARGET_ID18);

  dxl.torqueEnable(TARGET_ID1, false);
  dxl.torqueEnable(TARGET_ID2, false);
  dxl.torqueEnable(TARGET_ID3, false);
  dxl.torqueEnable(TARGET_ID4, false);
  dxl.torqueEnable(TARGET_ID5, false);
  dxl.torqueEnable(TARGET_ID6, false);
  dxl.torqueEnable(TARGET_ID7, false);
  dxl.torqueEnable(TARGET_ID8, false);

  dxl.torqueEnable(TARGET_ID11, false);
  dxl.torqueEnable(TARGET_ID12, false);
  dxl.torqueEnable(TARGET_ID13, false);
  dxl.torqueEnable(TARGET_ID14, false);
  dxl.torqueEnable(TARGET_ID15, false);
  dxl.torqueEnable(TARGET_ID16, false);
  dxl.torqueEnable(TARGET_ID17, false);
  dxl.torqueEnable(TARGET_ID18, false);

  Pgain_on();


  ran1 = dxl.presentPosition(TARGET_ID1); delay(5);
  ran2 = dxl.presentPosition(TARGET_ID2); delay(5);
  ran3 - dxl.presentPosition(TARGET_ID3); delay(5);
  ran4 = dxl.presentPosition(TARGET_ID4); delay(5);

  ran11 = dxl.presentPosition(TARGET_ID11); delay(5);
  ran12 = dxl.presentPosition(TARGET_ID12); delay(5);
  ran13 - dxl.presentPosition(TARGET_ID13); delay(5);
  ran14 = dxl.presentPosition(TARGET_ID14); delay(5);

  Serial.print(ran1);
  Serial.print(",");
  Serial.print(ran2);
  Serial.print(",");
  Serial.print(ran4);

  Serial.print(ran11);
  Serial.print(",");
  Serial.print(ran12);
  Serial.print(",");
  Serial.println(ran14);

  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(pin3, OUTPUT);
  
  

  tft.init();  // Initialise the TFT screen
  tft.setRotation(0);// Set the rotation before we calibrate
  touch_calibrate();// Calibrate the touch screen and retrieve the scaling factors
  tft.fillScreen(TFT_BLACK);  // Clear the screen
  tft.fillRect(0, 0, 240, 320, TFT_DARKGREY);  // Draw keypad background
  tft.fillRect(DISP_X, DISP_Y, DISP_W, DISP_H, TFT_BLACK);  // Draw number display area and frame
  tft.drawRect(DISP_X, DISP_Y, DISP_W, DISP_H, TFT_WHITE);

  tft.fillRect(UNDERDISP_X, UNDERDISP_Y, UNDERDISP_W, UNDERDISP_H, TFT_BLACK);  // Draw number display area and frame
  tft.drawRect(UNDERDISP_X, UNDERDISP_Y, UNDERDISP_W, UNDERDISP_H, TFT_WHITE);

  tft.fillRect(TIMERDISP_X, TIMERDISP_Y, TIMERDISP_W, TIMERDISP_H, TFT_BLACK);  // Draw number display area and frame
  tft.drawRect(TIMERDISP_X, TIMERDISP_Y, TIMERDISP_W, TIMERDISP_H, TFT_WHITE);

  drawKeypad();// Draw keypad
  
  // Use GLCD font for smaller text
  tft.setTextDatum(TL_DATUM);
  tft.setTextFont(1);  // Set font to GLCD font
  DISPwrite(VERSION_NUMBER);
  delay(2000);
  DISPwrite(bluetoothDeviceName);

  Serial.println("mode= " + mode);
  Serial.print("sw01State= " + sw01State);

  // timer = timerBegin(0, 80, true);
	// timerAttachInterrupt(timer, &onTimer, true);
	// timerAlarmWrite(timer, 33333, true);  //毎秒30回ポジションを保存する
	// timerAlarmEnable(timer);

  Serial1.println("ON_LED");
  Serial1.println("Green_LED");

  pinMode(swAudio, INPUT_PULLUP);
  Serial.println("setup done");
}

Action checkAction(String command) {
    command.trim();
    for (int i = 0; i < sizeof(ACTIONS); i += 1) {
        if (command == ACTIONS[i].command) {
        return ACTIONS[i];
        }
    }
    return ACTIONS[0];
}


void loop(void) {

  if (mainloop == false){
    digitalWrite(pin2, LOW);
    mainloop = true;
  } else {
    digitalWrite(pin2, HIGH);
    mainloop = false;
  }


  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    Action action = checkAction(command);
    if (action.id == 0) return;


    if (action.id == ArrowPressUp) {

    }


    if (action.id == ArrowPressDown) {
      
    }


    if (action.id == ArrowPressRight) {
      
    }


    if (action.id == ArrowPressLeft) {
      
    }


    if (action.id == ArrowOut) {
      
    }


    if (action.id == ArrowPressCenter) {
      
    }



    if (action.id == ButtonPressA) { //
        // Serial.println("A-button");
        mode = 11;
        playMotion();
    }


    if (action.id == ButtonPressB) { //
      // Serial.println("B-button");
      mode = 12;
      playMotion();
    }


    if (action.id == ButtonPressC) { //
      // Serial.println("C-button");
      mode = 13;
      playMotion();
    }


    if (action.id == ButtonPressD) { //
      // Serial.println("D-button");
      mode = 14;
      playMotion();
    }

    if (action.id == ButtonPressE) { //
      // Serial.println("E-button");
      mode = 15;
      playMotion();
    }


    if (action.id == ButtonOut) {
    }
  }




  uint16_t t_x = 0, t_y = 0; // To store the touch coordinates

  // Pressed will be set true is there is a valid touch on the screen
  bool pressed = tft.getTouch(&t_x, &t_y);

  // / Check if any key coordinate boxes contain the touch coordinates
  for (uint8_t b = 0; b < 12; b++) {
    if (pressed && key[b].contains(t_x, t_y)) {
      key[b].press(true);  // tell the button it is pressed
      drawKeypad();
    } else {
      key[b].press(false);  // tell the button it is NOT pressed
    }
  }

  // Check if any key has changed state
  //　b=0を一番左上のRUNとし、ボタンが押されたらそれを実行する。
  for (uint8_t b = 0; b < 12; b++) {

    if (b < 3) tft.setFreeFont(LABEL1_FONT);
    else tft.setFreeFont(LABEL2_FONT);
    if (key[b].justReleased()) key[b].drawButton();
    if (key[b].justPressed()) {
      key[b].drawButton(true);

      if (b == 0) {
        DISPwrite("RUN b=0");
        mode = 10;
      }

      if (b == 1) {
        if (mode >= 1 && mode <= 9) {
          stopRecording = true; // STOPボタンが押された場合
        } else {
          DISPwrite("MODE b=1");
        }
      }

      if (b == 2) {
        DISPwrite("REC b=2");
        mode = 0;
      }

      if (b == 3 ) {
        DISPwrite("b=3");
        sw01State = 0;
      }

      if (b == 4 ) {
        DISPwrite("b=4");
        sw02State = 0;
      }

      if (b == 5 ) {
        DISPwrite("b=5");
        sw03State = 0;
      }

      if (b == 6 ) {
        DISPwrite("b=6");
        sw04State = 0;
      }

      if (b == 7 ) {
        DISPwrite("b=7");
        sw05State = 0;
      }
      drawKeypad();

    }
    //DISPprint();
  }
  armloop();
  zero();
  slow();

  int swAudioState = digitalRead(swAudio);
  if (swAudioState == 0) {  // Assuming the switch pulls the pin LOW when pressed
    // Serial.print("Audio Switch ON  ");
  } else {
    // Serial.print("Audio Switch OFF  ");
  }
  if (swAudioState == 0) {
    audioMode = 1;
  }
  // Serial.print(" audioMode= ");
  // Serial.print(audioMode);
  // Serial.print(" swAudioState= ");
  // Serial.print(swAudioState);

}