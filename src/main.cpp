#include <Arduino.h>
#include <Dynamixel.h>
#include <esp_system.h>
#define DYNAMIXEL_SERIAL Serial2 // change as you want

#include <SPIFFS.h>
#include "FS.h"
#include <SPI.h>
#include <TFT_eSPI.h>
#include "freertos/semphr.h"

String numberBuffer1 = "test";

// ---- S/W Version ------------------
#define VERSION_NUMBER  "ver. 0.14.15"
// -----------------------------------


TaskHandle_t thp[1]; // マルチスレッドのタスクハンドル格納用


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
static const int HeadArrowPressUp = 35;
static const int HeadArrowPressDown = 36;
static const int HeadArrowPressLeft = 37;
static const int HeadArrowPressRight = 38;
static const int HeadArrowPressCenter = 39;
static const int HeadArrowOut = 40;
static const int MoveArrowPressUp = 41;
static const int MoveArrowPressDown = 42;
static const int MoveArrowPressLeft = 43;
static const int MoveArrowPressRight = 44;
static const int MoveArrowPressCenter = 45;
static const int MoveArrowOut = 46;
static const int End = 47;

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
  { HeadArrowPressUp, "HEAD_ARROW_PRESS_UP"},
  { HeadArrowPressDown, "HEAD_ARROW_PRESS_DOWN"},
  { HeadArrowPressLeft, "HEAD_ARROW_PRESS_LEFT"},
  { HeadArrowPressRight, "HEAD_ARROW_PRESS_RIGHT"},
  { HeadArrowPressCenter, "HEAD_ARROW_PRESS_CENTER"},
  { HeadArrowOut, "HEAD_ARROW_OUT"},
  { MoveArrowPressUp, "MOVE_ARROW_PRESS_UP"},
  { MoveArrowPressDown, "MOVE_ARROW_PRESS_DOWN"},
  { MoveArrowPressLeft, "MOVE_ARROW_PRESS_LEFT"},
  { MoveArrowPressRight, "MOVE_ARROW_PRESS_RIGHT"},
  { MoveArrowPressCenter, "MOVE_ARROW_PRESS_CENTER"},
  { MoveArrowOut, "MOVE_ARROW_OUT"},
  { End, "END" }
};



String bluetoothDeviceName = "YushunArm";


bool onlyLeftArm = false; //左手のみを使用するかどうか
bool mainloop = false;
bool stopRecording = false; //STOPボタンが押されたかどうか
bool stopPlaying = false; //STOPボタンが押されたかどうか
const int rightHandException = 2400;   //手が全開で脱力する時の閾値
const int leftHandException = 800;   //手が全開で脱力する時の閾値


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

const int defaultRecordNumber = 600;
int number = 17;
int* values = nullptr;  //動的メモリ用のポインタ
char buffer[16]; // 数値の一時的な保持のためのバッファ

int startRecordTime = 0;
int endRecordTime = 0;
int totalRecordTime = 0;
int playMotionTime = 200;

bool motionRequested = false;
int requestedMode = 0;


const int timer = defaultRecordNumber;
int TIMERLENGTH = 0;


int horizontalLevel = 0;  //0:中央, 1:左Level.1, 2:左Level.2, 3:左Level.3, -1:右Level.1, -2:右Level.2, -3:右Level.3
int verticalLevel = 0;  //0:中央, 1:上Level.1, 2:上Level.2, 3:上Level.3, -1:下Level.1, -2:下Level.2, -3:下Level.3

int currentHorizontalPos = 0;
int currentVerticalPos = 0;

const int pressButtonCount = 2;

int profileVelocity = 150;
int headProfileVelocity = 1500;

const int verticalHomePos = 1900; //中央
const int verticalMaxPos = 2200; //下
const int verticalMinPos = 1700; //上
const int horizontalHomePos = 2000; //中央
const int horizontalMaxPos = 2500; //左
const int horizontalMinPos = 1500; //右


int sw00State = 1, sw02State = 1, sw03State = 1, sw04State = 1, sw05State = 1, swAudioState = 1;
int sw01State = 1;
int targetPos01, targetPos02, targetPos03, targetPos04, targetPos05, targetPos06, targetPos07, targetPos08 = 0;
int targetPos11, targetPos12, targetPos13, targetPos14, targetPos15, targetPos16, targetPos17, targetPos18 = 0;
int targetPos21, targetPos22, targetPos23, targetPos24 = 0;


int s1, s2, s3, s4, s5, s6, s7, s8 = 0;
int s11, s12, s13, s14, s15, s16, s17, s18 = 0;
int s21, s22, s23, s24 = 0;
char receivedChar = 0;

const int s1diference = 500; //id:01モータの目標値と実測値の差分 この差分を超えるとモーションを終了する
const int s2diference = 400; //id:02モータの目標値と実測値の差分 この差分を超えるとモーションを終了する
const int s4diference = 800; //id:04モータの目標値と実測値の差分 この差分を超えるとモーションを終了する

const int s11diference = 500; //id:11モータの目標値と実測値の差分 この差分を超えるとモーションを終了する
const int s12diference = 400; //id:12モータの目標値と実測値の差分 この差分を超えるとモーションを終了する
const int s14diference = 800; //id:14モータの目標値と実測値の差分 この差分を超えるとモーションを終了する

int rightArmFlag = 0; //フリーの時、落下防止に動きを遅くするフラグ
int leftArmFlag = 0; //フリーの時、落下防止に動きを遅くするフラグ


int ran1, ran2, ran4, ran5, ran6, ran7, ran8 = 0;
int ran11, ran12, ran14, ran15, ran16, ran17, ran18 = 0;
int ran3 = 2100;    //なぜかモータの角度を取得できないので、固定値を入れておく
int ran13 = 1800;   //なぜかモータの角度を取得できないので、固定値を入れておく
int s08 = 0;
int s018 = 0;

int mode = 10; //1-9:モーション登録, 11-19:モーション再生
int audioMode = 0;

int moveMode = 0;

int O_time = 0;
const int O_t = 20;

int past_mode = 0;

int serialnumberP = 700;
int serialnumberI = 2;
int serialnumberD = 1;


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

const uint8_t TARGET_ID21 = 21;
const uint8_t TARGET_ID22 = 22;
const uint8_t TARGET_ID23 = 23;
const uint8_t TARGET_ID24 = 24;


int dxl_goal_position1[2];
int dxl_goal_position2[2];
bool dir = true;
int t = 0;


TFT_eSPI tft = TFT_eSPI();
#define CALIBRATION_FILE "/TouchCalData1"
#define REPEAT_CAL false
#define KEY_X 40
#define KEY_Y 90
#define KEY_W 74
#define KEY_H 45
#define KEY_SPACING_X 6
#define KEY_SPACING_Y 5
#define KEY_TEXTSIZE 0

#define LABEL1_FONT &FreeSansOblique12pt7b
#define LABEL2_FONT &FreeSansBold12pt7b

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

#define NUM_LEN 12
char numberBuffer[NUM_LEN + 1] = "";
uint8_t numberIndex = 0;

#define STATUS_X 120
#define STATUS_Y 65

char keyLabel[9][5] = {"RUN", "MODE", "REC", "1", "2", "3", "4", "5", "6"};
uint16_t keyColor[9] = {
TFT_RED, TFT_DARKGREY, TFT_DARKGREEN,
TFT_DARKGREY, TFT_DARKGREY, TFT_DARKGREY,
TFT_DARKGREY, TFT_DARKGREY, TFT_DARKGREY,
};

TFT_eSPI_Button key[9];


void DISPprint() {
  tft.setTextDatum(TL_DATUM);
  tft.setFreeFont(&FreeSans18pt7b);
  tft.setTextColor(DISP_TCOLOR);
  int xwidth = tft.drawString(numberBuffer1, DISP_X + 4, DISP_Y + 12);

  tft.fillRect(DISP_X + 4 + xwidth, DISP_Y + 1, DISP_W - xwidth - 5, DISP_H - 2, TFT_BLACK);
  delay(10);
}

void DISPreset() {
  tft.setTextDatum(TL_DATUM);
  tft.setFreeFont(&FreeSans18pt7b);
  tft.setTextColor(DISP_TCOLOR);
  numberBuffer1 = "";
  int xwidth = tft.drawString(numberBuffer1, DISP_X + 4, DISP_Y + 12);
  tft.fillRect(DISP_X + 4 + xwidth, DISP_Y + 1, DISP_W - xwidth - 5, DISP_H - 2, TFT_BLACK);
  delay(10);
}

void DISPwrite(String A) {
  Serial.print("DISP= ");
  Serial.println(A);
  tft.setTextDatum(TL_DATUM);
  tft.setFreeFont(&FreeSans18pt7b);
  tft.setTextColor(DISP_TCOLOR);
  numberBuffer1 = "";
  int xwidth = tft.drawString(numberBuffer1, DISP_X + 4, DISP_Y + 12);
  tft.fillRect(DISP_X + 4 + xwidth, DISP_Y + 1, DISP_W - xwidth - 5, DISP_H - 2, TFT_BLACK);
  delay(10);
  xwidth = tft.drawString(A, DISP_X + 4, DISP_Y + 12);
  tft.fillRect(DISP_X + 4 + xwidth, DISP_Y + 1, DISP_W - xwidth - 5, DISP_H - 2, TFT_BLACK);
  delay(10);
}

void UNDERDISPprint() {
  tft.setTextDatum(TL_DATUM);
  tft.setFreeFont(&FreeSans18pt7b);
  tft.setTextColor(UNDERDISP_TCOLOR);
  int xwidth = tft.drawString(numberBuffer1, UNDERDISP_X + 4, UNDERDISP_Y + 12);

  tft.fillRect(UNDERDISP_X + 4 + xwidth, UNDERDISP_Y + 1, UNDERDISP_W - xwidth - 5, UNDERDISP_H - 2, TFT_BLACK);
  delay(10);
}

void UNDERDISPreset() {
  tft.setTextDatum(TL_DATUM);
  tft.setFreeFont(&FreeSans18pt7b);
  tft.setTextColor(UNDERDISP_TCOLOR);
  numberBuffer1 = "";
  int xwidth = tft.drawString(numberBuffer1, UNDERDISP_X + 4, UNDERDISP_Y + 12);
  tft.fillRect(UNDERDISP_X + 4 + xwidth, UNDERDISP_Y + 1, UNDERDISP_W - xwidth - 5, UNDERDISP_H - 2, TFT_BLACK);
  delay(10);
}

void UNDERDISPwrite(String A) {
  tft.setTextDatum(TL_DATUM);
  tft.setFreeFont(&FreeSans18pt7b);
  tft.setTextColor(UNDERDISP_TCOLOR);
  numberBuffer1 = "";
  int xwidth = tft.drawString(numberBuffer1, UNDERDISP_X + 4, UNDERDISP_Y + 12);
  tft.fillRect(UNDERDISP_X + 4 + xwidth, UNDERDISP_Y + 1, UNDERDISP_W - xwidth - 5, UNDERDISP_H - 2, TFT_BLACK);
  delay(10);
  xwidth = tft.drawString(A, UNDERDISP_X + 4, UNDERDISP_Y + 12);
  tft.fillRect(UNDERDISP_X + 4 + xwidth, UNDERDISP_Y + 1, UNDERDISP_W - xwidth - 5, UNDERDISP_H - 2, TFT_BLACK);
  delay(10);
}

void TIMERDISPprint() {
  tft.setTextDatum(TL_DATUM);
  tft.setFreeFont(&FreeSans18pt7b);
  tft.setTextColor(UNDERDISP_TCOLOR);
  int xwidth = tft.drawString(numberBuffer1, UNDERDISP_X + 4, UNDERDISP_Y + 12);

  tft.fillRect(UNDERDISP_X + 4 + xwidth, UNDERDISP_Y + 1, UNDERDISP_W - xwidth - 5, UNDERDISP_H - 2, TFT_BLACK);
  delay(10);
}

void TIMERDISPreset() {
  tft.fillRect(TIMERDISP_X, TIMERDISP_Y, TIMERDISP_W, TIMERDISP_H, TFT_BLACK);
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

  if (!SPIFFS.begin()) {
    Serial.println("formatting file system");
    SPIFFS.format();
    SPIFFS.begin();
  }

  if (SPIFFS.exists(CALIBRATION_FILE)) {
    if (REPEAT_CAL)
    {
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
    tft.setTouch(calData);
  } else {
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

    File f = SPIFFS.open(CALIBRATION_FILE, "w");
    if (f) {
      f.write((const unsigned char *)calData, 14);
      f.close();
    }
  }
}

void status(const char *msg) {

  tft.setTextPadding(240);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft.setTextFont(0);
  tft.setTextDatum(TC_DATUM);
  tft.setTextSize(1);
  tft.drawString(msg, STATUS_X, STATUS_Y);
}

void drawKeypad() {
  for (uint8_t row = 0; row < 3; row++) {
    for (uint8_t col = 0; col < 3; col++) {
    uint8_t b = col + row * 3;

    if (b < 3) tft.setFreeFont(LABEL1_FONT);
    else tft.setFreeFont(LABEL2_FONT);

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

    } else {
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

void zero() {
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
  if (leftArmFlag == 4)leftArmFlag = 0;
}

void slow() {

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
  int de = 5;
  delay(de);

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
  targetPos18 = dxl.presentPosition(TARGET_ID18); delay(de);
  targetPos21 = dxl.presentPosition(TARGET_ID21); delay(de);
  targetPos23 = dxl.presentPosition(TARGET_ID23); delay(de);
  targetPos24 = dxl.presentPosition(TARGET_ID24); delay(de);

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
  Serial.print(targetPos18); Serial.print(", ");
  Serial.print(targetPos21); Serial.print(", ");
  Serial.print(targetPos23); Serial.print(", ");
  Serial.println(targetPos24);
}

void settingProfileVelocity(int settingTime) {
  dxl.profileVelocity(TARGET_ID1, settingTime);
  dxl.profileVelocity(TARGET_ID2, settingTime);
  dxl.profileVelocity(TARGET_ID3, settingTime);
  dxl.profileVelocity(TARGET_ID4, settingTime);
  dxl.profileVelocity(TARGET_ID5, settingTime);
  dxl.profileVelocity(TARGET_ID6, settingTime);
  dxl.profileVelocity(TARGET_ID7, settingTime);
  dxl.profileVelocity(TARGET_ID8, settingTime);
  dxl.profileVelocity(TARGET_ID11, settingTime);
  dxl.profileVelocity(TARGET_ID12, settingTime);
  dxl.profileVelocity(TARGET_ID13, settingTime);
  dxl.profileVelocity(TARGET_ID14, settingTime);
  dxl.profileVelocity(TARGET_ID15, settingTime);
  dxl.profileVelocity(TARGET_ID16, settingTime);
  dxl.profileVelocity(TARGET_ID17, settingTime);
  dxl.profileVelocity(TARGET_ID18, settingTime);
  
  delay(100);
}

void startMode() {
  verticalLevel = 0;
  horizontalLevel = 0;
  dxl.torqueEnable(TARGET_ID21, true);
  dxl.torqueEnable(TARGET_ID23, true);
  dxl.torqueEnable(TARGET_ID24, true);
  headProfileVelocity = 3000;
  dxl.profileVelocity(TARGET_ID21, headProfileVelocity);
  dxl.profileVelocity(TARGET_ID23, headProfileVelocity);
  headProfileVelocity = 1500;
  dxl.goalPosition(TARGET_ID21, verticalHomePos);
  dxl.goalPosition(TARGET_ID23, horizontalHomePos);
}

void endMode() {
  headProfileVelocity = 3000;
  dxl.profileVelocity(TARGET_ID21, headProfileVelocity);
  dxl.profileVelocity(TARGET_ID23, headProfileVelocity);
  headProfileVelocity = 1500;
  dxl.goalPosition(TARGET_ID21, verticalMaxPos);
  dxl.goalPosition(TARGET_ID23, horizontalHomePos);
  delay(3000);
  dxl.torqueEnable(TARGET_ID21, false);
  dxl.torqueEnable(TARGET_ID23, false);
  dxl.torqueEnable(TARGET_ID24, false);
}

void moveStraightFace() {
  dxl.goalPosition(TARGET_ID23, horizontalHomePos);
  horizontalLevel = 0;
}

void moveVertical(int direction) {
  if (verticalLevel < pressButtonCount && direction == 1) {
      verticalLevel++;
  } else if (verticalLevel > -pressButtonCount && direction == -1) {
      verticalLevel--;
  }
  dxl.goalPosition(TARGET_ID21, ((verticalHomePos - verticalMinPos) / pressButtonCount) * (-verticalLevel) + verticalHomePos);
}

void moveHorizontal(int direction) {
  if (horizontalLevel < pressButtonCount && direction == 1) {
      horizontalLevel++;
  } else if (horizontalLevel > -pressButtonCount && direction == -1) {
      horizontalLevel--;
  }
  dxl.goalPosition(TARGET_ID23, ((horizontalHomePos - horizontalMinPos) / pressButtonCount) * (-horizontalLevel) + horizontalHomePos);
}

void centerPosition() {
  dxl.goalPosition(TARGET_ID21, verticalHomePos);
  dxl.goalPosition(TARGET_ID23, horizontalHomePos);
  verticalLevel = 0;
  horizontalLevel = 0;
  dxl.profileVelocity(TARGET_ID21, headProfileVelocity);
  dxl.profileVelocity(TARGET_ID23, headProfileVelocity);
}


void stopMotion() {

  settingProfileVelocity(10000);

  dxl.goalPosition(TARGET_ID1, ran1);
  dxl.goalPosition(TARGET_ID2, ran2);
  dxl.goalPosition(TARGET_ID3, ran3);
  dxl.goalPosition(TARGET_ID4, ran4);
  dxl.goalPosition(TARGET_ID5, ran5);
  dxl.goalPosition(TARGET_ID6, ran6);
  dxl.goalPosition(TARGET_ID7, ran7);
  dxl.goalPosition(TARGET_ID8, ran8);
  dxl.goalPosition(TARGET_ID11, ran11);
  dxl.goalPosition(TARGET_ID12, ran12);
  dxl.goalPosition(TARGET_ID13, ran13);
  dxl.goalPosition(TARGET_ID14, ran14);
  dxl.goalPosition(TARGET_ID15, ran15);
  dxl.goalPosition(TARGET_ID16, ran16);
  dxl.goalPosition(TARGET_ID17, ran17);
  dxl.goalPosition(TARGET_ID18, ran18);

  delay(11000);
  Serial.println("stopMotion");
  Serial.println(ran13);

  settingProfileVelocity(150);
}

void showMemoryData() {
  // // メモリ使用率を表示
  uint32_t totalHeap = ESP.getHeapSize();
  uint32_t freeHeap = ESP.getFreeHeap();
  uint32_t usedHeap = totalHeap - freeHeap;
  float heapUsage = (float)usedHeap / totalHeap * 100;

  uint32_t totalPsram = ESP.getPsramSize();
  uint32_t freePsram = ESP.getFreePsram();
  uint32_t usedPsram = totalPsram - freePsram;
  float psramUsage = (float)usedPsram / totalPsram * 100;

  Serial.printf("Heap Max: %u bytes\n", totalHeap);
  Serial.printf("Heap Used: %u bytes\n", usedHeap);
  Serial.printf("Heap Usage: %.2f%%\n", heapUsage);

  if (totalPsram > 0) {
    Serial.printf("PSRAM Max: %u bytes\n", totalPsram);
    Serial.printf("PSRAM Used: %u bytes\n", usedPsram);
    Serial.printf("PSRAM Usage: %.2f%%\n", psramUsage);
  } else {
    Serial.println("PSRAM not available");
  }

  // SPIFFS information
  if (SPIFFS.begin()) {
    uint32_t totalSpiffs = SPIFFS.totalBytes();
    uint32_t usedSpiffs = SPIFFS.usedBytes();
    float spiffsUsage = (float)usedSpiffs / totalSpiffs * 100;

    Serial.printf("SPIFFS Max: %u bytes\n", totalSpiffs);
    Serial.printf("SPIFFS Used: %u bytes\n", usedSpiffs);
    Serial.printf("SPIFFS Usage: %.2f%%\n", spiffsUsage);

    SPIFFS.end();
  } else {
    Serial.println("SPIFFS Mount Failed");
  }
}

void deleteMotionData() {
  String filesToDelete[] = {"/test1.txt", "/test2.txt", "/test3.txt", "/test4.txt", "/test5.txt"};
  if (SPIFFS.begin()) {
    for (int i = 0; i < 5; i++) {
      if (SPIFFS.exists(filesToDelete[i])) {
        SPIFFS.remove(filesToDelete[i]);
        Serial.println("Deleted: " + filesToDelete[i]);
      } else {
        Serial.println("File not found: " + filesToDelete[i]);
      }
    }
    SPIFFS.end();
  } else {
    Serial.println("SPIFFS Mount Failed");
  }
}

void requestedMotion(int mode) {
  requestedMode = mode;
  motionRequested = true;
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


void handleSerial(){
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    Action action = checkAction(command);
    if (action.id == 0) return;


    if (action.id == ArrowPressUp) {
      Serial.println("ArrowPressUp");
    } else if (action.id == ArrowPressDown) {
      Serial.println("ArrowPressDown");
    } else if (action.id == ArrowPressRight) {
      Serial.println("ArrowPressRight");
    } else if (action.id == ArrowPressLeft) {
      Serial.println("ArrowPressLeft");
    } else if (action.id == ArrowPressCenter) {
      Serial.println("ArrowPressCenter");
      // stopMotion();
      stopPlaying = true;
    } else if (action.id == ArrowOut) {
      Serial.println("ArrowOut");

    } else if (action.id == ButtonPressA) {
      Serial.println("ButtonPressA");
      requestedMotion(11);
    } else if (action.id == ButtonPressB) {
      Serial.println("ButtonPressB");
      requestedMotion(12);
    } else if (action.id == ButtonPressC) {
      Serial.println("ButtonPressC");
      requestedMotion(13);
    } else if (action.id == ButtonPressD) {
      Serial.println("ButtonPressD");
      requestedMotion(14);
    } else if (action.id == ButtonPressE) {
      Serial.println("ButtonPressE");
      requestedMotion(15);
    } else if (action.id == ButtonPressY) {
      Serial.println("ButtonPressY");
      showMemoryData();
    } else if (action.id == ButtonPressZ) {
      Serial.println("ButtonPressZ");
      deleteMotionData();
    } else if (action.id == ButtonOut) {
      Serial.println("ButtonOut");

    }

    if (action.id == Start) {
      Serial.println("talkStart");
      startMode();
    }

    if (action.id == End) {
      Serial.println("endTalk");
      endMode();
    }

    if (action.id == HeadArrowPressUp) {
      Serial.println("HeadArrowPressUp");
      moveVertical(1);
    }
    if (action.id == HeadArrowPressDown) {
      Serial.println("HeadArrowPressDown");
      moveVertical(-1);
    }
    if (action.id == HeadArrowPressRight) {
      Serial.println("HeadArrowPressRight");
      moveHorizontal(1);
    }
    if (action.id == HeadArrowPressLeft) {
      Serial.println("HeadArrowPressLeft");
      moveHorizontal(-1);
    }
    if (action.id == HeadArrowPressCenter) {
      Serial.println("HeadArrowPressCenter");
      centerPosition();
    }
    if (action.id == HeadArrowOut) {
      Serial.println("HeadArrowOut");
    }


    if (action.id == MoveArrowPressUp) {
      Serial.println("移動キー上を押下時に送信");
      Serial.println("MoveArrowPressUp");
    }
    if (action.id == MoveArrowPressDown) {
      Serial.println("移動キ下を押下時に送信");
      Serial.println("MoveArrowPressDown");
    }
    if (action.id == MoveArrowPressRight) {
      Serial.println("移動キー右を押下時に送信");
      Serial.println("MoveArrowPressRight");
    }
    if (action.id == MoveArrowPressLeft) {
      Serial.println("移動キー左を押下時に送信");
      Serial.println("MoveArrowPressLeft");
    }
    if (action.id == MoveArrowOut) {
      Serial.println("移動キーの押下が終了した時に送信");
      Serial.println("MoveArrowOut");
    }

  }
  vTaskDelay(10 / portTICK_PERIOD_MS);
}


void recordMotion() {
  // ファイルの書き込み

  DISPreset();
  numberBuffer1 = "Writer";
  DISPprint();

  TIMERDISPreset();

  if (!SPIFFS.begin()) {
    Serial.println("SPIFFS Mount Failed");
    SPIFFS.end();  // SPIFFSの使用終了
    return;
  }

  if (mode < 10) {
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
      uint16_t t_x = 0, t_y = 0;
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

      Serial.print("レコード中 = ");
      Serial.print(i); Serial.print(" : ");

      //レコード書き出し

      Serial.print(" R : ");
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

      Serial.print(" L : ");
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

      if (stopRecording) { // STOPボタンが押されたかどうかをチェック
        for (int i = 3; i < 9; i++) {
          keyColor[i] = TFT_RED;
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
        mode = 10;
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

    if (!stopRecording) {
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
  Serial.print("mode = " + String(mode));
  drawKeypad();

  TIMERDISPreset();

  if (!SPIFFS.begin()) {
    Serial.println("SPIFFS Mount Failed");
    SPIFFS.end();  // SPIFFSの使用終了
    return;
  }

  if (mode == 11) {
    file = SPIFFS.open("/test1.txt");
  } else if (mode == 12) {
    file = SPIFFS.open("/test2.txt");
  } else if (mode == 13) {
    file = SPIFFS.open("/test3.txt");
  } else if (mode == 14) {
    file = SPIFFS.open("/test4.txt");
  } else if (mode == 15) {
    file = SPIFFS.open("/test5.txt");
  } else {
    Serial.println("Invalid mode");
    return;
  }

  if (!file) {
    Serial.println("ファイルの読み取りに失敗しました");
    return;
  }

  // ファイルの内容を格納する配列
  int index = 0;
  while (file.available()) {
    int pos = 0;
    while (true) {
      char c = file.read();
      if (c == ',' || c == '\n' || c == -1) {
        buffer[pos] = '\0';
        if (index < defaultRecordNumber * 36) { // 範囲チェック
          values[index++] = atoi(buffer);
        }
        pos = 0;
        if (c == '\n' || c == -1) {
          break;
        }
      } else {
        if (pos < sizeof(buffer) - 1) { // バッファオーバーフロー防止
          buffer[pos++] = c;
        }
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
  file.close();
  SPIFFS.end();  // SPIFFSの使用終了

  totalRecordTime = values[index - 2];

  DISPwrite("Play Time: " + String(totalRecordTime) + " ms");
  Serial.println("Play Time: " + String(totalRecordTime) + " ms");

  playMotionTime = totalRecordTime / 164;

  mode = 255;
  if (rightArmFlag == 0) mode = 256;
  if (leftArmFlag == 0) mode = 256;
  if (mode == 255) slow();
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

    settingProfileVelocity(150);

    for (int i = 0; i < playMotionTime && i * number + 15 < defaultRecordNumber * 36; i++) {

      // STOPボタンが押されたかどうかをチェック
      uint16_t t_x = 0, t_y = 0;
      bool pressed = tft.getTouch(&t_x, &t_y);
      if (pressed && key[1].contains(t_x, t_y)) {
        stopPlaying = true;
      }
      if (stopPlaying) { // STOPボタンが押されたかどうかをチェック
        for (int j = 3; j < 9; j++) {
          keyColor[j] = TFT_DARKGREEN;
        }

        DISPwrite("STOPPED");

        stopMotion();

        ///////////////////////////////////////////////////////////////
        dxl.torqueEnable(TARGET_ID3, false);
        dxl.torqueEnable(TARGET_ID5, false);
        dxl.torqueEnable(TARGET_ID6, false);
        dxl.torqueEnable(TARGET_ID7, false);
        dxl.torqueEnable(TARGET_ID8, false);

        dxl.torqueEnable(TARGET_ID13, false);
        dxl.torqueEnable(TARGET_ID15, false);
        dxl.torqueEnable(TARGET_ID16, false);
        dxl.torqueEnable(TARGET_ID17, false);
        dxl.torqueEnable(TARGET_ID18, false);
        mode = 10;
        ///////////////////////////////////////////////////////////////
        
        drawKeypad();

        break;
      }

      if (i * number + 15 >= defaultRecordNumber * 36) {
        Serial.println("インデックス範囲を超えました");
        return;
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

      vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    if (!stopPlaying) {
      DISPwrite("COMPLETE");
    } else {
      // DISPwrite("STOPPED");
      stopPlaying = false;
    }

    dxl.torqueEnable(TARGET_ID3, false);
    dxl.torqueEnable(TARGET_ID5, false);
    dxl.torqueEnable(TARGET_ID6, false);
    dxl.torqueEnable(TARGET_ID7, false);
    dxl.torqueEnable(TARGET_ID8, false);

    dxl.torqueEnable(TARGET_ID13, false);
    dxl.torqueEnable(TARGET_ID15, false);
    dxl.torqueEnable(TARGET_ID16, false);
    dxl.torqueEnable(TARGET_ID17, false);
    dxl.torqueEnable(TARGET_ID18, false);
    mode = 10;
    zero();
    slow();
  }
}


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
    // demo();
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
    dxl.driveMode(TARGET_ID21, 0x04);
    dxl.driveMode(TARGET_ID22, 0x05);
    dxl.driveMode(TARGET_ID23, 0x04);
    dxl.driveMode(TARGET_ID24, 0x04);

    settingProfileVelocity(150);

    dxl.profileVelocity(TARGET_ID21, headProfileVelocity);
    dxl.profileVelocity(TARGET_ID23, headProfileVelocity);
    dxl.profileVelocity(TARGET_ID24, profileVelocity);

    delay(10);

    if (swAudioState == 0) {
      audioMode = 1;
    }

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

void toggleMainLoop() {
  if (mainloop == false) {
    digitalWrite(pin2, LOW);
    mainloop = true;
  } else {
    digitalWrite(pin2, HIGH);
    mainloop = false;
  }
}

void handleKeyPress(uint8_t b) {
  if (b == 0) {
    DISPwrite("RUN b=0");
    mode = 10;
  } else if (b == 1) {
    if (mode >= 1 && mode <= 9) {
      stopRecording = true;
    } else {
      DISPwrite("MODE b=1");
    }
  } else if (b == 2) {
    DISPwrite("REC b=2");
    mode = 0;
  } else if (b >= 3 && b <= 7) {
    DISPwrite("b=" + String(b));
    int states[] = {sw01State, sw02State, sw03State, sw04State, sw05State};
    states[b - 3] = 0;
  }
}

void handleKeypad() {
  uint16_t t_x = 0, t_y = 0;
  bool pressed = tft.getTouch(&t_x, &t_y);
  for (uint8_t b = 0; b < 12; b++) {
    if (pressed && key[b].contains(t_x, t_y)) {
      key[b].press(true);
      drawKeypad();
    } else {
      key[b].press(false);
    }
  }
  for (uint8_t b = 0; b < 12; b++) {
    if (key[b].justReleased()) key[b].drawButton();
    if (key[b].justPressed()) {
      key[b].drawButton(true);
      handleKeyPress(b);
      drawKeypad();
    }
  }
}

void checkAudioMode() {
  int swAudioState = digitalRead(swAudio);
  if (swAudioState == 0) {
    audioMode = 1;
  }
}

void handleMotionRequest() {
  if (motionRequested) {
    mode = requestedMode;
    playMotion();
    motionRequested = false;
    mode = 10;
  }
}


void Core0a(void *args) {
  Serial.println("Core0a Start");
  while (true) { // 永久ループに変更
    handleSerial();
  }
}


void setup() {
  // Serial.begin(115200);
  Serial.begin(19200);
  Serial1.begin(115200, SERIAL_8N1, rs485TX, rs485RX);
  Serial1.flush(); // 受信バッファをクリア
  
  DYNAMIXEL_SERIAL.begin(1000000);
  dxl.attach(DYNAMIXEL_SERIAL, 1000000);

  // 動的メモリの確保
  values = (int*)malloc(defaultRecordNumber * 36 * sizeof(int));
  if (values == nullptr) {
    Serial.println("Memory allocation failed");
    while (1);
  }

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
  
  dxl.addModel<DxlModel::X>(TARGET_ID21);
  dxl.addModel<DxlModel::X>(TARGET_ID23);
  dxl.addModel<DxlModel::X>(TARGET_ID24);

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

  dxl.torqueEnable(TARGET_ID21, false);
  dxl.torqueEnable(TARGET_ID23, false);
  dxl.torqueEnable(TARGET_ID24, false);

  Pgain_on();

  ran1 = dxl.presentPosition(TARGET_ID1); delay(5);
  ran2 = dxl.presentPosition(TARGET_ID2); delay(10);
  ran3 - dxl.presentPosition(TARGET_ID3); delay(5);
  ran4 = dxl.presentPosition(TARGET_ID4); delay(5);
  ran5 = dxl.presentPosition(TARGET_ID5); delay(5);
  ran6 = dxl.presentPosition(TARGET_ID6); delay(5);
  ran7 = dxl.presentPosition(TARGET_ID7); delay(5);
  ran8 = dxl.presentPosition(TARGET_ID8); delay(5);

  ran11 = dxl.presentPosition(TARGET_ID11); delay(5);
  ran12 = dxl.presentPosition(TARGET_ID12); delay(10);
  ran13 - dxl.presentPosition(TARGET_ID13); delay(5);
  ran14 = dxl.presentPosition(TARGET_ID14); delay(5);
  ran15 = dxl.presentPosition(TARGET_ID15); delay(5);
  ran16 = dxl.presentPosition(TARGET_ID16); delay(5);
  ran17 = dxl.presentPosition(TARGET_ID17); delay(5);
  ran18 = dxl.presentPosition(TARGET_ID18); delay(5);

  // Serial.print("ran1 = ");
  // Serial.print(ran1);
  // Serial.print(", ran2 = ");
  // Serial.print(ran2);
  // Serial.print(", ran4 = ");
  // Serial.print(ran4);
  // Serial.print(", ran11 = ");
  // Serial.print(ran11);
  // Serial.print(", ran12 = ");
  // Serial.print(ran12);
  // Serial.print(", ran13 = ");
  // Serial.print(ran13);
  // Serial.print(", ran14 = ");
  // Serial.println(ran14);

  Serial.print(ran1);
  Serial.print(", ");
  Serial.print(ran2);
  Serial.print(", ran3 = ");
  Serial.print(ran3);
  Serial.print(", ");
  Serial.print(ran4);
  Serial.print(", ");
  Serial.print(ran5);
  Serial.print(", ");
  Serial.print(ran6);
  Serial.print(", ");
  Serial.print(ran7);
  Serial.print(", ");
  Serial.print(ran8);
  Serial.print(", ");
  Serial.print(ran11);
  Serial.print(", ");
  Serial.print(ran12);
  Serial.print(", ran13 = ");
  Serial.print(ran13);
  Serial.print(", ");
  Serial.print(ran14);
  Serial.print(", ");
  Serial.print(ran15);
  Serial.print(", ");
  Serial.print(ran16);
  Serial.print(", ");
  Serial.print(ran17);
  Serial.print(", ");
  Serial.println(ran18);



  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(pin3, OUTPUT);

  tft.init();
  tft.setRotation(0);
  touch_calibrate();
  tft.fillScreen(TFT_BLACK);
  tft.fillRect(0, 0, 240, 320, TFT_DARKGREY);
  tft.fillRect(DISP_X, DISP_Y, DISP_W, DISP_H, TFT_BLACK);
  tft.drawRect(DISP_X, DISP_Y, DISP_W, DISP_H, TFT_WHITE);

  tft.fillRect(UNDERDISP_X, UNDERDISP_Y, UNDERDISP_W, UNDERDISP_H, TFT_BLACK);
  tft.drawRect(UNDERDISP_X, UNDERDISP_Y, UNDERDISP_W, UNDERDISP_H, TFT_WHITE);

  tft.fillRect(TIMERDISP_X, TIMERDISP_Y, TIMERDISP_W, TIMERDISP_H, TFT_BLACK);
  tft.drawRect(TIMERDISP_X, TIMERDISP_Y, TIMERDISP_W, TIMERDISP_H, TFT_WHITE);

  drawKeypad();
  
  tft.setTextDatum(TL_DATUM);
  tft.setTextFont(1);
  DISPwrite(VERSION_NUMBER);
  delay(2000);
  // DISPwrite(bluetoothDeviceName);

  Serial.println("mode= " + mode);
  Serial.print("sw01State= " + sw01State);

  pinMode(swAudio, INPUT_PULLUP);

  disableCore0WDT();
  disableCore1WDT();

  // スタックサイズを増やし、優先順位を1に設定
  xTaskCreatePinnedToCore(Core0a, "Core0a", 8192, NULL, 1, &thp[0], 0);
  
  // SPIFFSの初期化とマウント
  if (!SPIFFS.begin(true)) {
    Serial.println("An error has occurred while mounting SPIFFS");
  } else {
    Serial.println("SPIFFS mounted successfully");
  }


  Serial.println("setup done");
}

void loop() {
  toggleMainLoop();
  handleSerial();
  handleKeypad();
  drawKeypad();
  armloop();
  zero();
  slow();
  checkAudioMode();
  handleMotionRequest();
}
