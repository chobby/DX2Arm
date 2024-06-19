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

// ---- S/W Version ------------------
#define VERSION_NUMBER  "TFT Ver. 0.6.2"
// -----------------------------------


bool onlyLeftArm = false; //左手のみを使用するかどうか
bool mainloop = false;
int exception = 1900;   //手が全開で脱力する時の閾値 左腕:1800

const uint8_t PIN_RTS = 11;

const int pin1 = 12;
const int pin2 = 22;
const int pin3 = 25;

const int numRecords = 200;
int number = 9; //←ここの数字はID+2とする
int values[numRecords * 9 ]; //←ここの数字はID＋１とする
char buffer[16]; // 数値の一時的な保持のためのバッファ


const int timer = numRecords;
int TIMERLENGTH = 0;


int sw00State = 1, sw02State = 1, sw03State = 1, sw04State = 1, sw05State = 1, swAudioState = 1;
int sw01State = 1;
int targetPos01, targetPos02, targetPos03, targetPos04, targetPos05, targetPos06, targetPos07, targetPos08 = 0;

int s1, s2, s3, s4, s5, s6, s7, s8 = 0;
char receivedChar = 0;

const int s1diference = 500; //id:01モータの目標値と実測値の差分 この差分を超えるとモーションを終了する
const int s2diference = 400; //id:02モータの目標値と実測値の差分 この差分を超えるとモーションを終了する
const int s4diference = 800; //id:04モータの目標値と実測値の差分 この差分を超えるとモーションを終了する

int z = 0; //フリーの時、落下防止に動きを遅くするフラグ
int ran1, ran2, ran4 = 0;
int s08 = 0; //腕の角度

int mode = 10; //1~9:モーション録画, 11~19:モーション再生
int audioMode = 0; //0:初期値,

int O_time = 0;
const int O_t = 20;

int past_mode = 0;

int serialnumberP = 300;
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

const uint16_t DYNAMIXEL_BAUDRATE = 1000000;
#define RXD2 16
#define TXD2 17

const int swAudio = 21;

Dynamixel dxl(RXD2, TXD2);
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


volatile int interruptCounter;
int totalInterruptCounter;

// hw_timer_t * timer = NULL;
// portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------



// void IRAM_ATTR onTimer() {
// 	portENTER_CRITICAL_ISR(&timerMux);
// 	interruptCounter++;
// 	portEXIT_CRITICAL_ISR(&timerMux);
// }

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

void touch_calibrate()
{
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

void drawKeypad()
{
  // Draw the keys
  for (uint8_t row = 0; row < 3; row++) {
    for (uint8_t col = 0; col < 3; col++) {
      uint8_t b = col + row * 3;

      if (b < 3) tft.setFreeFont(LABEL1_FONT);
      else tft.setFreeFont(LABEL2_FONT);
      if(audioMode == 0){
        
        if(mode > 9){
          keyColor[0] = TFT_DARKGREEN;
          keyColor[1] = TFT_DARKGREY; 
          keyColor[2] = TFT_RED;                 
          keyColor[3] = TFT_DARKGREEN;
          keyColor[4] = TFT_DARKGREEN;
          keyColor[5] = TFT_DARKGREEN;
          keyColor[6] = TFT_DARKGREEN;
          keyColor[7] = TFT_DARKGREEN;
          keyColor[8] = TFT_DARKGREEN;
        }else{
          keyColor[0] = TFT_DARKGREEN;
          keyColor[1] = TFT_DARKGREY; 
          keyColor[2] = TFT_RED;
          keyColor[3] = TFT_RED;
          keyColor[4] = TFT_RED;
          keyColor[5] = TFT_RED;
          keyColor[6] = TFT_RED;
          keyColor[7] = TFT_RED;
          keyColor[8] = TFT_RED;        
        }                                                
      }
      
      if(audioMode == 1){
        keyColor[0] = TFT_DARKGREY;
        keyColor[1] = TFT_BLUE; 
        keyColor[2] = TFT_DARKGREY;
        keyColor[3] = TFT_BLUE;
        keyColor[4] = TFT_BLUE;
        keyColor[5] = TFT_BLUE;
        keyColor[6] = TFT_BLUE;
        keyColor[7] = TFT_BLUE;
        keyColor[8] = TFT_BLUE;        
      }
      if(audioMode == 2){
        keyColor[0] = TFT_DARKGREY;
        keyColor[1] = TFT_BLUE; 
        keyColor[2] = TFT_DARKGREY;
        keyColor[3] = TFT_BLUE;
        keyColor[4] = TFT_DARKGREY;
        keyColor[5] = TFT_DARKGREY;
        keyColor[6] = TFT_DARKGREY;
        keyColor[7] = TFT_DARKGREY;
        keyColor[8] = TFT_DARKGREY;        
      }
      if(audioMode == 3){
        keyColor[0] = TFT_DARKGREY;
        keyColor[1] = TFT_BLUE; 
        keyColor[2] = TFT_DARKGREY;
        keyColor[3] = TFT_DARKGREY;
        keyColor[4] = TFT_BLUE;
        keyColor[5] = TFT_DARKGREY;
        keyColor[6] = TFT_DARKGREY;
        keyColor[7] = TFT_DARKGREY;
        keyColor[8] = TFT_DARKGREY;        
      }
      if(audioMode == 4){
        keyColor[0] = TFT_DARKGREY;
        keyColor[1] = TFT_BLUE; 
        keyColor[2] = TFT_DARKGREY;
        keyColor[3] = TFT_DARKGREY;
        keyColor[4] = TFT_DARKGREY;
        keyColor[5] = TFT_BLUE;
        keyColor[6] = TFT_DARKGREY;
        keyColor[7] = TFT_DARKGREY;
        keyColor[8] = TFT_DARKGREY;        
      }

      if(audioMode == 5){
        keyColor[0] = TFT_DARKGREY;
        keyColor[1] = TFT_BLUE; 
        keyColor[2] = TFT_DARKGREY;
        keyColor[3] = TFT_DARKGREY;
        keyColor[4] = TFT_DARKGREY;
        keyColor[5] = TFT_DARKGREY;
        keyColor[6] = TFT_BLUE;
        keyColor[7] = TFT_DARKGREY;
        keyColor[8] = TFT_DARKGREY;        
      }   

      if(audioMode == 6){
        keyColor[0] = TFT_DARKGREY;
        keyColor[1] = TFT_BLUE; 
        keyColor[2] = TFT_DARKGREY;
        keyColor[3] = TFT_DARKGREY;
        keyColor[4] = TFT_DARKGREY;
        keyColor[5] = TFT_DARKGREY;
        keyColor[6] = TFT_DARKGREY;
        keyColor[7] = TFT_BLUE;
        keyColor[8] = TFT_DARKGREY;        
      } 

      if(audioMode == 7){
        keyColor[0] = TFT_DARKGREY;
        keyColor[1] = TFT_BLUE; 
        keyColor[2] = TFT_DARKGREY;
        keyColor[3] = TFT_DARKGREY;
        keyColor[4] = TFT_DARKGREY;
        keyColor[5] = TFT_DARKGREY;
        keyColor[6] = TFT_DARKGREY;
        keyColor[7] = TFT_DARKGREY;
        keyColor[8] = TFT_BLUE;        
      }              

      key[b].initButton(&tft, KEY_X + col * (KEY_W + KEY_SPACING_X),
                        KEY_Y + row * (KEY_H + KEY_SPACING_Y), // x, y, w, h, outline, fill, text
                        KEY_W, KEY_H, TFT_WHITE, keyColor[b], TFT_WHITE,
                        keyLabel[b], KEY_TEXTSIZE);
      key[b].drawButton();
    }
  }
}

//------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------





void Pgain_on() {
  if (serialnumberI > 1) {
    dxl.positionPGain(TARGET_ID1, serialnumberP);
    dxl.positionPGain(TARGET_ID2, serialnumberP);
    dxl.positionPGain(TARGET_ID3, 700);
    dxl.positionPGain(TARGET_ID4, serialnumberP);
    dxl.positionPGain(TARGET_ID6, 500);
    dxl.positionPGain(TARGET_ID7, 500);
    dxl.positionPGain(TARGET_ID8, 1200);

    // dxl.positionIGain(TARGET_ID1, serialnumberI);
    // dxl.positionIGain(TARGET_ID2, serialnumberI);
    // dxl.positionIGain(TARGET_ID3, serialnumberI);
    // dxl.positionIGain(TARGET_ID4, serialnumberI);
    // dxl.positionIGain(TARGET_ID5, serialnumberI);
    // dxl.positionIGain(TARGET_ID6, serialnumberI);
    // dxl.positionIGain(TARGET_ID7, serialnumberI);
    // dxl.positionIGain(TARGET_ID8, serialnumberI);
  }
}


void range(int a1, int a2) {
  int r = 230;
  if (a1 < a2 + r && a2 - r < a1)z = z + 1;
}

void zero() { //フリーの時、落下防止に動きを遅くするフラグをONにする
  z = 1;
  int de = 1; delay(de);
  targetPos01 = dxl.presentPosition(TARGET_ID1); delay(de);
  targetPos02 = dxl.presentPosition(TARGET_ID2); delay(de);
  targetPos04 = dxl.presentPosition(TARGET_ID4); delay(de);
  targetPos08 = dxl.presentPosition(TARGET_ID8);
  s08 = targetPos08;

  range(targetPos01, ran1);
  range(targetPos02, ran2);
  range(targetPos04, ran4);

  //range(s1,730);
  //range(s2,1000);
  //range(s4,3000);

  if (z == 4)z = 0;
  Serial.print(z);
  Serial.print(":");
}

void slow() { //条件がONの時、スローにする。ただし腕が全開の時は例外とする


  // if (s08 < exception) { //腕が全開の時の数字
  //   // digitalWrite(led01, 0);
  //   // digitalWrite(led02, HIGH);
  //   // digitalWrite(led03, 0);
  // }

  if (onlyLeftArm == true) {
    if (z > 0 && s08 < exception) {
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
      dxl.torqueEnable(TARGET_ID4, false);
      dxl.torqueEnable(TARGET_ID6, false);
      delay(de);
      dxl.torqueEnable(TARGET_ID1, true);
      dxl.torqueEnable(TARGET_ID2, true);
      dxl.torqueEnable(TARGET_ID4, true);
      dxl.torqueEnable(TARGET_ID6, true);
      delay(12);

    } else {
      dxl.torqueEnable(TARGET_ID1, false);
      dxl.torqueEnable(TARGET_ID2, false);
      dxl.torqueEnable(TARGET_ID4, false);
      dxl.torqueEnable(TARGET_ID6, false);
    }
  } else {
    if (z > 0 && s08 > exception) {
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
      dxl.torqueEnable(TARGET_ID4, false);
      dxl.torqueEnable(TARGET_ID6, false);
      delay(de);
      dxl.torqueEnable(TARGET_ID1, true);
      dxl.torqueEnable(TARGET_ID2, true);
      dxl.torqueEnable(TARGET_ID4, true);
      dxl.torqueEnable(TARGET_ID6, true);
      delay(12);

    } else {
      dxl.torqueEnable(TARGET_ID1, false);
      dxl.torqueEnable(TARGET_ID2, false);
      dxl.torqueEnable(TARGET_ID4, false);
      dxl.torqueEnable(TARGET_ID6, false);
    }
  }

}




void demo() {
  Serial.print("mode= ");//デモの時は基本10になっている。
  Serial.print(mode);
  int de = 5;
  delay(de);
  targetPos01 = dxl.presentPosition(TARGET_ID1); delay(de);
  targetPos02 = dxl.presentPosition(TARGET_ID2); delay(de);
  targetPos03 = dxl.presentPosition(TARGET_ID3); delay(de);
  targetPos04 = dxl.presentPosition(TARGET_ID4); delay(de);
  targetPos05 = dxl.presentPosition(TARGET_ID5); delay(de);
  targetPos06 = dxl.presentPosition(TARGET_ID6); delay(de);
  targetPos07 = dxl.presentPosition(TARGET_ID7); delay(de);
  targetPos08 = dxl.presentPosition(TARGET_ID8);

  Serial.print(" demo = ");
  Serial.print(targetPos01); Serial.print(", ");
  Serial.print(targetPos02); Serial.print(", ");
  Serial.print(targetPos03); Serial.print(", ");
  Serial.print(targetPos04); Serial.print(", ");
  Serial.print(targetPos05); Serial.print(", ");
  Serial.print(targetPos06); Serial.print(", ");
  Serial.print(targetPos07); Serial.print(", ");
  Serial.println(targetPos08);

  zero();
  slow();
  //Serial.print("mode= ");
  //Serial.println(mode);
}


void recordMotion() {
  DISPreset();
  numberBuffer1 = "Writer";
  DISPprint();

  TIMERDISPreset();

  if (mode < 10) {//writerはmodeが1～9
    if (mode == 1) {
      drawKeypad();
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

    for (int i = 0; i < numRecords; i++) {
      DISPwrite(String(i)+"/"+String(numRecords));
      
      UNDERDISPwrite(String(mode)+", "+String(targetPos01)+", "+String(targetPos02)+", "+String(targetPos03)+", "+String(targetPos04));

      TIMERLENGTH = i;
      TIMERDISPwrite();
    

      int de = 6; delay(de);
      targetPos01 = dxl.presentPosition(TARGET_ID1); delay(de);
      targetPos02 = dxl.presentPosition(TARGET_ID2); delay(de);
      targetPos03 = dxl.presentPosition(TARGET_ID3); delay(de);
      targetPos04 = dxl.presentPosition(TARGET_ID4); delay(de);
      targetPos05 = dxl.presentPosition(TARGET_ID5); delay(de);
      targetPos06 = dxl.presentPosition(TARGET_ID6); delay(de);
      targetPos07 = dxl.presentPosition(TARGET_ID7); delay(de);
      targetPos08 = dxl.presentPosition(TARGET_ID8);

      //Serial.print("timer = ");  Serial.println(t);

      Serial1.write(0);

      Serial.print("レコード中 = ");
      Serial.print(i); Serial.print(" : ");
      Serial.print(targetPos01); Serial.print(", ");
      Serial.print(targetPos02); Serial.print(", ");
      Serial.print(targetPos03); Serial.print(", ");
      Serial.print(targetPos04); Serial.print(", ");
      Serial.print(targetPos05); Serial.print(", ");
      Serial.print(targetPos06); Serial.print(", ");
      Serial.print(targetPos07); Serial.print(", ");
      Serial.print(targetPos08); Serial.print(", MODE=");  
      Serial.print(mode);
      Serial.print(", Audio="); Serial.print(swAudioState);

      //レコード書き出し
      file.print(targetPos01);
      file.print(",");
      file.print(targetPos02);
      file.print(",");
      file.print(targetPos03);
      file.print(",");
      file.print(targetPos04);
      file.print(",");
      file.print(targetPos05);
      file.print(",");
      file.print(targetPos06);
      file.print(",");
      file.print(targetPos07);
      file.print(",");
      file.print(targetPos08);
      file.print(",");
      file.println(0);

    }

    DISPwrite("COMPLETE");
    
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

  mode = 255;
  Serial.print(z);
  if (z == 0)mode = 256;
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

    // シリアルモニタにデータを表示&モータ実行
    for (int i = 0; i < numRecords; i++) {

      int ss1 = values[i * number];
      int ss2 = values[i * number + 1];
      int ss3 = values[i * number + 2];
      int ss4 = values[i * number + 3];
      int ss5 = values[i * number + 4];
      int ss6 = values[i * number + 5];
      int ss7 = values[i * number + 6];
      int ss8 = values[i * number + 7];

      //  delay(3);
      // Serial.print(z);
      Serial.print("フレーム ");
      Serial.print(i + 1);
      Serial.println(": ");
      // Serial.print("目標 = ");
      // Serial.print(ss1); Serial.print(", ");
      // Serial.print(ss2); Serial.print(", ");
      // Serial.print(ss3); Serial.print(", ");
      // Serial.print(ss4); Serial.print(", ");
      // Serial.print(ss5); Serial.print(", ");
      // Serial.print(ss6); Serial.print(", ");
      // Serial.print(ss7); Serial.print(", ");
      // Serial.print(ss8); Serial.print(" :");
      Serial.print("  mode= ");
      Serial.println(mode);

      DISPwrite(String(i)+"/"+String(numRecords));

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
      s8 = dxl.presentPosition(TARGET_ID8);

      // Serial.print("実測 = ");
      // Serial.print(s1); Serial.print(", ");
      // Serial.print(s2); Serial.print(", ");
      // Serial.print(s3); Serial.print(", ");
      // Serial.print(s4); Serial.print(", ");
      // Serial.print(s5); Serial.print(", ");
      // Serial.print(s6); Serial.print(", ");
      // Serial.print(s7); Serial.print(", ");
      // Serial.println(s8);

      Serial.print("差分 = ");
      Serial.print(ss1 - s1); Serial.print(", ");
      Serial.print(ss2 - s2); Serial.print(", ");
      Serial.print(ss3 - s3); Serial.print(", ");
      Serial.print(ss4 - s4); Serial.print(", ");
      Serial.print(ss5 - s5); Serial.print(", ");
      Serial.print(ss6 - s6); Serial.print(", ");
      Serial.print(ss7 - s7); Serial.print(", ");
      Serial.println(ss8 - s8);


      range(s1, ran1);
      range(s2, ran2);
      range(s4, ran4);

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

      digitalWrite(pin1, LOW);

    }

    DISPwrite("COMPLETE");

    //dxl.torqueEnable(TARGET_ID1, false);
    //dxl.torqueEnable(TARGET_ID2, false);
    dxl.torqueEnable(TARGET_ID3, false);
    //dxl.torqueEnable(TARGET_ID4, false);
    dxl.torqueEnable(TARGET_ID5, false);
    dxl.torqueEnable(TARGET_ID6, false);
    dxl.torqueEnable(TARGET_ID7, false);
    dxl.torqueEnable(TARGET_ID8, false);
    mode = 0;
    zero();
    slow();



  }

  //turnOffLed();

}



//オーディオインタフェースモード
void audioLoop() {
  
  
 // delay(10);
  O_time++;

  //Serial.print("audioMode = ");
  //Serial.println(audioMode);

  if (audioMode == 1) {
    //    digitalWrite(led01, HIGH);
    //    digitalWrite(led02, HIGH);
    //    digitalWrite(led03, HIGH);
    drawKeypad();
    delay(1000);
    audioMode = 2;
  }

  swAudioState = digitalRead(swAudio);  //場所を変えない

  if (audioMode == 2) {
    //    digitalWrite(led01, HIGH);
    //    digitalWrite(led02, LOW);
    //    digitalWrite(led03, LOW);
    drawKeypad();
    if (swAudioState == 0) {
      mode = 11;
      audioMode = -1;
      O_time = 0;
  //    Serial.print("mode= ");  
  //    Serial.println(mode);
      playMotion();
    }
  }

  if (audioMode == 3) {
    //    digitalWrite(led01, LOW);
    //    digitalWrite(led02, HIGH);
    //    digitalWrite(led03, LOW);
    drawKeypad();
    if (swAudioState == 0) {
      mode = 12;
      audioMode = -1;
      O_time = 0;
   //   Serial.print("mode= ");  
   //   Serial.println(mode);
      playMotion();
    }
  }

  if (audioMode == 4) {
    //    digitalWrite(led01, LOW);
    //    digitalWrite(led02, LOW);
    //    digitalWrite(led03, HIGH);
    drawKeypad();
    if (swAudioState == 0) {
      mode = 13;
      audioMode = -1;
      O_time = 0;
  //    Serial.print("mode= ");  
  //    Serial.println(mode);
      playMotion();
    }
  }

  if (audioMode == 5) {
    //    digitalWrite(led01, LOW);
    //    digitalWrite(led02, LOW);
    //    digitalWrite(led03, HIGH);
    drawKeypad();
    if (swAudioState == 0) {
      mode = 14;
      audioMode = -1;
      O_time = 0;
  //    Serial.print("mode= ");  
  //    Serial.println(mode);
      playMotion();
    }
  }

  if (audioMode == 6) {
    //    digitalWrite(led01, LOW);
    //    digitalWrite(led02, LOW);
    //    digitalWrite(led03, HIGH);
    drawKeypad();
    if (swAudioState == 0) {
      mode = 15;
      audioMode = -1;
      O_time = 0;
  //    Serial.print("mode= ");  
  //    Serial.println(mode);
      playMotion();
    }
  }

  if (audioMode == 7) {
    //    digitalWrite(led01, LOW);
    //    digitalWrite(led02, LOW);
    //    digitalWrite(led03, HIGH);
    drawKeypad();
    if (swAudioState == 0) {
      mode = 16;
      audioMode = -1;
      O_time = 0;
  //    Serial.print("mode= ");  
  //    Serial.println(mode);
      playMotion();
    }
  }

  if (audioMode == 8) {
    audioMode = 0;
    drawKeypad();    
    //    turnOffLed();
  }

  // if (audioMode == -1){
  //   if (swAudioState == HIGH){
  //     audioMode = 0;
  //     O_time = 0;
  //   }
  // }

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
    demo();//デモを再生せよ
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

    dxl.torqueEnable(TARGET_ID1, true);
    dxl.torqueEnable(TARGET_ID2, true);
    dxl.torqueEnable(TARGET_ID3, true);
    dxl.torqueEnable(TARGET_ID4, true);
    dxl.torqueEnable(TARGET_ID5, true);
    dxl.torqueEnable(TARGET_ID6, true);
    dxl.torqueEnable(TARGET_ID7, true);
    dxl.torqueEnable(TARGET_ID8, true);


  }
}




void setup() {

  DYNAMIXEL_SERIAL.begin(1000000);
  dxl.attach(DYNAMIXEL_SERIAL, 1000000);
  Serial.begin(115200);
  SerialBT.begin("OryArm"); // Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

  dxl.addModel<DxlModel::X>(TARGET_ID1);
  dxl.addModel<DxlModel::X>(TARGET_ID2);
  dxl.addModel<DxlModel::X>(TARGET_ID3);
  dxl.addModel<DxlModel::X>(TARGET_ID4);
  dxl.addModel<DxlModel::X>(TARGET_ID5);
  dxl.addModel<DxlModel::X>(TARGET_ID6);
  dxl.addModel<DxlModel::X>(TARGET_ID7);
  dxl.addModel<DxlModel::X>(TARGET_ID8);

  Serial.println(VERSION_NUMBER);
  delay(2000);

  dxl.torqueEnable(TARGET_ID1, false);
  dxl.torqueEnable(TARGET_ID2, false);
  dxl.torqueEnable(TARGET_ID3, false);
  dxl.torqueEnable(TARGET_ID4, false);
  dxl.torqueEnable(TARGET_ID5, false);
  dxl.torqueEnable(TARGET_ID6, false);
  dxl.torqueEnable(TARGET_ID7, false);
  dxl.torqueEnable(TARGET_ID8, false);

  Pgain_on();

  ran1 = dxl.presentPosition(TARGET_ID1); delay(5);
  ran2 = dxl.presentPosition(TARGET_ID2); delay(5);
  ran4 = dxl.presentPosition(TARGET_ID4); delay(5);

  Serial.print(ran1);
  Serial.print(",");
  Serial.print(ran2);
  Serial.print(",");
  Serial.println(ran4);

  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(pin3, OUTPUT);

  //turnOffLed();
  //mode = 10;

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
  Serial.println("mode= " + mode);
  Serial.print("sw01State= " + sw01State);

  // timer = timerBegin(0, 80, true);
	// timerAttachInterrupt(timer, &onTimer, true);
	// timerAlarmWrite(timer, 33333, true);  //毎秒30回ポジションを保存する
	// timerAlarmEnable(timer);

  pinMode(swAudio, INPUT_PULLUP);

  
  Serial.println("setup done");
  
}








void loop(void) {

  if (mainloop == false){
    digitalWrite(pin2, LOW);
    mainloop = true;
  } else {
    digitalWrite(pin2, HIGH);
    mainloop = false;
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

    if (key[b].justReleased()) key[b].drawButton();     // draw normal

    if (key[b].justPressed()) {
      key[b].drawButton(true);  // draw invert
      drawKeypad();// Draw keypad


      if (b == 0) {
        DISPwrite("RUN b=0");
        mode = 10;
      }

      if (b == 1) {
        DISPwrite("MODE b=1");
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

    }
    //DISPprint();
  }
  armloop();

  int swAudioState = digitalRead(swAudio);
  if (swAudioState == 0) {  // Assuming the switch pulls the pin LOW when pressed
    Serial.print("Audio Switch ON  ");
  } else {
    Serial.print("Audio Switch OFF  ");
  }
  if (swAudioState == 0) {
    audioMode = 1;
  }
  Serial.print("  audioMode= ");
  Serial.print(audioMode);
  Serial.print("  swAudioState= ");
  Serial.print(swAudioState);

}