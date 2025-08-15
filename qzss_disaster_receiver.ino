//
// M5Stack Basic (M5Stack Core)で災危通報を表示
// 2025/08/02 @ksasao
// 
// ソースコードは
// https://www.switch-science.com/blogs/magazine/gps-qzss-dc-report-dcx-receiving の記事にある
// https://github.com/SWITCHSCIENCE/samplecodes/tree/master/GPS_shield_for_ESPr/espr_dev_qzss_drc_drx_decode
// を元に画面表示を追加したものです。
// 機材などの詳細は https://x.com/ksasao/status/1951457364667932775 を参照してください

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <QZQSM.h>
#include "QZSSDCX.h"
#include <M5Unified.h>
SFE_UBLOX_GNSS myGNSS;

const int rxPin = 16;
const int txPin = 17;

#define DBG_PRINT_SFRBX 0
#define DBG_PRINT_SAT 0
#define DBG_PRINT_PVT 0
#define DBG_PRINT_DCX_ALL 0

byte l1s_msg_buf[32];  // MAX 250 BITS
QZQSM dc_report;
DCXDecoder dcx_decoder;

// 衛星情報を格納する構造体
struct SatelliteInfo {
  uint8_t gnssId;
  uint8_t svId;
  uint8_t cno;
  bool isQZSS;
  bool isL1S;
};

#define MAX_SATELLITES 20
SatelliteInfo satellites[MAX_SATELLITES];
int satelliteCount = 0;
unsigned long lastUpdateTime = 0;

// dwrdを16進数文字列に変換して出力する関数
const char *dwrd_to_str(uint32_t value) {
  static const char hex_chars[] = "0123456789ABCDEF";  // 16進数文字
  static char buffer[9];                               // 8桁 + 終端文字
  // リトルエンディアンなので入れ替える
  buffer[8] = '\0';
  buffer[7] = hex_chars[value & 0xF];
  buffer[6] = hex_chars[value >> 4 & 0xF];
  buffer[5] = hex_chars[value >> 8 & 0xF];
  buffer[4] = hex_chars[value >> 12 & 0xF];
  buffer[3] = hex_chars[value >> 16 & 0xF];
  buffer[2] = hex_chars[value >> 20 & 0xF];
  buffer[1] = hex_chars[value >> 24 & 0xF];
  buffer[0] = hex_chars[value >> 28 & 0xF];
  return buffer;
}

// 衛星情報を画面に表示する関数
void displaySatelliteInfo() {
  M5.Lcd.fillScreen(0);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(1);
  
  // タイトル表示
  M5.Lcd.println("QZSS衛星情報監視中");
  M5.Lcd.println("==================");
  
  if (satelliteCount == 0) {
    M5.Lcd.println("衛星信号を受信中...");
    M5.Lcd.println("アンテナの向きを確認してください");
    return;
  }
  
  // QZSS衛星の情報を表示
  int qzssCount = 0;
  for (int i = 0; i < satelliteCount; i++) {
    if (satellites[i].isQZSS) {
      qzssCount++;
    }
  }
  
  M5.Lcd.printf("QZSS衛星: %d機受信中\n", qzssCount);
  M5.Lcd.println("------------------");
  
  // QZSS衛星の詳細情報
  for (int i = 0; i < satelliteCount && i < 8; i++) { // 画面に収まるよう8機まで表示
    if (satellites[i].isQZSS) {
      M5.Lcd.printf("QZSS-%02d: ", satellites[i].svId);
      
      // 信号強度をバーで表示
      int bars = satellites[i].cno / 5; // 信号強度を5段階で表示
      for (int j = 0; j < bars && j < 10; j++) {
        M5.Lcd.print("|");
      }
      M5.Lcd.printf(" (%ddB)\n", satellites[i].cno);
      
      // L1S信号受信状況
      if (satellites[i].isL1S) {
        M5.Lcd.println("  L1S信号: 受信中");
      }
    }
  }
  
  // その他の衛星システム情報
  M5.Lcd.println("------------------");
  M5.Lcd.println("その他の衛星:");
  
  int gpsCount = 0, galileoCount = 0, beidouCount = 0;
  for (int i = 0; i < satelliteCount; i++) {
    if (!satellites[i].isQZSS) {
      switch (satellites[i].gnssId) {
        case 0: gpsCount++; break;      // GPS
        case 2: galileoCount++; break;  // Galileo
        case 3: beidouCount++; break;   // BeiDou
      }
    }
  }
  
  if (gpsCount > 0) M5.Lcd.printf("GPS: %d機\n", gpsCount);
  if (galileoCount > 0) M5.Lcd.printf("Galileo: %d機\n", galileoCount);
  if (beidouCount > 0) M5.Lcd.printf("BeiDou: %d機\n", beidouCount);
  
  // 更新時刻
  M5.Lcd.println("------------------");
  M5.Lcd.printf("更新: %02d:%02d:%02d", 
    (millis() / 60000) % 60, 
    (millis() / 1000) % 60, 
    (millis() / 10) % 100);
}

void newSFRBX(UBX_RXM_SFRBX_data_t *data) {
#if DBG_PRINT_SFRBX
  Serial.print("SFRBX gnssId: ");
  Serial.print(data->gnssId);
  Serial.print(" svId: ");
  Serial.print(data->svId);
  Serial.print(" freqId: ");
  Serial.print(data->freqId);
  Serial.print(" numWords: ");
  Serial.print(data->numWords);
  Serial.print(" version: ");
  Serial.print(data->version);
  Serial.print(" ");
  for (int i = 0; i < data->numWords; i++) {
    Serial.print(dwrd_to_str(data->dwrd[i]));
  }
  Serial.println();
#endif

  // QZSS L1Sメッセージ解析
  if (data->gnssId == 5) {
    Serial.println("[L1S Message]");
    // SFRBXのdwrdはリトルエンディアンなので入れ替える
    for (int i = 0; i < min(int(data->numWords), 8); i++) {
      l1s_msg_buf[(i << 2) + 0] = (data->dwrd[i] >> 24) & 0xff;
      l1s_msg_buf[(i << 2) + 1] = (data->dwrd[i] >> 16) & 0xff;
      l1s_msg_buf[(i << 2) + 2] = (data->dwrd[i] >> 8) & 0xff;
      l1s_msg_buf[(i << 2) + 3] = (data->dwrd[i]) & 0xff;
    }

    byte pab = l1s_msg_buf[0];
    byte mt = l1s_msg_buf[1] >> 2;

    if (pab == 0x53 || pab == 0x9A || pab == 0xC6) {
      // Message Typeを表示
      struct {
        byte mt;
        const char *desc;
      } MTTable[] = {
        { 0, "Test Mode" },
        { 43, "DC Report" },
        { 44, "DCX message" },
        { 47, "Monitoring Station Information" },
        { 48, "PRN Mask" },
        { 49, "Data Issue Number" },
        { 50, "DGPS Correction" },
        { 51, "Satellite Health" },
        { 63, "Null message" },
      };
      for (int i = 0; i < sizeof(MTTable) / sizeof(MTTable[0]); i++) {
        if (MTTable[i].mt == mt) {
          Serial.print(mt);
          Serial.print(" ");
          Serial.println(MTTable[i].desc);
          break;
        }
      }
      // 災害・危機管理通報サービス（DC Report）のメッセージ内容を表示
      if (mt == 43) {
        dc_report.SetYear(2025);  // todo
        dc_report.Decode(l1s_msg_buf);
        Serial.println(dc_report.GetReport());
        M5.Lcd.fillScreen(RED);
        M5.Lcd.setTextColor(WHITE);
        M5.Lcd.setCursor(0,0);
        M5.Lcd.setTextSize(1);
        M5.Lcd.print("【災害通報受信】\n");
        M5.Lcd.print(dc_report.GetReport());
      }
      // 災害・危機管理通報サービス（拡張）（DCX）のメッセージ内容を表示
      else if (mt == 44) {
        dcx_decoder.decode(l1s_msg_buf);
        dcx_decoder.printSummary(Serial, dcx_decoder.r);
#if DBG_PRINT_DCX_ALL
        dcx_decoder.printAll(Serial, dcx_decoder.r);
#endif
        M5.Lcd.fillScreen(ORANGE);
        M5.Lcd.setTextColor(WHITE);
        M5.Lcd.setCursor(0,0);
        M5.Lcd.setTextSize(1);
        M5.Lcd.print("【DCX通報受信】\n");
        M5.Lcd.print("詳細はシリアル出力を確認");
      }
    }
  }
}

void newNAVSAT(UBX_NAV_SAT_data_t *data) {
  // 衛星情報を更新
  satelliteCount = 0;
  
  for (uint16_t block = 0; block < data->header.numSvs && satelliteCount < MAX_SATELLITES; block++) {
    satellites[satelliteCount].gnssId = data->blocks[block].gnssId;
    satellites[satelliteCount].svId = data->blocks[block].svId;
    satellites[satelliteCount].cno = data->blocks[block].cno;
    satellites[satelliteCount].isQZSS = (data->blocks[block].gnssId == 5);
    satellites[satelliteCount].isL1S = false; // L1S信号は別途判定
    
    satelliteCount++;
  }
  
  // 画面更新（1秒間隔）
  if (millis() - lastUpdateTime > 1000) {
    displaySatelliteInfo();
    lastUpdateTime = millis();
  }

#define NUM_GNSS 7
  int nGNSS[NUM_GNSS] = { 0 };
  for (uint16_t block = 0; block < data->header.numSvs; block++) {
    if (data->blocks[block].gnssId < NUM_GNSS) {
      nGNSS[data->blocks[block].gnssId]++;
    }
  }
  Serial.print(F("Satellites: "));
  Serial.print(data->header.numSvs);
  const char *gnssName[] = { "GPS", "SBAS", "Galileo", "BeiDou", "IMES", "QZSS", "GLONASS" };
  for (uint16_t i = 0; i < NUM_GNSS; i++) {
    if (nGNSS[i]) {
      Serial.print(" ");
      Serial.print(gnssName[i]);
      Serial.print(": ");
      Serial.print(nGNSS[i]);
    }
  }
  Serial.println();

#if DBG_PRINT_SAT
  // 衛星の種類を表示
  for (uint16_t block = 0; block < data->header.numSvs; block++) {
    switch (data->blocks[block].gnssId) {
      case 0:
        Serial.print(F("GPS     "));
        break;
      case 1:
        Serial.print(F("SBAS    "));
        break;
      case 2:
        Serial.print(F("Galileo "));
        break;
      case 3:
        Serial.print(F("BeiDou  "));
        break;
      case 4:
        Serial.print(F("IMES    "));
        break;
      case 5:
        Serial.print(F("QZSS    "));
        break;
      case 6:
        Serial.print(F("GLONASS "));
        break;
      default:
        Serial.print(F("UNKNOWN "));
        break;
    }

    // 衛星番号を表示
    Serial.print(data->blocks[block].svId);

    if (data->blocks[block].svId < 10) Serial.print(F("   "));
    else if (data->blocks[block].svId < 100) Serial.print(F("  "));
    else Serial.print(F(" "));

    // 信号の強さを表示
    for (uint8_t cno = 0; cno < data->blocks[block].cno; cno++)
      Serial.print(F("|"));

    Serial.println();
  }
#endif
}

void newNAVPVT(UBX_NAV_PVT_data_t *data) {
#if DBG_PRINT_PVT
  // 時刻表示
  Serial.print(F("Time: "));
  uint8_t hms = data->hour;
  if (hms < 10) Serial.print(F("0"));
  Serial.print(hms);
  Serial.print(F(":"));
  hms = data->min;
  if (hms < 10) Serial.print(F("0"));
  Serial.print(hms);
  Serial.print(F(":"));
  hms = data->sec;
  if (hms < 10) Serial.print(F("0"));
  Serial.print(hms);
  Serial.print(F("."));
  unsigned long millisecs = data->iTOW % 1000;
  if (millisecs < 100) Serial.print(F("0"));
  if (millisecs < 10) Serial.print(F("0"));
  Serial.print(millisecs);

  // 経度・緯度・高度表示
  long latitude = data->lat;
  Serial.print(F(" Lat: "));
  Serial.print(latitude);

  long longitude = data->lon;
  Serial.print(F(" Long: "));
  Serial.print(longitude);
  Serial.print(F(" (degrees * 10^-7)"));

  long altitude = data->hMSL;
  Serial.print(F(" Height above MSL: "));
  Serial.print(altitude);
  Serial.print(F(" (mm)"));

  Serial.print(F(" SIV: "));
  Serial.print(data->numSV);

  Serial.print(F(" Fix: "));
  Serial.print(data->fixType);

  Serial.println();
#endif
}

void setup() {
  auto cfg = M5.config();       // M5Stack初期設定用の構造体を代入
  M5.begin(cfg); 

  M5.Lcd.setTextFont(&fonts::efontJA_16);
  M5.Lcd.setCursor(0,0);
  M5.Display.setTextSize(1);
  
  M5.Lcd.println("QZSS衛星情報監視開始");
  M5.Lcd.println("衛星信号を待機中...");

  Serial.begin(115200);
  while (!Serial)
    ;  //ターミナルを開くまでまつ

  Serial1.begin(9600, SERIAL_8N1, rxPin, txPin);

  // myGNSS.enableDebugging(); // デバッグメッセージをSerialに出力する
  if (myGNSS.begin(Serial1) == false)  // Serial1を介してu-bloxモジュールに接続する
  {
    // ボーレートを変えてトライ
    Serial1.begin(38400, SERIAL_8N1, rxPin, txPin);
    if (myGNSS.begin(Serial1) == false) {
      Serial.println(F("u-blox GNSS not detected. Please check wiring. Freezing."));
      M5.Lcd.fillScreen(RED);
      M5.Lcd.setTextColor(WHITE);
      M5.Lcd.setCursor(0,0);
      M5.Lcd.println("GNSSモジュール未検出");
      M5.Lcd.println("配線を確認してください");
      while (1)
        ;
    }
  }

  myGNSS.setUART1Output(COM_TYPE_UBX);           // UART1から出力をUBXのみに設定する
  enableQZSSL1S();                               // QZSS L1S信号の受信を有効にする
  myGNSS.setAutoRXMSFRBXcallbackPtr(&newSFRBX);  // UBX-RXM-SFRBXメッセージ受信コールバック関数を登録
  myGNSS.setAutoNAVSATcallbackPtr(&newNAVSAT);   // UBX-NAV-SATメッセージ受信コールバック関数を登録
  myGNSS.setAutoPVTcallbackPtr(&newNAVPVT);      // UBX-NAV-PVTメッセージ受信コールバック関数を登録
  
  // 初期画面表示
  displaySatelliteInfo();
}

// QZSSのL1S信号を受信するよう設定する
bool enableQZSSL1S(void) {
  uint8_t customPayload[MAX_PAYLOAD_SIZE];
  ubxPacket customCfg = { 0, 0, 0, 0, 0, customPayload, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED };

  customCfg.cls = UBX_CLASS_CFG;
  customCfg.id = UBX_CFG_GNSS;
  customCfg.len = 0;
  customCfg.startingSpot = 0;

  if (myGNSS.sendCommand(&customCfg) != SFE_UBLOX_STATUS_DATA_RECEIVED)
    return (false);

  int numConfigBlocks = customPayload[3];
  for (int block = 0; block < numConfigBlocks; block++) {
    if (customPayload[(block * 8) + 4] == (uint8_t)SFE_UBLOX_GNSS_ID_QZSS) {
      customPayload[(block * 8) + 8] |= 0x01;      // set enable bit
      customPayload[(block * 8) + 8 + 2] |= 0x05;  // set 0x01 QZSS L1C/A 0x04 = QZSS L1S
    }
  }

  return (myGNSS.sendCommand(&customCfg) == SFE_UBLOX_STATUS_DATA_SENT);
}

void loop() {
  myGNSS.checkUblox();
  myGNSS.checkCallbacks();
  
  // M5Stackのボタン処理
  M5.update();
  if (M5.BtnA.wasPressed()) {
    // ボタンAで画面更新
    displaySatelliteInfo();
  }
}
