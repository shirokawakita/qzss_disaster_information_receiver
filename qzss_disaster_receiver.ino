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

#define DBG_PRINT_SFRBX 1
#define DBG_PRINT_SAT 1
#define DBG_PRINT_PVT 1
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

// L1S信号統計
unsigned long l1sMessageCount = 0;
unsigned long lastL1SMessageTime = 0;

// DC通報保存用の構造体
struct DCReport {
  unsigned long timestamp;
  uint8_t messageType;
  uint8_t svId;
  uint8_t numWords;
  uint32_t words[8];  // 最大8 words
  bool isValid;
};

#define MAX_DC_REPORTS 10
DCReport dcReports[MAX_DC_REPORTS];
int dcReportCount = 0;

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
  M5.Lcd.setTextFont(&fonts::efontJA_16);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(1);
  
  // タイトル表示
  M5.Lcd.println("QZSS L1S信号監視中");
  M5.Lcd.println("==================");
  
  if (satelliteCount == 0) {
    M5.Lcd.println("衛星信号を受信中...");
    M5.Lcd.println("アンテナの向きを確認してください");
    M5.Lcd.println("");
    M5.Lcd.println("DIPスイッチ設定:");
    M5.Lcd.println("TX: 1=ON,2=OFF,3=OFF,4=OFF");
    M5.Lcd.println("RX: 1=ON,2=OFF,3=OFF,4=OFF");
    M5.Lcd.println("PPS: 1=OFF (推奨)");
    return;
  }
  
  // QZSS衛星の情報を表示
  int qzssCount = 0;
  int qzssL1SReady = 0;
  int qzssL1SReceiving = 0;
  int qzssMaxCno = 0;
  
  for (int i = 0; i < satelliteCount; i++) {
    if (satellites[i].isQZSS) {
      qzssCount++;
      if (satellites[i].cno > qzssMaxCno) {
        qzssMaxCno = satellites[i].cno;
      }
      // L1S信号受信可能な衛星をカウント（信号強度が15dB以上）
      if (satellites[i].cno >= 15) {
        qzssL1SReady++;
      }
      // 実際にL1S信号を受信中の衛星をカウント
      if (satellites[i].isL1S) {
        qzssL1SReceiving++;
      }
    }
  }
  
  // L1S信号を受信したが衛星情報にない場合の処理
  // 保存されたDC通報からQZSS衛星情報を復元
  for (int i = 0; i < dcReportCount; i++) {
    if (dcReports[i].isValid && 
        (dcReports[i].messageType == 43 || dcReports[i].messageType == 44)) {
      // 最近の通報（5分以内）の場合、QZSS衛星としてカウント
      unsigned long timeSince = (millis() - dcReports[i].timestamp) / 1000;
      if (timeSince < 300) { // 5分 = 300秒
        qzssCount++;
        qzssL1SReceiving++;
        if (qzssMaxCno == 0) {
          qzssMaxCno = 20; // 推定信号強度
        }
      }
    }
  }
  
  // QZSS概要表示
  M5.Lcd.printf("QZSS衛星: %d機受信中\n", qzssCount);
  if (qzssCount > 0) {
    M5.Lcd.printf("L1S受信可能: %d機\n", qzssL1SReady);
    M5.Lcd.printf("L1S受信中: %d機\n", qzssL1SReceiving);
    M5.Lcd.printf("最高信号強度: %ddB\n", qzssMaxCno);
  }
  
  // QZSS衛星の詳細情報
  if (qzssCount > 0) {
    M5.Lcd.println("------------------");
    M5.Lcd.println("QZSS詳細:");
    
    // 通常の衛星情報を表示
    for (int i = 0; i < satelliteCount; i++) {
      if (satellites[i].isQZSS) {
        M5.Lcd.printf("QZSS-%02d: ", satellites[i].svId);
        
        // 信号強度をバーで表示
        if (satellites[i].cno > 0) {
          int bars = satellites[i].cno / 3; // 信号強度を3dB刻みで表示
          for (int j = 0; j < bars && j < 15; j++) {
            if (j < 5) {
              M5.Lcd.setTextColor(RED);    // 弱い信号は赤
            } else if (j < 10) {
              M5.Lcd.setTextColor(YELLOW); // 中程度は黄色
            } else {
              M5.Lcd.setTextColor(GREEN);  // 強い信号は緑
            }
            M5.Lcd.print("|");
          }
          M5.Lcd.setTextColor(WHITE);
          M5.Lcd.printf(" %ddB", satellites[i].cno);
          
          // L1S信号の状態を表示
          if (satellites[i].isL1S) {
            M5.Lcd.setTextColor(GREEN);
            M5.Lcd.print(" [L1S受信中]");
          } else if (satellites[i].cno >= 15) {
            M5.Lcd.setTextColor(CYAN);
            M5.Lcd.print(" [L1S受信可能]");
          } else if (satellites[i].cno >= 10) {
            M5.Lcd.setTextColor(YELLOW);
            M5.Lcd.print(" [L1S弱い]");
          } else {
            M5.Lcd.setTextColor(RED);
            M5.Lcd.print(" [L1S不可]");
          }
          M5.Lcd.setTextColor(WHITE);
        } else {
          M5.Lcd.setTextColor(RED);
          M5.Lcd.print("信号なし (0dB) [L1S不可]");
          M5.Lcd.setTextColor(WHITE);
        }
        M5.Lcd.println();
      }
    }
    
    // 保存されたDC通報からL1S受信衛星を表示
    for (int i = 0; i < dcReportCount; i++) {
      if (dcReports[i].isValid && 
          (dcReports[i].messageType == 43 || dcReports[i].messageType == 44)) {
        unsigned long timeSince = (millis() - dcReports[i].timestamp) / 1000;
        if (timeSince < 300) { // 5分以内
          M5.Lcd.printf("QZSS-%02d: ", dcReports[i].svId);
          M5.Lcd.setTextColor(CYAN);
          M5.Lcd.print("L1S受信済み ");
          M5.Lcd.setTextColor(WHITE);
          M5.Lcd.printf("(%lus前)", timeSince);
          M5.Lcd.println();
        }
      }
    }
  }
  
  // その他の衛星システム情報（信号強度付き）
  M5.Lcd.println("------------------");
  M5.Lcd.println("その他の衛星:");
  
  // 各システムの最高信号強度を記録
  int gpsMaxCno = 0, galileoMaxCno = 0, beidouMaxCno = 0, glonassMaxCno = 0;
  int gpsCount = 0, galileoCount = 0, beidouCount = 0, glonassCount = 0;
  
  for (int i = 0; i < satelliteCount; i++) {
    if (!satellites[i].isQZSS) {
      switch (satellites[i].gnssId) {
        case 0: // GPS
          gpsCount++;
          if (satellites[i].cno > gpsMaxCno) gpsMaxCno = satellites[i].cno;
          break;
        case 2: // Galileo
          galileoCount++;
          if (satellites[i].cno > galileoMaxCno) galileoMaxCno = satellites[i].cno;
          break;
        case 3: // BeiDou
          beidouCount++;
          if (satellites[i].cno > beidouMaxCno) beidouMaxCno = satellites[i].cno;
          break;
        case 6: // GLONASS
          glonassCount++;
          if (satellites[i].cno > glonassMaxCno) glonassMaxCno = satellites[i].cno;
          break;
      }
    }
  }
  
  if (gpsCount > 0) {
    M5.Lcd.printf("GPS: %d機 (最大%d dB)\n", gpsCount, gpsMaxCno);
  }
  if (galileoCount > 0) {
    M5.Lcd.printf("Galileo: %d機 (最大%d dB)\n", galileoCount, galileoMaxCno);
  }
  if (beidouCount > 0) {
    M5.Lcd.printf("BeiDou: %d機 (最大%d dB)\n", beidouCount, beidouMaxCno);
  }
  if (glonassCount > 0) {
    M5.Lcd.printf("GLONASS: %d機 (最大%d dB)\n", glonassCount, glonassMaxCno);
  }
  
  // 更新時刻
  M5.Lcd.println("------------------");
  M5.Lcd.printf("更新: %02d:%02d:%02d", 
    (millis() / 60000) % 60, 
    (millis() / 1000) % 60, 
    (millis() / 10) % 100);
  
  // 総衛星数表示
  M5.Lcd.printf("\n総衛星数: %d機", satelliteCount);
  
  // L1S信号の状態サマリー
  if (qzssL1SReceiving > 0) {
    M5.Lcd.setTextColor(CYAN);
    M5.Lcd.printf("\nL1S信号: 受信済み (%d機)", qzssL1SReceiving);
  } else if (qzssL1SReady > 0) {
    M5.Lcd.setTextColor(GREEN);
    M5.Lcd.printf("\nL1S信号: 受信可能 (%d機)", qzssL1SReady);
  } else if (qzssCount > 0) {
    M5.Lcd.setTextColor(YELLOW);
    M5.Lcd.printf("\nL1S信号: 待機中 (%d機)", qzssCount);
  } else {
    M5.Lcd.setTextColor(RED);
    M5.Lcd.printf("\nL1S信号: 未検出");
  }
  M5.Lcd.setTextColor(WHITE);
  
  // L1S統計情報
  if (l1sMessageCount > 0) {
    M5.Lcd.printf("\nL1S受信: %lu回", l1sMessageCount);
    if (lastL1SMessageTime > 0) {
      unsigned long timeSinceLast = (millis() - lastL1SMessageTime) / 1000;
      M5.Lcd.printf(" (最終: %lus前)", timeSinceLast);
    }
  }
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
    l1sMessageCount++;
    lastL1SMessageTime = millis();
    Serial.println("=== QZSS L1S MESSAGE DETECTED ===");
    Serial.print("[L1S Message] SV:");
    Serial.print(data->svId);
    Serial.print(" Freq:");
    Serial.print(data->freqId);
    Serial.print(" Words:");
    Serial.print(data->numWords);
    Serial.println();
    
    // L1S信号の頻度IDを確認（L1S信号は通常freqId=0）
    if (data->freqId == 0) {
      Serial.println("Confirmed L1S signal (freqId=0)");
    }
    
    // L1S信号受信を衛星情報に記録
    bool found = false;
    for (int i = 0; i < satelliteCount; i++) {
      if (satellites[i].isQZSS && satellites[i].svId == data->svId) {
        satellites[i].isL1S = true;
        Serial.printf("QZSS-%02d: L1S信号受信確認\n", data->svId);
        found = true;
        break;
      }
    }
    
    // 衛星情報にない場合は新しく追加
    if (!found && satelliteCount < MAX_SATELLITES) {
      satellites[satelliteCount].gnssId = 5; // QZSS
      satellites[satelliteCount].svId = data->svId;
      satellites[satelliteCount].cno = 0; // 信号強度は不明
      satellites[satelliteCount].isQZSS = true;
      satellites[satelliteCount].isL1S = true;
      Serial.printf("QZSS-%02d: 新規L1S信号受信確認\n", data->svId);
      satelliteCount++;
    }
    
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
          Serial.print("Message Type: ");
          Serial.print(mt);
          Serial.print(" (");
          Serial.print(MTTable[i].desc);
          Serial.println(")");
          break;
        }
      }
      // 災害・危機管理通報サービス（DC Report）のメッセージ内容を表示
      if (mt == 43) {
        // DC Report（災害・危機管理通報サービス）の日本語解析・表示
        Serial.println("=== DC Report Detected ===");
        Serial.print("Message Type: ");
        Serial.println(mt);
        
        // QZQSMライブラリで日本語解析
        dc_report.SetYear(2025);
        dc_report.Decode(l1s_msg_buf);
        String reportText = dc_report.GetReport();
        Serial.println("DC Report Content:");
        Serial.println(reportText);
        
        // DC通報を保存
        saveDCReport(mt, data->svId, data->numWords, data->dwrd);
        
        // 災害通報受信時の画面表示
        M5.Lcd.fillScreen(RED);
        M5.Lcd.setTextFont(&fonts::efontJA_16);
        M5.Lcd.setTextColor(WHITE);
        M5.Lcd.setCursor(0,0);
        M5.Lcd.setTextSize(1);
        M5.Lcd.print("【災害通報受信】\n");
        M5.Lcd.print("DC Report (Type 43)\n");
        M5.Lcd.print("受信時刻: ");
        M5.Lcd.print(millis() / 1000);
        M5.Lcd.print("秒\n");
        M5.Lcd.print("衛星ID: QZSS-");
        M5.Lcd.print(data->svId);
        M5.Lcd.print("\n");
        M5.Lcd.print("メッセージ長: ");
        M5.Lcd.print(data->numWords);
        M5.Lcd.print(" words\n");
        
        // 日本語の通報内容を表示
        M5.Lcd.print("通報内容:\n");
        M5.Lcd.print(reportText);
        
        M5.Lcd.print("\n3秒後に通常表示に戻ります");
        
        // 3秒間保持
        delay(3000);
      }
      // 災害・危機管理通報サービス（拡張）（DCX）のメッセージ内容を表示
      else if (mt == 44) {
        // DCX（災害・危機管理通報サービス拡張）の日本語解析・表示
        Serial.println("=== DCX Message Detected ===");
        Serial.print("Message Type: ");
        Serial.println(mt);
        
        // QZSSDCXライブラリで日本語解析
        dcx_decoder.decode(l1s_msg_buf);
        dcx_decoder.printSummary(Serial, dcx_decoder.r);
        
        // DCX通報を保存
        saveDCReport(mt, data->svId, data->numWords, data->dwrd);
        
        // DCX通報受信時の画面表示
        M5.Lcd.fillScreen(ORANGE);
        M5.Lcd.setTextFont(&fonts::efontJA_16);
        M5.Lcd.setTextColor(WHITE);
        M5.Lcd.setCursor(0,0);
        M5.Lcd.setTextSize(1);
        M5.Lcd.print("【DCX通報受信】\n");
        M5.Lcd.print("DCX Message (Type 44)\n");
        M5.Lcd.print("受信時刻: ");
        M5.Lcd.print(millis() / 1000);
        M5.Lcd.print("秒\n");
        M5.Lcd.print("衛星ID: QZSS-");
        M5.Lcd.print(data->svId);
        M5.Lcd.print("\n");
        M5.Lcd.print("メッセージ長: ");
        M5.Lcd.print(data->numWords);
        M5.Lcd.print(" words\n");
        
        // DCXの詳細情報を表示
        M5.Lcd.print("通報内容:\n");
        M5.Lcd.print("詳細はシリアル出力を確認");
        
        M5.Lcd.print("\n3秒後に通常表示に戻ります");
        
        // 3秒間保持
        delay(3000);
      }
    }
  }
}

void newNAVSAT(UBX_NAV_SAT_data_t *data) {
  // 衛星情報を更新
  int oldCount = satelliteCount;
  satelliteCount = 0;
  
  for (uint16_t block = 0; block < data->header.numSvs && satelliteCount < MAX_SATELLITES; block++) {
    satellites[satelliteCount].gnssId = data->blocks[block].gnssId;
    satellites[satelliteCount].svId = data->blocks[block].svId;
    satellites[satelliteCount].cno = data->blocks[block].cno;
    satellites[satelliteCount].isQZSS = (data->blocks[block].gnssId == 5);
    satellites[satelliteCount].isL1S = false; // L1S信号は別途判定（SFRBXで更新）
    
    satelliteCount++;
  }
  
  // デバッグ: 衛星情報更新をログ
  if (oldCount != satelliteCount) {
    Serial.print("Satellite count changed: ");
    Serial.print(oldCount);
    Serial.print(" -> ");
    Serial.println(satelliteCount);
  }
  
  // 画面更新（3秒間隔）
  if (millis() - lastUpdateTime > 3000) {
    Serial.println("Updating display...");
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
  // 衛星情報をシリアルに出力（文字化け防止）
  Serial.print("Satellites: ");
  Serial.print(data->header.numSvs);
  Serial.print(" | ");
  
  const char *gnssName[] = { "GPS", "SBAS", "Galileo", "BeiDou", "IMES", "QZSS", "GLONASS" };
  for (uint16_t i = 0; i < NUM_GNSS; i++) {
    if (nGNSS[i]) {
      Serial.print(gnssName[i]);
      Serial.print(":");
      Serial.print(nGNSS[i]);
      Serial.print(" ");
    }
  }
  Serial.println();
  
     // 全衛星の詳細情報を出力
   Serial.println("All Satellites:");
   for (uint16_t block = 0; block < data->header.numSvs; block++) {
     // 衛星システム名を取得
     const char* gnssName = "UNKNOWN";
     switch (data->blocks[block].gnssId) {
       case 0: gnssName = "GPS"; break;
       case 1: gnssName = "SBAS"; break;
       case 2: gnssName = "Galileo"; break;
       case 3: gnssName = "BeiDou"; break;
       case 4: gnssName = "IMES"; break;
       case 5: gnssName = "QZSS"; break;
       case 6: gnssName = "GLONASS"; break;
     }
     
     Serial.print("  ");
     Serial.print(gnssName);
     Serial.print("-");
     Serial.print(data->blocks[block].svId);
     Serial.print(": ");
     Serial.print(data->blocks[block].cno);
     Serial.print("dB");
     
           // QZSS衛星の場合は追加情報を表示
      if (data->blocks[block].gnssId == 5) {
        Serial.print(" [QZSS]");
        if (data->blocks[block].cno == 0) {
          Serial.print(" - 信号弱いかL1S未設定");
        } else {
          Serial.print(" - 信号受信中");
        }
      }
     Serial.println();
   }

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

  // 画面初期化
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextFont(&fonts::efontJA_16);
  M5.Lcd.setCursor(0,0);
  M5.Display.setTextSize(1);
  M5.Lcd.setTextColor(WHITE);
  
  // 初期表示
  M5.Lcd.println("QZSS L1S信号監視開始");
  M5.Lcd.println("GNSSモジュール初期化中...");
  delay(1000); // 画面表示の確認

  // シリアル通信の初期化
  Serial.begin(115200);
  delay(2000);  // シリアルポートの安定化を待つ
  
  Serial.println("=== QZSS Disaster Information Receiver ===");
  Serial.println("Initializing...");
  Serial.println("M5Stack display test...");
  Serial.flush();  // バッファをクリア

  Serial1.begin(9600, SERIAL_8N1, rxPin, txPin);

  // GNSSモジュールの初期化を試行
  M5.Lcd.println("GNSSモジュール接続中...");
  Serial.println("Connecting to GNSS module...");
  
  if (myGNSS.begin(Serial1) == false) {
    M5.Lcd.println("9600bpsで接続失敗");
    Serial.println("Failed to connect at 9600bps");
    M5.Lcd.println("38400bpsで再試行...");
    Serial.println("Trying 38400bps...");
    
    // ボーレートを変えてトライ
    Serial1.begin(38400, SERIAL_8N1, rxPin, txPin);
    if (myGNSS.begin(Serial1) == false) {
      Serial.println("u-blox GNSS not detected. Please check wiring.");
      M5.Lcd.fillScreen(RED);
      M5.Lcd.setTextFont(&fonts::efontJA_16);
      M5.Lcd.setTextColor(WHITE);
      M5.Lcd.setCursor(0,0);
      M5.Lcd.println("GNSSモジュール未検出");
      M5.Lcd.println("配線を確認してください");
             M5.Lcd.println("DIPスイッチ設定:");
       M5.Lcd.println("TX: 1=ON,2=OFF,3=OFF,4=OFF");
       M5.Lcd.println("RX: 1=ON,2=OFF,3=OFF,4=OFF");
       M5.Lcd.println("PPS: 1=ON (推奨)");
      while (1) {
        delay(1000);
        Serial.println("GNSS module not found - check connections");
      }
    } else {
      M5.Lcd.println("38400bpsで接続成功");
      Serial.println("Connected at 38400bps");
    }
  } else {
    M5.Lcd.println("9600bpsで接続成功");
    Serial.println("Connected at 9600bps");
  }

  // GNSS設定
  M5.Lcd.println("GNSS設定中...");
  myGNSS.setUART1Output(COM_TYPE_UBX);           // UART1から出力をUBXのみに設定する
  
  // QZSSの基本設定を有効にする
  M5.Lcd.println("QZSS基本設定中...");
  Serial.println("=== QZSS Basic Configuration ===");
  if (enableQZSS()) {
    M5.Lcd.println("QZSS基本設定成功");
    Serial.println("QZSS basic enabled successfully");
  } else {
    M5.Lcd.println("QZSS基本設定失敗");
    Serial.println("QZSS basic enable failed");
  }
  
  // QZSS L1S信号の受信を有効にする
  M5.Lcd.println("QZSS L1S信号設定中...");
  Serial.println("=== QZSS L1S Configuration ===");
  if (enableQZSSL1S()) {
    M5.Lcd.println("QZSS L1S設定成功");
    Serial.println("QZSS L1S enabled successfully");
  } else {
    M5.Lcd.println("QZSS L1S設定失敗");
    Serial.println("QZSS L1S enable failed");
  }
  Serial.println("=== End QZSS Configuration ===");
  
  // SFRBXメッセージの受信を有効にする
  Serial.println("Enabling SFRBX messages...");
  // SFRBXメッセージは自動的に受信されるため、特別な設定は不要
  
  // コールバック関数を登録
  myGNSS.setAutoRXMSFRBXcallbackPtr(&newSFRBX);  // UBX-RXM-SFRBXメッセージ受信コールバック関数を登録
  myGNSS.setAutoNAVSATcallbackPtr(&newNAVSAT);   // UBX-NAV-SATメッセージ受信コールバック関数を登録
  myGNSS.setAutoPVTcallbackPtr(&newNAVPVT);      // UBX-NAV-PVTメッセージ受信コールバック関数を登録
  
  M5.Lcd.println("初期化完了");
  M5.Lcd.println("衛星信号を待機中...");
  M5.Lcd.println("初回受信まで数分かかる場合があります");
  
  // 初期画面表示
  Serial.println("Initial display update...");
  delay(2000); // 画面表示の安定化を待つ
  
  // 強制的に画面を更新
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextFont(&fonts::efontJA_16);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.println("初期化完了");
  M5.Lcd.println("画面表示テスト中...");
  delay(2000);
  
  displaySatelliteInfo();
}

// QZSSの基本設定を有効にする
bool enableQZSS(void) {
  Serial.println("Enabling QZSS basic configuration...");
  
  // QZSSを有効にする
  if (myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_QZSS) == false) {
    Serial.println("Failed to enable QZSS");
    return false;
  }
  
  Serial.println("QZSS enabled successfully");
  
  // QZSS設定の確認
  delay(1000); // 設定の反映を待つ
  if (checkQZSSConfig()) {
    Serial.println("QZSS configuration verified");
  } else {
    Serial.println("QZSS configuration verification failed");
  }
  
  return true;
}

// QZSS設定を確認する
bool checkQZSSConfig(void) {
  Serial.println("Checking QZSS configuration...");
  
  uint8_t customPayload[MAX_PAYLOAD_SIZE];
  ubxPacket customCfg = { 0, 0, 0, 0, 0, customPayload, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED };

  customCfg.cls = UBX_CLASS_CFG;
  customCfg.id = UBX_CFG_GNSS;
  customCfg.len = 0;
  customCfg.startingSpot = 0;

  if (myGNSS.sendCommand(&customCfg) != SFE_UBLOX_STATUS_DATA_RECEIVED) {
    Serial.println("Failed to get GNSS config for verification");
    return false;
  }

  int numConfigBlocks = customPayload[3];
  Serial.print("GNSS Config Blocks: ");
  Serial.println(numConfigBlocks);
  
  for (int block = 0; block < numConfigBlocks; block++) {
    uint8_t gnssId = customPayload[(block * 8) + 4];
    uint8_t enable = customPayload[(block * 8) + 8];
    uint8_t signals = customPayload[(block * 8) + 8 + 2];
    
    if (gnssId == (uint8_t)SFE_UBLOX_GNSS_ID_QZSS) {
      Serial.print("QZSS Config - Enable: 0x");
      Serial.print(enable, HEX);
      Serial.print(" Signals: 0x");
      Serial.println(signals, HEX);
      
      if (enable & 0x01) {
        Serial.println("QZSS is enabled");
        return true;
      } else {
        Serial.println("QZSS is disabled");
        return false;
      }
    }
  }
  
  Serial.println("QZSS not found in configuration");
  return false;
}

// QZSSのL1S信号を受信するよう設定する
bool enableQZSSL1S(void) {
  uint8_t customPayload[MAX_PAYLOAD_SIZE];
  ubxPacket customCfg = { 0, 0, 0, 0, 0, customPayload, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED };

  customCfg.cls = UBX_CLASS_CFG;
  customCfg.id = UBX_CFG_GNSS;
  customCfg.len = 0;
  customCfg.startingSpot = 0;

  if (myGNSS.sendCommand(&customCfg) != SFE_UBLOX_STATUS_DATA_RECEIVED) {
    Serial.println("Failed to get GNSS config");
    return false;
  }

  int numConfigBlocks = customPayload[3];
  Serial.print("GNSS Config Blocks: ");
  Serial.println(numConfigBlocks);
  
  bool qzssFound = false;
  for (int block = 0; block < numConfigBlocks; block++) {
    uint8_t gnssId = customPayload[(block * 8) + 4];
    Serial.print("Block ");
    Serial.print(block);
    Serial.print(" GNSS ID: ");
    Serial.println(gnssId);
    
         if (gnssId == (uint8_t)SFE_UBLOX_GNSS_ID_QZSS) {
       qzssFound = true;
       Serial.println("QZSS found - enabling L1S");
       Serial.print("Before: enable=");
       Serial.print(customPayload[(block * 8) + 8], HEX);
       Serial.print(" signals=");
       Serial.println(customPayload[(block * 8) + 8 + 2], HEX);
       
       customPayload[(block * 8) + 8] |= 0x01;      // set enable bit
       customPayload[(block * 8) + 8 + 2] |= 0x05;  // set 0x01 QZSS L1C/A 0x04 = QZSS L1S
       
       Serial.print("After:  enable=");
       Serial.print(customPayload[(block * 8) + 8], HEX);
       Serial.print(" signals=");
       Serial.println(customPayload[(block * 8) + 8 + 2], HEX);
     }
  }
  
  if (!qzssFound) {
    Serial.println("QZSS not found in GNSS config");
    return false;
  }

  bool result = (myGNSS.sendCommand(&customCfg) == SFE_UBLOX_STATUS_DATA_SENT);
  Serial.print("QZSS L1S config result: ");
  Serial.println(result ? "SUCCESS" : "FAILED");
  return result;
}

// DC通報を保存する関数
void saveDCReport(uint8_t messageType, uint8_t svId, uint8_t numWords, uint32_t* words) {
  if (dcReportCount >= MAX_DC_REPORTS) {
    // 古い通報を削除（FIFO方式）
    for (int i = 0; i < MAX_DC_REPORTS - 1; i++) {
      dcReports[i] = dcReports[i + 1];
    }
    dcReportCount = MAX_DC_REPORTS - 1;
  }
  
  // 新しい通報を追加
  dcReports[dcReportCount].timestamp = millis();
  dcReports[dcReportCount].messageType = messageType;
  dcReports[dcReportCount].svId = svId;
  dcReports[dcReportCount].numWords = numWords;
  dcReports[dcReportCount].isValid = true;
  
  // wordデータをコピー
  for (int i = 0; i < min((int)numWords, 8); i++) {
    dcReports[dcReportCount].words[i] = words[i];
  }
  
  dcReportCount++;
  Serial.printf("DC Report saved: Type %d, SV %d, Total: %d\n", messageType, svId, dcReportCount);
}

// 保存したDC通報を表示する関数
void displaySavedDCReports() {
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextFont(&fonts::efontJA_16);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(1);
  
  M5.Lcd.println("=== 保存済みDC通報 ===");
  
  if (dcReportCount == 0) {
    M5.Lcd.println("保存された通報はありません");
    M5.Lcd.println("ボタンAで通常表示に戻る");
    return;
  }
  
  M5.Lcd.printf("総通報数: %d件\n", dcReportCount);
  M5.Lcd.println("==================");
  
  // 最新の5件を表示
  int startIndex = max(0, dcReportCount - 5);
  for (int i = startIndex; i < dcReportCount; i++) {
    if (!dcReports[i].isValid) continue;
    
    M5.Lcd.printf("%d. ", i + 1);
    
    // メッセージタイプ
    if (dcReports[i].messageType == 43) {
      M5.Lcd.print("DC Report");
    } else if (dcReports[i].messageType == 44) {
      M5.Lcd.print("DCX");
    } else {
      M5.Lcd.printf("Type %d", dcReports[i].messageType);
    }
    
    M5.Lcd.printf(" (QZSS-%d)\n", dcReports[i].svId);
    
    // 受信時刻
    unsigned long timeSince = (millis() - dcReports[i].timestamp) / 1000;
    M5.Lcd.printf("   %lus前\n", timeSince);
    
    // 通報内容の概要
    M5.Lcd.print("   ");
    if (dcReports[i].messageType == 43) {
      M5.Lcd.print("災害通報");
    } else if (dcReports[i].messageType == 44) {
      M5.Lcd.print("拡張災害通報");
    } else {
      M5.Lcd.printf("Type %d", dcReports[i].messageType);
    }
    M5.Lcd.println();
  }
  
  M5.Lcd.println("==================");
  M5.Lcd.println("ボタンA: 通常表示");
  M5.Lcd.println("ボタンC: 最新通報詳細");
}

// 詳細なDC通報表示関数
void displayDetailedDCReport() {
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextFont(&fonts::efontJA_16);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(1);
  
  M5.Lcd.println("=== DC通報詳細 ===");
  
  if (dcReportCount == 0) {
    M5.Lcd.println("保存された通報はありません");
    M5.Lcd.println("ボタンAで通常表示に戻る");
    return;
  }
  
  // 最新の通報を詳細表示
  int latestIndex = dcReportCount - 1;
  if (dcReports[latestIndex].isValid) {
    M5.Lcd.printf("最新通報 #%d\n", latestIndex + 1);
    M5.Lcd.println("==================");
    
    // メッセージタイプ
    if (dcReports[latestIndex].messageType == 43) {
      M5.Lcd.print("タイプ: DC Report");
    } else if (dcReports[latestIndex].messageType == 44) {
      M5.Lcd.print("タイプ: DCX");
    } else {
      M5.Lcd.printf("タイプ: %d", dcReports[latestIndex].messageType);
    }
    M5.Lcd.printf(" (Type %d)\n", dcReports[latestIndex].messageType);
    
    // 衛星ID
    M5.Lcd.printf("衛星ID: QZSS-%d\n", dcReports[latestIndex].svId);
    
    // 受信時刻
    unsigned long timeSince = (millis() - dcReports[latestIndex].timestamp) / 1000;
    M5.Lcd.printf("受信時刻: %lus前\n", timeSince);
    
    // メッセージ長
    M5.Lcd.printf("メッセージ長: %d words\n", dcReports[latestIndex].numWords);
    
    // Wordデータ（全表示）
    M5.Lcd.println("Word Data:");
    for (int i = 0; i < dcReports[latestIndex].numWords; i++) {
      M5.Lcd.printf("Word[%d]: %s\n", i, dwrd_to_str(dcReports[latestIndex].words[i]));
    }
    
    // 通報内容の解析（可能な場合）
    M5.Lcd.println("==================");
    M5.Lcd.println("通報内容解析:");
    
    if (dcReports[latestIndex].messageType == 43) {
      // DC Reportの場合
      M5.Lcd.println("DC Report (災害通報)");
      
      // 保存されたwordデータをl1s_msg_bufに復元
      for (int i = 0; i < dcReports[latestIndex].numWords; i++) {
        l1s_msg_buf[(i << 2) + 0] = (dcReports[latestIndex].words[i] >> 24) & 0xff;
        l1s_msg_buf[(i << 2) + 1] = (dcReports[latestIndex].words[i] >> 16) & 0xff;
        l1s_msg_buf[(i << 2) + 2] = (dcReports[latestIndex].words[i] >> 8) & 0xff;
        l1s_msg_buf[(i << 2) + 3] = (dcReports[latestIndex].words[i]) & 0xff;
      }
      
      // QZQSMライブラリで日本語解析
      dc_report.SetYear(2025);
      dc_report.Decode(l1s_msg_buf);
      String reportText = dc_report.GetReport();
      
      // 日本語の通報内容を表示
      M5.Lcd.print("通報内容:\n");
      M5.Lcd.print(reportText);
      
    } else if (dcReports[latestIndex].messageType == 44) {
      // DCXの場合
      M5.Lcd.println("DCX (拡張災害通報)");
      
      // 保存されたwordデータをl1s_msg_bufに復元
      for (int i = 0; i < dcReports[latestIndex].numWords; i++) {
        l1s_msg_buf[(i << 2) + 0] = (dcReports[latestIndex].words[i] >> 24) & 0xff;
        l1s_msg_buf[(i << 2) + 1] = (dcReports[latestIndex].words[i] >> 16) & 0xff;
        l1s_msg_buf[(i << 2) + 2] = (dcReports[latestIndex].words[i] >> 8) & 0xff;
        l1s_msg_buf[(i << 2) + 3] = (dcReports[latestIndex].words[i]) & 0xff;
      }
      
      // QZSSDCXライブラリで日本語解析
      dcx_decoder.decode(l1s_msg_buf);
      
      // DCXの解析結果を表示
      M5.Lcd.println("通報内容:");
      M5.Lcd.println("詳細はシリアル出力を確認");
      
      // シリアルに詳細を出力
      Serial.println("=== Saved DCX Report Analysis ===");
      dcx_decoder.printSummary(Serial, dcx_decoder.r);
      
    } else {
      M5.Lcd.println("未知のメッセージタイプ");
    }
  }
  
  M5.Lcd.println("==================");
  M5.Lcd.println("ボタンA: 通常表示");
  M5.Lcd.println("ボタンB: 一覧表示");
}

unsigned long lastDebugTime = 0;

void loop() {
  myGNSS.checkUblox();
  myGNSS.checkCallbacks();
  
  // デバッグ情報を定期的に出力（10秒間隔）
  if (millis() - lastDebugTime > 10000) {
    Serial.println("=== STATUS REPORT ===");
    Serial.print("Status: ");
    Serial.print("Satellites=");
    Serial.print(satelliteCount);
    Serial.print(" | Time=");
    Serial.print(millis() / 1000);
    Serial.println("s");
    
    // QZSS状態レポート
    int qzssCount = 0;
    int qzssMaxCno = 0;
    for (int i = 0; i < satelliteCount; i++) {
      if (satellites[i].isQZSS) {
        qzssCount++;
        if (satellites[i].cno > qzssMaxCno) {
          qzssMaxCno = satellites[i].cno;
        }
      }
    }
    Serial.print("QZSS Status: Count=");
    Serial.print(qzssCount);
    Serial.print(" MaxSignal=");
    Serial.print(qzssMaxCno);
    Serial.println("dB");
    
    // QZSS設定の確認
    if (qzssCount == 0) {
      Serial.println("WARNING: No QZSS satellites detected!");
      Serial.println("Possible causes:");
      Serial.println("1. QZSS not enabled in GNSS configuration");
      Serial.println("2. No QZSS satellites visible from current location");
      Serial.println("   (QZSS satellites are primarily visible in Asia-Pacific region)");
      Serial.println("3. Antenna orientation issue");
      Serial.println("4. Hardware connection problem");
      Serial.println("5. Time of day (QZSS satellites may not be visible at certain times)");
    }
    
    Serial.println("=== END STATUS REPORT ===");
    
    lastDebugTime = millis();
  }
  
  // M5Stackのボタン処理
  M5.update();
  if (M5.BtnA.wasPressed()) {
    // ボタンAで画面更新
    Serial.println("Button A pressed - updating display");
    displaySatelliteInfo();
  }
  if (M5.BtnB.wasPressed()) {
    // ボタンBで保存済みDC通報表示
    Serial.println("Button B pressed - displaying saved DC reports");
    displaySavedDCReports();
  }
  if (M5.BtnC.wasPressed()) {
    // ボタンCで詳細表示
    Serial.println("Button C pressed - detailed view");
    displayDetailedDCReport();
  }
}

