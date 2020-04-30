// M5StickCの内蔵マイクが閾値を超えた音量を検出するたび
// SwitchBotに接続し、コマンドを送信する
// スキャンを最初にしておくことで、検出からコマンド送信までの速度をアップ
// ディスプレイにログ出力
// 接続失敗した際に，再起動

#include <M5StickC.h>
#include <driver/i2s.h>
#include "arduinoFFT.h"
#include "BLEDevice.h"

// 5kHz -> 5/0.17=29.411... の四捨五入整数 = 29
// 値が-1の場合、周波数はなし
#define TH_FREQ_BAND -1
// 音量閾値
#define TH_VOLUME 0.1


// 手元のSwitchBotのMACアドレス
static String MAC_SWITCHBOT = "SWTICHBOT_MAC";

// SwitchBotのBLE情報
static BLEUUID SERV_SWITCHBOT("cba20d00-224d-11e6-9fb8-0002a5d5c51b");
static BLEUUID CHAR_SWITCHBOT("cba20002-224d-11e6-9fb8-0002a5d5c51b");
// SwitchBot へ送信するコマンド
// アームを倒し、引く動作の場合 {0x57, 0x01, 0x00}
// 以下２つはスマートフォンアプリでモードを変える必要あり
// アームを倒す動作の場合 {0x57, 0x01, 0x01}
// アームを引く動作の場合 {0x57, 0x01, 0x02}
static uint8_t cmdPress[3] = {0x57, 0x01, 0x02};

#define PIN_CLK  0
#define PIN_DATA 34
#define READ_LEN (2 * 1024)
#define SAMPLING_FREQUENCY 44100

uint8_t BUFFER[READ_LEN] = {0};
uint16_t *adcBuffer = NULL;
const uint16_t FFTsamples = 256;  // サンプル数は2のべき乗
double vReal[FFTsamples];  // vReal[]にサンプリングしたデーターを入れる
double vImag[FFTsamples];
arduinoFFT FFT = arduinoFFT(vReal, vImag, FFTsamples, SAMPLING_FREQUENCY);  // FFTオブジェクトを作る

bool doScan = true;
BLEScan* pBLEScan;
static BLEAddress *pGattServerAddress;
static BLEAdvertisedDevice* myDevice;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEClient*  pClient = NULL;

unsigned int sampling_period_us;
float dmax = 10000.0;

bool doDetection = false;
bool sendFlg = false;
bool cas = false;

//int printCount = 0;

void log(String s) {
  Serial.println(s);
  M5.Lcd.println(s);
}


// BLEへの接続　コールバック
class MyClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) {
      Serial.println("onConnect");
      //    log("cc conn");
    }
    void onDisconnect(BLEClient* pclient) {
      Serial.println("onDisconnect");
      //    log("dis");
      //    doScan = true;
      //    doDetection = false;
      if (cas) {
        esp_restart();
      }
    }
};

// アドバタイズ検出時のコールバック
class advdCallback: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
      if (advertisedDevice.haveServiceUUID()) {
        String addr = advertisedDevice.getAddress().toString().c_str();
        Serial.printf("It have service addr: %s \n", advertisedDevice.getAddress().toString().c_str());
        if (addr.equalsIgnoreCase(MAC_SWITCHBOT)) {
          // SwitchBot を発見
          //        if (advertisedDevice.getServiceUUID().equals(SERV_SWITCHBOT)) {
          log("found");

          advertisedDevice.getScan()->stop();
          pGattServerAddress = new BLEAddress(advertisedDevice.getAddress());
          myDevice = new BLEAdvertisedDevice(advertisedDevice);

          doScan = false;
          doDetection = true;
          m5LED(500, 2);
        }
      }
    }
};


void setup() {
  Serial.begin(115200);
  M5.begin();
  i2sInit();

  //  pinMode(M5_LED, OUTPUT);
  M5.Axp.ScreenBreath(8);    // 画面の輝度を少し下げる
  M5.Lcd.setTextSize(2);    // 画面内文字のサイズを大きく

  log("Start");

  xTaskCreate(mic_fft_task, "mic_fft_task", 2048, NULL, 1, NULL);

  // BLE 初期化
  BLEDevice::init("");
  // デバイスからのアドバタイズをスキャン
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new advdCallback());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // less or equal setInterval value
}

void loop() {

  if (doScan) {
    log("Scan");
    pBLEScan->start(5, false);
  }

  if (sendFlg == true && doScan == false) {
    // 音を検出
    // log reset
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 0);

    log("Detect Sound");
    if (connectAndSendCommand(*pGattServerAddress)) {
      log("Done!");
      sendFlg = false;
      doScan = false;
    } else {
      log("Failed.");
      doScan = true;
      delay(3000);
    }
  }

  vTaskDelay(500 / portTICK_RATE_MS);
}

// SwitchBot の GATT サーバへ接続 ～ Press コマンド送信
static bool connectAndSendCommand(BLEAddress pAddress) {
  cas = true;
  try {
    pClient = BLEDevice::createClient();

    pClient->setClientCallbacks(new MyClientCallback());

    log("Connecting");
    while (!pClient->connect(myDevice)) {
      log("reconnect");
      delay(1000);
    }

    // 対象サービスを得る
    BLERemoteService* pRemoteService = pClient->getService(SERV_SWITCHBOT);
    if (pRemoteService == nullptr) {
      log("e:service not found");
      return false;
    }

    // 対象キャラクタリスティックを得る
    pRemoteCharacteristic = pRemoteService->getCharacteristic(CHAR_SWITCHBOT);
    if (pRemoteCharacteristic == nullptr) {
      log("e:characteristic not found");
      return false;
    }

    // キャラクタリスティックに Press コマンドを書き込む
    pRemoteCharacteristic->writeValue(cmdPress, sizeof(cmdPress), false);
    //    pRemoteCharacteristic->writeValue("test", true);
    log("Send");

    cas = false;

    delay(3000);
    pClient->disconnect();
    pClient = NULL;
    delay(1000);
  }
  catch (...) {
    log("error");
    if (pClient) {
      pClient->disconnect();
      pClient = NULL;
    }
    return false;
  }

  return true;
}

// M5StickCのLEDを点滅させる
void m5LED(int t, int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(M5_LED, LOW);
    delay(t / 2);
    digitalWrite(M5_LED, HIGH);
    delay(t / 2);
  }
}
