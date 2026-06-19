/**
 * CarLock シンプルテスト版 + 自動再起動
 * BT接続 → RELAY_UNLOCK（GPIO5）作動
 * BT切断 → RELAY_LOCK（GPIO23）作動 → ESP32再起動（BLEスタックをリフレッシュ）
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define RELAY_LOCK    23
#define RELAY_UNLOCK   5
#define RELAY_ON    LOW
#define RELAY_OFF   HIGH
#define PULSE_MS    500

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer* pServer = nullptr;

volatile bool doLock    = false;
volatile bool doUnlock  = false;
volatile bool doRestart = false;

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* s) override {
        Serial.println("★ BT接続 → アンロック実行");
        doUnlock = true;
    }
    void onDisconnect(BLEServer* s) override {
        Serial.println("★ BT切断 → ロック実行");
        doLock    = true;
        doRestart = true;  // リレー作動後にESP32再起動でBLEリフレッシュ
    }
};

void pulseRelay(int pin, const char* label) {
    Serial.print("リレー作動: ");
    Serial.println(label);
    digitalWrite(pin, RELAY_ON);
    delay(PULSE_MS);
    digitalWrite(pin, RELAY_OFF);
    Serial.println("リレーOFF");
}

void setup() {
    Serial.begin(115200);
    Serial.println("=== CarLock テスト起動 ===");

    pinMode(RELAY_LOCK,   OUTPUT); digitalWrite(RELAY_LOCK,   RELAY_OFF);
    pinMode(RELAY_UNLOCK, OUTPUT); digitalWrite(RELAY_UNLOCK, RELAY_OFF);
    Serial.println("リレー初期化OK");

    BLEDevice::init("CarLock-ESP32");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    BLEService* svc = pServer->createService(SERVICE_UUID);
    BLECharacteristic* pTx = svc->createCharacteristic(
        CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
    pTx->addDescriptor(new BLE2902());
    svc->createCharacteristic(CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);

    svc->start();
    pServer->getAdvertising()->start();
    Serial.println("BLEアドバタイズ開始 - 接続待ち");
}

void loop() {
    if (doUnlock) {
        doUnlock = false;
        pulseRelay(RELAY_UNLOCK, "UNLOCK GPIO5");
    }
    if (doLock) {
        doLock = false;
        pulseRelay(RELAY_LOCK, "LOCK GPIO23");
    }
    if (doRestart) {
        doRestart = false;
        Serial.println("★ BLE停止 → 再起動");
        BLEDevice::deinit(false);  // BLEを正しくシャットダウンしてからリセット
        delay(1000);
        ESP.restart();
    }
}
