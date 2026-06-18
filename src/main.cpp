/**
 * ESP32 4-Relay Car Lock Controller
 * Board : Aideepen ESP32 Relay Module (4ch)
 * Control: BLE (Nordic UART Service)
 * App   : "Serial Bluetooth Terminal" (Android) / "BlueSee" (iOS)
 *
 * コマンド形式: "PINコード-コマンド"
 *   例) 1234-L  → ロック
 *       1234-U  → アンロック
 *       1234-H  → 5秒ハザード
 *       1234-R4 → RELAY_4 (予備)
 *       1234-STATUS → 状態確認
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ==================== 設定 ====================

#define BLE_DEVICE_NAME  "CarLock-ESP32"

const String PIN_CODE = "1234";

#define RELAY_1  23
#define RELAY_2   5
#define RELAY_3   4
#define RELAY_4  13

#define RELAY_ON   LOW
#define RELAY_OFF  HIGH

#define LOCK_PULSE_MS     500
#define HAZARD_5SEC_MS   5000
#define HAZARD_BLINK_MS   500

// ===============================================

#define SERVICE_UUID            "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer*         pServer           = nullptr;
BLECharacteristic* pTxCharacteristic = nullptr;
bool               deviceConnected   = false;

// BLEコールバック→loop()へのコマンド受け渡し用フラグ
// (BLEタスクからdirectlyにGPIOを操作しないため)
volatile int pendingCmd = 0;  // 0=なし 1=L 2=U 3=H 4=R4 9=STATUS

unsigned long relayOffTime = 0;
int           activeRelay  = -1;

bool          isBlinking      = false;
bool          blinkState      = false;
unsigned long blinkEndTime    = 0;
unsigned long blinkNextToggle = 0;

// ロック/アンロック後のハザードフィードバック
int           feedbackBlinks     = 0;   // 残り点滅回数
bool          feedbackActive     = false;
bool          feedbackState      = false;
unsigned long feedbackNextToggle = 0;

// ==================== BLE送信 ====================
void sendBLE(const String& msg) {
    Serial.println("TX: " + msg);
    if (deviceConnected && pTxCharacteristic) {
        pTxCharacteristic->setValue(msg.c_str());
        pTxCharacteristic->notify();
        delay(10);
    }
}

// ==================== リレー制御 (loop()内でのみ呼ぶ) ====================
void activateRelay(int pin, unsigned long durationMs, const String& ackMsg) {
    if (activeRelay != -1 || isBlinking || feedbackActive) {
        sendBLE("BUSY");
        return;
    }
    digitalWrite(pin, RELAY_ON);
    activeRelay  = pin;
    relayOffTime = millis() + durationMs;
    sendBLE(ackMsg);

    // ロック/アンロックと同時にハザードフィードバック開始
    if (pin == RELAY_1 || pin == RELAY_2) {
        feedbackBlinks     = (pin == RELAY_1) ? 1 : 2;
        feedbackActive     = true;
        feedbackState      = true;
        digitalWrite(RELAY_3, RELAY_ON);
        feedbackNextToggle = millis() + 400;  // 400ms ON
    }
}

void startHazard() {
    if (activeRelay != -1 || isBlinking || feedbackActive) {
        sendBLE("BUSY");
        return;
    }
    isBlinking      = true;
    blinkState      = true;
    blinkEndTime    = millis() + HAZARD_5SEC_MS;
    blinkNextToggle = millis() + HAZARD_BLINK_MS;
    digitalWrite(RELAY_3, RELAY_ON);
    sendBLE("HAZARD_ON");
}

// ==================== BLEコールバック ====================
class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pSrv) override {
        deviceConnected = true;
        Serial.println("BLE: 接続");
    }
    void onDisconnect(BLEServer* pSrv) override {
        deviceConnected = false;
        Serial.println("BLE: 切断 → 再アドバタイズ");
        pSrv->startAdvertising();
    }
};

class RxCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pChar) override {
        String rx = String(pChar->getValue().c_str());
        rx.trim();
        Serial.println("RX: " + rx);

        int sep = rx.indexOf('-');
        if (sep == -1 || rx.substring(0, sep) != PIN_CODE) {
            sendBLE("ERR:AUTH_FAIL");
            return;
        }

        String cmd = rx.substring(sep + 1);
        cmd.toUpperCase();

        // GPIO操作はloop()に任せる → フラグをセットするだけ
        if      (cmd == "L" || cmd == "LOCK")   pendingCmd = 1;
        else if (cmd == "U" || cmd == "UNLOCK")  pendingCmd = 2;
        else if (cmd == "H" || cmd == "HAZARD")  pendingCmd = 3;
        else if (cmd == "R4")                    pendingCmd = 4;
        else if (cmd == "STATUS")                pendingCmd = 9;
        else                                     sendBLE("ERR:UNKNOWN_CMD [" + cmd + "]");
    }
};

// ==================== setup ====================
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== CarLock ESP32 起動 ===");

    const int relayPins[] = { RELAY_1, RELAY_2, RELAY_3, RELAY_4 };
    for (int pin : relayPins) {
        digitalWrite(pin, RELAY_OFF);
        pinMode(pin, OUTPUT);
        digitalWrite(pin, RELAY_OFF);
    }
    Serial.println("リレー: 全OFF");

    BLEDevice::init(BLE_DEVICE_NAME);
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    BLEService* pService = pServer->createService(SERVICE_UUID);

    pTxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
    pTxCharacteristic->addDescriptor(new BLE2902());

    BLECharacteristic* pRxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    pRxCharacteristic->setCallbacks(new RxCallbacks());

    pService->start();
    pServer->getAdvertising()->start();

    Serial.println("BLE アドバタイズ開始: " BLE_DEVICE_NAME);
    Serial.println("PIN: " + PIN_CODE);
    Serial.println("準備完了 - スマホから接続してください");
}

// ==================== loop ====================
void loop() {
    unsigned long now = millis();

    // BLEから受け取ったコマンドをここで実行 (GPIO操作はloop()内のみ)
    if (pendingCmd != 0) {
        int cmd = pendingCmd;
        pendingCmd = 0;
        switch (cmd) {
            case 1: activateRelay(RELAY_1, LOCK_PULSE_MS, "LOCKING");   break;
            case 2: activateRelay(RELAY_2, LOCK_PULSE_MS, "UNLOCKING"); break;
            case 3: startHazard();                                       break;
            case 4: activateRelay(RELAY_4, LOCK_PULSE_MS, "RELAY4_ON"); break;
            case 9: sendBLE(activeRelay == -1 ? "STATUS:IDLE" : "STATUS:BUSY"); break;
        }
    }

    // ロック/アンロック 自動OFF
    if (activeRelay != -1 && now >= relayOffTime) {
        digitalWrite(activeRelay, RELAY_OFF);
        sendBLE("RELAY_OFF:GPIO" + String(activeRelay));
        activeRelay = -1;
    }

    // ハザードフィードバック点滅 (ロック後1回 / アンロック後2回)
    if (feedbackActive) {
        if (now >= feedbackNextToggle) {
            if (!feedbackState) {
                digitalWrite(RELAY_3, RELAY_ON);
                feedbackState      = true;
                feedbackNextToggle = now + 400;  // 400ms ON
            } else {
                digitalWrite(RELAY_3, RELAY_OFF);
                feedbackState = false;
                feedbackBlinks--;
                if (feedbackBlinks <= 0) {
                    feedbackActive = false;
                } else {
                    feedbackNextToggle = now + 400;  // 400ms OFF (次の点滅まで)
                }
            }
        }
    }

    // ハザード点滅
    if (isBlinking) {
        if (now >= blinkEndTime) {
            digitalWrite(RELAY_3, RELAY_OFF);
            isBlinking = false;
            sendBLE("HAZARD_OFF");
        } else if (now >= blinkNextToggle) {
            blinkState = !blinkState;
            digitalWrite(RELAY_3, blinkState ? RELAY_ON : RELAY_OFF);
            blinkNextToggle = now + HAZARD_BLINK_MS;
        }
    }
}
