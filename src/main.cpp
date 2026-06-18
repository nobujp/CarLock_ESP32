/**
 * ESP32 4-Relay Car Lock Controller
 * Board : Aideepen ESP32 Relay Module (4ch)
 * Control: BLE (Nordic UART Service)
 *
 * コマンド形式: "PINコード-コマンド"
 *   例) 1234-L              → ロック
 *       1234-U              → アンロック
 *       1234-H              → 5秒ハザード
 *       1234-R4             → RELAY_4 (予備)
 *       1234-STATUS         → 状態確認
 *       1234-SETPIN:新PIN   → PINコード変更（4〜8桁）
 *       1234-AUTOLOCK:ON       → 自動ロック有効
 *       1234-AUTOLOCK:OFF      → 自動ロック無効
 *       1234-AUTOLOCK:DELAY:30 → 切断後30秒でロック
 *       1234-AUTOUNLOCK:ON     → 自動アンロック有効（接続2秒後に解錠）
 *       1234-AUTOUNLOCK:OFF    → 自動アンロック無効
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>

// ==================== 設定 ====================

#define BLE_DEVICE_NAME  "CarLock-ESP32"

#define RELAY_1  23
#define RELAY_2   5
#define RELAY_3   4
#define RELAY_4  13

#define RELAY_ON   LOW
#define RELAY_OFF  HIGH

#define LOCK_PULSE_MS     500
#define HAZARD_5SEC_MS   5000
#define HAZARD_BLINK_MS   500

#define MAX_FAIL        3
#define LOCKOUT_MS  60000UL

// ===============================================

#define SERVICE_UUID            "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

Preferences         prefs;
String              pinCode;

BLEServer*          pServer           = nullptr;
BLECharacteristic*  pTxCharacteristic = nullptr;
bool                deviceConnected   = false;

// セキュリティ
int           failCount    = 0;
unsigned long lockoutUntil = 0;

// BLEコールバック→loop()へのコマンド受け渡し
volatile int  pendingCmd    = 0;
volatile bool pendingSetPin = false;
String        pendingNewPin = "";

// リレー制御
unsigned long relayOffTime = 0;
int           activeRelay  = -1;

// ハザード点滅
bool          isBlinking      = false;
bool          blinkState      = false;
unsigned long blinkEndTime    = 0;
unsigned long blinkNextToggle = 0;

// アンサーバックフィードバック
int           feedbackBlinks     = 0;
bool          feedbackActive     = false;
bool          feedbackState      = false;
unsigned long feedbackNextToggle = 0;

// 自動ロック
bool              autoLockEnabled = false;
unsigned long     autoLockDelay   = 30000UL;
volatile bool     autoLockPending = false;
unsigned long     autoLockTime    = 0;

// 自動アンロック（BLE接続2秒後に解錠）
bool              autoUnlockEnabled = false;
volatile bool     autoUnlockPending = false;
unsigned long     autoUnlockTime    = 0;
#define AUTOUNLOCK_DELAY_MS  2000UL

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

    if (pin == RELAY_1 || pin == RELAY_2) {
        feedbackBlinks     = (pin == RELAY_1) ? 1 : 2;
        feedbackActive     = true;
        feedbackState      = true;
        digitalWrite(RELAY_3, RELAY_ON);
        feedbackNextToggle = millis() + 400;
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
        deviceConnected  = true;
        failCount        = 0;
        autoLockPending  = false;  // 自動ロックキャンセル
        if (autoUnlockEnabled) {
            autoUnlockPending = true;
            autoUnlockTime    = millis() + AUTOUNLOCK_DELAY_MS;
            Serial.println("BLE: 接続 → 2秒後に自動アンロック");
        } else {
            Serial.println("BLE: 接続");
        }
    }
    void onDisconnect(BLEServer* pSrv) override {
        deviceConnected   = false;
        failCount         = 0;
        autoUnlockPending = false;  // 自動アンロックキャンセル
        if (autoLockEnabled) {
            autoLockPending = true;
            autoLockTime    = millis() + autoLockDelay;
            Serial.println("BLE: 切断 → 自動ロック待機 " +
                           String(autoLockDelay / 1000) + "秒後");
        } else {
            Serial.println("BLE: 切断 → 再アドバタイズ");
        }
        pSrv->startAdvertising();
    }
};

class RxCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pChar) override {
        String rx = String(pChar->getValue().c_str());
        rx.trim();
        Serial.println("RX: " + rx);

        // ロックアウト中
        if (millis() < lockoutUntil) {
            long remain = (long)((lockoutUntil - millis()) / 1000) + 1;
            sendBLE("ERR:LOCKED " + String(remain) + "sec");
            return;
        }

        int sep = rx.indexOf('-');
        if (sep == -1) {
            sendBLE("ERR:FORMAT");
            return;
        }

        // PIN認証
        if (rx.substring(0, sep) != pinCode) {
            failCount++;
            int remain = MAX_FAIL - failCount;
            if (failCount >= MAX_FAIL) {
                lockoutUntil = millis() + LOCKOUT_MS;
                failCount    = 0;
                sendBLE("ERR:LOCKED_OUT 60sec");
                Serial.println("ロックアウト発動！");
            } else {
                sendBLE("ERR:AUTH_FAIL あと" + String(remain) + "回");
            }
            return;
        }
        failCount = 0;

        String cmd = rx.substring(sep + 1);
        cmd.toUpperCase();

        // PINコード変更
        if (cmd.startsWith("SETPIN:")) {
            String newPin = cmd.substring(7);
            if (newPin.length() >= 4 && newPin.length() <= 8) {
                pendingNewPin = newPin;
                pendingSetPin = true;
            } else {
                sendBLE("ERR:PIN_LENGTH 4〜8桁");
            }
            return;
        }

        // 自動ロック設定
        if (cmd == "AUTOLOCK:ON") {
            autoLockEnabled = true;
            prefs.putBool("alOn", true);
            sendBLE("AUTOLOCK:ON/" + String(autoLockDelay / 1000) + "sec");
            return;
        }
        if (cmd == "AUTOLOCK:OFF") {
            autoLockEnabled = false;
            autoLockPending = false;
            prefs.putBool("alOn", false);
            sendBLE("AUTOLOCK:OFF");
            return;
        }
        if (cmd.startsWith("AUTOLOCK:DELAY:")) {
            int sec = cmd.substring(15).toInt();
            if (sec >= 5 && sec <= 120) {
                autoLockDelay = (unsigned long)sec * 1000UL;
                prefs.putInt("alSec", sec);
                sendBLE("AUTOLOCK:DELAY:" + String(sec) + "sec OK");
            } else {
                sendBLE("ERR:DELAY 5〜120秒");
            }
            return;
        }

        // 自動アンロック設定
        if (cmd == "AUTOUNLOCK:ON") {
            autoUnlockEnabled = true;
            prefs.putBool("auOn2", true);
            sendBLE("AUTOUNLOCK:ON");
            return;
        }
        if (cmd == "AUTOUNLOCK:OFF") {
            autoUnlockEnabled = false;
            autoUnlockPending = false;
            prefs.putBool("auOn2", false);
            sendBLE("AUTOUNLOCK:OFF");
            return;
        }

        // 通常コマンド（GPIO操作はloop()で実行）
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

    prefs.begin("carlock", false);
    pinCode         = prefs.getString("pin", "1234");
    autoLockEnabled   = prefs.getBool("alOn",  false);
    autoLockDelay     = (unsigned long)prefs.getInt("alSec", 30) * 1000UL;
    autoUnlockEnabled = prefs.getBool("auOn2", true);   // デフォルトON

    Serial.println("PIN: " + pinCode);
    Serial.println("自動ロック:   " + String(autoLockEnabled   ? "ON" : "OFF") +
                   " / " + String(autoLockDelay / 1000) + "秒");
    Serial.println("自動アンロック: " + String(autoUnlockEnabled ? "ON" : "OFF"));

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
    Serial.println("準備完了 - スマホから接続してください");
}

// ==================== loop ====================
void loop() {
    unsigned long now = millis();

    // 自動アンロック タイマー（接続2秒後）
    if (autoUnlockPending && now >= autoUnlockTime) {
        autoUnlockPending = false;
        Serial.println("自動アンロック実行！");
        activateRelay(RELAY_2, LOCK_PULSE_MS, "AUTO_UNLOCKED");
    }

    // 自動ロック タイマー
    if (autoLockPending && now >= autoLockTime) {
        autoLockPending = false;
        Serial.println("自動ロック実行！");
        activateRelay(RELAY_1, LOCK_PULSE_MS, "AUTO_LOCKED");
    }

    // PINコード変更
    if (pendingSetPin) {
        pendingSetPin = false;
        pinCode = pendingNewPin;
        prefs.putString("pin", pinCode);
        sendBLE("PIN_CHANGED:" + pinCode);
        Serial.println("PIN変更: " + pinCode);
    }

    // BLEコマンド実行
    if (pendingCmd != 0) {
        int cmd = pendingCmd;
        pendingCmd = 0;
        switch (cmd) {
            case 1: activateRelay(RELAY_1, LOCK_PULSE_MS, "LOCKING");   break;
            case 2: activateRelay(RELAY_2, LOCK_PULSE_MS, "UNLOCKING"); break;
            case 3: startHazard();                                       break;
            case 4: activateRelay(RELAY_4, LOCK_PULSE_MS, "RELAY4_ON"); break;
            case 9: {
                String s = "STATUS:";
                s += (activeRelay == -1 ? "IDLE" : "BUSY");
                s += " AUTOLOCK:";
                s += (autoLockEnabled ? "ON/" + String(autoLockDelay/1000) + "sec" : "OFF");
                sendBLE(s);
                break;
            }
        }
    }

    // ロック/アンロック 自動OFF
    if (activeRelay != -1 && now >= relayOffTime) {
        digitalWrite(activeRelay, RELAY_OFF);
        sendBLE("RELAY_OFF:GPIO" + String(activeRelay));
        activeRelay = -1;
    }

    // アンサーバック点滅（ロック後1回 / アンロック後2回）
    if (feedbackActive) {
        if (now >= feedbackNextToggle) {
            if (!feedbackState) {
                digitalWrite(RELAY_3, RELAY_ON);
                feedbackState      = true;
                feedbackNextToggle = now + 400;
            } else {
                digitalWrite(RELAY_3, RELAY_OFF);
                feedbackState = false;
                feedbackBlinks--;
                if (feedbackBlinks <= 0) {
                    feedbackActive = false;
                } else {
                    feedbackNextToggle = now + 400;
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
