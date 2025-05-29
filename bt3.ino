#include <ESP32Servo.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <DHT.h>
#include <Preferences.h>

// ====== CONSTANTS =======

#define DHTPIN 18       
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

const int ADC_PIN = 35;        
const float VREF = 3.3;        
const int ADC_MAX = 4095;     
const float VOLTAGE_DIVIDER_RATIO = 2.0; 

const int sensorPin = 34;
const int servoPin = 13;
Servo myServo;

Preferences preferences;

#define SERVICE_UUID      "12345678-1234-1234-1234-1234567890ab"

#define ANGLE_UUID        "12345678-1234-1234-1234-1234567890ac"

#define LIGHT_UUID        "12345678-1234-1234-1234-1234567890ad"
#define TEMP_UUID         "12345678-1234-1234-1234-1234567890b1"

#define AUTOMODE_UUID     "12345678-1234-1234-1234-1234567890ae"
#define INVERT_MODE_UUID  "12345678-1234-1234-1234-1234567890af"

#define LIGHT_SENS_UUID   "12345678-1234-1234-1234-1234567890b0"
#define TEMP_SENS_UUID    "12345678-1234-1234-1234-1234567890b2"

#define BATT_UUID "12345678-1234-1234-1234-1234567890b3"

// ====== TUNING PARAMS =======
const int notifyMinIntervalMs = 500;
const int stabilityWindow = 5; // 3-6       
int lightSensitivityLevel = 50; // 0-100
int tempSensitivityLevel = 50; // 0-100

// ============================

BLECharacteristic* angleCharacteristic;
BLECharacteristic* lightCharacteristic;
BLECharacteristic* autoModeCharacteristic;
BLECharacteristic* invertModeCharacteristic;
BLECharacteristic* lightSensCharacteristic;
BLECharacteristic* tempCharacteristic;
BLECharacteristic* tempSensCharacteristic;
BLECharacteristic* battCharacteristic;

// ====== CONSTANTS =======
const int MIN_ANGLE = 20;   
const int MAX_ANGLE = 180;

int lightHistory[stabilityWindow];
int historyIndex = 0;
bool bufferFilled = false;

int lastNotifiedLight = 0;
unsigned long lastLightNotifyTime = 0;

float lastNotifiedTemp = 0;
unsigned long lastTempNotifyTime = 0;

float lastNotifiedBatt = 0;
unsigned long lastBattNotifyTime = 0;

unsigned long lastServoMoveTime = 0;
const unsigned long batteryQuietTime = 5000; 

unsigned long lastBatteryCheckTime = 0;
const unsigned long batteryCheckInterval = 5000; 

int currentAngle = 0;
int smoothedLight = 0;
float currentTemp = 0.0;
bool autoMode = true; 
bool invertMode = false; 


int readSmoothedLight(int pin, int samples = 10) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(5);
  }
  return sum / samples;
}

int readSmoothedBattery(int pin, int samples = 30) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(2); 
  }
  return sum / samples;
}

float getBatteryVoltage() {
  int raw = readSmoothedBattery(ADC_PIN);
  float vpin = raw * VREF / ADC_MAX;
  return vpin * VOLTAGE_DIVIDER_RATIO;
}

bool isStable(int* history, int length, int threshold) {
  if (!bufferFilled) return false;
  
  int minVal = history[0], maxVal = history[0];
  for (int i = 1; i < length; i++) {
    if (history[i] < minVal) minVal = history[i];
    if (history[i] > maxVal) maxVal = history[i];
  }
  return (maxVal - minVal) <= threshold;
}

void smoothServoMove(Servo& servo, int fromAngle, int toAngle, int stepDelay = 8) {
  if (fromAngle == toAngle) return;

  int step = (fromAngle < toAngle) ? 1 : -1;
  angleCharacteristic->setValue(toAngle);
  angleCharacteristic->notify();

  for (int angle = fromAngle; angle != toAngle; angle += step) {
    servo.write(angle);
    delay(stepDelay);
  }
  servo.write(toAngle);
  currentAngle = toAngle; 
  lastServoMoveTime = millis(); 
}

void setAutoMode(bool enabled) {
  autoMode = enabled;
  uint8_t val = enabled ? 1 : 0;

  autoModeCharacteristic->setValue(&val, 1);
  autoModeCharacteristic->notify();

  preferences.begin("blinds", false);
  preferences.putBool("autoMode", autoMode);
  preferences.end();
}

void setInvertMode(bool enabled) {
  invertMode = enabled;
  uint8_t val = enabled ? 1 : 0;

  invertModeCharacteristic->setValue(&val, 1);
  invertModeCharacteristic->notify();

  preferences.begin("blinds", false);
  preferences.putBool("invertMode", invertMode);
  preferences.end();
}

void handleAutoAngleControl() {
  int stabilityThreshold = map(lightSensitivityLevel, 0, 100, 100, 10);
  if (!isStable(lightHistory, stabilityWindow, stabilityThreshold)) return;

  int lightAngle = invertMode
    ? map(smoothedLight, 0, 4095, MAX_ANGLE, MIN_ANGLE)
    : map(smoothedLight, 0, 4095, MIN_ANGLE, MAX_ANGLE);
  lightAngle = constrain(lightAngle, MIN_ANGLE, MAX_ANGLE);

  int tempAngle = map(currentTemp, 15, 35, MIN_ANGLE, MAX_ANGLE);
  tempAngle = constrain(tempAngle, MIN_ANGLE, MAX_ANGLE);

  int lightWeight = 100 - tempSensitivityLevel;
  int tempWeight = tempSensitivityLevel;
  int blendedAngle = (lightAngle * lightWeight + tempAngle * tempWeight) / 100;
  blendedAngle = constrain(blendedAngle, MIN_ANGLE, MAX_ANGLE);

  int deadband = map(lightSensitivityLevel, 0, 100, 20, 3);
  if (abs(blendedAngle - currentAngle) >= deadband) {
    smoothServoMove(myServo, currentAngle, blendedAngle);
  }
}


void setup() {
  Serial.begin(115200);

  // Load persistent settings
  preferences.begin("blinds", false);
  autoMode = preferences.getBool("autoMode", true);
  invertMode = preferences.getBool("invertMode", false);
  lightSensitivityLevel = preferences.getUChar("lightSens", 50);
  tempSensitivityLevel = preferences.getUChar("tempSens", 50);
  currentAngle = preferences.getUChar("angle", MIN_ANGLE);
  currentAngle = constrain(currentAngle, MIN_ANGLE, MAX_ANGLE);

  preferences.end();

  analogReadResolution(12);
  myServo.attach(servoPin);
  myServo.write(currentAngle);
  dht.begin();

  setupBLE();
}

void loop() {
  smoothedLight = readSmoothedLight(sensorPin);
  currentTemp = dht.readTemperature(); 

  lightHistory[historyIndex++] = smoothedLight;
  if (historyIndex >= stabilityWindow) {
    historyIndex = 0;
    bufferFilled = true;
  }

  // ==================== temp ====================

  unsigned long now = millis();
  float tempNotifyThreshold = map(tempSensitivityLevel, 0, 100, 3.0, 0.2);
  if ((abs(currentTemp - lastNotifiedTemp) >= tempNotifyThreshold) &&
      (now - lastTempNotifyTime >= notifyMinIntervalMs)) {
    tempCharacteristic->setValue(currentTemp);
    tempCharacteristic->notify();
    lastNotifiedTemp = currentTemp;
    lastTempNotifyTime = now;
  }

  // ==================== light ====================

  int lightNotifyThreshold = map(lightSensitivityLevel, 0, 100, 30, 5);
  if ((abs(smoothedLight - lastNotifiedLight) >= lightNotifyThreshold) &&
      (now - lastLightNotifyTime >= notifyMinIntervalMs)) {
    lightCharacteristic->setValue(smoothedLight);
    lightCharacteristic->notify();
    lastNotifiedLight = smoothedLight;
    lastLightNotifyTime = now;
  }

  // ==================== battery ====================


  float battV = getBatteryVoltage();

  bool quietEnough = millis() - lastServoMoveTime > batteryQuietTime;
  bool checkIntervalPassed = millis() - lastBatteryCheckTime > batteryCheckInterval;

  if (quietEnough && checkIntervalPassed) {
    lastBatteryCheckTime = millis();

    bool significantChange = abs(battV - lastNotifiedBatt) >= 0.05;
    bool notifyTimeout = millis() - lastBattNotifyTime > 5000;

    if (significantChange || notifyTimeout) {
      battCharacteristic->setValue(battV);
      battCharacteristic->notify();
      lastNotifiedBatt = battV;
      lastBattNotifyTime = millis();
    }
  }

  // ==================== auto mode ====================

  if (autoMode && bufferFilled) {
    handleAutoAngleControl();
  }


  delay(200);
}

class AutoModeCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    const uint8_t* data = pCharacteristic->getData();
    size_t length = pCharacteristic->getLength();

    if (length > 0) {
      uint8_t mode = data[0];

      if (mode == 0) {
        setAutoMode(false);
      } else if (mode == 1) {
        setAutoMode(true);
      }
    }
  }

  void onRead(BLECharacteristic* pCharacteristic) override {
    uint8_t val = autoMode ? 1 : 0;
    pCharacteristic->setValue(&val, 1);
  }
};

class InvertModeCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    const uint8_t* data = pCharacteristic->getData();
    size_t length = pCharacteristic->getLength();

    if (length > 0) {
      uint8_t mode = data[0];

      if (mode == 0) {
        setInvertMode(false);
      } else if (mode == 1) {
        setInvertMode(true);
      }
    }
  }

  void onRead(BLECharacteristic* pCharacteristic) override {
    uint8_t val = invertMode ? 1 : 0;
    pCharacteristic->setValue(&val, 1);
  }
};

class AngleCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    if (autoMode) {
      return;
    }

    const uint8_t* data = pCharacteristic->getData();
    size_t length = pCharacteristic->getLength();

    if (length > 0) {
      int newAngle = data[0];
      newAngle = constrain(newAngle, MIN_ANGLE, MAX_ANGLE);

      smoothServoMove(myServo, currentAngle, newAngle);

      preferences.begin("blinds", false);
      preferences.putUChar("angle", currentAngle);
      preferences.end();
    }
  }

  void onRead(BLECharacteristic* pCharacteristic) override {
    pCharacteristic->setValue(currentAngle);
  }
};

class SensitivityCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    const uint8_t* data = pCharacteristic->getData();
    size_t length = pCharacteristic->getLength();
    if (length > 0) {
      int newVal = constrain(data[0], 0, 100);

      if (pCharacteristic == lightSensCharacteristic) {
        lightSensitivityLevel = newVal;
        lightSensCharacteristic->setValue(lightSensitivityLevel);
        lightSensCharacteristic->notify();

        preferences.begin("blinds", false);
        preferences.putUChar("lightSens", lightSensitivityLevel);
        preferences.end();
      } else if (pCharacteristic == tempSensCharacteristic) {
        tempSensitivityLevel = newVal;
        tempSensCharacteristic->setValue(tempSensitivityLevel);
        tempSensCharacteristic->notify();

        preferences.begin("blinds", false);
        preferences.putUChar("tempSens", tempSensitivityLevel);
        preferences.end();
      }
    }
  }

  void onRead(BLECharacteristic* pCharacteristic) override {
    if (pCharacteristic == lightSensCharacteristic)
      pCharacteristic->setValue(lightSensitivityLevel);
    else if (pCharacteristic == tempSensCharacteristic)
      pCharacteristic->setValue(tempSensitivityLevel);
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
  }

  void onDisconnect(BLEServer* pServer) override {
    pServer->startAdvertising();
  }
};

void setupBLE() {
  BLEDevice::init("Aven Blinds Pro Max");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(BLEUUID(SERVICE_UUID), 30, 0);

  angleCharacteristic = pService->createCharacteristic(
    ANGLE_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
  );
  angleCharacteristic->setCallbacks(new AngleCallbacks());
  angleCharacteristic->addDescriptor(new BLE2902());

  lightCharacteristic = pService->createCharacteristic(
    LIGHT_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  lightCharacteristic->addDescriptor(new BLE2902());

  autoModeCharacteristic = pService->createCharacteristic(
    AUTOMODE_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
  );
  autoModeCharacteristic->setCallbacks(new AutoModeCallbacks());
  autoModeCharacteristic->addDescriptor(new BLE2902());
  int autoVal = autoMode ? 1 : 0;
  autoModeCharacteristic->setValue(autoVal);


  invertModeCharacteristic = pService->createCharacteristic(
    INVERT_MODE_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
  );
  invertModeCharacteristic->setCallbacks(new InvertModeCallbacks());
  invertModeCharacteristic->addDescriptor(new BLE2902());
  int invertVal = invertMode ? 1 : 0;
  invertModeCharacteristic->setValue(invertVal);


  lightSensCharacteristic = pService->createCharacteristic(
    LIGHT_SENS_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  lightSensCharacteristic->setCallbacks(new SensitivityCallbacks());
  lightSensCharacteristic->addDescriptor(new BLE2902());
  lightSensCharacteristic->setValue(lightSensitivityLevel);

  tempCharacteristic = pService->createCharacteristic(
    TEMP_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  tempCharacteristic->addDescriptor(new BLE2902());

  tempSensCharacteristic = pService->createCharacteristic(
    TEMP_SENS_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
  );
  tempSensCharacteristic->setCallbacks(new SensitivityCallbacks());
  tempSensCharacteristic->addDescriptor(new BLE2902());
  tempSensCharacteristic->setValue(tempSensitivityLevel);

  battCharacteristic = pService->createCharacteristic(
    BATT_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  battCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinInterval(0x20);
  pAdvertising->setMaxInterval(0x40);
  pAdvertising->start();
}