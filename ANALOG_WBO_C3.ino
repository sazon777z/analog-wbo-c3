/*
  =============================================================================
  Проект: ANALOG WBO C3 - Высокоточный беспроводной AFR-индикатор на ESP32-C3
  =============================================================================
  Описание:
    Чтение аналогового сигнала 0-5 В от контроллера ШЛЗ (LSU 4.9) через
    резистивный делитель 1:2 (10кОм / 10кОм).
    Вычисление AFR (10.0 - 20.0) и Lambda (0.68 - 1.36) с аппаратной
    eFuse-калибровкой АЦП, оверсэмплингом и адаптивным фильтром с нулевой задержкой.
    Передача данных по BLE (Nordic UART Service) с частотой 30-40 Гц.

  Совместимость:
    - Платы: ESP32-C3 Dev Module, ESP32-C3 SuperMini
    - Среда: Arduino IDE (ESP32 core v2.x / v3.x)
    - Приложение: Web Bluetooth Dashboard, Serial Bluetooth Terminal, nRF Connect
  =============================================================================
*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ======================== АППАРАТНАЯ КОНФИГУРАЦИЯ ========================
// Пин АЦП для измерения напряжения с делителя
// GPIO4 на ESP32-C3 (ADC1_CH4)
#define ADC_PIN                 4

// Коэффициент делителя напряжения: (R1 + R2) / R2
// Для двух резисторов по 10.0 кОм: (10 + 10) / 10 = 2.000
// При необходимости точной подстройки измерьте мультиметром и скорректируйте.
#define VOLTAGE_DIVIDER_RATIO   2.000f

// Программная калибровочная добавка напряжения (в Вольтах)
// Используется для компенсации разницы потенциалов масс (Ground Offset)
#define VOLTAGE_OFFSET_V        0.000f

// Параметры широкополосного контроллера (WBO)
// Стандарт: 0.0 В = 10.00 AFR; 5.0 В = 20.00 AFR
#define AFR_MIN_VOLTS           0.000f
#define AFR_MAX_VOLTS           5.000f
#define AFR_MIN_VAL             10.000f
#define AFR_MAX_VAL             20.000f

// Стехиометрическое соотношение для бензина
#define STOICHIOMETRIC_AFR      14.700f

// ======================== ПАРАМЕТРЫ ФИЛЬТРАЦИИ И АЦП ========================
// Количество выборок оверсэмплинга за один цикл
#define OVERSAMPLE_COUNT        32

// Адаптивный фильтр EMA (Exponential Moving Average):
// При резком изменении смеси (газ в пол / сброс) включается быстрый коэффициент
// для мгновенной реакции (нулевая задержка).
// При установившемся режиме включается медленный коэффициент для устранения шума.
#define FILTER_ALPHA_FAST       0.85f   // Быстрая реакция на скачки
#define FILTER_ALPHA_SLOW       0.20f   // Фильтрация мелких шумов
#define FAST_CHANGE_THRESHOLD_V 0.035f  // Порог скачка напряжения (35 мВ ~ 0.07 AFR)

// Частота отправки данных по BLE (в миллисекундах)
// 25 мс = 40 Гц, 33 мс = ~30 Гц
#define BLE_UPDATE_INTERVAL_MS  25

// ======================== BLE НАСТРОЙКИ (Nordic UART) ========================
#define DEVICE_NAME             "WBO_AFR_C3"
#define SERVICE_UUID            "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer*         pServer         = nullptr;
BLECharacteristic* pTxCharacteristic = nullptr;
bool               deviceConnected = false;
bool               oldDeviceConnected = false;

// ======================== ПЕРЕМЕННЫЕ СОСТОЯНИЯ ========================
float filteredVolts = 0.0f;
float currentAFR    = 14.7f;
float currentLambda = 1.0f;
unsigned long lastBleSendTime = 0;

// Серверные обратные вызовы для управления соединением BLE
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server, esp_ble_gatts_cb_param_t* param) {
    deviceConnected = true;
    Serial.println(F("[BLE] Клиент подключен!"));

    // Запрашиваем минимальный интервал подключения для сверхнизкой задержки:
    // min_interval = 6 (7.5ms), max_interval = 12 (15ms), latency = 0, timeout = 100 (1000ms)
    server->updateConnParams(param->connect.remote_bda, 6, 12, 0, 100);
  }

  void onDisconnect(BLEServer* server) {
    deviceConnected = false;
    Serial.println(F("[BLE] Клиент отключен. Возобновление рекламы..."));
  }
};

// Чтение аналогового напряжения с оверсэмплингом и калибровкой eFuse
float readSensorVoltageRaw() {
  uint32_t sumMillivolts = 0;

  for (int i = 0; i < OVERSAMPLE_COUNT; i++) {
    // analogReadMilliVolts использует заводскую калибровку eFuse ESP32
    sumMillivolts += analogReadMilliVolts(ADC_PIN);
    delayMicroseconds(30);
  }

  float pinMilliVolts = (float)sumMillivolts / (float)OVERSAMPLE_COUNT;
  float pinVolts = pinMilliVolts / 1000.0f;

  // Восстанавливаем реальное входное напряжение контроллера WBO (0 - 5.0 В)
  float inputVolts = (pinVolts * VOLTAGE_DIVIDER_RATIO) + VOLTAGE_OFFSET_V;

  // Ограничиваем в разумных пределах
  if (inputVolts < 0.0f) inputVolts = 0.0f;
  if (inputVolts > 5.2f) inputVolts = 5.2f;

  return inputVolts;
}

// Адаптивная фильтрация входного напряжения без фазовой задержки
float applyAdaptiveFilter(float rawVolts) {
  static bool initialized = false;
  if (!initialized) {
    filteredVolts = rawVolts;
    initialized = true;
    return filteredVolts;
  }

  float delta = fabsf(rawVolts - filteredVolts);
  float alpha = (delta > FAST_CHANGE_THRESHOLD_V) ? FILTER_ALPHA_FAST : FILTER_ALPHA_SLOW;

  filteredVolts = (alpha * rawVolts) + ((1.0f - alpha) * filteredVolts);
  return filteredVolts;
}

// Преобразование напряжения (0-5 В) в значение AFR (10.0-20.0)
float calculateAFR(float volts) {
  // Линейная зависимость:
  // 0.0 В  -> 10.0 AFR
  // 5.0 В  -> 20.0 AFR
  // Наклон (Slope) = (20.0 - 10.0) / (5.0 - 0.0) = 2.0 AFR/V
  float afr = AFR_MIN_VAL + (volts * ((AFR_MAX_VAL - AFR_MIN_VAL) / (AFR_MAX_VOLTS - AFR_MIN_VOLTS)));

  if (afr < AFR_MIN_VAL) afr = AFR_MIN_VAL;
  if (afr > AFR_MAX_VAL) afr = AFR_MAX_VAL;

  return afr;
}

void initBLE() {
  BLEDevice::init(DEVICE_NAME);

  // Настройка максимальной мощности передатчика для стабильной связи в салоне авто
  BLEDevice::setPower(ESP_PWR_LVL_P9);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  // TX Характеристика для отправки телеметрии (Notify)
  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLE2902());

  // RX Характеристика (для возможного приема калибровочных команд)
  BLECharacteristic* pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE
  );

  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // Подсказка для iOS/Android о быстром интервале
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println(F("[BLE] Сервис Nordic UART запущен и ожидает подключения."));
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println(F("========================================"));
  Serial.println(F("    WBO AFR Gauge Controller ESP32-C3   "));
  Serial.println(F("========================================"));

  // Настройка АЦП ESP32-C3:
  // При 11dB (или 12dB в зависимости от ядра) диапазон измерения пина до ~2.5 - 3.1 В.
  // При делителе 1:2 (10к/10к) максимальное напряжение на пине равно 2.50 В,
  // что идеально укладывается в диапазон максимальной точности АЦП.
  analogSetAttenuation(ADC_11db);
  analogReadResolution(12); // 12-битный АЦП (0-4095)

  // Первоначальный прогрев и считывание
  for (int i = 0; i < 10; i++) {
    readSensorVoltageRaw();
    delay(5);
  }

  initBLE();
  Serial.println(F("[Система] Инициализация завершена. Запуск основного цикла..."));
}

void loop() {
  // 1. Быстрое чтение АЦП и адаптивная фильтрация
  float rawV = readSensorVoltageRaw();
  float vIn = applyAdaptiveFilter(rawV);

  // 2. Расчет параметров смеси
  currentAFR = calculateAFR(vIn);
  currentLambda = currentAFR / STOICHIOMETRIC_AFR;

  // 3. Отправка данных по BLE с заданной периодичностью
  unsigned long now = millis();
  if (now - lastBleSendTime >= BLE_UPDATE_INTERVAL_MS) {
    lastBleSendTime = now;

    if (deviceConnected) {
      // Формат пакета: AFR,Вольты,Лямбда\n
      // Пример: "14.72,2.36,1.00\n"
      char packet[32];
      snprintf(packet, sizeof(packet), "%.2f,%.2f,%.2f\n", currentAFR, vIn, currentLambda);

      pTxCharacteristic->setValue((uint8_t*)packet, strlen(packet));
      pTxCharacteristic->notify();
    }
  }

  // Обработка переподключения BLE
  if (!deviceConnected && oldDeviceConnected) {
    delay(50); // Даем стеку BLE очиститься
    pServer->startAdvertising();
    Serial.println(F("[BLE] Перезапуск рекламы после отключения"));
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  // Периодический вывод отладки в Serial (раз в 500 мс)
  static unsigned long lastSerialTime = 0;
  if (now - lastSerialTime >= 500) {
    lastSerialTime = now;
    Serial.printf("[WBO] Вход: %.3f В | AFR: %.2f | Lambda: %.2f | BLE: %s\n",
                  vIn, currentAFR, currentLambda, deviceConnected ? "Подключен" : "Ожидание");
  }

  // Минимальная пауза для разгрузки FreeRTOS
  delay(1);
}
