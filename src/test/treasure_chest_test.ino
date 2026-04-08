/*
 * ============================================================================
 * ПРОЕКТ: "СУНДУК СОКРОВИЩ" - ТЕСТОВАЯ ПРОГРАММА С ЭМУЛЯЦИЕЙ ДАТЧИКОВ
 * ============================================================================
 * 
 * НАЗНАЧЕНИЕ:
 * Эта версия программы позволяет тестировать все функции устройства БЕЗ
 * подключения реальных датчиков ЭЭГ и ЭМГ. Сигналы эмулируются внутри Arduino.
 * 
 * КОМАНДЫ УПРАВЛЕНИЯ (Serial Monitor, 115200 бод):
 *   SUCCESS    - Симуляция успешного выполнения (расслаблен + бета-ритм)
 *   FAIL_EMG   - Симуляция провала по мышцам (напряжение лба)
 *   FAIL_EEG   - Симуляция провала по ЭЭГ (нет бета-ритма)
 *   AUTO       - Автоматический полный цикл (калибровка + тренинг)
 *   STATUS     - Показать текущее состояние
 *   HELP       - Полная справка по командам
 *   RESET      - Сброс и перезапуск калибровки
 * 
 * ОБОРУДОВАНИЕ:
 *   - Arduino Mega
 *   - Grove LED Bar v1.0 (пины 2, 3)
 *   - Сервопривод FS5103B (пин 9)
 *   - Датчики НЕ ТРЕБУЮТСЯ (эмулируются программно)
 * 
 * ВЕРСИЯ: 3.0 - Исправлена проблема с дёрганьем серво
 * ============================================================================
 */

#include <Grove_LED_Bar.h>
#include <Servo.h>

// ============================================================================
// НАСТРОЙКИ ПИНОВ
// ============================================================================
const int PIN_LED_DATA     = 2;   // Grove LED Bar DATA
const int PIN_LED_CLK      = 3;   // Grove LED Bar CLOCK
const int PIN_SERVO        = 9;   // Сервопривод PWM

// ============================================================================
// ТАЙМИНГИ (в миллисекундах)
// ============================================================================
const unsigned long TIME_CALIBRATION    = 30000;  // Калибровка: 30 сек
const unsigned long TIME_TRAINING       = 180000; // Тренинг: 3 минуты
const unsigned long SERVO_INTERVAL      = 200;    // Обновление серво: 200 мс
const unsigned long SERIAL_INTERVAL     = 200;    // Отправка данных: 200 мс

// ============================================================================
// ПАРАМЕТРЫ БИОУПРАВЛЕНИЯ
// ============================================================================
const float EMG_THRESHOLD_K      = 1.3;   // Коэффициент порога ЭМГ
const float BETA_THRESHOLD_K     = 0.7;   // Коэффициент порога бета-ритма
const float BETA_SUCCESS_RATIO   = 0.6;   // Требуемый % времени с бета-ритмом

// Скорости серво (градусов за обновление 200мс)
const float LID_SPEED_OPEN       = 0.15;  // Медленное открытие
const float LID_SPEED_CLOSE      = 0.30;  // Быстрое закрытие (×2)

// Углы серво
const int SERVO_CLOSED           = 0;     // Закрыто
const int SERVO_OPEN             = 90;    // Открыто
const int LID_SUCCESS_ANGLE      = 54;    // 60% от 90° = 54° (критерий успеха)

// ============================================================================
// ГЛОБАЛЬНЫЕ ОБЪЕКТЫ
// ============================================================================
Grove_LED_Bar bar(PIN_LED_DATA, PIN_LED_CLK, 1);  // 1 = обратный порядок
Servo servo;

// ============================================================================
// СОСТОЯНИЯ СИСТЕМЫ
// ============================================================================
enum State { CALIBRATION, TRAINING, FINISH };
State currentState = CALIBRATION;

// ============================================================================
// ПЕРЕМЕННЫЕ КАЛИБРОВКИ
// ============================================================================
unsigned long phaseStart = 0;
float emgThreshold = 0;
float betaThreshold = 0;
bool isCalibrated = false;

// ============================================================================
// ПЕРЕМЕННЫЕ ЭМГ (ЭМУЛИРОВАННЫЕ)
// ============================================================================
float emgBuffer[20];         // Буфер на 500 мс
uint8_t emgIdx = 0;
float emgSmoothed = 0;
bool isRelaxed = true;
int emgEmulated = 300;       // Эмулируемое значение ЭМГ

// ============================================================================
// ПЕРЕМЕННЫЕ ЭЭГ (ЭМУЛИРОВАННЫЕ)
// ============================================================================
float betaPower = 0;
float betaEmulated = 0.5;    // Эмулируемая мощность бета-ритма

// Статистика бета-ритма
uint16_t betaAboveCnt = 0;   // Счётчик периодов с высоким бета-ритмом
uint16_t validTimeCnt = 0;   // Счётчик валидных периодов

// ============================================================================
// ПЕРЕМЕННЫЕ СЕРВО
// ============================================================================
float lidAngle = 0;
unsigned long lastServoUpdate = 0;
unsigned long lastSerialSend = 0;

// ============================================================================
// ПЕРЕМЕННЫЕ ДЛЯ АВТОМАТИЧЕСКОГО РЕЖИМА
// ============================================================================
bool autoMode = false;
unsigned long autoStartTime = 0;

// ============================================================================
// ФУНКЦИЯ: Инициализация оборудования
// ============================================================================
void initHardware() {
  // LED Bar
  bar.begin();
  bar.setLevel(0);
  
  // Серво
  servo.attach(PIN_SERVO);
  servo.write(SERVO_CLOSED);
  lidAngle = SERVO_CLOSED;
  
  // Очистка буферов
  for (uint8_t i = 0; i < 20; i++) emgBuffer[i] = 0;
  
  Serial.begin(115200);
  while (!Serial) { ; }
}

// ============================================================================
// ФУНКЦИЯ: Сглаживание ЭМГ (скользящее среднее 500 мс)
// ============================================================================
float smoothEMG(int newValue) {
  emgBuffer[emgIdx] = newValue;
  emgIdx = (emgIdx + 1) % 20;
  
  float sum = 0;
  for (uint8_t i = 0; i < 20; i++) {
    sum += emgBuffer[i];
  }
  return sum / 20.0;
}

// ============================================================================
// ФУНКЦИЯ: Обновление положения крышки (вызывается каждые 200 мс)
// ============================================================================
void updateServo() {
  if (emgSmoothed < emgThreshold) {
    isRelaxed = true;
    lidAngle += LID_SPEED_OPEN;
    if (lidAngle > SERVO_OPEN) lidAngle = SERVO_OPEN;
  } else {
    isRelaxed = false;
    lidAngle -= LID_SPEED_CLOSE;
    if (lidAngle < SERVO_CLOSED) lidAngle = SERVO_CLOSED;
  }
  
  if (lidAngle < 0) lidAngle = 0;
  if (lidAngle > 90) lidAngle = 90;
  
  servo.write((int)lidAngle);
  
  int level = map((int)lidAngle, 0, 90, 0, 10);
  bar.setLevel(level);
}

// ============================================================================
// ФУНКЦИЯ: Генерация значений для авто-режима
// ============================================================================
void updateAutoMode() {
  unsigned long elapsed = millis() - autoStartTime;
  
  if (currentState == CALIBRATION) {
    // Во время калибровки: средние значения с шумом
    emgEmulated = 300 + random(-30, 31);
    betaEmulated = 0.5 + (random(-50, 51) / 1000.0);
  } 
  else if (currentState == TRAINING) {
    unsigned long trainingElapsed = elapsed - TIME_CALIBRATION;
    float progress = (float)trainingElapsed / TIME_TRAINING;
    
    // Первые 2/3 времени: хорошие значения
    if (progress < 0.66) {
      emgEmulated = 250 + random(-20, 21);
      betaEmulated = 0.7 + (random(-30, 31) / 1000.0);
    } else {
      // Последние 1/3: возможны колебания
      emgEmulated = 280 + random(-40, 41);
      betaEmulated = 0.65 + (random(-50, 51) / 1000.0);
    }
  }
  
  // Ограничения
  if (emgEmulated < 0) emgEmulated = 0;
  if (emgEmulated > 1023) emgEmulated = 1023;
  if (betaEmulated < 0) betaEmulated = 0;
  if (betaEmulated > 1.0) betaEmulated = 1.0;
}

// ============================================================================
// ФУНКЦИЯ: Обработка команд из Serial
// ============================================================================
void processCommands() {
  static String inputBuffer = "";
  
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        parseCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
}

// ============================================================================
// ФУНКЦИЯ: Парсинг команды
// ============================================================================
void parseCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  
  if (cmd.length() == 0) return;
  
  if (cmd == "SUCCESS") {
    runScenarioSuccess();
  }
  else if (cmd == "FAIL_EMG") {
    runScenarioFailEmg();
  }
  else if (cmd == "FAIL_EEG") {
    runScenarioFailEeg();
  }
  else if (cmd == "AUTO") {
    startAutoMode();
  }
  else if (cmd == "STATUS") {
    printStatus();
  }
  else if (cmd == "HELP" || cmd == "?") {
    printHelp();
  }
  else if (cmd == "RESET") {
    resetSystem();
  }
  else {
    Serial.print("ERROR: Неизвестная команда '");
    Serial.print(cmd);
    Serial.println("'");
    Serial.println("Введите HELP для справки");
  }
}

// ============================================================================
// СЦЕНАРИЙ: SUCCESS - успешное выполнение
// ============================================================================
void runScenarioSuccess() {
  Serial.println("\n=== СЦЕНАРИЙ: SUCCESS ===");
  Serial.println("Симуляция: расслаблен + бета-ритм выше порога");
  
  resetSystem();
  emgEmulated = 250;
  betaEmulated = 0.75;
  emgThreshold = 400;
  betaThreshold = 0.5;
  isCalibrated = true;
  currentState = TRAINING;
  phaseStart = millis();
  
  Serial.println("Пороги: ЭМГ < 400, Бета > 0.5");
  Serial.println("Ожидайте открытия крышки...\n");
}

// ============================================================================
// СЦЕНАРИЙ: FAIL_EMG - провал по мышцам
// ============================================================================
void runScenarioFailEmg() {
  Serial.println("\n=== СЦЕНАРИЙ: FAIL_EMG ===");
  Serial.println("Симуляция: напряжение мышц лба");
  
  resetSystem();
  emgEmulated = 600;
  betaEmulated = 0.75;
  emgThreshold = 400;
  betaThreshold = 0.5;
  isCalibrated = true;
  currentState = TRAINING;
  phaseStart = millis();
  
  Serial.println("Крышка будет закрываться!\n");
}

// ============================================================================
// СЦЕНАРИЙ: FAIL_EEG - провал по ЭЭГ
// ============================================================================
void runScenarioFailEeg() {
  Serial.println("\n=== СЦЕНАРИЙ: FAIL_EEG ===");
  Serial.println("Симуляция: нет бета-ритма");
  
  resetSystem();
  emgEmulated = 250;
  betaEmulated = 0.3;
  emgThreshold = 400;
  betaThreshold = 0.5;
  isCalibrated = true;
  currentState = TRAINING;
  phaseStart = millis();
  
  Serial.println("Крышка откроется, но камни НЕ появятся!\n");
}

// ============================================================================
// ФУНКЦИЯ: Запуск автоматического режима
// ============================================================================
void startAutoMode() {
  Serial.println("\n=== АВТОМАТИЧЕСКИЙ РЕЖИМ ===");
  Serial.println("Будет выполнена полная симуляция: калибровка + тренинг");
  
  resetSystem();
  autoMode = true;
  autoStartTime = millis();
  currentState = CALIBRATION;
  phaseStart = millis();
}

// ============================================================================
// ФУНКЦИЯ: Сброс системы
// ============================================================================
void resetSystem() {
  currentState = CALIBRATION;
  phaseStart = millis();
  isCalibrated = false;
  emgThreshold = 0;
  betaThreshold = 0;
  lidAngle = 0;
  servo.write(SERVO_CLOSED);
  bar.setLevel(0);
  betaAboveCnt = 0;
  validTimeCnt = 0;
  emgEmulated = 300;
  betaEmulated = 0.5;
  autoMode = false;
  
  for (uint8_t i = 0; i < 20; i++) emgBuffer[i] = 0;
  
  Serial.println("# Система сброшена");
}

// ============================================================================
// ФУНКЦИЯ: Печать справки
// ============================================================================
void printHelp() {
  Serial.println("\n+-----------------------------------------------------------+");
  Serial.println("|           КОМАНДЫ УПРАВЛЕНИЯ \"СУНДУК СОКРОВИЩ\"            |");
  Serial.println("+-----------------------------------------------------------+");
  Serial.println("| SUCCESS   - Симуляция УСПЕХА                              |");
  Serial.println("| FAIL_EMG  - Провал: напряжение мышц                       |");
  Serial.println("| FAIL_EEG  - Провал: нет бета-ритма                        |");
  Serial.println("| AUTO      - Полный автоматический цикл                    |");
  Serial.println("| STATUS    - Показать текущее состояние                    |");
  Serial.println("| HELP      - Эта справка                                   |");
  Serial.println("| RESET     - Сброс системы                                 |");
  Serial.println("+-----------------------------------------------------------+\n");
}

// ============================================================================
// ФУНКЦИЯ: Печать статуса
// ============================================================================
void printStatus() {
  Serial.println("\n=== ТЕКУЩЕЕ СОСТОЯНИЕ ===");
  Serial.print("Состояние: ");
  switch (currentState) {
    case CALIBRATION: Serial.println("КАЛИБРОВКА"); break;
    case TRAINING: Serial.println("ТРЕНИНГ"); break;
    case FINISH: Serial.println("ФИНИШ"); break;
  }
  Serial.print("ЭМГ: "); Serial.print(emgEmulated);
  Serial.print(" | Порог: "); Serial.println(emgThreshold, 1);
  Serial.print("Бета: "); Serial.print(betaEmulated, 3);
  Serial.print(" | Порог: "); Serial.println(betaThreshold, 3);
  Serial.print("Угол крышки: "); Serial.print(lidAngle, 1);
  Serial.print("° | Расслаблен: "); Serial.println(isRelaxed ? "ДА" : "НЕТ");
  Serial.print("Прогресс бета: "); Serial.print(betaAboveCnt);
  Serial.print("/"); Serial.println(validTimeCnt);
  Serial.println("=======================\n");
}

// ============================================================================
// ФУНКЦИЯ: Оценка результата
// ============================================================================
void evaluateResult() {
  bool lidSuccess = (lidAngle >= LID_SUCCESS_ANGLE);
  
  bool betaSuccess = false;
  if (validTimeCnt > 0) {
    float ratio = (float)betaAboveCnt / validTimeCnt;
    betaSuccess = (ratio >= BETA_SUCCESS_RATIO);
    
    Serial.print("#RESULT\t");
    Serial.print(ratio, 3);
    Serial.print('\t');
    Serial.println(betaSuccess ? "PASS" : "FAIL");
  }
  
  if (lidSuccess && betaSuccess) {
    Serial.println("✓ УСПЕХ: Все светодиоды горят!");
    bar.setLevel(10);
    
    for (uint8_t i = 0; i < 5; i++) {
      bar.setBits(0b0000000011);
      delay(200);
      bar.setBits(0);
      delay(200);
    }
    bar.setLevel(10);
  } else {
    Serial.println("✗ НЕУДАЧА: Красные светодиоды мигают");
    for (uint8_t i = 0; i < 3; i++) {
      bar.setBits(0b1111111000);
      delay(300);
      bar.setBits(0);
      delay(300);
    }
  }
  
  while (true) { delay(1000); }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  initHardware();
  
  phaseStart = millis();
  
  printHelp();
  Serial.println("Загрузка... Нажмите HELP для команд");
  
  bar.setLevel(1);
  delay(200);
  bar.setLevel(0);
  delay(200);
  bar.setLevel(1);
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  unsigned long currentTime = millis();
  
  // === ОБРАБОТКА КОМАНД ===
  processCommands();
  
  // === АВТОМАТИЧЕСКИЙ РЕЖИМ ===
  if (autoMode) {
    updateAutoMode();
  }
  
  // === СГЛАЖИВАНИЕ ЭМГ ===
  emgSmoothed = smoothEMG(emgEmulated);
  betaPower = betaEmulated;
  
  // === ОТПРАВКА ДАННЫХ (каждые 200 мс) ===
  if (currentTime - lastSerialSend >= SERIAL_INTERVAL) {
    Serial.print("EEG:"); Serial.print(rawEEG);
    Serial.print(" EMG:"); Serial.print(emgEmulated);
    Serial.print(" Beta:"); Serial.print(betaPower, 3);
    Serial.print(" Angle:"); Serial.println(lidAngle, 1);
    lastSerialSend = currentTime;
  }
  
  // === ОБНОВЛЕНИЕ СЕРВО (каждые 200 мс) ===
  if (currentTime - lastServoUpdate >= SERVO_INTERVAL) {
    updateServo();
    lastServoUpdate = currentTime;
  }
  
  // === МАШИНА СОСТОЯНИЙ ===
  switch (currentState) {
    case CALIBRATION: {
      unsigned long elapsed = millis() - phaseStart;
      if (elapsed >= TIME_CALIBRATION) {
        isCalibrated = true;
        currentState = TRAINING;
        phaseStart = millis();
        betaAboveCnt = 0;
        validTimeCnt = 0;
        lidAngle = 0;
        servo.write(SERVO_CLOSED);
        
        bar.setLevel(10);
        delay(300);
        bar.setLevel(0);
        delay(300);
        bar.setLevel(10);
        delay(300);
        bar.setLevel(0);
        
        Serial.println("# Калибровка завершена");
      } else {
        // Расчёт порогов во время калибровки
        static float emgSum = 0;
        static uint16_t emgCnt = 0;
        static float betaMax = 0;
        
        emgSum += emgSmoothed;
        emgCnt++;
        
        if (elapsed >= TIME_CALIBRATION / 2 && emgCnt >= 20) {
          emgThreshold = (emgSum / emgCnt) * EMG_THRESHOLD_K;
        }
        
        if (betaPower > betaMax) betaMax = betaPower;
        betaThreshold = betaMax * BETA_THRESHOLD_K;
        
        int level = map(elapsed, 0, TIME_CALIBRATION, 0, 10);
        bar.setLevel(level);
      }
      break;
    }
    
    case TRAINING: {
      unsigned long elapsed = millis() - phaseStart;
      if (elapsed >= TIME_TRAINING) {
        currentState = FINISH;
      } else {
        validTimeCnt++;
        if (isRelaxed && lidAngle > 10) {
          if (betaPower > betaThreshold) {
            betaAboveCnt++;
          }
        }
      }
      break;
    }
    
    case FINISH: {
      evaluateResult();
      break;
    }
  }
  
  delay(5);
}
