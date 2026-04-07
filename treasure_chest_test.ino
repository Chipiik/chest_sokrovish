/*
 * ============================================================================
 * ПРОЕКТ: "СУНДУК СОКРОВИЩ" - ТЕСТОВАЯ ПРОГРАММА С ЭМУЛЯЦИЕЙ ДАТЧИКОВ
 * ============================================================================
 * 
 * НАЗНАЧЕНИЕ:
 * Эта версия программы позволяет тестировать все функции устройства БЕЗ
 * подключения реальных датчиков ЭЭГ и ЭМГ. Сигналы эмулируются через
 * последовательный порт (Serial) с компьютера.
 * 
 * КАК ЭТО РАБОТАЕТ:
 * 1. Загрузите этот скетч в Arduino Mega
 * 2. Откройте Serial Monitor (115200 бод)
 * 3. Отправляйте команды для управления эмуляцией:
 * 
 * КОМАНДЫ УПРАВЛЕНИЯ:
 *   E <значение>  - установить уровень ЭМГ (0-1023)
 *   B <значение>  - установить мощность бета-ритма (0.0-1.0)
 *   R             - начать калибровку заново
 *   S             - показать текущее состояние
 *   A             - автоматический режим (синусоидальная эмуляция)
 *   M             - ручной режим (вы вводите значения)
 * 
 * ПРИМЕРЫ:
 *   E 300         - установить ЭМГ = 300
 *   B 0.65        - установить бета-ритм = 0.65
 *   E 250 B 0.7   - установить оба значения сразу
 * 
 * АВТОМАТИЧЕСКИЙ РЕЖИМ (A):
 *   - ЭМГ плавно меняется от 200 до 500 (симуляция расслабления/напряжения)
 *   - Бета-ритм меняется от 0.3 до 0.8 (симуляция когнитивной нагрузки)
 *   - Период цикла: 20 секунд
 * 
 * ОБОРУДОВАНИЕ:
 *   - Arduino Mega
 *   - Grove LED Bar v1.0 (пины 2, 3)
 *   - Сервопривод FS5103B (пин 9)
 *   - Датчики НЕ ТРЕБУЮТСЯ (эмулируются)
 * 
 * ВЕРСИЯ: 3.0 (Test Mode)
 * ============================================================================
 */

#include <Grove_LED_Bar.h>
#include <Servo.h>

// ============================================================================
// НАСТРОЙКИ ПИНОВ
// ============================================================================
const int PIN_LED_DATA     = 2;   // Grove LED Bar DATA
const int PIN_LED_CLK      = 3;   // Grove LED Bar CLOCK
const int PIN_SERVO        = 9;   // Сервопривод

// Пины датчиков НЕ используются в тестовом режиме
// const int PIN_EEG          = A0;  
// const int PIN_EMG          = A1;  

// ============================================================================
// ТАЙМИНГИ (в миллисекундах)
// ============================================================================
const unsigned long TIME_CALIBRATION    = 30000;  // Калибровка: 30 сек
const unsigned long TIME_TRAINING       = 180000; // Тренинг: 3 минуты
const unsigned long SERVO_INTERVAL      = 200;    // Обновление серво: 200 мс
const unsigned long EMG_SMOOTH_WINDOW   = 500;    // Окно сглаживания ЭМГ: 500 мс
const unsigned long SERIAL_INTERVAL     = 100;    // Отправка данных: 100 мс

// ============================================================================
// ПАРАМЕТРЫ БИОУПРАВЛЕНИЯ
// ============================================================================
const float EMG_THRESHOLD_K      = 1.3;   // Коэффициент порога ЭМГ
const float BETA_THRESHOLD_K     = 0.7;   // Коэффициент порога бета-ритма
const float BETA_SUCCESS_RATIO   = 0.6;   // Требуемый % времени с бета-ритмом

// Скорости серво (градусов за обновление)
const float LID_SPEED_OPEN       = 0.15;  // Медленное открытие
const float LID_SPEED_CLOSE      = 0.30;  // Быстрое закрытие (в 2 раза быстрее)

// Углы серво
const int SERVO_CLOSED           = 0;     // Закрыто
const int SERVO_OPEN             = 90;    // Открыто
const int LID_SUCCESS_ANGLE      = 54;    // 60% от 90° = 54° (критерий успеха)

// Диапазон бета-ритма (Гц)
const float BETA_FREQ_LOW        = 13.0;  // Для подростков (13-18 Гц)
const float BETA_FREQ_HIGH       = 18.0;

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
// ПЕРЕМЕННЫЕ ЭМГ
// ============================================================================
float emgBuffer[20];         // Буфер на 500 мс (20 × 25 мс)
uint8_t emgIdx = 0;
float emgSmoothed = 0;
bool isRelaxed = true;
int emgEmulated = 300;       // Эмулируемое значение ЭМГ

// ============================================================================
// ПЕРЕМЕННЫЕ ЭЭГ
// ============================================================================
float betaPower = 0;
float betaEmulated = 0.5;    // Эмулируемая мощность бета-ритма

// Статистика бета-ритма
uint16_t betaAboveCnt = 0;   // Счётчик периодов с высоким бета-ритмом
uint16_t validTimeCnt = 0;   // Счётчик валидных периодов (когда расслаблен)

// ============================================================================
// ПЕРЕМЕННЫЕ СЕРВО
// ============================================================================
float lidAngle = 0;
unsigned long lastServoUpdate = 0;

// ============================================================================
// ПЕРЕМЕННЫЕ ДЛЯ АВТОМАТИЧЕСКОГО РЕЖИМА
// ============================================================================
bool autoMode = true;        // true = авто, false = ручной
unsigned long autoStartTime = 0;
const unsigned long AUTO_CYCLE_TIME = 20000;  // 20 секунд цикл

// ============================================================================
// ПЕРЕМЕННЫЕ ДЛЯ ОТПРАВКИ ДАННЫХ
// ============================================================================
unsigned long lastSerialSend = 0;
unsigned long lastStatusPrint = 0;

// ============================================================================
// ФУНКЦИЯ: Инициализация оборудования
// ============================================================================
void initHardware() {
  // LED Bar
  bar.begin();
  bar.setBrightness(10);
  bar.setLevel(0);
  
  // Серво
  servo.attach(PIN_SERVO);
  servo.write(SERVO_CLOSED);
  lidAngle = SERVO_CLOSED;
  
  // Очистка буферов
  for (uint8_t i = 0; i < 20; i++) emgBuffer[i] = 0;
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
// ФУНКЦИЯ: Обновление положения крышки
// ============================================================================
void updateServo() {
  // Проверка уровня ЭМГ
  if (emgSmoothed < emgThreshold) {
    isRelaxed = true;
    // Расслаблен → открываем МЕДЛЕННО
    lidAngle += LID_SPEED_OPEN;
    if (lidAngle > SERVO_OPEN) lidAngle = SERVO_OPEN;
  } else {
    isRelaxed = false;
    // Напряжён → закрываем БЫСТРО (в 2 раза быстрее)
    lidAngle -= LID_SPEED_CLOSE;
    if (lidAngle < SERVO_CLOSED) lidAngle = SERVO_CLOSED;
  }
  
  // Ограничение диапазона
  if (lidAngle < 0) lidAngle = 0;
  if (lidAngle > 90) lidAngle = 90;
  
  // Установка угла серво
  servo.write((int)lidAngle);
  
  // Визуализация на LED Bar (прогресс открытия)
  int level = map((int)lidAngle, 0, 90, 0, 10);
  bar.setLevel(level);
}

// ============================================================================
// ФУНКЦИЯ: Генерация значений в автоматическом режиме
// ============================================================================
void updateAutoMode() {
  unsigned long elapsed = millis() - autoStartTime;
  float progress = (float)(elapsed % AUTO_CYCLE_TIME) / AUTO_CYCLE_TIME;
  
  // ЭМГ: синусоида от 200 до 500
  // Первые 10 сек - низкий ЭМГ (расслабление), следующие 10 сек - высокий
  if (progress < 0.5) {
    // Расслабление: 200-300
    emgEmulated = 200 + (int)(100 * sin(progress * PI * 2));
  } else {
    // Напряжение: 400-500
    emgEmulated = 400 + (int)(100 * sin((progress - 0.5) * PI * 2));
  }
  
  // Бета-ритм: меняется от 0.3 до 0.8
  // Максимум в середине каждого полуцикла
  betaEmulated = 0.55 + 0.25 * sin(progress * PI * 4);
  
  // Добавляем небольшой шум
  emgEmulated += random(-20, 21);
  betaEmulated += (random(-50, 51) / 1000.0);
  
  // Ограничения
  if (emgEmulated < 0) emgEmulated = 0;
  if (emgEmulated > 1023) emgEmulated = 1023;
  if (betaEmulated < 0) betaEmulated = 0;
  if (betaEmulated > 1.0) betaEmulated = 1.0;
}

// ============================================================================
// ФУНКЦИЯ: Обработка команд из Serial
// ============================================================================
void processSerialCommand() {
  static String inputBuffer = "";
  
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      // Конец команды
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
  
  // Команда: E <значение>
  if (cmd.startsWith("E ")) {
    int val = cmd.substring(2).toInt();
    if (val >= 0 && val <= 1023) {
      emgEmulated = val;
      Serial.print("# OK: EMG установлен в ");
      Serial.println(val);
    }
    return;
  }
  
  // Команда: B <значение>
  if (cmd.startsWith("B ")) {
    float val = cmd.substring(2).toFloat();
    if (val >= 0.0 && val <= 1.0) {
      betaEmulated = val;
      Serial.print("# OK: Beta установлен в ");
      Serial.println(val, 3);
    }
    return;
  }
  
  // Команда: E <val1> B <val2>
  if (cmd.startsWith("E ") && cmd.indexOf(" B ") > 0) {
    int spacePos = cmd.indexOf(" ");
    int bPos = cmd.indexOf(" B ");
    int eVal = cmd.substring(2, spacePos).toInt();
    float bVal = cmd.substring(bPos + 3).toFloat();
    
    if (eVal >= 0 && eVal <= 1023 && bVal >= 0.0 && bVal <= 1.0) {
      emgEmulated = eVal;
      betaEmulated = bVal;
      Serial.print("# OK: EMG=");
      Serial.print(eVal);
      Serial.print(", Beta=");
      Serial.println(bVal, 3);
    }
    return;
  }
  
  // Команда: R (сброс калибровки)
  if (cmd == "R") {
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
    Serial.println("# OK: Калибровка начата заново");
    return;
  }
  
  // Команда: S (статус)
  if (cmd == "S") {
    printStatus();
    return;
  }
  
  // Команда: A (авторежим)
  if (cmd == "A") {
    autoMode = true;
    autoStartTime = millis();
    Serial.println("# OK: Автоматический режим включен");
    return;
  }
  
  // Команда: M (ручной режим)
  if (cmd == "M") {
    autoMode = false;
    Serial.println("# OK: Ручной режим включен");
    Serial.println("# Используйте команды: E <0-1023>, B <0.0-1.0>");
    return;
  }
  
  // Команда: H (помощь)
  if (cmd == "H" || cmd == "?") {
    printHelp();
    return;
  }
  
  // Неизвестная команда
  Serial.print("# ERROR: Неизвестная команда '");
  Serial.print(cmd);
  Serial.println("'");
  Serial.println("# Введите 'H' для справки");
}

// ============================================================================
// ФУНКЦИЯ: Печать справки
// ============================================================================
void printHelp() {
  Serial.println("\n=== КОМАНДЫ УПРАВЛЕНИЯ ===");
  Serial.println("E <0-1023>       - Установить уровень ЭМГ");
  Serial.println("B <0.0-1.0>      - Установить мощность бета-ритма");
  Serial.println("E <val> B <val>  - Установить оба значения");
  Serial.println("R                - Начать калибровку заново");
  Serial.println("S                - Показать текущее состояние");
  Serial.println("A                - Автоматический режим (синусоидальная эмуляция)");
  Serial.println("M                - Ручной режим (вы вводите значения)");
  Serial.println("H или ?          - Эта справка");
  Serial.println("=========================\n");
}

// ============================================================================
// ФУНКЦИЯ: Печать статуса
// ============================================================================
void printStatus() {
  Serial.println("\n=== ТЕКУЩЕЕ СОСТОЯНИЕ ===");
  Serial.print("Режим: ");
  Serial.println(autoMode ? "АВТОМАТИЧЕСКИЙ" : "РУЧНОЙ");
  Serial.print("Состояние: ");
  switch (currentState) {
    case CALIBRATION: Serial.println("КАЛИБРОВКА"); break;
    case TRAINING: Serial.println("ТРЕНИНГ"); break;
    case FINISH: Serial.println("ФИНИШ"); break;
  }
  Serial.print("ЭМГ: ");
  Serial.print(emgEmulated);
  Serial.print(" (сглаженный: ");
  Serial.print(emgSmoothed, 1);
  Serial.print(", порог: ");
  Serial.print(emgThreshold, 1);
  Serial.println(")");
  Serial.print("Бета-ритм: ");
  Serial.print(betaEmulated, 3);
  Serial.print(" (порог: ");
  Serial.print(betaThreshold, 3);
  Serial.println(")");
  Serial.print("Угол крышки: ");
  Serial.print(lidAngle, 1);
  Serial.println(" градусов");
  Serial.print("Расслабление: ");
  Serial.println(isRelaxed ? "ДА" : "НЕТ");
  Serial.print("Бета выше порога: ");
  Serial.print(betaAboveCnt);
  Serial.print(" из ");
  Serial.print(validTimeCnt);
  if (validTimeCnt > 0) {
    Serial.print(" (");
    Serial.print((float)betaAboveCnt / validTimeCnt * 100, 1);
    Serial.println("%)");
  } else {
    Serial.println();
  }
  Serial.println("=========================\n");
}

// ============================================================================
// ФУНКЦИЯ: Отправка данных на компьютер (для визуализации)
// ============================================================================
void sendToPC() {
  // Формат: RAW_EEG, RAW_EMG, BETA_POWER, LID_ANGLE, THRESHOLD_FLAG
  Serial.print(emgEmulated);
  Serial.print('\t');
  Serial.print(betaEmulated, 4);
  Serial.print('\t');
  Serial.print(lidAngle, 1);
  Serial.print('\t');
  Serial.println(isRelaxed ? 1 : 0);
}

// ============================================================================
// СОСТОЯНИЕ: КАЛИБРОВКА (30 секунд)
// ============================================================================
void runCalibration() {
  unsigned long elapsed = millis() - phaseStart;
  
  // Завершение калибровки
  if (elapsed >= TIME_CALIBRATION) {
    isCalibrated = true;
    currentState = TRAINING;
    phaseStart = millis();
    
    // Сигнал успешной калибровки
    bar.setLevel(10);
    delay(300);
    bar.setLevel(0);
    delay(300);
    bar.setLevel(10);
    delay(300);
    bar.setLevel(0);
    
    // Сброс статистики для тренинга
    betaAboveCnt = 0;
    validTimeCnt = 0;
    lidAngle = 0;
    servo.write(SERVO_CLOSED);
    
    Serial.println("# КАЛИБРОВКА ЗАВЕРШЕНА");
    Serial.print("# Порог ЭМГ: ");
    Serial.println(emgThreshold, 1);
    Serial.print("# Порог бета-ритма: ");
    Serial.println(betaThreshold, 3);
    return;
  }
  
  // === РАСЧЁТ ПОРОГА ЭМГ ===
  float smoothed = smoothEMG(emgEmulated);
  static float emgSum = 0;
  static uint16_t emgCnt = 0;
  
  emgSum += smoothed;
  emgCnt++;
  
  // Вычисляем порог ЭМГ на середине калибровки
  if (elapsed >= TIME_CALIBRATION / 2 && emgCnt >= 20) {
    emgThreshold = (emgSum / emgCnt) * EMG_THRESHOLD_K;
  }
  
  // === РАСЧЁТ ПОРОГА БЕТА-РИТМА ===
  static float betaMax = 0;
  
  // Запоминаем максимум бета-ритма
  if (betaEmulated > betaMax) betaMax = betaEmulated;
  betaThreshold = betaMax * BETA_THRESHOLD_K;
  
  // === ВИЗУАЛИЗАЦИЯ ПРОГРЕССА КАЛИБРОВКИ ===
  int level = map(elapsed, 0, TIME_CALIBRATION, 0, 10);
  bar.setLevel(level);
  
  // Вывод прогресса каждые 5 секунд
  if (elapsed % 5000 < 100 && elapsed > 0) {
    Serial.print("# Калибровка: ");
    Serial.print(elapsed / 1000);
    Serial.print(" сек, ЭМГ порог: ");
    Serial.print(emgThreshold, 1);
    Serial.print(", Бета порог: ");
    Serial.println(betaThreshold, 3);
  }
}

// ============================================================================
// СОСТОЯНИЕ: ТРЕНИНГ (3 минуты)
// ============================================================================
void runTraining() {
  unsigned long elapsed = millis() - phaseStart;
  
  // Завершение тренинга
  if (elapsed >= TIME_TRAINING) {
    currentState = FINISH;
    Serial.println("# ТРЕНИНГ ЗАВЕРШЕН");
    return;
  }
  
  // === ОБРАБОТКА ЭЭГ ===
  betaPower = betaEmulated;
  validTimeCnt++;
  
  // Считаем бета-ритм ТОЛЬКО когда расслаблен
  if (isRelaxed && lidAngle > 10) {
    if (betaPower > betaThreshold) {
      betaAboveCnt++;
    }
  }
  
  // Вывод прогресса каждые 30 секунд
  if (elapsed % 30000 < 200 && elapsed > 0) {
    float ratio = validTimeCnt > 0 ? (float)betaAboveCnt / validTimeCnt : 0;
    Serial.print("# Прогресс: ");
    Serial.print(elapsed / 1000);
    Serial.print(" сек, Бета > порога: ");
    Serial.print(ratio * 100, 1);
    Serial.print("% (требуется 60%), Угол: ");
    Serial.print(lidAngle, 1);
    Serial.println(" град.");
  }
}

// ============================================================================
// СОСТОЯНИЕ: ФИНИШ (оценка результата)
// ============================================================================
void runFinish() {
  evaluateResult();
}

// ============================================================================
// ФУНКЦИЯ: Оценка результата
// ============================================================================
void evaluateResult() {
  static bool evaluated = false;
  
  if (evaluated) {
    delay(1000);
    return;
  }
  
  evaluated = true;
  
  // Критерий 1: Крышка открыта ≥60% (≥54 градуса)
  bool lidSuccess = (lidAngle >= LID_SUCCESS_ANGLE);
  
  // Критерий 2: Бета-ритм > порога ≥60% времени
  bool betaSuccess = false;
  float ratio = 0;
  
  if (validTimeCnt > 0) {
    ratio = (float)betaAboveCnt / validTimeCnt;
    betaSuccess = (ratio >= BETA_SUCCESS_RATIO);
  }
  
  // === ВЫВОД РЕЗУЛЬТАТА ===
  Serial.println("\n===========================================");
  Serial.println("           РЕЗУЛЬТАТЫ ИСПЫТАНИЯ");
  Serial.println("===========================================");
  Serial.print("Угол открытия крышки: ");
  Serial.print(lidAngle, 1);
  Serial.print(" / ");
  Serial.print(LID_SUCCESS_ANGLE);
  Serial.println(" градусов");
  Serial.print("Крышка открыта ≥60%: ");
  Serial.println(lidSuccess ? "✓ ДА" : "✗ НЕТ");
  Serial.println();
  Serial.print("Бета-ритм выше порога: ");
  Serial.print(ratio * 100, 1);
  Serial.print("% / 60%");
  Serial.println();
  Serial.print("Бета-ритм ≥60% времени: ");
  Serial.println(betaSuccess ? "✓ ДА" : "✗ НЕТ");
  Serial.println();
  Serial.println("-------------------------------------------");
  Serial.print("ОБЩИЙ РЕЗУЛЬТАТ: ");
  
  if (lidSuccess && betaSuccess) {
    Serial.println("✓ УСПЕХ! Драгоценные камни найдены!");
    Serial.println("===========================================");
    
    // ✓ УСПЕХ: зажигаем все светодиоды ("драгоценные камни")
    bar.setLevel(10);
    
    // Мигаем зелёным 5 раз
    for (uint8_t i = 0; i < 5; i++) {
      bar.setBits(0b0000000011);  // Только зелёные
      delay(200);
      bar.setBits(0);
      delay(200);
    }
    bar.setLevel(10);  // Оставляем включенными
    
  } else {
    Serial.println("✗ НЕУДАЧА! Попробуйте ещё раз.");
    Serial.println("===========================================");
    
    // ✗ НЕУДАЧА: мигаем красными
    for (uint8_t i = 0; i < 3; i++) {
      bar.setBits(0b1111111000);  // Только красные
      delay(300);
      bar.setBits(0);
      delay(300);
    }
  }
  
  Serial.println("\n# Введите 'R' для перезапуска");
  printStatus();
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);  // Скорость для Serial Monitor
  
  // Ждём открытия Serial порта
  delay(2000);
  
  initHardware();
  
  phaseStart = millis();
  autoStartTime = millis();
  
  // Сигнал начала работы
  bar.setLevel(1);
  delay(200);
  bar.setLevel(0);
  delay(200);
  bar.setLevel(1);
  
  Serial.println("\n===========================================");
  Serial.println("  СУНДУК СОКРОВИЩ - ТЕСТОВЫЙ РЕЖИМ");
  Serial.println("===========================================");
  Serial.println("Загрузка завершена. Начало калибровки...");
  Serial.println("Введите 'H' для списка команд");
  Serial.println("===========================================\n");
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  unsigned long currentTime = millis();
  
  // === ОБРАБОТКА КОМАНД ИЗ SERIAL ===
  processSerialCommand();
  
  // === АВТОМАТИЧЕСКИЙ РЕЖИМ ===
  if (autoMode) {
    updateAutoMode();
  }
  
  // === СГЛАЖИВАНИЕ ЭМГ ===
  emgSmoothed = smoothEMG(emgEmulated);
  
  // === ОТПРАВКА ДАННЫХ (каждые 100 мс) ===
  if (currentTime - lastSerialSend >= SERIAL_INTERVAL) {
    sendToPC();
    lastSerialSend = currentTime;
  }
  
  // === ОБНОВЛЕНИЕ СЕРВО (каждые 200 мс) ===
  if (currentTime - lastServoUpdate >= SERVO_INTERVAL) {
    updateServo();
    lastServoUpdate = currentTime;
  }
  
  // === МАШИНА СОСТОЯНИЙ ===
  switch (currentState) {
    case CALIBRATION:
      runCalibration();
      break;
      
    case TRAINING:
      runTraining();
      break;
      
    case FINISH:
      runFinish();
      break;
  }
  
  // Небольшая пауза
  delay(5);
}
