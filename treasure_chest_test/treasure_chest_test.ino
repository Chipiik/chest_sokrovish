/*
 * ============================================================================
 * ПРОЕКТ: "СУНДУК СОКРОВИЩ" - ТЕСТОВАЯ ПРОГРАММА С ЭМУЛЯЦИЕЙ ДАТЧИКОВ
 * ============================================================================
 * 
 * НАЗНАЧЕНИЕ:
 * Эта версия программы позволяет тестировать все функции устройства БЕЗ
 * подключения реальных датчиков ЭЭГ и ЭМГ. Сигналы эмулируются внутри Arduino.
 * 
 * КАК ЭТО РАБОТАЕТ:
 * 1. Загрузите этот скетч в Arduino Mega
 * 2. Откройте Serial Monitor (115200 бод)
 * 3. Вводите команды для управления сценариями:
 * 
 * КОМАНДЫ УПРАВЛЕНИЯ:
 *   SUCCESS    - Симуляция успешного выполнения (расслаблен + бета-ритм)
 *   FAIL_EMG   - Симуляция провала по мышцам (напряжение лба)
 *   FAIL_EEG   - Симуляция провала по ЭЭГ (нет бета-ритма)
 *   A CYCLE    - Автоматический полный цикл (калибровка + тренинг)
 *   I          - Показать текущее состояние (Info)
 *   H          - Полная справка по командам
 *   R          - Сброс и перезапуск калибровки
 * 
 * ОБОРУДОВАНИЕ:
 *   - Arduino Mega
 *   - Grove LED Bar v1.0 (пины 2, 3) - показывает прогресс открытия
 *   - Сервопривод FS5103B (пин 9) - открывает крышку сундука
 *   - Датчики НЕ ТРЕБУЮТСЯ (эмулируются программно)
 * 
 * ВЕРСИЯ: 4.0 (Test Mode с новыми командами)
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

// ============================================================================
// ТАЙМИНГИ (в миллисекундах)
// ============================================================================
const unsigned long TIME_CALIBRATION    = 30000;  // Калибровка: 30 сек
const unsigned long TIME_TRAINING       = 30000; // Тренинг: 3 минуты
const unsigned long SERVO_INTERVAL      = 200;    // Обновление серво: 200 мс
const unsigned long EMG_SMOOTH_WINDOW   = 500;    // Окно сглаживания ЭМГ: 500 мс
const unsigned long GRAPH_INTERVAL      = 200;    // Обновление графика: 200 мс

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
bool autoCycleRunning = false;
unsigned long autoStartTime = 0;

// ============================================================================
// ПЕРЕМЕННЫЕ ДЛЯ ОТРИСОВКИ ГРАФИКОВ
// ============================================================================
unsigned long lastGraphUpdate = 0;

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
  while (!Serial) {
    ; // Ждём подключения Serial
  }
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
// ФУНКЦИЯ: Генерация значений для авто-цикла (калибровка + тренинг)
// ============================================================================
void updateAutoCycle() {
  unsigned long elapsed = millis() - autoStartTime;
  
  if (currentState == CALIBRATION) {
    // Во время калибровки: средние значения с небольшим шумом
    emgEmulated = 300 + random(-30, 31);
    betaEmulated = 0.5 + (random(-50, 51) / 1000.0);
  } 
  else if (currentState == TRAINING) {
    unsigned long trainingElapsed = elapsed - TIME_CALIBRATION;
    float progress = (float)trainingElapsed / TIME_TRAINING;
    
    // Первые 2 минуты: хорошие значения (расслабление + бета-ритм)
    // Последняя минута: возможны помехи
    if (progress < 0.66) {
      // Хорошие значения: низкий ЭМГ, высокий бета-ритм
      emgEmulated = 250 + random(-20, 21);
      betaEmulated = 0.7 + (random(-30, 31) / 1000.0);
    } else {
      // Небольшие колебания
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
  
  // Команда: SUCCESS - симуляция успешного выполнения
  if (cmd == "SUCCESS") {
    runSuccessScenario();
    return;
  }
  
  // Команда: FAIL_EMG - симуляция провала по мышцам
  if (cmd == "FAIL_EMG") {
    runFailEmgScenario();
    return;
  }
  
  // Команда: FAIL_EEG - симуляция провала по ЭЭГ
  if (cmd == "FAIL_EEG") {
    runFailEegScenario();
    return;
  }
  
  // Команда: A CYCLE или ACYCLE - автоматический полный цикл
  if (cmd == "A CYCLE" || cmd == "ACYCLE") {
    startAutoCycle();
    return;
  }
  
  // Команда: I - показать текущее состояние (Info)
  if (cmd == "I") {
    printStatus();
    return;
  }
  
  // Команда: H - полная справка
  if (cmd == "H" || cmd == "?") {
    printHelp();
    return;
  }
  
  // Команда: R - сброс и перезапуск калибровки
  if (cmd == "R") {
    resetSystem();
    return;
  }
  
  // Неизвестная команда
  Serial.print("# ERROR: Неизвестная команда '");
  Serial.print(cmd);
  Serial.println("'");
  Serial.println("# Введите 'H' для справки");
}

// ============================================================================
// ФУНКЦИЯ: Сценарий SUCCESS - успешное выполнение
// ============================================================================
void runSuccessScenario() {
  Serial.println("\n=== ЗАПУСК СЦЕНАРИЯ: SUCCESS ===");
  Serial.println("Симуляция: Испытуемый расслаблен + бета-ритм выше порога");
  
  resetSystem();
  
  // Устанавливаем низкий ЭМГ (расслабление) и высокий бета-ритм
  emgEmulated = 250;
  betaEmulated = 0.75;
  
  // Пропускаем калибровку мгновенно
  emgThreshold = 400;  // Порог выше текущего значения
  betaThreshold = 0.5; // Порог ниже текущего значения
  isCalibrated = true;
  currentState = TRAINING;
  phaseStart = millis();
  
  Serial.println("Пороги установлены: ЭМГ < 400, Бета > 0.5");
  Serial.println("Ожидайте открытия крышки...\n");
}

// ============================================================================
// ФУНКЦИЯ: Сценарий FAIL_EMG - провал по мышцам
// ============================================================================
void runFailEmgScenario() {
  Serial.println("\n=== ЗАПУСК СЦЕНАРИЯ: FAIL_EMG ===");
  Serial.println("Симуляция: Испытуемый напрягает мышцы лба");
  
  resetSystem();
  
  // Устанавливаем высокий ЭМГ (напряжение)
  emgEmulated = 600;
  betaEmulated = 0.75;
  
  // Пропускаем калибровку мгновенно
  emgThreshold = 400;  // Порог ниже текущего значения → провал
  betaThreshold = 0.5;
  isCalibrated = true;
  currentState = TRAINING;
  phaseStart = millis();
  
  Serial.println("Пороги установлены: ЭМГ < 400, Бета > 0.5");
  Serial.println("Крышка будет закрываться из-за напряжения мышц!\n");
}

// ============================================================================
// ФУНКЦИЯ: Сценарий FAIL_EEG - провал по ЭЭГ
// ============================================================================
void runFailEegScenario() {
  Serial.println("\n=== ЗАПУСК СЦЕНАРИЯ: FAIL_EEG ===");
  Serial.println("Симуляция: Нет бета-ритма (рассеянное внимание)");
  
  resetSystem();
  
  // Устанавливаем низкий ЭМГ (расслабление) но низкий бета-ритм
  emgEmulated = 250;
  betaEmulated = 0.3;
  
  // Пропускаем калибровку мгновенно
  emgThreshold = 400;
  betaThreshold = 0.5; // Порог выше текущего значения → провал
  isCalibrated = true;
  currentState = TRAINING;
  phaseStart = millis();
  
  Serial.println("Пороги установлены: ЭМГ < 400, Бета > 0.5");
  Serial.println("Крышка откроется, но драгоценные камни НЕ появятся!\n");
}

// ============================================================================
// ФУНКЦИЯ: Запуск автоматического полного цикла
// ============================================================================
void startAutoCycle() {
  Serial.println("\n=== ЗАПУСК АВТОМАТИЧЕСКОГО ЦИКЛА ===");
  Serial.println("Будет выполнена полная симуляция: калибровка (30 сек) + тренинг (3 мин)");
  Serial.println("Наблюдайте за графиками в Serial Plotter!\n");
  
  resetSystem();
  autoCycleRunning = true;
  autoStartTime = millis();
  
  // Начинаем с калибровки
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
  autoCycleRunning = false;
  
  // Очистка буфера ЭМГ
  for (uint8_t i = 0; i < 20; i++) emgBuffer[i] = 0;
  
  Serial.println("# Система сброшена");
}

// ============================================================================
// ФУНКЦИЯ: Печать справки
// ============================================================================
void printHelp() {
  Serial.println("\n");
  Serial.println("+-----------------------------------------------------------+");
  Serial.println("|           КОМАНДЫ УПРАВЛЕНИЯ \"СУНДУК СОКРОВИЩ\"            |");
  Serial.println("+-----------------------------------------------------------+");
  Serial.println("| SUCCESS   - Симуляция УСПЕХА (расслаблен + бета-ритм)     |");
  Serial.println("| FAIL_EMG  - Провал: напряжение мышц лба                   |");
  Serial.println("| FAIL_EEG  - Провал: нет бета-ритма (рассеянное внимание)  |");
  Serial.println("| A CYCLE   - Полный автоматический цикл (3.5 мин)          |");
  Serial.println("| I         - Показать текущее состояние (Info)             |");
  Serial.println("| H         - Эта справка                                   |");
  Serial.println("| R         - Сброс системы и перезапуск калибровки         |");
  Serial.println("+-----------------------------------------------------------+");
  Serial.println("\nПодключите: Grove LED Bar (пины 2, 3) и Серво (пин 9)");
  Serial.println("Откройте Serial Plotter для просмотра графиков!\n");
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
    float ratio = (float)betaAboveCnt / validTimeCnt * 100;
    Serial.print(" (");
    Serial.print(ratio, 1);
    Serial.print("%, нужно >");
    Serial.print(BETA_SUCCESS_RATIO * 100, 0);
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
  
  // === АВТОМАТИЧЕСКИЙ ЦИКЛ (A CYCLE) ===
  if (autoCycleRunning) {
    updateAutoCycle();
  }
  
  // === СГЛАЖИВАНИЕ ЭМГ ===
  emgSmoothed = smoothEMG(emgEmulated);
  
  // === ОТПРАВКА ДАННЫХ (каждые 200 мс для графиков) ===
  if (currentTime - lastGraphUpdate >= GRAPH_INTERVAL) {
    sendToPC();
    lastGraphUpdate = currentTime;
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
