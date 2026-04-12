/*
 * ============================================================================
 * ПРОЕКТ: "СУНДУК СОКРОВИЩ" - Тренинг биоуправления для коррекции внимания (СДВГ)
 * ============================================================================
 * 
 * ЗАДАЧА:
 * Часть 1: Управление крышкой по ЭМГ (мышцы лба)
 *   - Удерживать ЭМГ ниже порога 3 минуты → крышка открывается (0° → 90°)
 *   - Если ЭМГ выше порога → крышка закрывается в 2 раза быстрее
 *   - Сглаживание ЭМГ скользящим средним (500 мс)
 * 
 * Часть 2: Детекция бета-ритма ЭЭГ (13-18 Гц)
 *   - Если бета-ритм > порога ≥60% времени → зажечь LEDs ("драгоценные камни")
 *   - Периоды с высоким ЭМГ не засчитываются
 * 
 * КАЛИБРОВКА (30 сек):
 *   - Порог ЭМГ: среднее значение в покое × коэффициент 1.3
 *   - Порог ЭЭГ: максимум бета-ритма с открытыми глазами × коэффициент 0.7
 * 
 * ОБОРУДОВАНИЕ:
 *   - Arduino Mega
 *   - Grove LED Bar v1.0 (пины 2, 3) - MY9221
 *   - Сервопривод FS5103B (пин 9)
 *   - Bitronics Lab EEG (A0)
 *   - Bitronics Lab EMG/ECG (A1)
 * 
 * ВЕРСИЯ: 3.0 - Исправлена проблема с дёрганьем серво
 * ============================================================================
 */

#include <Grove_LED_Bar.h>
#include <Servo.h>
#include <arduinoFFT.h>

// ============================================================================
// НАСТРОЙКИ ПИНОВ
// ============================================================================
const int PIN_LED_DATA     = 2;   // Grove LED Bar DATA
const int PIN_LED_CLK      = 3;   // Grove LED Bar CLOCK
const int PIN_SERVO        = 9;   // Сервопривод PWM
const int PIN_EEG          = A0;  // Датчик ЭЭГ Bitronics
const int PIN_EMG          = A1;  // Датчик ЭМГ Bitronics

// ============================================================================
// ТАЙМИНГИ (в миллисекундах)
// ============================================================================
const unsigned long TIME_CALIBRATION    = 30000;  // Калибровка: 30 сек
const unsigned long TIME_TRAINING       = 180000; // Тренинг: 3 минуты
const unsigned long SERVO_INTERVAL      = 200;    // Обновление серво: 200 мс (не чаще!)
const unsigned long SERIAL_INTERVAL     = 30;     // Отправка данных: 30 мс

// ============================================================================
// ПАРАМЕТРЫ БИОУПРАВЛЕНИЯ
// ============================================================================
const float EMG_THRESHOLD_K      = 1.3;   // Коэффициент порога ЭМГ (1.2 = легче)
const float BETA_THRESHOLD_K     = 0.7;   // Коэффициент порога бета-ритма
const float BETA_SUCCESS_RATIO   = 0.6;   // Требуемый % времени с бета-ритмом

// Скорости серво (градусов за обновление 200мс)
const float LID_SPEED_OPEN       = 0.15;  // Медленное открытие
const float LID_SPEED_CLOSE      = 0.30;  // Быстрое закрытие (×2)

// Углы серво
const int SERVO_CLOSED           = 0;     // Закрыто
const int SERVO_OPEN             = 90;    // Открыто
const int LID_SUCCESS_ANGLE      = 54;    // 60% от 90° = 54° (критерий успеха)

// Диапазон бета-ритма (Гц)
const float BETA_FREQ_LOW        = 13.0;  // Для подростков (13-18 Гц)
const float BETA_FREQ_HIGH       = 18.0;
// const float BETA_FREQ_LOW       = 12.0; // Для детей (12-15 Гц)
// const float BETA_FREQ_HIGH      = 15.0;

// ============================================================================
// FFT НАСТРОЙКИ
// ============================================================================
#define FFT_SIZE 64
const float SAMPLING_FREQ = 128.0;  // Частота дискретизации (Гц)

// ============================================================================
// ГЛОБАЛЬНЫЕ ОБЪЕКТЫ
// ============================================================================
Grove_LED_Bar bar(PIN_LED_DATA, PIN_LED_CLK, 1);  // 1 = обратный порядок
Servo servo;
arduinoFFT FFT = arduinoFFT();

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

// ============================================================================
// ПЕРЕМЕННЫЕ ЭЭГ
// ============================================================================
double eegReal[FFT_SIZE];    // Действительная часть для БПФ
double eegImag[FFT_SIZE];    // Мнимая часть для БПФ
uint8_t eegIdx = 0;

// Статистика бета-ритма
uint16_t betaAboveCnt = 0;   // Счётчик периодов с высоким бета-ритмом
uint16_t validTimeCnt = 0;   // Счётчик валидных периодов (когда расслаблен)

// ============================================================================
// ПЕРЕМЕННЫЕ СЕРВО
// ============================================================================
float lidAngle = 0;
unsigned long lastServoUpdate = 0;

// ============================================================================
// ПЕРЕМЕННЫЕ ДЛЯ ОТПРАВКИ ДАННЫХ
// ============================================================================
unsigned long lastSerialSend = 0;
int rawEMG = 0;
int rawEEG = 0;
float betaPower = 0;

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
  for (uint8_t i = 0; i < FFT_SIZE; i++) {
    eegReal[i] = 0;
    eegImag[i] = 0;
  }
}

// ============================================================================
// ФУНКЦИЯ: Чтение сенсора (усреднение 5 отсчётов)
// ============================================================================
int readSensor(int pin) {
  int sum = 0;
  for (uint8_t i = 0; i < 5; i++) {
    sum += analogRead(pin);
    delayMicroseconds(100);
  }
  return sum / 5;
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
// ФУНКЦИЯ: Расчёт мощности бета-ритма (БПФ)
// ============================================================================
float calcBetaPower() {
  // Применяем окно Хэмминга
  for (uint8_t i = 0; i < FFT_SIZE; i++) {
    double window = 0.54 - 0.46 * cos(2 * PI * i / (FFT_SIZE - 1));
    eegReal[i] *= window;
    eegImag[i] = 0;
  }
  
  // Прямое БПФ
  FFT.Windowing(eegReal, FFT_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(eegReal, eegImag, FFT_SIZE, FFT_FORWARD);
  FFT.ComplexToMagnitude(eegReal, eegImag, FFT_SIZE);
  
  // Находим индексы частот для бета-диапазона
  uint8_t betaLowIdx = (uint8_t)(BETA_FREQ_LOW * FFT_SIZE / SAMPLING_FREQ);
  uint8_t betaHighIdx = (uint8_t)(BETA_FREQ_HIGH * FFT_SIZE / SAMPLING_FREQ);
  
  // Суммируем мощность в бета-диапазоне
  float betaSum = 0;
  float totalSum = 0;
  
  for (uint8_t i = 1; i < FFT_SIZE / 2; i++) {
    totalSum += eegReal[i];
    
    if (i >= betaLowIdx && i <= betaHighIdx) {
      betaSum += eegReal[i];
    }
  }
  
  // Нормализованная мощность бета-ритма
  return (totalSum > 0) ? (betaSum / totalSum) : 0;
}

// ============================================================================
// ФУНКЦИЯ: Обновление положения крышки (вызывается каждые 200 мс)
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
// ФУНКЦИЯ: Отправка данных на компьютер (для Bitronics Lab)
// ============================================================================
void sendToPC() {
  // Формат: RAW_EEG, RAW_EMG, BETA_POWER, LID_ANGLE, THRESHOLD_FLAG
  Serial.print(rawEEG);
  Serial.print('\t');
  Serial.print(rawEMG);
  Serial.print('\t');
  Serial.print(betaPower, 4);
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
    return;
  }
  
  // === РАСЧЁТ ПОРОГА ЭМГ ===
  float smoothed = smoothEMG(rawEMG);
  static float emgSum = 0;
  static uint16_t emgCnt = 0;
  
  emgSum += smoothed;
  emgCnt++;
  
  // Вычисляем порог ЭМГ на середине калибровки
  if (elapsed >= TIME_CALIBRATION / 2 && emgCnt >= 20) {
    emgThreshold = (emgSum / emgCnt) * EMG_THRESHOLD_K;
  }
  
  // === РАСЧЁТ ПОРОГА БЕТА-РИТМА ===
  eegReal[eegIdx] = rawEEG;
  eegIdx = (eegIdx + 1) % FFT_SIZE;
  
  if (eegIdx == 0) {
    float beta = calcBetaPower();
    static float betaMax = 0;
    
    // Запоминаем максимум бета-ритма
    if (beta > betaMax) betaMax = beta;
    betaThreshold = betaMax * BETA_THRESHOLD_K;
  }
  
  // === ВИЗУАЛИЗАЦИЯ ПРОГРЕССА КАЛИБРОВКИ ===
  int level = map(elapsed, 0, TIME_CALIBRATION, 0, 10);
  bar.setLevel(level);
}

// ============================================================================
// СОСТОЯНИЕ: ТРЕНИНГ (3 минуты)
// ============================================================================
void runTraining() {
  unsigned long elapsed = millis() - phaseStart;
  
  // Завершение тренинга
  if (elapsed >= TIME_TRAINING) {
    currentState = FINISH;
    return;
  }
  
  // === ОБРАБОТКА ЭЭГ (каждые 64 отсчёта) ===
  eegReal[eegIdx] = rawEEG;
  eegIdx = (eegIdx + 1) % FFT_SIZE;
  
  if (eegIdx == 0) {
    betaPower = calcBetaPower();
    validTimeCnt++;
    
    // Считаем бета-ритм ТОЛЬКО когда расслаблен
    if (isRelaxed && lidAngle > 10) {
      if (betaPower > betaThreshold) {
        betaAboveCnt++;
      }
    }
  }
  
  // === ОТПРАВКА ДАННЫХ ДЛЯ ВИЗУАЛИЗАЦИИ ===
  Serial.print("#THRESHOLDS\t");
  Serial.print(emgThreshold, 1);
  Serial.print('\t');
  Serial.println(betaThreshold, 4);
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
  // Критерий 1: Крышка открыта ≥60% (≥54 градуса)
  bool lidSuccess = (lidAngle >= LID_SUCCESS_ANGLE);
  
  // Критерий 2: Бета-ритм > порога ≥60% времени
  bool betaSuccess = false;
  if (validTimeCnt > 0) {
    float ratio = (float)betaAboveCnt / validTimeCnt;
    betaSuccess = (ratio >= BETA_SUCCESS_RATIO);
    
    // Отладочная информация
    Serial.print("#RESULT\t");
    Serial.print(ratio, 3);
    Serial.print('\t');
    Serial.println(betaSuccess ? "PASS" : "FAIL");
  }
  
  // === ВИЗУАЛИЗАЦИЯ РЕЗУЛЬТАТА ===
  if (lidSuccess && betaSuccess) {
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
    // ✗ НЕУДАЧА: мигаем красными
    for (uint8_t i = 0; i < 3; i++) {
      bar.setBits(0b1111111000);  // Только красные
      delay(300);
      bar.setBits(0);
      delay(300);
    }
  }
  
  // Зависаем в этом состоянии
  while (true) {
    delay(1000);
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(112500);  // Скорость для Bitronics Lab
  
  initHardware();
  
  phaseStart = millis();
  
  // Сигнал начала работы
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
  
  // === ЧТЕНИЕ ДАТЧИКОВ (каждый цикл) ===
  rawEMG = readSensor(PIN_EMG);
  rawEEG = readSensor(PIN_EEG);
  
  // === СГЛАЖИВАНИЕ ЭМГ ===
  emgSmoothed = smoothEMG(rawEMG);
  
  // === ОТПРАВКА ДАННЫХ НА ПК (каждые 30 мс) ===
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
  
  // === НЕБОЛЬШАЯ ПАУЗА ДЛЯ СТАБИЛЬНОСТИ АЦП ===
  delay(5);
}
