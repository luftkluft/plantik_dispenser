// Plantik_pavt низкоскоростной алгоритм
char VERSION[] = "1.0.0";

// Подавление дребезга отключено.
const long RATTLING_INTERVAL   = 0;
const int RATTLING_SENSITIVITY = 0;

const long TERMINAL_INTERVAL = 2000;
const int MAX_TIMER          = 10; // время в секундах
const int START_DELAY        = 1000;

byte inputPortCByte  = B00000000;
byte outputPortBByte = B00000000;
byte outputPortDByte = B00000000;
byte modeByte = B00000000;
// Режимы:
// B00001000 - стартовый
// B00000001 - основной рабочий
// B00000010 - рабочий без таймера
// B00000100 - стоп

unsigned long previousMillis           = 0;
unsigned long rattlingCount            = 0;
unsigned long previousTimerMillis      = 0;
unsigned long previousDelayTimerMillis = 0;

unsigned int timerValue  = 0;
bool isTimerOn           = false;
bool isTimerUp           = false;
bool isStartDelayTimerOn = false;
bool isStartDelayTimerUp = false;

void setup() {
  pinMode(A0, INPUT);    // Время вибрации
  // pinMode(A1, INPUT); // Резерв
  pinMode(A2, INPUT);    // Кнопка Пуск
  pinMode(A3, INPUT);    // Кнопка Стоп
  pinMode(A4, INPUT);    // Датчик открытия замка
  // pinMode(A5, INPUT); // Резерв
  pinMode(13, OUTPUT);   // LED Пуск
  pinMode(12, OUTPUT);   // LED Квитирование
  pinMode(11, OUTPUT);   // LED Стоп
  pinMode(10, OUTPUT);   // LED Замок открыт
  pinMode(9, OUTPUT);    // LED Без таймера
  pinMode(7, OUTPUT);    // Мотор вибратора
  // pinMode(6, OUTPUT); // Резерв
  Serial.begin(9600);
  modeByte = B00001000;
  Serial.print("plantik_pavt v");
  Serial.println(VERSION);
  Serial.print("Initialization OK :)");
}

void loop() {
  unsigned long currentMillis = millis();
  if (bitRead(outputPortDByte, 7)) { // индикация команды на пуск двигателя
    outputPortBByte = B00010000;     // включить индикатор Квитирования
  } else {
    outputPortBByte &= ~(1 << 4);    // выключить индикатор Квитирование
  }

  if (bitRead(inputPortCByte, 4)) {  // если замок открыт
    resetTimer();
    resetStartDelayTimer();
    if (!isStartDelayTimerUp) {
      if (!isStartDelayTimerOn) previousDelayTimerMillis = currentMillis;
      isStartDelayTimerOn = true;    // запускаем таймер задержки
    }
  } else {
    if (!isStartDelayTimerUp) {
      if (!isStartDelayTimerOn) previousDelayTimerMillis = currentMillis;
      isStartDelayTimerOn = true;    // запускаем таймер задержки
    }

    if (!isTimerUp) {
      if (!isTimerOn) previousTimerMillis = currentMillis;
      isTimerOn = true;
    }
  }

  if (isTimerOn) {
    if (currentMillis - previousTimerMillis >= timerValue * 1000) {
      isTimerUp = true;
      isTimerOn = false;
    }
  }

  if (isStartDelayTimerOn) {
    if (currentMillis - previousDelayTimerMillis >= START_DELAY) {
      isStartDelayTimerUp = true;
      isStartDelayTimerOn = false;
    }
  }

  if (currentMillis - previousMillis >= RATTLING_INTERVAL) {
    rattelingFilter();
  }

  if (currentMillis - previousMillis >= TERMINAL_INTERVAL) {
    previousMillis = currentMillis;
    Serial.print("modeByte= ");
    Serial.println(modeByte, BIN);
    Serial.print("inputPortCByte= ");
    Serial.println(inputPortCByte, BIN);
    Serial.print("outputPortBByte= ");
    Serial.println(outputPortBByte, BIN);
    Serial.print("outputPortDByte= ");
    Serial.println(outputPortDByte, BIN);
    Serial.print("Timer= ");
    Serial.println(timerValue);
  }
  setTimer();
  checkStop();
  mainCycle();
  sensorsStatus ();
  writeOutputs();
}

void mainCycle() {
  if (bitRead(modeByte, 3)) {         // первое включение
    if (bitRead(inputPortCByte, 3)) { // если нажата кнопка Квитирование (Стоп)
      modeByte = B00000100;
    }
    outputPortBByte = B00010000;      // включить индикатор Ожидание квитирования
    outputPortDByte = B00000000;      // заблокировать мотор
  }

  if (bitRead(modeByte, 2)) {         // если режим Стоп
    outputPortDByte = B00000000;      // выключить мотор
    outputPortBByte &= ~(1 << 5);     // выключить индикатор Работа
    outputPortBByte &= ~(1 << 4);     // выключить индикатор Квитирование
    outputPortBByte |= (1 << 3);      // включить индикатор Стоп
    if (bitRead(inputPortCByte, 2)) { // если нажата кнопка Пуск
      modeByte = B00000001;
      resetTimer();
      resetStartDelayTimer();
    }
  }

  if (bitRead(modeByte, 0)) {         // если режим Работа
    setWorkMode();
    outputPortBByte &= ~(1 << 1);     // выключить индикатор NO_TIMER
    outputPortBByte |= (1 << 5);      // включить индикатор Работа
    outputPortBByte &= ~(1 << 3);     // выключить индикатор Стоп
    if (isTimerOn && !isStartDelayTimerOn) {
      outputPortDByte = B10000000;    // включить мотор
    } else {
      outputPortDByte = B00000000;    // выключить мотор
    }
  }

  if (bitRead(modeByte, 1)) {         // если режим Без Таймера
    setWorkMode();
    outputPortBByte |= (1 << 5);      // включить индикатор Работа
    outputPortBByte &= ~(1 << 3);     // выключить индикатор Стоп
    outputPortBByte |= (1 << 1);      // включить индикатор NO_TIMER
    if (bitRead(inputPortCByte, 4)) {
      outputPortDByte = B00000000;    // выключить мотор
    } else {
      if (!isStartDelayTimerOn) {
        outputPortDByte = B10000000;  // включить мотор
      }
    }
  }

}

void setWorkMode() {
  if (timerValue >= MAX_TIMER - 1) {
    modeByte = B00000010;             // переходим в режим без таймера
  } else {
    modeByte = B00000001;             // возвращаемся в режим работы с таймером
  }
}

void rattelingFilter() {
  byte portC = readPortC();
  if (inputPortCByte != portC) {
    rattlingCount = rattlingCount + 1;
    if (rattlingCount > RATTLING_SENSITIVITY) {
      inputPortCByte = portC;
      rattlingCount = 0;
    }
  }
  else {
    rattlingCount = 0;
  }
}

byte readPortC() {
  byte inputByte = B00000000;
  if (analogRead(A0) > 500) inputByte |= (1 << 0); // Задатчик таймера
  // if (digitalRead(A1)) inputByte |= (1 << 1);   // Резерв
  if (digitalRead(A2)) inputByte |= (1 << 2);      // Кнопка Пуск
  if (digitalRead(A3)) inputByte |= (1 << 3);      // Кнопка Стоп
  // if (digitalRead(A4)) inputByte |= (1 << 4);   // Датчик открытия замка
  if (!digitalRead(A4)) inputByte |= (1 << 4);     // Датчик открытия замка // инвертирован вход под npn датчик
  // if (digitalRead(A5)) inputByte |= (1 << 5);   // Резерв
  return inputByte;
}

void setTimer () {
  // время в секундах
  timerValue = analogRead(A0) * (MAX_TIMER / 1023.0);
}

void checkStop() {
  if (bitRead(inputPortCByte, 3)) modeByte = B00000100; // переход в режим Стоп
}

void writeOutputs() {
  digitalWrite(13, bitRead(outputPortBByte, 5));
  digitalWrite(12, bitRead(outputPortBByte, 4));
  digitalWrite(11, bitRead(outputPortBByte, 3));
  digitalWrite(10, bitRead(outputPortBByte, 2));
  digitalWrite(9, bitRead(outputPortBByte, 1));
  // digitalWrite(8, bitRead(outputPortBByte, 0));
  digitalWrite(7, bitRead(outputPortDByte, 7));
  // digitalWrite(6, bitRead(outputPortDByte, 6));
  // digitalWrite(5, bitRead(outputPortDByte, 5));
  // digitalWrite(4, bitRead(outputPortDByte, 4));
  // digitalWrite(3, bitRead(outputPortDByte, 3));
  // digitalWrite(2, bitRead(outputPortDByte, 2));
  // digitalWrite(1, bitRead(outputPortDByte, 1));
  // digitalWrite(0, bitRead(outputPortDByte, 0));
}

void resetTimer() {
  isTimerOn = false; // отключаем таймер
  isTimerUp = false; // сбрасываем флаг завершения отсчёта
}

void resetStartDelayTimer() {
  isStartDelayTimerOn = false; // отключаем таймер
  isStartDelayTimerUp = false; // сбрасываем флаг завершения отсчёта
}

void sensorsStatus () {
  if (bitRead(inputPortCByte, 4)) {
    outputPortBByte |= (1 << 2);
  } else {
    outputPortBByte &= ~(1 << 2);
  }
}
