#include <headers.h>           // фаил с дополнительными заголовками и константами
#include <Arduino.h>           // ядро Arduino
#include <ESP32Servo.h>        // библиотека для управления сервопиводами
#include <ServoSmooth.h>       // подключили либу
#include <ArduinoJson.h>       // библиотека для использования типа данных json
#include <OneWire.h>           // библиотека для управления протоколом One Wire
#include <DallasTemperature.h> // библиотека для управления датчиками температуры
#include <INA226.h>            // библиотека для работы с датчиком тока
#include <WiFi.h>              // библиотека для взаимодействия с wi-fi
#include <HTTPClient.h>        // библиотека для HTTP запросов
#include <cmath>               // библиотека для математических вычислений
#ifdef OTA
#include <ESP32FtpServer.h> // библиотека для FTP сервера
#endif
// #include <kinematic.h> // фаил с заголовками библиотеки кинематики

using namespace std;

#ifdef OTA
FtpServer FTP;
bool firmwareUpdate = false;
#endif

/*Создание обекта для инициализации http клиента*/
HTTPClient http;
/*Переменная для отсчета времени между запросами на сервер*/
u_int64_t TimeBeetwenRequest = millis();
/*Создание обектов для инициализации сервоприводов*/
ServoSmooth servo1(270), servo2(180), servo3(180), servo4(180);
ServoSmooth servo[4] = {servo1, servo2, servo3, servo4};
/*Создание обектов для инициализации шины One Wire*/
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

/*Пины подключения сервоприводов задаются в headers.h*/
const u_int8_t SERVO_PINS[4] = {SERVO1,
                                SERVO2,
                                SERVO3,
                                SERVO4};
u_int16_t servo_angle[4]; // переменна храненящая углы сервоприводов
u_int8_t tempSensors;     // переменная хранящая количество датчиков температуры
u_int8_t tempC[4];        // переменная хранящая значения с датчиков температуры

/*Создание объектов для управления задачами*/
TaskHandle_t Task0;
TaskHandle_t Task1;

INA226 INA(INA226_ADDRES);
// Объявляем и инициализируем JSON-документы
JsonDocument request;
JsonDocument payload;
JsonDocument buffer;

bool goToCords = false;

void setServo(ServoSmooth servo, u_int16_t angle)
{
  servo.setTargetDeg(angle);
  while (!servo.tickManual())
  {
    _(5000);
  }
}

u_int16_t httpResuest(JsonDocument _data, JsonDocument &_payload)
{
  u_int16_t httpResponseCode;
  String jsonString;
  /*выделение памяти для ссылки*/
  u_int16_t URL_MAX_LENGTH = 36 + strlen(SERVER_IP) + strlen(THING_NAME) + strlen(THING_SERVICE);
  char URL[URL_MAX_LENGTH] = "";
  /*формирование ссылки для api*/
  snprintf(URL, URL_MAX_LENGTH, "http://%s/Thingworx/Things/%s/Services/%s", SERVER_IP, THING_NAME, THING_SERVICE);
  /*запуск клиента и добавление заголовков*/
  http.begin(URL);
  http.addHeader(F("Content-Type"), F("application/json"));
  http.addHeader(F("Accept"), F("application/json"));
  http.addHeader(F("appkey"), APPKEY);
  http.addHeader(F("User-Agent"), F("Robot_ESP32/1.0"));
  /*отравка на сервер post запроса */
  serializeJson(_data, jsonString);
  httpResponseCode = http.POST(jsonString);
  DEBUG_PRINT(F("http.response.code."));
  DEBUG_PRINTLN(httpResponseCode);
  DEBUG_PRINT(F("payload.json."));
  String result = http.getString();
  deserializeJson(_payload, result);
  serializeJson(_payload, Serial);
  DEBUG_PRINTLN("");
  http.end();
  goToCords = true;
  return httpResponseCode;
}

void setup(void)
{
  DEBUG_BEGIN(9600);
  DEBUG_PRINTLN(" ");
  DEBUG_PRINTLN(F("i2c.started"));
  Wire.begin();

  DEBUG_PRINTLN(F("connecting.wifi"));
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  for (u_int8_t i = 0; i < 10; i++)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      break;
    }
    DEBUG_PRINT('.');
    _(100000);
  }
  if (WiFi.status() != WL_CONNECTED)
  {
    DEBUG_PRINTLN(F("wifi.not.connected"));
    DEBUG_PRINTLN(F("access.point.start"));
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
    _(100);
  }
#ifdef OTA
  DEBUG_PRINTLN(F("begin.ftp.server"));
  FTP.begin(FTP_USERNAME, FTP_PASSWORD);
  pinMode(SERVICE_BUTTON, INPUT);
  if (digitalRead(SERVICE_BUTTON))
  {
    digitalWrite(SERVICE_LED, HIGH);
    firmwareUpdate = true;
  }
#endif
  pinMode(EMERGENCY_SWITCH, INPUT);
  attachInterrupt(digitalPinToInterrupt(EMERGENCY_SWITCH),
                  emergencySwitchPressed,
                  HIGH); // Подключение обработчика прерывания

  DEBUG_PRINTLN(F("init.core."));
  DEBUG_PRINT(xPortGetCoreID());

  DEBUG_PRINTLN(F("one.wire.init"));
  sensors.begin(); // Запуск шины one wire

  DEBUG_PRINTLN(F("locating.devices"));
  DEBUG_PRINT(F("found."));
  tempSensors = sensors.getDeviceCount(); // найти устройства на шине
#ifdef _DEBUG_SERIAL
  Serial.print(tempSensors, DEC);
#endif
  DEBUG_PRINTLN(F(".devices"));
  DEBUG_PRINTLN("");

  /*стартовые углы сервоприводов задаются в файле headers.h*/
  u_int16_t SERVO_ANGLES[4] = {SERVO1_START_ANGLE,
                               SERVO2_START_ANGLE,
                               SERVO3_START_ANGLE,
                               SERVO4_START_ANGLE};

  for (u_int8_t i; i <= 3; i++)
  { /*конфигурирование пинов для сервоприводов*/
    servo[i].attach(SERVO_PINS[i], SERVO_ANGLES[i]);
    DEBUG_PRINT(F("servo."));
    DEBUG_PRINT(i);
    DEBUG_PRINT(F(".attach.pin."));
    DEBUG_PRINTLN(SERVO_PINS[i]);
    servo[i].setSpeed(SERVO_SPEED);
    servo[i].setAccel(SERVO_ACCEL);
    /*установка стартовых углов сервопривода*/
    servo[i].smoothStart();
    DEBUG_PRINT(F("servo."));
    DEBUG_PRINT(i);
    DEBUG_PRINT(F(".write."));
    DEBUG_PRINTLN(SERVO_ANGLES[i]);
  }

  task( // создание функции с бесконечным циклом в ядре 0
      loop0,
      "loop0",
      10000,
      NULL,
      1,
      &Task0,
      0);

  task( // создание функции с бесконечным циклом в ядре 1
      loop1,
      "loop1",
      10000,
      NULL,
      1,
      &Task1,
      1);
}

void loop0(void *)
{
  DEBUG_PRINT(F("start.infinity.loop."));
  DEBUG_PRINTLN(xPortGetCoreID());
  /*init*/

  /*end init*/
  for (;;)
  { // бесконечный цикл в ядре 0 с задержкой 100 мк/c
#ifdef OTA
    if (firmwareUpdate)
    {
      FTP.handleFTP();
    }
#endif
    updateTemp();
    _t(100);
  }
}

void loop1(void *)
{
  /*init*/
  DEBUG_FUNC(_t(200));
  DEBUG_PRINT(F("start.infinity.loop."));
  DEBUG_PRINTLN(xPortGetCoreID());
  u_int8_t httpResponseCode;
  request["n"] = 1;
  /*end init*/
  for (;;)
  { // бесконечный цикл в ядре 1 с задержкой 100 мк/c

    if (millis() - TimeBeetwenRequest >= TIME_BEETWEN_REQUESTS)
    {
      TimeBeetwenRequest += TIME_BEETWEN_REQUESTS;
      httpResponseCode = httpResuest(request, payload);
    }
    if (httpResponseCode == 200)
    {
      if (request["n"] < payload["N"] && buffer != payload && goToCords)
      {
        buffer = payload;
        for (u_int8_t i = 0; i <= 3; i++)
        { /*конфигурирование пинов для сервоприводов*/
          setServo(servo[i], (u_int16_t)(payload["A" + to_string(i)]));
          /*установка углов сервопривода*/
          DEBUG_PRINT("servo.");
          DEBUG_PRINT(i);
          DEBUG_PRINT('.');
          DEBUG_PRINT((u_int16_t)payload["A" + to_string(i)]);
          DEBUG_PRINTLN(".deg");
        } 
        DEBUG_PRINTLN(F("complete.servo"));
        request["n"] = payload["N"];
        goToCords = false;
      }
    }
    _t(100);
  }
}

void updateTemp(void)
/*функция для получения температуры с датчиков*/
{
  sensors.requestTemperatures();
  DEBUG_PRINTLN(F("sensors.request.temperatures"));
  for (int i = 0; i < tempSensors; i++)
  {
    request["t" + to_string(i)] = sensors.getTempCByIndex(i);
    DEBUG_PRINT(".");
    _t(100);
  }
  DEBUG_PRINTLN(F("complete.request.temperatures"));
}

void IRAM_ATTR emergencySwitchPressed(void)
/*функция для экстренной остановки кода*/
{
  DEBUG_PRINT(F("emergency.switch.pressed"));
  bool flag = true;
  for (;;)
  {
    if (digitalRead(EMERGENCY_SWITCH))
    {
      if (flag)
      {
        vTaskSuspend(Task0);
        _(100);
        vTaskSuspend(Task1);
        _(100);
        flag = !flag;
      }
      _(10000);
    }
    else
    {
      vTaskResume(Task0);
      vTaskResume(Task1);
      _(10000);
      break;
    }
  }
}
// не используется
void loop(void) {}

/*
DEBUG ON AND OTA ON
RAM:   [==        ]  17.2% (used 56372 bytes from 327680 bytes)
Flash: [========  ]  76.7% (used 1004945 bytes from 1310720 bytes)

DEBUG OFF AND OTA ON
RAM:   [==        ]  17.2% (used 56364 bytes from 327680 bytes)
Flash: [========  ]  75.5% (used 989329 bytes from 1310720 bytes)

DEBUG ON AND OTA OFF
RAM:   [=         ]  14.4% (used 47184 bytes from 327680 bytes)
Flash: [=======   ]  71.6% (used 938113 bytes from 1310720 bytes)

DEBUG OFF AND OTA OFF
RAM:   [=         ]  14.3% (used 46948 bytes from 327680 bytes)
Flash: [=======   ]  69.9% (used 916157 bytes from 1310720 bytes)
*/