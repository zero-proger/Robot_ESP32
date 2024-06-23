#include <ServoSmooth.h> // библиотека для медленного управления серво
#include <Arduino.h>     // ядро Arduino
#include <ArduinoJson.h> // библиотека для использования типа данных json

#pragma optimize
// Функции обьявленные в коде ниже
void loop0(void *);
void loop1(void *);
void emergencySwitchPressed(void);
void updateTemp(void);
void setServo(ServoSmooth servo, u_int16_t angle);
u_int16_t httpResuest(JsonDocument _data, JsonDocument &_payload);

// Cокращения функций
#define task(z, x, c, v, q, w, e) xTaskCreatePinnedToCore(z, x, c, v, q, w, e); // создать задачу привязанную к ядру
#define _(delayMicSec) delayMicroseconds(delayMicSec)                           // ожидание в мк/c
#define _t(delayMicSec) vTaskDelay(pdMS_TO_TICKS((int)(delayMicSec / 100)))

#define _DEBUG_SERIAL // чтобы включить отладку нужно раскоментировать

#ifdef _DEBUG_SERIAL
#define DEBUG_FUNC(x) x
#define DEBUG_BEGIN(x) Serial.begin(x)
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_FUNC(x)
#define DEBUG_BEGIN(x)
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

/*                                      ╭︎─︎─︎─︎─︎─︎─︎─︎─︎─︎┬︎─︎─︎─︎─︎─︎─︎─︎─︎─︎┬︎─︎─︎─︎─︎─︎─︎─︎─︎─︎╮︎
                                        │︎         ╡︎         ╞︎         │︎
                                        │︎         ╡︎  ESP32  ╞︎         │︎
                |ADC |RTC    EN─︎I  ─︎9  ─︎┤︎ EN      ╡︎  WR♂♂M  ╞︎     D23 ╞︎═︎ GPIO23─︎EMERGENCY_SWITCH
         Sens.VP─︎A1_0─︎0 ─︎GPIO36─︎I  ─︎5  ─︎┤︎ VP      ╡︎         ╞︎     D22 ╞︎═︎ GPIO22─︎SERVO1
         Sens.VN─︎A1_3─︎3 ─︎GPIO39─︎I  ─︎8  ─︎┤︎ VN      ╡︎         ╞︎     TX0 ╞︎═︎ GPIO1─︎SERVO2
           VDET1─︎A1_6─︎4 ─︎GPIO34─︎I  ─︎10 ─︎┤︎ D34     ╡︎         ╞︎     RX0 ╞︎═︎ GPIO3─︎SERVO3
           VDET2─︎A1_7─︎5 ─︎GPIO35─︎I  ─︎11 ─︎┤︎ D35     ╡︎         ╞     D21 ╞︎═︎ GPIO21─︎SERVO4
  XTAL_32-TOUCH9─︎A1_4─︎9 ─︎GPIO32─︎I/O─︎12 ═︎╡︎ D32     └︎┬︎┬︎┬︎┬︎╥︎┬︎┬︎┬︎┬︎┘︎     D19 ╞︎═︎ GPIO19─︎ONE_WIRE_BUS
  XTAL_32-TOUCH8─︎A1_5─︎8 ─︎GPIO33─︎I/O─︎13 ═︎╡︎ D33      PWR     D2     D18 ╞︎═︎ GPIO18─︎SERVICE_BUTTON
            DAC1─︎A2_8─︎6 ─︎GPIO25─︎I/O─︎14 ═︎╡︎ D25       ¤︎      ¤︎      D5  ╞︎═︎ GPIO4─︎RELAY_PIN
            DAC2─︎A2_9─︎7 ─︎GPIO26─︎I/O─︎15 ═︎╡︎ D26                     TX2 ╞︎═︎ GPIO17─︎
          TOUCH7─︎A2_7─︎17─︎GPIO27─︎I/O─︎16 ═︎╡︎ D27                     RX2 ╞︎═︎ GPIO16─︎
HSPI_CLK ─︎TOUCH6─︎A2_6─︎16─︎GPIO14─︎I/O─︎17 ═︎╡︎ D14   tg:@Thingworx     D4  ╞︎═︎ GPIO4─︎
HSPI_MISO─︎TOUCH5─︎A2_5─︎15─︎GPIO12─︎I/O─︎18 ═︎╡︎ D12                     D2  ╞︎═︎ GPIO2─︎SERVICE_LED
HSPI_MOSI─︎TOUCH4─︎A2_4─︎14─︎GPIO13─︎I/O─︎20 ═︎╡︎ D13                     D15 ╞︎═︎ GPIO15─︎
                                   GND ─︎┤︎ GND                     GND ├︎─︎ GND
                                   VIN ─︎┤︎ VIN                     3V3 ├︎─︎ 3.3V
                                        │︎        EN         B♂♂T      │︎
                                        │︎        ◙︎   ┌︎USB┐︎   ◙︎        │︎
                                        ╰︎─︎─︎─︎─︎─︎─︎─︎─︎─︎─︎─︎─︎┷︎─︎─︎─︎┷︎─︎─︎─︎─︎─︎─︎─︎─︎─︎──︎─︎╯︎
*/

// Шина One Wire для подключения датчиков температуры
#define ONE_WIRE_BUS 19    // пин
#define INA226_ADDRES 0x40 // hex

// Пины подключения кнопок
#define SERVICE_BUTTON 18   // пин кнопки для обновления прошивки
#define EMERGENCY_SWITCH 23 // пин переключателя для экстренной остановки
#define SERVICE_LED 2       // пин светодиода
#define RELAY_PIN 5         // пин подключения реле для инструмента

// Скорость сервоприводов
#define SERVO_SPEED 10 // градусы в секунду
#define SERVO_ACCEL 0  // ускорение 0.01 - 1.0

// Пины подключения сервоприводов
#define SERVO1 17
#define SERVO2 22
#define SERVO3 16
#define SERVO4 21

// Настройка стартовых углов севроприводов
#define SERVO1_START_ANGLE 135 // градусов
#define SERVO2_START_ANGLE 90  // градусов
#define SERVO3_START_ANGLE 90  // градусов
#define SERVO4_START_ANGLE 90  // градусов

// Длинны звеньев
#define LINK_LENGTH1 440 // мм Высота базы манипулятора
#define LINK_LENGTH2 440 // мм Длинна первого звена
#define LINK_LENGTH3 440 // мм Длинна второго звена
#define LINK_LENGTH4 440 // мм Длинна инструмента

// #define OTA // включит прошивку по воздуху если раскоментировать

#ifdef OTA
// данные для точки доступа или подключения к wifi
#define WIFI_MODE softAP // softAP точка доступа или begin подключение к wifi
#define WIFI_SSID "Tenda_2.4G"
#define WIFI_PASSWORD "23021969"
// данные для ftp
#define FTP_USERNAME "admin"
#define FTP_PASSWORD "admin"
#else
#define WIFI_MODE begin // softAP точка доступа или begin подключение к wifi
#define WIFI_SSID "Tenda_2.4G"
#define WIFI_PASSWORD "23021969"
#endif

// Настройки сервера
#define TIME_BEETWEN_REQUESTS 2000                    // время между отправкой данных на сервера в мил/c
#define SERVER_IP "92.50.188.246:38254"               // IP или домен сервера с портом
#define APPKEY "91cd633b-1268-4fcb-897c-67f999cae413" // ключ для доступа без авторизации к серверу
#define THING_NAME "Robot"                            // название сущьноси на сервере
#define THING_SERVICE "InOut"                         // сервис для обмена данными
