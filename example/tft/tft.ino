#include "Arduino.h"
#include "TFT_eSPI.h" /* Please use the TFT library provided in the library. */
#include "img_logo.h"
#include "pin_config.h"
#include "s8_uart.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <PMserial.h>
#include "time.h"

#define NTP_SERVER  "pool.ntp.org"
#define GMT_OFFSET_SEC  (-7*3600)
#define DAYLIGHT_OFFSET_SEC  0
#define MAIN_LOOP_WAIT 10000

#define WIFI_SSID "Jonesmo"
#define WIFI_PASSWORD "Rossydog11"
#define MQTT_HOST "192.168.68.132"
#define MQTT_PORT 1883

#define MQTT_TOPIC_RECV "sensor/co2_1/received" // MQTT topic to publish received IR codes
#define MQTT_TOPIC_SEND "sensor/co2_1/send" // MQTT topic to subscribe for sending IR codes
void callback(char* topic, byte* payload, unsigned int length) {}

bool time_ok = false;
WiFiClient espClient;
PubSubClient client(MQTT_HOST, MQTT_PORT, callback, espClient);

long lastReconnectAttempt = 0;
void setup_wifi(int timeout = 10000) {
  unsigned long startAttemptTime = millis();
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < timeout) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Failed to connect to WiFi within timeout period");
    return;
  }

  randomSeed(micros());
  struct tm timeinfo;
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
  }
  else  time_ok = true;
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect(int timeout = 10000) {
  // Call the setup_wifi function
  setup_wifi(5000);
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Failed to connect to WiFi, exiting...");
    return;
  }

  unsigned long startAttemptTime = millis();
  // Loop until we're reconnected
  while (!client.connected() && millis() - startAttemptTime < timeout) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "co2_2";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), "homeassistant", "Ohlighu3zeerieyo8Ual6uephu2quoh2saache0aet3reexee3oogh5iehuPhoh4")) {
      Serial.println("connected");
      client.subscribe(MQTT_TOPIC_RECV);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
  
  if (!client.connected()) {
    Serial.println("Failed to connect to MQTT within timeout period, exiting...");
  }
}
#define MIN_BRIGHT 1
#define DEFAULT_BRIGHTNESS 50
#define MAX_BRIGHT 255

#define SUNRISE_TIME 5.0    // Sunrise at 5 AM
#define SOLAR_NOON_TIME 13.0 // Solar noon at 1 PM
#define SUNSET_TIME 20.0    // Sunset at 8 PM


int getBrightness() {
  if (!time_ok)
    return DEFAULT_BRIGHTNESS;
  time_t rawtime;
  time(&rawtime);
  struct tm * timeinfo = localtime(&rawtime);
  float currentHour = timeinfo->tm_hour + timeinfo->tm_min / 60.0 + timeinfo->tm_sec / 3600.0;
  
  int brightness = 0;
  
  // Morning increase in brightness
  if (currentHour >= SUNRISE_TIME && currentHour < SOLAR_NOON_TIME) {
    float progress = (currentHour - SUNRISE_TIME) / (SOLAR_NOON_TIME - SUNRISE_TIME);
    brightness = MIN_BRIGHT + (MAX_BRIGHT - MIN_BRIGHT) * progress;
  }
  // Afternoon decrease in brightness
  else if (currentHour >= SOLAR_NOON_TIME && currentHour < SUNSET_TIME) {
    float progress = 1 - ((currentHour - SOLAR_NOON_TIME) / (SUNSET_TIME - SOLAR_NOON_TIME));
    brightness = MIN_BRIGHT + (MAX_BRIGHT - MIN_BRIGHT) * progress;
  }
  // Night time
  else {
    brightness = MIN_BRIGHT;
  }

  return brightness;
}


/* BEGIN CONFIGURATION */
#define DEBUG_BAUDRATE 115200

#define USE_SOFTWARE_SERIAL
#if (defined USE_SOFTWARE_SERIAL || defined ARDUINO_ARCH_RP2040)
  #define S8_RX_PIN 44         // Rx pin which the S8 Tx pin is attached to (change if it is needed)
  #define S8_TX_PIN 43         // Tx pin which the S8 Rx pin is attached to (change if it is needed)
#else
  #define S8_UART_PORT  0     // Change UART port if it is needed
#endif
/* END CONFIGURATION */


#ifdef USE_SOFTWARE_SERIAL
  SerialPM pms(PMSx003, 17,18); // PMSx003, UART
  HardwareSerial S8_serial(0);
#else
  #if defined(ARDUINO_ARCH_RP2040)
    REDIRECT_STDOUT_TO(Serial)    // to use printf (Serial.printf not supported)
    UART S8_serial(S8_TX_PIN, S8_RX_PIN, NC, NC);
  #else
    HardwareSerial S8_serial(S8_UART_PORT);   
  #endif
#endif


S8_UART *sensor_S8;
S8_sensor sensor;
/* The product now has two screens, and the initialization code needs a small change in the new version. The LCD_MODULE_CMD_1 is used to define the
 * switch macro. */
#define LCD_MODULE_CMD_1

TFT_eSPI tft = TFT_eSPI();

#if defined(LCD_MODULE_CMD_1)
typedef struct {
    uint8_t cmd;
    uint8_t data[14];
    uint8_t len;
} lcd_cmd_t;

lcd_cmd_t lcd_st7789v[] = {
    {0x11, {0}, 0 | 0x80},
    {0x3A, {0X05}, 1},
    {0xB2, {0X0B, 0X0B, 0X00, 0X33, 0X33}, 5},
    {0xB7, {0X75}, 1},
    {0xBB, {0X28}, 1},
    {0xC0, {0X2C}, 1},
    {0xC2, {0X01}, 1},
    {0xC3, {0X1F}, 1},
    {0xC6, {0X13}, 1},
    {0xD0, {0XA7}, 1},
    {0xD0, {0XA4, 0XA1}, 2},
    {0xD6, {0XA1}, 1},
    {0xE0, {0XF0, 0X05, 0X0A, 0X06, 0X06, 0X03, 0X2B, 0X32, 0X43, 0X36, 0X11, 0X10, 0X2B, 0X32}, 14},
    {0xE1, {0XF0, 0X08, 0X0C, 0X0B, 0X09, 0X24, 0X2B, 0X22, 0X43, 0X38, 0X15, 0X16, 0X2F, 0X37}, 14},
};
#endif

void printPMS()
{
#if defined(ESP8266) || defined(ESP8266)
  if (!pms.has_particulate_matter())
    return;
  Serial.printf("PM1.0 %2d, PM2.5 %2d, PM10 %2d [ug/m3]\n",
                pms.pm01, pms.pm25, pms.pm10);

  if (!pms.has_number_concentration())
    return;
  Serial.printf("N0.3 %4d, N0.5 %3d, N1.0 %2d, N2.5 %2d, N5.0 %2d, N10 %2d [#/100cc]\n",
                pms.n0p3, pms.n0p5, pms.n1p0, pms.n2p5, pms.n5p0, pms.n10p0);

  if (!pms.has_temperature_humidity() && !pms.has_formaldehyde())
    return;
  Serial.printf("%5.1f °C, %5.1f %%rh, %5.2f mg/m3 HCHO\n",
                pms.temp, pms.rhum, pms.hcho);
#else
  if (!pms.has_particulate_matter())
    return;
  Serial.print(F("PM1.0 "));
  Serial.print(pms.pm01);
  Serial.print(F(", "));
  Serial.print(F("PM2.5 "));
  Serial.print(pms.pm25);
  Serial.print(F(", "));
  Serial.print(F("PM10 "));
  Serial.print(pms.pm10);
  Serial.println(F(" [ug/m3]"));

  if (!pms.has_number_concentration())
    return;
  Serial.print(F("N0.3 "));
  Serial.print(pms.n0p3);
  Serial.print(F(", "));
  Serial.print(F("N0.5 "));
  Serial.print(pms.n0p5);
  Serial.print(F(", "));
  Serial.print(F("N1.0 "));
  Serial.print(pms.n1p0);
  Serial.print(F(", "));
  Serial.print(F("N2.5 "));
  Serial.print(pms.n2p5);
  Serial.print(F(", "));
  Serial.print(F("N5.0 "));
  Serial.print(pms.n5p0);
  Serial.print(F(", "));
  Serial.print(F("N10 "));
  Serial.print(pms.n10p0);
  Serial.println(F(" [#/100cc]"));

  if (!pms.has_temperature_humidity() && !pms.has_formaldehyde())
    return;
  Serial.print(pms.temp, 1);
  Serial.print(F(" °C"));
  Serial.print(F(", "));
  Serial.print(pms.rhum, 1);
  Serial.print(F(" %rh"));
  Serial.print(F(", "));
  Serial.print(pms.hcho, 2);
  Serial.println(F(" mg/m3 HCHO"));
#endif
}
void setup()
{
  setup_wifi();
  client.setServer(MQTT_HOST, 1883);
  client.setCallback(callback);

  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);

  Serial.begin(115200);
  // Wait port is open or timeout
  // int i = 0;
  // while (!Serial && i < 50) {
  //     delay(10);
  //     i++;
  // }
  
  // First message, we are alive
  // Serial.println("");
  // Serial.println("Init");
      // Initialize S8 sensor
  S8_serial.begin(S8_BAUDRATE, SERIAL_8N2, S8_RX_PIN, S8_TX_PIN, false);
  sensor_S8 = new S8_UART(S8_serial);
  
  pms.init();

  // Check if S8 is available
  sensor_S8->get_firmware_version(sensor.firm_version);
  int len = strlen(sensor.firm_version);
  // if (len == 0) {
  //     Serial.println("SenseAir S8 CO2 sensor not found!");
  //     while (1) { delay(1); };
  // }

  // Show basic S8 sensor info
  // Serial.println(">>> SenseAir S8 NDIR CO2 sensor <<<");
  // printf("Firmware version: %s\n", sensor.firm_version);
  // sensor.sensor_id = sensor_S8->get_sensor_ID();
  // Serial.print("Sensor ID: 0x"); printIntToHex(sensor.sensor_id, 4); Serial.println("");

  // Serial.println("Setup done!");
  // Serial.flush();
  // Serial.println("Hello T-Display-S3");

  tft.begin();

// #if defined(LCD_MODULE_CMD_1)
//     for (uint8_t i = 0; i < (sizeof(lcd_st7789v) / sizeof(lcd_cmd_t)); i++) {
//         tft.writecommand(lcd_st7789v[i].cmd);
//         for (int j = 0; j < lcd_st7789v[i].len & 0x7f; j++) {
//             tft.writedata(lcd_st7789v[i].data[j]);
//         }

//         if (lcd_st7789v[i].len & 0x80) {
//             delay(120);
//         }
//     }
// #endif

    tft.setRotation(3);
    tft.setSwapBytes(true);
    tft.pushImage(0, 0, 320, 170, (uint16_t *)img_logo);
    delay(1000);

    ledcSetup(0, 2000, 8);
    ledcAttachPin(PIN_LCD_BL, 0);
    ledcWrite(0, 255);

    tft.fillScreen(TFT_BLACK);
    tft.setFreeFont(&FreeMono24pt7b);
}
char data[500];
long last_loop = 0;
void loop()
{
  
  if (millis()-last_loop>MAIN_LOOP_WAIT){
    last_loop = millis();
    
    ledcWrite(0,getBrightness()); /* Screen brightness can be modified by adjusting this parameter. (0-255) */

    // Get CO2 measure
    pms.read();   // read the PM sensor
    sensor.co2 = sensor_S8->get_co2();
    if (sensor.co2<700)
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
    else if (sensor.co2<1000)
      tft.setTextColor(TFT_BLUE, TFT_BLACK);
    else if (sensor.co2<1300)
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    else 
      tft.setTextColor(TFT_RED, TFT_BLACK);
    String co2_str = String(sensor.co2) + "     ";

    tft.drawString((co2_str), 0, 0, 8);
    if (pms.pm01+pms.pm25<15)
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
    else if (pms.pm01+pms.pm25<100)
      tft.setTextColor(TFT_BLUE, TFT_BLACK);
    else if (pms.pm01+pms.pm25<300)
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    else 
      tft.setTextColor(TFT_RED, TFT_BLACK);
    String part_l1 = 
      "pm1.0: " + String(pms.pm01) + "     "+
      "pm2.5: " + String(pms.pm25) + "       ";

    tft.drawString((part_l1), 0, 80, 4);
    if (pms.pm10<10)
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
    else if (pms.pm10<50)
      tft.setTextColor(TFT_BLUE, TFT_BLACK);
    else if (pms.pm10<100)
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    else 
      tft.setTextColor(TFT_RED, TFT_BLACK);
    String part_l2 = 
      "pm10: " + String(pms.pm10) +"         ";
    tft.drawString((part_l2), 0, 105, 4);

    int count_all = pms.n0p3 +pms.n0p5+pms.n1p0+pms.n2p5+pms.n5p0+pms.n10p0;
    if (count_all<400)
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
    else if (count_all<5000)
      tft.setTextColor(TFT_BLUE, TFT_BLACK);
    else if (count_all<20000)
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    else 
      tft.setTextColor(TFT_RED, TFT_BLACK);
    String part_l3 = 
      "N0.3: " + String(pms.n0p3) + " "+
      "N0.5: " + String(pms.n0p5) + " "+
      "N1.0: " + String(pms.n1p0) + "    ";
    String part_l4 = 
      "N2.5: " + String(pms.n2p5) + " "+
      "N5.0: " + String(pms.n5p0) + " "+
      "N10.0: " + String(pms.n10p0)  +"    ";

    tft.drawString((part_l3), 0, 135, 2);
    tft.drawString((part_l4), 0, 155, 2);
    if (!client.connected()) {
      reconnect();
    }
    else{
      String payload = "{ \"ppm\": " + String(sensor.co2) + ","+
        "\"pm1p0\": " + String(pms.pm01) + ","+
        "\"pm2p5\": " + String(pms.pm25) + ","+
        "\"pm10\": " + String(pms.pm10) + ","+
        "\"N0p3\": " + String(pms.n0p3) + ","+
        "\"N0p5\": " + String(pms.n0p5) + ","+
        "\"N1p0\": " + String(pms.n1p0) + ","+
        "\"N2p5\": " + String(pms.n2p5) + ","+
        "\"N5p0\": " + String(pms.n5p0) + ","+
        "\"N10p0\": " + String(pms.n10p0) + "}";
      payload.toCharArray(data, (payload.length() + 1));
      client.publish(MQTT_TOPIC_SEND, data);

      client.loop();
    }
  }
}