/*
  An example showing rainbow colours on a 1.8" TFT LCD screen
  and to show a basic example of font use.

  Make sure all the display driver and pin connections are correct by
  editing the User_Setup.h file in the TFT_eSPI library folder.

  Note that yield() or delay(0) must be called in long duration for/while
  loops to stop the ESP8266 watchdog triggering.

  #########################################################################
  ###### DON'T FORGET TO UPDATE THE User_Setup.h FILE IN THE LIBRARY ######
  #########################################################################
*/

#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <SPI.h>
#include "pin_config.h"
#include <Arduino.h>
#include "s8_uart.h"


/* BEGIN CONFIGURATION */
#define DEBUG_BAUDRATE 115200

#if (defined USE_SOFTWARE_SERIAL || defined ARDUINO_ARCH_RP2040)
  #define S8_RX_PIN 5         // Rx pin which the S8 Tx pin is attached to (change if it is needed)
  #define S8_TX_PIN 4         // Tx pin which the S8 Rx pin is attached to (change if it is needed)
#else
  #define S8_UART_PORT  1     // Change UART port if it is needed
#endif
/* END CONFIGURATION */


#ifdef USE_SOFTWARE_SERIAL
  SoftwareSerial S8_serial(S8_RX_PIN, S8_TX_PIN);
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

TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h

unsigned long targetTime = 0;
byte red = 31;
byte green = 0;
byte blue = 0;
byte state = 0;
unsigned int colour = red << 11;
uint32_t runing = 0;

void setup(void)
{
    Serial.begin(115200);
      // Wait port is open or timeout
    int i = 0;
    while (!Serial && i < 50) {
        delay(10);
        i++;
    }
    
    // First message, we are alive
    Serial.println("");
    Serial.println("Init");
        // Initialize S8 sensor
    S8_serial.begin(S8_BAUDRATE);
    sensor_S8 = new S8_UART(S8_serial);

    // Check if S8 is available
    sensor_S8->get_firmware_version(sensor.firm_version);
    int len = strlen(sensor.firm_version);
    if (len == 0) {
        Serial.println("SenseAir S8 CO2 sensor not found!");
        while (1) { delay(1); };
    }

    // Show basic S8 sensor info
    Serial.println(">>> SenseAir S8 NDIR CO2 sensor <<<");
    printf("Firmware version: %s\n", sensor.firm_version);
    sensor.sensor_id = sensor_S8->get_sensor_ID();
    Serial.print("Sensor ID: 0x"); printIntToHex(sensor.sensor_id, 4); Serial.println("");

    Serial.println("Setup done!");
    Serial.flush();
    pinMode(PIN_POWER_ON, OUTPUT);
    digitalWrite(PIN_POWER_ON, HIGH);

    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);


    tft.fillScreen(TFT_RED); delay(1000);
    tft.fillScreen(TFT_GREEN); delay(1000);
    tft.fillScreen(TFT_BLUE); delay(1000);

    targetTime = millis() + 1000;
}

void loop()
{
    // Get CO2 measure
    sensor.co2 = sensor_S8->get_co2();
    printf("CO2 value = %d ppm\n", sensor.co2);
    if (millis() > runing) {
        Serial.print("Current running ");
        Serial.print(millis());
        Serial.println(" millis");
        runing = millis() + 1000;
    }
    if (targetTime < millis()) {
        targetTime = millis() + 10000;

        // Colour changing state machine
        for (int i = 0; i < tft.width(); i++) {
            tft.drawFastVLine(i, 0, tft.height(), colour);
            switch (state) {
            case 0:
                green += 2;
                if (green == 64) {
                    green = 63;
                    state = 1;
                }
                break;
            case 1:
                red--;
                if (red == 255) {
                    red = 0;
                    state = 2;
                }
                break;
            case 2:
                blue ++;
                if (blue == 32) {
                    blue = 31;
                    state = 3;
                }
                break;
            case 3:
                green -= 2;
                if (green == 255) {
                    green = 0;
                    state = 4;
                }
                break;
            case 4:
                red ++;
                if (red == 32) {
                    red = 31;
                    state = 5;
                }
                break;
            case 5:
                blue --;
                if (blue == 255) {
                    blue = 0;
                    state = 0;
                }
                break;
            }
            colour = red << 11 | green << 5 | blue;
        }

        // The standard ADAFruit font still works as before
        tft.setTextColor(TFT_BLACK);
        tft.setCursor (12, 5);
        tft.print("Original ADAfruit font!");

        // The new larger fonts do not use the .setCursor call, coords are embedded
        tft.setTextColor(TFT_BLACK, TFT_BLACK); // Do not plot the background colour

        // Overlay the black text on top of the rainbow plot (the advantage of not drawing the backgorund colour!)
        tft.drawCentreString("Font size 2", 80, 14, 2); // Draw text centre at position 80, 12 using font 2

        //tft.drawCentreString("Font size 2",81,12,2); // Draw text centre at position 80, 12 using font 2

        tft.drawCentreString("Font size 4", 80, 30, 4); // Draw text centre at position 80, 24 using font 4

        tft.drawCentreString("12.34", 80, 54, 6); // Draw text centre at position 80, 24 using font 6

        tft.drawCentreString("12.34 is in font size 6", 80, 92, 2); // Draw text centre at position 80, 90 using font 2

        // Note the x position is the top left of the font!

        // draw a floating point number
        float pi = 3.14159; // Value to print
        int precision = 3;  // Number of digits after decimal point
        int xpos = 50;      // x position
        int ypos = 110;     // y position
        int font = 2;       // font number only 2,4,6,7 valid. Font 6 only contains characters [space] 0 1 2 3 4 5 6 7 8 9 0 : a p m
        xpos += tft.drawFloat(pi, precision, xpos, ypos, font); // Draw rounded number and return new xpos delta for next print position
        tft.drawString(" is pi", xpos, ypos, font); // Continue printing from new x position
    }
}






