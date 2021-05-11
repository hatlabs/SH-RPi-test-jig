#include <Arduino.h>

#define SPI_CS_PIN 15
#define SPI_INT_PIN 27

#include <SPI.h>
#include <N2kMessages.h>
#include <NMEA2000_mcp.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object

#include "ReactESP.h"

tNMEA2000* nmea2000;

void handle_nmea2000_messages(const tN2kMsg& message) {
  unsigned char SID;
  unsigned char instance;
  tN2kTempSource source;
  double actual;
  double setpoint_temperature;

  if (message.PGN == 130316) {
    if (ParseN2kTemperatureExt(message, SID, instance, source, actual,
                               setpoint_temperature)) {
      if (source == N2kts_MainCabinTemperature) {
        Serial.println("Received PGN 130316.");
        if (actual == 42.) {
          Serial.println("Responding with Wind Speed");
          // send a response
          tN2kMsg N2kMsg;
          SetN2kWindSpeed(N2kMsg, 1, 42., 2., N2kWind_True_North);
          nmea2000->SendMsg(N2kMsg);
        };
      }
    }
  }
}

ReactESP app([]() {
  // setup serial output
  Serial.begin(115200);
  delay(100);

  tNMEA2000_mcp* n2k = new tNMEA2000_mcp(SPI_CS_PIN, MCP_16MHz, SPI_INT_PIN);
  nmea2000 = n2k;

  // toggle the LED pin at the rate of 4 Hz
  pinMode(LED_BUILTIN, OUTPUT);
  app.onRepeatMicros(1e6/40, []() {
    static bool state = false;
    digitalWrite(LED_BUILTIN, state);
    state = !state;
  });  

  // send some bogus repetitive data
  app.onRepeat(2000, []() {
    tN2kMsg N2kMsg;
    Serial.println("Sending boat speed.");
    SetN2kBoatSpeed(N2kMsg, 1, 1.0, 2.0, N2kSWRT_Paddle_wheel);
    nmea2000->SendMsg(N2kMsg);
  });

  // Reserve enough buffer for sending all messages.
  nmea2000->SetN2kCANSendFrameBufSize(250);
  // Set Product information
  nmea2000->SetProductInformation(
      "20210510",              // Manufacturer's Model serial code
      102,                     // Manufacturer's product code
      "SH-RPi Test Jig Responder",       // Manufacturer's Model ID
      "0.1.0.1 (2021-05-10)",  // Manufacturer's Software version code
      "0.1.0.0 (2021-05-10)"   // Manufacturer's Model version
  );
  // Set device information
  nmea2000->SetDeviceInformation(
      1,    // Unique number. Use e.g. Serial number.
      132,  // Device function=Analog to NMEA 2000 Gateway. See codes on
            // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
      25,   // Device class=Inter/Intranetwork Device. See codes on
           // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
      2046  // Just choosen free from code list on
            // http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
  );

  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly, 22);
  nmea2000->EnableForward(false);  // Disable all msg forwarding to USB (=Serial)
  nmea2000->Open();

  // No need to parse the messages at every single loop iteration; 1 ms will do
  app.onRepeat(1, []() { nmea2000->ParseMessages(); });

  app.onRepeat(100, []() {
    tN2kMsg N2kMsg;
    SetN2kTemperatureExt(N2kMsg, 1, 1, N2kts_MainCabinTemperature, 273.15 + 24.5);
    nmea2000->SendMsg(N2kMsg);
  });
});
