#include <Arduino.h>

#define SDA_PIN 16
#define SCL_PIN 17
#define ESP32_CAN_RX_PIN GPIO_NUM_34
#define ESP32_CAN_TX_PIN GPIO_NUM_32

#define DUT_POWER_ON_PIN 14
#define READY_TO_FLASH_PIN 27
#define FLASH_OK_PIN 13
#define DUT_PRESENT_PIN 15

#define ATT_I2C_ADDR 0x6d
#define VIN_MAX 32.0

#define DS3231_I2C_ADDR 0x68

#include <functional>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_ADS1X15.h>
#include <N2kMessages.h>
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <Wire.h>
#include <elapsedMillis.h>

#include "ReactESP.h"
#include "SensESP.h"
#include "sensors/digital_input.h"
#include "system/lambda_consumer.h"

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

TwoWire* i2c;
Adafruit_SSD1306* display;
Adafruit_ADS1115* ads1115;

// always wait between the state transitions until elapsedMillis > wait_until
elapsedMillis waiting;
int wait_duration = 0;

// only transmit the bogus temperature if this is true
bool can_tx_enabled = false;

void set_wait_duration(int duration) {
  wait_duration = duration;
  waiting = 0;
}

// test result variables
int flashing_successful = -1;
float vsup = -1;
float vcc = -1;
float vin = -1;
float wind_speed = 0;


enum class StateMachineState {
  NONE = 0,
  START,
  wait_for_dut,
  dut_present,
  wait_for_flasher,
  flasher_active,
  wait_flash_ok,
  flash_ok,
  flash_timeout,
  dut_powered,
  measure_vsup,
  vsup_ok,
  vsup_fail,
  measure_vcc,
  vcc_ok,
  vcc_fail,
  test_attiny_i2c,
  attiny_i2c_ok,
  attiny_i2c_fail,
  test_rtc_i2c,
  rtc_i2c_ok,
  rtc_i2c_fail,
  enable_can_tx,
  test_can_rx,
  can_rx_ok,
  can_rx_fail,
  report_test_result,
  END
};

StateMachineState state_machine_state = StateMachineState::START;
StateMachineState prev_state = StateMachineState::NONE;

void set_sm_state(StateMachineState new_state) {
  state_machine_state = new_state;
}

void state_NONE() {
  Serial.println("WARNING: Should never enter NONE state");
  set_sm_state(StateMachineState::START);
}

void state_START() {
  digitalWrite(READY_TO_FLASH_PIN, 0);
  digitalWrite(DUT_POWER_ON_PIN, 0);
  flashing_successful = false;
  vsup = -1;
  vcc = -1;
  vin = -1;
  wind_speed = 0;
  can_tx_enabled = false;

  set_sm_state(StateMachineState::wait_for_dut);
}

bool dut_present() {
  return !digitalRead(DUT_PRESENT_PIN);
}

void state_wait_for_dut() {
  if (dut_present()) {
    set_sm_state(StateMachineState::dut_present);
  }
}

elapsedMillis flashing = 0;
const int flash_timeout = 60*1000;  // flashing should be finished in 60 s

void state_dut_present() {
  // inform the flasher that it's OK to start flashing
  //digitalWrite(READY_TO_FLASH_PIN, true);
  //flashing = 0;
  //set_sm_state(StateMachineState::wait_for_flasher);
  set_sm_state(StateMachineState::flash_ok);
}

bool flasher_present() {
  return digitalRead(FLASH_OK_PIN);
}

void state_wait_for_flasher() {
  if (flasher_present()) {
    set_sm_state(StateMachineState::flasher_active);
  }
}

bool flasher_active() {
  // flasher has started flashing if FLASH_OK is low
  return !digitalRead(FLASH_OK_PIN);
}

void state_flasher_active() {
  if (flasher_active()) {
    set_sm_state(StateMachineState::wait_flash_ok);
  }
}

bool flash_ok() {
  return digitalRead(FLASH_OK_PIN);
}

void state_wait_flash_ok() {
  if (flashing > flash_timeout) {
    set_sm_state(StateMachineState::flash_timeout);
    return;
  }
  if (flash_ok()) {
    set_sm_state(StateMachineState::flash_ok);
  }
}

void state_flash_ok() {
  flashing_successful = true;
  digitalWrite(READY_TO_FLASH_PIN, false);
  digitalWrite(DUT_POWER_ON_PIN, true);
  set_sm_state(StateMachineState::dut_powered);
}

void state_flash_timeout() {
  flashing_successful = false;
  // unable to continue
  set_sm_state(StateMachineState::report_test_result);
}

void state_dut_powered() {
  // allow for Vsup to stabilize
  set_wait_duration(1000);
  set_sm_state(StateMachineState::measure_vsup);
}

float measure_vsup() {
  float vsup = (4.096/32768) * ads1115->readADC_SingleEnded(0);
  return vsup;
}

void state_measure_vsup() {
  vsup = ((300.+200.)/200.) * measure_vsup();
  Serial.printf("Vsup: %f\n", vsup);
  if (vsup < 1.5 || vsup > 2.7) {
    // fatal: can't expect vcc to turn on
    set_sm_state(StateMachineState::report_test_result);
  } else if (vsup < 2.5 || vsup > 2.68) {
    // out of bounds
    set_sm_state(StateMachineState::vsup_fail);
  } else {
    set_sm_state(StateMachineState::vsup_ok);
  }
}

void state_vsup_ok() {
  // TODO: report result
  set_wait_duration(1000);
  set_sm_state(StateMachineState::measure_vcc);
}

void state_vsup_fail() {
  // TODO: report result
  set_wait_duration(1000);
  set_sm_state(StateMachineState::measure_vcc);
}

float measure_vcc() {
  float voltage = (4.096/32768) * ads1115->readADC_SingleEnded(2);
  // adjust for the R1 = 1k , R2 = 1k voltage divider
  return voltage * ((1. + 1.) / 1.);
}

void state_measure_vcc() {
  float vcc = measure_vcc();
  Serial.printf("Vcc: %f\n", vcc);
  if (vcc < 5 || vcc > 5.2) {
    // vcc out of range
    set_sm_state(StateMachineState::vcc_fail);
  } else {
    set_sm_state(StateMachineState::vcc_ok);
  }
}

void state_vcc_ok() {
  // TODO: report result
  set_sm_state(StateMachineState::test_attiny_i2c);
}

void state_vcc_fail() {
  // TODO: report result
  set_sm_state(StateMachineState::report_test_result);
}

float measure_vin() {
  float voltage = (4.096/32768) * ads1115->readADC_SingleEnded(3);
  // adjust for the R1 = 4.7k , R2 = 1k voltage divider
  return voltage * ((1. + 4.7) / 1.);
}

bool read_i2c_byte_register(int addr, int register, uint8_t* value) {
  i2c->beginTransmission(ATT_I2C_ADDR);
  i2c->write(0x20);
  i2c->endTransmission();

  i2c->requestFrom(ATT_I2C_ADDR, 1);
  // wait a bit
  delay(1);
  if (i2c->available() >= 1) {
    *value = i2c->read();
    return true;
  } else {
    // device is not present
    return false;
  }
}

float get_att_vin() {
  float vin;
  uint8_t value;
  
  if (read_i2c_byte_register(ATT_I2C_ADDR, 0x20, &value)) {
    vin = VIN_MAX * value / 256.;
    return vin;
  } else {
    return -1;
  }
}

float get_att_vsup() {
  float vsup;
  uint8_t value;
  
  if (read_i2c_byte_register(ATT_I2C_ADDR, 0x21, &value)) {
    vsup = VIN_MAX * value / 256.;
    return vsup;
  } else {
    return -1;
  }
}

void state_test_attiny_i2c() {
  float vin = measure_vin();
  float att_vin = get_att_vin();
  float att_vsup = get_att_vsup();

  if (att_vin == -1 || att_vsup == -1) {
    // failed to read
    // TODO: report result
    set_sm_state(StateMachineState::attiny_i2c_fail);
  } else {
    // TODO: test for value correctness
    set_sm_state(StateMachineState::attiny_i2c_ok);
  }
}

void state_attiny_i2c_ok() {
  // TODO: report result
  set_sm_state(StateMachineState::test_rtc_i2c);
}

void state_attiny_i2c_fail() {
  // TODO: report result
  set_sm_state(StateMachineState::test_rtc_i2c);
}

int bcd_to_dec(uint8_t val) {
// Convert binary coded decimal to normal decimal numbers
	return ( (val/16*10) + (val%16) );
}

int get_rtc_seconds() {
  uint8_t value;
  if (read_i2c_byte_register(DS3231_I2C_ADDR, 0x00, &value)) {
    return bcd_to_dec(value);
  } else {
    return -1;
  }
}

void state_test_rtc_i2c() {
  int seconds = get_rtc_seconds();
  if (seconds == -1) {
    // TODO: report result
    set_sm_state(StateMachineState::rtc_i2c_fail);
  } else {
    set_sm_state(StateMachineState::rtc_i2c_ok);
  }
}

void state_rtc_i2c_ok() {
  // TODO: report result
  set_sm_state(StateMachineState::enable_can_tx);
}

void state_rtc_i2c_fail() {
  // TODO: report result
  set_sm_state(StateMachineState::enable_can_tx);
}

elapsedMillis waiting_for_can_rx = 0;

void state_enable_can_tx() {
  can_tx_enabled = true;
  waiting_for_can_rx = 0;
  set_sm_state(StateMachineState::test_can_rx);
}

void state_test_can_rx() {
  if (wind_speed == 42) {
    set_sm_state(StateMachineState::can_rx_ok);
    return;
  }
  
  if (waiting_for_can_rx > 5000) {
    set_sm_state(StateMachineState::can_rx_fail);
  }
}

void state_can_rx_ok() {
  // TODO: report result
  Serial.println("Got CAN rx");
  set_sm_state(StateMachineState::report_test_result);
}

void state_can_rx_fail() {
  // TODO: report result
  Serial.println("CAN rx failed");
  set_sm_state(StateMachineState::report_test_result);
}

void state_report_test_result() {

  set_sm_state(StateMachineState::END);
}

void state_END() {
  // wait until DUT is removed
}

std::function<void()> state_function[] = {
  state_NONE,
  state_START,
  state_wait_for_dut,
  state_dut_present,
  state_wait_for_flasher,
  state_flasher_active,
  state_wait_flash_ok,
  state_flash_ok,
  state_flash_timeout,
  state_dut_powered,
  state_measure_vsup,
  state_vsup_ok,
  state_vsup_fail,
  state_measure_vcc,
  state_vcc_ok,
  state_vcc_fail,
  state_test_attiny_i2c,
  state_attiny_i2c_ok,
  state_attiny_i2c_fail,
  state_test_rtc_i2c,
  state_rtc_i2c_ok,
  state_rtc_i2c_fail,
  state_enable_can_tx,
  state_test_can_rx,
  state_can_rx_ok,
  state_can_rx_fail,
  state_report_test_result,
  state_END
};

String state_name[] = {
  "NONE",
  "START",
  "wait_for_dut",
  "dut_present",
  "wait_for_flasher",
  "flasher_active",
  "wait_flash_ok",
  "flash_ok",
  "flash_timeout",
  "dut_powered",
  "measure_vsup",
  "vsup_ok",
  "vsup_fail",
  "measure_vcc",
  "vcc_ok",
  "vcc_fail",
  "test_attiny_i2c",
  "attiny_i2c_ok",
  "attiny_i2c_fail",
  "test_rtc_i2c",
  "rtc_i2c_ok",
  "rtc_i2c_fail",
  "enable_can_tx",
  "test_can_rx",
  "can_rx_ok",
  "can_rx_fail",
  "report_test_result",
  "END"
};

void print_state(StateMachineState state) {
  display->begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display->setRotation(2);
  display->clearDisplay();
  display->setTextSize(1);
  display->setCursor(0, 0);
  display->setTextColor(SSD1306_WHITE);
  display->printf("Uptime: %lu\n", micros() / 1000000);
  display->printf("%s\n", state_name[(int)state].c_str());
  display->display();

  Serial.printf("State: %s\n", state_name[(int)state].c_str());
  delay(5);
}

void handle_state_machine() {
  if (waiting < wait_duration) {
    return;
  }

  bool dut_present = !digitalRead(DUT_PRESENT_PIN);
  if (!dut_present && state_machine_state >= StateMachineState::dut_present) {
    // unexpected removal of DUT
    Serial.println("DUT removed; returning to START");
    set_sm_state(StateMachineState::START);
  }

  if (prev_state != state_machine_state) {
    print_state(state_machine_state);
  }

  prev_state = state_machine_state;

  state_function[(int)state_machine_state]();
}

void handle_nmea2000_messages(const tN2kMsg& message) {
  unsigned char SID;
  double speed;
  double angle;
  tN2kWindReference reference;

  Serial.printf("Received PGN: %ld\n", message.PGN);

  if (message.PGN == 130306) {
    if (ParseN2kWindSpeed(message, SID, speed, angle, reference)) {
      wind_speed = speed;
    }
  }
}


bool assert_int_almost_equal(int actual, int expected, int tol, String name) {
  if ((expected - tol < actual) && (actual < expected + tol)) {
    Serial.printf("%s OK: %d ~= %d\n", name.c_str(), actual, expected);
    return true;
  } else {
    Serial.printf("%s invalid: %d != %d\n", name.c_str(), actual, expected);
    return false;
  }
}

ReactESP app([]() {
  // setup serial output
  Serial.begin(115200);
  delay(100);

  Serial.println("Started.");

  // toggle the LED pin at rate of 2 Hz
  pinMode(LED_BUILTIN, OUTPUT);
  app.onRepeatMicros(1e6 / 2, []() {
    static bool state = false;
    digitalWrite(LED_BUILTIN, state);
    state = !state;
  });

  // initialize pins
  pinMode(DUT_POWER_ON_PIN, OUTPUT);
  pinMode(READY_TO_FLASH_PIN, OUTPUT);
  pinMode(FLASH_OK_PIN, INPUT);
  pinMode(DUT_PRESENT_PIN, INPUT);

  // initialize ADS1115
  ads1115 = new Adafruit_ADS1115;
  ads1115->begin();
  ads1115->setGain(GAIN_ONE);

  // initialize the display
  i2c = new TwoWire(0);
  i2c->begin(SDA_PIN, SCL_PIN);
  display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, i2c, -1);
  if (!display->begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  delay(100);
  display->setRotation(2);
  display->clearDisplay();
  display->display();

  // set up the state machine
  app.onRepeat(1, []() {
    handle_state_machine();
  });

  app.onRepeat(800, [] {
    if (can_tx_enabled) {
      tN2kMsg N2kMsg;
      Serial.println("Sending main cabin temperature");
      SetN2kTemperatureExt(N2kMsg, 1, 1, N2kts_MainCabinTemperature, 273.15 + 24.5);
      NMEA2000.SendMsg(N2kMsg);
    }
  });

  // input the NMEA 2000 messages

  // Reserve enough buffer for sending all messages. This does not work on small
  // memory devices like Uno or Mega
  NMEA2000.SetN2kCANSendFrameBufSize(250);
  // Set Product information
  NMEA2000.SetProductInformation(
      "20210506",                      // Manufacturer's Model serial code
      103,                             // Manufacturer's product code
      "SH-RPi Test Jig Controller",  // Manufacturer's Model ID
      "0.1.0.1 (2021-05-06)",          // Manufacturer's Software version code
      "0.0.3.1 (2021-05-06)"           // Manufacturer's Model version
  );
  // Set device information
  NMEA2000.SetDeviceInformation(
      1,    // Unique number. Use e.g. Serial number.
      130,  // Device function=Analog to NMEA 2000 Gateway. See codes on
            // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
      10,   // Device class=Inter/Intranetwork Device. See codes on
           // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
      2046  // Just choosen free from code list on
            // http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
  );

  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, 22);
  NMEA2000.EnableForward(false);  // Disable all msg forwarding to USB (=Serial)
  NMEA2000.SetMsgHandler(handle_nmea2000_messages);
  NMEA2000.Open();

  // No need to parse the messages at every single loop iteration; 1 ms will do
  app.onRepeat(1, []() { NMEA2000.ParseMessages(); });

  // enable all object that need enabling
  Enable::enable_all();
});
