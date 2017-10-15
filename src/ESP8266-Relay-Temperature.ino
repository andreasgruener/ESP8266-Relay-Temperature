#include <Homie.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <U8g2lib.h>

#define FW_NAME "Serverschrank"
#define FW_VERSION "2.3"

/* Magic sequence for Autodetectable Binary Upload */
const char *__FLAGGED_FW_NAME = "\xbf\x84\xe4\x13\x54" FW_NAME "\x93\x44\x6b\xa7\x75";
const char *__FLAGGED_FW_VERSION = "\x6a\x3f\x3e\x0e\xe1" FW_VERSION "\xb0\x30\x48\xd4\x1a";
/* End of magic sequence for Autodetectable Binary Upload */

// comment
const int PIN_RELAY = 16; // D0;
const int TEMPERATURE_INTERVAL = 60;
const float TURN_OFF_FAN_TEMPERATURE = 32.0f;
const float TURN_ON_FAN_TEMPERATURE = 33.0f;


int temperature_interval = TEMPERATURE_INTERVAL;
int measure_interval = 1;

unsigned long lastTemperatureSent = 0;
unsigned long lastRetry = 0;
int maxRetries = 5;
int retries = 0;
bool fanMode = false;
bool fanAutomatic = true;

float turnOffFanTemperature =  TURN_OFF_FAN_TEMPERATURE;
float turnOnFanTemperature = TURN_ON_FAN_TEMPERATURE;
float minTemperature = 99.0f;
float maxTemperature = -99.0f;
float currentTemperature = -99.0f;
float currentPressure = 0.0f;


U8G2_SSD1306_64X48_ER_F_HW_I2C u8g2(U8G2_R0); // hardware

#define SEALEVELPRESSURE_HPA (1013.25)


HomieNode lightNode("light", "switch");
HomieNode temperatureNode("temperature", "temperature");
HomieNode fanNode("fan", "fan");
HomieNode humidityNode("humidity", "humidity");

HomieNode pressureNode("pressure", "pressure");
HomieNode dewpointNode("dewpoint", "dewpoint");

// Temperature Bosch BMP
Adafruit_BMP280 bme;


void setup() {
  Serial.begin(115200);
  Serial << endl << endl;
  
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, LOW);

  Homie_setFirmware(FW_NAME, FW_VERSION);
  Homie_setBrand("uh27-iot");
  Homie.setSetupFunction(setupHandler).setLoopFunction(loopHandler);
 // accept broadcast messages
  Homie.setBroadcastHandler(broadcastHandler); // before Homie.setup()

  Homie.setup();

  bool success = bme.begin(0x76); //0x76); // DEFAULT S
  if ( !success ) {
    Serial.println("Could not find BME280 sensor!");
  }
  else {
    Serial.println("Found BMP");
  }
  u8g2.begin();
  u8g2.setFlipMode(true);
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_u8glib_4_tf);
    u8g2.drawStr(0,4,"Starting..");
  } while( u8g2.nextPage() );

}

/**
 * Initializes Homie Nodes
 */
void setupHandler() {
  
  lightNode.advertise("on").settable(lightOnHandler);
  lightNode.setProperty("on").send("false");
  
  fanNode.advertise("on").settable(fanOnHandler);
  fanNode.setProperty("on").send("false");
  fanNode.advertise("automatic").settable(fanAutomaticHandler);
  fanNode.setProperty("automatic").send("true");
  fanNode.advertise("fanOff").settable(fanTurnOffHandler);
  fanNode.setProperty("fanOff").send(String(turnOffFanTemperature));
  fanNode.advertise("fanOn").settable(fanTurnOnHandler);
  fanNode.setProperty("fanOn").send(String(turnOnFanTemperature));

  temperatureNode.setProperty("unit").send("c");
  temperatureNode.advertise("sensor_ok");
  temperatureNode.advertise("unit");
  temperatureNode.advertise("degrees");
  temperatureNode.advertise("delay").settable(delayHandler);
  temperatureNode.setProperty("delay").send(String(temperature_interval));

  pressureNode.setProperty("unit").send("hPa");
  pressureNode.advertise("hPa");

}

/*
 * Handler for setting temperature measurement delay
 */
bool delayHandler(const HomieRange& range, const String& value) { 
  Homie.getLogger() << "Got Delay " << value << endl;;
  temperature_interval = value.toInt();
  temperatureNode.setProperty("delay").send((String)temperature_interval);
  Homie.getLogger() << "Delay is " << temperature_interval << endl;
  return true;
}


bool lightOnHandler(const HomieRange& range, const String& value) {
  if (value != "true" && value != "false") return false;

  bool on = (value == "true");
  digitalWrite(PIN_RELAY, on ? HIGH : LOW);
  lightNode.setProperty("on").send(value);
  Homie.getLogger() << "Light is " << (on ? "on" : "off") << endl;

  return true;
}


/**
 * Unused sofar
 */
bool broadcastHandler(const String& level, const String& value) {
  Serial << "Received broadcast level " << level << ": " << value << endl;
  return true;
}


bool fanOnHandler(const HomieRange& range, const String& value) {
  if (value != "true" && value != "false") return false;

  bool on = (value == "true");
  fanSwitch(on);

  return true;
}

bool fanTurnOffHandler(const HomieRange& range, const String& value) {
  Homie.getLogger() << "Got fan turn off Temperature: " << value << endl;;
  setTurnOffTemperature(value.toInt());
  return true;
}

bool fanTurnOnHandler(const HomieRange& range, const String& value) {
  Homie.getLogger() << "Got fan turn on Temperature: " << value << endl;;
  setTurnOnTemperature(value.toInt());
  return true;
}

bool fanAutomaticHandler(const HomieRange& range, const String& value) {
  if (value != "true" && value != "false") return false;

  bool on = (value == "true");
  setAutomaticFan(on);
  return true;
}

void setTurnOnTemperature(float temperature) {
  turnOnFanTemperature = temperature;
  checkMeasurement(currentTemperature, currentPressure);
}

void setTurnOffTemperature(float temperature) {
  turnOffFanTemperature = temperature;
  checkMeasurement(currentTemperature, currentPressure);
}

void setAutomaticFan(bool automatic) {
  fanAutomatic = automatic;
  checkMeasurement(currentTemperature, currentPressure);
}


void fanSwitch(bool on) {
  digitalWrite(PIN_RELAY, on ? HIGH : LOW);
  fanNode.setProperty("on").send((on ? "true" : "false"));
  Homie.getLogger() << "Relay is " << (on ? "on" : "off") << endl;
  fanMode = on;
  display();
}

void display() {
  char FAN_ON[9] = "Fan ON";
  char FAN_OFF[10] = "Fan OFF";
  char cTf[5];
  char cPf[5];
  char maxf[5];
  char minf[5];
  char turnOfff[5];
  char turnOnf[5];


  dtostrf(currentTemperature, 2, 1, cTf);
  dtostrf(currentPressure, 4, 0, cPf);
  dtostrf(maxTemperature, 2, 1, maxf);
  dtostrf(minTemperature, 2, 1, minf);
  dtostrf(turnOffFanTemperature, 2,0, turnOfff);
  dtostrf(turnOnFanTemperature, 2,0, turnOnf);

  u8g2.firstPage();
  do {
    displaySmallRow(6,  minf, maxf);
    display3SmallRow(14,  turnOfff, fanMode ? FAN_ON : FAN_OFF, turnOnf);
    displayRow(38, cTf);
    displaySmallRow(48, "Luftdruck", cPf);
  } while ( u8g2.nextPage() );
  
}

void display2SmallRow(int row, char title[], char value[], char title2[], char value2[]) {
  u8g2.setFont(u8g2_font_profont10_mf);
  u8g2.drawStr(0,row,title);
  int string_width = u8g2.getStrWidth(value);
  u8g2.drawStr((32-string_width),row,value);
  u8g2.drawStr(32,row,title2);
  string_width = u8g2.getStrWidth(value2);
  u8g2.drawStr((64-string_width),row,value2);
}

void display3SmallRow(int row, char value[], char title[], char value2[]) {
  u8g2.setFont(u8g2_font_profont10_mf);
  u8g2.drawStr(0,row,value);
  int string_width = u8g2.getStrWidth(title);
  u8g2.drawStr((64-string_width)/2,row,title);  // centered
  string_width = u8g2.getStrWidth(value2);
  u8g2.drawStr((64-string_width),row,value2);
}

void displaySmallRow(int row, char title[], char value[]) {
  u8g2.setFont(u8g2_font_profont10_mf);
  u8g2.drawStr(0,row,title);
  int string_width = u8g2.getStrWidth(value);
  u8g2.drawStr((64-string_width),row,value);
}

void displayRow(int row, char value[]) {
  u8g2.setFont(u8g2_font_helvB08_tf);
  u8g2.drawStr(5,row,"C");
  u8g2.drawGlyph(0,row,0x00b0);
  u8g2.setFont(u8g2_font_helvB18_tn);
  int string_width = u8g2.getStrWidth(value);
  u8g2.drawStr((64-string_width),row,value);
}

/**
 * Check if fan limits are reached
 */
void checkMeasurement(float temperature, float pressure) {

  currentTemperature = temperature;
  currentPressure = pressure;

  maxTemperature = currentTemperature > maxTemperature ? currentTemperature : maxTemperature;
  minTemperature = currentTemperature < minTemperature ? currentTemperature : minTemperature;

  if (fanAutomatic && currentTemperature >= turnOnFanTemperature ) {
    fanSwitch(true);
  }
  if ( fanAutomatic && currentTemperature <= turnOffFanTemperature ) {
    fanSwitch(false);
  }

}


void checkSensors() {
  
  if (  ( retries < maxRetries && millis() - lastRetry >= 3000UL ) ||
        millis() - lastTemperatureSent >= temperature_interval * 1000UL ) {

    float temperature = 0;
    float pressure = 0;
    lastTemperatureSent =  millis();

    // successful the lasttime, new game
    if ( retries > maxRetries ) {
      retries = 0;
    } else if ( retries == maxRetries ) {
      Homie.getLogger() << "Max retries reached - Sensor not OK - giving up .." << endl;
      temperatureNode.setProperty("sensor_ok").send("0");
      retries++;
      return;
    }

    // try getting temperature -- sometimes there is no answer from the sensor
    temperature = bme.readTemperature();
    //Homie.getLogger() << "Temperature reading " << temperature << endl;
    if ( isnan(temperature) || temperature == 0 ) {
      retries++;
      Homie.getLogger() << "*** Temperature not available retrying " << retries << ". time " << " max Retries = " << lastRetry << "... " << millis() << endl;
      lastRetry = millis();
      temperatureNode.setProperty("sensor_ok").send("0");
    }
    else {

      Homie.getLogger() << "Temperature: " << temperature << " °C" << endl;
      temperatureNode.setProperty("degrees").send(String(temperature));

      pressure = bme.readPressure() / 100.0f;  // Pascal --> hPa
      Homie.getLogger() << "Pressure: " << pressure << " hPa" << endl;
      pressureNode.setProperty("hPa").send(String(pressure));

      temperatureNode.setProperty("sensor_ok").send("1");

      lastTemperatureSent = millis();
      //lastRetry = millis();
      retries = maxRetries + 1;
      
      checkMeasurement(temperature, pressure);
    }
    display();
  }
}





void loopHandler() {
  //Homie.getLogger() << "## Temperature: " << checkGY906Temperature() << " C" << endl;
  checkSensors();
}

void loop() {

  Homie.loop();
}
