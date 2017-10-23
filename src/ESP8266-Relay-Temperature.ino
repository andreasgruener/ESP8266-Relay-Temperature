#include <Homie.h>
#include <OneWire.h> 
#include <DallasTemperature.h>
#include <U8g2lib.h>

#define FW_NAME "Serverschrank"
#define FW_VERSION "2.6"

/* Magic sequence for Autodetectable Binary Upload */
const char *__FLAGGED_FW_NAME = "\xbf\x84\xe4\x13\x54" FW_NAME "\x93\x44\x6b\xa7\x75";
const char *__FLAGGED_FW_VERSION = "\x6a\x3f\x3e\x0e\xe1" FW_VERSION "\xb0\x30\x48\xd4\x1a";
/* End of magic sequence for Autodetectable Binary Upload */

// comment
const int PIN_RELAY = 16; // D0;
const int TEMPERATURE_INTERVAL = 2;
const float TURN_OFF_FAN_TEMPERATURE = 33.0f;
const float TURN_ON_FAN_TEMPERATURE = 35.0f;


int temperature_interval = TEMPERATURE_INTERVAL;
int measure_interval = 1;

unsigned long lastTemperatureSent = 0;
unsigned long lastRetry = 0;
int maxRetries = 5;
int retries = 0;
bool fanIsRunning = false;
bool fanAutomatic = true;

float turnOffFanTemperature =  TURN_OFF_FAN_TEMPERATURE;
float turnOnFanTemperature = TURN_ON_FAN_TEMPERATURE;
float minTemperature = 99.0f;
float maxTemperature = -99.0f;
float currentTemperature = -99.0f;


U8G2_SSD1306_64X48_ER_F_HW_I2C u8g2(U8G2_R0); // hardware

#define ONE_WIRE_BUS 12       // D1
#define ONE_WIRE_MAX_DEV 5 //The maximum number of devices
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);

int numberOfDevices; //Number of temperature devices found
DeviceAddress devAddr[ONE_WIRE_MAX_DEV];  //An array device temperature sensors
float tempDev[ONE_WIRE_MAX_DEV]; //Saving the last measurement of temperature
float tempDevLast[ONE_WIRE_MAX_DEV]; //Previous temperature measurement



HomieNode lightNode("light", "switch");
HomieNode temperatureNode("temperature", "temperature");
HomieNode fanNode("fan", "fan");
HomieNode humidityNode("humidity", "humidity");




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

  u8g2.begin();
  u8g2.setFlipMode(true);
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_u8glib_4_tf);
    u8g2.drawStr(0,4,"Scanning devices..");
  } while( u8g2.nextPage() );
  
  // Start up the library
  DS18B20.begin();
  Serial.print("Found ");
  int deviceCount = DS18B20.getDeviceCount();
  char dc[5];
  dtostrf(deviceCount, 2, 0, dc);

  // bool success = bme.begin(0x76); //0x76); // DEFAULT S
  // if ( !success ) {
  //   Serial.println("Could not find BME280 sensor!");
  // }
  // else {
  //   Serial.println("Found BMP");
  // }
  u8g2.begin();
  u8g2.setFlipMode(true);
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_u8glib_4_tf);
    u8g2.drawStr(0,14,"Starting..");
    u8g2.drawStr(0,24,"Found");
    u8g2.drawStr(30,24,dc);
    u8g2.drawStr(40,24,"Devices");
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
  checkMeasurement(currentTemperature);
}

void setTurnOffTemperature(float temperature) {
  turnOffFanTemperature = temperature;
  checkMeasurement(currentTemperature);
}

void setAutomaticFan(bool automatic) {
  fanAutomatic = automatic;
  checkMeasurement(currentTemperature);
}


void fanSwitch(bool on) {
  digitalWrite(PIN_RELAY, on ? HIGH : LOW);
  fanNode.setProperty("on").send((on ? "true" : "false"));
  Homie.getLogger() << "Relay is " << (on ? "on" : "off") << endl;
  fanIsRunning = on;
  display();
}

void display() {
  char FAN_ON[9] = "Fan ON";
  char FAN_OFF[10] = "Fan OFF";
 // char PRESSURE_TITLE[10] = "Luftdruck";
  char cTf[5];
  char cPf[5];
  char maxf[5];
  char minf[5];
  char turnOfff[5];
  char turnOnf[5];


  dtostrf(currentTemperature, 2, 1, cTf);

  dtostrf(maxTemperature, 2, 1, maxf);
  dtostrf(minTemperature, 2, 1, minf);
  dtostrf(turnOffFanTemperature, 2,0, turnOfff);
  dtostrf(turnOnFanTemperature, 2,0, turnOnf);

  u8g2.firstPage();
  do {
    displaySmallRow(6,  minf, maxf);
    display3SmallRow(14,  turnOfff, fanIsRunning ? FAN_ON : FAN_OFF, turnOnf);
    displayRow(38, cTf);
   // displaySmallRow(48, PRESSURE_TITLE, cPf);
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
void checkMeasurement(float temperature) {

  currentTemperature = temperature;

  maxTemperature = currentTemperature > maxTemperature ? currentTemperature : maxTemperature;
  minTemperature = currentTemperature < minTemperature ? currentTemperature : minTemperature;

  if (fanAutomatic && !fanIsRunning && currentTemperature >= turnOnFanTemperature ) {
    fanSwitch(true);
  }
  if ( fanAutomatic && fanIsRunning && currentTemperature <= turnOffFanTemperature ) {
    fanSwitch(false);
  }

}
//Convert device id to String
String GetAddressToString(DeviceAddress deviceAddress){
  String str = "";
  for (uint8_t i = 0; i < 8; i++){
    if( deviceAddress[i] < 16 ) str += String(0, HEX);
    str += String(deviceAddress[i], HEX);
  }
  return str;
}

float checkTemperature() {
  
  Serial.print("Parasite power is: "); 
  if( DS18B20.isParasitePowerMode() ){ 
    Serial.println("ON");
  }else{
    Serial.println("OFF");
  }
  
    // Loop through each device, print out address
    int numberOfDevices = DS18B20.getDeviceCount();
    Serial.print( "Device count: " );
    Serial.println( numberOfDevices );
    DS18B20.requestTemperatures();

    for(int i=0;i<numberOfDevices; i++){
      // Search the wire for address
      if( DS18B20.getAddress(devAddr[i], i) ){
        //devAddr[i] = tempDeviceAddress;
        Serial.print("Found device ");
        Serial.print(i, DEC);
        Serial.print(" with address: " + GetAddressToString(devAddr[i]));
        Serial.println();
      } else {
        Serial.print("Found ghost device at ");
        Serial.print(i, DEC);
        Serial.print(" but could not detect address. Check power and cabling");
      }
  
      //Get resolution of DS18b20
      Serial.print("Resolution: ");
      Serial.print(DS18B20.getResolution( devAddr[i] ));
      Serial.println();
  
      //Read temperature from DS18b20
      float tempC = DS18B20.getTempC( devAddr[i] );
      Serial.print("Temp C: ");
      Serial.println(tempC); 
      if (i == 0) {
        return tempC;
      }
      delay(100);
    }
    return 0;
  
  }


void checkSensors() {

 
  
  if (  ( retries < maxRetries && millis() - lastRetry >= 3000UL ) ||
        millis() - lastTemperatureSent >= temperature_interval * 1000UL ) {

    float temperature = 0;
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
    //temperature = bme.readTemperature();
    temperature =  checkTemperature();
    Homie.getLogger() << "Temperature reading " << temperature << endl;
    if ( isnan(temperature) || temperature == 0 ) {
      retries++;
      Homie.getLogger() << "*** Temperature not available retrying " << retries << ". time " << " max Retries = " << lastRetry << "... " << millis() << endl;
      lastRetry = millis();
      temperatureNode.setProperty("sensor_ok").send("0");
    }
    else {

      Homie.getLogger() << "Temperature: " << temperature << " °C" << endl;
      temperatureNode.setProperty("degrees").send(String(temperature));

      temperatureNode.setProperty("sensor_ok").send("1");

      lastTemperatureSent = millis();
      //lastRetry = millis();
      retries = maxRetries + 1;
      
      checkMeasurement(temperature);
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
