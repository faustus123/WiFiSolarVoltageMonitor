//=======================================================================
// ESP8266_VoltageSensorWebServer
//
// This sketch is the firmware on the HL WiFiSolarVoltageMonitor.
// The device consists of 2 ADS1115 ADC units and one bme280
// Temperature, humidity, and atmospheric pressure device. The
// First ADS1115 is used to read two currents via shunt resistors
// and the second is used to read 4 absolute voltages up to ~22V.
//
// This provides a minimal web server for monitoring the sensors via
// web browser. When initially powered, it will try and connect to
// WiFi network using saved credentials. If that fails, it will create
// its own access point called "DLVoltageMonitor" that can be used 
// to enter the credentials of the WiFi system which it will then save.
//
// Once connected to the local WiFi network, the web server can be used
// for both monitoring the sensors via web browser (using standard port
// 80) and downloading the latest values via JSON file.
//
// This assumes a wiring configuration where "A0-A1" read the voltage
// across a 100A 75mV shunt resistor (connect this to solar charger).
// The "A2-A3" are assumed connected across a 200A 75mV shunt resistor
// (connect this to inverter).
//
// The individual voltage sensors are labeled A0, A1, A2, A3 and are each
// scaled down by about a factor of ~11 using a resistor chain. Because the
// resistors have limited precision, a calibration feature is included
// to allow each of them to be calibrated. The calibration constants
// are stored in NV memory so it only needs to be done once. This is done
// via web browser so see that page for more details.
//
// This firmware includes an OTA (Over-The_Air) updater (port 8266) that
// can be used to update the firmware over WiFi.
//
// This code is maintained at:
//
//  https://github.com/faustus123/WiFiSolarVoltage
//=======================================================================



#include <WiFiManager.h>
#include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ADS1X15.h>   // https://github.com/RobTillaart/ADS1X15
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <DHT.h>
#include <EEPROM.h>
#include <ArduinoOTA.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <map>

// Name of Access Point (AP) to create when unable to connect to local WiFi
const char *APNAME = "DLVoltageMonitor";

// This specifies the number of digits to use when converting to
// Strings during calibration
int PRECISION = 5;

// Used to size arrays for holding calibration values
const int MAX_CALIB_VALUES = 32;

// Map channel names to alternatives for JSON records
// (this is filled at end of setup())
std::map<String, String> NAMES_MAP;

// Calibration constants. These are automatically ovwritten if
// constants are found in the EEPROM.
bool CALIB_ENABLED[4] = {true, true, true, true};
float A[4] = {0.0, 0.0, 0.0, 0.0};
float B[4] = {11.0, 11.0, 11.0, 11.0};
float C[4] = {0.0, 0.0 ,0.0, 0.0};

// Magic word used to make sure EEPROM has calibration constants in it.
unsigned int MAGIC = 0xDAD10757;
#define EEPROM_SIZE 512

// Setup our webserver
WiFiServer server(80);

// For getting and keeping current time (see https://github.com/arduino-libraries/NTPClient)
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60*60*1000); // 0 is for adjusting timezone (0=UTC). Only update once/hr)

// ADC (4 channel)
ADS1115 ADSI(0x49);  // Used for monitoring current
ADS1115 ADSV(0x48);  // Used for monitoring voltage

// E280 Temp, and Pressure sensor
Adafruit_BME280 bme280;
bool bme280active = false;

// DHT11 Temp, humidity sensor (n.b. when looking at sensor with pins point down, wire left to right as signal, Vcc, GND)
#define DHTPIN 0   // GPIO2 = D4  GPIO0 = D3
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
bool dht11active = false;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool oledactive = false;

// Button state
#define BUTTON_PIN 12
unsigned long last_button_dn_time = millis();
unsigned long last_button_up_time = millis();
int last_button_state = HIGH;

unsigned long html_page_requests = 0;

void ReadCalibration( );

struct SingleReading{
  float vA0_A1;
  float vA2_A3;
  float IA0_A1;
  float IA2_A3;
  
  float vA0;
  float vA1;
  float vA2;
  float vA3;

  float temp;
  float humidity;
  float atm_pressure;

  float dht_temp;
  float dht_humidity;
};


//----------------------------------------------
// setup
//----------------------------------------------
void setup()
{
  // Start serial monitor
  Serial.begin(74880); // the ESP8266 boot loader uses this. We can set it to something else here, but the first few messages will be garbled.
  Serial.println();

  // Set GPIO12=D6 to input (this is the button)
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Lower I2C clock speed for more reliability
  // Wire.setClock(10000);

   // Initialize OLED 128x64 pixel display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
  }else{
    Serial.println(F("SSD1306 found"));
    oledactive = true;
    display.display();
  }

  // Connect to WiFi
  // This will use cached credentials to try and connect. If unsuccessful,
  // it will automatically setup an AP where the credentials can be set
  // via web browser.
  WiFiManager wm;
  bool res = wm.autoConnect(APNAME); 

  // If we failed to connect to WiFi, restart the device so we can try again.
  if(!res) {
    Serial.println("Failed to connect");
    ESP.restart();
  }

  // If we get here, we're connected to WiFi
  Serial.println(" connected");

  // Connect to ntp Time Server
  Serial.printf("Connecting to ntp server to get current time ...\n");
  timeClient.begin();
  timeClient.update(); // n.b. this does not block and is not required to succeed

  //...............................................................................
  // Setup to allow Over The Air (OTA) updates to the firmware. This allows the
  // program to be updated via WiFi without having to connect via USB.
  // NOTE: To use this, select the approriate IP address for the serial port in
  // the Arduino IDE. This will allow uploading the new firmware but
  // IT WILL NOT ALLOW SERIAL MONITORING OVER THE WiFi!!!

  // Set password for firmware updates
  ArduinoOTA.setPassword("314159");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  //...............................................................................

  // Check communication with ADS1115 ADC
  ADSI.begin(D2, D1);
  if( ADSI.isConnected() ) {
    Serial.println("Connected to ADS1115 ADC (0x49)");
    ADSI.setGain(8); // set max voltage of +/- 512mV. In principle, the voltage difference should not go above ~75mV
                    // n.b. gain=0 +/- 6.144V   gain=1 +/-4.096V   gain=2 +/- 2.048V  gain=4  +/-1.024V   gain=8  +/-0.512V  gain=16 +/-0.256V
  }else{
    Serial.println("ERROR Connecting to ADS1115 ADC (0x49)!!!");
  }

  ADSV.begin(D2, D1);
  if( ADSV.isConnected() ) {
    Serial.println("Connected to ADS1115 ADC (0x48)");
    ADSV.setGain(2); // set max voltage of +/- 2.048V. With ~11 reduction factor, this allows us to measure up to ~+/-22V
                    // n.b. gain=0 +/- 6.144V   gain=1 +/-4.096V   gain=2 +/- 2.048V  gain=4  +/-1.024V   gain=8  +/-0.512V  gain=16 +/-0.256V
  }else{
    Serial.println("ERROR Connecting to ADS1115 ADC (0x48)!!!");
  }

  // BME280 Temp, pressure sensor
  bme280active = bme280.begin(0x76);
  if( !bme280active ){
    Serial.println("ERROR Connecting to bme280 (0x76)!!!");
    Serial.println(F("Could not find a valid BME280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bme280.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    // while (1) delay(10);
  }

  // DHT11 Temp, humidity sensor
  //dht.begin();
  //dht11active = true;

  // Initialize EEPROM for use (this is only for ESP8266 and not all arduinos!)
  EEPROM.begin(EEPROM_SIZE);
  ReadCalibration();

  // Fill names map so values can be translated into something more useful for JSON records
  NAMES_MAP["vA0"] = "v_battery"; 
  NAMES_MAP["vA1"] = "v_GND_battery"; 
  NAMES_MAP["vA2"] = "v_GND_solar"; 
  NAMES_MAP["vA3"] = "v_inverter"; 
  NAMES_MAP["IA0_A1"] = "i_solar"; 
  NAMES_MAP["IA2_A3"] = "i_inverter"; 
  NAMES_MAP["PA0_A1"] = "p_solar"; 
  NAMES_MAP["PA2_A3"] = "p_inverter"; 
  
  server.begin();
  Serial.printf("Web server started, open %s in a web browser\n", WiFi.localIP().toString().c_str());
}

//----------------------------------------------
// ReadAllValues
//
// This will read all of the sensors and write
// the values into the provided "vals" structure.
// Any sensors that are unavailable will have a
// value of -99.
//----------------------------------------------
void ReadAllValues(SingleReading* vals)
{
  // Initialize values to defaults in case any can't be read
  vals->vA0_A1 = vals->vA2_A3 = -99.0;  
  vals->IA0_A1 = vals->IA2_A3 = -99.0;  
  vals->vA0 = vals->vA1 = vals->vA2 = vals->vA3 = -99.0;
  vals->temp = vals->humidity = vals->atm_pressure = -99.0;
  vals->dht_temp = vals->dht_humidity = -99.0;

  // Read ADC's multiple times and average values
  int Nreads = 10;

  // Read voltage differences
  if( ADSI.isConnected() ){
    vals->vA0_A1 = vals->vA2_A3 = 0.0;
    for(int i=0; i<Nreads; i++){
      int16_t adc01 = ADSI.readADC_Differential_0_1();
      int16_t adc23 = ADSI.readADC_Differential_2_3();
      vals->vA0_A1 += ADSI.toVoltage( adc01 ) / float(Nreads);
      vals->vA2_A3 += ADSI.toVoltage( adc23 ) / float(Nreads);
    }
  }

  // Read voltages
  if( ADSV.isConnected() ){
    vals->vA0 = vals->vA1 = vals->vA2 = vals->vA3 = 0.0;
    for(int i=0; i<Nreads; i++){
      int16_t adc0 = ADSV.readADC(0);
      int16_t adc1 = ADSV.readADC(1);
      int16_t adc2 = ADSV.readADC(2);
      int16_t adc3 = ADSV.readADC(3);
      vals->vA0 += ADSV.toVoltage( adc0 ) / float(Nreads);
      vals->vA1 += ADSV.toVoltage( adc1 ) / float(Nreads);
      vals->vA2 += ADSV.toVoltage( adc2 ) / float(Nreads);
      vals->vA3 += ADSV.toVoltage( adc3 ) / float(Nreads);
    }
  }

  // Read environment
  if( bme280active ){
    vals->temp = 9.0/5.0*bme280.readTemperature() + 32.0; // convert to F
    vals->humidity = bme280.readHumidity();               // %
    vals->atm_pressure = bme280.readPressure()/1000.0;    // convert to kPa
  }

  if( dht11active ){
    vals->dht_temp = dht.readTemperature(true); // true= Farenheit
    vals->dht_humidity = dht.readHumidity();
    // Serial.print("vals->dht_temp: "); Serial.println(vals->dht_temp);
    if(isnan(vals->dht_temp    )) vals->dht_temp     = -99.0;
    if(isnan(vals->dht_humidity)) vals->dht_humidity = -99.0;
  }

}

//----------------------------------------------
// FitParabola
//
// Fit a parabola to a given set of points.
//
// This code copied from the Rivello post here:
// https://forum.arduino.cc/t/solved-fitting-a-quadratic-curve-with-arduino-possible/433104/5
//----------------------------------------------
void FitParabola( int N_points, float px[], float py[], float &a, float &b, float &c ) {

  int i;
  float S00, S10, S20, S30, S40, S01, S11, S21;
  float denom, x, y;
  S00=S10=S20=S30=S40=S01=S11=S21=0;
  
  for (i=0; i<N_points; i++) {  
    x = px[i];
    y = py[i];
    //S00 += 1; // x^0+y^0
    S10 += x;
    S20 += x * x;
    S30 += x * x * x;
    S40 += x * x * x * x;
    S01 += y;
    S11 += x * y;
    S21 += x * x * y;
  }
  S00 = N_points;
  
  denom =   S00*(S20*S40 - S30*S30) - S10*(S10*S40 - S20*S30) + S20*(S10*S30 - S20*S20);

  c = (S01*(S20*S40-S30*S30)-S11*(S10*S40-S20*S30)+S21*(S10*S30-S20*S20))/denom;
  b = (S00*(S11*S40-S30*S21)-S10*(S01*S40-S21*S20)+S20*(S01*S30-S11*S20))/denom;
  a = (  S00*(S20*S21 - S11*S30) - S10*(S10*S21 - S01*S30) + S20*(S10*S11 - S01*S20) )/denom;
}

//----------------------------------------------
// SaveCalibration
//
//----------------------------------------------
String SaveCalibration( int Nvals, float *volts, float adc[4][MAX_CALIB_VALUES] )
{
  String html;

  html += F("<center>Calibration values calculated from values in table below.</center>\r\n");

  float a[4], b[4], c[4];
  for( int i=0; i<4; i++ ){
    FitParabola( Nvals, adc[i], volts, a[i], b[i], c[i]);
    html += F("chan") + String(i) + F(": ") + String(a[i],PRECISION) + F("*x^2  +  ") + String(b[i],PRECISION) + F("*x  +  ") + String(c[i],PRECISION) + F("<br>\r\n");
    Serial.printf("chan%d : %f*x^2  +  %f*x  +  %f\n", i, a[i], b[i], c[i]);
  }

  // Write all parameters to EEPROM
  Serial.println( "Writing calibration constants to EEPROM ..." );
  int addr = 0;
  EEPROM.put(addr, MAGIC);  addr += sizeof(MAGIC); // First element should be the magic word
  for( int i=0; i<4; i++ ){
    EEPROM.put(addr, a[i]);  addr += sizeof(a[i]);
    EEPROM.put(addr, b[i]);  addr += sizeof(b[i]);
    EEPROM.put(addr, c[i]);  addr += sizeof(c[i]);
  }
  if (EEPROM.commit()) {
    Serial.println("EEPROM successfully committed");
  } else {
    Serial.println("ERROR! EEPROM commit failed");
  }
//  EEPROM.commit(); // commit to EEPROM (again, this is only for ESP8266)
  Serial.println( "Done." );

  // Now, read them back in so they can be used right away
  ReadCalibration();
 
  return html;
}

//----------------------------------------------
// ReadCalibration
//
//----------------------------------------------
void ReadCalibration( )
{

  // WARNING: Bypassing the 2nd order calibration values for now and
  // using measured resistor values
  double R1[4] = {0.1006, 0.1006, 0.1009, 0.1014};
  double R2[4] = { 1.002,  1.000,  1.002,  1.006};
  for( int i=0; i<4; i++ ){
    B[i] = (R2[i] + R1[i])/R1[i];
  }
  Serial.println( "Hardcoded calibration used." );
  return;


  // Read all parameters from EEPROM
  Serial.println( "Reading calibration constants from EEPROM ..." );
  int addr=0;
  unsigned int magic;
  EEPROM.get(addr, magic);  addr += sizeof(MAGIC); // First element should be the magic word
  if( magic != MAGIC ){
    Serial.printf("Error in magic word for calibrations (0x%x != 0x%x). Likely no calibration written yet.\n", magic, MAGIC);
    return;
  }
  for( int i=0; i<4; i++ ){
    EEPROM.get(addr, A[i]);  addr += sizeof(A[i]);
    EEPROM.get(addr, B[i]);  addr += sizeof(B[i]);
    EEPROM.get(addr, C[i]);  addr += sizeof(C[i]);

    Serial.printf("chan%d : %f*x^2  +  %f*x  +  %f\n", i, A[i], B[i], C[i]);
  }
  Serial.println( "Done." );
}


//----------------------------------------------
// ApplyCalibration
//
// Convert measured values from SingleReading into 
// calibrated values using current calibration.
// 
// Upon entry "vals" should contain the uncalibrated
// values obtained from a call to ReadAllValues().
//
// The calibration constants (e.g.) A[], B[], C[])
// will have already been read from EEPROM via
// ReadCalibration().
//
//----------------------------------------------
void ApplyCalibration(SingleReading* vals)
{
  // Copy values into array
  float values[4] = {vals->vA0, vals->vA1, vals->vA2, vals->vA3};

  // Loop over voltages, calibrating or copying
  for(int i=0; i<4; i++){
    float v = values[i];
    if( CALIB_ENABLED[i] ){
      values[i] = C[i] + B[i]*v + A[i]*v*v;
    }
  }

  // Copy calibrated values back into "vals"
  if(vals->vA0 != -99.0) vals->vA0 = values[0];
  if(vals->vA1 != -99.0) vals->vA1 = values[1];
  if(vals->vA2 != -99.0) vals->vA2 = values[2];
  if(vals->vA3 != -99.0) vals->vA3 = values[3];

  // Calculate currents from measured voltage differentials
  if(vals->vA0_A1 != -99.0) vals->IA0_A1 = -vals->vA0_A1 * 100.0/0.075; // 100A/75mV shunt
  if(vals->vA2_A3 != -99.0) vals->IA2_A3 = -vals->vA2_A3 * 200.0/0.075; // 200A/75mV shunt
}

//----------------------------------------------
// StatusJSON
//----------------------------------------------
String StatusJSON(void)
{
  SingleReading values; // 5th value is difference between A2 and A3
  ReadAllValues(&values);
  ApplyCalibration(&values);

  String formattedTime = timeClient.getFormattedTime();
  time_t epochTime = timeClient.getEpochTime(); // n.b. this is UTC time and the zone we want to report in the JSON record
  struct tm *ptm = gmtime ((time_t *)&epochTime); 
  int monthDay = ptm->tm_mday;
  int currentMonth = ptm->tm_mon+1;
  int currentYear = ptm->tm_year+1900;
  String currentDate = String(currentYear) + "-" + String(currentMonth) + "-" + String(monthDay) + " " + formattedTime;

  String json;
  json += F("{\n");
  json += "\"" + NAMES_MAP["vA0"] + "\" : \"" + String(values.vA0, PRECISION) + "\",\n";
  json += "\"" + NAMES_MAP["vA1"] + "\" : \"" + String(values.vA1, PRECISION) + "\",\n";
  json += "\"" + NAMES_MAP["vA2"] + "\" : \"" + String(values.vA2, PRECISION) + "\",\n";
  json += "\"" + NAMES_MAP["vA3"] + "\" : \"" + String(values.vA3, PRECISION) + "\",\n";

  json += "\"" + NAMES_MAP["IA0_A1"] + "\" : \"" + String(values.IA0_A1, PRECISION) + "\",\n";
  json += "\"" + NAMES_MAP["IA2_A3"] + "\" : \"" + String(values.IA2_A3, PRECISION) + "\",\n";

  float PA0_A1 = values.IA0_A1 * values.vA0; // Watts coming FROM solar charger
  float PA2_A3 = values.IA2_A3 * values.vA0; // Watts going TO inverter
  if( values.IA0_A1==-99.0 || values.vA0==-99.0) PA0_A1 = -99.0;
  if( values.IA2_A3==-99.0 || values.vA0==-99.0) PA2_A3 = -99.0;
  json += "\"" + NAMES_MAP["PA0_A1"] + "\" : \"" + String(PA0_A1, PRECISION) + "\",\n";
  json += "\"" + NAMES_MAP["PA2_A3"] + "\" : \"" + String(PA2_A3, PRECISION) + "\",\n";

  if( values.temp         != -99.0 ) json += "\"temp\" : \""         + String(values.temp,         PRECISION) + "\",\n";
  if( values.humidity     != -99.0 ) json += "\"humidity\" : \""     + String(values.humidity,     PRECISION) + "\",\n";
  if( values.atm_pressure != -99.0 ) json += "\"atm_pressure\" : \"" + String(values.atm_pressure, PRECISION) + "\",\n";
  if( values.dht_temp     != -99.0 ) json += "\"dht_temp\" : \""     + String(values.dht_temp,     PRECISION) + "\",\n";
  if( values.dht_humidity != -99.0 ) json += "\"dht_humidity\" : \"" + String(values.dht_humidity, PRECISION) + "\",\n";

  json += F("\"date\" : \"") + currentDate + F("\"\n");
  json += F("}\n");

  // Need to send proper response header first for JSON
  String response = F("HTTP/1.1 200 OK\r\n"
                      "Content-Type: application/json\r\n"
                      "Content-Length: ") + String( json.length() ) + F("\r\n\r\n");
  response += json;

  return response;
}

//----------------------------------------------
// GetSubstring
//
// Given a string, return the substring between the given
// start and end string patterns. If the end pattern is 
// not found or is empty, then the end of the string is used.
// If the start pattern is empty, then the value given for
// index_start is used for the start of the string.
// A starting index for the search can optionally be given.
//----------------------------------------------
String GetSubstring( const String str, const String &str_start, const String &str_end, int index_start=0)
{
  int substring_start = index_start;
  if( str_start.length()>0 ) substring_start = str.indexOf(str_start, index_start);
  if( substring_start<0 ) return "";
  
  int substring_end = str.length();
  if( str_end.length()>0 ){
    int tmp = str.indexOf(str_end, substring_start+2);
    if( tmp>=0 ) substring_end = tmp;
  }
  return str.substring( substring_start+1, substring_end); 
}

//----------------------------------------------
// HomePageHTML
//----------------------------------------------
String HomePageHTML(int &Npar, String *keys, String *vals)
{
  String html;

  html += F("<center><h2>Overview</h2></center>\r\n");
  html += F("<center><p style=\"width:900px;\">"
            "This device uses two ADS1115 quad-channel analog to digital converters to measure "
            "voltages and currents and makes them available on-demand "
            "via WiFi. The objective is to allow remote monitoring of a small solar power "
            "project. Four of the channels are assumed to be connected to either side of a pair of "
            "shunt resistors (A0_A1 to a 100A/75mV shunt and A2_A3 to a 200A/75mV shunt). "
            "</p></center>\r\n");
            
  html += F("<center><p style=\"width:900px;\">"
            "The other 4 channels: A0, A1, A2, A3 "
            "can be used to measure individual voltages up to ~22V. "
            "The precision seems to be about "
            "1mV. The resistors for each channel will have small differences so calibration is "
            "necessary. There is a calibration feature that allows data to "
            "be entered and appropriate calibration constants derived which are then stored in "
            "non-volatile memory. Please see the calibration page for more details. "
             "</p></center>\r\n");

  html += F("<center><p style=\"width:900px;\">"
            "Proper setup connects the A0_A1 pair to the solar charger shunt while the A2_A3 "
            "pair are connected to the inverter shunt. The A0 channel should be connected to the "
            "positive battery terminal. The other channels (A1, A2, A3) are read, but not assigned "
            "specific functions. "
            "</p></center>\r\n");

  return html;
}

//----------------------------------------------
// MonitorPageHTML
//----------------------------------------------
String MonitorPageHTML(void)
{
  // Read values and get calibrated values in different struct so we can show both
  SingleReading vals;
  ReadAllValues(&vals);
  SingleReading calibrated_vals = vals;
  ApplyCalibration(&calibrated_vals);

  // Copy into arrays
  float values[4] = {vals.vA0, vals.vA1, vals.vA2, vals.vA3};
  float calibrated_values[4] = {calibrated_vals.vA0, calibrated_vals.vA1, calibrated_vals.vA2, calibrated_vals.vA3};

  time_t epochTime = timeClient.getEpochTime() - 4*60*60; // n.b. the -4*60*60 converts to EST time for web page display
  struct tm *ptm = gmtime ((time_t *)&epochTime); 
  int monthDay = ptm->tm_mday;
  int currentMonth = ptm->tm_mon+1;
  int currentYear = ptm->tm_year+1900;
  int currentHour = ptm->tm_hour;
  int currentMin = ptm->tm_min;
  int currentSec = ptm->tm_sec;
  String currentDate = String(currentYear) + "-" + String(currentMonth) + "-" + String(monthDay) + " " + String(currentHour) + ":" + String(currentMin) + ":" + String(currentSec);

  String html;
  html.reserve(1024);               // prevent ram fragmentation

  html += F("<p>Host: <A href=\"http:\\\\");
  html +=  WiFi.localIP().toString().c_str();
  html += F("\">http:\\\\");
  html +=  WiFi.localIP().toString().c_str();
  html += F("</A>\r\n");
  
  // Voltages
  html += F("<table border=\"1\" bgcolor=\"black\">");
  html += F("<tr style=\"background-color:gold\"><th>chan</th><th>raw</th><th>calibrated</th>\r\n");
  for( int i=0; i<4; i++ ){
    html += F("<tr><td style=\"color:#5F5\">A");
    html += i;
    html += F("</td><td style=\"color:yellow\">");
    html += String(values[i], 4);
    html += F("V</td>");
    html += F("</td><td style=\"color:yellow\">");
    html += String(calibrated_values[i], 4);
    html += F("V</td></tr>\r\n");
  }
  html += F("</table>\r\n");

  // Currents/Power
  html += F("<table border=\"1\" bgcolor=\"black\">");
  html += F("<tr style=\"background-color:silver\"><th></th><th>Vdiff</th><th>current</th><th>power</th>\r\n");

  float P0_1 = calibrated_vals.IA0_A1 * calibrated_vals.vA0;
  if( calibrated_vals.IA0_A1==-99.0 || calibrated_vals.vA0==-99.0 ) P0_1 = -99.0;
  html += "<tr><td style=\"color:#5F5\">FROM Solar</td>\r\n";
  html += "<td style=\"color:orange\">" + String(calibrated_vals.vA0_A1*1000.0, 1) + "mV</td>\r\n";
  html += "<td style=\"color:orange\">" + String(calibrated_vals.IA0_A1, 2) + "A</td>\r\n";
  html += "<td style=\"color:orange\">"+ String(P0_1, 2) + "W</td></tr>\r\n";

  float P2_3 = calibrated_vals.IA2_A3 * calibrated_vals.vA0;
  if( calibrated_vals.IA2_A3==-99.0 || calibrated_vals.vA0==-99.0 ) P2_3 = -99.0;
  html += "<tr><td style=\"color:#5F5\">TO Inverter</td>\r\n";
  html += "<td style=\"color:orange\">" + String(calibrated_vals.vA2_A3*1000.0, 1) + "mV</td>\r\n";
  html += "<td style=\"color:orange\">" + String(calibrated_vals.IA2_A3, 2) + "A</td>\r\n";
  html += "<td style=\"color:orange\">"+ String(P2_3, 2) + "W</td></tr>\r\n";

  html += F("</table>\r\n");

  // Environment
  html += F("<table border=\"1\" bgcolor=\"black\">");
  html += F("<tr style=\"background-color:aqua\"><th>what</th><th>value</th>\r\n");

  html += "<tr><td style=\"color:orange\">temperature(bm3280)</td>";
  html += "<td style=\"color:orange\">" + String(calibrated_vals.temp, 2) + " F</td></tr>\r\n";

  html += "<tr><td style=\"color:orange\">humidity(bm3280)</td>";
  html += "<td style=\"color:orange\">" + String(calibrated_vals.humidity, 2) + " %</td></tr>\r\n";

  html += "<tr><td style=\"color:orange\">atmospheric pressure(bm3280)</td>";
  html += "<td style=\"color:orange\">" + String(calibrated_vals.atm_pressure, 5) + " kPa</td></tr>\r\n";

  html += "<tr><td style=\"color:orange\">temperature(DHT11)</td>";
  html += "<td style=\"color:orange\">" + String(calibrated_vals.dht_temp, 2) + " F</td></tr>\r\n";

  html += "<tr><td style=\"color:orange\">humidity(DHT11)</td>";
  html += "<td style=\"color:orange\">" + String(calibrated_vals.dht_humidity, 2) + " %</td></tr>\r\n";

  html += F("</table>\r\n");

  // Time read
  html += F("<p>Time read: ");
  html += currentDate;
  html += F("</p><hr>\r\n");

  // Calibration constants
  html += F("Calibration constants:<br>\r\n");
  for( int i=0; i<4; i++ ){
    if( CALIB_ENABLED[i] ){
      html += F("V") + String(i) + F(": ") + String(A[i], PRECISION) + F("*v^2 + ") + String(B[i], PRECISION) + F("*v + ") + String(C[i], PRECISION) + F("<br>\r\n");
    }else{
      html += F("V") + String(i) + F(": v<br>\r\n");
    }
  }

  return html;
}

//----------------------------------------------
// CalibratePageHTML
//----------------------------------------------
String CalibratePageHTML(int &Npar, String *keys, String *vals)
{
  // First, extract list of previously captured values
  int Nvals = 0;
  float adc[4][MAX_CALIB_VALUES];
  float volts[MAX_CALIB_VALUES];
  float new_values[4];
  float new_voltage=0;
  for(int i=0; i<Npar; i++){
    if( keys[i].startsWith("volts") ){
      int idx = keys[i].substring(5).toInt();
      volts[idx] = vals[i].toFloat();
      if( idx>=Nvals ) Nvals = idx+1;  // This is really not a safe way to do this!
    }
    if( keys[i].startsWith("A") ){
      int ichan = keys[i].substring(1,2).toInt();
      int idx = keys[i].substring(3).toInt();
      adc[ichan][idx] = vals[i].toFloat();
      if( idx>=Nvals ) Nvals = idx+1;  // This is really not a safe way to do this!
    }
    if( keys[i] == "voltage" ){
      SingleReading values;
      ReadAllValues(&values);
      new_values[0] = values.vA0;
      new_values[1] = values.vA1;
      new_values[2] = values.vA2;
      new_values[3] = values.vA3;
      new_voltage = vals[i].toFloat();
    }
  }

  // If we were passed a "voltage" value from the HTML form.
  // just submitted then read the ADCs and add them to the list.
  if( new_voltage>0.0 ){
    for(int i=0; i<4; i++) adc[i][Nvals] = new_values[i];
    volts[Nvals] = new_voltage;
    Nvals++;
  }
  
  String html;
  html += F("<center><h2>Calibration Tool</h2></center>\r\n");
  html += F("<center><p style=\"width:900px;\">To calibrate, connect all four ADC probes to the same voltage source "
            "(and the ground wire to the ground). Set the voltage to several values but include "
            "a small voltage (<2V), some in the middle (~12V) and a large one (~20V). It is observed that having "
            "the long lever arm of small and large voltages gives calibrations with much more consistent readings. "
            "At each voltage setting, enter the actual voltage "
            "and  hit \"enter\" or click the \"add point\" button. The more points, the better (up to 32). "
            "If you enter a wrong voltage, you'll need to hit the \"clear\" button and start over. "
            "Once you have entered several points, hit the \"finish\" button to fit the data and "
            "store the calibration. You can add more points after clicking \"finish\" as long as "
            "the window is still up. Just hit \"finish\" again when you want to save the calibration. "
            "All of the data points shown on the bottom will be used. "
            "</p></center>\r\n");

  // Add form for inputing new voltage. Includes hidden values to
  // hold previous entries.
  html += F("<form method=\"get\">\r\n"
            "<center><table border=\"1\"><tr>\r\n"
            "<td><input type=\"entry\" name=\"voltage\"/></td>\r\n"
            "<td><input type=\"submit\" value=\"add point\" /></td>\r\n"
            "</tr></table></center>\r\n");

  // Add any previously gathered entries
  for( int i=0; i<Nvals; i++ ){
    html += F("<input type=\"hidden\" name=\"volts") + String(i) + F("\" value=\"") +  String(volts[i],PRECISION) + F("\">\r\n");
    for( int ichan=0; ichan<4; ichan++)  html += F("<input type=\"hidden\" name=\"A") + String(ichan) + F("_") + String(i) + F("\" value=\"") +  String(adc[ichan][i],PRECISION) + F("\">\r\n");
  }

  // Close form
  html += F("</form>\r\n");

  // Add clear and finished buttons
  html += F("<center><table><tr>\r\n");
  html += F("<td><form><input type=\"submit\" value=\"clear\"></form></td>\r\n");
  html += F("<td width=\"100px\"> </td>\r\n");
  html += F("<td><form><input type=\"submit\" value=\"finished\"><input type=\"hidden\" name=\"finished\" value=\"1\">\r\n");
  // Add any previously gathered entries
  for( int i=0; i<Nvals; i++ ){
    html += F("<input type=\"hidden\" name=\"volts") + String(i) + F("\" value=\"") +  String(volts[i],PRECISION) + F("\">\r\n");
    for( int ichan=0; ichan<4; ichan++)  html += F("<input type=\"hidden\" name=\"A") + String(ichan) + F("_") + String(i) + F("\" value=\"") +  String(adc[ichan][i],PRECISION) + F("\">\r\n");
  }
  html += F("</form></td>\r\n");
  html += F("</tr></table></center>\r\n");
  
  // Check if we got her by the "finished" button being clicked.
  bool finished = false;
  for(int i=0; i<Npar; i++) if( keys[i] == "finished" ) finished = true;
  if( finished ) html += SaveCalibration( Nvals, volts, adc );

  // Print current list of values
  html += F("<hr>Current values<br><table border=\"2\">\r\n");
  html += F("<th>VOLTS</th> <th>A0</th> <th>A1</th> <th>A2</th> <th>A3</th>");
  for( int i=0; i<Nvals; i++ ){
    html += F("<tr>\r\n");
    html += F("<td>") + String(volts[i],PRECISION) + F("</td>");
    for( int ichan=0; ichan<4; ichan++)  html += F("<td>") + String(adc[ichan][i],PRECISION) + F("</td>");
    html += F("</tr>\r\n");
  }
  html += F("</table>\r\n");

  return html;
}

//----------------------------------------------
// HeaderHTML
//
// This will return an HTML header string that
// includes the preamble and standard info and
// menu at the top. The optional "refresh_period"
// paramter can be used to add a line that will
// cause the page to reload after the specified
// number of seconds. This is useful for the 
// monitor page. Do not pass a value (or pass 0)
// to not have the page automatically refresh.
//----------------------------------------------
String HeaderHTML(int refresh_period=0)
{
  // preamble
  String html;
  html = F("HTTP/1.1 200 OK\r\n"
           "Content-Type: text/html\r\n"
           "Connection: close\r\n");  // the connection will be closed after completion of the response

  // Optionally refresh the page
  if(refresh_period>0){
    html += F("Refresh: ");
    html += refresh_period;
    html += F("\r\n");
  }
 
  // Start of HTML
  html += F("\r\n"
           "<!DOCTYPE HTML>"
           "<html>\r\n");

  // Title
  html += F("<title>Hill-Lawrence Solar Voltage Monitor</title>\r\n"
            "<center><H1>Hill-Lawrence Solar Voltage Monitor</H1></center>\r\n"
            "<center><p style=\"width:800px;\">This device will measure the 8 voltages connected to the solar power "
            "system at my house and make them available via WiFi. Four are connected "
            "to either end of 2 shunt resistors to measure the current while a 5th is connected to the battery. "
            "The device uses some non-precise resistors so a calibration feature has been "
            "added (click the calibration link below). Environmental data is also read via bme280 device. "
            "</p></center>\r\n");

  // Menu bar
  html += F("<center><table><tr>\r\n"
            "<td width=\"30%\"><A href=\"index.html\">Home</A></td>\r\n"
            "<td width=\"30%\"><A href=\"monitor.html\">Monitor</A></td>\r\n"
            "<td width=\"30%\"><A href=\"calibrate.html\">Calibrate</A></td>\r\n"
            "<td width=\"30%\"><A href=\"status.json\">status.json</A></td>\r\n"
            "</table></center>\r\n"
            "<hr>\r\n");

  return html;
}

//----------------------------------------------
// prepareHtmlPage
//
// prepare a web page to be send to a client (web browser)
//----------------------------------------------
String prepareHtmlPage(const String &line)
{
  // Parse the given line to determine what page to render.
  // This should be the "GET" line of the request which will
  // start with "GET /" and then possibly have a page name
  // and parameters. We could use a better library, but will
  // just parse the line here ourselves.

  // Get which page they are requesting
  String page_type = GetSubstring( line, "/", "?");

  // Parse parameters
  int Npar = 0;
  const int max_vals = 64;
  String keys[max_vals];
  String vals[max_vals];
  int idx = line.indexOf("?");
  while( idx>=0 && idx<line.length() ){
    String par = GetSubstring( line, "", "&", idx);
    if( par.length() == 0 ) break;
    int tmp_idx = par.lastIndexOf(" ");
    if( tmp_idx>0 ) par = par.substring(0, tmp_idx);
    idx += par.length()+1;
    int eq = par.indexOf('=');
    if( eq != -1 ){
      keys[Npar] = par.substring(0, eq);
      vals[Npar] = par.substring(eq+1);
      Npar++;
    }
    if( tmp_idx>0 ) break;  // Remainder must be just the "HTTP/1.1" at end of line
  }

  String htmlPage;
  htmlPage.reserve(1024);               // prevent ram fragmentation

  int refresh_period = 0;
  Serial.printf("page_type = \"%s\"\n", page_type.c_str());
  if(page_type.startsWith("monitor.html")){
    refresh_period = 2;
  }

  if(page_type.startsWith("status.json")){
    htmlPage += StatusJSON();
  }else{
    htmlPage += HeaderHTML(refresh_period);

    if(page_type.startsWith("monitor.html"  )) htmlPage += MonitorPageHTML();
    else if(page_type.startsWith("calibrate.html")) htmlPage += CalibratePageHTML(Npar, keys, vals);
    else htmlPage += HomePageHTML(Npar, keys, vals);

    htmlPage += F("<hr><i>This code maintained at: <A href=\"https://github.com/faustus123/WiFiSolarVoltageMonitor\">https://github.com/faustus123/WiFiSolarVoltageMonitor</A></i>\r\n");
    htmlPage += F("</html>\r\n");
  }

  return htmlPage;
}

//----------------------------------------------
// HandleButton
//
// Check if button was pressed or released and respond accordingly
//----------------------------------------------
void HandleButton(void)
{
  int button_state = digitalRead(BUTTON_PIN);
  // Serial.print("button state: "); Serial.println(button_state);
  if( button_state != last_button_state){
    if( button_state == HIGH ){
      last_button_up_time = millis();
    }else{
      last_button_dn_time = millis();
    }
    last_button_state = button_state;
  }
}

//----------------------------------------------
// HandleDisplay
//
// Update display
//----------------------------------------------
void HandleDisplay(void)
{
  if (! oledactive ) return;

  display.clearDisplay();
  unsigned int tdiff_up = millis() - last_button_up_time;
  bool draw_display = (last_button_state==LOW || tdiff_up<4000);


  // Serial.println("tdiff_up: " + String(tdiff_up) + "  clear_display: " + String(clear_display) );
 
  if( draw_display ){
    display.setTextColor(WHITE);
    display.setTextSize(0);

    int y = 0;
    display.setCursor(0,y);
    String ipstr = String("IP: ") + WiFi.localIP().toString();
    display.print(ipstr);

    SingleReading vals;
    ReadAllValues(&vals);
    ApplyCalibration(&vals);
    y += 10;
    display.setCursor(0,y);
    String vstr = String("   Battery: " + String(vals.vA0, 2) + String("V"));
    display.print(vstr);

    y += 10;
    display.setCursor(0,y);
    String istr = String("Iin/Iout: " + String(vals.IA0_A1, 1) + String("A/") + String(vals.IA2_A3, 1) + String("A"));
    display.print(istr);

    // y += 10;
    // display.setCursor(0,y);
    // String istr1 = String("   Isolar: " + String(vals.IA0_A1, 1) + String("A"));
    // display.print(istr1);

    // y += 10;
    // display.setCursor(0,y);
    // String istr2 = String(" Iinverter: " + String(vals.IA2_A3, 1) + String("A"));
    // display.print(istr2);

    float temp = vals.temp==-99.0 ? vals.dht_temp:vals.temp;
    float humidity = vals.humidity==-99.0 ? vals.dht_humidity:vals.humidity;

    if(temp != -99.0){
      y += 10;
      display.setCursor(0,y);
      String tstr = String("      Temp: " + String(temp, 1) + String("F"));
      display.print(tstr);
    }

    if(humidity != -99.0 ){
      y += 10;
      display.setCursor(0,y);
      String hstr = String("  Humidity: " + String(humidity, 1) + String("%"));
      display.print(hstr);
    }

    if( vals.atm_pressure != -99.0 ){
      y += 10;
      display.setCursor(0,y);
      String pstr = String("Pressure: " + String(vals.atm_pressure, 1) + String("kPa"));
      display.print(pstr);
    }
  }

  display.display();
}

//----------------------------------------------
// loop
//----------------------------------------------
void loop()
{
  ArduinoOTA.handle(); // allows firmware upgrade over WiFi
  timeClient.update();

  // Check for button press
  HandleButton();

  // Update display
  HandleDisplay();
  
  // Service Web Server
  WiFiClient client = server.accept();
  // wait for a client (web browser) to connect
  if (client)
  {
    Serial.println("\n[Client connected]");
    String last_get_line = "GET /";
    while (client.connected())
    {
      // read line by line what the client (web browser) is requesting
      if (client.available())
      {
        String line = client.readStringUntil('\r');
        if( line.startsWith("GET") ) last_get_line = line;
        Serial.print(line);
        // wait for end of client's request, that is marked with an empty line
        if (line.length() == 1 && line[0] == '\n')
        {
          client.println(prepareHtmlPage(last_get_line));
          break;
        }
      }
    }

    while (client.available()) {
      // but first, let client finish its request
      // that's diplomatic compliance to protocols
      // (and otherwise some clients may complain, like curl)
      // (that is an example, prefer using a proper webserver library)
      client.read();
    }

    // close the connection:
    client.stop();
    html_page_requests++;
    Serial.printf("[Client disconnected page requests:");
    Serial.print(html_page_requests);
    Serial.println("]");
  }
}
