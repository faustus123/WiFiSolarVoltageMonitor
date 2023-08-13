//=======================================================================
// ESP8266_VoltageSensorWebServer
//
// This script provides a minimal web server for monitoring 4 voltages
// read by an ADS1115 analog to digital converter. It uses WiFiManager
// to allow the device to set up its own Access Point that can be used
// to configure it to connect to the local WiFi.
//
// This assumes a wiring configuration where A0 uses a resistor chain
// to scale down the voltage so the battery voltage may be measured.
// GND is on the battery negative.
// A1 is on the other side of a shunt from GND connecting to inverter.
// A2 and A3 are on either side of shunt connecting to GND of solar
// charger. 
//
// This includes a calibration feature where multiple data points may
// be gathered and then automatically fit to a quadratic function via
// regression method. The calibration is stored in non-volatile memory
// so persists through power cycles. Only A0 uses the calibration.
//
// This firmware includes an OTA updater (port 8266) that can be used 
// to update the firmware over WiFi.
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
#include <EEPROM.h>
#include <ArduinoOTA.h>


// These are used to allow user to press button to manually
// enter WiFi configuration mode and set up an AP.
// n.b. The manual switch was never fully implemented!
#define WIFI_CONFIG_PIN D4
#define WIFI_CONFIG_LED_PIN D3
int timeout = 120; // seconds to run for
const char *APNAME = "DLVoltageMonitor";

// This specifies the number of digits to use when converting to
// Strings during calibration
int PRECISION = 5;

// Used to size arrays for holding calibration values
const int MAX_CALIB_VALUES = 32;

// Calibration constants. These are automatically ovwritten if
// constants are found in the EEPROM.
bool CALIB_ENABLED[4] = {true, false, false, false};
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
ADS1115 ADS(0x48);

unsigned long html_page_requests = 0;

void ReadCalibration( );

struct SingleReading{
  float vA0;
  float vA1;
  float vA2;
  float vA3;

  float v2_3;
  float I2_3;
  float I1_GND;
};


//----------------------------------------------
// setup
//----------------------------------------------
void setup()
{
  // Start serial monitor
  Serial.begin(74880); // the ESP8266 boot loader uses this. We can set it to something else here, but the first few messages will be garbled.
  Serial.println();

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
  // program to updated via WiFi without having to connect via USB.
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
  ADS.begin(D2, D1);
  if( ADS.isConnected() ) {
    Serial.println("Connected to ADS1115 ADC");
    ADS.setGain(2); // set max voltage of +/- 2.048V. With ~11 reduction factor, this allows us to measure up to ~+/-22V
                    // n.b. gain=0 +/- 6.144V   gain=1 +/-4.096V   gain=2 +/- 2.048V  gain=4  +/-1.024V   gain=8  +/-0.512V  gain=16 +/-0.256V
  }else{
    Serial.println("ERROR Connecting to ADS1115 ADC!!!");
  }

  // Initialize EEPROM for use (this is only for ESP8266 and not all arduinos!)
  EEPROM.begin(EEPROM_SIZE);
  ReadCalibration();
  
  server.begin();
  Serial.printf("Web server started, open %s in a web browser\n", WiFi.localIP().toString().c_str());
}

//----------------------------------------------
// ReadAllValues
//
// This will read all each of the 4 ADC channels
// 10 times and average the readings. It will also
// do the same for the voltage difference between
// A2 and A3.
//----------------------------------------------
void ReadAllValues(float* values)
{
  // values[0] to values[3] are channels 0-4
  // values[4] is the difference between A2 and A3
 
  // If we somehow were unable to connect to the ADS1115
  // then just return all values of -99.
  if( !ADS.isConnected() ){
    for(int i=0; i<5; i++) values[i] = -99.0;
    return;
  }

  // Average 10 readings for each channel.
  // n.b. inner/outer loop order is on purpose to try and get
  // better average rather than re-use same reading multiple times.
  int Nreads = 10;
  float v_sum[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
  for( int j=0; j<Nreads; j++ ){
    for(int i=0; i<4; i++){
      int16_t adc = ADS.readADC(i);
      float v = ADS.toVoltage( adc );
      v_sum[i] += v;
    }

    int16_t adc = ADS.readADC_Differential_2_3();
    v_sum[4] += ADS.toVoltage( adc );
 }

  // Divide by number of reads to get average
  for(int i=0; i<5; i++){
     values[i] = v_sum[i] / (float)Nreads;
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
// Convert from measured values into actual values.
//
// Channel A0 is scaled by a resistor chain so needs
// calibration constants applied to scale up to voltage.
//
// Channels A1-A3 are just copied since they trust the
// ADS1115 internal circuitry.
//
// Upon entry values[4] will contain the voltage difference
// between A2 and A3 as read by a special call of the ADS115
// library. These should be reading across a shunt resistor
// The shunt parameters are used to calculate Amps which is
// written into calculated_values[4].
//
// Another shunt resistor is between A1 and GND so those
// parameters are used to convert A1 into Amps which is
// written into calculated_values[5].
// 
// Upon entry "values" should contain the uncalibrated
// values obtained from a call to ReadAllValues().
// If a non-NULL pointer is passed for "calibrated_values"
// then the calibrated values are written there.
// If no value (or NULL) is passed for "calibrated_values"
// the the contents of "values" will be overwritten with
// the calibrated values.
//
// The values in A[], B[], C[] will have already been
// read from EEPROM via ReadCalibration().
//
// NOTE: If calibrated_values is not NULL, it should point to 
// an array of at least 6 elements. If it is NULL, then values
// should point to an array of at least 6 elements!
//----------------------------------------------
void ApplyCalibration(float* values, float *calibrated_values=NULL)
{
  if( calibrated_values==NULL ) calibrated_values = values;

  // Voltages
  for(int i=0; i<4; i++){
    float v = values[i];
    if( CALIB_ENABLED[i] ){
      calibrated_values[i] = C[i] + B[i]*v + A[i]*v*v;
    }else{
      calibrated_values[i] = v;
    }
  }

  // Currents
  float I2_3 = values[4] * 100.0/0.075; // shunt resistor is 100A 75mV. A2 measures battery side of shunt while A3 measures solar charger side.
  float I1 = calibrated_values[1] * 200.0/0.075; // shunt resistor is 200A 75mV. A1 measures inverter side GND while ADS1115 device GND is battery side of shunt

  calibrated_values[4] = I2_3;
  calibrated_values[5] = I1;
}

//----------------------------------------------
// StatusJSON
//----------------------------------------------
String StatusJSON(void)
{
  float values[5]; // 5th value is difference between A2 and A3
  ReadAllValues(values);

  float calibrated_values[6];
  ApplyCalibration(values, calibrated_values);

  String formattedTime = timeClient.getFormattedTime();
  time_t epochTime = timeClient.getEpochTime(); // n.b. this is UTC time and the zone we want to report in the JSON record
  struct tm *ptm = gmtime ((time_t *)&epochTime); 
  int monthDay = ptm->tm_mday;
  int currentMonth = ptm->tm_mon+1;
  int currentYear = ptm->tm_year+1900;
  String currentDate = String(currentYear) + "-" + String(currentMonth) + "-" + String(monthDay) + " " + formattedTime;

  String json;
  json += F("{\n");
  for(int i=0; i<4; i++){
     json += F("\"V") + String(i) +  F("\" : \"") + String(calibrated_values[i], PRECISION) +  F("\",\n");
  }

  float I2_3 = calibrated_values[4];
  float I1 = calibrated_values[5];
  json += F("\"I1\" : \"") + String(I1, PRECISION) + F("\",\n");
  json += F("\"I2_3\" : \"") + String(I2_3, PRECISION) + F("\",\n");

  float P1 = I1*calibrated_values[0]; // Watts going TO inverter
  float P2_3 =  I2_3*calibrated_values[0]; // Watts coming FROM solar charger
  json += F("\"P1\" : \"") + String(P1, PRECISION) + F("\",\n");
  json += F("\"P2_3\" : \"") + String(P2_3, PRECISION) + F("\",\n");
  
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
            "This device uses an ADS1115 quad-channel "
            "analog to digital converter to measure voltages and make them available on-demand "
            "via WiFi. The objective is to allow remote monitoring of a small solar power "
            "project. The 4 channels can be connected to either side of a pair of shunt resistors "
            "to monitor two different currents. This setup assumes channels A0 and A1 monitor one shunt while "
            "channels A2 and A3 monitor another. The shunts are assumed to be 50A 75mV."
            "</p></center>\r\n");
            
  html += F("<center><p style=\"width:900px;\">"
            "The ADS1115 device is limited to measuring 3.3V max so a pair of resistors is used "
            "for each channel to step it down by about a factor of 11. This means this device is "
            "able to measure voltages from 0 up to about 36V. The precision seems to be about "
            "1mV. The resistors for each channel will have small differences so calibration is "
            "necessary. There is a calibration feature that allows data to "
            "be entered and appropriate calibration constants derived which are then stored in "
            "non-volatile memory. Please see the calibration page for more details. "
             "</p></center>\r\n");

  return html;
}

//----------------------------------------------
// MonitorPageHTML
//----------------------------------------------
String MonitorPageHTML(int &Npar, String *keys, String *vals)
{
  float values[5];
  ReadAllValues(values);

  float calibrated_values[6];
  ApplyCalibration(values, calibrated_values);

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

  html += F("<table border=\"1\" bgcolor=\"black\">");
  html += F("<tr style=\"background-color:silver\"><th></th><th>Vdiff</th><th>current</th><th>power</th>\r\n");

  float I2_3 = calibrated_values[4];
  float I1 = calibrated_values[5];
  float P2_3 = I2_3*calibrated_values[0];
  float P1 = I1*calibrated_values[0];
  html += "<tr><td style=\"color:#5F5\">FROM Solar</td>\r\n";
  html += "<td style=\"color:orange\">" + String(values[4]*1000.0, 1) + "mV</td>\r\n";
  html += "<td style=\"color:orange\">" + String(I2_3, 2) + "A</td>\r\n";
  html += "<td style=\"color:orange\">"+ String(P2_3, 2) + "W</td></tr>\r\n";

  html += "<tr><td style=\"color:#5F5\">TO Inverter</td>\r\n";
  html += "<td style=\"color:orange\">" + String(calibrated_values[1]*1000.0, 1) + "mV</td>\r\n";
  html += "<td style=\"color:orange\">" + String(I1, 2) + "A</td>\r\n";
  html += "<td style=\"color:orange\">"+ String(P1, 2) + "W</td></tr>\r\n";

  html += F("</table>\r\n");

  html += F("<p>Time read: ");
  html += currentDate;
  html += F("</p><hr>\r\n");

  // Calibration constants
  html += F("Calibration constants:<br>\r\n");
  for( int i=0; i<4; i++ ){
    html += F("V") + String(i) + F(": ") + String(A[i], PRECISION) + F("*v^2 + ") + String(B[i], PRECISION) + F("*v + ") + String(C[i], PRECISION) + F("<br>\r\n");
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
      float values[5];
      ReadAllValues(new_values);
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
            "a small voltage (<2V), some in the middle (~12V) and a large one (~20V). It is observed that having"
            "the long lever arm of small and large voltages gives calibrations with much more consistent readings."
            "At each voltage setting, enter the actual voltage "
            "and  hit \"enter\" or click the \"add point\" button. The more points, the better. "
            "If you enter a wrong voltage, you'll need to hit the \"clear\" button and start over. "
            "Once you have entered several points, hit the \"finish\" button to fit the data and "
            "store the calibration. You can add more points after clicking \"finish\" as long as "
            "the window is still up. Just hit \"finish\" again when you want to save the calibration."
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
            "<center><p style=\"width:800px;\">This device will measure the 4 voltages connected to the solar power "
            "system at my house and make them available via WiFi. They are connected "
            "to either end of 2 shunt resistors so the current can be obtained as well. "
            "The device uses some non-precise resistors so a calibration feature has been "
            "added (click the calibration link below)</p></center>\r\n");

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

    if(page_type.startsWith("monitor.html"  )) htmlPage += MonitorPageHTML(Npar, keys, vals);
    else if(page_type.startsWith("calibrate.html")) htmlPage += CalibratePageHTML(Npar, keys, vals);
    else htmlPage += HomePageHTML(Npar, keys, vals);

    htmlPage += F("<hr><i>This code maintained at: <A href=\"https://github.com/faustus123/WiFiSolarVoltageMonitor\">https://github.com/faustus123/WiFiSolarVoltageMonitor</A></i>\r\n");
    htmlPage += F("</html>\r\n");
  }

  return htmlPage;
}

//----------------------------------------------
// loop
//----------------------------------------------
void loop()
{
  ArduinoOTA.handle(); // allows firmware upgrade over WiFi
  timeClient.update();
  
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
