

#include <WiFiManager.h>
#include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ADS1X15.h>
#include <Wire.h>
#include <EEPROM.h>

// These are used to allow user to press button to manually
// enter WiFi configuration mode and set up an AP.
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
NTPClient timeClient(ntpUDP, "pool.ntp.org", -4*60*60, 60*60*1000); // -4*60*60 is for adjusting to EST timezone. Only update once/hr)

// ADC (4 channel)
ADS1115 ADS(0x48);

unsigned long html_page_requests = 0;

void ReadCalibration( );


//----------------------------------------------
// setup
//----------------------------------------------
void setup()
{
  // Start serial monitor
  Serial.begin(115200);
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

  // Check communication with ADS1115 ADC
  ADS.begin(D2, D1);
  if( ADS.isConnected() ) {
    Serial.println("Connected to ADS1115 ADC");
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
// 10 times and average the readings.
//----------------------------------------------
void ReadAllValues(float* values)
{

  // If we somehow were unable to connect to the ADS1115
  // then just return all values of -99.
  if( !ADS.isConnected() ){
    for(int i=0; i<4; i++) values[i] = -99.0;
    return;
  }

  // Average 10 readings for each channel.
  // n.b. inner/outer loop order is on purpose to try and get
  // better average rather than re-use same reading multiple times.
  int Nreads = 10;
  float v_sum[4] = {0.0, 0.0, 0.0, 0.0};
  for( int j=0; j<Nreads; j++ ){
    for(int i=0; i<4; i++){
      int16_t adc = ADS.readADC(i);
      float v = ADS.toVoltage( adc );
      v_sum[i] += v;
    }
  }

  // Divide by number of reads to get average
  for(int i=0; i<4; i++){
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
// Apply calibration values to convert from the
// voltage measured by ADS1115 (which has been
// scaled by resistor chain) into actual voltage.
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
//----------------------------------------------
void ApplyCalibration(float* values, float *calibrated_values=NULL)
{
  if( calibrated_values==NULL ) calibrated_values = values;

  for(int i=0; i<4; i++){
    float v = values[i];
    calibrated_values[i] = C[i] + B[i]*v + A[i]*v*v;
  } 
}

//----------------------------------------------
// StatusJSON
//----------------------------------------------
String StatusJSON(void)
{
  float values[4];
  ReadAllValues(values);

  float calibrated_values[4];
  ApplyCalibration(values, calibrated_values);

  String formattedTime = timeClient.getFormattedTime();
  time_t epochTime = timeClient.getEpochTime();
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

  float I0_1 = (calibrated_values[0] - calibrated_values[1]) * 50.0/0.075; // shunt resistor is 50A 75mV
  float I2_3 = (calibrated_values[2] - calibrated_values[3]) * 50.0/0.075; // shunt resistor is 50A 75mV
  json += F("\"I0_1\" : \"") + String(I0_1, PRECISION) + F("\",\n");
  json += F("\"I2_3\" : \"") + String(I2_3, PRECISION) + F("\",\n");
  json += F("\"date\" : \"") + currentDate + F("\"\n");

  json += F("}\n");

  return json;
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
// MonitorPageHTML
//----------------------------------------------
String MonitorPageHTML(int &Npar, String *keys, String *vals)
{
  float values[4];
  ReadAllValues(values);

  float calibrated_values[4];
  ApplyCalibration(values, calibrated_values);

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

  float I0_1 = (calibrated_values[0] - calibrated_values[1]) * 50.0/0.075; // shunt resistor is 50A 75mV
  float I2_3 = (calibrated_values[2] - calibrated_values[3]) * 50.0/0.075; // shunt resistor is 50A 75mV
  html += F("<p>Current from A0 to A1: ") + String(I0_1,PRECISION) + F("A</p>\r\n");
  html += F("<p>Current from A2 to A3: ") + String(I2_3,PRECISION) + F("A</p>\r\n");
  
  html += F("<p>Time read: ");
  html += timeClient.getFormattedTime();
  html += F("</p><hr>\r\n");

  // Calibration constants
  html += F("Calibration constants:<br\r\n");
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
      float values[4];
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
            "(and the ground wire to the ground). Set the voltage to several values with a "
            "focus on the range from 10V to 20V. At each setting, enter the actual voltage "
            "and  hit \"enter\" or click the \"add point\" button. The more points, the better. "
            "If you enter a wrong voltage, you'll need to hit the \"clear\" button and start over. "
            "Once you have entered several points, hit the \"finish\" button to fit the data and "
            "store the calibration. You can add more points after clicking \"finish\" as long as "
            "the window is still up. All of the data points shown on the bottom will be used. "
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
    if(page_type.startsWith("calibrate.html")) htmlPage += CalibratePageHTML(Npar, keys, vals);
  
    htmlPage += F("</html>\r\n");
  }

  return htmlPage;
}

//----------------------------------------------
// loop
//----------------------------------------------
void loop()
{
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
